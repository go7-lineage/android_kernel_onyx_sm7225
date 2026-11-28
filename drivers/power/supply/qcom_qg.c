// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024, Danila Tikhonov <danila@jiaxyga.com>
 */

#include <linux/bitfield.h>
#include <linux/cleanup.h>
#include <linux/debugfs.h>
#include <linux/devm-helpers.h>
#include <linux/fpu.h>
#include <linux/iio/consumer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/rtc.h>
#include <linux/seq_file.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#include "qcom_qg_profile.h"
#include "qcom_qg_soc.h"

/* BATT offsets */
#define QG_PERPH_SUBTYPE_REG		0x05
#define QG_SUBTYPE_IBAT_5A		0x03
#define QG_STATUS2_REG			0x09
#define QG_STATUS2_GOOD_OCV		BIT(1)
#define QG_INT_RT_STS_REG		0x10
#define QG_INT_RT_STS_BATTERY_MISSING	BIT(0)
#define QG_DATA_CTL1_REG		0x41
#define QG_DATA_CTL1_MASTER_HOLD	BIT(0)
#define QG_S2_NORMAL_MEAS_CTL2_REG	0x51
#define QG_S2_FIFO_LENGTH		GENMASK(5, 3)
#define QG_S2_NUM_OF_ACCUM		GENMASK(2, 0)
#define QG_S2_NORMAL_MEAS_CTL3_REG	0x52
#define QG_S3_SLEEP_OCV_IBAT_CTL1_REG	0x5d
#define QG_S3_ENTRY_FIFO_LENGTH		GENMASK(2, 0)
#define QG_S5_OCV_VALIDATE_MEAS_CTL1_REG	0x60
#define QG_S5_ALLOW			BIT(7)
#define QG_S7_PON_OCV_MEAS_CTL1_REG	0x64
#define QG_S7_ADC_CONV_DLY		GENMASK(3, 0)
#define QG_S7_ADC_CONV_DLY_512MS	0xa
#define QG_S7_PON_OCV_V_DATA0_REG	0x70
#define QG_S3_GOOD_OCV_V_DATA0_REG	0x74
#define QG_S2_NORMAL_AVG_V_DATA0_REG	0x80 /* 2-byte 0x80-0x81 */
#define QG_S2_NORMAL_AVG_I_DATA0_REG	0x82 /* 2-byte 0x82-0x83 */
#define QG_V_FIFO0_DATA0_REG		0x90
#define QG_I_FIFO0_DATA0_REG		0xa0
#define QG_LAST_ADC_V_DATA0_REG		0xc0 /* 2-byte 0xc0-0xc1 */
#define QG_LAST_ADC_I_DATA0_REG		0xc2 /* 2-byte 0xc2-0xc3 */

/* SRAM offsets */
#define QG_SDAM_VALID_OFFSET		0x46
#define QG_SDAM_OCV_OFFSET		0x4c /* 4-byte 0x4c-0x4f */
#define QG_SDAM_TIME_OFFSET		0x54
#define QG_SDAM_LEARNED_CAPACITY_OFFSET	0x68 /* 2-byte 0x68-0x69 */

#define QG_FIFO_MAX			8
#define QG_FIFO_RESET_VALUE		0x8000
#define QG_FIFO_LENGTH			5
#define QG_FIFO_SAMPLES			128
#define QG_FIFO_INTERVAL_MS		100
#define QG_S3_ENTRY_FIFOS		2
#define QG_SHUTDOWN_SOC_SECS		(48 * 3600)
#define QG_SHUTDOWN_SOC_DELTA		30
#define QG_SHUTDOWN_TEMP_DELTA		60
#define QG_JUMP_AFTER_MS		(30 * 60 * 1000)
#define QG_STEP_MIN_MS			20000

struct qcom_qg_chip {
	struct device *dev;
	struct regmap *regmap;
	unsigned int base;
	unsigned int ibat_step_na;

	struct iio_channel *batt_therm_chan;

	struct nvmem_device *sdam;

	struct power_supply *batt_psy;
	struct power_supply_battery_info *batt_info;

	struct qg_profile profile;
	struct qg_soc soc;
	struct mutex lock;
	struct delayed_work step_work;
	unsigned long step_due;
	ktime_t fifo_time;
	int fifo_ms;
	int irq[3];
	int capacity;
	int target;
	int ocv_uv;
	int cc_soc;
	bool estimating;
};

static const char * const qcom_qg_irq_names[] = { "fifo-done", "good-ocv", "vbat-empty" };

static int qcom_qg_read_raw(struct qcom_qg_chip *chip, u8 offset, u16 *raw)
{
	u8 readval[2];
	int ret;

	ret = regmap_bulk_read(chip->regmap, chip->base + offset, readval, 2);
	if (ret)
		return ret;

	*raw = get_unaligned_le16(readval);

	return 0;
}

static int qcom_qg_raw_to_ua(struct qcom_qg_chip *chip, u16 raw)
{
	return div_s64((s64)(s16)raw * chip->ibat_step_na, 1000);
}

static int qcom_qg_raw_to_uv(u16 raw)
{
	return div_u64((u64)raw * 194637, 1000);
}

static int qcom_qg_get_current(struct qcom_qg_chip *chip, u8 offset, int *val)
{
	u16 raw;
	int ret;

	ret = qcom_qg_read_raw(chip, offset, &raw);
	if (ret) {
		dev_err(chip->dev, "Failed to read current: %d\n", ret);
		return ret;
	}

	/*
	 * PSY API expects charging batteries to report a positive current, which is inverted
	 * to what the PMIC reports.
	 */
	*val = -qcom_qg_raw_to_ua(chip, raw);

	return 0;
}

static int qcom_qg_get_voltage(struct qcom_qg_chip *chip, u8 offset, int *val)
{
	u16 raw;
	int ret;

	ret = qcom_qg_read_raw(chip, offset, &raw);
	if (ret) {
		dev_err(chip->dev, "Failed to read voltage: %d\n", ret);
		return ret;
	}

	*val = qcom_qg_raw_to_uv(raw);

	return 0;
}

static int qcom_qg_hold(struct qcom_qg_chip *chip, bool hold)
{
	int ret;

	ret = regmap_clear_bits(chip->regmap, chip->base + QG_DATA_CTL1_REG,
				QG_DATA_CTL1_MASTER_HOLD);
	if (ret || !hold)
		return ret;

	return regmap_set_bits(chip->regmap, chip->base + QG_DATA_CTL1_REG,
			       QG_DATA_CTL1_MASTER_HOLD);
}

static int qcom_qg_config(struct qcom_qg_chip *chip)
{
	int ret, release;

	ret = qcom_qg_hold(chip, true);
	if (ret)
		return ret;

	ret = regmap_update_bits(chip->regmap, chip->base + QG_S2_NORMAL_MEAS_CTL2_REG,
				 QG_S2_FIFO_LENGTH | QG_S2_NUM_OF_ACCUM,
				 FIELD_PREP(QG_S2_FIFO_LENGTH, QG_FIFO_LENGTH - 1) |
				 FIELD_PREP(QG_S2_NUM_OF_ACCUM, ilog2(QG_FIFO_SAMPLES) - 1));
	if (!ret)
		ret = regmap_update_bits(chip->regmap,
					 chip->base + QG_S3_SLEEP_OCV_IBAT_CTL1_REG,
					 QG_S3_ENTRY_FIFO_LENGTH, QG_S3_ENTRY_FIFOS - 1);
	if (!ret)
		ret = regmap_write(chip->regmap, chip->base + QG_S2_NORMAL_MEAS_CTL3_REG,
				   QG_FIFO_INTERVAL_MS / 10);

	release = qcom_qg_hold(chip, false);
	chip->fifo_time = ktime_get_boottime();
	chip->fifo_ms = QG_FIFO_LENGTH * QG_FIFO_SAMPLES * QG_FIFO_INTERVAL_MS;
	if (ret || release)
		return ret ?: release;

	ret = regmap_clear_bits(chip->regmap, chip->base + QG_S5_OCV_VALIDATE_MEAS_CTL1_REG,
				QG_S5_ALLOW);
	if (ret)
		return ret;

	return regmap_update_bits(chip->regmap, chip->base + QG_S7_PON_OCV_MEAS_CTL1_REG,
				  QG_S7_ADC_CONV_DLY, QG_S7_ADC_CONV_DLY_512MS);
}

static int qcom_qg_rtc_time(time64_t *now)
{
	struct rtc_device *rtc __free(rtc_class_close) = rtc_class_open("rtc0");
	struct rtc_time tm;
	int ret;

	if (!rtc)
		return -ENODEV;

	ret = rtc_read_time(rtc, &tm);
	if (ret)
		return ret;

	*now = rtc_tm_to_time64(&tm);

	return 0;
}

static int qcom_qg_temp(struct qcom_qg_chip *chip, int fallback)
{
	int val;

	if (iio_read_channel_processed(chip->batt_therm_chan, &val) < 0)
		return fallback;

	return val / 100;
}

static void qcom_qg_save(struct qcom_qg_chip *chip, int temp)
{
	time64_t now;
	u8 buf[10];

	buf[0] = 1;
	buf[1] = chip->capacity;
	put_unaligned_le16(temp, &buf[2]);
	put_unaligned_le16(chip->soc.v.t_rbat, &buf[4]);
	put_unaligned_le32(chip->ocv_uv, &buf[6]);
	nvmem_device_write(chip->sdam, QG_SDAM_VALID_OFFSET, sizeof(buf), buf);

	if (qcom_qg_rtc_time(&now))
		return;

	put_unaligned_le32(now, buf);
	nvmem_device_write(chip->sdam, QG_SDAM_TIME_OFFSET, 4, buf);
}

static bool qcom_qg_step_allowed(struct qcom_qg_chip *chip)
{
	union power_supply_propval val;

	if (chip->target <= chip->capacity)
		return chip->target < chip->capacity;

	if (power_supply_get_property_from_supplier(chip->batt_psy, POWER_SUPPLY_PROP_ONLINE,
						    &val) || !val.intval)
		return false;

	if (!chip->capacity)
		return true;

	if (power_supply_get_property_from_supplier(chip->batt_psy, POWER_SUPPLY_PROP_STATUS,
						    &val))
		return false;

	return val.intval == POWER_SUPPLY_STATUS_CHARGING ||
	       val.intval == POWER_SUPPLY_STATUS_FULL;
}

static void qcom_qg_step(struct qcom_qg_chip *chip)
{
	int delay;
	u8 capacity;

	if (!qcom_qg_step_allowed(chip))
		return;

	chip->capacity += chip->target > chip->capacity ? 1 : -1;
	capacity = chip->capacity;
	nvmem_device_write(chip->sdam, QG_SDAM_VALID_OFFSET + 1, 1, &capacity);
	power_supply_changed(chip->batt_psy);

	if (!qcom_qg_step_allowed(chip))
		return;

	delay = max(chip->fifo_ms / (abs(chip->target - chip->capacity) + 1) - 2000,
		    QG_STEP_MIN_MS);
	chip->step_due = jiffies + msecs_to_jiffies(delay);
	mod_delayed_work(system_wq, &chip->step_work, msecs_to_jiffies(delay));
}

static void qcom_qg_step_work(struct work_struct *work)
{
	struct qcom_qg_chip *chip = container_of(work, struct qcom_qg_chip, step_work.work);

	guard(mutex)(&chip->lock);

	if (time_after_eq(jiffies, chip->step_due))
		qcom_qg_step(chip);
}

static void qcom_qg_event(struct qcom_qg_chip *chip, const struct qg_sample *fifo,
			  unsigned int n, int time_delta_ms, int good_ocv_uv, bool jump)
{
	union power_supply_propval val;
	struct qg_soc_in in;
	bool sent;
	u16 raw;

	in.temp = qcom_qg_temp(chip, chip->soc.v.temp);
	in.vbat_uv = qcom_qg_read_raw(chip, QG_LAST_ADC_V_DATA0_REG, &raw) ?
		     0 : qcom_qg_raw_to_uv(raw);
	in.charge_done = !power_supply_get_property_from_supplier(chip->batt_psy,
								   POWER_SUPPLY_PROP_STATUS,
								   &val) &&
			 val.intval == POWER_SUPPLY_STATUS_FULL;

	kernel_fpu_begin();
	sent = qg_soc_event(&chip->soc, &in, fifo, n, time_delta_ms, good_ocv_uv);
	if (sent)
		chip->target = qg_soc_report(&chip->soc, &in);
	kernel_fpu_end();

	if (!sent)
		return;

	chip->ocv_uv = chip->soc.v.b_ocv;
	chip->cc_soc = chip->soc.v.cc_soc;
	if (jump && chip->capacity != chip->target) {
		chip->capacity = chip->target;
		power_supply_changed(chip->batt_psy);
	}

	qcom_qg_save(chip, in.temp);
	qcom_qg_step(chip);
}

static irqreturn_t qcom_qg_fifo_done(int irq, void *data)
{
	struct qcom_qg_chip *chip = data;
	struct qg_sample fifo[QG_FIFO_MAX];
	u8 v[2 * QG_FIFO_MAX], i[2 * QG_FIFO_MAX];
	unsigned int ctl2, ctl3, n, k;
	ktime_t now = ktime_get_boottime();
	s64 delta, nominal;
	int ret;

	guard(mutex)(&chip->lock);

	delta = ktime_ms_delta(now, chip->fifo_time);
	chip->fifo_time = now;

	ret = regmap_read(chip->regmap, chip->base + QG_S2_NORMAL_MEAS_CTL2_REG, &ctl2);
	if (!ret)
		ret = regmap_read(chip->regmap, chip->base + QG_S2_NORMAL_MEAS_CTL3_REG, &ctl3);
	if (ret)
		goto err;

	n = FIELD_GET(QG_S2_FIFO_LENGTH, ctl2) + 1;
	ret = regmap_bulk_read(chip->regmap, chip->base + QG_V_FIFO0_DATA0_REG, v, 2 * n);
	if (!ret)
		ret = regmap_bulk_read(chip->regmap, chip->base + QG_I_FIFO0_DATA0_REG, i, 2 * n);
	if (ret)
		goto err;

	for (k = 0; k < n; k++) {
		u16 raw_v = get_unaligned_le16(&v[2 * k]), raw_i = get_unaligned_le16(&i[2 * k]);

		if (raw_v == QG_FIFO_RESET_VALUE || raw_i == QG_FIFO_RESET_VALUE) {
			dev_err_ratelimited(chip->dev, "FIFO entry %u is empty\n", k);
			return IRQ_HANDLED;
		}

		fifo[k].v_uv = qcom_qg_raw_to_uv(raw_v);
		fifo[k].i_ua = qcom_qg_raw_to_ua(chip, raw_i);
		fifo[k].count = BIT(FIELD_GET(QG_S2_NUM_OF_ACCUM, ctl2) + 1);
		fifo[k].interval_ms = ctl3 * 10;
	}

	nominal = (s64)n * fifo[0].count * fifo[0].interval_ms;
	if (abs(nominal - delta) >= div_s64(nominal, 10))
		delta = -1;

	qcom_qg_event(chip, fifo, n, delta, 0, false);

	return IRQ_HANDLED;

err:
	dev_err_ratelimited(chip->dev, "Failed to read the FIFO: %d\n", ret);

	return IRQ_HANDLED;
}

static void qcom_qg_good_ocv(struct qcom_qg_chip *chip)
{
	unsigned int status;
	u16 raw;

	guard(mutex)(&chip->lock);

	if (regmap_read(chip->regmap, chip->base + QG_STATUS2_REG, &status) ||
	    !(status & QG_STATUS2_GOOD_OCV) ||
	    qcom_qg_read_raw(chip, QG_S3_GOOD_OCV_V_DATA0_REG, &raw))
		return;

	qcom_qg_event(chip, NULL, 0, -1, qcom_qg_raw_to_uv(raw),
		      ktime_ms_delta(ktime_get_boottime(), chip->fifo_time) >= QG_JUMP_AFTER_MS);
}

static irqreturn_t qcom_qg_good_ocv_irq(int irq, void *data)
{
	qcom_qg_good_ocv(data);

	return IRQ_HANDLED;
}

static irqreturn_t qcom_qg_vbat_empty(int irq, void *data)
{
	struct qcom_qg_chip *chip = data;
	unsigned int status;

	guard(mutex)(&chip->lock);

	if (!regmap_read(chip->regmap, chip->base + QG_INT_RT_STS_REG, &status) &&
	    (status & QG_INT_RT_STS_BATTERY_MISSING))
		return IRQ_HANDLED;

	dev_warn(chip->dev, "battery empty\n");
	chip->target = 0;
	chip->capacity = 0;
	qcom_qg_save(chip, qcom_qg_temp(chip, chip->soc.v.temp));
	power_supply_changed(chip->batt_psy);

	return IRQ_HANDLED;
}

static int qcom_qg_start_capacity(struct qcom_qg_chip *chip, int temp)
{
	struct power_supply_battery_info *info = chip->batt_info;
	const struct qg_profile *profile = &chip->profile;
	int ret, pon_soc, full_soc, cutoff_soc, saved_temp;
	time64_t now, age;
	u8 saved[10], stamp[4];
	u16 raw;

	ret = qcom_qg_read_raw(chip, QG_S7_PON_OCV_V_DATA0_REG, &raw);
	if (ret)
		return ret;

	chip->ocv_uv = clamp(qcom_qg_raw_to_uv(raw), 2000000, 5000000);
	pon_soc = DIV_ROUND_CLOSEST(qg_profile_soc(profile, QG_TABLE_OCV2, temp, chip->ocv_uv),
				    100);

	if (nvmem_device_read(chip->sdam, QG_SDAM_VALID_OFFSET, sizeof(saved), saved) >= 0 &&
	    nvmem_device_read(chip->sdam, QG_SDAM_TIME_OFFSET, sizeof(stamp), stamp) >= 0 &&
	    !qcom_qg_rtc_time(&now)) {
		saved_temp = (s16)get_unaligned_le16(&saved[2]);
		age = now - get_unaligned_le32(stamp);

		if (saved[0] && age >= 0 && age <= QG_SHUTDOWN_SOC_SECS &&
		    (abs(saved_temp - temp) <= QG_SHUTDOWN_TEMP_DELTA ||
		     (saved_temp >= 0 && temp >= 0)) &&
		    abs(pon_soc - saved[1]) <= QG_SHUTDOWN_SOC_DELTA) {
			chip->ocv_uv = get_unaligned_le32(&saved[6]);

			return min(saved[1], 100);
		}
	}

	full_soc = DIV_ROUND_CLOSEST(qg_profile_soc(profile, QG_TABLE_OCV1, temp,
						    info->voltage_max_design_uv), 100);
	full_soc = min(full_soc, 99);
	cutoff_soc = DIV_ROUND_CLOSEST(qg_profile_soc(profile, QG_TABLE_OCV2, temp,
						      info->voltage_min_design_uv), 100);
	if (full_soc > cutoff_soc && pon_soc > cutoff_soc)
		return min(DIV_ROUND_UP((pon_soc - cutoff_soc) * 100, full_soc - cutoff_soc), 100);

	return pon_soc;
}

static int qcom_qg_state_show(struct seq_file *s, void *unused)
{
	struct qcom_qg_chip *chip = s->private;
	struct qg_soc_vars v;
	int capacity, target;
	u32 bits[5];

	scoped_guard(mutex, &chip->lock) {
		v = chip->soc.v;
		capacity = chip->capacity;
		target = chip->target;
	}

	memcpy(&bits[0], &v.init_soc, sizeof(u32));
	memcpy(&bits[1], &v.v_ocv, sizeof(u32));
	memcpy(&bits[2], &v.cc_uah, sizeof(u32));
	memcpy(&bits[3], &v.i_rshort, sizeof(u32));
	memcpy(&bits[4], &v.i_rlong, sizeof(u32));

	seq_printf(s, "capacity=%d target=%d s_soc=%d b_soc=%d r_soc=%d voltage_soc=%d cc_soc=%d",
		   capacity, target, v.s_soc, v.b_soc, v.r_soc, v.voltage_soc, v.cc_soc);
	seq_printf(s, " cc_soc_sw=%d soc_corr=%d cutoff_soc=%d full_soc=%d v_cap=%d b_ocv=%d",
		   v.cc_soc_sw, v.soc_corr, v.cutoff_soc, v.full_soc, v.v_cap, v.b_ocv);
	seq_printf(s, " t_rbat=%d fcc=%d fifo_i=%d fifo_length=%d temp=%d", v.t_rbat, v.fcc,
		   v.fifo_i, v.fifo_length, v.temp);
	seq_printf(s, " init_soc=%08x v_ocv=%08x cc_uah=%08x i_rshort=%08x i_rlong=%08x\n",
		   bits[0], bits[1], bits[2], bits[3], bits[4]);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(qcom_qg_state);

static void qcom_qg_debugfs_remove(void *data)
{
	debugfs_remove_recursive(data);
}

static int qcom_qg_setup_soc(struct platform_device *pdev, struct qcom_qg_chip *chip)
{
	struct fwnode_handle *node __free(fwnode_handle) =
		device_get_named_child_node(chip->dev, "battery-profile");
	struct power_supply_battery_info *info = chip->batt_info;
	struct qg_soc_cfg cfg = {
		.vcutoff_uv = info->voltage_min_design_uv,
		.vfloat_uv = info->voltage_max_design_uv,
		.iterm_ua = -info->charge_term_current_ua,
	};
	unsigned int i, status;
	int ret, temp, capacity;

	if (!node || !kernel_fpu_available())
		return -ENODEV;

	if (cfg.vcutoff_uv <= 0 || cfg.vfloat_uv <= cfg.vcutoff_uv || cfg.iterm_ua >= 0)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(chip->irq); i++) {
		chip->irq[i] = platform_get_irq_byname_optional(pdev, qcom_qg_irq_names[i]);
		if (chip->irq[i] < 0)
			return chip->irq[i];
	}

	ret = qg_profile_parse(chip->dev, node, &chip->profile);
	if (ret)
		return ret;

	ret = qcom_qg_config(chip);
	if (ret)
		return ret;

	regmap_read(chip->regmap, chip->base + QG_STATUS2_REG, &status);

	temp = qcom_qg_temp(chip, 250);
	capacity = qcom_qg_start_capacity(chip, temp);
	if (capacity < 0)
		return capacity;

	kernel_fpu_begin();
	qg_soc_start(&chip->soc, &chip->profile, &cfg, capacity, temp);
	kernel_fpu_end();

	guard(mutex)(&chip->lock);

	chip->capacity = capacity;
	chip->target = capacity;
	chip->cc_soc = chip->soc.v.cc_soc;
	qcom_qg_save(chip, temp);

	return 0;
}

static int qcom_qg_request_irqs(struct qcom_qg_chip *chip)
{
	static const irq_handler_t handlers[] = {
		qcom_qg_fifo_done, qcom_qg_good_ocv_irq, qcom_qg_vbat_empty,
	};
	struct dentry *debugfs;
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(chip->irq); i++) {
		ret = devm_request_threaded_irq(chip->dev, chip->irq[i], NULL, handlers[i],
						IRQF_ONESHOT, qcom_qg_irq_names[i], chip);
		if (ret)
			return dev_err_probe(chip->dev, ret, "Failed to request the %s interrupt\n",
					     qcom_qg_irq_names[i]);
	}

	debugfs = debugfs_create_dir("qcom_qg", NULL);
	debugfs_create_file("state", 0400, debugfs, chip, &qcom_qg_state_fops);
	ret = devm_add_action_or_reset(chip->dev, qcom_qg_debugfs_remove, debugfs);
	if (ret)
		return ret;

	return devm_device_init_wakeup(chip->dev);
}

/*
 * Yes, this function simply calculates the capacity based on
 * the current voltage. This will be rewritten in the future.
 */
static int qcom_qg_get_capacity(struct qcom_qg_chip *chip, int *val)
{
	int ret, voltage_now;
	int voltage_min = chip->batt_info->voltage_min_design_uv;
	int voltage_max = chip->batt_info->voltage_max_design_uv;

	if (chip->estimating) {
		*val = READ_ONCE(chip->capacity);
		return 0;
	}

	ret = qcom_qg_get_voltage(chip,
				QG_S2_NORMAL_AVG_V_DATA0_REG, &voltage_now);
	if (ret) {
		dev_err(chip->dev, "Failed to get current voltage: %d\n", ret);
		return ret;
	}

	if (voltage_now <= voltage_min)
		*val = 0;
	else if (voltage_now >= voltage_max)
		*val = 100;
	else
		*val = (((voltage_now - voltage_min) * 100) /
						(voltage_max - voltage_min));

	return 0;
}

static int qcom_qg_get_charge_full(struct qcom_qg_chip *chip, int *val)
{
	u8 raw[2];
	int ret;

	ret = nvmem_device_read(chip->sdam, QG_SDAM_LEARNED_CAPACITY_OFFSET, 2, raw);
	if (ret < 0)
		return ret;

	*val = get_unaligned_le16(raw) * 1000;
	if (!*val)
		*val = chip->batt_info->charge_full_design_uah;

	return *val > 0 ? 0 : -ENODATA;
}

static enum power_supply_property qcom_qg_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
};

static int qcom_qg_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct qcom_qg_chip *chip = power_supply_get_drvdata(psy);
	int ret, full, capacity;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (power_supply_get_property_from_supplier(psy, psp, val))
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = chip->batt_info->voltage_max_design_uv;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = chip->batt_info->voltage_min_design_uv;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = qcom_qg_get_voltage(chip,
				QG_LAST_ADC_V_DATA0_REG, &val->intval);
		if (ret)
			return ret;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		ret = qcom_qg_get_voltage(chip,
				QG_S2_NORMAL_AVG_V_DATA0_REG, &val->intval);
		if (ret)
			return ret;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		if (chip->estimating) {
			val->intval = READ_ONCE(chip->ocv_uv);
			break;
		}
		ret = nvmem_device_read(chip->sdam, QG_SDAM_OCV_OFFSET, 4, &val->intval);
		if (ret < 0)
			return ret;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = qcom_qg_get_current(chip,
				QG_LAST_ADC_I_DATA0_REG, &val->intval);
		if (ret)
			return ret;
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		ret = qcom_qg_get_current(chip,
				QG_S2_NORMAL_AVG_I_DATA0_REG, &val->intval);
		if (ret)
			return ret;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = chip->batt_info->charge_full_design_uah;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = qcom_qg_get_charge_full(chip, &val->intval);
		if (ret)
			return ret;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		ret = qcom_qg_get_charge_full(chip, &full);
		if (!ret)
			ret = qcom_qg_get_capacity(chip, &capacity);
		if (ret)
			return ret;
		if (chip->estimating)
			capacity = clamp(DIV_ROUND_CLOSEST(READ_ONCE(chip->cc_soc), 100), 0, 100);
		val->intval = div_s64((s64)full * capacity, 100);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = qcom_qg_get_capacity(chip, &val->intval);
		if (ret)
			return ret;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = iio_read_channel_processed
					(chip->batt_therm_chan, &val->intval);
		if (ret < 0)
			return ret;
		val->intval /= 100; /* 1/1000 °C (millidegC) to 1/10 °C */
		break;
	default:
		dev_err(chip->dev, "invalid property: %d\n", psp);
		return -EINVAL;
	}
	return 0;
}

static struct power_supply_desc batt_psy_desc = {
	.name = "qcom_qg",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = qcom_qg_props,
	.num_properties = ARRAY_SIZE(qcom_qg_props),
	.get_property = qcom_qg_get_property,
};

static int qcom_qg_probe(struct platform_device *pdev)
{
	struct qcom_qg_chip *chip;
	struct power_supply_config psy_cfg = {};
	int ret;
	unsigned int subtype;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &pdev->dev;

	ret = devm_mutex_init(chip->dev, &chip->lock);
	if (ret)
		return ret;

	/* Regmap */
	chip->regmap = dev_get_regmap(chip->dev->parent, NULL);
	if (!chip->regmap)
		return dev_err_probe(chip->dev, -ENODEV,
				     "Failed to locate the regmap\n");

	/* Get base address */
	ret = device_property_read_u32(chip->dev, "reg", &chip->base);
	if (ret < 0)
		return dev_err_probe(chip->dev, ret,
				     "Couldn't read base address\n");

	ret = regmap_read(chip->regmap, chip->base + QG_PERPH_SUBTYPE_REG, &subtype);
	if (ret)
		return dev_err_probe(chip->dev, ret, "Couldn't read subtype\n");
	chip->ibat_step_na = subtype == QG_SUBTYPE_IBAT_5A ? 152588 : 305176;

	/* ADC for thermal channel */
	chip->batt_therm_chan = devm_iio_channel_get(chip->dev, "batt-therm");
	if (IS_ERR(chip->batt_therm_chan))
		return dev_err_probe(chip->dev, PTR_ERR(chip->batt_therm_chan),
				     "Couldn't get batt-therm IIO channel\n");

	/* NVMEM for SDAM access */
	chip->sdam = devm_nvmem_device_get(chip->dev, NULL);
	if (IS_ERR(chip->sdam))
		return dev_err_probe(chip->dev, PTR_ERR(chip->sdam),
				     "Couldn't get SDAM nvmem device\n");

	psy_cfg.drv_data = chip;
	psy_cfg.fwnode = dev_fwnode(chip->dev);

	/* Power supply */
	chip->batt_psy =
		devm_power_supply_register(chip->dev, &batt_psy_desc, &psy_cfg);
	if (IS_ERR(chip->batt_psy))
		return dev_err_probe(chip->dev, PTR_ERR(chip->batt_psy),
				     "Failed to register power supply\n");

	/* Battery info */
	ret = power_supply_get_battery_info(chip->batt_psy, &chip->batt_info);
	if (ret)
		return dev_err_probe(chip->dev, ret,
				     "Failed to get battery info\n");

	platform_set_drvdata(pdev, chip);

	ret = devm_delayed_work_autocancel(chip->dev, &chip->step_work, qcom_qg_step_work);
	if (ret)
		return ret;

	ret = qcom_qg_setup_soc(pdev, chip);
	if (ret == -EPROBE_DEFER)
		return ret;
	if (ret) {
		dev_warn(chip->dev, "no battery profile or interrupts (%d), capacity from voltage only\n",
			 ret);
		return 0;
	}

	chip->estimating = true;

	return qcom_qg_request_irqs(chip);
}

static int qcom_qg_suspend(struct device *dev)
{
	struct qcom_qg_chip *chip = dev_get_drvdata(dev);
	unsigned int i, status;

	if (!chip->estimating)
		return 0;

	regmap_read(chip->regmap, chip->base + QG_STATUS2_REG, &status);
	if (device_may_wakeup(dev))
		for (i = 0; i < ARRAY_SIZE(chip->irq); i++)
			enable_irq_wake(chip->irq[i]);

	return 0;
}

static int qcom_qg_resume(struct device *dev)
{
	struct qcom_qg_chip *chip = dev_get_drvdata(dev);
	unsigned int i;

	if (!chip->estimating)
		return 0;

	if (device_may_wakeup(dev))
		for (i = 0; i < ARRAY_SIZE(chip->irq); i++)
			disable_irq_wake(chip->irq[i]);
	qcom_qg_good_ocv(chip);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(qcom_qg_pm_ops, qcom_qg_suspend, qcom_qg_resume);

static const struct of_device_id qcom_qg_of_match[] = {
	{ .compatible = "qcom,pm6150-qg", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, qcom_qg_of_match);

static struct platform_driver qcom_qg_driver = {
	.driver = {
		.name = "qcom,qcom_qg",
		.of_match_table = qcom_qg_of_match,
		.pm = pm_sleep_ptr(&qcom_qg_pm_ops),
	},
	.probe = qcom_qg_probe,
};

module_platform_driver(qcom_qg_driver);

MODULE_AUTHOR("Danila Tikhonov <danila@jiaxyga.com>");
MODULE_DESCRIPTION("Qualcomm PMIC QGauge (QG) driver");
MODULE_LICENSE("GPL");
