// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016-2019 The Linux Foundation. All rights reserved.
 * Copyright (c) 2023, Linaro Ltd.
 * Author: Casey Connolly <casey.connolly@linaro.org>
 *
 * This driver is for the switch-mode battery charger and boost
 * hardware found in pmi8998 and related PMICs.
 */

#include <linux/bits.h>
#include <linux/devm-helpers.h>
#include <linux/iio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeirq.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/workqueue.h>

enum smb_generation {
	SMB2,
	SMB5,
};

#define SMB_REG_OFFSET(smb) (smb->gen == SMB2 ? 0x600 : 0x100)

/* clang-format off */
#define BATTERY_CHARGER_STATUS_1			0x06
#define BATTERY_CHARGER_STATUS_MASK			GENMASK(2, 0)

#define BATTERY_CHARGER_STATUS_2			0x07
#define SMB2_CHARGER_ERROR_STATUS_BAT_OV_BIT		BIT(5)
#define SMB2_BAT_TEMP_STATUS_HOT_SOFT_LIMIT_BIT	BIT(3)
#define SMB2_BAT_TEMP_STATUS_COLD_SOFT_LIMIT_BIT	BIT(2)
#define SMB2_BAT_TEMP_STATUS_TOO_HOT_BIT		BIT(1)
#define SMB5_CHARGER_ERROR_STATUS_BAT_OV_BIT		BIT(1)
#define SMB2_BAT_TEMP_STATUS_TOO_COLD_BIT		BIT(0)

#define BATTERY_CHARGER_STATUS_7			0x0D
#define SMB5_BAT_TEMP_STATUS_HOT_SOFT_BIT		BIT(5)
#define SMB5_BAT_TEMP_STATUS_COLD_SOFT_BIT		BIT(4)
#define SMB5_BAT_TEMP_STATUS_TOO_HOT_BIT		BIT(3)
#define SMB5_BAT_TEMP_STATUS_TOO_COLD_BIT		BIT(2)

#define CHARGING_ENABLE_CMD				0x42
#define CHARGING_ENABLE_CMD_BIT				BIT(0)

#define CHGR_CFG2					0x51
#define CHG_EN_SRC_BIT					BIT(7)
#define CHG_EN_POLARITY_BIT				BIT(6)
#define PRETOFAST_TRANSITION_CFG_BIT			BIT(5)
#define BAT_OV_ECC_BIT					BIT(4)
#define I_TERM_BIT					BIT(3)
#define AUTO_RECHG_BIT					BIT(2)
#define EN_ANALOG_DROP_IN_VBATT_BIT			BIT(1)
#define CHARGER_INHIBIT_BIT				BIT(0)

#define PRE_CHARGE_CURRENT_CFG				0x60
#define PRE_CHARGE_CURRENT_SETTING_MASK			GENMASK(5, 0)

#define FAST_CHARGE_CURRENT_CFG				0x61
#define FAST_CHARGE_CURRENT_SETTING_MASK		GENMASK(7, 0)

#define FLOAT_VOLTAGE_CFG				0x70
#define FLOAT_VOLTAGE_SETTING_MASK			GENMASK(7, 0)

#define SMB2_FG_UPDATE_CFG_2_SEL			0x7D
#define SMB2_SOC_LT_CHG_RECHARGE_THRESH_SEL_BIT		BIT(2)
#define SMB2_VBT_LT_CHG_RECHARGE_THRESH_SEL_BIT		BIT(1)

#define SMB5_CHARGE_RCHG_SOC_THRESHOLD_CFG_REG		0x7D
#define SMB5_CHARGE_RCHG_SOC_THRESHOLD_CFG_MASK		GENMASK(7, 0)

#define OTG_CFG						0x153
#define OTG_EN_SRC_CFG_BIT				BIT(1)

#define APSD_STATUS					0x307
#define APSD_DTC_STATUS_DONE_BIT			BIT(0)

#define APSD_RESULT_STATUS				0x308
#define APSD_RESULT_STATUS_MASK				GENMASK(6, 0)
#define FLOAT_CHARGER_BIT				BIT(4)
#define DCP_CHARGER_BIT					BIT(3)
#define CDP_CHARGER_BIT					BIT(2)
#define OCP_CHARGER_BIT					BIT(1)
#define SDP_CHARGER_BIT					BIT(0)

#define USBIN_CMD_IL					0x340
#define USBIN_SUSPEND_BIT				BIT(0)

#define CMD_APSD					0x341
#define APSD_RERUN_BIT					BIT(0)

#define CMD_ICL_OVERRIDE				0x342
#define ICL_OVERRIDE_BIT				BIT(0)

#define TYPE_C_CFG					0x358
#define APSD_START_ON_CC_BIT				BIT(7)
#define FACTORY_MODE_DETECTION_EN_BIT			BIT(5)
#define VCONN_OC_CFG_BIT				BIT(1)

#define USBIN_OPTIONS_1_CFG				0x362
#define AUTO_SRC_DETECT_BIT				BIT(3)
#define HVDCP_EN_BIT					BIT(2)

#define USBIN_LOAD_CFG					0x365
#define ICL_OVERRIDE_AFTER_APSD_BIT			BIT(4)

#define USBIN_ICL_OPTIONS				0x366
#define USB51_MODE_BIT					BIT(1)
#define USBIN_MODE_CHG_BIT				BIT(0)

/* PMI8998 only */
#define TYPE_C_INTRPT_ENB_SOFTWARE_CTRL			0x368
#define SMB2_VCONN_EN_SRC_BIT				BIT(4)
#define VCONN_EN_VALUE_BIT				BIT(3)
#define TYPEC_POWER_ROLE_CMD_MASK			GENMASK(2, 0)
#define SMB5_EN_SNK_ONLY_BIT				BIT(1)

#define USBIN_CURRENT_LIMIT_CFG				0x370

#define USBIN_AICL_OPTIONS_CFG				0x380
#define SUSPEND_ON_COLLAPSE_USBIN_BIT			BIT(7)
#define USBIN_AICL_START_AT_MAX_BIT			BIT(5)
#define USBIN_AICL_PERIODIC_RERUN_EN_BIT		BIT(4)
#define USBIN_AICL_ADC_EN_BIT				BIT(3)
#define USBIN_AICL_EN_BIT				BIT(2)
#define USBIN_HV_COLLAPSE_RESPONSE_BIT			BIT(1)
#define USBIN_LV_COLLAPSE_RESPONSE_BIT			BIT(0)

// FIXME: drop these and their programming, no need to set min to 4.3v
#define USBIN_5V_AICL_THRESHOLD_CFG			0x381
#define USBIN_5V_AICL_THRESHOLD_CFG_MASK		GENMASK(2, 0)

#define USBIN_CONT_AICL_THRESHOLD_CFG			0x384
#define USBIN_CONT_AICL_THRESHOLD_CFG_MASK		GENMASK(5, 0)

#define ICL_STATUS(smb)					(SMB_REG_OFFSET(smb) + 0x07)
#define INPUT_CURRENT_LIMIT_MASK			GENMASK(7, 0)

#define POWER_PATH_STATUS(smb)				(SMB_REG_OFFSET(smb) + 0x0B)
#define P_PATH_USE_USBIN_BIT				BIT(4)
#define P_PATH_VALID_INPUT_POWER_SOURCE_STS_BIT		BIT(0)

/* 0x5xx region is PM8150b only Type-C registers */
#define SMB5_TYPE_C_SRC_STATUS_REG			0x508
#define SMB5_DETECTED_SNK_TYPE_MASK			GENMASK(4, 0)
#define SMB5_SRC_DEBUG_ACCESS_BIT			BIT(4)
#define SMB5_SRC_RD_OPEN_BIT				BIT(3)
#define SMB5_SRC_RD_RA_VCONN_BIT			BIT(2)
#define SMB5_AUDIO_ACCESS_RA_RA_BIT			BIT(0)

#define SMB5_TYPEC_SNK_STATUS_REG			0x506
#define SMB5_SNK_RP_1P5_BIT				BIT(2)
#define SMB5_SNK_RP_3P0_BIT				BIT(1)

#define SMB5_TYPEC_LEGACY_CABLE_STATUS_REG		0x50d
#define SMB5_TYPEC_LEGACY_CABLE_BIT			BIT(1)

/* Bits 2:0 match PMI8998 TYPE_C_INTRPT_ENB_SOFTWARE_CTRL */
#define SMB5_TYPE_C_MODE_CFG				0x544
#define SMB5_EN_TRY_SNK_BIT				BIT(4)
#define SMB5_EN_SNK_ONLY_BIT				BIT(1)

#define SMB5_TYPEC_TYPE_C_VCONN_CONTROL			0x546
#define SMB5_VCONN_EN_ORIENTATION_BIT			BIT(2)
#define SMB5_VCONN_EN_VALUE_BIT				BIT(1)
#define SMB5_VCONN_EN_SRC_BIT				BIT(0)


#define SMB5_TYPE_C_DEBUG_ACCESS_SINK			0x54a
#define SMB5_TYPEC_DEBUG_ACCESS_SINK_MASK		GENMASK(4, 0)

#define SMB5_DEBUG_ACCESS_SRC_CFG			0x54C
#define SMB5_EN_UNORIENTED_DEBUG_ACCESS_SRC_BIT	BIT(0)

#define SMB5_TYPE_C_EXIT_STATE_CFG			0x550
#define SMB5_BYPASS_VSAFE0V_DURING_ROLE_SWAP_BIT	BIT(3)
#define SMB5_SEL_SRC_UPPER_REF_BIT			BIT(2)
#define SMB5_EXIT_SNK_BASED_ON_CC_BIT			BIT(0)

/* Common */

#define BARK_BITE_WDOG_PET				0x643
#define BARK_BITE_WDOG_PET_BIT				BIT(0)

#define WD_CFG						0x651
#define WATCHDOG_TRIGGER_AFP_EN_BIT			BIT(7)
#define BARK_WDOG_INT_EN_BIT				BIT(6)
#define WDOG_TIMER_EN_ON_PLUGIN_BIT			BIT(1)

#define SNARL_BARK_BITE_WD_CFG				0x653

#define AICL_RERUN_TIME_CFG				0x661
#define AICL_RERUN_TIME_MASK				GENMASK(1, 0)
#define AIC_RERUN_TIME_3_SECS				0x0

/* FIXME: probably remove this so we get parallel charging? */
#define STAT_CFG					0x690
#define STAT_SW_OVERRIDE_CFG_BIT			BIT(6)

#define SDP_CURRENT_UA					500000
#define CDP_CURRENT_UA					1500000
#define DCP_CURRENT_UA					1500000
#define CURRENT_MAX_UA					DCP_CURRENT_UA
#define TYPEC_MEDIUM_CURRENT_UA				1500000
#define TYPEC_HIGH_CURRENT_UA				3000000

#define LIMIT_POLL_MS					30000

#define SMB2_FLOAT_VOLTAGE_MIN_UV			3487500
#define SMB2_FLOAT_VOLTAGE_MAX_UV			4920000
#define SMB2_FLOAT_VOLTAGE_STEP_UV			7500
#define SMB5_FLOAT_VOLTAGE_MIN_UV			3600000
#define SMB5_FLOAT_VOLTAGE_MAX_UV			4790000
#define SMB5_FLOAT_VOLTAGE_STEP_UV			10000
/* clang-format on */

enum charger_status {
	TRICKLE_CHARGE = 0,
	PRE_CHARGE,
	FAST_CHARGE,
	FULLON_CHARGE,
	TAPER_CHARGE,
	TERMINATE_CHARGE,
	INHIBIT_CHARGE,
	DISABLE_CHARGE,
};

static const u8 smb5_charger_status[] = {
	INHIBIT_CHARGE, TRICKLE_CHARGE, PRE_CHARGE, FULLON_CHARGE,
	TAPER_CHARGE, TERMINATE_CHARGE, DISABLE_CHARGE, DISABLE_CHARGE,
};

struct smb_init_register {
	u16 addr;
	u8 mask;
	u8 val;
};

/**
 * struct smb_chip - smb chip structure
 * @dev:		Device reference for power_supply
 * @name:		The platform device name
 * @base:		Base address for smb registers
 * @regmap:		Register map
 * @batt_info:		Battery data from DT
 * @initial_charge_enable: Charging enable state before hardware setup
 * @gen:		Generation of SMBx hardware block
 * @current_step_size_ua: Step size of current limits in uA
 * @current_limit_max_ua: Maximum charging current in uA
 * @status_change_work: Worker to handle plug/unplug events
 * @cable_irq:		USB plugin IRQ
 * @wakeup_enabled:	If the cable IRQ will cause a wakeup
 * @usb_in_i_chan:	USB_IN current measurement channel
 * @usb_in_v_chan:	USB_IN voltage measurement channel
 * @chg_psy:		Charger power supply instance
 */
struct smb_chip {
	struct device *dev;
	const char *name;
	unsigned int base;
	struct regmap *regmap;
	struct power_supply_battery_info *batt_info;
	u8 initial_charge_enable;
	enum smb_generation gen;
	unsigned int current_step_size_ua;
	unsigned int current_limit_max_ua;

	struct delayed_work status_change_work;
	int cable_irq;
	bool wakeup_enabled;

	struct iio_channel *usb_in_i_chan;
	struct iio_channel *usb_in_v_chan;

	struct power_supply *chg_psy;

	struct mutex limit_lock;
	int limit_start, limit_end;
	bool limit_stopped;
	struct delayed_work limit_work;
};

struct smb_match_data {
	const char *name;
	enum smb_generation gen;
	size_t init_seq_len;
	const struct smb_init_register *init_seq;
	unsigned int current_step_size_ua;
	unsigned int current_limit_max_ua;
};

static enum power_supply_property smb_properties[] = {
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD,
};

static bool smb_is_charging(struct smb_chip *chip)
{
	int rc;
	union power_supply_propval status;

	rc = power_supply_get_property(chip->chg_psy, POWER_SUPPLY_PROP_STATUS,
				       &status);
	return !(rc < 0 || status.intval != POWER_SUPPLY_STATUS_CHARGING);
}

static int smb_get_prop_usb_online(struct smb_chip *chip, int *val)
{
	unsigned int stat;
	int rc;

	rc = regmap_read(chip->regmap, chip->base + POWER_PATH_STATUS(chip), &stat);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read power path status: %d\n", rc);
		return rc;
	}

	*val = (stat & P_PATH_USE_USBIN_BIT) &&
	       (stat & P_PATH_VALID_INPUT_POWER_SOURCE_STS_BIT);
	return 0;
}

/*
 * Qualcomm "automatic power source detection" aka APSD
 * tells us what type of charger we're connected to.
 */
static int smb_apsd_get_charger_type(struct smb_chip *chip, int *val)
{
	unsigned int apsd_stat, stat;
	int usb_online = 0;
	int rc;

	rc = smb_get_prop_usb_online(chip, &usb_online);
	if (!usb_online) {
		*val = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		return rc;
	}

	rc = regmap_read(chip->regmap, chip->base + APSD_STATUS, &apsd_stat);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to read apsd status, rc = %d", rc);
		return rc;
	}
	if (!(apsd_stat & APSD_DTC_STATUS_DONE_BIT)) {
		dev_dbg(chip->dev, "Apsd not ready");
		return -EAGAIN;
	}

	rc = regmap_read(chip->regmap, chip->base + APSD_RESULT_STATUS, &stat);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to read apsd result, rc = %d", rc);
		return rc;
	}

	stat &= APSD_RESULT_STATUS_MASK;

	if (stat & CDP_CHARGER_BIT)
		*val = POWER_SUPPLY_USB_TYPE_CDP;
	else if (stat & (DCP_CHARGER_BIT | OCP_CHARGER_BIT | FLOAT_CHARGER_BIT))
		*val = POWER_SUPPLY_USB_TYPE_DCP;
	else /* SDP_CHARGER_BIT (or others) */
		*val = POWER_SUPPLY_USB_TYPE_SDP;

	return 0;
}

/* Return 1 when in overvoltage state, else 0 or -errno */
static int smbx_ov_status(struct smb_chip *chip)
{
	u16 reg;
	u8 mask;
	int rc;
	u32 val;

	switch (chip->gen) {
	case SMB2:
		reg = BATTERY_CHARGER_STATUS_2;
		mask = SMB2_CHARGER_ERROR_STATUS_BAT_OV_BIT;
		break;
	case SMB5:
		reg = BATTERY_CHARGER_STATUS_2;
		mask = SMB5_CHARGER_ERROR_STATUS_BAT_OV_BIT;
		break;
	}

	rc = regmap_read(chip->regmap, chip->base + reg, &val);
	if (rc)
		return rc;

	return !!(val & mask);
}

static int smb_get_prop_status(struct smb_chip *chip, int *val)
{
	u32 stat;
	int usb_online = 0;
	int rc;

	rc = smb_get_prop_usb_online(chip, &usb_online);
	if (!usb_online) {
		*val = POWER_SUPPLY_STATUS_DISCHARGING;
		return rc;
	}

	rc = regmap_read(chip->regmap,
			      chip->base + BATTERY_CHARGER_STATUS_1, &stat);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to read charging status ret=%d\n",
			rc);
		return rc;
	}

	rc = smbx_ov_status(chip);
	if (rc < 0)
		return rc;

	/* In overvoltage state */
	if (rc == 1) {
		*val = POWER_SUPPLY_STATUS_NOT_CHARGING;
		return 0;
	}

	stat = stat & BATTERY_CHARGER_STATUS_MASK;
	if (chip->gen == SMB5)
		stat = smb5_charger_status[stat];

	switch (stat) {
	case TRICKLE_CHARGE:
	case PRE_CHARGE:
	case FAST_CHARGE:
	case FULLON_CHARGE:
	case TAPER_CHARGE:
		*val = POWER_SUPPLY_STATUS_CHARGING;
		return rc;
	case DISABLE_CHARGE:
		*val = POWER_SUPPLY_STATUS_NOT_CHARGING;
		return rc;
	case TERMINATE_CHARGE:
	case INHIBIT_CHARGE:
		*val = POWER_SUPPLY_STATUS_FULL;
		return rc;
	default:
		*val = POWER_SUPPLY_STATUS_UNKNOWN;
		return rc;
	}
}

static inline int smb_get_current_limit(struct smb_chip *chip,
					 unsigned int *val)
{
	int rc = regmap_read(chip->regmap, chip->base + ICL_STATUS(chip), val);

	if (rc >= 0)
		*val *= chip->current_step_size_ua;
	return rc;
}

static inline int smb_get_current_now(struct smb_chip *chip,
					unsigned int *val)
{
	unsigned int stat;
	bool sink_connected;
	int rc = iio_read_channel_processed(chip->usb_in_i_chan, val);
	if (rc < 0 || chip->gen == SMB2)
		return rc;

	rc = regmap_read(chip->regmap, chip->base + SMB5_TYPE_C_SRC_STATUS_REG, &stat);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read type-C status rc=%d\n", rc);
		return rc;
	}

	switch (stat & SMB5_DETECTED_SNK_TYPE_MASK) {
	case SMB5_AUDIO_ACCESS_RA_RA_BIT:
	case SMB5_SRC_DEBUG_ACCESS_BIT:
	case SMB5_SRC_RD_RA_VCONN_BIT:
	case SMB5_SRC_RD_OPEN_BIT:
		sink_connected = true;
		break;
	default:
		sink_connected = false;
		break;
	}

	/*
	 * For PM8150B, scaling factor = reciprocal of
	 * 0.2V/A in Buck mode, 0.4V/A in Boost mode.
	 * For PMI632, scaling factor = reciprocal of
	 * 0.4V/A in Buck mode, 0.8V/A in Boost mode.
	 */
	// TODO: Make member in match data
	int buck_scale = 20;
	int boost_scale = 40;

	if (sink_connected) {
		*val = DIV_ROUND_CLOSEST(*val * 100, boost_scale);
		return rc;
	}

	if (!smb_is_charging(chip)) {
		*val = 0;
		return rc;
	}

	*val = DIV_ROUND_CLOSEST(*val * 100, buck_scale);

	return rc;
}

static int smb_set_current_limit(struct smb_chip *chip, unsigned int val)
{
	unsigned char val_raw;

	if (val > chip->current_limit_max_ua) {
		dev_err(chip->dev,
			"Can't set current limit higher than %u uA\n", chip->current_limit_max_ua);
		return -EINVAL;
	}
	val_raw = val / chip->current_step_size_ua;

	return regmap_write(chip->regmap, chip->base + USBIN_CURRENT_LIMIT_CFG,
			    val_raw);
}

static unsigned int smb5_typec_current_ua(struct smb_chip *chip)
{
	unsigned int stat;

	if (chip->gen != SMB5 ||
	    regmap_read(chip->regmap, chip->base + SMB5_TYPEC_LEGACY_CABLE_STATUS_REG, &stat) ||
	    (stat & SMB5_TYPEC_LEGACY_CABLE_BIT) ||
	    regmap_read(chip->regmap, chip->base + SMB5_TYPEC_SNK_STATUS_REG, &stat))
		return 0;

	if (stat & SMB5_SNK_RP_3P0_BIT)
		return TYPEC_HIGH_CURRENT_UA;
	if (stat & SMB5_SNK_RP_1P5_BIT)
		return TYPEC_MEDIUM_CURRENT_UA;
	return 0;
}

static void smb_status_change_work(struct work_struct *work)
{
	unsigned int charger_type, current_ua;
	int usb_online = 0;
	int count, rc;
	struct smb_chip *chip;

	chip = container_of(work, struct smb_chip, status_change_work.work);

	smb_get_prop_usb_online(chip, &usb_online);
	if (!usb_online)
		return;

	for (count = 0; count < 3; count++) {
		dev_dbg(chip->dev, "get charger type retry %d\n", count);
		rc = smb_apsd_get_charger_type(chip, &charger_type);
		if (rc != -EAGAIN)
			break;
		msleep(100);
	}

	if (rc < 0 && rc != -EAGAIN) {
		dev_err(chip->dev, "get charger type failed: %d\n", rc);
		return;
	}

	if (rc < 0) {
		rc = regmap_update_bits(chip->regmap, chip->base + CMD_APSD,
					APSD_RERUN_BIT, APSD_RERUN_BIT);
		schedule_delayed_work(&chip->status_change_work,
				      msecs_to_jiffies(1000));
		dev_dbg(chip->dev, "get charger type failed, rerun apsd\n");
		return;
	}

	switch (charger_type) {
	case POWER_SUPPLY_USB_TYPE_CDP:
		current_ua = CDP_CURRENT_UA;
		break;
	case POWER_SUPPLY_USB_TYPE_DCP:
		current_ua = chip->batt_info->constant_charge_current_max_ua;
		break;
	case POWER_SUPPLY_USB_TYPE_SDP:
	default:
		current_ua = SDP_CURRENT_UA;
		break;
	}

	current_ua = max(current_ua, smb5_typec_current_ua(chip));
	smb_set_current_limit(chip, current_ua);
	power_supply_changed(chip->chg_psy);
}

static int smb_get_iio_chan(struct smb_chip *chip, struct iio_channel *chan,
			     int *val)
{
	if (!smb_is_charging(chip)) {
		*val = 0;
		return 0;
	}

	if (IS_ERR(chan)) {
		dev_err(chip->dev, "Failed to chan, err = %li", PTR_ERR(chan));
		return PTR_ERR(chan);
	}

	return iio_read_channel_processed(chan, val);
}

static int smb5_get_prop_health(struct smb_chip *chip, int *val)
{
	int rc;
	unsigned int stat;

	rc = smbx_ov_status(chip);

	/* Treat any error as if we are in the overvoltage state */
	if (rc < 0)
		dev_err(chip->dev, "Couldn't determine overvoltage status!");
	if (rc) {
		dev_err(chip->dev, "battery over-voltage");
		*val = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		return 0;
	}

	rc = regmap_read(chip->regmap, chip->base + BATTERY_CHARGER_STATUS_7,
			 &stat);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read charger status 7 rc=%d\n", rc);
		return rc;
	}

	if (stat & SMB5_BAT_TEMP_STATUS_TOO_COLD_BIT)
		*val = POWER_SUPPLY_HEALTH_COLD;
	else if (stat & SMB5_BAT_TEMP_STATUS_TOO_HOT_BIT)
		*val = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (stat & SMB5_BAT_TEMP_STATUS_COLD_SOFT_BIT)
		*val = POWER_SUPPLY_HEALTH_COOL;
	else if (stat & SMB5_BAT_TEMP_STATUS_HOT_SOFT_BIT)
		*val = POWER_SUPPLY_HEALTH_WARM;
	else
		*val = POWER_SUPPLY_HEALTH_GOOD;

	return 0;
}

static int smb2_get_prop_health(struct smb_chip *chip, int *val)
{
	int rc;
	unsigned int stat;

	rc = regmap_read(chip->regmap, chip->base + BATTERY_CHARGER_STATUS_2,
			 &stat);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read charger status rc=%d\n", rc);
		return rc;
	}

	if (stat & SMB2_CHARGER_ERROR_STATUS_BAT_OV_BIT)
		*val = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	else if (stat & SMB2_BAT_TEMP_STATUS_TOO_COLD_BIT)
		*val = POWER_SUPPLY_HEALTH_COLD;
	else if (stat & SMB2_BAT_TEMP_STATUS_TOO_HOT_BIT)
		*val = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (stat & SMB2_BAT_TEMP_STATUS_COLD_SOFT_LIMIT_BIT)
		*val = POWER_SUPPLY_HEALTH_COOL;
	else if (stat & SMB2_BAT_TEMP_STATUS_HOT_SOFT_LIMIT_BIT)
		*val = POWER_SUPPLY_HEALTH_WARM;
	else
		*val = POWER_SUPPLY_HEALTH_GOOD;

	return 0;
}

static int smb_get_prop_health(struct smb_chip *chip, int *val)
{
	switch (chip->gen) {
	case SMB2:
		return smb2_get_prop_health(chip, val);
	case SMB5:
		return smb5_get_prop_health(chip, val);
	}

	return -EINVAL;
}

static int smb_batt_capacity(struct power_supply *psy, void *data)
{
	union power_supply_propval val;

	if (psy->desc->type != POWER_SUPPLY_TYPE_BATTERY ||
	    power_supply_get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &val))
		return 0;

	*(int *)data = val.intval;
	return 1;
}

/*
 * Stop charging once the battery reaches limit_end and resume at limit_start or below.
 * The capacity is read here, every LIMIT_POLL_MS while a limit is set and the input is
 * connected, and the system is kept awake as long as the battery charges towards the limit.
 */
static void smb_limit_work(struct work_struct *work)
{
	struct smb_chip *chip = container_of(to_delayed_work(work), struct smb_chip, limit_work);
	int capacity = -1, online = 0, rc;
	bool stop, changed = false, poll;

	smb_get_prop_usb_online(chip, &online);
	power_supply_for_each_psy(&capacity, smb_batt_capacity);

	mutex_lock(&chip->limit_lock);
	stop = chip->limit_stopped;
	if (chip->limit_end >= 100 || capacity < 0)
		stop = false;
	else if (capacity >= chip->limit_end)
		stop = true;
	else if (capacity <= chip->limit_start)
		stop = false;
	if (stop != chip->limit_stopped) {
		rc = regmap_update_bits(chip->regmap, chip->base + CHARGING_ENABLE_CMD,
					CHARGING_ENABLE_CMD_BIT, stop ? 0 : CHARGING_ENABLE_CMD_BIT);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't %s charging: %d\n", stop ? "stop" : "resume", rc);
		} else {
			chip->limit_stopped = stop;
			changed = true;
			dev_info(chip->dev, "charging %s at %d%%\n", stop ? "stopped" : "resumed",
				 capacity);
		}
	}
	poll = online && chip->limit_end < 100;
	if (poll && !chip->limit_stopped)
		pm_stay_awake(chip->dev);
	else
		pm_relax(chip->dev);
	mutex_unlock(&chip->limit_lock);

	if (changed)
		power_supply_changed(chip->chg_psy);
	if (poll)
		schedule_delayed_work(&chip->limit_work, msecs_to_jiffies(LIMIT_POLL_MS));
}

static int smb_set_limit(struct smb_chip *chip, bool end, int val)
{
	if (val < 0 || val > 100)
		return -EINVAL;
	mutex_lock(&chip->limit_lock);
	if (val == 100) {
		chip->limit_start = 100;
		chip->limit_end = 100;
	} else if (end) {
		chip->limit_end = max(val, 1);
		if (chip->limit_start >= chip->limit_end)
			chip->limit_start = chip->limit_end - 1;
	} else {
		chip->limit_start = val;
		if (chip->limit_end <= val)
			chip->limit_end = val + 1;
	}
	mutex_unlock(&chip->limit_lock);
	mod_delayed_work(system_wq, &chip->limit_work, 0);
	return 0;
}

static int smb_get_property(struct power_supply *psy,
			     enum power_supply_property psp,
			     union power_supply_propval *val)
{
	struct smb_chip *chip = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "Qualcomm";
		return 0;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = chip->name;
		return 0;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		return smb_get_current_limit(chip, &val->intval);
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		return smb_get_current_now(chip, &val->intval);
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		return smb_get_iio_chan(chip, chip->usb_in_v_chan,
					 &val->intval);
	case POWER_SUPPLY_PROP_ONLINE:
		return smb_get_prop_usb_online(chip, &val->intval);
	case POWER_SUPPLY_PROP_STATUS:
		return smb_get_prop_status(chip, &val->intval);
	case POWER_SUPPLY_PROP_HEALTH:
		return smb_get_prop_health(chip, &val->intval);
	case POWER_SUPPLY_PROP_USB_TYPE:
		return smb_apsd_get_charger_type(chip, &val->intval);
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
		val->intval = chip->limit_start;
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		val->intval = chip->limit_end;
		return 0;
	default:
		dev_err(chip->dev, "invalid property: %d\n", psp);
		return -EINVAL;
	}
}

static int smb_set_property(struct power_supply *psy,
			     enum power_supply_property psp,
			     const union power_supply_propval *val)
{
	struct smb_chip *chip = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		return regmap_update_bits(chip->regmap, chip->base + USBIN_CMD_IL,
					  USBIN_SUSPEND_BIT, !val->intval);
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		return smb_set_current_limit(chip, val->intval);
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
		return smb_set_limit(chip, false, val->intval);
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		return smb_set_limit(chip, true, val->intval);
	default:
		dev_err(chip->dev, "No setter for property: %d\n", psp);
		return -EINVAL;
	}
}

static int smb_property_is_writable(struct power_supply *psy,
				     enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		return 1;
	default:
		return 0;
	}
}

static irqreturn_t smb_handle_batt_overvoltage(int irq, void *data)
{
	struct smb_chip *chip = data;
	int rc;

	rc = smbx_ov_status(chip);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read charger status: %d\n", rc);
		return IRQ_HANDLED;
	}

	if (rc == 1) {
		/* The hardware stops charging automatically */
		dev_err(chip->dev, "battery overvoltage detected\n");
	}

	power_supply_changed(chip->chg_psy);

	return IRQ_HANDLED;
}

static irqreturn_t smb_handle_usb_plugin(int irq, void *data)
{
	struct smb_chip *chip = data;

	power_supply_changed(chip->chg_psy);

	schedule_delayed_work(&chip->status_change_work,
			      msecs_to_jiffies(1500));
	mod_delayed_work(system_wq, &chip->limit_work, 0);

	return IRQ_HANDLED;
}

static irqreturn_t smb_handle_usb_icl_change(int irq, void *data)
{
	struct smb_chip *chip = data;

	power_supply_changed(chip->chg_psy);

	return IRQ_HANDLED;
}

static irqreturn_t smb_handle_wdog_bark(int irq, void *data)
{
	struct smb_chip *chip = data;
	int rc;

	power_supply_changed(chip->chg_psy);

	rc = regmap_write(chip->regmap, chip->base + BARK_BITE_WDOG_PET,
			  BARK_BITE_WDOG_PET_BIT);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't pet the dog rc=%d\n", rc);

	return IRQ_HANDLED;
}

static const struct power_supply_desc smb_psy_desc = {
	.name = "SMB2_charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.usb_types = BIT(POWER_SUPPLY_USB_TYPE_SDP) |
		     BIT(POWER_SUPPLY_USB_TYPE_CDP) |
		     BIT(POWER_SUPPLY_USB_TYPE_DCP) |
		     BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN),
	.properties = smb_properties,
	.num_properties = ARRAY_SIZE(smb_properties),
	.get_property = smb_get_property,
	.set_property = smb_set_property,
	.property_is_writeable = smb_property_is_writable,
};

/* Init sequence derived from vendor downstream driver */
static const struct smb_init_register smb5_init_seq[] = {
	{ .addr = USBIN_CMD_IL, .mask = USBIN_SUSPEND_BIT, .val = 0 },
	{ .addr = TYPE_C_CFG,
	  .mask = APSD_START_ON_CC_BIT,
	  .val = 0 },
	/*
	 * Recharge when State Of Charge drops below 98%.
	 */
	{ .addr = SMB5_CHARGE_RCHG_SOC_THRESHOLD_CFG_REG,
	  .mask = SMB5_CHARGE_RCHG_SOC_THRESHOLD_CFG_MASK,
	  .val = 250 },
	/* Enable charging */
	{ .addr = CHARGING_ENABLE_CMD,
	  .mask = CHARGING_ENABLE_CMD_BIT,
	  .val = CHARGING_ENABLE_CMD_BIT },
	/* Enable BC1P2 auto Src detect */
	{ .addr = USBIN_OPTIONS_1_CFG,
	  .mask = AUTO_SRC_DETECT_BIT,
	  .val = AUTO_SRC_DETECT_BIT },
	/* Set the default SDP charger type to a 500ma USB 2.0 port */
	{ .addr = USBIN_ICL_OPTIONS,
	  .mask = USBIN_MODE_CHG_BIT,
	  .val = USBIN_MODE_CHG_BIT },
	{ .addr = CMD_ICL_OVERRIDE,
	  .mask = ICL_OVERRIDE_BIT,
	  .val = 0 },
	{ .addr = USBIN_LOAD_CFG,
	  .mask = ICL_OVERRIDE_AFTER_APSD_BIT,
	  .val = 0 },
	/* Disable watchdog */
	{ .addr = SNARL_BARK_BITE_WD_CFG, .mask = 0xff, .val = 0 },
	{ .addr = WD_CFG,
	  .mask = WATCHDOG_TRIGGER_AFP_EN_BIT | WDOG_TIMER_EN_ON_PLUGIN_BIT |
		  BARK_WDOG_INT_EN_BIT,
	  .val = 0 },
	/*
	 * Enable Automatic Input Current Limit, this will slowly ramp up the current
	 * When connected to a wall charger, and automatically stop when it detects
	 * the charger current limit (voltage drop?) or it reaches the programmed limit.
	 */
	{ .addr = USBIN_AICL_OPTIONS_CFG,
	  .mask = USBIN_AICL_PERIODIC_RERUN_EN_BIT | USBIN_AICL_ADC_EN_BIT
			| USBIN_AICL_EN_BIT | SUSPEND_ON_COLLAPSE_USBIN_BIT,
	  .val = USBIN_AICL_PERIODIC_RERUN_EN_BIT
			| USBIN_AICL_EN_BIT | SUSPEND_ON_COLLAPSE_USBIN_BIT },
};

/* Init sequence derived from vendor downstream driver */
static const struct smb_init_register smb2_init_seq[] = {
	/*
	 * By default configure us as an upstream facing port
	 * FIXME: This will be handled by the type-c driver
	 */
	{ .addr = TYPE_C_INTRPT_ENB_SOFTWARE_CTRL,
	  .mask = TYPEC_POWER_ROLE_CMD_MASK | SMB2_VCONN_EN_SRC_BIT |
		  VCONN_EN_VALUE_BIT,
	  .val = SMB2_VCONN_EN_SRC_BIT },
	/*
	 * Disable Type-C factory mode and stay in Attached.SRC state when VCONN
	 * over-current happens
	 */
	{ .addr = TYPE_C_CFG,
	  .mask = FACTORY_MODE_DETECTION_EN_BIT | VCONN_OC_CFG_BIT,
	  .val = 0 },
	/* Configure VBUS for software control */
	{ .addr = OTG_CFG, .mask = OTG_EN_SRC_CFG_BIT, .val = 0 },
	/*
	 * Use VBAT to determine the recharge threshold when battery is full
	 * rather than the state of charge.
	 */
	{ .addr = SMB2_FG_UPDATE_CFG_2_SEL,
	  .mask = SMB2_SOC_LT_CHG_RECHARGE_THRESH_SEL_BIT |
		  SMB2_VBT_LT_CHG_RECHARGE_THRESH_SEL_BIT,
	  .val = SMB2_VBT_LT_CHG_RECHARGE_THRESH_SEL_BIT },
	{ .addr = USBIN_OPTIONS_1_CFG, .mask = HVDCP_EN_BIT, .val = 0 },
	/*
	 * Match downstream defaults
	 * CHG_EN_SRC_BIT - charger enable is controlled by software
	 * CHG_EN_POLARITY_BIT - polarity of charge enable pin when in HW control
	 *                       pulled low on OnePlus 6 and SHIFT6mq
	 * PRETOFAST_TRANSITION_CFG_BIT -
	 * BAT_OV_ECC_BIT -
	 * I_TERM_BIT - Current termination ?? 0 = enabled
	 * AUTO_RECHG_BIT - Enable automatic recharge when battery is full
	 *                  0 = enabled
	 * EN_ANALOG_DROP_IN_VBATT_BIT
	 * CHARGER_INHIBIT_BIT - Inhibit charging based on battery voltage
	 *                       instead of ??
	 */
	{ .addr = CHGR_CFG2,
	  .mask = CHG_EN_SRC_BIT | CHG_EN_POLARITY_BIT |
		  PRETOFAST_TRANSITION_CFG_BIT | BAT_OV_ECC_BIT | I_TERM_BIT |
		  AUTO_RECHG_BIT | EN_ANALOG_DROP_IN_VBATT_BIT |
		  CHARGER_INHIBIT_BIT,
	  .val = CHARGER_INHIBIT_BIT },
	/*
	 * Set the enable command only once CHGR_CFG2 has selected software
	 * control, or a charger its watchdog has disabled stays disabled.
	 */
	{ .addr = CHARGING_ENABLE_CMD,
	  .mask = CHARGING_ENABLE_CMD_BIT,
	  .val = CHARGING_ENABLE_CMD_BIT },
	/* STAT pin software override, match downstream. Parallel charging? */
	{ .addr = STAT_CFG,
	  .mask = STAT_SW_OVERRIDE_CFG_BIT,
	  .val = STAT_SW_OVERRIDE_CFG_BIT },
	/* Set the default SDP charger type to a 500ma USB 2.0 port */
	{ .addr = USBIN_ICL_OPTIONS,
	  .mask = USB51_MODE_BIT | USBIN_MODE_CHG_BIT,
	  .val = USB51_MODE_BIT },
	/* Disable watchdog */
	{ .addr = SNARL_BARK_BITE_WD_CFG, .mask = 0xff, .val = 0 },
	{ .addr = WD_CFG,
	  .mask = WATCHDOG_TRIGGER_AFP_EN_BIT | WDOG_TIMER_EN_ON_PLUGIN_BIT |
		  BARK_WDOG_INT_EN_BIT,
	  .val = 0 },
	/* These bits aren't documented anywhere */
	{ .addr = USBIN_5V_AICL_THRESHOLD_CFG,
	  .mask = USBIN_5V_AICL_THRESHOLD_CFG_MASK,
	  .val = 0x3 },
	{ .addr = USBIN_CONT_AICL_THRESHOLD_CFG,
	  .mask = USBIN_CONT_AICL_THRESHOLD_CFG_MASK,
	  .val = 0x3 },
	/*
	 * Enable Automatic Input Current Limit, this will slowly ramp up the current
	 * When connected to a wall charger, and automatically stop when it detects
	 * the charger current limit (voltage drop?) or it reaches the programmed limit.
	 */
	{ .addr = USBIN_AICL_OPTIONS_CFG,
	  .mask = USBIN_AICL_START_AT_MAX_BIT | USBIN_AICL_ADC_EN_BIT |
		  USBIN_AICL_EN_BIT | SUSPEND_ON_COLLAPSE_USBIN_BIT |
		  USBIN_HV_COLLAPSE_RESPONSE_BIT |
		  USBIN_LV_COLLAPSE_RESPONSE_BIT,
	  .val = USBIN_HV_COLLAPSE_RESPONSE_BIT |
		 USBIN_LV_COLLAPSE_RESPONSE_BIT | USBIN_AICL_EN_BIT },
	/*
	 * Set pre charge current to default, the OnePlus 6 bootloader
	 * sets this very conservatively.
	 */
	{ .addr = PRE_CHARGE_CURRENT_CFG,
	  .mask = PRE_CHARGE_CURRENT_SETTING_MASK,
	  /* 20 * step size 25000 uA = 500000 uA */
	  .val = 20 },
};

struct smb_match_data pmi8998_match_data = {
	.init_seq = smb2_init_seq,
	.init_seq_len = ARRAY_SIZE(smb2_init_seq),
	.name = "pmi8998",
	.gen = SMB2,
	.current_step_size_ua = 25000,
	.current_limit_max_ua = 4800000,
};

struct smb_match_data pm660_match_data = {
	.init_seq = smb2_init_seq,
	.init_seq_len = ARRAY_SIZE(smb2_init_seq),
	.name = "pm660",
	.gen = SMB2,
	.current_step_size_ua = 25000,
	.current_limit_max_ua = 4800000,
};

struct smb_match_data pm8150b_match_data = {
	.init_seq = smb5_init_seq,
	.init_seq_len = ARRAY_SIZE(smb5_init_seq),
	.name = "pm8150b",
	.gen = SMB5,
	.current_step_size_ua = 50000,
	.current_limit_max_ua = 5000000,
};

struct smb_match_data pm7250b_match_data = {
	.init_seq = smb5_init_seq,
	.init_seq_len = ARRAY_SIZE(smb5_init_seq),
	.name = "pm7250b",
	.gen = SMB5,
	.current_step_size_ua = 50000,
	.current_limit_max_ua = 5000000,
};


static int smb_init_hw(struct smb_chip *chip, const struct smb_init_register *init_seq, size_t len)
{
	int rc, i;

	for (i = 0; i < len; i++) {
		dev_dbg(chip->dev, "%d: Writing 0x%02x to 0x%02x\n", i,
			init_seq[i].val, init_seq[i].addr);
		rc = regmap_update_bits(chip->regmap,
					chip->base + init_seq[i].addr,
					init_seq[i].mask,
					init_seq[i].val);
		if (rc < 0)
			return dev_err_probe(chip->dev, rc,
					     "%s: init command %d failed\n",
					     __func__, i);
	}

	return 0;
}

static void smb_restore_charge_enable(void *data)
{
	struct smb_chip *chip = data;
	int rc;

	rc = regmap_update_bits(chip->regmap,
				chip->base + CHARGING_ENABLE_CMD,
				CHARGING_ENABLE_CMD_BIT,
				chip->initial_charge_enable);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't restore charging state: %d\n", rc);
}

static int smb_init_irq(struct smb_chip *chip, int *irq, const char *name,
			 irqreturn_t (*handler)(int irq, void *data))
{
	int irqnum;
	int rc;

	irqnum = platform_get_irq_byname(to_platform_device(chip->dev), name);
	if (irqnum < 0)
		return irqnum;

	rc = devm_request_threaded_irq(chip->dev, irqnum, NULL, handler,
				       IRQF_ONESHOT, name, chip);
	if (rc < 0)
		return dev_err_probe(chip->dev, rc, "Couldn't request irq %s\n",
				     name);

	if (irq)
		*irq = irqnum;

	return 0;
}

static int smb_probe(struct platform_device *pdev)
{
	struct power_supply_config supply_config = {};
	struct power_supply_desc *desc;
	struct smb_chip *chip;
	const struct smb_match_data *match_data;
	unsigned int charge_enable;
	unsigned int float_voltage_sel;
	int float_voltage_uv, float_min_uv, float_max_uv, float_step_uv;
	unsigned int fcc_ua = 1950000;
	int rc, irq;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &pdev->dev;
	chip->name = pdev->name;
	chip->limit_start = 100;
	chip->limit_end = 100;
	mutex_init(&chip->limit_lock);

	chip->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!chip->regmap)
		return dev_err_probe(chip->dev, -ENODEV,
				     "failed to locate the regmap\n");

	rc = device_property_read_u32(chip->dev, "reg", &chip->base);
	if (rc < 0)
		return dev_err_probe(chip->dev, rc,
				     "Couldn't read base address\n");

	chip->usb_in_v_chan = devm_iio_channel_get(chip->dev, "usbin_v");
	if (IS_ERR(chip->usb_in_v_chan))
		return dev_err_probe(chip->dev, PTR_ERR(chip->usb_in_v_chan),
				     "Couldn't get usbin_v IIO channel\n");

	chip->usb_in_i_chan = devm_iio_channel_get(chip->dev, "usbin_i");
	if (IS_ERR(chip->usb_in_i_chan)) {
		return dev_err_probe(chip->dev, PTR_ERR(chip->usb_in_i_chan),
				     "Couldn't get usbin_i IIO channel\n");
	}

	rc = regmap_read(chip->regmap, chip->base + CHARGING_ENABLE_CMD,
			 &charge_enable);
	if (rc < 0)
		return dev_err_probe(chip->dev, rc,
				     "Couldn't read charging state\n");

	chip->initial_charge_enable = charge_enable & CHARGING_ENABLE_CMD_BIT;
	rc = devm_add_action_or_reset(chip->dev, smb_restore_charge_enable, chip);
	if (rc)
		return dev_err_probe(chip->dev, rc,
				     "Couldn't register charging state rollback\n");

	match_data = (const struct smb_match_data *)device_get_match_data(chip->dev);

	chip->gen = match_data->gen;
	chip->current_step_size_ua = match_data->current_step_size_ua;
	chip->current_limit_max_ua = match_data->current_limit_max_ua;

	dev_info(chip->dev, "Generation %s\n", chip->gen == SMB2 ? "SMB2" : "SMB5");

	rc = smb_init_hw(chip, match_data->init_seq, match_data->init_seq_len);
	if (rc < 0)
		return rc;

	supply_config.drv_data = chip;
	supply_config.fwnode = dev_fwnode(&pdev->dev);

	desc = devm_kzalloc(chip->dev, sizeof(smb_psy_desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;
	memcpy(desc, &smb_psy_desc, sizeof(smb_psy_desc));
	desc->name =
		devm_kasprintf(chip->dev, GFP_KERNEL, "%s-charger",
			       match_data->name);
	if (!desc->name)
		return -ENOMEM;

	rc = devm_delayed_work_autocancel(chip->dev, &chip->limit_work, smb_limit_work);
	if (rc)
		return rc;

	chip->chg_psy =
		devm_power_supply_register(chip->dev, desc, &supply_config);
	if (IS_ERR(chip->chg_psy))
		return dev_err_probe(chip->dev, PTR_ERR(chip->chg_psy),
				     "failed to register power supply\n");

	rc = power_supply_get_battery_info(chip->chg_psy, &chip->batt_info);
	if (rc)
		return dev_err_probe(chip->dev, rc,
				     "Failed to get battery info\n");
	if (chip->batt_info->constant_charge_current_max_ua > 0)
		fcc_ua = min_t(unsigned int, fcc_ua, chip->batt_info->constant_charge_current_max_ua);
	if (chip->batt_info->constant_charge_current_max_ua == -EINVAL)
		chip->batt_info->constant_charge_current_max_ua = DCP_CURRENT_UA;

	rc = devm_delayed_work_autocancel(chip->dev, &chip->status_change_work,
					  smb_status_change_work);
	if (rc)
		return dev_err_probe(chip->dev, rc,
				     "Failed to init status change work\n");

	if (power_supply_battery_info_has_prop(chip->batt_info,
					       POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX)) {
		float_voltage_uv =
			chip->batt_info->constant_charge_voltage_max_uv;
	} else if (power_supply_battery_info_has_prop(chip->batt_info,
					      POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN)) {
		float_voltage_uv = chip->batt_info->voltage_max_design_uv;
	} else {
		dev_warn(chip->dev, "No battery float voltage; preserving hardware setting\n");
		goto skip_float_voltage;
	}

	if (chip->gen == SMB5) {
		float_min_uv = SMB5_FLOAT_VOLTAGE_MIN_UV;
		float_max_uv = SMB5_FLOAT_VOLTAGE_MAX_UV;
		float_step_uv = SMB5_FLOAT_VOLTAGE_STEP_UV;
	} else {
		float_min_uv = SMB2_FLOAT_VOLTAGE_MIN_UV;
		float_max_uv = SMB2_FLOAT_VOLTAGE_MAX_UV;
		float_step_uv = SMB2_FLOAT_VOLTAGE_STEP_UV;
	}

	if (float_voltage_uv < float_min_uv || float_voltage_uv > float_max_uv)
		return dev_err_probe(chip->dev, -EINVAL,
				     "float voltage %d uV outside %d-%d uV\n",
				     float_voltage_uv, float_min_uv, float_max_uv);

	float_voltage_sel = (float_voltage_uv - float_min_uv) / float_step_uv;
	rc = regmap_update_bits(chip->regmap, chip->base + FLOAT_VOLTAGE_CFG,
				FLOAT_VOLTAGE_SETTING_MASK, float_voltage_sel);
	if (rc < 0)
		return dev_err_probe(chip->dev, rc, "Couldn't set vbat max\n");

skip_float_voltage:
	rc = smb_init_irq(chip, &irq, "bat-ov", smb_handle_batt_overvoltage);
	if (rc < 0)
		return rc;

	rc = smb_init_irq(chip, &chip->cable_irq, "usb-plugin",
			   smb_handle_usb_plugin);
	if (rc < 0)
		return rc;

	rc = smb_init_irq(chip, &irq, "usbin-icl-change",
			   smb_handle_usb_icl_change);
	if (rc < 0)
		return rc;
	rc = smb_init_irq(chip, &irq, "wdog-bark", smb_handle_wdog_bark);
	if (rc < 0)
		return rc;

	devm_device_init_wakeup(chip->dev);

	rc = devm_pm_set_wake_irq(chip->dev, chip->cable_irq);
	if (rc < 0)
		return dev_err_probe(chip->dev, rc, "Couldn't set wake irq\n");

	platform_set_drvdata(pdev, chip);

	/*
	 * This overrides all of the other current limits and is expected
	 * to be used for setting limits based on temperature. We set some
	 * relatively safe default value while still allowing a comfortably
	 * fast charging rate. Once temperature monitoring is hooked up we
	 * would expect this to be changed dynamically based on temperature
	 * reporting.
	 */
	rc = regmap_write(chip->regmap, chip->base + FAST_CHARGE_CURRENT_CFG,
			  fcc_ua / chip->current_step_size_ua);
	if (rc < 0)
		return dev_err_probe(chip->dev, rc,
				     "Couldn't write fast charge current cfg");

	rc = regmap_write_bits(chip->regmap, chip->base + AICL_RERUN_TIME_CFG,
			       AICL_RERUN_TIME_MASK, AIC_RERUN_TIME_3_SECS);
	if (rc < 0)
		return dev_err_probe(chip->dev, rc,
				     "Couldn't write fast AICL rerun time");

	devm_remove_action(chip->dev, smb_restore_charge_enable, chip);

	/* Initialise charger state */
	schedule_delayed_work(&chip->status_change_work, 0);

	return 0;
}

static void smb_shutdown(struct platform_device *pdev)
{
	struct smb_chip *chip = platform_get_drvdata(pdev);

	smb_set_limit(chip, true, 100);
	flush_delayed_work(&chip->limit_work);
}

static const struct of_device_id smb_match_id_table[] = {
	{ .compatible = "qcom,pmi8998-charger", .data = &pmi8998_match_data },
	{ .compatible = "qcom,pm660-charger", .data = &pm660_match_data },
	{ .compatible = "qcom,pm7250b-charger", .data = &pm7250b_match_data },
	{ .compatible = "qcom,pm8150b-charger", .data = &pm8150b_match_data },
	{ /* sentinal */ }
};
MODULE_DEVICE_TABLE(of, smb_match_id_table);

static struct platform_driver qcom_spmi_smb = {
	.probe = smb_probe,
	.shutdown = smb_shutdown,
	.driver = {
		.name = "qcom-smbx-charger",
		.of_match_table = smb_match_id_table,
		},
};

module_platform_driver(qcom_spmi_smb);

MODULE_AUTHOR("Casey Connolly <casey.connolly@linaro.org>");
MODULE_DESCRIPTION("Qualcomm SMB2 Charger Driver");
MODULE_LICENSE("GPL");
