// SPDX-License-Identifier: GPL-2.0-only
/*
 * Istaric IS5662 touch controller on the Onyx Boox Go 7.
 *
 * Reimplemented from the vendor driver (istaric_* and onyx_tp_*),
 * no stylus support as I don't have one to test with
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/unaligned.h>

#define IS_ADDR_CHIPID		0x180E8008
#define IS_LEN_CHIPID		0x12
#define IS_ADDR_REQ		0x180E801C
#define IS_ADDR_REPLY		0x180E802C
#define IS_ADDR_INFO		0x180E803C
#define IS_LEN_INFO		0x20
#define IS_MAILBOX_SIZE		0x10
#define IS_CHUNK		246
#define IS_BUF_SIZE		0x2800

#define IS_CMD_ENTER_BLOCK	0x01
#define IS_CMD_EXIT_BLOCK	0x02
#define IS_CMD_BUFFER_INFO	0x03
#define IS_CMD_DATA_INFO	0x04
#define IS_CMD_MODULE_INFO	0x09
#define IS_CMD_FW_INFO		0x1e
#define IS_CMD_REPORT_DESC	0x2e

#define IS_MAX_SLOTS		10

struct is5662 {
	struct i2c_client *client;
	struct input_dev *input;
	struct gpio_desc *rst, *vdd_en, *vcc_en;
	struct regulator *vdd, *vcc;
	u8 *buf;
	u32 block_addr, block_size;
	u32 touch_addr, report_size;
	u8 bitmap[8];
	u8 max_fingers, max_pens;
	u16 res_x, res_y;
	u32 screen_x, screen_y, finger_flag;
	bool slow_reset;
	int irq;
};

static int is_read(struct is5662 *ts, u32 addr, u8 *buf, u32 len)
{
	while (len) {
		u32 n = min_t(u32, len, IS_CHUNK);
		__be32 reg = cpu_to_be32(addr);
		struct i2c_msg msgs[] = {
			{ .addr = ts->client->addr, .len = 4, .buf = (u8 *)&reg },
			{ .addr = ts->client->addr, .flags = I2C_M_RD, .len = n, .buf = buf },
		};
		int ret = i2c_transfer(ts->client->adapter, msgs, 2);

		if (ret != 2)
			return ret < 0 ? ret : -EIO;
		addr += n;
		buf += n;
		len -= n;
	}
	return 0;
}

static int is_write(struct is5662 *ts, u32 addr, const u8 *data, u32 len)
{
	struct i2c_msg msg = { .addr = ts->client->addr };
	u8 *b = kmalloc(len + 4, GFP_KERNEL);
	int ret;

	if (!b)
		return -ENOMEM;
	put_unaligned_be32(addr, b);
	memcpy(b + 4, data, len);
	msg.len = len + 4;
	msg.buf = b;
	ret = i2c_transfer(ts->client->adapter, &msg, 1);
	kfree(b);
	return ret == 1 ? 0 : (ret < 0 ? ret : -EIO);
}

static u16 is_sum(const u8 *b, u32 n)
{
	u16 s = 0;

	while (n--)
		s += *b++;
	return s;
}

static bool is_frame_ok(const u8 *b, u32 max)
{
	u16 len = get_unaligned_le16(b);

	return len >= 4 && len <= max && get_unaligned_le16(b + len - 2) == is_sum(b, len - 2);
}

static int is_wait_ack(struct is5662 *ts, unsigned int timeout_ms)
{
	u8 hdr[8];
	unsigned int i;

	for (i = 0; i < timeout_ms / 10; i++) {
		usleep_range(10000, 10100);
		if (is_read(ts, IS_ADDR_REQ, hdr, sizeof(hdr)))
			continue;
		if (!hdr[0] && !hdr[1])
			return 0;
	}
	return -ETIMEDOUT;
}

static int is_send(struct is5662 *ts, u16 cmd, const u8 *payload, u16 plen)
{
	u8 pkt[IS_MAILBOX_SIZE];
	u16 len = plen + 6;
	int ret;

	if (len + 2 > sizeof(pkt))
		return -EINVAL;
	put_unaligned_le16(0xffff, pkt);
	put_unaligned_le16(len, pkt + 2);
	put_unaligned_le16(cmd, pkt + 4);
	if (plen)
		memcpy(pkt + 6, payload, plen);
	put_unaligned_le16(is_sum(pkt + 2, len - 2), pkt + len);
	ret = is_write(ts, IS_ADDR_REQ, pkt, len + 2);
	return ret ? ret : is_wait_ack(ts, 400);
}

static int is_request_once(struct is5662 *ts, u16 cmd, const u8 *payload, u16 plen, u32 want)
{
	bool block = want + 6 > IS_MAILBOX_SIZE;
	int ret;

	if (block) {
		if (!ts->block_addr || !ts->block_size)
			return -EINVAL;
		ret = is_send(ts, IS_CMD_ENTER_BLOCK, NULL, 0);
		if (ret)
			return ret;
	}
	ret = is_send(ts, cmd, payload, plen);
	if (ret || !want)
		goto out;

	if (block) {
		u32 len;

		ret = is_read(ts, ts->block_addr, ts->buf, 2);
		if (ret)
			goto out;
		len = max_t(u32, get_unaligned_le16(ts->buf), 0x33);
		ret = is_read(ts, ts->block_addr, ts->buf, min_t(u32, len, IS_BUF_SIZE));
	} else {
		ret = is_read(ts, IS_ADDR_REPLY, ts->buf, want + 6);
	}
	if (!ret && !is_frame_ok(ts->buf, IS_BUF_SIZE))
		ret = -EBADMSG;
out:
	if (block)
		is_send(ts, IS_CMD_EXIT_BLOCK, NULL, 0);
	return ret;
}

static int is_request(struct is5662 *ts, u16 cmd, const u8 *payload, u16 plen, u32 want)
{
	int i, ret = -EIO;

	for (i = 0; i < 5; i++) {
		ret = is_request_once(ts, cmd, payload, plen, want);
		if (!ret)
			return 0;
		usleep_range(10000, 10100);
	}
	dev_err(&ts->client->dev, "command 0x%02x failed: %d\n", cmd, ret);
	return ret;
}

static int is_identify(struct is5662 *ts)
{
	struct device *dev = &ts->client->dev;
	u8 *b = ts->buf;
	char id[8];
	int i, ret;
	u8 type;

	for (i = 0; i < 4; i++) {
		ret = is_read(ts, IS_ADDR_CHIPID, b, IS_LEN_CHIPID);
		if (!ret && get_unaligned_le32(b) <= IS_LEN_CHIPID && is_frame_ok(b, IS_LEN_CHIPID) &&
		    !(b[11] & 0xfd))
			break;
		usleep_range(10000, 11000);
	}
	if (i == 4) {
		dev_err(dev, "no valid chip id (firmware not in application mode?)\n");
		return -ENODEV;
	}
	memcpy(id, b + 4, 7);
	id[7] = 0;
	if (strncmp(id, "5662", 5) && strncmp(id, "5668", 5)) {
		dev_err(dev, "unexpected chip id '%s'\n", id);
		return -ENODEV;
	}

	for (i = 0; i < 5; i++) {
		ret = is_read(ts, IS_ADDR_INFO, b, IS_LEN_INFO);
		if (!ret && is_frame_ok(b, IS_LEN_INFO))
			break;
	}
	if (i < 5)
		dev_info(dev, "IS%s sensor 0x%02x project 0x%02x\n", id, b[12], b[13]);

	ret = is_read(ts, IS_ADDR_CHIPID, b, IS_LEN_CHIPID);
	if (ret)
		return ret;
	type = b[11];
	if (type == 1 || type == 0x80)
		return -ENODEV;
	ret = is_request(ts, IS_CMD_BUFFER_INFO, NULL, 0, 6);
	if (ret)
		return ret;
	ts->block_addr = get_unaligned_le32(b + 4);
	ts->block_size = get_unaligned_le16(b + 8);
	if (type != 2) {
		ret = is_request(ts, IS_CMD_DATA_INFO, NULL, 0, ts->block_size);
		if (ret)
			return ret;
		if (get_unaligned_le16(b + 4) <= 4)
			return -EINVAL;
		ts->touch_addr = get_unaligned_le32(b + 0x20);
		ts->report_size = get_unaligned_le16(b + 0x24);
	}

	if (!is_request(ts, IS_CMD_FW_INFO, NULL, 0, 200))
		dev_info(dev, "firmware %02X.%02X.%02X.%02X\n", b[0x14], b[0x15], b[0x16], b[0x17]);

	ret = is_request(ts, IS_CMD_MODULE_INFO, NULL, 0, 0x80);
	if (ret)
		return ret;
	ts->max_pens = b[8];
	ts->res_x = get_unaligned_le16(b + 0xd);
	ts->res_y = get_unaligned_le16(b + 0xf);
	ts->max_fingers = b[0x12];
	return 0;
}

static int is_read_report_desc(struct is5662 *ts)
{
	u32 size = 7, f;
	int ret;

	ret = is_request(ts, IS_CMD_REPORT_DESC, NULL, 0, 8);
	if (ret)
		return ret;
	memcpy(ts->bitmap, ts->buf + 4, sizeof(ts->bitmap));
	for (f = 0; f < 64; f++) {
		if (!(ts->bitmap[f / 8] & BIT(f % 8)))
			continue;
		switch (f) {
		case 0: case 0xf: size += 4; break;
		case 1: size += 2; break;
		case 2 ... 4: size += ts->max_fingers; break;
		case 5 ... 0xc: size += ts->max_fingers * 2; break;
		case 0xd: case 0xe: size += ts->max_fingers * 4; break;
		case 0x11: size += 0x15; break;
		case 0x12: size += 8; break;
		case 0x13: case 0x14: case 0x1c: size += ts->max_pens; break;
		case 0x15 ... 0x1b: size += ts->max_pens * 2; break;
		}
	}
	size &= 0xff;
	ts->report_size = min(ts->report_size, size);
	return 0;
}

static void is_transform(struct is5662 *ts, u16 *x, u16 *y)
{
	if (ts->finger_flag & 1)
		*x = ts->res_x - *x;
	if (ts->finger_flag & 2)
		*y = ts->res_y - *y;
	if (ts->finger_flag & 4) {
		*x = ts->res_x ? ts->screen_x * *x / ts->res_x : 0;
		*y = ts->res_y ? ts->screen_y * *y / ts->res_y : 0;
	}
	if (ts->finger_flag & 8)
		swap(*x, *y);
}

static irqreturn_t is_irq(int irq, void *data)
{
	struct is5662 *ts = data;
	u8 *b = ts->buf, *p;
	const u8 *slot = NULL, *xs = NULL, *ys = NULL, *ps = NULL;
	u8 n, pens = 0;
	u32 f, i;

	if (!ts->touch_addr || is_read(ts, ts->touch_addr, b, ts->report_size))
		return IRQ_HANDLED;
	if (!get_unaligned_le16(b) || !is_frame_ok(b, ts->report_size)) {
		dev_dbg(&ts->client->dev, "bad report\n");
		return IRQ_HANDLED;
	}
	n = b[4];
	p = b + 5;
	for (f = 0; f < 64; f++) {
		if (!(ts->bitmap[f / 8] & BIT(f % 8)))
			continue;
		switch (f) {
		case 1: pens = get_unaligned_le16(p) >> 13; p += 2; break;
		case 2: if (n) { slot = p; p += n; } break;
		case 4: if (n) p += n; break;
		case 5: if (n) { xs = p; p += n * 2; } break;
		case 6: if (n) { ys = p; p += n * 2; } break;
		case 7: if (n) { ps = p; p += n * 2; } break;
		case 0x14: if (pens) p += 1; break;
		case 0x15 ... 0x17: case 0x1a: case 0x1b: if (pens) p += pens * 2; else pens = 0; break;
		case 0x1c: case 0x1d: p += pens; break;
		}
	}
	if (n && (!xs || !ys || !ps)) {
		dev_dbg(&ts->client->dev, "touch data error\n");
		return IRQ_HANDLED;
	}
	for (i = 0; i < n; i++) {
		u16 x = get_unaligned_le16(xs + 2 * i), y = get_unaligned_le16(ys + 2 * i);
		u16 pressure = get_unaligned_le16(ps + 2 * i);

		input_event(ts->input, EV_ABS, ABS_MT_SLOT, slot ? slot[i] : i);
		if (!pressure) {
			input_event(ts->input, EV_KEY, BTN_TOUCH, 0);
			input_event(ts->input, EV_ABS, ABS_MT_PRESSURE, 0);
			input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
		} else {
			input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
			is_transform(ts, &x, &y);
			input_event(ts->input, EV_ABS, ABS_MT_POSITION_X, x);
			input_event(ts->input, EV_ABS, ABS_MT_POSITION_Y, y);
			input_event(ts->input, EV_KEY, BTN_TOUCH, 1);
			input_event(ts->input, EV_ABS, ABS_MT_PRESSURE, pressure);
		}
	}
	if (n)
		input_sync(ts->input);
	return IRQ_HANDLED;
}

static int is_power(struct is5662 *ts, bool on)
{
	int ret = 0;

	if (on) {
		if (ts->vdd)
			ret = regulator_enable(ts->vdd);
		gpiod_set_value_cansleep(ts->vdd_en, 1);
		if (!ret && ts->vcc)
			ret = regulator_enable(ts->vcc);
		gpiod_set_value_cansleep(ts->vcc_en, 1);
	} else {
		gpiod_set_value_cansleep(ts->vcc_en, 0);
		if (ts->vcc)
			regulator_disable(ts->vcc);
		usleep_range(100, 200);
		gpiod_set_value_cansleep(ts->vdd_en, 0);
		if (ts->vdd)
			regulator_disable(ts->vdd);
	}
	return ret;
}

static void is_reset(struct is5662 *ts)
{
	gpiod_set_value_cansleep(ts->rst, 1);
	if (ts->slow_reset)
		usleep_range(100, 200);
	else
		usleep_range(5000, 5100);
	gpiod_set_value_cansleep(ts->rst, 0);
	msleep(200);
}

static struct regulator *is_supply(struct device *dev, const char *name)
{
	struct regulator *r = devm_regulator_get_optional(dev, name);

	return PTR_ERR(r) == -ENODEV ? NULL : r;
}

static int is_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct is5662 *ts;
	int ret;

	if (client->irq <= 0)
		return dev_err_probe(dev, -EINVAL, "no interrupt\n");
	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;
	ts->client = client;
	ts->irq = client->irq;
	ts->buf = devm_kzalloc(dev, IS_BUF_SIZE, GFP_KERNEL);
	if (!ts->buf)
		return -ENOMEM;
	i2c_set_clientdata(client, ts);

	ts->rst = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ts->rst))
		return dev_err_probe(dev, PTR_ERR(ts->rst), "reset-gpios\n");
	ts->vdd_en = devm_gpiod_get_optional(dev, "vdd-enable", GPIOD_OUT_LOW);
	if (IS_ERR(ts->vdd_en))
		return PTR_ERR(ts->vdd_en);
	ts->vcc_en = devm_gpiod_get_optional(dev, "vcc-enable", GPIOD_OUT_LOW);
	if (IS_ERR(ts->vcc_en))
		return PTR_ERR(ts->vcc_en);
	ts->vdd = is_supply(dev, "vdd");
	if (IS_ERR(ts->vdd))
		return dev_err_probe(dev, PTR_ERR(ts->vdd), "vdd\n");
	ts->vcc = is_supply(dev, "vcc");
	if (IS_ERR(ts->vcc))
		return dev_err_probe(dev, PTR_ERR(ts->vcc), "vcc\n");
	if (ts->vdd) {
		ret = regulator_set_voltage(ts->vdd, 1800000, 1818000);
		if (ret)
			return dev_err_probe(dev, ret, "vdd 1.8 V\n");
	}
	if (ts->vcc) {
		ret = regulator_set_voltage(ts->vcc, 3300000, 3333000);
		if (ret)
			return dev_err_probe(dev, ret, "vcc 3.3 V\n");
	}

	ts->screen_x = 1264;
	ts->screen_y = 1680;
	device_property_read_u32(dev, "touchscreen-size-x", &ts->screen_x);
	device_property_read_u32(dev, "touchscreen-size-y", &ts->screen_y);
	ts->finger_flag = 0;
	ts->slow_reset = true;

	ret = is_power(ts, true);
	if (ret)
		return ret;
	usleep_range(10000, 10100);
	is_reset(ts);
	msleep(400);

	ret = is_identify(ts);
	if (!ret)
		ret = is_read_report_desc(ts);
	if (ret)
		goto err_power;
	dev_info(dev, "%u fingers, %ux%u, report %u bytes at 0x%08x\n", ts->max_fingers, ts->res_x,
		 ts->res_y, ts->report_size, ts->touch_addr);

	ts->input = devm_input_allocate_device(dev);
	if (!ts->input) {
		ret = -ENOMEM;
		goto err_power;
	}

	ts->input->name = "onyx_ts_istaric";
	ts->input->phys = "input/ts";
	ts->input->id.bustype = BUS_I2C;
	ts->input->id.vendor = 0x3236;
	ts->input->id.version = 0x0010;
	input_set_capability(ts->input, EV_KEY, BTN_TOUCH);
	input_set_abs_params(ts->input, ABS_MT_TOUCH_MAJOR, 0, 0xff, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_X, 0, ts->screen_x - 1, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_Y, 0, ts->screen_y - 1, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_PRESSURE, 0, 0x1000, 0, 0);
	ret = input_mt_init_slots(ts->input, IS_MAX_SLOTS, INPUT_MT_DIRECT);
	if (!ret)
		ret = input_register_device(ts->input);
	if (ret)
		goto err_power;

	ret = devm_request_threaded_irq(dev, ts->irq, NULL, is_irq, IRQF_ONESHOT, "istaric", ts);
	if (ret)
		goto err_power;
	return 0;

err_power:
	is_power(ts, false);
	return ret;
}

static int is_suspend(struct device *dev)
{
	struct is5662 *ts = dev_get_drvdata(dev);

	disable_irq(ts->irq);
	is_power(ts, false);
	return 0;
}

static int is_resume(struct device *dev)
{
	struct is5662 *ts = dev_get_drvdata(dev);
	int i;

	gpiod_set_value_cansleep(ts->rst, 0);
	is_power(ts, true);
	usleep_range(10000, 10100);
	gpiod_set_value_cansleep(ts->rst, 1);
	usleep_range(100, 200);
	gpiod_set_value_cansleep(ts->rst, 0);
	usleep_range(10000, 10100);
	for (i = 0; i < 100; i++) {
		if (!is_read(ts, IS_ADDR_CHIPID + 11, ts->buf, 1) && !ts->buf[0])
			break;
		usleep_range(10000, 10100);
	}
	is_read_report_desc(ts);
	enable_irq(ts->irq);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(is_pm_ops, is_suspend, is_resume);

static const struct of_device_id is_of_match[] = {
	{ .compatible = "istaric,is5662" },
	{ }
};
MODULE_DEVICE_TABLE(of, is_of_match);

static struct i2c_driver is_driver = {
	.driver = {
		.name = "istaric-is5662",
		.of_match_table = is_of_match,
		.pm = pm_sleep_ptr(&is_pm_ops),
	},
	.probe = is_probe,
};
module_i2c_driver(is_driver);

MODULE_DESCRIPTION("Istaric IS5662 touchscreen (Onyx Boox Go 7)");
MODULE_LICENSE("GPL");
