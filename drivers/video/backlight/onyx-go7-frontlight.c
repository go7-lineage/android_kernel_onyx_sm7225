// SPDX-License-Identifier: GPL-2.0-only
/*
 * Frontlight of the Onyx Boox Go 7 (LM3630A)
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include "onyx-go7-frontlight-tables.h"

#define REG_CTRL 0x00
#define REG_CONFIG 0x01
#define REG_BOOST 0x02
#define REG_BRT_A 0x03
#define REG_BRT_B 0x04
#define REG_I_A 0x05
#define REG_I_B 0x06
#define REG_ONOFF_RAMP 0x07
#define REG_RUN_RAMP 0x08

#define CTRL_SLEEP BIT(7)
#define CTRL_BANKS (BIT(2) | BIT(1))

#define GO7_FL_MAX (GO7_FL_LEVELS - 1)

struct go7_fl {
	struct regmap *map;
	struct gpio_desc *enable;
	struct regulator_bulk_data supplies[2];
	u8 warmth;
	bool on;
};

static void go7_fl_power_off(struct go7_fl *fl)
{
	regmap_update_bits(fl->map, REG_CTRL, CTRL_BANKS, 0);
	usleep_range(10000, 12000);
	gpiod_set_value_cansleep(fl->enable, 0);
	regulator_bulk_disable(ARRAY_SIZE(fl->supplies), fl->supplies);
	fl->on = false;
}

static int go7_fl_power_on(struct go7_fl *fl)
{
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(fl->supplies), fl->supplies);
	if (ret)
		return ret;
	usleep_range(10000, 12000);
	gpiod_set_value_cansleep(fl->enable, 1);
	usleep_range(3000, 4000);
	fl->on = true;

	/* no PWM input; boost with slow start, 40 V and 1.2 A limits */
	ret = regmap_update_bits(fl->map, REG_CONFIG, 0x07, 0);
	if (!ret)
		ret = regmap_write(fl->map, REG_BOOST, 0x7c);
	if (ret)
		go7_fl_power_off(fl);
	return ret;
}

static int go7_fl_update_status(struct backlight_device *bl)
{
	struct go7_fl *fl = bl_get_data(bl);
	unsigned int br = backlight_get_brightness(bl), w = fl->warmth, i;
	/* the lowest levels switch without a ramp */
	unsigned int ramp = br > 3 ? 0x09 : 0;
	const struct {
		u8 reg, mask, val;
	} seq[] = {
		{ REG_ONOFF_RAMP, 0x3f, ramp },
		{ REG_RUN_RAMP, 0x3f, ramp },
		{ REG_I_A, 0x1f, go7_fl_current[w][br] },
		{ REG_I_B, 0x1f, go7_fl_current[GO7_FL_MAX - w][br] },
		{ REG_BRT_A, 0xff, go7_fl_code[w][br] },
		{ REG_BRT_B, 0xff, go7_fl_code[GO7_FL_MAX - w][br] },
		{ REG_CTRL, CTRL_BANKS, CTRL_BANKS },
	};
	int ret;

	if (!br) {
		if (fl->on)
			go7_fl_power_off(fl);
		return 0;
	}
	if (!fl->on) {
		ret = go7_fl_power_on(fl);
		if (ret)
			return ret;
	}
	ret = regmap_update_bits(fl->map, REG_CTRL, CTRL_SLEEP, 0);
	if (ret)
		return ret;
	usleep_range(1000, 2000);
	for (i = 0; i < ARRAY_SIZE(seq); i++) {
		ret = regmap_update_bits(fl->map, seq[i].reg, seq[i].mask,
					 seq[i].val);
		if (ret)
			return ret;
	}
	return 0;
}

static const struct backlight_ops go7_fl_ops = {
	.update_status = go7_fl_update_status,
};

static ssize_t warmth_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct go7_fl *fl = bl_get_data(to_backlight_device(dev));

	return sysfs_emit(buf, "%u\n", fl->warmth);
}

static ssize_t warmth_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t n)
{
	struct backlight_device *bl = to_backlight_device(dev);
	struct go7_fl *fl = bl_get_data(bl);
	u8 warmth;
	int ret;

	ret = kstrtou8(buf, 10, &warmth);
	if (ret)
		return ret;
	if (warmth > GO7_FL_MAX)
		return -EINVAL;
	fl->warmth = warmth;
	ret = backlight_update_status(bl);
	return ret ?: n;
}
static DEVICE_ATTR_RW(warmth);

static const struct regmap_config go7_fl_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x50,
};

static int go7_fl_probe(struct i2c_client *client)
{
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		/* the levels are steps of perceived brightness */
		.scale = BACKLIGHT_SCALE_NON_LINEAR,
		.max_brightness = GO7_FL_MAX,
	};
	struct device *dev = &client->dev;
	struct backlight_device *bl;
	struct go7_fl *fl;
	int ret;

	fl = devm_kzalloc(dev, sizeof(*fl), GFP_KERNEL);
	if (!fl)
		return -ENOMEM;
	i2c_set_clientdata(client, fl);

	fl->map = devm_regmap_init_i2c(client, &go7_fl_regmap);
	if (IS_ERR(fl->map))
		return PTR_ERR(fl->map);
	fl->enable = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(fl->enable))
		return dev_err_probe(dev, PTR_ERR(fl->enable), "enable GPIO\n");
	fl->supplies[0].supply = "vdd";
	fl->supplies[1].supply = "vddio";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(fl->supplies),
				      fl->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "supplies\n");

	bl = devm_backlight_device_register(dev, "frontlight", dev, fl,
					    &go7_fl_ops, &props);
	if (IS_ERR(bl))
		return PTR_ERR(bl);
	return device_create_file(&bl->dev, &dev_attr_warmth);
}

static void go7_fl_off(struct i2c_client *client)
{
	struct go7_fl *fl = i2c_get_clientdata(client);

	if (fl->on)
		go7_fl_power_off(fl);
}

static const struct of_device_id go7_fl_of_match[] = {
	{ .compatible = "onyx,go7-frontlight" },
	{}
};
MODULE_DEVICE_TABLE(of, go7_fl_of_match);

static struct i2c_driver go7_fl_driver = {
	.driver = {
		.name = "onyx-go7-frontlight",
		.of_match_table = go7_fl_of_match,
	},
	.probe = go7_fl_probe,
	.remove = go7_fl_off,
	.shutdown = go7_fl_off,
};
module_i2c_driver(go7_fl_driver);

MODULE_DESCRIPTION("Onyx Boox Go 7 frontlight");
MODULE_LICENSE("GPL");
