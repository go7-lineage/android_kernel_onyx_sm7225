// SPDX-License-Identifier: GPL-2.0-only
/*
 * Onyx Boox Go 7 e-ink TCON
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>

#define MXO_CMD_IDCODE		0xe0
#define MXO_CMD_USERCODE	0xc0
#define MXO1300_IDCODE		0x43202b41

enum { VSYS, VDD1V8, VDD3V, VDD1V2, NUM_SUPPLIES };
static const char * const supply_names[NUM_SUPPLIES] = { "vsys", "vdd1v8", "vdd3v", "vdd1v2" };
static const int supply_uv[NUM_SUPPLIES] = { 0, 1800000, 3000000, 1150000 };

struct mxo {
	struct i2c_client *client;
	struct regulator_bulk_data supplies[NUM_SUPPLIES];
	struct gpio_desc *reset, *done;
	u32 idcode, usercode;
};

static int mxo_read_cmd(struct mxo *mxo, u8 cmd, u32 *val)
{
	u8 wbuf[4] = { cmd, 0, 0, 0 };
	__le32 rbuf;
	struct i2c_msg msgs[] = {
		{ .addr = mxo->client->addr, .len = sizeof(wbuf), .buf = wbuf },
		{ .addr = mxo->client->addr, .flags = I2C_M_RD, .len = sizeof(rbuf), .buf = (u8 *)&rbuf },
	};
	int ret = i2c_transfer(mxo->client->adapter, msgs, ARRAY_SIZE(msgs));

	if (ret < 0)
		return ret;
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;
	*val = le32_to_cpu(rbuf);
	return 0;
}

static int mxo_power_on(struct mxo *mxo)
{
	int ret;

	ret = regulator_bulk_enable(VDD3V, mxo->supplies);
	if (ret)
		return ret;
	gpiod_set_value_cansleep(mxo->reset, 1);
	ret = regulator_enable(mxo->supplies[VDD3V].consumer);
	if (ret)
		goto err_low;
	usleep_range(2000, 2500);
	ret = regulator_enable(mxo->supplies[VDD1V2].consumer);
	if (ret)
		goto err_3v;
	usleep_range(1000, 1500);
	if (mxo->done && gpiod_get_value_cansleep(mxo->done) == 1) {
		regulator_disable(mxo->supplies[VDD1V2].consumer);
		msleep(20);
		ret = regulator_enable(mxo->supplies[VDD1V2].consumer);
		if (ret)
			goto err_3v;
		usleep_range(2000, 2500);
	}
	usleep_range(3000, 3500);
	gpiod_set_value_cansleep(mxo->reset, 0);
	usleep_range(3000, 3500);
	return 0;

err_3v:
	regulator_disable(mxo->supplies[VDD3V].consumer);
err_low:
	gpiod_set_value_cansleep(mxo->reset, 1);
	regulator_bulk_disable(VDD3V, mxo->supplies);
	return ret;
}

static void mxo_power_off(void *data)
{
	struct mxo *mxo = data;

	gpiod_set_value_cansleep(mxo->reset, 1);
	usleep_range(1000, 1500);
	regulator_disable(mxo->supplies[VDD3V].consumer);
	usleep_range(2000, 2500);
	regulator_disable(mxo->supplies[VDD1V2].consumer);
	usleep_range(1000, 1500);
	regulator_bulk_disable(VDD3V, mxo->supplies);
}

static ssize_t chipid_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxo *mxo = dev_get_drvdata(dev);

	return sysfs_emit(buf, "0x%08x usercode 0x%08x\n", mxo->idcode, mxo->usercode);
}
static DEVICE_ATTR_RO(chipid);

static struct attribute *mxo_attrs[] = {
	&dev_attr_chipid.attr,
	NULL
};
ATTRIBUTE_GROUPS(mxo);

static int mxo_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct mxo *mxo;
	int i, ret;

	mxo = devm_kzalloc(dev, sizeof(*mxo), GFP_KERNEL);
	if (!mxo)
		return -ENOMEM;
	mxo->client = client;
	i2c_set_clientdata(client, mxo);

	for (i = 0; i < NUM_SUPPLIES; i++)
		mxo->supplies[i].supply = supply_names[i];
	ret = devm_regulator_bulk_get(dev, NUM_SUPPLIES, mxo->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "supplies\n");
	for (i = 0; i < NUM_SUPPLIES; i++) {
		if (!supply_uv[i])
			continue;
		ret = regulator_set_voltage(mxo->supplies[i].consumer, supply_uv[i],
					    supply_uv[i] + supply_uv[i] / 100);
		if (ret)
			dev_warn(dev, "%s: can't set %d uV: %d\n", supply_names[i], supply_uv[i], ret);
	}

	mxo->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(mxo->reset))
		return dev_err_probe(dev, PTR_ERR(mxo->reset), "reset-gpios\n");
	mxo->done = devm_gpiod_get_optional(dev, "done", GPIOD_IN);
	if (IS_ERR(mxo->done))
		return dev_err_probe(dev, PTR_ERR(mxo->done), "done-gpios\n");

	ret = mxo_power_on(mxo);
	if (ret)
		return dev_err_probe(dev, ret, "power on\n");
	ret = devm_add_action_or_reset(dev, mxo_power_off, mxo);
	if (ret)
		return ret;

	msleep(10);
	ret = mxo_read_cmd(mxo, MXO_CMD_IDCODE, &mxo->idcode);
	if (!ret)
		ret = mxo_read_cmd(mxo, MXO_CMD_USERCODE, &mxo->usercode);
	if (ret)
		dev_warn(dev, "no answer on the configuration interface: %d\n", ret);
	else
		dev_info(dev, "%s (id 0x%08x) usercode 0x%08x\n",
			 mxo->idcode == MXO1300_IDCODE ? "mxo1300" : "unknown FPGA", mxo->idcode,
			 mxo->usercode);
	return 0;
}

static int mxo_suspend(struct device *dev)
{
	struct mxo *mxo = dev_get_drvdata(dev);

	gpiod_set_value_cansleep(mxo->reset, 1);
	return 0;
}

static int mxo_resume(struct device *dev)
{
	struct mxo *mxo = dev_get_drvdata(dev);

	gpiod_set_value_cansleep(mxo->reset, 0);
	usleep_range(3000, 3500);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(mxo_pm_ops, mxo_suspend, mxo_resume);

static const struct of_device_id mxo_of_match[] = {
	{ .compatible = "onyx,go7-mxo1300" },
	{ }
};
MODULE_DEVICE_TABLE(of, mxo_of_match);

static struct i2c_driver mxo_driver = {
	.driver = {
		.name = "onyx-go7-tcon",
		.of_match_table = mxo_of_match,
		.pm = pm_sleep_ptr(&mxo_pm_ops),
		.dev_groups = mxo_groups,
	},
	.probe = mxo_probe,
};
module_i2c_driver(mxo_driver);

MODULE_DESCRIPTION("Onyx Boox Go 7 e-ink TCON power and reset");
MODULE_LICENSE("GPL");
