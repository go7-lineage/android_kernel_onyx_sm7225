// SPDX-License-Identifier: GPL-2.0-only

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>

#define HALL_DEBOUNCE_MS 500

struct onyx_hall {
	struct device *dev;
	struct input_dev *input;
	struct gpio_desc *gpio;
	struct delayed_work work;
	struct wakeup_source *ws;
	int irq;
	int state, last; /* 0 = closed, 1 = open */
	bool alt_key, suspended;
};

static void hall_key(struct onyx_hall *h, unsigned int code)
{
	if (h->alt_key) {
		input_report_key(h->input, KEY_LEFTALT, 1);
		input_sync(h->input);
	}
	input_report_key(h->input, code, 1);
	input_sync(h->input);
	input_report_key(h->input, code, 0);
	input_sync(h->input);
	if (h->alt_key) {
		msleep(20);
		input_report_key(h->input, KEY_LEFTALT, 0);
		input_sync(h->input);
	}
}

static void hall_work(struct work_struct *work)
{
	struct onyx_hall *h = container_of(to_delayed_work(work), struct onyx_hall, work);

	if (h->state == h->last)
		return;
	h->last = h->state;
	dev_dbg(h->dev, "cover %s\n", h->state ? "open" : "closed");
	if (h->suspended && h->state == 0)
		return;
	hall_key(h, h->state ? KEY_WAKEUP : KEY_POWER);
}

static irqreturn_t hall_irq(int irq, void *data)
{
	struct onyx_hall *h = data;

	__pm_wakeup_event(h->ws, 2000);
	h->state = gpiod_get_value_cansleep(h->gpio);
	mod_delayed_work(system_wq, &h->work, msecs_to_jiffies(HALL_DEBOUNCE_MS));
	return IRQ_HANDLED;
}

static ssize_t state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct onyx_hall *h = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", h->last);
}
static DEVICE_ATTR_RO(state);

static struct attribute *hall_attrs[] = {
	&dev_attr_state.attr,
	NULL
};
ATTRIBUTE_GROUPS(hall);

static void hall_cancel(void *d)
{
	cancel_delayed_work_sync(d);
}

static void hall_ws_unregister(void *d)
{
	wakeup_source_unregister(d);
}

static int hall_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct onyx_hall *h;
	int ret;

	h = devm_kzalloc(dev, sizeof(*h), GFP_KERNEL);
	if (!h)
		return -ENOMEM;
	h->dev = dev;
	platform_set_drvdata(pdev, h);
	h->alt_key = device_property_read_bool(dev, "onyx,send-alt-key");

	ret = devm_regulator_get_enable_optional(dev, "vdd");
	if (ret && ret != -ENODEV)
		return dev_err_probe(dev, ret, "vdd\n");

	h->gpio = devm_gpiod_get(dev, "irq", GPIOD_IN);
	if (IS_ERR(h->gpio))
		return dev_err_probe(dev, PTR_ERR(h->gpio), "irq-gpios\n");
	h->irq = gpiod_to_irq(h->gpio);
	if (h->irq < 0)
		return h->irq;

	h->input = devm_input_allocate_device(dev);
	if (!h->input)
		return -ENOMEM;
	h->input->name = "onyx-hall";
	h->input->phys = "m1kbd/input2";
	input_set_capability(h->input, EV_KEY, KEY_POWER);
	input_set_capability(h->input, EV_KEY, KEY_WAKEUP);
	if (h->alt_key)
		input_set_capability(h->input, EV_KEY, KEY_LEFTALT);
	ret = input_register_device(h->input);
	if (ret)
		return ret;

	h->ws = wakeup_source_register(dev, "hall_wakelock");
	if (!h->ws)
		return -ENOMEM;
	ret = devm_add_action_or_reset(dev, hall_ws_unregister, h->ws);
	if (ret)
		return ret;
	INIT_DELAYED_WORK(&h->work, hall_work);
	ret = devm_add_action_or_reset(dev, hall_cancel, &h->work);
	if (ret)
		return ret;

	h->state = h->last = gpiod_get_value_cansleep(h->gpio);
	ret = devm_request_threaded_irq(dev, h->irq, NULL, hall_irq,
		IRQF_ONESHOT | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "onyx_hall", h);
	if (ret)
		return ret;
	return device_init_wakeup(dev, true);
}

static int hall_suspend(struct device *dev)
{
	struct onyx_hall *h = dev_get_drvdata(dev);

	h->suspended = true;
	if (device_may_wakeup(dev))
		enable_irq_wake(h->irq);
	return 0;
}

static int hall_resume(struct device *dev)
{
	struct onyx_hall *h = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(h->irq);
	h->suspended = false;
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(hall_pm_ops, hall_suspend, hall_resume);

static const struct of_device_id hall_of_match[] = {
	{ .compatible = "onyx,go7-hall" },
	{ }
};
MODULE_DEVICE_TABLE(of, hall_of_match);

static struct platform_driver hall_driver = {
	.probe = hall_probe,
	.driver = {
		.name = "onyx-go7-hall",
		.of_match_table = hall_of_match,
		.pm = pm_sleep_ptr(&hall_pm_ops),
		.dev_groups = hall_groups,
	},
};
module_platform_driver(hall_driver);

MODULE_DESCRIPTION("Onyx Boox Go 7 cover sensor");
MODULE_LICENSE("GPL");
