/*
 *
 * Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define	LID_DEV_NAME	"hall_sensor"
#define HALL_INPUT	"/dev/input/hall_dev"

static int old_INT_state;

struct hall_data {
	int gpio;	/* device use gpio number */
	int irq;	/* device request irq number */
	int active_low;	/* gpio active high or low for valid value */
	bool wakeup;	/* device can wakeup system or not */
	struct input_dev *hall_dev;
	struct regulator *vddio;
	u32 min_uv;	/* device allow minimum voltage */
	u32 max_uv;	/* device allow max voltage */
	struct switch_dev sdev;
	int state;
	struct delayed_work switch_work;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

struct hall_data *tdata;

static void swtich_work_fuc(struct work_struct *work)
{
	int value;

	/*printk(KERN_ERR "hall swtich_work_fuc\n");*/

	value = (gpio_get_value_cansleep(tdata->gpio) ? 0 : 1);

	pr_err("hall gpio = %d, state=%d, old state=%d\n", gpio_get_value_cansleep(tdata->gpio), value, old_INT_state);

	tdata->state = value;

	if (tdata->state != old_INT_state) {
		old_INT_state = tdata->state;
		pr_err("hall-trigger state=%d\n", tdata->state);
		switch_set_state(&tdata->sdev, tdata->state);
	}

}

static irqreturn_t hall_interrupt_handler(int irq, void *dev)
{
	struct hall_data *data = dev;

	schedule_delayed_work(&data->switch_work, 0);

	return IRQ_HANDLED;
}

static int hall_parse_dt(struct device *dev, struct hall_data *data)
{
	unsigned int tmp;
	/*
	u32 tempval;
	int rc;
	*/
	struct device_node *np = dev->of_node;

	data->gpio = of_get_named_gpio_flags(dev->of_node,
			"linux,gpio-int", 0, &tmp);
	if (!gpio_is_valid(data->gpio)) {
		dev_err(dev, "hall gpio is not valid\n");
		return -EINVAL;
	}
	data->active_low = tmp & OF_GPIO_ACTIVE_LOW ? 0 : 1;

	data->wakeup = of_property_read_bool(np, "linux,wakeup");
/*
	rc = of_property_read_u32(np, "linux,max-uv", &tempval);
	if (rc) {
		dev_err(dev, "unable to read max-uv\n");
		return -EINVAL;
	}
	data->max_uv = tempval;

	rc = of_property_read_u32(np, "linux,min-uv", &tempval);
	if (rc) {
		dev_err(dev, "unable to read min-uv\n");
		return -EINVAL;
	}
	data->min_uv = tempval;
*/
	dev_err(dev, "hall gpio = %d, active_low = %d, wakeup = %d\n", data->gpio, data->active_low, data->wakeup);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void hall_early_resume(struct early_suspend *h)
{
	dev_err(dev, "hall_driver_resume\n");
	tdata->state = (gpio_get_value_cansleep(tdata->gpio) ? 0 : 1);
	dev_err(dev, "hall state=%d, old state=%d\n", tdata->state, old_INT_state);

	if (tdata->state != old_INT_state) {
		old_INT_state = tdata->state;
		dev_err(dev, "hall-trigger state=%d\n", tdata->state);
		switch_set_state(&tdata->sdev, tdata->state);
	}
}

static void hall_early_suspend(struct early_suspend *h)
{
	dev_err(dev, "hall_early_suspend\n");
}
#endif

static int hall_driver_probe(struct platform_device *dev)
{
	struct hall_data *data;
	int err = 0;
	int irq_flags;

	dev_err(&dev->dev, "hall_driver probe\n");
	data = devm_kzalloc(&dev->dev, sizeof(struct hall_data), GFP_KERNEL);
	if (data == NULL) {
		err = -ENOMEM;
		dev_err(&dev->dev,
				"failed to allocate memory %d\n", err);
		goto exit;
	}
	dev_set_drvdata(&dev->dev, data);
	if (dev->dev.of_node) {
		err = hall_parse_dt(&dev->dev, data);
		if (err < 0) {
			dev_err(&dev->dev, "Failed to parse device tree\n");
			goto exit;
		}
	} else if (dev->dev.platform_data != NULL) {
		memcpy(data, dev->dev.platform_data, sizeof(*data));
	} else {
		dev_err(&dev->dev, "No valid platform data.\n");
		err = -ENODEV;
		goto exit;
	}

	if (!gpio_is_valid(data->gpio)) {
		dev_err(&dev->dev, "gpio is not valid\n");
		err = -EINVAL;
		goto exit;
	}

	irq_flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
		| IRQF_ONESHOT;
	err = gpio_request_one(data->gpio, GPIOF_DIR_IN, "hall_sensor_irq");
	if (err) {
		dev_err(&dev->dev, "unable to request gpio %d\n", data->gpio);
		goto exit;
	}

	data->sdev.name = "water_det";
	err = switch_dev_register(&data->sdev);
	if (err < 0)
		goto no_memory;

	data->state = (gpio_get_value_cansleep(data->gpio) ? 0 : 1);
	old_INT_state = data->state;
	switch_set_state(&data->sdev, data->state);
	dev_err(&dev->dev, "hall old_INT_state = %d\n", old_INT_state);

	data->irq = gpio_to_irq(data->gpio);
	err = devm_request_threaded_irq(&dev->dev, data->irq, NULL,
			hall_interrupt_handler,
			irq_flags, "hall_sensor", data);
	if (err < 0) {
		dev_err(&dev->dev, "request irq failed : %d\n", data->irq);
		goto free_gpio;
	}

	device_init_wakeup(&dev->dev, data->wakeup);
	enable_irq_wake(data->irq);

	/*err = hall_config_regulator(dev, true);
	if (err < 0) {
		dev_err(&dev->dev, "Configure power failed: %d\n", err);
		goto free_irq;
	}

	err = hall_set_regulator(dev, true);
	if (err < 0) {
		dev_err(&dev->dev, "power on failed: %d\n", err);
		goto err_regulator_init;
	}*/
	INIT_DELAYED_WORK(&data->switch_work, swtich_work_fuc);

#ifdef CONFIG_HAS_EARLYSUSPEND
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->early_suspend.suspend = hall_early_suspend;
	data->early_suspend.resume = hall_early_resume;
	register_early_suspend(&data->early_suspend);
#endif

	tdata = data;

	return 0;
/*
err_regulator_init:
	hall_config_regulator(dev, false);
free_irq:
	disable_irq_wake(data->irq);
	device_init_wakeup(&dev->dev, 0);
*/
no_memory:
	dev_err(&dev->dev, "exit with %d\n", err);
free_gpio:
	gpio_free(data->gpio);
exit:
	return err;
}

static int hall_driver_remove(struct platform_device *dev)
{
	struct hall_data *data = dev_get_drvdata(&dev->dev);

	disable_irq_wake(data->irq);
	device_init_wakeup(&dev->dev, 0);
	if (data->gpio)
		gpio_free(data->gpio);
	/*
	hall_set_regulator(dev, false);
	hall_config_regulator(dev, false);
	*/
	return 0;
}

static struct platform_device_id hall_id[] = {
	{LID_DEV_NAME, 0 },
	{ },
};


static struct of_device_id hall_match_table[] = {
	{.compatible = "qcom, hall", },
	{ },
};

static struct platform_driver hall_driver = {
	.driver = {
		.name = LID_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(hall_match_table),
	},
	.probe = hall_driver_probe,
	.remove = hall_driver_remove,
	.id_table = hall_id,
};

static int __init hall_init(void)
{
	return platform_driver_register(&hall_driver);
}

static void __exit hall_exit(void)
{
	platform_driver_unregister(&hall_driver);
}

module_init(hall_init);
module_exit(hall_exit);
MODULE_DESCRIPTION("Hall sensor driver");
MODULE_LICENSE("GPL v2");
