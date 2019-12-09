
/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/list.h>
#include <linux/pinctrl/consumer.h>
#include <linux/spinlock.h>
#include <linux/delay.h>

/* #define CONFIG_KTD_FLASH_DEBUG */
#undef CDBG
#undef CONFIG_KTD_ALLGPIO
#define CONFIG_KTD_FLASH_DEBUG
#ifdef CONFIG_KTD_FLASH_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

#define LED_KTD_FLASH_DRIVER_NAME	"qcom,leds-ktd-flash"
#define LED_TRIGGER_DEFAULT		"none"
#define LED_KTD_CDEV_NUM 2

#define LED_KTD_FLASH_CURR_MIN 94 /* 1500/16 */
#define LED_KTD_FLASH_CURR_MIN_HALF 47
#define LED_KTD_TORCH_CURR_MIN 31 /* 500/16 */
#define LED_KTD_TORCH_CURR_MIN_HALF 15
#define LED_KTD_FLASH_DURA_MIN 262 /* msec */
#define LED_KTD_FLASH_DURA_MIN_HALF 131
#define LED_KTD_FLASH_CURR_SET_ADDR 0x80
#define LED_KTD_TORCH_CURR_SET_ADDR 0x60
#define LED_KTD_FLASH_DURA_SET_ADDR 0x20
#define LED_KTD_FLASH_MIN_CURRENT_ADDR 0x40

#define GPIO_OUT_LOW          (0 << 1)
#define GPIO_OUT_HIGH         (1 << 1)

#define SEND_LOW(gpio_ctrl) do {\
	gpio_set_value(gpio_ctrl, GPIO_OUT_LOW); \
		udelay(80);\
	gpio_set_value(gpio_ctrl, GPIO_OUT_HIGH); \
		udelay(40);\
} while (0)

#define SEND_HIGH(gpio_ctrl) do {\
	gpio_set_value(gpio_ctrl, GPIO_OUT_LOW); \
		udelay(40);\
	gpio_set_value(gpio_ctrl, GPIO_OUT_HIGH); \
		udelay(80);\
} while (0)

enum flash_led_type {
	FLASH = 0,
	TORCH,
	NONE,
};

struct ktd_flash_led_cdev {
	struct led_classdev cdev;
	u16	duration;
	u16	max_current;
	u16 prgm_current;
	struct ktd_flash_led_data *parent;
	u8 type;
};

struct ktd_flash_led_data {
	struct gpio *flash_ctrl;
#ifdef CONFIG_KTD_ALLGPIO
	struct gpio *flash_strobe;
	struct gpio *flash_tx;
#endif
	struct platform_device *pdev;
	struct gpio ctrl_gpio[3];
	struct ktd_flash_led_cdev flash_cdev;
	struct ktd_flash_led_cdev torch_cdev;
	u8 suspend_state;
};

/*
 * Flash LED data structure containing flash LED attributes
 */
struct ktd_flash_led {
	struct platform_device *pdev;
	struct ktd_flash_led_data *flash_node;
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_default;
	struct pinctrl_state *gpio_state_suspend;
	int				num_leds;
};

unsigned long flash_duration = 0;
unsigned long min_current = 0;
unsigned int curr_manual_mode = 0;
static int turn_on;
static int is_torch_used_from_camera;

static DEFINE_SPINLOCK(ktd_lock);

static struct of_device_id led_ktd_flash_of_match[] = {
	{.compatible = LED_KTD_FLASH_DRIVER_NAME,},
	{},
};

static int led_ktd_flash_senddata(struct ktd_flash_led_data *flash_node, const unsigned char data)
{
	unsigned long int_flags;
	unsigned ctrl_gpio = flash_node->flash_ctrl->gpio;

	if (flash_node->suspend_state == 1) {
		pr_err("flash node suspended!!\n");
		return 0;
	}
	CDBG(" led_ktd_flash_senddata start: gpio=%d, data=%2X\n", ctrl_gpio, data);

	spin_lock_irqsave(&ktd_lock, int_flags);

	gpio_set_value(ctrl_gpio, GPIO_OUT_HIGH);
	udelay(40); /* TDS: data start. typical 10 us */
	/* fill the data. */
	if (data&0x80)
		SEND_HIGH(ctrl_gpio);
	else
		SEND_LOW(ctrl_gpio);
	if (data&0x40)
		SEND_HIGH(ctrl_gpio);
	else
		SEND_LOW(ctrl_gpio);
	if (data&0x20)
		SEND_HIGH(ctrl_gpio);
	else
		SEND_LOW(ctrl_gpio);
	if (data&0x10)
		SEND_HIGH(ctrl_gpio);
	else
		SEND_LOW(ctrl_gpio);
	if (data&0x08)
		SEND_HIGH(ctrl_gpio);
	else
		SEND_LOW(ctrl_gpio);
	if (data&0x04)
		SEND_HIGH(ctrl_gpio);
	else
		SEND_LOW(ctrl_gpio);
	if (data&0x02)
		SEND_HIGH(ctrl_gpio);
	else
		SEND_LOW(ctrl_gpio);
	if (data&0x01)
		SEND_HIGH(ctrl_gpio);
	else
		SEND_LOW(ctrl_gpio);
	/* fill EOD */
	gpio_set_value(ctrl_gpio, GPIO_OUT_LOW);
	udelay(20); /* typical val: 2 us */
	gpio_set_value(ctrl_gpio, GPIO_OUT_HIGH);
	udelay(400); /* typical val: 350 us */
	spin_unlock_irqrestore(&ktd_lock, int_flags);

	/*CDBG("led_ktd_flash_senddata end");*/

	return 0;
}

static void led_ktd_flash_set_flash_current(struct ktd_flash_led_data *flash_node, int curr)
{
	unsigned char flash_curr_level;
	unsigned char data;

	flash_curr_level = (unsigned char)(curr / LED_KTD_FLASH_CURR_MIN);
	if (flash_curr_level != 0 && curr % LED_KTD_FLASH_CURR_MIN < LED_KTD_FLASH_CURR_MIN_HALF)
		flash_curr_level--;
	if (flash_curr_level > 0x0F)
		flash_curr_level = 0x0F;

	data = LED_KTD_FLASH_CURR_SET_ADDR | (flash_curr_level & 0x0F);
	CDBG("led_ktd_flash_set_flash_current: curr = %d, level = %d, data = %x", curr, flash_curr_level, data);

	led_ktd_flash_senddata(flash_node, (const unsigned char)data);
}

static void led_ktd_flash_set_torch_current(struct ktd_flash_led_data *flash_node, int curr)
{
	unsigned char torch_curr_level;
	unsigned char data;

	torch_curr_level = (unsigned char)(curr / LED_KTD_TORCH_CURR_MIN);
	if (torch_curr_level != 0 && curr % LED_KTD_TORCH_CURR_MIN < LED_KTD_TORCH_CURR_MIN_HALF)
		torch_curr_level--;
	if (torch_curr_level > 0x0F)
		torch_curr_level = 0x0F;

	data = LED_KTD_TORCH_CURR_SET_ADDR | (torch_curr_level & 0x0F);
	CDBG("led_ktd_flash_set_torch_current: curr = %d, level = %d, data = %x\n", curr, torch_curr_level, data);

	led_ktd_flash_senddata(flash_node, (const unsigned char)data);
}

static void led_ktd_flash_set_flash_duration(struct ktd_flash_led_data *flash_node, int time)
{
	unsigned char flash_dura_level;
	unsigned char data;

	if (time > 0) {
		flash_dura_level = (unsigned char)(time / LED_KTD_FLASH_DURA_MIN);
		if (time % LED_KTD_FLASH_DURA_MIN > LED_KTD_FLASH_DURA_MIN_HALF)
			flash_dura_level++;
		if (flash_dura_level == 0)
			flash_dura_level++;
		if (flash_dura_level > 0x07)
			flash_dura_level = 0x07;
	} else {
		flash_dura_level = 0x0;
	}

	data = LED_KTD_FLASH_DURA_SET_ADDR | (flash_dura_level & 0x0F);
	CDBG("led_ktd_flash_set_flash_duration: time = %d, level = %d, data = %x", time, flash_dura_level, data);

	led_ktd_flash_senddata(flash_node, (const unsigned char)data);
}

static void led_ktd_flash_set_min_current(struct ktd_flash_led_data *flash_node, int value)
{
	unsigned char min_current_level = 0;
	unsigned char data;

	if (value > 0) {
		if (value > 0x07)
			min_current_level = 0x07;
		else
			min_current_level = value;
	} else {
		min_current_level = 0x0;
	}

	data = LED_KTD_FLASH_MIN_CURRENT_ADDR | (min_current_level & 0x0F);
	CDBG("led_ktd_flash_set_min_current: value = %d, min_current_level = %d, data = %x"
	     , value, min_current_level, data);

	led_ktd_flash_senddata(flash_node, (const unsigned char)data);
}

static enum led_brightness led_ktd_brightness_get(struct led_classdev *led_cdev)
{
	CDBG("led_ktd_brightness_get: brightness:%d\n", led_cdev->brightness);
	return led_cdev->brightness;
}

static void led_ktd_brightness_set(struct led_classdev *led_cdev,
						enum led_brightness value)
{
	struct ktd_flash_led_cdev *flash_led_cdev;

	CDBG("led_ktd_brightness_set: brightness value:%d\n", value);
	flash_led_cdev = container_of(led_cdev, struct ktd_flash_led_cdev, cdev);
	if (value < LED_OFF) {
		pr_err("Invalid brightness value\n");
		return;
	}

	if (flash_led_cdev->parent->suspend_state == 1) {
		led_cdev->brightness = LED_OFF;
		pr_err("flash node suspended!!\n");
		return;
	}

	if (value > flash_led_cdev->cdev.max_brightness)
		value = flash_led_cdev->cdev.max_brightness;

	led_cdev->brightness = value;
#ifdef CONFIG_KTD_FLASH_DEBUG
	if (curr_manual_mode == 1 && value != LED_OFF)
		value = flash_led_cdev->prgm_current;
#endif

	if (value == LED_OFF) {
		/* set led disable */
		led_ktd_flash_senddata(flash_led_cdev->parent, 0xA0);
		/* gpio_set_value(flash_led_cdev->parent->flash_strobe->gpio,
		 * GPIO_OUT_LOW); */
		/* gpio_set_value(flash_led_cdev->parent->flash_tx->gpio,
		 * GPIO_OUT_LOW); */
		if (flash_led_cdev->type == TORCH) {
			is_torch_used_from_camera = 0;
			turn_on = 0;
		}
		return;
	}

	if (flash_led_cdev->type == FLASH) {
		/* set flash duration */
		led_ktd_flash_set_flash_duration(flash_led_cdev->parent, flash_led_cdev->parent->flash_cdev.duration);
		/* set flash op current */
		led_ktd_flash_set_flash_current(flash_led_cdev->parent, value);
		/* set flash enable */
		led_ktd_flash_senddata(flash_led_cdev->parent, 0xA2);
		/* gpio_set_value(flash_led_cdev->parent->flash_strobe->gpio,
		 * GPIO_OUT_HIGH); */
	} else {
		/* turn off timer function */
		led_ktd_flash_set_flash_duration(flash_led_cdev->parent, 0);
		/* set torch op current */
		led_ktd_flash_set_torch_current(flash_led_cdev->parent, value);
		/* set torch enable */
		led_ktd_flash_senddata(flash_led_cdev->parent, 0xA1);
		/* gpio_set_value(flash_led_cdev->parent->flash_strobe->gpio,
		 * GPIO_OUT_HIGH); */
		/* gpio_set_value(flash_led_cdev->parent->flash_tx->gpio,
		 * GPIO_OUT_HIGH); */
		is_torch_used_from_camera = 1;
	}
}

static ssize_t flash_current_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct ktd_flash_led_cdev *flash_led_cdev = container_of(led_cdev, struct ktd_flash_led_cdev, cdev);

	return snprintf(buf, PAGE_SIZE, "%lu\n", (unsigned long)flash_led_cdev->prgm_current);
}

static ssize_t flash_current_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct ktd_flash_led_cdev *flash_led_cdev = container_of(led_cdev, struct ktd_flash_led_cdev, cdev);
	unsigned long value;
	ssize_t ret;

	ret = kstrtoul(buf, 10, &value);
	if (ret)
		return ret;
#ifdef CONFIG_KTD_FLASH_DEBUG
	flash_led_cdev->prgm_current = value;
	/* led_ktd_flash_set_flash_current(flash_led_cdev->parent, value); */
	/* flash_current = value; */
	curr_manual_mode = 1;
#endif
	return size;
}

static DEVICE_ATTR(flash_current, 0644, flash_current_show, flash_current_store);

static ssize_t torch_off_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct ktd_flash_led_cdev *flash_led_cdev = container_of(led_cdev, struct ktd_flash_led_cdev, cdev);

	if (is_torch_used_from_camera == 0) {
		led_ktd_flash_senddata(flash_led_cdev->parent, 0xA0);
		turn_on = 0;
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", turn_on);
}
static DEVICE_ATTR(torch_off, 0444, torch_off_show, NULL);

static ssize_t torch_on_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct ktd_flash_led_cdev *flash_led_cdev = container_of(led_cdev, struct ktd_flash_led_cdev, cdev);

	if (is_torch_used_from_camera == 0) {
		/* turn off timer function */
		led_ktd_flash_set_flash_duration(flash_led_cdev->parent, 0);
		/* set torch op current */
		led_ktd_flash_set_torch_current(flash_led_cdev->parent, flash_led_cdev->prgm_current);
		/* set torch enable */
		led_ktd_flash_senddata(flash_led_cdev->parent, 0xa1);
		turn_on = 1;
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", turn_on);
}
static DEVICE_ATTR(torch_on, 0444, torch_on_show, NULL);

static ssize_t torch_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", turn_on);
}
static DEVICE_ATTR(torch_state, 0444, torch_state_show, NULL);

static ssize_t torch_current_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct ktd_flash_led_cdev *flash_led_cdev = container_of(led_cdev, struct ktd_flash_led_cdev, cdev);

	return snprintf(buf, PAGE_SIZE, "%lu\n", (unsigned long)flash_led_cdev->prgm_current);
}

static ssize_t torch_current_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct ktd_flash_led_cdev *flash_led_cdev = container_of(led_cdev, struct ktd_flash_led_cdev, cdev);
	unsigned long value;
	ssize_t ret;

	ret = kstrtoul(buf, 10, &value);
	if (ret)
		return ret;
#ifdef CONFIG_KTD_FLASH_DEBUG
	flash_led_cdev->prgm_current = value;
	/* led_ktd_flash_set_torch_current(flash_led_cdev->parent, value); */
	/* torch_current = value; */
	curr_manual_mode = 1;
#endif
	return size;
}

static DEVICE_ATTR(torch_current, 0644, torch_current_show, torch_current_store);

static ssize_t flash_duration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%lu\n", flash_duration);
}

static ssize_t flash_duration_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct ktd_flash_led_cdev *flash_led_cdev = container_of(led_cdev, struct ktd_flash_led_cdev, cdev);
	unsigned long value;
	ssize_t ret;

	ret = kstrtoul(buf, 10, &value);
	if (ret)
		return ret;
#ifdef CONFIG_KTD_FLASH_DEBUG
	led_ktd_flash_set_flash_duration(flash_led_cdev->parent, value);
	flash_duration = value;
#endif
	return size;
}

static DEVICE_ATTR(flash_duration, 0644, flash_duration_show, flash_duration_store);

static ssize_t min_current_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%lu\n", min_current);
}

static ssize_t min_current_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct ktd_flash_led_cdev *flash_led_cdev = container_of(led_cdev, struct ktd_flash_led_cdev, cdev);
	unsigned long value;
	ssize_t ret;

	ret = kstrtoul(buf, 10, &value);
	if (ret)
		return ret;
#ifdef CONFIG_KTD_FLASH_DEBUG
	led_ktd_flash_set_min_current(flash_led_cdev->parent, value);
	min_current = value;
#endif
	return size;
}

static DEVICE_ATTR(min_current, 0644, min_current_show, min_current_store);

static int led_ktd_flash_parse_each_cdev_dt(struct device_node *node,
					struct ktd_flash_led_data *flash_node)
{
	struct ktd_flash_led_cdev *flash_led_cdev = NULL;
	const char *temp_string;
	int rc = 0;
	u32 val;

	rc = of_property_read_string(node, "label", &temp_string);
	if (!rc) {
		if (strcmp(temp_string, "flash") == 0) {
			flash_led_cdev = &flash_node->flash_cdev;
			flash_led_cdev->type = FLASH;
		} else if (strcmp(temp_string, "torch") == 0) {
			flash_led_cdev = &flash_node->torch_cdev;
			flash_led_cdev->type = TORCH;
		} else {
			pr_err("Wrong flash LED type\n");
			return -EINVAL;
		}
	} else if (rc < 0) {
		pr_err("Unable to read flash type\n");
		return rc;
	}

	rc = of_property_read_string(node, "qcom,led-name",
					&flash_led_cdev->cdev.name);
	if (rc < 0) {
		rc = of_property_read_string(node, "linux,name",
						&flash_led_cdev->cdev.name);
		if (rc < 0) {
			pr_err("Unable to read flash name\n");
			return rc;
		}
	}

	rc = of_property_read_string(node, "qcom,default-led-trigger",
			&flash_led_cdev->cdev.default_trigger);
	if (rc < 0) {
		rc = of_property_read_string(node, "linux,default-trigger",
						&flash_led_cdev->cdev.default_trigger);
		if (rc < 0) {
			pr_err("Unable to read trigger name\n");
			return rc;
		}
	}

	rc = of_property_read_u32(node, "qcom,max-current", &val);
	if (!rc) {
		flash_led_cdev->max_current = (u16)val;
		flash_led_cdev->cdev.max_brightness = val;
	} else if (rc < 0) {
		pr_err("Unable to read max current\n");
		return rc;
	}

	rc = of_property_read_u32(node, "qcom,current", &val);
	if (!rc) {
		flash_led_cdev->prgm_current = (u16)val;
	} else if (rc != -EINVAL) {
		pr_err("Unable to read current settings\n");
		goto error;
	} else {
		pr_err("Set default current\n");
		flash_led_cdev->prgm_current = (flash_led_cdev->max_current / 2);
	}

	rc = of_property_read_u32(node, "qcom,duration", &val);
	if (!rc)
		flash_led_cdev->duration = (u16)val;
	else if (rc != -EINVAL) {
		pr_err("Unable to read clamp current\n");
		goto error;
	} else {
		if (flash_led_cdev->type == FLASH) {
			pr_err("Set default flash duratioin\n");
			flash_led_cdev->duration = 1050;
		} else {
			pr_err("No duration needed for torch type\n");
			flash_led_cdev->duration = 0;
		}
	}

	flash_led_cdev->cdev.brightness_set = led_ktd_brightness_set;
	flash_led_cdev->cdev.brightness_get = led_ktd_brightness_get;
	flash_led_cdev->parent = flash_node;
	return 0;
error:
	return rc;
}

static int led_ktd_flash_parse_each_led_dt(struct device_node *node,
					struct ktd_flash_led_data *flash_node)
{
	struct device_node *temp;
	int rc = 0, num_cdev = 0;

	flash_node->flash_ctrl->gpio = of_get_named_gpio(node, "qcom,flash-ctrl", 0);
	if (flash_node->flash_ctrl->gpio < 0) {
		pr_err("Looking up %s property in node %s failed. rc =  %d\n",
			"flash-ctrl", node->full_name, flash_node->flash_ctrl->gpio);
		rc = -EINVAL;
		goto error;
	} else {
		rc = of_property_read_string(node, "qcom,flash-ctrl-label", &flash_node->flash_ctrl->label);
		if (rc < 0) {
			pr_err("Unable to read flash-ctrl-label name\n");
			return rc;
		}
		rc = gpio_request_one(flash_node->flash_ctrl->gpio, GPIOF_OUT_INIT_HIGH, flash_node->flash_ctrl->label);
		if (rc) {
			pr_err("%s: Failed to request gpio %d,rc = %d\n",
				__func__, flash_node->flash_ctrl->gpio, rc);
			return rc;
		}
	}

#ifdef CONFIG_KTD_ALLGPIO
	flash_node->flash_strobe->gpio = of_get_named_gpio(node, "qcom,flash-strobe", 0);
	if (flash_node->flash_strobe->gpio < 0) {
		pr_err("Looking up %s property in node %s failed. rc =  %d\n",
			"flash-strobe", node->full_name, flash_node->flash_strobe->gpio);
		gpio_free(flash_node->flash_ctrl->gpio);
		rc = -EINVAL;
		goto error;
	} else {
		rc = of_property_read_string(node, "qcom,flash-strobe-label", &flash_node->flash_strobe->label);
		if (rc < 0) {
			pr_err("Unable to read flash-strobe-label name\n");
			gpio_free(flash_node->flash_ctrl->gpio);
			goto error;
		}
		rc = gpio_request_one(flash_node->flash_strobe->gpio, GPIOF_OUT_INIT_LOW
				      , flash_node->flash_strobe->label);
		if (rc) {
			pr_err("%s: Failed to request gpio %d,rc = %d\n",
				__func__, flash_node->flash_strobe->gpio, rc);
			gpio_free(flash_node->flash_ctrl->gpio);
			goto error;
		}
	}

	flash_node->flash_tx->gpio = of_get_named_gpio(node, "qcom,flash-tx", 0);
	if (flash_node->flash_tx->gpio < 0) {
		pr_err("Looking up %s property in node %s failed. rc =  %d\n",
			"flash-tx", node->full_name, flash_node->flash_tx->gpio);
		gpio_free(flash_node->flash_ctrl->gpio);
		gpio_free(flash_node->flash_strobe->gpio);
		rc = -EINVAL;
		goto error;
	} else {
		rc = of_property_read_string(node, "qcom,flash-tx-label", &flash_node->flash_tx->label);
		if (rc < 0) {
			pr_err("Unable to read flash-tx-label name\n");
			gpio_free(flash_node->flash_ctrl->gpio);
			gpio_free(flash_node->flash_strobe->gpio);
			rc = -EINVAL;
			goto error;
		}
		rc = gpio_request_one(flash_node->flash_tx->gpio, GPIOF_OUT_INIT_LOW, flash_node->flash_tx->label);
		if (rc) {
			pr_err("%s: Failed to request gpio %d,rc = %d\n",
				__func__, flash_node->flash_tx->gpio, rc);
			gpio_free(flash_node->flash_ctrl->gpio);
			gpio_free(flash_node->flash_strobe->gpio);
			goto error;
		}
	}

#endif
	temp = NULL;
	while ((temp = of_get_next_child(node, temp)))
		num_cdev++;

	if (num_cdev != LED_KTD_CDEV_NUM) {
		pr_err("Wrong led cdev number: %d except: %d\n", num_cdev, LED_KTD_CDEV_NUM);
		return -EINVAL;
	}

	for_each_child_of_node(node, temp) {
		rc = led_ktd_flash_parse_each_cdev_dt(temp, flash_node);
		if (rc) {
			gpio_free(flash_node->flash_ctrl->gpio);
#ifdef CONFIG_KTD_ALLGPIO
			gpio_free(flash_node->flash_strobe->gpio);
			gpio_free(flash_node->flash_tx->gpio);
#endif
			return rc;
		}
	}

	return 0;
error:
	return rc;
}

int led_ktd_flash_probe(struct platform_device *pdev)
{
	struct ktd_flash_led *flash_led = NULL;
	struct device_node *temp, *node = pdev->dev.of_node;
	int rc, i = 0, num_leds = 0;

	if (!node) {
		pr_err("of_node NULL\n");
		return -EINVAL;
	}

	flash_led = devm_kzalloc(&pdev->dev, sizeof(struct ktd_flash_led),
				 GFP_KERNEL);
	if (flash_led == NULL)
		return -ENOMEM;

	flash_led->pdev = pdev;

	temp = NULL;
	while ((temp = of_get_next_child(node, temp)))
		num_leds++;

	if (!num_leds)
		return -ECHILD;

	flash_led->num_leds = num_leds;
	flash_led->flash_node = devm_kzalloc(&pdev->dev,
			(sizeof(struct ktd_flash_led_data) * num_leds),
			GFP_KERNEL);
	if (!flash_led->flash_node) {
		pr_err("Unable to allocate memory\n");
		devm_kfree(&pdev->dev, flash_led);
		return -ENOMEM;
	}

	flash_led->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR_OR_NULL(flash_led->pinctrl)) {
		pr_err("%s:failed to get pinctrl\n", __func__);
		rc = PTR_ERR(flash_led->pinctrl);
		goto error;
	}

	flash_led->gpio_state_default = pinctrl_lookup_state(flash_led->pinctrl,
		"ktd_flash_default");
	if (IS_ERR_OR_NULL(flash_led->gpio_state_default)) {
		pr_err("%s:can not get active pinstate\n", __func__);
		rc = -EINVAL;
		goto error;
	}

	flash_led->gpio_state_suspend = pinctrl_lookup_state(flash_led->pinctrl,
		"ktd_flash_suspend");
	if (IS_ERR_OR_NULL(flash_led->gpio_state_suspend))
		pr_err("%s:can not get suspend pinstate\n", __func__);


	rc = pinctrl_select_state(flash_led->pinctrl,
		flash_led->gpio_state_default);
	if (rc)
		pr_err("%s:set state failed!\n", __func__);

	for_each_child_of_node(node, temp) {
		flash_led->flash_node[i].pdev = pdev;
		flash_led->flash_node[i].flash_ctrl = &flash_led->flash_node[i].ctrl_gpio[0];
#ifdef CONFIG_KTD_ALLGPIO
		flash_led->flash_node[i].flash_strobe = &flash_led->flash_node[i].ctrl_gpio[1];
		flash_led->flash_node[i].flash_tx = &flash_led->flash_node[i].ctrl_gpio[2];
#endif

		rc = led_ktd_flash_parse_each_led_dt(temp, &flash_led->flash_node[i]);
		if (rc) {
			pr_err("%s: Failed to parse led dt. rc = %d\n",
				__func__, rc);
			/* free gpios of previous nodes */
			goto error;
		}

		/* Disable LVP */
		led_ktd_flash_senddata(&flash_led->flash_node[i], 0x00);
		/* led_ktd_flash_senddata(&flash_led->flash_node[i], 0x40); */
		led_ktd_flash_set_min_current(&flash_led->flash_node[i], 0x07);
		/* led_ktd_flash_set_flash_current(&flash_led->flash_node[i],
		 * flash_led->flash_node[i].flash_cdev.prgm_current); */
		/* led_ktd_flash_set_torch_current(&flash_led->flash_node[i],
		 * flash_led->flash_node[i].torch_cdev.prgm_current); */
		/* led_ktd_flash_set_flash_duration(&flash_led->flash_node[i],
		 * flash_led->flash_node[i].flash_cdev.duration); */
		/* led_ktd_flash_senddata(&flash_led->flash_node[i], 0xA1); */

		rc = led_classdev_register(&pdev->dev, &flash_led->flash_node[i].flash_cdev.cdev);
		if (rc) {
			pr_err("%s: Failed to register led dev. rc = %d\n",
				__func__, rc);
			goto error;
		}
		rc = device_create_file(flash_led->flash_node[i].flash_cdev.cdev.dev, &dev_attr_flash_duration);
		if (rc < 0) {
			dev_err(flash_led->flash_node[i].flash_cdev.cdev.dev, "failed to create flash_duration file\n");
			goto error_flash;
		}
		rc = device_create_file(flash_led->flash_node[i].flash_cdev.cdev.dev, &dev_attr_flash_current);
		if (rc < 0) {
			dev_err(flash_led->flash_node[i].flash_cdev.cdev.dev, "failed to create flash_current file\n");
			goto error_flash;
		}
		rc = device_create_file(flash_led->flash_node[i].flash_cdev.cdev.dev, &dev_attr_min_current);
		if (rc < 0) {
			dev_err(flash_led->flash_node[i].flash_cdev.cdev.dev, "failed to create min_current file\n");
			goto error_flash;
		}

		rc = led_classdev_register(&pdev->dev, &flash_led->flash_node[i].torch_cdev.cdev);
		if (rc) {
			pr_err("%s: Failed to register led dev. rc = %d\n",
				__func__, rc);
			led_classdev_unregister(&flash_led->flash_node[i].flash_cdev.cdev);
			goto error;
		}

		rc = device_create_file(flash_led->flash_node[i].torch_cdev.cdev.dev, &dev_attr_torch_current);
		if (rc < 0) {
			dev_err(flash_led->flash_node[i].torch_cdev.cdev.dev, "failed to create torch_current file\n");
			goto error_torch;
		}
		rc = device_create_file(flash_led->flash_node[i].torch_cdev.cdev.dev, &dev_attr_min_current);
		if (rc < 0) {
			dev_err(flash_led->flash_node[i].torch_cdev.cdev.dev, "failed to create min_current file\n");
			goto error_flash;
		}

		rc = device_create_file(flash_led->flash_node[i].torch_cdev.cdev.dev, &dev_attr_torch_off);
		if (rc < 0) {
			dev_err(flash_led->flash_node[i].torch_cdev.cdev.dev, "failed to create torch_off file\n");
			goto error_flash;
		}
		rc = device_create_file(flash_led->flash_node[i].torch_cdev.cdev.dev, &dev_attr_torch_on);
		if (rc < 0) {
			dev_err(flash_led->flash_node[i].torch_cdev.cdev.dev, "failed to create torch_on file\n");
			goto error_flash;
		}
		rc = device_create_file(flash_led->flash_node[i].torch_cdev.cdev.dev, &dev_attr_torch_state);
		if (rc < 0) {
			dev_err(flash_led->flash_node[i].torch_cdev.cdev.dev, "failed to create torch_state file\n");
			goto error_flash;
		}

		flash_led->flash_node[i].suspend_state = 0;

		i++;
	}

	platform_set_drvdata(pdev, flash_led);

	CDBG("%s:probe successfully!\n", __func__);
	return 0;
error_torch:
	led_classdev_unregister(&flash_led->flash_node[i].torch_cdev.cdev);
error_flash:
	led_classdev_unregister(&flash_led->flash_node[i].flash_cdev.cdev);
error:
	if (!IS_ERR_OR_NULL(flash_led->pinctrl))
		devm_pinctrl_put(flash_led->pinctrl);
	devm_kfree(&pdev->dev, flash_led->flash_node);
	devm_kfree(&pdev->dev, flash_led);
	return rc;
}

int led_ktd_flash_remove(struct platform_device *pdev)
{
	int i;
	struct ktd_flash_led *flash_led =
	    (struct ktd_flash_led *)platform_get_drvdata(pdev);
	if (!IS_ERR_OR_NULL(flash_led->pinctrl))
		devm_pinctrl_put(flash_led->pinctrl);

	for (i = 0; i < flash_led->num_leds; i++) {
		gpio_free(flash_led->flash_node[i].flash_ctrl->gpio);
#ifdef CONFIG_KTD_ALLGPIO
		gpio_free(flash_led->flash_node[i].flash_strobe->gpio);
		gpio_free(flash_led->flash_node[i].flash_tx->gpio);
#endif
		led_classdev_unregister(&flash_led->flash_node[i].flash_cdev.cdev);
		led_classdev_unregister(&flash_led->flash_node[i].torch_cdev.cdev);

	}
	devm_kfree(&pdev->dev, flash_led->flash_node);
	devm_kfree(&pdev->dev, flash_led);
	return 0;
}

static int ktd_dev_pm_suspend(struct device *dev)
{
	struct platform_device *pdev;
	struct ktd_flash_led *flash_led;
	int i;

	pdev = container_of(dev, struct platform_device, dev);
	flash_led = (struct ktd_flash_led *)platform_get_drvdata(pdev);
	CDBG("ktd_dev_pm_suspend\n");
	for (i = 0; i < flash_led->num_leds; i++) {
		CDBG("ktd_dev_pm_suspend: torch[%d] flash[%d]\n", flash_led->flash_node[i].torch_cdev.cdev.brightness
		     , flash_led->flash_node[i].flash_cdev.cdev.brightness);
		if (flash_led->flash_node[i].flash_cdev.cdev.brightness == 0 &&
			flash_led->flash_node[i].torch_cdev.cdev.brightness == 0) {
			CDBG("ktd_dev_pm_suspend: suspend node[%d]\n", i);
			flash_led->flash_node[i].suspend_state = 1;
			/*Keep led on in suspend*/
			if (turn_on == 0) {
				gpio_set_value(flash_led->flash_node[i].flash_ctrl->gpio, GPIO_OUT_LOW);
				gpio_free(flash_led->flash_node[i].flash_ctrl->gpio);
			} else {
				CDBG("ktd_dev_pm_suspend: Keep torch on in suspend[%d]\n", i);
			}
#ifdef CONFIG_KTD_ALLGPIO
			gpio_free(flash_led->flash_node[i].flash_strobe->gpio);
			gpio_free(flash_led->flash_node[i].flash_tx->gpio);
#endif
		}
	}
	return 0;
}

static int ktd_dev_pm_resume(struct device *dev)
{
	struct platform_device *pdev;
	struct ktd_flash_led *flash_led;
	int i;

	pdev = container_of(dev, struct platform_device, dev);
	flash_led = (struct ktd_flash_led *)platform_get_drvdata(pdev);
	CDBG("ktd_dev_pm_resume\n");
	for (i = 0; i < flash_led->num_leds; i++) {
		if (flash_led->flash_node[i].flash_cdev.cdev.brightness == 0 &&
			flash_led->flash_node[i].torch_cdev.cdev.brightness == 0) {
			gpio_request_one(flash_led->flash_node[i].flash_ctrl->gpio, GPIOF_OUT_INIT_HIGH
					 , flash_led->flash_node[i].flash_ctrl->label);
#ifdef CONFIG_KTD_ALLGPIO
			gpio_request_one(flash_led->flash_node[i].flash_strobe->gpio, GPIOF_OUT_INIT_LOW
					 , flash_led->flash_node[i].flash_strobe->label);
			gpio_request_one(flash_led->flash_node[i].flash_tx->gpio, GPIOF_OUT_INIT_LOW
					 , flash_led->flash_node[i].flash_tx->label);
#endif
			flash_led->flash_node[i].suspend_state = 0;
			/* Disable LVP */
			led_ktd_flash_senddata(&flash_led->flash_node[i], 0x00);
			led_ktd_flash_set_min_current(&flash_led->flash_node[i], 0x07);
		}
	}
	return 0;
}

static const struct dev_pm_ops ktd_dev_pm_ops = {
	.suspend = &ktd_dev_pm_suspend,
	.resume = &ktd_dev_pm_resume,
};

static struct platform_driver led_ktd_flash_driver = {
	.probe = led_ktd_flash_probe,
	.remove = led_ktd_flash_remove,
	.driver = {
		.name = LED_KTD_FLASH_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = led_ktd_flash_of_match,
		.pm = &ktd_dev_pm_ops,
	}
};

static int __init led_ktd_flash_init(void)
{
	return platform_driver_register(&led_ktd_flash_driver);
}

static void __exit led_ktd_flash_exit(void)
{
	return platform_driver_unregister(&led_ktd_flash_driver);
}

late_initcall(led_ktd_flash_init);
module_exit(led_ktd_flash_exit);

MODULE_DESCRIPTION("KTD FLASH LEDs driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("leds:leds-ktd-flash");

