/* drivers/input/misc/hsppad038_i2c.c
 *
 * Pressure device driver (HSPPAD038)
 *
 * Copyright (C) 2012-2014 ALPS ELECTRIC CO., LTD. All Rights Reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*#define ALPS_PRS_DEBUG 0*/

#ifdef ALPS_PRS_DEBUG
#define DEBUG 1
#endif

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#define HSPPAD_DRIVER_NAME		"hsppad038"
#define HSPPAD_LOG_TAG			"[HSPPAD], "
#define HSPPAD_INPUT_DEVICE_NAME	"alps_pressure"

#define HSPPAD_DELAY(us)	usleep_range(us, us)

#define I2C_RETRIES		5

#define DELAY_TIME		10000

/* Comannd for hsppad */
#define HSPPAD_GET		0xAC

#define HSPPAD_DATA_ACCESS_NUM	5
#define HSPPAD_INITIALL_DELAY	20

/*VDD 2.375V-3.46V VLOGIC 1.8V +-5%*/
#define HSPPAD038_VDD_MIN_UV		1800000
#define HSPPAD038_VDD_MAX_UV		1800000
/*
#define HSPPAD038_VI2C_MIN_UV	1750000
#define HSPPAD038_VI2C_MAX_UV	1950000
*/
/* input types for timestamps  */
#define INPUT_EVENT_TIME_TYPE		EV_MSC
#define INPUT_EVENT_TIME_MSB		MSC_SCAN
#define INPUT_EVENT_TIME_LSB		MSC_MAX

struct hsppad_data {
	struct input_dev	*input;
	struct i2c_client	*i2c;
	struct work_struct	work_data;
	struct workqueue_struct	*work_queue;
	struct hrtimer		poll_timer;
	struct mutex		lock;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend	early_suspend_h;
#endif
	unsigned int		delay_msec;
	bool			factive;
	bool			fsuspend;
	/* power control */
	struct regulator *vdd;
	bool power_enabled;
	int previous_prs;
	int previous_tmp;
	bool flag_previous;
};

static struct kobject *pres_sysfs_link;

/*--------------------------------------------------------------------------
 *power control function
 *--------------------------------------------------------------------------*/
static int hsppad_power_ctl(struct hsppad_data *sensor, bool on)
{
	int rc = 0;

	if (on && (!sensor->power_enabled)) {
		rc = regulator_enable(sensor->vdd);
		if (rc) {
			dev_err(&sensor->i2c->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			return rc;
		}
		sensor->power_enabled = true;
	} else if ((!on) && (sensor->power_enabled)) {

		if (rc) {
			dev_err(&sensor->i2c->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}

		sensor->power_enabled = false;
	} else {
		dev_warn(&sensor->i2c->dev,
				"Ignore power status change from %d to %d\n",
				on, sensor->power_enabled);
	}
	return rc;
}

static int hsppad_power_init(struct hsppad_data *sensor)
{
	int ret = 0;

	sensor->vdd = regulator_get(&sensor->i2c->dev, "vdd");
	if (IS_ERR(sensor->vdd)) {
		ret = PTR_ERR(sensor->vdd);
		dev_err(&sensor->i2c->dev,
			"Regulator get failed vdd ret=%d\n", ret);
		return ret;
	}

	if (regulator_count_voltages(sensor->vdd) > 0) {
		ret = regulator_set_voltage(sensor->vdd, HSPPAD038_VDD_MIN_UV,
					   HSPPAD038_VDD_MAX_UV);
		if (ret) {
			dev_err(&sensor->i2c->dev,
				"Regulator set_vtg failed vdd ret=%d\n", ret);
			/*goto reg_vdd_put;*/
		}
	}

/*
	sensor->vlogic = regulator_get(&sensor->i2c->dev, "vlogic");
	if (IS_ERR(sensor->vlogic)) {
		ret = PTR_ERR(sensor->vlogic);
		dev_err(&sensor->i2c->dev,
			"Regulator get failed vlogic ret=%d\n", ret);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(sensor->vlogic) > 0) {
		ret = regulator_set_voltage(sensor->vlogic,
				HSPPAD038_VLOGIC_MIN_UV,
				HSPPAD038_VLOGIC_MAX_UV);
		if (ret) {
			dev_err(&sensor->i2c->dev,
			"Regulator set_vtg failed vlogic ret=%d\n", ret);
			//goto reg_vlogic_put;
		}
	}
*/
/*
	sensor->vi2c = regulator_get(&sensor->client->dev, "vi2c");
	if (IS_ERR(sensor->vi2c)) {
		ret = PTR_ERR(sensor->vi2c);
		dev_info(&sensor->client->dev,
			"Regulator get failed vi2c ret=%d\n", ret);
		sensor->vi2c = NULL;
	} else if (regulator_count_voltages(sensor->vi2c) > 0) {
		ret = regulator_set_voltage(sensor->vi2c,
				MPU6050_VI2C_MIN_UV,
				MPU6050_VI2C_MAX_UV);
		if (ret) {
			dev_err(&sensor->client->dev,
			"Regulator set_vtg failed vi2c ret=%d\n", ret);
			goto reg_vi2c_put;
		}
	}
*/
	return 0;
/*
reg_vi2c_put:
	regulator_put(sensor->vi2c);
	if (regulator_count_voltages(sensor->vlogic) > 0)
		regulator_set_voltage(sensor->vlogic, 0, MPU6050_VLOGIC_MAX_UV);

reg_vlogic_put:
	regulator_put(sensor->vlogic);
*/
/*
reg_vdd_set_vtg:
	if (regulator_count_voltages(sensor->vdd) > 0)
		regulator_set_voltage(sensor->vdd, 0, HSPPAD038_VDD_MAX_UV);
*/
/*reg_vdd_put:
	regulator_put(sensor->vdd);
	return ret
*/;
}

static int hsppad_power_deinit(struct hsppad_data *sensor)
{
	int ret = 0;

	return 0;
/*
	if (regulator_count_voltages(sensor->vlogic) > 0)
		regulator_set_voltage(sensor->vlogic, 0, HSPPAD038_VLOGIC_MAX_UV);
	regulator_put(sensor->vlogic);
*/
	if (regulator_count_voltages(sensor->vdd) > 0)
		regulator_set_voltage(sensor->vdd, 0, HSPPAD038_VDD_MAX_UV);
	/*regulator_put(sensor->vdd);*/
	return ret;
}

/*--------------------------------------------------------------------------
 * i2c read/write function
 *--------------------------------------------------------------------------*/
static int hsppad_i2c_read(struct i2c_client *i2c, u8 *rxData, int length)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
			.addr	= i2c->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxData,
		},
	};

	do {
		err = i2c_transfer(i2c->adapter, msgs, 1);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		dev_err(&i2c->adapter->dev, "read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int hsppad_i2c_write(struct i2c_client *i2c, u8 *txData, int length)
{
	int err;
	int tries = 0;

	struct i2c_msg msg[] = {
		{
			.addr	= i2c->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txData,
		},
	};

	do {
		err = i2c_transfer(i2c->adapter, msg, 1);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		dev_err(&i2c->adapter->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}


/*--------------------------------------------------------------------------
 * hsppad function
 *--------------------------------------------------------------------------*/
static int hsppad_get_pressure_data(
			struct hsppad_data *hsppad, int *pt)
{
	int err = -1;
	u8 sx[HSPPAD_DATA_ACCESS_NUM];

	if (hsppad->fsuspend != 0)
		return err;
	err = hsppad_i2c_read(hsppad->i2c, sx,
		HSPPAD_DATA_ACCESS_NUM);
	if (err < 0)
		return err;

	if ((sx[0] & 0xFE) != 0x40) {
		dev_dbg(&hsppad->i2c->adapter->dev,
		HSPPAD_LOG_TAG "status bit error=0x%02X\n", sx[0]);

		dev_dbg(&hsppad->i2c->adapter->dev,
				HSPPAD_LOG_TAG "[ERROR ]prs:%d,tmp:%d\n", pt[0], pt[1]);

		if (hsppad->flag_previous) {
			pt[0] = hsppad->previous_prs;
			pt[1] = hsppad->previous_tmp;
			return err;
		} else {
			return -EPERM;
		}
	} else {
			pt[0] = (int) (((u16)sx[1] << 8) | (u16)sx[2]);
			pt[1] = (int) (((u16)sx[3] << 8) | (u16)sx[4]);
			dev_dbg(&hsppad->i2c->adapter->dev,
					HSPPAD_LOG_TAG "prs:%d,tmp:%d\n", pt[0], pt[1]);
			hsppad->previous_prs = pt[0];
			hsppad->previous_tmp = pt[1];
			if (!hsppad->flag_previous)
				hsppad->flag_previous = true;
	}

	return err;
}

static int hsppad_force_setup(struct hsppad_data *hsppad)
{
	u8 buf = HSPPAD_GET;

	return hsppad_i2c_write(hsppad->i2c, &buf, 1);
}

static void hsppad_measure_start(struct hsppad_data *hsppad)
{
	dev_err(&hsppad->i2c->adapter->dev,
		HSPPAD_LOG_TAG "%s\n", __func__);

	hsppad_force_setup(hsppad);

	hrtimer_start(&hsppad->poll_timer,
			ns_to_ktime(hsppad->delay_msec * 1000 * 1000),
			HRTIMER_MODE_REL);

	if (!hsppad->fsuspend)
		hsppad->factive = true;
}

static void hsppad_measure_stop(struct hsppad_data *hsppad)
{
	dev_err(&hsppad->i2c->adapter->dev,
		HSPPAD_LOG_TAG "%s\n", __func__);

	hrtimer_cancel(&hsppad->poll_timer);

	if (!hsppad->fsuspend)
		hsppad->factive = false;
	/*cancel_delayed_work(&hsppad->work_data);*/ /*move before mutex lock*/
}

static void hsppad_get_hardware_data(
			struct hsppad_data *hsppad, int *pt)
{
	hsppad_force_setup(hsppad);
	HSPPAD_DELAY(DELAY_TIME);
	hsppad_get_pressure_data(hsppad, pt);
}


/*--------------------------------------------------------------------------
 * sysfs
 *--------------------------------------------------------------------------*/
static ssize_t hsppad_enable_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct hsppad_data *hsppad = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", (hsppad->factive) ? 1 : 0);
}

static ssize_t hsppad_enable_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t size)
{
	struct hsppad_data *hsppad = dev_get_drvdata(dev);
	int new_value;

	if (sysfs_streq(buf, "1"))
		new_value = 1;
	else if (sysfs_streq(buf, "0"))
		new_value = 0;
	else {
		dev_err(&hsppad->i2c->adapter->dev,
			HSPPAD_LOG_TAG "%s: invalid value %d\n",
			__func__, *buf);
		return -EINVAL;
	}

	dev_dbg(&hsppad->i2c->adapter->dev,
		HSPPAD_LOG_TAG "%s, enable = %d\n", __func__, new_value);

	if (new_value == 0)
		/*cancel_delayed_work(&hsppad->work_data);*/
		cancel_work_sync(&hsppad->work_data);

	mutex_lock(&hsppad->lock);
	if (new_value)
		hsppad_measure_start(hsppad);
	else
		hsppad_measure_stop(hsppad);
	mutex_unlock(&hsppad->lock);

	return size;
}

static ssize_t hsppad_delay_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct hsppad_data *hsppad = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", hsppad->delay_msec);
}

static ssize_t hsppad_delay_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t size)
{
	int err;
	long new_delay;
	struct hsppad_data *hsppad = dev_get_drvdata(dev);

	err = strict_strtol(buf, 10, &new_delay);
	if (err < 0)
		return err;

	mutex_lock(&hsppad->lock);
	if (new_delay < 10)
		new_delay = 10;
	else if (new_delay > 200)
		new_delay = 200;

	hsppad->delay_msec = (int)new_delay;
	mutex_unlock(&hsppad->lock);

	dev_dbg(&hsppad->i2c->adapter->dev,
		HSPPAD_LOG_TAG "%s, rate = %d (msec)\n",
		__func__, hsppad->delay_msec);

	return size;
}

static ssize_t hsppad_get_hw_data_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int pt[2];
	struct hsppad_data *hsppad = dev_get_drvdata(dev);

	if (!hsppad->factive) {
		mutex_lock(&hsppad->lock);
		hsppad_get_hardware_data(hsppad, pt);
		mutex_unlock(&hsppad->lock);
	} else
		dev_err(&hsppad->i2c->adapter->dev,
			HSPPAD_LOG_TAG "Please turn off sensor\n");

	return snprintf(buf, PAGE_SIZE, "%d\n", pt[0]);
}
/* ALPS modified 20150323 start */
static ssize_t hsppad_get_hw_data_store(struct device *dev,
	    struct device_attribute *attr,
	    const char *buf, size_t size)
{
	return size;
}

static ssize_t hsppad_get_hw_data_show_hPa(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int pt[2];
	int data[2];
	struct hsppad_data *hsppad = dev_get_drvdata(dev);

	if (!hsppad->factive) {
		mutex_lock(&hsppad->lock);
		hsppad_get_hardware_data(hsppad, pt);
		mutex_unlock(&hsppad->lock);
	} else
		dev_err(&hsppad->i2c->adapter->dev,
			HSPPAD_LOG_TAG "Please turn off sensor\n");

	data[1] = (pt[1] * 125 - 2621400) * 100 / 65535;
	data[0] = pt[0] - (139 * data[1] + 1970)/100/100;

	if (data[0] > 65535)
		data[0] = 65535;
	if (data[0] < 0)
		data[0] = 0;

	data[0] = data[0] * 860 / 65535 + 250;   /* ALPS modified 20150401 bug fix */
	data[1] = data[1] * 125 / 65535 - 40;    /* ALPS modified 20150401 bug fix */

	return snprintf(buf, PAGE_SIZE, "%d\n", data[0]);
}

static ssize_t hsppad_get_hw_data_store_hPa(struct device *dev,
	    struct device_attribute *attr,
	    const char *buf, size_t size)
{
	return size;
}
/* ALPS modified 20150323 end */
/* ALPS modified 20150402 start */
static ssize_t hsppad_get_WIA_data_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int err = -1;
	u8 sx[3];
	u8 buffer;
	struct hsppad_data *hsppad = dev_get_drvdata(dev);

	mutex_lock(&hsppad->lock);

	buffer = 0x01;
	err = hsppad_i2c_write(hsppad->i2c, &buffer, 1);
	if (err != 0)
			dev_err(&hsppad->i2c->adapter->dev,
			HSPPAD_LOG_TAG "write_error\n");

	HSPPAD_DELAY(DELAY_TIME);

	err = hsppad_i2c_read(hsppad->i2c, sx, 3);
	if (err != 0)
			dev_err(&hsppad->i2c->adapter->dev,
			HSPPAD_LOG_TAG "read_error\n");
	mutex_unlock(&hsppad->lock);

	dev_dbg(&hsppad->i2c->adapter->dev,
	HSPPAD_LOG_TAG "status:%d, sx[1]:%d, sx[2]:%d\n", sx[0], sx[1], sx[2]);

	return snprintf(buf, PAGE_SIZE, "%d\n", sx[2]);

}

static ssize_t hsppad_get_WIA_data_store(struct device *dev,
	    struct device_attribute *attr,
	    const char *buf, size_t size)
{
	return size;
}
/* ALPS modified 20150402 end */

static ssize_t get_devnum(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct hsppad_data *hsppad = dev_get_drvdata(dev);
	int devnum, ret;
	const char *devname = NULL;

	devname = dev_name(&hsppad->input->dev);
	ret = sscanf(devname+5, "%d", &devnum);
	if (!ret)
		pr_info("[Sensor] get device number faild");

	return snprintf(buf, PAGE_SIZE, "%d\n", devnum);
}

static struct device_attribute attributes[] = {
	__ATTR(enable, S_IWUGO | S_IRUGO,
		hsppad_enable_show, hsppad_enable_store),
	__ATTR(delay, S_IWUGO | S_IRUGO,
		hsppad_delay_show, hsppad_delay_store),
/* ALPS modified 20150323 start */
	__ATTR(get_hw_data, S_IRUGO,
		hsppad_get_hw_data_show,  hsppad_get_hw_data_store),
	__ATTR(get_hw_data_hPa, S_IRUGO,
		hsppad_get_hw_data_show_hPa,  hsppad_get_hw_data_store_hPa),
/* ALPS modified 20150323 end */
/* ALPS modified 20150402 start */
	__ATTR(get_WIA_data, S_IRUGO,
		hsppad_get_WIA_data_show,  hsppad_get_WIA_data_store),
/* ALPS modified 20150402 end */
	__ATTR(devnum, S_IRUGO,
		get_devnum, NULL),
};

static int hsppad_create_sysfs(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto out_sysfs;
	return 0;

out_sysfs:
	for (; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "Unable to create interface\n");
	return -EIO;
}

static void hsppad_remove_sysfs(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
}


/*--------------------------------------------------------------------------
 * suspend/resume function
 *--------------------------------------------------------------------------*/

static int hsppad_pm_suspend(struct device *dev)
{
	struct hsppad_data *hsppad = dev_get_drvdata(dev);

	dev_err(&hsppad->i2c->dev, HSPPAD_LOG_TAG "%s\n", __func__);

	/*cancel_delayed_work(&hsppad->work_data);*/
	cancel_work_sync(&hsppad->work_data);
	mutex_lock(&hsppad->lock);
	hsppad->fsuspend = true;
	hsppad_measure_stop(hsppad);
	mutex_unlock(&hsppad->lock);
	return 0;
}

static int hsppad_pm_resume(struct device *dev)
{
	struct hsppad_data *hsppad = dev_get_drvdata(dev);

	dev_err(&hsppad->i2c->dev, HSPPAD_LOG_TAG "%s\n", __func__);

	mutex_lock(&hsppad->lock);
	if (hsppad->factive)
		hsppad_measure_start(hsppad);
	hsppad->fsuspend = false;
	mutex_unlock(&hsppad->lock);
	return 0;
}

static int hsppad_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct hsppad_data *hsppad = i2c_get_clientdata(client);

	dev_err(&client->adapter->dev, HSPPAD_LOG_TAG "%s\n", __func__);

	/*cancel_delayed_work(&hsppad->work_data);*/
	cancel_work_sync(&hsppad->work_data);
	mutex_lock(&hsppad->lock);
	hsppad->fsuspend = true;
	hsppad_measure_stop(hsppad);
	mutex_unlock(&hsppad->lock);
	return 0;
}

static int hsppad_resume(struct i2c_client *client)
{
	struct hsppad_data *hsppad = i2c_get_clientdata(client);

	dev_err(&client->adapter->dev, HSPPAD_LOG_TAG "%s\n", __func__);

	mutex_lock(&hsppad->lock);
	if (hsppad->factive)
		hsppad_measure_start(hsppad);
	hsppad->fsuspend = false;
	hsppad->flag_previous = false;
	mutex_unlock(&hsppad->lock);
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void hsppad_early_suspend(struct early_suspend *handler)
{
	struct hsppad_data *hsppad = container_of(handler,
		struct hsppad_data, early_suspend_h);
	hsppad_suspend(hsppad->i2c, PMSG_SUSPEND);
}

static void hsppad_early_resume(struct early_suspend *handler)
{
	struct hsppad_data *hsppad = container_of(handler,
		struct hsppad_data, early_suspend_h);
	hsppad_resume(hsppad->i2c);
}
#endif

static enum hrtimer_restart hsppad_timer_func(struct hrtimer *timer)
{
	struct hsppad_data *hsppad = container_of(timer,
		struct hsppad_data, poll_timer);

	queue_work(hsppad->work_queue, &hsppad->work_data);
	hrtimer_forward_now(&hsppad->poll_timer,
			ns_to_ktime(hsppad->delay_msec * 1000 * 1000));

	return HRTIMER_RESTART;
}

/*--------------------------------------------------------------------------
 * work function
 *--------------------------------------------------------------------------*/
static void hsppad_polling(struct work_struct *work)
{
	int pt[2];
	struct hsppad_data *hsppad = container_of(work,
		struct hsppad_data, work_data);

	mutex_lock(&hsppad->lock);
	if (hsppad->factive) {

		struct timespec ts;
		int64_t timestamp;

		get_monotonic_boottime(&ts);
		timestamp = timespec_to_ns(&ts);

		if (hsppad_get_pressure_data(hsppad, pt) == 0) {
			input_report_abs(hsppad->input, ABS_BRAKE, pt[0]);
			input_report_abs(hsppad->input, ABS_GAS, pt[1]);
			input_event(hsppad->input, INPUT_EVENT_TIME_TYPE, INPUT_EVENT_TIME_MSB,	timestamp >> 32);
			input_event(hsppad->input, INPUT_EVENT_TIME_TYPE, INPUT_EVENT_TIME_LSB,	timestamp & 0xffffffff);
			input_sync(hsppad->input);
		}
		hsppad_force_setup(hsppad);
	}
	mutex_unlock(&hsppad->lock);
}


/*--------------------------------------------------------------------------
 * i2c device
 *--------------------------------------------------------------------------*/
static int hsppad_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int rc;
	struct hsppad_data *hsppad;
	dev_dbg(&client->adapter->dev,
		HSPPAD_LOG_TAG "%s\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->adapter->dev, "client not i2c capable\n");
		rc = -ENODEV;
		goto out_region;
	}

	hsppad = kzalloc(sizeof(struct hsppad_data), GFP_KERNEL);
	if (!hsppad) {

		dev_err(&client->adapter->dev,
			"failed to allocate memory for module data\n");
		rc = -ENOMEM;
		goto out_region;
	}

	hsppad->i2c = client;
	i2c_set_clientdata(client, hsppad);

	mutex_init(&hsppad->lock);

	hsppad->delay_msec = HSPPAD_INITIALL_DELAY;

	hsppad->input = input_allocate_device();
	if (!hsppad->input) {
		rc = -ENOMEM;
		dev_err(&client->adapter->dev, "input_allocate_device\n");
		goto out_kzalloc;
	}

	input_set_drvdata(hsppad->input, hsppad);
	dev_dbg(&client->adapter->dev, "input_allocate_device\n");

	rc = hsppad_power_init(hsppad);
	if (rc) {
		dev_err(&client->dev, "Failed to init regulator\n");
		/*goto err_free_enable_gpio;*/
	}
	rc = hsppad_power_ctl(hsppad, true);
	if (rc) {
		dev_err(&client->dev, "Failed to power on device\n");
		goto err_deinit_regulator;
	}

	usleep(3000);

	hsppad->input->name		= HSPPAD_INPUT_DEVICE_NAME;
	hsppad->input->id.bustype	= BUS_I2C;
	hsppad->input->evbit[0]		= BIT_MASK(EV_ABS);
	input_set_abs_params(hsppad->input, ABS_BRAKE, 250, 1110, 0 , 0);
	input_set_abs_params(hsppad->input, ABS_GAS, -40, 85, 0 , 0);

	/* configure input device for timestamps */
	__set_bit(INPUT_EVENT_TIME_TYPE, hsppad->input->evbit);
	__set_bit(INPUT_EVENT_TIME_MSB, hsppad->input->mscbit);
	__set_bit(INPUT_EVENT_TIME_LSB, hsppad->input->mscbit);

	rc = input_register_device(hsppad->input);
	if (rc) {
		rc = -ENOMEM;
		dev_err(&client->adapter->dev, "input_register_device\n");
		goto out_idev_allc;
	}

	dev_dbg(&client->adapter->dev, "input_register_device\n");

	hrtimer_init(&hsppad->poll_timer, CLOCK_MONOTONIC,
			HRTIMER_MODE_REL);
	hsppad->poll_timer.function = hsppad_timer_func;

	hsppad->work_queue = alloc_workqueue("hsppad_poll_work",
				WQ_UNBOUND | WQ_MEM_RECLAIM | WQ_HIGHPRI, 1);
	/*hsppad->work_queue = create_singlethread_workqueue("hsppad_poll_work");*/
	INIT_WORK(&hsppad->work_data, hsppad_polling);

	rc = hsppad_create_sysfs(&hsppad->input->dev);
	if (rc) {
		rc = -ENOMEM;
		dev_err(&client->adapter->dev, "hsppad_create_sysfs\n");
		goto out_idev_reg;
	}

	dev_dbg(&client->adapter->dev, "hsppad_create_sysfs\n");

	pres_sysfs_link = kobject_create_and_add("pres", NULL);
	if (pres_sysfs_link != NULL)
		rc = sysfs_create_link(pres_sysfs_link, &hsppad->input->dev.kobj, "link");
	else
		rc = -ENODEV;

	if (rc < 0) {
			pr_err("%s: could not create sysfs link for pres\n", __func__);
			goto out_idev_reg;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	hsppad->early_suspend_h.suspend = hsppad_early_suspend;
	hsppad->early_suspend_h.resume  = hsppad_early_resume;
	register_early_suspend(&hsppad->early_suspend_h);
	dev_dbg(&client->adapter->dev, "register_early_suspend\n");
#endif

	hsppad->factive = false;
	hsppad->fsuspend = false;
	hsppad->flag_previous = false;
	dev_info(&client->adapter->dev,
		HSPPAD_LOG_TAG "detected %s pressure sensor\n",
		HSPPAD_DRIVER_NAME);

	return 0;

out_idev_reg:
	input_unregister_device(hsppad->input);
out_idev_allc:
	input_free_device(hsppad->input);
out_kzalloc:
	kfree(hsppad);
err_deinit_regulator:
	hsppad_power_deinit(hsppad);
out_region:

	return rc;
}

static int hsppad_remove(struct i2c_client *client)
{
	struct hsppad_data *hsppad = i2c_get_clientdata(client);

	dev_dbg(&client->adapter->dev, "%s\n", __func__);
	hsppad_measure_stop(hsppad);
	cancel_work_sync(&hsppad->work_data);
	/*cancel_delayed_work(&hsppad->work_data);*/
	destroy_workqueue(hsppad->work_queue);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&hsppad->early_suspend_h);
#endif
	hsppad_remove_sysfs(&hsppad->input->dev);
	input_unregister_device(hsppad->input);
	input_free_device(hsppad->input);
	kfree(hsppad);
	return 0;
}

static const struct dev_pm_ops hsppad_pm_ops = {
	.suspend	= hsppad_pm_suspend,
	.resume		= hsppad_pm_resume,
};

static const struct i2c_device_id hsppad_id[] = {
	{ HSPPAD_DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, hsppad_id);

static struct of_device_id hsppad038_of_match[] = {
		{ .compatible  = "alps,hsppad038",},
		{ },
};
MODULE_DEVICE_TABLE(of, hsppad038_of_match);

static struct i2c_driver hsppad_driver = {
	.probe		= hsppad_probe,
	.remove		= hsppad_remove,
	.id_table	= hsppad_id,
	.driver		= {
		.name = HSPPAD_DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = hsppad038_of_match,
		.pm		= &hsppad_pm_ops,
	},
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= hsppad_suspend,
	.resume		= hsppad_resume,
#endif
};

module_i2c_driver(hsppad_driver);



/*
static struct i2c_driver mpu6050_i2c_driver = {
	.driver	= {
		.name	= "mpu6050",
		.owner	= THIS_MODULE,
		.pm	= &mpu6050_pm,
		.of_match_table = mpu6050_of_match,
	},
	.probe		= mpu6050_probe,
	.remove		= mpu6050_remove,
	.id_table	= mpu6050_ids,
};

module_i2c_driver(mpu6050_i2c_driver);
*/

MODULE_DESCRIPTION("Alps Pressure Input Device");
MODULE_AUTHOR("ALPS ELECTRIC CO., LTD.");
MODULE_LICENSE("GPL v2");
