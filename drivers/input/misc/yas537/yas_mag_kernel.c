/*
 * Copyright (c) 2014-2015 Yamaha Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include "yas.h"
#include <linux/cei_hw_id.h>

// S- [PM99] Grace_Chang add for device tree
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
// E- [PM99] Grace_Chang add for device tree

// S- [PM99] Grace_Chang add for sensor power
#include <linux/regulator/consumer.h>

/* POWER SUPPLY VOLTAGE RANGE */
#define YAS537_VDD_MIN_UV	1700000
#define YAS537_VDD_MAX_UV	1900000
#define YAS537_VIO_MIN_UV	2000000
#define YAS537_VIO_MAX_UV	3300000

// E- [PM99] Grace_Chang add for sensor power

#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS532 \
		    || YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS533
#define YAS_MSM_NAME		"yas532_mag"
#endif
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS537
#define YAS_MSM_NAME		"yas537_mag"
#endif
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS539
#define YAS_MSM_NAME		"yas539_mag"
#endif

static struct i2c_client *this_client;
static struct regulator *vdd;
static struct regulator *vio;

struct yas_state {
	struct mutex lock;
	struct yas_mag_driver mag;
	struct input_dev *input_dev;
	struct work_struct	work_data;
	struct workqueue_struct	*work_queue;
	struct hrtimer poll_timer;
	int32_t delay;
	atomic_t enable;
	int32_t compass_data[3];
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend sus;
#endif
};

/* [PM99] S- Grace_Chang  Add new file path */
/* SYSFS symbolic link */
static struct kobject *mag_sysfs_link;
/* [PM99] E- Grace_Chang  Add new file path */


static int yas_power_on(struct i2c_client *client)
{

	int rc = 0;

	rc = regulator_enable(vdd);
		if (rc) {
			dev_err(&client->dev, "Regulator vdd enable failed rc=%d\n", rc);
			return rc;
		}

		rc = regulator_enable(vio);
		if (rc) {
			dev_err(&client->dev, "Regulator vio enable failed rc=%d\n", rc);
			regulator_disable(vdd);
			return rc;
		}
	return rc;

}

static int yas_power_init(struct i2c_client *client, bool on)
{
	int rc = 0;

	if (!on) {
		if (regulator_count_voltages(vdd) > 0)
			regulator_set_voltage(vdd, 0, YAS537_VDD_MAX_UV);

		regulator_put(vdd);

		if (regulator_count_voltages(vio) > 0)
			regulator_set_voltage(vio, 0, YAS537_VIO_MAX_UV);

		regulator_put(vio);
	} else {
		vdd = regulator_get(&client->dev, "vdd");
		if (IS_ERR(vdd)) {
			rc = PTR_ERR(vdd);
			dev_err(&client->dev,
				"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(vdd) > 0) {
			rc = regulator_set_voltage(vdd, YAS537_VDD_MIN_UV,
						   YAS537_VDD_MAX_UV);
			if (rc) {
				dev_err(&client->dev,
					"Regulator set failed vdd rc=%d\n",
					rc);
				goto reg_vdd_put;
			}
		}

		vio = regulator_get(&client->dev, "vio");
		if (IS_ERR(vio)) {
			rc = PTR_ERR(vio);
			dev_err(&client->dev,
				"Regulator get failed vio rc=%d\n", rc);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(vio) > 0) {
			rc = regulator_set_voltage(vio, YAS537_VIO_MIN_UV,
						   YAS537_VIO_MAX_UV);
			if (rc) {
				dev_err(&client->dev,
				"Regulator set failed vio rc=%d\n", rc);
				goto reg_vio_put;
			}
		}
	}

	return 0;

reg_vio_put:
	regulator_put(vio);
reg_vdd_set:
	if (regulator_count_voltages(vdd) > 0)
		regulator_set_voltage(vdd, 0, YAS537_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(vdd);
	return rc;
}

static int yas_device_open(int32_t type)
{
	return 0;
}

static int yas_device_close(int32_t type)
{
	return 0;
}

static int yas_device_write(int32_t type, uint8_t addr, const uint8_t *buf,
		int len)
{
	uint8_t tmp[2];
	if (sizeof(tmp) - 1 < len)
		return -1;
	tmp[0] = addr;
	memcpy(&tmp[1], buf, len);
	if (i2c_master_send(this_client, tmp, len + 1) < 0)
		return -1;
	return 0;
}

static int yas_device_read(int32_t type, uint8_t addr, uint8_t *buf, int len)
{
	struct i2c_msg msg[2];
	int err;
	msg[0].addr = this_client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &addr;
	msg[1].addr = this_client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = buf;
	err = i2c_transfer(this_client->adapter, msg, 2);
	if (err != 2) {
		dev_err(&this_client->dev,
				"i2c_transfer() read error: "
				"slave_addr=%02x, reg_addr=%02x, err=%d\n",
				this_client->addr, addr, err);
		return err;
	}
	return 0;
}

static void yas_usleep(int us)
{
	usleep_range(us, us + 1000);
}

static uint32_t yas_current_time(void)
{
	return jiffies_to_msecs(jiffies);
}

static int yas_enable(struct yas_state *st)
{
	if (!atomic_cmpxchg(&st->enable, 0, 1)) {
		mutex_lock(&st->lock);
		st->mag.set_enable(1);
		hrtimer_start(&st->poll_timer,
			ns_to_ktime(st->delay * 1000 * 1000),
			HRTIMER_MODE_REL);
		mutex_unlock(&st->lock);
		/*schedule_delayed_work(&st->work, 0);*/
	}
	return 0;
}

static int yas_disable(struct yas_state *st)
{
	if (atomic_cmpxchg(&st->enable, 1, 0)) {
		/*cancel_delayed_work_sync(&st->work);*/
		hrtimer_cancel(&st->poll_timer);
		cancel_work_sync(&st->work_data);
		mutex_lock(&st->lock);
		st->mag.set_enable(0);
		/*hrtimer_cancel(&st->poll_timer);*/
		mutex_unlock(&st->lock);
	}
	return 0;
}

/* Sysfs interface */
static ssize_t yas_full_scale_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0\n");
}

static ssize_t yas_full_scale_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static ssize_t yas_enable_device_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	return sprintf(buf, "%d\n", atomic_read(&st->enable));
}

static ssize_t yas_enable_device_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int enable;
	if (kstrtoint(buf, 10, &enable) < 0)
		return -EINVAL;
	if (enable)
		yas_enable(st);
	else
		yas_disable(st);
	return count;
}

static ssize_t yas_position_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int ret;
	mutex_lock(&st->lock);
	ret = st->mag.get_position();
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return sprintf(buf, "%d\n", ret);
}

static ssize_t yas_position_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int ret, position;
	sscanf(buf, "%d\n", &position);
	mutex_lock(&st->lock);
	ret = st->mag.set_position(position);
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return count;
}

static ssize_t yas_pollrate_ms_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int32_t delay;
	mutex_lock(&st->lock);
	delay = st->delay;
	mutex_unlock(&st->lock);
	return sprintf(buf, "%d\n", delay);
}

static ssize_t yas_pollrate_ms_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int delay;
	if (kstrtoint(buf, 10, &delay) < 0)
		return -EINVAL;
	if (delay <= 0)
		delay = 0;
	mutex_lock(&st->lock);
	if (st->mag.set_delay(delay) == YAS_NO_ERROR)
		st->delay = delay;
	mutex_unlock(&st->lock);
	return count;
}

#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS532 \
	|| YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS533 \
	|| YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS537
static ssize_t yas_hard_offset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int8_t hard_offset[3];
	int ret;
	mutex_lock(&st->lock);
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS532 \
		    || YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS533
	ret = st->mag.ext(YAS532_GET_HW_OFFSET, hard_offset);
#endif
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS537
	ret = st->mag.ext(YAS537_GET_HW_OFFSET, hard_offset);
#endif
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return sprintf(buf, "%d %d %d\n", hard_offset[0], hard_offset[1],
			hard_offset[2]);
}
#endif

#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS532 \
		    || YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS533
static ssize_t yas_hard_offset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int32_t tmp[3];
	int8_t hard_offset[3];
	int ret, i;
	sscanf(buf, "%d %d %d\n", &tmp[0], &tmp[1], &tmp[2]);
	for (i = 0; i < 3; i++)
		hard_offset[i] = (int8_t)tmp[i];
	mutex_lock(&st->lock);
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS532 \
		    || YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS533
	ret = st->mag.ext(YAS532_SET_HW_OFFSET, hard_offset);
#endif
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return count;
}
#endif

static ssize_t yas_static_matrix_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int16_t m[9];
	int ret;
	mutex_lock(&st->lock);
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS532 \
		    || YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS533
	ret = st->mag.ext(YAS532_GET_STATIC_MATRIX, m);
#endif
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS537
	ret = st->mag.ext(YAS537_GET_STATIC_MATRIX, m);
#endif
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS539
	ret = st->mag.ext(YAS539_GET_STATIC_MATRIX, m);
#endif
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return sprintf(buf, "%d %d %d %d %d %d %d %d %d\n", m[0], m[1], m[2],
			m[3], m[4], m[5], m[6], m[7], m[8]);
}

static ssize_t yas_static_matrix_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int16_t m[9];
	int ret;
	sscanf(buf, "%hd %hd %hd %hd %hd %hd %hd %hd %hd\n", &m[0], &m[1],
			&m[2], &m[3], &m[4], &m[5], &m[6], &m[7], &m[8]);
	mutex_lock(&st->lock);
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS532 \
		    || YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS533
	ret = st->mag.ext(YAS532_SET_STATIC_MATRIX, m);
#endif
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS537
	ret = st->mag.ext(YAS537_SET_STATIC_MATRIX, m);
#endif
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS539
	ret = st->mag.ext(YAS539_SET_STATIC_MATRIX, m);
#endif
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return count;
}

/* [PM99] S- BUG#19 Grace_Chang G+Gyro & M sensor FTM function */
#define CHECK_RANGE(X, MIN, MAX)    (X>=MIN && X<=MAX)?1:0
#define CHECK_GREATER(X, MIN)    (X>=MIN)?1:0
#define YAS537_DEVICE_ID 0x07
int yas537_self_test_check_result(int ret, int id, int dir, int sx, int sy, int ohx, int ohy, int ohz)
{
	if(ret != YAS_NO_ERROR)
	    return 0;
	if(id != YAS537_DEVICE_ID)
	    return 0;
	if(!CHECK_RANGE(dir, 0, 359))
	    return 0;
	if(!CHECK_GREATER(sx,24))
	    return 0;
	if(!CHECK_GREATER(sy,31))
	    return 0;
	//if(!CHECK_RANGE(ohx, -1000, 1000))
	//    return 0;
	//if(!CHECK_RANGE(ohy, -1000, 1000))
	//    return 0;
	//if(!CHECK_RANGE(ohz, -1000, 1000))
	//    return 0;

	return 1;
}
/* [PM99] E- BUG#19 Grace_Chang G+Gyro & M sensor FTM function */

static ssize_t yas_self_test_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS532 \
		    || YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS533
	struct yas532_self_test_result r;
#endif
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS537
	struct yas537_self_test_result r;
#endif
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS539
	struct yas539_self_test_result r;
#endif
	int ret;
	/* [PM99] S- BUG#19 Grace_Chang G+Gyro & M sensor FTM function */
	int result;
	/* [PM99] E- BUG#19 Grace_Chang G+Gyro & M sensor FTM function */

	mutex_lock(&st->lock);
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS532 \
		    || YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS533
	ret = st->mag.ext(YAS532_SELF_TEST, &r);
#endif
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS537
	ret = st->mag.ext(YAS537_SELF_TEST, &r);
#endif
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS539
	ret = st->mag.ext(YAS539_SELF_TEST, &r);
#endif
	mutex_unlock(&st->lock);

#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS532 \
		    || YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS533
	return sprintf(buf, "%d %d %d %d %d %d %d %d %d %d %d\n", ret, r.id,
			r.xy1y2[0], r.xy1y2[1], r.xy1y2[2], r.dir, r.sx, r.sy,
			r.xyz[0], r.xyz[1], r.xyz[2]);
#endif
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS537
/* [PM99] S- BUG#19 Grace_Chang G+Gyro & M sensor FTM function */
#ifdef ORG_VER
	return sprintf(buf, "%d %d %d %d %d %d %d %d\n", ret, r.id, r.dir,
			r.sx, r.sy, r.xyz[0], r.xyz[1], r.xyz[2]);
#else
	result = yas537_self_test_check_result(ret, r.id, r.dir, r.sx, r.sy, r.xyz[0], r.xyz[1], r.xyz[2]);
	return sprintf(buf, "ret=%d, id=%d, dir=%d, sx=%d, sy=%d, ohxyz=%d %d %d\nself_test: %s\n",
			ret, r.id, r.dir, r.sx, r.sy, r.xyz[0], r.xyz[1], r.xyz[2], (result)?"Pass":"Fail");
#endif
/* [PM99] E- BUG#19 Grace_Chang G+Gyro & M sensor FTM function */
#endif
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS539
	return sprintf(buf, "%d %d %d %d %d %d %d %d %d\n", ret, r.id, r.dir,
			r.sxy1y2[0], r.sxy1y2[1], r.sxy1y2[2], r.xyz[0],
			r.xyz[1], r.xyz[2]);
#endif
}

static ssize_t yas_self_test_noise_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int32_t xyz_raw[3];
	int ret;
	mutex_lock(&st->lock);
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS532 \
		    || YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS533
	ret = st->mag.ext(YAS532_SELF_TEST_NOISE, xyz_raw);
#endif
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS537
	ret = st->mag.ext(YAS537_SELF_TEST_NOISE, xyz_raw);
#endif
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS539
	ret = st->mag.ext(YAS539_SELF_TEST_NOISE, xyz_raw);
#endif
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return sprintf(buf, "%d %d %d\n", xyz_raw[0], xyz_raw[1], xyz_raw[2]);
}

#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS537 \
	|| YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS539
static ssize_t yas_mag_average_sample_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int8_t mag_average_sample;
	int ret;
	mutex_lock(&st->lock);
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS537
	ret = st->mag.ext(YAS537_GET_AVERAGE_SAMPLE, &mag_average_sample);
#endif
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS539
	ret = st->mag.ext(YAS539_GET_AVERAGE_SAMPLE, &mag_average_sample);
#endif
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return sprintf(buf, "%d\n", mag_average_sample);
}

static ssize_t yas_mag_average_sample_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int32_t tmp;
	int8_t mag_average_sample;
	int ret;
	sscanf(buf, "%d\n", &tmp);
	mag_average_sample = (int8_t)tmp;
	mutex_lock(&st->lock);
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS537
	ret = st->mag.ext(YAS537_SET_AVERAGE_SAMPLE, &mag_average_sample);
#endif
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS539
	ret = st->mag.ext(YAS539_SET_AVERAGE_SAMPLE, &mag_average_sample);
#endif
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return count;
}

static ssize_t yas_ouflow_thresh_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int16_t thresh[6];
	int ret;
	mutex_lock(&st->lock);
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS537
	ret = st->mag.ext(YAS537_GET_OUFLOW_THRESH, thresh);
#endif
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS539
	ret = st->mag.ext(YAS539_GET_OUFLOW_THRESH, thresh);
#endif
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return sprintf(buf, "%d %d %d %d %d %d\n", thresh[0], thresh[1],
			thresh[2], thresh[3], thresh[4], thresh[5]);
}
#endif

/* [PM99] S- BUG#19 Grace_Chang G+Gyro & M sensor FTM function */
#define YAS537_REG_DIDR			(0x80)
static ssize_t yas_ping(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	uint8_t data;
	yas_device_read(YAS_TYPE_MAG, YAS537_REG_DIDR, &data, 1);
	return sprintf(buf, "0x2e:0x%02x\n", data);
}
/* [PM99] E- BUG#19 Grace_Chang G+Gyro & M sensor FTM function */

static ssize_t yas_data_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int32_t last[3], i;
	mutex_lock(&st->lock);
	for (i = 0; i < 3; i++)
		last[i] = st->compass_data[i];
	mutex_unlock(&st->lock);
	return sprintf(buf, "%d %d %d\n", last[0], last[1], last[2]);
}

static ssize_t get_devnum(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int devnum, ret;
	const char *devname = NULL;

	devname = dev_name(&st->input_dev->dev);
	ret = sscanf(devname+5, "%d", &devnum);
	if (!ret)
		pr_err("[Sensor] get device number failed");

	return snprintf(buf, PAGE_SIZE, "%d\n", devnum);
}

static DEVICE_ATTR(full_scale, S_IRUGO|S_IWUSR|S_IWGRP, yas_full_scale_show,
		yas_full_scale_store);
static DEVICE_ATTR(pollrate_ms, S_IRUGO|S_IWUSR|S_IWGRP, yas_pollrate_ms_show,
		yas_pollrate_ms_store);
static DEVICE_ATTR(enable_device, S_IRUGO|S_IWUSR|S_IWGRP, yas_enable_device_show,
		yas_enable_device_store);
static DEVICE_ATTR(data, S_IRUGO, yas_data_show, NULL);
static DEVICE_ATTR(position, S_IRUSR|S_IWUSR, yas_position_show,
		yas_position_store);
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS532 \
		    || YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS533
static DEVICE_ATTR(hard_offset, S_IRUSR|S_IWUSR, yas_hard_offset_show,
		yas_hard_offset_store);
#endif
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS537
static DEVICE_ATTR(hard_offset, S_IRUSR, yas_hard_offset_show, NULL);
#endif
static DEVICE_ATTR(static_matrix, S_IRUSR|S_IWUSR,
		yas_static_matrix_show, yas_static_matrix_store);
static DEVICE_ATTR(self_test, S_IRUSR, yas_self_test_show, NULL);
static DEVICE_ATTR(self_test_noise, S_IRUSR, yas_self_test_noise_show, NULL);
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS537 \
	|| YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS539
static DEVICE_ATTR(mag_average_sample, S_IRUSR|S_IWUSR,
		yas_mag_average_sample_show, yas_mag_average_sample_store);
static DEVICE_ATTR(ouflow_thresh, S_IRUSR, yas_ouflow_thresh_show, NULL);
#endif
/* [PM99] S- BUG#19 Grace_Chang G+Gyro & M sensor FTM function */
static DEVICE_ATTR(ping, S_IRUGO|S_IWUSR|S_IWGRP, yas_ping, NULL);
/* [PM99] E- BUG#19 Grace_Chang G+Gyro & M sensor FTM function */
static DEVICE_ATTR(devnum, S_IWUSR | S_IRUGO, get_devnum, NULL);

static struct attribute *yas_attributes[] = {
	&dev_attr_pollrate_ms.attr,
	&dev_attr_full_scale.attr,
	&dev_attr_enable_device.attr,
	&dev_attr_data.attr,
	&dev_attr_position.attr,
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS532 \
	|| YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS533 \
	|| YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS537
	&dev_attr_hard_offset.attr,
#endif
	&dev_attr_static_matrix.attr,
	&dev_attr_self_test.attr,
	&dev_attr_self_test_noise.attr,
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS537 \
	|| YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS539
	&dev_attr_mag_average_sample.attr,
	&dev_attr_ouflow_thresh.attr,
#endif
	/* [PM99] S- BUG#19 Grace_Chang G+Gyro & M sensor FTM function */
	&dev_attr_ping.attr,
	&dev_attr_devnum.attr,
	/* [PM99] E- BUG#19 Grace_Chang G+Gyro & M sensor FTM function */
	NULL
};
static struct attribute_group yas_attribute_group = {
	.attrs = yas_attributes
};

static enum hrtimer_restart yas_timer_func(struct hrtimer *timer)
{
	struct yas_state *st = container_of(timer,
		struct yas_state, poll_timer);

	queue_work(st->work_queue, &st->work_data);
	hrtimer_forward_now(&st->poll_timer,
			ns_to_ktime(st->delay * 1000 * 1000));

	return HRTIMER_RESTART;
}


static void yas_work_func(struct work_struct *work)
{
	struct yas_state *st
		= container_of(work,
			struct yas_state, work_data);
	struct yas_data mag[1];
	/*int32_t delay;
	uint32_t time_before, time_after;
	*/
	int ret, i;
	struct timespec ts;
	int64_t timestamp;

	get_monotonic_boottime(&ts);
	timestamp = timespec_to_ns(&ts);

	/*time_before = yas_current_time();*/
	mutex_lock(&st->lock);
	ret = st->mag.measure(mag, 1);
	if (ret == 1) {
		for (i = 0; i < 3; i++)
			st->compass_data[i] = mag[0].xyz.v[i];
	}
	/*delay = st->delay;*/
	mutex_unlock(&st->lock);
	if (ret == 1) {
		/* report magnetic data in [nT] */
		input_report_abs(st->input_dev, ABS_X, mag[0].xyz.v[0]);
		input_report_abs(st->input_dev, ABS_Y, mag[0].xyz.v[1]);
		input_report_abs(st->input_dev, ABS_Z, mag[0].xyz.v[2]);
		input_event(st->input_dev, EV_MSC, MSC_SCAN,	timestamp >> 32);
		input_event(st->input_dev, EV_MSC, MSC_MAX,	timestamp & 0xffffffff);
		input_sync(st->input_dev);
	}
	/*time_after = yas_current_time();
	delay = delay - (time_after - time_before);
	if (delay <= 0)
		delay = 1;
	schedule_delayed_work(&st->work, msecs_to_jiffies(delay));
	*/
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void yas_early_suspend(struct early_suspend *h)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	if (atomic_read(&st->enable)) {
		/*cancel_delayed_work_sync(&st->work);*/
		cancel_work_sync(&st->work_data);
		st->mag.set_enable(0);
	}
}

static void yas_late_resume(struct early_suspend *h)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	if (atomic_read(&st->enable)) {
		st->mag.set_enable(1);
		/*schedule_delayed_work(&st->work, 0);*/
	}
}
#endif

static int yas_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct yas_state *st = NULL;
	struct input_dev *input_dev = NULL;
	int ret, i;
	int rc = 0; 
	int hwid;

	pr_info("[Sensor] %s , enter", __FUNCTION__ );

	hwid = get_cei_hw_id();

	if (hwid < DVT2) {
		pr_err("[Sensor] %s , yas driver not for DVT1 & EVT", __func__);
		return -EFAULT;
	}
	this_client = i2c;
	input_dev = input_allocate_device();
	if (input_dev == NULL) {
		ret = -ENOMEM;
		goto error_free;
	}
	st = kzalloc(sizeof(struct yas_state), GFP_KERNEL);
	if (st == NULL) {
		ret = -ENOMEM;
		goto error_ret;
	}
	i2c_set_clientdata(i2c, st);

	// S- [PM99] Grace_Chang add for sensor power
	/* Set I2C power */
	//yas_power_set(i2c, 1);
	rc = yas_power_init(i2c,1);
	// E- [PM99] Grace_Chang add for sensor power
	if(!rc)
		yas_power_on(i2c);
	else{
		pr_err("[Sensor] %s , yas_power_init fail", __func__);
		return -EFAULT;
	}

	msleep(80);

	input_dev->name = YAS_MSM_NAME;
	input_dev->dev.parent = &i2c->dev;
	input_dev->id.bustype = BUS_I2C;
	set_bit(EV_ABS, input_dev->evbit);
	/* configure input device for timestamps */
	__set_bit(EV_MSC, input_dev->evbit);
	__set_bit(MSC_SCAN, input_dev->mscbit);
	__set_bit(MSC_MAX, input_dev->mscbit);
	input_set_abs_params(input_dev, ABS_X, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_Z, INT_MIN, INT_MAX, 0, 0);

	input_set_drvdata(input_dev, st);
	atomic_set(&st->enable, 0);
	st->input_dev = input_dev;
	st->delay = YAS_DEFAULT_SENSOR_DELAY;
	st->mag.callback.device_open = yas_device_open;
	st->mag.callback.device_close = yas_device_close;
	st->mag.callback.device_write = yas_device_write;
	st->mag.callback.device_read = yas_device_read;
	st->mag.callback.usleep = yas_usleep;
	st->mag.callback.current_time = yas_current_time;
	/*INIT_DELAYED_WORK(&st->work, yas_work_func);*/
	hrtimer_init(&st->poll_timer, CLOCK_MONOTONIC,
				HRTIMER_MODE_REL);
		st->poll_timer.function = yas_timer_func;

	st->work_queue = alloc_workqueue("yas_work_func",
				WQ_UNBOUND | WQ_MEM_RECLAIM | WQ_HIGHPRI, 1);
	INIT_WORK(&st->work_data, yas_work_func);

	mutex_init(&st->lock);
#ifdef CONFIG_HAS_EARLYSUSPEND
	st->sus.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	st->sus.suspend = yas_early_suspend;
	st->sus.resume = yas_late_resume;
	register_early_suspend(&st->sus);
#endif
	for (i = 0; i < 3; i++)
		st->compass_data[i] = 0;

	ret = input_register_device(input_dev);
	if (ret)
		goto error_free_device;

	ret = sysfs_create_group(&st->input_dev->dev.kobj,
				 &yas_attribute_group);
	if (ret)
		goto error_unregister_data;

	/* [PM99] S- Grace_Chang  Add new file path */
	/* create sysfs link for yas537 */
	mag_sysfs_link = kobject_create_and_add("mag", NULL);
	if (mag_sysfs_link != NULL) {
		ret = sysfs_create_link(mag_sysfs_link, &st->input_dev->dev.kobj, "link");
	} else {
		ret = -ENODEV;
	}
	if (ret < 0)
		return ret;
	/* [PM99] E- Grace_Chang  Add new file path */

	ret = yas_mag_driver_init(&st->mag);
	if (ret < 0) {
		ret = -EFAULT;
		goto error_remove_sysfs;
	}
	ret = st->mag.init();
	if (ret < 0) {
		ret = -EFAULT;
		goto error_remove_sysfs;
	}
	pr_info("[Sensor] %s , probe success, exit", __FUNCTION__ );
	return 0;

error_remove_sysfs:
	sysfs_remove_group(&st->input_dev->dev.kobj, &yas_attribute_group);
error_unregister_data:
	input_unregister_device(input_dev);
error_free_device:
	input_free_device(input_dev);
error_free:
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&st->sus);
#endif
	kfree(st);
error_ret:
	i2c_set_clientdata(i2c, NULL);
	this_client = NULL;
	pr_info("[Sensor] %s , probe fail, exit", __FUNCTION__ );
	return ret;
}

static int yas_remove(struct i2c_client *i2c)
{
	struct yas_state *st = i2c_get_clientdata(i2c);
	if (st != NULL) {
#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&st->sus);
#endif
		yas_disable(st);
		st->mag.term();
		cancel_work_sync(&st->work_data);
		destroy_workqueue(st->work_queue);
		sysfs_remove_group(&st->input_dev->dev.kobj,
				&yas_attribute_group);
		input_unregister_device(st->input_dev);
		input_free_device(st->input_dev);
		kfree(st);
		this_client = NULL;
	}
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int yas_suspend(struct device *dev)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	if (atomic_read(&st->enable)) {
		/*cancel_delayed_work_sync(&st->work);
		st->mag.set_enable(0);
		*/
		/*cancel_delayed_work_sync(&st->work);*/
		hrtimer_cancel(&st->poll_timer);
		cancel_work_sync(&st->work_data);
		st->mag.set_enable(0);
	}
	return 0;
}

static int yas_resume(struct device *dev)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	if (atomic_read(&st->enable)) {
		/*st->mag.set_enable(1);
		schedule_delayed_work(&st->work, 0);
		*/
		st->mag.set_enable(1);
		hrtimer_start(&st->poll_timer,
			ns_to_ktime(st->delay * 1000 * 1000),
			HRTIMER_MODE_REL);
	}
	return 0;
}

static SIMPLE_DEV_PM_OPS(yas_pm_ops, yas_suspend, yas_resume);
#define YAS_PM_OPS (&yas_pm_ops)
#else
#define YAS_PM_OPS NULL
#endif

static const struct i2c_device_id yas_id[] = {
	{YAS_MSM_NAME, 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, yas_id);

// S- [PM99] Grace_Chang add for device tree
static struct of_device_id yas537_of_match[] = {
		{ .compatible  = "yas,yas537",},
		{ },
};
MODULE_DEVICE_TABLE(of, yas533_of_match);
// E- [PM99] Grace_Chang add for device tree

static struct i2c_driver yas_driver = {
	.driver = {
		.name	= YAS_MSM_NAME,
		.owner	= THIS_MODULE,
		.pm	= YAS_PM_OPS,
		.of_match_table = yas537_of_match,	// [PM99] Grace_Chang add for device tree
	},
	.probe		= yas_probe,
	.remove		= yas_remove,
	.id_table	= yas_id,
};
static int __init yas_driver_init(void)
{
	return i2c_add_driver(&yas_driver);
}

static void __exit yas_driver_exit(void)
{
	i2c_del_driver(&yas_driver);
}

module_init(yas_driver_init);
module_exit(yas_driver_exit);

MODULE_DESCRIPTION("Yamaha Magnetometer I2C driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.9.0.1100");
