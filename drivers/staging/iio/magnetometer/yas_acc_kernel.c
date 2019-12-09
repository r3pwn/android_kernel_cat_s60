/*
 * Copyright (c) 2013-2015 Yamaha Corporation
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
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include "yas.h"
#include <linux/regulator/consumer.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/cei_hw_id.h>

/* POWER SUPPLY VOLTAGE RANGE */
#define STK831X_VDD_MIN_UV	1750000
#define STK831X_VDD_MAX_UV	1950000
#define STK831X_VIO_MIN_UV	2000000
#define STK831X_VIO_MAX_UV	3300000

static struct i2c_client *this_client;

enum {
	YAS_SCAN_ACCEL_X,
	YAS_SCAN_ACCEL_Y,
	YAS_SCAN_ACCEL_Z,
	YAS_SCAN_TIMESTAMP,
};
struct yas_state {
	struct mutex lock;
	spinlock_t spin_lock;
	struct yas_acc_driver acc;
	struct i2c_client *client;
	struct iio_trigger  *trig;
	struct delayed_work work;
	struct regulator *vdd;
	struct regulator *vio;
	int16_t sampling_frequency;
	atomic_t pseudo_irq_enable;
	int32_t accel_data[3];
	int32_t calib_bias[3];
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend sus;
#endif
};


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
#if 0
static int STK_i2c_Rx(char *rxData, int length)
{
	uint8_t retry;
	struct i2c_msg msgs[] = {
	
		{
			.addr = this_client->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = this_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	for (retry = 0; retry <= 3; retry++)
	{
		if (i2c_transfer(this_client->adapter, msgs, 2) > 0)
			break;
		else
			mdelay(10);
	}

	if (retry > 3)
	{
		printk(KERN_ERR "%s: i2c error, retry over 3\n", __func__);
		return -EIO;
	} else
		return 0;
}

#endif
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

static int yas_pseudo_irq_enable(struct iio_dev *indio_dev)
{
	struct yas_state *st = iio_priv(indio_dev);

	if (!atomic_cmpxchg(&st->pseudo_irq_enable, 0, 1)) {
		mutex_lock(&st->lock);
		st->acc.set_enable(1);
		mutex_unlock(&st->lock);
		schedule_delayed_work(&st->work, 150);
	}
	return 0;
}

static int yas_pseudo_irq_disable(struct iio_dev *indio_dev)
{
	struct yas_state *st = iio_priv(indio_dev);

	if (atomic_cmpxchg(&st->pseudo_irq_enable, 1, 0)) {
		cancel_delayed_work_sync(&st->work);
		mutex_lock(&st->lock);
		st->acc.set_enable(0);
		mutex_unlock(&st->lock);
	}
	return 0;
}

static int yas_set_pseudo_irq(struct iio_dev *indio_dev, int enable)
{
	if (enable)
		yas_pseudo_irq_enable(indio_dev);
	else
		yas_pseudo_irq_disable(indio_dev);
	return 0;
}

static int yas_data_rdy_trig_poll(struct iio_dev *indio_dev)
{
	struct yas_state *st = iio_priv(indio_dev);
	unsigned long flags;

	spin_lock_irqsave(&st->spin_lock, flags);
	iio_trigger_poll(st->trig, iio_get_time_ns());
	spin_unlock_irqrestore(&st->spin_lock, flags);
	return 0;
}

static irqreturn_t yas_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct yas_state *st = iio_priv(indio_dev);
	int len = 0, i, j;
	int32_t *acc;

	acc = (int32_t *) kmalloc(indio_dev->scan_bytes, GFP_KERNEL);
	if (acc == NULL)
		goto done;
	if (!bitmap_empty(indio_dev->active_scan_mask, indio_dev->masklength)) {
		j = 0;
		for (i = 0; i < 3; i++) {
			if (test_bit(i, indio_dev->active_scan_mask)) {
				acc[j] = st->accel_data[i];
				j++;
			}
		}
		len = j * 4;
	}

	/* Guaranteed to be aligned with 8 byte boundary */
	if (indio_dev->scan_timestamp)
		*(s64 *)((u8 *)acc + ALIGN(len, sizeof(s64))) = pf->timestamp;
	iio_push_to_buffers(indio_dev, (u8 *)acc);
	kfree(acc);
done:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

static int yas_data_rdy_trigger_set_state(struct iio_trigger *trig,
		bool state)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);

	yas_set_pseudo_irq(indio_dev, state);
	return 0;
}

static const struct iio_trigger_ops yas_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = &yas_data_rdy_trigger_set_state,
};

static int yas_probe_trigger(struct iio_dev *indio_dev)
{
	int ret;
	struct yas_state *st = iio_priv(indio_dev);

	indio_dev->pollfunc = iio_alloc_pollfunc(&iio_pollfunc_store_time,
			&yas_trigger_handler, IRQF_ONESHOT, indio_dev,
			"%s_consumer%d", indio_dev->name, indio_dev->id);
	if (indio_dev->pollfunc == NULL) {
		ret = -ENOMEM;
		goto error_ret;
	}
	st->trig = iio_trigger_alloc("%s-dev%d",
			indio_dev->name,
			indio_dev->id);
	if (!st->trig) {
		ret = -ENOMEM;
		goto error_dealloc_pollfunc;
	}
	st->trig->dev.parent = &st->client->dev;
	st->trig->ops = &yas_trigger_ops;
	iio_trigger_set_drvdata(st->trig, indio_dev);
	ret = iio_trigger_register(st->trig);
	if (ret)
		goto error_free_trig;
	return 0;

error_free_trig:
	iio_trigger_free(st->trig);
error_dealloc_pollfunc:
	iio_dealloc_pollfunc(indio_dev->pollfunc);
error_ret:
	return ret;
}

static void yas_remove_trigger(struct iio_dev *indio_dev)
{
	struct yas_state *st = iio_priv(indio_dev);

	iio_trigger_unregister(st->trig);
	iio_trigger_free(st->trig);
	iio_dealloc_pollfunc(indio_dev->pollfunc);
}

static const struct iio_buffer_setup_ops yas_buffer_setup_ops = {
	.preenable = &iio_sw_buffer_preenable,
	.postenable = &iio_triggered_buffer_postenable,
	.predisable = &iio_triggered_buffer_predisable,
};

static void yas_remove_buffer(struct iio_dev *indio_dev)
{
	iio_buffer_unregister(indio_dev);
	iio_kfifo_free(indio_dev->buffer);
};

static int yas_probe_buffer(struct iio_dev *indio_dev)
{
	int ret;
	struct iio_buffer *buffer;

	buffer = iio_kfifo_allocate(indio_dev);
	if (!buffer) {
		ret = -ENOMEM;
		goto error_ret;
	}
	buffer->scan_timestamp = true;
	indio_dev->buffer = buffer;
	indio_dev->setup_ops = &yas_buffer_setup_ops;
	indio_dev->modes |= INDIO_BUFFER_TRIGGERED;
	ret = iio_buffer_register(indio_dev, indio_dev->channels,
			indio_dev->num_channels);
	if (ret)
		goto error_free_buf;
	iio_scan_mask_set(indio_dev, indio_dev->buffer, YAS_SCAN_ACCEL_X);
	iio_scan_mask_set(indio_dev, indio_dev->buffer, YAS_SCAN_ACCEL_Y);
	iio_scan_mask_set(indio_dev, indio_dev->buffer, YAS_SCAN_ACCEL_Z);
	return 0;

error_free_buf:
	iio_kfifo_free(indio_dev->buffer);
error_ret:
	return ret;
}

static ssize_t yas_position_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct yas_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&st->lock);
	ret = st->acc.get_position();
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return sprintf(buf, "%d\n", ret);
}

static ssize_t yas_position_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct yas_state *st = iio_priv(indio_dev);
	int ret, position;

	sscanf(buf, "%d\n", &position);
	mutex_lock(&st->lock);
	ret = st->acc.set_position(position);
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return count;
}

static ssize_t yas_sampling_frequency_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct yas_state *st = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", st->sampling_frequency);
}

static ssize_t yas_sampling_frequency_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct yas_state *st = iio_priv(indio_dev);
	int ret, data, delay;

	ret = kstrtoint(buf, 10, &data);
	if (ret)
		return ret;
	if (data <= 0)
		return -EINVAL;
	mutex_lock(&st->lock);
	st->sampling_frequency = data;
	delay = MSEC_PER_SEC / st->sampling_frequency;
	st->acc.set_delay(delay);
	mutex_unlock(&st->lock);
	return count;
}

static ssize_t yas_ping_show(struct device *dev,
	       struct device_attribute *attr, char *buf)
{
#if 0

	int result;
	char buffer[2] = "";

	buffer[0] = 0x12;
	result = STK_i2c_Rx(buffer, 2);
	if (result < 0)
	{
	       printk(KERN_ERR "%s:failed\n", __func__);
	       return sprintf(buf, "%s\n", "0xFF");
	}
	/* stk8313 chip ID : 0x3C */
	return sprintf(buf, "0x%x\n", buffer[0]);
#endif
#if 1
	/* uint8_t *value = ""; */
	uint8_t REG_PLAT = 0x12;
	char value[2] = "";

	value[0] = 0x12;
	yas_device_read(YAS_TYPE_ACC, REG_PLAT, value, 2);
	return sprintf(buf, "0x%x\n", (int)*value);
#endif

}

#define STK_K_SUCCESS_TUNE			0x04
#define STK_K_SUCCESS_FT2			0x03
#define STK_K_SUCCESS_FT1			0x02
#define STK_K_SUCCESS_FILE			0x01
#define STK_K_NO_CALI				0xFF
#define STK_K_RUNNING				0xFE
#define STK_K_FAIL_LRG_DIFF			0xFD
#define STK_K_FAIL_OPEN_FILE			0xFC
#define STK_K_FAIL_W_FILE				0xFB
#define STK_K_FAIL_R_BACK				0xFA
#define STK_K_FAIL_R_BACK_COMP		0xF9
#define STK_K_FAIL_I2C				0xF8
#define STK_K_FAIL_K_PARA				0xF7
#define STK_K_FAIL_OUT_RG			0xF6
#define STK_K_FAIL_ENG_I2C			0xF5
#define STK_K_FAIL_FT1_USD			0xF4
#define STK_K_FAIL_FT2_USD			0xF3
#define STK_K_FAIL_WRITE_NOFST		0xF2
#define STK_K_FAIL_OTP_5T				0xF1
#define STK_K_FAIL_PLACEMENT			0xF0
#define YAS_ODR_400HZ			(0x00)
#define YAS_ODR_200HZ			(0x01)
#define YAS_ODR_100HZ			(0x02)
#define YAS_ODR_50HZ			(0x03)
#define YAS_ODR_25HZ			(0x04)
#define YAS_ODR_12_5HZ			(0x05)
#define YAS_ODR_6_25HZ			(0x06)
#define YAS_ODR_3_125HZ			(0x07)
#define STK_SAMPLE_CAL			50
#define STK_SAMPLE_ALL			(STK_SAMPLE_CAL+10)
#define STK_ACC_CALI_VER0			0x3D
#define STK_ACC_CALI_VER1			0x02

struct yas_acc_odr {
	int delay;
	uint8_t odr;
};

static const struct yas_acc_odr yas_odr_tbl[] = {
	{3,	YAS_ODR_400HZ},
	{5,	YAS_ODR_200HZ},
	{10,	YAS_ODR_100HZ},
	{20,	YAS_ODR_50HZ},
	{40,	YAS_ODR_25HZ},
	{80,	YAS_ODR_12_5HZ},
	{160,	YAS_ODR_6_25HZ},
	{320,	YAS_ODR_3_125HZ},
};

extern int yas_read_reg(uint8_t adr, uint8_t *val);
extern int yas_write_reg(uint8_t adr, uint8_t val);

#define STK_ACC_CALI_FILE			"/persist/stkacccali.conf"
#define STK_ACC_CALI_FILE_SIZE		10

static int32_t stk_get_file_content(char *r_buf, int8_t buf_size)
{
	struct file  *cali_file;
	mm_segment_t fs;
	ssize_t ret;

    cali_file = filp_open(STK_ACC_CALI_FILE, O_RDONLY, 0);
    if (IS_ERR(cali_file))
	{
	printk(KERN_ERR "%s: filp_open error, no offset file!\n", __func__);
	return -ENOENT;
	} else
	{
		fs = get_fs();
		set_fs(get_ds());
		ret = cali_file->f_op->read(cali_file, r_buf, STK_ACC_CALI_FILE_SIZE, &cali_file->f_pos);
		if (ret < 0)
		{
			printk(KERN_ERR "%s: read error, ret=%zd\n", __func__, ret);
			filp_close(cali_file, NULL);
			return -EIO;
		}
		set_fs(fs);
    }

    filp_close(cali_file, NULL);	
	return 0;
}

static int stk_store_in_file(struct yas_state *stk, int offset[], char mode)
{
	struct file  *cali_file;
	char r_buf[STK_ACC_CALI_FILE_SIZE] = {0};
	char w_buf[STK_ACC_CALI_FILE_SIZE] = {0};
	int int_to_adc[3] = {0};

	mm_segment_t fs;
	ssize_t ret;
	int8_t i;

	if (offset[0] >= 0)
		int_to_adc[0] = offset[0]<<4;
	else
		int_to_adc[0] = (offset[0]+4096)<<4;
	if (offset[1] >= 0)
		int_to_adc[1] = offset[1]<<4;
	else
		int_to_adc[1] = (offset[1]+4096)<<4;
	if (offset[2] >= 0)
		int_to_adc[2] = offset[2]<<4;
	else
		int_to_adc[2] = (offset[2]+4096)<<4;

		w_buf[0] = 0x3D/*STK_ACC_CALI_VER0*/;
		w_buf[1] = 0x02/*STK_ACC_CALI_VER1*/;
		w_buf[2] = (char)((int_to_adc[0] & 0xFF00)>>8);
		w_buf[3] = (char) (int_to_adc[0] & 0x00FF);
		w_buf[4] = (char)((int_to_adc[1] & 0xFF00)>>8);
		w_buf[5] = (char) (int_to_adc[1] & 0x00FF);
		w_buf[6] = (char)((int_to_adc[2] & 0xFF00)>>8);
		w_buf[7] = (char) (int_to_adc[2] & 0x00FF);
		w_buf[8] = mode;

    cali_file = filp_open(STK_ACC_CALI_FILE, O_CREAT | O_RDWR, 0666);

    if (IS_ERR(cali_file))
	{
	printk(KERN_ERR "%s: filp_open error!\n", __func__);
	return -STK_K_FAIL_OPEN_FILE;
	} else
	{
		fs = get_fs();
		set_fs(get_ds());

		ret = cali_file->f_op->write(cali_file, w_buf, STK_ACC_CALI_FILE_SIZE, &cali_file->f_pos);
		if (ret != STK_ACC_CALI_FILE_SIZE)
		{
			printk(KERN_ERR "%s: write error!\n", __func__);
			filp_close(cali_file, NULL);
			return -STK_K_FAIL_W_FILE;
		}
		cali_file->f_pos = 0x00;
		ret = cali_file->f_op->read(cali_file, r_buf, STK_ACC_CALI_FILE_SIZE, &cali_file->f_pos);
		if (ret < 0)
		{
			printk(KERN_ERR "%s: read error!\n", __func__);
			filp_close(cali_file, NULL);
			return -STK_K_FAIL_R_BACK;
		}
		set_fs(fs);

		/* printk(KERN_INFO "%s: read ret=%d!\n", __func__, ret); */
		for (i = 0; i < STK_ACC_CALI_FILE_SIZE; i++)
		{
			if (r_buf[i] != w_buf[i])
			{
				printk(KERN_ERR "%s: read back error, r_buf[%x](0x%x) != w_buf[%x](0x%x)\n",
					__func__, i, r_buf[i], i, w_buf[i]);
				filp_close(cali_file, NULL);
				return -STK_K_FAIL_R_BACK_COMP;
			}
		}
    }
    filp_close(cali_file, NULL);	

	printk(KERN_INFO "%s successfully\n", __func__);
	return 0;
}

void yas_acc_handle_first_en(struct yas_state *stk)
{
	char r_buf[STK_ACC_CALI_FILE_SIZE] = {0};
	int offset[3] = {0};
	char mode;

	if ((stk_get_file_content(r_buf, STK_ACC_CALI_FILE_SIZE)) == 0)
	{printk(KERN_INFO "%s 0x%x 0x%x\n", __func__, r_buf[0], r_buf[1]);
		if (r_buf[0] == STK_ACC_CALI_VER0 && r_buf[1] == STK_ACC_CALI_VER1)
		{
			if (r_buf[2] & 0x80)
				stk->calib_bias[0] = (((int)r_buf[2]<<4) + (r_buf[3]>>4) - 4096)*(9806550/256);
			else
				stk->calib_bias[0] = (((int)r_buf[2]<<4) + (r_buf[3]>>4))*(9806550/256);
			printk(KERN_INFO "%s r_buf[2]=0x%x 0x%x stk->calib_bias[0]= %d\n", __func__, r_buf[2], r_buf[3], stk->calib_bias[1]);
			if (r_buf[4] & 0x80)
				stk->calib_bias[1] = (((int)r_buf[4]<<4) + (r_buf[5]>>4) - 4096)*(9806550/256);
			else
				stk->calib_bias[1] = (((int)r_buf[4]<<4) + (r_buf[5]>>4))*(9806550/256);
			printk(KERN_INFO "%s r_buf[4]=0x%x 0x%x stk->calib_bias[1]= %d\n", __func__, r_buf[4], r_buf[5], stk->calib_bias[0]);
			if (r_buf[6] & 0x80)
				stk->calib_bias[2] = (((int)r_buf[6]<<4) + (r_buf[7]>>4) - 4096)*(9806550/256);
			else
				stk->calib_bias[2] = (((int)r_buf[6]<<4) + (r_buf[7]>>4))*(9806550/256);
			printk(KERN_INFO "%s r_buf[6]=0x%x 0x%x stk->calib_bias[2]= %d\n", __func__, r_buf[6], r_buf[7], stk->calib_bias[2]);

			mode = r_buf[8];
			/*
			stk->calib_bias[0] = -(stk->calib_bias[0]);
			stk->calib_bias[1] = -(stk->calib_bias[1]);
			*/
			printk(KERN_INFO "%s: set offset:%d,%d,%d, mode=%d\n", __func__, stk->calib_bias[0], stk->calib_bias[1], stk->calib_bias[2], mode);

		} else
		{
			printk(KERN_ERR "%s: cali version number error! r_buf=0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n",
					__func__, r_buf[0], r_buf[1], r_buf[2], r_buf[3], r_buf[4], r_buf[5], r_buf[6], r_buf[7], r_buf[8]);
			/* return -EINVAL; */
		}
	} else
	{printk(KERN_INFO "%s 111\n", __func__);
		offset[0] = offset[1] = offset[2] = 0;
		stk_store_in_file(stk, offset, STK_K_NO_CALI);
	}
	return;
}

static int cali_res = 0xFF;

static int yas_acc_SetCali(struct yas_state *stk)
{
	char org_enable;
	int acc_ave[3] = {0, 0, 0};
	int acc_ave_tmp[3] = {0, 0, 0};
	int state, axis, idx;
/* int new_offset[3]; */
/* char char_offset[3] = {0}; */
	int result = 0;
	uint8_t buffer[2] = "";
	uint8_t reg_offset[3] = {0};
/* char store_location = sstate; */
	int gdelay_ms, real_delay_ms = 10;
/* char offset[3]; */
	int pre_rawdata[3] = {0};
	int isShake = 0;
	struct yas_data acc[1];

	stk->calib_bias[0] = stk->calib_bias[1] = stk->calib_bias[2] = 0;

	gdelay_ms = stk->acc.get_delay();
	org_enable = stk->acc.get_enable();

	printk(KERN_INFO "%s:org_enable=%d\n", __func__, org_enable);
	printk(KERN_INFO "%s:gdelay_ms=%d\n", __func__, gdelay_ms);

	if (!org_enable) {
		result = stk->acc.set_enable(1);
		if (result) {
			printk(KERN_INFO "%s:set_enable err=%d\n", __func__, result);
			goto err_i2c_rw;
		}
	}
	cancel_delayed_work_sync(&stk->work);

	result = stk->acc.set_delay(10);
	if (result) goto err_i2c_rw;

	if (yas_write_reg(0X0b/*YAS_REG_SR*/, yas_odr_tbl[2].odr) < 0) {
		printk(KERN_INFO "%s:yas_device_write 0X0b 0x%x err\n", __func__, yas_odr_tbl[6].odr);
		goto err_i2c_rw;
		}
	msleep(1);

	if (yas_read_reg(0X0b/*YAS_REG_SR*/, &buffer[0]) < 0)
		goto err_i2c_rw;
	printk(KERN_INFO "%s:YAS_REG_SR=0x%x\n", __func__, buffer[0]);

	/* set chip offset */
	/* STK831x_SetOffset(reg_offset); */
	for (idx = 0; idx < 3; idx++) {
		if (yas_write_reg(0x0f /*OFSX*/+idx, reg_offset[idx]) < 0) {
			printk(KERN_INFO "%s:yas_write_reg %0x0x 0x%x err\n", __func__, 0x0f+idx, reg_offset[idx]);
			goto err_i2c_rw;
		}
	}
	msleep(20);

	for (state = 0; state < STK_SAMPLE_ALL; state++) {
		msleep(real_delay_ms);
		stk->acc.measure_raw(acc, 1);
		pr_info("%s: [%d]acc=%d,%d,%d\n", __func__, state, acc[0].xyz.v[0], acc[0].xyz.v[1], acc[0].xyz.v[2]);
		if (state < 10)
			continue;

		for (axis = 0; axis < 3; axis++) {
			if (state > 10 &&
			((pre_rawdata[axis] > acc[0].xyz.v[axis] && (pre_rawdata[axis] - acc[0].xyz.v[axis]) > 20) ||
			(acc[0].xyz.v[axis] > pre_rawdata[axis] && (acc[0].xyz.v[axis] - pre_rawdata[axis]) > 20))) {
				isShake = 1;
				pr_info("%s: pre_rawdata=%d,%d,%d\n",
						__func__, pre_rawdata[0], pre_rawdata[1], pre_rawdata[2]);
				break;
			} else
				pre_rawdata[axis] = acc[0].xyz.v[axis];

			acc_ave_tmp[axis] += acc[0].xyz.v[axis];
		}
		if (isShake)
			break;
	}

	if (isShake)
		goto err_shake;

	for (axis = 0; axis < 3; axis++)
	{
		acc_ave_tmp[axis] /= STK_SAMPLE_CAL;
	}
	pr_info("%s: acc_ave=%d,%d,%d\n", __func__, acc_ave_tmp[0], acc_ave_tmp[1], acc_ave_tmp[2]);
/*
	acc_ave[0] = -acc_ave[0];
	acc_ave[1] = -acc_ave[1];
	acc_ave[2] = -(acc_ave[2]) - 256;
*/
	acc_ave[0] = -acc_ave_tmp[1];
	acc_ave[1] = -acc_ave_tmp[0];
	acc_ave[2] = -(acc_ave_tmp[2]) - 256;

	printk(KERN_INFO "%s: offset:%d,%d,%d\n", __func__, acc_ave[0], acc_ave[1], acc_ave[2]);

	if ((acc_ave[0] > 52 || acc_ave[0] < -52) ||
	(acc_ave[1] > 52 || acc_ave[1] < -52) ||
	(acc_ave[2] > 154 || acc_ave[2] < -154))
		goto err_shake;
/*
	stk->calib_bias[0] = -(acc_ave[1])*(9806550/256);
	stk->calib_bias[1] = -(acc_ave[0])*(9806550/256);
	stk->calib_bias[2] = acc_ave[2]*(9806550/256);
*/

	stk->calib_bias[0] = (acc_ave[0])*(9806550/256);
	stk->calib_bias[1] = (acc_ave[1])*(9806550/256);
	stk->calib_bias[2] = acc_ave[2]*(9806550/256);

	printk(KERN_INFO "%s: offset:%d,%d,%d\n", __func__, stk->calib_bias[0], stk->calib_bias[1], stk->calib_bias[2]);

			result = stk_store_in_file(stk, acc_ave, STK_K_SUCCESS_FILE);
			if (result)
			{
				printk(KERN_INFO "%s:write calibration failed\n", __func__);
				goto err_write_offset;
			} else
			{
				printk(KERN_INFO "%s successfully\n", __func__);
			}

	stk->acc.set_delay(gdelay_ms);

	if (!org_enable)
		stk->acc.set_enable(0);
	else
		schedule_delayed_work(&stk->work, 0);
	cali_res = 1;
	return 0;

err_shake:
	pr_err("%s: shake error\n", __func__);
	   isShake = 0;
	   if (!org_enable)
		   stk->acc.set_enable(0);
	   else
		   schedule_delayed_work(&stk->work, 0);
       return cali_res = 0xff;

err_i2c_rw:
	printk(KERN_ERR "%s: i2c read/write error, err=0x%x\n", __func__, result);
	if (!org_enable)
		stk->acc.set_enable(0);
	else
		schedule_delayed_work(&stk->work, 0);
	return cali_res = 0xff;

err_write_offset:
	printk(KERN_ERR "%s: write_offset error, err=0x%x\n", __func__, result);
	if (!org_enable)
		stk->acc.set_enable(0);
	else
		schedule_delayed_work(&stk->work, 0);
	return cali_res = 0xff;
}

static int yas_acc_GetCali(struct yas_state *stk)
{
	return cali_res;
}

static ssize_t yas_acc_cali_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct yas_state *stk = iio_priv(indio_dev);
	int status = yas_acc_GetCali(stk);

	return scnprintf(buf, PAGE_SIZE,  "%02x\n", status);
}

static ssize_t yas_acc_cali_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct yas_state *stk = iio_priv(indio_dev);
	unsigned long data;
	int error;

	error = strict_strtoul(buf, 10, &data);
	if (error)
	{
		printk(KERN_ERR "%s: strict_strtoul failed, error=0x%x\n", __func__, error);
		return error;
	}
	yas_acc_SetCali(stk);
	return count;
}

static ssize_t stk831x_offset_show(struct device *dev,
	       struct device_attribute *attr, char *buf)
{
/* struct iio_dev *indio_dev = dev_get_drvdata(dev); */
/* struct yas_state *stk = iio_priv(indio_dev); */
	char r_buf[STK_ACC_CALI_FILE_SIZE] = {0};
	int cali_offset[3] = {0, 0, 0};

	if ((stk_get_file_content(r_buf, STK_ACC_CALI_FILE_SIZE)) == 0)
	{
		if (r_buf[0] == STK_ACC_CALI_VER0 && r_buf[1] == STK_ACC_CALI_VER1)
		{
			if (r_buf[2] & 0x80)
				cali_offset[0] = (((int)r_buf[2]<<4) + (r_buf[3]>>4) - 4096);
			else
				cali_offset[0] = (((int)r_buf[2]<<4) + (r_buf[3]>>4));
			pr_info("%s r_buf[2]=0x%x 0x%x stk->calib_bias[0]= %d\n",
						__func__, r_buf[2], r_buf[3], cali_offset[0]);
			if (r_buf[4] & 0x80)
				cali_offset[1] = (((int)r_buf[4]<<4) + (r_buf[5]>>4) - 4096);
			else
				cali_offset[1] = (((int)r_buf[4]<<4) + (r_buf[5]>>4));
			pr_info("%s r_buf[4]=0x%x 0x%x stk->calib_bias[1]= %d\n",
						__func__, r_buf[4], r_buf[5], cali_offset[1]);
			if (r_buf[6] & 0x80)
				cali_offset[2] = (((int)r_buf[6]<<4) + (r_buf[7]>>4) - 4096);
			else
				cali_offset[2] = (((int)r_buf[6]<<4) + (r_buf[7]>>4));
			printk(KERN_INFO "%s r_buf[6]=0x%x 0x%x stk->calib_bias[2]= %d\n", __func__, r_buf[6], r_buf[7], cali_offset[2]);
/*
			cali_offset[0] = -(cali_offset[0]);
			cali_offset[1] = -(cali_offset[1]);
*/
			printk(KERN_INFO "%s: get offset:%d,%d,%d\n", __func__, cali_offset[0], cali_offset[1], cali_offset[2]);
		} else
			printk(KERN_ERR "%s: cali version number error!", __func__);
	} else{
		printk(KERN_INFO "%s:open CALI file error", __func__);
	}

       return sprintf(buf, "%4d %4d %4d\n", cali_offset[0], cali_offset[1], cali_offset[2]);
}

static int yas_write_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan,
		int val,
		int val2,
		long mask)
{
	struct yas_state  *st = iio_priv(indio_dev);

	mutex_lock(&st->lock);

	switch (mask) {
	case IIO_CHAN_INFO_CALIBBIAS:
		st->calib_bias[chan->channel2 - IIO_MOD_X] = val;
		break;
	}

	mutex_unlock(&st->lock);

	return 0;
}

static int yas_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan,
		int *val,
		int *val2,
		long mask) {
	struct yas_state  *st = iio_priv(indio_dev);
	int ret = -EINVAL;

	if (chan->type != IIO_ACCEL)
		return -EINVAL;

	mutex_lock(&st->lock);

	switch (mask) {
	case 0:
		*val = st->accel_data[chan->channel2 - IIO_MOD_X];
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_SCALE:
	case IIO_CHAN_INFO_CALIBSCALE:
		/* Gain : counts / m/s^2 = 1000000 [um/s^2] */
		/* Scaling factor : 1000000 / Gain = 1 */
		*val = 0;
		*val2 = 1;
		ret = IIO_VAL_INT_PLUS_MICRO;
		break;
	case IIO_CHAN_INFO_CALIBBIAS:
		*val = st->calib_bias[chan->channel2 - IIO_MOD_X];
		ret = IIO_VAL_INT;
		break;
	}

	mutex_unlock(&st->lock);

	return ret;
}

static void yas_work_func(struct work_struct *work)
{
	struct yas_data acc[1];
	struct yas_state *st =
		container_of((struct delayed_work *)work,
				struct yas_state, work);
	struct iio_dev *indio_dev = iio_priv_to_dev(st);
	uint32_t time_before, time_after;
	int32_t delay;
	int ret, i;

	time_before = jiffies_to_msecs(jiffies);
	mutex_lock(&st->lock);
	ret = st->acc.measure(acc, 1);
	if (ret == 1) {
		for (i = 0; i < 3; i++) {
			st->accel_data[i] = acc[0].xyz.v[i]
				- st->calib_bias[i];
		}
		/*
			printk(KERN_ALERT"accel_data %d %d %d; calib_bias %d %d %d\n",
				acc[0].xyz.v[0], acc[0].xyz.v[1], acc[0].xyz.v[2],
				st->calib_bias[0], st->calib_bias[1], st->calib_bias[2]);
			printk(KERN_ALERT"After accel_data : %d %d %d\n",
				st->accel_data[0], st->accel_data[1],st->accel_data[2]);
		*/
	}
	mutex_unlock(&st->lock);
	if (ret == 1)
		yas_data_rdy_trig_poll(indio_dev);
	time_after = jiffies_to_msecs(jiffies);
	delay = MSEC_PER_SEC / st->sampling_frequency
		- (time_after - time_before);
	if (delay <= 0)
		delay = 1;
	schedule_delayed_work(&st->work, msecs_to_jiffies(delay));
}

#define YAS_ACCEL_INFO_SHARED_MASK			\
	(BIT(IIO_CHAN_INFO_SCALE))
#define YAS_ACCEL_INFO_SEPARATE_MASK			\
	(BIT(IIO_CHAN_INFO_RAW) |			\
	 BIT(IIO_CHAN_INFO_CALIBBIAS) |			\
	 BIT(IIO_CHAN_INFO_CALIBSCALE))

#define YAS_ACCELEROMETER_CHANNEL(axis)		\
{							\
	.type = IIO_ACCEL,				\
	.modified = 1,					\
	.channel2 = IIO_MOD_##axis,			\
	.info_mask_separate = YAS_ACCEL_INFO_SEPARATE_MASK,   \
	.info_mask_shared_by_type = YAS_ACCEL_INFO_SHARED_MASK,\
	.scan_index = YAS_SCAN_ACCEL_##axis,		\
	.scan_type = IIO_ST('s', 32, 32, 0)		\
}

static const struct iio_chan_spec yas_channels[] = {
	YAS_ACCELEROMETER_CHANNEL(X),
	YAS_ACCELEROMETER_CHANNEL(Y),
	YAS_ACCELEROMETER_CHANNEL(Z),
	IIO_CHAN_SOFT_TIMESTAMP(YAS_SCAN_TIMESTAMP)
};

static IIO_DEVICE_ATTR(sampling_frequency, S_IRUSR|S_IWUSR,
		yas_sampling_frequency_show,
		yas_sampling_frequency_store, 0);
static IIO_DEVICE_ATTR(position, S_IRUSR|S_IWUSR,
		yas_position_show, yas_position_store, 0);
static IIO_DEVICE_ATTR(ping, S_IRUSR,
		yas_ping_show, NULL, 0);
static IIO_DEVICE_ATTR(cali, S_IRUSR|S_IWUSR,
		yas_acc_cali_show,
		yas_acc_cali_store, 0);
static IIO_DEVICE_ATTR(offset, S_IRUSR,
		stk831x_offset_show, NULL, 0);

static struct attribute *yas_attributes[] = {
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_dev_attr_position.dev_attr.attr,
	&iio_dev_attr_ping.dev_attr.attr,
		&iio_dev_attr_cali.dev_attr.attr,
	&iio_dev_attr_offset.dev_attr.attr,
	NULL
};
static const struct attribute_group yas_attribute_group = {
	.attrs = yas_attributes,
};

static const struct iio_info yas_info = {
	.read_raw = &yas_read_raw,
	.write_raw = &yas_write_raw,
	.attrs = &yas_attribute_group,
	.driver_module = THIS_MODULE,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void yas_early_suspend(struct early_suspend *h)
{
	struct yas_state *st = container_of(h,
			struct yas_state, sus);
	if (atomic_read(&st->pseudo_irq_enable)) {
		cancel_delayed_work_sync(&st->work);
		st->acc.set_enable(0);
	}
}


static void yas_late_resume(struct early_suspend *h)
{
	struct yas_state *st = container_of(h,
			struct yas_state, sus);
	if (atomic_read(&st->pseudo_irq_enable)) {
		st->acc.set_enable(1);
		schedule_delayed_work(&st->work, 0);
	}
}
#endif

/* For Sensortek stk8313 accelerometer power */
static int yas_power_on(struct yas_state *st)
{

	int rc = 0;

	rc = regulator_enable(st->vdd);
		if (rc) {
			dev_err(&st->client->dev, "Regulator vdd enable failed rc=%d\n", rc);
			return rc;
		}

		rc = regulator_enable(st->vio);
		if (rc) {
			dev_err(&st->client->dev, "Regulator vio enable failed rc=%d\n", rc);
			regulator_disable(st->vdd);
			return rc;
		}
	return rc;

}


static int yas_power_init(struct yas_state *st, bool on)
{
	int rc;

	if (!on) {
		if (regulator_count_voltages(st->vdd) > 0)
			regulator_set_voltage(st->vdd, 0, STK831X_VDD_MAX_UV);

		regulator_put(st->vdd);

		if (regulator_count_voltages(st->vio) > 0)
			regulator_set_voltage(st->vio, 0, STK831X_VIO_MAX_UV);

		regulator_put(st->vio);
	} else {
		st->vdd = regulator_get(&st->client->dev, "vdd");
		if (IS_ERR(st->vdd)) {
			rc = PTR_ERR(st->vdd);
			dev_err(&st->client->dev,
				"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(st->vdd) > 0) {
			rc = regulator_set_voltage(st->vdd, STK831X_VDD_MIN_UV,
						   STK831X_VDD_MAX_UV);
			if (rc) {
				dev_err(&st->client->dev,
					"Regulator set failed vdd rc=%d\n",
					rc);
				goto reg_vdd_put;
			}
		}

		st->vio = regulator_get(&st->client->dev, "vio");
		if (IS_ERR(st->vio)) {
			rc = PTR_ERR(st->vio);
			dev_err(&st->client->dev,
				"Regulator get failed vio rc=%d\n", rc);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(st->vio) > 0) {
			rc = regulator_set_voltage(st->vio, STK831X_VIO_MIN_UV,
						   STK831X_VIO_MAX_UV);
			if (rc) {
				dev_err(&st->client->dev,
				"Regulator set failed vio rc=%d\n", rc);
				goto reg_vio_put;
			}
		}
	}

	return 0;

reg_vio_put:
	regulator_put(st->vio);
reg_vdd_set:
	if (regulator_count_voltages(st->vdd) > 0)
		regulator_set_voltage(st->vdd, 0, STK831X_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(st->vdd);
	return rc;
}
/* For Sensortek stk8313 accelerometer power */

struct yas_state *st_tmp;

static int yas_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct yas_state *st;
	struct iio_dev *indio_dev;
	int ret, i;
	int hwid;

	hwid = get_cei_hw_id();

	if (hwid > DVT1) {
		pr_err("[Sensor] %s , yas driver not for DVT2", __func__);
		return -EFAULT;
	}

	this_client = i2c;
	indio_dev = iio_device_alloc(sizeof(*st));
	if (!indio_dev) {
		ret = -ENOMEM;
		goto error_ret;
	}
	i2c_set_clientdata(i2c, indio_dev);

	indio_dev->name = YAS_ACC_NAME;
	indio_dev->dev.parent = &i2c->dev;
	indio_dev->info = &yas_info;
	indio_dev->channels = yas_channels;
	indio_dev->num_channels = ARRAY_SIZE(yas_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->id = 1;
	dev_set_name(&indio_dev->dev, "iio:device%d", indio_dev->id);

	st = iio_priv(indio_dev);
	st->client = i2c;
	st->sampling_frequency = 20;
	st->acc.callback.device_open = yas_device_open;
	st->acc.callback.device_close = yas_device_close;
	st->acc.callback.device_read = yas_device_read;
	st->acc.callback.device_write = yas_device_write;
	st->acc.callback.usleep = yas_usleep;
	st->acc.callback.current_time = yas_current_time;
	INIT_DELAYED_WORK(&st->work, yas_work_func);
	mutex_init(&st->lock);
#ifdef CONFIG_HAS_EARLYSUSPEND
	st->sus.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	st->sus.suspend = yas_early_suspend;
	st->sus.resume = yas_late_resume;
	register_early_suspend(&st->sus);
#endif
	for (i = 0; i < 3; i++) {
		st->accel_data[i] = 0;
		st->calib_bias[i] = 0;
	}

	/* For Sensortek stk8313 accelerometer power */
	ret = yas_power_init(st, true);
	if (ret < 0) {
		dev_err(&st->client->dev, "power init failed! err=%d", ret);
	}
	ret = yas_power_on(st);
	if (ret < 0) {
		dev_err(&st->client->dev, "power on failed! err=%d\n", ret);
	}
	/* For Sensortek stk8313 accelerometer power */
    msleep(5);
	ret = yas_probe_buffer(indio_dev);
	if (ret)
		goto error_free_dev;
	ret = yas_probe_trigger(indio_dev);
	if (ret)
		goto error_remove_buffer;
	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_remove_trigger;
	ret = yas_acc_driver_init(&st->acc);
	if (ret < 0) {
		ret = -EFAULT;
		goto error_unregister_iio;
	}
	ret = st->acc.init();
	if (ret < 0) {
		ret = -EFAULT;
		goto error_unregister_iio;
	}
	spin_lock_init(&st->spin_lock);
	st_tmp = st;

	return 0;

error_unregister_iio:
	iio_device_unregister(indio_dev);
error_remove_trigger:
	yas_remove_trigger(indio_dev);
error_remove_buffer:
	yas_remove_buffer(indio_dev);
error_free_dev:
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&st->sus);
#endif
	iio_device_free(indio_dev);
error_ret:
	i2c_set_clientdata(i2c, NULL);
	this_client = NULL;
	return ret;
}

static int yas_remove(struct i2c_client *i2c)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(i2c);
	struct yas_state *st;

	if (indio_dev) {
		st = iio_priv(indio_dev);
#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&st->sus);
#endif
		yas_pseudo_irq_disable(indio_dev);
		st->acc.term();
		iio_device_unregister(indio_dev);
		yas_remove_trigger(indio_dev);
		yas_remove_buffer(indio_dev);
		iio_device_free(indio_dev);
		this_client = NULL;
	}
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int yas_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct yas_state *st = iio_priv(indio_dev);

	if (atomic_read(&st->pseudo_irq_enable)) {
		cancel_delayed_work_sync(&st->work);
		st->acc.set_enable(0);
	}
	return 0;
}

static int yas_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct yas_state *st = iio_priv(indio_dev);

	if (atomic_read(&st->pseudo_irq_enable)) {
		st->acc.set_enable(1);
		schedule_delayed_work(&st->work, 0);
	}
	return 0;
}

static SIMPLE_DEV_PM_OPS(yas_pm_ops, yas_suspend, yas_resume);
#define YAS_PM_OPS (&yas_pm_ops)
#else
#define YAS_PM_OPS NULL
#endif

static const struct i2c_device_id yas_id[] = {
	{YAS_ACC_NAME, 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, yas_id);

static struct of_device_id stk831x_dts_table[] = {
		{ .compatible  = "stk,stk8313",},
		{ },
};

static struct i2c_driver yas_driver = {
	.driver = {
		.name	= YAS_ACC_NAME,
		.owner	= THIS_MODULE,
		.pm	= YAS_PM_OPS,
		.of_match_table = stk831x_dts_table,
	},
	.probe		= yas_probe,
	.remove		= yas_remove,
	.id_table	= yas_id,
};
module_i2c_driver(yas_driver);

MODULE_DESCRIPTION("Yamaha Acceleration I2C driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("5.6.1032");
