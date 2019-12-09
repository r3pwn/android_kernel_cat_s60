/*
 * Copyright (c) 2015 Yamaha Corporation
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

#include "yas.h"

#if YAS_ACC_DRIVER == YAS_ACC_DRIVER_STK8313

#define YAS_RANGE_2G			(0)
#define YAS_RANGE_4G			(1)
#define YAS_RANGE_8G			(2)
#define YAS_RANGE			YAS_RANGE_8G

#define YAS_RESOLUTION			(256)

#define YAS_GRAVITY_EARTH		(9806550)

#define YAS_SOFTRESET_WAIT_TIME		(1000)
#define YAS_STABLE_WAIT_TIME		(15000)
#define YAS_OTP_WAIT_TIME		(1000)
#define YAS_OTP_READY_WAIT_TIME		(2000)
#define YAS_OTP_READY_WAIT_COUNT	(10)

#define YAS_DEFAULT_POSITION		(6)
#define YAS_MODE_STANDBY		(0X00)
#define YAS_MODE_ACTIVE			(0X01)

#define YAS_REG_XOUT1			(0X00)
#define YAS_REG_XOUT2			(0X01)
#define YAS_REG_YOUT1			(0X02)
#define YAS_REG_YOUT2			(0X03)
#define YAS_REG_ZOUT1			(0X04)
#define YAS_REG_ZOUT2			(0X05)
#define YAS_REG_DATA			YAS_REG_XOUT1

#define YAS_REG_SRST			(0X07)
#define YAS_REG_MODE			(0X0a)
#define YAS_REG_SR				(0X0b)
#define YAS_REG_PLAT			(0x12)
#define YAS_REG_STH				(0X16)
#define YAS_REG_SWRST			(0X20)
#define YAS_REG_AFECTRL			(0X24)
#define YAS_REG_OTPADDR			(0X3d)
#define YAS_REG_OTPDATA			(0X3e)
#define YAS_REG_OTPCTRL			(0X3f)

#define YAS_VAL_SOFTRESET		(0x00)
#define YAS_VAL_ODR_ACTIVE		(0x01)

#define YAS_VAL_RANGE_2G		(0x00)
#define YAS_VAL_RANGE_4G		(0x40)
#define YAS_VAL_RANGE_8G		(0x80)
#if YAS_RANGE == YAS_RANGE_2G
#define YAS_VAL_RANGE			YAS_VAL_RANGE_2G
#elif YAS_RANGE == YAS_RANGE_4G
#define YAS_VAL_RANGE			YAS_VAL_RANGE_4G
#elif YAS_RANGE == YAS_RANGE_8G
#define YAS_VAL_RANGE			YAS_VAL_RANGE_8G
#else
#define YAS_VAL_RANGE			YAS_VAL_RANGE_2G
#endif

#define YAS_VAL_OTPADDR			(0X70)
#define YAS_VAL_OTPCTRL			(0X02)

#define YAS_ODR_400HZ			(0x00)
#define YAS_ODR_200HZ			(0x01)
#define YAS_ODR_100HZ			(0x02)
#define YAS_ODR_50HZ			(0x03)
#define YAS_ODR_25HZ			(0x04)
#define YAS_ODR_12_5HZ			(0x05)
#define YAS_ODR_6_25HZ			(0x06)
#define YAS_ODR_3_125HZ			(0x07)

#define YAS_SENSOR_DELAY		(10)

const static unsigned int OTPReg[6]={0x67,0x68,0x69,0x6A,0x6C,0x6E};
const static unsigned int EngReg[6]={0x29,0x2D,0x31,0x2A,0x2E,0x32};

struct yas_odr {
	int delay;
	uint8_t odr;
};

struct yas_module {
	int initialized;
	int enable;
	int delay;
	int position;
	uint8_t odr;
	int turn_on_time;
	struct yas_driver_callback cbk;
};

static const struct yas_odr yas_odr_tbl[] = {
	{3,	YAS_ODR_400HZ},
	{5,	YAS_ODR_200HZ},
	{10,	YAS_ODR_100HZ},
	{20,	YAS_ODR_50HZ},
	{40,	YAS_ODR_25HZ},
	{80,	YAS_ODR_12_5HZ},
	{160,	YAS_ODR_6_25HZ},
	{320,	YAS_ODR_3_125HZ},
};

static const int8_t yas_position_map[][9] = {
	{ 0, -1,  0,  1,  0,  0,  0,  0,  1 },/* top/upper-left */
	{ 1,  0,  0,  0,  1,  0,  0,  0,  1 },/* top/upper-right */
	{ 0,  1,  0, -1,  0,  0,  0,  0,  1 },/* top/lower-right */
	{-1,  0,  0,  0, -1,  0,  0,  0,  1 },/* top/lower-left */
	{ 0,  1,  0,  1,  0,  0,  0,  0, -1 },/* bottom/upper-right */
	{-1,  0,  0,  0,  1,  0,  0,  0, -1 },/* bottom/upper-left */
	{ 0, -1,  0, -1,  0,  0,  0,  0, -1 },/* bottom/lower-left */
	{ 1,  0,  0,  0, -1,  0,  0,  0, -1 },/* bottom/lower-right */
};

static struct yas_module module;

int yas_read_reg(uint8_t adr, uint8_t *val);
int yas_write_reg(uint8_t adr, uint8_t val);
static void yas_set_odr(int delay);
static int yas_power_up(void);
static int yas_power_down(void);
static int yas_init(void);
static int yas_term(void);
static int yas_get_delay(void);
static int yas_set_delay(int delay);
static int yas_get_enable(void);
static int yas_set_enable(int enable);
static int yas_get_position(void);
static int yas_set_position(int position);
static int yas_measure(struct yas_data *raw, int num);
static int yas_ext(int32_t cmd, void *result);

int
yas_read_reg(uint8_t adr, uint8_t *val)
{
	return module.cbk.device_read(YAS_TYPE_ACC, adr, val, 1);
}

int
yas_write_reg(uint8_t adr, uint8_t val)
{
	return module.cbk.device_write(YAS_TYPE_ACC, adr, &val, 1);
}

static void
yas_set_odr(int delay)
{
	int i;
	for (i = 1; i < NELEMS(yas_odr_tbl) &&
		     delay >= yas_odr_tbl[i].delay; i++)
		;
	module.odr = yas_odr_tbl[i-1].odr;
	module.turn_on_time = yas_odr_tbl[i-1].delay * YAS_STABLE_WAIT_TIME;
}
#if 0
static int
yas_otp_operate(void)
{
	int i;
	uint8_t value = 0;
	int count = YAS_OTP_READY_WAIT_COUNT;

	for (i = 0; i < 2; i++) {
		if (yas_write_reg(YAS_REG_OTPADDR, YAS_VAL_OTPADDR) < 0){
			printk("yas_otp_operate err 0 i=%d\n", i);
			return YAS_ERROR_DEVICE_COMMUNICATION;
			}
	}
	if (yas_write_reg(YAS_REG_OTPCTRL, YAS_VAL_OTPCTRL) < 0){
		printk("yas_otp_operate err 0\n");
		return YAS_ERROR_DEVICE_COMMUNICATION;
		}

	module.cbk.usleep(YAS_OTP_WAIT_TIME);

	while ((value >> 7) != 1 && count != 0) {
		module.cbk.usleep(YAS_OTP_READY_WAIT_TIME);
		printk("yas_otp_operate err count=%d\n", count);

		if (yas_read_reg(YAS_REG_OTPCTRL, &value) < 0){
			printk("yas_otp_operate err 1\n");
			return YAS_ERROR_DEVICE_COMMUNICATION;
			}
		--count;
	}

	if (yas_read_reg(YAS_REG_OTPDATA, &value) < 0){
		printk("yas_otp_operate err 2\n");
		return YAS_ERROR_DEVICE_COMMUNICATION;
		}
	printk("yas_otp_operate value=%d\n", value);
	if (value != 0) {
		if (yas_write_reg(YAS_REG_AFECTRL, value) < 0){
			printk("yas_otp_operate err 3\n");
			return YAS_ERROR_DEVICE_COMMUNICATION;
			}
	} else{
	printk("yas_otp_operate err 4\n");
		return YAS_ERROR_ERROR;
		}

	return YAS_NO_ERROR;
}
#else
static int
yas_otp_operate(uint8_t rReg, uint8_t *value)
{
	uint8_t data = 0;
	int i;
	int count = YAS_OTP_READY_WAIT_COUNT;
	*value = 0;

	for (i = 0; i < 10; i++) {
		if (yas_write_reg(YAS_REG_OTPADDR, rReg) < 0){
			printk("yas_otp_operate err 0 i=%d\n", i);
			return YAS_ERROR_DEVICE_COMMUNICATION;
		}
		if (yas_read_reg(YAS_REG_OTPADDR, &data) < 0){
			printk("yas_otp_operate err 0 i=%d\n", i);
			return YAS_ERROR_DEVICE_COMMUNICATION;
		}
		printk(KERN_INFO"%s : write [%02x] , read [%02x] , count [%d]\n",__func__, rReg, data, i);
		if(rReg == data)
			break;
	}
	if (yas_write_reg(YAS_REG_OTPCTRL, YAS_VAL_OTPCTRL) < 0){
		printk("yas_otp_operate err 0\n");
		return YAS_ERROR_DEVICE_COMMUNICATION;
		}

	module.cbk.usleep(YAS_OTP_WAIT_TIME);

	while ((*value >> 7) != 1 && count != 0) {
		module.cbk.usleep(YAS_OTP_READY_WAIT_TIME);
		printk("yas_otp_operate count=%d\n", count);

		if (yas_read_reg(YAS_REG_OTPCTRL, value) < 0){
			printk("yas_otp_operate err 1\n");
			return YAS_ERROR_DEVICE_COMMUNICATION;
			}
		--count;
	}

	if (yas_read_reg(YAS_REG_OTPDATA, value) < 0){
		printk("yas_otp_operate err 2\n");
		return YAS_ERROR_DEVICE_COMMUNICATION;
		}
	printk("yas_otp_operate value=0x%x\n", *value);

	return YAS_NO_ERROR;
}
#endif

static int yas_acc_SetVD(void)
{
	int result, i;
	uint8_t buffer;
	uint8_t reg24 = 0, readvalue = 0;
	
	module.cbk.usleep(YAS_OTP_WAIT_TIME);
	
	result = yas_otp_operate(0x66, &reg24);
	if(result < 0)
	{
		printk(KERN_ERR "%s: read back 0x66 error, result=%d\n", __func__, result);
		return result;
	}
	
	printk(KERN_INFO "%s:Read 0x66 = 0x%x\n",  __func__, reg24);
	if(reg24 != 0)
	{
		printk(KERN_INFO "%s:write 0x%x to 0x24\n",  __func__, reg24);
		if (yas_write_reg(YAS_REG_AFECTRL, reg24) < 0){
			printk(KERN_ERR "%s:write 0x%x failed\n", __func__, YAS_REG_AFECTRL);
			return YAS_ERROR_DEVICE_COMMUNICATION;
		}
		
		for(i=0;i<6;i++)
		{	
			result = yas_otp_operate(OTPReg[i], &readvalue);
			if(result < 0)
			{
				printk(KERN_ERR "%s: read back 0x%x error, result=%d\n", __func__, OTPReg[i], result);
				return result;
			}
            printk(KERN_INFO "%s:Read 0x%x = 0x%x\n",  __func__, OTPReg[i], reg24);
			if (yas_write_reg(EngReg[i], readvalue) < 0){
				printk(KERN_ERR "%s:write 0x%x failed\n", __func__, EngReg[i]);
				return YAS_ERROR_DEVICE_COMMUNICATION;
			}
		}
	}
	else
	{
		result = yas_otp_operate(0x70, &reg24);
		if(result < 0)
		{
			printk(KERN_ERR "%s: read back 0x70 error, result=%d\n", __func__, result);
			return result;
		}
		
		printk(KERN_INFO "%s:Read 0x70 = 0x%x\n",  __func__, reg24);
		if(reg24 != 0)
		{
			printk(KERN_INFO "%s:write 0x%x to 0x24\n",  __func__, reg24);
			if (yas_write_reg(YAS_REG_AFECTRL, reg24) < 0){
				printk(KERN_ERR "%s:write 0x%x failed\n", __func__, YAS_REG_AFECTRL);
				return YAS_ERROR_DEVICE_COMMUNICATION;
			}
		}	
		else
		{
			printk(KERN_INFO "%s: reg24=0, do nothing\n", __func__);
			return 0;
		}
	}

	if (yas_read_reg(YAS_REG_AFECTRL, &buffer) < 0){
		printk(KERN_ERR "%s:read 0x%x failed\n", __func__, YAS_REG_AFECTRL);
		return YAS_ERROR_DEVICE_COMMUNICATION;
	}
	printk(KERN_ERR "%s:read 0x%x = 0x%x\n", __func__, YAS_REG_AFECTRL, buffer);

	if(buffer != reg24)
	{
		printk(KERN_ERR "%s: error, reg24=0x%x, read=0x%x\n", __func__, reg24, buffer);
		return -1;
	}
	printk(KERN_INFO "%s: successfully\n", __func__);
	
	return 0;
}

static int
yas_power_up(void)
{
	int rt;

	if (yas_write_reg(YAS_REG_SWRST, YAS_VAL_SOFTRESET) < 0) {
		printk("yas_power_up err 0\n");
		return YAS_ERROR_DEVICE_COMMUNICATION;
	}

	module.cbk.usleep(YAS_SOFTRESET_WAIT_TIME);

	if (yas_write_reg(YAS_REG_SR, module.odr) < 0){
		printk("yas_power_up err 1\n");
		return YAS_ERROR_DEVICE_COMMUNICATION;
		}
	if (yas_write_reg(YAS_REG_STH, YAS_VAL_RANGE) < 0){
		printk("yas_power_up err 2\n");
		return YAS_ERROR_DEVICE_COMMUNICATION;
		}
	if (yas_write_reg(YAS_REG_MODE, YAS_MODE_ACTIVE) < 0){
		printk("yas_power_up err 3\n");
		return YAS_ERROR_DEVICE_COMMUNICATION;
		}

	rt = yas_acc_SetVD();
	if (rt < 0)
		return rt;

	//module.cbk.usleep(module.turn_on_time);

	return YAS_NO_ERROR;
}

static int
yas_power_down(void)
{
	if (yas_write_reg(YAS_REG_MODE, YAS_MODE_STANDBY) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;

	return YAS_NO_ERROR;
}

static int
yas_init(void)
{
	if (module.initialized)
		return YAS_ERROR_INITIALIZE;

	if (module.cbk.device_open(YAS_TYPE_ACC) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;

	module.enable = 0;
	module.delay = YAS_SENSOR_DELAY;
	module.position = YAS_DEFAULT_POSITION;
	yas_set_odr(module.delay);
	yas_power_down();
	module.cbk.device_close(YAS_TYPE_ACC);
	module.initialized = 1;
	return YAS_NO_ERROR;
}

static int
yas_term(void)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	yas_set_enable(0);
	module.initialized = 0;
	return YAS_NO_ERROR;
}

static int
yas_get_delay(void)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	return module.delay;
}

static int
yas_set_delay(int delay)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	if (delay < 0)
		return YAS_ERROR_ARG;
	module.delay = delay;

	return YAS_NO_ERROR;
}

static int
yas_get_enable(void)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	return module.enable;
}

extern struct yas_state *st_tmp;
extern void yas_acc_handle_first_en(struct yas_state *stk);
static int first_en = 0;

static int
yas_set_enable(int enable)
{
	int rt;
    printk("%s enable=%d\n", __func__, enable);
	
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	if (enable != 0)
		enable = 1;
	if (module.enable == enable)
		return YAS_NO_ERROR;
	if (enable) {
		if (module.cbk.device_open(YAS_TYPE_ACC))
			return YAS_ERROR_DEVICE_COMMUNICATION;
		rt = yas_power_up();
		if (rt < 0) {
			module.cbk.device_close(YAS_TYPE_ACC);
			printk("%s yas_power_up\n", __func__);
			return rt;
		}
		if(!first_en) {
			yas_acc_handle_first_en(st_tmp);
			first_en = 1;
		}
	} else {
		yas_power_down();

		module.cbk.device_close(YAS_TYPE_ACC);
	}
	module.enable = enable;
	return YAS_NO_ERROR;
}

static int
yas_get_position(void)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	return module.position;
}

static int
yas_set_position(int position)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	if (position < 0 || position > 7)
		return YAS_ERROR_ARG;
	module.position = position;
	return YAS_NO_ERROR;
}

static int
yas_measure(struct yas_data *raw, int num)
{
	uint8_t buf[6];
	int16_t dat[3];
	int i;

	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	if (raw == NULL || num < 0)
		return YAS_ERROR_ARG;
	if (num == 0 || module.enable == 0)
		return 0;
	if (module.cbk.device_read(YAS_TYPE_ACC
				   , YAS_REG_DATA
				   , buf
				   , 6) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	for (i = 0; i < 3; i++)
		dat[i] = ((int16_t)((buf[i*2] << 8) | buf[i*2+1])) >> 4;

	for (i = 0; i < 3; i++) {
		raw->xyz.v[i] = yas_position_map[module.position][i*3] * dat[0]
			+ yas_position_map[module.position][i*3+1] * dat[1]
			+ yas_position_map[module.position][i*3+2] * dat[2];
		raw->xyz.v[i] *= (YAS_GRAVITY_EARTH / YAS_RESOLUTION);
	}
	raw->type = YAS_TYPE_ACC;
	if (module.cbk.current_time == NULL)
		raw->timestamp = 0;
	else
		raw->timestamp = module.cbk.current_time();
	raw->accuracy = 0;
	return 1;
}

static int
yas_measure_raw(struct yas_data *raw, int num)
{
	uint8_t buf[6];
	int16_t dat[3];
	int i;

	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	if (raw == NULL || num < 0)
		return YAS_ERROR_ARG;
	if (num == 0 || module.enable == 0)
		return 0;
	if (module.cbk.device_read(YAS_TYPE_ACC
				   , YAS_REG_DATA
				   , buf
				   , 6) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	for (i = 0; i < 3; i++)
		raw->xyz.v[i] = dat[i] = ((int16_t)((buf[i*2] << 8) | buf[i*2+1])) >> 4;

	raw->type = YAS_TYPE_ACC;
	if (module.cbk.current_time == NULL)
		raw->timestamp = 0;
	else
		raw->timestamp = module.cbk.current_time();
	raw->accuracy = 0;
	return 1;
}

static int
yas_ext(int32_t cmd, void *result)
{
	(void)cmd;
	(void)result;
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	return YAS_NO_ERROR;
}

int
yas_acc_driver_init(struct yas_acc_driver *f)
{
	if (f == NULL
	    || f->callback.device_open == NULL
	    || f->callback.device_close == NULL
	    || f->callback.device_write == NULL
	    || f->callback.device_read == NULL
	    || f->callback.usleep == NULL)
		return YAS_ERROR_ARG;
	f->init = yas_init;
	f->term = yas_term;
	f->get_delay = yas_get_delay;
	f->set_delay = yas_set_delay;
	f->get_enable = yas_get_enable;
	f->set_enable = yas_set_enable;
	f->get_position = yas_get_position;
	f->set_position = yas_set_position;
	f->measure = yas_measure;
	f->measure_raw = yas_measure_raw;
	f->ext = yas_ext;
	module.cbk = f->callback;
	module.initialized = 0;
	return YAS_NO_ERROR;
}

#endif
