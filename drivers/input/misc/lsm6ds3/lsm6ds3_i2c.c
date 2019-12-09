/*
 * STMicroelectronics lsm6ds3 i2c driver
 *
 * Copyright 2014 STMicroelectronics Inc.
 *
 * Giuseppe Barba <giuseppe.barba@st.com>
 * v 1.1.0
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/types.h>

#include	<linux/platform_data/lsm6ds3.h>
#include	"lsm6ds3_core.h"

/* S- [PM99] Grace_Chang add for sensor power*/
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/cei_hw_id.h>

/* POWER SUPPLY VOLTAGE RANGE */
#define LSM6DS3_VDD_MIN_UV	1700000
#define LSM6DS3_VDD_MAX_UV	1900000

static struct regulator *vdd;

static int lsm6ds3_power_on(struct i2c_client *client)
{

	int rc = 0;

	rc = regulator_enable(vdd);
	if (rc) {
		dev_err(&client->dev, "Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}
	return rc;
}

static int lsm6ds3_power_init(struct i2c_client *client, bool on)
{
	int rc = 0;

	if (!on) {
		if (regulator_count_voltages(vdd) > 0)
			regulator_set_voltage(vdd, 0, LSM6DS3_VDD_MAX_UV);

		regulator_put(vdd);

	} else {
		vdd = regulator_get(&client->dev, "vdd");
		if (IS_ERR(vdd)) {
			rc = PTR_ERR(vdd);
			dev_err(&client->dev,
				"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(vdd) > 0) {
			rc = regulator_set_voltage(vdd, LSM6DS3_VDD_MIN_UV,
						   LSM6DS3_VDD_MAX_UV);
			if (rc) {
				dev_err(&client->dev,
					"Regulator set failed vdd rc=%d\n",
					rc);
				goto reg_vdd_put;
			}
		}

	}

	return 0;

reg_vdd_put:
	regulator_put(vdd);
	return rc;
}

static int lsm6ds3_i2c_read(struct lsm6ds3_data *cdata, u8 reg_addr, int len,
							u8 *data, bool b_lock)
{
	int err = 0;
	struct i2c_msg msg[2];
	struct i2c_client *client = to_i2c_client(cdata->dev);

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].len = 1;
	msg[0].buf = &reg_addr;

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = data;

	if (b_lock) {
		mutex_lock(&cdata->bank_registers_lock);
		err = i2c_transfer(client->adapter, msg, 2);
		mutex_unlock(&cdata->bank_registers_lock);
	} else
		err = i2c_transfer(client->adapter, msg, 2);

	return err;
}

static int lsm6ds3_i2c_write(struct lsm6ds3_data *cdata, u8 reg_addr, int len,
							u8 *data, bool b_lock)
{
	int err = 0;
	u8 send[len + 1];
	struct i2c_msg msg;
	struct i2c_client *client = to_i2c_client(cdata->dev);

	send[0] = reg_addr;
	memcpy(&send[1], data, len * sizeof(u8));
	len++;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.len = len;
	msg.buf = send;

	if (b_lock) {
		mutex_lock(&cdata->bank_registers_lock);
		err = i2c_transfer(client->adapter, &msg, 1);
		mutex_unlock(&cdata->bank_registers_lock);
	} else
		err = i2c_transfer(client->adapter, &msg, 1);

	return err;
}


static const struct lsm6ds3_transfer_function lsm6ds3_tf_i2c = {
	.write = lsm6ds3_i2c_write,
	.read = lsm6ds3_i2c_read,
};

static int lsm6ds3_i2c_probe(struct i2c_client *client,
						const struct i2c_device_id *id)
{
	int err;
	int rc = 0;
	int hwid;
	struct lsm6ds3_data *cdata;

	pr_info("[Sensor] %s , enter", __func__);

	hwid = get_cei_hw_id();

	if (hwid < DVT2) {
		pr_err("[Sensor] %s , lsm6ds3 driver not for DVT1 & EVT", __func__);
		return -EFAULT;
	}

	cdata = kmalloc(sizeof(*cdata), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->dev = &client->dev;
	cdata->name = client->name;
	cdata->tf = &lsm6ds3_tf_i2c;
	i2c_set_clientdata(client, cdata);

/* S- [PM99] Grace_Chang add for sensor power */
	/* Set I2C power */
	rc = lsm6ds3_power_init(client, 1);
	if(!rc)
		lsm6ds3_power_on(client);
	else{
		pr_err("[Sensor] %s , lsm6ds3_power_init fail", __func__);
		return -EFAULT;
	}

	msleep(80);
/* E- [PM99] Grace_Chang add for sensor power */

	err = lsm6ds3_common_probe(cdata, client->irq, BUS_I2C);
	if (err < 0)
		goto free_data;

	pr_info("[Sensor] %s , exit", __func__);
	return 0;

/*exit1:
	lsm6ds3_power_init(cdata, 0);*/
free_data:
	kfree(cdata);
	return err;
}

static int lsm6ds3_i2c_remove(struct i2c_client *client)
{
	/* TODO: check the function */
	struct lsm6ds3_data *cdata = i2c_get_clientdata(client);

	lsm6ds3_common_remove(cdata);
	dev_info(cdata->dev, "%s: removed\n", LSM6DS3_ACC_GYR_DEV_NAME);
	kfree(cdata);
	return 0;
}

#ifdef CONFIG_PM
static int lsm6ds3_suspend(struct device *dev)
{
	struct lsm6ds3_data *cdata = i2c_get_clientdata(to_i2c_client(dev));

	return lsm6ds3_common_suspend(cdata);
}

static int lsm6ds3_resume(struct device *dev)
{
	struct lsm6ds3_data *cdata = i2c_get_clientdata(to_i2c_client(dev));

	return lsm6ds3_common_resume(cdata);
}

static const struct dev_pm_ops lsm6ds3_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(lsm6ds3_suspend, lsm6ds3_resume)
};

#define LSM6DS3_PM_OPS		(&lsm6ds3_pm_ops)
#else /* CONFIG_PM */
#define LSM6DS3_PM_OPS		NULL
#endif /* CONFIG_PM */


static const struct i2c_device_id lsm6ds3_ids[] = {
	{"lsm6ds3", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, lsm6ds3_ids);

#ifdef CONFIG_OF
static const struct of_device_id lsm6ds3_id_table[] = {
	{.compatible = "st,lsm6ds3", },
	{ },
};
MODULE_DEVICE_TABLE(of, lsm6ds3_id_table);
#endif

static struct i2c_driver lsm6ds3_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = LSM6DS3_ACC_GYR_DEV_NAME,
		.pm = LSM6DS3_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = lsm6ds3_id_table,
#endif
	},
	.probe    = lsm6ds3_i2c_probe,
	.remove   = lsm6ds3_i2c_remove,
	.id_table = lsm6ds3_ids,
};

module_i2c_driver(lsm6ds3_i2c_driver);

MODULE_DESCRIPTION("STMicroelectronics lsm6ds3 i2c driver");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_LICENSE("GPL v2");
