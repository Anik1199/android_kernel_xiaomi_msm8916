/*
 *
 * FocalTech ft5x06 TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
 * Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 * Copyright (C) 2016 XiaoMi, Inc.
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
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/input/ft5346_tran.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <asm/uaccess.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>

#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
/* Early-suspend level */
#define FT_SUSPEND_LEVEL 1
#endif

#if CTP_PROC_INTERFACE
#include "ft5x06_test_lib.h"
#endif


#define FT_DEBUG_DIR_NAME   "ts_debug"

#define TPD_MAX_POINTS_5	5
#define TPD_MAX_POINTS_2	2
#define AUTO_CLB_NEED   1
#define AUTO_CLB_NONEED	 0
struct Upgrade_Info fts_updateinfo[] = {
	{0x55, "FT5x06", TPD_MAX_POINTS_5, AUTO_CLB_NEED, 50, 30, 0x79, 0x03, 1, 2000},
	{0x08, "FT5606", TPD_MAX_POINTS_5, AUTO_CLB_NEED, 50, 30, 0x79, 0x06, 100, 2000},
	{0x0a, "FT5x16", TPD_MAX_POINTS_5, AUTO_CLB_NEED, 50, 30, 0x79, 0x07, 1, 1500},
	{0x05, "FT6208", TPD_MAX_POINTS_2, AUTO_CLB_NONEED, 60, 30, 0x79, 0x05, 10, 2000},
	{0x06, "FT6x06", TPD_MAX_POINTS_2, AUTO_CLB_NONEED, 100, 30, 0x79, 0x08, 10, 2000},
	{0x36, "FT6x36", TPD_MAX_POINTS_2, AUTO_CLB_NONEED, 100, 30, 0x79, 0x18, 10, 2000},
	{0x55, "FT5x06i", TPD_MAX_POINTS_5, AUTO_CLB_NEED, 50, 30, 0x79, 0x03, 1, 2000},
	{0x14, "FT5336", TPD_MAX_POINTS_5, AUTO_CLB_NONEED, 30, 30, 0x79, 0x11, 10, 2000},
	{0x13, "FT3316", TPD_MAX_POINTS_5, AUTO_CLB_NONEED, 30, 30, 0x79, 0x11, 10, 2000},
	{0x12, "FT5436i", TPD_MAX_POINTS_5, AUTO_CLB_NONEED, 30, 30, 0x79, 0x11, 10, 2000},
	{0x11, "FT5336i", TPD_MAX_POINTS_5, AUTO_CLB_NONEED, 30, 30, 0x79, 0x11, 10, 2000},
	{0x54, "FT5x46", TPD_MAX_POINTS_5, AUTO_CLB_NONEED, 2, 2, 0x54, 0x2c, 10, 1350},
	{0x58, "FT5x22", TPD_MAX_POINTS_5, AUTO_CLB_NONEED, 2, 2, 0x58, 0x2c, 20, 2000},
	{0x59, "FT5x26", TPD_MAX_POINTS_5, AUTO_CLB_NONEED, 30, 50, 0x79, 0x10, 1, 2000},

};

#define FT_STORE_TS_INFO(buf, id, name, max_tch, group_id, fw_vkey_support, \
			fw_name, fw_maj, fw_min, fw_sub_min) \
			snprintf(buf, FT_INFO_MAX_LEN, \
				"controller\t= focaltech\n" \
				"model\t\t= 0x%x\n" \
				"name\t\t= %s\n" \
				"max_touches\t= %d\n" \
				"drv_ver\t\t= 0x%x\n" \
				"group_id\t= 0x%x\n" \
				"fw_vkey_support\t= %s\n" \
				"fw_name\t\t= %s\n" \
				"fw_ver\t\t= %d.%d.%d\n", id, name, \
				max_tch, FT_DRIVER_VERSION, group_id, \
				fw_vkey_support, fw_name, fw_maj, fw_min, \
				fw_sub_min)


#if CTP_PROC_INTERFACE
#define CTP_PARENT_PROC_NAME "touchscreen"
#define CTP_VERSION_PROC_NAME "ctp_version"
#define CTP_BINUPDATE_PROC_NAME "ctp_binupdate"
#define CTP_OPENHSORT_PROC_NAME "ctp_openshort_test"
#define CTP_RAWDATA_PROC_NAME "ctp_rawdata"
#define CTP_TESTRESULT_PROC_NAME "ctp_testresult"

unsigned char g_focalscaptested = 0;
static char binfilename[50] = {"\0"};
static int binfilelen;

static struct semaphore g_device_mutex;

static ssize_t ctp_version_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos);
static ssize_t ctp_version_write(struct file *filp, const char __user *userbuf, size_t count, loff_t *ppos);
static const struct file_operations ctp_version_proc_fops = {
	.write = ctp_version_write,
	.read = ctp_version_read,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static ssize_t ctp_binupdate_proc_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos);
static ssize_t ctp_binupdate_proc_write(struct file *filp, const char __user *userbuf, size_t count, loff_t *ppos);
static const struct file_operations ctp_binupdate_procs_fops = {
	.write = ctp_binupdate_proc_write,
	.read = ctp_binupdate_proc_read,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static ssize_t ctp_openshort_proc_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos);
static ssize_t ctp_openshort_proc_write(struct file *filp, const char __user *userbuf, size_t count, loff_t *ppos);
static const struct file_operations ctp_openshort_procs_fops = {
	.write = ctp_openshort_proc_write,
	.read = ctp_openshort_proc_read,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static ssize_t ctp_rawdata_proc_read(struct file *file, char __user *buf, size_t count, loff_t *ppos);
static ssize_t ctp_rawdata_proc_write(struct file *filp, const char __user *userbuf, size_t count, loff_t *ppos);
static const struct file_operations ctp_rawdata_procs_fops = {
	.write = ctp_rawdata_proc_write,
	.read = ctp_rawdata_proc_read,
	.open = simple_open,
	.owner = THIS_MODULE,
};


static ssize_t ctp_testresult_proc_read(struct file *file, char __user *buf, size_t count, loff_t *ppos);
static ssize_t ctp_testresult_proc_write(struct file *filp, const char __user *userbuf, size_t count, loff_t *ppos);
static const struct file_operations ctp_testresult_procs_fops = {
	.write = ctp_testresult_proc_write,
	.read = ctp_testresult_proc_read,
	.open = simple_open,
	.owner = THIS_MODULE,
};

#endif

#if FTS_PROC_APK_DEBUG
#define PROC_UPGRADE			0
#define PROC_READ_REGISTER		1
#define PROC_WRITE_REGISTER	2
#define PROC_RAWDATA			3
#define PROC_AUTOCLB			4

#define PROC_NAME	"ft5x0x-debug"
static unsigned char proc_operate_mode = PROC_RAWDATA;
static struct proc_dir_entry *ft5x0x_proc_entry;
#endif

static u8 is_ic_update_crash;
static struct i2c_client *update_client;


#if CTP_PROC_INTERFACE
struct i2c_client *g_focalclient;
#endif

#if CTP_PROC_INTERFACE
int focal_i2c_Read(unsigned char *writebuf,
			int writelen, unsigned char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				.addr = g_focalclient->addr,
				.flags = 0,
				.len = writelen,
				.buf = writebuf,
			},
			{
				.addr = g_focalclient->addr,
				.flags = I2C_M_RD,
				.len = readlen,
				.buf = readbuf,
			},
		};
		ret = i2c_transfer(g_focalclient->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&g_focalclient->dev, "f%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
				.addr = g_focalclient->addr,
				.flags = I2C_M_RD,
				.len = readlen,
				.buf = readbuf,
			},
		};
		ret = i2c_transfer(g_focalclient->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&g_focalclient->dev, "%s:i2c read error.\n", __func__);
	}
	return ret;
}
/*write data by i2c*/
int focal_i2c_Write(unsigned char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr = g_focalclient->addr,
			.flags = 0,
			.len = writelen,
			.buf = writebuf,
		},
	};

	ret = i2c_transfer(g_focalclient->adapter, msg, 1);
	if (ret < 0)
		dev_err(&g_focalclient->dev, "%s i2c write error.\n", __func__);

	return ret;
}
#endif

static int ft5x06_i2c_read(struct i2c_client *client, char *writebuf,
						   int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = 0,
				.len = writelen,
				.buf = writebuf,
			},
			{
				.addr = client->addr,
				.flags = I2C_M_RD,
				.len = readlen,
				.buf = readbuf,
			},
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "%s: i2c read error.\n", __func__);
	} else {
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = I2C_M_RD,
				.len = readlen,
				.buf = readbuf,
			},
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}
	return ret;
}

static int ft5x06_i2c_write(struct i2c_client *client, char *writebuf,
							int writelen)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = writelen,
			.buf = writebuf,
		},
	};

	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s: i2c write error.\n", __func__);

	return ret;
}

static int ft5x0x_write_reg(struct i2c_client *client, u8 addr, const u8 val)
{
	u8 buf[2] = {0};

	buf[0] = addr;
	buf[1] = val;

	return ft5x06_i2c_write(client, buf, sizeof(buf));
}

static int ft5x0x_read_reg(struct i2c_client *client, u8 addr, u8 *val)
{
	return ft5x06_i2c_read(client, &addr, 1, val, 1);
}

static void ft5x06_update_fw_vendor_id(struct ft5x06_ts_data *data)
{
	struct i2c_client *client = data->client;
	u8 reg_addr;
	int err;

	reg_addr = FT_REG_FW_VENDOR_ID;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &data->fw_vendor_id, 1);
	dev_info(&client->dev, "Vendor Id = %x\n", data->fw_vendor_id);
	if (err < 0)
		dev_err(&client->dev, "fw vendor id read failed");
}

static void ft5x06_update_fw_ver(struct ft5x06_ts_data *data)
{
	struct i2c_client *client = data->client;
	u8 reg_addr;
	int err;

	reg_addr = FT_REG_FW_VER;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &data->fw_ver[0], 1);
	if (err < 0)
		dev_err(&client->dev, "fw major version read failed");

	reg_addr = FT_REG_FW_MIN_VER;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &data->fw_ver[1], 1);
	if (err < 0)
		dev_err(&client->dev, "fw minor version read failed");

	reg_addr = FT_REG_FW_SUB_MIN_VER;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &data->fw_ver[2], 1);
	if (err < 0)
		dev_err(&client->dev, "fw sub minor version read failed");

	CTP_DEBUG("FT:ft5x06_update_fw_ver\n");
	dev_info(&client->dev, "Firmware version = %d.%d.%d\n",
			 data->fw_ver[0], data->fw_ver[1], data->fw_ver[2]);
}

static irqreturn_t ft5x06_ts_interrupt(int irq, void *dev_id)
{
	struct ft5x06_ts_data *data = dev_id;
	struct input_dev *ip_dev;
	int rc, i;
	u32 id, x, y, status, num_touches;
	u8 reg = 0x00, *buf;
	bool update_input = false;

	if (!data) {
		CTP_ERROR("%s: Invalid data\n", __func__);
		return IRQ_HANDLED;
	}

	ip_dev = data->input_dev;
	buf = data->tch_data;

	rc = ft5x06_i2c_read(data->client, &reg, 1,
						 buf, data->tch_data_len);
	if (rc < 0) {
		dev_err(&data->client->dev, "%s: read data fail\n", __func__);
		return IRQ_HANDLED;
	}

	for (i = 0; i < data->pdata->num_max_touches; i++) {
		id = (buf[FT_TOUCH_ID_POS + FT_ONE_TCH_LEN * i]) >> 4;
		if (id >= FT_MAX_ID)
			break;

		update_input = true;

		x = (buf[FT_TOUCH_X_H_POS + FT_ONE_TCH_LEN * i] & 0x0F) << 8 |
			(buf[FT_TOUCH_X_L_POS + FT_ONE_TCH_LEN * i]);
		y = (buf[FT_TOUCH_Y_H_POS + FT_ONE_TCH_LEN * i] & 0x0F) << 8 |
			(buf[FT_TOUCH_Y_L_POS + FT_ONE_TCH_LEN * i]);

		status = buf[FT_TOUCH_EVENT_POS + FT_ONE_TCH_LEN * i] >> 6;

		num_touches = buf[FT_TD_STATUS] & FT_STATUS_NUM_TP_MASK;

		/* invalid combination */
		if (!num_touches && !status && !id)
			break;

		input_mt_slot(ip_dev, id);
		if (status == FT_TOUCH_DOWN || status == FT_TOUCH_CONTACT) {
			input_mt_report_slot_state(ip_dev, MT_TOOL_FINGER, 1);
			input_report_abs(ip_dev, ABS_MT_POSITION_X, x);
			input_report_abs(ip_dev, ABS_MT_POSITION_Y, y);
		} else
			input_mt_report_slot_state(ip_dev, MT_TOOL_FINGER, 0);
	}

	if (update_input) {
		input_mt_report_pointer_emulation(ip_dev, false);
		input_sync(ip_dev);
	}

	return IRQ_HANDLED;
}

static int ft5x06_power_on(struct ft5x06_ts_data *data, bool on)
{
	int rc;

	if (!on)
		goto power_off;

	rc = regulator_enable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
				"Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(data->vdd);
	}

	return rc;

power_off:
	rc = regulator_disable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_disable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
				"Regulator vcc_i2c disable failed rc=%d\n", rc);
		rc = regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
					"Regulator vdd enable failed rc=%d\n", rc);
		}
	}

	return rc;
}

static int ft5x06_power_init(struct ft5x06_ts_data *data, bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;

	data->vdd = regulator_get(&data->client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		rc = PTR_ERR(data->vdd);
		dev_err(&data->client->dev,
				"Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		rc = regulator_set_voltage(data->vdd, FT_VTG_MIN_UV,
								   FT_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
					"Regulator set_vtg failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}

	data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
	if (IS_ERR(data->vcc_i2c)) {
		rc = PTR_ERR(data->vcc_i2c);
		dev_err(&data->client->dev,
				"Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0) {
		rc = regulator_set_voltage(data->vcc_i2c, FT_I2C_VTG_MIN_UV,
								   FT_I2C_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
					"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}

	return 0;

reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);
reg_vdd_set_vtg:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);

	regulator_put(data->vdd);

	if (regulator_count_voltages(data->vcc_i2c) > 0)
		regulator_set_voltage(data->vcc_i2c, 0, FT_I2C_VTG_MAX_UV);

	regulator_put(data->vcc_i2c);
	return 0;
}

static int ft5x06_ts_pinctrl_init(struct ft5x06_ts_data *ft5x06_data)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	ft5x06_data->ts_pinctrl = devm_pinctrl_get(&(ft5x06_data->client->dev));
	if (IS_ERR_OR_NULL(ft5x06_data->ts_pinctrl)) {
		dev_dbg(&ft5x06_data->client->dev,
				"Target does not use pinctrl\n");
		retval = PTR_ERR(ft5x06_data->ts_pinctrl);
		ft5x06_data->ts_pinctrl = NULL;
		return retval;
	}

	ft5x06_data->gpio_state_active
		= pinctrl_lookup_state(ft5x06_data->ts_pinctrl,
							   "pmx_ts_active");
	if (IS_ERR_OR_NULL(ft5x06_data->gpio_state_active)) {
		dev_dbg(&ft5x06_data->client->dev,
				"Can not get ts default pinstate\n");
		retval = PTR_ERR(ft5x06_data->gpio_state_active);
		ft5x06_data->ts_pinctrl = NULL;
		return retval;
	}

	ft5x06_data->gpio_state_suspend
		= pinctrl_lookup_state(ft5x06_data->ts_pinctrl,
							   "pmx_ts_suspend");
	if (IS_ERR_OR_NULL(ft5x06_data->gpio_state_suspend)) {
		dev_err(&ft5x06_data->client->dev,
				"Can not get ts sleep pinstate\n");
		retval = PTR_ERR(ft5x06_data->gpio_state_suspend);
		ft5x06_data->ts_pinctrl = NULL;
		return retval;
	}

	return 0;
}

static int ft5x06_ts_pinctrl_select(struct ft5x06_ts_data *ft5x06_data,
									bool on)
{
	struct pinctrl_state *pins_state;
	int ret;

	pins_state = on ? ft5x06_data->gpio_state_active
				 : ft5x06_data->gpio_state_suspend;
	if (!IS_ERR_OR_NULL(pins_state)) {
		ret = pinctrl_select_state(ft5x06_data->ts_pinctrl, pins_state);
		if (ret) {
			dev_err(&ft5x06_data->client->dev,
					"can not set %s pins\n",
					on ? "pmx_ts_active" : "pmx_ts_suspend");
			return ret;
		}
	} else {
		dev_err(&ft5x06_data->client->dev,
				"not a valid '%s' pinstate\n",
				on ? "pmx_ts_active" : "pmx_ts_suspend");
	}

	return 0;
}


#ifdef CONFIG_PM
static int ft5x06_ts_suspend(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	char txbuf[2], i;
	int err;

	if (data->loading_fw) {
		dev_info(dev, "Firmware loading in process...\n");
		return 0;
	}

	if (data->suspended) {
		dev_info(dev, "Already in suspend state\n");
		return 0;
	}

	disable_irq(data->client->irq);

	/* release all touches */
	for (i = 0; i < data->pdata->num_max_touches; i++) {
		input_mt_slot(data->input_dev, i);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
	}
	input_mt_report_pointer_emulation(data->input_dev, false);
	input_sync(data->input_dev);

	if (gpio_is_valid(data->pdata->reset_gpio)) {
		txbuf[0] = FT_REG_PMODE;
		txbuf[1] = FT_PMODE_HIBERNATE;
		err = ft5x06_i2c_write(data->client, txbuf, sizeof(txbuf));

		msleep(data->pdata->hard_rst_dly);
	}


	if (data->pdata->power_on) {
		err = data->pdata->power_on(false);
		if (err) {
			dev_err(dev, "power off failed");
			goto pwr_off_fail;
		}
	} else {
		err = ft5x06_power_on(data, false);
		if (err) {
			dev_err(dev, "power off failed");
			goto pwr_off_fail;
		}
	}

	data->suspended = true;

	return 0;

pwr_off_fail:
	if (gpio_is_valid(data->pdata->reset_gpio)) {
		gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
		msleep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
	}
	enable_irq(data->client->irq);
	return err;

}

static int ft5x06_ts_resume(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	int err;

	if (!data->suspended) {
		dev_dbg(dev, "Already in awake state\n");
		return 0;
	}


	if (data->pdata->power_on) {
		err = data->pdata->power_on(true);
		if (err) {
			dev_err(dev, "power on failed");
			return err;
		}
	} else {
		err = ft5x06_power_on(data, true);
		if (err) {
			dev_err(dev, "power on failed");
			return err;
		}
	}

	if (gpio_is_valid(data->pdata->reset_gpio)) {
		gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
		msleep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
	}

	msleep(data->pdata->soft_rst_dly);

	enable_irq(data->client->irq);

	data->suspended = false;

	return 0;
}

static const struct dev_pm_ops ft5x06_ts_pm_ops = {
#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
	.suspend = ft5x06_ts_suspend,
	.resume = ft5x06_ts_resume,
#endif
};

#else
static int ft5x06_ts_suspend(struct device *dev)
{
	return 0;
}

static int ft5x06_ts_resume(struct device *dev)
{
	return 0;
}

#endif

#if defined(CONFIG_FB)

static void fb_notify_resume_work(struct work_struct *work)
{
	   struct ft5x06_ts_data *ft5x06_data =
			   container_of(work, struct ft5x06_ts_data, fb_notify_work);
	   ft5x06_ts_resume(&ft5x06_data->client->dev);
}
static int fb_notifier_callback(struct notifier_block *self,
								unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct ft5x06_ts_data *ft5x06_data =
		container_of(self, struct ft5x06_ts_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
		ft5x06_data && ft5x06_data->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
		   schedule_work(&ft5x06_data->fb_notify_work);
		 else if (*blank == FB_BLANK_POWERDOWN) {
			flush_work(&ft5x06_data->fb_notify_work);
			ft5x06_ts_suspend(&ft5x06_data->client->dev);
		}
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void ft5x06_ts_early_suspend(struct early_suspend *handler)
{
	struct ft5x06_ts_data *data = container_of(handler,
								  struct ft5x06_ts_data,
								  early_suspend);

	ft5x06_ts_suspend(&data->client->dev);
}

static void ft5x06_ts_late_resume(struct early_suspend *handler)
{
	struct ft5x06_ts_data *data = container_of(handler,
								  struct ft5x06_ts_data,
								  early_suspend);

	ft5x06_ts_resume(&data->client->dev);
}
#endif


int hid_to_i2c(struct i2c_client *client)
{
	u8 auc_i2c_write_buf[5] = {0};
	int bRet = 0;
#ifdef HIDTOI2C_DISABLE

	return 2;

#endif
	auc_i2c_write_buf[0] = 0xeb;
	auc_i2c_write_buf[1] = 0xaa;
	auc_i2c_write_buf[2] = 0x09;
	ft5x06_i2c_write(client, auc_i2c_write_buf, 3);
	msleep(10);
	auc_i2c_write_buf[0] = auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = 0;
	ft5x06_i2c_read(client, auc_i2c_write_buf, 0, auc_i2c_write_buf, 3);
	CTP_DEBUG("auc_i2c_write_buf[0]:%x, auc_i2c_write_buf[1]:%x, auc_i2c_write_buf[2]:%x\n", auc_i2c_write_buf[0], auc_i2c_write_buf[1], auc_i2c_write_buf[2]);
	if (0xeb == auc_i2c_write_buf[0] && 0xaa == auc_i2c_write_buf[1] && 0x08 == auc_i2c_write_buf[2])
		bRet = 1;
	else bRet = 0;
	return bRet;
}

static int ft5x06_fw_upgrade_start(struct i2c_client *client,
								   const u8 *data, u32 data_len)
{
	struct ft5x06_ts_data *ts_data = i2c_get_clientdata(client);
	struct fw_upgrade_info info = ts_data->pdata->info;

	u8 reg_addr;
	u8 chip_id = 0x00;
	u8 w_buf[FT_MAX_WR_BUF] = {0}, r_buf[FT_MAX_RD_BUF] = {0};
	u8 pkt_buf[FT_FW_PKT_LEN + FT_FW_PKT_META_LEN];
	int i, j, temp;
	u32 pkt_num, pkt_len;

	u8 fw_ecc;
	int i_ret;

#if 1
	reg_addr = FT_REG_ID;
	temp = ft5x06_i2c_read(client, &reg_addr, 1, &chip_id, 1);
	CTP_DEBUG("Update:Read ic info:%x\n", chip_id);
	if (temp < 0)
		dev_err(&client->dev, "version read failed");

	if (is_ic_update_crash)
		 chip_id = CTP_IC_TYPE_2 ;

	for (i = 0; i < sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info); i++) {
		if (chip_id == fts_updateinfo[i].CHIP_ID) {
			info.auto_cal = fts_updateinfo[i].AUTO_CLB;
			info.delay_55 = fts_updateinfo[i].delay_55;
			info.delay_aa = fts_updateinfo[i].delay_aa;
			info.delay_erase_flash = fts_updateinfo[i].delay_earse_flash;
			info.delay_readid = fts_updateinfo[i].delay_readid;
			info.upgrade_id_1 = fts_updateinfo[i].upgrade_id_1;
			info.upgrade_id_2 = fts_updateinfo[i].upgrade_id_2;

			break;
		}
	}

	ts_data->family_id = chip_id;

	if (i >= sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info)) {
		info.auto_cal = fts_updateinfo[11].AUTO_CLB;
		info.delay_55 = fts_updateinfo[11].delay_55;
		info.delay_aa = fts_updateinfo[11].delay_aa;
		info.delay_erase_flash = fts_updateinfo[11].delay_earse_flash;
		info.delay_readid = fts_updateinfo[11].delay_readid;
		info.upgrade_id_1 = fts_updateinfo[11].upgrade_id_1;
		info.upgrade_id_2 = fts_updateinfo[11].upgrade_id_2;
	}

#endif

	dev_err(&client->dev, "id1 = 0x%x id2 = 0x%x family_id=0x%x, data_len = %d\n",
			info.upgrade_id_1, info.upgrade_id_2, ts_data->family_id, data_len);
	/* determine firmware size */
	i_ret = hid_to_i2c(client);

	if (i_ret == 0)
		CTP_DEBUG("[FTS] hid change to i2c fail ! \n");

	for (i = 0, j = 0; i < FT_UPGRADE_LOOP; i++) {
		msleep(FT_EARSE_DLY_MS);
		ft5x0x_write_reg(client, 0xfc, FT_UPGRADE_AA);
		msleep(info.delay_aa);


		ft5x0x_write_reg(client, 0xfc, FT_UPGRADE_55);
		msleep(200);
		i_ret = hid_to_i2c(client);
		if (i_ret == 0)
			CTP_DEBUG("[FTS] hid change to i2c fail ! \n");

		msleep(10);
		/* Enter upgrade mode */
		w_buf[0] = FT_UPGRADE_55;
		ft5x06_i2c_write(client, &w_buf[0], 1);
		usleep(FT_55_AA_DLY_NS);
		w_buf[0] = FT_UPGRADE_AA;
		ft5x06_i2c_write(client, &w_buf[0], 1);
		if (i_ret < 0) {
			CTP_DEBUG("[FTS] failed writing  0x55 and 0xaa ! \n");
			continue;
		}


		/* check READ_ID */
		msleep(info.delay_readid);
		w_buf[0] = FT_READ_ID_REG;
		w_buf[1] = 0x00;
		w_buf[2] = 0x00;
		w_buf[3] = 0x00;

		ft5x06_i2c_read(client, w_buf, 4, r_buf, 2);


		CTP_DEBUG("r_buf[0] :%X, r_buf[1]: %X\n", r_buf[0], r_buf[1]);
		if (r_buf[0] != info.upgrade_id_1
			|| r_buf[1] != info.upgrade_id_2) {
			dev_err(&client->dev, "Upgrade ID mismatch(%d), IC=0x%x 0x%x, info=0x%x 0x%x\n",
					i, r_buf[0], r_buf[1],
					info.upgrade_id_1, info.upgrade_id_2);
		} else
			break;
	}
	CTP_DEBUG("Begin to update \n\n");
	if (i >= FT_UPGRADE_LOOP) {
		dev_err(&client->dev, "Abort upgrade\n");
		return -EIO;
	}

	/* erase app and panel paramenter area */
	CTP_DEBUG("Step 4:erase app and panel paramenter area\n");
	w_buf[0] = FT_ERASE_APP_REG;
	ft5x06_i2c_write(client, w_buf, 1);
	msleep(1350);

	for (i = 0; i < 15; i++) {
			w_buf[0] = 0x6a;
			r_buf[0] = r_buf[1] = 0x00;
			ft5x06_i2c_read(client, w_buf, 1, r_buf, 2);

			if (0xF0 == r_buf[0] && 0xAA == r_buf[1])
				break;
			msleep(50);

		}

		w_buf[0] = 0xB0;
		w_buf[1] = (u8) ((data_len >> 16) & 0xFF);
		w_buf[2] = (u8) ((data_len >> 8) & 0xFF);
		w_buf[3] = (u8) (data_len & 0xFF);

		ft5x06_i2c_write(client, w_buf, 4);

	/* program firmware */
	CTP_DEBUG("Step 5:program firmware\n");
	pkt_num = (data_len) / FT_FW_PKT_LEN;
	pkt_buf[0] = FT_FW_START_REG;
	pkt_buf[1] = 0x00;
	fw_ecc = 0;
	temp = 0;

	for (j = 0; j < pkt_num; j++) {
		temp = j * FT_FW_PKT_LEN;
		pkt_buf[2] = (u8) (temp >> 8);
		pkt_buf[3] = (u8) temp;
		pkt_len = FT_FW_PKT_LEN;
		pkt_buf[4] = (u8) (pkt_len >> 8);
		pkt_buf[5] = (u8) pkt_len;

		for (i = 0; i < FT_FW_PKT_LEN; i++) {
			pkt_buf[6 + i] = data[j * FT_FW_PKT_LEN + i];
			fw_ecc ^= pkt_buf[6 + i];
		}

		ft5x06_i2c_write(client, pkt_buf, FT_FW_PKT_LEN + 6);

		for (i = 0; i < 30; i++) {
			w_buf[0] = 0x6a;
			r_buf[0] = r_buf[1] = 0x00;
			ft5x06_i2c_read(client, w_buf, 1, r_buf, 2);

			if ((j + 0x1000) == (((r_buf[0]) << 8) | r_buf[1]))
				break;
			msleep(1);
		}
	}

	/* send remaining bytes */
	CTP_DEBUG("Step 6:send remaining bytes\n");
	if ((data_len) % FT_FW_PKT_LEN > 0) {
		temp = pkt_num * FT_FW_PKT_LEN;
		pkt_buf[2] = (u8) (temp >> 8);
		pkt_buf[3] = (u8) temp;
		temp = (data_len) % FT_FW_PKT_LEN;
		pkt_buf[4] = (u8) (temp >> 8);
		pkt_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) {
			pkt_buf[6 + i] = data[pkt_num * FT_FW_PKT_LEN + i];
			fw_ecc ^= pkt_buf[6 + i];
		}

		ft5x06_i2c_write(client, pkt_buf, temp + 6);

		for (i = 0;i < 30; i++) {
			w_buf[0] = 0x6a;
			r_buf[0] = r_buf[1] = 0x00;
			ft5x06_i2c_read(client, w_buf, 1, r_buf, 2);

			if ((j + 0x1000) == (((r_buf[0]) << 8) | r_buf[1]))
				break;
			msleep(1);
		}
	}

	msleep(50);

		/*********Step 6: read out checksum***********************/
		/*send the opration head */
		CTP_DEBUG("Step 7: read out checksum\n");
		w_buf[0] = 0x64;
		ft5x06_i2c_write(client, w_buf, 1);
		msleep(300);

		temp = 0;
		w_buf[0] = 0x65;
		w_buf[1] = (u8)(temp >> 16);
		w_buf[2] = (u8)(temp >> 8);
		w_buf[3] = (u8)(temp);
		temp = data_len;
		w_buf[4] = (u8)(temp >> 8);
		w_buf[5] = (u8)(temp);
		i_ret = ft5x06_i2c_write(client, w_buf, 6);
		msleep(data_len/256);

		for (i = 0; i < 100; i++) {
		w_buf[0] = 0x6a;
		r_buf[0] = r_buf[1] = 0x00;
		ft5x06_i2c_read(client, w_buf, 1, r_buf, 2);

		if (0xF0 == r_buf[0] && 0x55 == r_buf[1])
			break;
		msleep(1);

		}
		w_buf[0] = 0x66;
		ft5x06_i2c_read(client, w_buf, 1, r_buf, 1);
		if (r_buf[0] != fw_ecc) {
			dev_err(&client->dev, "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n",
			r_buf[0],
			fw_ecc);

		return -EIO;
		}

	/* reset */
	CTP_DEBUG("Step 8: reset \n");
	w_buf[0] = FT_REG_RESET_FW;
	ft5x06_i2c_write(client, w_buf, 1);
	msleep(ts_data->pdata->soft_rst_dly);

	CTP_DEBUG("Firmware upgrade successful\n");

	return 0;
}

#if TPD_AUTO_UPGRADE
/*lenovo project*/
static unsigned char CTPM_FW1[] = {
#include "ft_app_ic_chuanying_ofilm.txt"
};

static unsigned char CTPM_FW2[] = {

};


u8 fts_ctpm_update_project_setting(struct i2c_client *client)
{
	u8 buf[128];
	u8 w_buf[4], r_buf[2];
	u32 i = 0, j = 0;
		struct ft5x06_ts_data *ts_data = i2c_get_clientdata(client);

	for (i = 0, j = 0; i < FT_UPGRADE_LOOP; i++) {
		msleep(FT_EARSE_DLY_MS);

		if (gpio_is_valid(ts_data->pdata->reset_gpio)) {
			gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 0);
			msleep(ts_data->pdata->hard_rst_dly);
			gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 1);
		}

		if (i <= (FT_UPGRADE_LOOP / 2))
			msleep(30 + i * 3);
		else
			msleep(30 - (i - (FT_UPGRADE_LOOP / 2)) * 2);

		/* Enter upgrade mode */
			w_buf[0] = FT_UPGRADE_55;
		ft5x06_i2c_write(client, &w_buf[0], 1);
		usleep(FT_55_AA_DLY_NS);
			w_buf[0] = FT_UPGRADE_AA;
		ft5x06_i2c_write(client, &w_buf[0], 1);

		/* check READ_ID */
		msleep(10);
		w_buf[0] = FT_READ_ID_REG;
		w_buf[1] = 0x00;
		w_buf[2] = 0x00;
		w_buf[3] = 0x00;

		ft5x06_i2c_read(client, w_buf, 4, r_buf, 2);

		if (r_buf[0] != 0x79 || r_buf[1] != 0x11)
			continue;
		else
			break;
	}
	/*--------- read current project setting  ---------- */
	/*set read start address */

	buf[0] = 0x03;
	buf[1] = 0x00;
	buf[2] = (u8)(0x07b0 >> 8);
	buf[3] = (u8)(0x07b0);

	ft5x06_i2c_read(client, buf, 4, buf, 128);
	msleep(10);

	ft5x0x_write_reg(client, 0xfc, 0xaa);
	msleep(30);
	ft5x0x_write_reg(client, 0xfc, 0x55);
	msleep(200);

	is_ic_update_crash = 1;
	return buf[4];
}

int fts_ctpm_fw_upgrade_with_i_file(struct ft5x06_ts_data *data)
{
	struct i2c_client *client = data->client;
	int  flag_TPID = 0;
	u8 *pbt_buf = 0x0;
	int rc = 0, fw_len = 0;
	u8 uc_host_fm_ver, uc_tp_fm_ver, vendor_id, ic_type;
	u8 reg_addr;

	reg_addr = 0xA6;
	ft5x06_i2c_read(client, &reg_addr, 1, &uc_tp_fm_ver, 1);
	reg_addr = 0xA8;
	ft5x06_i2c_read(client, &reg_addr, 1, &vendor_id, 1);
	reg_addr = 0xA3;
	ft5x06_i2c_read(client, &reg_addr, 1, &ic_type, 1);

	CTP_DEBUG("ic_type:%d, uc_tp_fm_ver:%d, vendor_id:%d, vendor_id:%d", ic_type, uc_tp_fm_ver, vendor_id, vendor_id);
	if ((ic_type != CTP_IC_TYPE_2)) {
		CTP_ERROR("IC type dismatch, please check");
	}

	if (vendor_id == 0xA8 || vendor_id == 0x00 || ic_type == 0xA3 || ic_type == 0x00) {
		CTP_ERROR("vend_id read error, need project");
		vendor_id = fts_ctpm_update_project_setting(client);
		flag_TPID = 1;
	}

	if (vendor_id == VENDOR_O_FILM) {
		pbt_buf = CTPM_FW1;
		fw_len = sizeof(CTPM_FW1);
		CTP_DEBUG("update firmware size:%d", fw_len);
	} else if (vendor_id == VENDOR_MUTTO) {
		pbt_buf = CTPM_FW2;
		fw_len = sizeof(CTPM_FW2);
		CTP_DEBUG("update firmware size:%d", fw_len);
	} else {
		CTP_ERROR("read vendor_id fail");
		return -EPERM;
	}

	CTP_DEBUG("update firmware size:%d", fw_len);

	if (sizeof(CTPM_FW1) < 8 || sizeof(CTPM_FW1) > 54  * 1024) {
		CTP_ERROR(" FW length error\n");
		return -EPERM;
	}

	if ((pbt_buf[fw_len - 8] ^ pbt_buf[fw_len - 6]) == 0xFF
		&& (pbt_buf[fw_len - 7] ^ pbt_buf[fw_len - 5]) == 0xFF
		&& (pbt_buf[fw_len - 3] ^ pbt_buf[fw_len - 4]) == 0xFF) {

		if (vendor_id != pbt_buf[fw_len-1]) {
			CTP_ERROR("vendor_id dismatch, ic:%x, file:%x", vendor_id, pbt_buf[fw_len-1]);
			return -EPERM;
		}

		uc_host_fm_ver = pbt_buf[fw_len - 2];
		CTP_DEBUG("[FTS] uc_tp_fm_ver = %d.\n", uc_tp_fm_ver);
		CTP_DEBUG("[FTS] uc_host_fm_ver = %d.\n", uc_host_fm_ver);

		if ((uc_tp_fm_ver < uc_host_fm_ver) || (is_ic_update_crash == 1)) {
			rc = ft5x06_fw_upgrade_start(update_client, pbt_buf, fw_len);
			if (rc != 0)
				CTP_DEBUG("[FTS]  upgrade failed rc = %d.\n", rc);
			else
				CTP_DEBUG("[FTS] upgrade successfully.\n");
		}
	}

	return rc;
}
#endif

#if CTP_SYS_APK_UPDATE
static ssize_t ft5x06_fw_name_show(struct device *dev,
								   struct device_attribute *attr, char *buf)
{
	u8 fw_version = 0x00;

	ft5x0x_read_reg(update_client, FT5x0x_REG_FW_VER, &fw_version);

	return sprintf(buf, "firmware version %02X\n", fw_version);
}

static ssize_t ft5x06_fw_name_store(struct device *dev,
									struct device_attribute *attr,
									const char *buf, size_t size)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	if (size > FT_FW_NAME_MAX_LEN - 1)
		return -EINVAL;

	strlcpy(data->fw_name, buf, size);
	if (data->fw_name[size-1] == '\n')
		data->fw_name[size-1] = 0;

	return size;
}

static DEVICE_ATTR(fw_name, 0664, ft5x06_fw_name_show, ft5x06_fw_name_store);

static int ft5x06_auto_cal(struct i2c_client *client)
{
	struct ft5x06_ts_data *data = i2c_get_clientdata(client);
	u8 temp = 0, i;

	/* set to factory mode */
	msleep(2 * data->pdata->soft_rst_dly);
	ft5x0x_write_reg(client, FT_REG_DEV_MODE, FT_FACTORYMODE_VALUE);
	msleep(data->pdata->soft_rst_dly);

	/* start calibration */
	ft5x0x_write_reg(client, FT_DEV_MODE_REG_CAL, FT_CAL_START);
	msleep(2 * data->pdata->soft_rst_dly);
	for (i = 0; i < FT_CAL_RETRY; i++) {
		ft5x0x_read_reg(client, FT_REG_CAL, &temp);
		/*return to normal mode, calibration finish */
		if (((temp & FT_CAL_MASK) >> FT_4BIT_SHIFT) == FT_CAL_FIN)
			break;
	}

	/*calibration OK */
	msleep(2 * data->pdata->soft_rst_dly);
	ft5x0x_write_reg(client, FT_REG_DEV_MODE, FT_FACTORYMODE_VALUE);
	msleep(data->pdata->soft_rst_dly);

	/* store calibration data */
	ft5x0x_write_reg(client, FT_DEV_MODE_REG_CAL, FT_CAL_STORE);
	msleep(2 * data->pdata->soft_rst_dly);

	/* set to normal mode */
	ft5x0x_write_reg(client, FT_REG_DEV_MODE, FT_WORKMODE_VALUE);
	msleep(2 * data->pdata->soft_rst_dly);

	return 0;
}


static int ft5x06_fw_upgrade(struct device *dev, bool force)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	const struct firmware *fw = NULL;
	int rc;
	u8 fw_file_maj, fw_file_min, fw_file_sub_min, fw_file_vendor_id;
	bool fw_upgrade = false;

	if (data->suspended) {
		dev_err(dev, "Device is in suspend state: Exit FW upgrade\n");
		return -EBUSY;
	}

	rc = request_firmware(&fw, data->fw_name, dev);
	if (rc < 0) {
		dev_err(dev, "Request firmware failed - %s (%d)\n",
						data->fw_name, rc);
		return rc;
	}

	if (fw->size < FT_FW_MIN_SIZE || fw->size > FT_FW_MAX_SIZE) {
		dev_err(dev, "Invalid firmware size (%zu)\n", fw->size);
		rc = -EIO;
		goto rel_fw;
	}

	if (data->family_id == FT6X36_ID) {
		fw_file_maj = FT_FW_FILE_MAJ_VER_FT6X36(fw);
		fw_file_vendor_id = FT_FW_FILE_VENDOR_ID_FT6X36(fw);
	} else {
		fw_file_maj = FT_FW_FILE_MAJ_VER(fw);
		fw_file_vendor_id = FT_FW_FILE_VENDOR_ID(fw);
	}
	fw_file_min = FT_FW_FILE_MIN_VER(fw);
	fw_file_sub_min = FT_FW_FILE_SUB_MIN_VER(fw);

	dev_info(dev, "Current firmware: %d.%d.%d", data->fw_ver[0],
				data->fw_ver[1], data->fw_ver[2]);
	dev_info(dev, "New firmware: %d.%d.%d", fw_file_maj,
				fw_file_min, fw_file_sub_min);

	if (force)
		fw_upgrade = true;
	else if ((data->fw_ver[0] < fw_file_maj) &&
		data->fw_vendor_id == fw_file_vendor_id)
		fw_upgrade = true;

	if (!fw_upgrade) {
		dev_info(dev, "Exiting fw upgrade...\n");
		rc = -EFAULT;
		goto rel_fw;
	}

	/* start firmware upgrade */
	if (FT_FW_CHECK(fw, data)) {
		rc = ft5x06_fw_upgrade_start(data->client, fw->data, fw->size);
		if (rc < 0)
			dev_err(dev, "update failed (%d). try later...\n", rc);
		else if (data->pdata->info.auto_cal)
			ft5x06_auto_cal(data->client);
	} else {
		dev_err(dev, "FW format error\n");
		rc = -EIO;
	}

	ft5x06_update_fw_ver(data);

	FT_STORE_TS_INFO(data->ts_info, data->family_id, data->pdata->name,
			data->pdata->num_max_touches, data->pdata->group_id,
			data->pdata->fw_vkey_support ? "yes" : "no",
			data->pdata->fw_name, data->fw_ver[0],
			data->fw_ver[1], data->fw_ver[2]);
rel_fw:
	release_firmware(fw);
	return rc;
}


static ssize_t ft5x06_update_fw_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	return snprintf(buf, 2, "%d\n", data->loading_fw);
}

static ssize_t ft5x06_update_fw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	unsigned long val;
	int rc;

	if (size > 2)
		return -EINVAL;

	rc = kstrtoul(buf, 10, &val);
	if (rc != 0)
		return rc;

	if (data->suspended) {
		dev_info(dev, "In suspend state, try again later...\n");
		return size;
	}

	mutex_lock(&data->input_dev->mutex);
	if (!data->loading_fw  && val) {
		data->loading_fw = true;
		ft5x06_fw_upgrade(dev, false);
		data->loading_fw = false;
	}
	mutex_unlock(&data->input_dev->mutex);

	return size;
}

static DEVICE_ATTR(update_fw, 0664, ft5x06_update_fw_show,
				ft5x06_update_fw_store);

static ssize_t ft5x06_force_update_fw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	unsigned long val;
	int rc;

	if (size > 2)
		return -EINVAL;

	rc = kstrtoul(buf, 10, &val);
	if (rc != 0)
		return rc;

	mutex_lock(&data->input_dev->mutex);
	if (!data->loading_fw  && val) {
		data->loading_fw = true;
		ft5x06_fw_upgrade(dev, true);
		data->loading_fw = false;
	}
	mutex_unlock(&data->input_dev->mutex);

	return size;
}

static DEVICE_ATTR(force_update_fw, 0664, ft5x06_update_fw_show,
				ft5x06_force_update_fw_store);


#define FT_DEBUG_DIR_NAME	"ts_debug"

static bool ft5x06_debug_addr_is_valid(int addr)
{
	if (addr < 0 || addr > 0xFF) {
		pr_err("FT reg address is invalid: 0x%x\n", addr);
		return false;
	}

	return true;
}

static int ft5x06_debug_data_set(void *_data, u64 val)
{
	struct ft5x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (ft5x06_debug_addr_is_valid(data->addr))
		dev_info(&data->client->dev,
			"Writing into FT registers not supported\n");

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

static int ft5x06_debug_data_get(void *_data, u64 *val)
{
	struct ft5x06_ts_data *data = _data;
	int rc;
	u8 reg;

	mutex_lock(&data->input_dev->mutex);

	if (ft5x06_debug_addr_is_valid(data->addr)) {
		rc = ft5x0x_read_reg(data->client, data->addr, &reg);
		if (rc < 0)
			dev_err(&data->client->dev,
				"FT read register 0x%x failed (%d)\n",
				data->addr, rc);
		else
			*val = reg;
	}

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_data_fops, ft5x06_debug_data_get,
			ft5x06_debug_data_set, "0x%02llX\n");

static int ft5x06_debug_addr_set(void *_data, u64 val)
{
	struct ft5x06_ts_data *data = _data;

	if (ft5x06_debug_addr_is_valid(val)) {
		mutex_lock(&data->input_dev->mutex);
		data->addr = val;
		mutex_unlock(&data->input_dev->mutex);
	}

	return 0;
}

static int ft5x06_debug_addr_get(void *_data, u64 *val)
{
	struct ft5x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (ft5x06_debug_addr_is_valid(data->addr))
		*val = data->addr;

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_addr_fops, ft5x06_debug_addr_get,
			ft5x06_debug_addr_set, "0x%02llX\n");

static int ft5x06_debug_suspend_set(void *_data, u64 val)
{
	struct ft5x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (val)
		ft5x06_ts_suspend(&data->client->dev);
	else
		ft5x06_ts_resume(&data->client->dev);

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

static int ft5x06_debug_suspend_get(void *_data, u64 *val)
{
	struct ft5x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);
	*val = data->suspended;
	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_suspend_fops, ft5x06_debug_suspend_get,
			ft5x06_debug_suspend_set, "%lld\n");

static int ft5x06_debug_dump_info(struct seq_file *m, void *v)
{
	struct ft5x06_ts_data *data = m->private;

	seq_printf(m, "%s\n", data->ts_info);

	return 0;
}

static int debugfs_dump_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, ft5x06_debug_dump_info, inode->i_private);
}

static const struct file_operations debug_dump_info_fops = {
	.owner		= THIS_MODULE,
	.open		= debugfs_dump_info_open,
	.read		= seq_read,
	.release	= single_release,
};

#endif

static int ft5x0x_GetFirmwareSize(char *firmware_name)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];
	memset(filepath, 0, sizeof(filepath));

	sprintf(filepath, "%s", firmware_name);
	CTP_ERROR("filepath=%s\n", filepath);
	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		CTP_ERROR("error occured while opening file %s.\n", filepath);
		return -EPERM;
	}
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL);
	return fsize;
}

static int ft5x0x_ReadFirmware(char *firmware_name, unsigned char *firmware_buf)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	char filepath[128];
	loff_t pos;
	mm_segment_t old_fs;

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s", firmware_name);
	CTP_INFO("filepath=%s\n", filepath);
	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		CTP_ERROR("error occured while opening file %s.\n", filepath);
		return -EPERM;
	}
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;

	vfs_read(pfile, firmware_buf, fsize, &pos);

	filp_close(pfile, NULL);
	set_fs(old_fs);
	return 0;
}

void delay_qt_ms(unsigned long  w_ms)
{
	unsigned long i;
	unsigned long j;

	for (i = 0; i < w_ms; i++) {
		for (j = 0; j < 1000; j++)
			udelay(1);
	}
}

int fts_ctpm_auto_clb(void)
{
	unsigned char uc_temp;
	unsigned char i ;

	printk("[FTS] start auto CLB.\n");
	msleep(200);
	ft5x0x_write_reg(update_client, 0, 0x40);
	delay_qt_ms(100);
	ft5x0x_write_reg(update_client, 2, 0x4);
	delay_qt_ms(300);
	for (i = 0; i < 100; i++) {
		ft5x0x_read_reg(update_client, 0, &uc_temp);
		if (((uc_temp&0x70)>>4) == 0x0)
			break;
		delay_qt_ms(200);
		printk("[FTS] waiting calibration %d\n", i);
	}

	printk("[FTS] calibration OK.\n");

	msleep(300);
	ft5x0x_write_reg(update_client, 0, 0x40);
	delay_qt_ms(100);
	ft5x0x_write_reg(update_client, 2, 0x5);
	delay_qt_ms(300);
	ft5x0x_write_reg(update_client, 0, 0x0);
	msleep(300);
	printk("[FTS] store CLB result OK.\n");
	return 0;
}


int fts_ctpm_fw_upgrade_with_app_file(char *firmware_name)
{
	unsigned char *pbt_buf = NULL;
	int i_ret;
	u8 fwver;
	int fwsize = ft5x0x_GetFirmwareSize(firmware_name);

	CTP_DEBUG("enter fw_upgrade_with_app_file");
	if (fwsize <= 0) {
		CTP_ERROR("%s ERROR:Get firmware size failed\n", __FUNCTION__);
		return -EPERM;
	}

	pbt_buf = (unsigned char *) kmalloc(fwsize+1, GFP_ATOMIC);
	if (ft5x0x_ReadFirmware(firmware_name, pbt_buf)) {
		CTP_ERROR("%s() - ERROR: request_firmware failed\n", __FUNCTION__);
		kfree(pbt_buf);
		return -EPERM;
	}

	if ((pbt_buf[fwsize - 8] ^ pbt_buf[fwsize - 6]) != 0xFF
		|| (pbt_buf[fwsize - 7] ^ pbt_buf[fwsize - 5]) != 0xFF
		|| (pbt_buf[fwsize - 3] ^ pbt_buf[fwsize - 4]) != 0xFF) {
		CTP_ERROR("the update file is not correct, please check\n");
	CTP_ERROR("checksum is %2x, %2x, %2x, %2x, %2x, %2x", pbt_buf[fwsize - 8], pbt_buf[fwsize - 6], pbt_buf[fwsize - 7], pbt_buf[fwsize - 5], pbt_buf[fwsize - 3], pbt_buf[fwsize - 4]);
	return -EPERM;
	}
	/*call the upgrade function*/
	i_ret =  ft5x06_fw_upgrade_start(update_client, pbt_buf, fwsize);
	if (i_ret != 0) {
		CTP_ERROR("%s() - ERROR:[FTS] upgrade failed i_ret = %d.\n", __FUNCTION__, i_ret);


	} else {
		CTP_INFO("[FTS] upgrade successfully.\n");
		if (ft5x0x_read_reg(update_client, FT5x0x_REG_FW_VER, &fwver) >= 0)
			CTP_INFO("the new fw ver is 0x%02x\n", fwver);

	}
	kfree(pbt_buf);
	return i_ret;
}

#if CTP_SYS_APK_UPDATE

static ssize_t ft5x0x_fwupgradeapp_show(struct device *dev,
										struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return -EPERM;
}
#endif

#if (CTP_PROC_INTERFACE || CTP_SYS_APK_UPDATE)

static ssize_t ft5x0x_fwupgradeapp_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{


	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	char fwname[128];
	memset(fwname, 0, sizeof(fwname));
	sprintf(fwname, "%s", buf);
	fwname[count-1] = '\0';

	disable_irq(client->irq);

	fts_ctpm_fw_upgrade_with_app_file(fwname);

	enable_irq(client->irq);

	return count;
}

#endif

#if CTP_SYS_APK_UPDATE

static DEVICE_ATTR(ftsfwupgradeapp, S_IRUGO|S_IWUSR, ft5x0x_fwupgradeapp_show, ft5x0x_fwupgradeapp_store);
#endif

#if CTP_PROC_INTERFACE

#define FT5X0X_INI_FILEPATH "/system/etc/"

static int ft5x0x_GetInISize(char *config_name)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];
	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s%s", FT5X0X_INI_FILEPATH, config_name);
	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL);
	return fsize;
}

static int ft5x0x_ReadInIData(char *config_name, char *config_buf)

{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	char filepath[128];
	loff_t pos;
	mm_segment_t old_fs;
	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s%s", FT5X0X_INI_FILEPATH, config_name);
	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, config_buf, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);
	return 0;
}

static int ft5x0x_get_testparam_from_ini(char *config_name)

{
	char *filedata = NULL;
	int inisize = ft5x0x_GetInISize(config_name);
	pr_info("inisize = %d \n ", inisize);
	if (inisize <= 0) {
		pr_err("%s ERROR:Get firmware size failed\n", __func__);
		return -EIO;
	}
	filedata = kmalloc(inisize + 1, GFP_ATOMIC);
	if (ft5x0x_ReadInIData(config_name, filedata)) {
		pr_err("%s() - ERROR: request_firmware failed\n", __func__);
		kfree(filedata);
		return -EIO;
	} else {
		pr_info("ft5x0x_ReadInIData successful\n");
	}
	SetParamData(filedata);
	return 0;
}

#if CTP_SYS_APK_UPDATE
static ssize_t ft5x0x_ftsmcaptest_show(struct device *dev, struct device_attribute *attr, char *buf)

{
	/* place holder for future use */
	return -EPERM;
}

static ssize_t ft5x0x_ftsmcaptest_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)

{
	char cfgname[128];
	memset(cfgname, 0, sizeof(cfgname));
	sprintf(cfgname, "%s", buf);
	cfgname[count-1] = '\0';
	Init_I2C_Write_Func(focal_i2c_Write);
	Init_I2C_Read_Func(focal_i2c_Read);
	if (ft5x0x_get_testparam_from_ini(cfgname) < 0)
		CTP_ERROR("get testparam from ini failure\n");
	else {
		if (true == StartTestTP())
			CTP_INFO("tp test pass\n");
		else
			CTP_INFO("tp test failure\n");
		FreeTestParamData();
	}
	return count;
}

static DEVICE_ATTR(ftsmcaptest, S_IRUGO|S_IWUSR, ft5x0x_ftsmcaptest_show, ft5x0x_ftsmcaptest_store);
#endif

#endif

#ifdef CONFIG_OF
static int ft5x06_get_dt_coords(struct device *dev, char *name,
								struct ft5x06_ts_platform_data *pdata)
{
	u32 coords[FT_COORDS_ARR_SIZE];
	struct property *prop;
	struct device_node *np = dev->of_node;
	int coords_size, rc;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	coords_size = prop->length / sizeof(u32);
	if (coords_size != FT_COORDS_ARR_SIZE) {
		dev_err(dev, "invalid %s\n", name);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(np, name, coords, coords_size);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read %s\n", name);
		return rc;
	}

	if (!strcmp(name, "focaltech,panel-coords")) {
		pdata->panel_minx = coords[0];
		pdata->panel_miny = coords[1];
		pdata->panel_maxx = coords[2];
		pdata->panel_maxy = coords[3];
	} else if (!strcmp(name, "focaltech,display-coords")) {
		pdata->x_min = coords[0];
		pdata->y_min = coords[1];
		pdata->x_max = coords[2];
		pdata->y_max = coords[3];
	} else {
		dev_err(dev, "unsupported property %s\n", name);
		return -EINVAL;
	}

	return 0;
}

static int ft5x06_parse_dt(struct device *dev,
						   struct ft5x06_ts_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;
	struct property *prop;
	u32 temp_val, num_buttons;
	u32 button_map[MAX_BUTTONS];

	pdata->name = "focaltech";
	rc = of_property_read_string(np, "focaltech,name", &pdata->name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read name\n");
		return rc;
	}

	rc = ft5x06_get_dt_coords(dev, "focaltech,panel-coords", pdata);
	if (rc && (rc != -EINVAL))
		return rc;

	rc = ft5x06_get_dt_coords(dev, "focaltech,display-coords", pdata);
	if (rc)
		return rc;

	pdata->i2c_pull_up = of_property_read_bool(np,
						 "focaltech,i2c-pull-up");

	pdata->no_force_update = of_property_read_bool(np,
							 "focaltech,no-force-update");
	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "focaltech,reset-gpio",
						0, &pdata->reset_gpio_flags);
	if (pdata->reset_gpio < 0)
		return pdata->reset_gpio;

	pdata->irq_gpio = of_get_named_gpio_flags(np, "focaltech,irq-gpio",
					  0, &pdata->irq_gpio_flags);
	if (pdata->irq_gpio < 0)
		return pdata->irq_gpio;

	pdata->fw_name = "ft_fw.bin";
	rc = of_property_read_string(np, "focaltech,fw-name", &pdata->fw_name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw name\n");
		return rc;
	}

	rc = of_property_read_u32(np, "focaltech,group-id", &temp_val);
	if (!rc)
		pdata->group_id = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,hard-reset-delay-ms",
							  &temp_val);
	if (!rc)
		pdata->hard_rst_dly = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,soft-reset-delay-ms",
							  &temp_val);
	if (!rc)
		pdata->soft_rst_dly = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,num-max-touches", &temp_val);
	if (!rc)
		pdata->num_max_touches = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,fw-delay-aa-ms", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay aa\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_aa =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-delay-55-ms", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay 55\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_55 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-upgrade-id1", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw upgrade id1\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.upgrade_id_1 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-upgrade-id2", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw upgrade id2\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.upgrade_id_2 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-delay-readid-ms",
							  &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay read id\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_readid =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-delay-era-flsh-ms",
							  &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay erase flash\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_erase_flash =  temp_val;

	pdata->info.auto_cal = of_property_read_bool(np,
						   "focaltech,fw-auto-cal");

	pdata->fw_vkey_support = of_property_read_bool(np,
							 "focaltech,fw-vkey-support");

	pdata->ignore_id_check = of_property_read_bool(np,
							 "focaltech,ignore-id-check");

	rc = of_property_read_u32(np, "focaltech,family-id", &temp_val);
	if (!rc)
		pdata->family_id = temp_val;
	else
		return rc;

	prop = of_find_property(np, "focaltech,button-map", NULL);
	if (prop) {
		num_buttons = prop->length / sizeof(temp_val);
		if (num_buttons > MAX_BUTTONS)
			return -EINVAL;

		rc = of_property_read_u32_array(np, "focaltech,button-map", button_map,
									    num_buttons);
		if (rc) {
			dev_err(dev, "Unable to read key codes\n");
			return rc;
		}
	}

	return 0;
}
#else
static int ft5x06_parse_dt(struct device *dev,
			   struct ft5x06_ts_platform_data *pdata)
{
	return -ENODEV;
}
#endif

#if CTP_PROC_INTERFACE
static ssize_t ctp_version_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int num, i;
	u8 fw_info[6] = {0x00};
	u8 reg_addr = 0xA3;
	if (*ppos)
		return 0;

	ft5x06_i2c_read(update_client, &reg_addr, 1, fw_info, 6);
	for (i = 0; i < sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info); i++)
		if (fw_info[0] == fts_updateinfo[i].CHIP_ID)
			break;
	if (i >= sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info))
		i = 0;
	num = sprintf(user_buf, "IC is %s, vendor:%02X, fw_version:%02X\n", fts_updateinfo[i].FTS_NAME, fw_info[5], fw_info[3]);
	*ppos += num;
	return num;
}


static ssize_t ctp_version_write(struct file *filp, const char __user *userbuf, size_t count, loff_t *ppos)
{
	return -EPERM;
}

static ssize_t ctp_binupdate_proc_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	if (*ppos)
		return 0;
	*ppos += count;

	return count;
}
static ssize_t ctp_binupdate_proc_write(struct file *filp, const char __user *userbuf, size_t count, loff_t *ppos)
{
	if (*ppos)
		return 0;
	*ppos += count;
	memset(binfilename, '\0', 50);
	memcpy(binfilename, userbuf, count);
	binfilelen = count;

	ft5x0x_fwupgradeapp_store(&update_client->dev, NULL, binfilename, binfilelen);
	return count;
}

static ssize_t ctp_openshort_proc_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char *ptr = buf;
	char cfgname[128];
	u8 fw_info[6] = {0x00};
	u8 reg_addr = 0xA3;
	u8 result = 0;

	if (*ppos) {
		CTP_INFO("tp test again return\n");
		return 0;
	}
	*ppos += count;

	ft5x06_i2c_read(update_client, &reg_addr, 1, fw_info, 6);
	CTP_INFO("ic is %x", fw_info[5]);

	if (fw_info[5] == VENDOR_O_FILM) {
		sprintf(cfgname, "%s", "ft5x06_ic_v14_ofilm.ini");
		CTP_INFO("ic is oufei");
	} else if (fw_info[5] == VENDOR_MUTTO) {
		sprintf(cfgname, "%s", "ft5x06_ic_v14_mutto.ini");
		CTP_INFO("ic is mudong");
	} else {
		CTP_INFO("no ini match the project ctp, please check!");
		return count;
	}

	Init_I2C_Write_Func(focal_i2c_Write);
	Init_I2C_Read_Func(focal_i2c_Read);
	if (ft5x0x_get_testparam_from_ini(cfgname) < 0) {
		CTP_ERROR("get testparam from ini failure\n");
		sprintf(ptr, "result=%d\n", 0);
	} else {
		if (true == StartTestTP()) {
		   CTP_INFO("tp test pass\n");
		   result = 1;
		} else {
			CTP_INFO("tp test failure\n");
			result = 0;
			g_focalscaptested = 1;
		}
		FreeTestParamData();
	}
	return  sprintf(ptr, "result=%d\n", result);
}
static ssize_t ctp_openshort_proc_write(struct file *filp, const char __user *userbuf, size_t count, loff_t *ppos)
{
	return -EPERM;
}

short ctp_iTxNum = 12;
short ctp_iRxNum = 18;
static void focal_msleep(int ms)
{
	msleep(ms);
}

int ctp_ReadReg(struct i2c_client *client, unsigned char RegAddr, unsigned char *RegData)
{
	return ft5x06_i2c_read(client, &RegAddr, 1, RegData, 1);
}

int ctp_WriteReg(struct i2c_client *client, unsigned char RegAddr, unsigned char RegData)
{
	unsigned char cmd[2] = {0};
	cmd[0] = RegAddr;
	cmd[1] = RegData;
	return ft5x06_i2c_write(client, cmd, 2);
}

static int StartScan(void)
{
	int err = 0, i = 0;
	unsigned char regvalue = 0x00;

	if (ctp_WriteReg(update_client, 0x00, 0x40) < 0) {
		CTP_ERROR("Enter factory failure\n");
	}
	focal_msleep(100);
	err = ctp_ReadReg(update_client, 0x00, &regvalue);
	if (err < 0) {
		CTP_ERROR("Enter StartScan %d \n", regvalue);
		return err;
	} else {
		regvalue |= 0x80;
		err = ctp_WriteReg(update_client, 0x00, regvalue);
		if (err < 0)
			return err;
		else {
			for (i = 0; i < 20; i++) {
				focal_msleep(8);
				err = ctp_ReadReg(update_client, 0x00, &regvalue);
				if (err < 0)
					return err;
				else {
					if (0 == (regvalue >> 7)) {
						break;
					}
				}
			}
			if (i >= 20)
				return -EIO;
		}
	}
	return 0;

}

void ctp_GetRawData(struct i2c_client *client, s32 RawData[TX_NUM_MAX][RX_NUM_MAX])

{
	unsigned char LineNum = 0;
	unsigned char I2C_wBuffer[3];
	unsigned char rrawdata[RX_NUM_MAX*2];
	unsigned char j = 0, loop = 0, len = 0, i = 0;
	short mtk_readlen = 8;
	short ByteNum = 0;
	int ReCode = 0;
	CTP_DEBUG("enter get rawdata");
	if (ctp_WriteReg(update_client, 0x00, 0x40) >= 0) {
		if (StartScan() >= 0) {
			CTP_DEBUG("enter get start scan");
			for (LineNum = 0; LineNum < ctp_iTxNum; LineNum++) {
				I2C_wBuffer[0] = 0x01;
				I2C_wBuffer[1] = LineNum;
				ReCode = ft5x06_i2c_write(client, I2C_wBuffer, 2);
				ByteNum = ctp_iRxNum * 2;
				if (ReCode >= 0) {
					if (ByteNum % mtk_readlen == 0)
						loop = ByteNum / mtk_readlen;
					else
						loop = ByteNum / mtk_readlen + 1;
					for (j = 0; j < loop; j++) {
						len = ByteNum - j * mtk_readlen;
						if (len > mtk_readlen)
							len = mtk_readlen;
						I2C_wBuffer[0] = (unsigned char)(0x10 + j * mtk_readlen);
						I2C_wBuffer[1] = len;
						memset(rrawdata, 0x00, sizeof(rrawdata));
						ReCode = ft5x06_i2c_read(client, I2C_wBuffer, 2, rrawdata, len);
						if (ReCode >= 0) {
							for (i = 0; i < (len >> 1); i++) {
								RawData[LineNum][i+j*mtk_readlen/2] = (short)((unsigned short)(rrawdata[i << 1]) << 8) \
									+ (unsigned short)rrawdata[(i << 1) + 1];
							}
						} else {
							CTP_ERROR("Get Rawdata failure\n");
							break;
						}
					}
				}
			}
		}
	}
}


s32 RawData[TX_NUM_MAX][RX_NUM_MAX];


static ssize_t ctp_rawdata_proc_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int i, j;
	if (*ppos)
		return 0;

	ctp_GetRawData(update_client, RawData);
	memcpy(buf, RawData, TX_NUM_MAX*RX_NUM_MAX*sizeof(s32));
	i = 0;
	j = 0;

#if CTP_DEBUG_ON
	for (i = 0; i < ctp_iTxNum; i++) {
		for (j = 0; j < ctp_iRxNum; j++)
			printk("%d  ", RawData[i][j]);
		printk("\n");
	}
#endif

	ctp_WriteReg(update_client, 0x00, 0x00);

	*ppos += count;
	return count;
}
static ssize_t ctp_rawdata_proc_write(struct file *filp, const char __user *userbuf, size_t count, loff_t *ppos)
{
	return -EPERM;
}

static ssize_t ctp_testresult_proc_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	ssize_t ret_count = 0;
	char *databuf = NULL;
	databuf = kmalloc((2 * 1024), GFP_ATOMIC);

	if (*ppos)
		return 0;

	if (databuf == NULL) {
		 ret_count = sprintf(buf, "Alloc memery failed! \n");
		 return ret_count;
	}

	if (down_interruptible(&g_device_mutex))
		return -ERESTARTSYS;

	if (g_focalscaptested == 1) {
		 ret_count = focal_save_error_data(databuf, (2 * 1024));

		 memcpy(buf, databuf, ret_count);
		 g_focalscaptested = 0;
	} else
		 ret_count = sprintf(buf, "Please tested at first! \n");

	up(&g_device_mutex);
	kfree(databuf);
	*ppos += ret_count;
	return ret_count;
}

static ssize_t ctp_testresult_proc_write(struct file *filp, const char __user *userbuf, size_t count, loff_t *ppos)
{
	return -EPERM;
}

void create_ctp_proc(void)
{
	struct proc_dir_entry *ctp_device_proc = NULL;
	struct proc_dir_entry *ctp_version_proc = NULL;
	struct proc_dir_entry *ctp_binupdate_proc = NULL;
	struct proc_dir_entry *ctp_openshort_proc = NULL;
	struct proc_dir_entry *ctp_rawdata_proc = NULL;
	struct proc_dir_entry *ctp_testresult_proc = NULL;

	ctp_device_proc = proc_mkdir(CTP_PARENT_PROC_NAME, NULL);
	if (ctp_device_proc == NULL) {
		CTP_ERROR("ft5x06: create parent_proc fail\n");
		return;
	}

	ctp_version_proc = proc_create(CTP_VERSION_PROC_NAME, 0666, ctp_device_proc, &ctp_version_proc_fops);
	if (ctp_version_proc == NULL) {
		CTP_ERROR("ft5x06: create ctp_vesrion_proc fail\n");
	}

	ctp_binupdate_proc = proc_create(CTP_BINUPDATE_PROC_NAME, 0666, ctp_device_proc, &ctp_binupdate_procs_fops);
	if (ctp_binupdate_proc == NULL) {
		CTP_ERROR("ft5x06: create ctp_vesrion_proc fail\n");
	}

	ctp_openshort_proc = proc_create(CTP_OPENHSORT_PROC_NAME, 0666, ctp_device_proc, &ctp_openshort_procs_fops);
	if (ctp_openshort_proc == NULL) {
		CTP_ERROR("ft5x06: create openshort_proc fail\n");
	}

	ctp_rawdata_proc = proc_create(CTP_RAWDATA_PROC_NAME, 0777, ctp_device_proc, &ctp_rawdata_procs_fops);
	if (ctp_rawdata_proc == NULL) {
		CTP_ERROR("ft5x06: create ctp_rawdata_proc fail\n");
	}

	ctp_testresult_proc = proc_create(CTP_TESTRESULT_PROC_NAME, 0777, ctp_device_proc, &ctp_testresult_procs_fops);
	if (ctp_testresult_proc == NULL) {
		CTP_ERROR("ft5x06: create ctp_testresult_proc fail\n");
	}

}
#endif


#if FTS_PROC_APK_DEBUG
int ft5x0x_i2c_Read(struct i2c_client *client, char *writebuf,
			int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			},
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			},
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "f%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			},
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}
	return ret;
}
/*write data by i2c*/
int ft5x0x_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		},
	};

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s i2c write error.\n", __func__);

	return ret;
}

static ssize_t ft5x0x_debug_write(struct file *filp, const char __user *buff, size_t len, loff_t *ppos)
{
	struct i2c_client *client = update_client;
	unsigned char writebuf[FTS_PACKET_LENGTH];
	int buflen = len;
	int writelen = 0;
	int ret = 0;

	if (*ppos)
		return -EPERM;

	if (copy_from_user(&writebuf, buff, buflen)) {
		dev_err(&client->dev, "%s:copy from user error\n", __func__);
		return -EFAULT;
	}
	proc_operate_mode = writebuf[0];
	CTP_INFO("write mode %x", proc_operate_mode);
	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		{
			char upgrade_file_path[128];
			memset(upgrade_file_path, 0, sizeof(upgrade_file_path));
			sprintf(upgrade_file_path, "%s", writebuf + 1);
			upgrade_file_path[buflen-1] = '\0';

			disable_irq(client->irq);

			ret = fts_ctpm_fw_upgrade_with_app_file(upgrade_file_path);

			enable_irq(client->irq);
			if (ret < 0) {
				dev_err(&client->dev, "%s:upgrade failed.\n", __func__);
				return ret;
			}
		}
		break;
	case PROC_READ_REGISTER:
		writelen = 1;
		ret = ft5x0x_i2c_Write(client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_WRITE_REGISTER:
		writelen = 2;
		ret = ft5x0x_i2c_Write(client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_RAWDATA:
		break;
	case PROC_AUTOCLB:
		fts_ctpm_auto_clb();
		break;
	default:
		break;
	}

	*ppos += len;
	return len;
}

unsigned char debug_read_buf[PAGE_SIZE];

/*interface of read proc*/
static ssize_t ft5x0x_debug_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	struct i2c_client *client = update_client;
	int ret = 0;
	int num_read_chars = 0;
	int readlen = 0;
	u8 regvalue = 0x00, regaddr = 0x00;

	if (*ppos)
		return -EPERM;

	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		/*after calling ft5x0x_debug_write to upgrade*/
		regaddr = 0xA6;
		ret = ft5x0x_read_reg(client, regaddr, &regvalue);
		if (ret < 0)
			num_read_chars = sprintf(debug_read_buf, "%s", "get fw version failed.\n");
		else
			num_read_chars = sprintf(debug_read_buf, "current fw version:0x%02x\n", regvalue);
		break;
	case PROC_READ_REGISTER:
		readlen = 1;
		ret = ft5x0x_i2c_Read(client, NULL, 0, debug_read_buf, readlen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:read iic error\n", __func__);
			return ret;
		} else

		num_read_chars = 1;
		break;
	case PROC_RAWDATA:
		break;
	default:
		break;
	}

	memcpy(user_buf, debug_read_buf, num_read_chars);
	*ppos += num_read_chars;
	return num_read_chars;
}

static const struct file_operations ctp_apk_proc_fops = {
	.write = ft5x0x_debug_write,
	.read = ft5x0x_debug_read,
	.open = simple_open,
	.owner = THIS_MODULE,
};

int ft5x0x_create_apk_debug_channel(struct i2c_client *client)
{
	ft5x0x_proc_entry = proc_create(PROC_NAME, 0666, NULL, &ctp_apk_proc_fops);
	if (NULL == ft5x0x_proc_entry) {
		dev_err(&client->dev, "Couldn't create proc entry!\n");
		return -ENOMEM;
	}
	return 0;
}

void ft5x0x_release_apk_debug_channel(void)
{
	if (ft5x0x_proc_entry)
		remove_proc_entry(PROC_NAME, NULL);
}

#endif

static int ft5x06_ts_probe(struct i2c_client *client,
						   const struct i2c_device_id *id)
{
	struct ft5x06_ts_platform_data *pdata;
	struct ft5x06_ts_data *data;
	struct input_dev *input_dev;
	struct dentry *temp;
	u8 reg_value;
	u8 reg_addr;
	int err, len;
	u8 ic_name;
#if TPD_AUTO_UPGRADE
	int ret_auto_upgrade = 0;
	int i;
#endif

	temp = NULL;
	update_client = client;
#if CTP_PROC_INTERFACE
	g_focalclient = client;
	sema_init(&g_device_mutex, 1);
#endif

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
							 sizeof(struct ft5x06_ts_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		err = ft5x06_parse_dt(&client->dev, pdata);
		if (err) {
			dev_err(&client->dev, "DT parsing failed\n");
			return err;
		}
	} else
		pdata = client->dev.platform_data;

	if (!pdata) {
		dev_err(&client->dev, "Invalid pdata\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C not supported\n");
		return -ENODEV;
	}

	data = devm_kzalloc(&client->dev,
						sizeof(struct ft5x06_ts_data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Not enough memory\n");
		return -ENOMEM;
	}

	if (pdata->fw_name) {
		len = strlen(pdata->fw_name);
		if (len > FT_FW_NAME_MAX_LEN - 1) {
			dev_err(&client->dev, "Invalid firmware name\n");
			return -EINVAL;
		}

		strlcpy(data->fw_name, pdata->fw_name, len + 1);
	}

	data->tch_data_len = FT_TCH_LEN(pdata->num_max_touches);
	data->tch_data = devm_kzalloc(&client->dev,
								  data->tch_data_len, GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Not enough memory\n");
		return -ENOMEM;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev, "failed to allocate input device\n");
		return -ENOMEM;
	}

	data->input_dev = input_dev;
	data->client = client;
	data->pdata = pdata;

	input_dev->name = "ft5x06_tran";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	input_set_drvdata(input_dev, data);
	i2c_set_clientdata(client, data);

	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_mt_init_slots(input_dev, pdata->num_max_touches, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, pdata->x_min,
						 pdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, pdata->y_min,
						 pdata->y_max, 0, 0);

	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev, "Input device registration failed\n");
		goto free_inputdev;
	}

	if (pdata->power_init) {
		err = pdata->power_init(true);
		if (err) {
			dev_err(&client->dev, "power init failed");
			goto unreg_inputdev;
		}
	} else {
		err = ft5x06_power_init(data, true);
		if (err) {
			dev_err(&client->dev, "power init failed");
			goto unreg_inputdev;
		}
	}

	if (pdata->power_on) {
		err = pdata->power_on(true);
		if (err) {
			dev_err(&client->dev, "power on failed");
			goto pwr_deinit;
		}
	} else {
		err = ft5x06_power_on(data, true);
		if (err) {
			dev_err(&client->dev, "power on failed");
			goto pwr_deinit;
		}
	}

	err = ft5x06_ts_pinctrl_init(data);
	if (!err && data->ts_pinctrl) {
		err = ft5x06_ts_pinctrl_select(data, true);
		if (err < 0)
			goto pwr_off;
	}

	if (gpio_is_valid(pdata->irq_gpio)) {
		err = gpio_request(pdata->irq_gpio, "ft5x06_irq_gpio");
		if (err) {
			dev_err(&client->dev, "irq gpio request failed");
			goto pwr_off;
		}
		err = gpio_direction_input(pdata->irq_gpio);
		if (err) {
			dev_err(&client->dev,
					"set_direction for irq gpio failed\n");
			goto free_irq_gpio;
		}
	}

	if (gpio_is_valid(pdata->reset_gpio)) {
		err = gpio_request(pdata->reset_gpio, "ft5x06_reset_gpio");
		if (err) {
			dev_err(&client->dev, "reset gpio request failed");
			goto free_irq_gpio;
		}

		err = gpio_direction_output(pdata->reset_gpio, 0);
		if (err) {
			dev_err(&client->dev,
					"set_direction for reset gpio failed\n");
			goto free_reset_gpio;
		}
		msleep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
	}

	/* make sure CTP already finish startup process */
	msleep(data->pdata->soft_rst_dly);

	/* check the controller id */
	reg_addr = FT_REG_ID;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0) {
		dev_err(&client->dev, "version read failed");
		goto free_reset_gpio;
	}

	ic_name = reg_value;

	dev_info(&client->dev, "Device ID = 0x%x\n", reg_value);

	if ((pdata->family_id != reg_value) && (!pdata->ignore_id_check)) {
		dev_err(&client->dev, "%s:Unsupported controller\n", __func__);
		goto free_reset_gpio;
	}

	data->family_id = pdata->family_id;

	err = request_threaded_irq(client->irq, NULL,
							   ft5x06_ts_interrupt,
							   pdata->irq_gpio_flags | IRQF_ONESHOT,
							   client->dev.driver->name, data);
	if (err) {
		dev_err(&client->dev, "request irq failed\n");
		goto free_reset_gpio;
	}

#if CTP_SYS_APK_UPDATE
	err = device_create_file(&client->dev, &dev_attr_fw_name);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
	}

	err = device_create_file(&client->dev, &dev_attr_ftsfwupgradeapp);
	if (err) {
		dev_err(&client->dev, "upgradeapp sys file creation failed\n");
	}

	err = device_create_file(&client->dev, &dev_attr_ftsmcaptest);
	if (err) {
		dev_err(&client->dev, "ftsmcaptest sys file creation failed\n");
	}
	err = device_create_file(&client->dev, &dev_attr_update_fw);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
	}

	err = device_create_file(&client->dev, &dev_attr_force_update_fw);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
	}

	data->dir = debugfs_create_dir(FT_DEBUG_DIR_NAME, NULL);
	if (data->dir == NULL || IS_ERR(data->dir)) {
		pr_err("debugfs_create_dir failed(%ld)\n", PTR_ERR(data->dir));
		err = PTR_ERR(data->dir);
	} else {
		temp = debugfs_create_file("addr", S_IRUSR | S_IWUSR, data->dir, data,
					   &debug_addr_fops);
		if (temp == NULL || IS_ERR(temp)) {
			pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
			err = PTR_ERR(temp);
		}

		temp = debugfs_create_file("data", S_IRUSR | S_IWUSR, data->dir, data,
					   &debug_data_fops);
		if (temp == NULL || IS_ERR(temp)) {
			pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
			err = PTR_ERR(temp);
		}

		temp = debugfs_create_file("suspend", S_IRUSR | S_IWUSR, data->dir,
						data, &debug_suspend_fops);
		if (temp == NULL || IS_ERR(temp)) {
			pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
			err = PTR_ERR(temp);
		}

		temp = debugfs_create_file("dump_info", S_IRUSR | S_IWUSR, data->dir,
						data, &debug_dump_info_fops);
		if (temp == NULL || IS_ERR(temp)) {
			pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
			err = PTR_ERR(temp);
		}
	}
#endif

	data->ts_info = devm_kzalloc(&client->dev,
								 FT_INFO_MAX_LEN, GFP_KERNEL);
	if (!data->ts_info) {
		dev_err(&client->dev, "Not enough memory\n");
		goto free_irq_gpio;
	}

	/*get some register information */
	reg_addr = FT_REG_POINT_RATE;
	ft5x06_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0)
		dev_err(&client->dev, "report rate read failed");

	dev_info(&client->dev, "report rate = %dHz\n", reg_value * 10);

	reg_addr = FT_REG_THGROUP;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0)
		dev_err(&client->dev, "threshold read failed");

	dev_dbg(&client->dev, "touch threshold = %d\n", reg_value * 4);

	ft5x06_update_fw_ver(data);

	CTP_DEBUG("FT:ft5x06_update_fw_vendor_id\n");
	ft5x06_update_fw_vendor_id(data);

	FT_STORE_TS_INFO(data->ts_info, data->family_id, data->pdata->name,
					 data->pdata->num_max_touches, data->pdata->group_id,
					 data->pdata->fw_vkey_support ? "yes" : "no",
					 data->pdata->fw_name, data->fw_ver[0],
					 data->fw_ver[1], data->fw_ver[2]);

#if defined(CONFIG_FB)
	INIT_WORK(&data->fb_notify_work, fb_notify_resume_work);
	data->fb_notif.notifier_call = fb_notifier_callback;

	err = fb_register_client(&data->fb_notif);

	if (err)
		dev_err(&client->dev, "Unable to register fb_notifier: %d\n",
				err);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN +
								FT_SUSPEND_LEVEL;
	data->early_suspend.suspend = ft5x06_ts_early_suspend;
	data->early_suspend.resume = ft5x06_ts_late_resume;
	register_early_suspend(&data->early_suspend);
#endif

#if CTP_PROC_INTERFACE
	create_ctp_proc();
#endif

#if FTS_PROC_APK_DEBUG
	ft5x0x_create_apk_debug_channel(update_client);
#endif


#if TPD_AUTO_UPGRADE
	{
		CTP_DEBUG("********************Enter CTP Auto Upgrade********************\n");
		msleep(50);
		i = 0;
		do {
			ret_auto_upgrade = fts_ctpm_fw_upgrade_with_i_file(data);
			i++;
			if (ret_auto_upgrade < 0) {
				CTP_DEBUG(" ctp upgrade fail err = %d \n", ret_auto_upgrade);
			}
		} while ((ret_auto_upgrade < 0) && (i < 3));
	}
#endif

   CTP_DEBUG("Transsion probe Finished\n");

	return 0;


free_reset_gpio:
	if (gpio_is_valid(pdata->reset_gpio))
		gpio_free(pdata->reset_gpio);
	if (data->ts_pinctrl) {
		err = ft5x06_ts_pinctrl_select(data, false);
		if (err < 0)
			CTP_ERROR("Cannot get idle pinctrl state\n");
	}
free_irq_gpio:
	if (gpio_is_valid(pdata->irq_gpio))
		gpio_free(pdata->irq_gpio);
	if (data->ts_pinctrl) {
		err = ft5x06_ts_pinctrl_select(data, false);
		if (err < 0)
			CTP_ERROR("Cannot get idle pinctrl state\n");
	}
pwr_off:
	if (pdata->power_on)
		pdata->power_on(false);
	else
		ft5x06_power_on(data, false);
pwr_deinit:
	if (pdata->power_init)
		pdata->power_init(false);
	else
		ft5x06_power_init(data, false);
unreg_inputdev:
	input_unregister_device(input_dev);
	input_dev = NULL;
free_inputdev:
	input_free_device(input_dev);
	return err;
}

static int ft5x06_ts_remove(struct i2c_client *client)
{
	struct ft5x06_ts_data *data = i2c_get_clientdata(client);
	int retval;

#if CTP_SYS_APK_UPDATE
	device_remove_file(&client->dev, &dev_attr_fw_name);
#endif

#if FTS_PROC_APK_DEBUG
	ft5x0x_release_apk_debug_channel();
#endif


#if defined(CONFIG_FB)
	if (fb_unregister_client(&data->fb_notif))
		dev_err(&client->dev, "Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&data->early_suspend);
#endif
	free_irq(client->irq, data);

	if (gpio_is_valid(data->pdata->reset_gpio))
		gpio_free(data->pdata->reset_gpio);

	if (gpio_is_valid(data->pdata->irq_gpio))
		gpio_free(data->pdata->irq_gpio);

	if (data->ts_pinctrl) {
		retval = ft5x06_ts_pinctrl_select(data, false);
		if (retval < 0)
			CTP_ERROR("Cannot get idle pinctrl state\n");
	}

	if (data->pdata->power_on)
		data->pdata->power_on(false);
	else
		ft5x06_power_on(data, false);

	if (data->pdata->power_init)
		data->pdata->power_init(false);
	else
		ft5x06_power_init(data, false);

	input_unregister_device(data->input_dev);

	return 0;
}

static const struct i2c_device_id ft5x06_ts_id[] = {
	{"ft5x06_tran", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ft5x06_ts_id);

#ifdef CONFIG_OF
static struct of_device_id ft5x06_match_table[] = {
	{.compatible = "focaltech,5346i",},
	{},
};
#else
#define ft5x06_match_table NULL
#endif

static struct i2c_driver ft5x06_ts_driver = {
	.probe = ft5x06_ts_probe,
	.remove = ft5x06_ts_remove,
	.driver = {
		.name = "ft5x06_tran",
		.owner = THIS_MODULE,
		.of_match_table = ft5x06_match_table,
#ifdef CONFIG_PM
		.pm = &ft5x06_ts_pm_ops,
#endif
	},
	.id_table = ft5x06_ts_id,
};

static int __init ft5x06_ts_init(void)
{
	return i2c_add_driver(&ft5x06_ts_driver);
}
module_init(ft5x06_ts_init);

static void __exit ft5x06_ts_exit(void)
{
	i2c_del_driver(&ft5x06_ts_driver);
}
module_exit(ft5x06_ts_exit);

MODULE_DESCRIPTION("FocalTech ft5x06 TouchScreen driver");
MODULE_LICENSE("GPL v2");
