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
#include <linux/input/ft5x06_720p.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <asm/uaccess.h>

#if CTP_CHARGER_DETECT
#include <linux/power_supply.h>
#endif

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>

#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
/* Early-suspend level */
#define FT_SUSPEND_LEVEL 1
#endif

u8 TP_Maker, LCD_Maker;

#if BoardId_SUPPORT_FW
char boardid_info_tp[64] = {0,};
#endif

#define FT_DEBUG_DIR_NAME   "ts_debug"

#define TPD_MAX_POINTS_5	5
#define TPD_MAX_POINTS_10   10

#define TPD_MAX_POINTS_2	2
#define AUTO_CLB_NEED   1
#define AUTO_CLB_NONEED	 0
static struct Upgrade_Info fts_updateinfo[] = {
	{0x55, "FT5x06", TPD_MAX_POINTS_5, AUTO_CLB_NEED, 50, 30, 0x79, 0x03, 1, 2000},
	{0x08, "FT5606", TPD_MAX_POINTS_5, AUTO_CLB_NEED, 50, 30, 0x79, 0x06, 100, 2000},
	{0x0a, "FT5x16", TPD_MAX_POINTS_5, AUTO_CLB_NEED, 50, 30, 0x79, 0x07, 1, 1500},
	{0x05, "FT6208", TPD_MAX_POINTS_2, AUTO_CLB_NONEED, 60, 30, 0x79, 0x05, 10, 2000},
	{0x06, "FT6x06", TPD_MAX_POINTS_2, AUTO_CLB_NONEED, 100, 30, 0x79, 0x08, 10, 2000},
	{0x36, "FT6x36", TPD_MAX_POINTS_2, AUTO_CLB_NONEED, 100, 30, 0x79, 0x18, 10, 2000},
	{0x55, "FT5x06i", TPD_MAX_POINTS_5, AUTO_CLB_NEED, 50, 30, 0x79, 0x03, 1, 2000},
	{0x14, "FT5336", TPD_MAX_POINTS_10, AUTO_CLB_NONEED, 30, 30, 0x79, 0x11, 10, 2000},
	{0x13, "FT3316", TPD_MAX_POINTS_5, AUTO_CLB_NONEED, 30, 30, 0x79, 0x11, 10, 2000},
	{0x12, "FT5436i", TPD_MAX_POINTS_5, AUTO_CLB_NONEED, 30, 30, 0x79, 0x11, 10, 2000},
	{0x11, "FT5336i", TPD_MAX_POINTS_5, AUTO_CLB_NONEED, 30, 30, 0x79, 0x11, 10, 2000},
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

static u8 is_ic_update_crash;
static struct i2c_client *update_client;

#if CTP_CHARGER_DETECT
extern int power_supply_get_battery_charge_state(struct power_supply *psy);
static struct power_supply	*batt_psy;
static u8 is_charger_plug;
static u8 pre_charger_status;

#endif


static ssize_t ft5x06_ts_disable_keys_show(struct device *dev,
	struct device_attribute *attr, char *buf);

static ssize_t ft5x06_ts_disable_keys_store(struct device *dev,
struct device_attribute *attr, const char *buf, size_t count);


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

static void ft5x06_update_fw_vendor_id(struct ft5x06_ts_data *data)
{
	struct i2c_client *client = data->client;
	u8 reg_addr;
	int err;

	reg_addr = FT_REG_FW_VENDOR_ID;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &data->fw_vendor_id, 1);
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

#if CTP_CHARGER_DETECT
	if (!batt_psy)

		batt_psy = power_supply_get_by_name("usb");
	else {
		is_charger_plug = (u8)power_supply_get_battery_charge_state(batt_psy);

		if (is_charger_plug != pre_charger_status) {
			pre_charger_status = is_charger_plug;
			ft5x0x_write_reg(update_client, 0x8B, is_charger_plug);

		}
	}

#endif

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

		if (y == 2000) {

                if (data->disable_keys)
                    break;

			y = 1344;

			switch (x) {
			case 180:
				x = 150;
				break;
			case 540:
				x = 360;
				break;
			case 900:
				x = 580;
				break;
			default:
				break;
			}
		}

		input_mt_slot(ip_dev, id);
		if (status == FT_TOUCH_DOWN || status == FT_TOUCH_CONTACT) {
			input_mt_report_slot_state(ip_dev, MT_TOOL_FINGER, 1);
			input_report_abs(ip_dev, ABS_MT_POSITION_X, x);
			input_report_abs(ip_dev, ABS_MT_POSITION_Y, y);
		} else {
			input_mt_report_slot_state(ip_dev, MT_TOOL_FINGER, 0);
		}
	 }

	if (update_input) {
		input_mt_report_pointer_emulation(ip_dev, false);
		input_sync(ip_dev);
	}

	if (num_touches == 0) {
		 for (i = 0; i < data->pdata->num_max_touches; i++) {
			input_mt_slot(ip_dev, i);
			input_mt_report_slot_state(ip_dev, MT_TOOL_FINGER, 0);
		}
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

#if CTP_CHARGER_DETECT
		batt_psy = power_supply_get_by_name("usb");
		if (!batt_psy)
			CTP_ERROR("tp resume battery supply not found\n");
		else {
			is_charger_plug = (u8)power_supply_get_battery_charge_state(batt_psy);

			CTP_DEBUG("is_charger_plug %d, prev %d", is_charger_plug, pre_charger_status);
			if (is_charger_plug) {
				ft5x0x_write_reg(update_client, 0x8B, 1);
			} else {
				ft5x0x_write_reg(update_client, 0x8B, 0);
			}
		}
		pre_charger_status = is_charger_plug;
#endif


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
		if (*blank == FB_BLANK_UNBLANK || *blank == FB_BLANK_NORMAL
                 || *blank == FB_BLANK_VSYNC_SUSPEND || *blank == FB_BLANK_HSYNC_SUSPEND )
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
	u8 is_5336_new_bootloader = false;
	u8 is_5336_fwsize_30 = false;
	u8 fw_ecc;

#if 1
	reg_addr = FT_REG_ID;
	temp = ft5x06_i2c_read(client, &reg_addr, 1, &chip_id, 1);
	if (temp < 0) {
		dev_err(&client->dev, "version read failed");
	}

	if (is_ic_update_crash) {
		 chip_id = CTP_IC_TYPE_0;

	}
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
		info.auto_cal = fts_updateinfo[0].AUTO_CLB;
		info.delay_55 = fts_updateinfo[0].delay_55;
		info.delay_aa = fts_updateinfo[0].delay_aa;
		info.delay_erase_flash = fts_updateinfo[0].delay_earse_flash;
		info.delay_readid = fts_updateinfo[0].delay_readid;
		info.upgrade_id_1 = fts_updateinfo[0].upgrade_id_1;
		info.upgrade_id_2 = fts_updateinfo[0].upgrade_id_2;
	}
#endif

	CTP_DEBUG("enter fw_upgrade_start");
	dev_err(&client->dev, "id1 = 0x%x id2 = 0x%x family_id=0x%x\n",
			info.upgrade_id_1, info.upgrade_id_2, ts_data->family_id);
	/* determine firmware size */
	if (*(data + data_len - FT_BLOADER_SIZE_OFF) == FT_BLOADER_NEW_SIZE)
		is_5336_fwsize_30 = true;
	else
		is_5336_fwsize_30 = false;


	for (i = 0, j = 0; i < FT_UPGRADE_LOOP; i++) {
		msleep(FT_EARSE_DLY_MS);

	if (gpio_is_valid(ts_data->pdata->reset_gpio)) {
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 0);
		msleep(ts_data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 1);
	}

		if (i <= (FT_UPGRADE_LOOP / 2))
			msleep(info.delay_55 + i * 3);
		else
			msleep(info.delay_55 - (i - (FT_UPGRADE_LOOP / 2)) * 2);

		/* Enter upgrade mode */
		w_buf[0] = FT_UPGRADE_55;
		ft5x06_i2c_write(client, &w_buf[0], 1);
		usleep(FT_55_AA_DLY_NS);
		w_buf[0] = FT_UPGRADE_AA;
		ft5x06_i2c_write(client, &w_buf[0], 1);

		/* check READ_ID */
		msleep(info.delay_readid);
		w_buf[0] = FT_READ_ID_REG;
		w_buf[1] = 0x00;
		w_buf[2] = 0x00;
		w_buf[3] = 0x00;

		ft5x06_i2c_read(client, w_buf, 4, r_buf, 2);

		CTP_DEBUG("%X, %X", r_buf[0], r_buf[1]);
		if (r_buf[0] != info.upgrade_id_1
			|| r_buf[1] != info.upgrade_id_2) {
			dev_err(&client->dev, "Upgrade ID mismatch(%d), IC=0x%x 0x%x, info=0x%x 0x%x\n",
					i, r_buf[0], r_buf[1],
					info.upgrade_id_1, info.upgrade_id_2);
		} else
			break;
	}

	if (i >= FT_UPGRADE_LOOP) {
		dev_err(&client->dev, "Abort upgrade\n");
		return -EIO;
	}

	w_buf[0] = 0xcd;
	ft5x06_i2c_read(client, w_buf, 1, r_buf, 1);

	if (r_buf[0] <= 4)
		is_5336_new_bootloader = FT_BLOADER_VERSION_LZ4;
	else if (r_buf[0] == 7)
		is_5336_new_bootloader = FT_BLOADER_VERSION_Z7;
	else if (r_buf[0] >= 0x0f &&
			 ((ts_data->family_id == FT_FT5336_FAMILY_ID_0x11) ||
			  (ts_data->family_id == FT_FT5336_FAMILY_ID_0x12) ||
			  (ts_data->family_id == FT_FT5336_FAMILY_ID_0x13) ||
			  (ts_data->family_id == FT_FT5336_FAMILY_ID_0x14)))
		is_5336_new_bootloader = FT_BLOADER_VERSION_GZF;
	else
		is_5336_new_bootloader = FT_BLOADER_VERSION_LZ4;

	dev_dbg(&client->dev, "bootloader type=%d, r_buf=0x%x, family_id=0x%x\n",
			is_5336_new_bootloader, r_buf[0], ts_data->family_id);

	/* erase app and panel paramenter area */
	w_buf[0] = FT_ERASE_APP_REG;
	ft5x06_i2c_write(client, w_buf, 1);
	msleep(info.delay_erase_flash);

	if (is_5336_fwsize_30) {
		w_buf[0] = FT_ERASE_PANEL_REG;
		ft5x06_i2c_write(client, w_buf, 1);
	}
	msleep(FT_EARSE_DLY_MS);

	/* program firmware */
	if (is_5336_new_bootloader == FT_BLOADER_VERSION_LZ4
		|| is_5336_new_bootloader == FT_BLOADER_VERSION_Z7)
		data_len = data_len - FT_DATA_LEN_OFF_OLD_FW;
	else
		data_len = data_len - FT_DATA_LEN_OFF_NEW_FW;

	pkt_num = (data_len) / FT_FW_PKT_LEN;
	pkt_len = FT_FW_PKT_LEN;
	pkt_buf[0] = FT_FW_START_REG;
	pkt_buf[1] = 0x00;
	fw_ecc = 0;

	for (i = 0; i < pkt_num; i++) {
		temp = i * FT_FW_PKT_LEN;
		pkt_buf[2] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[3] = (u8) temp;
		pkt_buf[4] = (u8) (pkt_len >> FT_8BIT_SHIFT);
		pkt_buf[5] = (u8) pkt_len;

		for (j = 0; j < FT_FW_PKT_LEN; j++) {
			pkt_buf[6 + j] = data[i * FT_FW_PKT_LEN + j];
			fw_ecc ^= pkt_buf[6 + j];
		}

		ft5x06_i2c_write(client, pkt_buf,
						 FT_FW_PKT_LEN + FT_FW_PKT_META_LEN);
		msleep(FT_FW_PKT_DLY_MS);
	}

	/* send remaining bytes */
	if ((data_len) % FT_FW_PKT_LEN > 0) {
		temp = pkt_num * FT_FW_PKT_LEN;
		pkt_buf[2] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[3] = (u8) temp;
		temp = (data_len) % FT_FW_PKT_LEN;
		pkt_buf[4] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) {
			pkt_buf[6 + i] = data[pkt_num * FT_FW_PKT_LEN + i];
			fw_ecc ^= pkt_buf[6 + i];
		}

		ft5x06_i2c_write(client, pkt_buf, temp + FT_FW_PKT_META_LEN);
		msleep(FT_FW_PKT_DLY_MS);
	}

	/* send the finishing packet */
	if (is_5336_new_bootloader == FT_BLOADER_VERSION_LZ4 ||
		is_5336_new_bootloader == FT_BLOADER_VERSION_Z7) {
		for (i = 0; i < FT_FINISHING_PKT_LEN_OLD_FW; i++) {
			if (is_5336_new_bootloader  == FT_BLOADER_VERSION_Z7)
				temp = FT_MAGIC_BLOADER_Z7 + i;
			else if (is_5336_new_bootloader ==
					 FT_BLOADER_VERSION_LZ4)
				temp = FT_MAGIC_BLOADER_LZ4 + i;
			pkt_buf[2] = (u8)(temp >> 8);
			pkt_buf[3] = (u8)temp;
			temp = 1;
			pkt_buf[4] = (u8)(temp >> 8);
			pkt_buf[5] = (u8)temp;
			pkt_buf[6] = data[data_len + i];
			fw_ecc ^= pkt_buf[6];

			ft5x06_i2c_write(client,
							 pkt_buf, temp + FT_FW_PKT_META_LEN);
			msleep(FT_FW_PKT_DLY_MS);
		}
	} else if (is_5336_new_bootloader == FT_BLOADER_VERSION_GZF) {
		for (i = 0; i < FT_FINISHING_PKT_LEN_NEW_FW; i++) {
			if (is_5336_fwsize_30)
				temp = FT_MAGIC_BLOADER_GZF_30 + i;
			else
				temp = FT_MAGIC_BLOADER_GZF + i;
			pkt_buf[2] = (u8)(temp >> 8);
			pkt_buf[3] = (u8)temp;
			temp = 1;
			pkt_buf[4] = (u8)(temp >> 8);
			pkt_buf[5] = (u8)temp;
			pkt_buf[6] = data[data_len + i];
			fw_ecc ^= pkt_buf[6];

			ft5x06_i2c_write(client,
							 pkt_buf, temp + FT_FW_PKT_META_LEN);
			msleep(FT_FW_PKT_DLY_MS);

		}
	}

	/* verify checksum */
	w_buf[0] = FT_REG_ECC;
	ft5x06_i2c_read(client, w_buf, 1, r_buf, 1);
	if (r_buf[0] != fw_ecc) {
		dev_err(&client->dev, "ECC error! dev_ecc=%02x fw_ecc=%02x\n",
				r_buf[0], fw_ecc);
		return -EIO;
	}

	/* reset */
	w_buf[0] = FT_REG_RESET_FW;
	ft5x06_i2c_write(client, w_buf, 1);
	msleep(ts_data->pdata->soft_rst_dly);

	dev_info(&client->dev, "Firmware upgrade successful\n");

	return 0;
}

#if TPD_AUTO_UPGRADE
static unsigned char CTPM_FW1[] = {
#include "ft_app_ic_biel_1080p.txt"
};

static unsigned char CTPM_FW2[] = {
#include "ft_app_ic_biel_720p.txt"
};

static unsigned char CTPM_FW3[] = {
#include "ft_app_ic_tpk_1080p.txt"
};

static unsigned char CTPM_FW4[] = {
#include "ft_app_ic_tpk_720p.txt"
};

static unsigned char CTPM_FW5[] = {
#include "ft_app_ic_oufei_720p.txt"
};

static unsigned char CTPM_FW6[] = {
#include "ft_app_ic_lens_720p.txt"
};

static unsigned char CTPM_FW7[] = {
#include "ft_app_ic_boe_720p_a9a.txt"
};

static unsigned char CTPM_FW8[] = {
#include "ft_app_ic_oufei_720p_a9a.txt"
};

static unsigned char CTPM_FW9[] = {
#include "ft_app_ic_lens_720p_a9a.txt"
};

static unsigned char CTPM_FW10[] = {
#include "ft_app_ic_ebbg_720p_a9a.txt"
};

static unsigned char CTPM_FW11[] = {
#include "ft_app_ic_tianma_720p_a9a.txt"
};


static u8 fts_ctpm_update_project_setting(struct i2c_client *client)
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

static int fts_ctpm_fw_upgrade_with_i_file(struct ft5x06_ts_data *data)
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

	if ((ic_type != CTP_IC_TYPE_0) && (ic_type != CTP_IC_TYPE_1))
		CTP_ERROR("IC type dismatch, please check");

	if (vendor_id == 0xA8 || vendor_id == 0x00 || ic_type == 0xA3 || ic_type == 0x00) {
		CTP_ERROR("vend_id read error, need project");
		vendor_id = fts_ctpm_update_project_setting(client);
		flag_TPID = 1;
	}

if ((strcmp(boardid_info_tp, "S88509A1_M27") == 0) || (strcmp(boardid_info_tp, "S88509A1_M20") == 0) || (strcmp(boardid_info_tp, "S88509A1_M21") == 0)) {
	CTP_INFO("Update FW For A9");
	if (vendor_id == VENDOR_BIEL_1080P) {
		pbt_buf = CTPM_FW1;
		fw_len = sizeof(CTPM_FW1);
		CTP_DEBUG("update firmware size:%d", fw_len);
	} else if (vendor_id == VENDOR_BIEL_720P) {
		pbt_buf = CTPM_FW2;
		fw_len = sizeof(CTPM_FW2);
		CTP_DEBUG("update firmware size:%d", fw_len);
	} else if (vendor_id == VENDOR_TPK_1080P) {
		pbt_buf = CTPM_FW3;
		fw_len = sizeof(CTPM_FW3);
		CTP_DEBUG("update firmware size:%d", fw_len);
	} else if (vendor_id == VENDOR_TPK_720P) {
		pbt_buf = CTPM_FW4;
		fw_len = sizeof(CTPM_FW4);
		CTP_DEBUG("update firmware size:%d", fw_len);
	} else if (vendor_id == VENDOR_OUFEI_720P) {
		pbt_buf = CTPM_FW5;
		fw_len = sizeof(CTPM_FW5);
		CTP_DEBUG("update firmware size:%d", fw_len);
	} else if (vendor_id == VENDOR_LENS_720P) {
		pbt_buf = CTPM_FW6;
		fw_len = sizeof(CTPM_FW6);
		CTP_DEBUG("update firmware size:%d", fw_len);
	} else {
		CTP_ERROR("read vendor_id fail");
		return -EPERM;
	}
} else if (strcmp(boardid_info_tp, "S88509A1_M50") == 0) {
	CTP_INFO("Update FW For A9A");
	if (vendor_id == VENDOR_BIEL_720P) {
		if (LCD_Maker == 0x35) {
		pbt_buf = CTPM_FW7;
		fw_len = sizeof(CTPM_FW7);
		CTP_DEBUG("BOE & BOE");
		} else if (LCD_Maker == 0x37) {
		pbt_buf = CTPM_FW10;
		fw_len = sizeof(CTPM_FW10);
		CTP_DEBUG("BOE & EBBG");
		} else {
					return -EPERM;
		}
	} else if (vendor_id == VENDOR_OUFEI_720P) {
		if (LCD_Maker == 0x35) {
			pbt_buf = CTPM_FW8;
			fw_len = sizeof(CTPM_FW8);
			CTP_DEBUG("OUFEI & BOE");
		} else if (LCD_Maker == 0x36) {
			pbt_buf = CTPM_FW11;
			fw_len = sizeof(CTPM_FW11);
			CTP_DEBUG("OUFEI & TIANMA");
		} else {
				 return -EPERM;
		}
	} else if (vendor_id == VENDOR_LENS_720P) {
		pbt_buf = CTPM_FW9;
		fw_len = sizeof(CTPM_FW9);
		CTP_DEBUG("update firmware size:%d", fw_len);
	} else {
		CTP_ERROR("read vendor_id fail");
		return -EPERM;
	}

} else {
	 CTP_ERROR("Error Board ID.");
		 return -EPERM;
}

	CTP_DEBUG("update firmware size:%d", fw_len);
	if (sizeof(CTPM_FW1) < 8 || sizeof(CTPM_FW1) > 32 * 1024 || sizeof(CTPM_FW3) < 8 || sizeof(CTPM_FW3) > 32 * 1024) {
		CTP_ERROR("FW length error\n");
		return -EPERM;
	}

	if ((pbt_buf[fw_len - 8] ^ pbt_buf[fw_len - 6]) == 0xFF
		&& (pbt_buf[fw_len - 7] ^ pbt_buf[fw_len - 5]) == 0xFF
		&& (pbt_buf[fw_len - 3] ^ pbt_buf[fw_len - 4]) == 0xFF) {

		if (vendor_id != pbt_buf[fw_len - 1]) {
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

	if (!strcmp(name, "ftech,panel-coords")) {
		pdata->panel_minx = coords[0];
		pdata->panel_miny = coords[1];
		pdata->panel_maxx = coords[2];
		pdata->panel_maxy = coords[3];
	} else if (!strcmp(name, "ftech,display-coords")) {
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

	pdata->name = "ftech";
	rc = of_property_read_string(np, "ftech,name", &pdata->name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read name\n");
		return rc;
	}

	rc = ft5x06_get_dt_coords(dev, "ftech,panel-coords", pdata);
	if (rc && (rc != -EINVAL))
		return rc;

	rc = ft5x06_get_dt_coords(dev, "ftech,display-coords", pdata);
	if (rc)
		return rc;


	pdata->i2c_pull_up = of_property_read_bool(np, "ftech,i2c-pull-up");

	pdata->no_force_update = of_property_read_bool(np,"ftech,no-force-update");
	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "ftech,reset-gpio",
						0, &pdata->reset_gpio_flags);
	if (pdata->reset_gpio < 0)
		return pdata->reset_gpio;

	pdata->irq_gpio = of_get_named_gpio_flags(np, "ftech,irq-gpio", 0, &pdata->irq_gpio_flags);
	if (pdata->irq_gpio < 0)
		return pdata->irq_gpio;

	pdata->fw_name = "ft_fw.bin";
	rc = of_property_read_string(np, "ftech,fw-name", &pdata->fw_name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw name\n");
		return rc;
	}

	rc = of_property_read_u32(np, "ftech,group-id", &temp_val);
	if (!rc)
		pdata->group_id = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "ftech,hard-reset-delay-ms",
							  &temp_val);
	if (!rc)
		pdata->hard_rst_dly = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "ftech,soft-reset-delay-ms",
							  &temp_val);
	if (!rc)
		pdata->soft_rst_dly = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "ftech,num-max-touches", &temp_val);
	if (!rc)
	pdata->num_max_touches = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "ftech,fw-delay-aa-ms", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay aa\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_aa =  temp_val;

	rc = of_property_read_u32(np, "ftech,fw-delay-55-ms", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay 55\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_55 =  temp_val;

	rc = of_property_read_u32(np, "ftech,fw-upgrade-id1", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw upgrade id1\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.upgrade_id_1 =  temp_val;

	rc = of_property_read_u32(np, "ftech,fw-upgrade-id2", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw upgrade id2\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.upgrade_id_2 =  temp_val;

	rc = of_property_read_u32(np, "ftech,fw-delay-readid-ms",
							  &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay read id\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_readid =  temp_val;

	rc = of_property_read_u32(np, "ftech,fw-delay-era-flsh-ms",
							  &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay erase flash\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_erase_flash =  temp_val;

	pdata->info.auto_cal = of_property_read_bool(np,
						   "ftech,fw-auto-cal");

	pdata->fw_vkey_support = of_property_read_bool(np,
							 "ftech,fw-vkey-support");

	pdata->ignore_id_check = of_property_read_bool(np,
							 "ftech,ignore-id-check");

	rc = of_property_read_u32(np, "ftech,family-id", &temp_val);
	if (!rc)
		pdata->family_id = temp_val;
	else
		return rc;

	prop = of_find_property(np, "ftech,button-map", NULL);
	if (prop) {
		num_buttons = prop->length / sizeof(temp_val);
		if (num_buttons > MAX_BUTTONS)
			return -EINVAL;

		rc = of_property_read_u32_array(np,
										"ftech,button-map", button_map,
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

void parse_cmldine_for_tp(void)
{
		char *boadrid_start;

			 boadrid_start = strstr(saved_command_line, "board_id=");

		if (boadrid_start != NULL) {
						strncpy(boardid_info_tp, boadrid_start+sizeof("board_id=")-1, 12);
						CTP_INFO("boardid_info_tp:%s\n", boardid_info_tp);
		} else
			pr_debug("boarid not define!\n");
}

static DEVICE_ATTR(disable_keys, S_IWUSR | S_IRUSR, ft5x06_ts_disable_keys_show,
		   ft5x06_ts_disable_keys_store);

static struct attribute *ft5x06_ts_attrs[] = {
    &dev_attr_disable_keys.attr,
	NULL
};

static const struct attribute_group ft5x06_ts_attr_group = {
	.attrs = ft5x06_ts_attrs,
};

static int ft5x06_proc_init(struct ft5x06_ts_data *data)
{
       struct i2c_client *client = data->client;

       int ret = 0;
       char *buf, *path = NULL;
       char *key_disabler_sysfs_node;
       struct proc_dir_entry *proc_entry_tp = NULL;
       struct proc_dir_entry *proc_symlink_tmp = NULL;

       buf = kzalloc(sizeof(struct ft5x06_ts_data), GFP_KERNEL);
       if (buf)
               path = "/devices/soc.0/78b9000.i2c/i2c-5/5-003e";

       proc_entry_tp = proc_mkdir("touchpanel", NULL);
       if (proc_entry_tp == NULL) {
               dev_err(&client->dev, "Couldn't create touchpanel dir in procfs\n");
               ret = -ENOMEM;
       }

       key_disabler_sysfs_node = kzalloc(sizeof(struct ft5x06_ts_data), GFP_KERNEL);
       if (key_disabler_sysfs_node)
               sprintf(key_disabler_sysfs_node, "/sys%s/%s", path, "disable_keys");
       proc_symlink_tmp = proc_symlink("capacitive_keys_enable",
                       proc_entry_tp, key_disabler_sysfs_node);
       if (proc_symlink_tmp == NULL) {
               dev_err(&client->dev, "Couldn't create capacitive_keys_enable symlink\n");
               ret = -ENOMEM;
       }

       kfree(buf);
       kfree(key_disabler_sysfs_node);
       return ret;
}

static int ft5x06_ts_probe(struct i2c_client *client,
						   const struct i2c_device_id *id)
{
	struct ft5x06_ts_platform_data *pdata;
	struct ft5x06_ts_data *data;
	struct input_dev *input_dev;
	struct dentry *temp;
	u8 reg_value;
	u8 reg_addr;
	int err;
	int len;
	u8 ic_name;
#if TPD_AUTO_UPGRADE
	int ret_auto_upgrade = 0;
	int i;
#endif

	temp = NULL;
	update_client = client;

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
	if (!data)
		return -ENOMEM;

	input_dev = input_allocate_device();
	if (!input_dev)
		return -ENOMEM;

	data->input_dev = input_dev;
	data->client = client;
	data->pdata = pdata;

	input_dev->name = "ft5x06_720p";
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
		if (err)
			goto free_irq_gpio;

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

	disable_irq(data->client->irq);



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
	ft5x06_update_fw_vendor_id(data);

	FT_STORE_TS_INFO(data->ts_info, data->family_id, data->pdata->name,
					 data->pdata->num_max_touches, data->pdata->group_id,
					 data->pdata->fw_vkey_support ? "yes" : "no",
					 data->pdata->fw_name, data->fw_ver[0],
					 data->fw_ver[1], data->fw_ver[2]);

#if BoardId_SUPPORT_FW
	parse_cmldine_for_tp();
#endif

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

#if CTP_CHARGER_DETECT
	batt_psy = power_supply_get_by_name("usb");
	if (!batt_psy)
		CTP_DEBUG("tp battery supply not found\n");
#endif

        err = sysfs_create_group(&client->dev.kobj, &ft5x06_ts_attr_group);
	if (err) {
		dev_err(&client->dev, "Failure %d creating sysfs group\n",
			err);
		goto free_reset_gpio;
        }

        ft5x06_proc_init(data);
	enable_irq(data->client->irq);

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

        sysfs_remove_group(&client->dev.kobj, &ft5x06_ts_attr_group);

	input_unregister_device(data->input_dev);

	return 0;
}

static ssize_t ft5x06_ts_disable_keys_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	const char c = data->disable_keys ? '1' : '0';
	return sprintf(buf, "%c\n", c);
}

static ssize_t ft5x06_ts_disable_keys_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	int i;

	if (sscanf(buf, "%u", &i) == 1 && i < 2) {
		data->disable_keys = (i == 1);
		return count;
	} else {
		dev_dbg(dev, "disable_keys write error\n");
		return -EINVAL;
	}
}

static const struct i2c_device_id ft5x06_ts_id[] = {
	{"ft5x06_720p", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ft5x06_ts_id);

#ifdef CONFIG_OF
static struct of_device_id ft5x06_match_table[] = {
	{.compatible = "focaltech,5336",},
	{},
};
#else
#define ft5x06_match_table NULL
#endif

static struct i2c_driver ft5x06_ts_driver = {
	.probe = ft5x06_ts_probe,
	.remove = ft5x06_ts_remove,
	.driver = {
		.name = "ft5x06_720p",
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
