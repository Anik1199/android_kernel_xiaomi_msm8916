/*
 * Goodix GT9xx touchscreen driver
 *
 * Copyright  (C)  2010 - 2014 Goodix. Ltd.
 * Copyright (C) 2016 XiaoMi, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * Version: 2.4
 * Release Date: 2014/11/28
 */

#ifndef _GOODIX_GT9XX_H_
#define _GOODIX_GT9XX_H_

#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <asm/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#endif
#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define GTP_CONFIG_OF


#define GTP_CUSTOM_CFG		0
#define GTP_CHANGE_X2Y		0
#define GTP_DRIVER_SEND_CFG   1
#define GTP_HAVE_TOUCH_KEY	0
#define GTP_POWER_CTRL_SLEEP  0
#define GTP_ICS_SLOT_REPORT   1

#define GTP_AUTO_UPDATE	   1
#define GTP_HEADER_FW_UPDATE  1
#define GTP_AUTO_UPDATE_CFG   0

#define GTP_COMPATIBLE_MODE   0

#define GTP_CREATE_WR_NODE	1
#define GTP_ESD_PROTECT	   1

#define GTP_WITH_PEN		  0
#define GTP_PEN_HAVE_BUTTON   0

#ifdef WT_GESTURE_WAKEUP
#define GTP_GESTURE_WAKEUP	1
#else
#define GTP_GESTURE_WAKEUP	0
#endif

#define WT_CTP_OPEN_SHORT_TEST 1

#define GTP_DEBUG_ON		  0
#define GTP_DEBUG_ARRAY_ON	0
#define GTP_DEBUG_FUNC_ON	 0

#if GTP_COMPATIBLE_MODE
typedef enum {
	CHIP_TYPE_GT9  = 0,
	CHIP_TYPE_GT9F = 1,
} CHIP_TYPE_T;
#endif

struct goodix_ts_data {
	spinlock_t irq_lock;
	struct i2c_client *client;
	struct input_dev  *input_dev;
	struct hrtimer timer;
	struct work_struct  work;
	s32 irq_is_disable;
	s32 use_irq;
	u16 abs_x_max;
	u16 abs_y_max;
	u8  max_touch_num;
	u8  int_trigger_type;
	u8  green_wake_mode;
	u8  enter_update;
	u8  gtp_is_suspend;
	u8  gtp_rawdiff_mode;
	int  gtp_cfg_len;
	u8  fw_error;
	u8  pnl_init_error;

#if defined(CONFIG_FB)
	struct notifier_block notifier;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif

#if GTP_WITH_PEN
	struct input_dev *pen_dev;
#endif

#if GTP_ESD_PROTECT
	spinlock_t esd_lock;
	u8  esd_running;
	s32 clk_tick_cnt;
#endif
#if GTP_COMPATIBLE_MODE
	u16 bak_ref_len;
	s32 ref_chk_fs_times;
	s32 clk_chk_fs_times;
	CHIP_TYPE_T chip_type;
	u8 rqst_processing;
	u8 is_950;
#endif

};

extern u16 show_len;
extern u16 total_len;
extern int gtp_rst_gpio;
extern int gtp_int_gpio;




/* sensor_opt1 sensor_opt2 Sensor_ID
	GND		 GND		  0
	VDDIO	  GND		  1
	NC		   GND		  2
	GND		 NC/300K	3
	VDDIO	  NC/300K	4
	NC		   NC/300K	5
*/


#define CTP_CFG_GROUP0 {\
0x45, 0x38, 0x04, 0x80, 0x07, 0x0A, 0x34, 0x00, 0x01, 0x88, \
0x32, 0x06, 0x50, 0x46, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, \
0x01, 0x00, 0x08, 0x19, 0x1B, 0x1E, 0x14, 0x0E, 0x0E, 0x0F, \
0x0B, 0x00, 0xBB, 0x32, 0x00, 0x00, 0x00, 0x02, 0x45, 0x19, \
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x01, \
0x00, 0x0A, 0x19, 0x54, 0xC5, 0x02, 0x07, 0x00, 0x00, 0x04, \
0x80, 0x0B, 0x00, 0x6C, 0x0D, 0x00, 0x5F, 0x0F, 0x00, 0x4D, \
0x13, 0x00, 0x45, 0x16, 0x00, 0x45, 0x00, 0x00, 0x00, 0x00, \
0x85, 0x60, 0x35, 0xFF, 0xFF, 0x19, 0x00, 0x52, 0x01, 0x00, \
0x00, 0x64, 0x00, 0x00, 0xFF, 0x7F, 0x02, 0x00, 0x00, 0xD4, \
0x30, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
0x00, 0x00, 0x1C, 0x1A, 0x18, 0x16, 0x14, 0x12, 0x10, 0x0E, \
0x0C, 0x0A, 0x08, 0x06, 0x04, 0x02, 0x00, 0xFF, 0xFF, 0xFF, \
0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
0x00, 0x00, 0x24, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, \
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, \
0x10, 0x27, 0x20, 0x4E, 0x00, 0x0F, 0x14, 0x03, 0x07, 0x00, \
0x00, 0x28, 0x00, 0x0B, 0x0C, 0x00, 0x00, 0x00, 0x03, 0x00, \
0x06, 0x0A, 0x00, 0x01, 0x00, 0x00, 0x01, 0x24, 0x60, 0x00, \
0x00, 0x6B, 0x80, 0x00, 0x01, 0x00, 0xAF, 0x50, 0x3C, 0x28, \
0xB8, 0x0B, 0x00, 0x00, 0x00, 0x00, 0xF1, 0x01\
}


#define CTP_CFG_GROUP1 {\
}


#define CTP_CFG_GROUP2 {\
}


#define CTP_CFG_GROUP3 {\
}

#define CTP_CFG_GROUP4 {\
}


#define CTP_CFG_GROUP5 {\
}


#define GTP_RST_PORT	16
#define GTP_INT_PORT	17

#define GTP_GPIO_AS_INPUT(pin)		do {\
											gpio_direction_input(pin);\
									} while (0)
#define GTP_GPIO_AS_INT(pin)		do {\
											GTP_GPIO_AS_INPUT(pin);\
									} while (0)
#define GTP_GPIO_GET_VALUE(pin)		 gpio_get_value(pin)
#define GTP_GPIO_OUTPUT(pin, level)	  gpio_direction_output(pin, level)
#define GTP_GPIO_REQUEST(pin, label)	gpio_request(pin, label)
#define GTP_GPIO_FREE(pin)			  gpio_free(pin)
#define GTP_IRQ_TAB					 {IRQ_TYPE_EDGE_RISING, IRQ_TYPE_EDGE_FALLING, IRQ_TYPE_LEVEL_LOW, IRQ_TYPE_LEVEL_HIGH}


#if GTP_CUSTOM_CFG
#define GTP_MAX_HEIGHT		800
#define GTP_MAX_WIDTH		480
#define GTP_INT_TRIGGER		0
#else
#define GTP_MAX_HEIGHT		4096
#define GTP_MAX_WIDTH		4096
#define GTP_INT_TRIGGER		1
#endif
#define GTP_MAX_TOUCH		 5


#if GTP_HAVE_TOUCH_KEY
	#define GTP_KEY_TAB  {KEY_MENU, KEY_HOME, KEY_BACK}
#endif


#define GTP_DRIVER_VERSION		  "V2.4<2014/11/28>"
#define GTP_I2C_NAME				"Goodix-TS"
#define GT91XX_CONFIG_PROC_FILE	 "gt9xx_config"
#define GTP_POLL_TIME		 10
#define GTP_ADDR_LENGTH	   2
#define GTP_CONFIG_MIN_LENGTH 186
#define GTP_CONFIG_MAX_LENGTH 240
#define FAIL				  0
#define SUCCESS			   1
#define SWITCH_OFF			0
#define SWITCH_ON			 1


#define GTP_REG_BAK_REF				 0x99D0
#define GTP_REG_MAIN_CLK				0x8020
#define GTP_REG_CHIP_TYPE			   0x8000
#define GTP_REG_HAVE_KEY				0x804E
#define GTP_REG_MATRIX_DRVNUM		   0x8069
#define GTP_REG_MATRIX_SENNUM		   0x806A

#define GTP_FL_FW_BURN			  0x00
#define GTP_FL_ESD_RECOVERY		 0x01
#define GTP_FL_READ_REPAIR		  0x02

#define GTP_BAK_REF_SEND				0
#define GTP_BAK_REF_STORE			   1
#define CFG_LOC_DRVA_NUM				29
#define CFG_LOC_DRVB_NUM				30
#define CFG_LOC_SENS_NUM				31

#define GTP_CHK_FW_MAX				  40
#define GTP_CHK_FS_MNT_MAX			  300
#define GTP_BAK_REF_PATH				"/data/gtp_ref.bin"
#define GTP_MAIN_CLK_PATH			   "/data/gtp_clk.bin"
#define GTP_RQST_CONFIG				 0x01
#define GTP_RQST_BAK_REF				0x02
#define GTP_RQST_RESET				  0x03
#define GTP_RQST_MAIN_CLOCK			 0x04
#define GTP_RQST_RESPONDED			  0x00
#define GTP_RQST_IDLE				   0xFF



#define GTP_READ_COOR_ADDR	0x814E
#define GTP_REG_SLEEP		 0x8040
#define GTP_REG_SENSOR_ID	 0x814A
#define GTP_REG_CONFIG_DATA   0x8047
#define GTP_REG_VERSION	   0x8140
#define GUP_REG_HW_INFO			 0x4220
#define GUP_REG_FW_MSG			  0x41E4
#define GUP_REG_PID_VID			 0x8140


#define RESOLUTION_LOC		3
#define TRIGGER_LOC		   8

#define CFG_GROUP_LEN(p_cfg_grp)  (sizeof(p_cfg_grp) / sizeof(p_cfg_grp[0]))

#define GTP_INFO(fmt, arg...)		printk("<<-GTP-INFO->> "fmt"\n", ##arg)
#define GTP_ERROR(fmt, arg...)		printk("<<-GTP-ERROR->> "fmt"\n", ##arg)
#define GTP_DEBUG(fmt, arg...)		do {\
						if (GTP_DEBUG_ON)\
							printk("<<-GTP-DEBUG->> [%d]"fmt"\n", __LINE__, ##arg);\
					} while (0)
#define GTP_DEBUG_ARRAY(array, num)	do {\
						s32 i;\
						u8 *a = array;\
						if (GTP_DEBUG_ARRAY_ON) {\
							printk("<<-GTP-DEBUG-ARRAY->>\n");\
							for (i = 0; i < (num); i++) {\
								printk("%02x   ", (a)[i]);\
								if ((i + 1) % 10 == 0) {\
									printk("\n");\
								} \
							} \
							printk("\n");\
						} \
					} while (0)
#define GTP_DEBUG_FUNC()		do {\
						if (GTP_DEBUG_FUNC_ON)\
						printk("<<-GTP-FUNC->> Func:%s@Line:%d\n", __func__, __LINE__);\
					} while (0)
#define GTP_SWAP(x, y)			do {\
						typeof(x) z = x;\
						x = y;\
						y = z;\
					} while (0)


#ifdef CONFIG_OF
int gtp_parse_dt_cfg(struct device *dev, u8 *cfg, int *cfg_len, u8 sid);
#endif

#if GTP_GESTURE_WAKEUP
typedef enum {
	DOZE_DISABLED = 0,
	DOZE_ENABLED = 1,
	DOZE_WAKEUP = 2,
} DOZE_T;
#endif


#endif /* _GOODIX_GT9XX_H_ */
