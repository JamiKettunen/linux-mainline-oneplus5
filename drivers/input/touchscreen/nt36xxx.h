// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2010 - 2017 Novatek, Inc.
 */

#ifndef NT36XXX_H
#define NT36XXX_H

#include <linux/i2c.h>

#include "nt36xxx_mem_map.h"

/* GPIO number */
#define NVTTOUCH_INT_PIN 943

/* INT trigger mode */
#define INT_TRIGGER_TYPE IRQ_TYPE_EDGE_RISING

/* I2C driver info */
#define NVT_I2C_NAME "NVT-ts"
#define I2C_BLDR_Address 0x01
#define I2C_FW_Address 0x01
#define I2C_HW_Address 0x62

/* Input device info */
#define NVT_TS_NAME "NVTCapacitiveTouchScreen"

/* Touch info */
#define TOUCH_DEFAULT_MAX_WIDTH 1080
#define TOUCH_DEFAULT_MAX_HEIGHT 2246
#define TOUCH_MAX_FINGER_NUM 10
#define TOUCH_FORCE_NUM 1000

/* Point data length */
#define POINT_DATA_LEN 65

struct nvt_i2c {
	struct i2c_client *client;
	struct input_dev *input;

	struct work_struct ts_work;
	struct workqueue_struct *ts_workq;

	bool dev_pm_suspend;
	struct completion dev_pm_suspend_completion;

	struct regulator *vddio_reg;
	struct regulator *lab_reg;
	struct regulator *ibb_reg;
	const char *vddio_reg_name;
	const char *lab_reg_name;
	const char *ibb_reg_name;

	int8_t phys[32];
	uint8_t fw_ver;
	uint8_t x_num;
	uint8_t y_num;
	uint16_t abs_x_max;
	uint16_t abs_y_max;
	uint8_t max_touch_num;
	uint8_t max_button_num;
	unsigned int int_trigger_type;
	int irq_gpio;
	int reset_gpio;
	struct mutex lock;
	struct mutex mdata_lock;
	const struct nvt_ts_mem_map *mmap;
	uint8_t carrier_system;
	uint16_t nvt_pid;
};

typedef enum {
	RESET_STATE_INIT = 0xA0, /* IC reset */
	RESET_STATE_REK, /* ReK baseline */
	RESET_STATE_REK_FINISH, /* Baseline is ready */
	RESET_STATE_NORMAL_RUN, /* Normal run */
	RESET_STATE_MAX = 0xAF
} RST_COMPLETE_STATE;

typedef enum {
	EVENT_MAP_HOST_CMD = 0x50,
	EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE = 0x51,
	EVENT_MAP_RESET_COMPLETE = 0x60,
	EVENT_MAP_FWINFO = 0x78,
	EVENT_MAP_PROJECTID = 0x9A,
} I2C_EVENT_MAP;

#endif
