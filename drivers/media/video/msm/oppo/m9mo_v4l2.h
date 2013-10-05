/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef M9MO_V412_H
#define M9MO_V412_H
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <mach/camera.h>
#include <media/msm_camera.h>
#include <media/v4l2-subdev.h>
#include "msm_camera_i2c.h"
#include "msm_camera_eeprom.h"

char m9mo_proc_data[256] = {0x00};
char m9mo_cmd[16] = {0x00};
char m9mo_category = 0x00;
char m9mo_byte = 0x00;
char m9mo_read_data[16] = {0x00};

enum m9mo_focus_state{
	FOCUSED_SUCCESS = 0,
	FOCUSING,
	FOCUSED_FAILED,
	FOCUS_MAX,
};
enum m9mo_focus_state current_status;
enum m9mo_focus_state last_status = FOCUSING;

typedef enum
{
	M9MO_AE_MODE_AUTO,
	M9MO_AE_MODE_TOUCH,
	M9MO_AE_MODE_FACE,
	M9MO_AE_MODE_MAX
}m9mo_ae_mode;
#endif

