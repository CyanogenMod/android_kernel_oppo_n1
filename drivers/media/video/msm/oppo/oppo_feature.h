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
#ifndef OPPO_FEATURE_H
#define OPPO_FEATURE_H
#include "msm_sensor.h"


enum OPPO_FEATURE{
    OPPO_CAF = 0,
    OPPO_ASD,
    MAX_FEATURE,
};
struct frame_info_t
{
	int16_t brightness;
	u_int32_t wave_detect;
	u_int32_t exp_time;
	u_int32_t shutter;
	u_int16_t wb;
	u_int8_t  af_position;
	u_int16_t gain;
	int32_t gravity_data[3];
	bool gravity_valid;
	bool capture_start;
	bool single_af;
	int32_t single_af_delay;
	bool wd_valid;
	int32_t wd_delay_frame;
	bool ae_stable;
};

#define WDV_CAF_LOW_THR 			(1500)
#define ANTI_WDV_SHAKE_DELAY 		(20)
#define DO_AF_DELAY 				(2)
struct oppo_interface{
   void (*notify)(struct msm_sensor_ctrl_t*,  int32_t);
   void (*process)(struct msm_sensor_ctrl_t*, struct frame_info_t*);
};

int32_t oppo_scene_detect_init(struct oppo_interface *interface);
int32_t oppo_caf_init(struct oppo_interface *interface);

#endif

