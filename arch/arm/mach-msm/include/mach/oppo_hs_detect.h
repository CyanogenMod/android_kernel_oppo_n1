/*
 * Copyright (C) 2013 The CyanogenMod Project
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _OPPO_HS_DETECT_H_
#define _OPPO_HS_DETECT_H_

enum {
	NO_DEVICE = 0,
	HS_WITH_MIC,
	HS_WITHOUT_MIC
};

int oppo_hs_detect_power(int enable);
void oppo_hs_detect_update(int hs_on, int mic_on);

#endif /* _OPPO_HS_DETECT_H_ */
