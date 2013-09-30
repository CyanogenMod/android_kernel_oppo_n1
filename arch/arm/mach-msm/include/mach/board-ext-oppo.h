/* arch/arm/mach-msm/include/mach/board-ext-oppo.h
 *
 * Oppo board.h extensions
 *
 * Copyright (C) 2013 CyanogenMod
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

#ifndef __ASM_ARCH_MSM_BOARD_EXT_OPPO_H
#define __ASM_ARCH_MSM_BOARD_EXT_OPPO_H

#define MSM_CAMERA_FLASH_SRC_OPPO (0x00000001<<7)

struct msm_camera_sensor_flash_oppo {
	uint32_t low_current;
	uint32_t high_current;
	int (*led_control)(unsigned state);
};

#endif /* __ASM_ARCH_MSM_BOARD_EXT_OPPO_H */
