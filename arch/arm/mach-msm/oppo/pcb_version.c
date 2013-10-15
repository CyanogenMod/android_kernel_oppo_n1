/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
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

#include <asm/setup.h>

#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>

#include <linux/pcb_version.h>

static int ftm_mode = 0;
static int __init board_mfg_mode_init(char *s)
{
	if (!strcmp(s, "normal"))
		ftm_mode = MSM_BOOT_MODE_NORMAL;
	else if (!strcmp(s, "factory2"))
		ftm_mode = MSM_BOOT_MODE_FACTORY;
	else if (!strcmp(s, "ftmrecovery"))
		ftm_mode = MSM_BOOT_MODE_RECOVERY;
	else if (!strcmp(s, "charge"))
		ftm_mode = MSM_BOOT_MODE_CHARGE;
	else if (!strcmp(s, "ftmwifi"))
		ftm_mode = MSM_BOOT_MODE_WLAN;
	else if (!strcmp(s, "ftmrf"))
		ftm_mode = MSM_BOOT_MODE_RF;
	else
		ftm_mode = MSM_BOOT_MODE_NORMAL;

	pr_err("board_mfg_mode_init" "ftm_mode=%d\n", ftm_mode);

	return 0;
}
__setup("oppo_ftm_mode=", board_mfg_mode_init);

int get_boot_mode(void)
{
	return ftm_mode;
}

static char pwron_event[16];
static int __init start_reason_setup(char *str)
{
	strcpy(pwron_event, str);
	printk(KERN_INFO "%s: parse poweron reason %s\n", __func__, pwron_event);

	return 1;
}
__setup("androidboot.startupmode=", start_reason_setup);

char * get_start_reason(void)
{
	return pwron_event;
}

static char boot_mode[16];
static int __init boot_mode_setup(char *str)
{
	strcpy(boot_mode, str);

	printk(KERN_INFO "%s: parse boot_mode is %s\n", __func__, boot_mode);
	return 1;
}
__setup("androidboot.mode=", boot_mode_setup);

char * get_boot_mode_str(void)
{
	return boot_mode;
}

static int current_pcb_version_num = PCB_VERSION_UNKNOWN;
int get_pcb_version(void)
{
	return current_pcb_version_num;
}
EXPORT_SYMBOL(get_pcb_version);

static char *saved_command_line_pcb_version;
int __init board_pcb_verson_init(char *s)
{
	saved_command_line_pcb_version = s;

	if (!strcmp(s, "evb"))
		current_pcb_version_num = PCB_VERSION_EVB;
	else if (!strcmp(s, "evt"))
		current_pcb_version_num = PCB_VERSION_EVT;
	else if (!strcmp(s, "dvt"))
		current_pcb_version_num = PCB_VERSION_DVT;
	else if (!strcmp(s, "pvt"))
		current_pcb_version_num = PCB_VERSION_PVT;
	else if (!strcmp(s, "td_evb"))
		current_pcb_version_num = PCB_VERSION_EVB_TD;
	else if (!strcmp(s, "td_pvt2"))
		current_pcb_version_num = PCB_VERSION_PVT2_TD;
	else if (!strcmp(s, "td_pvt3"))
		current_pcb_version_num = PCB_VERSION_PVT3_TD;
	else if (!strcmp(s, "n1t_evt"))
		current_pcb_version_num = PCB_VERSION_EVT_N1;
	else if (!strcmp(s, "n1f_evt"))
		current_pcb_version_num = PCB_VERSION_EVT_N1F;
	else if (!strcmp(s, "n1f_evt3"))
		current_pcb_version_num = PCB_VERSION_EVT3_N1F;
	else if (!strcmp(s, "n1f_dvt"))
		current_pcb_version_num = PCB_VERSION_DVT_N1F;
	else if (!strcmp(s, "n1f_pvt"))
		current_pcb_version_num = PCB_VERSION_PVT_N1F;
	else if (!strcmp(s, "n1t_evt3"))
		current_pcb_version_num = PCB_VERSION_EVT3_N1T;
	else if (!strcmp(s, "n1t_dvt"))
		current_pcb_version_num = PCB_VERSION_DVT_N1T;
	else if (!strcmp(s, "n1t_pvt"))
		current_pcb_version_num = PCB_VERSION_PVT_N1T;

	return 0;
}
__setup("oppo.pcb_version=", board_pcb_verson_init);

