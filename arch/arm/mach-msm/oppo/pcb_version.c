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
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/string.h>

#include <linux/pcb_version.h>

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
	else if (!strcmp(s, "n1w_evt"))
		current_pcb_version_num = PCB_VERSION_EVT_N1W;
	else if (!strcmp(s, "n1w_dvt"))
		current_pcb_version_num = PCB_VERSION_DVT_N1W;
	else if (!strcmp(s, "n1w_pvt"))
		current_pcb_version_num = PCB_VERSION_PVT_N1W;

	return 0;
}
__setup("oppo.pcb_version=", board_pcb_verson_init);

char *get_hw_pcb_version(void)
{
	char *hw_version = "NULL";
	switch(get_pcb_version()) {
		case PCB_VERSION_EVB:
		case PCB_VERSION_EVB_TD:

			hw_version ="EVB";
			break;
		case PCB_VERSION_EVT:
		case PCB_VERSION_EVT_TD:
		case PCB_VERSION_EVT_N1:
		case PCB_VERSION_EVT_N1F:
		case PCB_VERSION_EVT_N1W:	
			hw_version = "EVT";
			break;
		case PCB_VERSION_DVT:
		case PCB_VERSION_DVT_TD:
		case PCB_VERSION_DVT_N1F:
		case PCB_VERSION_DVT_N1T:
		case PCB_VERSION_DVT_N1W:	
			hw_version = "DVT";
			break;
		case PCB_VERSION_PVT:
		case PCB_VERSION_PVT_TD:
		case PCB_VERSION_PVT_N1F:
		case PCB_VERSION_PVT_N1T:
		case PCB_VERSION_PVT_N1W:	
			hw_version = "PVT";
			break;
		case PCB_VERSION_PVT2_TD:
			hw_version = "PVT2";
			break;
		case PCB_VERSION_PVT3_TD:
			hw_version = "PVT3";
			break;
		case PCB_VERSION_EVT3_N1F:
		case PCB_VERSION_EVT3_N1T:
			hw_version = "EVT3";
			break;
		default:
			hw_version = "UNKOWN";
		}

	return hw_version;
}
EXPORT_SYMBOL(get_hw_pcb_version);

char *get_hw_rf_version(void)
{
	char *rf_version ="NULL";
	if (get_pcb_version() >= PCB_VERSION_EVB && get_pcb_version() <= PCB_VERSION_PVT) {
		rf_version = "X909";
	} else if (get_pcb_version() >= PCB_VERSION_EVB_TD && get_pcb_version() <= PCB_VERSION_PVT3_TD) {
		rf_version = "X909T";
	} else if (get_pcb_version() == PCB_VERSION_EVT_N1 || (get_pcb_version() >= PCB_VERSION_EVT3_N1T && get_pcb_version() <= PCB_VERSION_PVT_N1T)) {
		rf_version = "N1T";
	} else if (get_pcb_version() >= PCB_VERSION_EVT_N1F && get_pcb_version() <= PCB_VERSION_PVT_N1F) {
		rf_version = "N1";
	} else if (get_pcb_version() >= PCB_VERSION_EVT_N1W && get_pcb_version() <= PCB_VERSION_PVT_N1W) {
		rf_version = "N1W";
	}

	return rf_version;
}
EXPORT_SYMBOL(get_hw_rf_version);
