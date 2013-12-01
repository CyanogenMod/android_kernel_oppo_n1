/************************************************************ 
** Copyright (C), 2010-2013, OPPO Mobile Comm Corp., Ltd
** All rights reserved. 
************************************************************/
#ifndef _PCB_VERSION_H
#define _PCB_VERSION_H

enum {
	MSM_BOOT_MODE_NORMAL,
	MSM_BOOT_MODE_FASTBOOT,
	MSM_BOOT_MODE_RECOVERY,
	MSM_BOOT_MODE_FACTORY,
	MSM_BOOT_MODE_RF,
	MSM_BOOT_MODE_WLAN,
	MSM_BOOT_MODE_CHARGE,
};

/*OPPO 2013-08-23 zhangpan add begin for read hw version*/
enum {
	PCB_VERSION_EVB,
	PCB_VERSION_EVT,
	PCB_VERSION_DVT,
	PCB_VERSION_PVT,
	PCB_VERSION_EVB_TD,
	PCB_VERSION_EVT_TD,
	PCB_VERSION_DVT_TD,
	PCB_VERSION_PVT_TD,
	PCB_VERSION_PVT2_TD,
	PCB_VERSION_PVT3_TD,

	PCB_VERSION_EVT_N1,	 //900mv
	PCB_VERSION_EVT_N1F,	 //1800mv
	PCB_VERSION_EVT3_N1F,
	PCB_VERSION_DVT_N1F,
	PCB_VERSION_PVT_N1F,
	PCB_VERSION_EVT3_N1T,
	PCB_VERSION_DVT_N1T,
	PCB_VERSION_PVT_N1T,
	PCB_VERSION_EVT_N1W,
	PCB_VERSION_DVT_N1W,
	PCB_VERSION_PVT_N1W,

	PCB_VERSION_UNKNOWN,
};
/*OPPO 2013-08-23 zhangpan add end for read hw version*/

int get_boot_mode(void);
int get_pcb_version(void);
char * get_boot_mode_str(void);
char * get_start_reason(void);
void set_need_pin_process_flag(int flag);
int get_sim_status(void);

#endif /* _PCB_VERSION_H */
