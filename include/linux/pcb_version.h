/************************************************************
** Copyright (C), 2010-2013, OPPO Mobile Comm Corp., Ltd
** All rights reserved.
************************************************************/
#ifndef _PCB_VERSION_H
#define _PCB_VERSION_H

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

int get_pcb_version(void);
char *get_hw_pcb_version(void);
char *get_hw_rf_version(void);

#endif /* _PCB_VERSION_H */
