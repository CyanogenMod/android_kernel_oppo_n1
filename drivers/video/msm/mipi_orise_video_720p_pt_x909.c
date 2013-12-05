/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
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

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_orise_x909.h"

static struct msm_panel_info pinfo;
#if 0
static struct mipi_dsi_phy_ctrl dsi_video_mode_phy_db_720p = {
    /* regulator */
	{0x03, 0x0a, 0x04, 0x00, 0x20},
	/* timing */
	{0x83, 0x31, 0x13, 0x00, 0x42, 0x4d, 0x18, 0x35,
	0x21, 0x03, 0x04, 0xa0},
    /* phy ctrl */
	{0x5f, 0x00, 0x00, 0x10},
    /* strength */
	{0xff, 0x00, 0x06, 0x00},
	/* pll control */
	{0x0, 0x0e, 0x30, 0xc0, 0x00, 0x40, 0x03, 0x62,
	0x40, 0x07, 0x07,
	0x00, 0x1a, 0x00, 0x00, 0x02, 0x00, 0x20, 0x00, 0x01 },
};
#endif
#if 1
static struct mipi_dsi_phy_ctrl dsi_video_mode_phy_db_1080p = { 
	/* 1920*1200, RGB888, 4 Lane 60 fps video mode */ 
	/* regulator */ 
	{0x03, 0x0a, 0x04, 0x00, 0x20}, 
	/* timing */ 
	{0xea, 0x9a, 0x3b, 0x00, 0xad, 0xa7, 0x3d, 0x9c, 
	0x42, 0x03, 0x04, 0xa0}, 
	/* phy ctrl */ 
	{0x5f, 0x00, 0x00, 0x10}, 
	/* strength */ 
	{0xff, 0x00, 0x06, 0x00}, 
	/* pll control */ 
	{0x0, 0x7f, 0x31, 0xda, 0x00, 0x50, 0x48, 0x63,
	0x41, 0x0f, 0x01, 
	0x00, 0x14, 0x03, 0x00, 0x02, 0x00, 0x20, 0x00, 0x01 }, 
		//{0x0, 0xcc, 0x31, 0xda, 0x00, 0x40, 0x03, 0x62, 
		//0x40, 0x07, 0x01, 
		//0x00, 0x1a, 0x00, 0x00, 0x02, 0x00, 0x20, 0x00, 0x01 }, 
}; 


#endif

static int __init mipi_video_orise_720p_pt_init(void)
{
	int ret;

	if (msm_fb_detect_client("mipi_video_orise_720p"))
		return 0;

#if 0
	if(get_pcb_version() < 20)
	{
		printk("huyu----%s: lcd is 720p!--\n", __func__);
		pinfo.xres = 720;
		pinfo.yres = 1280;
		pinfo.lcdc.xres_pad = 0;
		pinfo.lcdc.yres_pad = 0;
		
		pinfo.type = MIPI_VIDEO_PANEL;
		pinfo.pdest = DISPLAY_1;
		pinfo.wait_cycle = 0;
		pinfo.bpp = 24;
		pinfo.lcdc.h_back_porch = 100;
		pinfo.lcdc.h_front_porch = 100;
		pinfo.lcdc.h_pulse_width = 8;
		pinfo.lcdc.v_back_porch = 2;		//32;
		pinfo.lcdc.v_front_porch = 32;
		pinfo.lcdc.v_pulse_width = 1;
		pinfo.lcdc.border_clr = 0;	/* blk */
		pinfo.lcdc.underflow_clr = 0xff;	/* blue */
		pinfo.lcdc.hsync_skew = 0;
		pinfo.bl_max = 200;
		pinfo.bl_min = 1;
		pinfo.fb_num = 2;
		
		pinfo.mipi.mode = DSI_VIDEO_MODE;
		
		pinfo.mipi.pulse_mode_hsa_he = FALSE;	//unrelated
		pinfo.mipi.hfp_power_stop = TRUE;
		pinfo.mipi.hbp_power_stop = FALSE;
		pinfo.mipi.hsa_power_stop = FALSE;		//unrelated
		pinfo.mipi.eof_bllp_power_stop = FALSE;
		pinfo.mipi.bllp_power_stop = FALSE;
		
		pinfo.mipi.traffic_mode = DSI_NON_BURST_SYNCH_EVENT;	//or DSI_BURST_MODE
		pinfo.mipi.dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
		pinfo.mipi.vc = 0;
		pinfo.mipi.rgb_swap = DSI_RGB_SWAP_RGB;
		pinfo.mipi.data_lane0 = TRUE;
		pinfo.mipi.data_lane1 = TRUE;
		pinfo.mipi.data_lane2 = TRUE;
		pinfo.mipi.data_lane3 = TRUE;
		pinfo.mipi.t_clk_post = 0x19;
		pinfo.mipi.t_clk_pre = 0x37;
		pinfo.mipi.stream = 0; /* dma_p */
		pinfo.mipi.mdp_trigger = 0;
		pinfo.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;
		pinfo.mipi.frame_rate = 55;
		pinfo.mipi.dsi_phy_db = &dsi_video_mode_phy_db_720p;
		pinfo.mipi.tx_eot_append = TRUE;
		pinfo.mipi.esc_byte_ratio = 4;
	
		//ret = mipi_orise_device_register_720p(&pinfo, MIPI_DSI_PRIM,
							MIPI_DSI_PANEL_720P_PT);
		if (ret)
			printk(KERN_ERR "%s: failed to register device!\n", __func__);
	}
	else
#endif
	{
		printk("huyu----%s: lcd is 1080p!--\n", __func__);
		pinfo.xres = 1080;
		pinfo.yres = 1920;

		pinfo.lcdc.xres_pad = 0;
		pinfo.lcdc.yres_pad = 0;

		pinfo.type = MIPI_VIDEO_PANEL;
		pinfo.pdest = DISPLAY_1;
		pinfo.wait_cycle = 0;
		pinfo.bpp = 24;
#if 1		
		pinfo.lcdc.h_back_porch = 100;//80;
		pinfo.lcdc.h_front_porch = 130;//120;
		pinfo.lcdc.h_pulse_width = 8;
		pinfo.lcdc.v_back_porch = 5;		//must > 4,Otherwise,it will increase the burden of clock	huyu
		pinfo.lcdc.v_front_porch = 3;
		pinfo.lcdc.v_pulse_width = 2;
#else
		pinfo.lcdc.h_back_porch = 50;
		pinfo.lcdc.h_front_porch = 100;
		pinfo.lcdc.h_pulse_width = 10;
		pinfo.lcdc.v_back_porch = 4;	
		pinfo.lcdc.v_front_porch = 4;
		pinfo.lcdc.v_pulse_width = 2;
#endif
		
		pinfo.lcdc.border_clr = 0;	/* blk */
/* OPPO 2013-01-09 zhengzk Modify begin for underrun display blackscreen */
#if 0
		pinfo.lcdc.underflow_clr = 0xff;	/* blue */
#else
		pinfo.lcdc.underflow_clr = 0x00;	/* black */
#endif
/* OPPO 2013-01-09 zhengzk Modify end */
		pinfo.lcdc.hsync_skew = 0;
		pinfo.bl_max = 128;		//for lm3528
		pinfo.bl_min = 1;
		pinfo.fb_num = 2;

		pinfo.mipi.mode = DSI_VIDEO_MODE;

		pinfo.mipi.pulse_mode_hsa_he = FALSE;	//unrelated
		pinfo.mipi.hfp_power_stop = TRUE;
		pinfo.mipi.hbp_power_stop = FALSE;
		pinfo.mipi.hsa_power_stop = FALSE;		//unrelated
		pinfo.mipi.eof_bllp_power_stop = FALSE;
		pinfo.mipi.bllp_power_stop = FALSE;

		pinfo.mipi.traffic_mode = DSI_BURST_MODE;	//or DSI_NON_BURST_SYNCH_EVENT
		pinfo.mipi.dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
		pinfo.mipi.vc = 0;
		pinfo.mipi.rgb_swap = DSI_RGB_SWAP_RGB;
		pinfo.mipi.data_lane0 = TRUE;
		pinfo.mipi.data_lane1 = TRUE;
		pinfo.mipi.data_lane2 = TRUE;
		pinfo.mipi.data_lane3 = TRUE;
		pinfo.mipi.t_clk_post = 0x19;
		pinfo.mipi.t_clk_pre = 0x37;
		pinfo.mipi.stream = 0; /* dma_p */
		pinfo.mipi.mdp_trigger = 0;
		pinfo.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;
		pinfo.mipi.frame_rate = 60;//55;
		pinfo.mipi.dsi_phy_db = &dsi_video_mode_phy_db_1080p;
		pinfo.mipi.tx_eot_append = TRUE;
		pinfo.mipi.esc_byte_ratio = 4;
		ret = mipi_orise_device_register_1080p(&pinfo, MIPI_DSI_PRIM,
							MIPI_DSI_PANEL_720P_PT);
		if (ret)
			printk(KERN_ERR "%s: failed to register device!\n", __func__);
	}

	return ret;
}

module_init(mipi_video_orise_720p_pt_init);
