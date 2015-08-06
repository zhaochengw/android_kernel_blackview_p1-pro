
/* Copyright Statement:
 *  
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/


#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"
#include <cust_adc.h>    	// dongteng add for lcm detect
// dongteng add for lcm detect ,read adc voltage
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  			(540)
#define FRAME_HEIGHT 			(960)

#define MIN_VOLTAGE (1450)     // dongteng add for lcm detect
#define MAX_VOLTAGE (1549)     // dongteng add for lcm detect
#define COMPARE_BY_ADC   0

#define REGFLAG_DELAY          	0XFE
#define REGFLAG_END_OF_TABLE  	0xFFF   // END OF REGISTERS MARKER

#define LCM_ID_HX8389B			0x83890c

#define LCM_DSI_CMD_MODE									0
#define MIPI_VIDEO_MODE

#ifndef TRUE
    #define   TRUE     1
#endif
 
#ifndef FALSE
    #define   FALSE    0
#endif
static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

 struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static unsigned int lcm_compare_id(void);
static struct LCM_setting_table lcm_initialization_setting[] = {
	
	
{0xB9,  3,  {0xFF,0x83,0x89}},

{0xB1, 16,  {0x7F,0x09,0x09,0x32,0x32,0x50,0x10,0xF2,0x58,0x80,0x20,0x20,0xF8,0xAA,0xAA,0xA0,0x00,0x80,0x30,0x00}},

{0xB2, 10,  {0x82,0x50,0x05,0x07,0x40,0x38,0x11,0x64,0x55,0x09}},

{0xB4, 11,  {0x70,0x70,0x70,0x70,0x00,0x00,0x10,0x76,0x10,0x76,0x90}},

{0xD3, 35,  {0x00,0x00,0x00,0x00,0x00,0x08,0x00,0x32,0x10,0x00,0x00,0x00,0x03,0xC6,0x03,0xC6,0x00,0x00,0x00,0x00,0x35,0x33,0x04,0x04,
0x37,0x00,0x00,0x00,0x05,0x08,0x00,0x00,0x0A,0x00,0x01}},

{0xD5, 38,  {0x18,0x18,0x18,0x18,0x19,0x19,0x18,0x18,0x20,0x21,0x24,0x25,0x18,0x18,0x18,0x18,0x00,0x01,0x04,0x05,0x02,0x03,0x06,0x07,0x18,
0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18}},

{0xD6, 38,  {0x18,0x18,0x18,0x18,0x18,0x18,0x19,0x19,0x25,0x24,0x21,0x20,0x18,0x18,0x18,0x18,0x07,0x06,0x03,0x02,0x05,0x04,0x01,0x00,0x18,
0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18}},

{0xE0, 42,  {0x00,0x0F,0x16,0x35,0x3B,0x3F,0x21,0x43,0x07,0x0B,0x0D,0x18,0x0E,0x10,0x12,0x11,0x13,0x06,0x10,0x13,0x18,0x00,0x0F,0x15,
0x35,0x3B,0x3F,0x21,0x42,0x07,0x0B,0x0D,0x18,0x0D,0x11,0x13,0x11,0x12,0x07,0x11,0x12,0x17}},

{0xB6,  2,  {0x6D,0x6D}},
{0xD2,  1,  {0x33}},
{0xC7,  4,{0x00,0x80,0x00,0xC0}},
{0xCC,  1,  {0x02}},



// Sleep Out
{0x11, 1,  {0X00}},
{REGFLAG_DELAY, 150, {}}, 

// Display On
{0x29, 1,  {0X00}}, 
{REGFLAG_DELAY, 120, {}},


{REGFLAG_END_OF_TABLE, 0x00, {}}
};

/*

static struct LCM_setting_table lcm_set_window[] = {
	{0x2A,	4,	{0x00, 0x00, (FRAME_WIDTH>>8), (FRAME_WIDTH&0xFF)}},
	{0x2B,	4,	{0x00, 0x00, (FRAME_HEIGHT>>8), (FRAME_HEIGHT&0xFF)}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
*/

static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 50, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},

    // Sleep Mode On
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

/*
static struct LCM_setting_table lcm_backlight_level_setting[] = {
	{0x51, 1, {0xFF}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
*/

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }
	
}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

		// enable tearing-free
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_DISABLED;//LCM_DBI_TE_MODE_VSYNC_OR_HSYNC;//LCM_DBI_TE_MODE_DISABLED;//LCM_DBI_TE_MODE_VSYNC_ONLY;
		params->dbi.te_edge_polarity		= LCM_POLARITY_FALLING;

#if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
#else
		//params->dsi.mode   = SYNC_EVENT_VDO_MODE;
		params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif
	

		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_TWO_LANE;
	
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding 	= LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format		= LCM_DSI_FORMAT_RGB888;
		
		params->dsi.intermediat_buffer_num = 2;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage
		
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		
		params->dsi.word_count=540*3;	//DSI CMD mode need set these two bellow params, different to 6577

		params->dsi.vertical_active_line=960;
/*

			params->dsi.vertical_sync_active				= 4;  //---3
			params->dsi.vertical_backporch					= 8; //---14
			params->dsi.vertical_frontporch 				= 8;  //----8
			params->dsi.vertical_active_line				= FRAME_HEIGHT; 
	
			params->dsi.horizontal_sync_active				= 6; //96
			params->dsi.horizontal_backporch				= 30; //96
			params->dsi.horizontal_frontporch				= 30; //48
			params->dsi.horizontal_active_pixel 			= FRAME_WIDTH;
*/
#if 1  //new LCD
			params->dsi.vertical_sync_active				= 2; //5
			params->dsi.vertical_backporch					= 5; //2
			params->dsi.vertical_frontporch					= 9;  //9
			params->dsi.vertical_active_line				= FRAME_HEIGHT; 

			params->dsi.horizontal_sync_active				= 30;//20
			params->dsi.horizontal_backporch				= 40;//46
			params->dsi.horizontal_frontporch				= 26;//21
			params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
#else
			params->dsi.vertical_sync_active				=2;  //---3
			params->dsi.vertical_backporch					= 10; //---14
			params->dsi.vertical_frontporch 				= 5;  //----8
			params->dsi.vertical_active_line				= FRAME_HEIGHT; 
	
			params->dsi.horizontal_sync_active				= 36; //96
			params->dsi.horizontal_backporch				= 36; //96
			params->dsi.horizontal_frontporch				= 24; //48
			params->dsi.horizontal_active_pixel 			= FRAME_WIDTH;	
#endif
				
		// Bit rate calculation
  params->dsi.PLL_CLOCK=238;//208

}




static void lcm_init(void)
{
  push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	
}


static void lcm_suspend(void)
{
#ifdef BUILD_LK

#else
	printk("hx8389:start %s\n", __func__);
#endif
#if 1
	SET_RESET_PIN(1);
  MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(130);
#endif

   // mt_set_gpio_out(GPIO_DISP_LRSTB_PIN, GPIO_OUT_ONE);
   // MDELAY(130);
	//push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);



}


static void lcm_resume(void)
{
#ifdef BUILD_LK

#else
	printk("hx8389:start %s\n", __func__);
#endif
#if 1
	SET_RESET_PIN(1);
  MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(130);
	#endif
 lcm_init();

}

static unsigned int lcm_esd_check(void)
{
	#ifdef BUILD_LK
		printf("lcm_esd_check()\n");
	#else
		printk("lcm_esd_check()\n");
	#endif 
 #ifndef BUILD_LK
	char  buffer[3];
	char  buffer1[4];
	int   array[4];

	if(lcm_esd_test)
	{
		lcm_esd_test = FALSE;
		return TRUE;
	}
	//MDELAY(20); 
	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x0a, buffer, 1);
	
	array[0] = 0x00043700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x09, buffer1, 3);
	
	if(buffer[0]==0x1c&&buffer1[0]==0x80&&buffer1[1]==0x73&&buffer1[2]==0x04)
	{
		#ifdef BUILD_LK
	 printf("esdcheck: 0x%x , 0x%x , 0x%x , 0x%x\n",buffer[0],buffer1[0],buffer1[1],buffer1[2]); 
		#else
		  printk("esdcheck: 0x%x , 0x%x , 0x%x , 0x%x\n",buffer[0],buffer1[0],buffer1[1],buffer1[2]);
	  #endif
		return FALSE;
	}
	else
	{	
			#ifdef BUILD_LK
		printf("%s %d\n FALSE", __func__, __LINE__);
		#else
		printk("%s %d\n FALSE", __func__, __LINE__);
		#endif	 
		return TRUE;
	}
 #endif

}

static unsigned int lcm_esd_recover(void)
{
	
	#ifdef BUILD_LK
		printf("lcm_esd_recover()\n");
	#else
		printk("lcm_esd_recover()\n");
	#endif	
	
	lcm_init();	

	return TRUE;
}
static unsigned int lcm_compare_id(void)
{
 
	unsigned int id=0;
	unsigned char buffer[2];
	unsigned char buffer1[2];
	unsigned char buffer2[2];
	unsigned int array[16];  

	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(30);
	SET_RESET_PIN(1);
	MDELAY(120);//Must over 6 ms

	array[0]=0x00043902;
	array[1]=0x8983FFB9;// page enable
	dsi_set_cmdq(&array, 2, 1);
	MDELAY(10);
	
	 array[0]=0x00023902;
    array[1]=0x000013ba;
    dsi_set_cmdq(&array, 2, 1);
    MDELAY(10);

    array[0] = 0x00023700;// return byte number
    dsi_set_cmdq(&array, 1, 1);
    MDELAY(10);

    read_reg_v2(0xF4, buffer, 2);
    id = buffer[0];
	//MDELAY(10);
	//read_reg_v2(0xF4, buffer, 2);
	//id = buffer[0]; 
	
#if defined(BUILD_LK)
  printf("luke: lcm:%d, buffer[0]: 0x%0x  buffer[1]: 0x%0x \n", __LINE__,buffer[0],buffer[1] );
	printf("luke: lcm:%d, id = 0x%08x\n", __LINE__, id);
#else
	printk("luke: lcm:%d, id = 0x%08x\n", __LINE__, id);
#endif
         
	return  (0x89 == id)?1:0;   
}

LCM_DRIVER hx8389c_qhd_cpt5p0_xlw_glt_s666_lcm_drv = 
{
    .name			= "hx8389c_qhd_cpt5p0_xlw_glt_s666",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
      .compare_id    = lcm_compare_id,	
  //.esd_check = lcm_esd_check,
  //.esd_recover = lcm_esd_recover,	
};

