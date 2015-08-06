
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


#ifdef BUILD_LK
#include "platform/mt_gpio.h"
#else
#include <linux/string.h>
#if defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <mach/mt_gpio.h>
#endif
#endif
#include "lcm_drv.h"

#include <cust_adc.h>    	
#define MIN_VOLTAGE (500)     
#define MAX_VOLTAGE (700)     
#define LCM_COMPARE_BY_ADC (0) 

#define FRAME_WIDTH  		(540)
#define FRAME_HEIGHT 		(960)
#define REGFLAG_DELAY       0XFE
#define REGFLAG_END_OF_TABLE      	0xFFF   
#define LCM_ID1  0x98
#define LCM_ID2  0x07
#define LCM_ID3  0x07
#define LCM_DSI_CMD_MODE	0

#ifndef TRUE
    #define   TRUE     1
#endif
#ifndef FALSE
    #define   FALSE    0
#endif

static LCM_UTIL_FUNCS lcm_util = {0};
#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		        lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)				        lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);

 struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};
/*
static void ILI9806C_set_reset_pin(int high){
	mt_set_gpio_mode(GPIO_DISP_LRSTB_PIN, GPIO_MODE_GPIO);
	if(1 == high)
		mt_set_gpio_out(GPIO_DISP_LRSTB_PIN, GPIO_OUT_ONE);
	else
		mt_set_gpio_out(GPIO_DISP_LRSTB_PIN, GPIO_OUT_ZERO);
}
#define SET_RESET_PIN(v)    (ILI9806C_set_reset_pin(v))
*/
static struct LCM_setting_table lcm_initialization_setting[] = {
/*

	//LCDD (Peripheral) Setting		
  	
{0xFF,5,{0xFF,0x98,0x07,0x00,0x01}},     // Change to Page 1
{0x31,1,{0x00}}, // 00  COLUM  //  02  2dot

{0x40,1,{0x55}},  //50        Power Contorl 1
{0x41,1,{0x66}}, //33         Power Contorl 2

{0x42,1,{0x02}}, //33 // Power Contorl 3 
{0x43,1,{0x89}},  // Power Contorl 4 

{0x44,1,{0x88}},  // Power Contorl 5
{0x46,1,{0xCC}},  //DC  // DDVDL pumping clock * 1  // Power Contorl 7

{0x47,1,{0xCC}},  // Power Contorl 8
{0x50,1,{0xA8}},   //68  // Power Contorl 9
{0x51,1,{0xA8}},   // 78  // Power Contorl 10        
    
{0x52,1,{0x00}},  // VCOM Control 1               
{0x53,1,{0x60}}, //75   VCOM     62   70  // VCOM Control 2

{0x60,1,{0x05}}, // SOURCE TIMING ADJUST 1  
{0x61,1,{0x00}}, // 00  // SOURCE TIMING ADJUST 2

{0x62,1,{0x08}}, // 00  // SOURCE TIMING ADJUST 3
{0x63,1,{0x00}}, // 00  // SOURCE TIMING ADJUST 4
{0x64,1,{0x88}},        // SOURCE TIMING ADJUST 5

//++++++++++++++++++ Gamma Setting +++++++++++++++++
{0xA0,1,{0x00}},  // Gamma 0  // Positive Gamma Control 1
{0xA1,1,{0x07}},  // Gamma 4  // Positive Gamma Control 2

{0xA2,1,{0x0E}},  // Gamma 8  // Positive Gamma Control 3
{0xA3,1,{0x0d}},  // Gamma 16 // Positive Gamma Control 4

{0xA4,1,{0x05}},  // Gamma 24  // Positive Gamma Control 5
{0xA5,1,{0x08}}, // Gamma 52   // Positive Gamma Control 6

{0xA6,1,{0x06}},  // Gamma 80   // Positive Gamma Control 7
{0xA7,1,{0x04}},  // Gamma 108  // Positive Gamma Control 8

{0xA8,1,{0x08}},  // Gamma 147  // Positive Gamma Control 9
{0xA9,1,{0x0C}},  // Gamma 175  // Positive Gamma Control 10

{0xAA,1,{0x12}},  // Gamma 203  // Positive Gamma Control 11
{0xAB,1,{0x08}},  // Gamma 231  // Positive Gamma Control 12

{0xAC,1,{0x10}},  // Gamma 239  // Positive Gamma Control 13
{0xAD,1,{0x18}},  // Gamma 247  // Positive Gamma Control 14

{0xAE,1,{0x0F}},  // Gamma 251  // Positive Gamma Control 15
{0xAF,1,{0x00}},  // Gamma 255  // Positive Gamma Control 16

///==============Nagitive
{0xC0,1,{0x00}},  // Gamma 0   // Negative Gamma Control 1
{0xC1,1,{0x08}},  // Gamma 4  // Negative Gamma Control 2

{0xC2,1,{0x0E}},  // Gamma 8  // Negative Gamma Control 3
{0xC3,1,{0x0C}},  // Gamma 16  // Negative Gamma Control 4

{0xC4,1,{0x05}},  // Gamma 24  // Negative Gamma Control 5
{0xC5,1,{0x08}},  // Gamma 52  // Negative Gamma Control 6

{0xC6,1,{0x06}},  // Gamma 80  // Negative Gamma Control 7
{0xC7,1,{0x03}},  // Gamma 108  // Negative Gamma Control 8

{0xC8,1,{0x08}},  // Gamma 147  // Negative Gamma Control 9
{0xC9,1,{0x0c}},  // Gamma 175  // Negative Gamma Control 10

{0xCA,1,{0x15}},  // Gamma 203  // Negative Gamma Control 11
{0xCB,1,{0x09}},  // Gamma 231  // Negative Gamma Control 12

{0xCC,1,{0x0F}},  // Gamma 239  // Negative Gamma Control 13
{0xCD,1,{0x18}},  // Gamma 247  // Negative Gamma Control 14

{0xCE,1,{0x0F}},  // Gamma 251  // Negative Gamma Control 15
{0xCF,1,{0x00}},  // Gamma 255  // Negative Gamma Control 16

{0xFF,5,{0xFF,0x98,0x07,0x00,0x06}},     // Change to Page 6
{0x00,1,{0x21}},
 
{0x01,1,{0x0A}},
{0x02,1,{0x00}}, 
   
{0x03,1,{0x00}},
{0x04,1,{0x0B}},

{0x05,1,{0x01}},
{0x06,1,{0x80}},  
  
{0x07,1,{0x06}},
{0x08,1,{0x01}},

{0x09,1,{0x00}},    
{0x0A,1,{0x00}},  
  
{0x0B,1,{0x00}},    
{0x0C,1,{0x0B}}, //CHANGE 

{0x0D,1,{0x0B}}, //CHANGE 
{0x0E,1,{0x00}},

{0x0F,1,{0x00}},
{0x10,1,{0xF0}},

{0x11,1,{0xF4}},
{0x12,1,{0x04}},

{0x13,1,{0x00}},
{0x14,1,{0x00}},

{0x15,1,{0xC0}},
{0x16,1,{0x08}},

{0x17,1,{0x00}},
{0x18,1,{0x00}},

{0x19,1,{0x00}},
{0x1A,1,{0x00}},

{0x1B,1,{0x00}},
{0x1C,1,{0x00}},

{0x1D,1,{0x00}},
{0x20,1,{0x01}},

{0x21,1,{0x23}},
{0x22,1,{0x45}},

{0x23,1,{0x67}},
{0x24,1,{0x01}},

{0x25,1,{0x23}},
{0x26,1,{0x45}},

{0x27,1,{0x67}},
{0x30,1,{0x01}},

{0x31,1,{0x11}},
{0x32,1,{0x00}},

{0x33,1,{0xEE}},
{0x34,1,{0xFF}},

{0x35,1,{0x22}},  
{0x36,1,{0xCB}},

{0x37,1,{0x22}},
{0x38,1,{0xDA}},

{0x39,1,{0x22}},
{0x3A,1,{0xAD}},

{0x3B,1,{0x22}},
{0x3C,1,{0xBC}},

{0x3D,1,{0x76}},
{0x3E,1,{0x67}},

{0x3F,1,{0x22}},
{0x40,1,{0x22}},

{0x41,1,{0x22}},
{0x42,1,{0x22}},

{0x43,1,{0x22}},

{0xFF,5,{0xFF,0x98,0x07,0x00,0x07}},     // Change to Page 7
{0x02,1,{0x77}},    //DDVDH/DDVDL ADVANCE ADJUST
 
//{0x00,1,{0x11}},    
//{0x17,1,{0x22}}, 

{0x18,1,{0x1d}}, 

{0xFF,5,{0xFF,0x98,0x07,0x00,0x00}},     // Change to Page 0
{0x35,1,{0x00}},  //TE ON	
	*/

// enable ILI9807	
{0xFF,5,{0xFF,0x98,0x07,0x00,0x01}},

{0x31,1,{0x00}},

{0x40,1,{0x55}},

{0x41,1,{0x55}},
	
{0x43,1,{0x89}},
	
{0x44,1,{0x86}},
	
{0x46,1,{0xEE}},

{0x47,1,{0xDD}},

{0x50,1,{0xA0}},

{0x51,1,{0xA0}},

{0x52,1,{0x00}},

{0x53,1,{0x23}},

{0x55,1,{0x25}},

{0x60,1,{0x02}},

{0x61,1,{0x08}},

{0x62,1,{0x08}},

{0x63,1,{0x08}},

{0x64,1,{0x88}},

{0xA0,1,{0x00}},

{0xA1,1,{0x0F}},

{0xA2,1,{0x18}},

{0xA3,1,{0x10}},

{0xA4,1,{0x09}},

{0xA5,1,{0x0D}},

{0xA6,1,{0x07}},

{0xA7,1,{0x05}},

{0xA8,1,{0x03}},

{0xA9,1,{0x09}},

{0xAA,1,{0x11}},

{0xAB,1,{0x08}},

{0xAC,1,{0x0D}},

{0xAD,1,{0x15}},

{0xAE,1,{0x0C}},

{0xAF,1,{0x00}},

{0xC0,1,{0x00}},

{0xC1,1,{0x0F}},

{0xC2,1,{0x18}},

{0xC3,1,{0x10}},

{0xC4,1,{0x0A}},

{0xC5,1,{0x0D}},

{0xC6,1,{0x07}},

{0xC7,1,{0x05}},

{0xC8,1,{0x03}},

{0xC9,1,{0x09}},

{0xCA,1,{0x11}},

{0xCB,1,{0x08}},

{0xCC,1,{0x0D}},

{0xCD,1,{0x15}},

{0xCE,1,{0x0C}},

{0xCF,1,{0x00}},
	
{0xFF,5,{0xFF,0x98,0x07,0x00,0x06}},
	
{0x00,1,{0x21}},

{0x01,1,{0x06}},

{0x02,1,{0x20}},

{0x03,1,{0x02}},

{0x04,1,{0x01}},

{0x05,1,{0x01}},

{0x06,1,{0x80}},

{0x07,1,{0x04}},

{0x08,1,{0x03}},

{0x09,1,{0x00}},

{0x0A,1,{0x00}},

{0x0B,1,{0x00}},

{0x0C,1,{0x01}},

{0x0D,1,{0x01}},

{0x0E,1,{0x00}},

{0x0F,1,{0x00}},

{0x10,1,{0xFF}},

{0x11,1,{0xF0}},

{0x12,1,{0x00}},

{0x13,1,{0x00}},

{0x14,1,{0x00}},

{0x15,1,{0xC0}},

{0x16,1,{0x08}},

{0x17,1,{0x00}},

{0x18,1,{0x00}},

{0x19,1,{0x00}},

{0x1A,1,{0x00}},

{0x1B,1,{0x00}},

{0x1C,1,{0x00}},

{0x1D,1,{0x00}},

{0x20,1,{0x01}},

{0x21,1,{0x23}},

{0x22,1,{0x45}},

{0x23,1,{0x67}},

{0x24,1,{0x01}},

{0x25,1,{0x23}},

{0x26,1,{0x45}},

{0x27,1,{0x67}},

{0x30,1,{0x10}},

{0x31,1,{0x96}},

{0x32,1,{0x87}},

{0x33,1,{0x96}},

{0x34,1,{0x87}},

{0x35,1,{0xAB}},

{0x36,1,{0xCD}},

{0x37,1,{0xAB}},

{0x38,1,{0xCD}},

{0x39,1,{0xDC}},

{0x3A,1,{0xBA}},

{0x3B,1,{0xDC}},

{0x3C,1,{0xBA}},

{0x3D,1,{0x69}},

{0x3E,1,{0x78}},

{0x3F,1,{0x69}},

{0x40,1,{0x78}},

{0x41,1,{0x22}},

{0x42,1,{0x22}},

{0x43,1,{0x22}},

{0xFF,5,{0xFF,0x98,0x07,0x00,0x07}},
	
//{0xFF 5 {0xBC,0x02,0x00,0xbf}},
	
{0xbf,2,{0x02,0x77}},
	
//{0xFF 5 {0xBC,0x02,0x00,0xbf}},

{0xFF,5,{0xFF,0x98,0x07,0x00,0x00}},

{0xB7,2,{0x50,0x02}},

{0xBD,2,{0x00,0x00}},

{0xBC,2,{0x01,0x00}},

{0x36,1,{0x00}}, //00


{0xBC,2,{0x01,0x00}},

{0x3a,1,{0x77}}, 


{0xBC,2,{0x00,0x00}},
	
{0x11,	0,	{0x00}},    

{REGFLAG_DELAY, 150, {}},                             
{0x29,	0,	{0x00}},	//DISPLAY ON    
	                          
{REGFLAG_DELAY, 10, {}},  
 
	
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
	{REGFLAG_DELAY, 10, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 10, {}},

	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

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
    params->dbi.te_mode				= LCM_DBI_TE_MODE_VSYNC_ONLY;
    params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
    params->dsi.mode   = SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE;
    
    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM				= LCM_TWO_LANE;
    
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding 	= LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format	  = LCM_DSI_FORMAT_RGB888;
    
    // Video mode setting		
    params->dsi.intermediat_buffer_num = 2;
    
    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
    
    params->dsi.word_count=540*3;	//DSI CMD mode need set these two bellow params, different to 6577
    params->dsi.vertical_active_line=960;

    params->dsi.vertical_sync_active				=4;
    params->dsi.vertical_backporch				= 16;
    params->dsi.vertical_frontporch				= 20;//10lai
    params->dsi.vertical_active_line				= FRAME_HEIGHT;

    params->dsi.horizontal_sync_active				= 10;///////////////20 20 4  20  14  6
     params->dsi.horizontal_backporch				= 80;//80lai
    params->dsi.horizontal_frontporch				= 80;
    params->dsi.horizontal_blanking_pixel				= 60;////you
    params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

    // Bit rate calculation
	  params->dsi.PLL_CLOCK=208;
    params->dsi.HS_TRAIL=12;	
   // params->dsi.pll_div2=1;		// div2=0,1,2,3;div2_real=1,2,4,4//1               //0
   // params->dsi.fbk_sel=1;		 // fbk_sel=0,1,2,3;fbk_sel_real=1,2,4,4
   // params->dsi.fbk_div =16;		// fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	//16	
   
}

static void lcm_init(void)
{
    SET_RESET_PIN(1);
     MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
//	lcm_initialization_setting[13].para_list[0] += 1;
}

static void lcm_suspend(void)
{	
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	SET_RESET_PIN(1);     
        MDELAY(1);
        SET_RESET_PIN(0);
        MDELAY(10);
        SET_RESET_PIN(1);
        MDELAY(100);

}
//static unsigned int vcom = 0x5000;
static void lcm_resume(void)
{
	
//	unsigned int data_array[16];
	lcm_init();
	#if 0
	//data_array[0] = 0x00023902;
//	data_array[0] = 0x00023902;
//	data_array[1] = 0x00000053;
//	data_array[1] |= vcom;
//	dsi_set_cmdq(&data_array,2,1);
	
	data_array[0] = 0x00023902;
	data_array[1] = 0x00000055;
	data_array[1] |= vcom;
	dsi_set_cmdq(&data_array,2,1);
	
	//data_array[0] = 0x00063902;
	//data_array[1] = 0x0698FFFF;
	//data_array[2] = 0x00000004;
	//dsi_set_cmdq(&data_array,3,1);
	vcom+=0x100;
	#endif
	

	//push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}

/*
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);
	unsigned int data_array[16];
	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	data_array[3]= 0x00053902;
	data_array[4]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[5]= (y1_LSB);
	data_array[6]= 0x002c3909;
	dsi_set_cmdq(data_array, 7, 0);
}

static void lcm_setbacklight(unsigned int level)
{
	unsigned int default_level = 145;
	unsigned int mapped_level = 0;

	//for LGE backlight IC mapping table
	if(level > 255) 
			level = 255;

	if(level >0) 
			mapped_level = default_level+(level)*(255-default_level)/(255);
	else
			mapped_level=0;

	// Refresh value of backlight level.
	lcm_backlight_level_setting[0].para_list[0] = mapped_level;

	push_table(lcm_backlight_level_setting, sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);
}
*/

static struct LCM_setting_table lcm_compare_id_setting[] = {
	

	
  {0xFF,5,{0xFF,0x98,0x07,0x00,0x01}}, 
	{REGFLAG_DELAY, 10, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static unsigned int lcm_compare_id(void)
{
	unsigned int id1 = 0,id2 = 0,id3=0;
	unsigned char buffer[2];
	unsigned int array[16];
      SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
    	SET_RESET_PIN(0);
    	MDELAY(10);
    	SET_RESET_PIN(1);
    	MDELAY(150);
	push_table(lcm_compare_id_setting, sizeof(lcm_compare_id_setting) / sizeof(struct LCM_setting_table), 1);
	array[0] = 0x00023700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x00, buffer, 2);
	id1 = buffer[0]; 
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x01, buffer, 2);
	id2 = buffer[0];
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x01, buffer, 2);
	id3 = buffer[0];
     #ifdef BUILD_LK
	         printf("luke: ili9806e  %d  id1:%x  id2:%x id3:%x \n",__LINE__,id1, id2,id3);
       #endif
#if LCM_COMPARE_BY_ADC
//#ifdef AUXADC_LCM_VOLTAGE_CHANNEL
	    int data[4] = {0,0,0,0};
	    int res = 0;
	    int rawdata = 0;
	    int lcm_vol = 0;
            if(LCM_ID1==id1 && LCM_ID2==id2 && LCM_ID3 ==id3){
		    res = IMM_GetOneChannelValue(0,data,&rawdata);
		    if(res < 0){ 
			#ifdef BUILD_LK
			        printf("sunkui: ili9806e  %d  ADC get LCM_ID voltage error! \n",__LINE__);
			#endif
			return 0;
		    }
		    lcm_vol = data[0]*1000+data[1]*10;
		  #ifdef BUILD_LK
		    		printf("sunkui: ili9806e  %d  ADC get LCM_ID :%d mv \n",__LINE__,lcm_vol);
		   #endif
		    if (lcm_vol>=MIN_VOLTAGE &&lcm_vol <= MAX_VOLTAGE){
			     return 1;
		    }else{
		   	     return 0;
		    }
          }else{
          	return 0;
        }
//#endif
#else
//=============
       if(LCM_ID1==id1 && LCM_ID2==id2 && LCM_ID3 ==id3){
                   return 1;
       	}else{
                   return 0;
	}
#endif
}

LCM_DRIVER ili9807_qhd_ivo5p0_huarui_hxlt_x2605_lcm_drv = 
{
        .name = "ili9807_qhd_ivo5p0_huarui_hxlt_x2605",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id    = lcm_compare_id,	
#if (LCM_DSI_CMD_MODE)
	//.set_backlight	= lcm_setbacklight,
        //.update         = lcm_update,
#endif
};

