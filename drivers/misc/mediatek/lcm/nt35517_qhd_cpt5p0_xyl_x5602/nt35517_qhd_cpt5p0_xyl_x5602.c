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

#define FRAME_WIDTH  										(540)
#define FRAME_HEIGHT 										(960)

#define REGFLAG_DELAY             							0xAB
#define REGFLAG_END_OF_TABLE      							0x100  // END OF REGISTERS MARKER

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

#define LCM_ID       (0x55)

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
     
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},


    // Display ON
    {0x29, 1, {0x00}},
     {REGFLAG_DELAY, 50, {}},
	 {0x32, 1, {0x00}},
{REGFLAG_DELAY, 20, {}},
  {REGFLAG_END_OF_TABLE, 0x00, {}}


};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 10, {}},
    
    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_initialization_setting[] = {
{0xF0, 5 ,{0x55,0xAA,0x52,0x08,0x00}},

//Forward Scan      CTB=CRL=0
{0xB1, 3 ,{0x7C,0x00,0x00}},

// Source hold time
//{0xB6,0x05}},

// Gate EQ control
{0xB7, 2 ,{0x72,0x72}},

// Source EQ control (Mode 2)
{0xB8, 4 ,{0x01,0x04,0x04,0x04}},

// Bias Current
{0xBB, 3 ,{0x53,0x03,0x53}},

// Inversion
{0xBC, 3 ,{0x00,0x00,0x00 }},       // column

//#Frame Rate
{0xBD, 5 ,{0x01,0x38,0x08,0x40,0x01}},

{0xC7, 1 ,{0x70}},

// Display Timing: Dual 8-phase 4-overlap
{0xCA, 11 ,{0x01,0xE4,0xE4,0xE4,0xE4,0xE4,0xE4,0x08,0x08,0x00,0x00}},


//*************************************
// Select CMD2, Page 1
//*************************************
{0xF0, 5 ,{0x55,0xAA,0x52,0x08,0x01}},


// AVDD: 5.5V
{0xB0, 3 ,{0x0A,0x0A,0x0A}},

// AVEE: -5.5V
{0xB1, 3 ,{0x0A,0x0A,0x0A}},

// VCL: -3.5V
{0xB2, 3 ,{0x02,0x02,0x02}},

// VGH: 15.0V
{0xB3, 3 ,{0x10,0x10,0x10}},

// VGLX: -10.0V
{0xB4, 3 ,{0x06,0x06,0x06}},

// AVDD: 3xVDDB
{0xB6, 3 ,{0x44,0x44,0x44}},

// AVEE: -2xVDDB
{0xB7, 3 ,{0x34,0x34,0x34}},

// VCL: -2xVDDB   pump clock : Hsync
{0xB8, 3 ,{0x15,0x15,0x15}},

// VGH: 2xAVDD-AVEE
{0xB9, 3 ,{0x34,0x34,0x34}},

// VGLX: AVEE-AVDD
{0xBA, 3 ,{0x24,0x24,0x24}},

// Set VGMP = 5.2V / VGSP = 0V
{0xBC, 3 ,{0x00,0x8F ,0x00}},

// Set VGMN = -5.2V / VGSN = 0V
{0xBD, 3 ,{0x00,0x8F,0x00}},

// VMSEL 0: 0xBE00  ;  1 : 0xBF00 
{0xC1, 1 ,{0x00}}, 
  
// Set VCOM_offset
{0xBE, 1 ,{0x2D}},//6a 70 74 65 61 63


// Pump:0x00 or PFM:0x50 control
{0xC2, 1 ,{0x00}},

// Gamma Gradient Control
//{ 0xD0,4,{0x0F, 0x0F, 0x10, 0x10}},
//R(+) MCR cmd
{0xD1,16,{0x00,0x06,0x00,0x8F,0x00,0xB8,0x00,0xD5,0x00,0xEC,0x01,0x12,0x01,0x30,0x01,0x5E}},

{0xD2,16,{0x01,0x83,0x01,0xBD,0x01,0xE9,0x02,0x2F,0x02,0x68,0x02,0x6A,0x02,0x9D,0x02,0xD6}},

{0xD3,16,{0x02,0xFA,0x03,0x25,0x03,0x41,0x03,0x63,0x03,0x76,0x03,0x8B,0x03,0x95,0x03,0x9D}},

{0xD4,4,{0x03,0xF4,0x03,0xFE}},

//G(+) MCR cmd
{0xD5,16,{0x00,0x06,0x00,0x8F,0x00,0xB8,0x00,0xD5,0x00,0xEC,0x01,0x12,0x01,0x30,0x01,0x5E}},

{0xD6,16,{0x01,0x83,0x01,0xBD,0x01,0xE9,0x02,0x2F,0x02,0x68,0x02,0x6A,0x02,0x9D,0x02,0xD6}},

{0xD7,16,{0x02,0xFA,0x03,0x25,0x03,0x41,0x03,0x63,0x03,0x76,0x03,0x8B,0x03,0x95,0x03,0x9D}},

{0xD8,4,{0x03,0xF4,0x03,0xFE}},

//B(+) MCR cmd
{0xD9,16,{0x00,0x06,0x00,0x8F,0x00,0xB8,0x00,0xD5,0x00,0xEC,0x01,0x12,0x01,0x30,0x01,0x5E}},

{0xDD,16,{0x01,0x83,0x01,0xBD,0x01,0xE9,0x02,0x2F,0x02,0x68,0x02,0x6A,0x02,0x9D,0x02,0xD6}},

{0xDE,16,{0x02,0xFA,0x03,0x25,0x03,0x41,0x03,0x63,0x03,0x76,0x03,0x8B,0x03,0x95,0x03,0x9D}},

{0xDF,4,{0x03,0xF4,0x03,0xFE}},

//R(-) MCR cmd
{0xE0,16,{0x00,0x06,0x00,0x8F,0x00,0xB8,0x00,0xD5,0x00,0xEC,0x01,0x12,0x01,0x30,0x01,0x5E}},

{0xE1,16,{0x01,0x83,0x01,0xBD,0x01,0xE9,0x02,0x2F,0x02,0x68,0x02,0x6A,0x02,0x9D,0x02,0xD6}},

{0xE2,16,{0x02,0xFA,0x03,0x25,0x03,0x41,0x03,0x63,0x03,0x76,0x03,0x8B,0x03,0x95,0x03,0x9D}},

{0xE3,4,{0x03,0xF4,0x03,0xFE}},

//G(-) MCR cmd
{0xE4,16,{0x00,0x06,0x00,0x8F,0x00,0xB8,0x00,0xD5,0x00,0xEC,0x01,0x12,0x01,0x30,0x01,0x5E}},

{0xE5,16,{0x01,0x83,0x01,0xBD,0x01,0xE9,0x02,0x2F,0x02,0x68,0x02,0x6A,0x02,0x9D,0x02,0xD6}},

{0xE6,16,{0x02,0xFA,0x03,0x25,0x03,0x41,0x03,0x63,0x03,0x76,0x03,0x8B,0x03,0x95,0x03,0x9D}},

{0xE7,4,{0x03,0xF4,0x03,0xFE}},

//B(-) MCR cmd
{0xE8,16,{0x00,0x06,0x00,0x8F,0x00,0xB8,0x00,0xD5,0x00,0xEC,0x01,0x12,0x01,0x30,0x01,0x5E}},

{0xE9,16,{0x01,0x83,0x01,0xBD,0x01,0xE9,0x02,0x2F,0x02,0x68,0x02,0x6A,0x02,0x9D,0x02,0xD6}},

{0xEA,16,{0x02,0xFA,0x03,0x25,0x03,0x41,0x03,0x63,0x03,0x76,0x03,0x8B,0x03,0x95,0x03,0x9D}},

{0xEB,4,{0x03,0xF4,0x03,0xFE}},


//*************************************
// TE On                               
//*************************************
{0x35,1,{0x00}},

//*************************************
// Sleep Out
//*************************************
{0x11,1,{0x00}},

{REGFLAG_DELAY, 120, {}},
//*************************************
// Display On
//*************************************
{0x29, 1 ,{0x00}},
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
    params->dbi.te_mode 			= LCM_DBI_TE_MODE_DISABLED;//LCM_DBI_TE_MODE_DISABLED;
    params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
    params->dsi.mode   =   SYNC_EVENT_VDO_MODE;  //SYNC_PULSE_VDO_MODE;
    
    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM				= LCM_TWO_LANE;

    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;



    //params->dsi.packet_size=256;
    params->dsi.intermediat_buffer_num = 2;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage
    
    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
    
    params->dsi.word_count=540*3;	//DSI CMD mode need set these two bellow params, different to 6577    480*3
    
    params->dsi.vertical_active_line=960;///854
  
     params->dsi.vertical_sync_active				       = 6;//6;2
    params->dsi.vertical_backporch					=10;//10;// 70;20
    params->dsi.vertical_frontporch					= 10;//10;//70;	20
    params->dsi.vertical_active_line				= FRAME_HEIGHT;
    
    params->dsi.horizontal_sync_active			= 8;//6;2
    params->dsi.horizontal_backporch				= 80;//80;//22;80
    params->dsi.horizontal_frontporch				= 80;//80;//120;80
    params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
   // params->dsi.compatibility_for_nvk =0;		// this parameter would be set to 1 if DriverIC is NTK's and when force match DSI clock for NTK's
   // params->dsi.noncont_clock= 1;
  //  params->dsi.noncont_clock_period=2;

    
  params->dsi.PLL_CLOCK =235;// 190


}


static void lcm_init(void)
{


    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(60);
    SET_RESET_PIN(1);
    MDELAY(120);
push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);

}


static void lcm_suspend(void)
{
    unsigned int data_array[16];
#if defined(BUILD_LK)

#else
      data_array[0] = 0x00002200;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(20);
#endif
    data_array[0] = 0x00280500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(10);

    data_array[0] = 0x00100500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(120);

}


static void lcm_resume(void)
{
	lcm_init();
}
         

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
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0]= 0x00053902;
    data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
    data_array[2]= (y1_LSB);
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0]= 0x002c3909;
    dsi_set_cmdq(data_array, 1, 0);
}

static unsigned int lcm_compare_id(void)
{	unsigned int  data_array[16];
	unsigned char buffer_c5[3];
	unsigned char buffer_04[3];
	
	SET_RESET_PIN(1);
	MDELAY(20);
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(50);

	data_array[0] = 0x00033700;
	dsi_set_cmdq(data_array, 1, 1);
	read_reg_v2(0x04, buffer_04, 3);

	data_array[0] = 0x00033700;
	dsi_set_cmdq(data_array, 1, 1);
	read_reg_v2(0x04, buffer_04, 3);
	
	data_array[0]=0x00063902;
	data_array[1]=0x52AA55F0;
	data_array[2]=0x00000108;
	dsi_set_cmdq(&data_array,3,1);
	
	data_array[0] = 0x00033700;
	dsi_set_cmdq(data_array, 1, 1);
	read_reg_v2(0xC5, buffer_c5, 3);
    #ifdef BUILD_LK
    printf("fangxing: NT35517 %s %d, id0 = 0x%x, id1 = 0x%x\n", __func__,__LINE__, buffer_c5[0],buffer_c5[1]);
    #else

    printk("fangxing: NT35517 %s %d,  id0 = 0x%x, id1 = 0x%x\n", __func__,__LINE__, buffer_c5[0],buffer_c5[1]);
    #endif
	if ((buffer_c5[0]==0x55)&&(buffer_c5[1]==0x17)){
		return 1;
	}else{
		return 0;
	}
   
}
// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER nt35517_qhd_cpt5p0_xyl_x5602_lcm_drv = 
{
    .name			= "nt35517_qhd_cpt5p0_xyl_x5602",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id    = lcm_compare_id,
};

