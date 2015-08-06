
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
#include <cust_adc.h>    	
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
#endif

#include <cust_adc.h>    	
#define MIN_VOLTAGE (0)     
#define MAX_VOLTAGE (49)     
#define COMPARE_BY_ADC  0  // lcm compare id by adc

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);

#define FRAME_WIDTH  			(720)
#define FRAME_HEIGHT 			(1280)

#define REGFLAG_DELAY          	0XFE
#define REGFLAG_END_OF_TABLE  	0xFF   
#define LCM_ID_HX8394D 		0x94

#define LCM_DSI_CMD_MODE		0
#define MIPI_VIDEO_MODE

#ifndef TRUE
    #define   TRUE     1
#endif
 
#ifndef FALSE
    #define   FALSE    0
#endif
static unsigned int lcm_esd_test = FALSE;      
static LCM_UTIL_FUNCS lcm_util = {0};
#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))
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


static struct LCM_setting_table lcm_initialization_setting[] = {
   
#if   1  //HX8394F_CMI55_601_HD    				
	
	
	//	B9
	{0xB9, 3, { 0xFF,  0x83,  0x94}}, 
	
	//	BA
	{0xBA, 6, { 0x63, 0x03,  0x68,  0x6B,  0xB2,  0xC0}},
	
	//	B1
	{0xB1, 10, {0x50,  0x12,  0x72,  0x09,  0x33,  0x54,  0x71,  0x31,  0x70,  0x2F}},
	
	//	B2
	{0xB2, 6, { 0x00, 0x80,  0x64,  0x0E,  0x0D,  0x2F}},
	
	//	B4
	{0xB4, 21, {0x64, 0x65,  0x64,  0x65,  0x64,  0x65,  0x01,  0x05,  0x7E,  0x55,  
	0x00,  0x3F,  0x64,  0x65,  0x64,  0x65,  0x64,  0x65,  0x01,  0x05,  0x7E}},
	
	//	B6
	#if 0
	{0xB6, 2, { 0x47, 0x47}}, 
	#else   
	{0xB6, 2, { 0x42,  0x42}},  
	#endif
	//	D3
	{0xD3, 33, {0x00, 0x00, 0x07, 0x07, 0x40, 0x07, 0x10, 0x00, 0x08, 0x10, 0x08, 0x00, 0x08, 0x54, 0x15, 
	0x0E, 0x05, 0x0E, 0x02, 0x15, 0x06, 0x05, 0x06, 0x47, 0x44, 0x0A, 0x0A, 0x4B, 0x10,0x07,0x07, 0x0E,0x40}}, 
	
	
	//	D5
	{0xD5, 44, {0x1A,0x1A,0x1B, 0x1B,0x00,0x01,0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,0x09,0x0A,0x0B, 0x24, 
	0x25, 0x18,0x18, 0x26,0x27,0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,0x18,0x18,0x18,0x18, 
	0x18, 0x18, 0x20, 0x21, 0x18, 0x18, 0x18, 0x18}},   
	
	//	D6, GIP Backword
	{0xD6, 44, {0x1A,0x1A,  0x1B,  0x1B, 0x0B,  0x0A,0x09, 0x08,  0x07,  0x06,0x05,0x04,0x03,0x02, 0x01, 0x00,
	 0x21, 0x20,  0x18,  0x18,  0x27,  0x26, 0x18,0x18,0x18, 0x18,0x18,0x18, 0x18, 0x18,0x18,  0x18,  0x18,   0x18, 
	 0x18,0x18,  0x18, 0x18, 0x25, 0x24, 0x18,  0x18,  0x18,  0x18}},   
	
	//	E0												  
	{0xE0, 58, {0x00,0x03, 0x0E, 0x14,0x17, 0x1B, 0x1F,0x1F,0x40, 0x53, 0x64,0x64, 0x6F,0x83,0x88, 0x8A, 0x96, 
	0x97, 0x92, 0x9E, 0xAD, 0x55,  0x52,  0x55,  0x59, 0x5B, 0x60,0x64, 0x7F,  0x00,  0x03, 0x0D,0x13,0x16, 
	0x1B,0x1F,0x1F,  0x40,  0x53,  0x64,  0x64, 0x6F,0x83,  0x88,  0x8A, 0x96,0x97,0x92, 0x9E,  0xAD,  0x55, 0x52,
	0x55,0x59,0x5B, 0x60,0x64,0x7F}},		 
	
	//	C0
	{0xC0, 2, { 0x1F, 0x73}}, 
	
	//	CC
	{0xCC, 1, { 0x0b}},    
	
	//	D4
	{0xD4, 1, { 0x02}},  

	// Sleep Out 
	{0x11,  0,  {0x00}},
	{REGFLAG_DELAY, 120, {0}}, 

	// Set Power Option   HX5186 Mode 
	//{0xBF, 7, {0x40,0x81,0x50,0x02,0x1A,0xFC,0x02}},

	// Display ON 
	{0x29,  0,  {0x00}},
	{REGFLAG_DELAY, 20, {0}},

#endif



};
/*
static struct LCM_setting_table lcm_sleep_out_setting[] = {
	{0x11, 1, {0x00}},
        {REGFLAG_DELAY, 120, {}},

	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 50, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},

	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


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

unsigned int vcomadj=0x46;
static void lcm_regdata(void)
{
unsigned int data_array[16];


data_array[0] = 0x00043902;
data_array[1] = 0x9483FFB9;
dsi_set_cmdq(&data_array, 2, 1);
MDELAY(1);

data_array[0] = 0x00073902;
data_array[1] = 0x680363BA;
data_array[2] = 0x00C0B26B;
dsi_set_cmdq(&data_array, 3, 1);
MDELAY(1);

data_array[0] = 0x000B3902;
data_array[1] = 0x721250B1;
data_array[2] = 0x71543309;
data_array[3] = 0x002F7051;
dsi_set_cmdq(&data_array, 4, 1);
MDELAY(1);

data_array[0] = 0x00073902;
data_array[1] = 0x648000B2;
data_array[2] = 0x002F0D0E;
dsi_set_cmdq(&data_array, 3, 1);
MDELAY(1);

data_array[0] = 0x00163902;
data_array[1] = 0x6C6D6CB4;
data_array[2] = 0x016D6C6D;
data_array[3] = 0x0075FF01;
data_array[4] = 0x6E6F6E3F;
data_array[5] = 0x016F6E6F;
data_array[6] = 0x0000FF01;
dsi_set_cmdq(&data_array, 7, 1);
MDELAY(1);

data_array[0] = 0x00033902;
data_array[1] = 0x005353B6;  //vcom=0x53
dsi_set_cmdq(&data_array, 2, 1);
MDELAY(1);

/*
data_array[0] = 0x00033902;
data_array[1] = 0x000000B6 |((vcomadj<<16)&0xff0000)|((vcomadj<<8)&0xff00);
dsi_set_cmdq(&data_array, 2, 1);
MDELAY(1);
vcomadj+=2;
*/

data_array[0] = 0x00223902;
data_array[1] = 0x070000D3;
data_array[2] = 0x10074007;
data_array[3] = 0x08100800;
data_array[4] = 0x15540800;
data_array[5] = 0x020E050E;
data_array[6] = 0x06050615;
data_array[7] = 0x0A0A4447;
data_array[8] = 0x0707104B;
data_array[9] = 0x0000400E;
dsi_set_cmdq(&data_array, 10, 1);
MDELAY(1);

data_array[0] = 0x002D3902;
data_array[1] = 0x1B1A1AD5;
data_array[2] = 0x0201001B;
data_array[3] = 0x06050403;
data_array[4] = 0x0A090807;
data_array[5] = 0x1825240B;
data_array[6] = 0x18272618;
data_array[7] = 0x18181818;
data_array[8] = 0x18181818;
data_array[9] = 0x18181818;
data_array[10] = 0x20181818;
data_array[11] = 0x18181821;
data_array[12] = 0x00000018;
dsi_set_cmdq(&data_array, 13, 1);
MDELAY(1);

data_array[0] = 0x002D3902;
data_array[1] = 0x1B1A1AD6;
data_array[2] = 0x090A0B1B;
data_array[3] = 0x05060708;
data_array[4] = 0x01020304;
data_array[5] = 0x18202100;
data_array[6] = 0x18262718;
data_array[7] = 0x18181818;
data_array[8] = 0x18181818;
data_array[9] = 0x18181818;
data_array[10] = 0x25181818;
data_array[11] = 0x18181824;
data_array[12] = 0x00000018;
dsi_set_cmdq(&data_array, 13, 1);
MDELAY(1);

data_array[0] = 0x003B3902;
data_array[1] = 0x1F0F00E0;
data_array[2] = 0x3733302D;
data_array[3] = 0x8F7F6F36;
data_array[4] = 0xABA69890;
data_array[5] = 0xAAB1B4AA;
data_array[6] = 0x5557BCB3;
data_array[7] = 0x58545058;
data_array[8] = 0x0F00605B;
data_array[9] = 0x33302D1F;
data_array[10] = 0x7F6F3637;
data_array[11] = 0xA698908F;
data_array[12] = 0xB1B4AAAB;
data_array[13] = 0x57BCB3AA;
data_array[14] = 0x54505855;
data_array[15] = 0x00605B58;
dsi_set_cmdq(&data_array, 16, 1);
MDELAY(1);

data_array[0] = 0x00033902;
data_array[1] = 0x00731FC0;
dsi_set_cmdq(&data_array, 2, 1);
MDELAY(1);

data_array[0] = 0x00023902;
data_array[1] = 0x00000bCC;
dsi_set_cmdq(&data_array, 2, 1);
MDELAY(1);

data_array[0] = 0x00023902;
data_array[1] = 0x000002D4;
dsi_set_cmdq(&data_array, 2, 1);
MDELAY(1);

//add
data_array[0] = 0x00023902;
data_array[1] = 0x000001bd;
dsi_set_cmdq(&data_array, 2, 1);

data_array[0] = 0x00023902;
data_array[1] = 0x000060b1;
dsi_set_cmdq(&data_array, 2, 1);

data_array[0] = 0x00023902;
data_array[1] = 0x000000bd;
dsi_set_cmdq(&data_array, 2, 1);
MDELAY(1);


data_array[0] = 0x00110500;
dsi_set_cmdq(&data_array, 1, 1);
MDELAY(150);

data_array[0] = 0x00083902;
data_array[1] = 0x508140BF;
data_array[2] = 0x02FC1A02;
dsi_set_cmdq(&data_array, 3, 1);
MDELAY(1);

data_array[0] = 0x00290500;
dsi_set_cmdq(&data_array, 1, 1);
MDELAY(20);

}


static void lcm_init(void)
{
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(50);
    SET_RESET_PIN(1);
    MDELAY(120);
   
   // push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
lcm_regdata();
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
    params->dbi.te_mode                 = LCM_DBI_TE_MODE_DISABLED;
		params->dbi.te_edge_polarity		        = LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
#else
		params->dsi.mode   = SYNC_PULSE_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 
		//params->dsi.mode   = SYNC_EVENT_VDO_MODE;
#endif
	
		// DSI
		/* Command mode setting */
    params->dsi.LANE_NUM                = LCM_FOUR_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
	//	params->dsi.packet_size=256;

		// Video mode setting		
	//	params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

			  params->dsi.intermediat_buffer_num  = 2;
			 params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
			        params->dsi.word_count=720 * 3;
			  params->dsi.vertical_sync_active = 4;
			params->dsi.vertical_backporch = 12;
			params->dsi.vertical_frontporch = 15;
			params->dsi.vertical_active_line = FRAME_HEIGHT;
			
			params->dsi.horizontal_sync_active = 20;
			params->dsi.horizontal_backporch = 30;
			params->dsi.horizontal_frontporch = 30;
			 //   params->dsi.horizontal_blanking_pixel = 60;
			params->dsi.horizontal_active_pixel = FRAME_WIDTH;
			    params->dsi.PLL_CLOCK =192;// 190
}

static void lcm_suspend(void)
{
	

	unsigned int data_array[2];
/*
//	data_array[0] = 0x00220500; // D
	//dsi_set_cmdq(&data_array, 1, 1);
//	MDELAY(200);
	data_array[0] = 0x00280500; // Display Off 
	dsi_set_cmdq(&data_array, 1, 1);           
	MDELAY(160); //35                          
	data_array[0] = 0x00100500; // Sleep In    
	dsi_set_cmdq(&data_array, 1, 1);  
	MDELAY(20);  
//	SET_RESET_PIN(0);    
//	MDELAY(10); 
		*/
	
	SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(20);
    //SET_RESET_PIN(1);
    //MDELAY(120);         
	      		
}

static void lcm_resume(void)
{
#ifndef BUILD_LK
	lcm_init();
#endif
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

  data_array[0]= 0x00052902;
  data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
  data_array[2]= (x1_LSB);
  dsi_set_cmdq(&data_array, 3, 1);

  data_array[0]= 0x00052902;
  data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
  data_array[2]= (y1_LSB);
  dsi_set_cmdq(&data_array, 3, 1);

  data_array[0]= 0x00290508; //HW bug, so need send one HS packet
  dsi_set_cmdq(&data_array, 1, 1);

  data_array[0]= 0x002c3909;
  dsi_set_cmdq(&data_array, 1, 0);

}

/*static void lcm_setbacklight(unsigned int level)
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

static unsigned int lcm_esd_check(void)
{
	#ifdef BUILD_LK
		printf("lcm_esd_check()\n");
	#else
		printk("lcm_esd_check()\n");
	#endif 
 #ifndef BUILD_LK
	char  buffer[3];
	int   array[4];

	if(lcm_esd_test)
	{
		lcm_esd_test = FALSE;
		return TRUE;
	}

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x0a, buffer, 1);
	if(buffer[0]==0x9c)
	{
		return FALSE;
	}
	else
	{	
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
*/
static unsigned int lcm_compare_id(void)                                                         
{                                                                                                
	unsigned int id=0;
    unsigned char buffer[2];
    unsigned int array[16];


    SET_RESET_PIN(1);
    MDELAY(20);
    SET_RESET_PIN(0);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(120);

    array[0]=0x00043902;
    array[1]=0x9483FFB9;// page enable
    dsi_set_cmdq(&array, 2, 1);
    MDELAY(10);

    array[0]=0x00023902;
    array[1]=0x000063ba;
    dsi_set_cmdq(&array, 2, 1);
    MDELAY(10);

    array[0] = 0x00023700;// return byte number
    dsi_set_cmdq(&array, 1, 1);
    MDELAY(10);

    read_reg_v2(0xF4, buffer, 2);
    id = buffer[0];

#ifdef BUILD_LK
    printf("=====>compare id for test %s, id = 0x%08x\n", __func__, id);
#else
    printk("=====>compare id for test %s, id = 0x%08x\n", __func__, id);
#endif


  if (LCM_ID_HX8394D == id) {
#if COMPARE_BY_ADC
		int data[4] = {0,0,0,0};
		int res = 0;
		int rawdata = 0;
		int lcm_vol = 0;

		res = IMM_GetOneChannelValue(AUXADC_LCM_VOLTAGE_CHANNEL, data, &rawdata);
		if(res < 0) {
		#ifdef BUILD_LK
			printf("[adc_uboot  OTM1283A k550]: get data error\n");
		#endif
			return 0;
		}

		lcm_vol = data[0] * 1000 + data[1] * 10;
    #ifdef BUILD_LK
		printf("lcm_vol = : %d\n",lcm_vol);
    #endif
		if(lcm_vol >= MIN_VOLTAGE && lcm_vol <= MAX_VOLTAGE) {
			return 1;
		}else{
			return 0;
		}
#endif
		return 1;
       	}else{
		return 0; 
       }                                                                        
}                                                                                                


LCM_DRIVER hx8394f_hd_cmi5p5_blj_trx_x5602_lcm_drv = 
{
    .name			= "hx8394f_hd_cmi5p5_blj_trx_x5602",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
 // .esd_check     = lcm_esd_check,
 	//.esd_recover   = lcm_esd_recover,
	.compare_id    = lcm_compare_id,	
#if (LCM_DSI_CMD_MODE)
	//.set_backlight	= lcm_setbacklight,
    .update         = lcm_update,
#endif
};

