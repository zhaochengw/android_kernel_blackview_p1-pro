
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
//add by zhaoshoufeng for chuanma   	
#define MIN_VOLTAGE (400)     
#define MAX_VOLTAGE (600)   
//add by zhaoshoufeng for chuanma     
#define COMPARE_BY_ADC  0  // lcm compare id by adc

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);

#define FRAME_WIDTH  			(540)
#define FRAME_HEIGHT 			(960)

#define REGFLAG_DELAY          	0XFE
#define REGFLAG_END_OF_TABLE  	0xFFF   
#define LCM_ID_OTM9605A			0x9605

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
   
{0x00,1,{0x00}},
{0xFF,3,{0x96,0x05,0x01}},

{0x00,1,{0x80}},
{0xFF,2,{0x96,0x05}},

{0x00,1,{0x89}},
{0xC0,1,{0x01}},

{0x00,1,{0x91}},
{0xC5,2,{0xE6,0x02}},

{0x00,1,{0xB4}},
{0xC0,1,{0x50}},

{0x00,1,{0xA0}},
{0xC1,1,{0x02}},

{0x00,1,{0x80}},
{0xC4,1,{0x30}},

{0x00,1,{0x87}},
{0xC4,1,{0x40}},

{0x00,1,{0x93}},
{0xC5,1,{0x00}},

{0x00,1,{0x83}},
{0xC5,1,{0x00}},

{0x00,1,{0x80}},
{0xCE,6,{0x8B,0x03,0x00,0x8A,0x03,0x00}},

{0x00,1,{0x90}},
{0xCE,14,{0x23,0xC2,0x00,0x23,0xC3,0x00,0x33,0xC5,0x00,0x33,0xC6,0x00,0x00,0x00}},

{0x00,1,{0xA0}},
{0xCE,14,{0x38,0x09,0x03,0xC2,0x00,0x00,0x00,0x38,0x08,0x03,0xC3,0x00,0x00,0x00}},

{0x00,1,{0xB0}},
{0xCE,14,{0x38,0x07,0x03,0xC4,0x00,0x00,0x00,0x38,0x06,0x03,0xC5,0x00,0x00,0x00}},

{0x00,1,{0xC0}},
{0xCE,14,{0x38,0x05,0x03,0xBE,0x00,0x00,0x00,0x38,0x04,0x03,0xBF,0x00,0x00,0x00}},

{0x00,1,{0xD0}},
{0xCE,14,{0x38,0x03,0x03,0xC0,0x00,0x00,0x00,0x38,0x02,0x03,0xC1,0x00,0x00,0x00}},

{0x00,1,{0xC0}},
{0xCF,10,{0x47,0x47,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00}},

{0x00,1,{0xC0}},
{0xcb,15,{0x00,0x00,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x00,0x00,0x04,0x00,0x00}},

{0x00,1,{0xD0}},
{0xcb,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04}},

{0x00,1,{0xE0}},
{0xCB,10,{0x00,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0x80}},
{0xcc,10,{0x00,0x00,0x02,0x0A,0x0C,0x0E,0x10,0x21,0x22,0x06}},

{0x00,1,{0x90}},
{0xcc,15,{0x00,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x09,0x0B}},

{0x00,1,{0xA0}},
{0xcc,15,{0x0D,0x0F,0x21,0x22,0x05,0x00,0x00,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0xB0}},
{0xcc,10,{0x00,0x00,0x05,0x0B,0x09,0x0F,0x0D,0x21,0x22,0x01}},

{0x00,1,{0xC0}},
{0xcc,15,{0x00,0x00,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x0C,0x0A}},

{0x00,1,{0xD0}},
{0xcc,15,{0x10,0x0E,0x21,0x22,0x02,0x00,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},



{0x00,1,{0x00}},
{0xD8,2,{0x57,0x57}},

{0x00,1,{0x00}},
{0xD9,1,{0x48}},//47->3a->36->4a

{0x00,1,{0x00}},
{0xE1,16,{0x01,0x07,0x0c,0x0d,0x05,0x0d,0x0a,0x09,0x05,0x08,0x0f,0x08,0x0f,0x11,0x0a,0x03}},

{0x00,1,{0x00}},
{0xE2,16,{0x01,0x07,0x0b,0x0d,0x06,0x0d,0x0a,0x09,0x05,0x08,0x10,0x07,0x0F,0x11,0x0a,0x03}},
////////////////////////////////////// 
{0x00,1,{0xB1}},
{0xC5,1,{0x28}},

{0x00,1,{0x80}},
{0xC4,1,{0x9C}},

{0x00,1,{0xC0}},
{0xC5,1,{0x00}},



{0x00,1,{0xB2}},
{0xF5,4,{0x15,0x00,0x15,0x00}},

{0x00,1,{0x93}},
{0xC5,3,{0x03,0x66,0x66}},

{0x00,1,{0xa2}},
{0xC0,3,{0x03,0x00,0x05}},

{0x00,1,{0x80}},
{0xC1,2,{0x36,0x66}},

{0x00,1,{0x89}},
{0xC0,1,{0x01}},

//{0x00,1,{0xA0}},
//{0xC1,1,{0x00}},

{0x00,1,{0xC5}},
{0xB0,1,{0x03}},
/////////////////////////////////////// 

{0x00,1,{0x00}},
{0xFF,3,{0xFF,0xFF,0xFF}},




	{0x11,1,{0x00}},//SLEEP OUT
	{REGFLAG_DELAY,120,{}},

	{0x29,1,{0x00}},//Display ON
	{REGFLAG_DELAY,20,{}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},

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
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;//LCM_DBI_TE_MODE_DISABLED; //
		params->dbi.te_edge_polarity		        = LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
#else
		params->dsi.mode   = SYNC_PULSE_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 
		//params->dsi.mode   = SYNC_EVENT_VDO_MODE;
#endif
	
		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_TWO_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
	//	params->dsi.packet_size=256;

		// Video mode setting		
		params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		params->dsi.word_count=540*3;	

		params->dsi.vertical_sync_active				= 4;
	params->dsi.vertical_backporch					= 16;   //16
	params->dsi.vertical_frontporch					= 20;    //15
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 
	params->dsi.horizontal_sync_active				= 10;
	params->dsi.horizontal_backporch				= 64;  //64
	params->dsi.horizontal_frontporch				= 64;
	params->dsi.horizontal_blanking_pixel		       = 60;
	params->dsi.horizontal_active_pixel		       = FRAME_WIDTH;
		     params->dsi.PLL_CLOCK =245;// 190
			 
	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd          = 0x0a;
	params->dsi.lcm_esd_check_table[0].count        = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;
}


static void lcm_init(void)
{
	SET_RESET_PIN(1);
  MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(20);//50
	SET_RESET_PIN(1);
	MDELAY(50);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
	unsigned int data_array[2];
//	data_array[0] = 0x00220500; // D
//	dsi_set_cmdq(&data_array, 1, 1);
//	MDELAY(200);
	data_array[0] = 0x00280500; // Display Off 
	dsi_set_cmdq(&data_array, 1, 1);           
	MDELAY(120); //35                          
	data_array[0] = 0x00100500; // Sleep In    
	dsi_set_cmdq(&data_array, 1, 1);  
	MDELAY(20);  
	//SET_RESET_PIN(0);    
	//MDELAY(10);          
	       		
}

static void lcm_resume(void)
{
	lcm_init();
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

static unsigned int lcm_compare_id(void)                                                         
{                                                                                                
	int array[4];                                                                                
	char buffer[5];                                                                              
	char id_high=0;                                                                              
	char id_low=0;                                                                               
	int id=0;                                                                                    
	                                                                                             
	SET_RESET_PIN(1);                                                                            
	SET_RESET_PIN(0);                                                                            
	MDELAY(10);                                                                                  
	SET_RESET_PIN(1);                                                                            
	MDELAY(200);                                                                                 
	                                                                                             
	array[0] = 0x00053700;                                                                       
	dsi_set_cmdq(array, 1, 1);                                                                   
	read_reg_v2(0xa1, buffer, 5);                                                                
	                                                                                             
	id_high = buffer[2];                                                                         
	id_low = buffer[3];                                                                          
	id = (id_high<<8) | id_low;                                                                  
                                                                                                 
     #if defined(BUILD_LK)                                                                       
        printf("luke:zhershi lcm:%d, buffer[0]: 0x%0x  buffer[1]: 0x%0x  buffer[2]: 0x%0x buffer[3]: 0x%0x  buffer[4]: 0x%0x \n",
       							__LINE__,buffer[0],buffer[1],buffer[2],buffer[3],buffer[4]);                                                               
     #endif                                                                                      
#if  COMPARE_BY_ADC                                                                              
                                                                                                
    if (LCM_ID_OTM9605A == id) {                                                                 
				int data[4] = {0,0,0,0};                                                                 
				int res = 0;                                                                             
				int rawdata = 0;                                                                         
				int lcm_vol = 0;                                                                         
    		                                                                                         
				res = IMM_GetOneChannelValue(AUXADC_LCM_VOLTAGE_CHANNEL, data, &rawdata);                
				if(res < 0) {                                                                            
					#ifdef BUILD_LK                                                                          
						printf("luke:=zhershi===line:%d====\n",__LINE__);                                            
					#endif                                                                                   
					return 0;                                                                              
				}                                                                                        
    		                                                                                         
				lcm_vol = data[0] * 1000 + data[1] * 10;                                                 
    		                                                                                         
				#ifdef BUILD_LK                                                                          
				printf("luke:==zhershi==line:%d===== lcm_vol= %d\n",__LINE__,lcm_vol);                          
				#endif                                                             
                                                               
				if(lcm_vol >= MIN_VOLTAGE && lcm_vol <= MAX_VOLTAGE) {                                    
						return 1;        //luke                                                                     
				}else{                                                                                   
						return 0;	                                                                             
				}                                                                                         
    }else{                                                                                       
          return 0;                                                                                 
    }                                                                                            
#else 
     #if defined(BUILD_LK)                                                                       
        printf("luke: lcm:%d, buffer[0]: 0x%0x  buffer[1]: 0x%0x \n",  __LINE__,buffer[0],buffer[1] );
	printf("luke: lcm:%d, id = 0x%08x\n", __LINE__, id);                                                                       
     #endif                                                                                                                                        
    if(LCM_ID_OTM9605A == id) {                                                                  
    	    return 1;                                                                              
    	}else{                                                                                     
    	    return 0;                                                                              
    	}                                                                                          
#endif                                                                                               
}                                                                                                


LCM_DRIVER otm9605_qhd_cmi5p0_chuanma_otd_x5602_lcm_drv = 
{
    .name			= "otm9605_qhd_cmi5p0_chuanma_otd_x5602",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
    .esd_check     = lcm_esd_check,
 	.esd_recover   = lcm_esd_recover,
	.compare_id    = lcm_compare_id,	
#if (LCM_DSI_CMD_MODE)
	//.set_backlight	= lcm_setbacklight,
    .update         = lcm_update,
#endif
};

