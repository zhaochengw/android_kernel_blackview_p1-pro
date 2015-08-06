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
#else
#include <linux/string.h>
#endif

#include "lcm_drv.h"
#ifdef BUILD_LK
    #include <platform/mt_gpio.h>
#elif defined(BUILD_UBOOT)
    #include <asm/arch/mt_gpio.h>
#else
    #include <mach/mt_gpio.h>
#endif



#include <cust_adc.h>    	
#define MIN_VOLTAGE (200)   //id is 300mv  
#define MAX_VOLTAGE (400)     
#define COMPARE_BY_ADC  0

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
extern int lcm_id_vol;




#define FRAME_WIDTH                                          (720)
#define FRAME_HEIGHT                                         (1280)

#define REGFLAG_DELAY                                         0XFE
#define REGFLAG_END_OF_TABLE                          0x1FF   // END OF REGISTERS MARKER

#define LCM_ID_OTM1287A   0x1287
#define LCM_DSI_CMD_MODE                                    0

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))



#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)        lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                    lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg                                            lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)                   lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)  

static struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};


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
        #if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
        #else
		params->dsi.mode   = SYNC_PULSE_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 
        #endif
	
		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Video mode setting		
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		
		params->dsi.vertical_sync_active				= 6;// 3    2
		params->dsi.vertical_backporch					= 16;// 20   1
		params->dsi.vertical_frontporch					= 20; // 1  12
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 10;// 50  2
		params->dsi.horizontal_backporch				= 85;//90
		params->dsi.horizontal_frontporch				= 85;//90
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	        //params->dsi.LPX=8; 

		// Bit rate calculation
		//1 Every lane speed
		//params->dsi.pll_select=1;
		//params->dsi.PLL_CLOCK  = LCM_DSI_6589_PLL_CLOCK_377;
		params->dsi.PLL_CLOCK=227;//208
	 	params->dsi.ssc_disable                         = 1;
}

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


static struct LCM_setting_table lcm_initialization_setting[] = {
	  

{0x00,1,{0x00}},

{0xff,3,{0x12,0x87,0x01}},
	
{0x00,1,{0x80}},
	        
{0xff,2,{0x12,0x87}},

{0x00,1,{0xa6}},
             
{0xb3,1,{0x0f}},

{0x00,1,{0x80}},
             
{0xc0,9,{0x00,0x64,0x00,0x10,0x10,0x00,0x64,0x10,0x10}},

{0x00,1,{0x90}},
             
{0xc0,6,{0x00,0x5c,0x00,0x01,0x00,0x04}},

{0x00,1,{0xa2}},
         
{0xC0,3,{0x01,0x00,0x00}},
         
{0x00,1,{0xb3}},
                
{0xc0,2,{0x00,0x55}},
{0x00,1,{0xb5}},
                
{0xc0,1,{0x18}},
         
{0x00,1,{0x81}},
                
{0xc1,1,{0x55}},
         
{0x00,1,{0xa0}},
                
{0xc4,14,{0x05,0x10,0x04,0x02,0x05,0x15,0x11,0x05,0x10,0x07,0x02,0x05,0x15,0x11}},
         
{0x00,1,{0xb0}},
                
{0xc4,2,{0x00,0x00}},

{0x00,1,{0x91}},
                
{0xc5,2,{0x29,0x52}},
         
{0x00,1,{0x00}},
                
{0xd8,2,{0xbe,0xbe}},
                
{0x00,1,{0x00}},
 		            
{0xd9,1,{0x59}},           
         
{0x00,1,{0xb3}},
                
{0xc5,1,{0x84}},
         
{0x00,1,{0xbb}},
                
{0xc5,1,{0x8a}},

{0x00,1,{0x82}},
		     
{0xC4,1,{0x0a}},
         
{0x00,1,{0xc6}},
		     
{0xb0,1,{0x03}},
         
{0x00,1,{0x00}},
               
{0xd0,1,{0x40}},
         
{0x00,1,{0x00}},
                
{0xd1,2,{0x00,0x00}},
         
{0x00,1,{0xb2}},
                
{0xf5,2,{0x00,0x00}},

{0x00,1,{0xb6}},
                
{0xf5,2,{0x00,0x00}},
         
{0x00,1,{0x94}},
                
{0xf5,2,{0x00,0x00}},
         
{0x00,1,{0xd2}},
                
{0xf5,2,{0x06,0x15}},
         
{0x00,1,{0xb4}},
         
{0xc5,1,{0xcc}},
		     
{0x00,1,{0x90}},
                
{0xf5,4,{0x02,0x11,0x02,0x15}},

{0x00,1,{0x90}},
                
{0xc5,1,{0x50}},
         
{0x00,1,{0x94}},
               
{0xc5,1,{0x66}},
         
{0x00,1,{0x80}},
                
{0xcb,11,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
         
{0x00,1,{0x90}},
             
{0xcb,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0x00,0xff,0x00}},
         
{0x00,1,{0xa0}},
                //panel timing state control
{0xcb,15,{0xff,0x00,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0xb0}},
                //panel timing state control
{0xcb,15,{0x00,0x00,0x00,0xff,0x00,0xff,0x00,0xff,0x00,0xff,0x00,0x00,0x00,0x00,0x00}},
         
{0x00,1,{0xc0}},
                //panel timing state control
{0xcb,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x05,0x00,0x05,0x05,0x05,0x05,0x05}},
         
{0x00,1,{0xd0}},
                //panel timing state control
{0xcb,15,{0x05,0x05,0x05,0x05,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05}},
         
{0x00,1,{0xe0}},
                //panel timing state control
{0xcb,14,{0x05,0x00,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x00,0x00}},
         
{0x00,1,{0xf0}},
                //panel timing state control
{0xcb,11,{0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff}},

{0x00,1,{0x80}},
                //panel pad mapping control
{0xcc,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x07,0x00,0x0d,0x09,0x0f,0x0b,0x11}},
         
{0x00,1,{0x90}},
                //panel pad mapping control
{0xcc,15,{0x15,0x13,0x17,0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x06}},
         
{0x00,1,{0xa0}},
                //panel pad mapping control
{0xcc,14,{0x08,0x00,0x0e,0x0a,0x10,0x0c,0x12,0x16,0x14,0x18,0x02,0x04,0x00,0x00}},
         
{0x00,1,{0xb0}},
                //panel pad mapping control
{0xcc,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x02,0x00,0x0c,0x10,0x0a,0x0e,0x14}},
         
{0x00,1,{0xc0}},
                //panel pad mapping control
{0xcc,15,{0x18,0x12,0x16,0x08,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03}},

{0x00,1,{0xd0}},
                //panel pad mapping control
{0xcc,14,{0x01,0x00,0x0b,0x0f,0x09,0x0d,0x13,0x17,0x11,0x15,0x07,0x05,0x00,0x00}},
         
{0x00,1,{0x80}},
                //panel VST setting
{0xce,12,{0x87,0x03,0x28,0x86,0x03,0x28,0x85,0x03,0x28,0x84,0x03,0x28}},
         
{0x00,1,{0x90}},
                //panel VEND setting
{0xce,14,{0x34,0xfc,0x28,0x34,0xfd,0x28,0x34,0xfe,0x28,0x34,0xff,0x28,0x00,0x00}},
         
{0x00,1,{0xa0}},
                //panel CLKA1/2 setting
{0xce,14,{0x38,0x07,0x05,0x00,0x00,0x28,0x00,0x38,0x06,0x05,0x01,0x00,0x28,0x00}},
         
{0x00,1,{0xb0}},
                //panel CLKA3/4 setting
{0xce,14,{0x38,0x05,0x05,0x02,0x00,0x28,0x00,0x38,0x04,0x05,0x03,0x00,0x28,0x00}},

{0x00,1,{0xc0}},
                //panel CLKb1/2 setting
{0xce,14,{0x38,0x03,0x05,0x04,0x00,0x28,0x00,0x38,0x02,0x05,0x05,0x00,0x28,0x00}},
         
{0x00,1,{0xd0}},
                //panel CLKb3/4 setting
{0xce,14,{0x38,0x01,0x05,0x06,0x00,0x28,0x00,0x38,0x00,0x05,0x07,0x00,0x28,0x00}},
         
{0x00,1,{0x80}},
                //panel CLKc1/2 setting
{0xcf,14,{0x38,0x07,0x05,0x00,0x00,0x18,0x25,0x38,0x06,0x05,0x01,0x00,0x18,0x25}},
         
{0x00,1,{0x90}},
                //panel CLKc3/4 setting
{0xcf,14,{0x38,0x05,0x05,0x02,0x00,0x18,0x25,0x38,0x04,0x05,0x03,0x00,0x18,0x25}},
         
{0x00,1,{0xa0}},
                //panel CLKd1/2 setting
{0xcf,14,{0x38,0x03,0x05,0x04,0x00,0x18,0x25,0x38,0x02,0x05,0x05,0x00,0x18,0x25}},

{0x00,1,{0xb0}},
                //panel CLKd3/4 setting
{0xcf,14,{0x38,0x01,0x05,0x06,0x00,0x18,0x25,0x38,0x00,0x05,0x07,0x00,0x18,0x25}},
         
{0x00,1,{0xc0}},
                //panel ECLK setting
{0xcf,11,{0x01,0x01,0x20,0x20,0x00,0x00,0x01,0x81,0x00,0x03,0x08}},
         
{0x00,1,{0x00}},
                                                                                                                                                                  
{0xE1,20,{0x08,0x13,0x1d,0x2b,0x3b,0x4b,0x4e,0x7e,0x75,0x91,0x72,0x5b,0x6a,0x47,0x46,0x3d,0x31,0x24,0x1a,0x12}},
                                                                                         
{0x00,1,{0x00}},
                                                                                                                                                              
{0xE2,20,{0x08,0x13,0x1d,0x2b,0x3b,0x4b,0x4e,0x7e,0x75,0x91,0x72,0x5b,0x6a,0x47,0x46,0x3d,0x31,0x24,0x1a,0x12}},
         
{0x00,1,{0x00}},
               
{0xff,3,{0xff,0xff,0xff}},

 {0x11,  1,  {0x00}},
 {REGFLAG_DELAY, 150, {}},
 {0x29,  1,  {0x00}},
 {REGFLAG_DELAY, 50, {}},

 {REGFLAG_END_OF_TABLE, 0x00, {}}



};

static unsigned int lcm_compare_id(void);

static void lcm_init(void)
{
    unsigned int data_array[16];    
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(1);
    SET_RESET_PIN(1);
    MDELAY(10);
    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
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
    dsi_set_cmdq(&data_array, 3, 1);
    
    data_array[0]= 0x00053902;
    data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
    data_array[2]= (y1_LSB);
    dsi_set_cmdq(&data_array, 3, 1);
    
    data_array[0]= 0x00290508;
    dsi_set_cmdq(&data_array, 1, 1);
    
    data_array[0]= 0x002c3909;
    dsi_set_cmdq(&data_array, 1, 0);


}

static void lcm_suspend(void)
{
    unsigned int data_array[16];
    data_array[0]=0x00280500;
    dsi_set_cmdq(&data_array,1,1);
    MDELAY(10);
    data_array[0]=0x00100500;
    dsi_set_cmdq(&data_array,1,1);
    MDELAY(100);
}

static void lcm_resume(void)
{
  /*  unsigned int data_array[16];
    data_array[0]=0x00110500;
    dsi_set_cmdq(&data_array,1,1);
    MDELAY(100);
    data_array[0]=0x00290500;
    dsi_set_cmdq(&data_array,1,1);
    MDELAY(10);*/
    lcm_init();
}

static unsigned int lcm_compare_id(void)
{
    int array[4];
    char buffer[5];
    char id_high=0;
    char id_low=0;
    int id=0;

	SET_RESET_PIN(0);
    MDELAY(25);
	SET_RESET_PIN(1);
    MDELAY(100);

    array[0] = 0x00053700;// read id return two byte,version and id
    dsi_set_cmdq(array, 1, 1);
    read_reg_v2(0xA1,buffer, 5);

    id_high = buffer[2]; 
    id_low = buffer[3];
    id = (id_high<<8)|id_low; 

    #ifdef BUILD_LK
    printf("luke: OTM1283A %s %d, id = 0x%08x\n", __func__,__LINE__, id);
    #else

    printk("luke: OTM1283A %s %d, id = 0x%08x\n", __func__,__LINE__, id);
    #endif
	
       if (LCM_ID_OTM1287A == id) {
#if COMPARE_BY_ADC
		int data[4] = {0,0,0,0};
		int res = 0;
		int rawdata = 0;
		int lcm_vol = 0;

		res = IMM_GetOneChannelValue(AUXADC_LCM_VOLTAGE_CHANNEL, data, &rawdata);
		if(res < 0) {
		#ifdef BUILD_LK
			printf("[adc_uboot  OTM1283A ruix]: get data error\n");
		#endif
			return 0;
		}

		lcm_vol = data[0] * 1000 + data[1] * 10;
//				printf("lcm_vol = : %d\n",lcm_vol);

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

static unsigned int lcm_esd_check(void)
{
    
}

static unsigned int lcm_esd_recover(void)
{

}


LCM_DRIVER otm1287_hd720_auo5p5_jieke_x5602_lcm_drv = 
{
    .name            = "otm1287_hd720_auo5p5_jieke_x5602",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id     = lcm_compare_id,
   // .esd_check = lcm_esd_check,
    //.esd_recover = lcm_esd_recover,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
    };


