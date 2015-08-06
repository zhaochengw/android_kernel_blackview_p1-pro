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

#define FRAME_WIDTH                                          (720)
#define FRAME_HEIGHT                                         (1280)

#define REGFLAG_DELAY                                         0XFE
#define REGFLAG_END_OF_TABLE                          0x1FF   // END OF REGISTERS MARKER

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
		
		params->dsi.vertical_sync_active				= 4;// 3    2
		params->dsi.vertical_backporch					= 18;// 20   1
		params->dsi.vertical_frontporch					= 18; // 1  12
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active			= 4;// 50  2
		params->dsi.horizontal_backporch				= 62;//90
		params->dsi.horizontal_frontporch				= 62;//90
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	        //params->dsi.LPX=8; 

		// Bit rate calculation
		//1 Every lane speed
		//params->dsi.pll_select=1;
		//params->dsi.PLL_CLOCK  = LCM_DSI_6589_PLL_CLOCK_377;
		params->dsi.PLL_CLOCK=210;//208
//		params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
//		params->dsi.pll_div2=0;		// div2=0,1,2,3;div1_real=1,2,4,4	

//		params->dsi.fbk_div =7;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	
		//params->dsi.compatibility_for_nvk = 1;		// this parameter would be set to 1 if DriverIC is NTK's and when force match DSI clock for NTK's
}
//static int vcom = 0x20;
static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for(i = 0; i < count; i++) {
        
        unsigned cmd;
        cmd = table[i].cmd;
        
        switch (cmd) {
         /*	   case 0xb6:
            	   table[i].para_list[0] = vcom;
            	   table[i].para_list[1] = vcom;
            	   dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
            	   vcom += 2;
            	   break;
          */
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

{0xB9,3,{0xF1,0x12,0x81}},	
					
{0xBA,27,{0x33,0x81,0x06,0xf9,0x0e,0x0e,0x00,0x00,0x00,0x00,              
          0x00,0x00,0x00,0x00,0x44,0x25,0x00,0x91,0x0a,0x00,	
          0x00,0x02,0x4f,0x11,0x00,0x00,0x37}},
/*{0xBA,27,{0x33,0x81,0x05,0xf9,0x0e,0x0e,0x02,0x00,0x00,0x00,              
          0x00,0x00,0x00,0x00,0x44,0x25,0x00,0x91,0x0a,0x00,	
          0x00,0x02,0x4f,0x11,0x00,0x00,0x37}},  */   //new     
		
{0xB8,1,{0xA4}}, //ECP A4-> A6

{0xB3,17,{0x02,0x00,0x06,0x06,0x10,0x10,0x05,0x05,0x00,0x00,
					0x00,0x03,0xFF,0x00,0x00,0x00,0x00}},
					
{0xC0,9,{0x73,0x73,0x50,0x50,0x00,0x00,0x08,0x72,0x00}},  // 72->30

{0xBf,2,{0x02,0x11}},///Set power	

{0xBC,1,{0x46}},  ///Set VDC

{0xCC,1,{0x0B}},  ///Set Panel

{0xB4,1,{0x80}}, ///Set Panel inversion

{0xB2,2,{0xC8,0x02}},///Set RSO
	
{0xE3,10,{0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0xc0,0x14}},//µ¹2 c0->00


{0xB1,10,{0x20,0x50,0xe3,0x1E,0x1E,0x33,0x77,0x01,0x9B,0x0C}},//
//{0xB1,10,{0x22,0x54,0xe3,0x1f,0x1f,0x11,0x77,0x01,0x9B,0x0C}},//new 

{0xB5,2,{0x09,0x09}}, //Set POWER  09->0c

{0xB6,2,{0x18,0x18}},  // Set VCOM   2c->18

/*{0xE9,63,{0x04,0x00,0x0F,0x05,0x13,0x0A,0xA0,0x12,0x31,0x23,
					0x37,0x11,0x0A,0xA0,0x37,0x38,0x03,0x00,0x03,0x00,
					0x00,0x00,0x03,0x00,0x03,0x00,0x00,0x00,0x75,0x75,
					0x31,0x88,0x88,0x88,0x88,0x88,0x13,0x88,0x88,0x64,
					0x64,0x20,0x88,0x88,0x88,0x88,0x88,0x02,0x88,0x88,
					0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
					0x00,0x00,0x00}},*/
{0xE9,63,{0x02,0x00,0x0F,0x05,0x13,0x0A,0xb0,0x12,0x31,0x23,
					0x37,0x12,0x40,0xb0,0x27,0x38,0x03,0x00,0x03,0x00,
					0x00,0x00,0x03,0x00,0x03,0x00,0x00,0x00,0x75,0x75,
					0x31,0x88,0x88,0x88,0x88,0x88,0x13,0x88,0x88,0x64,
					0x64,0x20,0x88,0x88,0x88,0x88,0x88,0x02,0x88,0x88,
					0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
					0x00,0x00,0x00}},//new
					
                      
{0xEA,48,{0x02,0x21,0x00,0x00,0x02,0x46,0x02,0x88,0x88,0x88,
					0x88,0x88,0x64,0x88,0x88,0x13,0x57,0x13,0x88,0x88,
					0x57,0x02,0x31,0x75,0x88,0x88,0x00,0x0f,0x00,0x00,
					0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
					0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}}, //new 0f->14

/*{0xE0,34,{0x00,0x06,0x09,0x2A,0x36,0x3F,0x3b,0x3a,0x06,0x0D,0x0F,0x12,0x14,0x11,0x12,0x11,0x17,
	  0x00,0x06,0x09,0x2A,0x36,0x3F,0x3b,0x3a,0x06,0x0D,0x0F,0x12,0x14,0x11,0x12,0x11,0x17}},      */
	  
{0xE0,34,{0x00,0x09,0x10,0x2e,0x33,0x3F,0x47,0x40,0x08,0x0e,0x0d,0x12,0x13,0x10,0x0f,0x0c,0x11, 
	  		  0x00,0x09,0x10,0x2e,0x33,0x3F,0x47,0x40,0x08,0x0e,0x0d,0x12,0x13,0x10,0x0f,0x0c,0x11}},   //new  					
{0x11,0,{}},
{REGFLAG_DELAY, 120, {}},
{0x29,0,{}},
{REGFLAG_DELAY, 20, {}},
{REGFLAG_END_OF_TABLE, 0x00, {}}

};

static unsigned int lcm_compare_id(void);

static void lcm_init(void)
{
    unsigned int data_array[16];    
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);
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
unsigned int data_array[2];
//	data_array[0] = 0x00220500; // D
	//dsi_set_cmdq(&data_array, 1, 1);
//	MDELAY(200);
	data_array[0] = 0x00280500; // Display Off 
	dsi_set_cmdq(&data_array, 1, 1);           
	MDELAY(120); //35                          
	data_array[0] = 0x00100500; // Sleep In    
	dsi_set_cmdq(&data_array, 1, 1);  
	MDELAY(20);  
	SET_RESET_PIN(0);    
	MDELAY(10); 
		/*
	
	SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(120);         
	   */    		
}

static void lcm_resume(void)
{
    	lcm_init();
    /*	 unsigned int data_array[16];
    data_array[0]=0x00110500;
    dsi_set_cmdq(&data_array,1,1);
    MDELAY(150);
    data_array[0]=0x00290500;
    dsi_set_cmdq(&data_array,1,1);
    MDELAY(10);*/
}

static unsigned int lcm_compare_id(void)
{
    unsigned int id = 0;
    unsigned char buffer[3];
    unsigned int data_array[16];


	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(50);
	SET_RESET_PIN(1);
	MDELAY(120);
  
	// Set Maximum return byte = 1
    data_array[0] = 0x00033700;// read id return two byte,version and id
    dsi_set_cmdq(data_array, 1, 1);
    //MDELAY(10);


    read_reg_v2(0x04, buffer, 3);
    id = buffer[0]; //we only need ID
    id<<=8;
    id|= buffer[1]; //we test buffer 1
    id<<=8;
    id|= buffer[2]; //we test buffer 1
#if defined(BUILD_LK)
    printf("[fl10801->lcm_esd_check] %s buffer[0] = %x; buffer[1]= %x; buffer[2]= %x;\n", __func__,buffer[0],buffer[1],buffer[2]);
#elif defined(BUILD_UBOOT)
    printf("[fl10801->lcm_esd_check] %s buffer[0] = %x; buffer[1]= %x; buffer[2]= %x;\n", __func__,buffer[0],buffer[1],buffer[2]);
#else
    printk("[fl10801->lcm_esd_check] %s buffer[0] = %x; buffer[1]= %x; buffer[2]= %x;\n", __func__,buffer[0],buffer[1],buffer[2]);
#endif

#ifdef BUILD_LK
    printf("[lcm_compare_id] fl10801_ic:lcd id 0x%x\n",id);
#else
    printk("[lcm_compare_id] fl10801_ic:lcd id 0x%x\n",id);
#endif

    return (0x18211f == id)? 1:0;
}

static unsigned int lcm_esd_check(void)
{
    
}

static unsigned int lcm_esd_recover(void)
{

}


LCM_DRIVER fl11281_hd720_ivo5p5_huayu_trx_x5602_lcm_drv = 
{
    .name            = "fl11281_hd720_ivo5p5_huayu_trx_x5602",
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


