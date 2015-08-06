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

#define LCM_ID_OTM1283A   0x1284
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
		params->dsi.vertical_backporch					= 10;// 20   1
		params->dsi.vertical_frontporch					= 10; // 1  12
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 10;// 50  2
		params->dsi.horizontal_backporch				= 90;//90
		params->dsi.horizontal_frontporch				= 90;//90
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	        //params->dsi.LPX=8; 

		// Bit rate calculation
		//1 Every lane speed
		//params->dsi.pll_select=1;
		//params->dsi.PLL_CLOCK  = LCM_DSI_6589_PLL_CLOCK_377;
	params->dsi.PLL_CLOCK=208;//208
	
		//params->dsi.compatibility_for_nvk = 1;		// this parameter would be set to 1 if DriverIC is NTK's and when force match DSI clock for NTK's
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
{0x00,1,{0x00}},//EXTC=1
{0xFF,4,{0x12,0x84,0x01}},

{0x00,1,{0x80}},//OriseModeEnable
{0xFF,3,{0x12,0x84}},

{0x00,1,{0x91}},//OriseModeEnable
{0xB0,1,{0x9A}},

{0x00,1,{0x92}},//swapºóÑ¡ÔñMIPI4LANE
{0xFF,2,{0x30,0x02}},

	//***************PanelSetting*****************

{0x00,1,{0x80}},//TCONSetting
{0xC0,9,{0x00,0x64,0x00,0x0F,0x11,0x00,0x64,0x0F,0x11}},
			
{0x00,1,{0x90}},//PanelTimingSetting
{0xC0,6,{0x00,0x55,0x00,0x01,0x00,0x04}},//GATEShift5CGOE

{0x00,1,{0xB3}},//Interval
{0xC0,2,{0x00,0x55}},//00-1dot10-2dot50-column

{0x00,1,{0x81}},//FrameRate60Hz
{0xC1,1,{0x66}},

	//******************BOEPowerIC******************
{0x00,1,{0xB2}},//VGLO1
{0xF5,2,{0x00,0x00}},

{0x00,1,{0xB4}},//VGLO1_S
{0xF5,2,{0x00,0x00}},

{0x00,1,{0xB6}},//VGLO2
{0xF5,2,{0x00,0x00}},

{0x00,1,{0xB8}},//VGLO2_S
{0xF5,2,{0x00,0x00}},

{0x00,1,{0x94}},//VCLpumpdis
{0xF5,2,{0x00,0x00}},

{0x00,1,{0xD2}},//VCLreg.en
{0xF5,2,{0x06,0x15}},

{0x00,1,{0xB4}},//VGLO1/2Pulllowsetting
{0xC5,1,{0xCC}},//d[7]vglo1d[6]vglo2=>0:pullvss,0x1:pullvgl

	//***************PowerSetting******************

{0x00,1,{0x90}},//Mode-3
{0xF5,4,{0x02,0x11,0x02,0x15}},

{0x00,1,{0x90}},//2xVPNL,0x1.5*=00,0x2*=50,0x3*=a0
{0xC5,1,{0x50}},

{0x00,1,{0x94}},
{0xC5,1,{0x66}},

{0x00,1,{0xA0}},//DCDCSetting
{0xC4,14,{0x05,0x10,0x06,0x02,0x05,0x15,0x11,0x05,0x10,0x07,0x02,0x05,0x15,0x10}},

{0x00,1,{0xB0}},//ClampVoltageSetting
{0xC4,2,{0x00,0x00}},//00:5.6,0x-5.6,0x33:6,0x-6

{0x00,1,{0x91}},//VGH=12V,0xVGL=-12V
{0xC5,2,{0x0B,0x62}},//,0x33,0x66,0x66}},//50}},//Pumpratio:VGH=6x,0xVGL=-5x


{0x00,1,{0x00}},//GVDD=4.6V,0xNGVDD=-4.6V
{0xD8,2,{0xB6,0xB6}},

{0x00,1,{0xB3}},//VDD_18V=1.7V,0xLVDSVDD=1.6V
{0xC5,1,{0x84}},

{0x00,1,{0xBB}},//LVDVoltageLevelSetting--
{0xC5,1,{0x8A}},

{0x00,1,{0x82}},//chopper
{0xC4,1,{0x0A}},

{0x00,1,{0xC6}},//debounce
{0xB0,1,{0x03}},

{0x00,1,{0xC2}},//prechargedisable
{0xF5,1,{0x40}},

{0x00,1,{0xC3}},//sampleholdgvdd
{0xF5,1,{0x85}},

{0x00,1,{0x87}},//enop
{0xC4,1,{0x18}},
	//**********************controlsetting**********************

{0x00,1,{0x00}},//ID1
{0xD0,1,{0x40}},

{0x00,1,{0x00}},//ID2,0xID3
{0xD1,2,{0x00,0x00}},

	//*************GAMMATUNING***********************
{0x00,1,{0x00}},
{0xE1,20,{0x1C,0x3E,0x4B,0x5B,0x6A,0x78,0x7B,0xA5,0x95,0xAE,0x55,0x42,0x54,0x39,0x38,0x2D,0x22,0x18,0x13,0x10}},
	
{0x00,1,{0x00}},
{0xE2,20,{0x1C,0x3E,0x4A,0x5B,0x6A,0x79,0x7B,0xA5,0x96,0xAE,0x55,0x42,0x55,0x39,0x38,0x2D,0x22,0x18,0x13,0x10}},
	
	//{0x00,1,{0x00}},//VCOMDC=-1.524
	//{0xD9,1,{0x48}},

	//*****************PanelTimingstatecontrol****************

{0x00,1,{0x80}},
{0xCB,11,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0x90}},
{0xcb,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0xA0}},
{0xcb,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0xB0}},
{0xcb,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0xC0}},
{0xcb,15,{0x05,0x05,0x05,0x05,0x05,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0xD0}},
{0xcb,15,{0x00,0x00,0x00,0x00,0x00,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x00,0x00}},

{0x00,1,{0xE0}},
{0xCB,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x05}},

{0x00,1,{0xF0}},
{0xCB,11,{0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}},

	//****************panelpadmappingcontrol*********************

{0x00,1,{0x80}},
{0xcc,15,{0x09,0x0B,0x0D,0x0F,0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0x90}},
{0xcc,15,{0x00,0x00,0x00,0x00,0x00,0x2D,0x2E,0x0A,0x0C,0x0E,0x10,0x02,0x04,0x00,0x00}},

{0x00,1,{0xA0}},
{0xCC,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2D,0x2E}},

{0x00,1,{0xB0}},
{0xcc,15,{0x10,0x0E,0x0C,0x0A,0x04,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0xC0}},
{0xcc,15,{0x00,0x00,0x00,0x00,0x00,0x2E,0x2D,0x0F,0x0D,0x0B,0x09,0x03,0x01,0x00,0x00}},

{0x00,1,{0xD0}},
{0xCC,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2E,0x2D}},

	//****************************paneltimingsetting*************************

{0x00,1,{0x80}},//panelVSTsetting
{0xCE,12,{0x8B,0x03,0x00,0x8A,0x03,0x00,0x89,0x03,0x00,0x88,0x03,0x00}},

{0x00,1,{0xA0}},//panelCLKA1/2setting
{0xce,14,{0x38,0x07,0x84,0xFC,0x8B,0x04,0x00,0x38,0x06,0x84,0xFD,0x8B,0x04,0x00}},
	
{0x00,1,{0xB0}},//panelCLKA3/4setting
{0xce,14,{0x38,0x05,0x84,0xFE,0x8B,0x04,0x00,0x38,0x04,0x84,0xFF,0x8B,0x04,0x00}},

{0x00,1,{0xC0}},//panelCLKb1/2setting
{0xce,14,{0x38,0x03,0x85,0x00,0x8B,0x04,0x00,0x38,0x02,0x85,0x01,0x8B,0x04,0x00}},

{0x00,1,{0xD0}},//panelCLKb3/4setting
{0xce,14,{0x38,0x01,0x85,0x02,0x8B,0x04,0x00,0x38,0x00,0x85,0x03,0x8B,0x04,0x00}},

{0x00,1,{0x80}},//panelCLKc1/2setting
{0xcf,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0x90}},//panelCLKc1/2setting
{0xcf,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0xA0}},//panelCLKd1/2setting
{0xcf,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0xB0}},//panelCLKd3/4setting
{0xcf,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0xC0}},//panelECLKsetting
{0xCF,12,{0x01,0x01,0x20,0x20,0x00,0x00,0x02,0x01,0x00,0x02,0x02}},//8th02->01
						//81,0x00,0x03gateEQRiseFall
						//01,0x00,0x03gateEQFall,0xdummyclkoff
						//08:BLANKINGCLKOUTPUTkeeping
						//02:dummyclkstopatSTV

{0x00,1,{0xB5}},//TCON_GOA_OUTSetting
{0xC5,6,{0x33,0xF1,0xFF,0x33,0xF1,0xFF}},//normaloutputwithVGH/VGL

	//**********************OriseModeDisable***************

{0x00,1,{0x00}},//OriseModeDisable
{0xFF,3,{0xFF,0xFF,0xFF}},


	{0x11,0,{}},
	{REGFLAG_DELAY, 150, {}},
	{0x29,0,{}},
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
    unsigned int data_array[16];
    data_array[0]=0x00110500;
    dsi_set_cmdq(&data_array,1,1);
    MDELAY(100);
    data_array[0]=0x00290500;
    dsi_set_cmdq(&data_array,1,1);
    MDELAY(10);
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
 
    return (LCM_ID_OTM1283A == id)?1:0;
}

static unsigned int lcm_esd_check(void)
{
    
}

static unsigned int lcm_esd_recover(void)
{

}


LCM_DRIVER otm1284a_hd720_cpt5p0_jmo_ym_lcm_drv = 
{
    .name            = "otm1284a_hd720_cpt5p0_jmo_ym",
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


