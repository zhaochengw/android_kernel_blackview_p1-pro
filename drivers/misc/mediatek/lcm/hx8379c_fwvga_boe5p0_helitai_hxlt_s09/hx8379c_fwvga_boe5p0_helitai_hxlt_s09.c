#ifdef BUILD_LK

#else
	#include <linux/string.h>
	#ifndef BUILD_UBOOT
		#include <linux/kernel.h>
	#endif
#endif

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
	#include <string.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
#endif


#include "lcm_drv.h"


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
//RGK add
// ---------------------------------------------------------------------------
#include <cust_adc.h>    	// zhoulidong  add for lcm detect
#define MIN_VOLTAGE (800)     // zhoulidong  add for lcm detect
#define MAX_VOLTAGE (1000)     // zhoulidong  add for lcm detect
// zhoulidong  add for lcm detect ,read adc voltage
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define FRAME_WIDTH  					(480)
#define FRAME_HEIGHT 					(854)

#define REGFLAG_DELAY             			0XFE
#define REGFLAG_END_OF_TABLE      			0xFF   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE				0

#define LCM_ID_HX8389B 0x89

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    				(lcm_util.set_reset_pin((v)))
#define UDELAY(n) 					(lcm_util.udelay(n))
#define MDELAY(n) 					(lcm_util.mdelay(n))

#define PFX "[HX8389B_BOE]"

#if defined(BUILD_UBOOT)
	#define print(fmt, arg...)         printf(PFX "uboot %s: \n" fmt, __FUNCTION__ ,##arg)
#elif defined(BUILD_LK)
	#define print(fmt, arg...)         printf(PFX "lk %s: \n" fmt, __FUNCTION__ ,##arg)
#else
	#define print(fmt, arg...)         printk(PFX "kernel %s: \n" fmt, __FUNCTION__ ,##arg)
#endif



// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)					lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)		lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg					lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size) 		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    
       

static struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] = {
/*
{0xB9,3,{0xFF,0x83,0x79}},                                            	
{0xB1,20,{0x64,0x19,0x19,0x31,0x31,0x50,0xD0,0xEC,0x54,0x80,0x38,0x38,
	  0xF8,0x23,0x32,0x22,0x00,0x80,0x30,0x00}}, 
 {0xB2,9,{0x80,0xFE,0x0B,0x04,0x00,0x50,0x11,0x42,0x1D}},              
	{0xB4,10,{0x6E,0x6E,0x6E,0x75,0x6E,0x75,0x22,0x86,0x23,0x86}}, 
{0xCC,1,{0x02}}, 
{0xD2,1,{0x44}}, 
{0xD3,29,{0x00,0x07,0x00,0x00,0x00,0x06,0x06,0x32,0x10,0x04,0x00,0x04,
          0x03,0x6E,0x03,0x6E,0x00,0x06,0x00,0x06,0x21,0x22,0x05,0x05,
          0x23,0x05,0x05,0x23,0x09}},  
                                      	
{0xD5,32,{0x18,0x18,0x19,0x19,0x01,0x00,0x03,0x02,0x21,0x20,
          0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,
          0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18}}, 	
{0xD6,32,         {0x18,0x18,0x18,0x18,0x00,0x01,0x02,0x03,0x20,0x21,
          0x19,0x19,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,
          0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18}},                     	
{0xE0,42,         {0x0c,0x20,0x24,0x32,0x39,0x3F,0x2c,0x42,0x07,0x0b,
          0x0d,0x17,0x0E,0x12,0x14,0x12,0x14,0x06,0x11,0x12,
          0x17,0x0c,0x20,0x24,0x32,0x39,0x3F,0x2c,0x42,0x07,
          0x0b,0x0d,0x17,0x0E,0x12,0x14,0x12,0x14,0x06,0x11,
          0x12,0x17}},  
	{0xB6,2,{0x43,0x43}},                                                
	{0x11,0,{}},  	
	{REGFLAG_DELAY, 150, {}},                                                   
	{0x29,0,{}},                                                     	
	{REGFLAG_DELAY, 10, {}},	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
	*/
	{0xB9,3,{0xFF,0x83,0x79}},                                            	
{0xB1,16,{0x64,0x14,0x14,0x31,0x31,0x90,0xD0,0xEC,0x9e,0x80,0x38,0x38,
	  0xF8,0x22,0x22,0x22}}, 
 {0xB2,9,{0x80,0xFE,0x0B,0x04,0x00,0x50,0x11,0x42,0x1D}},              
	{0xB4,10,{0x6E,0x6E,0x6E,0x6E,0x6E,0x6E,0x22,0x86,0x23,0x86}}, 
	{0xc7,4,{0x00,0x00,0x00,0xc0}}, 
{0xCC,1,{0x02}}, 
{0xD2,1,{0x00}}, 
{0xD3,29,{0x00,0x07,0x00,0x00,0x00,0x0e,0x0e,0x32,0x10,0x04,0x00,0x02,
          0x03,0x6E,0x03,0x6E,0x00,0x06,0x00,0x06,0x21,0x22,0x05,0x03,
          0x23,0x05,0x05,0x23,0x09}},  
                                      	
{0xD5,34,{0x18,0x18,0x19,0x19,0x01,0x00,0x03,0x02,0x21,0x20,
          0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,
          0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x00,0x00}}, 	
{0xD6,32,         {0x18,0x18,0x18,0x18,0x02,0x03,0x00,0x01,0x20,0x21,
          0x19,0x19,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,
          0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18}},                     	
{0xE0,42,         {0x00,0x0d,0x1c,0x17,0x18,0x3F,0x34,0x3c,0x0b,0x0f,
          0x10,0x19,0x10,0x14,0x17,0x16,0x17,0x09,0x13,0x13,
          0x17,0x00,0x0d,0x1c,0x16,0x19,0x3F,0x33,0x3e,0x0a,
          0x0f,0x11,0x1a,0x10,0x14,0x16,0x14,0x15,0x07,0x12,
          0x13,0x18}},  
	{0xB6,2,{0x7a,0x7a}},                                                
	{0x11,0,{}},  	
	{REGFLAG_DELAY, 150, {}},                                                   
	{0x29,0,{}},                                                     	
	{REGFLAG_DELAY, 10, {}},	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


#define LCD_PIN_ID_XL_HSD_IC_HX8389B 1
#define LCD_PIN_ID_XL_BOE_IC_HX8389B 3


static void init_lcm_registers_XL_BOE(void)
{
//显亮	BOE玻璃5.5寸HX8389b 20131206 新到初始化代�?//20140321 调整锁屏唤醒�?	unsigned int data_array[16];

	

unsigned int data_array[16];

data_array[0]=0x00043902;
data_array[1]=0x7983FFB9;
dsi_set_cmdq(&data_array,2,1);

data_array[0]=0x00153902;
data_array[1]=0x191964B1;
data_array[2]=0xD0503131;
data_array[3]=0x388054EC;
data_array[4]=0x2222F838;
data_array[5]=0x30800022;
data_array[6]=0x00000000;
dsi_set_cmdq(&data_array,7,1);

data_array[0]=0x000A3902;
data_array[1]=0x0BFE80B2;
data_array[2]=0x11500004;
data_array[3]=0x00001D42;
dsi_set_cmdq(&data_array,4,1);

data_array[0]=0x000B3902;
data_array[1]=0x7A6E7AB4;
data_array[2]=0x226E7A6E;
data_array[3]=0x00862386;
dsi_set_cmdq(&data_array,4,1);


data_array[0]=0x00023902;
data_array[1]=0x000002CC;
dsi_set_cmdq(&data_array,2,1);

data_array[0]=0x001E3902;
data_array[1]=0x000700D3;
data_array[2]=0x06060000;
data_array[3]=0x00041032;
data_array[4]=0x036E0304;
data_array[5]=0x0006006E;
data_array[6]=0x05222106;
data_array[7]=0x05052305;
data_array[8]=0x00000923;
dsi_set_cmdq(&data_array,9,1);
/*
data_array[0]=0x00213902;
data_array[1]=0x181818D5;
data_array[2]=0x02010018;
data_array[3]=0x19212003;
data_array[4]=0x18181819;
data_array[5]=0x18181818;
data_array[6]=0x18181818;
data_array[7]=0x18181818;
data_array[8]=0x18181818;
data_array[9]=0x00000018;
dsi_set_cmdq(&data_array,10,1);
*/

data_array[0]=0x00233902;
data_array[1]=0x191818D5;
data_array[2]=0x03000119;
data_array[3]=0x18202102;
data_array[4]=0x18181818;
data_array[5]=0x18181818;
data_array[6]=0x18181818;
data_array[7]=0x18181818;
data_array[8]=0x18181818;
data_array[9]=0x00000018;
dsi_set_cmdq(&data_array,10,1);

data_array[0]=0x00213902;
data_array[1]=0x181818D6;
data_array[2]=0x02010018;
data_array[3]=0x19212003;
data_array[4]=0x18181819;
data_array[5]=0x18181818;
data_array[6]=0x18181818;
data_array[7]=0x18181818;
data_array[8]=0x18181818;
data_array[9]=0x00000018;
dsi_set_cmdq(&data_array,10,1);

data_array[0]=0x002B3902;
data_array[1]=0x1C0D00E0;//0x24060CE0
data_array[2]=0x343F1817;
data_array[3]=0x100F0B3C;
data_array[4]=0x17141019;
data_array[5]=0x13091716;
data_array[6]=0x0D001713;
data_array[7]=0x3F19161C;
data_array[8]=0x0F0A3E33;
data_array[9]=0x14101A11;
data_array[10]=0x07151416;
data_array[11]=0x00181312;
dsi_set_cmdq(&data_array,12,1);

data_array[0]=0x00033902;
data_array[1]=0x005353B6;//2525
dsi_set_cmdq(&data_array,2,1);

data_array[0] = 0x00110500;                  
dsi_set_cmdq(&data_array, 1, 1);
MDELAY(150);//john modify from 200-->120,no need 200ms


data_array[0] = 0x00290500;                          
dsi_set_cmdq(&data_array, 1, 1);
MDELAY(10);
/*
data_array[0]=0x00043902;
data_array[1]=0x7983FFB9;
dsi_set_cmdq(&data_array,2,1);

data_array[0]=0x00143902;
data_array[1]=0x1a1a64B1;
data_array[2]=0xD0909131;
data_array[3]=0x3880D8EC;
data_array[4]=0x3223F838;
data_array[5]=0x30800022;
dsi_set_cmdq(&data_array,6,1);

data_array[0]=0x000A3902;
data_array[1]=0x0BFE80B2;
data_array[2]=0x11500004;
data_array[3]=0x00001D42;
dsi_set_cmdq(&data_array,4,1);

data_array[0]=0x000B3902;
data_array[1]=0x008000B4;
data_array[2]=0x00860086;
data_array[3]=0x00860086;
dsi_set_cmdq(&data_array,4,1);


data_array[0]=0x00023902;
data_array[1]=0x000033D2;
dsi_set_cmdq(&data_array,2,1);

data_array[0]=0x001E3902;
data_array[1]=0x000700D3;
data_array[2]=0x06060000;
data_array[3]=0x00061032;
data_array[4]=0x03700306;
data_array[5]=0x00080070;
data_array[6]=0x06111108;
data_array[7]=0x06061306;
data_array[8]=0x00000913;
dsi_set_cmdq(&data_array,9,1);


data_array[0]=0x00023902;
data_array[1]=0x000006CC;
dsi_set_cmdq(&data_array,2,1);

data_array[0]=0x00213902;
data_array[1]=0x181818D5;
data_array[2]=0x02010018;
data_array[3]=0x19212003;
data_array[4]=0x18181819;
data_array[5]=0x18181818;
data_array[6]=0x18181818;
data_array[7]=0x18181818;
data_array[8]=0x18181818;
data_array[9]=0x00000018;
dsi_set_cmdq(&data_array,10,1);

data_array[0]=0x00213902;
data_array[1]=0x191818D6;
data_array[2]=0x03000119;
data_array[3]=0x18202102;
data_array[4]=0x18181818;
data_array[5]=0x18181818;
data_array[6]=0x18181818;
data_array[7]=0x18181818;
data_array[8]=0x18181818;
data_array[9]=0x00000018;
dsi_set_cmdq(&data_array,10,1);

data_array[0]=0x002B3902;
data_array[1]=0x241f00E0;//0x24060CE0
data_array[2]=0x2C3F3932;
data_array[3]=0x0D0B0742;
data_array[4]=0x14120E17;
data_array[5]=0x11061412;
data_array[6]=0x1f001813;
data_array[7]=0x3F393224;
data_array[8]=0x0B07422C;
data_array[9]=0x120E170D;
data_array[10]=0x06141214;
data_array[11]=0x00181311;
dsi_set_cmdq(&data_array,12,1);

data_array[0]=0x00033902;
data_array[1]=0x002B2BB6;//2525
dsi_set_cmdq(&data_array,2,1);

data_array[0] = 0x00110500;                  
dsi_set_cmdq(&data_array, 1, 1);
MDELAY(180);//john modify from 200-->120,no need 200ms


data_array[0] = 0x00290500;                          
dsi_set_cmdq(&data_array, 1, 1);
MDELAY(10);
*/



}

static struct LCM_setting_table lcm_set_window[] = {
	{0x2A,	4,	{0x00, 0x00, (FRAME_WIDTH>>8), (FRAME_WIDTH&0xFF)}},
	{0x2B,	4,	{0x00, 0x00, (FRAME_HEIGHT>>8), (FRAME_HEIGHT&0xFF)}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 10, {}},
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_in_setting[] = {
	// Display off sequence
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 40, {}},

    // Sleep Mode On
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
		params->dbi.te_mode 			= LCM_DBI_TE_MODE_VSYNC_ONLY;
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
#else
		params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif
	
		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM		    = LCM_TWO_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		params->dsi.word_count = 480*3;
		params->dsi.vertical_active_line = 854;
		params->dsi.compatibility_for_nvk = 0;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
		params->dsi.packet_size=256;

		// Video mode setting		
		params->dsi.intermediat_buffer_num = 2;

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
/*
		params->dsi.vertical_sync_active				= 9;
		params->dsi.vertical_backporch					= 6;//12,6
		params->dsi.vertical_frontporch					= 14;//5		
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 
	
		params->dsi.horizontal_sync_active				= 32;//5
		params->dsi.horizontal_backporch				= 32;
		params->dsi.horizontal_frontporch				= 32;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
*/
		params->dsi.vertical_sync_active = 5;
		params->dsi.vertical_backporch = 6;
		params->dsi.vertical_frontporch = 6;
		params->dsi.vertical_active_line = FRAME_HEIGHT;
		
		params->dsi.horizontal_sync_active =54;
		params->dsi.horizontal_backporch = 54;
		params->dsi.horizontal_frontporch = 54;
		params->dsi.horizontal_blanking_pixel = 60;
		params->dsi.horizontal_active_pixel = FRAME_WIDTH;
		// 6572 平台
	params->dsi.PLL_CLOCK = 210; //this value must be in MTK suggested table
}

static void lcm_init(void)
{
	
	SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
        MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);
//       	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	init_lcm_registers_XL_BOE();
}

static void lcm_suspend(void)
{
	//print("%s\n", __func__);
	unsigned int data_array[16];
	
	data_array[0] = 0x00100500;                  
        dsi_set_cmdq(&data_array, 1, 1);
        MDELAY(120);//

	SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
        MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);


	//push_table(lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) / sizeof(struct LCM_setting_table), 1);

}


static void lcm_resume(void)
{
//	print("%s\n", __func__);
//	print("init_lcm_registers_XL_BOE\n");
	lcm_init();
	//push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}


static unsigned int lcm_compare_id(void)
{
    unsigned int id = 0, id2 = 0;
    unsigned char buffer[2];
    unsigned int data_array[16];
//  printf("sunkui ,%s, id = 0x%08x\n", __func__, id);

    

    SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);	
  //  return 1;
    /*	
    data_array[0] = 0x00110500;		// Sleep Out
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(120);
    */
    
    //*************Enable CMD2 Page1  *******************//
    data_array[0]=0x00043902;
    data_array[1]=0x7983FFB9;
   // data_array[2]=0x00000108;
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(10); 
    data_array[0]=0x00033902;
    data_array[1]=0x009351ba;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00013700;// read id return two byte,version and id
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(10); 
    
    read_reg_v2(0xF4, buffer, 1);
    id = buffer[0]; //we only need ID
    id2= buffer[1]; //we test buffer 1
    #ifdef BUILD_LK
		printf("%s, lcm: id = 0x%08x\n", __func__, id);
    #else
		printk("%s, lcm: id = 0x%08x\n", __func__, id);
    #endif

	if(0x79 == id)
		{

			    return 1;
		}
//	if ( get_lcm_pin_id()==LCD_PIN_ID_XL_BOE_IC_HX8389B )

	return 0;
}


LCM_DRIVER hx8379c_fwvga_boe5p0_helitai_hxlt_s09_lcm_drv = 
{
    	.name		= "hx8379c_fwvga_boe5p0_helitai_hxlt_s09",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
};

