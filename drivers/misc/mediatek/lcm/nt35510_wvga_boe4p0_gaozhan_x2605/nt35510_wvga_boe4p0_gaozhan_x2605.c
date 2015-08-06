#ifndef BUILD_LK
#include <linux/string.h>
#include <mach/mt_gpio.h>
#else
#include <platform/mt_gpio.h>
#endif

#include "lcm_drv.h"
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#include <cust_adc.h>
#define MIN_VOLTAGE (200)
#define MAX_VOLTAGE (400)
#define LCM_ID_HX8379			0x79 

#define FRAME_WIDTH  										(480)
#define FRAME_HEIGHT 										(800)

#define REGFLAG_DELAY             							0xFEE
#define REGFLAG_END_OF_TABLE      							0xFFE   // END OF REGISTERS MARKER

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V3(para_tbl,size,force_update)        	lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)									lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] = {
	
	{0xF0, 5, {0x55,0xAA,0x52,0x08,0x01}},
	
	{0xB6, 3, {0x34,0x34,0x34}},
	
	{0xB0, 3, {0x09,0x09,0x09}},
	
	{0xB7, 3, {0x24,0x24,0x24}}, 
	
	{0xB1, 3, {0x0C,0x0C,0x0C}},
	
	{0xB8, 1, {0x34}},

	{0xB2, 1, {0x00}},
			
	{0xB9, 3, {0x24,0x24,0x24}}, 
	
	{0xB3, 3, {0x05,0x05,0x05}},

	{0xBF, 1, {0x01}},  
	
	{0xBA, 3, {0x34,0x34,0x34}},  
	
	{0xB5, 3, {0x0B,0x0B,0x0B}}, 
	
	{0xBC, 3, {0x00,0x90,0x00}}, 
 
	{0xBD, 3, {0x00,0x90,0x00}},  
	
	{0xBE, 2, {0x00,0x5D}},  
	
	//Positive Gamma for RED
	{0xD1, 52,{0x00,0x37,0x00,0x52,0x00,0x7B,0x00,0x99,0x00,0xB1,0x00,0xD2,0x00,0xF6,0x01,0x27,0x01,0x4E,0x01,0x8C,0x01,0xBE,0x02,0x0B,0x02,0x48,0x02,0x4A,0x02,0x7E,0x02,0xBC,0x02,0xE1,0x03,0x10,0x03,0x31,0x03,0x5A,0x03,0x94,0x03,0x94,0x03,0x9F,0x03,0xB3,0x03,0xB9,0x03,0xC1}},
		
	//Positive Gamma for GREEN
	{0xD2, 52,{0x00,0x37,0x00,0x52,0x00,0x7B,0x00,0x99,0x00,0xB1,0x00,0xD2,0x00,0xF6,0x01,0x27,0x01,0x4E,0x01,0x8C,0x01,0xBE,0x02,0x0B,0x02,0x48,0x02,0x4A,0x02,0x7E,0x02,0xBC,0x02,0xE1,0x03,0x10,0x03,0x31,0x03,0x5A,0x03,0x94,0x03,0x94,0x03,0x9F,0x03,0xB3,0x03,0xB9,0x03,0xC1}},	
	
	//Positive Gamma for Blue
	{0xD3, 52,{0x00,0x37,0x00,0x52,0x00,0x7B,0x00,0x99,0x00,0xB1,0x00,0xD2,0x00,0xF6,0x01,0x27,0x01,0x4E,0x01,0x8C,0x01,0xBE,0x02,0x0B,0x02,0x48,0x02,0x4A,0x02,0x7E,0x02,0xBC,0x02,0xE1,0x03,0x10,0x03,0x31,0x03,0x5A,0x03,0x94,0x03,0x94,0x03,0x9F,0x03,0xB3,0x03,0xB9,0x03,0xC1}},
	
	//Negative Gamma for RED
	{0xD4, 52,{0x00,0x37,0x00,0x52,0x00,0x7B,0x00,0x99,0x00,0xB1,0x00,0xD2,0x00,0xF6,0x01,0x27,0x01,0x4E,0x01,0x8C,0x01,0xBE,0x02,0x0B,0x02,0x48,0x02,0x4A,0x02,0x7E,0x02,0xBC,0x02,0xE1,0x03,0x10,0x03,0x31,0x03,0x5A,0x03,0x94,0x03,0x94,0x03,0x9F,0x03,0xB3,0x03,0xB9,0x03,0xC1}},
	
	//Negative Gamma for Green
	{0xD5, 52,{0x00,0x37,0x00,0x52,0x00,0x7B,0x00,0x99,0x00,0xB1,0x00,0xD2,0x00,0xF6,0x01,0x27,0x01,0x4E,0x01,0x8C,0x01,0xBE,0x02,0x0B,0x02,0x48,0x02,0x4A,0x02,0x7E,0x02,0xBC,0x02,0xE1,0x03,0x10,0x03,0x31,0x03,0x5A,0x03,0x94,0x03,0x94,0x03,0x9F,0x03,0xB3,0x03,0xB9,0x03,0xC1}},
		
	//Negative Gamma for Blue
	{0xD6, 52,{0x00,0x37,0x00,0x52,0x00,0x7B,0x00,0x99,0x00,0xB1,0x00,0xD2,0x00,0xF6,0x01,0x27,0x01,0x4E,0x01,0x8C,0x01,0xBE,0x02,0x0B,0x02,0x48,0x02,0x4A,0x02,0x7E,0x02,0xBC,0x02,0xE1,0x03,0x10,0x03,0x31,0x03,0x5A,0x03,0x94,0x03,0x94,0x03,0x9F,0x03,0xB3,0x03,0xB9,0x03,0xC1}},
	

	{0xF0, 5, {0x55,0xAA,0x52,0x08,0x00}},

	{0xB5, 1, {0x50}},

	{0xB1, 2, {0xFC,0x00}},   	
	
	{0xB6, 1, {0x0A}},	
	
	{0xB7, 2, {0x70,0x70}},  
	
	{0xB8, 4, {0x01,0x05,0x05,0x05}},	 
	
	{0xBC, 3, {0x00,0x00,0x00}},	
	
	{0xCC, 3, {0x03,0x00,0x00}},
	
	{0xBD, 5, {0x01,0x84,0x07,0x31,0x00}},

	{0xBA, 1, {0x01}},	 
	
	{0xFF, 4, {0xAA,0x55,0x25,0x01}},
	
	{0x35, 1, {0x00}},
	
	{0x3A, 1, {0x77}},

	{0x11, 0, {0x00}},
	{REGFLAG_DELAY, 120, {}},

	{0x29, 0, {0x00}},
	{REGFLAG_DELAY, 150, {}},

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
    params->dsi.mode   = SYNC_EVENT_VDO_MODE;//BURST_VDO_MODE

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
    params->dsi.packet_size=256;
    params->dsi.intermediat_buffer_num = 2;
    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.vertical_sync_active				= 2;
    params->dsi.vertical_backporch					= 8;             //->16
    params->dsi.vertical_frontporch					= 10;  //
    params->dsi.vertical_active_line				= FRAME_HEIGHT;
    
    params->dsi.horizontal_sync_active				= 2;
    params->dsi.horizontal_backporch				= 50;
    params->dsi.horizontal_frontporch				= 50;
     params->dsi.horizontal_blanking_pixel				= 60;
    params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	params->dsi.PLL_CLOCK = 225; //this value must be in MTK suggested table

//    params->dsi.noncont_clock=1;
//    params->dsi.noncont_clock_period=2;

	params->dsi.esd_check_enable = 0;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.lcm_esd_check_table[0].cmd          = 0x0a;
	params->dsi.lcm_esd_check_table[0].count        = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;
}


static void lcm_init(void)
{
    SET_RESET_PIN(1);
	UDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(5);
    SET_RESET_PIN(1);
    MDELAY(120);
  push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);

}


static void lcm_suspend(void)
{
    SET_RESET_PIN(1);
	UDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(5);
    SET_RESET_PIN(1);
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
	data_array[3]= 0x00053902;
	data_array[4]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[5]= (y1_LSB);
	data_array[6]= 0x002c3909;

	dsi_set_cmdq(&data_array, 7, 0);

}


static unsigned int lcm_compare_id(void)
{
	unsigned int id = 0;
	unsigned char buffer[3];
	unsigned int array[16];
	
	SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);
    array[0]=0x00063902;
    array[1]=0x52aa55f0;
    array[2]=0x00000108;
    dsi_set_cmdq(array, 3, 1);
    
	array[0] = 0x00033700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0xC5, buffer, 3);
	id = buffer[1]; //we only need ID
#ifdef BUILD_LK
	/*The Default Value should be 0x00,0x80,0x00*/
	printf("\n\n\n\n[soso]%s, id0 = 0x%08x,id1 = 0x%08x,id2 = 0x%08x\n", __func__, buffer[0],buffer[1],buffer[2]);
#endif 
    return (id == 0x10)?1:0;
}

static unsigned int rgk_lcm_compare_id(void)
{
    int data[4] = {0,0,0,0};
    int res = 0;
    int rawdata = 0;
    int lcm_vol = 0;

#ifdef AUXADC_LCM_VOLTAGE_CHANNEL
    res = IMM_GetOneChannelValue(AUXADC_LCM_VOLTAGE_CHANNEL,data,&rawdata);
    if(res < 0)
    { 
	#ifdef BUILD_LK
	printf("[adc_uboot]: get data error\n");
	#endif
	return 0;
		   
    }
#endif

    lcm_vol = data[0]*1000+data[1]*10;

#ifdef BUILD_LK
	printf("[adc_uboot]: lcm_vol= %d\n",lcm_vol);
#else
	printk("[adc_kernel]: lcm_vol= %d\n",lcm_vol);
#endif
    if (lcm_vol>=MIN_VOLTAGE &&lcm_vol <= MAX_VOLTAGE)
    {
		return 1;
    }

    return 0;

}


LCM_DRIVER nt35510_wvga_boe4p0_gaozhan_x2605_lcm_drv = 
{
    .name			= "nt35510_wvga_boe4p0_gaozhan_x2605",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
	.compare_id		= lcm_compare_id,
};
