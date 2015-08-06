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
    params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
    params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
    params->dsi.mode   = SYNC_PULSE_VDO_MODE;

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
    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.vertical_sync_active				= 6;
    params->dsi.vertical_backporch					= 5;
    params->dsi.vertical_frontporch					= 6;
    params->dsi.vertical_active_line				= FRAME_HEIGHT;

    params->dsi.horizontal_sync_active				= 36;
    params->dsi.horizontal_backporch				= 36;
    params->dsi.horizontal_frontporch				= 36;
    params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	params->dsi.PLL_CLOCK = 185; //this value must be in MTK suggested table

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

     unsigned int data_array[16];

   //HX8379C_BOE3.97IPS
    data_array[0]=0x00043902;//Enable external Command
    data_array[1]=0x7983FFB9; 
    dsi_set_cmdq(&data_array, 2, 1);
      
    data_array[0]=0x00153902;
    data_array[1]=0x181844B1;
    data_array[2]=0xD0503131;
    data_array[3]=0x388094EE;  
    data_array[4]=0x2222F838; 
    data_array[5]=0x30800022;  
    data_array[6]=0x00000000;
    dsi_set_cmdq(&data_array, 7, 1);

    data_array[0]=0x000A3902;
    data_array[1]=0x0B3C80b2; //  
    data_array[2]=0x11500004;
    data_array[3]=0x00001D42;   
    dsi_set_cmdq(&data_array, 4, 1); 

    data_array[0]=0x000B3902;
    data_array[1]=0x807080b4;   
    data_array[2]=0x12708070; 
    data_array[3]=0x00921392;    
    dsi_set_cmdq(&data_array, 4, 1);

	data_array[0]=0x33D21500; 
    dsi_set_cmdq(&data_array, 1, 1);
	
    data_array[0]=0x001E3902;
    data_array[1]=0x000700d3; 
    data_array[2]=0x06060000; 
    data_array[3]=0x00041032; 
    data_array[4]=0x036F0304; 
    data_array[5]=0x0007006F; 
    data_array[6]=0x05222107; 
    data_array[7]=0x05052305; 
    data_array[8]=0x00000923;  
    dsi_set_cmdq(&data_array, 9, 1);

    data_array[0]=0x00213902;//Enable external Command//3
    data_array[1]=0x191818d5; 
    data_array[2]=0x03000119; 
    data_array[3]=0x18202102; 
    data_array[4]=0x18181818; 
    data_array[5]=0x18181818; 
    data_array[6]=0x18181818; 
    data_array[7]=0x18181818; 
    data_array[8]=0x18181818; 
    data_array[9]=0x00000018; 
    dsi_set_cmdq(&data_array, 10, 1);

    data_array[0]=0x00213902;
    data_array[1]=0x181818d6; 
    data_array[2]=0x00030218; 
    data_array[3]=0x19212001; 
    data_array[4]=0x18181819; 
    data_array[5]=0x18181818; 
    data_array[6]=0x18181818; 
    data_array[7]=0x18181818; 
    data_array[8]=0x18181818; 
    data_array[9]=0x00000018; 
    dsi_set_cmdq(&data_array, 10, 1);

	data_array[0]=0x002B3902;
    data_array[1]=0x140F00E0; 
    data_array[2]=0x223C3836; 
    data_array[3]=0x0D0A063F; 
    data_array[4]=0x14130F17; 
    data_array[5]=0x13071313; 
    data_array[6]=0x0F001411; 
    data_array[7]=0x3C383614; 
    data_array[8]=0x0A063F22; 
    data_array[9]=0x130F170D; 
    data_array[10]=0x07131314;
    data_array[11]=0x00141113;  
    dsi_set_cmdq(&data_array, 12, 1);

/*    data_array[0]=0x00023902;
    data_array[1]=0x0000773A;
    dsi_set_cmdq(&data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00000036;
    dsi_set_cmdq(&data_array, 2, 1);*/

    data_array[0]=0x00023902;
    data_array[1]=0x000002CC;
    dsi_set_cmdq(&data_array, 2, 1);//rotation 0e

    data_array[0]=0x00033902;
    data_array[1]=0x005454B6;  
    dsi_set_cmdq(&data_array, 2, 1);

	data_array[0]=0x00023902;
    data_array[1]=0x0000773A;
    dsi_set_cmdq(&data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00000036;
    dsi_set_cmdq(&data_array, 2, 1);

    data_array[0] = 0x00110500; 
    dsi_set_cmdq(&data_array, 1, 1);
    MDELAY(120);
    
    data_array[0] = 0x00290500;
    dsi_set_cmdq(&data_array, 1, 1);
    MDELAY(20);   

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
	int array[4];
	char buffer[5];
	int id=0;

    SET_RESET_PIN(1);
	UDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(5);
    SET_RESET_PIN(1);
    MDELAY(120);

	array[0]=0x00043902;
	array[1]=0x7983FFB9;
	dsi_set_cmdq(array, 2, 1);
	
	array[0] = 0x00013700;
    dsi_set_cmdq(array, 1, 1);
	
	MDELAY(10);
	read_reg_v2(0xF4, buffer, 4);// NC 0x00  0x98 0x16
	
	id = buffer[1]; //we only need ID
	
   #ifdef BUILD_LK
		printf("zbuffer %s \n", __func__);
		printf("%s id = 0x%08x \n", __func__, id);
	#else
		printk("zbuffer %s \n", __func__);
		printk("%s id = 0x%08x \n", __func__, id);
	
   #endif
	 
  	return (LCM_ID_HX8379 == id)?1:0;
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


LCM_DRIVER hx8379c_dsi_vdo_azet_boe_wvga_lcm_drv = 
{
    .name			= "hx8379c_dsi_vdo_azet_boe_wvga",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
	.compare_id		= lcm_compare_id,
};
