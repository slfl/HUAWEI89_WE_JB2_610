/*****************************************************************************
	Copyright (C), 1988-2012, Huawei Tech. Co., Ltd.
	FileName: otm9608a
	Author: l00183577   Version: 0.1  Date: 2012/04/21
	Description: add driver for otm9608a
	Version: 0.1
	History: 
	<author>     <time>         <defeat ID>             <desc>
*****************************************************************************/
#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
    #include <platform/disp_drv_platform.h>
#elif defined(BUILD_UBOOT)
    #include <asm/arch/mt_gpio.h>
#else
    #include <linux/delay.h>
    #include <mach/mt_gpio.h>
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  		(540)
#define FRAME_HEIGHT 		(960)

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

#define REGFLAG_DELAY       		0XFE
#define REGFLAG_END_OF_TABLE    	0xFD   // END OF REGISTERS MARKER

#define LCM_ID_OTM9608A 0x9608

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};
const static unsigned char LCD_MODULE_ID = 0x03;
const static unsigned int BL_MIN_LEVEL = 20;

#define SET_RESET_PIN(v)    			(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 				(lcm_util.udelay(n))
#define MDELAY(n) 				(lcm_util.mdelay(n))
#define LCM_DSI_CMD_MODE      1
// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)						lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)			lcm_util.dsi_write_regs(addr, pdata, byte_nums)
//#define read_reg                                            lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)                   lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

static struct LCM_setting_table
{
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[128];
};
static struct LCM_setting_table tianma_ips_init[] = {

{0xff,3,{0x96,0x08,0x01}},// Enable cmd
{0x00,1,{0x80}},
{0xff,2,{0x96,0x08}},

{0x00,1,{0x00}},
{0xA0,1,{0x00}},            

{0x00,1,{0x80}},
{0xB3,5,{0x00,0x00,0x20,0x00,0x00}},

{0x00,1,{0xC0}},
{0xB3,1,{0x09}},

{0x00,1,{0x80}},
{0xC0,9,{0x00,0x48,0x00,0x0f,0x11,0x00,0x48,0x0f,0x11}},

{0x00,1,{0x92}},
{0xC0,4,{0x00,0x10,0x00,0x13}},

{0x00,1,{0xa2}},
{0xC0,3,{0x0c,0x05,0x02}},

{0x00,1,{0xb3}},
{0xC0,2,{0x00,0x50}},
   {0x00,1,{0x81}},      
   {0xc1,1,{0x55}}, //refresh rate 0x55->60Hz
   {0x00,1,{0x80}},      
   {0xc4,2,{0x00,0x84}}, 

   {0x00,1,{0xa0}},      
   {0xc4,8,{0x33,0x09,0x90,0x2b,0x33,0x09,0x90,0x54}}, 

   {0x00,1,{0x80}},      
   {0xc5,4,{0x08,0x00,0xa0,0x11}}, 

   {0x00,1,{0x90}},      
   {0xc5,7,{0x96,0x19,0x01,0x79,0x33,0x33,0x34}}, 

   {0x00,1,{0xa0}},      
   {0xc5,7,{0x96,0x16,0x00,0x79,0x33,0x33,0x34}},

   {0x00,1,{0x00}},      
   {0xd8,2,{0x5f,0x5f}}, 

   {0x00,1,{0x00}},																																		
   {0xE1,16,{0x01,0x0e,0x15,0x0E,0x07,0x13,0x0C,0x0B,0x03,0x06,0x09,0x07,0x0C,0x0D,0x08,0x01}}, 

   {0x00,1,{0x00}},	//GOA ECLK Setting and GOA Other Options1 and GOA Signal Toggle Option Setting																																				
   {0xE2,16,{0x01,0x0e,0x15,0x0E,0x07,0x13,0x0C,0x0B,0x03,0x06,0x09,0x07,0x0C,0x0D,0x08,0x01}},

   {0x00,1,{0xb0}},      
   {0xc5,2,{0x04,0xa8}},

   {0x00,1,{0x80}},      
   {0xc6,1,{0x64}},

   {0x00,1,{0xb0}},      
   {0xc6,5,{0x03,0x10,0x00,0x1f,0x12}},

   {0x00,1,{0xb7}},      
   {0xb0,1,{0x10}},

   {0x00,1,{0xc0}},      
   {0xb0,1,{0x55}},

   {0x00,1,{0xb1}},      
   {0xb0,1,{0x03}},

   {0x00,1,{0x80}},      
   {0xcb,10,{0x05,0x00,0x00,0x00,0x00,0x05,0x00,0x00,0x00,0x00}},

   {0x00,1,{0x90}},      
   {0xcb,15,{0x55,0x55,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

   {0x00,1,{0xa0}},      
   {0xcb,15,{0x00,0x00,0x00,0x00,0x00,0x55,0x55,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

   {0x00,1,{0xb0}},      
   {0xcb,10,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

   {0x00,1,{0xc0}},      
   {0xcb,15,{0x55,0x55,0x00,0x00,0x00,0x04,0x00,0x04,0x00,0x04,0x00,0x04,0x04,0x04,0x00}},

   {0x00,1,{0xd0}},      
   {0xcb,15,{0x04,0x00,0x00,0x00,0x00,0x55,0x55,0x00,0x00,0x00,0x04,0x00,0x04,0x00,0x04}},

   {0x00,1,{0xe0}},      
   {0xcb,10,{0x00,0x04,0x04,0x04,0x00,0x04,0x00,0x00,0x00,0x00}},

   {0x00,1,{0xf0}},      
   {0xcb,10,{0x0f,0x00,0xcc,0x00,0x00,0x0f,0x00,0xcc,0xc3,0x00}},

   {0x00,1,{0x80}},      
   {0xcc,10,{0x25,0x26,0x00,0x00,0x00,0x0c,0x00,0x0a,0x00,0x10}},

   {0x00,1,{0x90}},      
   {0xcc,15,{0x00,0x0e,0x02,0x04,0x00,0x06,0x00,0x00,0x00,0x00,0x25,0x26,0x00,0x00,0x00}},

   {0x00,1,{0xa0}},      
   {0xcc,15,{0x0b,0x00,0x09,0x00,0x0f,0x00,0x0d,0x01,0x03,0x00,0x05,0x00,0x00,0x00,0x00}},

   {0x00,1,{0xb0}},      
   {0xcc,10,{0x26,0x25,0x00,0x00,0x00,0x0d,0x00,0x0f,0x00,0x09}},

   {0x00,1,{0xc0}},      
   {0xcc,15,{0x00,0x0b,0x03,0x01,0x00,0x05,0x00,0x00,0x00,0x00,0x26,0x25,0x00,0x00,0x00}},

   {0x00,1,{0xd0}},      
   {0xcc,15,{0x0e,0x00,0x10,0x00,0x0a,0x00,0x0c,0x04,0x02,0x00,0x06,0x00,0x00,0x00,0x00}},

   {0x00,1,{0x80}},      
   {0xce,12,{0x8a,0x03,0x28,0x89,0x03,0x28,0x88,0x03,0x28,0x87,0x03,0x28}},

   {0x00,1,{0x90}},      
   {0xce,15,{0x38,0x0f,0x28,0x38,0x0e,0x28,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

  {0x00,1,{0xa0}},      
  {0xce,14,{0x38,0x06,0x03,0xc1,0x00,0x28,0x00,0x38,0x05,0x03,0xc2,0x00,0x28,0x00}},

   {0x00,1,{0xb0}},      
   {0xce,14,{0x38,0x04,0x03,0xc3,0x00,0x28,0x00,0x38,0x03,0x03,0xc4,0x00,0x28,0x00}},
   
   {0x00,1,{0xc0}},      
   {0xce,14,{0x38,0x02,0x03,0xc5,0x00,0x28,0x00,0x38,0x01,0x03,0xc6,0x00,0x28,0x00}},

   {0x00,1,{0xd0}},      
   {0xce,14,{0x38,0x00,0x03,0xc7,0x00,0x28,0x00,0x30,0x00,0x03,0xc8,0x00,0x28,0x00}},

   {0x00,1,{0x80}},      
   {0xcf,14,{0xf0,0x00,0x00,0x10,0x00,0x00,0x00,0xf0,0x00,0x00,0x10,0x00,0x00,0x00}},

   {0x00,1,{0x90}},      
   {0xcf,14,{0xf0,0x00,0x00,0x10,0x00,0x00,0x00,0xf0,0x00,0x00,0x10,0x00,0x00,0x00}},

   {0x00,1,{0xa0}},      
   {0xcf,14,{0xf0,0x00,0x00,0x10,0x00,0x00,0x00,0xf0,0x00,0x00,0x10,0x00,0x00,0x00}},

   {0x00,1,{0xb0}},      
   {0xcf,14,{0xf0,0x00,0x00,0x10,0x00,0x00,0x00,0xf0,0x00,0x00,0x10,0x00,0x00,0x00}},

   {0x00,1,{0xc0}},      
   {0xcf,10,{0x01,0x01,0x20,0x20,0x00,0x00,0x02,0x01,0x00,0x00}},
   
   {0x35, 1, {0x00}},

   {0x53,1,{0x24}},  
   {0x55,1,{0x00}},  

   {0x11, 1, {0x00}},
   {REGFLAG_DELAY, 120, {}},

   {0x29, 1, {0x00}},
   {REGFLAG_DELAY, 20, {}},
   {REGFLAG_END_OF_TABLE, 0x00, {}},
};
static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
    {0x11, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
    {0x29, 0, {0x00}},
    {REGFLAG_DELAY, 20, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_mode_in_setting[] = {
    // Display off sequence
    {0x28, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Sleep Mode On
    {0x10, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_backlight_level_setting[] = {
	{0x51, 1, {0xFF}},
	
	//{0x53, 1, {0x24}},
	//{0x55,1,{0x03}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


/*Optimization LCD initialization time*/
static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for(i = 0; i < count; i++) {

        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd) {

            case REGFLAG_DELAY :
                if(table[i].count <= 20)
                    mdelay(table[i].count);
                else
                    msleep(table[i].count);
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
#if (LCM_DSI_CMD_MODE)
        // enable tearing-free
        params->dbi.te_mode                 = LCM_DBI_TE_MODE_VSYNC_ONLY;
        params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
#endif

#if (LCM_DSI_CMD_MODE)
        params->dsi.mode   = CMD_MODE;
#else
        params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif
        // DSI
        /* Command mode setting */
        params->dsi.LANE_NUM                = LCM_TWO_LANE;
        //The following defined the fomat for data coming from LCD engine.
        params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
        params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
        params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
        params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;


        // Video mode setting       
        params->dsi.intermediat_buffer_num = 2;

        params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

        params->dsi.vertical_sync_active                = 3;
        params->dsi.vertical_backporch                  = 12;
        params->dsi.vertical_frontporch                 = 2;
        params->dsi.vertical_active_line                = FRAME_HEIGHT;

        params->dsi.horizontal_sync_active              = 10;
        params->dsi.horizontal_backporch                = 50;
        params->dsi.horizontal_frontporch               = 50;
        params->dsi.horizontal_active_pixel             = FRAME_WIDTH;
        //refresh rate = 60fps , IC spec need clk < 275.5MHz
        params->dsi.PLL_CLOCK =LCM_DSI_6589_PLL_CLOCK_240_5;
        // Bit rate calculation
        //params->dsi.LPX=6;
        //params->dsi.pll_div1=39;        // fref=26MHz, fvco=fref*(div1+1)   (div1=0~63, fvco=500MHZ~1GHz)
        //params->dsi.pll_div2=1;         // div2=0~15: fout=fvo/(2*div2)

}
static void lcm_init(void)
{
    lcm_util.set_gpio_mode(GPIO_DISP_LRSTB_PIN, GPIO_MODE_00);  //huawei use GPIO 49: LSA0 to be reset pin
    lcm_util.set_gpio_dir(GPIO_DISP_LRSTB_PIN, GPIO_DIR_OUT);
    /*Optimization LCD initialization time*/
    lcm_util.set_gpio_out(GPIO_DISP_LRSTB_PIN, GPIO_OUT_ONE);
    mdelay(30);  //lcm power on , reset output high , delay 30ms ,then output low
    lcm_util.set_gpio_out(GPIO_DISP_LRSTB_PIN, GPIO_OUT_ZERO);
    msleep(30);
    lcm_util.set_gpio_out(GPIO_DISP_LRSTB_PIN, GPIO_OUT_ONE);
    msleep(50);
     push_table(tianma_ips_init, sizeof(tianma_ips_init) / sizeof(struct LCM_setting_table), 1);
    #ifdef BUILD_LK
	printf("LCD otm9608a_tianma lcm_init\n");
    #else
	printk("LCD otm9608a_tianma lcm_init\n");
    #endif
}
static void lcm_suspend(void)
{
#ifdef BUILD_LK
	printf("LCD otm9608a_tianma lcm_suspend\n");
#else
	printk("LCD otm9608a_tianma lcm_suspend\n");
#endif
    push_table(lcm_sleep_mode_in_setting, sizeof(lcm_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_resume(void)
{

#ifdef BUILD_LK
	printf("LCD otm9608a_tianma lcm_resume\n");
#else
	printk("LCD otm9608a_tianma lcm_resume\n");
#endif

	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
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

/*heighten the brightness of qimei LCD*/
static void lcm_setbacklight(unsigned int level)
{
	// Refresh value of backlight level.
        unsigned int tmp_level = 0;
        //After improving LCD refresh rate , Power waste is high , so need reduce brightness
        tmp_level = level * 70 / 100;
        //lcm_backlight_level_setting[0].para_list[0] = level;
        lcm_backlight_level_setting[0].para_list[0] = tmp_level;

#ifdef BUILD_LK
        printf("LCD otm9608a_tianma lcm_setbacklight tmp_level=%d,level=%d\n",tmp_level,level);
#else
        printk("LCD otm9608a_tianma lcm_setbacklight tmp_level=%d,level=%d\n",tmp_level,level);
#endif

	push_table(lcm_backlight_level_setting, sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);
}
/******************************************************************************
  Function:       lcm_set_pwm_level
  Description:    set different values for each LCD
  Input:          level
  Output:         NONE
  Return:         mapped_level
  Others:         none
******************************************************************************/
static unsigned int lcm_set_pwm_level(unsigned int level )
{
    unsigned int mapped_level = 0;
    if( 0 == level)
    {
        mapped_level = level;
    }
    else if(( 0 < level ) && (  BL_MIN_LEVEL > level ))
    {
        //Some 3rd APK will set values < 20 , set value(1-19) is 9
        mapped_level = 9;
    }
    else
    {
        //Reduce min brightness value
        mapped_level = (unsigned int)((level-8) * 8 /10);
    }
    #ifdef BUILD_LK
        printf("uboot:otm9608a_lcm_set_pwm mapped_level = %d,level=%d\n",mapped_level,level);
    #else
        printk("kernel:otm9608a_lcm_set_pwm mapped_level = %d,level=%d\n",mapped_level,level);
    #endif
    return mapped_level;
}
#ifndef BUILD_LK
//static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
static unsigned int count = 0;
static unsigned int uncount = 0;
static unsigned int recount = 0;


static unsigned int lcm_esd_check(void)
{
    static int err_count = 0;
    unsigned char buffer_1[12];
    unsigned int array[16];
    int i;
    unsigned char fResult;
    //printk("lcm_esd_check<<<\n");
    for(i = 0; i < 12; i++)
      buffer_1[i] = 0x00;

    //---------------------------------
    // Set Maximum Return Size
    //---------------------------------
    array[0] = 0x00013700;
    dsi_set_cmdq(array, 1, 1);

    //---------------------------------
    // Read [9Ch, 00h, ECC] + Error Report(4 Bytes)
    //---------------------------------
    read_reg_v2(0x0A, buffer_1, 7);

#if 0
    printk(KERN_EMERG "lcm_esd_check: read(0x0A)\n");
    for(i = 0; i < 7; i++)
      printk(KERN_EMERG "buffer_1[%d]:0x%x \n",i,buffer_1[i]);
#endif

    // printk(KERN_EMERG "jjyang lcm_esd_check read(0x0A)=%x\n",read_reg(0x0A));
    // printk("jjyang lcm_esd_check ID read(0x04)=%x,%x,%x\n",read_reg(0x04),read_reg(0x04),read_reg(0x04));
    // printk("jjyang lcm_esd_check ID read(0xDB)=%x\n",read_reg(0xDB));

    //---------------------------------
    // Judge Readout & Error Report
    //---------------------------------
    if(buffer_1[3] == 0x02) // Check data identifier of error report
    {
      if(buffer_1[4] & 0x02) // Check SOT sync error
        err_count++;
      else
        err_count = 0;
    }
    else
    {
      err_count = 0;
    }

    //printk(KERN_EMERG "jjyang lcm_esd_check err_count=%d\n",err_count);
    if((buffer_1[0] != 0x9C) || (err_count >= 2))
    {
      err_count = 0;
      uncount++;

      //printk(KERN_EMERG "jjyang lcm_esd_check unnormal uncount=%d\n",uncount);
      //printk("lcm_esd_check>>>\n");

      fResult = 1;
      //return TRUE;
    }
    else
    {
      count++;
      //printk(KERN_EMERG "jjyang lcm_esd_check normal count=%d\n",count);
      //printk("lcm_esd_check>>>\n");

      fResult = 0;
      //return FALSE;
    }

    //---------------------------------
    // Shut-Down Peripherial
    //---------------------------------
    array[0] = 0x00002200;
    dsi_set_cmdq(array, 1, 1);

    //---------------------------------
    // Set Maximum Return Size
    //---------------------------------
    array[0] = 0x00033700;
    dsi_set_cmdq(array, 1, 1);

    //---------------------------------
    // Clear D-PHY Buffer
    // Read [WC, WC, ECC, P1, P2, P3, CRC0, CRC1]+ Error Report(4 Bytes)
    //---------------------------------
    read_reg_v2(0xBC, buffer_1, 12);
#if 0
    printk(KERN_EMERG "lcm_esd_check: read(0xBC)\n");
    for(i = 0; i < 12; i++)
      printk(KERN_EMERG "buffer_1[%d]:0x%x \n",i,buffer_1[i]);
#endif

    if(fResult) return TRUE;
    else return FALSE;
}


/*heighten the brightness of qimei LCD*/
static unsigned int lcm_esd_recover(void)
{

    unsigned char para = 0;

	//printk(KERN_EMERG "jjyang lcm_esd_recover\n");

    lcm_init();


    //MDELAY(10);
    //push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
    //MDELAY(10);

        /*heighten the brightness of qimei LCD*/
	lcm_setbacklight(200);

	recount++;

	printk(KERN_EMERG "jjyang lcm_esd_recover recover recount=%d\n",recount);


    return TRUE;
}
#endif
static unsigned int lcm_compare_id(void)
{
    unsigned char LCD_ID_value = 0;

#ifdef BUILD_LK
	printf("otm9608a_lcm_compare_id\n");
#else
	printk("otm9608a_lcm_compare_id\n");
#endif
    LCD_ID_value = which_lcd_module();
    if(LCD_MODULE_ID == LCD_ID_value)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
LCM_DRIVER tianma_otm9608a_lcm_drv =
{
    .name           = "tm_otm9608a",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
    /*heighten the brightness of qimei LCD*/
    .set_backlight  = lcm_setbacklight,
    .set_pwm_level	= lcm_set_pwm_level,
    //.set_pwm      = lcm_setpwm,
    //.get_pwm      = lcm_getpwm
    //.set_cabcmode = lcm_setcabcmode,
    //.esd_check     = lcm_esd_check,
    /*heighten the brightness of qimei LCD*/
    //.esd_recover       = lcm_esd_recover,        
    .compare_id     = lcm_compare_id,
#endif
};
