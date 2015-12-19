#if 1 /* def CONFIG_CYPRESS_TTSP */
#include <linux/cyttsp4_bus.h>
#include <linux/cyttsp4_core.h>
#include <linux/cyttsp4_i2c.h>
#include <linux/cyttsp4_btn.h>
#include <linux/cyttsp4_mt.h>

#include <mach/mt_gpio.h>
#include <mach/mt_typedefs.h>
#include <linux/input.h>
#include <mach/mt_pm_ldo.h>
#include <linux/hardware_self_adapt.h>

#define CYTTSP4_I2C_TCH_ADR 0x1a
#define CYTTSP4_I2C_IRQ_GPIO 70	/* sample value from Blue */
#define CYTTSP4_I2C_RST_GPIO 10	/* sample value from Blue */

//#define CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_PLATFORM_FW_UPGRADE
#define CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_PLATFORM_TTCONFIG_UPGRADE 1

extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
                                     kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
                                     kal_bool auto_umask);

#define CUST_EINT_TOUCH_PANEL_NUM              5
//#define CUST_EINT_TOUCH_PANEL_NUM              79
#define CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN      0
#define CUST_EINT_TOUCH_PANEL_POLARITY         CUST_EINT_POLARITY_LOW
#define CUST_EINT_TOUCH_PANEL_SENSITIVE        CUST_EINT_EDGE_SENSITIVE
#define CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN      CUST_EINT_DEBOUNCE_DISABLE

#define CUST_EINT_POLARITY_LOW              0
#define CUST_EINT_POLARITY_HIGH             1
#define CUST_EINT_DEBOUNCE_DISABLE          0
#define CUST_EINT_DEBOUNCE_ENABLE           1
#define CUST_EINT_EDGE_SENSITIVE            0
#define CUST_EINT_LEVEL_SENSITIVE           1

extern void printGPIO_Status(int gpioIdx);  // printGPIO_Status(GPIO9)

// GPIO77 mode1:URXD1 mode2:EINT79

/* // Orig Eint5 */
#define GPIO_CTP_EINT_PIN         GPIO124
#define GPIO_CTP_EINT_PIN_M_GPIO  GPIO_MODE_00
#define GPIO_CTP_EINT_PIN_M_EINT  GPIO_MODE_01


//#define GPIO_CTP_RST_PIN         GPIO125  //For MTK EVB

#define GPIO_CTP_RST_PIN         GPIO140  //For Huawei

#define GPIO_CTP_RST_PIN_M_GPIO  GPIO_MODE_00


//lm DMA
#include <linux/dma-mapping.h> 

static u8 *I2CDMABuf_va = NULL;
static u32 I2CDMABuf_pa = NULL;
  
static void cyttsp4_init_i2c_alloc_dma_buffer(void)
{
  I2CDMABuf_va = (u8 *)dma_alloc_coherent(NULL, 4096, &I2CDMABuf_pa, GFP_KERNEL);
  if(!I2CDMABuf_va){
	  pr_err("Allocate DMA I2C Buffer failed!\n");
	}
  else{
	  pr_err("Allocate DMA I2C Buffer Success!\n");
	}

}
static void cyttsp4_init_i2c_free_dma_buffer(void)
{
  dma_free_coherent(NULL, 4096, I2CDMABuf_va, I2CDMABuf_pa);
  I2CDMABuf_va = NULL;
  I2CDMABuf_pa = 0;
  pr_info("Free DMA I2C Buffer Success!\n");
}

int cyttsp4_MTK_i2c_write(struct i2c_client *client, const uint8_t *buf, int len)
{
  //pr_info("cyttsp4_MTK_i2c_write() client:0x%p buf:0x%p len:%d\n", client, buf, len);

  int i = 0;

  if(len <= 8){
	  //pr_info("cyttsp4_MTK_i2c_write() length < 8! Normal mode\n");
	  client->addr = client->addr & I2C_MASK_FLAG;
	  return i2c_master_send(client, buf, len);
	}
  else{
	  //pr_info("cyttsp4_MTK_i2c_write() length > 8! DMA mode\n");
	  for(i = 0 ; i < len; i++){
		I2CDMABuf_va[i] = buf[i];
	  }

	  client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
	  return i2c_master_send(client, I2CDMABuf_pa, len);
	}    
}

int cyttsp4_MTK_i2c_read(struct i2c_client *client, uint8_t *buf, int len)
{
  //pr_info("cyttsp4_MTK_i2c_read() client:0x%p buf:0x%p len:%d\n", client, buf, len);
  int i = 0, err = 0;

  if(len <= 8){
	  //pr_info("cyttsp4_MTK_i2c_read() length < 8! Normal mode\n");
	  client->addr = client->addr & I2C_MASK_FLAG;
	  return i2c_master_recv(client, buf, len);
	}
  else{
	  //pr_info("cyttsp4_MTK_i2c_read() length > 8! DMA mode\n");
	  client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
	  err = i2c_master_recv(client, I2CDMABuf_pa, len);
                   
	  if(err < 0){
		  return err;
		}

	  for(i = 0; i < len; i++){
		  buf[i] = I2CDMABuf_va[i];
		}
	  return err;

	}
}


extern void eint_interrupt_handler(void) ;


void cyttsp4_mtk_gpio_interrupt_register()
{
  //printk("cyttsp4_mtk_gpio_interrupt_register\n");
	mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
	mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, eint_interrupt_handler, 1);
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
}

void cyttsp4_mtk_gpio_interrupt_enable()
{
  //printk("cyttsp4_mtk_gpio_interrupt_enable\n");
  mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
}

void cyttsp4_mtk_gpio_interrupt_disable()
{
  printk("cyttsp4_mtk_gpio_interrupt_disable\n");
  mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
}


static int cyttsp4_xres(struct cyttsp4_core_platform_data *pdata,
		struct device *dev)
{
  int rc = 0;
  /* mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO); */
  /* mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT); */

  /* printGPIO_Status(GPIO77); */
  /* printGPIO_Status(GPIO124); */
  /* printGPIO_Status(GPIO140); */

  //  cyttsp4_mtk_gpio_interrupt_disable();
  dev_info(dev,"%s: RESET CYTTSP gpio=%d ----r=%d start\n", __func__,GPIO_CTP_RST_PIN, rc);
  
  #if 0
  mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);  
  mdelay(2);
  mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
  mdelay(4);
  mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);  
  mdelay(2);
  #else 
  mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);  
  msleep(20);
  mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
  msleep(40);
  mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);  
  msleep(20);
  #endif 
  dev_info(dev,"%s: RESET CYTTSP gpio=%d r=%d\n", __func__,GPIO_CTP_RST_PIN, rc);
  //  cyttsp4_mtk_gpio_interrupt_enable();
  //  testGPIO77();
  return rc;
}

//add begin by linghai
static ssize_t cyttps4_virtualkeys_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
		__stringify(EV_KEY) ":"
		__stringify(KEY_BACK) ":120:1340:200:100"
		":" __stringify(EV_KEY) ":"
		__stringify(KEY_HOMEPAGE) ":360:1340:200:100"
		":" __stringify(EV_KEY) ":"
		__stringify(KEY_MENU) ":600:1340:200:100"
		"\n");
}

static struct kobj_attribute cyttsp4_virtualkeys_attr = {
	.attr = {
		.name = "virtualkeys.cyttsp4_mt",
		.mode = S_IRUGO,
	},
	.show = &cyttps4_virtualkeys_show,
};

static struct attribute *cyttsp4_properties_attrs[] = {
	&cyttsp4_virtualkeys_attr.attr,
	NULL
};

static struct attribute_group cyttsp4_properties_attr_group = {
	.attrs = cyttsp4_properties_attrs,
};
// add end by linghai


static int cyttsp4_init(struct cyttsp4_core_platform_data *pdata,
		int on, struct device *dev)
{
  printk("cyttsp4_init\n");
	hw_product_type board_id;
	board_id=get_hardware_product_version();
	int rc = 0;
//add by linghai begin
	struct kobject *properties_kobj;
	int ret;
//add by linghai end
	if (on) {
		cyttsp4_init_i2c_alloc_dma_buffer();

	  mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	  mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	  mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	  
	  mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	  mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	  mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	  mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);

		if((board_id & HW_VER_MAIN_MASK) == HW_G700U_VER)
		{
			hwPowerOn(MT65XX_POWER_LDO_VGP4, VOL_2800, "TP");

			properties_kobj = kobject_create_and_add("board_properties", NULL);
			if (properties_kobj)
			ret = sysfs_create_group(properties_kobj,
				&cyttsp4_properties_attr_group);
			if (!properties_kobj || ret)
				pr_err("%s: failed to create board_properties\n", __func__);
		}
		else if((board_id & HW_VER_MAIN_MASK) == HW_G610U_VER)
		{
			if((board_id) == HW_G610U_VER_A)
			{
				hwPowerOn(MT65XX_POWER_LDO_VGP4, VOL_2800, "TP");
			}
			else
			{
				hwPowerOn(MT65XX_POWER_LDO_VGP5, VOL_1800, "TP");
				hwPowerOn(MT65XX_POWER_LDO_VGP4, VOL_2800, "TP");
			}
		}
		else
			pr_err("power on cyttsp4 error\n");
	}
	else {
			if((board_id & HW_VER_MAIN_MASK) == HW_G700U_VER) 
			{
				hwPowerDown(MT65XX_POWER_LDO_VGP4, "TP");
			}
			else if((board_id & HW_VER_MAIN_MASK) == HW_G610U_VER)
			{
				if((board_id) == HW_G610U_VER_A)
				{
					hwPowerDown(MT65XX_POWER_LDO_VGP4, "TP");
				}
				else
				{
					hwPowerDown(MT65XX_POWER_LDO_VGP5, "TP");
					hwPowerDown(MT65XX_POWER_LDO_VGP4, "TP");
				}
			}
			else
				pr_err("power down cyttsp4 error\n");
	  cyttsp4_init_i2c_free_dma_buffer();
	}

	

	/* dev_info(dev, */
	/* 	"%s: INIT CYTTSP RST gpio=%d and IRQ gpio=%d r=%d\n", */
	/* 	__func__, CYTTSP4_I2C_IRQ_GPIO, CYTTSP4_I2C_RST_GPIO, rc); */
	return rc;
}

static int cyttsp4_wakeup(struct device *dev)
{
	int rc = 0;

	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_EINT_PIN, GPIO_OUT_ZERO);
	udelay(2000);
	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);

	dev_info(dev,
		"%s: WAKEUP CYTTSP gpio=%d r=%d\n", __func__,
		GPIO_CTP_EINT_PIN, rc);
	return rc;
}

static int cyttsp4_sleep(struct device *dev)
{
	return 0;
}

static int cyttsp4_power(struct cyttsp4_core_platform_data *pdata,
		int on, struct device *dev, atomic_t *ignore_irq)
{
	if (on)
		return cyttsp4_wakeup(dev);

	return cyttsp4_sleep(dev);
}
/* Button to keycode conversion */
static u16 cyttsp4_btn_keys[] = {
	/* use this table to map buttons to keycodes (see input.h) */
	KEY_BACK,		/* 158 */
	KEY_HOMEPAGE,	/* 172 */
	KEY_MENU,		/* 139 */
	KEY_SEARCH,		/* 217 */
	KEY_VOLUMEDOWN,	/* 114 */
	KEY_VOLUMEUP,		/* 115 */
	KEY_CAMERA,		/* 212 */
	KEY_POWER		/* 116 */
};

static struct touch_settings cyttsp4_sett_btn_keys = {
	.data = (uint8_t *)&cyttsp4_btn_keys[0],
	.size = ARRAY_SIZE(cyttsp4_btn_keys),
	.tag = 0,
};

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_PLATFORM_FW_UPGRADE
#include "HUAWEI_G700_FW_V0004.h"
static struct cyttsp4_touch_firmware cyttsp4_firmware = {
	.img = cyttsp4_img,
	.size = ARRAY_SIZE(cyttsp4_img),
	.ver = cyttsp4_ver,
	.vsize = ARRAY_SIZE(cyttsp4_ver),
};
#else
static struct cyttsp4_touch_firmware cyttsp4_firmware = {
	.img = NULL,
	.size = 0,
	.ver = NULL,
	.vsize = 0,
};
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_PLATFORM_TTCONFIG_UPGRADE
#include <linux/cyttsp4_params.h>
static struct touch_settings cyttsp4_sett_param_regs = {
	.data = (uint8_t *)&cyttsp4_param_regs[0],
	.size = ARRAY_SIZE(cyttsp4_param_regs),
	.tag = 0,
};

static struct touch_settings cyttsp4_sett_param_size = {
	.data = (uint8_t *)&cyttsp4_param_size[0],
	.size = ARRAY_SIZE(cyttsp4_param_size),
	.tag = 0,
};
#include <linux/Ofilm_G700_config.h>
static struct touch_settings cyttsp4_G700_sett_ofilm_param_regs = {
       .data = (uint8_t *)&cyttsp4_G700_ofilm_param_regs[0],
       .size = ARRAY_SIZE(cyttsp4_G700_ofilm_param_regs),
       .tag = 0,
};

#include <linux/Truely_G700_config.h>
static struct touch_settings cyttsp4_G700_sett_truly_param_regs = {
       .data = (uint8_t *)&cyttsp4_G700_truly_param_regs[0],
       .size = ARRAY_SIZE(cyttsp4_G700_truly_param_regs),
       .tag = 0,
};
struct cyttsp4_sett_param_map cyttsp4_G700_config_param_map[] = {
    
	[0] = {
			  .id = 0,
			  .param = &cyttsp4_G700_sett_ofilm_param_regs,
		  },
	
	[1] = {
			  .id = 2,
			  .param = &cyttsp4_G700_sett_truly_param_regs,
		  },
    [2] = {
			  .param = NULL,
		  },
		  
};
#include <linux/Eely_G610_config.h>
static struct touch_settings cyttsp4_G610_sett_eely_param_regs = {
       .data = (uint8_t *)&cyttsp4_G610_eely_param_regs[0],
       .size = ARRAY_SIZE(cyttsp4_G610_eely_param_regs),
       .tag = 0,
};
#include <linux/Truely_G610_config.h>
static struct touch_settings cyttsp4_G610_sett_truly_param_regs = {
       .data = (uint8_t *)&cyttsp4_G610_truly_param_regs[0],
       .size = ARRAY_SIZE(cyttsp4_G610_truly_param_regs),
       .tag = 0,
};
#include <linux/Ofilm_G610_config.h>
static struct touch_settings cyttsp4_G610_sett_ofilm_param_regs = {
       .data = (uint8_t *)&cyttsp4_G610_ofilm_param_regs[0],
       .size = ARRAY_SIZE(cyttsp4_G610_ofilm_param_regs),
       .tag = 0,
};

#include <linux/Mutto_G610_config.h>
static struct touch_settings cyttsp4_G610_sett_mutto_param_regs = {
       .data = (uint8_t *)&cyttsp4_G610_mutto_param_regs[0],
       .size = ARRAY_SIZE(cyttsp4_G610_mutto_param_regs),
       .tag = 0,
};

#include <linux/Gis_G610_config.h>
static struct touch_settings cyttsp4_G610_sett_gis_param_regs = {
       .data = (uint8_t *)&cyttsp4_G610_gis_param_regs[0],
       .size = ARRAY_SIZE(cyttsp4_G610_gis_param_regs),
       .tag = 0,
};
struct cyttsp4_sett_param_map cyttsp4_G610_config_param_map[] = {
    
	[0] = {
			  .id = 0,
			  .param = &cyttsp4_G610_sett_ofilm_param_regs,
		  },
	[1] = {
			  .id = 1,
			  .param = &cyttsp4_G610_sett_eely_param_regs,
		  },
	[2] = {
			  .id = 2,
			  .param = &cyttsp4_G610_sett_truly_param_regs,
		  },
	
	[3] = {
			  .id = 4,
			  .param = &cyttsp4_G610_sett_gis_param_regs,
		  },
	[4] = {
			   .id = 6,
			  .param = &cyttsp4_G610_sett_mutto_param_regs,
	        },
	[5] = {
			  .param = NULL,
	        },
		  
};
#else
static struct touch_settings cyttsp4_sett_param_regs = {
	.data = NULL,
	.size = 0,
	.tag = 0,
};

static struct touch_settings cyttsp4_sett_param_size = {
	.data = NULL,
	.size = 0,
	.tag = 0,
};
#endif
static struct cyttsp4_loader_platform_data _cyttsp4_G700_loader_platform_data = {
	.fw = &cyttsp4_firmware,
	.param_regs = &cyttsp4_sett_param_regs,
	.param_size = &cyttsp4_sett_param_size,
	.param_map =cyttsp4_G700_config_param_map,  
	.flags = 1,
};
static struct cyttsp4_loader_platform_data _cyttsp4_G610_loader_platform_data = {
	.fw = &cyttsp4_firmware,
	.param_regs = &cyttsp4_sett_param_regs,
	.param_size = &cyttsp4_sett_param_size,
	.param_map =cyttsp4_G610_config_param_map,  
	.flags = 1,
};
static struct cyttsp4_core_platform_data _cyttsp4_G610_core_platform_data = {
	.irq_gpio = CYTTSP4_I2C_IRQ_GPIO,
	.use_configure_sensitivity = 1,
	.xres = cyttsp4_xres,
	.init = cyttsp4_init,
	.power = cyttsp4_power,
	.sett = {
		NULL,	/* Reserved */
		NULL,	/* Command Registers */
		NULL,	/* Touch Report */
		NULL,	/* Cypress Data Record */
		NULL,	/* Test Record */
		NULL,	/* Panel Configuration Record */
		NULL, /* &cyttsp4_sett_param_regs, */
		NULL, /* &cyttsp4_sett_param_size, */
		NULL,	/* Reserved */
		NULL,	/* Reserved */
		NULL,	/* Operational Configuration Record */
		NULL, /* &cyttsp4_sett_ddata, *//* Design Data Record */
		NULL, /* &cyttsp4_sett_mdata, *//* Manufacturing Data Record */
		NULL,	/* Config and Test Registers */
		&cyttsp4_sett_btn_keys,	/* button-to-keycode table */
	},
	.loader_pdata = & _cyttsp4_G610_loader_platform_data,
};
static struct cyttsp4_core_platform_data _cyttsp4_G700_core_platform_data = {
	.irq_gpio = CYTTSP4_I2C_IRQ_GPIO,
	.use_configure_sensitivity = 1,
	.xres = cyttsp4_xres,
	.init = cyttsp4_init,
	.power = cyttsp4_power,
	.sett = {
		NULL,	/* Reserved */
		NULL,	/* Command Registers */
		NULL,	/* Touch Report */
		NULL,	/* Cypress Data Record */
		NULL,	/* Test Record */
		NULL,	/* Panel Configuration Record */
		NULL, /* &cyttsp4_sett_param_regs, */
		NULL, /* &cyttsp4_sett_param_size, */
		NULL,	/* Reserved */
		NULL,	/* Reserved */
		NULL,	/* Operational Configuration Record */
		NULL, /* &cyttsp4_sett_ddata, *//* Design Data Record */
		NULL, /* &cyttsp4_sett_mdata, *//* Manufacturing Data Record */
		NULL,	/* Config and Test Registers */
		&cyttsp4_sett_btn_keys,	/* button-to-keycode table */
	},
	.loader_pdata = &_cyttsp4_G700_loader_platform_data,
};

#define CY_MAXX 880
#define CY_MAXY 1280
#define CY_MINX 0
#define CY_MINY 0

#define CY_ABS_MIN_X CY_MINX
#define CY_ABS_MIN_Y CY_MINY
#define CY_ABS_MAX_X CY_MAXX
#define CY_ABS_MAX_Y CY_MAXY
#define CY_ABS_MIN_P 0
#define CY_ABS_MIN_W 0
#define CY_ABS_MAX_P 255
#define CY_ABS_MAX_W 255

#define CY_ABS_MIN_T 0

#define CY_ABS_MAX_T 15

#define CY_IGNORE_VALUE 0xFFFF

static const uint16_t cyttsp4_abs[] = {
	ABS_MT_POSITION_X, CY_ABS_MIN_X, CY_ABS_MAX_X, 0, 0,
	ABS_MT_POSITION_Y, CY_ABS_MIN_Y, CY_ABS_MAX_Y, 0, 0,
	ABS_MT_PRESSURE, CY_ABS_MIN_P, CY_ABS_MAX_P, 0, 0,
	CY_IGNORE_VALUE, CY_ABS_MIN_W, CY_ABS_MAX_W, 0, 0,
	ABS_MT_TRACKING_ID, CY_ABS_MIN_T, CY_ABS_MAX_T, 0, 0,
	ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0,
	ABS_MT_TOUCH_MINOR, 0, 255, 0, 0,
	ABS_MT_ORIENTATION, -128, 127, 0, 0,
};

struct touch_framework cyttsp4_framework = {
	.abs = (uint16_t *)&cyttsp4_abs[0],
	.size = ARRAY_SIZE(cyttsp4_abs),
	.enable_vkeys = 0,
};


/* struct cyttsp4_device cyttsp4_mt_device = { */
/* 	.name = CYTTSP4_MT_NAME, */
/* 	.core_id = "main_ttsp_core", */
/* 	.dev = { */
/* 		.platform_data = &_cyttsp4_mt_platform_data, */
/* 	} */
/* }; */

/* struct cyttsp4_device cyttsp4_btn_device = { */
/* 	.name = CYTTSP4_BTN_NAME, */
/* 	.core_id = "main_ttsp_core", */
/* 	.dev = { */
/* 			.platform_data = &_cyttsp4_btn_platform_data, */
/* 	} */
/* }; */

#endif /* CONFIG_CYPRESS_TTSP */


static struct i2c_board_info mtk_ttsp_i2c_tpd=
{ // KEVKEV
		I2C_BOARD_INFO(CYTTSP4_I2C_NAME, CYTTSP4_I2C_TCH_ADR),
		//.irq =  MSM_GPIO_TO_INT(CYTTSP4_I2C_IRQ_GPIO),
		//#define GIC_PRIVATE_SIGNALS     (32)
		//#define MT_EINT_IRQ_ID                      (GIC_PRIVATE_SIGNALS + 116)
		.irq =  -1,
		.platform_data = CYTTSP4_I2C_NAME,
};

static struct cyttsp4_core_info cyttsp4_G700_core_info = {
	.name = CYTTSP4_CORE_NAME,
	.id = "main_ttsp_core",
	.adap_id = CYTTSP4_I2C_NAME,
	.platform_data = &_cyttsp4_G700_core_platform_data,
};
static struct cyttsp4_core_info cyttsp4_G610_core_info  = {
	.name = CYTTSP4_CORE_NAME,
	.id = "main_ttsp_core",
	.adap_id = CYTTSP4_I2C_NAME,
	.platform_data = &_cyttsp4_G610_core_platform_data,
};

static struct cyttsp4_mt_platform_data _cyttsp4_mt_virtualkey_platform_data = {
	.frmwrk = &cyttsp4_framework,
	.flags = 0x40,
	.inp_dev_name = CYTTSP4_MT_NAME,
};

struct cyttsp4_device_info cyttsp4_mt_virtualkey_info  = {
	.name = CYTTSP4_MT_NAME,
	.core_id = "main_ttsp_core",
	.platform_data = &_cyttsp4_mt_virtualkey_platform_data,
};

static struct cyttsp4_mt_platform_data _cyttsp4_mt_novirtualkey_platform_data = {
	.frmwrk = &cyttsp4_framework,
	.flags = 0x00,
	.inp_dev_name = CYTTSP4_MT_NAME,
};

struct cyttsp4_device_info cyttsp4_mt_novirtualkey_info  = {
	.name = CYTTSP4_MT_NAME,
	.core_id = "main_ttsp_core",
	.platform_data = &_cyttsp4_mt_novirtualkey_platform_data,
};

static struct cyttsp4_btn_platform_data _cyttsp4_btn_platform_data = {
	.inp_dev_name = CYTTSP4_BTN_NAME,
};

struct cyttsp4_device_info cyttsp4_btn_info = {
	.name = CYTTSP4_BTN_NAME,
	.core_id = "main_ttsp_core",
	.platform_data = &_cyttsp4_btn_platform_data,
};

//static struct cyttsp4_core cyttsp4_core_device = {
//	.name = CYTTSP4_CORE_NAME,
//	.id = "main_ttsp_core",
//	.adap_id = CYTTSP4_I2C_NAME,
//	.dev = {
//		.platform_data = &_cyttsp4_core_platform_data,
//	},
//};

static int __init tpd_ttsp_init(void) {
  printk("MediaTek TTDA ttsp touch panel driver init\n");
  i2c_register_board_info(0, &mtk_ttsp_i2c_tpd, 1);
  return 0;
}
module_init(tpd_ttsp_init);
