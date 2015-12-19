#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_mag.h>
#include <linux/hardware_self_adapt.h>

static struct mag_hw cust_mag_hw = {
    .i2c_num = 3,
    .direction = 7,
    .power_id = MT65XX_POWER_NONE,  /*!< LDO is not used */
    .power_vol= VOL_DEFAULT,        /*!< LDO is not used */
};

/*modify compass drection*/
struct mag_hw* yamaha533_auto_get_cust_mag_hw(void)
{
    hw_product_type boardType = get_hardware_product_version();
    if((boardType & HW_VER_MAIN_MASK) == HW_G700U_VER)
    {
		cust_mag_hw.direction=7;
	}
    return &cust_mag_hw;
}

