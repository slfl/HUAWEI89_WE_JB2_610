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
/*****************************************************************************
 *
 * Filename:
 * ---------
 *   sensor.c
 *
 * Project:
 * --------
 *   DUMA
 *
 * Description:
 * ------------
 *   Source code of Sensor driver
 *
 *
 * Author:
 * -------
 *   PC Huang (MTK02204)
 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Revision:$
 * $Modtime:$
 * $Log:$
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <asm/system.h>
#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"
#include "mt9v113mipiyuv_Sensor.h"
#include "mt9v113mipiyuv_Camera_Sensor_para.h"
#include "mt9v113mipiyuv_CameraCustomized.h"

#define MT9V113MIPI_DEBUG
#ifdef MT9V113MIPI_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif
UINT16 WBcount = AWB_MODE_AUTO;

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);



inline MT9V113MIPI_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
    char puSendCmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	iReadRegI2C(puSendCmd , 2, (u8*)&get_byte,2,MT9V113MIPI_WRITE_ID);
    return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);
}

inline int MT9V113MIPI_write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char puSendCmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
	iWriteRegI2C(puSendCmd , 4,MT9V113MIPI_WRITE_ID);
    return 0;
}



/*******************************************************************************
* // Adapter for Winmo typedef 
********************************************************************************/
#define WINMO_USE 0
#define Sleep(ms) mdelay(ms)
#define RETAILMSG(x,...)

struct MT9V113MIPI_sensor_struct MT9V113MIPI_Sensor_Driver;
MSDK_SENSOR_CONFIG_STRUCT MT9V113MIPISensorConfigData;



void sequencer_refresh(void)
{
	MT9V113MIPI_write_cmos_sensor( 0x098C, 0xA103 );   
	MT9V113MIPI_write_cmos_sensor( 0x0990, 0x06 );    
	Sleep(25);
	MT9V113MIPI_write_cmos_sensor( 0x098C, 0xA103 );   
	MT9V113MIPI_write_cmos_sensor( 0x0990, 0x05 );    
	Sleep(25);
}


void MT9V113MIPI_set_dummy(kal_uint16 pixels, kal_uint16 lines)
{
		kal_uint32 min_shutter_step = 0;	
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x271F);
		MT9V113MIPI_write_cmos_sensor(0x0990, MT9V113MIPI_Sensor_Driver.Preview_Lines_In_Frame);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2721);
		MT9V113MIPI_write_cmos_sensor(0x0990, MT9V113MIPI_Sensor_Driver.Preview_Pixels_In_Line);
		min_shutter_step = ((FACTOR_60HZ / MT9V113MIPI_Sensor_Driver.Preview_Pixels_In_Line) + 1);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2411);	//60Hz
		MT9V113MIPI_write_cmos_sensor(0x0990, min_shutter_step);
		min_shutter_step = ((FACTOR_50HZ / MT9V113MIPI_Sensor_Driver.Preview_Pixels_In_Line) + 1);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2413);	//50Hz
		MT9V113MIPI_write_cmos_sensor(0x0990, min_shutter_step);	
		sequencer_refresh();
}



void MT9V113MIPIFixFrameRate(kal_uint32 Min,kal_uint32 Max)
{
	kal_uint16 Min_Exposure;
	kal_uint32 Max_Exposure;
	if(MT9V113MIPI_Sensor_Driver.Banding == AE_FLICKER_MODE_50HZ){
		Min_Exposure = (MACRO_50HZ * 10)/Min;
	}else{
		Min_Exposure = (MACRO_60HZ * 10)/Min;
	}	
	MT9V113MIPI_write_cmos_sensor(0x098C, 0xA20C); 
	MT9V113MIPI_write_cmos_sensor(0x0990, Min_Exposure); 
	Max_Exposure = (kal_uint32)(((MT9V113MIPI_Sensor_Driver.Preview_PClk* 1000*1000*10)/(2*Max))/MT9V113MIPI_Sensor_Driver.Preview_Pixels_In_Line); 			
	if(Max_Exposure>MT9V113MIPI_Sensor_Driver.Preview_Lines_In_Frame)
		MT9V113MIPI_Sensor_Driver.Preview_Lines_In_Frame = Max_Exposure;
	MT9V113MIPI_set_dummy(MT9V113MIPI_Sensor_Driver.Preview_Pixels_In_Line,MT9V113MIPI_Sensor_Driver.Preview_Lines_In_Frame);
	MT9V113MIPI_write_cmos_sensor(0x098C, 0x2721);
	MT9V113MIPI_write_cmos_sensor(0x098C, 0x271F);		
}



/*************************************************************************
* FUNCTION
*	MT9V113MIPI_NightMode
*
* DESCRIPTION
*	This function night mode of MT9V113MIPI.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void MT9V113MIPI_night_mode(kal_bool enable)
{
		if(enable)
	{
        MT9V113MIPI_Sensor_Driver.Min_Frame_Rate = 50;
		MT9V113MIPI_Sensor_Driver.Max_Frame_Rate = 300;
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA20C); 		
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0018);	 	
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2212);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x00D0);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA103); 		
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0006); 		
		Sleep(15);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA103); 		
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0005); 		
		Sleep(10);                           
	}
	else
    {
		MT9V113MIPI_Sensor_Driver.Min_Frame_Rate = 75;
		MT9V113MIPI_Sensor_Driver.Max_Frame_Rate = 300;
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA20C);		
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0014);		
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2212);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x00BC);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA103);		
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0006);		
		Sleep(15);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA103);		
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0005);		
		Sleep(10);
	}
	
	MT9V113MIPIFixFrameRate(MT9V113MIPI_Sensor_Driver.Min_Frame_Rate,MT9V113MIPI_Sensor_Driver.Max_Frame_Rate);

}	


static void MT9V113MIPI_YUV_sensor_initial_setting(void)
{
    int value,count=5;
    printk("MT9V113MIPI_Initial_Setting!!!\n");
    
		MT9V113MIPI_write_cmos_sensor(0x001A, 0x0011);
		MT9V113MIPI_write_cmos_sensor(0x001A, 0x0018);
		MT9V113MIPI_write_cmos_sensor(0x0014, 0x2145);
		MT9V113MIPI_write_cmos_sensor(0x0014, 0x2145);
		MT9V113MIPI_write_cmos_sensor(0x0010, 0x0631);
		MT9V113MIPI_write_cmos_sensor(0x0012, 0x0000);
		MT9V113MIPI_write_cmos_sensor(0x0014, 0x244B);
		Sleep(10);
		MT9V113MIPI_write_cmos_sensor(0x0014, 0x304B);
    #if 1
    while(!(((value = MT9V113MIPI_read_cmos_sensor(0x0014))>> 15) & 0x1))
    {
        value=MT9V113MIPI_read_cmos_sensor(0x0014);
        printk("l00183577 : MT9V113MIPI_read_cmos_sensor(0x0014):%d, count:%d\n",value,count);
        if ((count --)==0)
            break;
        Sleep(10);
    }//POLL 0x0014[15]---> 1
    count=5;
     #endif
		MT9V113MIPI_write_cmos_sensor(0x0014, 0xB04A);
		MT9V113MIPI_write_cmos_sensor(0x0018, 0x402C);
		Sleep(10);
		MT9V113MIPI_write_cmos_sensor(0x3400, 0x7A38);
		MT9V113MIPI_write_cmos_sensor(0x321C, 0x0003);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x02F0);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0000);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x02F2);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0210);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x02F4);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x001A);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2145);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x02F4);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA134);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0001);
		MT9V113MIPI_write_cmos_sensor(0x31E0, 0x0001);
	
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2703);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0280);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2705);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x01E0);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2707);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0280);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2709);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x01E0);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x270D);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0000);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x270F);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0000);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2711);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x01E7);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2713);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0287);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2715);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0001);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2717);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0025);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2719);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x001A);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x271B);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x006B);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x271D);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x006B);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x271F);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0206);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2721);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0364);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2723);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0000);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2725);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0000);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2727);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x01E7);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2729);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0287);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x272B);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0001);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x272D);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0025);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x272F);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x001A);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2731);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x006B);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2733);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x006B);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2735);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0426);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2737);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0363);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2739);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0000);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x273B);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x027F);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x273D);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0000);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x273F);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x01DF);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2747);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0000);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2749);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x027F);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x274B);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0000);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x274D);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x01DF);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x222D);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0088);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA408);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0020);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA409);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0023);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA40A);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0027);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA40B);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x002A);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2411);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0088);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2413);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x00A4);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2415);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0088);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2417);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x00A4);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA404);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0010);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA40D);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0002);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA40E);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0003);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA410);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x000A);
		MT9V113MIPI_write_cmos_sensor(0x364E, 0x0350);
		MT9V113MIPI_write_cmos_sensor(0x3650, 0x22ED);
		MT9V113MIPI_write_cmos_sensor(0x3652, 0x0513);
		MT9V113MIPI_write_cmos_sensor(0x3654, 0x6C70);
		MT9V113MIPI_write_cmos_sensor(0x3656, 0x5015);
		MT9V113MIPI_write_cmos_sensor(0x3658, 0x0130);
		MT9V113MIPI_write_cmos_sensor(0x365A, 0x444D);
		MT9V113MIPI_write_cmos_sensor(0x365C, 0x18D3);
		MT9V113MIPI_write_cmos_sensor(0x365E, 0x5FB1);
		MT9V113MIPI_write_cmos_sensor(0x3660, 0x6415);
		MT9V113MIPI_write_cmos_sensor(0x3662, 0x00D0);
		MT9V113MIPI_write_cmos_sensor(0x3664, 0x014C);
		MT9V113MIPI_write_cmos_sensor(0x3666, 0x7BB2);
		MT9V113MIPI_write_cmos_sensor(0x3668, 0x31B1);
		MT9V113MIPI_write_cmos_sensor(0x366A, 0x46D5);
		MT9V113MIPI_write_cmos_sensor(0x366C, 0x0130);
		MT9V113MIPI_write_cmos_sensor(0x366E, 0x338D);
		MT9V113MIPI_write_cmos_sensor(0x3670, 0x0593);
		MT9V113MIPI_write_cmos_sensor(0x3672, 0x13D1);
		MT9V113MIPI_write_cmos_sensor(0x3674, 0x4875);
		MT9V113MIPI_write_cmos_sensor(0x3676, 0x992E);
		MT9V113MIPI_write_cmos_sensor(0x3678, 0x910E);
		MT9V113MIPI_write_cmos_sensor(0x367A, 0xAF92);
		MT9V113MIPI_write_cmos_sensor(0x367C, 0x1732);
		MT9V113MIPI_write_cmos_sensor(0x367E, 0x7BD3);
		MT9V113MIPI_write_cmos_sensor(0x3680, 0x98EE);
		MT9V113MIPI_write_cmos_sensor(0x3682, 0xEF4D);
		MT9V113MIPI_write_cmos_sensor(0x3684, 0x8872);
		MT9V113MIPI_write_cmos_sensor(0x3686, 0x0352);
		MT9V113MIPI_write_cmos_sensor(0x3688, 0x2792);
		MT9V113MIPI_write_cmos_sensor(0x368A, 0xDB6D);
		MT9V113MIPI_write_cmos_sensor(0x368C, 0xF52D);
		MT9V113MIPI_write_cmos_sensor(0x368E, 0xA532);
		MT9V113MIPI_write_cmos_sensor(0x3690, 0x0213);
		MT9V113MIPI_write_cmos_sensor(0x3692, 0x10D5);
		MT9V113MIPI_write_cmos_sensor(0x3694, 0x8BCE);
		MT9V113MIPI_write_cmos_sensor(0x3696, 0xFC2D);
		MT9V113MIPI_write_cmos_sensor(0x3698, 0xA532);
		MT9V113MIPI_write_cmos_sensor(0x369A, 0x67F1);
		MT9V113MIPI_write_cmos_sensor(0x369C, 0x1034);
		MT9V113MIPI_write_cmos_sensor(0x369E, 0x1113);
		MT9V113MIPI_write_cmos_sensor(0x36A0, 0x2EF3);
		MT9V113MIPI_write_cmos_sensor(0x36A2, 0x39F7);
		MT9V113MIPI_write_cmos_sensor(0x36A4, 0xB097);
		MT9V113MIPI_write_cmos_sensor(0x36A6, 0x81BA);
		MT9V113MIPI_write_cmos_sensor(0x36A8, 0x2CF3);
		MT9V113MIPI_write_cmos_sensor(0x36AA, 0x1373);
		MT9V113MIPI_write_cmos_sensor(0x36AC, 0x4457);
		MT9V113MIPI_write_cmos_sensor(0x36AE, 0xFAF6);
		MT9V113MIPI_write_cmos_sensor(0x36B0, 0xEC19);
		MT9V113MIPI_write_cmos_sensor(0x36B2, 0x0E73);
		MT9V113MIPI_write_cmos_sensor(0x36B4, 0x0873);
		MT9V113MIPI_write_cmos_sensor(0x36B6, 0x34F7);
		MT9V113MIPI_write_cmos_sensor(0x36B8, 0x9EB7);
		MT9V113MIPI_write_cmos_sensor(0x36BA, 0x9B9A);
		MT9V113MIPI_write_cmos_sensor(0x36BC, 0x0E33);
		MT9V113MIPI_write_cmos_sensor(0x36BE, 0x2013);
		MT9V113MIPI_write_cmos_sensor(0x36C0, 0x3C37);
		MT9V113MIPI_write_cmos_sensor(0x36C2, 0xA0D7);
		MT9V113MIPI_write_cmos_sensor(0x36C4, 0x935A);
		MT9V113MIPI_write_cmos_sensor(0x36C6, 0xCD11);
		MT9V113MIPI_write_cmos_sensor(0x36C8, 0x0353);
		MT9V113MIPI_write_cmos_sensor(0x36CA, 0x2516);
		MT9V113MIPI_write_cmos_sensor(0x36CC, 0x8437);
		MT9V113MIPI_write_cmos_sensor(0x36CE, 0xA01A);
		MT9V113MIPI_write_cmos_sensor(0x36D0, 0xFCF1);
		MT9V113MIPI_write_cmos_sensor(0x36D2, 0xAD91);
		MT9V113MIPI_write_cmos_sensor(0x36D4, 0x29D4);
		MT9V113MIPI_write_cmos_sensor(0x36D6, 0x0AB6);
		MT9V113MIPI_write_cmos_sensor(0x36D8, 0x9436);
		MT9V113MIPI_write_cmos_sensor(0x36DA, 0xA872);
		MT9V113MIPI_write_cmos_sensor(0x36DC, 0xD00F);
		MT9V113MIPI_write_cmos_sensor(0x36DE, 0x2B36);
		MT9V113MIPI_write_cmos_sensor(0x36E0, 0x0714);
		MT9V113MIPI_write_cmos_sensor(0x36E2, 0xFEB9);
		MT9V113MIPI_write_cmos_sensor(0x36E4, 0xD211);
		MT9V113MIPI_write_cmos_sensor(0x36E6, 0x22F1);
		MT9V113MIPI_write_cmos_sensor(0x36E8, 0x5BD5);
		MT9V113MIPI_write_cmos_sensor(0x36EA, 0x98D3);
		MT9V113MIPI_write_cmos_sensor(0x36EC, 0xF4D9);
		MT9V113MIPI_write_cmos_sensor(0x36EE, 0x78B5);
		MT9V113MIPI_write_cmos_sensor(0x36F0, 0xAA57);
		MT9V113MIPI_write_cmos_sensor(0x36F2, 0xF65A);
		MT9V113MIPI_write_cmos_sensor(0x36F4, 0x52DB);
		MT9V113MIPI_write_cmos_sensor(0x36F6, 0x4CDE);
		MT9V113MIPI_write_cmos_sensor(0x36F8, 0x7475);
		MT9V113MIPI_write_cmos_sensor(0x36FA, 0xD936);
		MT9V113MIPI_write_cmos_sensor(0x36FC, 0xEDFA);
		MT9V113MIPI_write_cmos_sensor(0x36FE, 0x7DDA);
		MT9V113MIPI_write_cmos_sensor(0x3700, 0x46DE);
		MT9V113MIPI_write_cmos_sensor(0x3702, 0x4775);
		MT9V113MIPI_write_cmos_sensor(0x3704, 0xE5F6);
		MT9V113MIPI_write_cmos_sensor(0x3706, 0xF9DA);
		MT9V113MIPI_write_cmos_sensor(0x3708, 0x281B);
		MT9V113MIPI_write_cmos_sensor(0x370A, 0x4C1E);
		MT9V113MIPI_write_cmos_sensor(0x370C, 0x0396);
		MT9V113MIPI_write_cmos_sensor(0x370E, 0xF016);
		MT9V113MIPI_write_cmos_sensor(0x3710, 0x85DB);
		MT9V113MIPI_write_cmos_sensor(0x3712, 0x239B);
		MT9V113MIPI_write_cmos_sensor(0x3714, 0x6B9E);
		MT9V113MIPI_write_cmos_sensor(0x3644, 0x0154);
		MT9V113MIPI_write_cmos_sensor(0x3642, 0x00DC);
		MT9V113MIPI_write_cmos_sensor(0x3210, 0x09B8);
		MT9V113MIPI_write_cmos_sensor(0x0018, 0x0028);
		Sleep(10);
		//20121029 modify pixel format
		//9V113MIPI_write_cmos_sensor(0x098C, 0x2755);
		//9V113MIPI_write_cmos_sensor(0x0990, 0x0001);
		
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA20C);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0010);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA24F);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0038);									
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2306);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0616);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x231C);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x00B0);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2308);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0xFAEB);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x231E);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0xFF57);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x230A);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x005A);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2320);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0014);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x230C);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0xFE50);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2322);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0066);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x230E);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0503);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2324);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0008);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2310);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0xFE31);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2326);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0xFFAD);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2312);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0xFE37);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2328);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0130);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2314);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0xFBD4);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x232A);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x01BA);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2316);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0766);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x232C);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0xFD2D);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2318);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x001C);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x231A);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0039);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x232E);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0001);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2330);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0xFFEF);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA366);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0080);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA367);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0080);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA368);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0080);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA369);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0080);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA36A);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0080);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA36B);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0080);	
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA348);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0008);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA349);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0002);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA34A);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0090);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA34B);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x00FF);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA34C);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0075);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA34D);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x00EF);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA351);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0000);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA352);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x007F);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA354);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0043);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA355);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0001);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA35D);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0078);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA35E);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0086);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA35F);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x007E);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA360);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0082);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2361);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0040);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA363);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x00D2);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA364);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x00F6);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA302);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0000);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA303);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x00EF);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x274F);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0004);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2741);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0004);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xAB1F);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x00C7);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xAB31);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x001E);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xAB20);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0024);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xAB21);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0046);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xAB22);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0002);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xAB24);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0000);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2B28);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x170C);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2B2A);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x3E80);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA103);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0006);
#if 1
    while(value=MT9V113MIPI_read_cmos_sensor(0xA103))
    {
        value= MT9V113MIPI_read_cmos_sensor(0xA103);
        SENSORDB("[MT9V113MIPI]: MT9V113MIPI_read_cmos_sensor(0xA103):%d, count:%d\n",value,count);
        if ((count --)==0)
            break;
        Sleep(10);
    }//POLL 0xA103 ---> 0
    count =5;
#endif
		Sleep(100);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0xA103);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0005);
 #if 1
    while(value =MT9V113MIPI_read_cmos_sensor(0xA103))
    {
        value=MT9V113MIPI_read_cmos_sensor(0xA103);
        SENSORDB("[MT9V113MIPI] : MT9V113MIPI_read_cmos_sensor(0xA103)  final,:%d, count:%d\n",value,count);
        if ((count --)==0)
            break;
    }//POLL 0xA103 ---> 0
        count =5;
#endif
    Sleep(300);//DELAY=300
}



void MT9V113MIPI_Init_Para(void)
{
	SENSORDB("[Enter]:MT9V113_Init_Para\n");		
	WBcount = AWB_MODE_AUTO;
	MT9V113MIPI_Sensor_Driver.Preview_PClk = 24;// 12Mhz
	MT9V113MIPI_Sensor_Driver.Dummy_Pixels = 0;
	MT9V113MIPI_Sensor_Driver.Dummy_Lines= 0;
	MT9V113MIPI_Sensor_Driver.Min_Frame_Rate = 75;
	MT9V113MIPI_Sensor_Driver.Max_Frame_Rate = 300;
	MT9V113MIPI_Sensor_Driver.Preview_Pixels_In_Line = MT9V113MIPI_DEFUALT_PREVIEW_LINE_LENGTH + MT9V113MIPI_Sensor_Driver.Dummy_Pixels;
	MT9V113MIPI_Sensor_Driver.Preview_Lines_In_Frame = MT9V113MIPI_DEFUALT_PREVIEW_FRAME_LENGTH + MT9V113MIPI_Sensor_Driver.Dummy_Lines;
}



static kal_uint16 MT9V113MIPI_power_on(void)
{
	MT9V113MIPI_Sensor_Driver.sensor_id = 0;
	MT9V113MIPI_Sensor_Driver.sensor_id = MT9V113MIPI_read_cmos_sensor(0x0000);

   	
	SENSORDB("MT9V113MIPI_Sensor_id =%x\n",MT9V113MIPI_Sensor_Driver.sensor_id);
	return MT9V113MIPI_Sensor_Driver.sensor_id;
}




/*************************************************************************
* FUNCTION
*	MT9V113MIPIOpen
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 MT9V113MIPIOpen(void)
{
	 if (MT9V113MIPI_power_on() != MT9V113_SENSOR_ID) 
	 	{
	 	   SENSORDB("[MT9V113]Error:read sensor ID fail\n");
		   return ERROR_SENSOR_CONNECT_FAIL;
	 	}
     MT9V113MIPI_YUV_sensor_initial_setting();
     MT9V113MIPI_Init_Para();
	return ERROR_NONE;
}	



/*************************************************************************
* FUNCTION
*	MT9V113MIPI_GetSensorID
*
* DESCRIPTION
*	This function get the sensor ID
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 MT9V113MIPI_GetSensorID(kal_uint32 *sensorID)
{
     SENSORDB("[Enter]:MT9V113 MT9V113MIPIGetSensorID func zhijie: \n");

	 *sensorID = MT9V113MIPI_power_on();
	 if (*sensorID != MT9V113_SENSOR_ID) 
	 	{
	 	   SENSORDB("[MT9V113MIPI]Error:read sensor ID fail\n");
		   *sensorID = 0xFFFFFFFF;
		   return ERROR_SENSOR_CONNECT_FAIL;
	 	}
	return ERROR_NONE;
}  


/*************************************************************************
* FUNCTION
*	MT9V113MIPIClose
*
* DESCRIPTION
*	This function is to turn off sensor module power.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 MT9V113MIPIClose(void)
{
	return ERROR_NONE;
}



static void MT9V113MIPI_HVMirror(kal_uint8 image_mirror)
{
	switch (image_mirror)
	{
	case IMAGE_NORMAL:
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2717);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0024);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x272D);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0024);
		break;
	case IMAGE_H_MIRROR:
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2717);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0025);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x272D);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0025);
		break;
	case IMAGE_V_MIRROR:
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2717);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0026);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x272D);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0026);
		break;
	case IMAGE_HV_MIRROR:
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2717);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0027);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x272D);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0027);
		break;
	default:
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x2717);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0024);
		MT9V113MIPI_write_cmos_sensor(0x098C, 0x272D);
		MT9V113MIPI_write_cmos_sensor(0x0990, 0x0024);
		break;
	}
	MT9V113MIPI_write_cmos_sensor(0x098C, 0xA103);
   	MT9V113MIPI_write_cmos_sensor(0x0990, 0x0006);
}



/*************************************************************************
* FUNCTION
*	MT9V113MIPIPreview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 MT9V113MIPIPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)

{	
	SENSORDB("MT9V113MIPIPreview\n");
    MT9V113MIPI_Sensor_Driver.Camco_mode = MT9V113MIPI_CAM_PREVIEW;
	MT9V113MIPI_Sensor_Driver.Preview_PClk = 24;// 12Mhz
	MT9V113MIPI_Sensor_Driver.Dummy_Pixels = 0;
	MT9V113MIPI_Sensor_Driver.Dummy_Lines= 0;
	MT9V113MIPI_Sensor_Driver.Min_Frame_Rate = 75;
	MT9V113MIPI_Sensor_Driver.Max_Frame_Rate = 300;
	MT9V113MIPI_Sensor_Driver.StartX=1;
	MT9V113MIPI_Sensor_Driver.StartY=1;
	MT9V113MIPI_Sensor_Driver.iGrabWidth = MT9V113MIPI_IMAGE_SENSOR_VGA_WIDTH - 16;//16;
	MT9V113MIPI_Sensor_Driver.iGrabheight = MT9V113MIPI_IMAGE_SENSOR_VGA_HEIGHT - 12;//16;
	//MT9V113MIPI_HVMirror(sensor_config_data->SensorImageMirror);
	MT9V113MIPI_Sensor_Driver.Preview_Pixels_In_Line = MT9V113MIPI_DEFUALT_PREVIEW_LINE_LENGTH + MT9V113MIPI_Sensor_Driver.Dummy_Pixels;
	MT9V113MIPI_Sensor_Driver.Preview_Lines_In_Frame = MT9V113MIPI_DEFUALT_PREVIEW_FRAME_LENGTH + MT9V113MIPI_Sensor_Driver.Dummy_Lines;
	MT9V113MIPI_set_dummy(MT9V113MIPI_Sensor_Driver.Preview_Lines_In_Frame, MT9V113MIPI_Sensor_Driver.Preview_Pixels_In_Line);
	image_window->GrabStartX = MT9V113MIPI_Sensor_Driver.StartX;
	image_window->GrabStartY = MT9V113MIPI_Sensor_Driver.StartY;
	image_window->ExposureWindowWidth = MT9V113MIPI_Sensor_Driver.iGrabWidth;
	image_window->ExposureWindowHeight = MT9V113MIPI_Sensor_Driver.iGrabheight;
    return ERROR_NONE; 
}



UINT32 MT9V113MIPICapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	SENSORDB("MT9V113MIPICapture\n");
	MT9V113MIPI_Sensor_Driver.Camco_mode = MT9V113MIPI_CAM_CAPTURE;
	MT9V113MIPI_Sensor_Driver.StartX=1;
	MT9V113MIPI_Sensor_Driver.StartY=1;
	MT9V113MIPI_Sensor_Driver.iGrabWidth=MT9V113MIPI_IMAGE_SENSOR_VGA_WIDTH - 16;
	MT9V113MIPI_Sensor_Driver.iGrabheight=MT9V113MIPI_IMAGE_SENSOR_VGA_HEIGHT - 12;
	image_window->GrabStartX = MT9V113MIPI_Sensor_Driver.StartX;
	image_window->GrabStartY = MT9V113MIPI_Sensor_Driver.StartY;
	image_window->ExposureWindowWidth = MT9V113MIPI_Sensor_Driver.iGrabWidth;
	image_window->ExposureWindowHeight = MT9V113MIPI_Sensor_Driver.iGrabheight;
	return ERROR_NONE;
}



UINT32 MT9V113MIPIGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
	SENSORDB("MT9V113MIPIGetResolution\n");
	pSensorResolution->SensorFullWidth=MT9V113MIPI_IMAGE_SENSOR_VGA_WIDTH - 16;  
	pSensorResolution->SensorFullHeight=MT9V113MIPI_IMAGE_SENSOR_VGA_HEIGHT - 12;
	pSensorResolution->SensorPreviewWidth=MT9V113MIPI_IMAGE_SENSOR_VGA_WIDTH - 16;
	pSensorResolution->SensorPreviewHeight=MT9V113MIPI_IMAGE_SENSOR_VGA_HEIGHT - 12;
	pSensorResolution->SensorVideoWidth=MT9V113MIPI_IMAGE_SENSOR_VGA_WIDTH - 16;
	pSensorResolution->SensorVideoHeight=MT9V113MIPI_IMAGE_SENSOR_VGA_HEIGHT - 12;
	return ERROR_NONE;
}



UINT32 MT9V113MIPIGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
					  MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{ 
		SENSORDB("MT9V113MIPIGetInfo\n");
		pSensorInfo->SensorPreviewResolutionX=MT9V113MIPI_IMAGE_SENSOR_VGA_WIDTH;
		pSensorInfo->SensorPreviewResolutionY=MT9V113MIPI_IMAGE_SENSOR_VGA_HEIGHT;
		pSensorInfo->SensorFullResolutionX=MT9V113MIPI_IMAGE_SENSOR_VGA_WIDTH;
		pSensorInfo->SensorFullResolutionY=MT9V113MIPI_IMAGE_SENSOR_VGA_HEIGHT;
		pSensorInfo->SensorCameraPreviewFrameRate=30;
		pSensorInfo->SensorVideoFrameRate=30;
		pSensorInfo->SensorStillCaptureFrameRate=30;
		pSensorInfo->SensorWebCamCaptureFrameRate=15;
		pSensorInfo->SensorResetActiveHigh=FALSE; 
		pSensorInfo->SensorResetDelayCount=4;
		pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_UYVY;
		pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW; 
		pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
		pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
		pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_HIGH;
		pSensorInfo->SensorInterruptDelayLines = 1; 
		pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;
		pSensorInfo->CaptureDelayFrame = 1; 
		pSensorInfo->PreviewDelayFrame = 2; 
		pSensorInfo->VideoDelayFrame = 0; 
		pSensorInfo->SensorMasterClockSwitch = 0; 
		pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA; 		
		switch (ScenarioId)
		{
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				pSensorInfo->SensorClockFreq=26;
				pSensorInfo->SensorClockDividCount= 7;
				pSensorInfo->SensorClockRisingCount= 0;
				pSensorInfo->SensorClockFallingCount= 4;
				pSensorInfo->SensorPixelClockCount= 3;
				pSensorInfo->SensorDataLatchCount= 2;
				pSensorInfo->SensorGrabStartX = 4; 
				pSensorInfo->SensorGrabStartY = 2;
				//add mipi interface setting
				pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;			
	            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
		        pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
		        pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
	            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
	            pSensorInfo->SensorPacketECCOrder = 1;
			break;
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				pSensorInfo->SensorClockFreq=26;
				pSensorInfo->SensorClockDividCount= 7;
				pSensorInfo->SensorClockRisingCount= 0;
				pSensorInfo->SensorClockFallingCount= 4;
				pSensorInfo->SensorPixelClockCount= 3;
				pSensorInfo->SensorDataLatchCount= 2;
				pSensorInfo->SensorGrabStartX = 4; 
				pSensorInfo->SensorGrabStartY = 2;	
				//add mipi interface setting
				pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;			
	            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
		        pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
		        pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
	            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
	            pSensorInfo->SensorPacketECCOrder = 1;
			break;
			default:
				pSensorInfo->SensorClockFreq=26;
				pSensorInfo->SensorClockDividCount=7;
				pSensorInfo->SensorClockRisingCount=0;
				pSensorInfo->SensorClockFallingCount=4;
				pSensorInfo->SensorPixelClockCount=3;
				pSensorInfo->SensorDataLatchCount=2;
				pSensorInfo->SensorGrabStartX = 4; 
				pSensorInfo->SensorGrabStartY = 2;
				//add mipi interface setting
				pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;			
	            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
		        pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
		        pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
	            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
	            pSensorInfo->SensorPacketECCOrder = 1;
			break;
		}
		memcpy(pSensorConfigData, &MT9V113MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));	
		return ERROR_NONE;
}	/* MT9V113MIPIGetInfo() */


UINT32 MT9V113MIPIControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	 switch (ScenarioId)
	 {
		 case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		 case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			  MT9V113MIPIPreview(pImageWindow, pSensorConfigData);
			  break;
		 case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			  MT9V113MIPICapture(pImageWindow, pSensorConfigData);
		 default:
			  break; 
	 }
	 return ERROR_NONE;
}



/*************************************************************************
* FUNCTION
*	MT9V113MIPI_set_param_wb
*
* DESCRIPTION
*	wb setting.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
BOOL MT9V113MIPI_set_param_wb(UINT16 para)
{
	SENSORDB("MT9V113MIPI_set_param_wb = %d\n",para);
	if(WBcount == para)
		return TRUE;
	  switch (para)
	  { 		   
		  case AWB_MODE_AUTO:
			  {
				  MT9V113MIPI_write_cmos_sensor( 0x098C, 0xA11F); 
				  MT9V113MIPI_write_cmos_sensor( 0x0990, 0x0001); 
				  MT9V113MIPI_write_cmos_sensor( 0x098C, 0xA103); 
				  MT9V113MIPI_write_cmos_sensor( 0x0990, 0x0006);  		  
			  } 			   
			  break;
		  case AWB_MODE_CLOUDY_DAYLIGHT:
			  {
				  MT9V113MIPI_write_cmos_sensor(  0x098C, 0xA11F);	  
				  MT9V113MIPI_write_cmos_sensor(  0x0990, 0x0000);	 
				  MT9V113MIPI_write_cmos_sensor(  0x098C, 0xA103);	  
				  MT9V113MIPI_write_cmos_sensor(  0x0990, 0x0006);	  
				  Sleep(25);
				  MT9V113MIPI_write_cmos_sensor(  0x098C, 0xA353);	  
				  MT9V113MIPI_write_cmos_sensor(  0x0990, 0x007F);	  
				  MT9V113MIPI_write_cmos_sensor(  0x098C, 0xA34E);	 
				  MT9V113MIPI_write_cmos_sensor(  0x0990, 0x00A6);	  
				  MT9V113MIPI_write_cmos_sensor(  0x098C, 0xA350);	 
				  MT9V113MIPI_write_cmos_sensor(  0x0990, 0x007F);	  
				  MT9V113MIPI_write_cmos_sensor(  0x098C, 0xA103);	  
				  MT9V113MIPI_write_cmos_sensor(  0x0990, 0x0005);	  
	        }			   
			  break;
		  case AWB_MODE_DAYLIGHT:
			  {
				  MT9V113MIPI_write_cmos_sensor( 0x098C, 0xA11F);	  
				  MT9V113MIPI_write_cmos_sensor( 0x0990, 0x0000);	
				  MT9V113MIPI_write_cmos_sensor( 0x098C, 0xA103);	 
				  MT9V113MIPI_write_cmos_sensor( 0x0990, 0x0006);	
				  Sleep(25);															   
				  MT9V113MIPI_write_cmos_sensor( 0x098C, 0xA353);	  
				  MT9V113MIPI_write_cmos_sensor( 0x0990, 0x007F);	
				  MT9V113MIPI_write_cmos_sensor( 0x098C, 0xA34E);	  
				  MT9V113MIPI_write_cmos_sensor( 0x0990, 0x00A2);	 
				  MT9V113MIPI_write_cmos_sensor( 0x098C, 0xA350);	 
				  MT9V113MIPI_write_cmos_sensor( 0x0990, 0x0079);	  
				  MT9V113MIPI_write_cmos_sensor( 0x098C, 0xA103);	 
				  MT9V113MIPI_write_cmos_sensor( 0x0990, 0x0005);	  
			  } 	 
			  break;
		  case AWB_MODE_INCANDESCENT: 
			  {
				  MT9V113MIPI_write_cmos_sensor( 0x098C, 0xA11F);	
				  MT9V113MIPI_write_cmos_sensor( 0x0990, 0x0000);	 
				  MT9V113MIPI_write_cmos_sensor( 0x098C, 0xA103);	
				  MT9V113MIPI_write_cmos_sensor( 0x0990, 0x0006);	 
				  Sleep(25);			   
				  MT9V113MIPI_write_cmos_sensor( 0x098C, 0xA353);	 
				  MT9V113MIPI_write_cmos_sensor( 0x0990, 0x0037);	 
				  MT9V113MIPI_write_cmos_sensor( 0x098C, 0xA34E);	 
				  MT9V113MIPI_write_cmos_sensor( 0x0990, 0x00A6);	 
				  MT9V113MIPI_write_cmos_sensor( 0x098C, 0xA350);	 
				  MT9V113MIPI_write_cmos_sensor( 0x0990, 0x0082);	
				  MT9V113MIPI_write_cmos_sensor( 0x098C, 0xA103);	 
				  MT9V113MIPI_write_cmos_sensor( 0x0990, 0x0005);	 
			  } 	  
			  break;  
		  case AWB_MODE_FLUORESCENT:
			  {
				  MT9V113MIPI_write_cmos_sensor(  0x098C, 0xA11F);	  
				  MT9V113MIPI_write_cmos_sensor(  0x0990, 0x0000);	  
				  MT9V113MIPI_write_cmos_sensor(  0x098C, 0xA103);	  
				  MT9V113MIPI_write_cmos_sensor(  0x0990, 0x0006);	  
				  Sleep(25);
				  MT9V113MIPI_write_cmos_sensor(  0x098C, 0xA353);	  
				  MT9V113MIPI_write_cmos_sensor(  0x0990, 0x0030);	  
				  MT9V113MIPI_write_cmos_sensor(  0x098C, 0xA34E);	 
				  MT9V113MIPI_write_cmos_sensor(  0x0990, 0x0099);	 
				  MT9V113MIPI_write_cmos_sensor(  0x098C, 0xA350);	  
				  MT9V113MIPI_write_cmos_sensor(  0x0990, 0x0081);	 
				  MT9V113MIPI_write_cmos_sensor(  0x098C, 0xA103);	 
				  MT9V113MIPI_write_cmos_sensor(  0x0990, 0x0005);	 
			  }   
			  break;  
		  case AWB_MODE_TUNGSTEN:
			 {
				  MT9V113MIPI_write_cmos_sensor(  0x098C, 0xA11F);	 
				  MT9V113MIPI_write_cmos_sensor(  0x0990, 0x0000);	 
				  MT9V113MIPI_write_cmos_sensor(  0x098C, 0xA103);	  
				  MT9V113MIPI_write_cmos_sensor(  0x0990, 0x0006);	  
				  Sleep(25);
				  MT9V113MIPI_write_cmos_sensor(  0x098C, 0xA353);	 
				  MT9V113MIPI_write_cmos_sensor(  0x0990, 0x0000);	  
				  MT9V113MIPI_write_cmos_sensor(  0x098C, 0xA34E);	  
				  MT9V113MIPI_write_cmos_sensor(  0x0990, 0x0093);	 
				  MT9V113MIPI_write_cmos_sensor(  0x098C, 0xA350);	 
				  MT9V113MIPI_write_cmos_sensor(  0x0990, 0x007F);	  
				  MT9V113MIPI_write_cmos_sensor(  0x098C, 0xA103);	  
				  MT9V113MIPI_write_cmos_sensor(  0x0990, 0x0005);	 
		    break;
			  }
		  default:
			  return FALSE;
	  }
	WBcount = para;
	  return TRUE;
	
}


/*************************************************************************
* FUNCTION
*	MT9V113MIPI_set_param_effect
*
* DESCRIPTION
*	effect setting.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
BOOL MT9V113MIPI_set_param_effect(UINT16 para)
{
    SENSORDB("MT9V113MIPI_set_param_effect = %d\n",para);
    switch (para)
    {
        case MEFFECT_OFF:
        {
            MT9V113MIPI_write_cmos_sensor(0x98C, 0x2759);
            MT9V113MIPI_write_cmos_sensor(0x990, 0x6440);
            MT9V113MIPI_write_cmos_sensor(0x98C, 0x275B);
            MT9V113MIPI_write_cmos_sensor(0x990, 0x6440);
            MT9V113MIPI_write_cmos_sensor(0x98C, 0xA103);
            MT9V113MIPI_write_cmos_sensor(0x990, 0x05);
        }
            break;
        case MEFFECT_NEGATIVE:
        {
            MT9V113MIPI_write_cmos_sensor(0x98C, 0x2759);
            MT9V113MIPI_write_cmos_sensor(0x990, 0x0043);
            MT9V113MIPI_write_cmos_sensor(0x98C, 0x275B);
            MT9V113MIPI_write_cmos_sensor(0x990, 0x0943);
            MT9V113MIPI_write_cmos_sensor(0x98C, 0xA103);
            MT9V113MIPI_write_cmos_sensor(0x990, 0x05);
        }
            break;
        case MEFFECT_SEPIA:
        {
            MT9V113MIPI_write_cmos_sensor(0x98C, 0x2759);
            MT9V113MIPI_write_cmos_sensor(0x990, 0x6442);
            MT9V113MIPI_write_cmos_sensor(0x98C, 0x275B);
            MT9V113MIPI_write_cmos_sensor(0x990, 0x6442);
            MT9V113MIPI_write_cmos_sensor(0x98C, 0x2763); //Add Chroma value setting
            MT9V113MIPI_write_cmos_sensor(0x990, 0xb023);
            MT9V113MIPI_write_cmos_sensor(0x98C, 0xA103);
            MT9V113MIPI_write_cmos_sensor(0x990, 0x05);
        }
            break;
        case MEFFECT_SEPIAGREEN:
        {
            MT9V113MIPI_write_cmos_sensor(0x98C, 0x2759);
            MT9V113MIPI_write_cmos_sensor(0x990, 0x6442);
            MT9V113MIPI_write_cmos_sensor(0x98C, 0x275B);
            MT9V113MIPI_write_cmos_sensor(0x990, 0x6442);
            MT9V113MIPI_write_cmos_sensor(0x98C, 0x2763);
            MT9V113MIPI_write_cmos_sensor(0x990, 0x0080);
            MT9V113MIPI_write_cmos_sensor(0x98C, 0xA103);
            MT9V113MIPI_write_cmos_sensor(0x990, 0x05);
        }
            break;
        case MEFFECT_SEPIABLUE:
        {
            MT9V113MIPI_write_cmos_sensor(0x98C, 0x2759);
            MT9V113MIPI_write_cmos_sensor(0x990, 0x6442);
            MT9V113MIPI_write_cmos_sensor(0x98C, 0x275B);
            MT9V113MIPI_write_cmos_sensor(0x990, 0x6442);
            MT9V113MIPI_write_cmos_sensor(0x98C, 0x2763);
            MT9V113MIPI_write_cmos_sensor(0x990, 0x3500);//Decease Chroma value
            MT9V113MIPI_write_cmos_sensor(0x98C, 0xA103);
            MT9V113MIPI_write_cmos_sensor(0x990, 0x05);
        }
            break;
        case MEFFECT_MONO:
        {
            MT9V113MIPI_write_cmos_sensor(0x98C, 0x2759);
            MT9V113MIPI_write_cmos_sensor(0x990, 0x6441);
            MT9V113MIPI_write_cmos_sensor(0x98C, 0x275B);
            MT9V113MIPI_write_cmos_sensor(0x990, 0x6441);
            MT9V113MIPI_write_cmos_sensor(0x98C, 0xA103);
            MT9V113MIPI_write_cmos_sensor(0x990, 0x05);
        }
            break;
        default:
            return KAL_FALSE;
	 }
	 return KAL_TRUE;
}



/*************************************************************************
* FUNCTION
*	MT9V113MIPI_set_param_banding
*
* DESCRIPTION
*	banding setting.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
BOOL MT9V113MIPI_set_param_banding(UINT16 para)
{
	SENSORDB("MT9V113MIPI_set_param_banding = %d\n",para);
	switch (para)
	{
		case AE_FLICKER_MODE_50HZ:
	    {
			MT9V113MIPI_Sensor_Driver.Banding = AE_FLICKER_MODE_50HZ;
			MT9V113MIPI_write_cmos_sensor(0x098C, 0xA11E);		
			MT9V113MIPI_write_cmos_sensor(0x0990, 0x0002);		
			MT9V113MIPI_write_cmos_sensor(0x098C, 0xA404);		
			MT9V113MIPI_write_cmos_sensor(0x0990, 0x00C0);		
			MT9V113MIPI_write_cmos_sensor(0x098C, 0xA103);		
			MT9V113MIPI_write_cmos_sensor(0x0990, 0x0006);		
	    }
		break;
		case AE_FLICKER_MODE_60HZ:
	    {
			MT9V113MIPI_Sensor_Driver.Banding = AE_FLICKER_MODE_60HZ;
			MT9V113MIPI_write_cmos_sensor(0x098C, 0xA11E);		
			MT9V113MIPI_write_cmos_sensor(0x0990, 0x0002);		
			MT9V113MIPI_write_cmos_sensor(0x098C, 0xA404);		
			MT9V113MIPI_write_cmos_sensor(0x0990, 0x00A0);		
			MT9V113MIPI_write_cmos_sensor(0x098C, 0xA103);		
			MT9V113MIPI_write_cmos_sensor(0x0990, 0x0006);		
	    }
		break;
	    default:
	        return KAL_FALSE;
	}
	MT9V113MIPIFixFrameRate(MT9V113MIPI_Sensor_Driver.Min_Frame_Rate,MT9V113MIPI_Sensor_Driver.Max_Frame_Rate);
	return KAL_TRUE;
} 




/*************************************************************************
* FUNCTION
*	MT9V113MIPI_set_param_exposure
*
* DESCRIPTION
*	exposure setting.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
BOOL MT9V113MIPI_set_param_exposure(UINT16 para)
{
	kal_uint16 base_target = 0;
	SENSORDB("MT9V113MIPI_set_param_exposure = %d\n",para);
	switch (para)
	{
		case AE_EV_COMP_13: 
			base_target = 0x54;
			break;  
		case AE_EV_COMP_10:  
			base_target = 0x48;
			break;    
		case AE_EV_COMP_07: 
			base_target = 0x40;
			break;    
		case AE_EV_COMP_03:	
			base_target = 0x38;
			break;    
		case AE_EV_COMP_00: 
			base_target = 0x30;
			break;    
		case AE_EV_COMP_n03: 
			base_target = 0x2C;
			break;    
		case AE_EV_COMP_n07:			
			base_target = 0x28;
			break;    
		case AE_EV_COMP_n10:   
			base_target = 0x24;
			break;
		case AE_EV_COMP_n13: 
			base_target = 0x20;
			break;
		default:
			return FALSE;
	}
	MT9V113MIPI_write_cmos_sensor(0x098C, 0xA24F);		
	MT9V113MIPI_write_cmos_sensor(0x0990, base_target);
	MT9V113MIPI_write_cmos_sensor(0x098C, 0xA103);		
	MT9V113MIPI_write_cmos_sensor(0x0990, 0x0005);
	return TRUE;
}



UINT32 MT9V113MIPIYUVSensorSetting(FEATURE_ID iCmd, UINT32 iPara)
{
    SENSORDB("MT9V113MIPIYUVSensorSetting = %d\n",iCmd);
	switch (iCmd) {
	case FID_SCENE_MODE:	 

		    if (iPara == SCENE_MODE_OFF)
		    {
		        MT9V113MIPI_night_mode(FALSE); 
		    }
		    else if (iPara == SCENE_MODE_NIGHTSCENE)
		    {
	            MT9V113MIPI_night_mode(TRUE); 
		    }				
	     break; 	    
	case FID_AWB_MODE:
           MT9V113MIPI_set_param_wb(iPara);
	     break;
	case FID_COLOR_EFFECT:
           MT9V113MIPI_set_param_effect(iPara);
	     break;
	case FID_AE_EV:	    	    
           MT9V113MIPI_set_param_exposure(iPara);
	     break;
	case FID_AE_FLICKER:	    	    	    
           MT9V113MIPI_set_param_banding(iPara);
	     break;
	case FID_ZOOM_FACTOR:
	     MT9V113MIPI_Sensor_Driver.Digital_Zoom_Factor= iPara; 		
	     break; 
	default:
	     break;
	}
	return TRUE;
}



UINT32 MT9V113MIPIYUVSetVideoMode(UINT16 u2FrameRate)
{
    SENSORDB("MT9V113MIPIYUVSetVideoMode = %d\n",u2FrameRate);
	if (u2FrameRate == 30)
    {	
		MT9V113MIPI_Sensor_Driver.Min_Frame_Rate = 270;
		MT9V113MIPI_Sensor_Driver.Max_Frame_Rate = 270;
    }
    else if (u2FrameRate == 15)       
    {            
		MT9V113MIPI_Sensor_Driver.Min_Frame_Rate = 150;
		MT9V113MIPI_Sensor_Driver.Max_Frame_Rate = 150;
    }
    else 
    {
        printk("Wrong frame rate setting \n");
    }   
	MT9V113MIPIFixFrameRate(MT9V113MIPI_Sensor_Driver.Min_Frame_Rate,MT9V113MIPI_Sensor_Driver.Max_Frame_Rate);
    return TRUE;
}

void MT9V113MIPIYUVGetExifInfo(UINT32 exifAddr)
{
    SENSOR_EXIF_INFO_STRUCT* pExifInfo = (SENSOR_EXIF_INFO_STRUCT*)exifAddr;
    pExifInfo->FNumber = 28;
    pExifInfo->AEISOSpeed = AE_ISO_100;
    pExifInfo->AWBMode = WBcount;
    pExifInfo->CapExposureTime = 0;
    pExifInfo->FlashLightTimeus = 0;
    pExifInfo->RealISOValue = AE_ISO_100;
}



UINT32 MT9V113MIPIFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
							 UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
	UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
	UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
	UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
	UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
	MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
	MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;

	switch (FeatureId)
	{
		case SENSOR_FEATURE_GET_RESOLUTION:
			*pFeatureReturnPara16++=MT9V113MIPI_IMAGE_SENSOR_VGA_WIDTH;
			*pFeatureReturnPara16=MT9V113MIPI_IMAGE_SENSOR_VGA_WIDTH;
			*pFeatureParaLen=4;
		     break;
		case SENSOR_FEATURE_GET_PERIOD:
			*pFeatureReturnPara16++=MT9V113MIPI_IMAGE_SENSOR_VGA_WIDTH+MT9V113MIPI_Sensor_Driver.Dummy_Pixels;
			*pFeatureReturnPara16=MT9V113MIPI_IMAGE_SENSOR_VGA_WIDTH+MT9V113MIPI_Sensor_Driver.Dummy_Lines;
			*pFeatureParaLen=4;
		     break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			*pFeatureReturnPara32 = MT9V113MIPI_Sensor_Driver.Preview_PClk* 1000*1000;
			*pFeatureParaLen=4;
		     break;
		case SENSOR_FEATURE_SET_ESHUTTER:
	
		     break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			 MT9V113MIPI_night_mode((BOOL) *pFeatureData16);
		     break;
		case SENSOR_FEATURE_SET_GAIN:
			 break; 
		case SENSOR_FEATURE_SET_FLASHLIGHT:
		     break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		     break;
		case SENSOR_FEATURE_SET_REGISTER:
			 MT9V113MIPI_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
		     break;
		case SENSOR_FEATURE_GET_REGISTER:
			 pSensorRegData->RegData = MT9V113MIPI_read_cmos_sensor(pSensorRegData->RegAddr);
		     break;
		case SENSOR_FEATURE_GET_CONFIG_PARA:
			 memcpy(pSensorConfigData, &MT9V113MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
			 *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
		     break;
		case SENSOR_FEATURE_SET_CCT_REGISTER:
		case SENSOR_FEATURE_GET_CCT_REGISTER:
		case SENSOR_FEATURE_SET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
		case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
		case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
		case SENSOR_FEATURE_GET_GROUP_INFO:
		case SENSOR_FEATURE_GET_ITEM_INFO:
		case SENSOR_FEATURE_SET_ITEM_INFO:
		case SENSOR_FEATURE_GET_ENG_INFO:
		     break;
		case SENSOR_FEATURE_GET_GROUP_COUNT:
		     break; 
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			*pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
			*pFeatureParaLen=4;
		     break;
		case SENSOR_FEATURE_SET_YUV_CMD:
			 MT9V113MIPIYUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
		     break;	
		case SENSOR_FEATURE_SET_VIDEO_MODE:
		     MT9V113MIPIYUVSetVideoMode(*pFeatureData16);
		     break; 
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
             MT9V113MIPI_GetSensorID(pFeatureReturnPara32); 
            break;     
		case SENSOR_FEATURE_GET_EXIF_INFO:
			SENSORDB("SENSOR_FEATURE_GET_EXIF_INFO\n");
			SENSORDB("EXIF addr = 0x%x\n",*pFeatureData32);
			MT9V113MIPIYUVGetExifInfo(*pFeatureData32);
			break;
		default:
			 break;			
	}
	return ERROR_NONE;
}	/* MT9V113MIPIFeatureControl() */



SENSOR_FUNCTION_STRUCT	SensorFuncMT9V113MIPI=
{
	MT9V113MIPIOpen,
	MT9V113MIPIGetInfo,
	MT9V113MIPIGetResolution,
	MT9V113MIPIFeatureControl,
	MT9V113MIPIControl,
	MT9V113MIPIClose
};



UINT32 MT9V113MIPI_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncMT9V113MIPI;
	return ERROR_NONE;
}
