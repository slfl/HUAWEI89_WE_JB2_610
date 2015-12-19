/*****************************************************************************
 *
 * Filename:
 * ---------
 *   Sensor.c
 *
 * Project:
 * --------
 *   DUMA
 *
 * Description:
 * ------------
 *   Image sensor driver function
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
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

#include "ov8850mipiraw_Sensor.h"
#include "ov8850mipiraw_Camera_Sensor_para.h"
#include "ov8850mipiraw_CameraCustomized.h"

#define OV8850_DEBUG
#define OV8850_DRIVER_TRACE
#define LOG_TAG "[OV88507MIPIRaw]"
#ifdef OV8850_DEBUG
#define SENSORDB(fmt,arg...) printk(LOG_TAG "%s: " fmt "\n", __FUNCTION__ ,##arg)
#else
#define SENSORDB(x,...)
#endif
//#define ACDK
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);



MSDK_SCENARIO_ID_ENUM CurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;
static OV8850_sensor_struct OV8850_sensor =
{
  .eng =
  {
    .reg = OV8850_CAMERA_SENSOR_REG_DEFAULT_VALUE,
    .cct = OV8850_CAMERA_SENSOR_CCT_DEFAULT_VALUE,
  },
  .eng_info =
  {
    .SensorId = OV8850_SENSOR_ID,
    .SensorType = CMOS_SENSOR,
    .SensorOutputDataFormat = OV8850_COLOR_FORMAT,
  },
  .shutter = 0x20,  
  .gain = 0x20,
  .pv_pclk = OV8850_PREVIEW_CLK,
  .cap_pclk = OV8850_CAPTURE_CLK,
  .pclk = OV8850_PREVIEW_CLK,
  .frame_height = OV8850_PV_PERIOD_LINE_NUMS,
  .line_length = OV8850_PV_PERIOD_PIXEL_NUMS,
  .is_zsd = KAL_FALSE, //for zsd
  .dummy_pixels = 0,
  .dummy_lines = 0,  //for zsd
  .is_autofliker = KAL_FALSE,
};

static DEFINE_SPINLOCK(OV8850_drv_lock);

kal_uint16 OV8850_read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
    char puSendCmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    iReadRegI2C(puSendCmd , 2, (u8*)&get_byte,1,OV8850_sensor.write_id);
#ifdef OV8850_DRIVER_TRACE
    //SENSORDB("OV8850_read_cmos_sensor, addr:%x;get_byte:%x \n",addr,get_byte);
#endif
    return get_byte;
}


kal_uint16 OV8850_write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
    //kal_uint16 reg_tmp;

    char puSendCmd[3] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para & 0xFF)};

    iWriteRegI2C(puSendCmd , 3,OV8850_sensor.write_id);
    return 0;
}

////////////////////////////////////////////////////////
#if defined(OV8850_USE_OTP)
struct ov8850_otp_struct ov8850_otp;
static bool OV8850_OTP_FRIST_READ = KAL_FALSE;

//bank num of one group
#define BANK_NUM 5
#define LENC_NUM 62
#define BANK_ITEM_NUM 16


/*************************************************************************
* FUNCTION
*   OV8850_Check_Otp_Group
*
* DESCRIPTION
*   This function check which OTP goup that OTP info lays in
*
* PARAMETERS
*
* RETURNS
*   0 for error, else return the group num:1,2,3
*
* GLOBALS AFFECTED
*
*************************************************************************/

int OV8850_Check_Otp_Group()
{
    kal_uint16 bank;
    kal_uint16 addr;
    int i = 0;
    int group = 0;

    for ( group = 2; group >= 0; group--)
    {
        bank = 0xc0 | (group*5+1); //From Group 3 to Group 1, Group 3 start from bank 11.;

        SENSORDB("OV8850_Check_Otp_Group: selecting group:%d , bank:0x%x\n", (group+1), bank);

        OV8850_write_cmos_sensor(0x3d84, bank);//select bank;
        OV8850_write_cmos_sensor(0x3d81, 0x01);//read OTP into buffer
        msleep(20);

        addr = 0x3d00;
        ov8850_otp.module_info.year = OV8850_read_cmos_sensor(addr++);
        ov8850_otp.module_info.month = OV8850_read_cmos_sensor(addr++);
        ov8850_otp.module_info.date = OV8850_read_cmos_sensor(addr++);
        ov8850_otp.module_info.fuseID[0] = OV8850_read_cmos_sensor(addr++);
        ov8850_otp.module_info.fuseID[1] = OV8850_read_cmos_sensor(addr++);
        ov8850_otp.module_info.fuseID[2] = OV8850_read_cmos_sensor(addr++);
        ov8850_otp.module_info.code = OV8850_read_cmos_sensor(addr++);
        ov8850_otp.module_info.version = OV8850_read_cmos_sensor(addr++);

        OV8850_write_cmos_sensor(0x3d81, 0x00);//disable otp read
        for ( i = 0; i<BANK_ITEM_NUM; i++)//clear otp buffer
        {
            OV8850_write_cmos_sensor(0x3d00 + i, 0x00);
        }

        SENSORDB("ov8850_otp.module_info.year:%d \n",ov8850_otp.module_info.year);
        SENSORDB("ov8850_otp.module_info.month:%d \n",ov8850_otp.module_info.month);
        SENSORDB("ov8850_otp.module_info.date:%d \n",ov8850_otp.module_info.date);
        SENSORDB("ov8850_otp.module_info.code:0x%x \n",ov8850_otp.module_info.code);
        SENSORDB("oov8850_otp.module_info.version :0x%x \n",ov8850_otp.module_info.version);

        if (ov8850_otp.module_info.version && ov8850_otp.module_info.year)
            return (group+1);//found the correct group
    }

    return 0;//no correct otp goup found
        
    
}

/*************************************************************************
* FUNCTION
*   OV8850_Read_OTP
*
* DESCRIPTION
*   This function read the OTP info for Module info, LENC, AWB, VCM cal.
*
* PARAMETERS
*
* RETURNS
*  
*
* GLOBALS AFFECTED
*
*************************************************************************/
int OV8850_Read_OTP(int group)
{
    kal_uint16 bank;
    kal_uint16 addr;
    int i;
    int lencIndex;
    int bankIndex;

    bank = (group -1)*5 + 1;

    SENSORDB("OV8850_Read_OTP: read bank:%d \n", bank);

    OV8850_write_cmos_sensor(0x3d84, 0xc0 | bank);//select bank;
    OV8850_write_cmos_sensor(0x3d81, 0x01);//read OTP into buffer
    msleep(20);

    addr = 0x3d00;
    //module info
    ov8850_otp.module_info.year = OV8850_read_cmos_sensor(addr++);
    ov8850_otp.module_info.month = OV8850_read_cmos_sensor(addr++);
    ov8850_otp.module_info.date = OV8850_read_cmos_sensor(addr++);
    ov8850_otp.module_info.fuseID[0] = OV8850_read_cmos_sensor(addr++);
    ov8850_otp.module_info.fuseID[1] = OV8850_read_cmos_sensor(addr++);
    ov8850_otp.module_info.fuseID[2] = OV8850_read_cmos_sensor(addr++);
    ov8850_otp.module_info.code = OV8850_read_cmos_sensor(addr++);
    ov8850_otp.module_info.version = OV8850_read_cmos_sensor(addr++);
    //AWB cal info
    ov8850_otp.awb_info.wb_RG_H = OV8850_read_cmos_sensor(addr++);
    ov8850_otp.awb_info.wb_RG_L= OV8850_read_cmos_sensor(addr++);
    ov8850_otp.awb_info.wb_BG_H= OV8850_read_cmos_sensor(addr++);
    ov8850_otp.awb_info.wb_BG_L= OV8850_read_cmos_sensor(addr++);
    ov8850_otp.awb_info.wb_GbGr_H= OV8850_read_cmos_sensor(addr++);
    ov8850_otp.awb_info.wb_GbGr_L= OV8850_read_cmos_sensor(addr++);
    //AWB VCM info
    ov8850_otp.vcm_info.start_current= OV8850_read_cmos_sensor(addr++);
    ov8850_otp.vcm_info.max_current = OV8850_read_cmos_sensor(addr++);

    OV8850_write_cmos_sensor(0x3d81, 0x00);//disable otp read
    for ( i = 0; i<BANK_ITEM_NUM; i++)//clear otp buffer
    {
        OV8850_write_cmos_sensor(0x3d00 + i, 0x00);
    }

    lencIndex = 0;

    bank ++;//the next bank is LENC

    for (bankIndex = 0; bankIndex < (BANK_NUM -1); bankIndex++ )
    {
    
        SENSORDB("OV8850_Read_OTP: read lenc bank:%d \n", bank);

        OV8850_write_cmos_sensor(0x3d84, 0xc0 | bank);//select bank;
        OV8850_write_cmos_sensor(0x3d81, 0x01);//read OTP into buffer
        msleep(20);

        addr = 0x3d00;

        for (i = 0; (i< BANK_ITEM_NUM) && (lencIndex < LENC_NUM); i++)
        {
            ov8850_otp.lenc[lencIndex++] = OV8850_read_cmos_sensor(addr++);
        }  

        OV8850_write_cmos_sensor(0x3d81, 0x00);//disable otp read
        for ( i = 0; i<BANK_ITEM_NUM; i++)//clear otp buffer
        {
            OV8850_write_cmos_sensor(0x3d00 + i, 0x00);
        }

        bank++;
    }
    

}

/*************************************************************************
* FUNCTION
*   OV8850_Dump_OTP
*
* DESCRIPTION
*   This function Dump the OTP info.
*
* PARAMETERS
*
* RETURNS
*  
*
* GLOBALS AFFECTED
*
*************************************************************************/

void OV8850_Dump_OTP()
{
    int i=0;
    SENSORDB("OV8850_Dump_OTP BEGIN \n");

    SENSORDB("ov8850_otp.module_info.year:%d \n",ov8850_otp.module_info.year);
    SENSORDB("ov8850_otp.module_info.month:%d \n",ov8850_otp.module_info.month);
    SENSORDB("ov8850_otp.module_info.date:%d \n",ov8850_otp.module_info.date);
    SENSORDB("ov8850_otp.module_info.fuseID[0]:0x%x \n",ov8850_otp.module_info.fuseID[0]);
    SENSORDB("ov8850_otp.module_info.fuseID[0]:0x%x \n",ov8850_otp.module_info.fuseID[0]);
    SENSORDB("ov8850_otp.module_info.fuseID[0]:0x%x \n",ov8850_otp.module_info.fuseID[0]);
    SENSORDB("ov8850_otp.module_info.code:0x%x \n",ov8850_otp.module_info.code);
    SENSORDB("ov8850_otp.module_info.version :0x%x \n",ov8850_otp.module_info.version);
    
    SENSORDB("ov8850_otp.awb_info.wb_RG_H :0x%x \n",ov8850_otp.awb_info.wb_RG_H);
    SENSORDB("ov8850_otp.awb_info.wb_RG_L:0x%x \n",ov8850_otp.awb_info.wb_RG_L);
    SENSORDB("ov8850_otp.awb_info.wb_BG_H:0x%x \n",ov8850_otp.awb_info.wb_BG_H);
    SENSORDB("ov8850_otp.awb_info.wb_BG_L:0x%x \n",ov8850_otp.awb_info.wb_BG_L);
    SENSORDB("ov8850_otp.awb_info.wb_GbGr_H:0x%x \n",ov8850_otp.awb_info.wb_GbGr_H);
    SENSORDB("ov8850_otp.awb_info.wb_GbGr_L:0x%x \n",ov8850_otp.awb_info.wb_GbGr_L);
    
    SENSORDB("ov8850_otp.vcm_info.start_current:0x%x \n",ov8850_otp.vcm_info.start_current);
    SENSORDB("ov8850_otp.vcm_info.max_current:0x%x \n",ov8850_otp.vcm_info.max_current);

    //for (i = 0; i<LENC_NUM; i++)
    //{
    //    SENSORDB("ov8850_otp.lenc[%d]:%x \n",i,ov8850_otp.lenc[i]);
    //}

    SENSORDB("OV8850_Dump_OTP END \n");

}

/*************************************************************************
* FUNCTION
*   OV8850_Upate_AWB_Gain
*
* DESCRIPTION
*   This function Update the AWB info to sensor register.
*
* PARAMETERS
*
* RETURNS
*  
*
* GLOBALS AFFECTED
*
*************************************************************************/
void OV8850_Upate_AWB_Gain(int R_gain, int G_gain, int B_gain)
{
    if (R_gain > 0x400)
    {
        OV8850_write_cmos_sensor(0x3400, R_gain >> 8);
        OV8850_write_cmos_sensor(0x3401, R_gain & 0x00ff);
    }

    if (G_gain > 0x400)
    {
        OV8850_write_cmos_sensor(0x3402, G_gain >> 8);
        OV8850_write_cmos_sensor(0x3403, G_gain & 0x00ff);
    }

    if (B_gain > 0x400)
    {
        OV8850_write_cmos_sensor(0x3404, B_gain >> 8);
        OV8850_write_cmos_sensor(0x3405, B_gain & 0x00ff);
    }

    return;
}

/*************************************************************************
* FUNCTION
*   OV8850_Upate_Otp_AWB
*
* DESCRIPTION
*   This function Update the AWB info.
*
* PARAMETERS
*
* RETURNS
*  
*
* GLOBALS AFFECTED
*
*************************************************************************/
void OV8850_Upate_Otp_AWB()
{
    int R_gain, G_gain, B_gain, G_gain_R, G_gain_B;
    int rg,bg;

    rg = (ov8850_otp.awb_info.wb_RG_H << 8) + ov8850_otp.awb_info.wb_RG_L;
    bg = (ov8850_otp.awb_info.wb_BG_H << 8) + ov8850_otp.awb_info.wb_BG_L;

    SENSORDB("OV8850_Upate_Otp_AWB, r/g:0x%x, b/g:0x%x\n", rg, bg);
    if ((0 == bg) || (0 == rg))
    {
        SENSORDB("OV8850_Upate_Otp_AWB:ERROR Invalid rg and bg\n");
        return;
    }

    //caculate G gain, 0x400 = 1x gain
    if ( bg < BG_TYPICAL)
    {
        if ( rg < RG_TYPICAL)
        {
            //bg < BG_TYPICAL  and rg < RG_TYPICAL
            G_gain = 0x400;
            B_gain = 0x400 * BG_TYPICAL /bg;
            R_gain = 0x400 * RG_TYPICAL/rg;
        }
        else
        {
            //bg < BG_TYPICAL, and rg >= RG_TYPICAL
            R_gain = 0x400;
            G_gain = 0x400 * rg /RG_TYPICAL;
            B_gain = G_gain * BG_TYPICAL / bg;
        }
    }
    else
    {
        if ( rg < RG_TYPICAL )
        {
            //bg > = BG_TYPICAL, and rg < RG_TYPICAL
            B_gain = 0x400;
            G_gain = 0x400 * bg / BG_TYPICAL;
            R_gain = G_gain * RG_TYPICAL /rg;
        }
        else
        {
            //bg >= BG_TYPICAL, and rg >=RG_TYPICAL
            G_gain_B = 0x400 * bg /BG_TYPICAL;
            G_gain_R = 0x400 * rg /RG_TYPICAL;

            if (G_gain_B > G_gain_R)
            {
                B_gain = 0x400;
                G_gain = G_gain_B;
                R_gain = G_gain * RG_TYPICAL / rg;
            }
            else
            {
                R_gain = 0x400;
                G_gain = G_gain_R;
                B_gain = G_gain * BG_TYPICAL / bg;
            }
        }
    }

    SENSORDB("OV8850_Upate_Otp_AWB: R_gain:0x%x,G_gain:0x%x,B_gain:0x%x\n", R_gain,G_gain,B_gain);

    OV8850_Upate_AWB_Gain(R_gain, G_gain, B_gain);
    
}

/*************************************************************************
* FUNCTION
*   OV8850_Update_Otp_LENC
*
* DESCRIPTION
*   This function Update the LENC info to sensor register.
*
* PARAMETERS
*
* RETURNS
* 
*
* GLOBALS AFFECTED
*
*************************************************************************/

void OV8850_Update_Otp_LENC()
{
    int i, temp;

    temp = OV8850_read_cmos_sensor(0x5000);
    temp = 0x80 | temp;
    OV8850_write_cmos_sensor(0x5000, temp);//Enalbe LENC

    for (i = 0; i < LENC_NUM; i++)
    {
        OV8850_write_cmos_sensor(0x5800+i, ov8850_otp.lenc[i]);
    }
}

/*************************************************************************
* FUNCTION
*   OV8850_Update_Otp
*
* DESCRIPTION
*   This function  Check, Read, and Update OTP info.
*
* PARAMETERS
*
* RETURNS
* 
*
* GLOBALS AFFECTED
*
*************************************************************************/

void OV8850_Update_Otp()
{
    int group;

    if ( KAL_TRUE == OV8850_OTP_FRIST_READ)
    {  
        SENSORDB("OV8850_Update_Otp  OTP has been readout \n");
    }
    else
    {
        group = OV8850_Check_Otp_Group();

        if (!group)
        {       
            SENSORDB("OV8850_Update_Otp Error:Cannot find correct OTP group \n");
            return;
        }
        OV8850_Read_OTP(group);
 
        spin_lock(&OV8850_drv_lock);
        OV8850_OTP_FRIST_READ = KAL_TRUE;
        spin_unlock(&OV8850_drv_lock);
    }

    OV8850_Dump_OTP();
    OV8850_Upate_Otp_AWB();
    OV8850_Update_Otp_LENC();
    
}

#endif //end of #if defined(OV8850_USE_OTP)
/////////////////////////////////////////////////////////



void OV8850_Write_Shutter(kal_uint16 ishutter)
{

    kal_uint16 extra_shutter = 0;
    kal_uint16 realtime_fp = 0;
    kal_uint16 frame_height = 0;
    kal_uint16 line_length = 0;

    unsigned long flags;
#ifdef OV8850_DRIVER_TRACE
    SENSORDB("OV8850_write_shutter:%x \n",ishutter);
#endif
   if (!ishutter) ishutter = 1; /* avoid 0 */

    if (OV8850_sensor.pv_mode){
        //line_length = OV8850_PV_PERIOD_PIXEL_NUMS;
        frame_height = OV8850_PV_PERIOD_LINE_NUMS + OV8850_sensor.dummy_lines;
    }
    else if (OV8850_sensor.video_mode) {
        //line_length = OV8850_VIDEO_PERIOD_PIXEL_NUMS;
        frame_height = OV8850_PV_PERIOD_LINE_NUMS + OV8850_sensor.dummy_lines;
    }
    else{
        //line_length = OV8850_FULL_PERIOD_PIXEL_NUMS;
        frame_height = OV8850_FULL_PERIOD_LINE_NUMS + OV8850_sensor.dummy_lines;
    }

    if(ishutter > (frame_height -4))
    {
        extra_shutter = ishutter - frame_height  +4;
        SENSORDB("[shutter > frame_height] frame_height:%x extra_shutter:%x \n",frame_height,extra_shutter);
    }
    else  
    {
        extra_shutter = 0;
    }
    frame_height += extra_shutter;
    OV8850_sensor.frame_height = frame_height;
    SENSORDB("OV8850_sensor.is_autofliker:%x, OV8850_sensor.frame_height: %x \n",OV8850_sensor.is_autofliker,OV8850_sensor.frame_height);

    if(OV8850_sensor.is_autofliker == KAL_TRUE)
    {
        realtime_fp = OV8850_sensor.pclk *10 / (OV8850_sensor.line_length * OV8850_sensor.frame_height);
        SENSORDB("[OV8850_Write_Shutter]pv_clk:%d\n",OV8850_sensor.pclk);
        SENSORDB("[OV8850_Write_Shutter]line_length:%d\n",OV8850_sensor.line_length);
        SENSORDB("[OV8850_Write_Shutter]frame_height:%d\n",OV8850_sensor.frame_height);
        SENSORDB("[OV8850_Write_Shutter]framerate(10base):%d\n",realtime_fp);

        if((realtime_fp >= 297)&&(realtime_fp <= 303))
        {
            realtime_fp = 296;
            spin_lock_irqsave(&OV8850_drv_lock,flags);
            OV8850_sensor.frame_height = OV8850_sensor.pclk *10 / (OV8850_sensor.line_length * realtime_fp);
            spin_unlock_irqrestore(&OV8850_drv_lock,flags);

            SENSORDB("[autofliker realtime_fp=30,extern heights slowdown to 29.6fps][height:%d]",OV8850_sensor.frame_height);
        }
      else if((realtime_fp >= 147)&&(realtime_fp <= 153))
        {
            realtime_fp = 146;
            spin_lock_irqsave(&OV8850_drv_lock,flags);
            OV8850_sensor.frame_height = OV8850_sensor.pclk *10 / (OV8850_sensor.line_length * realtime_fp);
            spin_unlock_irqrestore(&OV8850_drv_lock,flags);
            SENSORDB("[autofliker realtime_fp=15,extern heights slowdown to 14.6fps][height:%d]",OV8850_sensor.frame_height);
        }
    //OV8850_sensor.frame_height = OV8850_sensor.frame_height +(OV8850_sensor.frame_height>>7);

    }
    OV8850_write_cmos_sensor(0x380e, (OV8850_sensor.frame_height>>8)&0xFF);
    OV8850_write_cmos_sensor(0x380f, (OV8850_sensor.frame_height)&0xFF);

    OV8850_write_cmos_sensor(0x3500, (ishutter >> 12) & 0xF);
    OV8850_write_cmos_sensor(0x3501, (ishutter >> 4) & 0xFF);
    OV8850_write_cmos_sensor(0x3502, (ishutter << 4) & 0xFF);

}


/*************************************************************************
* FUNCTION
*   OV8850_Set_Dummy
*
* DESCRIPTION
*   This function set dummy pixel or dummy line of OV8850
*
* PARAMETERS
*   iPixels : dummy pixel
*   iLines :  dummy linel
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/

static void OV8850_Set_Dummy(const kal_uint16 iPixels, const kal_uint16 iLines)
{
    kal_uint16 line_length, frame_height;
#ifdef OV8850_DRIVER_TRACE
    SENSORDB("OV8850_Set_Dummy:iPixels:%x; iLines:%x \n",iPixels,iLines);
#endif
    OV8850_sensor.dummy_lines = iLines;
    OV8850_sensor.dummy_pixels = iPixels;

    switch (CurrentScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            //case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
            line_length = OV8850_PV_PERIOD_PIXEL_NUMS + iPixels;
            frame_height = OV8850_PV_PERIOD_LINE_NUMS + iLines;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            line_length = OV8850_VIDEO_PERIOD_PIXEL_NUMS + iPixels;
            frame_height = OV8850_VIDEO_PERIOD_LINE_NUMS + iLines;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            //case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
            line_length = OV8850_FULL_PERIOD_PIXEL_NUMS + iPixels;
            frame_height = OV8850_FULL_PERIOD_LINE_NUMS + iLines;
            break;
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
            line_length = OV8850_FULL_PERIOD_PIXEL_NUMS + iPixels;
            frame_height = OV8850_FULL_PERIOD_LINE_NUMS + iLines;
            break;
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
            line_length = OV8850_PV_PERIOD_PIXEL_NUMS + iPixels;
            frame_height = OV8850_PV_PERIOD_LINE_NUMS + iLines;
            break;
    }

#ifdef OV8850_DRIVER_TRACE
    SENSORDB("line_length:%x; frame_height:%x \n",line_length,frame_height);
#endif

    if ((line_length >= 0x1FFF)||(frame_height >= 0xFFF))
    {
        #ifdef OV8850_DRIVER_TRACE
        SENSORDB("Warnning: line length or frame height is overflow!!!!!!!!  \n");
        #endif
        return ERROR_NONE;
    }
//	if((line_length == OV8850_sensor.line_length)&&(frame_height == OV8850_sensor.frame_height))
//		return ;
    spin_lock(&OV8850_drv_lock);
    OV8850_sensor.line_length = line_length;
    OV8850_sensor.frame_height = frame_height;
    spin_unlock(&OV8850_drv_lock);

    SENSORDB("line_length:%x; frame_height:%x \n",line_length,frame_height);

    /*  Add dummy pixels: */
    /* 0x380c [0:4], 0x380d defines the PCLKs in one line of OV8850  */  
    /* Add dummy lines:*/
    /* 0x380e [0:1], 0x380f defines total lines in one frame of OV8850 */
    OV8850_write_cmos_sensor(0x380c, line_length >> 8);
    OV8850_write_cmos_sensor(0x380d, line_length & 0xFF);
    OV8850_write_cmos_sensor(0x380e, frame_height >> 8);
    OV8850_write_cmos_sensor(0x380f, frame_height & 0xFF);
    return ERROR_NONE;
}   /*  OV8850_Set_Dummy    */


/*************************************************************************
* FUNCTION
*	OV8850_SetShutter
*
* DESCRIPTION
*	This function set e-shutter of OV8850 to change exposure time.
*
* PARAMETERS
*   iShutter : exposured lines
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/


void set_OV8850_shutter(kal_uint16 iShutter)
{

    unsigned long flags;

#ifdef OV8850_DRIVER_TRACE
    SENSORDB("set_OV8850_shutter:%x \n",iShutter);
#endif
    #if 0
    if((OV8850_sensor.pv_mode == KAL_FALSE)&&(OV8850_sensor.is_zsd == KAL_FALSE))
    {
        SENSORDB("[set_OV8850_shutter]now is in 1/4size cap mode\n");
        //return;
    }
    else if((OV8850_sensor.is_zsd == KAL_TRUE)&&(OV8850_sensor.is_zsd_cap == KAL_TRUE))
    {
        SENSORDB("[set_OV8850_shutter]now is in zsd cap mode\n");

        //SENSORDB("[set_OV8850_shutter]0x3500:%x\n",OV8850_read_cmos_sensor(0x3500));
        //SENSORDB("[set_OV8850_shutter]0x3500:%x\n",OV8850_read_cmos_sensor(0x3501));
        //SENSORDB("[set_OV8850_shutter]0x3500:%x\n",OV8850_read_cmos_sensor(0x3502));
        //return;
    }
    #endif
    #if 0
    if(OV8850_sensor.shutter == iShutter)
    {
        SENSORDB("[set_OV8850_shutter]shutter is the same with previous, skip\n");
        return;
    }
    #endif

    spin_lock_irqsave(&OV8850_drv_lock,flags);
    OV8850_sensor.shutter = iShutter;
    spin_unlock_irqrestore(&OV8850_drv_lock,flags);

    OV8850_Write_Shutter(iShutter);

}   /*  Set_OV8850_Shutter */

 kal_uint16 OV8850Gain2Reg(const kal_uint16 iGain)
{
    kal_uint16 iReg = 0x00;

    //iReg = ((iGain / BASEGAIN) << 4) + ((iGain % BASEGAIN) * 16 / BASEGAIN);
    iReg = iGain *16 / BASEGAIN;

    iReg = iReg & 0xFF;
#ifdef OV8850_DRIVER_TRACE
    SENSORDB("OV8850Gain2Reg:iGain:%x; iReg:%x \n",iGain,iReg);
#endif
    return iReg;
}


kal_uint16 OV8850_SetGain(kal_uint16 iGain)
{
   kal_uint16 iReg;
   unsigned long flags;
#ifdef OV8850_DRIVER_TRACE
   SENSORDB("OV8850_SetGain:%x;\n",iGain);
#endif
    #if 0
    if(OV8850_sensor.gain == iGain)
    {
        SENSORDB("[OV8850_SetGain]:gain is the same with previous,skip\n");
        return ERROR_NONE;
    }
    #endif
   spin_lock_irqsave(&OV8850_drv_lock,flags);
   OV8850_sensor.gain = iGain;
   spin_unlock_irqrestore(&OV8850_drv_lock,flags);

  iReg = OV8850Gain2Reg(iGain);
   
    if (iReg < 0x10) //MINI gain is 0x10	 16 = 1x
    {
        iReg = 0x10;
    }

    else if(iReg > 0xFF) //max gain is 0xFF
    {
        iReg = 0xFF;
    }

    //OV8850_write_cmos_sensor(0x350a, (iReg>>8)&0xFF);
    OV8850_write_cmos_sensor(0x350b, iReg&0xFF);//only use 0x350b for gain control
    return ERROR_NONE;
}




/*************************************************************************
* FUNCTION
*	OV8850_SetGain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*   iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/

#if 0
void OV8850_set_isp_driving_current(kal_uint16 current)
{
#ifdef OV8850_DRIVER_TRACE
   SENSORDB("OV8850_set_isp_driving_current:current:%x;\n",current);
#endif
  //iowrite32((0x2 << 12)|(0<<28)|(0x8880888), 0xF0001500);
}
#endif

/*************************************************************************
* FUNCTION
*	OV8850_NightMode
*
* DESCRIPTION
*	This function night mode of OV8850.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void OV8850_night_mode(kal_bool enable)
{
}   /*  OV8850_NightMode    */


/* write camera_para to sensor register */
static void OV8850_camera_para_to_sensor(void)
{
    kal_uint32 i;
#ifdef OV8850_DRIVER_TRACE
    SENSORDB("OV8850_camera_para_to_sensor\n");
#endif
  for (i = 0; 0xFFFFFFFF != OV8850_sensor.eng.reg[i].Addr; i++)
  {
    OV8850_write_cmos_sensor(OV8850_sensor.eng.reg[i].Addr, OV8850_sensor.eng.reg[i].Para);
  }
  for (i = OV8850_FACTORY_START_ADDR; 0xFFFFFFFF != OV8850_sensor.eng.reg[i].Addr; i++)
  {
    OV8850_write_cmos_sensor(OV8850_sensor.eng.reg[i].Addr, OV8850_sensor.eng.reg[i].Para);
  }
  OV8850_SetGain(OV8850_sensor.gain); /* update gain */
}

/* update camera_para from sensor register */
static void OV8850_sensor_to_camera_para(void)
{
  kal_uint32 i;
  kal_uint32 temp_data;
  
#ifdef OV8850_DRIVER_TRACE
   SENSORDB("OV8850_sensor_to_camera_para\n");
#endif
  for (i = 0; 0xFFFFFFFF != OV8850_sensor.eng.reg[i].Addr; i++)
  {
    temp_data = OV8850_read_cmos_sensor(OV8850_sensor.eng.reg[i].Addr);

    spin_lock(&OV8850_drv_lock);
    OV8850_sensor.eng.reg[i].Para = temp_data;
    spin_unlock(&OV8850_drv_lock);

    }
  for (i = OV8850_FACTORY_START_ADDR; 0xFFFFFFFF != OV8850_sensor.eng.reg[i].Addr; i++)
  {
    temp_data = OV8850_read_cmos_sensor(OV8850_sensor.eng.reg[i].Addr);

    spin_lock(&OV8850_drv_lock);
    OV8850_sensor.eng.reg[i].Para = temp_data;
    spin_unlock(&OV8850_drv_lock);
  }
}

/* ------------------------ Engineer mode ------------------------ */
inline static void OV8850_get_sensor_group_count(kal_int32 *sensor_count_ptr)
{
#ifdef OV8850_DRIVER_TRACE
   SENSORDB("OV8850_get_sensor_group_count\n");
#endif
  *sensor_count_ptr = OV8850_GROUP_TOTAL_NUMS;
}

inline static void OV8850_get_sensor_group_info(MSDK_SENSOR_GROUP_INFO_STRUCT *para)
{
#ifdef OV8850_DRIVER_TRACE
   SENSORDB("OV8850_get_sensor_group_info\n");
#endif
  switch (para->GroupIdx)
  {
  case OV8850_PRE_GAIN:
    sprintf(para->GroupNamePtr, "CCT");
    para->ItemCount = 5;
    break;
  case OV8850_CMMCLK_CURRENT:
    sprintf(para->GroupNamePtr, "CMMCLK Current");
    para->ItemCount = 1;
    break;
  case OV8850_FRAME_RATE_LIMITATION:
    sprintf(para->GroupNamePtr, "Frame Rate Limitation");
    para->ItemCount = 2;
    break;
  case OV8850_REGISTER_EDITOR:
    sprintf(para->GroupNamePtr, "Register Editor");
    para->ItemCount = 2;
    break;
  default:
    ASSERT(0);
  }
}

inline static void OV8850_get_sensor_item_info(MSDK_SENSOR_ITEM_INFO_STRUCT *para)
{

  const static kal_char *cct_item_name[] = {"SENSOR_BASEGAIN", "Pregain-R", "Pregain-Gr", "Pregain-Gb", "Pregain-B"};
  const static kal_char *editer_item_name[] = {"REG addr", "REG value"};
  
#ifdef OV8850_DRIVER_TRACE
	 SENSORDB("OV8850_get_sensor_item_info\n");
#endif
  switch (para->GroupIdx)
  {
  case OV8850_PRE_GAIN:
    switch (para->ItemIdx)
    {
    case OV8850_SENSOR_BASEGAIN:
    case OV8850_PRE_GAIN_R_INDEX:
    case OV8850_PRE_GAIN_Gr_INDEX:
    case OV8850_PRE_GAIN_Gb_INDEX:
    case OV8850_PRE_GAIN_B_INDEX:
      break;
    default:
      ASSERT(0);
    }
    sprintf(para->ItemNamePtr, cct_item_name[para->ItemIdx - OV8850_SENSOR_BASEGAIN]);
    para->ItemValue = OV8850_sensor.eng.cct[para->ItemIdx].Para * 1000 / BASEGAIN;
    para->IsTrueFalse = para->IsReadOnly = para->IsNeedRestart = KAL_FALSE;
    para->Min = OV8850_MIN_ANALOG_GAIN * 1000;
    para->Max = OV8850_MAX_ANALOG_GAIN * 1000;
    break;
  case OV8850_CMMCLK_CURRENT:
    switch (para->ItemIdx)
    {
    case 0:
      sprintf(para->ItemNamePtr, "Drv Cur[2,4,6,8]mA");
      switch (OV8850_sensor.eng.reg[OV8850_CMMCLK_CURRENT_INDEX].Para)
      {
      case ISP_DRIVING_2MA:
        para->ItemValue = 2;
        break;
      case ISP_DRIVING_4MA:
        para->ItemValue = 4;
        break;
      case ISP_DRIVING_6MA:
        para->ItemValue = 6;
        break;
      case ISP_DRIVING_8MA:
        para->ItemValue = 8;
        break;
      default:
        ASSERT(0);
      }
      para->IsTrueFalse = para->IsReadOnly = KAL_FALSE;
      para->IsNeedRestart = KAL_TRUE;
      para->Min = 2;
      para->Max = 8;
      break;
    default:
      ASSERT(0);
    }
    break;
  case OV8850_FRAME_RATE_LIMITATION:
    switch (para->ItemIdx)
    {
    case 0:
      sprintf(para->ItemNamePtr, "Max Exposure Lines");
      para->ItemValue = 5998;
      break;
    case 1:
      sprintf(para->ItemNamePtr, "Min Frame Rate");
      para->ItemValue = 5;
      break;
    default:
      ASSERT(0);
    }
    para->IsTrueFalse = para->IsNeedRestart = KAL_FALSE;
    para->IsReadOnly = KAL_TRUE;
    para->Min = para->Max = 0;
    break;
  case OV8850_REGISTER_EDITOR:
    switch (para->ItemIdx)
    {
    case 0:
    case 1:
      sprintf(para->ItemNamePtr, editer_item_name[para->ItemIdx]);
      para->ItemValue = 0;
      para->IsTrueFalse = para->IsReadOnly = para->IsNeedRestart = KAL_FALSE;
      para->Min = 0;
      para->Max = (para->ItemIdx == 0 ? 0xFFFF : 0xFF);
      break;
    default:
      ASSERT(0);
    }
    break;
  default:
    ASSERT(0);
  }
}

inline static kal_bool OV8850_set_sensor_item_info(MSDK_SENSOR_ITEM_INFO_STRUCT *para)
{
  kal_uint16 temp_para;
#ifdef OV8850_DRIVER_TRACE
   SENSORDB("OV8850_set_sensor_item_info\n");
#endif
  switch (para->GroupIdx)
  {
  case OV8850_PRE_GAIN:
    switch (para->ItemIdx)
    {
    case OV8850_SENSOR_BASEGAIN:
    case OV8850_PRE_GAIN_R_INDEX:
    case OV8850_PRE_GAIN_Gr_INDEX:
    case OV8850_PRE_GAIN_Gb_INDEX:
    case OV8850_PRE_GAIN_B_INDEX:
        spin_lock(&OV8850_drv_lock);
        OV8850_sensor.eng.cct[para->ItemIdx].Para = para->ItemValue * BASEGAIN / 1000;
        spin_unlock(&OV8850_drv_lock);

        OV8850_SetGain(OV8850_sensor.gain); /* update gain */
        break;
    default:
        ASSERT(0);
    }
    break;
  case OV8850_CMMCLK_CURRENT:
    switch (para->ItemIdx)
    {
    case 0:
      switch (para->ItemValue)
      {
      case 2:
        temp_para = ISP_DRIVING_2MA;
        break;
      case 3:
      case 4:
        temp_para = ISP_DRIVING_4MA;
        break;
      case 5:
      case 6:
        temp_para = ISP_DRIVING_6MA;
        break;
      default:
        temp_para = ISP_DRIVING_8MA;
        break;
      }
        //OV8850_set_isp_driving_current((kal_uint16)temp_para);
        spin_lock(&OV8850_drv_lock);
        OV8850_sensor.eng.reg[OV8850_CMMCLK_CURRENT_INDEX].Para = temp_para;
        spin_unlock(&OV8850_drv_lock);
      break;
    default:
      ASSERT(0);
    }
    break;
  case OV8850_FRAME_RATE_LIMITATION:
    ASSERT(0);
    break;
  case OV8850_REGISTER_EDITOR:
    switch (para->ItemIdx)
    {
      static kal_uint32 fac_sensor_reg;
    case 0:
      if (para->ItemValue < 0 || para->ItemValue > 0xFFFF) return KAL_FALSE;
      fac_sensor_reg = para->ItemValue;
      break;
    case 1:
      if (para->ItemValue < 0 || para->ItemValue > 0xFF) return KAL_FALSE;
      OV8850_write_cmos_sensor(fac_sensor_reg, para->ItemValue);
      break;
    default:
      ASSERT(0);
    }
    break;
  default:
    ASSERT(0);
  }
  return KAL_TRUE;
}


//Sensor globle setting: 4 lane 504M bps/lane
void OV8850_globle_setting(void)
{
/*
    ;Input Clock = 24MHz
    ;4-lane MIPI
    ;Max data rate = 516Mbps/lane
    ;DCBLC ON with auto load mode, LENC OFF, DPC ON, MWB ON
    ;MIPI data rate = 516Mbps/lane
    ;free running MIPI clock with short package
*/
    SENSORDB("OV8850_globle_setting  start \n");
    OV8850_write_cmos_sensor(0x0103, 0x01);
    msleep(6);
    OV8850_write_cmos_sensor(0x0102, 0x01);
    OV8850_write_cmos_sensor(0x3002, 0x08);
    OV8850_write_cmos_sensor(0x3004, 0x00);
    OV8850_write_cmos_sensor(0x3005, 0x00);
    OV8850_write_cmos_sensor(0x3011, 0x41);//4
    OV8850_write_cmos_sensor(0x3012, 0x08);
    OV8850_write_cmos_sensor(0x3014, 0x4a);
    OV8850_write_cmos_sensor(0x3015, 0x0a);//4 
    OV8850_write_cmos_sensor(0x3021, 0x00);
    OV8850_write_cmos_sensor(0x3022, 0x02);
    OV8850_write_cmos_sensor(0x3081, 0x02);
    OV8850_write_cmos_sensor(0x3083, 0x01);

    OV8850_write_cmos_sensor(0x309a, 0x00);
    OV8850_write_cmos_sensor(0x309b, 0x00);
    OV8850_write_cmos_sensor(0x309c, 0x00);
    OV8850_write_cmos_sensor(0x30b3, 0x2b);//516Mbps/lane
    OV8850_write_cmos_sensor(0x30b4, 0x02);
    OV8850_write_cmos_sensor(0x30b5, 0x04);
    OV8850_write_cmos_sensor(0x30b6, 0x01);
    OV8850_write_cmos_sensor(0x3104, 0xa1);
    OV8850_write_cmos_sensor(0x3106, 0x01);

    OV8850_write_cmos_sensor(0x3503, 0x07);
    OV8850_write_cmos_sensor(0x350a, 0x00);
    OV8850_write_cmos_sensor(0x350b, 0x38);
    OV8850_write_cmos_sensor(0x3602, 0x50);
    OV8850_write_cmos_sensor(0x3620, 0x64);
    OV8850_write_cmos_sensor(0x3622, 0x0f);
    OV8850_write_cmos_sensor(0x3623, 0x68);

    OV8850_write_cmos_sensor(0x3625, 0x40);
    OV8850_write_cmos_sensor(0x3631, 0x83);
    OV8850_write_cmos_sensor(0x3633, 0x34);
    OV8850_write_cmos_sensor(0x3634, 0x03);
    OV8850_write_cmos_sensor(0x364c, 0x00);
    OV8850_write_cmos_sensor(0x364d, 0x00);
    OV8850_write_cmos_sensor(0x364e, 0x00);
    OV8850_write_cmos_sensor(0x364f, 0x00);
    OV8850_write_cmos_sensor(0x3660, 0x80);
    OV8850_write_cmos_sensor(0x3662, 0x10);
    OV8850_write_cmos_sensor(0x3665, 0x00);
    OV8850_write_cmos_sensor(0x3666, 0x00);
    OV8850_write_cmos_sensor(0x366f, 0x20);

    OV8850_write_cmos_sensor(0x3703, 0x2e);
    OV8850_write_cmos_sensor(0x3732, 0x05);
    OV8850_write_cmos_sensor(0x373d, 0x22);
    OV8850_write_cmos_sensor(0x3754, 0xc0);
    OV8850_write_cmos_sensor(0x3756, 0x2a);
    OV8850_write_cmos_sensor(0x3759, 0x0f);
    OV8850_write_cmos_sensor(0x376b, 0x44);
    OV8850_write_cmos_sensor(0x3795, 0x00);
    OV8850_write_cmos_sensor(0x379c, 0x0c);
    OV8850_write_cmos_sensor(0x3810, 0x00);
    OV8850_write_cmos_sensor(0x3811, 0x04);
    OV8850_write_cmos_sensor(0x3812, 0x00);
    OV8850_write_cmos_sensor(0x3813, 0x04);

    OV8850_write_cmos_sensor(0x3826, 0x00);
    OV8850_write_cmos_sensor(0x3a04, 0x09);
    OV8850_write_cmos_sensor(0x3a05, 0xa9);
    OV8850_write_cmos_sensor(0x4000, 0x10);//DCLBC auto load mode

    OV8850_write_cmos_sensor(0x4002, 0xc5);
    OV8850_write_cmos_sensor(0x4005, 0x18);
    OV8850_write_cmos_sensor(0x4006, 0x20);
    OV8850_write_cmos_sensor(0x4008, 0x20);//DCBLC on
    OV8850_write_cmos_sensor(0x4009, 0x10);
    OV8850_write_cmos_sensor(0x404f, 0x7f);

    OV8850_write_cmos_sensor(0x4300, 0xff);
    OV8850_write_cmos_sensor(0x4301, 0x00);
    OV8850_write_cmos_sensor(0x4315, 0x00);
    OV8850_write_cmos_sensor(0x4512, 0x01);
    OV8850_write_cmos_sensor(0x4800, 0x14);//short pacakge enable
    OV8850_write_cmos_sensor(0x4837, 0x10);
    OV8850_write_cmos_sensor(0x4a00, 0xaa);
    OV8850_write_cmos_sensor(0x4a03, 0x01);
    OV8850_write_cmos_sensor(0x4a05, 0x08);
    OV8850_write_cmos_sensor(0x4d00, 0x04);
    OV8850_write_cmos_sensor(0x4d01, 0x52);
    OV8850_write_cmos_sensor(0x4d02, 0xfe);
    OV8850_write_cmos_sensor(0x4d03, 0x05);
    OV8850_write_cmos_sensor(0x4d04, 0xff);
    OV8850_write_cmos_sensor(0x4d05, 0xff);
    OV8850_write_cmos_sensor(0x5000, 0x06);//LENC ON
    OV8850_write_cmos_sensor(0x5001, 0x01);//MWB ON
    OV8850_write_cmos_sensor(0x5002, 0x80);
    OV8850_write_cmos_sensor(0x5041, 0x04);
    OV8850_write_cmos_sensor(0x5043, 0x48);
    OV8850_write_cmos_sensor(0x5e00, 0x00);
    OV8850_write_cmos_sensor(0x5e10, 0x1c);
    OV8850_write_cmos_sensor(0x0100, 0x01);

}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
void OV8850_1632_1224_4Lane_30fps_Mclk24M_setting(void)
{
/*
    Input clock for all 4 Lane MIPI settings are 24Mhz. 
    The maximum MIPI data rate is decided from 3264x2448 30fps. 
    Other resolution keep same data rate as 8M 30fps.
    Raw 10bit 1632*1224 30fps 4lane 516M bps/lane,SCLK=204M
*/
    OV8850_write_cmos_sensor(0x0100, 0x00);// software standby
    OV8850_write_cmos_sensor(0x3011, 0x41);//2 
    OV8850_write_cmos_sensor(0x3015, 0x0a);// MIPI mode,  ca
    OV8850_write_cmos_sensor(0x3090, 0x02);//204MHz SCLK
    OV8850_write_cmos_sensor(0x3091, 0x11);
    OV8850_write_cmos_sensor(0x3092, 0x00);
    OV8850_write_cmos_sensor(0x3093, 0x00);
    OV8850_write_cmos_sensor(0x3094, 0x00);
    OV8850_write_cmos_sensor(0x3098, 0x03);//240MHz DAC
    OV8850_write_cmos_sensor(0x3099, 0x1e);//1c
    OV8850_write_cmos_sensor(0x30b3, 0x2a);//504Mbps/lane 3c
    OV8850_write_cmos_sensor(0x30b4, 0x02);
    OV8850_write_cmos_sensor(0x30b5, 0x04);
    OV8850_write_cmos_sensor(0x30b6, 0x01);
    OV8850_write_cmos_sensor(0x3500, 0x00);
    OV8850_write_cmos_sensor(0x3501, 0x75);
    OV8850_write_cmos_sensor(0x3502, 0x80);
    OV8850_write_cmos_sensor(0x3624, 0x04);
    OV8850_write_cmos_sensor(0x3680, 0xe0);
    OV8850_write_cmos_sensor(0x3702, 0xf3);//db
    OV8850_write_cmos_sensor(0x3704, 0x71);
    OV8850_write_cmos_sensor(0x3708, 0xe6);
    OV8850_write_cmos_sensor(0x3709, 0xc3);
    OV8850_write_cmos_sensor(0x371F, 0x0c);//0x18
    OV8850_write_cmos_sensor(0x3739, 0x30);
    OV8850_write_cmos_sensor(0x373a, 0x51);
    OV8850_write_cmos_sensor(0x373C, 0x20);//38
    OV8850_write_cmos_sensor(0x3781, 0x0c);
    OV8850_write_cmos_sensor(0x3786, 0x16);
    OV8850_write_cmos_sensor(0x3796, 0x64);//78
    OV8850_write_cmos_sensor(0x3800, 0x00);
    OV8850_write_cmos_sensor(0x3801, 0x08);
    OV8850_write_cmos_sensor(0x3802, 0x00);
    OV8850_write_cmos_sensor(0x3803, 0x08);
    OV8850_write_cmos_sensor(0x3804, 0x0c);
    OV8850_write_cmos_sensor(0x3805, 0xD7);
    OV8850_write_cmos_sensor(0x3806, 0x09);
    OV8850_write_cmos_sensor(0x3807, 0xA7);
    OV8850_write_cmos_sensor(0x3808, 0x06);
    OV8850_write_cmos_sensor(0x3809, 0x60);
    OV8850_write_cmos_sensor(0x380a, 0x04);
    OV8850_write_cmos_sensor(0x380b, 0xC8);
    OV8850_write_cmos_sensor(0x380c, 0x0E);//for 30fps
    OV8850_write_cmos_sensor(0x380d, 0x18);
    OV8850_write_cmos_sensor(0x380e, 0x07);
    OV8850_write_cmos_sensor(0x380f, 0x5c);//fa
    OV8850_write_cmos_sensor(0x3814, 0x31);
    OV8850_write_cmos_sensor(0x3815, 0x31);
    OV8850_write_cmos_sensor(0x3820, 0x11);
    OV8850_write_cmos_sensor(0x3821, 0x0f);
    OV8850_write_cmos_sensor(0x4001, 0x02);
    OV8850_write_cmos_sensor(0x4004, 0x04);
    OV8850_write_cmos_sensor(0x4100, 0x04);
    OV8850_write_cmos_sensor(0x4101, 0x04);
    OV8850_write_cmos_sensor(0x4102, 0x04);
    OV8850_write_cmos_sensor(0x4104, 0x04);
    OV8850_write_cmos_sensor(0x4109, 0x04);

    OV8850_write_cmos_sensor(0x4005, 0x18);
    OV8850_write_cmos_sensor(0x404f, 0xa0);

    OV8850_write_cmos_sensor(0x0100, 0x01); //wake up from software standby

}

void OV8850_3264_2448_4Lane_22fps_Mclk24M_setting(void)
{
/*
    Raw 10bit 3264*2448 22.4fps 4lane 516M bps/lane,SCLK=204M
*/
    OV8850_write_cmos_sensor(0x0100, 0x00); // software standby
    OV8850_write_cmos_sensor(0x3011, 0x41); // 4 Lane, MIPI enable
    OV8850_write_cmos_sensor(0x3015, 0x0a); // MIPI mode,
    OV8850_write_cmos_sensor(0x3090, 0x02); //204MHz SCLK 
    OV8850_write_cmos_sensor(0x3091, 0x11); 
    OV8850_write_cmos_sensor(0x3092, 0x00); 
    OV8850_write_cmos_sensor(0x3093, 0x00); 
    OV8850_write_cmos_sensor(0x3094, 0x00); 
    OV8850_write_cmos_sensor(0x3098, 0x03); //240MHz DAC 
    OV8850_write_cmos_sensor(0x3099, 0x1e); 
    OV8850_write_cmos_sensor(0x30b3, 0x2b);//516Mbps/lane
    OV8850_write_cmos_sensor(0x30b4, 0x02); 
    OV8850_write_cmos_sensor(0x30b5, 0x04); 
    OV8850_write_cmos_sensor(0x30b6, 0x01); 
    OV8850_write_cmos_sensor(0x3500, 0x00); 
    OV8850_write_cmos_sensor(0x3501, 0x9D); 
    OV8850_write_cmos_sensor(0x3502, 0x00); 
    OV8850_write_cmos_sensor(0x3624, 0x00);
    OV8850_write_cmos_sensor(0x3680, 0xB0);
    OV8850_write_cmos_sensor(0x3702, 0x6E);
    OV8850_write_cmos_sensor(0x3704, 0x55);
    OV8850_write_cmos_sensor(0x3708, 0xe4); 
    OV8850_write_cmos_sensor(0x3709, 0xc3); 
    OV8850_write_cmos_sensor(0x371F, 0x0D);
    OV8850_write_cmos_sensor(0x3739, 0x80);
    OV8850_write_cmos_sensor(0x373a, 0x51);
    OV8850_write_cmos_sensor(0x373C, 0x24);
    OV8850_write_cmos_sensor(0x3781, 0xc8);
    OV8850_write_cmos_sensor(0x3786, 0x08);
    OV8850_write_cmos_sensor(0x3796, 0x43);
    OV8850_write_cmos_sensor(0x3800, 0x00); 
    OV8850_write_cmos_sensor(0x3801, 0x0C); 
    OV8850_write_cmos_sensor(0x3802, 0x00); 
    OV8850_write_cmos_sensor(0x3803, 0x0c); 
    OV8850_write_cmos_sensor(0x3804, 0x0c); 
    OV8850_write_cmos_sensor(0x3805, 0xD3); 
    OV8850_write_cmos_sensor(0x3806, 0x09); 
    OV8850_write_cmos_sensor(0x3807, 0xa3); 
    OV8850_write_cmos_sensor(0x3808, 0x0c); 
    OV8850_write_cmos_sensor(0x3809, 0xc0); 
    OV8850_write_cmos_sensor(0x380a, 0x09); 
    OV8850_write_cmos_sensor(0x380b, 0x90); 
    OV8850_write_cmos_sensor(0x380c, 0x0e); 
    OV8850_write_cmos_sensor(0x380d, 0x18); 
    OV8850_write_cmos_sensor(0x380e, 0x09); 
    OV8850_write_cmos_sensor(0x380f, 0xDA); 
    OV8850_write_cmos_sensor(0x3814, 0x11); 
    OV8850_write_cmos_sensor(0x3815, 0x11); 
    OV8850_write_cmos_sensor(0x3820, 0x10); 
    OV8850_write_cmos_sensor(0x3821, 0x0e); 
    OV8850_write_cmos_sensor(0x4001, 0x06);
    OV8850_write_cmos_sensor(0x4004, 0x04); 
    OV8850_write_cmos_sensor(0x4100, 0x22); 
    OV8850_write_cmos_sensor(0x4101, 0x23); 
    OV8850_write_cmos_sensor(0x4102, 0x44); 
    OV8850_write_cmos_sensor(0x4104, 0x5c); 
    OV8850_write_cmos_sensor(0x4109, 0x03);

    OV8850_write_cmos_sensor(0x4005, 0x1a);

    OV8850_write_cmos_sensor(0x0100, 0x01); //wake up from software standby

}

void OV8850_1920_1080_4Lane_30fps_Mclk24M_setting(void)
{
/*
    Raw 10bit 1920*1080 30fps 4lane 516M bps/lane,SCLK=204M
*/
    OV8850_write_cmos_sensor(0x0100, 0x00);// software standby
    OV8850_write_cmos_sensor(0x3011, 0x41);// 4 Lane, MIPI enable
    OV8850_write_cmos_sensor(0x3015, 0x0a);// MIPI mode,
    OV8850_write_cmos_sensor(0x3090, 0x02);// 204MHz SCLK 
    OV8850_write_cmos_sensor(0x3091, 0x11); 
    OV8850_write_cmos_sensor(0x3092, 0x00); 
    OV8850_write_cmos_sensor(0x3093, 0x00); 
    OV8850_write_cmos_sensor(0x3094, 0x00); 
    OV8850_write_cmos_sensor(0x3098, 0x03);//240MHz DAC 
    OV8850_write_cmos_sensor(0x3099, 0x1e); 
    OV8850_write_cmos_sensor(0x30b3, 0x2b);//516Mbps/lane
    OV8850_write_cmos_sensor(0x30b4, 0x02); 
    OV8850_write_cmos_sensor(0x30b5, 0x04); 
    OV8850_write_cmos_sensor(0x30b6, 0x01); 
    OV8850_write_cmos_sensor(0x3500, 0x00); 
    OV8850_write_cmos_sensor(0x3501, 0x75);      
    OV8850_write_cmos_sensor(0x3502, 0x80); 
    OV8850_write_cmos_sensor(0x3624, 0x04);
    OV8850_write_cmos_sensor(0x3680, 0xe0);
    OV8850_write_cmos_sensor(0x3702, 0xF3); 
    OV8850_write_cmos_sensor(0x3704, 0x71);
    OV8850_write_cmos_sensor(0x3708, 0xe3);
    OV8850_write_cmos_sensor(0x3709, 0xc3); 
    OV8850_write_cmos_sensor(0x371F, 0x0C); 
    OV8850_write_cmos_sensor(0x3739, 0x30);
    OV8850_write_cmos_sensor(0x373a, 0x51);
    OV8850_write_cmos_sensor(0x373C, 0x20); 
    OV8850_write_cmos_sensor(0x3781, 0x0c);
    OV8850_write_cmos_sensor(0x3786, 0x16);
    OV8850_write_cmos_sensor(0x3796, 0x64);
    OV8850_write_cmos_sensor(0x3800, 0x00); 
    OV8850_write_cmos_sensor(0x3801, 0x0c);
    OV8850_write_cmos_sensor(0x3802, 0x01); 
    OV8850_write_cmos_sensor(0x3803, 0x40); 
    OV8850_write_cmos_sensor(0x3804, 0x0c); 
    OV8850_write_cmos_sensor(0x3805, 0xd3); 
    OV8850_write_cmos_sensor(0x3806, 0x08); 
    OV8850_write_cmos_sensor(0x3807, 0x73); 
    OV8850_write_cmos_sensor(0x3808, 0x07); 
    OV8850_write_cmos_sensor(0x3809, 0x80); 
    OV8850_write_cmos_sensor(0x380a, 0x04); 
    OV8850_write_cmos_sensor(0x380b, 0x38); 
    OV8850_write_cmos_sensor(0x380c, 0x0e);//for 30fps  
    OV8850_write_cmos_sensor(0x380d, 0x18); 
    OV8850_write_cmos_sensor(0x380e, 0x07); 
    OV8850_write_cmos_sensor(0x380f, 0x5c); 
    OV8850_write_cmos_sensor(0x3814, 0x11); 
    OV8850_write_cmos_sensor(0x3815, 0x11); 
    OV8850_write_cmos_sensor(0x3820, 0x10); 
    OV8850_write_cmos_sensor(0x3821, 0x0e); 
    OV8850_write_cmos_sensor(0x4001, 0x02);
    OV8850_write_cmos_sensor(0x4004, 0x04);
    OV8850_write_cmos_sensor(0x4100, 0x04); 
    OV8850_write_cmos_sensor(0x4101, 0x04); 
    OV8850_write_cmos_sensor(0x4102, 0x04); 
    OV8850_write_cmos_sensor(0x4104, 0x04);
    OV8850_write_cmos_sensor(0x4109, 0x04);

    OV8850_write_cmos_sensor(0x4005, 0x18);
    OV8850_write_cmos_sensor(0x404f, 0xa0);

    OV8850_write_cmos_sensor(0x0100, 0x01);//wake up from software standby

}

UINT32 OV8850Open(void)
{
    kal_uint16 sensor_id=0; 
    int i;
    const kal_uint16 sccb_writeid[] = {OV8850_SLAVE_WRITE_ID_1,OV8850_SLAVE_WRITE_ID_2};

   spin_lock(&OV8850_drv_lock);
   OV8850_sensor.is_zsd = KAL_FALSE;  //for zsd full size preview
   OV8850_sensor.is_zsd_cap = KAL_FALSE;
   OV8850_sensor.is_autofliker = KAL_FALSE; //for autofliker.
   OV8850_sensor.pv_mode = KAL_TRUE;
   OV8850_sensor.pclk = OV8850_PREVIEW_CLK;
   spin_unlock(&OV8850_drv_lock);
   
  for(i = 0; i <(sizeof(sccb_writeid)/sizeof(sccb_writeid[0])); i++)
    {
        spin_lock(&OV8850_drv_lock);
        OV8850_sensor.write_id = sccb_writeid[i];
        OV8850_sensor.read_id = (sccb_writeid[i]|0x01);
        spin_unlock(&OV8850_drv_lock);

        sensor_id=((OV8850_read_cmos_sensor(0x300A) << 8) | OV8850_read_cmos_sensor(0x300B));

#ifdef OV8850_DRIVER_TRACE
        SENSORDB("OV8850Open, sensor_id:%x \n",sensor_id);
#endif
        if(OV8850_SENSOR_ID == sensor_id)
        {
            SENSORDB("OV8850 slave write id:%x \n",OV8850_sensor.write_id);
            break;
        }
    }
  
    // check if sensor ID correct
    if (sensor_id != OV8850_SENSOR_ID) 
    {
        SENSORDB("OV8850 Check ID fails! \n");
        sensor_id = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }

    OV8850_globle_setting();

#if defined(OV8850_USE_OTP)
    OV8850_Update_Otp();
#endif

    SENSORDB("test for bootimage \n");

   return ERROR_NONE;
}   /* OV8850Open  */

/*************************************************************************
* FUNCTION
*   OV5642GetSensorID
*
* DESCRIPTION
*   This function get the sensor ID 
*
* PARAMETERS
*   *sensorID : return the sensor ID 
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 OV8850GetSensorID(UINT32 *sensorID) 
{
  //added by mandrave
   int i;
   const kal_uint16 sccb_writeid[] = {OV8850_SLAVE_WRITE_ID_1,OV8850_SLAVE_WRITE_ID_2};
 

    for(i = 0; i <(sizeof(sccb_writeid)/sizeof(sccb_writeid[0])); i++)
    {
        spin_lock(&OV8850_drv_lock);
        OV8850_sensor.write_id = sccb_writeid[i];
        OV8850_sensor.read_id = (sccb_writeid[i]|0x01);
        spin_unlock(&OV8850_drv_lock);

        *sensorID=((OV8850_read_cmos_sensor(0x300A) << 8) | OV8850_read_cmos_sensor(0x300B));	

#ifdef OV8850_DRIVER_TRACE
        SENSORDB("OV8850Open, sensor_id:%x \n",*sensorID);
#endif
        if(OV8850_SENSOR_ID == *sensorID)
        {
            SENSORDB("OV8850 slave write id:%x \n",OV8850_sensor.write_id);
            break;
        }
    }

    // check if sensor ID correct		
    if (*sensorID != OV8850_SENSOR_ID) 
    {
        *sensorID = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }

   return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*	OV8850Close
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
UINT32 OV8850Close(void)
{
#ifdef OV8850_DRIVER_TRACE
   SENSORDB("OV8850Close\n");
#endif

    return ERROR_NONE;
}   /* OV8850Close */

/*************************************************************************
* FUNCTION
* OV8850Preview
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
UINT32 OV8850Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint16 dummy_line;
#ifdef OV8850_DRIVER_TRACE
    SENSORDB("OV8850Preview \n");
#endif

    OV8850_1632_1224_4Lane_30fps_Mclk24M_setting();

    //msleep(10);
    spin_lock(&OV8850_drv_lock);
    OV8850_sensor.pv_mode = KAL_TRUE;
    spin_unlock(&OV8850_drv_lock);

#if 0
    switch (sensor_config_data->SensorOperationMode)
    {
        case MSDK_SENSOR_OPERATION_MODE_VIDEO: 
            spin_lock(&OV8850_drv_lock);
            OV8850_sensor.video_mode = KAL_TRUE;
            spin_unlock(&OV8850_drv_lock);
            dummy_line = 0;
#ifdef OV8850_DRIVER_TRACE
            SENSORDB("Video mode \n");
#endif
            break;
        default: /* ISP_PREVIEW_MODE */
            spin_lock(&OV8850_drv_lock);
            OV8850_sensor.video_mode = KAL_FALSE;
            spin_unlock(&OV8850_drv_lock);
            dummy_line = 0;
#ifdef OV8850_DRIVER_TRACE
            SENSORDB("Camera preview mode \n");
#endif
            break;
    }
#endif
    spin_lock(&OV8850_drv_lock);
    OV8850_sensor.video_mode = KAL_FALSE;
    spin_unlock(&OV8850_drv_lock);
    dummy_line = 0;

    spin_lock(&OV8850_drv_lock);
    OV8850_sensor.dummy_pixels = 0;
    OV8850_sensor.dummy_lines = 0;
    OV8850_sensor.line_length = OV8850_PV_PERIOD_PIXEL_NUMS;
    OV8850_sensor.frame_height = OV8850_PV_PERIOD_LINE_NUMS + dummy_line;
    OV8850_sensor.pclk = OV8850_PREVIEW_CLK;
    spin_unlock(&OV8850_drv_lock);

    OV8850_Set_Dummy(0, dummy_line); /* modify dummy_pixel must gen AE table again */
    //OV8850_Write_Shutter(OV8850_sensor.shutter);

    msleep(10);

    return ERROR_NONE;

}   /*  OV8850Preview   */


/*************************************************************************
* FUNCTION
* OV8850VIDEO
*
* DESCRIPTION
*	This function start the sensor Video preview.
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

UINT32 OV8850VIDEO(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    kal_uint16 dummy_line;
    kal_uint16 ret;
#ifdef OV8850_DRIVER_TRACE
    SENSORDB("OV8850VIDEO \n");
#endif

    OV8850_1920_1080_4Lane_30fps_Mclk24M_setting();

    spin_lock(&OV8850_drv_lock);
    OV8850_sensor.pv_mode = KAL_FALSE;
    spin_unlock(&OV8850_drv_lock);

    spin_lock(&OV8850_drv_lock);
    OV8850_sensor.video_mode = KAL_TRUE;
    spin_unlock(&OV8850_drv_lock);
    dummy_line = 0;

    spin_lock(&OV8850_drv_lock);
    OV8850_sensor.dummy_pixels = 0;
    OV8850_sensor.dummy_lines = 0;
    OV8850_sensor.line_length = OV8850_VIDEO_PERIOD_PIXEL_NUMS;
    OV8850_sensor.frame_height = OV8850_VIDEO_PERIOD_LINE_NUMS+ dummy_line;
    OV8850_sensor.pclk = OV8850_VIDEO_CLK;
    spin_unlock(&OV8850_drv_lock);

    OV8850_Set_Dummy(0, dummy_line); /* modify dummy_pixel must gen AE table again */
    msleep(10);

    return ERROR_NONE;
	
}   /*  OV8850VIDEO   */


/*************************************************************************
* FUNCTION
*    OV8850ZsdPreview
*
* DESCRIPTION
*    This function setup the CMOS sensor in Full Size output  mode
*
* PARAMETERS
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 OV8850ZsdPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
    MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)

{
    kal_uint16 dummy_pixel = 0;
    kal_uint16 dummy_line = 0;
    kal_uint16 ret;
#ifdef OV8850_DRIVER_TRACE
    SENSORDB("OV8850ZsdPreview \n");
#endif

    OV8850_3264_2448_4Lane_22fps_Mclk24M_setting();
    spin_lock(&OV8850_drv_lock);
    OV8850_sensor.pv_mode = KAL_FALSE;
    spin_unlock(&OV8850_drv_lock);


    spin_lock(&OV8850_drv_lock);
    OV8850_sensor.video_mode = KAL_FALSE;
    spin_unlock(&OV8850_drv_lock);
    dummy_line = 0;

    spin_lock(&OV8850_drv_lock);
    OV8850_sensor.dummy_pixels = 0;
    OV8850_sensor.dummy_lines = 0;
    OV8850_sensor.line_length = OV8850_FULL_PERIOD_PIXEL_NUMS;
    OV8850_sensor.frame_height = OV8850_FULL_PERIOD_LINE_NUMS+ dummy_line;
    OV8850_sensor.pclk = OV8850_ZSD_PRE_CLK;
    spin_unlock(&OV8850_drv_lock);

    OV8850_Set_Dummy(0, dummy_line); /* modify dummy_pixel must gen AE table again */
    msleep(10);

    return ERROR_NONE;
}



/*************************************************************************
* FUNCTION
*OV8850Capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 OV8850Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    kal_uint16 dummy_pixel = 0;
    kal_uint16 dummy_line = 0;
    kal_uint16 ret;
#ifdef OV8850_DRIVER_TRACE
    SENSORDB("OV8850Capture start \n");
#endif


    spin_lock(&OV8850_drv_lock);
    OV8850_sensor.video_mode = KAL_FALSE;
    OV8850_sensor.is_autofliker = KAL_FALSE;
    OV8850_sensor.pv_mode = KAL_FALSE;
    spin_unlock(&OV8850_drv_lock);

    OV8850_3264_2448_4Lane_22fps_Mclk24M_setting();

    spin_lock(&OV8850_drv_lock);
    OV8850_sensor.dummy_pixels = 0;
    OV8850_sensor.dummy_lines = 0;
    spin_unlock(&OV8850_drv_lock);

    dummy_pixel = 0;
    dummy_line = 0;

    spin_lock(&OV8850_drv_lock);
    OV8850_sensor.pclk = OV8850_CAPTURE_CLK;
    OV8850_sensor.line_length = OV8850_FULL_PERIOD_PIXEL_NUMS + dummy_pixel;
    OV8850_sensor.frame_height = OV8850_FULL_PERIOD_LINE_NUMS + dummy_line;
    spin_unlock(&OV8850_drv_lock);

    OV8850_Set_Dummy(dummy_pixel, dummy_line);


#ifdef OV8850_DRIVER_TRACE
    SENSORDB("OV8850Capture end\n");
#endif
    msleep(10);

    return ERROR_NONE;
}   /* OV8850_Capture() */

UINT32 OV8850GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
#ifdef OV8850_DRIVER_TRACE
    SENSORDB("OV8850GetResolution \n");
#endif
    pSensorResolution->SensorFullWidth=OV8850_IMAGE_SENSOR_FULL_WIDTH;
    pSensorResolution->SensorFullHeight=OV8850_IMAGE_SENSOR_FULL_HEIGHT;
    pSensorResolution->SensorPreviewWidth=OV8850_IMAGE_SENSOR_PV_WIDTH;
    pSensorResolution->SensorPreviewHeight=OV8850_IMAGE_SENSOR_PV_HEIGHT;
    pSensorResolution->SensorVideoWidth=OV8850_IMAGE_SENSOR_VIDEO_WIDTH;
    pSensorResolution->SensorVideoHeight=OV8850_IMAGE_SENSOR_VIDEO_HEIGHT;
    return ERROR_NONE;
}/* OV8850GetResolution() */

UINT32 OV8850GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
        MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
        MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
#ifdef OV8850_DRIVER_TRACE
    SENSORDB("OV8850GetInfo£¬FeatureId:%d\n",ScenarioId);
#endif

    switch(ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
            pSensorInfo->SensorPreviewResolutionX=OV8850_IMAGE_SENSOR_FULL_WIDTH;
            pSensorInfo->SensorPreviewResolutionY=OV8850_IMAGE_SENSOR_FULL_HEIGHT;
            pSensorInfo->SensorFullResolutionX=OV8850_IMAGE_SENSOR_FULL_WIDTH;
            pSensorInfo->SensorFullResolutionY=OV8850_IMAGE_SENSOR_FULL_HEIGHT;
            pSensorInfo->SensorCameraPreviewFrameRate = 15;
            break;
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            pSensorInfo->SensorPreviewResolutionX=OV8850_IMAGE_SENSOR_PV_WIDTH;
            pSensorInfo->SensorPreviewResolutionY=OV8850_IMAGE_SENSOR_PV_HEIGHT;
            pSensorInfo->SensorFullResolutionX=OV8850_IMAGE_SENSOR_FULL_WIDTH;
            pSensorInfo->SensorFullResolutionY=OV8850_IMAGE_SENSOR_FULL_HEIGHT;
            pSensorInfo->SensorCameraPreviewFrameRate=30;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            pSensorInfo->SensorPreviewResolutionX=OV8850_IMAGE_SENSOR_PV_WIDTH;
            pSensorInfo->SensorPreviewResolutionY=OV8850_IMAGE_SENSOR_PV_HEIGHT;
            pSensorInfo->SensorFullResolutionX=OV8850_IMAGE_SENSOR_FULL_WIDTH;
            pSensorInfo->SensorFullResolutionY=OV8850_IMAGE_SENSOR_FULL_HEIGHT;
            pSensorInfo->SensorCameraPreviewFrameRate=30;
            break;
        default:
            pSensorInfo->SensorPreviewResolutionX=OV8850_IMAGE_SENSOR_PV_WIDTH;
            pSensorInfo->SensorPreviewResolutionY=OV8850_IMAGE_SENSOR_PV_HEIGHT;
            pSensorInfo->SensorFullResolutionX=OV8850_IMAGE_SENSOR_FULL_WIDTH;
            pSensorInfo->SensorFullResolutionY=OV8850_IMAGE_SENSOR_FULL_HEIGHT;
            pSensorInfo->SensorCameraPreviewFrameRate = 30;
            break;
    }

    //pSensorInfo->SensorCameraPreviewFrameRate=30;
    pSensorInfo->SensorVideoFrameRate=30;
    pSensorInfo->SensorStillCaptureFrameRate=10;
    pSensorInfo->SensorWebCamCaptureFrameRate=15;
    pSensorInfo->SensorResetActiveHigh=FALSE; //low active
    pSensorInfo->SensorResetDelayCount=5; 

    pSensorInfo->SensorOutputDataFormat=OV8850_COLOR_FORMAT;
    pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;	
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorInterruptDelayLines = 4;
    pSensorInfo->SensroInterfaceType        = SENSOR_INTERFACE_TYPE_MIPI;
    pSensorInfo->CaptureDelayFrame = 1; 
    pSensorInfo->PreviewDelayFrame = 3; 
    pSensorInfo->VideoDelayFrame = 1;

    pSensorInfo->SensorMasterClockSwitch = 0; 
    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_4MA;
    pSensorInfo->AEShutDelayFrame = 0;   /* The frame of setting shutter default 0 for TG int */
    pSensorInfo->AESensorGainDelayFrame = 0;/* The frame of setting sensor gain */
    pSensorInfo->AEISPGainDelayFrame = 2;    
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockDividCount=3;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount= 3;
            pSensorInfo->SensorDataLatchCount= 2;
            pSensorInfo->SensorGrabStartX = OV8850_PV_X_START; 
            pSensorInfo->SensorGrabStartY = OV8850_PV_Y_START; 

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_4_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
            pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
            pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockDividCount=	3;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount= 3;
            pSensorInfo->SensorDataLatchCount= 2;
            pSensorInfo->SensorGrabStartX = OV8850_PV_X_START; 
            pSensorInfo->SensorGrabStartY = OV8850_PV_Y_START; 

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_4_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
            pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
            pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
            pSensorInfo->SensorPacketECCOrder = 1;

            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockDividCount= 3;
            pSensorInfo->SensorClockRisingCount=0;
            pSensorInfo->SensorClockFallingCount=2;
            pSensorInfo->SensorPixelClockCount=3;
            pSensorInfo->SensorDataLatchCount=2;
            pSensorInfo->SensorGrabStartX = OV8850_FULL_X_START; 
            pSensorInfo->SensorGrabStartY = OV8850_FULL_Y_START; 

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_4_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
            pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
            pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
            pSensorInfo->SensorPacketECCOrder = 1;

            break;
        default:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockDividCount=3;
            pSensorInfo->SensorClockRisingCount=0;
            pSensorInfo->SensorClockFallingCount=2;
            pSensorInfo->SensorPixelClockCount=3;
            pSensorInfo->SensorDataLatchCount=2;
            pSensorInfo->SensorGrabStartX = OV8850_PV_X_START; 
            pSensorInfo->SensorGrabStartY = OV8850_PV_Y_START; 

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_4_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
            pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
            pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
    }
#if 0
    //OV8850PixelClockDivider=pSensorInfo->SensorPixelClockCount;
    memcpy(pSensorConfigData, &OV8850SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
#endif
  return ERROR_NONE;
}	/* OV8850GetInfo() */


UINT32 OV8850Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
        MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
#ifdef OV8850_DRIVER_TRACE
    SENSORDB("OV8850Control£¬FeatureId:%d\n",ScenarioId);
#endif	

    spin_lock(&OV8850_drv_lock);
    CurrentScenarioId = ScenarioId;
    spin_unlock(&OV8850_drv_lock);

    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            OV8850Preview(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            OV8850VIDEO(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            OV8850Capture(pImageWindow, pSensorConfigData);
        break;
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
            OV8850ZsdPreview(pImageWindow, pSensorConfigData);
            break;
        default:
            return ERROR_INVALID_SCENARIO_ID;
    }
    return ERROR_NONE;
}	/* OV8850Control() */

UINT32 OV8850SetAutoFlickerMode(kal_bool bEnable, UINT16 u2FrameRate)
{

    //kal_uint32 pv_max_frame_rate_lines = OV8850_sensor.dummy_lines;

    SENSORDB("[OV8850SetAutoFlickerMode] frame rate(10base) = %d %d\n", bEnable, u2FrameRate);

    if(bEnable)
    {
        spin_lock(&OV8850_drv_lock);
        OV8850_sensor.is_autofliker = KAL_TRUE;
        spin_unlock(&OV8850_drv_lock);
    }
    else
    {
        spin_lock(&OV8850_drv_lock);
        OV8850_sensor.is_autofliker = KAL_FALSE;
        spin_unlock(&OV8850_drv_lock);
    }
    SENSORDB("[OV8850SetAutoFlickerMode]bEnable:%x \n",bEnable);
	return ERROR_NONE;
}


UINT32 OV8850SetCalData(PSET_SENSOR_CALIBRATION_DATA_STRUCT pSetSensorCalData)
{
    UINT32 i;
    SENSORDB("OV8850 Sensor write calibration data num = %d \r\n", pSetSensorCalData->DataSize);
    SENSORDB("OV8850 Sensor write calibration data format = %x \r\n", pSetSensorCalData->DataFormat);
    if(pSetSensorCalData->DataSize <= MAX_SHADING_DATA_TBL){
        for (i = 0; i < pSetSensorCalData->DataSize; i++){
            if (((pSetSensorCalData->DataFormat & 0xFFFF) == 1) && ((pSetSensorCalData->DataFormat >> 16) == 1)){
                SENSORDB("OV8850 Sensor write calibration data: address = %x, value = %x \r\n",(pSetSensorCalData->ShadingData[i])>>16,(pSetSensorCalData->ShadingData[i])&0xFFFF);
                OV8850_write_cmos_sensor((pSetSensorCalData->ShadingData[i])>>16, (pSetSensorCalData->ShadingData[i])&0xFFFF);
            }
        }
    }
    return ERROR_NONE;
}

UINT32 OV8850SetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) 
{
    kal_uint32 pclk;
    kal_int16 dummyLine;
    kal_uint16 lineLength,frameHeight;

    SENSORDB("OV8850SetMaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
    switch (scenarioId) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            pclk = OV8850_PREVIEW_CLK;
            lineLength = OV8850_PV_PERIOD_PIXEL_NUMS;
            frameHeight = (10 * pclk)/frameRate/lineLength;
            dummyLine = frameHeight - OV8850_PV_PERIOD_LINE_NUMS;
            if (dummyLine < 0){
            dummyLine = 0;
            }
            OV8850_Set_Dummy(0, dummyLine);
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            pclk = OV8850_VIDEO_CLK;
            lineLength = OV8850_VIDEO_PERIOD_PIXEL_NUMS;
            frameHeight = (10 * pclk)/frameRate/lineLength;
            dummyLine = frameHeight - OV8850_VIDEO_PERIOD_LINE_NUMS;
            if (dummyLine < 0){
            dummyLine = 0;
            }
            OV8850_Set_Dummy(0, dummyLine);
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
            pclk = OV8850_CAPTURE_CLK;
            lineLength = OV8850_FULL_PERIOD_PIXEL_NUMS;
            frameHeight = (10 * pclk)/frameRate/lineLength;
            if(frameHeight < OV8850_FULL_PERIOD_LINE_NUMS)
            frameHeight = OV8850_FULL_PERIOD_LINE_NUMS;
            dummyLine = frameHeight - OV8850_FULL_PERIOD_LINE_NUMS;
            SENSORDB("OV8850SetMaxFramerateByScenario: scenarioId = %d, frame rate calculate = %d\n",((10 * pclk)/frameHeight/lineLength));
            OV8850_Set_Dummy(0, dummyLine);
            break;		
        default:
            break;
    }	
        return ERROR_NONE;
}


UINT32 OV8850GetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
{
    switch (scenarioId) {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        *pframeRate = 300;
        break;
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
    case MSDK_SCENARIO_ID_CAMERA_ZSD:
        *pframeRate = 150;
        break;
    case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
    case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
        *pframeRate = 300;
        break;
    default:
        break;
    }

    return ERROR_NONE;
}

UINT32 OV8850SetVideoMode(UINT16 u2FrameRate)
{
	kal_int16 dummy_line;
    /* to fix VSYNC, to fix frame rate */
#ifdef OV8850_DRIVER_TRACE
    SENSORDB("OV8850SetVideoMode£¬u2FrameRate:%d\n",u2FrameRate);
#endif	

    if((30 == u2FrameRate)||(15 == u2FrameRate)||(24 == u2FrameRate))
    {
        dummy_line = OV8850_sensor.pclk / u2FrameRate / OV8850_sensor.line_length - OV8850_sensor.frame_height;
        if (dummy_line < 0) 
            dummy_line = 0;
#ifdef OV8850_DRIVER_TRACE
        SENSORDB("dummy_line %d\n", dummy_line);
#endif
        OV8850_Set_Dummy(0, dummy_line); /* modify dummy_pixel must gen AE table again */
        spin_lock(&OV8850_drv_lock);
        OV8850_sensor.video_mode = KAL_TRUE;
        spin_unlock(&OV8850_drv_lock);
    }
    else if(0 == u2FrameRate)
    {
        spin_lock(&OV8850_drv_lock);
        OV8850_sensor.video_mode = KAL_FALSE;
        spin_unlock(&OV8850_drv_lock);

        SENSORDB("disable video mode\n");
    }
    else{
        SENSORDB("[OV8850SetVideoMode],Error Framerate, u2FrameRate=%d",u2FrameRate);
    }
    return ERROR_NONE;
}


UINT32 OV8850FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
        UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
    UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
    UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
    UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
    UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
    UINT32 OV8850SensorRegNumber;
    UINT32 i;
    //PNVRAM_SENSOR_DATA_STRUCT pSensorDefaultData=(PNVRAM_SENSOR_DATA_STRUCT) pFeaturePara;
    //MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
    MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
    //MSDK_SENSOR_GROUP_INFO_STRUCT *pSensorGroupInfo=(MSDK_SENSOR_GROUP_INFO_STRUCT *) pFeaturePara;
    //MSDK_SENSOR_ITEM_INFO_STRUCT *pSensorItemInfo=(MSDK_SENSOR_ITEM_INFO_STRUCT *) pFeaturePara;
    //MSDK_SENSOR_ENG_INFO_STRUCT	*pSensorEngInfo=(MSDK_SENSOR_ENG_INFO_STRUCT *) pFeaturePara;
    PSET_SENSOR_CALIBRATION_DATA_STRUCT pSetSensorCalData=(PSET_SENSOR_CALIBRATION_DATA_STRUCT)pFeaturePara;

#ifdef OV8850_DRIVER_TRACE
    //SENSORDB("OV8850FeatureControl£¬FeatureId:%d\n",FeatureId); 
#endif		
    switch (FeatureId)
    {
        case SENSOR_FEATURE_GET_RESOLUTION:
            *pFeatureReturnPara16++=OV8850_IMAGE_SENSOR_FULL_WIDTH;
            *pFeatureReturnPara16=OV8850_IMAGE_SENSOR_FULL_HEIGHT;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_PERIOD:/* 3 */
            *pFeatureReturnPara16++= OV8850_sensor.line_length;
            *pFeatureReturnPara16= OV8850_sensor.frame_height;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:  /* 3 */
            *pFeatureReturnPara32 = OV8850_sensor.pclk;
            break;
        case SENSOR_FEATURE_SET_ESHUTTER:	/* 4 */
            set_OV8850_shutter(*pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            //OV8850_night_mode((BOOL) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_GAIN:	/* 6 */
            OV8850_SetGain((UINT16) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            OV8850_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            pSensorRegData->RegData = OV8850_read_cmos_sensor(pSensorRegData->RegAddr);
            break;
        case SENSOR_FEATURE_SET_CCT_REGISTER:
            //memcpy(&OV8850_sensor.eng.cct, pFeaturePara, sizeof(OV8850_sensor.eng.cct));
            OV8850SensorRegNumber = OV8850_FACTORY_END_ADDR;
            for (i=0;i<OV8850SensorRegNumber;i++)
            {
                spin_lock(&OV8850_drv_lock);
                OV8850_sensor.eng.cct[i].Addr=*pFeatureData32++;
                OV8850_sensor.eng.cct[i].Para=*pFeatureData32++;
                spin_unlock(&OV8850_drv_lock);
            }

            break;
        case SENSOR_FEATURE_GET_CCT_REGISTER:/* 12 */
            if (*pFeatureParaLen >= sizeof(OV8850_sensor.eng.cct) + sizeof(kal_uint32))
            {
                *((kal_uint32 *)pFeaturePara++) = sizeof(OV8850_sensor.eng.cct);
                memcpy(pFeaturePara, &OV8850_sensor.eng.cct, sizeof(OV8850_sensor.eng.cct));
            }
            break;
        case SENSOR_FEATURE_SET_ENG_REGISTER:
            //memcpy(&OV8850_sensor.eng.reg, pFeaturePara, sizeof(OV8850_sensor.eng.reg));
            OV8850SensorRegNumber = OV8850_ENGINEER_END;
            for (i=0;i<OV8850SensorRegNumber;i++)
            {
                spin_lock(&OV8850_drv_lock);
                OV8850_sensor.eng.reg[i].Addr=*pFeatureData32++;
                OV8850_sensor.eng.reg[i].Para=*pFeatureData32++;
                spin_unlock(&OV8850_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_ENG_REGISTER:	/* 14 */
            if (*pFeatureParaLen >= sizeof(OV8850_sensor.eng.reg) + sizeof(kal_uint32))
            {
                *((kal_uint32 *)pFeaturePara++) = sizeof(OV8850_sensor.eng.reg);
                memcpy(pFeaturePara, &OV8850_sensor.eng.reg, sizeof(OV8850_sensor.eng.reg));
            }
            break;
        case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
            ((PNVRAM_SENSOR_DATA_STRUCT)pFeaturePara)->Version = NVRAM_CAMERA_SENSOR_FILE_VERSION;
            ((PNVRAM_SENSOR_DATA_STRUCT)pFeaturePara)->SensorId = OV8850_SENSOR_ID;
            memcpy(((PNVRAM_SENSOR_DATA_STRUCT)pFeaturePara)->SensorEngReg, &OV8850_sensor.eng.reg, sizeof(OV8850_sensor.eng.reg));
            memcpy(((PNVRAM_SENSOR_DATA_STRUCT)pFeaturePara)->SensorCCTReg, &OV8850_sensor.eng.cct, sizeof(OV8850_sensor.eng.cct));
            *pFeatureParaLen = sizeof(NVRAM_SENSOR_DATA_STRUCT);
            break;
        case SENSOR_FEATURE_GET_CONFIG_PARA:
            memcpy(pFeaturePara, &OV8850_sensor.cfg_data, sizeof(OV8850_sensor.cfg_data));
            *pFeatureParaLen = sizeof(OV8850_sensor.cfg_data);
            break;
        case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
            OV8850_camera_para_to_sensor();
            break;
        case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
            OV8850_sensor_to_camera_para();
            break;
        case SENSOR_FEATURE_GET_GROUP_COUNT:
            OV8850_get_sensor_group_count((kal_uint32 *)pFeaturePara);
            *pFeatureParaLen = 4;
            break;
        case SENSOR_FEATURE_GET_GROUP_INFO:
            OV8850_get_sensor_group_info((MSDK_SENSOR_GROUP_INFO_STRUCT *)pFeaturePara);
            *pFeatureParaLen = sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_ITEM_INFO:
            OV8850_get_sensor_item_info((MSDK_SENSOR_ITEM_INFO_STRUCT *)pFeaturePara);
            *pFeatureParaLen = sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_SET_ITEM_INFO:
            OV8850_set_sensor_item_info((MSDK_SENSOR_ITEM_INFO_STRUCT *)pFeaturePara);
            *pFeatureParaLen = sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_ENG_INFO:
            memcpy(pFeaturePara, &OV8850_sensor.eng_info, sizeof(OV8850_sensor.eng_info));
            *pFeatureParaLen = sizeof(OV8850_sensor.eng_info);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            OV8850SetVideoMode(*pFeatureData16);
            break; 
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            OV8850GetSensorID(pFeatureReturnPara32); 
            break; 
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            OV8850SetAutoFlickerMode((BOOL)*pFeatureData16,*(pFeatureData16+1));
            break;
        case SENSOR_FEATURE_SET_CALIBRATION_DATA:
            OV8850SetCalData(pSetSensorCalData);
            break;
        case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            OV8850SetMaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, *(pFeatureData32+1));
            break;
        case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            OV8850GetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, (MUINT32 *)(*(pFeatureData32+1)));
            break;
        default:
            break;
    }
    return ERROR_NONE;
}/* OV8850FeatureControl() */
SENSOR_FUNCTION_STRUCT SensorFuncOV8850=
{
    OV8850Open,
    OV8850GetInfo,
    OV8850GetResolution,
    OV8850FeatureControl,
    OV8850Control,
    OV8850Close
};

UINT32 OV8850_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
/* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncOV8850;

    return ERROR_NONE;
}/* SensorInit() */


