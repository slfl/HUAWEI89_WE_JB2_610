/*******************************************************************************************/
      
 
/*******************************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>    
#include <linux/fs.h>
#include <asm/atomic.h>
#include <asm/system.h>
#include <linux/xlog.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "s5k4h5yxmipiraw_Sensor.h"
#include "s5k4h5yxmipiraw_Camera_Sensor_para.h"
#include "s5k4h5yxmipiraw_CameraCustomized.h"
static DEFINE_SPINLOCK(s5k4h5yxmipiraw_drv_lock);

#define S5K4H5YX_DEBUG
//#define S5K4H5YX_DEBUG_SOFIA

#ifdef S5K4H5YX_DEBUG
	#define S5K4H5YXDB(fmt, arg...) xlog_printk(ANDROID_LOG_DEBUG, "[S5K4H5YXMIPI]" , fmt, ##arg)
#else
	#define S5K4H5YXDB(x,...)
#endif

#ifdef S5K4H5YX_DEBUG_SOFIA
	#define S5K4H5YXDBSOFIA(fmt, arg...) xlog_printk(ANDROID_LOG_DEBUG, "[S5K4H5YXMIPI]", fmt, ##arg)
#else
	#define S5K4H5YXDBSOFIA(x,...)
#endif

#define mDELAY(ms)  mdelay(ms)
#define Capture8M

//add this globle data
kal_uint32 s5k4h5yx_video_frame_length = 2060;
kal_uint32 s5k4h5yx_video_line_length = 4044;
kal_uint32 s5k4h5yx_preview_frame_length = 2260;
kal_uint32 s5k4h5yx_preview_line_length = 3688;
kal_uint32 s5k4h5yx_capture_frame_length =2486;
kal_uint32 s5k4h5yx_capture_line_length = 4044;

kal_uint32 S5K4H5YX_FeatureControl_PERIOD_PixelNum;
kal_uint32 S5K4H5YX_FeatureControl_PERIOD_LineNum;

kal_uint32 cont_preview_line_length = 3688;
kal_uint32 cont_preview_frame_length = 2260;
kal_uint32 cont_capture_line_length = 4044;
kal_uint32 cont_capture_frame_length = 2486;

MSDK_SENSOR_CONFIG_STRUCT S5K4H5YXSensorConfigData;

kal_uint32 S5K4H5YX_FAC_SENSOR_REG;

MSDK_SCENARIO_ID_ENUM S5K4H5YXCurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;

/* FIXME: old factors and DIDNOT use now. s*/
SENSOR_REG_STRUCT S5K4H5YXSensorCCT[]=CAMERA_SENSOR_CCT_DEFAULT_VALUE;
SENSOR_REG_STRUCT S5K4H5YXSensorReg[ENGINEER_END]=CAMERA_SENSOR_REG_DEFAULT_VALUE;
/* FIXME: old factors and DIDNOT use now. e*/

static S5K4H5YX_PARA_STRUCT s5k4h5yx;

extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);

#define S5K4H5YX_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 1, S5K4H5YXMIPI_WRITE_ID)

kal_uint16 S5K4H5YX_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,S5K4H5YXMIPI_WRITE_ID);
    return get_byte;
}

#define Sleep(ms) mdelay(ms)

void S5K4H5YX_write_shutter(kal_uint32 shutter)
{
	kal_uint32 frame_length = 0,line_length=0;
    kal_uint32 extra_lines = 0;
	kal_uint32 max_exp_shutter = 0;
	unsigned long flags;
	kal_uint32 line_length_read;

	S5K4H5YXDB("[S5K4H5YX] [S5K4H5YX_write_shutter] shutter=%d\n", shutter);

  if (shutter < 3)
	  shutter = 3;

  if (s5k4h5yx.sensorMode == SENSOR_MODE_PREVIEW) 
  {
  	S5K4H5YXDB("[S5K4H5YX] [S5K4H5YX_write_shutter] SENSOR_MODE_PREVIEW \n");
	  if(shutter > (cont_preview_frame_length - 16))
		  frame_length = shutter + 16;
	  else 
		  frame_length = cont_preview_frame_length;

  }
  else if(s5k4h5yx.sensorMode==SENSOR_MODE_VIDEO)
  {
  	S5K4H5YXDB("[S5K4H5YX] [S5K4H5YX_write_shutter] SENSOR_MODE_VIDEO \n");
  	   frame_length = s5k4h5yx_video_frame_length;
	   if(shutter > (frame_length - 16))
	   	shutter = frame_length - 16;
  }
  else
  {
  	S5K4H5YXDB("[S5K4H5YX] [S5K4H5YX_write_shutter] SENSOR_MODE_CAPTURE \n");
	  if(shutter > (cont_capture_frame_length - 16))
		  frame_length = shutter + 16;
	  else 
		  frame_length = cont_capture_frame_length;
  }
  
 	S5K4H5YX_write_cmos_sensor(0x0104, 0x01);    //Grouped parameter hold    

	S5K4H5YX_write_cmos_sensor(0x0340, (frame_length >>8) & 0xFF);
 	S5K4H5YX_write_cmos_sensor(0x0341, frame_length & 0xFF);	 

 	S5K4H5YX_write_cmos_sensor(0x0202, (shutter >> 8) & 0xFF);
 	S5K4H5YX_write_cmos_sensor(0x0203, shutter  & 0xFF);
 
 	S5K4H5YX_write_cmos_sensor(0x0104, 0x00);    //Grouped parameter release

	line_length_read = ((S5K4H5YX_read_cmos_sensor(0x0342)<<8)+S5K4H5YX_read_cmos_sensor(0x0343));

	S5K4H5YXDB("[S5K4H5YX] [S5K4H5YX_write_shutter] shutter=%d,  line_length_read=%d, frame_length=%d\n", shutter, line_length_read, frame_length);
}   /* write_S5K4H5YX_shutter */


void write_S5K4H5YX_gain(kal_uint16 gain)
{
	S5K4H5YXDB("[S5K4H5YX] [write_S5K4H5YX_gain] gain=%d\n", gain);
	S5K4H5YX_write_cmos_sensor(0x0104, 0x01);	
	S5K4H5YX_write_cmos_sensor(0x0204,(gain>>8));
	S5K4H5YX_write_cmos_sensor(0x0205,(gain&0xff));
	S5K4H5YX_write_cmos_sensor(0x0104, 0x00);
	return;
}

/*************************************************************************
* FUNCTION
*    S5K4H5YX_SetGain
*
* DESCRIPTION
*    This function is to set global gain to sensor.
*
* PARAMETERS
*    gain : sensor global gain(base: 0x40)
*
* RETURNS
*    the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
void S5K4H5YX_SetGain(UINT16 iGain)
{
	unsigned long flags;
	spin_lock_irqsave(&s5k4h5yxmipiraw_drv_lock,flags);
	s5k4h5yx.realGain = iGain;
	spin_unlock_irqrestore(&s5k4h5yxmipiraw_drv_lock,flags);
	S5K4H5YXDB("[S5K4H5YX] [S5K4H5YX_SetGain] gain=%d\n", iGain);
	write_S5K4H5YX_gain(iGain);
}   /*  S5K4H5YX_SetGain_SetGain  */


/*************************************************************************
* FUNCTION
*    read_S5K4H5YX_gain
*
* DESCRIPTION
*    This function is to set global gain to sensor.
*
* PARAMETERS
*    None
*
* RETURNS
*    gain : sensor global gain
*
* GLOBALS AFFECTED
*
*************************************************************************/
kal_uint16 read_S5K4H5YX_gain(void)
{
	kal_uint16 read_gain=0;

	read_gain=((S5K4H5YX_read_cmos_sensor(0x0204) << 8) | S5K4H5YX_read_cmos_sensor(0x0205));
	S5K4H5YXDB("[S5K4H5YX] [read_S5K4H5YX_gain] gain=%d\n", read_gain);
	spin_lock(&s5k4h5yxmipiraw_drv_lock);
	s5k4h5yx.sensorGlobalGain = read_gain;
	spin_unlock(&s5k4h5yxmipiraw_drv_lock);
	return s5k4h5yx.sensorGlobalGain;
}  /* read_S5K4H5YX_gain */


void S5K4H5YX_camera_para_to_sensor(void)
{
    kal_uint32    i;
    for(i=0; 0xFFFFFFFF!=S5K4H5YXSensorReg[i].Addr; i++)
    {
        S5K4H5YX_write_cmos_sensor(S5K4H5YXSensorReg[i].Addr, S5K4H5YXSensorReg[i].Para);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=S5K4H5YXSensorReg[i].Addr; i++)
    {
        S5K4H5YX_write_cmos_sensor(S5K4H5YXSensorReg[i].Addr, S5K4H5YXSensorReg[i].Para);
    }
    for(i=FACTORY_START_ADDR; i<FACTORY_END_ADDR; i++)
    {
        S5K4H5YX_write_cmos_sensor(S5K4H5YXSensorCCT[i].Addr, S5K4H5YXSensorCCT[i].Para);
    }
}


/*************************************************************************
* FUNCTION
*    S5K4H5YX_sensor_to_camera_para
*
* DESCRIPTION
*    // update camera_para from sensor register
*
* PARAMETERS
*    None
*
* RETURNS
*    gain : sensor global gain(base: 0x40)
*
* GLOBALS AFFECTED
*
*************************************************************************/
void S5K4H5YX_sensor_to_camera_para(void)
{
    kal_uint32    i, temp_data;
    for(i=0; 0xFFFFFFFF!=S5K4H5YXSensorReg[i].Addr; i++)
    {
         temp_data = S5K4H5YX_read_cmos_sensor(S5K4H5YXSensorReg[i].Addr);
		 spin_lock(&s5k4h5yxmipiraw_drv_lock);
		 S5K4H5YXSensorReg[i].Para =temp_data;
		 spin_unlock(&s5k4h5yxmipiraw_drv_lock);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=S5K4H5YXSensorReg[i].Addr; i++)
    {
        temp_data = S5K4H5YX_read_cmos_sensor(S5K4H5YXSensorReg[i].Addr);
		spin_lock(&s5k4h5yxmipiraw_drv_lock);
		S5K4H5YXSensorReg[i].Para = temp_data;
		spin_unlock(&s5k4h5yxmipiraw_drv_lock);
    }
}

/*************************************************************************
* FUNCTION
*    S5K4H5YX_get_sensor_group_count
*
* DESCRIPTION
*    //
*
* PARAMETERS
*    None
*
* RETURNS
*    gain : sensor global gain(base: 0x40)
*
* GLOBALS AFFECTED
*
*************************************************************************/
kal_int32  S5K4H5YX_get_sensor_group_count(void)
{
    return GROUP_TOTAL_NUMS;
}

void S5K4H5YX_get_sensor_group_info(kal_uint16 group_idx, kal_int8* group_name_ptr, kal_int32* item_count_ptr)
{
   switch (group_idx)
   {
        case PRE_GAIN:
            sprintf((char *)group_name_ptr, "CCT");
            *item_count_ptr = 2;
            break;
        case CMMCLK_CURRENT:
            sprintf((char *)group_name_ptr, "CMMCLK Current");
            *item_count_ptr = 1;
            break;
        case FRAME_RATE_LIMITATION:
            sprintf((char *)group_name_ptr, "Frame Rate Limitation");
            *item_count_ptr = 2;
            break;
        case REGISTER_EDITOR:
            sprintf((char *)group_name_ptr, "Register Editor");
            *item_count_ptr = 2;
            break;
        default:
            ASSERT(0);
	}
}

void S5K4H5YX_get_sensor_item_info(kal_uint16 group_idx,kal_uint16 item_idx, MSDK_SENSOR_ITEM_INFO_STRUCT* info_ptr)
{
    kal_int16 temp_reg=0;
    kal_uint16 temp_gain=0, temp_addr=0, temp_para=0;

    switch (group_idx)
    {
        case PRE_GAIN:
           switch (item_idx)
          {
              case 0:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-R");
                  temp_addr = PRE_GAIN_R_INDEX;
              break;
              case 1:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-Gr");
                  temp_addr = PRE_GAIN_Gr_INDEX;
              break;
              case 2:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-Gb");
                  temp_addr = PRE_GAIN_Gb_INDEX;
              break;
              case 3:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-B");
                  temp_addr = PRE_GAIN_B_INDEX;
              break;
              case 4:
                 sprintf((char *)info_ptr->ItemNamePtr,"SENSOR_BASEGAIN");
                 temp_addr = SENSOR_BASEGAIN;
              break;
              default:
                 ASSERT(0);
          }

            temp_para= S5K4H5YXSensorCCT[temp_addr].Para;
			//temp_gain= (temp_para/s5k4h5yx.sensorBaseGain) * 1000;

            info_ptr->ItemValue=temp_gain;
            info_ptr->IsTrueFalse=KAL_FALSE;
            info_ptr->IsReadOnly=KAL_FALSE;
            info_ptr->IsNeedRestart=KAL_FALSE;
            info_ptr->Min= S5K4H5YX_MIN_ANALOG_GAIN * 1000;
            info_ptr->Max= S5K4H5YX_MAX_ANALOG_GAIN * 1000;
            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Drv Cur[2,4,6,8]mA");

                    //temp_reg=MT9P017SensorReg[CMMCLK_CURRENT_INDEX].Para;
                    temp_reg = ISP_DRIVING_2MA;
                    if(temp_reg==ISP_DRIVING_2MA)
                    {
                        info_ptr->ItemValue=2;
                    }
                    else if(temp_reg==ISP_DRIVING_4MA)
                    {
                        info_ptr->ItemValue=4;
                    }
                    else if(temp_reg==ISP_DRIVING_6MA)
                    {
                        info_ptr->ItemValue=6;
                    }
                    else if(temp_reg==ISP_DRIVING_8MA)
                    {
                        info_ptr->ItemValue=8;
                    }

                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_TRUE;
                    info_ptr->Min=2;
                    info_ptr->Max=8;
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case FRAME_RATE_LIMITATION:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Max Exposure Lines");
                    info_ptr->ItemValue=    111;  //MT9P017_MAX_EXPOSURE_LINES;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_TRUE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0;
                    break;
                case 1:
                    sprintf((char *)info_ptr->ItemNamePtr,"Min Frame Rate");
                    info_ptr->ItemValue=12;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_TRUE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0;
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case REGISTER_EDITOR:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"REG Addr.");
                    info_ptr->ItemValue=0;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0xFFFF;
                    break;
                case 1:
                    sprintf((char *)info_ptr->ItemNamePtr,"REG Value");
                    info_ptr->ItemValue=0;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0xFFFF;
                    break;
                default:
                ASSERT(0);
            }
            break;
        default:
            ASSERT(0);
    }
}



kal_bool S5K4H5YX_set_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, kal_int32 ItemValue)
{
//   kal_int16 temp_reg;
   kal_uint16  temp_gain=0,temp_addr=0, temp_para=0;

   switch (group_idx)
    {
        case PRE_GAIN:
            switch (item_idx)
            {
              case 0:
                temp_addr = PRE_GAIN_R_INDEX;
              break;
              case 1:
                temp_addr = PRE_GAIN_Gr_INDEX;
              break;
              case 2:
                temp_addr = PRE_GAIN_Gb_INDEX;
              break;
              case 3:
                temp_addr = PRE_GAIN_B_INDEX;
              break;
              case 4:
                temp_addr = SENSOR_BASEGAIN;
              break;
              default:
                 ASSERT(0);
          }

		 temp_gain=((ItemValue*BASEGAIN+500)/1000);			//+500:get closed integer value

		  if(temp_gain>=1*BASEGAIN && temp_gain<=16*BASEGAIN)
          {
//             temp_para=(temp_gain * s5k4h5yx.sensorBaseGain + BASEGAIN/2)/BASEGAIN;
          }
          else
			  ASSERT(0);
		  spin_lock(&s5k4h5yxmipiraw_drv_lock);
          S5K4H5YXSensorCCT[temp_addr].Para = temp_para;
		  spin_unlock(&s5k4h5yxmipiraw_drv_lock);
          S5K4H5YX_write_cmos_sensor(S5K4H5YXSensorCCT[temp_addr].Addr,temp_para);

            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    //no need to apply this item for driving current
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case FRAME_RATE_LIMITATION:
            ASSERT(0);
            break;
        case REGISTER_EDITOR:
            switch (item_idx)
            {
                case 0:
					spin_lock(&s5k4h5yxmipiraw_drv_lock);
                    S5K4H5YX_FAC_SENSOR_REG=ItemValue;
					spin_unlock(&s5k4h5yxmipiraw_drv_lock);
                    break;
                case 1:
                    S5K4H5YX_write_cmos_sensor(S5K4H5YX_FAC_SENSOR_REG,ItemValue);
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

static void S5K4H5YX_SetDummy( const kal_uint32 iPixels, const kal_uint32 iLines )
{
kal_uint16 line_length = 0;
kal_uint16 frame_length = 0;

S5K4H5YXDB("[S5K4H5YX] [S5K4H5YX_SetDummy] iPixels=%d\n", iPixels);
S5K4H5YXDB("[S5K4H5YX] [S5K4H5YX_SetDummy] iLines=%d\n", iLines);

if ( SENSOR_MODE_PREVIEW == s5k4h5yx.sensorMode )	
{
	S5K4H5YXDB("[S5K4H5YX] [S5K4H5YX_SetDummy] SENSOR_MODE_PREVIEW\n");
	line_length = s5k4h5yx_preview_line_length ;
	frame_length = s5k4h5yx_preview_frame_length + iLines;
}
else if( SENSOR_MODE_VIDEO == s5k4h5yx.sensorMode )		
{
	S5K4H5YXDB("[S5K4H5YX] [S5K4H5YX_SetDummy] SENSOR_MODE_VIDEO\n");
	line_length = s5k4h5yx_video_line_length;
	frame_length = s5k4h5yx_video_frame_length;
}
else
{
	S5K4H5YXDB("[S5K4H5YX] [S5K4H5YX_SetDummy] SENSOR_MODE_CAPTURE\n");
	line_length = s5k4h5yx_capture_line_length ;
	frame_length = s5k4h5yx_capture_frame_length + iLines;
}

if(s5k4h5yx.maxExposureLines > frame_length - 16)
{
	S5K4H5YXDB("[S5K4H5YX] [S5K4H5YX_SetDummy] maxExposureLines > frame_length - 16\n");
	return;
}

ASSERT(line_length < S5K4H5YX_MAX_LINE_LENGTH); 	//0xCCCC
ASSERT(frame_length < S5K4H5YX_MAX_FRAME_LENGTH);	//0xFFFF

S5K4H5YX_write_cmos_sensor(0x0104, 0x01);	//Grouped parameter hold

//Set total frame length
S5K4H5YXDB("[S5K4H5YX] [S5K4H5YX_SetDummy] frame_length = %d\n", frame_length);

S5K4H5YX_write_cmos_sensor(0x0340, (frame_length >> 8) & 0xFF);
S5K4H5YX_write_cmos_sensor(0x0341, frame_length & 0xFF);

//Set total line length
S5K4H5YXDB("[S5K4H5YX] [S5K4H5YX_SetDummy] line_length = %d\n", line_length);

S5K4H5YX_write_cmos_sensor(0x0342, (line_length >> 8) & 0xFF);
S5K4H5YX_write_cmos_sensor(0x0343, line_length & 0xFF);

S5K4H5YX_write_cmos_sensor(0x0104, 0x00);	//Grouped parameter release
}   /*  S5K4H5YX_SetDummy */

void S5K4H5YXPreviewSetting(void)
{
   S5K4H5YXDB("[S5K4H5YX] [S5K4H5YXPreviewSetting]\n");
   S5K4H5YX_write_cmos_sensor(0x0100, 0x00);	   //  [0] mode_select
    msleep(25);
   S5K4H5YX_write_cmos_sensor(0x0101, 0x00);	   //  [1:0]   image_orientation ([0] mirror en, [1] flip en)
   S5K4H5YX_write_cmos_sensor(0x0204, 0x00);	   //  [15:8]  analogue_gain_code_global H
   S5K4H5YX_write_cmos_sensor(0x0205, 0x20);	   //  [7:0]   analogue_gain_code_global L
   S5K4H5YX_write_cmos_sensor(0x0200, 0x0D);	   //  [15:8]  fine_integration_time H
   S5K4H5YX_write_cmos_sensor(0x0201, 0x78);	   //  [7:0]   fine_integration_time L
   S5K4H5YX_write_cmos_sensor(0x0202, 0x04);	   //  [15:8]  coarse_integration_time H
   S5K4H5YX_write_cmos_sensor(0x0203, 0xE2);	   //  [7:0]   coarse_integration_time L
   S5K4H5YX_write_cmos_sensor(0x0340, 0x08);//04	   //  [15:8]  frame_length_lines H
   S5K4H5YX_write_cmos_sensor(0x0341, 0xD4);//EE	   //  [7:0]   frame_length_lines L
   S5K4H5YX_write_cmos_sensor(0x0342, 0x0E);	   //  [15:8]  line_length_pck H
   S5K4H5YX_write_cmos_sensor(0x0343, 0x68);	   //  [7:0]   line_length_pck L
   S5K4H5YX_write_cmos_sensor(0x0344, 0x00);	   //  [11:8]  x_addr_start H
   S5K4H5YX_write_cmos_sensor(0x0345, 0x08);	   //  [7:0]   x_addr_start L
   S5K4H5YX_write_cmos_sensor(0x0346, 0x00);	   //  [11:8]  y_addr_start H
   S5K4H5YX_write_cmos_sensor(0x0347, 0x06);	   //  [7:0]   y_addr_start L
   S5K4H5YX_write_cmos_sensor(0x0348, 0x0C);	   //  [11:8]  x_addr_end H
   S5K4H5YX_write_cmos_sensor(0x0349, 0xC7);	   //  [7:0]   x_addr_end L
   S5K4H5YX_write_cmos_sensor(0x034A, 0x09);	   //  [11:8]  y_addr_end H
   S5K4H5YX_write_cmos_sensor(0x034B, 0x99);	   //  [7:0]   y_addr_end L
   S5K4H5YX_write_cmos_sensor(0x034C, 0x06);	   //  [11:8]  x_output_size H
   S5K4H5YX_write_cmos_sensor(0x034D, 0x60);	   //  [7:0]   x_output_size L
   S5K4H5YX_write_cmos_sensor(0x034E, 0x04);	   //  [11:8]  y_output_size H
   S5K4H5YX_write_cmos_sensor(0x034F, 0xCA);	   //  [7:0]   y_output_size L
   S5K4H5YX_write_cmos_sensor(0x0390, 0x01);	   //  [7:0]   binning_mode ([0] binning enable)
   S5K4H5YX_write_cmos_sensor(0x0391, 0x22);	   //  [7:0]   binning_type (22h : 2x2 binning, 44h : 4x4 binning)
   S5K4H5YX_write_cmos_sensor(0x0381, 0x01);	   //  [4:0]   x_even_inc
   S5K4H5YX_write_cmos_sensor(0x0383, 0x03);	   //  [4:0]   x_odd_inc
   S5K4H5YX_write_cmos_sensor(0x0385, 0x01);	   //  [4:0]   y_even_inc
   S5K4H5YX_write_cmos_sensor(0x0387, 0x03);	   //  [4:0]   y_odd_inc
   S5K4H5YX_write_cmos_sensor(0x0301, 0x02);   //  [3:0]   vt_pix_clk_div						
   S5K4H5YX_write_cmos_sensor(0x0303, 0x01);   //  [3:0]   vt_sys_clk_div						
   S5K4H5YX_write_cmos_sensor(0x0305, 0x06);   //  [5:0]   pre_pll_clk_div						
   S5K4H5YX_write_cmos_sensor(0x0306, 0x00);   //  [9:8]   pll_multiplier H 					
   S5K4H5YX_write_cmos_sensor(0x0307, 0x7D);   //  [7:0]   pll_multiplier L 					
   S5K4H5YX_write_cmos_sensor(0x0309, 0x02);   //  [3:0]   op_pix_clk_div						
   S5K4H5YX_write_cmos_sensor(0x030B, 0x01);   //  [3:0]   op_sys_clk_div						
   S5K4H5YX_write_cmos_sensor(0x3C59, 0x00);   //  [2:0]   reg_PLL_S							
   S5K4H5YX_write_cmos_sensor(0x030D, 0x06);   //  [5:0]   out_pre_pll_clk_div					
   S5K4H5YX_write_cmos_sensor(0x030E, 0x00);   //  [9:8]   out_pll_multiplier H 				
   S5K4H5YX_write_cmos_sensor(0x030F, 0xA8);   //A5 	   //  [7:0]   out_pll_multiplier L 
   S5K4H5YX_write_cmos_sensor(0x3C5A, 0x00);   //  [2:0]   reg_out_PLL_S						
   S5K4H5YX_write_cmos_sensor(0x0310, 0x01);   //  [0] pll_mode (01h : 2-PLL, 00h : 1-PLL)	
   S5K4H5YX_write_cmos_sensor(0x3C50, 0x53);   //  [7:4]   reg_DIV_DBR							 
   S5K4H5YX_write_cmos_sensor(0x3C62, 0x02);   //  [31:24] requested_link_bit_rate_mbps HH		
   S5K4H5YX_write_cmos_sensor(0x3C63, 0xA0);   //  [23:16] requested_link_bit_rate_mbps HL		
   S5K4H5YX_write_cmos_sensor(0x3C64, 0x00);   //  [15:8]  requested_link_bit_rate_mbps LH		
   S5K4H5YX_write_cmos_sensor(0x3C65, 0x00);   //  [7:0]   requested_link_bit_rate_mbps LL	
   S5K4H5YX_write_cmos_sensor(0x3C1E, 0x00);	   //  [3] reg_isp_fe_TN_SMIA_sync_se
   S5K4H5YX_write_cmos_sensor(0x0202, 0x00);
   S5K4H5YX_write_cmos_sensor(0x302A, 0x0A);
   S5K4H5YX_write_cmos_sensor(0x303D, 0x06);
   S5K4H5YX_write_cmos_sensor(0x304B, 0x2A);
   S5K4H5YX_write_cmos_sensor(0x0204, 0x02);
   S5K4H5YX_write_cmos_sensor(0x0205, 0x00);
   S5K4H5YX_write_cmos_sensor(0x3205, 0x84);
   S5K4H5YX_write_cmos_sensor(0x3207, 0x85);
   S5K4H5YX_write_cmos_sensor(0x3214, 0x94);
   S5K4H5YX_write_cmos_sensor(0x3216, 0x95);
   S5K4H5YX_write_cmos_sensor(0x303a, 0x9f);
   S5K4H5YX_write_cmos_sensor(0x3201, 0x07);
   S5K4H5YX_write_cmos_sensor(0x3051, 0xff);
   S5K4H5YX_write_cmos_sensor(0x3052, 0xff);
   S5K4H5YX_write_cmos_sensor(0x3054, 0xF0);
   S5K4H5YX_write_cmos_sensor(0x305C, 0x8F);
   S5K4H5YX_write_cmos_sensor(0x302D, 0x7F);
   S5K4H5YX_write_cmos_sensor(0x3903, 0x1F);
   S5K4H5YX_write_cmos_sensor(0x0100, 0x01);	   //  [0] mode_select
}

	
void S5K4H5YXVideoSetting(void)
{
    S5K4H5YXDB("[S5K4H5YX] [S5K4H5YXVideoSetting]\n");
    S5K4H5YX_write_cmos_sensor(0x0100, 0x00);		//	[0] mode_select
    msleep(25);
    S5K4H5YX_write_cmos_sensor(0x0101, 0x00);		//	[1:0]	image_orientation ([0] mirror en, [1] flip en)
    S5K4H5YX_write_cmos_sensor(0x0204, 0x00);		//	[15:8]	analogue_gain_code_global H
    S5K4H5YX_write_cmos_sensor(0x0205, 0x20);		//	[7:0]	analogue_gain_code_global L
    S5K4H5YX_write_cmos_sensor(0x0200, 0x0D);		//	[15:8]	fine_integration_time H
    S5K4H5YX_write_cmos_sensor(0x0201, 0x78);		//	[7:0]	fine_integration_time L
    S5K4H5YX_write_cmos_sensor(0x0202, 0x04);		//	[15:8]	coarse_integration_time H
    S5K4H5YX_write_cmos_sensor(0x0203, 0xE2);		//	[7:0]	coarse_integration_time L
    S5K4H5YX_write_cmos_sensor(0x0340, 0x08);		//	[15:8]	frame_length_lines H
    S5K4H5YX_write_cmos_sensor(0x0341, 0x0C);//E2		//	[7:0]	frame_length_lines L
    S5K4H5YX_write_cmos_sensor(0x0342, 0x0F);		//	[15:8]	line_length_pck H
    S5K4H5YX_write_cmos_sensor(0x0343, 0xCC);		//	[7:0]	line_length_pck L
    S5K4H5YX_write_cmos_sensor(0x0344, 0x00);		//	[11:8]	x_addr_start H
    S5K4H5YX_write_cmos_sensor(0x0345, 0x08);		//	[7:0]	x_addr_start L
    S5K4H5YX_write_cmos_sensor(0x0346, 0x01);		//	[11:8]	y_addr_start H
    S5K4H5YX_write_cmos_sensor(0x0347, 0x3A);		//	[7:0]	y_addr_start L
    S5K4H5YX_write_cmos_sensor(0x0348, 0x0C);		//	[11:8]	x_addr_end H
    S5K4H5YX_write_cmos_sensor(0x0349, 0xC7);		//	[7:0]	x_addr_end L
    S5K4H5YX_write_cmos_sensor(0x034A, 0x08);		//	[11:8]	y_addr_end H
    S5K4H5YX_write_cmos_sensor(0x034B, 0x65);		//	[7:0]	y_addr_end L
    S5K4H5YX_write_cmos_sensor(0x034C, 0x0C);		//	[11:8]	x_output_size H
    S5K4H5YX_write_cmos_sensor(0x034D, 0xC0);		//	[7:0]	x_output_size L
    S5K4H5YX_write_cmos_sensor(0x034E, 0x07);		//	[11:8]	y_output_size H
    S5K4H5YX_write_cmos_sensor(0x034F, 0x2C);		//	[7:0]	y_output_size L
    S5K4H5YX_write_cmos_sensor(0x0390, 0x00);		//	[7:0]	binning_mode ([0] binning enable)
    S5K4H5YX_write_cmos_sensor(0x0391, 0x00);		//	[7:0]	binning_type (22h : 2x2 binning, 44h : 4x4 binning)
    S5K4H5YX_write_cmos_sensor(0x0381, 0x01);		//	[4:0]	x_even_inc
    S5K4H5YX_write_cmos_sensor(0x0383, 0x01);		//	[4:0]	x_odd_inc
    S5K4H5YX_write_cmos_sensor(0x0385, 0x01);		//	[4:0]	y_even_inc
    S5K4H5YX_write_cmos_sensor(0x0387, 0x01);		//	[4:0]	y_odd_inc
    S5K4H5YX_write_cmos_sensor(0x0301, 0x02);		//	[3:0]	vt_pix_clk_div						
    S5K4H5YX_write_cmos_sensor(0x0303, 0x01);		//	[3:0]	vt_sys_clk_div						
    S5K4H5YX_write_cmos_sensor(0x0305, 0x06);		//	[5:0]	pre_pll_clk_div 					
    S5K4H5YX_write_cmos_sensor(0x0306, 0x00);		//	[9:8]	pll_multiplier H					
    S5K4H5YX_write_cmos_sensor(0x0307, 0x7D);//8C		//	[7:0]	pll_multiplier L					
    S5K4H5YX_write_cmos_sensor(0x0309, 0x02);		//	[3:0]	op_pix_clk_div						
    S5K4H5YX_write_cmos_sensor(0x030B, 0x01);		//	[3:0]	op_sys_clk_div						
    S5K4H5YX_write_cmos_sensor(0x3C59, 0x00);		//	[2:0]	reg_PLL_S							
    S5K4H5YX_write_cmos_sensor(0x030D, 0x06);		//	[5:0]	out_pre_pll_clk_div 				
    S5K4H5YX_write_cmos_sensor(0x030E, 0x00);		//	[9:8]	out_pll_multiplier H				
    S5K4H5YX_write_cmos_sensor(0x030F, 0xA8);		//A5		//	[7:0]	out_pll_multiplier L
    S5K4H5YX_write_cmos_sensor(0x3C5A, 0x00);		//	[2:0]	reg_out_PLL_S						
    S5K4H5YX_write_cmos_sensor(0x0310, 0x01);		//	[0] pll_mode (01h : 2-PLL, 00h : 1-PLL) 
    S5K4H5YX_write_cmos_sensor(0x3C50, 0x53);		//	[7:4]	reg_DIV_DBR 						 
    S5K4H5YX_write_cmos_sensor(0x3C62, 0x02);		//	[31:24] requested_link_bit_rate_mbps HH 	
    S5K4H5YX_write_cmos_sensor(0x3C63, 0xA0);		//	[23:16] requested_link_bit_rate_mbps HL 	
    S5K4H5YX_write_cmos_sensor(0x3C64, 0x00);		//	[15:8]	requested_link_bit_rate_mbps LH 	
    S5K4H5YX_write_cmos_sensor(0x3C65, 0x00);		//	[7:0]	requested_link_bit_rate_mbps LL 	
    S5K4H5YX_write_cmos_sensor(0x3C1E, 0x00);		//	[3] reg_isp_fe_TN_SMIA_sync_se
    S5K4H5YX_write_cmos_sensor(0x0202, 0x00);
    S5K4H5YX_write_cmos_sensor(0x302A, 0x0A);
    S5K4H5YX_write_cmos_sensor(0x303D, 0x06);
    S5K4H5YX_write_cmos_sensor(0x304B, 0x2A);
    S5K4H5YX_write_cmos_sensor(0x0204, 0x02);
    S5K4H5YX_write_cmos_sensor(0x0205, 0x00);
    S5K4H5YX_write_cmos_sensor(0x3205, 0x84);
    S5K4H5YX_write_cmos_sensor(0x3207, 0x85);
    S5K4H5YX_write_cmos_sensor(0x3214, 0x94);
    S5K4H5YX_write_cmos_sensor(0x3216, 0x95);
    S5K4H5YX_write_cmos_sensor(0x303a, 0x9f);
    S5K4H5YX_write_cmos_sensor(0x3201, 0x07);
    S5K4H5YX_write_cmos_sensor(0x3051, 0xff);
    S5K4H5YX_write_cmos_sensor(0x3052, 0xff);
    S5K4H5YX_write_cmos_sensor(0x3054, 0xF0);
    S5K4H5YX_write_cmos_sensor(0x305C, 0x8F);
    S5K4H5YX_write_cmos_sensor(0x302D, 0x7F);	  
    S5K4H5YX_write_cmos_sensor(0x3903, 0x1F);
    S5K4H5YX_write_cmos_sensor(0x0100, 0x01);		//	[0] mode_select

}

void S5K4H5YXCaptureSetting(void)
{
    S5K4H5YXDB("[S5K4H5YX] [S5K4H5YXCaptureSetting]\n");
    S5K4H5YX_write_cmos_sensor(0x0100, 0x00);		//	[0]	mode_select
    msleep(25);
    S5K4H5YX_write_cmos_sensor(0x0101, 0x00);		//	[1:0]	image_orientation ([0] mirror en, [1] flip en)
    S5K4H5YX_write_cmos_sensor(0x0204, 0x00);		//	[15:8]	analogue_gain_code_global H
    S5K4H5YX_write_cmos_sensor(0x0205, 0x20);		//	[7:0]	analogue_gain_code_global L
    S5K4H5YX_write_cmos_sensor(0x0200, 0x0D);		//	[15:8]	fine_integration_time H
    S5K4H5YX_write_cmos_sensor(0x0201, 0x78);		//	[7:0]	fine_integration_time L
    S5K4H5YX_write_cmos_sensor(0x0202, 0x04);		//	[15:8]	coarse_integration_time H
    S5K4H5YX_write_cmos_sensor(0x0203, 0xE2);		//	[7:0]	coarse_integration_time L
    S5K4H5YX_write_cmos_sensor(0x0340, 0x09);		//	[15:8]	frame_length_lines H
    S5K4H5YX_write_cmos_sensor(0x0341, 0xB6);		//	[7:0]	frame_length_lines L
    S5K4H5YX_write_cmos_sensor(0x0342, 0x0F);		//	[15:8]	line_length_pck H
    S5K4H5YX_write_cmos_sensor(0x0343, 0xCC);		//	[7:0]	line_length_pck L
    S5K4H5YX_write_cmos_sensor(0x0344, 0x00);		//	[11:8]	x_addr_start H
    S5K4H5YX_write_cmos_sensor(0x0345, 0x08);		//	[7:0]	x_addr_start L
    S5K4H5YX_write_cmos_sensor(0x0346, 0x00);		//	[11:8]	y_addr_start H
    S5K4H5YX_write_cmos_sensor(0x0347, 0x06);		//	[7:0]	y_addr_start L
    S5K4H5YX_write_cmos_sensor(0x0348, 0x0C);		//	[11:8]	x_addr_end H
    S5K4H5YX_write_cmos_sensor(0x0349, 0xC7);		//	[7:0]	x_addr_end L
    S5K4H5YX_write_cmos_sensor(0x034A, 0x09);		//	[11:8]	y_addr_end H
    S5K4H5YX_write_cmos_sensor(0x034B, 0x99);		//	[7:0]	y_addr_end L
    S5K4H5YX_write_cmos_sensor(0x034C, 0x0C);		//	[11:8]	x_output_size H
    S5K4H5YX_write_cmos_sensor(0x034D, 0xC0);		//	[7:0]	x_output_size L
    S5K4H5YX_write_cmos_sensor(0x034E, 0x09);		//	[11:8]	y_output_size H
    S5K4H5YX_write_cmos_sensor(0x034F, 0x94);		//	[7:0]	y_output_size L
    S5K4H5YX_write_cmos_sensor(0x0390, 0x00);		//	[7:0]	binning_mode ([0] binning enable)
    S5K4H5YX_write_cmos_sensor(0x0391, 0x00);		//	[7:0]	binning_type (22h : 2x2 binning, 44h : 4x4 binning)
    S5K4H5YX_write_cmos_sensor(0x0381, 0x01);		//	[4:0]	x_even_inc
    S5K4H5YX_write_cmos_sensor(0x0383, 0x01);		//	[4:0]	x_odd_inc
    S5K4H5YX_write_cmos_sensor(0x0385, 0x01);		//	[4:0]	y_even_inc
    S5K4H5YX_write_cmos_sensor(0x0387, 0x01);		//	[4:0]	y_odd_inc
    S5K4H5YX_write_cmos_sensor(0x0301, 0x02);		//	[3:0]	vt_pix_clk_div
    S5K4H5YX_write_cmos_sensor(0x0303, 0x01);		//	[3:0]	vt_sys_clk_div
    S5K4H5YX_write_cmos_sensor(0x0305, 0x06);		//	[5:0]	pre_pll_clk_div
    S5K4H5YX_write_cmos_sensor(0x0306, 0x00);		//	[9:8]	pll_multiplier H
    S5K4H5YX_write_cmos_sensor(0x0307, 0x7D);		//	[7:0]	pll_multiplier L
    S5K4H5YX_write_cmos_sensor(0x0309, 0x02);		//	[3:0]	op_pix_clk_div
    S5K4H5YX_write_cmos_sensor(0x030B, 0x01);		//	[3:0]	op_sys_clk_div
    S5K4H5YX_write_cmos_sensor(0x3C59, 0x00);		//	[2:0]	reg_PLL_S
    S5K4H5YX_write_cmos_sensor(0x030D, 0x06);		//	[5:0]	out_pre_pll_clk_div
    S5K4H5YX_write_cmos_sensor(0x030E, 0x00);		//	[9:8]	out_pll_multiplier H
    S5K4H5YX_write_cmos_sensor(0x030F, 0xA8);		//A5		//	[7:0]	out_pll_multiplier L
    S5K4H5YX_write_cmos_sensor(0x3C5A, 0x00);		//	[2:0]	reg_out_PLL_S
    S5K4H5YX_write_cmos_sensor(0x0310, 0x01);		//	[0]	pll_mode (01h : 2-PLL, 00h : 1-PLL)
    S5K4H5YX_write_cmos_sensor(0x3C50, 0x53);		//	[7:4]	reg_DIV_DBR
    S5K4H5YX_write_cmos_sensor(0x3C62, 0x02);		//	[31:24]	requested_link_bit_rate_mbps HH
    S5K4H5YX_write_cmos_sensor(0x3C63, 0xA0);		//	[23:16]	requested_link_bit_rate_mbps HL
    S5K4H5YX_write_cmos_sensor(0x3C64, 0x00);		//	[15:8]	requested_link_bit_rate_mbps LH
    S5K4H5YX_write_cmos_sensor(0x3C65, 0x00);		//	[7:0]	requested_link_bit_rate_mbps LL
    S5K4H5YX_write_cmos_sensor(0x3C1E, 0x00);		//	[3]	reg_isp_fe_TN_SMIA_sync_
    S5K4H5YX_write_cmos_sensor(0x0202, 0x00);
    S5K4H5YX_write_cmos_sensor(0x302A, 0x0A);
    S5K4H5YX_write_cmos_sensor(0x303D, 0x06);
    S5K4H5YX_write_cmos_sensor(0x304B, 0x2A);
    S5K4H5YX_write_cmos_sensor(0x0204, 0x02);
    S5K4H5YX_write_cmos_sensor(0x0205, 0x00);
    S5K4H5YX_write_cmos_sensor(0x3205, 0x84);
    S5K4H5YX_write_cmos_sensor(0x3207, 0x85);
    S5K4H5YX_write_cmos_sensor(0x3214, 0x94);
    S5K4H5YX_write_cmos_sensor(0x3216, 0x95);
    S5K4H5YX_write_cmos_sensor(0x303a, 0x9f);
    S5K4H5YX_write_cmos_sensor(0x3201, 0x07);
    S5K4H5YX_write_cmos_sensor(0x3051, 0xff);
    S5K4H5YX_write_cmos_sensor(0x3052, 0xff);
    S5K4H5YX_write_cmos_sensor(0x3054, 0xF0);
    S5K4H5YX_write_cmos_sensor(0x305C, 0x8F);
    S5K4H5YX_write_cmos_sensor(0x302D, 0x7F);
    S5K4H5YX_write_cmos_sensor(0x3903, 0x1F);
    S5K4H5YX_write_cmos_sensor(0x0100, 0x01);		//	[0]	mode_select
}

/*************************************************************************
* FUNCTION
*   S5K4H5YXOpen
*
* DESCRIPTION
*   This function initialize the registers of CMOS sensor
*
* PARAMETERS
*   None
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/

UINT32 S5K4H5YXOpen(void)
{

	volatile signed int i;
	kal_uint16 sensor_id = 0;

	S5K4H5YXDB("[S5K4H5YX] [S5K4H5YXOpen]\n");

	//  Read sensor ID to adjust I2C is OK?
	for(i=0;i<3;i++)
	{
		sensor_id = (S5K4H5YX_read_cmos_sensor(0x0000)<<8)|S5K4H5YX_read_cmos_sensor(0x0001);
		S5K4H5YXDB("[S5K4H5YX] [S5K4H5YXOpen] sensor_id=%x\n",sensor_id);
		if(sensor_id != S5K4H5YX_SENSOR_ID)
		{
			return ERROR_SENSOR_CONNECT_FAIL;
		}else
			break;
	}
	spin_lock(&s5k4h5yxmipiraw_drv_lock);
	s5k4h5yx.sensorMode = SENSOR_MODE_INIT;
	s5k4h5yx.S5K4H5YXAutoFlickerMode = KAL_FALSE;
	s5k4h5yx.S5K4H5YXVideoMode = KAL_FALSE;
	s5k4h5yx.DummyLines= 0;
	s5k4h5yx.DummyPixels= 0;

	s5k4h5yx.pvPclk =  (25000);  
	s5k4h5yx.videoPclk = (25000); 
	
	spin_unlock(&s5k4h5yxmipiraw_drv_lock);

    	switch(S5K4H5YXCurrentScenarioId)
		{
			case MSDK_SCENARIO_ID_CAMERA_ZSD:
				#if defined(ZSD15FPS)
				spin_lock(&s5k4h5yxmipiraw_drv_lock);
				s5k4h5yx.capPclk = (25000);
				spin_unlock(&s5k4h5yxmipiraw_drv_lock);
				#else
				spin_lock(&s5k4h5yxmipiraw_drv_lock);
				s5k4h5yx.capPclk = (25000);
				spin_unlock(&s5k4h5yxmipiraw_drv_lock);
				#endif
				break;
        	default:
				spin_lock(&s5k4h5yxmipiraw_drv_lock);
				s5k4h5yx.capPclk = (25000);
				spin_unlock(&s5k4h5yxmipiraw_drv_lock);
				break;
          }
	spin_lock(&s5k4h5yxmipiraw_drv_lock);
	s5k4h5yx.shutter = 0x4EA;
	s5k4h5yx.pvShutter = 0x4EA;
	s5k4h5yx.maxExposureLines =S5K4H5YX_PV_PERIOD_LINE_NUMS -4;

	s5k4h5yx.ispBaseGain = BASEGAIN;//0x40
	s5k4h5yx.sensorGlobalGain = 0x1f;//sensor gain read from 0x350a 0x350b; 0x1f as 3.875x
	s5k4h5yx.pvGain = 0x1f;
	spin_unlock(&s5k4h5yxmipiraw_drv_lock);
    return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*   S5K4H5YXGetSensorID
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
UINT32 S5K4H5YXGetSensorID(UINT32 *sensorID)
{
    int  retry = 1;

	S5K4H5YXDB("[S5K4H5YX] [S5K4H5YXGetSensorID]\n");
	//S5K4H5YX_write_cmos_sensor(0x0103,0x01);// Reset sensor
    //mDELAY(10);

    // check if sensor ID correct
    do {
        *sensorID = (S5K4H5YX_read_cmos_sensor(0x0000)<<8)|S5K4H5YX_read_cmos_sensor(0x0001);
        if (*sensorID == S5K4H5YX_SENSOR_ID)
        	{
        		S5K4H5YXDB("[S5K4H5YX] [S5K4H5YXGetSensorID] Sensor ID = 0x%04x\n", *sensorID);
            	break;
        	}
        S5K4H5YXDB("[S5K4H5YX] [S5K4H5YXGetSensorID] Read Sensor ID Fail = 0x%04x\n", *sensorID);
        retry--;
    } while (retry > 0);

    if (*sensorID != S5K4H5YX_SENSOR_ID) {
        *sensorID = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*   S5K4H5YX_SetShutter
*
* DESCRIPTION
*   This function set e-shutter of s5k4h5yx to change exposure time.
*
* PARAMETERS
*   shutter : exposured lines
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void S5K4H5YX_SetShutter(kal_uint32 iShutter)
{
unsigned long flags;
	S5K4H5YXDB("[S5K4H5YX] [S5K4H5YX_SetShutter] shutter = %d\n", iShutter);
spin_lock(&s5k4h5yxmipiraw_drv_lock);
s5k4h5yx.shutter= iShutter;
spin_unlock(&s5k4h5yxmipiraw_drv_lock);
S5K4H5YX_write_shutter(iShutter);
 return;
}   /*  S5K4H5YX_SetShutter   */



/*************************************************************************
* FUNCTION
*   S5K4H5YX_read_shutter
*
* DESCRIPTION
*   This function to  Get exposure time.
*
* PARAMETERS
*   None
*
* RETURNS
*   shutter : exposured lines
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 S5K4H5YX_read_shutter(void)
{

	kal_uint16 temp_reg1, temp_reg2 ,temp_reg3;
	UINT32 shutter =0;

	shutter = (S5K4H5YX_read_cmos_sensor(0x0202) << 8) | S5K4H5YX_read_cmos_sensor(0x0203);
	S5K4H5YXDB("[S5K4H5YX] [S5K4H5YX_read_shutter] shutter = %d\n", shutter);
	return shutter;
}

/*************************************************************************
* FUNCTION
*   S5K4H5YX_night_mode
*
* DESCRIPTION
*   This function night mode of s5k4h5yx.
*
* PARAMETERS
*   none
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void S5K4H5YX_NightMode(kal_bool bEnable)
{
}/*	S5K4H5YX_NightMode */



/*************************************************************************
* FUNCTION
*   S5K4H5YXClose
*
* DESCRIPTION
*   This function is to turn off sensor module power.
*
* PARAMETERS
*   None
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 S5K4H5YXClose(void)
{
    return ERROR_NONE;
}	/* S5K4H5YXClose() */

void S5K4H5YXSetFlipMirror(kal_int32 imgMirror)
{
	kal_int16 mirror=0,flip=0;
	S5K4H5YXDB("[S5K4H5YX] [S5K4H5YXSetFlipMirror] imgMirror = %d\n", imgMirror);
	    switch (imgMirror)
    {
        case IMAGE_NORMAL: //B
            S5K4H5YX_write_cmos_sensor(0x0101, 0x03);	//Set normal
            break;
        case IMAGE_V_MIRROR: //Gr X
            S5K4H5YX_write_cmos_sensor(0x0101, 0x01);	//Set flip
            break;
        case IMAGE_H_MIRROR: //Gb
            S5K4H5YX_write_cmos_sensor(0x0101, 0x02);	//Set mirror
            break;
        case IMAGE_HV_MIRROR: //R
            S5K4H5YX_write_cmos_sensor(0x0101, 0x00);	//Set mirror and flip
            break;
    }
}


/*************************************************************************
* FUNCTION
*   S5K4H5YXPreview
*
* DESCRIPTION
*   This function start the sensor preview.
*
* PARAMETERS
*   *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 S5K4H5YXPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

	S5K4H5YXDB("[S5K4H5YX] [S5K4H5YXPreview]\n");

	// preview size
	if(s5k4h5yx.sensorMode == SENSOR_MODE_PREVIEW)
	{
	}
	else
	{
		S5K4H5YXPreviewSetting();
	}
	spin_lock(&s5k4h5yxmipiraw_drv_lock);
	s5k4h5yx.sensorMode = SENSOR_MODE_PREVIEW; // Need set preview setting after capture mode
	s5k4h5yx.DummyPixels = 0;//define dummy pixels and lines
	s5k4h5yx.DummyLines = 0 ;
	cont_preview_line_length=s5k4h5yx_preview_line_length;
	cont_preview_frame_length=s5k4h5yx_preview_frame_length+s5k4h5yx.DummyLines;
	S5K4H5YX_FeatureControl_PERIOD_PixelNum = s5k4h5yx_preview_line_length;
	S5K4H5YX_FeatureControl_PERIOD_LineNum = cont_preview_frame_length;
	spin_unlock(&s5k4h5yxmipiraw_drv_lock);


	//set mirror & flip
	//S5K4H5YXDB("[S5K4H5YXPreview] mirror&flip: %d \n",sensor_config_data->SensorImageMirror);
	spin_lock(&s5k4h5yxmipiraw_drv_lock);
	s5k4h5yx.imgMirror = sensor_config_data->SensorImageMirror;
	spin_unlock(&s5k4h5yxmipiraw_drv_lock);
	//S5K4H5YXSetFlipMirror(sensor_config_data->SensorImageMirror);
	S5K4H5YXSetFlipMirror(IMAGE_HV_MIRROR);
    msleep(10);
    return ERROR_NONE;
}	/* S5K4H5YXPreview() */



UINT32 S5K4H5YXVideo(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

	S5K4H5YXDB("[S5K4H5YX] [S5K4H5YXVideo]\n");

	if(s5k4h5yx.sensorMode == SENSOR_MODE_VIDEO)
	{
		// do nothing
	}
	else
	{
		S5K4H5YXVideoSetting();

	}
	spin_lock(&s5k4h5yxmipiraw_drv_lock);
	s5k4h5yx.sensorMode = SENSOR_MODE_VIDEO;
	S5K4H5YX_FeatureControl_PERIOD_PixelNum = s5k4h5yx_video_line_length;
	S5K4H5YX_FeatureControl_PERIOD_LineNum = s5k4h5yx_video_frame_length;
	spin_unlock(&s5k4h5yxmipiraw_drv_lock);

	spin_lock(&s5k4h5yxmipiraw_drv_lock);
	s5k4h5yx.imgMirror = sensor_config_data->SensorImageMirror;
	spin_unlock(&s5k4h5yxmipiraw_drv_lock);
	//S5K4H5YXSetFlipMirror(sensor_config_data->SensorImageMirror);
	S5K4H5YXSetFlipMirror(IMAGE_HV_MIRROR);
    msleep(10);

    return ERROR_NONE;
}


UINT32 S5K4H5YXCapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

 	kal_uint32 shutter = s5k4h5yx.shutter;
	kal_uint32 temp_data;
	//kal_uint32 pv_line_length , cap_line_length,

	S5K4H5YXDB("[S5K4H5YX] [S5K4H5YXCapture]\n");

	if( SENSOR_MODE_CAPTURE== s5k4h5yx.sensorMode)
	{
		S5K4H5YXDB("[S5K4H5YX] [S5K4H5YXCapture] BusrtShot!!\n");
	}else{

	//Record Preview shutter & gain
	shutter=S5K4H5YX_read_shutter();
	temp_data =  read_S5K4H5YX_gain();
	spin_lock(&s5k4h5yxmipiraw_drv_lock);
	s5k4h5yx.pvShutter =shutter;
	s5k4h5yx.sensorGlobalGain = temp_data;
	s5k4h5yx.pvGain =s5k4h5yx.sensorGlobalGain;
	s5k4h5yx.sensorMode = SENSOR_MODE_CAPTURE;	
	spin_unlock(&s5k4h5yxmipiraw_drv_lock);

	S5K4H5YXDB("[S5K4H5YX] [S5K4H5YXCapture] s5k4h5yx.shutter=%d, read_pv_shutter=%d, read_pv_gain = 0x%x\n",s5k4h5yx.shutter, shutter,s5k4h5yx.sensorGlobalGain);

	// Full size setting
	S5K4H5YXCaptureSetting();

    //rewrite pixel number to Register ,for mt6589 line start/end;
	S5K4H5YX_SetDummy(s5k4h5yx.DummyPixels,s5k4h5yx.DummyLines);

	spin_lock(&s5k4h5yxmipiraw_drv_lock);

	s5k4h5yx.imgMirror = sensor_config_data->SensorImageMirror;
	s5k4h5yx.DummyPixels = 0;//define dummy pixels and lines
	s5k4h5yx.DummyLines = 0 ;
	cont_capture_line_length = s5k4h5yx_capture_line_length;
	cont_capture_frame_length = s5k4h5yx_capture_frame_length + s5k4h5yx.DummyLines;
	S5K4H5YX_FeatureControl_PERIOD_PixelNum = s5k4h5yx_capture_line_length;
	S5K4H5YX_FeatureControl_PERIOD_LineNum = cont_capture_frame_length;
	spin_unlock(&s5k4h5yxmipiraw_drv_lock);

	//S5K4H5YXDB("[S5K4H5YXCapture] mirror&flip: %d\n",sensor_config_data->SensorImageMirror);
	//S5K4H5YXSetFlipMirror(sensor_config_data->SensorImageMirror);
	S5K4H5YXSetFlipMirror(IMAGE_HV_MIRROR);

	//#if defined(MT6575)||defined(MT6577)
    if(S5K4H5YXCurrentScenarioId==MSDK_SCENARIO_ID_CAMERA_ZSD)
    {
		S5K4H5YXDB("[S5K4H5YX] [S5K4H5YXCapture] S5K4H5YXCapture exit ZSD!\n");
		return ERROR_NONE;
    }
	//#endif   
	}
    
    msleep(10);

    return ERROR_NONE;
}	/* S5K4H5YXCapture() */

UINT32 S5K4H5YXGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{

    S5K4H5YXDB("[S5K4H5YX] [S5K4H5YXGetResolution]\n");

	//1632*1226
	#ifdef Capture8M
	pSensorResolution->SensorPreviewWidth	= 1632;
    pSensorResolution->SensorPreviewHeight	= 1224;
	#else
	pSensorResolution->SensorPreviewWidth	= S5K4H5YX_IMAGE_SENSOR_PV_WIDTH;
    pSensorResolution->SensorPreviewHeight	= S5K4H5YX_IMAGE_SENSOR_PV_HEIGHT;
	#endif

	//3264*2452
	#ifdef Capture8M
	pSensorResolution->SensorFullWidth		= 3264;
    pSensorResolution->SensorFullHeight		= 2448;
	#else
    pSensorResolution->SensorFullWidth		= S5K4H5YX_IMAGE_SENSOR_FULL_WIDTH;
    pSensorResolution->SensorFullHeight		= S5K4H5YX_IMAGE_SENSOR_FULL_HEIGHT;
	#endif
	
	//3264*1836
    pSensorResolution->SensorVideoWidth		= S5K4H5YX_IMAGE_SENSOR_VIDEO_WIDTH;
    pSensorResolution->SensorVideoHeight    = S5K4H5YX_IMAGE_SENSOR_VIDEO_HEIGHT;
    return ERROR_NONE;
}   /* S5K4H5YXGetResolution() */

UINT32 S5K4H5YXGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
                                                MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	 S5K4H5YXDB("[S5K4H5YX] [S5K4H5YXGetInfo]\n");
	#ifdef Capture8M
	pSensorInfo->SensorPreviewResolutionX= 1632;
	pSensorInfo->SensorPreviewResolutionY= 1224;
	#else
	pSensorInfo->SensorPreviewResolutionX= S5K4H5YX_IMAGE_SENSOR_PV_WIDTH;
	pSensorInfo->SensorPreviewResolutionY= S5K4H5YX_IMAGE_SENSOR_PV_HEIGHT;
	#endif
	
	#ifdef Capture8M
	pSensorInfo->SensorFullResolutionX= 3264;
    pSensorInfo->SensorFullResolutionY= 2448;
	#else
	pSensorInfo->SensorFullResolutionX= S5K4H5YX_IMAGE_SENSOR_FULL_WIDTH;
    pSensorInfo->SensorFullResolutionY= S5K4H5YX_IMAGE_SENSOR_FULL_HEIGHT;
	#endif
	
	spin_lock(&s5k4h5yxmipiraw_drv_lock);
	s5k4h5yx.imgMirror = pSensorConfigData->SensorImageMirror ;
	spin_unlock(&s5k4h5yxmipiraw_drv_lock);

   	pSensorInfo->SensorOutputDataFormat= SENSOR_OUTPUT_FORMAT_RAW_B;
    pSensorInfo->SensorClockPolarity =SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;

    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;

    pSensorInfo->CaptureDelayFrame = 1;
    pSensorInfo->PreviewDelayFrame = 1;
    pSensorInfo->VideoDelayFrame = 2;
    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_4MA; //change MCLK drive current  from 8mA to 4mA
    pSensorInfo->AEShutDelayFrame = 0;//0;		    /* The frame of setting shutter default 0 for TG int */
    pSensorInfo->AESensorGainDelayFrame = 0 ;//0;     /* The frame of setting sensor gain */
    pSensorInfo->AEISPGainDelayFrame = 2;

    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;
			
			#ifdef Capture8M
			pSensorInfo->SensorGrabStartX = 0;
            pSensorInfo->SensorGrabStartY = 1;
			#else
            pSensorInfo->SensorGrabStartX = S5K4H5YX_PV_X_START;
            pSensorInfo->SensorGrabStartY = S5K4H5YX_PV_Y_START;
			#endif
	
            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_4_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 6;//14
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = S5K4H5YX_VIDEO_X_START;
            pSensorInfo->SensorGrabStartY = S5K4H5YX_VIDEO_Y_START;
			
            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_4_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 6;//14
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

			#ifdef Capture8M
			pSensorInfo->SensorGrabStartX = 0;	//2*S5K4H5YX_IMAGE_SENSOR_PV_STARTX;
            pSensorInfo->SensorGrabStartY = 1;	//2*S5K4H5YX_IMAGE_SENSOR_PV_STARTY;
			#else
            pSensorInfo->SensorGrabStartX = S5K4H5YX_FULL_X_START;	//2*S5K4H5YX_IMAGE_SENSOR_PV_STARTX;
            pSensorInfo->SensorGrabStartY = S5K4H5YX_FULL_Y_START;	//2*S5K4H5YX_IMAGE_SENSOR_PV_STARTY;
			#endif

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_4_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 6;//14
            pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        default:
			pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = S5K4H5YX_PV_X_START;
            pSensorInfo->SensorGrabStartY = S5K4H5YX_PV_Y_START;

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_4_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 6;//14
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
    }

    memcpy(pSensorConfigData, &S5K4H5YXSensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

    return ERROR_NONE;
}   /* S5K4H5YXGetInfo() */


UINT32 S5K4H5YXControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
		spin_lock(&s5k4h5yxmipiraw_drv_lock);
		S5K4H5YXCurrentScenarioId = ScenarioId;
		spin_unlock(&s5k4h5yxmipiraw_drv_lock);
		//S5K4H5YXDB("ScenarioId=%d\n",ScenarioId);
		S5K4H5YXDB("[S5K4H5YX] [S5K4H5YXControl]\n");
		S5K4H5YXDB("[S5K4H5YX] [S5K4H5YXControl] ScenarioId=%d\n",S5K4H5YXCurrentScenarioId);
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            S5K4H5YXPreview(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			S5K4H5YXVideo(pImageWindow, pSensorConfigData);
			break;   
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            S5K4H5YXCapture(pImageWindow, pSensorConfigData);
            break;

        default:
            return ERROR_INVALID_SCENARIO_ID;

    }
    return ERROR_NONE;
} /* S5K4H5YXControl() */


UINT32 S5K4H5YXSetVideoMode(UINT16 u2FrameRate)
{

    kal_uint32 MIN_Frame_length =0,frameRate=0,extralines=0;
    S5K4H5YXDB("[S5K4H5YX] [S5K4H5YXSetVideoMode] frame rate = %d\n", u2FrameRate);
	
	if(u2FrameRate==0)
	{
		S5K4H5YXDB("[S5K4H5YX] [S5K4H5YXSetVideoMode] Disable Video Mode or dynimac fps\n");
		return KAL_TRUE;
	}
	if(u2FrameRate >30 || u2FrameRate <5)
	    S5K4H5YXDB("[S5K4H5YX] [S5K4H5YXSetVideoMode] error frame rate seting\n");

    if(s5k4h5yx.sensorMode == SENSOR_MODE_VIDEO)//video ScenarioId recording
    {
    	S5K4H5YXDB("[S5K4H5YX] [S5K4H5YXSetVideoMode] SENSOR_MODE_VIDEO\n");
    	if(s5k4h5yx.S5K4H5YXAutoFlickerMode == KAL_TRUE)
    	{
    		if (u2FrameRate==30)
				frameRate= 306;
			else if(u2FrameRate==15)
				frameRate= 148;//148;
			else
				frameRate=u2FrameRate*10;

			MIN_Frame_length = (s5k4h5yx.videoPclk*10000)/(S5K4H5YX_VIDEO_PERIOD_PIXEL_NUMS + s5k4h5yx.DummyPixels)/frameRate*10;
    	}
		else
			MIN_Frame_length = (s5k4h5yx.videoPclk*10000) /(S5K4H5YX_VIDEO_PERIOD_PIXEL_NUMS + s5k4h5yx.DummyPixels)/u2FrameRate;

		if((MIN_Frame_length <=S5K4H5YX_VIDEO_PERIOD_LINE_NUMS))
		{
			MIN_Frame_length = S5K4H5YX_VIDEO_PERIOD_LINE_NUMS;
		}
		extralines = MIN_Frame_length - S5K4H5YX_VIDEO_PERIOD_LINE_NUMS;

		spin_lock(&s5k4h5yxmipiraw_drv_lock);
		s5k4h5yx.DummyPixels = 0;//define dummy pixels and lines
		s5k4h5yx.DummyLines = extralines ;
		spin_unlock(&s5k4h5yxmipiraw_drv_lock);

		S5K4H5YX_SetDummy(s5k4h5yx.DummyPixels,extralines);
    }
	else if(s5k4h5yx.sensorMode == SENSOR_MODE_CAPTURE)
	{
		S5K4H5YXDB("[S5K4H5YX] [S5K4H5YXSetVideoMode] SENSOR_MODE_CAPTURE\n");
		if(s5k4h5yx.S5K4H5YXAutoFlickerMode == KAL_TRUE)
    	{
			#if defined(ZSD15FPS)
    		if (u2FrameRate==15)
			    frameRate= 148;
			#else
    		if (u2FrameRate==13)
				frameRate= 130;
			#endif
			else
				frameRate=u2FrameRate*10;
			
			MIN_Frame_length = (s5k4h5yx.capPclk*10000) /(S5K4H5YX_FULL_PERIOD_PIXEL_NUMS + s5k4h5yx.DummyPixels)/frameRate*10;
    	}
		else
			MIN_Frame_length = (s5k4h5yx.capPclk*10000) /(S5K4H5YX_FULL_PERIOD_PIXEL_NUMS + s5k4h5yx.DummyPixels)/u2FrameRate;

		if((MIN_Frame_length <=S5K4H5YX_FULL_PERIOD_LINE_NUMS))
		{
			MIN_Frame_length = S5K4H5YX_FULL_PERIOD_LINE_NUMS;
			S5K4H5YXDB("[S5K4H5YX] [S5K4H5YXSetVideoMode] current fps = %d\n", (s5k4h5yx.capPclk*10000) /(S5K4H5YX_FULL_PERIOD_PIXEL_NUMS)/S5K4H5YX_FULL_PERIOD_LINE_NUMS);

		}
		S5K4H5YXDB("[S5K4H5YX] [S5K4H5YXSetVideoMode] current fps (10 base)= %d\n", (s5k4h5yx.pvPclk*10000)*10/(S5K4H5YX_FULL_PERIOD_PIXEL_NUMS + s5k4h5yx.DummyPixels)/MIN_Frame_length);

		extralines = MIN_Frame_length - S5K4H5YX_FULL_PERIOD_LINE_NUMS;

		spin_lock(&s5k4h5yxmipiraw_drv_lock);
		s5k4h5yx.DummyPixels = 0;//define dummy pixels and lines
		s5k4h5yx.DummyLines = extralines ;
		spin_unlock(&s5k4h5yxmipiraw_drv_lock);

		S5K4H5YX_SetDummy(s5k4h5yx.DummyPixels,extralines);
	}
	S5K4H5YXDB("[S5K4H5YX] [S5K4H5YXSetVideoMode] MIN_Frame_length=%d,s5k4h5yx.DummyLines=%d\n",MIN_Frame_length,s5k4h5yx.DummyLines);

    return KAL_TRUE;
}

UINT32 S5K4H5YXSetAutoFlickerMode(kal_bool bEnable, UINT16 u2FrameRate)
{
	//return ERROR_NONE;
	if(bEnable) {   // enable auto flicker
		S5K4H5YXDB("[S5K4H5YX] [S5K4H5YXSetAutoFlickerMode] enable\n");
		spin_lock(&s5k4h5yxmipiraw_drv_lock);
		s5k4h5yx.S5K4H5YXAutoFlickerMode = KAL_TRUE;
		spin_unlock(&s5k4h5yxmipiraw_drv_lock);
    } else {
    	S5K4H5YXDB("[S5K4H5YX] [S5K4H5YXSetAutoFlickerMode] disable\n");
    	spin_lock(&s5k4h5yxmipiraw_drv_lock);
        s5k4h5yx.S5K4H5YXAutoFlickerMode = KAL_FALSE;
		spin_unlock(&s5k4h5yxmipiraw_drv_lock);
    }

    return ERROR_NONE;
}

UINT32 S5K4H5YXSetTestPatternMode(kal_bool bEnable)
{
    return TRUE;
}

UINT32 S5K4H5YXMIPISetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) 
{
	kal_uint32 pclk;
	kal_int16 dummyLine;
	kal_uint16 lineLength,frameHeight;
		
	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			S5K4H5YXDB("[S5K4H5YX] [S5K4H5YXMIPISetMaxFramerateByScenario] MSDK_SCENARIO_ID_CAMERA_PREVIEW\n");
			#if 0
			pclk = 250000000;
			lineLength = S5K4H5YX_PV_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - S5K4H5YX_PV_PERIOD_LINE_NUMS;
			s5k4h5yx.sensorMode = SENSOR_MODE_PREVIEW;
			S5K4H5YX_SetDummy(0, dummyLine);	
			#endif
			S5K4H5YX_SetDummy(0, 0);
			break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			S5K4H5YXDB("[S5K4H5YX] [S5K4H5YXMIPISetMaxFramerateByScenario] MSDK_SCENARIO_ID_VIDEO_PREVIEW\n");
			#if 0
			pclk = 250000000;
			lineLength = S5K4H5YX_VIDEO_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - S5K4H5YX_VIDEO_PERIOD_LINE_NUMS;
			s5k4h5yx.sensorMode = SENSOR_MODE_VIDEO;
			S5K4H5YX_SetDummy(0, dummyLine);
			#endif
			S5K4H5YX_SetDummy(0, 0);
			break;			
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:	
			S5K4H5YXDB("[S5K4H5YX] [S5K4H5YXMIPISetMaxFramerateByScenario] MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG or MSDK_SCENARIO_ID_CAMERA_ZSD\n");
			#if 0
			pclk = 250000000;
			lineLength = S5K4H5YX_FULL_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - S5K4H5YX_FULL_PERIOD_LINE_NUMS;
			s5k4h5yx.sensorMode = SENSOR_MODE_CAPTURE;
			S5K4H5YX_SetDummy(0, dummyLine);
			#endif
			S5K4H5YX_SetDummy(0, 0);
			break;		
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
            break;
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
			break;
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
			break;		
		default:
			break;
	}	
	return ERROR_NONE;
}


UINT32 S5K4H5YXMIPIGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
{

	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 *pframeRate = 300;
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 *pframeRate = 250;
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



UINT32 S5K4H5YXFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
                                                                UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
    UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
    UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
    UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
    UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
    UINT32 SensorRegNumber;
    UINT32 i;
    PNVRAM_SENSOR_DATA_STRUCT pSensorDefaultData=(PNVRAM_SENSOR_DATA_STRUCT) pFeaturePara;
    MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
    MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_GROUP_INFO_STRUCT *pSensorGroupInfo=(MSDK_SENSOR_GROUP_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ITEM_INFO_STRUCT *pSensorItemInfo=(MSDK_SENSOR_ITEM_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ENG_INFO_STRUCT	*pSensorEngInfo=(MSDK_SENSOR_ENG_INFO_STRUCT *) pFeaturePara;

    switch (FeatureId)
    {
        case SENSOR_FEATURE_GET_RESOLUTION:
			#ifdef Capture8M
            *pFeatureReturnPara16++= 3264;
            *pFeatureReturnPara16= 2448;
			#else
			*pFeatureReturnPara16++= S5K4H5YX_IMAGE_SENSOR_FULL_WIDTH;
            *pFeatureReturnPara16= S5K4H5YX_IMAGE_SENSOR_FULL_HEIGHT;
			#endif
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_PERIOD:
				*pFeatureReturnPara16++= S5K4H5YX_FeatureControl_PERIOD_PixelNum;
				*pFeatureReturnPara16= S5K4H5YX_FeatureControl_PERIOD_LineNum;
				*pFeatureParaLen=4;
				break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			switch(S5K4H5YXCurrentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					*pFeatureReturnPara32 = 138670000;
					*pFeatureParaLen=4;
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					*pFeatureReturnPara32 = 147330000;
					*pFeatureParaLen=4;
					break;	 
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara32 = 138670000;
					*pFeatureParaLen=4;
					break;
				default:
					*pFeatureReturnPara32 = 138670000;
					*pFeatureParaLen=4;
					break;
			}
		    break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            S5K4H5YX_SetShutter(*pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            S5K4H5YX_NightMode((BOOL) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_GAIN:
            S5K4H5YX_SetGain((UINT16) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            //S5K4H5YX_isp_master_clock=*pFeatureData32;
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            S5K4H5YX_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            pSensorRegData->RegData = S5K4H5YX_read_cmos_sensor(pSensorRegData->RegAddr);
            break;
        case SENSOR_FEATURE_SET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&s5k4h5yxmipiraw_drv_lock);
                S5K4H5YXSensorCCT[i].Addr=*pFeatureData32++;
                S5K4H5YXSensorCCT[i].Para=*pFeatureData32++;
				spin_unlock(&s5k4h5yxmipiraw_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=S5K4H5YXSensorCCT[i].Addr;
                *pFeatureData32++=S5K4H5YXSensorCCT[i].Para;
            }
            break;
        case SENSOR_FEATURE_SET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&s5k4h5yxmipiraw_drv_lock);
                S5K4H5YXSensorReg[i].Addr=*pFeatureData32++;
                S5K4H5YXSensorReg[i].Para=*pFeatureData32++;
				spin_unlock(&s5k4h5yxmipiraw_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=S5K4H5YXSensorReg[i].Addr;
                *pFeatureData32++=S5K4H5YXSensorReg[i].Para;
            }
            break;
        case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
            if (*pFeatureParaLen>=sizeof(NVRAM_SENSOR_DATA_STRUCT))
            {
                pSensorDefaultData->Version=NVRAM_CAMERA_SENSOR_FILE_VERSION;
                pSensorDefaultData->SensorId=S5K4H5YX_SENSOR_ID;
                memcpy(pSensorDefaultData->SensorEngReg, S5K4H5YXSensorReg, sizeof(SENSOR_REG_STRUCT)*ENGINEER_END);
                memcpy(pSensorDefaultData->SensorCCTReg, S5K4H5YXSensorCCT, sizeof(SENSOR_REG_STRUCT)*FACTORY_END_ADDR);
            }
            else
                return FALSE;
            *pFeatureParaLen=sizeof(NVRAM_SENSOR_DATA_STRUCT);
            break;
        case SENSOR_FEATURE_GET_CONFIG_PARA:
            memcpy(pSensorConfigData, &S5K4H5YXSensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
            *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
            break;
        case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
            S5K4H5YX_camera_para_to_sensor();
            break;

        case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
            S5K4H5YX_sensor_to_camera_para();
            break;
        case SENSOR_FEATURE_GET_GROUP_COUNT:
            *pFeatureReturnPara32++=S5K4H5YX_get_sensor_group_count();
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_GROUP_INFO:
            S5K4H5YX_get_sensor_group_info(pSensorGroupInfo->GroupIdx, pSensorGroupInfo->GroupNamePtr, &pSensorGroupInfo->ItemCount);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_ITEM_INFO:
            S5K4H5YX_get_sensor_item_info(pSensorItemInfo->GroupIdx,pSensorItemInfo->ItemIdx, pSensorItemInfo);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_SET_ITEM_INFO:
            S5K4H5YX_set_sensor_item_info(pSensorItemInfo->GroupIdx, pSensorItemInfo->ItemIdx, pSensorItemInfo->ItemValue);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_GET_ENG_INFO:
            pSensorEngInfo->SensorId = 129;
            pSensorEngInfo->SensorType = CMOS_SENSOR;
            pSensorEngInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_B;
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ENG_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
            *pFeatureParaLen=4;
            break;

        case SENSOR_FEATURE_INITIALIZE_AF:
            break;
        case SENSOR_FEATURE_CONSTANT_AF:
            break;
        case SENSOR_FEATURE_MOVE_FOCUS_LENS:
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            S5K4H5YXSetVideoMode(*pFeatureData16);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            S5K4H5YXGetSensorID(pFeatureReturnPara32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            S5K4H5YXSetAutoFlickerMode((BOOL)*pFeatureData16, *(pFeatureData16+1));
	        break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            S5K4H5YXSetTestPatternMode((BOOL)*pFeatureData16);
            break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			S5K4H5YXMIPISetMaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, *(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			S5K4H5YXMIPIGetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, (MUINT32 *)(*(pFeatureData32+1)));
			break;
        default:
            break;
    }
    return ERROR_NONE;
}	/* S5K4H5YXFeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncS5K4H5YX=
{
    S5K4H5YXOpen,
    S5K4H5YXGetInfo,
    S5K4H5YXGetResolution,
    S5K4H5YXFeatureControl,
    S5K4H5YXControl,
    S5K4H5YXClose
};

UINT32 S5K4H5YX_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncS5K4H5YX;

    return ERROR_NONE;
}   /* SensorInit() */

