/*=============================================================================*/
/* CONFIDENTIAL																   */
/* Copyright(c) 2010-2012 Yamaha Corporation								   */
/*=============================================================================*/

/*=============================================================================*/
/* File 	yas_pcb_test.c													   */
/* Brief	pcb test program for yas530/yas532								   */
/* Date 	2012/03/07														   */
/* Revision	1.3.0															   */
/*=============================================================================*/

#include "yas_pcb_test.h"
#include "yas_1.h"

#include <linux/types.h>
#include <linux/list.h>
#include <linux/hwmsen_helper.h>
#include <linux/i2c.h>
//#include "..\usb\EvbUsbDrv.h"
//#include <windows.h>

/* define */
/* reg num */
#define YAS530_CAL_REG_NUM			( 16 )
#define YAS532_CAL_REG_NUM			( 14 )
#define YAS_MEASURE_DATA_REG_NUM	( 8 )

/* default value */
#define YAS_TEST1_DEFAULT			( 0x00 )
#define YAS_TEST2_DEFAULT			( 0x00 )
#define YAS_INTERVAL_DEFAULT		( 0x00 )
#define YAS_CONFIG_DEFAULT			( 0x01 )		/* INTON = 1 */
#define YAS_COIL_DEFAULT			( 0x00 )

/* measure command */
#define YAS_MEASURE_COMMAND_START	( 0x01 )
#define YAS_MEASURE_COMMAND_LDTC	( 0x02 )
#define YAS_MEASURE_COMMAND_FORS	( 0x04 )

#define YAS_MEASURE_BUSY			( 0x80 )

#define YAS_MEASURE_WAIT_TIME		( 2 )			/* ms */
#define YAS_HARD_OFFSET_CORRECT		( 16 )
#define	YAS_COIL_INIT_CALC_NUM		( 5 )

#define YAS_HARD_OFFSET_MASK		( 0x3F )

#define YAS_INT_CHECK				( 1 )
#define YAS_INT_NOTCHECK			( 0 )
#define YAS_INT_HIGH				( 1 )
#define YAS_INT_LOW					( 0 )

#define YAS_ACC_Z					( 9806550UL )	/* m/s2 */

#define YAS530_DEVICE_ID			( 0x01 )		/* device id for MS-3E */
#define YAS532_DEVICE_ID			( 0x02 )		/* device id for MS-3R */

#define YAS530_VERSION_A			( 0 )			/* YAS530 (MS-3E Aver) */
#define YAS530_VERSION_B			( 1 )			/* YAS530 (MS-3E Bver) */
/*#define YAS530_VERSION_AB			( 0 )*/			/* YAS532 (MS-3R ABver) */
#define YAS532_VERSION_AC			( 1 )			/* YAS532 (MS-3R ACver) */

#define YAS530_COEF_VERSION_A		(  380 )
#define YAS530_COEF_VERSION_B		(  550 )
/*#define YAS532_COEF_VERSION_AB		( 1800 ) */
/*#define YAS532_COEF_VERSION_AC		(  900 ) */
#define YAS532_COEFX_VERSION_AC		(  850 )
#define YAS532_COEFY1_VERSION_AC	(  750 )
#define YAS532_COEFY2_VERSION_AC	(  750 )

#define YAS530_RAWDATA_CENTER		( 2048 )
#define YAS530_RAWDATA_OVERFLOW		( 4095 )
#define YAS532_RAWDATA_CENTER		( 4096 )
#define YAS532_RAWDATA_OVERFLOW		( 8190 )

struct yas_vector {
    int32_t v[3];
};
typedef enum _tagTestId
{
	YAS_TEST1 = 0,
	YAS_TEST3,
	YAS_TEST4,
	YAS_TEST5,
#ifdef YAS_PCBTEST_EXTRA
	YAS_TEST6,
	YAS_TEST7,
#else
	YAS_TEST6 = YAS_TEST5,
#endif
	YAS_TEST2,
	YAS_TESTMAX
} YAS_TESTID;

typedef struct yas_correction
{
	int32_t s32Cx, s32Cy1, s32Cy2;
	int32_t s32A2, s32A3, s32A4, s32A5, s32A6, s32A7, s32A8, s32A9, s32K;
	int32_t s32ZFlag;
	int32_t s32Rx, s32Ry1, s32Ry2;
	int32_t s32Fx, s32Fy1, s32Fy2;
	int32_t s32Ver;
} YAS_CORRECTION;

typedef struct yas_sensitivity
{
	int32_t s32Sx, s32Sy, s32Sz;
} YAS_SENSITIVITY;


/* values */
static uint8_t gu08State = ( uint8_t )YAS_TEST1;
static struct yas_pcb_test_callback g_callback;
static struct yas_vector gstXy1y2;
static int8_t gs08HardOffset[ 3 ];
static YAS_CORRECTION gstCorrect;
static uint8_t gu08DevId;
static int32_t gs32Center;
static int32_t gs32Overflow;
/*F00184246*/
static struct i2c_client * test_client = NULL;
/*F00184246*/
/* functions */
static int yas_check_state( YAS_TESTID id );
static void yas_update_state( YAS_TESTID id );
static int yas_power_on( void );
static int yas_power_off( void );
static int yas_read_cal( uint8_t *pu08Buf, int size );
/*static void yas_calc_correction( const uint8_t *pu08Data );*/
static int yas_set_offset( const int8_t *ps08Offset );
static int yas_measure( struct yas_vector *pstXy1y2, int *temperature, uint8_t u08Command, uint8_t u08CheckIni );
static int yas_is_flow_occued( struct yas_vector *pstXy1y2 );
static void yas_calc_sensitivity( struct yas_vector *pstXy1y2, int temperature, YAS_SENSITIVITY *pstYasSensitivity );
static void yas_calc_position( struct yas_vector *pstXy1y2, struct yas_vector *pstXyz, int temperature );
/*static void yas_calc_magnetic_field( struct yas_vector *pstXyz, int32_t s32Coef );*/
static int yas_test1( int *id );
static int yas_test2( void );
static int yas_test3( void );
static int yas_test4( int *x, int *y1, int *y2 );
static int yas_test5( int *direction );
static int yas_test6( int *sx, int *sy );
#ifdef YAS_PCBTEST_EXTRA
static int yas_test7( int *ohx, int *ohy, int *ohz );
#endif

void yamaha_test_init_clint(struct i2c_client * client);

void yamaha_test_init_clint(struct i2c_client  * client)
{
     test_client = client;
}
static int  yas530_power_on(void)
{
	/*该函数为空 即可*/
	return 0;
}
static int  yas530_power_off(void)
{
	/*该函数为空 即可*/
	return 0;
}

static int  i2c_open(void)
{
	
     
	return 0;
}

static int  i2c_close(void)
{
	
	return 0;
}
static int  i2c_write(uint8_t slave, uint8_t addr, const uint8_t *buf, int len )
{

	//I2CWriteRegN_Usb_Ecom(slave,addr,buf,len);
	
	if(hwmsen_write_block(test_client,addr,buf,len))
      {
          printk("i2c_write i2c transfer error");
      }   
	
	return 0;
}

static int  i2c_read(uint8_t slave, uint8_t addr, uint8_t *buf, int len)
{

	//I2CReadReg_Usb_Ecom(slave,addr,buf,len);
	if (hwmsen_read_block(test_client, addr, buf, len))
      {
           printk("i2c_read i2c transfer error");
           return -1;
      } 
      
    
	return 0;
}

static void  msleep(int msec)
{
	msleep(msec*10);
	return ;
}
static int read_intpin ( int *low_or_high )
{
	/*该函数为空 即可*/
	return 0;
}


static int
yas_check_state( YAS_TESTID id )
{
	int result = YAS_ERROR_TEST_ORDER;
	uint8_t u08Mask;
	const uint8_t u08TestTable[] =
	{
		( 1 << YAS_TEST1 ),																																/* T1 */
							( 1 << YAS_TEST3 ) 																					   | ( 1 << YAS_TEST2 ),/* T3 */
							( 1 << YAS_TEST3 ) | ( 1 << YAS_TEST4 )																   | ( 1 << YAS_TEST2 ),/* T4 */
												 ( 1 << YAS_TEST4 ) | ( 1 << YAS_TEST5 ) | ( 1 << YAS_TEST6 )					   | ( 1 << YAS_TEST2 ),/* T5 */
												 					  ( 1 << YAS_TEST5 ) | ( 1 << YAS_TEST6 )					   | ( 1 << YAS_TEST2 ),/* T6 */
#ifdef YAS_PCBTEST_EXTRA
												 					  ( 1 << YAS_TEST5 ) | ( 1 << YAS_TEST6 ) | ( 1 << YAS_TEST7 ) | ( 1 << YAS_TEST2 ),/* T7 */
												 					  					   ( 1 << YAS_TEST6 ) | ( 1 << YAS_TEST7 ) | ( 1 << YAS_TEST2 ),/* T8 */
#endif
	};

	if( ( YAS_TEST1 <= id )
	 && ( id < YAS_TESTMAX ) )
	{
		u08Mask = ( uint8_t )( 1 << id );
		if( u08Mask == ( u08TestTable[ gu08State ] & u08Mask ) )
		{
			result = YAS_NO_ERROR;
		}
	}

	return result;
}

static void
yas_update_state( YAS_TESTID id )
{

	if( YAS_TEST2 <= id )
	{
		gu08State = ( uint8_t )YAS_TEST1;
	}
#ifdef YAS_PCBTEST_EXTRA
	else if( YAS_TEST5 == gu08State )
	{
		if( YAS_TEST5 == id )
		{
			gu08State += 2;		/* T5 仺 T7 */
		}
		else
		{
			if( YAS_TEST6 == id )
			{
				gu08State++;	/* T5 仺 T6 */
			}
		}
	}
	else if( YAS_TEST6 == gu08State )
	{
		if( YAS_TEST5 == id )
		{
			gu08State++;		/* T6 仺 T7 */
		}
	}
#endif
	else
	{
		if( id == gu08State )
		{
			gu08State++;		/* Tn 仺 Tn+1 */
		}
	}
}

static int
yas_power_on( void )
{
	int result = YAS_NO_ERROR;
	int ret;

	if( NULL != g_callback.power_on )
	{
		ret = g_callback.power_on();
		if( 0 != ret )
		{
			result = YAS_ERROR_POWER;
		}
	}

	return result;
}

static int
yas_power_off( void )
{
	int result = YAS_NO_ERROR;
	int ret;

	if( NULL != g_callback.power_off )
	{
		ret = g_callback.power_off();
		if( 0 != ret )
		{
			result = YAS_ERROR_POWER;
		}
	}

	return result;
}

static int
yas_read_cal( uint8_t *pu08Buf, int size )
{
	int i;
	int ret;

	/* Dummy read */
       for (i = 0; i < size; i++) { /* dummy read */
        if (g_callback.i2c_read(YAS_ADDR_SLAVE,YAS_ADDR_CAL + i, &pu08Buf[i], 1) < 0) {
            return YAS_ERROR_I2C;
        }
    }
    for (i = 0; i < size; i++) {
        if (g_callback.i2c_read(YAS_ADDR_SLAVE,YAS_ADDR_CAL + i, &pu08Buf[i], 1) < 0) {
            return YAS_ERROR_I2C;
        }
    }
    #if 0
	ret = g_callback.i2c_read( YAS_ADDR_SLAVE, YAS_ADDR_CAL, pu08Buf, size );
	if( 0 != ret )
	{
		return YAS_ERROR_I2C;
	}

	ret = g_callback.i2c_read( YAS_ADDR_SLAVE, YAS_ADDR_CAL, pu08Buf, size );
	if( 0 != ret )
	{
		return YAS_ERROR_I2C;
	}
    #endif

	/* cal register is all 0 */
	for ( i = 0; i < size; i++ )
	{
		if ( pu08Buf[ i ] != 0x00 )
		{
			return YAS_NO_ERROR;
		}
	}

	return YAS_ERROR_CALREG;
}

static void
yas530_calc_correction( const uint8_t *pu08Data )
{
	uint8_t u08Dx  = pu08Data[ 0 ];
	uint8_t u08Dy1 = pu08Data[ 1 ];
	uint8_t u08Dy2 = pu08Data[ 2 ];
	uint8_t u08D2  = ( pu08Data[ 3 ] >> 2 ) & 0x3F;
	uint8_t u08D3  = ( uint8_t )( ( ( pu08Data[ 3 ] << 2 ) & 0x0C ) | ( ( pu08Data[ 4 ] >> 6 ) & 0x03 ) );
	uint8_t u08D4  = pu08Data[ 4 ] & 0x3F;
	uint8_t u08D5  = ( pu08Data[ 5 ] >> 2 ) & 0x3f;
	uint8_t u08D6  = ( uint8_t )( ( ( pu08Data[ 5 ] << 4 ) & 0x30 ) | ( ( pu08Data[ 6 ] >> 4 ) & 0x0F ) );
	uint8_t u08D7  = ( uint8_t )( ( ( pu08Data[ 6 ] << 3 ) & 0x78 ) | ( ( pu08Data[ 7 ] >> 5 ) & 0x07 ) );
	uint8_t u08D8  = ( uint8_t )( ( ( pu08Data[ 7 ] << 1 ) & 0x3E ) | ( ( pu08Data[ 8 ] >> 7 ) & 0x01 ) );
	uint8_t u08D9  = ( uint8_t )( ( ( pu08Data[ 8 ] << 1 ) & 0xFE ) | ( ( pu08Data[ 9 ] >> 7 ) & 0x01 ) );
	uint8_t u08D0  = ( pu08Data[ 9 ] >> 2 ) & 0x1F;
	uint8_t u08ZFlag = ( pu08Data[ 11 ] >> 5 ) & 0x01;
	uint8_t u08Rx	 = ( uint8_t )( ( ( pu08Data[ 11 ] << 1 ) & 0x3E ) | ( ( pu08Data[ 12 ] >> 7 ) & 0x01 ) );
	uint8_t u08Fx	 = ( uint8_t )( ( pu08Data[ 12 ] >> 5 ) & 0x03 );
	uint8_t u08Ry1	 = ( uint8_t )( ( ( pu08Data[ 12 ] << 1 ) & 0x3E ) | ( ( pu08Data[ 13 ] >> 7 ) & 0x01 ) );
	uint8_t u08Fy1	 = ( uint8_t )( ( pu08Data[ 13 ] >> 5 ) & 0x03 );
	uint8_t u08Ry2	 = ( uint8_t )( ( ( pu08Data[ 13 ] << 1 ) & 0x3E ) | ( ( pu08Data[ 14 ] >> 7 ) & 0x01 ) );
	uint8_t u08Fy2	 = ( uint8_t )( ( pu08Data[ 14 ] >> 5 ) & 0x03 );
	uint8_t u08Ver	 = pu08Data[ 15 ] & 0x07;

	gstCorrect.s32Cx  = ( int32_t )( ( u08Dx * 6 ) - 768 );
	gstCorrect.s32Cy1 = ( int32_t )( ( u08Dy1 * 6 ) - 768 );
	gstCorrect.s32Cy2 = ( int32_t )( ( u08Dy2 * 6 ) - 768 );
	gstCorrect.s32A2  = ( int32_t )( u08D2 - 32 );
	gstCorrect.s32A3  = ( int32_t )( u08D3 - 8 );
	gstCorrect.s32A4  = ( int32_t )( u08D4 - 32 );
	gstCorrect.s32A5  = ( int32_t )( u08D5 + 38 );
	gstCorrect.s32A6  = ( int32_t )( u08D6 - 32 );
	gstCorrect.s32A7  = ( int32_t )( u08D7 - 64 );
	gstCorrect.s32A8  = ( int32_t )( u08D8 - 32 );
	gstCorrect.s32A9  = ( int32_t )u08D9;
	gstCorrect.s32K 	= ( int32_t )( u08D0 ) + 10;
	gstCorrect.s32ZFlag = ( int32_t )u08ZFlag;
	gstCorrect.s32Rx	= ( int32_t )( ( int8_t )( u08Rx << 2 ) ) >> 2;
	gstCorrect.s32Fx	= ( int32_t )u08Fx;
	gstCorrect.s32Ry1	= ( int32_t )( ( int8_t )( u08Ry1 << 2 ) ) >> 2;
	gstCorrect.s32Fy1	= ( int32_t )u08Fy1;
	gstCorrect.s32Ry2	= ( int32_t )( ( int8_t )( u08Ry2 << 2 ) ) >> 2;
	gstCorrect.s32Fy2	= ( int32_t )u08Fy2;
	gstCorrect.s32Ver = ( int32_t )u08Ver;
}

static void
yas532_calc_correction( const uint8_t *pu08Data )
{
	uint8_t u08Dx  = pu08Data[ 0 ];
	uint8_t u08Dy1 = pu08Data[ 1 ];
	uint8_t u08Dy2 = pu08Data[ 2 ];
	uint8_t u08D2  = ( pu08Data[ 3 ] >> 2 ) & 0x3F;
	uint8_t u08D3  = ( uint8_t )( ( ( pu08Data[ 3 ] << 2 ) & 0x0C ) | ( ( pu08Data[ 4 ] >> 6 ) & 0x03 ) );
	uint8_t u08D4  = pu08Data[ 4 ] & 0x3F;
	uint8_t u08D5  = ( pu08Data[ 5 ] >> 2 ) & 0x3f;
	uint8_t u08D6  = ( uint8_t )( ( ( pu08Data[ 5 ] << 4 ) & 0x30 ) | ( ( pu08Data[ 6 ] >> 4 ) & 0x0F ) );
	uint8_t u08D7  = ( uint8_t )( ( ( pu08Data[ 6 ] << 3 ) & 0x78 ) | ( ( pu08Data[ 7 ] >> 5 ) & 0x07 ) );
	uint8_t u08D8  = ( uint8_t )( ( ( pu08Data[ 7 ] << 1 ) & 0x3E ) | ( ( pu08Data[ 8 ] >> 7 ) & 0x01 ) );
	uint8_t u08D9  = ( uint8_t )( ( ( pu08Data[ 8 ] << 1 ) & 0xFE ) | ( ( pu08Data[ 9 ] >> 7 ) & 0x01 ) );
	uint8_t u08D0  = ( pu08Data[ 9 ] >> 2 ) & 0x1F;
	uint8_t u08Rx	 = ( uint8_t )( ( pu08Data[ 10 ] >> 1 ) & 0x3F );
	uint8_t u08Fx	 = ( uint8_t )( ( ( pu08Data[ 10 ] & 0x01 ) << 1 ) | ( ( pu08Data[ 11 ] >> 7 ) & 0x01 ) );
	uint8_t u08Ry1	 = ( uint8_t )( ( pu08Data[ 11 ] >> 1 ) & 0x3F );
	uint8_t u08Fy1	 = ( uint8_t )( ( ( pu08Data[ 11 ] & 0x01 ) << 1 ) | ( ( pu08Data[ 12 ] >> 7 ) & 0x01 ) );
	uint8_t u08Ry2	 = ( uint8_t )( ( pu08Data[ 12 ] >> 1 ) & 0x3F );
	uint8_t u08Fy2	 = ( uint8_t )( ( ( pu08Data[ 12 ] & 0x01 ) << 1 ) | ( ( pu08Data[ 13 ] >> 7 ) & 0x01 ) );
	uint8_t u08Ver	 = pu08Data[ 13 ] & 0x01;

	gstCorrect.s32Cx  = ( int32_t )( ( u08Dx * 10 ) - 1280 );
	gstCorrect.s32Cy1 = ( int32_t )( ( u08Dy1 * 10 ) - 1280 );
	gstCorrect.s32Cy2 = ( int32_t )( ( u08Dy2 * 10 ) - 1280 );
	gstCorrect.s32A2  = ( int32_t )( u08D2 - 32 );
	gstCorrect.s32A3  = ( int32_t )( u08D3 - 8 );
	gstCorrect.s32A4  = ( int32_t )( u08D4 - 32 );
	gstCorrect.s32A5  = ( int32_t )( u08D5 + 38 );
	gstCorrect.s32A6  = ( int32_t )( u08D6 - 32 );
	gstCorrect.s32A7  = ( int32_t )( u08D7 - 64 );
	gstCorrect.s32A8  = ( int32_t )( u08D8 - 32 );
	gstCorrect.s32A9  = ( int32_t )u08D9;
	gstCorrect.s32K 	= ( int32_t )u08D0;
	gstCorrect.s32ZFlag = ( int32_t )1;
	gstCorrect.s32Rx	= ( int32_t )( ( int8_t )( u08Rx << 2 ) ) >> 2;
	gstCorrect.s32Fx	= ( int32_t )u08Fx;
	gstCorrect.s32Ry1	= ( int32_t )( ( int8_t )( u08Ry1 << 2 ) ) >> 2;
	gstCorrect.s32Fy1	= ( int32_t )u08Fy1;
	gstCorrect.s32Ry2	= ( int32_t )( ( int8_t )( u08Ry2 << 2 ) ) >> 2;
	gstCorrect.s32Fy2	= ( int32_t )u08Fy2;
	gstCorrect.s32Ver = ( int32_t )u08Ver;
}

static int
yas_set_offset( const int8_t *ps08Offset )
{
	int result = YAS_NO_ERROR;
	int ret;
	uint8_t u08Addr;
	uint8_t u08Data;
	uint8_t i;

	for( i = 0; i < 3; i++ )
	{
		u08Addr = ( uint8_t )( YAS_ADDR_OFFSET + i );
		u08Data = ( uint8_t )ps08Offset[ i ] & YAS_HARD_OFFSET_MASK;
		ret = g_callback.i2c_write( YAS_ADDR_SLAVE, u08Addr, &u08Data, 1 );
		if( 0 != ret )
		{
			result = YAS_ERROR_I2C;
			break;
		}
	}

	return result;
}

static int
yas_measure( struct yas_vector *pstXy1y2, int *temperature, uint8_t u08Command, uint8_t u08CheckIni )
{
	int ret;
       int i;
	uint8_t u08Buf[ YAS_MEASURE_DATA_REG_NUM ];
	int low_or_high;

	if( ( YAS_INT_CHECK == u08CheckIni )
	 && ( g_callback.read_intpin != NULL ) )
	{
		ret = g_callback.read_intpin( &low_or_high );
		if( ( 0 != ret )
		 || ( YAS_INT_HIGH != low_or_high ) )
		{
			return YAS_ERROR_INTERRUPT;
		}
	}

	ret = g_callback.i2c_write( YAS_ADDR_SLAVE, YAS_ADDR_MEASURE_COMMAND, &u08Command, 1 );
	if( 0 != ret )
	{
             printk("yas_measureYAS_ADDR_MEASURE_COMMAND error\n");
             return YAS_ERROR_I2C;
	}

	g_callback.msleep( YAS_MEASURE_WAIT_TIME*2 );

	if( ( YAS_INT_CHECK == u08CheckIni )
	 && ( g_callback.read_intpin != NULL ) )
	{
		ret = g_callback.read_intpin( &low_or_high );
		if( ( 0 != ret )
		 || ( YAS_INT_LOW != low_or_high ) )
		{
			return YAS_ERROR_INTERRUPT;
		}
	}
       #if 0
       for (i = 0; i < YAS_MEASURE_DATA_REG_NUM; i++) { /* dummy read */
        if (g_callback.i2c_read(YAS_ADDR_SLAVE,YAS_ADDR_MEASURE_DATA + i, &u08Buf[i], 1) < 0) {
            printk("yas_measureYAS_ADDR_MEASURE_DATA error,i=%d,u08Buf[i]=%d\n",i,u08Buf[i]);
            return YAS_ERROR_I2C;
        }
        }
       #endif
        #if 1
	ret = g_callback.i2c_read( YAS_ADDR_SLAVE, YAS_ADDR_MEASURE_DATA, u08Buf, YAS_MEASURE_DATA_REG_NUM );
	if( 0 != ret )
	{
              printk("yas_measureYAS_ADDR_MEASURE_DATA error,u08Buf[0]=%d\n",u08Buf[0]);
              return YAS_ERROR_I2C;
	}
       #endif
	/* calc measure data */
       
	if ( gu08DevId == YAS532_DEVICE_ID )
	{
		*temperature	 =			  ( ( ( int32_t )( u08Buf[ 0 ] & 0x7F ) << 3 ) | ( ( u08Buf[ 1 ] >> 5 ) & 0x07 ) );
		pstXy1y2->v[ 0 ] = ( int32_t )( ( ( int32_t )( u08Buf[ 2 ] & 0x7F ) << 6 ) | ( ( u08Buf[ 3 ] >> 2 ) & 0x3F ) );
		pstXy1y2->v[ 1 ] = ( int32_t )( ( ( int32_t )( u08Buf[ 4 ] & 0x7F ) << 6 ) | ( ( u08Buf[ 5 ] >> 2 ) & 0x3F ) );
		pstXy1y2->v[ 2 ] = ( int32_t )( ( ( int32_t )( u08Buf[ 6 ] & 0x7F ) << 6 ) | ( ( u08Buf[ 7 ] >> 2 ) & 0x3F ) );
	}
	else
	{
		*temperature	 =			  ( ( ( int32_t )( u08Buf[ 0 ] & 0x7F ) << 2 ) | ( ( u08Buf[ 1 ] >> 6 ) & 0x03 ) );
		pstXy1y2->v[ 0 ] = ( int32_t )( ( ( int32_t )( u08Buf[ 2 ] & 0x7F ) << 5 ) | ( ( u08Buf[ 3 ] >> 3 ) & 0x1F ) );
		pstXy1y2->v[ 1 ] = ( int32_t )( ( ( int32_t )( u08Buf[ 4 ] & 0x7F ) << 5 ) | ( ( u08Buf[ 5 ] >> 3 ) & 0x1F ) );
		pstXy1y2->v[ 2 ] = ( int32_t )( ( ( int32_t )( u08Buf[ 6 ] & 0x7F ) << 5 ) | ( ( u08Buf[ 7 ] >> 3 ) & 0x1F ) );
	}
     
       printk("yas_measureYAS_ADDR_MEASURE_DATA u08Buf[0]=%d\n",u08Buf[0]);
       printk("yas_measureYAS_ADDR_MEASURE_DATA u08Buf[1]=%d\n",u08Buf[1]);
       printk("yas_measureYAS_ADDR_MEASURE_DATA u08Buf[2]=%d\n",u08Buf[2]);
       printk("yas_measureYAS_ADDR_MEASURE_DATA u08Buf[3]=%d\n",u08Buf[3]);
       printk("yas_measureYAS_ADDR_MEASURE_DATA u08Buf[4]=%d\n",u08Buf[4]);
       printk("yas_measureYAS_ADDR_MEASURE_DATA u08Buf[5]=%d\n",u08Buf[5]);
       printk("yas_measureYAS_ADDR_MEASURE_DATA u08Buf[6]=%d\n",u08Buf[6]);
       printk("yas_measureYAS_ADDR_MEASURE_DATA u08Buf[7]=%d\n",u08Buf[7]);
       /*
	if( YAS_MEASURE_BUSY == ( u08Buf[ 0 ] & YAS_MEASURE_BUSY ) )
        
	{
               printk("yas_measure YAS_MEASURE_BUSY \n");
               return YAS_ERROR_BUSY;
	}*/

	return YAS_NO_ERROR;
}

static int
yas_is_flow_occued( struct yas_vector *pstXy1y2 )
{
	int result = YAS_NO_ERROR;
	int32_t s32Tmp;
	uint8_t i;

	for( i = 0; i < 3; i++ )
	{
		s32Tmp = pstXy1y2->v[ i ];
		if( s32Tmp <= 0 )
		{
			result = YAS_ERROR_UNDERFLOW;
		}
		else if( gs32Overflow <= s32Tmp )
		{
			result = YAS_ERROR_OVERFLOW;
		}
		else
		{
			/* do nothing */
		}
	}

	return result;
}
static void
yas_calc_sensitivity( struct yas_vector *pstXy1y2, int temperature, YAS_SENSITIVITY *pstYasSensitivity )
{
	/* calc XYZ data from xy1y2 data */
	int32_t s32Sx  = pstXy1y2->v[ 0 ] - ( ( gstCorrect.s32Cx  * ( temperature ) ) / 100 );
	int32_t s32Sy1 = pstXy1y2->v[ 1 ] - ( ( gstCorrect.s32Cy1 * ( temperature ) ) / 100 );
	int32_t s32Sy2 = pstXy1y2->v[ 2 ] - ( ( gstCorrect.s32Cy2 * ( temperature ) ) / 100 );
	int32_t s32Sy  =  s32Sy1 - s32Sy2;
	int32_t s32Sz  = -s32Sy1 - s32Sy2;

	pstYasSensitivity->s32Sx = s32Sx;
	pstYasSensitivity->s32Sy = s32Sy;
	pstYasSensitivity->s32Sz = s32Sz;
}

static void
yas_calc_position( struct yas_vector *pstXy1y2, struct yas_vector *pstXyz, int temperature )
{
	YAS_SENSITIVITY stSensitivity;
	YAS_SENSITIVITY *pst;

	yas_calc_sensitivity( pstXy1y2, temperature, &stSensitivity );

	pst = &stSensitivity;
	pstXyz->v[ 0 ] = ( gstCorrect.s32K * ( (			  100 * pst->s32Sx ) + ( gstCorrect.s32A2 * pst->s32Sy ) + ( gstCorrect.s32A3 * pst->s32Sz ) ) ) / 10;
	pstXyz->v[ 1 ] = ( gstCorrect.s32K * ( ( gstCorrect.s32A4 * pst->s32Sx ) + ( gstCorrect.s32A5 * pst->s32Sy ) + ( gstCorrect.s32A6 * pst->s32Sz ) ) ) / 10;
	pstXyz->v[ 2 ] = ( gstCorrect.s32K * ( ( gstCorrect.s32A7 * pst->s32Sx ) + ( gstCorrect.s32A8 * pst->s32Sy ) + ( gstCorrect.s32A9 * pst->s32Sz ) ) ) / 10;
}

static void
yas530_calc_magnetic_field( struct yas_vector *pstXyz, int32_t s32Coef )
{
	int32_t s32Ox;
	int32_t s32Oy1;
	int32_t s32Oy2;
	int32_t s32Oy;
	int32_t s32Oz;
	static const int32_t s32HTbl[] = { 1748, 1948, 2148, 2348 };

	s32Ox  = gstXy1y2.v[ 0 ] - s32HTbl[ gstCorrect.s32Fx  ] + ( gs08HardOffset[ 0 ] - gstCorrect.s32Rx	) * s32Coef;
	s32Oy1 = gstXy1y2.v[ 1 ] - s32HTbl[ gstCorrect.s32Fy1 ] + ( gs08HardOffset[ 1 ] - gstCorrect.s32Ry1 ) * s32Coef;
	s32Oy2 = gstXy1y2.v[ 2 ] - s32HTbl[ gstCorrect.s32Fy2 ] + ( gs08HardOffset[ 2 ] - gstCorrect.s32Ry2 ) * s32Coef;
	s32Oy  = s32Oy1 - s32Oy2;
	s32Oz  = -s32Oy1 - s32Oy2;

	pstXyz->v[ 0 ] = ( gstCorrect.s32K * ( (			  100 * s32Ox ) + ( gstCorrect.s32A2 * s32Oy ) + ( gstCorrect.s32A3 * s32Oz ) ) ) / 10;
	pstXyz->v[ 1 ] = ( gstCorrect.s32K * ( ( gstCorrect.s32A4 * s32Ox ) + ( gstCorrect.s32A5 * s32Oy ) + ( gstCorrect.s32A6 * s32Oz ) ) ) / 10;
	pstXyz->v[ 2 ] = ( gstCorrect.s32K * ( ( gstCorrect.s32A7 * s32Ox ) + ( gstCorrect.s32A8 * s32Oy ) + ( gstCorrect.s32A9 * s32Oz ) ) ) / 10;
}

static void
yas532_calc_magnetic_field( struct yas_vector *pstXyz, int32_t s32CoefX, int32_t s32CoefY1, int32_t s32CoefY2 )
{
	int32_t s32Ox;
	int32_t s32Oy1;
	int32_t s32Oy2;
	int32_t s32Oy;
	int32_t s32Oz;
	static const int32_t s32HTbl[] = { 3721, 3971, 4221, 4471 };

	s32Ox  = gstXy1y2.v[ 0 ] - s32HTbl[ gstCorrect.s32Fx  ] + ( gs08HardOffset[ 0 ] - gstCorrect.s32Rx	) * s32CoefX;
	s32Oy1 = gstXy1y2.v[ 1 ] - s32HTbl[ gstCorrect.s32Fy1 ] + ( gs08HardOffset[ 1 ] - gstCorrect.s32Ry1 ) * s32CoefY1;
	s32Oy2 = gstXy1y2.v[ 2 ] - s32HTbl[ gstCorrect.s32Fy2 ] + ( gs08HardOffset[ 2 ] - gstCorrect.s32Ry2 ) * s32CoefY2;
	s32Oy  = s32Oy1 - s32Oy2;
	s32Oz  = -s32Oy1 - s32Oy2;

	pstXyz->v[ 0 ] = ( gstCorrect.s32K * ( (			  100 * s32Ox ) + ( gstCorrect.s32A2 * s32Oy ) + ( gstCorrect.s32A3 * s32Oz ) ) ) / 10;
	pstXyz->v[ 1 ] = ( gstCorrect.s32K * ( ( gstCorrect.s32A4 * s32Ox ) + ( gstCorrect.s32A5 * s32Oy ) + ( gstCorrect.s32A6 * s32Oz ) ) ) / 10;
	pstXyz->v[ 2 ] = ( gstCorrect.s32K * ( ( gstCorrect.s32A7 * s32Ox ) + ( gstCorrect.s32A8 * s32Oy ) + ( gstCorrect.s32A9 * s32Oz ) ) ) / 10;
}

static int
yas_test1( int *id )
{
	int result = YAS_ERROR_ARG;
	int ret;

	if( NULL != id )
	{
		result = yas_power_on();
		if( YAS_NO_ERROR == result )
		{
			result = YAS_ERROR_I2C;
			ret = g_callback.i2c_read( YAS_ADDR_SLAVE, YAS_ADDR_ID, &gu08DevId, 1 );
			if( 0 == ret )
			{
				*id = ( int )gu08DevId;
				result = YAS_NO_ERROR;
			}
		}
	}

	return result;
}

static int
yas_test2( void )
{
	return yas_power_off();
}

static int
yas_test3( void )
{
	int result;
	int ret;
	uint8_t u08Data;
	uint8_t pu08Buf[ YAS530_CAL_REG_NUM ];

	u08Data = YAS_TEST1_DEFAULT;
	ret = g_callback.i2c_write( YAS_ADDR_SLAVE, YAS_ADDR_TEST1, &u08Data, 1 );
	if( 0 != ret )
	{
		return YAS_ERROR_I2C;
	}

	u08Data = YAS_TEST2_DEFAULT;
	ret = g_callback.i2c_write( YAS_ADDR_SLAVE, YAS_ADDR_TEST2, &u08Data, 1 );
	if( 0 != ret )
	{
		return YAS_ERROR_I2C;
	}

	u08Data = YAS_INTERVAL_DEFAULT;
	ret = g_callback.i2c_write( YAS_ADDR_SLAVE, YAS_ADDR_MEASURE_INTERVAL, &u08Data, 1 );
	if( 0 != ret )
	{
		return YAS_ERROR_I2C;
	}

	if ( gu08DevId == YAS532_DEVICE_ID )
	{
		gs32Center = YAS532_RAWDATA_CENTER;
		gs32Overflow = YAS532_RAWDATA_OVERFLOW;
		result = yas_read_cal( pu08Buf, YAS532_CAL_REG_NUM );
		if ( YAS_NO_ERROR == result )
		{
			yas532_calc_correction( pu08Buf );
		}
	}
	else
	{
		gs32Center = YAS530_RAWDATA_CENTER;
		gs32Overflow = YAS530_RAWDATA_OVERFLOW;
		result = yas_read_cal( pu08Buf, YAS530_CAL_REG_NUM );
		if ( YAS_NO_ERROR == result )
		{
			yas530_calc_correction( pu08Buf );
		}
	}

	if( YAS_NO_ERROR != result )
	{
		return result;
	}

	u08Data = ( uint8_t )( YAS_CONFIG_DEFAULT
						| ( uint8_t )( ( pu08Buf[ 9 ]  & 0x03 ) << 3 )
						| ( uint8_t )( ( pu08Buf[ 10 ] & 0x80 ) >> 5 ) );
	ret = g_callback.i2c_write( YAS_ADDR_SLAVE, YAS_ADDR_CONFIG, &u08Data, 1 );
	if( 0 != ret )
	{
		return YAS_ERROR_I2C;
	}

	u08Data = YAS_COIL_DEFAULT;
	ret = g_callback.i2c_write( YAS_ADDR_SLAVE, YAS_ADDR_COIL, &u08Data, 1 );
	if( 0 != ret )
	{
		return YAS_ERROR_I2C;
	}

	return YAS_NO_ERROR;
}

static int
yas_test4( int *x, int *y1, int *y2 )
{
	int result;
	struct yas_vector stXy1y2;
	int temperature;
	int32_t s32Tmp;
	int8_t s08Correct = YAS_HARD_OFFSET_CORRECT;
	uint8_t i;
	uint8_t k;
	uint8_t u08Command;

	if( ( NULL == x )
	 || ( NULL == y1 )
	 || ( NULL == y2 ) )
	{
		return YAS_ERROR_ARG;
	}

	gs08HardOffset[ 0 ] = 0;
	gs08HardOffset[ 1 ] = 0;
	gs08HardOffset[ 2 ] = 0;
	result = yas_set_offset( &gs08HardOffset[ 0 ] );
	if( YAS_NO_ERROR == result )
	{
		/* calc hard offset */
		for( i = 0; i < YAS_COIL_INIT_CALC_NUM; i++ )
		{
			u08Command = YAS_MEASURE_COMMAND_START;
			result = yas_measure( &stXy1y2, &temperature, u08Command, YAS_INT_NOTCHECK );
                      printk("yas_measure result=%d \n",result);
			if( YAS_NO_ERROR != result )
			{
                              printk("yas_measure error \n");
                              break;
			}

			for( k = 0; k < 3; k++ )
			{
				s32Tmp = stXy1y2.v[ k ];
				if( gs32Center < s32Tmp )
				{
					gs08HardOffset[ k ] += s08Correct;
				}
				else if( s32Tmp < gs32Center )
				{
					gs08HardOffset[ k ] -= s08Correct;
				}
				else
				{
					/* do nothing */
				}
			}

			result = yas_set_offset( &gs08HardOffset[ 0 ] );
			if( YAS_NO_ERROR != result )
			{
				break;
			}

			s08Correct = ( int8_t )( ( uint8_t )s08Correct >> 1 );
		}

		if( YAS_NO_ERROR == result )
		{
			*x	= ( int )gs08HardOffset[ 0 ];
			*y1 = ( int )gs08HardOffset[ 1 ];
			*y2 = ( int )gs08HardOffset[ 2 ];
		}
	}

	return result;
}

#if 0
static int
yas_test5( int *direction )
{
	int result = YAS_ERROR_ARG;
	int ret;
	struct yas_vector stXyz;
	uint8_t u08Command;
	struct yas_utility stYasUtility;
	struct yas_vector stAcc;
	struct yas_matrix stMatrix;
	struct yas_vector stDirection;
	int nTemp;

	if( NULL != direction )
	{
		u08Command = YAS_MEASURE_COMMAND_START;
		result = yas_measure( &gstXy1y2, &nTemp, u08Command, YAS_INT_NOTCHECK );

		if( YAS_NO_ERROR == result )
		{
			yas_calc_position( &gstXy1y2, &stXyz, nTemp );
			result = yas_utility_init( &stYasUtility );
			if( YAS_NO_ERROR == result )
			{
				stAcc.v[ 0 ] = 0;
				stAcc.v[ 1 ] = 0;
				stAcc.v[ 2 ] = YAS_ACC_Z;
				result = YAS_ERROR_DIRCALC;
				ret = stYasUtility.get_rotation_matrix( &stAcc, &stXyz, &stMatrix );
				if( YAS_NO_ERROR == ret )
				{
					ret = stYasUtility.get_euler( &stMatrix, &stDirection );
					if( YAS_NO_ERROR == ret )
					{
						*direction = ( int )stDirection.v[ 0 ];
						result = yas_is_flow_occued( &gstXy1y2 );
					}
				}
			}
		}
	}

	return result;
}
#endif
static int
yas_test6( int *sx, int *sy )
{
	int result = YAS_ERROR_ARG;
	struct yas_vector stXy1y2P;
	struct yas_vector stXy1y2N;
	int temperature;
	uint8_t u08Command;

	if( ( NULL != sx )
	 && ( NULL != sy ) )
	{
		u08Command = YAS_MEASURE_COMMAND_START | YAS_MEASURE_COMMAND_LDTC;
		//result = yas_measure( &stXy1y2P, &temperature, u08Command, YAS_INT_CHECK );
		result = yas_measure( &stXy1y2P, &temperature, u08Command, YAS_INT_NOTCHECK );
		  

		if( YAS_NO_ERROR == result )
		{
			u08Command = YAS_MEASURE_COMMAND_START | YAS_MEASURE_COMMAND_LDTC | YAS_MEASURE_COMMAND_FORS;
			result = yas_measure( &stXy1y2N, &temperature, u08Command, YAS_INT_NOTCHECK );
			if( YAS_NO_ERROR == result )
			{
				if ( gu08DevId == YAS532_DEVICE_ID )
				{
					*sx = ( int )( gstCorrect.s32K * 100 * ( stXy1y2P.v[ 0 ] - stXy1y2N.v[ 0 ] ) );
					*sx /= 1000;
					*sx /= YAS_VCORE;
					*sy = ( int )( gstCorrect.s32K * gstCorrect.s32A5
							 * ( ( stXy1y2P.v[ 1 ] - stXy1y2N.v[ 1 ] ) - ( stXy1y2P.v[ 2 ] - stXy1y2N.v[ 2 ] ) ) );
					*sy /= 1000;
					*sy /= YAS_VCORE;
				}
				else
				{
					*sx = ( int )( stXy1y2N.v[ 0 ] - stXy1y2P.v[ 0 ] );
					*sy = ( int )( ( stXy1y2N.v[ 1 ] - stXy1y2P.v[ 1 ] ) - ( stXy1y2N.v[ 2 ] - stXy1y2P.v[ 2 ] ) );
				}

				result = yas_is_flow_occued( &stXy1y2P );
				if( YAS_NO_ERROR == result )
				{
					result = yas_is_flow_occued( &stXy1y2N );
				}
			}
		}
	}

	return result;
}

#ifdef YAS_PCBTEST_EXTRA
static int
yas_test7( int *ohx, int* ohy, int* ohz )
{
	int nRet = YAS_ERROR_ARG;
	struct yas_vector stOhxyz;
	int32_t s32Coef;

	if( ( NULL != ohx )
	 && ( NULL != ohy )
	 && ( NULL != ohz ) )
	{
		if( gstCorrect.s32ZFlag != 0 )
		{
			if ( gu08DevId == YAS532_DEVICE_ID )
			{
				switch( gstCorrect.s32Ver )
				{
					case YAS532_VERSION_AC :
						break;
					default :
						return YAS_ERROR_I2C;
						/* break; */
				}

				/* calculate Ohx/y/z[nT] */
				yas532_calc_magnetic_field( &stOhxyz, YAS532_COEFX_VERSION_AC,
									YAS532_COEFY1_VERSION_AC, YAS532_COEFY2_VERSION_AC );
			}
			else
			{
				switch( gstCorrect.s32Ver )
				{
					case YAS530_VERSION_A :
						s32Coef = YAS530_COEF_VERSION_A;
						break;
					case YAS530_VERSION_B :
						s32Coef = YAS530_COEF_VERSION_B;
						break;
					default :
						return YAS_ERROR_I2C;
						/* break; */
				}

				/* calculate Ohx/y/z[nT] */
				yas530_calc_magnetic_field( &stOhxyz, s32Coef );
			}

			/* [nT]仺[uT] */
			*ohx = stOhxyz.v[ 0 ] / 1000;
			*ohy = stOhxyz.v[ 1 ] / 1000;
			*ohz = stOhxyz.v[ 2 ] / 1000;

			nRet = YAS_NO_ERROR;
		}
		else
		{
			nRet = YAS_ERROR_NOT_SUPPORTED;
		}
	}

	return nRet;
}
#endif

/* test 1 */
static int
power_on_and_device_check( int *id )
{
	int result = yas_check_state( YAS_TEST1 );
	int ret;

	if( YAS_NO_ERROR == result )
	{
		result = YAS_ERROR_I2C;
		ret = g_callback.i2c_open();
		if( 0 == ret )
		{
			result = yas_test1( id );
                      printk("power_on_and_device_check id=%d\n",*id);
			ret = g_callback.i2c_close();
			if( 0 != ret )
			{
				result = YAS_ERROR_I2C;
			}
		}

		if( YAS_NO_ERROR == result )
		{
			yas_update_state( YAS_TEST1 );
		}
	}

	return result;
}

/* test 2 */
static int
power_off( void )
{
	int result = yas_check_state( YAS_TEST2 );

	if( YAS_NO_ERROR == result )
	{
		result = yas_test2();

		if( YAS_NO_ERROR == result )
		{
			yas_update_state( YAS_TEST2 );
		}
	}

	return result;
}

/* test 3 */
static int
initialization( void )
{
	int result = yas_check_state( YAS_TEST3 );
	int ret;

	if( YAS_NO_ERROR == result )
	{
		result = YAS_ERROR_I2C;
		ret = g_callback.i2c_open();
		if( 0 == ret )
		{
			result = yas_test3();
			ret = g_callback.i2c_close();
			if( 0 != ret )
			{
				result = YAS_ERROR_I2C;
			}
		}

		if( YAS_NO_ERROR == result )
		{
			yas_update_state( YAS_TEST3 );
		}
	}

	return result;
}

/* test 4 */
static int
offset_control_measurement_and_set_offset_register( int *x, int *y1, int *y2 )
{
	int result = yas_check_state( YAS_TEST4 );
	int ret;
	printk("yas_check_state result=%d\n",result);
	if( YAS_NO_ERROR == result )
	{
	    printk("offset_control_measurement_and_set_offset_register  111 \n");
		result = YAS_ERROR_I2C;
		ret = g_callback.i2c_open();
	    printk("offset_control_measurement_and_set_offset_register  222 \n");
		if( 0 == ret )
		{
			printk("offset_control_measurement_and_set_offset_register  333 \n");
			result = yas_test4( x, y1, y2 );
			ret = g_callback.i2c_close();
			printk("offset_control_measurement_and_set_offset_register  444 \n");
			if( 0 != ret )
			{
				result = YAS_ERROR_I2C;
			}
			printk("offset_control_measurement_and_set_offset_register  555 \n");
		}
		printk("offset_control_measurement_and_set_offset_register  666 \n");
		if( YAS_NO_ERROR == result )
		{
			printk("offset_control_measurement_and_set_offset_register  777 \n");
			yas_update_state( YAS_TEST4 );
		}
	}
	printk("offset_control_measurement_and_set_offset_register  888 \n");
	return result;
}

/* test 5 */
#if 0
static int
direction_measurement( int *direction )
{
	int result = yas_check_state( YAS_TEST5 );
	int ret;

	if( YAS_NO_ERROR == result )
	{
		result = YAS_ERROR_I2C;
		ret = g_callback.i2c_open();
		if( 0 == ret )
		{
			result = yas_test5( direction );
			ret = g_callback.i2c_close();
			if( 0 != ret )
			{
				result = YAS_ERROR_I2C;
			}
		}

		if( YAS_NO_ERROR == result )
		{
			yas_update_state( YAS_TEST5 );
		}
	}

	return result;
}
#endif
/* test 6 */
static int
sensitivity_measurement_of_magnetic_sensor_by_test_coil( int *sx, int *sy )
{
	int result = yas_check_state( YAS_TEST6 );
	int ret;
	printk("sensitivity_measurement_of_magnetic_sensor_by_test_coil  111 \n");
	if( YAS_NO_ERROR == result )
	{
		printk("sensitivity_measurement_of_magnetic_sensor_by_test_coil  222 \n");
		result = YAS_ERROR_I2C;
		ret = g_callback.i2c_open();
		if( 0 == ret )
		{
			printk("sensitivity_measurement_of_magnetic_sensor_by_test_coil  333 \n");
			result = yas_test6( sx, sy );
			ret = g_callback.i2c_close();
			printk("sensitivity_measurement_of_magnetic_sensor_by_test_coil  444 \n");
			if( 0 != ret )
			{
				printk("sensitivity_measurement_of_magnetic_sensor_by_test_coil  555 \n");
				result = YAS_ERROR_I2C;
			}
		}
		printk("sensitivity_measurement_of_magnetic_sensor_by_test_coil  666 \n");
		if( YAS_NO_ERROR == result )
		{
			printk("sensitivity_measurement_of_magnetic_sensor_by_test_coil  777 \n");
			yas_update_state( YAS_TEST6 );
		}
	}
	printk("sensitivity_measurement_of_magnetic_sensor_by_test_coil  888 \n");
	return result;
}

/* test 7 */
static int
magnetic_field_level_check( int *ohx, int *ohy, int *ohz )
{
#ifdef YAS_PCBTEST_EXTRA
	int result = yas_check_state( YAS_TEST7 );

	if( YAS_NO_ERROR == result )
	{
		result = yas_test7( ohx, ohy, ohz );

		if( YAS_NO_ERROR == result )
		{
			yas_update_state( YAS_TEST7 );
		}
	}

	return result;
#else
	return YAS_ERROR_NOT_SUPPORTED;
#endif
}

/* pcb test module initialize */
int
yas_pcb_test_init( struct yas_pcb_test *func )
{
	int result = YAS_ERROR_ARG;

	if( ( NULL != func )
	 && ( NULL != func->callback.i2c_open )
	 && ( NULL != func->callback.i2c_close )
	 && ( NULL != func->callback.i2c_write )
	 && ( NULL != func->callback.i2c_read )
	 && ( NULL != func->callback.msleep ) )
	{
		func->power_on_and_device_check = power_on_and_device_check;
		func->initialization = initialization;
		func->offset_control_measurement_and_set_offset_register = offset_control_measurement_and_set_offset_register;
	//	func->direction_measurement = direction_measurement;
		func->sensitivity_measurement_of_magnetic_sensor_by_test_coil = sensitivity_measurement_of_magnetic_sensor_by_test_coil;
		func->magnetic_field_level_check = magnetic_field_level_check;
		func->power_off = power_off;


        func->callback.i2c_close=  i2c_close;
		func->callback.i2c_open =  i2c_open ;
		func->callback.i2c_read  = i2c_read ;
		func->callback.i2c_write = i2c_write ;
		func->callback.msleep   =  msleep;
		func->callback.power_off  = yas530_power_off;
		func->callback.power_on  = yas530_power_on;
		func->callback.read_intpin  =  read_intpin;
		

		g_callback = func->callback;

		if( YAS_TEST1 != gu08State )
		{
			yas_power_off();
		}

		gu08State = ( uint8_t )YAS_TEST1;

		result = YAS_NO_ERROR;
	}

	return result;
}

/* end of file */
