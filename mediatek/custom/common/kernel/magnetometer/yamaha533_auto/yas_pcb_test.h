/*=============================================================================*/
/* CONFIDENTIAL																   */
/* Copyright(c) 2010-2012 Yamaha Corporation								   */
/*=============================================================================*/

/*=============================================================================*/
/* File 	yas_pcb_test.h													   */
/* Date 	2012/03/07														   */
/* Revision	1.3.0															   */
/*=============================================================================*/

#ifndef __YAS_PCB_TEST_H__
#define __YAS_PCB_TEST_H__

#include "yas_types.h"
#include <linux/types.h>
#include <linux/list.h>
#define	YAS_PCBTEST_EXTRA

/* error code */
#define YAS_NO_ERROR				( 0 )
#define YAS_ERROR_I2C				( -1 )
#define YAS_ERROR_POWER 			( -2 )
#define YAS_ERROR_TEST_ORDER		( -3 )
#define YAS_ERROR_INTERRUPT			( -4 )
#define YAS_ERROR_BUSY				( -5 )
#define YAS_ERROR_OVERFLOW			( -6 )
#define YAS_ERROR_UNDERFLOW			( -7 )
#define YAS_ERROR_DIRCALC			( -8 )
#define YAS_ERROR_NOT_SUPPORTED		( -9 )
#define YAS_ERROR_CALREG			( -10 )
#define YAS_ERROR_ARG				( -128 )

/* addr */
#define YAS_ADDR_SLAVE				( 0x2E )

#define YAS_ADDR_ID					( 0x80 )
#define YAS_ADDR_COIL				( 0x81 )
#define YAS_ADDR_MEASURE_COMMAND	( 0x82 )
#define YAS_ADDR_CONFIG				( 0x83 )
#define YAS_ADDR_MEASURE_INTERVAL	( 0x84 )
#define YAS_ADDR_OFFSET				( 0x85 )
#define YAS_ADDR_TEST1				( 0x88 )
#define YAS_ADDR_TEST2				( 0x89 )
#define YAS_ADDR_CAL				( 0x90 )
#define YAS_ADDR_MEASURE_DATA		( 0xB0 )

/* V Core */
#define YAS_VCORE					( 28)

/* typedef */
/*
struct yas_vector
{
	int32_t v[ 3 ];
};*/

struct yas_pcb_test_callback
{
	int	 ( *power_on )( void );
	int	 ( *power_off )( void );
	int	 ( *i2c_open )( void );
	int	 ( *i2c_close )( void );
	int	 ( *i2c_write )( uint8_t slave, uint8_t addr, const uint8_t *buf, int len );
	int	 ( *i2c_read )( uint8_t slave, uint8_t addr, uint8_t *buf, int len );
	void ( *msleep )( int msec );
	int	 ( *read_intpin )( int *low_or_high );
};

struct yas_pcb_test
{
	int	( *power_on_and_device_check )( int *id );
	int	( *initialization )( void );
	int	( *offset_control_measurement_and_set_offset_register )( int *x, int *y1, int *y2 );
	int ( *direction_measurement )( int *direction );
	int	( *sensitivity_measurement_of_magnetic_sensor_by_test_coil )( int *sx, int *sy );
	int	( *magnetic_field_level_check )( int *ohx, int *ohy, int *ohz );
	int	( *power_off )( void );
	struct yas_pcb_test_callback callback;
};

/* prototype functions */
#ifdef __cplusplus
extern "C" {
#endif

int yas_pcb_test_init( struct yas_pcb_test *func );

#ifdef __cplusplus
}
#endif

#endif	/* ! __YAS_PCB_TEST_H__ */

/* end of file */
