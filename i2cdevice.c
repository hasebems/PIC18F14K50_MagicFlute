/*
 * File:   i2cdevice.c
 * Author: M.Hasebe
 *
 * Created on 2014/12/14
 */

#include <stdio.h>
#include <stdlib.h>

#include "i2c-lib.h"

//-------------------------------------------------------------------------
//			Constants
//-------------------------------------------------------------------------
//static unsigned char GPIO_EXPANDER_ADDRESS = 0x3e;
static unsigned char PRESSURE_SENSOR_ADDRESS = 0x5d;
static unsigned char TOUCH_SENSOR_ADDRESS = 0x5a;
static unsigned char LED_BLINKM_ADDRESS = 0x09;
//static unsigned char ADC_ADDRESS = 0x48;
//static unsigned char LED_ADA88_ADDRESS = 0x70;
static unsigned char ACCEL_SENSOR_ADDRESS = 0x1d;

// I2C Bus Control Definition
#define I2C_WRITE_CMD 0
#define I2C_READ_CMD 1


//-------------------------------------------------------------------------
//			I2c Device Access Functions
//-------------------------------------------------------------------------
void initI2c( void )
{
	i2c_enable();
}
//-------------------------------------------------------------------------
void quitI2c( void )
{
	i2c_disable();
}
//-------------------------------------------------------------------------
void writeI2c( unsigned char adrs, unsigned char data )
{
	i2c_start();
	i2c_send_byte((adrs<<1) | I2C_WRITE_CMD);   // アドレスを1ビット左にシフトし、末尾にR/Wビット（Write=0）を付与
	i2c_send_byte(data);
	i2c_stop();
}
//-------------------------------------------------------------------------
void writeI2cWithCmd( unsigned char adrs, unsigned char cmd, unsigned char data )
{
	i2c_start();
	i2c_send_byte((adrs<<1) | I2C_WRITE_CMD);   // アドレスを1ビット左にシフトし、末尾にR/Wビット（Write=0）を付与
	i2c_send_byte(cmd);
	i2c_send_byte(data);
	i2c_stop();
}
//-------------------------------------------------------------------------
void writeI2cWithCmdAndMultiData( unsigned char adrs, unsigned char cmd, unsigned char* data, int length )
{
	int i=0;
	i2c_start();
	i2c_send_byte((adrs<<1) | I2C_WRITE_CMD);   // アドレスを1ビット左にシフトし、末尾にR/Wビット（Write=0）を付与
	i2c_send_byte(cmd);
	while (i<length){
		i2c_send_byte(*(data+i));
		i++;
	}
	i2c_stop();
}
//-------------------------------------------------------------------------
unsigned char readI2c( unsigned char adrs )
{
	unsigned char data;
	i2c_start();
	i2c_send_byte((adrs<<1) | I2C_READ_CMD);   // アドレスを1ビット左にシフトし、末尾にR/Wビット（Read=0）を付与
	data = i2c_read_byte(0);
	i2c_stop();
	return data;
}
//-------------------------------------------------------------------------
void readI2cWithCmd( unsigned char adrs, unsigned char cmd, unsigned char* data, int length )
{
	int i=0;
	i2c_start();
	i2c_send_byte((adrs<<1) | I2C_WRITE_CMD);   // アドレスを1ビット左にシフトし、末尾にR/Wビット（Write=0）を付与
	i2c_send_byte(cmd);

	i2c_repeat_start();
	i2c_send_byte((adrs<<1) | I2C_READ_CMD);   // アドレスを1ビット左にシフトし、末尾にR/Wビット（Read=0）を付与
	while (i<length){
		if ( length > i+1 ){
			*(data+i) = i2c_read_byte(1);
		}
		else {	//	Final
			*(data+i) = i2c_read_byte(0);
		}
		i++;
	}
	i2c_stop();
}


//-------------------------------------------------------------------------
//			LPS331AP (Pressure Sencer : I2c Device)
//-------------------------------------------------------------------------
//	for Pressure Sencer
#define		PRES_SNCR_RESOLUTION		0x10
#define		PRES_SNCR_PWRON				0x20
#define		PRES_SNCR_START				0x21
#define		PRES_SNCR_ONE_SHOT			0x01
#define		PRES_SNCR_RCV_DT_FLG		0x27
#define		PRES_SNCR_RCV_TMPR			0x01
#define		PRES_SNCR_RCV_PRES			0x02
#define		PRES_SNCR_DT_L				0x28
#define		PRES_SNCR_DT_M				0x29
#define		PRES_SNCR_DT_H				0x2a
//-------------------------------------------------------------------------
void LPS331AP_init( void )
{
	//	Init Parameter
	writeI2cWithCmd( PRESSURE_SENSOR_ADDRESS, PRES_SNCR_PWRON, 0x80 );	//	Power On
	writeI2cWithCmd( PRESSURE_SENSOR_ADDRESS, PRES_SNCR_RESOLUTION, 0x7A );	//	Resolution
	//	Pressure Sencer
	writeI2cWithCmd( PRESSURE_SENSOR_ADDRESS, PRES_SNCR_START, PRES_SNCR_ONE_SHOT );	//	Start One shot
}
//-------------------------------------------------------------------------
short LPS331AP_getPressure( void )
{
	unsigned char rdDt, dt[3];
	short	  prsData = 0;	//	can not get a value

	readI2cWithCmd( PRESSURE_SENSOR_ADDRESS, PRES_SNCR_RCV_DT_FLG, &rdDt, 1 );
	if ( rdDt & PRES_SNCR_RCV_PRES ){
		readI2cWithCmd( PRESSURE_SENSOR_ADDRESS, PRES_SNCR_DT_L, &dt[0], 1 );
		readI2cWithCmd( PRESSURE_SENSOR_ADDRESS, PRES_SNCR_DT_M, &dt[1], 1 );
		readI2cWithCmd( PRESSURE_SENSOR_ADDRESS, PRES_SNCR_DT_H, &dt[2], 1 );

		prsData = (dt[2]<<8)|dt[1];
		prsData = prsData*10/16;

		//	Pressure Sencer
		writeI2cWithCmd( PRESSURE_SENSOR_ADDRESS, PRES_SNCR_START, PRES_SNCR_ONE_SHOT );	//	Start One shot
	}
	
	return prsData;	//	10 times of Pressure(hPa)
}

//-------------------------------------------------------------------------
//			MPR121 (Touch Sencer : I2c Device)
//-------------------------------------------------------------------------
//	for Touch Sencer
#define		TCH_SNCR_TOUCH_STATUS1		0x00
#define		TCH_SNCR_TOUCH_STATUS2		0x01
#define 	TCH_SNCR_ELE_CFG			0x5e
#define 	TCH_SNCR_MHD_R				0x2b
#define 	TCH_SNCR_MHD_F				0x2f
#define 	TCH_SNCR_ELE0_T				0x41
#define 	TCH_SNCR_FIL_CFG			0x5d
#define 	TCH_SNCR_MHDPROXR			0x36
#define 	TCH_SNCR_EPROXTTH			0x59

// Threshold defaults
#define		E_THR_T      0x02	// Electrode touch threshold
#define		E_THR_R      0x01	// Electrode release threshold
#define		PROX_THR_T   0x02	// Prox touch threshold
#define		PROX_THR_R   0x02	// Prox release threshold
#if 0
//-------------------------------------------------------------------------
void accessMPR121( void )
{
//	int		address = TOUCH_SENSOR_ADDRESS;  // I2C

	// Set Address
//	if (ioctl(i2cDscript, I2CSLAVE_, address) < 0){
//		printf("Unable to get bus access to talk to slave(TOUCH)\n");
//		exit(1);
//	}
}
//-------------------------------------------------------------------------
void initMPR121( void )
{
	int	i, j;

	//	Start Access
	accessMPR121();

	//	Init Parameter
	// Put the MPR into setup mode
    writeI2c(TCH_SNCR_ELE_CFG,0x00);

    // Electrode filters for when data is > baseline
    unsigned char gtBaseline[] = {
		0x01,  //MHD_R
		0x01,  //NHD_R
		0x00,  //NCL_R
		0x00   //FDL_R
	};
	for ( i=0; i<4; i++ ) writeI2c(TCH_SNCR_MHD_R+i,gtBaseline[i]);

	// Electrode filters for when data is < baseline
	unsigned char ltBaseline[] = {
        0x01,   //MHD_F
        0x01,   //NHD_F
        0xFF,   //NCL_F
        0x02    //FDL_F
	};
	for ( i=0; i<4; i++ ) writeI2c(TCH_SNCR_MHD_F+i,ltBaseline[i]);

    // Electrode touch and release thresholds
    unsigned char electrodeThresholds[] = {
        E_THR_T, // Touch Threshhold
        E_THR_R  // Release Threshold
	};

    for( j=0; j<12; j++ ){
		for ( i=0; i<2; i++ ){
        	writeI2c(TCH_SNCR_ELE0_T+(j*2)+i,electrodeThresholds[i]);
    	}
	}

    // Proximity Settings
    unsigned char proximitySettings[] = {
        0xff,   //MHD_Prox_R
        0xff,   //NHD_Prox_R
        0x00,   //NCL_Prox_R
        0x00,   //FDL_Prox_R
        0x01,   //MHD_Prox_F
        0x01,   //NHD_Prox_F
        0xff,   //NCL_Prox_F
        0xff,   //FDL_Prox_F
        0x00,   //NHD_Prox_T
        0x00,   //NCL_Prox_T
        0x00    //NFD_Prox_T
	};
    for ( i=0; i<11; i++ ) writeI2c(TCH_SNCR_MHDPROXR+i,proximitySettings[i]);

    unsigned char proxThresh[] = {
        PROX_THR_T, // Touch Threshold
        PROX_THR_R  // Release Threshold
	};
    for ( i=0; i<2; i++ ) writeI2c(TCH_SNCR_EPROXTTH+i,proxThresh[i]);

    writeI2c(TCH_SNCR_FIL_CFG,0x04);

    // Set the electrode config to transition to active mode
    writeI2c(TCH_SNCR_ELE_CFG,0x0c);
}
//-------------------------------------------------------------------------
unsigned short getTchSwData( void )
{
	unsigned char buf[2] = { 0xff, 0xff };

	//	Start Access
	accessMPR121();

	if (read(i2cDscript, buf, 2) != 2) {	// Read back data into buf[]
		printf("Unable to read from slave(Touch)\n");
		//exit(1);
		return 0xffff;
	}

	return (buf[1]<<8) | buf[0];
}
#endif

//-------------------------------------------------------------------------
//			ADXL345 (Acceleration Sencer : I2c Device)
//-------------------------------------------------------------------------
#if 0
//	for Acceleration Sencer
#define ACCEL_SNCR_PWR_CTRL			0x2d
#define ACCEL_SNCR_DATA_FORMAT		0x31
//-------------------------------------------------------------------------
void accessADXL345( void )
{
	int		address = ACCEL_SENSOR_ADDRESS;  // I2C

	// Set Address
	if (ioctl(i2cDscript, I2CSLAVE_, address) < 0){
		printf("Unable to get bus access to talk to slave(ACCEL)\n");
		exit(1);
	}
}
//-------------------------------------------------------------------------
void initADXL345( void )
{
	//	Start Access
	accessADXL345();
	writeI2c(ACCEL_SNCR_PWR_CTRL,0x08);			//	Start Measurement
	writeI2c(ACCEL_SNCR_DATA_FORMAT,0x04);		//	Left Justified
}
//-------------------------------------------------------------------------
void getAccel( signed short* value )
{
	unsigned short tmp;

	accessADXL345();
	tmp = readI2c(0x32);
	tmp |= readI2c(0x33) << 8;
	*value = (signed short)tmp;

	tmp = readI2c(0x34);
	tmp |= readI2c(0x35) << 8;
	*(value+1) = (signed short)tmp;

	tmp = readI2c(0x36);
	tmp |= readI2c(0x37) << 8;
	*(value+2) = (signed short)tmp;
}
#endif

//-------------------------------------------------------------------------
//			BlinkM ( Full Color LED : I2c Device)
//-------------------------------------------------------------------------
void BlinkM_init( void )
{
	unsigned char color[3] = {0x00,0x00,0x00};
	writeI2cWithCmd( LED_BLINKM_ADDRESS, 'o', 0 );
	writeI2cWithCmd( LED_BLINKM_ADDRESS, 'f', 80 );
	writeI2cWithCmdAndMultiData( LED_BLINKM_ADDRESS, 'n', color, 3 );
}
//-------------------------------------------------------------------------
const unsigned char tNoteToColor[13][3] = {
	//	R	  G		B
	{ 0xff, 0x00, 0x00 },
	{ 0xe0, 0x20, 0x00 },
	{ 0xc0, 0x40, 0x00 },
	{ 0xa0, 0x60, 0x00 },
	{ 0x80, 0x80, 0x00 },
	{ 0x00, 0xff, 0x00 },
	{ 0x00, 0x80, 0x80 },
	{ 0x00, 0x00, 0xff },
	{ 0x20, 0x00, 0xe0 },
	{ 0x40, 0x00, 0xc0 },
	{ 0x60, 0x00, 0xa0 },
	{ 0x80, 0x00, 0x80 },
	{ 0x00, 0x00, 0x00 }
};
//-------------------------------------------------------------------------
void BlinkM_changeColor( unsigned char note )
{
	writeI2cWithCmdAndMultiData( LED_BLINKM_ADDRESS, 'c', (unsigned char*)tNoteToColor[note], 3 );
}
