/*
 * File:   i2cdevice.c
 * Author: M.Hasebe
 *
 * Created on 2014/12/14
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <xc.h>

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
//			I2C Basic Functions
//-------------------------------------------------------------------------
//#ifndef _XTAL_FREQ
    /* 例：4MHzの場合、4000000 をセットする */
//    #define _XTAL_FREQ 48000000
//#endif

bool i2cErr;
unsigned char i2cAdrs;
//-------------------------------------------------------------------------
void initI2c( void )
{
    SSPSTAT = 0b00000000;      // I2C 400kHz
    SSPADD = 0x1d;             // I2Cbus Baud rate,  48MHz/((SSP1ADD + 1)*4) = 400kHz -> 0x1d, 100kHz -> 0x77
    SSPCON1 = 0b00001000;      // I2C enable, Master Mode
	SSPCON2 = 0x00;
	i2cErr = false;
	i2cAdrs = 0;
	SSPCON1bits.SSPEN = 1;		//	I2C enable
}
//-------------------------------------------------------------------------
void quitI2c( void )
{
    SSPCON1bits.SSPEN = 0;      // I2C disable
}
//-------------------------------------------------------------------------
// I2C通信がビジー状態を脱するまで待つ
bool i2c_check(void){
	volatile int cnt=0;
    while ( ( SSPCON2 & 0x5F ) || ( SSPSTAT & 0x05 ) ){	//	Buffer Full Check
		if ( cnt++ > 1000 ){
			i2cErr = ((SSPCON2 & 0x18)&&(i2cAdrs == TOUCH_SENSOR_ADDRESS))? true:false;
			return false;
		}
	}
	return true;
}
//-------------------------------------------------------------------------
bool i2c_wait(void){
	volatile int cnt=0;
    while ( ( SSPCON2 & 0x5F ) || ( SSPSTAT & 0x04 ) ){
		if ( cnt++ > 1000 ){
			i2cErr = ((SSPCON2 & 0x18)&&(i2cAdrs == TOUCH_SENSOR_ADDRESS))? true:false;
			return false;
		}
	}
	return true;
}
//-------------------------------------------------------------------------
void i2c_err(void)
{
	SSPCON1bits.WCOL=0;
	SSPCON2bits.PEN = 1;
	while(SSPCON2bits.PEN);	
}

//-------------------------------------------------------------------------
//			I2c Device Access Functions
//-------------------------------------------------------------------------
void writeI2c( unsigned char adrs, unsigned char data )
{
	if ( i2c_check() == false ){i2c_err(); return;}

	SSPCON2bits.SEN = 1;       //  Start Condition Enabled bit
	while(SSPCON2bits.SEN);

	SSPBUF = (adrs<<1) | I2C_WRITE_CMD;
    if ( i2c_check() == false ){i2c_err(); return;}
	//while(SSPCON2bits.ACKSTAT==1);

	SSPBUF = data;
    if ( i2c_check() == false ){i2c_err(); return;}
	//while(SSPCON2bits.ACKSTAT==1);

	SSPCON2bits.PEN = 1;       // Stop Condition Enable bit
	while(SSPCON2bits.PEN);
	
//	i2c_start();
//	i2c_send_byte((adrs<<1) | I2C_WRITE_CMD);   // アドレスを1ビット左にシフトし、末尾にR/Wビット（Write=0）を付与
//	i2c_send_byte(data);
//	i2c_stop();
}
//-------------------------------------------------------------------------
void writeI2cWithCmd( unsigned char adrs, unsigned char cmd, unsigned char data )
{
    if ( i2c_check() == false ){i2c_err(); return;}

	SSPCON2bits.SEN = 1;       //  Start Condition Enabled bit
	while(SSPCON2bits.SEN);

	SSPBUF = (adrs<<1) | I2C_WRITE_CMD;
    if ( i2c_check() == false ){i2c_err(); return;}
	//while(SSPCON2bits.ACKSTAT==1);

	SSPBUF = cmd;
    if ( i2c_check() == false ){i2c_err(); return;}
	//while(SSPCON2bits.ACKSTAT==1);

	SSPBUF = data;
    if ( i2c_check() == false ){i2c_err(); return;}
	//while(SSPCON2bits.ACKSTAT==1);

	SSPCON2bits.PEN = 1;       // Stop Condition Enable bit
	while(SSPCON2bits.PEN);

//	i2c_start();
//	i2c_send_byte((adrs<<1) | I2C_WRITE_CMD);   // アドレスを1ビット左にシフトし、末尾にR/Wビット（Write=0）を付与
//	i2c_send_byte(cmd);
//	i2c_send_byte(data);
//	i2c_stop();
}
//-------------------------------------------------------------------------
void writeI2cWithCmdAndMultiData( unsigned char adrs, unsigned char cmd, unsigned char* data, int length )
{
	int i=0;

    if ( i2c_check() == false ){i2c_err(); return;}

	SSPCON2bits.SEN = 1;       //  Start Condition Enabled bit
	while(SSPCON2bits.SEN);

	SSPBUF = (adrs<<1) | I2C_WRITE_CMD;
    if ( i2c_check() == false ){i2c_err(); return;}
	//while(SSPCON2bits.ACKSTAT==1);

	SSPBUF = cmd;
    if ( i2c_check() == false ){i2c_err(); return;}
	//while(SSPCON2bits.ACKSTAT==1);

	while (i<length){
		SSPBUF = *(data+i);
		if ( i2c_check() == false ){i2c_err(); return;}
		//while(SSPCON2bits.ACKSTAT==1);
		i++;
	}

	SSPCON2bits.PEN = 1;       // Stop Condition Enable bit
	while(SSPCON2bits.PEN);

//	i2c_start();
//	i2c_send_byte((adrs<<1) | I2C_WRITE_CMD);   // アドレスを1ビット左にシフトし、末尾にR/Wビット（Write=0）を付与
//	i2c_send_byte(cmd);
//	while (i<length){
//		i2c_send_byte(*(data+i));
//		i++;
//	}
//	i2c_stop();
}
//-------------------------------------------------------------------------
//void readI2c( unsigned char adrs, unsigned char* data )
//{
//	i2c_start();
//	i2c_send_byte((adrs<<1) | I2C_READ_CMD);   // アドレスを1ビット左にシフトし、末尾にR/Wビット（Read=1）を付与
//	*data = i2c_read_byte(0);
//	i2c_stop();
//}
//-------------------------------------------------------------------------
void readI2cWithCmd( unsigned char adrs, unsigned char cmd, unsigned char* data, int length )
{
	int i=0;

    if ( i2c_check() == false ){i2c_err(); return;}

	SSPCON2bits.SEN = 1;       //  Start Condition Enabled bit
	while(SSPCON2bits.SEN);

	SSPBUF = (adrs<<1) | I2C_WRITE_CMD;
    if ( i2c_check() == false ){i2c_err(); return;}
	//while(SSPCON2bits.ACKSTAT==1);

	SSPBUF = cmd;
    if ( i2c_check() == false ){i2c_err(); return;}
	//while(SSPCON2bits.ACKSTAT==1);

    SSPCON2bits.RSEN = 1;      //  Start Condition Enabled bit
	while(SSPCON2bits.RSEN);

	SSPBUF = (adrs<<1) | I2C_READ_CMD;
    if ( i2c_check() == false ){i2c_err(); return;}
	//while(SSPCON2bits.ACKSTAT==1);

	while (i<length){

		if ( i2c_check() == false ){i2c_err(); return;}
		SSPCON2bits.RCEN = 1;
		if ( i2c_wait() == false ){i2c_err(); return;}
		*(data+i) = SSPBUF;
		if ( i2c_check() == false ){i2c_err(); return;}

		if ( length > i+1 )	SSPCON2bits.ACKDT = 0;     // ACK
		else				SSPCON2bits.ACKDT = 1;     // NO_ACK
		SSPCON2bits.ACKEN = 1;
		if ( i2c_check() == false ){i2c_err(); return;}

		i++;
	}

	SSPCON2bits.PEN = 1;       // Stop Condition Enable bit
	while(SSPCON2bits.PEN);

//	i2c_start();
//	i2c_send_byte((adrs<<1) | I2C_WRITE_CMD);   // アドレスを1ビット左にシフトし、末尾にR/Wビット（Write=0）を付与
//	i2c_send_byte(cmd);
//	i2c_repeat_start();
//	i2c_send_byte((adrs<<1) | I2C_READ_CMD);   // アドレスを1ビット左にシフトし、末尾にR/Wビット（Read=1）を付与
//	while (i<length){
//		if ( length > i+1 ){
//			*(data+i) = i2c_read_byte(1);
//		}
//		else {	//	Final
//			*(data+i) = i2c_read_byte(0);
//		}
//		i++;
//	}
//	i2c_stop();
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
	i2cAdrs = PRESSURE_SENSOR_ADDRESS;
	writeI2cWithCmd( PRESSURE_SENSOR_ADDRESS, PRES_SNCR_RESOLUTION, 0x6A );	//	Resolution
	writeI2cWithCmd( PRESSURE_SENSOR_ADDRESS, PRES_SNCR_PWRON, 0xf0 );	//	Power On
}
//-------------------------------------------------------------------------
int LPS331AP_getPressure( void )
{
	unsigned char	dt[3];
	float	tmpPrs = 0;

	i2cAdrs = PRESSURE_SENSOR_ADDRESS;
	readI2cWithCmd( PRESSURE_SENSOR_ADDRESS, PRES_SNCR_DT_L|0x80, dt, 3 );
	tmpPrs = (float)(((unsigned long)dt[2]<<16)|((unsigned long)dt[1]<<8)|dt[0]);
	tmpPrs = tmpPrs*10/4096;

	return (int)tmpPrs;
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
//-------------------------------------------------------------------------
void MPR121_init( void )
{
	int	i, j;

	i2cAdrs = TOUCH_SENSOR_ADDRESS;
	//	Init Parameter
	// Put the MPR into setup mode
	writeI2cWithCmd( TOUCH_SENSOR_ADDRESS, TCH_SNCR_ELE_CFG, 0x00 );

    // Electrode filters for when data is > baseline
    const unsigned char gtBaseline[] = {
		0x01,  //MHD_R
		0x01,  //NHD_R
		0x00,  //NCL_R
		0x00   //FDL_R
	};
	for ( i=0; i<4; i++ )
		writeI2cWithCmd( TOUCH_SENSOR_ADDRESS, TCH_SNCR_MHD_R+i, gtBaseline[i] );


	// Electrode filters for when data is < baseline
	const unsigned char ltBaseline[] = {
        0x01,   //MHD_F
        0x01,   //NHD_F
        0xFF,   //NCL_F
        0x02    //FDL_F
	};
	for ( i=0; i<4; i++ )
		writeI2cWithCmd( TOUCH_SENSOR_ADDRESS, TCH_SNCR_MHD_F+i, ltBaseline[i] );

    // Electrode touch and release thresholds
    const unsigned char electrodeThresholds[] = {
        E_THR_T, // Touch Threshhold
        E_THR_R  // Release Threshold
	};

    for( j=0; j<12; j++ ){
		for ( i=0; i<2; i++ ){
			writeI2cWithCmd( TOUCH_SENSOR_ADDRESS, TCH_SNCR_ELE0_T+(j*2)+i, electrodeThresholds[i] );
    	}
	}

    // Proximity Settings
    const unsigned char proximitySettings[] = {
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
    for ( i=0; i<11; i++ )
		writeI2cWithCmd( TOUCH_SENSOR_ADDRESS, TCH_SNCR_MHDPROXR+i, proximitySettings[i] );

    const unsigned char proxThresh[] = {
        PROX_THR_T, // Touch Threshold
        PROX_THR_R  // Release Threshold
	};
    for ( i=0; i<2; i++ )
		writeI2cWithCmd( TOUCH_SENSOR_ADDRESS, TCH_SNCR_EPROXTTH+i, proxThresh[i] );

	writeI2cWithCmd( TOUCH_SENSOR_ADDRESS, TCH_SNCR_FIL_CFG, 0x04 );

    // Set the electrode config to transition to active mode
	writeI2cWithCmd( TOUCH_SENSOR_ADDRESS, TCH_SNCR_ELE_CFG, 0x0c );
}
//-------------------------------------------------------------------------
unsigned char MPR121_getTchSwData( void )
{
	unsigned char buf[2];

	i2cAdrs = TOUCH_SENSOR_ADDRESS;
	readI2cWithCmd( TOUCH_SENSOR_ADDRESS, TCH_SNCR_TOUCH_STATUS1, buf, 2 );

//	return (buf[1]<<8) | buf[0];
	return buf[0];
}

//-------------------------------------------------------------------------
//			ADXL345 (Acceleration Sencer : I2c Device)
//-------------------------------------------------------------------------
//	for Acceleration Sencer
#define ACCEL_SNCR_PWR_CTRL			0x2d
#define ACCEL_SNCR_DATA_FORMAT		0x31
//-------------------------------------------------------------------------
void ADXL345_init( void )
{
	//	Start Access
	i2cAdrs = ACCEL_SENSOR_ADDRESS;
	writeI2cWithCmd(ACCEL_SENSOR_ADDRESS,ACCEL_SNCR_PWR_CTRL,0x08);			//	Start Measurement
	writeI2cWithCmd(ACCEL_SENSOR_ADDRESS,ACCEL_SNCR_DATA_FORMAT,0x04);		//	Left Justified
}
//-------------------------------------------------------------------------
void ADXL345_getAccel( signed short* value )
{
	unsigned short tmp;
	unsigned char reg[2];

	i2cAdrs = ACCEL_SENSOR_ADDRESS;

	readI2cWithCmd(ACCEL_SENSOR_ADDRESS,0x32,reg,2);
	tmp = reg[0];
	tmp |= reg[1] << 8;
	*value = (signed short)tmp;

	readI2cWithCmd(ACCEL_SENSOR_ADDRESS,0x34,reg,2);
	tmp = reg[0];
	tmp |= reg[1] << 8;
	*(value+1) = (signed short)tmp;

	readI2cWithCmd(ACCEL_SENSOR_ADDRESS,0x36,reg,2);
	tmp = reg[0];
	tmp |= reg[1] << 8;
	*(value+2) = (signed short)tmp;
}

//-------------------------------------------------------------------------
//			BlinkM ( Full Color LED : I2c Device)
//-------------------------------------------------------------------------
void BlinkM_init( void )
{
	unsigned char color[3] = {0x00,0x00,0x00};

	i2cAdrs = LED_BLINKM_ADDRESS;
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
	i2cAdrs = LED_BLINKM_ADDRESS;
	writeI2cWithCmdAndMultiData( LED_BLINKM_ADDRESS, 'c', (unsigned char*)tNoteToColor[note], 3 );
}
