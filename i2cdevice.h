/* 
 * File:   i2cdevice.h
 * Author: jca03205
 *
 * Created on 2014/12/14, 18:25
 */

#ifndef I2CDEVICE_H
#define	I2CDEVICE_H

#ifdef	__cplusplus
extern "C" {
#endif

extern bool i2cErr;
    
void initI2c( void );

void LPS331AP_init( void );
int LPS331AP_getPressure( int* retPrs );

void MPR121_init( void );
int MPR121_getTchSwData( unsigned char* retSw );

void ADXL345_init( void );
int ADXL345_getAccel( signed short* value );

void ADS1015_init( void );
void ADS1015_setNext( int adNum );
int ADS1015_getVolume( unsigned char* reg );

void BlinkM_init( void );
int BlinkM_changeColor( unsigned char note );

#ifdef	__cplusplus
}
#endif

#endif	/* I2CDEVICE_H */

