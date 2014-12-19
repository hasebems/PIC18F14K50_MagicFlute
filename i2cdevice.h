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

void initI2c( void );

void LPS331AP_init( void );
short LPS331AP_getPressure( void );

void BlinkM_init( void );
void BlinkM_changeColor( unsigned char note );

#ifdef	__cplusplus
}
#endif

#endif	/* I2CDEVICE_H */

