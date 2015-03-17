/* 
 * File:   mfconfig.h
 * Author: jca03205
 *
 * Created on 2015/01/31, 9:13
 */

#ifndef MFCONFIG_H
#define	MFCONFIG_H

#ifdef	__cplusplus
extern "C" {
#endif

#define		USE_I2C_PRESSURE_SENSOR			1
#define		USE_I2C_ACCELERATOR_SENSOR		1
#define		USE_I2C_TOUCH_SENSOR			1
#define		USE_I2C_BLINKM				0

#define         USE_PRESSURE_SENSOR_LPS25H              1   // 0:LPS331AP

#ifdef	__cplusplus
}
#endif

#endif	/* MFCONFIG_H */

