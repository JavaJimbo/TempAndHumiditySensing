/* 
 * File:   MAX31865_RTD.h
 * Author: Jim
 *
 * Created on January 28, 2020, 11:19 AM
 */

#include <plib.h>

#ifndef MAX31865_RTD_H
#define	MAX31865_RTD_H

#define false FALSE
#define true TRUE

#define MAX31865_CS LATAbits.LATA10

// VBias OFF, Mode 0, 1 shot, 3 Wire,   Fault, Fault status,  60 Hz
//         D7      D6     D5      D4      D3D2           D1      D0
//         0,      0,      1,      1,      00,            1,     0,
// #define MAX31865_CONFIG 0b00110010
#define MAX31856_CONFIG_FAULTSTAT 0x02

#define MAX31856_CONFIG_REG 0x00
#define MAX31856_CONFIG_BIAS 0x80
#define MAX31856_CONFIG_MODEAUTO 0x40
#define MAX31856_CONFIG_MODEOFF 0x00
#define MAX31856_CONFIG_1SHOT 0x20
#define MAX31856_CONFIG_3WIRE 0x10
#define MAX31856_CONFIG_24WIRE 0x00
#define MAX31856_CONFIG_FAULTSTAT 0x02
#define MAX31856_CONFIG_FILT50HZ 0x01
#define MAX31856_CONFIG_FILT60HZ 0x00

#define MAX31856_RTDMSB_REG 0x01
#define MAX31856_RTDLSB_REG 0x02
#define MAX31856_HFAULTMSB_REG 0x03
#define MAX31856_HFAULTLSB_REG 0x04
#define MAX31856_LFAULTMSB_REG 0x05
#define MAX31856_LFAULTLSB_REG 0x06
#define MAX31856_FAULTSTAT_REG 0x07

#define MAX31865_FAULT_HIGHTHRESH 0x80
#define MAX31865_FAULT_LOWTHRESH 0x40
#define MAX31865_FAULT_REFINLOW 0x20
#define MAX31865_FAULT_REFINHIGH 0x10
#define MAX31865_FAULT_RTDINLOW 0x08
#define MAX31865_FAULT_OVUV 0x04

#define CONFIG_REGISTER_WRITE 0x80

float TemperatureCalc(unsigned short RTintReading);
unsigned char CheckMAX31865Fault();
unsigned short ReadRawRTDTemperature();
void ConfigureMAX31865();


#endif	/* MAX31865_RTD_H */

