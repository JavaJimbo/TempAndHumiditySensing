/* 
 * File:   SHT31_I2C.h
 * Author: Jim Sedgwick
 *
 * Created Jan 28, 2020
 */

#include "GenericTypeDefs.h"

#ifndef SHT31_I2C_H
#define	SHT31_I2C_H

#define SHT31_MEASUREMENT_DELAY 100
#define SHT31_I2C_ADDRESS 0x44

#define UINT32 unsigned long
#define BOOL unsigned char
#define UINT8 unsigned char

unsigned char I2CReceiveByte(unsigned char busID, unsigned char NACKflag);
unsigned char ReadSensirionSHT31(unsigned char busID, unsigned char device, unsigned short SHT31command, unsigned char *ptrData, unsigned char numBytes);
void ConvertSensirionHumidityTempX100(unsigned char *ptrSensirionData, long *Humidity, long *TempCelsius);

void initI2C(unsigned char busID);
BOOL StartTransfer(unsigned char busID, BOOL restart);
BOOL TransmitOneByte(unsigned char busID, unsigned char data);
void StopTransfer(unsigned char busID);
void print_status(unsigned char busID);
unsigned char I2CGetACK(unsigned char busID);


#endif	/* SHT31_I2C_H */

