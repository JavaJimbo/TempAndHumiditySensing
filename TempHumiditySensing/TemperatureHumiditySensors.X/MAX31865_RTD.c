/* 
 * File:   MAX31865_RTD.h
 * Routines for reading MAX31865 RTD-to-Digital converter 
 * and converting raw RTD data to temperature.
 * Adapted from Arduino example code from Adafruit MAX31865 breakout board.
 * 
 * Author: Jim Sedgwick
 * 
 * Created on January 28, 2020, 11:19 AM
 */

#include "MAX31865_RTD.h"
#include <math.h>

/**************************************************************************/
/*!
    @brief Read the temperature in C from the RTD through calculation of the
    resistance. Uses
   http://www.analog.com/media/en/technical-documentation/application-notes/AN709_0.pdf
   technique
    @param RTDnominal The 'nominal' resistance of the RTD sensor, usually 100
    or 1000
    @param refResistor The value of the matching reference resistor, usually
    430 or 4300
    @returns Temperature in C
*/
/**************************************************************************/
float TemperatureCalc(unsigned short RTintReading)
{
  float Z1, Z2, Z3, Z4, Rt, temp;
  #define RTDnominal 1000
  #define refResistor 4300

#define RTD_A 3.9083e-3
#define RTD_B -5.775e-7  
  
  Rt = (float) RTintReading;
  Rt /= 32768;
  Rt *= refResistor;

  // Serial.print("\nResistance: "); Serial.println(Rt, 8);

  Z1 = -RTD_A;
  Z2 = RTD_A * RTD_A - (4 * RTD_B);
  Z3 = (4 * RTD_B) / RTDnominal;
  Z4 = 2 * RTD_B;

  temp = Z2 + (Z3 * Rt);
  temp = (sqrt(temp) + Z1) / Z4;

  if (temp >= 0)
    return temp;

  // ugh.
  Rt /= RTDnominal;
  Rt *= 100; // normalize to 100 ohm

  float rpoly = Rt;

  temp = -242.02;
  temp += 2.2228 * rpoly;
  rpoly *= Rt; // square
  temp += 2.5859e-3 * rpoly;
  rpoly *= Rt; // ^3
  temp -= 4.8260e-6 * rpoly;
  rpoly *= Rt; // ^4
  temp -= 2.8183e-8 * rpoly;
  rpoly *= Rt; // ^5
  temp += 1.5243e-10 * rpoly;

  return temp;
}

// VBIAS ON = 1, AUTO off = 0, CLEAR 1 Shot = 0, RTD 3 Wire = 1, Fault = 00, Fault status clear = 0, 60HZ = 0
#define STARTUP_CONFIGURATION 0b10010000
// To clear faults use:
#define MAX31856_CLEAR_FAULT (STARTUP_CONFIGURATION | MAX31856_CONFIG_FAULTSTAT)

unsigned char CheckMAX31865Fault()
{
    unsigned char fault, dummyByte;
    
    // Read Fault register:
    MAX31865_CS = 0;
    dummyByte = SendReceiveSPI(2, MAX31856_FAULTSTAT_REG);
    fault = SendReceiveSPI(2, 0x00);
    MAX31865_CS = 1;      
            
    if (fault) 
    {
        printf("\rFault register: %02X", fault);
                
        if (fault & MAX31865_FAULT_HIGHTHRESH) 
                    printf("\rRTD High Threshold"); 
        if (fault & MAX31865_FAULT_LOWTHRESH) 
                    printf("\rRTD Low Threshold"); 
        if (fault & MAX31865_FAULT_REFINLOW) 
                    printf("\rREFIN- > 0.85 x Bias"); 
        if (fault & MAX31865_FAULT_REFINHIGH) 
                    printf("\rREFIN- < 0.85 x Bias - FORCE- open"); 
        if (fault & MAX31865_FAULT_RTDINLOW) 
                    printf("\rRTDIN- < 0.85 x Bias - FORCE- open"); 
        if (fault & MAX31865_FAULT_OVUV) 
                    printf("\rUnder/Over voltage"); 
        
        // Now clear fault bits in config register        
        MAX31865_CS = 0;
        dummyByte = SendReceiveSPI(2, CONFIG_REGISTER_WRITE);
        dummyByte = SendReceiveSPI(2, MAX31856_CLEAR_FAULT);        
        MAX31865_CS = 1;   
        
        return false;
    }
    return true;
}

unsigned short ReadRawRTDTemperature()
{
    unsigned char LSBbyte, MSBbyte, dummyByte, DataInByte, DataOutByte;
    unsigned short tempInt, RawTemperatureInt = 0x0000;
    
    // Read Config:
    MAX31865_CS = 0;
    dummyByte = SendReceiveSPI(2, 0x00);
    DataInByte = SendReceiveSPI(2, 0x00);
    MAX31865_CS = 1;        

    // Set One Shot to start conversion
    DataOutByte = DataInByte | MAX31856_CONFIG_1SHOT;
    MAX31865_CS = 0;
    dummyByte = SendReceiveSPI(2, CONFIG_REGISTER_WRITE);
    DataInByte = SendReceiveSPI(2, DataOutByte);        
    MAX31865_CS = 1;        

    // Wait for conversion
    DelayMs(100);  // Was 200          
            
    // Read  RTD MSB and LSB:
    MAX31865_CS = 0;
    dummyByte = SendReceiveSPI(2, MAX31856_RTDMSB_REG);
    MSBbyte = SendReceiveSPI(2, 0x00);
    LSBbyte = SendReceiveSPI(2, 0x00);        
    MAX31865_CS = 1;    
            
    tempInt = MSBbyte;
    tempInt = tempInt << 8;
    tempInt = tempInt | LSBbyte;         
    tempInt = tempInt >> 1;  
    
    RawTemperatureInt = tempInt;
    return RawTemperatureInt;
}

void ConfigureMAX31865()
{
unsigned char dummyByte;

    MAX31865_CS = 0;
    DelayMs(10);
    dummyByte = SendReceiveSPI(2, CONFIG_REGISTER_WRITE);
    dummyByte = SendReceiveSPI(2, STARTUP_CONFIGURATION);        
    MAX31865_CS = 1;        
}
