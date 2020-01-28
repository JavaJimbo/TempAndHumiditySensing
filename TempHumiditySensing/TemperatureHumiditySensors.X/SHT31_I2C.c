/* SHT31_I2C.c - I2C routines for reading/writing to Sensirion SHT31
 * 
 * For PIC32MX220F032D microcontroller which has two I2C ports
 * 
 * Author: Jim Sedgwick
 *
 * Created Jan 28, 2020 
 */
#include "SHT31_I2C.h"

#include <plib.h>

#define true TRUE
#define false FALSE
#define RD 1
#define WR 0
#define SYS_FREQ 80000000

void initI2C(unsigned char busID)
{                   
    I2CConfigure(busID, I2C_ENABLE_HIGH_SPEED | I2C_STOP_IN_IDLE | I2C_ENABLE_SMB_SUPPORT);
    I2CSetFrequency(busID, SYS_FREQ, 400000);
    I2CEnable(busID, TRUE);
}


//START TRANSFER
BOOL StartTransfer (unsigned char busID, BOOL restart)
{
    I2C_STATUS  status;  
    
    if(restart) I2CRepeatStart(busID);   // Start = 0;  Restart = 1
    else
    {
        // Wait for the bus to be idle, then start the transfer
        while( !I2CBusIsIdle(busID) );
        if(I2CStart(busID) != I2C_SUCCESS)
        {
            // printf("\rStart Error");
            return FALSE;
        }
    }
    // Wait for START to complete
    do {
        status = I2CGetStatus(busID);
    } while (!(status & I2C_START));    
    
    return TRUE;
}

//TRANSMIT ONE BYTE
BOOL TransmitOneByte(unsigned char busID, unsigned char data)
{
    // Wait for the transmitter to be ready   
    while(!I2CTransmitterIsReady(busID));
     
    //Transmit the byte
    if(I2CSendByte(busID, data) == I2C_MASTER_BUS_COLLISION)
    {
        printf("\rERROR: Master Bus Collision");
        return FALSE;
    }    
    // Wait for the transmission to finish
    while(!I2CTransmissionHasCompleted(busID));    
    return TRUE;
}

//STOP TRANSFER
void StopTransfer(unsigned char busID)
{
    I2C_STATUS  status;
    // Send the Stop signal
    I2CStop(busID);
    // Wait for the signal to complete
    do
    {
        status = I2CGetStatus(busID);
        //char chip[16];
        //sprintf(chip,"Stop: %x",status);
        DelayMs(1);
        // print_error(chip);

    } while ( !(status & I2C_STOP) );
}


/*
void print_status(unsigned char busID)
{
    I2C_STATUS  status;
    status = I2CGetStatus(busID);    
    printf("STATUS: %x",status);    
    DelayMs(500);
    I2CClearStatus (busID, I2C_ARBITRATION_LOSS);
}
*/

unsigned char I2CGetACK(unsigned char busID)
{
    switch (busID)
    {
        case I2C1:
            return(I2C1STATbits.ACKSTAT);
            break;
        case I2C2:
            return(I2C2STATbits.ACKSTAT);
            break;
            
/* The PIC 32MX220 has only two I2C ports, so ports 3-5 are commented out here:
        case I2C3:
            return(I2C3STATbits.ACKSTAT);
            break;
        case I2C4:
            return(I2C4STATbits.ACKSTAT);
            break;
        case I2C5:
            return(I2C5STATbits.ACKSTAT);
            break;
*/            
        default:
            return 0;
    }    
}

unsigned char I2CReceiveByte(unsigned char busID, unsigned char NACKflag)
{
unsigned char dataByte = 0;
    switch (busID)
    {
        case I2C1:
            I2C1CONbits.RCEN = 1;
            while (!I2CReceivedDataIsAvailable(busID)); 
            dataByte = I2C1RCV; // Read data byte from buffer
            if (NACKflag) I2C1CONbits.ACKDT = 1; // Send NACK to EEPROM
            else I2C1CONbits.ACKDT = 0; // Send ACK to EEPROM
            I2C1CONbits.ACKEN = 1; 
            while (I2C1CONbits.ACKEN == 1); // Wait till ACK/NACK sequence is over 
            break;
            
        case I2C2:
            I2C2CONbits.RCEN = 1;
            while (!I2CReceivedDataIsAvailable(busID)); 
            dataByte = I2C2RCV; // Read data byte from buffer
            if (NACKflag) I2C2CONbits.ACKDT = 1; // Send NACK to EEPROM
            else I2C2CONbits.ACKDT = 0; // Send ACK to EEPROM
            I2C2CONbits.ACKEN = 1; 
            while (I2C2CONbits.ACKEN == 1); // Wait till ACK/NACK sequence is over 
            break;
            
/* The PIC 32MX220 has only two I2C ports, so ports 3-5 are commented out here:            
        case I2C3:
            I2C3CONbits.RCEN = 1;
            while (!I2CReceivedDataIsAvailable(busID)); 
            dataByte = I2C3RCV; // Read data byte from buffer
            if (NACKflag) I2C3CONbits.ACKDT = 1; // Send NACK to EEPROM
            else I2C3CONbits.ACKDT = 0; // Send ACK to EEPROM
            I2C3CONbits.ACKEN = 1; 
            while (I2C3CONbits.ACKEN == 1); // Wait till ACK/NACK sequence is over 
            break;
            
        case I2C4:
            I2C4CONbits.RCEN = 1;
            while (!I2CReceivedDataIsAvailable(busID)); 
            dataByte = I2C4RCV; // Read data byte from buffer
            if (NACKflag) I2C4CONbits.ACKDT = 1; // Send NACK to EEPROM
            else I2C4CONbits.ACKDT = 0; // Send ACK to EEPROM
            I2C4CONbits.ACKEN = 1; 
            while (I2C4CONbits.ACKEN == 1); // Wait till ACK/NACK sequence is over 
            break;
            
        case I2C5:
            I2C5CONbits.RCEN = 1;
            while (!I2CReceivedDataIsAvailable(busID)); 
            dataByte = I2C5RCV; // Read data byte from buffer
            if (NACKflag) I2C5CONbits.ACKDT = 1; // Send NACK to EEPROM
            else I2C5CONbits.ACKDT = 0; // Send ACK to EEPROM
            I2C5CONbits.ACKEN = 1; 
            while (I2C5CONbits.ACKEN == 1); // Wait till ACK/NACK sequence is over 
            break;
*/ 
        default:
            break;            
    }
    return dataByte;
}



unsigned char ReadSensirionSHT31(unsigned char busID, unsigned char device, unsigned short SHT31command, unsigned char *ptrData, unsigned char numBytes) 
{
    short i;
    unsigned short command;
    unsigned char SHT31commandMSB, SHT31commandLSB;
    unsigned char I2CAddressByte = (device << 1) & 0xFE;
    
    command = SHT31command;
    SHT31commandLSB = (unsigned char) (command & 0x00FF);
    SHT31commandMSB = (unsigned char) ((command >> 8)& 0x00FF);
    
    INTClearFlag(INT_SOURCE_I2C(busID));
    
    StartTransfer(busID, false);

    if (!TransmitOneByte(busID, I2CAddressByte | WR)) return false; // Send WRITE command and I2C device address
    if (I2CGetACK(busID)) return 0; // Get ACK from EEPROM

    if (!TransmitOneByte(busID,  SHT31commandMSB)) return false; // Send EEPROM high address byte
    if (I2CGetACK(busID)) return 0; // Get ACK from EEPROM

    if (!TransmitOneByte(busID, SHT31commandLSB)) return false; // Send EEPROM low address byte
    if (I2CGetACK(busID)) return 0; // Get ACK from EEPROM      
    
    DelayMs(SHT31_MEASUREMENT_DELAY);
    
    StartTransfer(busID, true); // Now send START sequence again:
    
    if (!TransmitOneByte(busID, I2CAddressByte | RD)) return 0; // Now send ID with READ Command    
    if (I2CGetACK(busID)) return 0; // Get ACK from EEPROM

    // Now receive block of data:
    for (i = 0; i < numBytes; i++) 
    {
        if (i < numBytes - 1) ptrData[i] = I2CReceiveByte(busID, false);            
        else ptrData[i] = I2CReceiveByte(busID, true);
    }
    StopTransfer(busID);
    
    return true;
}

void ConvertSensirionHumidityTempX100(unsigned char *ptrSensirionData, long *Humidity, long *TempCelsius)
{
    long stemp;
    long shum;

    stemp = (long) (((unsigned long) ptrSensirionData[0] << 8) | ptrSensirionData[1]);
    // simplified (65536 instead of 65535) integer version of:
    //temp = (stemp * 175.0f) / 65535.0f - 45.0f;
    stemp = ((4375 * stemp) >> 14) - 4500;
    *TempCelsius = stemp;   
        
    shum = ((unsigned long) ptrSensirionData[3] << 8) | ptrSensirionData[4];
    // simplified (65536 instead of 65535) integer version of:
    //humidity = (shum * 100.0f) / 65535.0f;
    shum = (625 * shum) >> 12;
    *Humidity = shum;     
}