/*********************************************************************
 *
 *                  General Delay routines
 *
 *********************************************************************
 * FileName:        DelayPIC32.c
 * 1-10-17 JBS: Tweaked for Olimex 220 and XC32 Compiler V1.30
 * 1-27-20 JBS: Updated for PIC32MX220 
 ********************************************************************/
#include "Delay.h"

void DelayMs(long ms){
unsigned char i;

    while(ms--){
        i=4;
        while(i--)
        {
            // Delay10us(25); 
            Delay10us(6); 
        }
    }
}

void Delay10us(long dwCount){
unsigned short i;

	while(dwCount--)	
        for (i=0; i<206; i++);
		// for (i=0; i<196; i++);
}

void DelayUs(unsigned short Count){
unsigned short i;

	while(Count--)	
		for (i=0; i<1000; i++);
}