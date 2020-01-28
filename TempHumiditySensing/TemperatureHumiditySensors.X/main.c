/***************************************************************************************
 * Project:     TemperatureHumiditySensors
 * 
 *              C language code for MAX31865 RTD converter IC 
 *              and Sensirion SHT31 humidity and temperature sensor.
 * 
 *              Output is Humidity (RH) x100 and Temperature Celsius x100 
 *              sent to RS232 port using UART 2 at 19200 baud.
 * 
 *              For PIC 32MX220F032D microcontroller.
 *              Project created using Microchip MPLAB X IDE V5.15 
 *              Using Microchip XC32 Compiler V1.30
 * 
 *              Note: The PIC 32MX220 has a Peripheral Pin Select (PPS) feature 
 *              which maps various IO functions to different IO pins.
 * 
 *              This allows flexibility but is also a colossal pain in the ass 
 *              because of lousy Microchip documentation.
 *              Note that I2C is for fixed pins, 
 *              but the RS232 UART and SPI SDO and SDI must have PPS IO set.
 *              For details see InitializeSystem() function below.  
 * 
 *              Used with HyperTerminal with Append line feeds to incoming line ends option 
 *              under Hyper Terminal File/Properties/Settings/ASCII Setup/ASCII Receiving/
 *              ..Or just add /n to every /r below.
 * 
 * 
 * FileName:    main.c 
 *   
 * 1-27-20 Warwick: Works with Three wire RTD and MAX31865. Fault detection and clearing also works.
 * 1-28-20 Warwick: Got Sensirion SHT31 reading humidity and temperature.
 * 
 ****************************************************************************************/
#define TESTOUT LATCbits.LATC2
#define _SUPPRESS_PLIB_WARNING

#define PWM1 OC4RS
#define PWM2 OC3RS

#define SYS_FREQ 60000000  // With 8 Mhz crystal and FPLLMUL = MUL_15
#define GetPeripheralClock() SYS_FREQ 

#include "Delay.h"
#include <plib.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "SHT31_I2C.h"
#include "MAX31865_RTD.h"

/** CONFIGURATION **************************************************/
#pragma config UPLLEN   = ON            // USB PLL Enabled
#pragma config FPLLMUL  = MUL_15        // PLL Multiplier for 220 - yields 60 Mhz
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
#pragma config FWDTEN   = OFF           // Watchdog Timer
#pragma config WDTPS    = PS1           // Watchdog Timer Postscale
#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = HS            // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable (KLO was off)
#pragma config FNOSC    = PRIPLL        // Oscillator Selection
#pragma config CP       = OFF           // Code Protect
#pragma config BWP      = OFF           // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx3      // ICE/ICD Comm Channel Select
#pragma config JTAGEN   = OFF           // Use JTAG pins for normal IO
#pragma config DEBUG    = OFF            // Enable/disable debugging


/*** DEFINES *****************************************************************/
#define HOSTuart UART2
#define HOSTbits U2STAbits
#define HOST_VECTOR _UART_2_VECTOR     
#define MAXBUFFER 255
#define MAXSERVOS 64

#define DATAuart UART1
#define DATAbits U1STAbits
#define DATA_VECTOR _UART_1_VECTOR     



/** I N C L U D E S **********************************************************/
#include "GenericTypeDefs.h"

/** V A R I A B L E S ********************************************************/


unsigned char HOSTRxBuffer[MAXBUFFER + 1];
unsigned char HOSTTxBuffer[MAXBUFFER + 1];
unsigned char HOSTBufferFull = false;

unsigned char DATARxBuffer[MAXBUFFER + 1];
unsigned char DATATxBuffer[MAXBUFFER + 1];
unsigned char DATABufferFull = false;

/** V A R I A B L E S ******************************
 **************************/

/** P R I V A T E  P R O T O T Y P E S ***************************************/

unsigned char intFlag = false, displayPWMFlag = false;
void InitializeSystem(void);
void ConfigAd(void);

void putch(unsigned char ch);

union convertType 
{
    unsigned char byte[2];
    long integer;
} convert;

unsigned long dataLength = 0;
unsigned char hostChar = 0, DATAchar = 0;
#define NUM_AD_INPUTS 2
unsigned short arrADreading[NUM_AD_INPUTS];
unsigned long Timer5Counter = 0;
unsigned long PORTBreg = 0;

void InitializeSPI(short SPIchannel)
{
    SpiChnOpen(SPIchannel, SPI_OPEN_MSTEN | SPI_OPEN_MODE8 | SPI_OPEN_CKP_HIGH | SPI_OPEN_ON, 600);  
}

// This version uses the PIC 32 SPI port 
unsigned char SendReceiveSPI(short SPIchannel, unsigned char dataOut)
{
unsigned char dataIn;

	SpiChnPutC(SPIchannel, dataOut);
	dataIn = SpiChnGetC(SPIchannel);    
	return(dataIn);
}

#define SHT31_MEAS_HIGHREP 0x2400 // Command For Sension SHT31: Measurement High Repeatability with Clock Stretch Disabled 

int main(void) 
{
    unsigned short tempInt;
    unsigned short counter = 0x0000;
    unsigned char SensirionData[6];
    long MAX31865_Temperature;
    long temp;
    long humidity;
    
    DelayMs(200);
    InitializeSystem();    
    MAX31865_CS = 1;
    
    DelayMs(100);
    
    printf("\r\r\r\rStarting - Initializing PIC SPI port #2 for MAX31865...");
    InitializeSPI(2);      
    printf("DONE.");
    ConfigureMAX31865();        
    printf("\rInitializing PIC I2C Bus #1 for Sensirion...");
    initI2C(I2C1);
    printf("DONE.\r\rReading humidity and Temperature:\r");
    
    while(1)
    {
        DelayMs(1000);        
        if (!ReadSensirionSHT31(I2C1, SHT31_I2C_ADDRESS, SHT31_MEAS_HIGHREP, SensirionData, 6))
        {
            printf("\rSensirion Read error...");
            humidity = temp = 0;
        }
        else ConvertSensirionHumidityTempX100(SensirionData, &humidity, &temp);
        
        tempInt = ReadRawRTDTemperature();
        if (CheckMAX31865Fault()) 
        {
            MAX31865_Temperature = (long) (TemperatureCalc(tempInt) * 100);
            printf("\r#%d: Sensirion RH x100: %d, Temp Celsius x100: %d, MAX31865 RTD x100: %d", counter++, humidity, temp, MAX31865_Temperature);
        }
    }
       
} // End main())


void putch(unsigned char ch) {
    while (!IFS1bits.U2TXIF); // set when register is empty 
    U2TXREG = ch;
}

void InitializeSystem(void) 
{
    // ConfigAd();
    
    mJTAGPortEnable(false);           

    // DIGITAL OUTPUTS: 
    PORTSetPinsDigitalOut(IOPORT_A, BIT_10); // This is the chip select for MAX31865
    MAX31865_CS = 1;  // Initialize to high to unselect IC

    PORTSetPinsDigitalOut(IOPORT_C, BIT_2); 
    TESTOUT = 0;
    
    // Set up PPS IO for SPI #2 SDO and SDI.
    // Note that SPI clock is on fixed pin and doesn't use PPS.
    PPSOutput(2, RPB5, SDO2);    
    PPSInput(3, SDI2, RPB13);     
    
    //ATMEL_WRITE_PROTECT = 1; // Enable EEPROM write protection at startup, and disable chip select
    //ATMEL_CS = 1;

    // Enable analog inputs
    ANSELCbits.ANSC0 = 1; // AN6  
    ANSELCbits.ANSC1 = 1; // AN7     

    // Disable analog on digital inputs:    
    ANSELBbits.ANSB15 = 0;
    ANSELBbits.ANSB14 = 0;
    ANSELBbits.ANSB13 = 0;    
    ANSELBbits.ANSB0 = 0;
    ANSELBbits.ANSB1 = 0;
    ANSELBbits.ANSB2 = 0;
    ANSELCbits.ANSC3 = 0;
    
    ANSELCbits.ANSC2 = 0;

    TESTOUT = 1;

    /*
    // Set up DATA UART #1   
    PPSInput(3,U1RX, RPC3);     // Assign U1RX to pin RPC3
    PPSOutput(1,RPB3,U1TX);     // Assign U1TX to pin RPB3
    
    // Configure UART #1 (DATA UART) - NOTE: This UART isn't being used yet
    UARTConfigure(DATAuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(DATAuart, UART_INTERRUPT_ON_RX_NOT_EMPTY); //  | UART_INTERRUPT_ON_TX_DONE  
    UARTSetLineControl(DATAuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(DATAuart, SYS_FREQ, 19200);
    UARTEnable(DATAuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure UART #1 Interrupts
    INTEnable(INT_U1TX, INT_DISABLED);
    INTEnable(INT_U1RX, INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(DATAuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(DATAuart), INT_SUB_PRIORITY_LEVEL_0);    
    */
    

    // Set up PPS pins for Uart #2:
    PPSOutput(4, RPC9, U2TX); // Assign U2TX to pin RPC8
    PPSInput(2, U2RX, RPC8);  // Assign U2RX to pin RPA8
    
    // Set up HOST UART #2 (HOST UART) at 19200 baud.
    UARTConfigure(HOSTuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(HOSTuart, UART_INTERRUPT_ON_RX_NOT_EMPTY); //  | UART_INTERRUPT_ON_TX_DONE  
    UARTSetLineControl(HOSTuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(HOSTuart, SYS_FREQ, 19200);
    UARTEnable(HOSTuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure UART #2 Interrupts
    INTEnable(INT_U2TX, INT_DISABLED);
    INTEnable(INT_U2RX, INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(HOSTuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(HOSTuart), INT_SUB_PRIORITY_LEVEL_0);    
    // Turn on the interrupts
    INTEnableSystemMultiVectoredInt();    
}//end UserInit


/* A/D NOT USED IN THIS VERSION
void ConfigAd(void) 
{
    //mPORTCSetPinsAnalogIn(BIT_0 | BIT_1);
    //mPORTBSetPinsDigitalOut(BIT_0 | BIT_1);
    // ---- configure and enable the ADC ----

    // ensure the ADC is off before setting the configuration
    CloseADC10();

    // define setup parameters for OpenADC10
    //                 Turn module on | ouput in integer | trigger mode auto | enable autosample
#define PARAM1  ADC_MODULE_ON | ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON

    // ADC ref external    | disable offset test    | enable scan mode | perform  samples | use dual buffers | use only mux A
#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_4 | ADC_ALT_BUF_ON | ADC_ALT_INPUT_OFF

    //                   use ADC internal clock | set sample time
#define PARAM3  ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_31

    //  set AM7 (A1 on Olimex 220 board) input to analog
    // #define PARAM4    ENABLE_AN0_ANA | ENABLE_AN1_ANA| ENABLE_AN2_ANA | ENABLE_AN3_ANA


// USE AN6 and AN7    
#define PARAM5 SKIP_SCAN_AN0 | SKIP_SCAN_AN1 |SKIP_SCAN_AN2 | SKIP_SCAN_AN3 | SKIP_SCAN_AN4 | SKIP_SCAN_AN5 |\
    SKIP_SCAN_AN8 | SKIP_SCAN_AN9 | SKIP_SCAN_AN10 |\
    SKIP_SCAN_AN11 | SKIP_SCAN_AN12 | SKIP_SCAN_AN13 | SKIP_SCAN_AN14 | SKIP_SCAN_AN15
    
#define PARAM4    ENABLE_AN6_ANA | ENABLE_AN7_ANA     

    // set negative reference to Vref for Mux A
    SetChanADC10(ADC_CH0_NEG_SAMPLEA_NVREF);

    // open the ADC
    OpenADC10(PARAM1, PARAM2, PARAM3, PARAM4, PARAM5);

    ConfigIntADC10(ADC_INT_PRI_2 | ADC_INT_SUB_PRI_2 | ADC_INT_ON);

    // clear the interrupt flag
    mAD1ClearIntFlag();

    // Enable the ADC
    EnableADC10();
}

void __ISR(_ADC_VECTOR, IPL2AUTO) ADHandler(void) {
    unsigned long offSet;
    unsigned char i;

    mAD1IntEnable(INT_DISABLED);
    mAD1ClearIntFlag();

    // Determine which buffer is idle and create an offset
    offSet = 8 * ((~ReadActiveBufferADC10() & 0x01));

    for (i = 0; i < NUM_AD_INPUTS; i++)
        arrADreading[i] = (unsigned long) ReadADC10(offSet + i); // read the result of channel 0 conversion from the idle buffer
}


void __ISR(_TIMER_5_VECTOR, IPL2AUTO) Timer5Handler(void) {
    mT5ClearIntFlag(); // Clear interrupt flag
    if (Timer5Counter) Timer5Counter--;
}

void __ISR(_CHANGE_NOTICE_VECTOR, IPL2AUTO) ChangeNotice_Handler(void) {

    // Step #1 - always clear the mismatch condition first
    PORTBreg = PORTB & 0x0003;

    // Step #2 - then clear the interrupt flag
    mCNBClearIntFlag();

}

// Timer 2 generates an interrupt every 50 microseconds approximately
void __ISR(_TIMER_2_VECTOR, IPL2AUTO) Timer2Handler(void) 
{
    static int intCounter = 0;    
    mT2ClearIntFlag(); // Clear interrupt flag       
    
    intCounter++;
    if (intCounter >= 50)
    {
        intCounter = 0;
        intFlag = true;
    }
}
*/

/* NOT USED
void __ISR(DATA_VECTOR, IPL2AUTO) IntUart1Handler(void) 
{
    static short RXindex = 0;
    unsigned char ch;

    if (INTGetFlag(INT_SOURCE_UART_RX(DATAuart))) {
        INTClearFlag(INT_SOURCE_UART_RX(DATAuart));
        if (DATAbits.OERR || DATAbits.FERR) {
            if (UARTReceivedDataIsAvailable(DATAuart))
                ch = UARTGetDataByte(DATAuart);
            DATAbits.OERR = 0;
        }

        if (UARTReceivedDataIsAvailable(DATAuart)) 
        {
            ch = UARTGetDataByte(DATAuart);
            if (RXindex < MAXBUFFER)
                DATARxBuffer[RXindex++] = ch;
            if (ch == '\r')
            {
                RXindex = 0;
                DATABufferFull = true;
            }
        }
    }
}
*/

#define ESC 27
#define CR 13
#define BACKSPACE 8
void __ISR(HOST_VECTOR, IPL2AUTO) IntHostUartHandler(void) 
{
unsigned char ch;
static unsigned long i = 0;

    if (INTGetFlag(INT_SOURCE_UART_RX(HOSTuart))) 
    {
        INTClearFlag(INT_SOURCE_UART_RX(HOSTuart));                 
        if (HOSTbits.OERR || HOSTbits.FERR) {
            if (UARTReceivedDataIsAvailable(HOSTuart))
                ch = UARTGetDataByte(HOSTuart);
            HOSTbits.OERR = 0;            
        }
        if (UARTReceivedDataIsAvailable(HOSTuart)) 
        {
            ch = UARTGetDataByte(HOSTuart);            
            if (ch != 0 && ch != '\n') {            
                if (ch == BACKSPACE) 
                {
                    if (i != 0) i--;
                    HOSTRxBuffer[i] = '\0';
                    while(!UARTTransmitterIsReady(HOSTuart));
                    UARTSendDataByte (HOSTuart, ' ');
                    while(!UARTTransmitterIsReady(HOSTuart));
                    UARTSendDataByte (HOSTuart, BACKSPACE);                
                } 
                else if (i < MAXBUFFER) 
                {
                    HOSTRxBuffer[i] = ch;
                    i++;
                }            
                if ('\r' == ch || ' ' == ch) 
                {
                    HOSTBufferFull = true;
                    HOSTRxBuffer[i] = '\0';
                    i = 0;
                }
            }
        }
    }         
    
    if (INTGetFlag(INT_SOURCE_UART_TX(HOSTuart))) 
        INTClearFlag(INT_SOURCE_UART_TX(HOSTuart));            
}

    //do {
    //    ch = UARTGetDataByte(HOSTuart);
    //} while (ch);

    /* NOT USED
    // Set counter inputs    
    PPSInput(3, T4CK, RPB2);

    // Set up timers as counters
    T1CON = 0x00;
    T1CONbits.TCS = 1; // Use external counter as input source (motor encoder)
    T1CONbits.TCKPS1 = 0; // 1:1 Prescaler
    T1CONbits.TCKPS0 = 0;
    T1CONbits.TSYNC = 1;
    PR1 = 0xFFFF;
    T1CONbits.TON = 1; // Let her rip 

    T4CON = 0x00;
    T4CONbits.TCS = 1; // Use external counter as input source (motor encoder)
    T4CONbits.TCKPS2 = 0; // 1:1 Prescaler
    T4CONbits.TCKPS1 = 0;
    T4CONbits.TCKPS0 = 0;
    T4CONbits.T32 = 0; // TMRx and TMRy form separate 16-bit timers
    PR4 = 0xFFFF;
    T4CONbits.TON = 1; // Let her rip     

    // Set up Timer 5 interrupt with a priority of 2
    ConfigIntTimer5(T5_INT_ON | T5_INT_PRIOR_2);
    OpenTimer5(T5_ON | T5_SOURCE_INT | T5_PS_1_8, 7500);

    // Set up Timer 2 for PWM time base    
    T2CON = 0x00;
    T2CONbits.TCKPS2 = 0; // 1:1 Prescaler
    T2CONbits.TCKPS1 = 0;
    T2CONbits.TCKPS0 = 0;
    PR2 = 3000; // Use 50 microsecond rollover for 20 khz
    T2CONbits.TON = 1; // Let her rip       
    
    // Set up Timer 2 interrupt with a priority of 2
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
    // OpenTimer2(T2_ON | T2_SOURCE_INT);
    

    // Set up PWM OC3
    PPSOutput(4, RPC4, OC3);    
    OC3CON = 0x00;
    OC3CONbits.OC32 = 0; // 16 bit PWM
    OC3CONbits.ON = 1; // Turn on PWM
    OC3CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC3CONbits.OCM2 = 1; // PWM mode enabled, no fault pin
    OC3CONbits.OCM1 = 1;
    OC3CONbits.OCM0 = 0;
    OC3RS = 0;

    // Set up PWM OC4 on D6 on the Olimex 220 board:
    PPSOutput(3, RPC6, OC4);
    OC4CON = 0x00;
    OC4CONbits.OC32 = 0; // 16 bit PWM
    OC4CONbits.ON = 1; // Turn on PWM
    OC4CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC4CONbits.OCM2 = 1; // PWM mode enabled, no fault pin
    OC4CONbits.OCM1 = 1;
    OC4CONbits.OCM0 = 0;
    OC4RS = 0;
    */

/*    
    // Set up interrupt on change for the PORT B input pins RB0 and RB1
    CNCONBbits.ON = 1; // CN is enabled
    CNCONBbits.SIDL = 0; // CPU Idle does not affect CN operation
    CNENBbits.CNIEB0 = 1; // Enable RB0 change notice    
    CNENBbits.CNIEB1 = 1; // Enable RB1 change notice

    // Read port B to clear mismatch condition
    dummyRead = PORTB;
    */ 
     
    /*
    mCNSetIntPriority(2);
    mCNSetIntSubPriority(2);
    mCNBClearIntFlag();
    mCNBIntEnable(BIT_0 | BIT_1);
    */
    /*
    while(1)
    {
        if (HOSTBufferFull)
        {
            HOSTBufferFull = false;
            printf("\rReceived: %s", HOSTRxBuffer);
        }
        TestByteIn = SendReceiveSPI(2, TestByteOut++);        
        DelayMs(100);        
    }
                
    while(1) 
    {    
        if (HOSTBufferFull)
        {
            HOSTBufferFull = false; 
            printf("\rReceived: ");
            q = 0;
            command = 0;
            for (p = 0; p < MAXBUFFER; p++) 
            {
                ch = HOSTRxBuffer[p];
                if (ch >= 'a' && ch <= 'z') ch = ch - 'a' + 'A';
                
                if (ch >= 'A' && ch <= 'Z') command = ch;
                else if (ch == ' ') command = ' ';
                
                putch(ch);
                if (ch == '\r' || ch == ' ')break;
                if ( ((ch >= '0' && ch <= '9') || ch == '.' || ch == '-') && q < MAXNUM) NUMbuffer[q++] = ch;
            }
            if (q) 
            {
                NUMbuffer[q] = '\0';
                ProbeSlotNumber = atoi(NUMbuffer);
            }
            if (command) 
            {
                switch (command) 
                {
                    case 'M':                 
                        break;
                    default:
                        printf("\rCommand: %c", command);
                        break;
                } // end switch command                
            } // End if command             
        } // End if HOSTBufferFull             
    } // End while(1))
    */ 
