/**
  The main.c file for Searchlight Control System Motor Driver Module FW (MDC2022) 

  @Company
    Torka AS

  @File Name
    main.c

  @Summary
    This is the main.c using PIC32MM0256GPM MCUs. 

  @Description
    This is the main.c using PIC32MM0256GPM MCUs. 
  
    This source file provides main entry point for system initialization and application code development.
    Generation Information :
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.171.2
        Device            :  PIC32MM0256GPM064
    The generated drivers are tested against the following:
        Compiler          :  XC32 v4.45
        MPLAB 	          :  MPLAB X v6.20

  @Revisions
    "Rev.: 000" - 2025.01.27 - OPR, Torka AS as
        - Minumum funksjonal versjon release.
 
*/

/**
  Section: Defines
*/
#define DECODE_BUFFER_SIZE 64
#define BTN_AUX_1 0x01
#define BTN_AUX_2 0x02
#define BTN_AUX_3 0x04
#define BTN_AUX_4 0x08
#define BTN_EMERGENCY_STOP 0x10
#define BTN_EMERGENCY_RELEASE 0x20


/**
  Section: Included Files
*/
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "mcc_generated_files/system.h"
#include "mcc_generated_files/pin_manager.h"
#include "mcc_generated_files/coretimer.h"
#include "mcc_generated_files/adc1.h"
#include "mcc_generated_files/mccp1_compare.h"
#include "mcc_generated_files/mccp3_compare.h"
#include "mcc_generated_files/usb/../usb/usb_device_cdc.h"
#include "mcc_generated_files/usb/../usb/usb.h"
#include "DB_Debounce.h"



// Version Tags
char VersionTag1[] = __DATE__;     // "Jan 24 2011"
char VersionTag2[] = __TIME__;     // "14:45:20"
char RevBuild[] = "Rev.: 000";

uint8_t readBuffer[DECODE_BUFFER_SIZE];
uint8_t writeBuffer[DECODE_BUFFER_SIZE];
char Buff[DECODE_BUFFER_SIZE];
uint8_t myData[DECODE_BUFFER_SIZE];
uint8_t myIndex, CS;

volatile bool TimerEvent1ms;
volatile uint32_t myTime=0;
bool TimerEvent10ms=false, TimerEvent50ms=false, TimerEvent250ms=false, TimerEvent1s=false;
uint32_t myLastTime=0;

typedef struct {
    uint32_t data[16];
    uint32_t lastValue;
    uint8_t index;
    uint8_t notFirstRound;
}  __attribute__ ((packed)) stMA_t;

volatile stMA_t MA_AN2, MA_AN3, MA_AN4, MA_AN5, MA_AN6, MA_AN7, MA_AN11, MA_AN12, MA_AN13;
volatile uint16_t AD[9];
volatile ADC1_CHANNEL channel;
unsigned char Blink=0x80;

unsigned int uiDimTableLED[] = { 250, 900, 1023 }; 
//unsigned int uiDimTable2[] = { 135, 160, 0 }; 
unsigned int uiDimTableElfilm[] = { 256, 256, 256 }; 


/**
    Prototypes
 */
void MCC_USB_CDC_DemoTasks(void);
void UpdateTimers(void);
uint16_t MA_SetValueAndGetAvarage(volatile stMA_t * dataStorage, uint16_t newValue);


void ADC1_CallBack(void)
{
    if(channel==PRESSURE_1_AN) {
        AD[0] = MA_SetValueAndGetAvarage(&MA_AN2, ADC1_ConversionResultGet(PRESSURE_1_AN));
        channel = PRESSURE_2_AN;
    } else if(channel==PRESSURE_2_AN) {
        AD[1] = MA_SetValueAndGetAvarage(&MA_AN3, ADC1_ConversionResultGet(PRESSURE_2_AN));
        channel = PRESSURE_3_AN;
    } else if(channel==PRESSURE_3_AN) {
        AD[2] = MA_SetValueAndGetAvarage(&MA_AN4, ADC1_ConversionResultGet(PRESSURE_3_AN));
        channel = PRESSURE_4_AN;
    } else if(channel==PRESSURE_4_AN) {
        AD[3] = MA_SetValueAndGetAvarage(&MA_AN11, ADC1_ConversionResultGet(PRESSURE_4_AN));
        channel = WINCH_1_AN;
    } else if(channel==WINCH_1_AN) {
        AD[4] = MA_SetValueAndGetAvarage(&MA_AN12, ADC1_ConversionResultGet(WINCH_1_AN));
        channel = WINCH_2_AN;
    } else if(channel==WINCH_2_AN) {
        AD[5] = MA_SetValueAndGetAvarage(&MA_AN13, ADC1_ConversionResultGet(WINCH_2_AN));
        channel = WINCH_3_AN;
    } else if(channel==WINCH_3_AN) {
        AD[6] = MA_SetValueAndGetAvarage(&MA_AN5, ADC1_ConversionResultGet(WINCH_3_AN));
        channel = WINCH_4_AN;
    } else if(channel==WINCH_4_AN) {
        AD[7] = MA_SetValueAndGetAvarage(&MA_AN6, ADC1_ConversionResultGet(WINCH_4_AN));
        channel = LIGHT_SENSE;
    } else {
        AD[8] = MA_SetValueAndGetAvarage(&MA_AN7, ADC1_ConversionResultGet(LIGHT_SENSE));
        channel = PRESSURE_1_AN;
    }
    ADC1_ChannelSelect(channel);
}


void UpdateTimers(void)
{
    static int32_t my10msCnt, my50msCnt, my250msCnt, my1000msCnt, my15sCnt;
    int32_t delta;
    
    delta = myTime-myLastTime;
    myLastTime = myTime;
    my10msCnt += delta;
    
    // Update Time Keepers
    while(my10msCnt>=10) {
        TimerEvent10ms = true;
        my10msCnt -= 10;
        my50msCnt += 10;
        while(my50msCnt>=50) {
            TimerEvent50ms = true;
            my50msCnt -= 50;
            my250msCnt += 50;
            while(my250msCnt>=250) {
                TimerEvent250ms=true;
                my250msCnt -= 250;
            }            
            my1000msCnt += 50;
            while(my1000msCnt>=1000) {
                TimerEvent1s=true;
                my1000msCnt -= 1000;
            }  //..while(my1000msCnt>=1000)
        }  //..while(my50msCnt>=50)
    }  //..while(my10msCnt>=10)
    
}


uint16_t MA_SetValueAndGetAvarage(volatile stMA_t * dataStorage, uint16_t newValue)
{
    uint8_t i;
    uint32_t value;
    
    // Prime table in first round
    if(dataStorage->notFirstRound==0) {
        for(i=0;i<16;i++) {
            dataStorage->data[i] = newValue;
        }
        dataStorage->lastValue = newValue;
        dataStorage->lastValue <<= 4;
        dataStorage->notFirstRound=1;
    }

    // Update Avarage Value and store current
    dataStorage->lastValue -= dataStorage->data[dataStorage->index];
    dataStorage->lastValue += newValue; 
    dataStorage->data[dataStorage->index++] = newValue;
    if(dataStorage->index>=16) {
        dataStorage->index=0;
    }
    
    return (uint16_t)(dataStorage->lastValue>>4);
}

/*******************************************\
| NAME: I2H      							              |
|-------------------------------------------|
| FUNCTION:                                 |
|  Convert an integer (1 byte) to a HEX     |
|  value as a string.                       |
|                                           |
| INPUT:                                    |
|  B - Byte value to convert.               |
|                                           |
| OUTPUT:                                   |
|  A pointer to a static string (szHex).    |
|  holding the value from the last call to  |
|  function.                                |
\*******************************************/
char *I2H(unsigned char B)
{
	static char szHex[3];
	unsigned char Nibble;
	
	// First Hi-nibble
	Nibble = B >> 4;
	if(Nibble < 10)
	  szHex[0] = '0' + Nibble;
	else  
	  szHex[0] = 'A' + Nibble - 10;

	// Then Lo-nibble
	Nibble = B & 0x0F;
	if(Nibble < 10)
	  szHex[1] = '0' + Nibble;
	else  
	  szHex[1] = 'A' + Nibble - 10;
		
	szHex[2] = '\0';

	return szHex;	
}

/*******************************************\
| NAME: xtoi    							|
|-------------------------------------------|
| FUNCTION:                                 |
|  Convert a HEX value as a string (2 char) |
|  to an integer value (1 byte).            |
|                                           |
| INPUT:                                    |
|  S - Pointer to HEX value to convert.     |
|                                           |
| OUTPUT:                                   |
|  Unsigned char holding the value.         |
\*******************************************/
unsigned char xtoi(char *S)
{
  unsigned char ret=0;
  
  // First Hi-nibble
  if(S[0]>='A')
    ret = S[0] - 'A' + 10;
  else 
    ret = S[0] - '0';
  ret <<=4;
    
  // Then Lo-nibble
  if(S[1]>='A')
    ret += S[1] - 'A' + 10;
  else 
    ret += S[1] - '0';
    
  return ret;  
}

/*******************************************\
| NAME: Send2PC  							              |
|-------------------------------------------|
| FUNCTION:                                 |
|  Dim Send2PC.                             |
|                                           |
| INPUT:                                    |
|  ID - Data ID                             |
|  Data - Data to send.                     |
|  Len - Length of Data.                    |
|                                           |
| OUTPUT:                                   |
|  None                                     |
|                                           |
| NOTE:                                     |
|  Uses writeBuffer.                              |
\*******************************************/
void Send2PC(unsigned char ID,char *Data, unsigned char Len)
{
  char Checksum=0;
	int i,x=0;
	char *S;
	
	
	// Build telegram
	writeBuffer[x++] = ':';  // SOT
	S = I2H(ID);
	writeBuffer[x++] = S[0];  // Add Header ID - Hi
	writeBuffer[x++] = S[1];  // Add Header ID - Lo
	for(i=0;i<Len;i++) {
  	S = I2H(Data[i]);
  	writeBuffer[x++] = S[0];  // Add Data - Hi
	  writeBuffer[x++] = S[1];  // Add Data - Lo
	}
  writeBuffer[x] = 0;
  	

	// Calculate checksum
	for(i=0;i<strlen(writeBuffer);i++) {
		Checksum ^= writeBuffer[i];
  }	

  // Print Checksum
	S = I2H(Checksum);
	writeBuffer[x++] = S[0];  // Add CRC - Hi
	writeBuffer[x++] = S[1];  // Add CRC - Lo
	writeBuffer[x++] = 0x0D;  // Add EOT
	writeBuffer[x++] = 0x0A;  // Add EOT
  writeBuffer[x] = 0;

  putUSBUSART(writeBuffer,strlen(writeBuffer));
  
  return;
}

void SetPalett(unsigned int Value)
{
  static uint8_t Modus;
  unsigned int DimValue;
  
  switch(Modus) {
    case 0:
      ELFILM_EN_SetHigh();  // El-film on    
      
      // In the zone?
      //if(Value<0x0300)
      if(Value<0x0E85)
        Modus++; 
    break;
    case 1:
      ELFILM_EN_SetHigh();  // El-film on    

      // In the zone?
      //if(Value>0x0320)
      if(Value>0x0E88)
        Modus--; 
      //if(Value<0x0200)
      if(Value<0x0E80)
        Modus++; 

    break;
    case 2:
    default:
      Modus=2;
      ELFILM_EN_SetLow();  // El-film off    

      // In the zone?
      //if(Value>0x0220)
      if(Value>0x0E82)
        Modus--; 
    break;    
  }  
    
  
  // Set new PWM
  MCCP1_COMPARE_DualEdgeBufferedConfig( 0, uiDimTableLED[Modus] );  // LED DIM
  MCCP3_COMPARE_DualEdgeBufferedConfig( 0, uiDimTableElfilm[Modus] );    // EL-FILM DIM
  
}  


/*
    Main application
 */
int main(void)
{
    uint8_t BlinkCnt;
    uint8_t Switches=0;
    uint8_t myLEDOut=0;
    bool blankSent=false;
        
    // initialize the device
    SYSTEM_Initialize();
    
    // Start indicating all well...
    LED_AUX_1_SetLow();
    LED_AUX_2_SetLow();
    LED_AUX_3_SetLow();
    LED_AUX_4_SetLow();
    ELFILM_EN_SetLow();  // El-film off    
    MCCP1_COMPARE_DualEdgeBufferedConfig( 0, uiDimTableLED[0] );  // LED DIM
    MCCP3_COMPARE_DualEdgeBufferedConfig( 0, uiDimTableElfilm[0] );    // EL-FILM DIM
    COMMON_BTN_AUX_SetLow();
    COMMON_BTN_EMERGENCY_SetLow();
    
    
    // Prepare the ADC
    ADC1_Enable();
    channel = PRESSURE_1_AN;
    ADC1_ChannelSelect(channel);
    ADC1_InterruptEnable();
    
    
    
    // Enable WDT
    WDTCONbits.ON=1;
    
    // Main loop
    while (1)
    {
        
        
        // 1ms Timer Event
        if(TimerEvent1ms) {
            TimerEvent1ms = false;
            UpdateTimers();
                        
        }  // ..if(TimerEvent1ms)

        
        // 10ms Timer Event
        if(TimerEvent10ms) {
            TimerEvent10ms = false;

            // Scan Switches
            if(BTN_AUX_1_GetValue()==0) {
                Switches |= BTN_AUX_1;
            }
            if(BTN_AUX_2_GetValue()==0) {
                Switches |= BTN_AUX_2;
            }
            if(BTN_AUX_3_GetValue()==0) {
                Switches |= BTN_AUX_3;
            }
            if(BTN_AUX_4_GetValue()==0) {
                Switches |= BTN_AUX_4;
            }
            if(BTN_EMERGENCY_STOP_GetValue()==0) {
                Switches |= BTN_EMERGENCY_STOP;
            }
            if(BTN_EMERGENCY_RELEASE_GetValue()==0) {
                Switches |= BTN_EMERGENCY_RELEASE;
            }
    
            ADC1_SoftwareTriggerEnable();
            
        }  //..if(my10msTimer)
        
        
        // 50ms Timer Event
        if(TimerEvent50ms) {
            TimerEvent50ms = false;
 
        }  //..if(TimerEvent50ms)
        
        
        // 250ms Timer Event
        if(TimerEvent250ms) {
            TimerEvent250ms = false;
        
            // Status blink
            BlinkCnt++;
            BlinkCnt &= 0x07;
            if(Blink&(0x01<<BlinkCnt)) {
                OP_STATUS_SetHigh();
            } else {
                OP_STATUS_SetLow();    
            }
            
            // Send Telegram 02
            if(Blink==0xA0) {
                for(uint8_t i=0;i<9;i++) {
                    myData[i*2] = (uint8_t)(AD[i]>>8);
                    myData[(i*2)+1] = (uint8_t)(AD[i]&0x00FF);
                }
                myData[18] = Switches;
                
                // EMERGENCY_STOP?
                if(((Switches&BTN_EMERGENCY_STOP) != 0) && ((Switches&BTN_EMERGENCY_RELEASE)==0)) {
                    Switches = BTN_EMERGENCY_STOP;  // Keep state until released
                    if(!blankSent) {
                        
                        // NULL-out AN-data...
                        for(uint8_t i=0;i<4;i++) {
                            myData[i*2] = 0x0F;
                            myData[(i*2)+1] = 0xFF;
                            myData[8+(i*2)] = 0x07;
                            myData[8+(i*2)+1] = 0xFF;
                        }
                        Send2PC(2,myData,19);  // Send if EMERGENCY STOP once to cancel input
                        blankSent = true;
                    }
                } else {
                    Send2PC(2,myData,19);  // Send if not EMERGENCY STOP 
                    Switches = 0;
                    blankSent = false;
                }
                
            }

            // Update Leds
            if((Switches&BTN_EMERGENCY_STOP) == 0) { 
                if(myLEDOut&0x01) {
                    LED_AUX_1_SetHigh();
                } else {
                    LED_AUX_1_SetLow();
                }
                if(myLEDOut&0x02) {
                    LED_AUX_2_SetHigh();
                } else {
                    LED_AUX_2_SetLow();
                }
                if(myLEDOut&0x04) {
                    LED_AUX_3_SetHigh();
                } else {
                    LED_AUX_3_SetLow();
                }
                if(myLEDOut&0x08) {
                    LED_AUX_4_SetHigh();
                } else {
                    LED_AUX_4_SetLow();
                }
            } else if(BlinkCnt) {
                LED_AUX_1_SetLow();
                LED_AUX_2_SetLow();
                LED_AUX_3_SetLow();
                LED_AUX_4_SetLow();                
            } else {
                LED_AUX_1_SetHigh();
                LED_AUX_2_SetHigh();
                LED_AUX_3_SetHigh();
                LED_AUX_4_SetHigh();
            }
            
            
        }  //..if(TimerEvent250ms)
        
        
        // 1s Timer Event
        if(TimerEvent1s) {
            TimerEvent1s = false;

           
        }  //..if(TimerEvent1s)

        
        // USB HANDLER
        //-------------
        if( USBGetDeviceState() < CONFIGURED_STATE ) {
            ELFILM_EN_SetLow();  // El-film off
        } else if( USBIsDeviceSuspended()== true ) {
            ELFILM_EN_SetLow();  // El-film off
        } else {
            Blink=0xA0;

            SetPalett(AD[8]);

            if( USBUSARTIsTxTrfReady() == true) {
                uint8_t i;
                uint8_t numBytesRead;

                numBytesRead = getsUSBUSART(readBuffer, sizeof(readBuffer));

                for(i=0; i<numBytesRead; i++)
                {
                    switch(readBuffer[i])
                    {
                        case ':':
                            myIndex=0;
                            Buff[myIndex++] = ':';
                            break;
                        case 0x0A:
                            Buff[myIndex] = '\0';
                            CS = Buff[0];
                            for(myIndex=1;myIndex<strlen(Buff)-2;myIndex++) {
                                CS ^= Buff[myIndex];
                            }
                            // Checksum OK?
                            if(CS == xtoi(&(Buff[strlen(Buff)-2]))) {
                                // LED telegram :030nxx
                                if((Buff[1]=='0')&&(Buff[2]=='3')) {
                                    myLEDOut = xtoi(&(Buff[3]));
                                }
                            }
                            myIndex=0;
                            break;
                        case 0x0D:
                            break;
                        default:
                            if(myIndex<DECODE_BUFFER_SIZE) {
                                Buff[myIndex++] = readBuffer[i];
                            } else {
                                myIndex=0;
                            }
                            break;
                    }
                }  
            }

            CDCTxService();
            
        }
        
    
        // Guard the Watchdog
        WDTCONbits.WDTCLRKEY=0x5743;  // Magic sequence to reset WDT
                
        
    }
    return 1; 
}
/**
 End of File
*/

