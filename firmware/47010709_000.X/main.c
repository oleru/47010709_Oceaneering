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
    "Rev.: 000" - 2024.11.02 - OPR, Torka AS as
        - Minumum funksjonal versjon release.
 
*/

/**
  Section: Defines
*/


/**
  Section: Included Files
*/
#include <stdint.h>
#include "mcc_generated_files/system.h"
#include "mcc_generated_files/pin_manager.h"
#include "mcc_generated_files/coretimer.h"
#include "mcc_generated_files/adc1.h"
#include "mcc_generated_files/usb/../usb/usb_device_cdc.h"
#include "DB_Debounce.h"



// Version Tags
char VersionTag1[] = __DATE__;     // "Jan 24 2011"
char VersionTag2[] = __TIME__;     // "14:45:20"
char RevBuild[] = "Rev.: 000";

volatile bool TimerEvent1ms;
volatile uint32_t myTime=0;
bool TimerEvent10ms=false, TimerEvent50ms=false, TimerEvent250ms=false, TimerEvent1s=false, TimerEvent15s=false;
uint32_t myLastTime=0;
uint32_t mySystemTimeOutTimer;

typedef struct {
    uint32_t data[16];
    uint32_t lastValue;
    uint8_t index;
    uint8_t notFirstRound;
}  __attribute__ ((packed)) stMA_t;

volatile stMA_t MA_AN4, MA_AN5, MA_AN6, MA_AN7, MA_AN18;


unsigned char Blink=0xF0;


/**
    Prototypes
 */
void MCC_USB_CDC_DemoTasks(void);
void UpdateTimers(void);
uint16_t MA_SetValueAndGetAvarage(volatile stMA_t * dataStorage, uint16_t newValue);


void ADC1_CallBack(void)
{
//    MBS_HoldRegisters[MBS_MD_FOCUS_FB] = MA_SetValueAndGetAvarage(&MA_AN4, ADC1_ConversionResultGet(MD_FOCUS_FB_AN4));
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
                mySystemTimeOutTimer++;
                my15sCnt += 1;
                while(my15sCnt>=15) {
                    TimerEvent15s=true;
                    my15sCnt -= 15;
                }  //..while(my15sCnt>=15)
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


/*
    Main application
 */
int main(void)
{
    uint8_t BlinkCnt;
        
    // initialize the device
    SYSTEM_Initialize();
    
    // Start indicating all well...

    
    // Prepare the ADC
    ADC1_Enable();

    
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
            
        }  //..if(TimerEvent250ms)
        
        
        // 1s Timer Event
        if(TimerEvent1s) {
            TimerEvent1s = false;

           
        }  //..if(TimerEvent1s)

        
        // 15s Timer Event
        if(TimerEvent15s) {
            TimerEvent15s = false;
                        
        }  //..if(TimerEvent15s)
        
        
        // Pin-Pong response 

        MCC_USB_CDC_DemoTasks(); 
        
        // Guard the Watchdog
        WDTCONbits.WDTCLRKEY=0x5743;  // Magic sequence to reset WDT
                
        
    }
    return 1; 
}
/**
 End of File
*/

