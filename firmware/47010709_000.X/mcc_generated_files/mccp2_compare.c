/**
  MCCP2 Generated Driver File 

  @Company
    Microchip Technology Inc.

  @File Name
    mccp2.c

  @Summary
    This is the generated driver implementation file for the MCCP2 driver using PIC24 / dsPIC33 / PIC32MM MCUs

  @Description
    This header file provides implementations for driver APIs for MCCP2. 
    Generation Information : 
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.171.2
        Device            :  PIC32MM0256GPM064
    The generated drivers are tested against the following:
        Compiler          :  XC32 v2.50
        MPLAB             :  MPLAB X v6.05
*/

/*
    (c) 2020 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

/**
  Section: Included Files
*/

#include "mccp2_compare.h"

/** OC Mode.

  @Summary
    Defines the OC Mode.

  @Description
    This data type defines the OC Mode of operation.

*/

static uint16_t         gMCCP2Mode;

/**
  Section: Driver Interface
*/


void MCCP2_COMPARE_Initialize (void)
{
    // ON enabled; MOD Dual Edge Compare, Buffered(PWM); ALTSYNC disabled; SIDL disabled; OPS Each Time Base Period Match; CCPSLP disabled; TMRSYNC disabled; RTRGEN disabled; CCSEL disabled; ONESHOT disabled; TRIGEN disabled; T32 16 Bit; SYNC None; OPSSRC Special Event Trigger; TMRPS 1:1; CLKSEL SYSCLK; 
    CCP2CON1 = (0x80008005 & 0xFFFF7FFF); //Disabling CCPON bit
    //OCCEN enabled; OCDEN disabled; ASDGM disabled; OCEEN disabled; ICGSM Level-Sensitive mode; OCFEN disabled; ICS ICM2; SSDG disabled; AUXOUT OC Signal; ASDG 0; OCAEN disabled; OCBEN disabled; OENSYNC enabled; PWMRSEN enabled; 
    CCP2CON2 = 0x84188000;
    //DT 0; OETRIG disabled; OSCNT None; POLACE disabled; POLBDF disabled; PSSBDF Tri-state; OUTM Steerable single output; PSSACE Tri-state; 
    CCP2CON3 = 0x00;
    //SCEVT disabled; TRSET disabled; ICOV disabled; ASEVT disabled; ICGARM disabled; RBWIP disabled; TRCLR disabled; RAWIP disabled; TMRHWIP disabled; TMRLWIP disabled; PRLWIP disabled; 
    CCP2STAT = 0x00;
    //TMRL 0; TMRH 0; 
    CCP2TMR = 0x00;
    //PRH 0; PRL 65535; 
    CCP2PR = 0xFFFF;
    //CMPA 0; 
    CCP2RA = 0x00;
    //CMPB 0; 
    CCP2RB = 0x00;
    //BUFL 0; BUFH 0; 
    CCP2BUF = 0x00;

    CCP2CON1bits.ON = 0x1; //Enabling CCP

    gMCCP2Mode = CCP2CON1bits.MOD;

}

void __attribute__ ((weak)) MCCP2_COMPARE_CallBack(void)
{
    // Add your custom callback code here
}

void MCCP2_COMPARE_Tasks( void )
{
    if(IFS2bits.CCP2IF)
    {
		// MCCP2 COMPARE callback function 
		MCCP2_COMPARE_CallBack();
		
        IFS2CLR= 1 << _IFS2_CCP2IF_POSITION;
    }
}

void __attribute__ ((weak)) MCCP2_COMPARE_TimerCallBack(void)
{
    // Add your custom callback code here
}


void MCCP2_COMPARE_TimerTasks( void )
{
    if(IFS2bits.CCT2IF)
    {
		// MCCP2 COMPARE Timer callback function 
		MCCP2_COMPARE_TimerCallBack();
	
        IFS2CLR= 1 << _IFS2_CCT2IF_POSITION;
    }
}

void MCCP2_COMPARE_Start( void )
{
    /* Start the Timer */
    CCP2CON1SET = (1 << _CCP2CON1_ON_POSITION);
}

void MCCP2_COMPARE_Stop( void )
{
    /* Start the Timer */
    CCP2CON1CLR = (1 << _CCP2CON1_ON_POSITION);
}

void MCCP2_COMPARE_SingleCompare16ValueSet( uint16_t value )
{   
    CCP2RA = value;
}


void MCCP2_COMPARE_DualCompareValueSet( uint16_t priVal, uint16_t secVal )
{

    CCP2RA = priVal;

    CCP2RB = secVal;
}

void MCCP2_COMPARE_DualEdgeBufferedConfig( uint16_t priVal, uint16_t secVal )
{

    CCP2RA = priVal;

    CCP2RB = secVal;
}

void MCCP2_COMPARE_CenterAlignedPWMConfig( uint16_t priVal, uint16_t secVal )
{

    CCP2RA = priVal;

    CCP2RB = secVal;
}

void MCCP2_COMPARE_EdgeAlignedPWMConfig( uint16_t priVal, uint16_t secVal )
{

    CCP2RA = priVal;

    CCP2RB = secVal;
}

void MCCP2_COMPARE_VariableFrequencyPulseConfig( uint16_t priVal )
{
    CCP2RA = priVal;
}

bool MCCP2_COMPARE_IsCompareCycleComplete( void )
{
    return(IFS2bits.CCP2IF);
}

bool MCCP2_COMPARE_TriggerStatusGet( void )
{
    return( CCP2STATbits.CCPTRIG );
    
}

void MCCP2_COMPARE_TriggerStatusSet( void )
{
    CCP2STATSET = (1 << _CCP2STAT_TRSET_POSITION);
}

void MCCP2_COMPARE_TriggerStatusClear( void )
{
    /* Clears the trigger status */
    CCP2STATCLR = (1 << _CCP2STAT_TRCLR_POSITION);
}

bool MCCP2_COMPARE_SingleCompareStatusGet( void )
{
    return( CCP2STATbits.SCEVT );
}

void MCCP2_COMPARE_SingleCompareStatusClear( void )
{
    /* Clears the trigger status */
    CCP2STATCLR = (1 << _CCP2STAT_SCEVT_POSITION);
    
}
/**
 End of File
*/
