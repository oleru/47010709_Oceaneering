/**
  PIN MANAGER Generated Driver File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the generated manager file for the PIC24 / dsPIC33 / PIC32MM MCUs device.  This manager
    configures the pins direction, initial state, analog setting.
    The peripheral pin select, PPS, configuration is also handled by this manager.

  @Description:
    This source file provides implementations for PIN MANAGER.
    Generation Information :
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.171.2
        Device            :  PIC32MM0256GPM064
    The generated drivers are tested against the following:
        Compiler          :  XC32 v2.50
        MPLAB 	          :  MPLAB X v6.05
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

#ifndef _PIN_MANAGER_H
#define _PIN_MANAGER_H
/**
    Section: Includes
*/
#include <xc.h>
#include <stdbool.h>
/**
    Section: Device Pin Macros
*/
/**
  @Summary
    Sets the GPIO pin, RA10, high using LATA10.

  @Description
    Sets the GPIO pin, RA10, high using LATA10.

  @Preconditions
    The RA10 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA10 high (1)
    LED_AUX_1_SetHigh();
    </code>

*/
#define LED_AUX_1_SetHigh()          ( LATASET = (1 << 10) )
/**
  @Summary
    Sets the GPIO pin, RA10, low using LATA10.

  @Description
    Sets the GPIO pin, RA10, low using LATA10.

  @Preconditions
    The RA10 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA10 low (0)
    LED_AUX_1_SetLow();
    </code>

*/
#define LED_AUX_1_SetLow()           ( LATACLR = (1 << 10) )

/**
  @Summary
    Sets a value to the GPIO pin.

  @Description
    Sets or Resets the GPIO pin, RA10, low or high using LATA10.

  @Preconditions
    The RA10 must be set to an output.

  @Returns
    None.

  @Param
    bool value; : value to be set to the GPIO pin.

  @Example
    <code>
    // Set RA10 to low.
    LED_AUX_1_SetValue(false);
    </code>

*/
inline static void LED_AUX_1_SetValue(bool value)
{
  if(value)
  {
    LED_AUX_1_SetHigh();
  }
  else
  {
    LED_AUX_1_SetLow();
  }
}

/**
  @Summary
    Toggles the GPIO pin, RA10, using LATA10.

  @Description
    Toggles the GPIO pin, RA10, using LATA10.

  @Preconditions
    The RA10 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RA10
    LED_AUX_1_Toggle();
    </code>

*/
#define LED_AUX_1_Toggle()           ( LATAINV = (1 << 10) )
/**
  @Summary
    Reads the value of the GPIO pin, RA10.

  @Description
    Reads the value of the GPIO pin, RA10.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RA10
    postValue = LED_AUX_1_GetValue();
    </code>

*/
#define LED_AUX_1_GetValue()         PORTAbits.RA10
/**
  @Summary
    Configures the GPIO pin, RA10, as an input.

  @Description
    Configures the GPIO pin, RA10, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA10 as an input
    LED_AUX_1_SetDigitalInput();
    </code>

*/
#define LED_AUX_1_SetDigitalInput()   ( TRISASET = (1 << 10) )
/**
  @Summary
    Configures the GPIO pin, RA10, as an output.

  @Description
    Configures the GPIO pin, RA10, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA10 as an output
    LED_AUX_1_SetDigitalOutput();
    </code>

*/
#define LED_AUX_1_SetDigitalOutput()   ( TRISACLR = (1 << 10) )
/**
  @Summary
    Sets the GPIO pin, RA2, high using LATA2.

  @Description
    Sets the GPIO pin, RA2, high using LATA2.

  @Preconditions
    The RA2 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA2 high (1)
    WINCH_3_AN_SetHigh();
    </code>

*/
#define WINCH_3_AN_SetHigh()          ( LATASET = (1 << 2) )
/**
  @Summary
    Sets the GPIO pin, RA2, low using LATA2.

  @Description
    Sets the GPIO pin, RA2, low using LATA2.

  @Preconditions
    The RA2 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA2 low (0)
    WINCH_3_AN_SetLow();
    </code>

*/
#define WINCH_3_AN_SetLow()           ( LATACLR = (1 << 2) )

/**
  @Summary
    Sets a value to the GPIO pin.

  @Description
    Sets or Resets the GPIO pin, RA2, low or high using LATA2.

  @Preconditions
    The RA2 must be set to an output.

  @Returns
    None.

  @Param
    bool value; : value to be set to the GPIO pin.

  @Example
    <code>
    // Set RA2 to low.
    WINCH_3_AN_SetValue(false);
    </code>

*/
inline static void WINCH_3_AN_SetValue(bool value)
{
  if(value)
  {
    WINCH_3_AN_SetHigh();
  }
  else
  {
    WINCH_3_AN_SetLow();
  }
}

/**
  @Summary
    Toggles the GPIO pin, RA2, using LATA2.

  @Description
    Toggles the GPIO pin, RA2, using LATA2.

  @Preconditions
    The RA2 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RA2
    WINCH_3_AN_Toggle();
    </code>

*/
#define WINCH_3_AN_Toggle()           ( LATAINV = (1 << 2) )
/**
  @Summary
    Reads the value of the GPIO pin, RA2.

  @Description
    Reads the value of the GPIO pin, RA2.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RA2
    postValue = WINCH_3_AN_GetValue();
    </code>

*/
#define WINCH_3_AN_GetValue()         PORTAbits.RA2
/**
  @Summary
    Configures the GPIO pin, RA2, as an input.

  @Description
    Configures the GPIO pin, RA2, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA2 as an input
    WINCH_3_AN_SetDigitalInput();
    </code>

*/
#define WINCH_3_AN_SetDigitalInput()   ( TRISASET = (1 << 2) )
/**
  @Summary
    Configures the GPIO pin, RA2, as an output.

  @Description
    Configures the GPIO pin, RA2, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA2 as an output
    WINCH_3_AN_SetDigitalOutput();
    </code>

*/
#define WINCH_3_AN_SetDigitalOutput()   ( TRISACLR = (1 << 2) )
/**
  @Summary
    Sets the GPIO pin, RA3, high using LATA3.

  @Description
    Sets the GPIO pin, RA3, high using LATA3.

  @Preconditions
    The RA3 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA3 high (1)
    WINCH_4_AN_SetHigh();
    </code>

*/
#define WINCH_4_AN_SetHigh()          ( LATASET = (1 << 3) )
/**
  @Summary
    Sets the GPIO pin, RA3, low using LATA3.

  @Description
    Sets the GPIO pin, RA3, low using LATA3.

  @Preconditions
    The RA3 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA3 low (0)
    WINCH_4_AN_SetLow();
    </code>

*/
#define WINCH_4_AN_SetLow()           ( LATACLR = (1 << 3) )

/**
  @Summary
    Sets a value to the GPIO pin.

  @Description
    Sets or Resets the GPIO pin, RA3, low or high using LATA3.

  @Preconditions
    The RA3 must be set to an output.

  @Returns
    None.

  @Param
    bool value; : value to be set to the GPIO pin.

  @Example
    <code>
    // Set RA3 to low.
    WINCH_4_AN_SetValue(false);
    </code>

*/
inline static void WINCH_4_AN_SetValue(bool value)
{
  if(value)
  {
    WINCH_4_AN_SetHigh();
  }
  else
  {
    WINCH_4_AN_SetLow();
  }
}

/**
  @Summary
    Toggles the GPIO pin, RA3, using LATA3.

  @Description
    Toggles the GPIO pin, RA3, using LATA3.

  @Preconditions
    The RA3 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RA3
    WINCH_4_AN_Toggle();
    </code>

*/
#define WINCH_4_AN_Toggle()           ( LATAINV = (1 << 3) )
/**
  @Summary
    Reads the value of the GPIO pin, RA3.

  @Description
    Reads the value of the GPIO pin, RA3.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RA3
    postValue = WINCH_4_AN_GetValue();
    </code>

*/
#define WINCH_4_AN_GetValue()         PORTAbits.RA3
/**
  @Summary
    Configures the GPIO pin, RA3, as an input.

  @Description
    Configures the GPIO pin, RA3, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA3 as an input
    WINCH_4_AN_SetDigitalInput();
    </code>

*/
#define WINCH_4_AN_SetDigitalInput()   ( TRISASET = (1 << 3) )
/**
  @Summary
    Configures the GPIO pin, RA3, as an output.

  @Description
    Configures the GPIO pin, RA3, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA3 as an output
    WINCH_4_AN_SetDigitalOutput();
    </code>

*/
#define WINCH_4_AN_SetDigitalOutput()   ( TRISACLR = (1 << 3) )
/**
  @Summary
    Sets the GPIO pin, RA8, high using LATA8.

  @Description
    Sets the GPIO pin, RA8, high using LATA8.

  @Preconditions
    The RA8 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA8 high (1)
    COMMON_BTN_AUX_SetHigh();
    </code>

*/
#define COMMON_BTN_AUX_SetHigh()          ( LATASET = (1 << 8) )
/**
  @Summary
    Sets the GPIO pin, RA8, low using LATA8.

  @Description
    Sets the GPIO pin, RA8, low using LATA8.

  @Preconditions
    The RA8 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA8 low (0)
    COMMON_BTN_AUX_SetLow();
    </code>

*/
#define COMMON_BTN_AUX_SetLow()           ( LATACLR = (1 << 8) )

/**
  @Summary
    Sets a value to the GPIO pin.

  @Description
    Sets or Resets the GPIO pin, RA8, low or high using LATA8.

  @Preconditions
    The RA8 must be set to an output.

  @Returns
    None.

  @Param
    bool value; : value to be set to the GPIO pin.

  @Example
    <code>
    // Set RA8 to low.
    COMMON_BTN_AUX_SetValue(false);
    </code>

*/
inline static void COMMON_BTN_AUX_SetValue(bool value)
{
  if(value)
  {
    COMMON_BTN_AUX_SetHigh();
  }
  else
  {
    COMMON_BTN_AUX_SetLow();
  }
}

/**
  @Summary
    Toggles the GPIO pin, RA8, using LATA8.

  @Description
    Toggles the GPIO pin, RA8, using LATA8.

  @Preconditions
    The RA8 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RA8
    COMMON_BTN_AUX_Toggle();
    </code>

*/
#define COMMON_BTN_AUX_Toggle()           ( LATAINV = (1 << 8) )
/**
  @Summary
    Reads the value of the GPIO pin, RA8.

  @Description
    Reads the value of the GPIO pin, RA8.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RA8
    postValue = COMMON_BTN_AUX_GetValue();
    </code>

*/
#define COMMON_BTN_AUX_GetValue()         PORTAbits.RA8
/**
  @Summary
    Configures the GPIO pin, RA8, as an input.

  @Description
    Configures the GPIO pin, RA8, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA8 as an input
    COMMON_BTN_AUX_SetDigitalInput();
    </code>

*/
#define COMMON_BTN_AUX_SetDigitalInput()   ( TRISASET = (1 << 8) )
/**
  @Summary
    Configures the GPIO pin, RA8, as an output.

  @Description
    Configures the GPIO pin, RA8, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA8 as an output
    COMMON_BTN_AUX_SetDigitalOutput();
    </code>

*/
#define COMMON_BTN_AUX_SetDigitalOutput()   ( TRISACLR = (1 << 8) )
/**
  @Summary
    Sets the GPIO pin, RA9, high using LATA9.

  @Description
    Sets the GPIO pin, RA9, high using LATA9.

  @Preconditions
    The RA9 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA9 high (1)
    BTN_AUX_4B_SetHigh();
    </code>

*/
#define BTN_AUX_4B_SetHigh()          ( LATASET = (1 << 9) )
/**
  @Summary
    Sets the GPIO pin, RA9, low using LATA9.

  @Description
    Sets the GPIO pin, RA9, low using LATA9.

  @Preconditions
    The RA9 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA9 low (0)
    BTN_AUX_4B_SetLow();
    </code>

*/
#define BTN_AUX_4B_SetLow()           ( LATACLR = (1 << 9) )

/**
  @Summary
    Sets a value to the GPIO pin.

  @Description
    Sets or Resets the GPIO pin, RA9, low or high using LATA9.

  @Preconditions
    The RA9 must be set to an output.

  @Returns
    None.

  @Param
    bool value; : value to be set to the GPIO pin.

  @Example
    <code>
    // Set RA9 to low.
    BTN_AUX_4B_SetValue(false);
    </code>

*/
inline static void BTN_AUX_4B_SetValue(bool value)
{
  if(value)
  {
    BTN_AUX_4B_SetHigh();
  }
  else
  {
    BTN_AUX_4B_SetLow();
  }
}

/**
  @Summary
    Toggles the GPIO pin, RA9, using LATA9.

  @Description
    Toggles the GPIO pin, RA9, using LATA9.

  @Preconditions
    The RA9 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RA9
    BTN_AUX_4B_Toggle();
    </code>

*/
#define BTN_AUX_4B_Toggle()           ( LATAINV = (1 << 9) )
/**
  @Summary
    Reads the value of the GPIO pin, RA9.

  @Description
    Reads the value of the GPIO pin, RA9.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RA9
    postValue = BTN_AUX_4B_GetValue();
    </code>

*/
#define BTN_AUX_4B_GetValue()         PORTAbits.RA9
/**
  @Summary
    Configures the GPIO pin, RA9, as an input.

  @Description
    Configures the GPIO pin, RA9, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA9 as an input
    BTN_AUX_4B_SetDigitalInput();
    </code>

*/
#define BTN_AUX_4B_SetDigitalInput()   ( TRISASET = (1 << 9) )
/**
  @Summary
    Configures the GPIO pin, RA9, as an output.

  @Description
    Configures the GPIO pin, RA9, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA9 as an output
    BTN_AUX_4B_SetDigitalOutput();
    </code>

*/
#define BTN_AUX_4B_SetDigitalOutput()   ( TRISACLR = (1 << 9) )
/**
  @Summary
    Sets the GPIO pin, RB0, high using LATB0.

  @Description
    Sets the GPIO pin, RB0, high using LATB0.

  @Preconditions
    The RB0 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB0 high (1)
    PRESSURE_1_AN_SetHigh();
    </code>

*/
#define PRESSURE_1_AN_SetHigh()          ( LATBSET = (1 << 0) )
/**
  @Summary
    Sets the GPIO pin, RB0, low using LATB0.

  @Description
    Sets the GPIO pin, RB0, low using LATB0.

  @Preconditions
    The RB0 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB0 low (0)
    PRESSURE_1_AN_SetLow();
    </code>

*/
#define PRESSURE_1_AN_SetLow()           ( LATBCLR = (1 << 0) )

/**
  @Summary
    Sets a value to the GPIO pin.

  @Description
    Sets or Resets the GPIO pin, RB0, low or high using LATB0.

  @Preconditions
    The RB0 must be set to an output.

  @Returns
    None.

  @Param
    bool value; : value to be set to the GPIO pin.

  @Example
    <code>
    // Set RB0 to low.
    PRESSURE_1_AN_SetValue(false);
    </code>

*/
inline static void PRESSURE_1_AN_SetValue(bool value)
{
  if(value)
  {
    PRESSURE_1_AN_SetHigh();
  }
  else
  {
    PRESSURE_1_AN_SetLow();
  }
}

/**
  @Summary
    Toggles the GPIO pin, RB0, using LATB0.

  @Description
    Toggles the GPIO pin, RB0, using LATB0.

  @Preconditions
    The RB0 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RB0
    PRESSURE_1_AN_Toggle();
    </code>

*/
#define PRESSURE_1_AN_Toggle()           ( LATBINV = (1 << 0) )
/**
  @Summary
    Reads the value of the GPIO pin, RB0.

  @Description
    Reads the value of the GPIO pin, RB0.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RB0
    postValue = PRESSURE_1_AN_GetValue();
    </code>

*/
#define PRESSURE_1_AN_GetValue()         PORTBbits.RB0
/**
  @Summary
    Configures the GPIO pin, RB0, as an input.

  @Description
    Configures the GPIO pin, RB0, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB0 as an input
    PRESSURE_1_AN_SetDigitalInput();
    </code>

*/
#define PRESSURE_1_AN_SetDigitalInput()   ( TRISBSET = (1 << 0) )
/**
  @Summary
    Configures the GPIO pin, RB0, as an output.

  @Description
    Configures the GPIO pin, RB0, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB0 as an output
    PRESSURE_1_AN_SetDigitalOutput();
    </code>

*/
#define PRESSURE_1_AN_SetDigitalOutput()   ( TRISBCLR = (1 << 0) )
/**
  @Summary
    Sets the GPIO pin, RB1, high using LATB1.

  @Description
    Sets the GPIO pin, RB1, high using LATB1.

  @Preconditions
    The RB1 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB1 high (1)
    PRESSURE_2_AN_SetHigh();
    </code>

*/
#define PRESSURE_2_AN_SetHigh()          ( LATBSET = (1 << 1) )
/**
  @Summary
    Sets the GPIO pin, RB1, low using LATB1.

  @Description
    Sets the GPIO pin, RB1, low using LATB1.

  @Preconditions
    The RB1 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB1 low (0)
    PRESSURE_2_AN_SetLow();
    </code>

*/
#define PRESSURE_2_AN_SetLow()           ( LATBCLR = (1 << 1) )

/**
  @Summary
    Sets a value to the GPIO pin.

  @Description
    Sets or Resets the GPIO pin, RB1, low or high using LATB1.

  @Preconditions
    The RB1 must be set to an output.

  @Returns
    None.

  @Param
    bool value; : value to be set to the GPIO pin.

  @Example
    <code>
    // Set RB1 to low.
    PRESSURE_2_AN_SetValue(false);
    </code>

*/
inline static void PRESSURE_2_AN_SetValue(bool value)
{
  if(value)
  {
    PRESSURE_2_AN_SetHigh();
  }
  else
  {
    PRESSURE_2_AN_SetLow();
  }
}

/**
  @Summary
    Toggles the GPIO pin, RB1, using LATB1.

  @Description
    Toggles the GPIO pin, RB1, using LATB1.

  @Preconditions
    The RB1 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RB1
    PRESSURE_2_AN_Toggle();
    </code>

*/
#define PRESSURE_2_AN_Toggle()           ( LATBINV = (1 << 1) )
/**
  @Summary
    Reads the value of the GPIO pin, RB1.

  @Description
    Reads the value of the GPIO pin, RB1.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RB1
    postValue = PRESSURE_2_AN_GetValue();
    </code>

*/
#define PRESSURE_2_AN_GetValue()         PORTBbits.RB1
/**
  @Summary
    Configures the GPIO pin, RB1, as an input.

  @Description
    Configures the GPIO pin, RB1, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB1 as an input
    PRESSURE_2_AN_SetDigitalInput();
    </code>

*/
#define PRESSURE_2_AN_SetDigitalInput()   ( TRISBSET = (1 << 1) )
/**
  @Summary
    Configures the GPIO pin, RB1, as an output.

  @Description
    Configures the GPIO pin, RB1, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB1 as an output
    PRESSURE_2_AN_SetDigitalOutput();
    </code>

*/
#define PRESSURE_2_AN_SetDigitalOutput()   ( TRISBCLR = (1 << 1) )
/**
  @Summary
    Sets the GPIO pin, RB10, high using LATB10.

  @Description
    Sets the GPIO pin, RB10, high using LATB10.

  @Preconditions
    The RB10 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB10 high (1)
    D_N_SetHigh();
    </code>

*/
#define D_N_SetHigh()          ( LATBSET = (1 << 10) )
/**
  @Summary
    Sets the GPIO pin, RB10, low using LATB10.

  @Description
    Sets the GPIO pin, RB10, low using LATB10.

  @Preconditions
    The RB10 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB10 low (0)
    D_N_SetLow();
    </code>

*/
#define D_N_SetLow()           ( LATBCLR = (1 << 10) )

/**
  @Summary
    Sets a value to the GPIO pin.

  @Description
    Sets or Resets the GPIO pin, RB10, low or high using LATB10.

  @Preconditions
    The RB10 must be set to an output.

  @Returns
    None.

  @Param
    bool value; : value to be set to the GPIO pin.

  @Example
    <code>
    // Set RB10 to low.
    D_N_SetValue(false);
    </code>

*/
inline static void D_N_SetValue(bool value)
{
  if(value)
  {
    D_N_SetHigh();
  }
  else
  {
    D_N_SetLow();
  }
}

/**
  @Summary
    Toggles the GPIO pin, RB10, using LATB10.

  @Description
    Toggles the GPIO pin, RB10, using LATB10.

  @Preconditions
    The RB10 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RB10
    D_N_Toggle();
    </code>

*/
#define D_N_Toggle()           ( LATBINV = (1 << 10) )
/**
  @Summary
    Reads the value of the GPIO pin, RB10.

  @Description
    Reads the value of the GPIO pin, RB10.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RB10
    postValue = D_N_GetValue();
    </code>

*/
#define D_N_GetValue()         PORTBbits.RB10
/**
  @Summary
    Configures the GPIO pin, RB10, as an input.

  @Description
    Configures the GPIO pin, RB10, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB10 as an input
    D_N_SetDigitalInput();
    </code>

*/
#define D_N_SetDigitalInput()   ( TRISBSET = (1 << 10) )
/**
  @Summary
    Configures the GPIO pin, RB10, as an output.

  @Description
    Configures the GPIO pin, RB10, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB10 as an output
    D_N_SetDigitalOutput();
    </code>

*/
#define D_N_SetDigitalOutput()   ( TRISBCLR = (1 << 10) )
/**
  @Summary
    Sets the GPIO pin, RB13, high using LATB13.

  @Description
    Sets the GPIO pin, RB13, high using LATB13.

  @Preconditions
    The RB13 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB13 high (1)
    LED_AUX_2_SetHigh();
    </code>

*/
#define LED_AUX_2_SetHigh()          ( LATBSET = (1 << 13) )
/**
  @Summary
    Sets the GPIO pin, RB13, low using LATB13.

  @Description
    Sets the GPIO pin, RB13, low using LATB13.

  @Preconditions
    The RB13 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB13 low (0)
    LED_AUX_2_SetLow();
    </code>

*/
#define LED_AUX_2_SetLow()           ( LATBCLR = (1 << 13) )

/**
  @Summary
    Sets a value to the GPIO pin.

  @Description
    Sets or Resets the GPIO pin, RB13, low or high using LATB13.

  @Preconditions
    The RB13 must be set to an output.

  @Returns
    None.

  @Param
    bool value; : value to be set to the GPIO pin.

  @Example
    <code>
    // Set RB13 to low.
    LED_AUX_2_SetValue(false);
    </code>

*/
inline static void LED_AUX_2_SetValue(bool value)
{
  if(value)
  {
    LED_AUX_2_SetHigh();
  }
  else
  {
    LED_AUX_2_SetLow();
  }
}

/**
  @Summary
    Toggles the GPIO pin, RB13, using LATB13.

  @Description
    Toggles the GPIO pin, RB13, using LATB13.

  @Preconditions
    The RB13 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RB13
    LED_AUX_2_Toggle();
    </code>

*/
#define LED_AUX_2_Toggle()           ( LATBINV = (1 << 13) )
/**
  @Summary
    Reads the value of the GPIO pin, RB13.

  @Description
    Reads the value of the GPIO pin, RB13.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RB13
    postValue = LED_AUX_2_GetValue();
    </code>

*/
#define LED_AUX_2_GetValue()         PORTBbits.RB13
/**
  @Summary
    Configures the GPIO pin, RB13, as an input.

  @Description
    Configures the GPIO pin, RB13, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB13 as an input
    LED_AUX_2_SetDigitalInput();
    </code>

*/
#define LED_AUX_2_SetDigitalInput()   ( TRISBSET = (1 << 13) )
/**
  @Summary
    Configures the GPIO pin, RB13, as an output.

  @Description
    Configures the GPIO pin, RB13, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB13 as an output
    LED_AUX_2_SetDigitalOutput();
    </code>

*/
#define LED_AUX_2_SetDigitalOutput()   ( TRISBCLR = (1 << 13) )
/**
  @Summary
    Sets the GPIO pin, RB15, high using LATB15.

  @Description
    Sets the GPIO pin, RB15, high using LATB15.

  @Preconditions
    The RB15 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB15 high (1)
    COMMON_BTN_EMERGENCY_SetHigh();
    </code>

*/
#define COMMON_BTN_EMERGENCY_SetHigh()          ( LATBSET = (1 << 15) )
/**
  @Summary
    Sets the GPIO pin, RB15, low using LATB15.

  @Description
    Sets the GPIO pin, RB15, low using LATB15.

  @Preconditions
    The RB15 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB15 low (0)
    COMMON_BTN_EMERGENCY_SetLow();
    </code>

*/
#define COMMON_BTN_EMERGENCY_SetLow()           ( LATBCLR = (1 << 15) )

/**
  @Summary
    Sets a value to the GPIO pin.

  @Description
    Sets or Resets the GPIO pin, RB15, low or high using LATB15.

  @Preconditions
    The RB15 must be set to an output.

  @Returns
    None.

  @Param
    bool value; : value to be set to the GPIO pin.

  @Example
    <code>
    // Set RB15 to low.
    COMMON_BTN_EMERGENCY_SetValue(false);
    </code>

*/
inline static void COMMON_BTN_EMERGENCY_SetValue(bool value)
{
  if(value)
  {
    COMMON_BTN_EMERGENCY_SetHigh();
  }
  else
  {
    COMMON_BTN_EMERGENCY_SetLow();
  }
}

/**
  @Summary
    Toggles the GPIO pin, RB15, using LATB15.

  @Description
    Toggles the GPIO pin, RB15, using LATB15.

  @Preconditions
    The RB15 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RB15
    COMMON_BTN_EMERGENCY_Toggle();
    </code>

*/
#define COMMON_BTN_EMERGENCY_Toggle()           ( LATBINV = (1 << 15) )
/**
  @Summary
    Reads the value of the GPIO pin, RB15.

  @Description
    Reads the value of the GPIO pin, RB15.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RB15
    postValue = COMMON_BTN_EMERGENCY_GetValue();
    </code>

*/
#define COMMON_BTN_EMERGENCY_GetValue()         PORTBbits.RB15
/**
  @Summary
    Configures the GPIO pin, RB15, as an input.

  @Description
    Configures the GPIO pin, RB15, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB15 as an input
    COMMON_BTN_EMERGENCY_SetDigitalInput();
    </code>

*/
#define COMMON_BTN_EMERGENCY_SetDigitalInput()   ( TRISBSET = (1 << 15) )
/**
  @Summary
    Configures the GPIO pin, RB15, as an output.

  @Description
    Configures the GPIO pin, RB15, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB15 as an output
    COMMON_BTN_EMERGENCY_SetDigitalOutput();
    </code>

*/
#define COMMON_BTN_EMERGENCY_SetDigitalOutput()   ( TRISBCLR = (1 << 15) )
/**
  @Summary
    Sets the GPIO pin, RB2, high using LATB2.

  @Description
    Sets the GPIO pin, RB2, high using LATB2.

  @Preconditions
    The RB2 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB2 high (1)
    PRESSURE_3_AN_SetHigh();
    </code>

*/
#define PRESSURE_3_AN_SetHigh()          ( LATBSET = (1 << 2) )
/**
  @Summary
    Sets the GPIO pin, RB2, low using LATB2.

  @Description
    Sets the GPIO pin, RB2, low using LATB2.

  @Preconditions
    The RB2 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB2 low (0)
    PRESSURE_3_AN_SetLow();
    </code>

*/
#define PRESSURE_3_AN_SetLow()           ( LATBCLR = (1 << 2) )

/**
  @Summary
    Sets a value to the GPIO pin.

  @Description
    Sets or Resets the GPIO pin, RB2, low or high using LATB2.

  @Preconditions
    The RB2 must be set to an output.

  @Returns
    None.

  @Param
    bool value; : value to be set to the GPIO pin.

  @Example
    <code>
    // Set RB2 to low.
    PRESSURE_3_AN_SetValue(false);
    </code>

*/
inline static void PRESSURE_3_AN_SetValue(bool value)
{
  if(value)
  {
    PRESSURE_3_AN_SetHigh();
  }
  else
  {
    PRESSURE_3_AN_SetLow();
  }
}

/**
  @Summary
    Toggles the GPIO pin, RB2, using LATB2.

  @Description
    Toggles the GPIO pin, RB2, using LATB2.

  @Preconditions
    The RB2 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RB2
    PRESSURE_3_AN_Toggle();
    </code>

*/
#define PRESSURE_3_AN_Toggle()           ( LATBINV = (1 << 2) )
/**
  @Summary
    Reads the value of the GPIO pin, RB2.

  @Description
    Reads the value of the GPIO pin, RB2.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RB2
    postValue = PRESSURE_3_AN_GetValue();
    </code>

*/
#define PRESSURE_3_AN_GetValue()         PORTBbits.RB2
/**
  @Summary
    Configures the GPIO pin, RB2, as an input.

  @Description
    Configures the GPIO pin, RB2, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB2 as an input
    PRESSURE_3_AN_SetDigitalInput();
    </code>

*/
#define PRESSURE_3_AN_SetDigitalInput()   ( TRISBSET = (1 << 2) )
/**
  @Summary
    Configures the GPIO pin, RB2, as an output.

  @Description
    Configures the GPIO pin, RB2, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB2 as an output
    PRESSURE_3_AN_SetDigitalOutput();
    </code>

*/
#define PRESSURE_3_AN_SetDigitalOutput()   ( TRISBCLR = (1 << 2) )
/**
  @Summary
    Sets the GPIO pin, RB3, high using LATB3.

  @Description
    Sets the GPIO pin, RB3, high using LATB3.

  @Preconditions
    The RB3 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB3 high (1)
    PRESSURE_4_AN_SetHigh();
    </code>

*/
#define PRESSURE_4_AN_SetHigh()          ( LATBSET = (1 << 3) )
/**
  @Summary
    Sets the GPIO pin, RB3, low using LATB3.

  @Description
    Sets the GPIO pin, RB3, low using LATB3.

  @Preconditions
    The RB3 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB3 low (0)
    PRESSURE_4_AN_SetLow();
    </code>

*/
#define PRESSURE_4_AN_SetLow()           ( LATBCLR = (1 << 3) )

/**
  @Summary
    Sets a value to the GPIO pin.

  @Description
    Sets or Resets the GPIO pin, RB3, low or high using LATB3.

  @Preconditions
    The RB3 must be set to an output.

  @Returns
    None.

  @Param
    bool value; : value to be set to the GPIO pin.

  @Example
    <code>
    // Set RB3 to low.
    PRESSURE_4_AN_SetValue(false);
    </code>

*/
inline static void PRESSURE_4_AN_SetValue(bool value)
{
  if(value)
  {
    PRESSURE_4_AN_SetHigh();
  }
  else
  {
    PRESSURE_4_AN_SetLow();
  }
}

/**
  @Summary
    Toggles the GPIO pin, RB3, using LATB3.

  @Description
    Toggles the GPIO pin, RB3, using LATB3.

  @Preconditions
    The RB3 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RB3
    PRESSURE_4_AN_Toggle();
    </code>

*/
#define PRESSURE_4_AN_Toggle()           ( LATBINV = (1 << 3) )
/**
  @Summary
    Reads the value of the GPIO pin, RB3.

  @Description
    Reads the value of the GPIO pin, RB3.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RB3
    postValue = PRESSURE_4_AN_GetValue();
    </code>

*/
#define PRESSURE_4_AN_GetValue()         PORTBbits.RB3
/**
  @Summary
    Configures the GPIO pin, RB3, as an input.

  @Description
    Configures the GPIO pin, RB3, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB3 as an input
    PRESSURE_4_AN_SetDigitalInput();
    </code>

*/
#define PRESSURE_4_AN_SetDigitalInput()   ( TRISBSET = (1 << 3) )
/**
  @Summary
    Configures the GPIO pin, RB3, as an output.

  @Description
    Configures the GPIO pin, RB3, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB3 as an output
    PRESSURE_4_AN_SetDigitalOutput();
    </code>

*/
#define PRESSURE_4_AN_SetDigitalOutput()   ( TRISBCLR = (1 << 3) )
/**
  @Summary
    Sets the GPIO pin, RB4, high using LATB4.

  @Description
    Sets the GPIO pin, RB4, high using LATB4.

  @Preconditions
    The RB4 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB4 high (1)
    LIGHT_SENSE_SetHigh();
    </code>

*/
#define LIGHT_SENSE_SetHigh()          ( LATBSET = (1 << 4) )
/**
  @Summary
    Sets the GPIO pin, RB4, low using LATB4.

  @Description
    Sets the GPIO pin, RB4, low using LATB4.

  @Preconditions
    The RB4 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB4 low (0)
    LIGHT_SENSE_SetLow();
    </code>

*/
#define LIGHT_SENSE_SetLow()           ( LATBCLR = (1 << 4) )

/**
  @Summary
    Sets a value to the GPIO pin.

  @Description
    Sets or Resets the GPIO pin, RB4, low or high using LATB4.

  @Preconditions
    The RB4 must be set to an output.

  @Returns
    None.

  @Param
    bool value; : value to be set to the GPIO pin.

  @Example
    <code>
    // Set RB4 to low.
    LIGHT_SENSE_SetValue(false);
    </code>

*/
inline static void LIGHT_SENSE_SetValue(bool value)
{
  if(value)
  {
    LIGHT_SENSE_SetHigh();
  }
  else
  {
    LIGHT_SENSE_SetLow();
  }
}

/**
  @Summary
    Toggles the GPIO pin, RB4, using LATB4.

  @Description
    Toggles the GPIO pin, RB4, using LATB4.

  @Preconditions
    The RB4 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RB4
    LIGHT_SENSE_Toggle();
    </code>

*/
#define LIGHT_SENSE_Toggle()           ( LATBINV = (1 << 4) )
/**
  @Summary
    Reads the value of the GPIO pin, RB4.

  @Description
    Reads the value of the GPIO pin, RB4.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RB4
    postValue = LIGHT_SENSE_GetValue();
    </code>

*/
#define LIGHT_SENSE_GetValue()         PORTBbits.RB4
/**
  @Summary
    Configures the GPIO pin, RB4, as an input.

  @Description
    Configures the GPIO pin, RB4, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB4 as an input
    LIGHT_SENSE_SetDigitalInput();
    </code>

*/
#define LIGHT_SENSE_SetDigitalInput()   ( TRISBSET = (1 << 4) )
/**
  @Summary
    Configures the GPIO pin, RB4, as an output.

  @Description
    Configures the GPIO pin, RB4, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB4 as an output
    LIGHT_SENSE_SetDigitalOutput();
    </code>

*/
#define LIGHT_SENSE_SetDigitalOutput()   ( TRISBCLR = (1 << 4) )
/**
  @Summary
    Sets the GPIO pin, RB8, high using LATB8.

  @Description
    Sets the GPIO pin, RB8, high using LATB8.

  @Preconditions
    The RB8 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB8 high (1)
    LED_AUX_3_SetHigh();
    </code>

*/
#define LED_AUX_3_SetHigh()          ( LATBSET = (1 << 8) )
/**
  @Summary
    Sets the GPIO pin, RB8, low using LATB8.

  @Description
    Sets the GPIO pin, RB8, low using LATB8.

  @Preconditions
    The RB8 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB8 low (0)
    LED_AUX_3_SetLow();
    </code>

*/
#define LED_AUX_3_SetLow()           ( LATBCLR = (1 << 8) )

/**
  @Summary
    Sets a value to the GPIO pin.

  @Description
    Sets or Resets the GPIO pin, RB8, low or high using LATB8.

  @Preconditions
    The RB8 must be set to an output.

  @Returns
    None.

  @Param
    bool value; : value to be set to the GPIO pin.

  @Example
    <code>
    // Set RB8 to low.
    LED_AUX_3_SetValue(false);
    </code>

*/
inline static void LED_AUX_3_SetValue(bool value)
{
  if(value)
  {
    LED_AUX_3_SetHigh();
  }
  else
  {
    LED_AUX_3_SetLow();
  }
}

/**
  @Summary
    Toggles the GPIO pin, RB8, using LATB8.

  @Description
    Toggles the GPIO pin, RB8, using LATB8.

  @Preconditions
    The RB8 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RB8
    LED_AUX_3_Toggle();
    </code>

*/
#define LED_AUX_3_Toggle()           ( LATBINV = (1 << 8) )
/**
  @Summary
    Reads the value of the GPIO pin, RB8.

  @Description
    Reads the value of the GPIO pin, RB8.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RB8
    postValue = LED_AUX_3_GetValue();
    </code>

*/
#define LED_AUX_3_GetValue()         PORTBbits.RB8
/**
  @Summary
    Configures the GPIO pin, RB8, as an input.

  @Description
    Configures the GPIO pin, RB8, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB8 as an input
    LED_AUX_3_SetDigitalInput();
    </code>

*/
#define LED_AUX_3_SetDigitalInput()   ( TRISBSET = (1 << 8) )
/**
  @Summary
    Configures the GPIO pin, RB8, as an output.

  @Description
    Configures the GPIO pin, RB8, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB8 as an output
    LED_AUX_3_SetDigitalOutput();
    </code>

*/
#define LED_AUX_3_SetDigitalOutput()   ( TRISBCLR = (1 << 8) )
/**
  @Summary
    Sets the GPIO pin, RB9, high using LATB9.

  @Description
    Sets the GPIO pin, RB9, high using LATB9.

  @Preconditions
    The RB9 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB9 high (1)
    BTN_EMERGENCY_RELEASE_SetHigh();
    </code>

*/
#define BTN_EMERGENCY_RELEASE_SetHigh()          ( LATBSET = (1 << 9) )
/**
  @Summary
    Sets the GPIO pin, RB9, low using LATB9.

  @Description
    Sets the GPIO pin, RB9, low using LATB9.

  @Preconditions
    The RB9 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB9 low (0)
    BTN_EMERGENCY_RELEASE_SetLow();
    </code>

*/
#define BTN_EMERGENCY_RELEASE_SetLow()           ( LATBCLR = (1 << 9) )

/**
  @Summary
    Sets a value to the GPIO pin.

  @Description
    Sets or Resets the GPIO pin, RB9, low or high using LATB9.

  @Preconditions
    The RB9 must be set to an output.

  @Returns
    None.

  @Param
    bool value; : value to be set to the GPIO pin.

  @Example
    <code>
    // Set RB9 to low.
    BTN_EMERGENCY_RELEASE_SetValue(false);
    </code>

*/
inline static void BTN_EMERGENCY_RELEASE_SetValue(bool value)
{
  if(value)
  {
    BTN_EMERGENCY_RELEASE_SetHigh();
  }
  else
  {
    BTN_EMERGENCY_RELEASE_SetLow();
  }
}

/**
  @Summary
    Toggles the GPIO pin, RB9, using LATB9.

  @Description
    Toggles the GPIO pin, RB9, using LATB9.

  @Preconditions
    The RB9 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RB9
    BTN_EMERGENCY_RELEASE_Toggle();
    </code>

*/
#define BTN_EMERGENCY_RELEASE_Toggle()           ( LATBINV = (1 << 9) )
/**
  @Summary
    Reads the value of the GPIO pin, RB9.

  @Description
    Reads the value of the GPIO pin, RB9.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RB9
    postValue = BTN_EMERGENCY_RELEASE_GetValue();
    </code>

*/
#define BTN_EMERGENCY_RELEASE_GetValue()         PORTBbits.RB9
/**
  @Summary
    Configures the GPIO pin, RB9, as an input.

  @Description
    Configures the GPIO pin, RB9, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB9 as an input
    BTN_EMERGENCY_RELEASE_SetDigitalInput();
    </code>

*/
#define BTN_EMERGENCY_RELEASE_SetDigitalInput()   ( TRISBSET = (1 << 9) )
/**
  @Summary
    Configures the GPIO pin, RB9, as an output.

  @Description
    Configures the GPIO pin, RB9, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB9 as an output
    BTN_EMERGENCY_RELEASE_SetDigitalOutput();
    </code>

*/
#define BTN_EMERGENCY_RELEASE_SetDigitalOutput()   ( TRISBCLR = (1 << 9) )
/**
  @Summary
    Sets the GPIO pin, RC0, high using LATC0.

  @Description
    Sets the GPIO pin, RC0, high using LATC0.

  @Preconditions
    The RC0 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC0 high (1)
    WINCH_1_AN_SetHigh();
    </code>

*/
#define WINCH_1_AN_SetHigh()          ( LATCSET = (1 << 0) )
/**
  @Summary
    Sets the GPIO pin, RC0, low using LATC0.

  @Description
    Sets the GPIO pin, RC0, low using LATC0.

  @Preconditions
    The RC0 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC0 low (0)
    WINCH_1_AN_SetLow();
    </code>

*/
#define WINCH_1_AN_SetLow()           ( LATCCLR = (1 << 0) )

/**
  @Summary
    Sets a value to the GPIO pin.

  @Description
    Sets or Resets the GPIO pin, RC0, low or high using LATC0.

  @Preconditions
    The RC0 must be set to an output.

  @Returns
    None.

  @Param
    bool value; : value to be set to the GPIO pin.

  @Example
    <code>
    // Set RC0 to low.
    WINCH_1_AN_SetValue(false);
    </code>

*/
inline static void WINCH_1_AN_SetValue(bool value)
{
  if(value)
  {
    WINCH_1_AN_SetHigh();
  }
  else
  {
    WINCH_1_AN_SetLow();
  }
}

/**
  @Summary
    Toggles the GPIO pin, RC0, using LATC0.

  @Description
    Toggles the GPIO pin, RC0, using LATC0.

  @Preconditions
    The RC0 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RC0
    WINCH_1_AN_Toggle();
    </code>

*/
#define WINCH_1_AN_Toggle()           ( LATCINV = (1 << 0) )
/**
  @Summary
    Reads the value of the GPIO pin, RC0.

  @Description
    Reads the value of the GPIO pin, RC0.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RC0
    postValue = WINCH_1_AN_GetValue();
    </code>

*/
#define WINCH_1_AN_GetValue()         PORTCbits.RC0
/**
  @Summary
    Configures the GPIO pin, RC0, as an input.

  @Description
    Configures the GPIO pin, RC0, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC0 as an input
    WINCH_1_AN_SetDigitalInput();
    </code>

*/
#define WINCH_1_AN_SetDigitalInput()   ( TRISCSET = (1 << 0) )
/**
  @Summary
    Configures the GPIO pin, RC0, as an output.

  @Description
    Configures the GPIO pin, RC0, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC0 as an output
    WINCH_1_AN_SetDigitalOutput();
    </code>

*/
#define WINCH_1_AN_SetDigitalOutput()   ( TRISCCLR = (1 << 0) )
/**
  @Summary
    Sets the GPIO pin, RC1, high using LATC1.

  @Description
    Sets the GPIO pin, RC1, high using LATC1.

  @Preconditions
    The RC1 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC1 high (1)
    WINCH_2_AN_SetHigh();
    </code>

*/
#define WINCH_2_AN_SetHigh()          ( LATCSET = (1 << 1) )
/**
  @Summary
    Sets the GPIO pin, RC1, low using LATC1.

  @Description
    Sets the GPIO pin, RC1, low using LATC1.

  @Preconditions
    The RC1 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC1 low (0)
    WINCH_2_AN_SetLow();
    </code>

*/
#define WINCH_2_AN_SetLow()           ( LATCCLR = (1 << 1) )

/**
  @Summary
    Sets a value to the GPIO pin.

  @Description
    Sets or Resets the GPIO pin, RC1, low or high using LATC1.

  @Preconditions
    The RC1 must be set to an output.

  @Returns
    None.

  @Param
    bool value; : value to be set to the GPIO pin.

  @Example
    <code>
    // Set RC1 to low.
    WINCH_2_AN_SetValue(false);
    </code>

*/
inline static void WINCH_2_AN_SetValue(bool value)
{
  if(value)
  {
    WINCH_2_AN_SetHigh();
  }
  else
  {
    WINCH_2_AN_SetLow();
  }
}

/**
  @Summary
    Toggles the GPIO pin, RC1, using LATC1.

  @Description
    Toggles the GPIO pin, RC1, using LATC1.

  @Preconditions
    The RC1 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RC1
    WINCH_2_AN_Toggle();
    </code>

*/
#define WINCH_2_AN_Toggle()           ( LATCINV = (1 << 1) )
/**
  @Summary
    Reads the value of the GPIO pin, RC1.

  @Description
    Reads the value of the GPIO pin, RC1.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RC1
    postValue = WINCH_2_AN_GetValue();
    </code>

*/
#define WINCH_2_AN_GetValue()         PORTCbits.RC1
/**
  @Summary
    Configures the GPIO pin, RC1, as an input.

  @Description
    Configures the GPIO pin, RC1, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC1 as an input
    WINCH_2_AN_SetDigitalInput();
    </code>

*/
#define WINCH_2_AN_SetDigitalInput()   ( TRISCSET = (1 << 1) )
/**
  @Summary
    Configures the GPIO pin, RC1, as an output.

  @Description
    Configures the GPIO pin, RC1, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC1 as an output
    WINCH_2_AN_SetDigitalOutput();
    </code>

*/
#define WINCH_2_AN_SetDigitalOutput()   ( TRISCCLR = (1 << 1) )
/**
  @Summary
    Sets the GPIO pin, RC13, high using LATC13.

  @Description
    Sets the GPIO pin, RC13, high using LATC13.

  @Preconditions
    The RC13 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC13 high (1)
    LED_AUX_4_SetHigh();
    </code>

*/
#define LED_AUX_4_SetHigh()          ( LATCSET = (1 << 13) )
/**
  @Summary
    Sets the GPIO pin, RC13, low using LATC13.

  @Description
    Sets the GPIO pin, RC13, low using LATC13.

  @Preconditions
    The RC13 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC13 low (0)
    LED_AUX_4_SetLow();
    </code>

*/
#define LED_AUX_4_SetLow()           ( LATCCLR = (1 << 13) )

/**
  @Summary
    Sets a value to the GPIO pin.

  @Description
    Sets or Resets the GPIO pin, RC13, low or high using LATC13.

  @Preconditions
    The RC13 must be set to an output.

  @Returns
    None.

  @Param
    bool value; : value to be set to the GPIO pin.

  @Example
    <code>
    // Set RC13 to low.
    LED_AUX_4_SetValue(false);
    </code>

*/
inline static void LED_AUX_4_SetValue(bool value)
{
  if(value)
  {
    LED_AUX_4_SetHigh();
  }
  else
  {
    LED_AUX_4_SetLow();
  }
}

/**
  @Summary
    Toggles the GPIO pin, RC13, using LATC13.

  @Description
    Toggles the GPIO pin, RC13, using LATC13.

  @Preconditions
    The RC13 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RC13
    LED_AUX_4_Toggle();
    </code>

*/
#define LED_AUX_4_Toggle()           ( LATCINV = (1 << 13) )
/**
  @Summary
    Reads the value of the GPIO pin, RC13.

  @Description
    Reads the value of the GPIO pin, RC13.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RC13
    postValue = LED_AUX_4_GetValue();
    </code>

*/
#define LED_AUX_4_GetValue()         PORTCbits.RC13
/**
  @Summary
    Configures the GPIO pin, RC13, as an input.

  @Description
    Configures the GPIO pin, RC13, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC13 as an input
    LED_AUX_4_SetDigitalInput();
    </code>

*/
#define LED_AUX_4_SetDigitalInput()   ( TRISCSET = (1 << 13) )
/**
  @Summary
    Configures the GPIO pin, RC13, as an output.

  @Description
    Configures the GPIO pin, RC13, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC13 as an output
    LED_AUX_4_SetDigitalOutput();
    </code>

*/
#define LED_AUX_4_SetDigitalOutput()   ( TRISCCLR = (1 << 13) )
/**
  @Summary
    Sets the GPIO pin, RC4, high using LATC4.

  @Description
    Sets the GPIO pin, RC4, high using LATC4.

  @Preconditions
    The RC4 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC4 high (1)
    ELFILM_EN_SetHigh();
    </code>

*/
#define ELFILM_EN_SetHigh()          ( LATCSET = (1 << 4) )
/**
  @Summary
    Sets the GPIO pin, RC4, low using LATC4.

  @Description
    Sets the GPIO pin, RC4, low using LATC4.

  @Preconditions
    The RC4 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC4 low (0)
    ELFILM_EN_SetLow();
    </code>

*/
#define ELFILM_EN_SetLow()           ( LATCCLR = (1 << 4) )

/**
  @Summary
    Sets a value to the GPIO pin.

  @Description
    Sets or Resets the GPIO pin, RC4, low or high using LATC4.

  @Preconditions
    The RC4 must be set to an output.

  @Returns
    None.

  @Param
    bool value; : value to be set to the GPIO pin.

  @Example
    <code>
    // Set RC4 to low.
    ELFILM_EN_SetValue(false);
    </code>

*/
inline static void ELFILM_EN_SetValue(bool value)
{
  if(value)
  {
    ELFILM_EN_SetHigh();
  }
  else
  {
    ELFILM_EN_SetLow();
  }
}

/**
  @Summary
    Toggles the GPIO pin, RC4, using LATC4.

  @Description
    Toggles the GPIO pin, RC4, using LATC4.

  @Preconditions
    The RC4 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RC4
    ELFILM_EN_Toggle();
    </code>

*/
#define ELFILM_EN_Toggle()           ( LATCINV = (1 << 4) )
/**
  @Summary
    Reads the value of the GPIO pin, RC4.

  @Description
    Reads the value of the GPIO pin, RC4.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RC4
    postValue = ELFILM_EN_GetValue();
    </code>

*/
#define ELFILM_EN_GetValue()         PORTCbits.RC4
/**
  @Summary
    Configures the GPIO pin, RC4, as an input.

  @Description
    Configures the GPIO pin, RC4, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC4 as an input
    ELFILM_EN_SetDigitalInput();
    </code>

*/
#define ELFILM_EN_SetDigitalInput()   ( TRISCSET = (1 << 4) )
/**
  @Summary
    Configures the GPIO pin, RC4, as an output.

  @Description
    Configures the GPIO pin, RC4, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC4 as an output
    ELFILM_EN_SetDigitalOutput();
    </code>

*/
#define ELFILM_EN_SetDigitalOutput()   ( TRISCCLR = (1 << 4) )
/**
  @Summary
    Sets the GPIO pin, RC9, high using LATC9.

  @Description
    Sets the GPIO pin, RC9, high using LATC9.

  @Preconditions
    The RC9 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC9 high (1)
    OP_STATUS_SetHigh();
    </code>

*/
#define OP_STATUS_SetHigh()          ( LATCSET = (1 << 9) )
/**
  @Summary
    Sets the GPIO pin, RC9, low using LATC9.

  @Description
    Sets the GPIO pin, RC9, low using LATC9.

  @Preconditions
    The RC9 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC9 low (0)
    OP_STATUS_SetLow();
    </code>

*/
#define OP_STATUS_SetLow()           ( LATCCLR = (1 << 9) )

/**
  @Summary
    Sets a value to the GPIO pin.

  @Description
    Sets or Resets the GPIO pin, RC9, low or high using LATC9.

  @Preconditions
    The RC9 must be set to an output.

  @Returns
    None.

  @Param
    bool value; : value to be set to the GPIO pin.

  @Example
    <code>
    // Set RC9 to low.
    OP_STATUS_SetValue(false);
    </code>

*/
inline static void OP_STATUS_SetValue(bool value)
{
  if(value)
  {
    OP_STATUS_SetHigh();
  }
  else
  {
    OP_STATUS_SetLow();
  }
}

/**
  @Summary
    Toggles the GPIO pin, RC9, using LATC9.

  @Description
    Toggles the GPIO pin, RC9, using LATC9.

  @Preconditions
    The RC9 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RC9
    OP_STATUS_Toggle();
    </code>

*/
#define OP_STATUS_Toggle()           ( LATCINV = (1 << 9) )
/**
  @Summary
    Reads the value of the GPIO pin, RC9.

  @Description
    Reads the value of the GPIO pin, RC9.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RC9
    postValue = OP_STATUS_GetValue();
    </code>

*/
#define OP_STATUS_GetValue()         PORTCbits.RC9
/**
  @Summary
    Configures the GPIO pin, RC9, as an input.

  @Description
    Configures the GPIO pin, RC9, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC9 as an input
    OP_STATUS_SetDigitalInput();
    </code>

*/
#define OP_STATUS_SetDigitalInput()   ( TRISCSET = (1 << 9) )
/**
  @Summary
    Configures the GPIO pin, RC9, as an output.

  @Description
    Configures the GPIO pin, RC9, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC9 as an output
    OP_STATUS_SetDigitalOutput();
    </code>

*/
#define OP_STATUS_SetDigitalOutput()   ( TRISCCLR = (1 << 9) )
/**
  @Summary
    Sets the GPIO pin, RD0, high using LATD0.

  @Description
    Sets the GPIO pin, RD0, high using LATD0.

  @Preconditions
    The RD0 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD0 high (1)
    BTN_EMERGENCY_STOP_SetHigh();
    </code>

*/
#define BTN_EMERGENCY_STOP_SetHigh()          ( LATDSET = (1 << 0) )
/**
  @Summary
    Sets the GPIO pin, RD0, low using LATD0.

  @Description
    Sets the GPIO pin, RD0, low using LATD0.

  @Preconditions
    The RD0 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD0 low (0)
    BTN_EMERGENCY_STOP_SetLow();
    </code>

*/
#define BTN_EMERGENCY_STOP_SetLow()           ( LATDCLR = (1 << 0) )

/**
  @Summary
    Sets a value to the GPIO pin.

  @Description
    Sets or Resets the GPIO pin, RD0, low or high using LATD0.

  @Preconditions
    The RD0 must be set to an output.

  @Returns
    None.

  @Param
    bool value; : value to be set to the GPIO pin.

  @Example
    <code>
    // Set RD0 to low.
    BTN_EMERGENCY_STOP_SetValue(false);
    </code>

*/
inline static void BTN_EMERGENCY_STOP_SetValue(bool value)
{
  if(value)
  {
    BTN_EMERGENCY_STOP_SetHigh();
  }
  else
  {
    BTN_EMERGENCY_STOP_SetLow();
  }
}

/**
  @Summary
    Toggles the GPIO pin, RD0, using LATD0.

  @Description
    Toggles the GPIO pin, RD0, using LATD0.

  @Preconditions
    The RD0 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RD0
    BTN_EMERGENCY_STOP_Toggle();
    </code>

*/
#define BTN_EMERGENCY_STOP_Toggle()           ( LATDINV = (1 << 0) )
/**
  @Summary
    Reads the value of the GPIO pin, RD0.

  @Description
    Reads the value of the GPIO pin, RD0.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RD0
    postValue = BTN_EMERGENCY_STOP_GetValue();
    </code>

*/
#define BTN_EMERGENCY_STOP_GetValue()         PORTDbits.RD0
/**
  @Summary
    Configures the GPIO pin, RD0, as an input.

  @Description
    Configures the GPIO pin, RD0, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD0 as an input
    BTN_EMERGENCY_STOP_SetDigitalInput();
    </code>

*/
#define BTN_EMERGENCY_STOP_SetDigitalInput()   ( TRISDSET = (1 << 0) )
/**
  @Summary
    Configures the GPIO pin, RD0, as an output.

  @Description
    Configures the GPIO pin, RD0, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD0 as an output
    BTN_EMERGENCY_STOP_SetDigitalOutput();
    </code>

*/
#define BTN_EMERGENCY_STOP_SetDigitalOutput()   ( TRISDCLR = (1 << 0) )
/**
  @Summary
    Sets the GPIO pin, RD1, high using LATD1.

  @Description
    Sets the GPIO pin, RD1, high using LATD1.

  @Preconditions
    The RD1 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD1 high (1)
    BTN_AUX_1_SetHigh();
    </code>

*/
#define BTN_AUX_1_SetHigh()          ( LATDSET = (1 << 1) )
/**
  @Summary
    Sets the GPIO pin, RD1, low using LATD1.

  @Description
    Sets the GPIO pin, RD1, low using LATD1.

  @Preconditions
    The RD1 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD1 low (0)
    BTN_AUX_1_SetLow();
    </code>

*/
#define BTN_AUX_1_SetLow()           ( LATDCLR = (1 << 1) )

/**
  @Summary
    Sets a value to the GPIO pin.

  @Description
    Sets or Resets the GPIO pin, RD1, low or high using LATD1.

  @Preconditions
    The RD1 must be set to an output.

  @Returns
    None.

  @Param
    bool value; : value to be set to the GPIO pin.

  @Example
    <code>
    // Set RD1 to low.
    BTN_AUX_1_SetValue(false);
    </code>

*/
inline static void BTN_AUX_1_SetValue(bool value)
{
  if(value)
  {
    BTN_AUX_1_SetHigh();
  }
  else
  {
    BTN_AUX_1_SetLow();
  }
}

/**
  @Summary
    Toggles the GPIO pin, RD1, using LATD1.

  @Description
    Toggles the GPIO pin, RD1, using LATD1.

  @Preconditions
    The RD1 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RD1
    BTN_AUX_1_Toggle();
    </code>

*/
#define BTN_AUX_1_Toggle()           ( LATDINV = (1 << 1) )
/**
  @Summary
    Reads the value of the GPIO pin, RD1.

  @Description
    Reads the value of the GPIO pin, RD1.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RD1
    postValue = BTN_AUX_1_GetValue();
    </code>

*/
#define BTN_AUX_1_GetValue()         PORTDbits.RD1
/**
  @Summary
    Configures the GPIO pin, RD1, as an input.

  @Description
    Configures the GPIO pin, RD1, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD1 as an input
    BTN_AUX_1_SetDigitalInput();
    </code>

*/
#define BTN_AUX_1_SetDigitalInput()   ( TRISDSET = (1 << 1) )
/**
  @Summary
    Configures the GPIO pin, RD1, as an output.

  @Description
    Configures the GPIO pin, RD1, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD1 as an output
    BTN_AUX_1_SetDigitalOutput();
    </code>

*/
#define BTN_AUX_1_SetDigitalOutput()   ( TRISDCLR = (1 << 1) )
/**
  @Summary
    Sets the GPIO pin, RD2, high using LATD2.

  @Description
    Sets the GPIO pin, RD2, high using LATD2.

  @Preconditions
    The RD2 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD2 high (1)
    BTN_AUX_2_SetHigh();
    </code>

*/
#define BTN_AUX_2_SetHigh()          ( LATDSET = (1 << 2) )
/**
  @Summary
    Sets the GPIO pin, RD2, low using LATD2.

  @Description
    Sets the GPIO pin, RD2, low using LATD2.

  @Preconditions
    The RD2 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD2 low (0)
    BTN_AUX_2_SetLow();
    </code>

*/
#define BTN_AUX_2_SetLow()           ( LATDCLR = (1 << 2) )

/**
  @Summary
    Sets a value to the GPIO pin.

  @Description
    Sets or Resets the GPIO pin, RD2, low or high using LATD2.

  @Preconditions
    The RD2 must be set to an output.

  @Returns
    None.

  @Param
    bool value; : value to be set to the GPIO pin.

  @Example
    <code>
    // Set RD2 to low.
    BTN_AUX_2_SetValue(false);
    </code>

*/
inline static void BTN_AUX_2_SetValue(bool value)
{
  if(value)
  {
    BTN_AUX_2_SetHigh();
  }
  else
  {
    BTN_AUX_2_SetLow();
  }
}

/**
  @Summary
    Toggles the GPIO pin, RD2, using LATD2.

  @Description
    Toggles the GPIO pin, RD2, using LATD2.

  @Preconditions
    The RD2 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RD2
    BTN_AUX_2_Toggle();
    </code>

*/
#define BTN_AUX_2_Toggle()           ( LATDINV = (1 << 2) )
/**
  @Summary
    Reads the value of the GPIO pin, RD2.

  @Description
    Reads the value of the GPIO pin, RD2.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RD2
    postValue = BTN_AUX_2_GetValue();
    </code>

*/
#define BTN_AUX_2_GetValue()         PORTDbits.RD2
/**
  @Summary
    Configures the GPIO pin, RD2, as an input.

  @Description
    Configures the GPIO pin, RD2, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD2 as an input
    BTN_AUX_2_SetDigitalInput();
    </code>

*/
#define BTN_AUX_2_SetDigitalInput()   ( TRISDSET = (1 << 2) )
/**
  @Summary
    Configures the GPIO pin, RD2, as an output.

  @Description
    Configures the GPIO pin, RD2, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD2 as an output
    BTN_AUX_2_SetDigitalOutput();
    </code>

*/
#define BTN_AUX_2_SetDigitalOutput()   ( TRISDCLR = (1 << 2) )
/**
  @Summary
    Sets the GPIO pin, RD3, high using LATD3.

  @Description
    Sets the GPIO pin, RD3, high using LATD3.

  @Preconditions
    The RD3 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD3 high (1)
    BTN_AUX_3_SetHigh();
    </code>

*/
#define BTN_AUX_3_SetHigh()          ( LATDSET = (1 << 3) )
/**
  @Summary
    Sets the GPIO pin, RD3, low using LATD3.

  @Description
    Sets the GPIO pin, RD3, low using LATD3.

  @Preconditions
    The RD3 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD3 low (0)
    BTN_AUX_3_SetLow();
    </code>

*/
#define BTN_AUX_3_SetLow()           ( LATDCLR = (1 << 3) )

/**
  @Summary
    Sets a value to the GPIO pin.

  @Description
    Sets or Resets the GPIO pin, RD3, low or high using LATD3.

  @Preconditions
    The RD3 must be set to an output.

  @Returns
    None.

  @Param
    bool value; : value to be set to the GPIO pin.

  @Example
    <code>
    // Set RD3 to low.
    BTN_AUX_3_SetValue(false);
    </code>

*/
inline static void BTN_AUX_3_SetValue(bool value)
{
  if(value)
  {
    BTN_AUX_3_SetHigh();
  }
  else
  {
    BTN_AUX_3_SetLow();
  }
}

/**
  @Summary
    Toggles the GPIO pin, RD3, using LATD3.

  @Description
    Toggles the GPIO pin, RD3, using LATD3.

  @Preconditions
    The RD3 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RD3
    BTN_AUX_3_Toggle();
    </code>

*/
#define BTN_AUX_3_Toggle()           ( LATDINV = (1 << 3) )
/**
  @Summary
    Reads the value of the GPIO pin, RD3.

  @Description
    Reads the value of the GPIO pin, RD3.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RD3
    postValue = BTN_AUX_3_GetValue();
    </code>

*/
#define BTN_AUX_3_GetValue()         PORTDbits.RD3
/**
  @Summary
    Configures the GPIO pin, RD3, as an input.

  @Description
    Configures the GPIO pin, RD3, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD3 as an input
    BTN_AUX_3_SetDigitalInput();
    </code>

*/
#define BTN_AUX_3_SetDigitalInput()   ( TRISDSET = (1 << 3) )
/**
  @Summary
    Configures the GPIO pin, RD3, as an output.

  @Description
    Configures the GPIO pin, RD3, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD3 as an output
    BTN_AUX_3_SetDigitalOutput();
    </code>

*/
#define BTN_AUX_3_SetDigitalOutput()   ( TRISDCLR = (1 << 3) )
/**
  @Summary
    Sets the GPIO pin, RD4, high using LATD4.

  @Description
    Sets the GPIO pin, RD4, high using LATD4.

  @Preconditions
    The RD4 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD4 high (1)
    BTN_AUX_4_SetHigh();
    </code>

*/
#define BTN_AUX_4_SetHigh()          ( LATDSET = (1 << 4) )
/**
  @Summary
    Sets the GPIO pin, RD4, low using LATD4.

  @Description
    Sets the GPIO pin, RD4, low using LATD4.

  @Preconditions
    The RD4 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD4 low (0)
    BTN_AUX_4_SetLow();
    </code>

*/
#define BTN_AUX_4_SetLow()           ( LATDCLR = (1 << 4) )

/**
  @Summary
    Sets a value to the GPIO pin.

  @Description
    Sets or Resets the GPIO pin, RD4, low or high using LATD4.

  @Preconditions
    The RD4 must be set to an output.

  @Returns
    None.

  @Param
    bool value; : value to be set to the GPIO pin.

  @Example
    <code>
    // Set RD4 to low.
    BTN_AUX_4_SetValue(false);
    </code>

*/
inline static void BTN_AUX_4_SetValue(bool value)
{
  if(value)
  {
    BTN_AUX_4_SetHigh();
  }
  else
  {
    BTN_AUX_4_SetLow();
  }
}

/**
  @Summary
    Toggles the GPIO pin, RD4, using LATD4.

  @Description
    Toggles the GPIO pin, RD4, using LATD4.

  @Preconditions
    The RD4 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RD4
    BTN_AUX_4_Toggle();
    </code>

*/
#define BTN_AUX_4_Toggle()           ( LATDINV = (1 << 4) )
/**
  @Summary
    Reads the value of the GPIO pin, RD4.

  @Description
    Reads the value of the GPIO pin, RD4.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RD4
    postValue = BTN_AUX_4_GetValue();
    </code>

*/
#define BTN_AUX_4_GetValue()         PORTDbits.RD4
/**
  @Summary
    Configures the GPIO pin, RD4, as an input.

  @Description
    Configures the GPIO pin, RD4, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD4 as an input
    BTN_AUX_4_SetDigitalInput();
    </code>

*/
#define BTN_AUX_4_SetDigitalInput()   ( TRISDSET = (1 << 4) )
/**
  @Summary
    Configures the GPIO pin, RD4, as an output.

  @Description
    Configures the GPIO pin, RD4, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD4 as an output
    BTN_AUX_4_SetDigitalOutput();
    </code>

*/
#define BTN_AUX_4_SetDigitalOutput()   ( TRISDCLR = (1 << 4) )

/**
    Section: Function Prototypes
*/
/**
  @Summary
    Configures the pin settings of the PIC32MM0256GPM064
    The peripheral pin select, PPS, configuration is also handled by this manager.

  @Description
    This is the generated manager file for the PIC24 / dsPIC33 / PIC32MM MCUs device.  This manager
    configures the pins direction, initial state, analog setting.
    The peripheral pin select, PPS, configuration is also handled by this manager.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    void SYSTEM_Initialize(void)
    {
        // Other initializers are called from this function
        PIN_MANAGER_Initialize();
    }
    </code>

*/
void PIN_MANAGER_Initialize (void);



#endif
