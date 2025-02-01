/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.c

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

#include <stdbool.h>
#include "DB_Debounce.h"


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: File Scope or Global Data                                         */
/* ************************************************************************** */
/* ************************************************************************** */



/* ************************************************************************** */
/* ************************************************************************** */
// Section: Local Functions                                                   */
/* ************************************************************************** */
/* ************************************************************************** */



/* ************************************************************************** */
/* ************************************************************************** */
// Section: Interface Functions                                               */
/* ************************************************************************** */
/* ************************************************************************** */

bool DB_DebounceSignal(bool newSignal, DB_debounce_struct_t * param)
{
    
    if((param->v1==newSignal) && (param->v2==newSignal) && (param->value!=newSignal)) {
        param->value = newSignal;
        param->handled = false;
    }
    param->v2 = param->v1;
    param->v1 = newSignal;
    
    return param->value;

}

void DB_SetSignal(bool newSignal, DB_debounce_struct_t * param)
{
    
    if(param->value!=newSignal) {
        param->value = newSignal;
        param->v2 = newSignal;
        param->v1 = newSignal;
        param->handled = false;
    }

}


/* *****************************************************************************
 End of File
 */
