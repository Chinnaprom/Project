/*
 * File:   Timer.c
 * Author: Jennarong Chinnaprom
 *
 * Created on 31 July 2557, 02:45 PM.
 */

#include <plib.h>
#include "PIC32MX460F512L.h"

/*
*******************************************************************************
*                       T1_init                                               *
*******************************************************************************
*/
void T2_init(void)
{
        /* Open Timer2 , Prescale 64, 16 bits MODE, 12500=10msec */
        OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_64 | T2_32BIT_MODE_OFF | T2_GATE_OFF | T2_IDLE_STOP, 1250);
        /* set up the timer interrupt with a priority of 3 */
        ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_3 );
}
