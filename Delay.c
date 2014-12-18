/*
 * File:   Delay.c
 * Author: __Chack
 *
 * Created on 3 March 2557, 10:57 AM.
 */

#include <plib.h>
#include "PIC32MX460F512L.h"

void __Delay_ms(unsigned long Period_ms)
{
    unsigned long x = 0;

    x = (Period_ms * (SYS_FREQ/2/1000));

    OpenCoreTimer(x);

    while (!mCTGetIntFlag());           /* wait for flag to set */
    mCTClearIntFlag();                  /* clear CoreTimer flag */

    CloseCoreTimer();
}

void __Delay_us(unsigned long Period_us)
{
    unsigned long x = 0;

    x = (Period_us * (SYS_FREQ/2/1000000));

    OpenCoreTimer(x);

    while (!mCTGetIntFlag());           /* wait for flag to set */
    mCTClearIntFlag();                  /* clear CoreTimer flag */

    CloseCoreTimer();
}
