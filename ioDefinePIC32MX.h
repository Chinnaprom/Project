/*
 * File:   ioDefinePIC32MX.h
 * Author: CHACK
 *
 * Created on July 31, 2014, 10:21 AM
 */

#ifndef IODEFINEPIC32MX_H
#define	IODEFINEPIC32MX_H

#ifdef	__cplusplus
extern "C" {
#endif


    /******************************************************************************
    *                       Main.c                                                *
    ******************************************************************************/
    /* Set Pin Output, LED */
    #define LED_PinOutPut   mPORTASetPinsDigitalOut(BIT_1 | BIT_2)
    /* Enable Pin, LED */
    #define SetLED1         mPORTASetBits(BIT_1)
    #define ClearLED1       mPORTAClearBits(BIT_1)
    #define ToggleLED1      mPORTAToggleBits(BIT_1)
    #define SetLED2         mPORTASetBits(BIT_2)
    #define ClearLED2       mPORTAClearBits(BIT_2)
    #define ToggleLED2      mPORTAToggleBits(BIT_2)


    /******************************************************************************
    *                       ADC.c                                                 *
    ******************************************************************************/
    /* Set Pin Output, SPI */
    #define CS_PinOutput    mPORTDSetPinsDigitalOut(BIT_1)
    /* Enable Pin, SPI */
    #define SetCSPin        mPORTDSetBits(BIT_1)
    #define ClearCSPin      mPORTDClearBits(BIT_1)


    /******************************************************************************
    *                       RTC.c                                                 *
    ******************************************************************************/
    /* Set Pin Input INT of RTC PCF8583 */
    #define INT_PinInPut    mPORTFSetPinsDigitalIn(BIT_2)
    /*      */
    #define INT_PinInPut    mPORTFReadBits(BIT_2);
    
    
#ifdef	__cplusplus
}
#endif

#endif	/* IODEFINEPIC32MX_H */
