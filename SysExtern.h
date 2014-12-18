/*
 * File:   SysExtern.h
 * Author: CHACK
 *
 * Created on July 31, 2014, 10:22 AM
 */

#ifndef SYSEXTERN_H
#define	SYSEXTERN_H

#ifdef	__cplusplus
extern "C" {
#endif

    /******************************************************************************
    *                       Main.c extern                                         *
    ******************************************************************************/
    extern UINT32       DataADC;
    extern UINT32       MovAver[ARRAY_SIZE];
    extern UINT32       DataAver;


    /******************************************************************************
    *                       UART.c extern                                         *
    ******************************************************************************/
    extern UINT8        buf[1024];
    extern UINT8        bufTime[40];
    extern int          indexBuf;


    /******************************************************************************
    *                       ADC.c extern                                          *
    ******************************************************************************/
    extern unsigned int MISO; /* Data from SPI buffer */
    extern unsigned int MOSI; /* Data Out to port SPI */


    /******************************************************************************
    *                       RTC_PCF8583.c extern                                  *
    ******************************************************************************/
    extern  UINT8 seconds;      // Global date/time variables
    extern  UINT8 minutes;      // Global date/time variables
    extern  UINT8 hours;        // Global date/time variables
    extern  UINT8 day;          // Global date/time variables
    extern  UINT8 month;        // Global date/time variables
    extern  int   year;         // Global date/time variables
    extern  int   yearBase;     // Global date/time variables
    extern  int   yearBase1;    // Global date/time variables
    extern  char  i2cData[10];
    extern  UINT8 i2cData_write[10];
    extern  unsigned char SlaveAddress;
    extern  unsigned char DataSz;

    /* variable for save value from RX */
    extern  int sec;
    extern  int min;
    extern  int hou;
    extern  int dat;
    extern  int mon;
    extern  int yea;
    extern  int wda;

    /* Clock Alarm */
    extern  int secondAlarm;
    extern  int minuteAlarm;
    extern  int hourAlarm;
    extern  int dayAlarm;
    extern  int monthAlarm;
    extern  int weekdayAlarm;  /* RTC memory location 0Eh (alarm_weekday/month): Bit0=Sunday, ... Bit6=Saturday, Bit7=Not used */


#ifdef	__cplusplus
}
#endif

#endif	/* SYSEXTERN_H */
