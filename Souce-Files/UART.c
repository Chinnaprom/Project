/*
 * File:   UART.c
 * Author: Jennarong Chinnaprom
 *
 * Created on 31 July 2557, 10:53 AM.
 */

#include <plib.h>
#include "PIC32MX460F512L.h"


/******************************************************************************
 *                       SPI1_init                                            *
 *                                                                            *
 * PIN Connection :                                                           *
 *            ---------------------------------                               *
 *            |     PIC32MX   |   MCP3202     |                               *
 *            ---------------------------------                               *
 *            |       PIN70   |   Clock       |                               *
 *            |       PIN72   |   MOSI        |                               *
 *            |       PIN76   |   CS          |                               *
 *            |        PIN9   |   MISO        |                               *
 *            ---------------------------------                               *
 *                                                                            *
 * Input  : None                                                              *
 *                                                                            *
 * Output : None                                                              *
 *                                                                            *
 * Overflow : set SPI register for confugration                               *
 *****************************************************************************/
int SPI1_init()
{
        IEC0CLR     = 0x03800000; /* disable all interrupts             */
        SPI1CON     = 0;          /* Stops and resets the SPI1.         */
        MISO        = SPI1BUF;    /* clears the receive buffer          */
//        IFS0CLR     = 0x03800000; /* clear any existing event          */
//        IPC5CLR=0x1f000000;       /* clear the priority                */
//        IPC5SET=0x0d000000;       /* Set IPL=3, Subpriority 1          */
        IEC0SET     = 0x03800000; /* Enable Rx, Tx and Error interrupts */
        SPI1BRG     = 500;        /* Use clock frequency = 79.6k        */
        SPI1STATCLR = 0x40;       /* clear the Overflow                 */
        SPI1CON     = 0x8B20;     /* SPI ON, 32-bit transfer, SMP=1, Master mode  0x8A60 */
        return 0;
}/* SPI1_init */


/******************************************************************************
 *                       writeSPI1                                            *
 *                                                                            *
 * Input  : Data Output                                                       *
 *                                                                            *
 * Output : Data Input                                                        *
 *                                                                            *
 * Overflow : writeSPI1, send one byte of data and receive one back at the    *
 *            same time                                                       *
 ******************************************************************************/
int writeSPI1( unsigned int i)
{
        ClearCSPin;                         /* CS = LOW, PIN 76           */
        SPI1BUF = i;                        /* write to buffer for TX     */
        while( !SPI1STATbits.SPIRBF);       /* wait for transfer complete */
        SetCSPin;                           /* CS = HIGH, PIN 76          */
        return SPI1BUF;                     /* read the received value    */
}//writeSPI1


/******************************************************************************
 *                       Function Moving Averrage                             *
 *                                                                            *
 * Input  : New Data                                                          *
 *                                                                            *
 * Output : Data Averrage                                                     *
 *                                                                            *
 * Overflow : Averrage New Data and Old Data                                  *
 ******************************************************************************/
void MovingAverrage(UINT32 Data)
{
        int i;
        int j;
        UINT64 total = 0;
        DataAver = 0;

        for(i=ARRAY_SIZE-1; i>=0; i--) /* Add new data and romve old data */
        {
                if(i > 0)  { MovAver[i] = MovAver[i-1]; }
                if(i == 0) { MovAver[i] = Data;         }
        }

        for(j=0; j<=ARRAY_SIZE-1; j++){ total +=  MovAver[j]; } /* Total Data */

        DataAver = total/ARRAY_SIZE;
}
