
/*
 * File:   UART.c
 * Author: Jennarong Chinnaprom
 *
 * Created on 31 July 2557, 10:19 AM.
 */

#include <plib.h>
#include "PIC32MX460F512L.h"


/* RX -> pin 49
 * TX -> pin 50 */

/*
*******************************************************************************
*                       UART2_init                                            *
*******************************************************************************
*/
int UART2_init( unsigned int desired_baud )
{
        /* Configure UART module, set buad rate, turn on UART, etc. */
        UARTConfigure(UART_MODULE_ID, UART_ENABLE_PINS_TX_RX_ONLY);
        UARTSetFifoMode(UART_MODULE_ID, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
        UARTSetLineControl(UART_MODULE_ID, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
        UARTSetDataRate(UART_MODULE_ID, GetPeripheralClock(), DESIRED_BAUDRATE);
        UARTEnable(UART_MODULE_ID, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

        /* Configure UART RX Interrupt */
        INTEnable(INT_SOURCE_UART_RX(UART_MODULE_ID), INT_ENABLED);
        INTSetVectorPriority(INT_VECTOR_UART(UART_MODULE_ID), INT_PRIORITY_LEVEL_2);
        INTSetVectorSubPriority(INT_VECTOR_UART(UART_MODULE_ID), INT_SUB_PRIORITY_LEVEL_0);

        /* Enable multi-vector interrupts */
        INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
        INTEnableInterrupts();
        WriteString("SartUART2...\r\n");
}


/*
*******************************************************************************
*                       WriteString                                           *
*******************************************************************************
*/
void WriteString(const char *string)
{
        while(*string != '\0')
        {
                while(!UARTTransmitterIsReady(UART_MODULE_ID));       /* Wait for Ready to send */
                UARTSendDataByte(UART_MODULE_ID, *string);            /* Send data              */
                string++;
                while(!UARTTransmissionHasCompleted(UART_MODULE_ID)); /* Wati for send complete */
        }
}


/*
*******************************************************************************
*                       PutCharacter                                          *
*******************************************************************************
*/
void PutCharacter(const char character)
{
        while(!UARTTransmitterIsReady(UART_MODULE_ID));
        UARTSendDataByte(UART_MODULE_ID, character);
        while(!UARTTransmissionHasCompleted(UART_MODULE_ID));
}
