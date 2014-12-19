/*
 * File:   Main.c
 * Author: Jennarong Chinnaprom
 *
 * Created on July 28, 2014, 2:18 PM
 */

/******************************************************************************
*                       Include File                                          *
******************************************************************************/
#include <p32xxxx.h>
#include <plib.h>            /* Peripheral Library */
#include "PIC32MX460F512L.h" /* Link file          */

#pragma config FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FPLLODIV = DIV_1, FWDTEN = OFF
#pragma config POSCMOD = HS, FNOSC = PRIPLL, FPBDIV = DIV_1
#pragma config UPLLEN = ON, UPLLIDIV = DIV_2
#pragma config DEBUG = OFF, ICESEL = ICS_PGx2, CP = OFF, BWP = OFF, PWP = OFF
#pragma FWDTEN = OFF, FCKSM = CSDCMD


/******************************************************************************
*                       Bits Field Variable				      *
******************************************************************************/
EX_FLAGS  _data_status_0;


/******************************************************************************
*                       Global Variable                                       *
******************************************************************************/
UINT32   counter   = MAX_COUNTER;        /* Count 100 msec */
float    VolteImag = 0.0;


/******************************************************************************
*                       Extern Variable                                       *
******************************************************************************/
/*          ADC                 */
unsigned int MISO            = 0x00000000; /* Data from SPI buffer */
unsigned int MOSI            = 0xD0000000; /* Data Out to port SPI */
UINT32       DataADC         = 0x0;
float        Volte           = 0.0;
UINT32       MovAver[ARRAY_SIZE];
UINT32       DataAver;
/*          UART                */
UINT8        buf[1024];      /* buffer for store string for send to echo  */
UINT8        bufTime[40];    /* buffer for Read command set Time and Data */
int          indexBuf;          /* index of bufTime                          */
/*          RTC                 */
UINT8 input = 0; /* input pin int from RTC Module */
int   sec;
int   min;
int   hou;
int   dat;
int   mon;
int   yea;
int   wda;


/******************************************************************************
*                       Main Function                                         *
******************************************************************************/
int main()
{
        System_init();

        setTimerAlarm(10, TIMER_SECONDS);   //set select Mode, and set time

        SetLED2;

        MoveCursorToPosition(FirstLine);
        WriteString_LCD("PIC32MX460512L");
        MoveCursorToPosition(SecondLine);
        WriteString_LCD("Project....");
        
        WriteString("...");
        
        while(1)
        {
                input = INT_PinInPut;
                /* Clock Alarm, every 10 sec */
                if( !input )
                {
                        restartTimerAlarm();
                        ToggleLED2;
//                        MISO      = writeSPI1( MOSI );          /* Transmit             */
//                        DataADC   = MISO ;
//                        DataADC   = DataADC >> 15;              /* Shif 15-bit          */
//                        DataADC   = DataADC & (0b111111111111); /* Select bit0 to bit11 */
//                        MovingAverrage(DataADC);
//                        Volte     = ( ((float)DataAver*5.0)/4095.0 )*1000.0; /* Convert 12-bit to Volte */
//                        VolteImag = Volte;
//                        Volte     = Volte - 1985.35;
//                        sprintf(buf, "%d, %.2f mV, %.2f mV\r\n", DataADC, VolteImag, Volte);
//                        WriteString(buf); /* Show in Serial Monitor */
                }
                
                /* read value from ADC MCP3202 every 1 sec */
                if( dflag_start_comu_adc )
                {
                        dflag_start_comu_adc = 0;
                        Read_Time();
                        TransformTime();
                        ClearScreen_LCD(); //hours, minutes, seconds, day, month, year
                        MoveCursorToPosition(FirstLine);
                        sprintf(buf, "T: %02d:%02d:%02d", hours, minutes, seconds);
                        WriteString_LCD(buf);
                        MoveCursorToPosition(SecondLine);
                        sprintf(buf, "D: %02d/%02d/%d", day, month, year );
                        WriteString_LCD(buf);
                }/* d flag */

        }/* while Loop */
        return 0;
}



/******************************************************************************
*                       Interrupt Timer2 : 3msec                              *
******************************************************************************/
void __ISR(_TIMER_2_VECTOR, IPL3) Timer2Handler(void) /* Interrupt Timer2 : 1 msec */
{
        /* clear the interrupt flag   */
        mT2ClearIntFlag();

        /* if-else, counter: 100 msec */
        if( counter == 0 )
        {
                dflag_start_comu_adc = 1;
                counter = MAX_COUNTER;       /* Set new Value to counter variable */
        }
        else{ counter--; }
}


/******************************************************************************
*                       UART 2 interrupt handler, set at priority level 2     *
******************************************************************************/
void __ISR(_UART2_VECTOR, ipl2) IntUart2Handler(void)
{
        /* Is this an RX interrupt? */
        if(INTGetFlag(INT_SOURCE_UART_RX(UART_MODULE_ID)))
        {
                /* Clear the RX interrupt Flag             */
                INTClearFlag(INT_SOURCE_UART_RX(UART_MODULE_ID));
                /* Echo what we just received. WriteString */
                char text = UARTGetDataByte(UART_MODULE_ID);
                /* Chack Text */
                if(text == '{')
                {
                        indexBuf = 0;
                }
                else if(text == '}' && indexBuf == 21) // {00,04,11,17,11,2014,1}
                {
                        /* echo index */
                        sprintf(buf, "%d", indexBuf);
                        WriteString(buf);
                        /* char to int */
                        sec = ((bufTime[0] - '0')*10)  + (bufTime[1] - '0');
                        min = ((bufTime[3] - '0')*10)  + (bufTime[4] - '0');
                        hou = ((bufTime[6] - '0')*10)  + (bufTime[7] - '0');
                        dat = ((bufTime[9] - '0')*10)  + (bufTime[10] - '0');
                        mon = ((bufTime[12] - '0')*10) + (bufTime[13] - '0');
                        yea = ((bufTime[15] - '0')*1000) + ((bufTime[16] - '0')*100) + ((bufTime[17] - '0')*10) + ((bufTime[18] - '0')*1);
                        wda = (bufTime[20] - '0');
                        /* Write(writeAddress, secounds, minutes, houre, date, month, year, wday)*/
                        Write_Time(0x02, sec, min, hou, dat, mon, yea, wda);
                }
                else if(text != '{' && text != '}' && indexBuf < 21)
                {
                        bufTime[indexBuf++] = text;
                }
                /* Toggle LED to indicate UART activity    */
                mPORTAToggleBits(BIT_2);
        }

        /* We don't care about TX interrupt */
        if ( INTGetFlag(INT_SOURCE_UART_TX(UART_MODULE_ID)) )
        {
                INTClearFlag(INT_SOURCE_UART_TX(UART_MODULE_ID));
        }
}


/******************************************************************************
*                       Local Functions                                       *
******************************************************************************/
void System_init(void)
{
        /* Configure cache, wait states and peripheral bus clock                    */
        /* Configure the device for maximum performance but do not change the PBDIV */
        /* Given the options, this function will change the flash wait states, RAM  */
        /* wait state and enable prefetch cache but will not change the PBDIV.      */
        /* The PBDIV value is already set via the pragma FPBDIV option above...     */
        SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

        /* Explorer-16 LEDs are on lower 8-bits of PORTA and to use all LEDs, JTAG port must be disabled. */
        mJTAGPortEnable(DEBUG_JTAGPORT_OFF); //Disable JTAG Port

        /* Enable multi-vector interrupts */
        INTEnableSystemMultiVectoredInt();

        /* Configuration, Timer2, SPI1 and UART2 */
        T2_init();
        SPI1_init();
        UART2_init( DESIRED_BAUDRATE );
        
        /* set pin output */
        LED_PinOutPut;
        CS_PinOutput;
        INT_PinInPut;
        
        /* set initial value in Array */
        int i;
        for(i=0; i<ARRAY_SIZE-1; i++){ MovAver[i] = 0; }

        /* Opent I2C1 */
        OpenI2C1( I2C_EN|I2C_ACK|I2C_ACK_EN, BRG_VAL );
        
        /* LCD */
        Initialize_LCD();
}
