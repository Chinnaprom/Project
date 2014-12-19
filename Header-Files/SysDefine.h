/*
 * File:   SysDefine.h
 * Author: CHACK
 *
 * Created on July 31, 2014, 10:22 AM
 */

#ifndef SYSDEFINE_H
#define	SYSDEFINE_H

#ifdef	__cplusplus
extern "C" {
#endif


        /******************************************************************************
        *                       main.c define                                         *
        ******************************************************************************/
        #define SYS_FREQ        (80000000UL)      /* Clock Frequency 80 MHz       */
        #define MAX_COUNTER     (1000)             /* Count Interrupt              */
        #define ARRAY_SIZE      (20)              /* Size fo Array, MovingAverage */


        /******************************************************************************
        *                       UART.c define                                         *
        ******************************************************************************/
        #define GetPeripheralClock()        (SYS_FREQ/(1 << OSCCONbits.PBDIV)) /* CTIS */
        #define GetInstructionClock()       (SYS_FREQ)
        #define UART_MODULE_ID              (UART2)      /* PIM is connected to Explorer through UART2 module */
//        #define DESIRED_BAUDRATE            (250000)     /* The desired BaudRate */
        #define DESIRED_BAUDRATE            (115200)     /* The desired BaudRate */


        /******************************************************************************
        *                       ADC.c define                                          *
        ******************************************************************************/


        /******************************************************************************
        *                       RTC_PCF8583.c define                                  *
        ******************************************************************************/
        #define PBCLK                   (SYS_FREQ/2)
        #define BRG_VAL                 0x168
        #define Nop()                   asm( "nop" )

        #define RTC_ADDR_W              0xA2
        #define RTC_ADDR_R              0x51

        #define SECOUNDS_ADDRESS        0x02
        #define MINUTES_ADDRESS         0x03
        #define HOURS_ADDRESS           0x04
        #define DAY_ADDRESS             0x05
        #define MONTH_ADDRESS           0x06
        #define YEAR_BASE10_ADDRESS     0x10
        #define YEAR_BASE11_ADDRESS     0x11

        #define LOCATION_CONTROL	0x00
        #define LOCATION_ALARM_CONTROL	0x08
        #define LOCATION_ALARM_TIMER	0x0F
        #define LOCATION_BASE_YEAR	0x10	/* Because PCF8583 only hold 4 years -
                                                   : we uses this address (two bytes) in NVRAM to store base year */

        /* Clock alarm modes - Bits 4-5 in RTC Alarm control register */
        #define NO_CLOCK_ALARM		0x00
        #define CLOCK_DAILY_ALARM	0x10
        #define CLOCK_WEEKDAY_ALARM	0x20
        #define CLOCK_DATED_ALARM	0x30

        /* Timer alarm modes - Bits 0-3 in RTC Alarm control register */
        #define NO_TIMER		0x00
        #define TIMER_HUNDRS_OF_A_SEC	0x01
        #define TIMER_SECONDS		0x02
        #define TIMER_MINUTES		0x03
        #define TIMER_HOURS		0x04
        #define TIMER_DAYS		0x05


        /******************************************************************************
        *                       SDMMC.c define                                        *
        ******************************************************************************/
        #define FAIL    FALSE

        // insert LED definitions in your code if desired
        #define READ_LED            _RA2
        #define WRITE_LED           _RA2

        // Init ERROR code definitions
        #define E_COMMAND_ACK       0x80
        #define E_INIT_TIMEOUT      0x81

        #define FPB                 SYS_FREQ

        /*
        * Pin-Assignments according to DS-51583b, DS-51775a
        * GP:  "general purpose chip" -> PIC32MX360F512L
        * USB: "USB On-The-Go"        -> PIC32MX460F512L
        *  log. GP-SPI1/2 USB-SPI1/2
        *  SCK   RF6/RG6  RD10/RG6
        *  SDI   RF7/RG7   RC4/RG7
        *  SDO   RF8/RG8   RD0/RG8
        *  ~CS   RB1/RB9   RB1/RB9
        *  CD    RF0/RG0   RF0/ -
        *  WD    RF1/RG1   RF1/ -
        */

        // I/O definitions (for PIC32MX460F512L !!!)
        #define SDWP        _RF1      // Write Protect input
        #define SDCD        _RF0      // Card Detect input
        #define SDCS        _RB1      // Card Select output
        //+
        #define TRIS_SCK    _TRISD10  // SCK pin direction
        #define TRIS_SDI    _TRISC4   // SDI pin direction
        #define TRIS_SDO    _TRISD0   // SDO pin direction
        #define TRIS_CS     _TRISB1   // ~CS pin direction
        #define TRIS_CD     _TRISF0   //  CD pin direction
        #define TRIS_WD     _TRISF1   //  WD pin direction
        //+
        #define PCFG        (1)       // port configuration : ADC (Port B) digital bits

        // SD card commands
        #define RESET           0 // a.k.a. GO_IDLE (CMD0)
        #define INIT            1 // a.k.a. SEND_OP_COND (CMD1)
        #define READ_SINGLE     17
        #define WRITE_SINGLE    24

        // additional commands (not used)
        #define SEND_CSD        9
        #define SEND_CID        10
        #define SET_BLEN        16
        #define APP_CMD         55
        #define SEND_APP_OP     41

        // SD card responses
        #define DATA_START      0xFE
        #define DATA_ACCEPT     0x05

        // timeouts
        #define I_TIMEOUT       10000
        #define R_TIMEOUT       25000
        #define W_TIMEOUT       250000



        /******************************************************************************
        *                       LCD.c define                                          *
        ******************************************************************************/
        /*  DATA bus tristate register  */
        #define LCD_BusOutput   mPORTESetPinsDigitalOut(BIT_0 | BIT_1 | BIT_2 | BIT_3 |BIT_4 | BIT_5 | BIT_6 | BIT_7)
        /*  Enable and Register select as output */
        #define E_PinOutput     mPORTGSetPinsDigitalOut(BIT_12)
        #define RS_PinOutput    mPORTGSetPinsDigitalOut(BIT_13)

        /*  Enable Pin  */
        #define SetEnablePin    mPORTGSetBits(BIT_12)
        #define ClearEnablePin  mPORTGClearBits(BIT_12)

        /*  Register Select Pin  */
        #define SetRSPin        mPORTGSetBits(BIT_13)
        #define ClearRSPin      mPORTGClearBits(BIT_13)

        /*  DATA BUS FOR LCD 8bits  */
        #define LCD_BUS(_value)     (mPORTEWrite((unsigned char)(_value)))

        #define FirstLine       0x80
        #define SecondLine      0xC0
        #define ThirdLine       0x94
        #define FourthLine      0xD4


#ifdef	__cplusplus
}
#endif

#endif	/* SYSDEFINE_H */

