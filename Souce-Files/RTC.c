#include <plib.h>
#include "PIC32MX460F512L.h"


/******************************************************************************
*                       Extern Variable                                       *
******************************************************************************/
UINT8 seconds;
UINT8 minutes;
UINT8 hours;
UINT8 day;
UINT8 month;
int year;

int yearBase = 0;
int yearBase1 = 0;

unsigned char SlaveAddress;
char i2cData[10];
UINT8 i2cData_write[10];
unsigned char DataSz;

int secondAlarm;
int minuteAlarm;
int hourAlarm;
int dayAlarm;
int monthAlarm;
int weekdayAlarm;

/* SCL >> PIN66 *
 * SDA >> PIN67 */

/******************************************************************************
 *                       Function Read_RTC                                    *
 *                                                                            *
 * Input  : Address of Regrister                                              *
 *                                                                            *
 * Output : Data at Requirement Address                                       *
 *                                                                            *
 * Overflow : Read Data from Register of RTC [PCF8583]                        *
 ******************************************************************************/
unsigned char Read_RTC( unsigned char registerAddress )
{

        unsigned char data;

        // Now Readback the data from the serial RTC
        SlaveAddress = RTC_ADDR_R;	        //0b01010001 Serial Sleve address
        i2cData[0]   = (SlaveAddress << 1) | 0; //Slave Device Address and WR Command (to write the address)
        i2cData[1]   = registerAddress;	        //Slave location to read (high address byte)
        DataSz = 2;
        int Index    = 0;

        StartI2C1();    //Send the Start Bit
        IdleI2C1();	//Wait to complete

        //send the address to read from the serial eeprom
        dflag_acknowledge_i2c = 1;
        while( DataSz )
        {
                MasterWriteI2C1( i2cData[Index++] );
                IdleI2C1();		//Wait to complete

                DataSz--;

                //ACKSTAT is 0 when slave acknowledge. if 1 then slave has not acknowledge the data.
                if( I2C1STATbits.ACKSTAT )
                {
                        dflag_acknowledge_i2c = 0;
                        break;
                }
        }

        if(dflag_acknowledge_i2c == 1)
        {
                //now send a start sequence again
                RestartI2C1();	//Send the Restart condition

                //wait for this bit to go back to zero
                IdleI2C1();	//Wait to complete

                SlaveAddress = 0x51;	//0b01010000 Serial EEPROM address

                MasterWriteI2C1( (SlaveAddress << 1) | 1 ); //transmit read command
                IdleI2C1();		//Wait to complete

                data = MasterReadI2C1();

                StopI2C1();	//Send the Stop condition
                IdleI2C1();	//Wait to complete
        }
        dflag_acknowledge_i2c = 1;

        return data;
}




/******************************************************************************
 *                       Function Write_RTC                                   *
 *                                                                            *
 * Input  : Address of Regrister, value                                       *
 *                                                                            *
 * Output : None                                                              *
 *                                                                            *
 * Overflow : Read Data from Register of RTC [PCF8583]                        *
 ******************************************************************************/
void Write_RTC( unsigned char registerAddress, unsigned char value)
{
        i2cData_write[0] = RTC_ADDR_W;      // RTC Device Address and WR Command
        i2cData_write[1] = registerAddress; // RTC Location to Program
        i2cData_write[2] = value;           // Data to Program

        DataSz    = 3;
        int Index = 0;

        StartI2C1();	//Send the Start Bit
        IdleI2C1();	//Wait to complete

        while( DataSz )
        {
                MasterWriteI2C1( i2cData_write[Index++] );
                IdleI2C1();		//Wait to complete

                DataSz--;

                //ACKSTAT is 0 when slave acknowledge. if 1 then slave has not acknowledge the data.
                if( I2C1STATbits.ACKSTAT ){ break; }
        }

        StopI2C1();	//Send the Stop condition
        IdleI2C1();	//Wait to complete
}



/******************************************************************************
 *                       Function Read_Time                                   *
 *                                                                            *
 * Input  : None                                                              *
 *                                                                            *
 * Output : None                                                              *
 *                                                                            *
 * Overflow : Read Data from Register of RTC [PCF8583]                        *
 ******************************************************************************/
void Read_Time()
{
        seconds   = Read_RTC( SECOUNDS_ADDRESS );
        minutes   = Read_RTC( MINUTES_ADDRESS );
        hours     = Read_RTC( HOURS_ADDRESS );
        day       = Read_RTC( DAY_ADDRESS );
        month     = Read_RTC( MONTH_ADDRESS );
        yearBase  = Read_RTC( YEAR_BASE10_ADDRESS );
        yearBase1 = Read_RTC( YEAR_BASE11_ADDRESS );
}



/******************************************************************************
 *                       Function Write_RTC                                   *
 *                                                                            *
 * Input  : Address of Regrister                                              *
 *                                                                            *
 * Output : None                                                              *
 *                                                                            *
 * Overflow : Read Data from Register of RTC [PCF8583]                        *
 ******************************************************************************/
void Write_Time( unsigned char registerAddress, int seconds, int minutes, int hours,
       int date, int month, int year, int wday)
{
        UINT8 yearSend = ( (year-48)*10 ) + (year-48) + 2000;
        // Send Data to RTC to program one location
        i2cData_write[0] = RTC_ADDR_W;                                 // RTC Device Address and WR Command
        i2cData_write[1] = registerAddress;                            // RTC Location to Program
        i2cData_write[2] = ( ((seconds / 10) << 4) + (seconds % 10) ); // Data to Program
        i2cData_write[3] = ( ((minutes / 10) << 4) + (minutes % 10) );	              // Data to Program
        i2cData_write[4] = ( ((hours / 10) << 4)   + (hours % 10) );	              // Data to Program
        i2cData_write[5] = ( ((UINT8)(yearSend % 4) << 6) | ( ((date / 10) << 4) + (date % 10) ) ); // Data to Program
        i2cData_write[6] = (( ((month / 10) << 4)  + (month % 10) ) | (wday << 5));   // Data to Program

        DataSz    = 7;
        int Index = 0;

        StartI2C1();	//Send the Start Bit
        IdleI2C1();	//Wait to complete

        while( DataSz )
        {
                MasterWriteI2C1( i2cData_write[Index++] );
                IdleI2C1();		//Wait to complete

                DataSz--;

                //ACKSTAT is 0 when slave acknowledge. if 1 then slave has not acknowledge the data.
                if( I2C1STATbits.ACKSTAT ){ break; }
        }

        StopI2C1();	//Send the Stop condition
        IdleI2C1();	//Wait to complete



        /*******************************************************/
        /******** Write year base address 0x10 and 0x11 ********/
        /*******************************************************/
        int yearBase = year - ( year % 4 );
        // Send Data to RTC to program one location
        i2cData_write[0] = RTC_ADDR_W;              // RTC Device Address and WR Command
        i2cData_write[1] = YEAR_BASE10_ADDRESS;     // RTC Location to Program
        i2cData_write[2] = ( yearBase >> 8 );       // Data to Program
        i2cData_write[3] = ( yearBase & 0x00FF );   // Data to Program

        DataSz = 4;
        Index  = 0;

        StartI2C1();	//Send the Start Bit
        IdleI2C1();	//Wait to complete

        while( DataSz )
        {
                MasterWriteI2C1( i2cData_write[Index++] );
                IdleI2C1();		//Wait to complete

                DataSz--;

                //ACKSTAT is 0 when slave acknowledge. if 1 then slave has not acknowledge the data.
                if( I2C1STATbits.ACKSTAT ){ break; }
        }

        StopI2C1();	//Send the Stop condition
        IdleI2C1();	//Wait to complete
}



void setClockAlarm(UINT8 clockAlarmFunction)
{
        if(clockAlarmFunction == NO_CLOCK_ALARM)
	{
		UINT8 control = Read_RTC(LOCATION_ALARM_CONTROL);
		control &= 0x01001111;				// no interrupt, 'no clock alarm'
		Write_RTC(LOCATION_ALARM_CONTROL, control);

		return;
	}

	UINT8 control = 0b11000100;				// stop counting, don't mask, enable alarm control register, reset the alarm flag
	Write_RTC(LOCATION_CONTROL, control);

	// Set Alarm time - Alarm time/date registers 0x09 - 0x0e
        i2cData_write[0] = RTC_ADDR_W;      // RTC Device Address and WR Command
        i2cData_write[1] = 0x09;            // set the register pointer to (0x09) - milliseconds
        i2cData_write[2] = 0x00;            // set 00 at milliseconds
        i2cData_write[3] = ( ((secondAlarm / 10) << 4) + (secondAlarm % 10) );
        i2cData_write[4] = ( ((minuteAlarm / 10) << 4) + (minuteAlarm % 10) );
        i2cData_write[5] = ( ((minuteAlarm / 10) << 4) + (minuteAlarm % 10) );
        i2cData_write[6] = ( ((dayAlarm / 10) << 4) + (dayAlarm % 10) );
        i2cData_write[7] = ( ((monthAlarm / 10) << 4) + (monthAlarm % 10) );

        DataSz    = 8;
        int Index = 0;

        StartI2C1();	//Send the Start Bit
        IdleI2C1();	//Wait to complete

        while( DataSz )
        {
                // 0x0D - alarm date
                if(Index == 6)
                {
                        if(clockAlarmFunction == CLOCK_DATED_ALARM)
                        {
                                MasterWriteI2C1( i2cData_write[Index++] );
                        }
                        else // for 'daily alarm' or 'weekday alarm' Set 00 at date
                        {
                                MasterWriteI2C1( 0x00 );
                                Index++;
                        }
                }
                // 0x0E - alarm month
                else if(Index == 7 )
                {
                        if(clockAlarmFunction == CLOCK_WEEKDAY_ALARM)
                        {
                                MasterWriteI2C1( weekdayAlarm );
                                Index++;
                        }
                        else if(clockAlarmFunction == CLOCK_DATED_ALARM)
                        {
                                MasterWriteI2C1( i2cData_write[Index++] );
                        }
                }
                else{ MasterWriteI2C1( i2cData_write[Index++] ); }

                IdleI2C1();		//Wait to complete

                DataSz--;

                //ACKSTAT is 0 when slave acknowledge. if 1 then slave has not acknowledge the data.
                if( I2C1STATbits.ACKSTAT ){ break; }
        }
        StopI2C1();	//Send the Stop condition
        IdleI2C1();	//Wait to complete


	UINT8 controlAlarm =  0b10000000;			// enable alarm interrupt
	controlAlarm |= ((clockAlarmFunction & 0b00110000));
        Write_RTC(LOCATION_ALARM_CONTROL, controlAlarm);

	control &= 0b00111111;					// start counting
        Write_RTC(LOCATION_CONTROL, control);
}



void setTimerAlarm(UINT8 timer, UINT8 timerFunction)
{
        if(timerFunction == NO_TIMER)
	{
                UINT8 control = Read_RTC(LOCATION_ALARM_CONTROL);
		control &= 0b01111000;				/* no interrupt, 'no timer' */
                Write_RTC(LOCATION_ALARM_CONTROL, control);
		return;
	}

	UINT8 control = 0b11000100; /* stop counting, don't mask, enable alarm control register, reset the timer flag */
	Write_RTC(LOCATION_CONTROL, control);

	Write_RTC(LOCATION_ALARM_TIMER, ( ((timer / 10) << 4) + (timer % 10) ));
	Write_RTC(0x07, 0x00);                                  /* reset timer counter */

	UINT8 controlAlarm = 0b11001000;			/* set 'timer alarm' on, timer interrupt on */
	controlAlarm |= (timerFunction & 0b00000111);
	Write_RTC(LOCATION_ALARM_CONTROL, controlAlarm);

	control &= 0b00111111;					/* start counting */
        Write_RTC(LOCATION_CONTROL, control);
}




void i2c_wait(unsigned int cnt)
{
	while(--cnt)
	{
		Nop();
		Nop();
	}
}


void TransformTime()
{
        /* show seconds */
        seconds  =  ((seconds & 0xF0) >> 4)*10 + (seconds & 0x0F);  /* Transform seconds */
        minutes  =  ((minutes & 0xF0) >> 4)*10 + (minutes & 0x0F);  /* Transform months  */
        hours    =  ((hours   & 0xF0) >> 4)*10 + (hours   & 0x0F);  /* Transform hours   */
        year     =  (int)((day >> 6) & 0x03);
        day      =  ((day     & 0x30) >> 4)*10 + (day   & 0x0F);    /* Transform day     */
        month    =  ((month   & 0x10) >> 4)*10 + (month & 0x0F);    /* Transform month   */
        /* Transform year */
        yearBase = yearBase << 8;
        yearBase = yearBase | yearBase1;
        year     = year + yearBase;

        /* echo Time and Date */
        sprintf(buf, "T: %02d:%02d:%02d   D: %02d/%02d/%d\r\n", hours, minutes, seconds, day, month, year );
        WriteString(buf); /* Show in Serial Monitor */
}


/* Restart alarm, must be called to re-enable alarm after clock alarm interrupt is handled */
void restartClockAlarm()
{
	UINT8 control = 0b00000100; /* enable alarm control register, reset the alarm and timer flags */
	Write_RTC(LOCATION_CONTROL, control);
}


/* Restart alarm and reset timer counter, must be called to re-enable alarm after timer alarm interrupt is handled */
void restartTimerAlarm()
{
	Write_RTC(0x07, 0x00);	/* reset timer counter */
        restartClockAlarm();
}
