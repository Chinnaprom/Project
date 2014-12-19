/*
 * File:   SysFlag.h
 * Author: chack
 *
 * Created on 8 September 2557, 9:05 AM.
 */

#ifndef SYSFLAG_H
#define	SYSFLAG_H

#ifdef	__cplusplus
extern "C" {
#endif

/******************************************************************************
*                       Type Define Struction 				      *
******************************************************************************/
typedef union
{
        unsigned char Byte;      /* Allows us to refer to the flags 'en masse' */
        struct
        {
                unsigned char
                b7 : 1,       /* Explanation of bit7 */
                b6 : 1,       /* Explanation of bit6 */
                b5 : 1,       /* Explanation of bit5 */
                b4 : 1,       /* Explanation of bit4 */
                b3 : 1,       /* Explanation of bit3 */
                b2 : 1,       /* Explanation of bit2 */
                b1 : 1,       /* Explanation of bit1 */
                b0 : 1;       /* Explanation of bit0 */
        } BIT;
} EX_FLAGS;

/******************************************************************************
*                       Variable Flage Define Type 			      *
******************************************************************************/
extern  EX_FLAGS  _data_status_0; /* Allocation for the Flags */

/******************************************************************************
*                       Flage Define Type 				      *
******************************************************************************/
#define	dflag_start_comu_adc    			(_data_status_0.BIT.b7)
#define dflag_acknowledge_i2c                           (_data_status_0.BIT.b6)
#define dflag_acknowledge_i2c_write                     (_data_status_0.BIT.b5)
#define dflag_c                                         (_data_status_0.BIT.b4)
#define dflag_d                                         (_data_status_0.BIT.b3)
#define dflag_e                                         (_data_status_0.BIT.b2)
#define dflag_f                                         (_data_status_0.BIT.b1)
#define dflag_g                                         (_data_status_0.BIT.b0)


#ifdef	__cplusplus
}
#endif

#endif	/* SYSFLAG_H */
