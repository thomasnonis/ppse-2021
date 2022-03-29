/*****************************************************************************
* © 2014 Microchip Technology Inc. and its subsidiaries.
* You may use this software and any derivatives exclusively with
* Microchip products.
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".
* NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
* INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
* AND FITNESS FOR A PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP
* PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.
* TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
* CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF
* FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
* MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE
* OF THESE TERMS.
*****************************************************************************/
/** @file  i2cIO.c
*   I2C functions to I/F PIC24 with SSC7150
*****************************************************************************
*  MM7150 with Explorer 16 Development Board Sample Code i2c i/o file
*
*   Company : Microchip Technology Inc.
*
*   File name : i2cIO.c
*
*   Summary : Module for MM7150 demo code which interfaces 
*           PIC24 (Explorer16 board) to SSC7150 via i2c bus/calls 
*   
*   Functions : i2c_cmd_WrRd
*
*   Revisions : 0.3 10-6-16 A21444 - use MCC, removed gets_I2C1
*             : 0.2 9-18-14 C21674 - enhanced error handling, added timer2 interrupt 
*                                    to exit for unresponsive i2c, gets_I2C1 check status
*                                    bits on exit with no data 
*             : 0.1 8-4-14 C21674
*             : 0.0 7-1-14 C16368  - Initial version created
******************************************************************************/
//****************************************************************************
//****************************************************************************
//  Section : Included files
//****************************************************************************
//****************************************************************************

#include "..\headers\app.h"
extern UINT8 i2c_adjust;
extern volatile UINT16 I2C_TIMEOUT_1MS_CNTR;

//*****************************************************************************
//*****************************************************************************
//  Section : Code
//*****************************************************************************
//*****************************************************************************

/** i2c_cmd_WrRd
* @note	i2c write,read, and combined write/read commands, start timer2 interrupt to exit on unresponsive i2c bus
* @param ucCmd error code
* @param ucBytes_wr Number of bytes to write to slave 
* @param ucData_wr Pointer to data buffer to send to slave 
* @param usBytes_rd Number of bytes to read from slave
* @param ucData_rd Pointer to data buffer from slave
* @param bAdjust Use 1st 2 bytes returned as new length (=TRUE) 
* @return I2C_SUCCESS(=0), I2C_BUF_OVRFLO(=0x22)
*/
UINT8 i2c_cmd_WrRd(UINT8 ucCmd, UINT8 ucBytes_wr,  UINT8 *ucData_wr, UINT16 usBytes_rd,  UINT8 *ucData_rd, BOOL bAdjust)
{
    static I2C1_MESSAGE_STATUS status;
    static I2C1_TRANSACTION_REQUEST_BLOCK readTRB[2];

    if (ucBytes_wr > BUF_150)                                       // sanity check for maximum buffer size
        return I2C_BUF_OVRFLO;                                      // return i2c buffer overflow error code to calling routine

    StartI2CTimer();                                                // start timer2 interrupt in case i2c hangs in 'while loop' functions

    switch(ucCmd)
    {
        case WRITE:
            while(1)
            {
                I2C1_MasterWrite(ucData_wr, ucBytes_wr, SLAVE_ADDR, &status);
                while( status == I2C1_MESSAGE_PENDING && I2C_TIMEOUT_1MS_CNTR < I2_TIMEOUT_PERIOD);
                if( I2C_TIMEOUT_1MS_CNTR >= I2_TIMEOUT_PERIOD )
                    break;
                if( status==I2C1_MESSAGE_ADDRESS_NO_ACK || status==I2C1_DATA_NO_ACK )
                {
                    delay(10);
                    continue;
                }
                break;
            }
            break;

        case READ:
            while(1)
            {
                i2c_adjust = bAdjust;
                I2C1_MasterRead(ucData_rd, usBytes_rd, SLAVE_ADDR, &status);
                while( status == I2C1_MESSAGE_PENDING && I2C_TIMEOUT_1MS_CNTR < I2_TIMEOUT_PERIOD);
                if( I2C_TIMEOUT_1MS_CNTR >= I2_TIMEOUT_PERIOD )
                    break;
                if( status==I2C1_MESSAGE_ADDRESS_NO_ACK || status==I2C1_DATA_NO_ACK )
                {
                    delay(10);
                    continue;
                }
                break;
            }
            break;

        case WR_RD:
            I2C1_MasterWriteTRBBuild(&readTRB[0], ucData_wr, ucBytes_wr, SLAVE_ADDR);
            I2C1_MasterReadTRBBuild(&readTRB[1], ucData_rd, usBytes_rd, SLAVE_ADDR);
            while(1)
            {
                i2c_adjust = bAdjust;
                I2C1_MasterTRBInsert(2, readTRB, &status);
                while( status == I2C1_MESSAGE_PENDING && I2C_TIMEOUT_1MS_CNTR < I2_TIMEOUT_PERIOD);
                if( I2C_TIMEOUT_1MS_CNTR >= I2_TIMEOUT_PERIOD )
                    break;
                if( status==I2C1_MESSAGE_ADDRESS_NO_ACK || status==I2C1_DATA_NO_ACK )
                {
                    delay(10);
                    continue;
                }
                break;
            }
            break;
    }
  
    StopI2CTimer();                                                 //turn off timer2 interrupt 
    switch( status )
    {
        case I2C1_MESSAGE_FAIL:
            i2cIO_error( I2C_ERROR );
        case I2C1_MESSAGE_PENDING:
            i2cIO_error( I2C_TIMEOUT_ERR );
        case I2C1_MESSAGE_COMPLETE:
            break;
        case I2C1_STUCK_START:
            i2cIO_error( I2C_ERROR );
        case I2C1_MESSAGE_ADDRESS_NO_ACK:
        case I2C1_DATA_NO_ACK:
            i2cIO_error( NOT_ACK );
        case I2C1_LOST_STATE:
        default:
            i2cIO_error( I2C_ERROR );
    }
    return I2C_SUCCESS;
}
