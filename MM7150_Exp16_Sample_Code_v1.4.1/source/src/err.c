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
/** @file  err.c
*   Error handling for PIC24 with SSC7150
*****************************************************************************
*   MM7150 with Explorer 16 Development Board Sample Code error handling file
*
*   Company : Microchip Technology Inc.
*
*   File name : err.c
*
*   Summary : Module for MM7150 demo code which interfaces 
*           PIC24 (Explorer16 board) to SSC7150 via i2c bus/calls 
*   
*   Functions : error_handler
*               i2cIO_error
*
*   Revisions : 0.2 10-6-16 A21444 - use MCC
*             : 0.1 7-9-15 C21674 - modified error_handler to check for more codes requiring POR of system
*             : 0.0 9-18-14 C21674 - initial version, enhanced error handling, added timer2 interrupt to exit for unresponsive i2c 
******************************************************************************/
//****************************************************************************
//****************************************************************************
//  Section : Included files
//****************************************************************************
//****************************************************************************

#include "..\headers\app.h"

//*****************************************************************************
//*****************************************************************************
//  Section : File scope variables and functions
//*****************************************************************************
//*****************************************************************************

extern volatile UINT16 I2C_TIMEOUT_1MS_CNTR;
extern volatile UINT8 PB_EVT;
extern BOOL bWakeExpected;

//*****************************************************************************
//*****************************************************************************
//  Section : Code
//*****************************************************************************
//*****************************************************************************

/** error_handler
* @note	Display error string LCD and to terminal emulator connected to UART2 on Explorer 16 board
* @param str function name string
* @param errval error value (in hex) returned to failing function
* @return 
*/
void error_handler( char *str, UINT8 offset, UINT8 errval)
{
    char err_buf[64];
    UINT16 wait_cnt;

    //output to uart
    sprintf ( err_buf, "***Error in function: %s:%02X errval=0x%X*** \n\r", str, offset, errval);
    u2out(err_buf);

        
    //output to LCD screen
    LCD_ClearScreen ( ) ;

    sprintf ( err_buf, "%s:%02X err=0x%X", str, offset, errval);
    LCD_PutString(err_buf, 16);

    if ((errval <= POWER_ON_FAIL) || (((errval & 0xF0) > 0x10)&&(errval & 0xF0) < 0x40) )
    {  
        LCD_PutString("POR Exp16 Board ", 16);
        u2out("    Exp16 Board with MM7150-PICTail requires PowerOnReset\n\r");
        while (1);                                                  //hang here
    }    

    LCD_PutString("Push S5 to cont ", 16);
    u2out("    Press button S5 to continue...\n\r");

    wait_cnt = 0;
    while(1)
    {
        if (SW_S5_LOW)                                              //wait for user to press S5 (which has no interrupt associated with it) to acknowledge the error and continue
        {
            PB_EVT = 0xFF;                                          //cancel the button push
            PORTA &= ~LED_D03;                                      //turn error LED D3 off
            LCD_ClearScreen ( ) ;
            break;
        }

        //blink LED D3 as error indicator
        delay(DELAY100);                                            // Delay 100 msec
        wait_cnt++;
        if (wait_cnt == 5)
            PORTA |= LED_D03;                                       // turn on LED D3 to indicate error
        if (wait_cnt > 10)
        {
            PORTA &= ~LED_D03;                                      // Blink LED D3 to indicate error
            wait_cnt = 0;
        }
    }

}



/** i2cIO_error
* @note	error return function, stops i2c, does not return 
* @param ucCode error code
* @return 
*/
void i2cIO_error (UINT8 ucCode)
{
    char err_buf[64];


    StopI2CTimer();                                                 //turn off timer2 interrupt 

    sprintf(err_buf, "    ***i2cIO_error code = 0x%X***\n\r", ucCode);
    u2out(err_buf);
  
    //output to LCD screen
    LCD_ClearScreen ( );

    sprintf ( err_buf, "i2c error=0x%02X  ", ucCode);
    LCD_PutString(err_buf, 16);

    LCD_PutString("POR Exp16 Board ", 16);
    u2out("    Explorer16 Board with MM7150-PICTail requires PowerOnReset\n\r");
    
    if (bWakeExpected)
    {
		u2out("\n\r**NOTE: Wake signal expected, check that HOST_TO_SH_WAKE signal connected**\n\r"); 
    }

    while (1);                                                      //hang here
}   

/** StartI2CTimer
* @note	Start Timer2 interrupts, used to interrupt i2c bus that is non-responsive
* @return 
*/
void StartI2CTimer()
{
    I2C_TIMEOUT_1MS_CNTR = 0;                                       //reset the timeout timer count
    TMR2_Start();
}

/** StopI2CTimer
* @note	Stop Timer2 and associated interrupt
* @return 
*/
void StopI2CTimer()
{
    TMR2_Stop();
}
