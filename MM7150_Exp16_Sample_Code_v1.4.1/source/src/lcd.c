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
/** @file  lcd.c
*   Explorer16 LCD functions
*****************************************************************************
*   MM7150 with Explorer 16 Development Board Sample Code system file
*
*   Company : Microchip Technology Inc.
*
*   File name : lcd.c
*
*   Summary : LCD functions module for MM7150 demo code
*
*   Functions : LCD_Initialize
*               LCD_PutString
*               LCD_PutChar
*               LCD_ClearScreen
*               LCD_CarriageReturn
*		LCD_ShiftCursorLeft
*		LCD_ShiftCursorRight
*		LCD_ShiftCursorUp
*		LCD_ShiftCursorDown
*		LCD_Wait
*		LCD_CursorEnable
*
*   Revisions : 0.2 9-30-14 C21674 - changed param name for LCD_Wait() from delay to LCD_delay
*             : 0.1 8-4-14 C21674
*             : 0.0 7-1-14 C16368  - Initial version created
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

#define LCD_F_INSTR         1000                                    // Define a fast instruction execution time in terms of loop time (typically > 43us_
#define LCD_S_INSTR         3000                                    // Define a slow instruction execution time in terms of loop time (typically > 1.35ms)
#define LCD_STARTUP         20000                                   // Define the startup time for the LCD in terms of loop time (typically > 30ms)
#define LCD_MAX_COLUMN      16

#define LCD_SendData(data) { PMADDR = 0x0001; PMDIN1 = data; LCD_Wait(LCD_F_INSTR); }
#define LCD_SendCommand(command, delay) { PMADDR = 0x0000; PMDIN1 = command; LCD_Wait(delay); }
#define LCD_COMMAND_CLEAR_SCREEN        0x01
#define LCD_COMMAND_RETURN_HOME         0x02
#define LCD_COMMAND_ENTER_DATA_MODE     0x06
#define LCD_COMMAND_CURSOR_OFF          0x0C
#define LCD_COMMAND_CURSOR_ON           0x0F
#define LCD_COMMAND_MOVE_CURSOR_LEFT    0x10
#define LCD_COMMAND_MOVE_CURSOR_RIGHT   0x14
#define LCD_COMMAND_SET_MODE_8_BIT      0x38
#define LCD_COMMAND_ROW_0_HOME          0x80
#define LCD_COMMAND_ROW_1_HOME          0xC0

static void LCD_CarriageReturn ( void ) ;
static void LCD_ShiftCursorLeft ( void ) ;
static void LCD_ShiftCursorRight ( void ) ;
static void LCD_ShiftCursorUp ( void ) ;
static void LCD_ShiftCursorDown ( void ) ;
static void LCD_Wait ( UINT32 ) ;

static UINT8 row ;
static UINT8 column ;

//*****************************************************************************
//*****************************************************************************
//  Section : Code
//*****************************************************************************
//*****************************************************************************

/** LCD_Initialize
* @note	Initializes the LCD screen.  Can take several hundred milliseconds.
* @param
* @return TRUE=initialized, FALSE= otherwise
*/
BOOL LCD_Initialize ( void )
{
    
    PMMODE = 0x03ff ;                                               // Enable PMP Module, No Address & Data Muxing,
                                                                    // Enable RdWr Port, Enable Enb Port, No Chip Select,
                                                                    // Select RdWr and Enb signals Active High
    PMCON = 0x8383 ;
                                                                    // Enable A0
    PMAEN = 0x0001 ;

    LCD_Wait ( LCD_STARTUP ) ;
    LCD_Wait ( LCD_STARTUP ) ;

    LCD_SendCommand ( LCD_COMMAND_SET_MODE_8_BIT ,     LCD_F_INSTR + LCD_STARTUP ) ;
    LCD_SendCommand ( LCD_COMMAND_CURSOR_OFF ,         LCD_F_INSTR ) ;
    LCD_SendCommand ( LCD_COMMAND_ENTER_DATA_MODE ,    LCD_S_INSTR ) ;

    LCD_ClearScreen ( ) ;

    return TRUE ;
}

/** LCD_PutString
* @note	Puts a string on the LCD screen, unsupported characters will be discarded. May block or throw away characters is LCD is not ready
*           or buffer space is not available. Will terminate when either a null terminator character (0x00) is reached or the length number
*           of characters is printed, which ever comes first.
* @param inputString string to print
* @param length length of string to print
* @return 
*/
void LCD_PutString ( char* inputString , UINT16 length )
{
    while (length--)
    {
        switch (*inputString)
        {
            case 0x00:
                return ;

            default:
                LCD_PutChar ( *inputString++ ) ;
                break ;
        }
    }
}

/** LCD_PutChar
* @note	Puts a character on the LCD screen. Unsupported characters will be discarded. May block or throw away characters is LCD is not ready
*           or buffer space is not available. 
* @param inputCharacter character to print
* @return
*/
void LCD_PutChar ( char inputCharacter )
{
    switch (inputCharacter)
    {
        case '\r':
            LCD_CarriageReturn ( ) ;
            break ;

        case '\n':
            if (row == 0)
            {
                LCD_ShiftCursorDown ( ) ;
            }
            else
            {
                LCD_ShiftCursorUp ( ) ;
            }
            break ;

        case '\b':
            LCD_ShiftCursorLeft ( ) ;
            LCD_PutChar ( ' ' ) ;
            LCD_ShiftCursorLeft ( ) ;
            break ;

        default:
            LCD_SendData ( inputCharacter ) ;
            column++ ;

            if (column == LCD_MAX_COLUMN)
            {
                column = 0 ;
                if (row == 0)
                {
                    LCD_SendCommand ( LCD_COMMAND_ROW_1_HOME , LCD_S_INSTR ) ;
                    row = 1 ;
                }
                else
                {
                    LCD_SendCommand ( LCD_COMMAND_ROW_0_HOME , LCD_S_INSTR ) ;
                    row = 0 ;
                }
            }
            break ;
    }
}

/** LCD_ClearScreen
* @note	Clears the screen, if possible.
* @param 
* @return
*/
void LCD_ClearScreen ( void )
{
    LCD_SendCommand ( LCD_COMMAND_CLEAR_SCREEN , LCD_S_INSTR ) ;
    LCD_SendCommand ( LCD_COMMAND_RETURN_HOME ,  LCD_S_INSTR ) ;

    row = 0 ;
    column = 0 ;
}


/*******************************************************************/
/*******************************************************************/
/* Private Functions ***********************************************/
/*******************************************************************/
/*******************************************************************/

/** LCD_CarriageReturn
* @note	Handles a carriage return
* @param 
* @return
*/
static void LCD_CarriageReturn ( void )
{
    if (row == 0)
    {
        LCD_SendCommand ( LCD_COMMAND_ROW_0_HOME , LCD_S_INSTR ) ;
    }
    else
    {
        LCD_SendCommand ( LCD_COMMAND_ROW_1_HOME , LCD_S_INSTR ) ;
    }
    column = 0 ;
}

/** LCD_ShiftCursorLeft
* @note	Shifts cursor left one spot (wrapping if required)
* @param 
* @return
*/
static void LCD_ShiftCursorLeft ( void )
{
    UINT8 i ;

    if (column == 0)
    {
        if (row == 0)
        {
            LCD_SendCommand ( LCD_COMMAND_ROW_1_HOME , LCD_S_INSTR ) ;
            row = 1 ;
        }
        else
        {
            LCD_SendCommand ( LCD_COMMAND_ROW_0_HOME , LCD_S_INSTR ) ;
            row = 0 ;
        }

        //Now shift to the end of the row
        for (i = 0 ; i < ( LCD_MAX_COLUMN - 1 ) ; i++)
        {
            LCD_ShiftCursorRight ( ) ;
        }
    }
    else
    {
        column-- ;
        LCD_SendCommand ( LCD_COMMAND_MOVE_CURSOR_LEFT , LCD_F_INSTR ) ;
    }
}

/** LCD_ShiftCursorRight
* @note	Shifts cursor right one spot (wrapping if required)
* @param 
* @return
*/
static void LCD_ShiftCursorRight ( void )
{
    LCD_SendCommand ( LCD_COMMAND_MOVE_CURSOR_RIGHT , LCD_F_INSTR ) ;
    column++ ;

    if (column == LCD_MAX_COLUMN)
    {
        column = 0 ;
        if (row == 0)
        {
            LCD_SendCommand ( LCD_COMMAND_ROW_1_HOME , LCD_S_INSTR ) ;
            row = 1 ;
        }
        else
        {
            LCD_SendCommand ( LCD_COMMAND_ROW_0_HOME , LCD_S_INSTR ) ;
            row = 0 ;
        }
    }
}


/** LCD_ShiftCursorUp
* @note	Shifts cursor up one spot (wrapping if required)
* @param 
* @return
*/
static void LCD_ShiftCursorUp ( void )
{
    UINT8 i ;

    for (i = 0 ; i < LCD_MAX_COLUMN ; i++)
    {
        LCD_ShiftCursorLeft ( ) ;
    }
}

/** LCD_ShiftCursorDown
* @note	Shifts cursor dpwn one spot (wrapping if required)
* @param 
* @return
*/
static void LCD_ShiftCursorDown ( void )
{
    UINT8 i ;

    for (i = 0 ; i < LCD_MAX_COLUMN ; i++)
    {
        LCD_ShiftCursorRight ( ) ;
    }
}


/** LCD_Wait
* @note	wait function that just cycle burns
* @param delay artibrary delay time based on loop counts
* @return
*/
static void LCD_Wait ( UINT32 lcd_delay )
{
    while (lcd_delay)
    {
        lcd_delay-- ;
    }
}


/** LCD_CursorEnable
* @note	Enables/disables the cursor
* @param enable specifies if the cursor should be on or off
* @return
*/
void LCD_CursorEnable ( BOOL enable )
{
    if (enable == TRUE)
    {
        LCD_SendCommand ( LCD_COMMAND_CURSOR_ON , LCD_S_INSTR ) ;
    }
    else
    {
        LCD_SendCommand ( LCD_COMMAND_CURSOR_OFF , LCD_S_INSTR ) ;
    }
}
