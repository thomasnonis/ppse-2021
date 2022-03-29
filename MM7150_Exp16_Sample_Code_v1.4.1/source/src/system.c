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
/** @file  system.c
*   Explorer16 + PIC24 board level functions 
*****************************************************************************
*   MM7150 with Explorer 16 Development Board Sample Code system file
*
*   Company : Microchip Technology Inc.
*
*   File name : system.c
*
*   Summary : Explorer 16 Development board with PIC24FJ128GA010 processor 
*           peripheral modules initialization and setup for MM7150 demo code 
*           for LEDs, LCD, UART2, push-buttons
*
*   Functions : sys_init
*               LED_init
*               UART2_init
*               Wake_signal
*               u2out
*               isPressed
*               lcd_put_data
*               uart_put_data
*               delay
*
*   Revisions : 0.5 10-6-16 A21444 - use MCC, removed Wake_init and buttons_init, added delay
*             : 0.4 7-9-15 C21674  - modified uart_put_data to accept CR-LF flag
*             : 0.3 2-04-15 C21674 - wake_init & wake_signal added
*                                  - UART2 speed change to 19200
*             : 0.2 9-30-14 C21674 - Openi2c1 and UART2_init global params added
*             : 0.1 8-4-14 C21674  - 
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

volatile BOOL TIMER_1MS_FLG;                                        // 1 msec timer interrupt flag
volatile UINT16 I2C_TIMEOUT_1MS_CNTR;                               // 1 msec timer2 interrupt counter
volatile BOOL EC_DATA_AVAIL = FALSE;                                // EC data available for read flag
volatile UINT8 PB_EVT = 0xFF;                                       // Push Button press event
volatile BOOL DEBOUNCE = FALSE;                                     // Debounce flag (initially set to FALSE, ie. NOT currently debouncing a switch closure)
volatile BOOL bFF_INT = FALSE;                                // EC data available for read flag

BOOL process_packet = FALSE;
UINT32 POR_TIMER;

//****************************************************************************
//****************************************************************************
//  Section : Code 
//****************************************************************************
//****************************************************************************

/** sys_init
* @note	initializes LEDs, push-buttons, LCD, i2c, and UART communication
* @param 
* @return 
*/
void sys_init()
{
    POR_TIMER = 0;					//reset POR timer

#ifdef DEBUG_MODE
	debug_toggle_init();
#endif

    //SYSTEM_Initialize();

    LED_init();

#ifdef UART2_EN
    UART2_init();
#endif

    LCD_Initialize();
}

#ifdef DEBUG_MODE
void debug_toggle_init()
{
	LATBbits.LATB0 = 1;
	TRISBbits.TRISB0=0;												//config PortB sig 0 (RB0) as an output
	asm volatile("nop"); 
}

void debug_toggle(int state)
{
	LATBbits.LATB0 = state;											//toggle RB0 as a scope trigger for debugging timing events

}
#endif

/** Wake_signal
* @note	assert wake signal on RE9, wait 1 ms, deassert
* @param 
* @return 
*/ 
void Wake_signal()
{

    WAKE_SetLow();                                                  //assert wake signal
    delay(2);                                                       //spec says 3µs assertion, let's use ms delay and wait ~2 ms
    WAKE_SetHigh();                                                 //de-assert wake signal

}

/** LED_init
* @note	config LED pins as outputs, turn on LEDs, dance display
* @param 
* @return 
*/ 
void LED_init()
{
    LED_D9_SetHigh();
    delay(DELAY100);
    LED_D9_SetLow();
    LED_D8_SetHigh();
    delay(DELAY100);
    LED_D8_SetLow();
    LED_D7_SetHigh();
    delay(DELAY100);
    LED_D7_SetLow();
    LED_D6_SetHigh();
    delay(DELAY100);
    LED_D6_SetLow();
    LED_D5_SetHigh();
    delay(DELAY100);
    LED_D5_SetLow();
    LED_D4_SetHigh();
    delay(DELAY100);
    LED_D4_SetLow();
    LED_D3_SetHigh();
    delay(DELAY100);
    LED_D3_SetLow();
    delay(DELAY100);

    LED_D9_SetHigh();
    LED_D8_SetHigh();
    LED_D7_SetHigh();
    LED_D6_SetHigh();
    LED_D5_SetHigh();
    LED_D4_SetHigh();
    LED_D3_SetHigh();
    delay(DELAY250);
    LED_D9_SetLow();
    LED_D8_SetLow();
    LED_D7_SetLow();
    LED_D6_SetLow();
    LED_D5_SetLow();
    LED_D4_SetLow();
    LED_D3_SetLow();
    return;
}


/** UART2_Init
* @note	UART2 Initialization function
* @param
* @return 
*/
void UART2_init (void)
{
    UINT8 i;
    char clear_line[] = "\n\r\0";

    for (i = 0; i < 25; i++)                                       //clear UART2 screen
       u2out(clear_line);
}


/** u2out
* @note	Output string to UART2
* @param cStr input string 
* @return 
*/
void u2out (char *cStr)
{
#ifdef UART2_EN 
    unsigned int len, num;

    len = strlen(cStr);
    while(len)
    {
        num = UART2_WriteBuffer ( (uint8_t *)cStr, len );
        len -= num;
        cStr += num;
    }
#endif
}


/** isPressed
* @note	Push buttons debounced (if interrupt driven), PB_EVT Global set = 0xFF 
* @param 
* @return button value or fail (=0xFF)
*/ 
UINT8 isPressed (void)
{

    if (PB_EVT != 0xFF)                                             // button pressed, check the global button press flag value
    {
        switch(PB_EVT)
        {
            case _SW_S3_UP:
                delay(DBNCE_VAL);                                   // 220 msec debouncing delay (interrupt blackout period)
                DEBOUNCE = FALSE;                                   // done debouncing switch closure (allow interrupts to occur once again)
                PB_EVT = 0xFF;                                      // reset the button event flag
                return _SW_S3_UP;                                   // return UP button value
                
            case _SW_S6_DWN:
                delay(DBNCE_VAL);                                   // 220 msec debouncing delay (interrupt blackout period)
                DEBOUNCE = FALSE;                                   // done debouncing switch closure (allow interrupts to occur once again)
                PB_EVT = 0xFF;                                      // reset the button event flag
                return _SW_S6_DWN;                                  // return DOWN button value
            
            case _SW_S5_SLCT:                                       // push-button S5 does not have interrupt associated with it
                PB_EVT = 0xFF;                                      // reset the button event flag
                return _SW_S5_SLCT;                                 // return SELECT button value

            case _SW_S4_RST:
                delay(DBNCE_VAL);                                   // 220 msec debouncing delay (interrupt blackout period)
                DEBOUNCE = FALSE;                                   // done debouncing switch closure (allow interrupts to occur once again)
                PB_EVT = 0xFF;                                      // reset the button event flag
                u2out("\n\r");
                return _SW_S4_RST;                                  //return RESET button value
        }
    }

    return 0xFF;
}


/** lcd_put_data
* @note	Display data to LCD
* @param dDx new X data
* @param dDy new Y data
* @param dDz new Z data
* @param dDw new W data
* @param ucCount number of data elements 
* @param dMult scaling value
* @return 
*/ 
void lcd_put_data(double dDx, double dDy, double dDz, double dDw, UINT8 ucCount, double dMult)
{
    char str[40];

    // The 2 cases below implement different formatting to allow the LCD to display neatly
    if (dMult == .1)
    {
        if (ucCount == 1) 
            sprintf(str, "     %.1f", dDx);                         // Create a string that includes the new data
        if (ucCount == 3) 
            sprintf(str, "X% .1f   Y% .1f       Z% .1f", dDx, dDy, dDz);
        if (ucCount == 4) 
           sprintf(str, "X% .1f Y% .1f Z% .1f W% .1f", dDx, dDy, dDz, dDw);
    }

    else if (dMult == .01 || dMult == .001)
    {
        if(ucCount == 1) 
            sprintf(str, "% .2f", dDx);                             // Create a string that includes the new data
        if(ucCount == 3) 
            sprintf(str, "X% .2f  Y% .2f      Z% .2f", dDx, dDy, dDz);
        if(ucCount == 4) 
            sprintf(str, "X% .2f  Y% .2f  Z% .2f W% .2f", dDx, dDy, dDz, dDw);
   }

    else            
    {
        if (ucCount == 1)
            sprintf(str, "      %.0f", dDx);                         // Create a string that includes the new data
        if (ucCount == 3)
            sprintf(str, "X% 05.0f  Y% 05.0f      Z% 05.0f", dDx, dDy, dDz);
        if (ucCount == 4)
            sprintf(str, "X% .0f Y% .0f Z% .0f W% .0f", dDx, dDy, dDz, dDw);
    }
    
    LCD_ClearScreen();                                              // Clear LCD before setting output
    LCD_PutString(str, sizeof(str));                                // Set data string to the LCD screen

}

/** uart_put_data
* @note	Display data to terminal emulator connected to UART2 on Explorer 16 board
* @param dDx new X data
* @param dDy new Y data
* @param dDz new Z data
* @param dDw new W data
* @param ucCount number of data elements 
* @param CRLF output a carriage-return-line-feed (TRUE/FALSE)
* @return 
*/ 
void uart_put_data(double dDx, double dDy, double dDz, double dDw, UINT8 ucCount, BOOL CRLF)
{
    char str[60];

    if (ucCount == 1) 
		sprintf(str, "% 3.3f%s", dDx, CRLF ? "\n\r" : " ");         // Create a string that includes the new data
    
    if (ucCount == 3) 
      sprintf(str, "X:% 4.3f Y:% 4.3f Z:% 4.3f%s", dDx, dDy, dDz, CRLF ? "\n\r" : " ");

    if (ucCount == 4)
        sprintf(str, "X:% 4.3f Y:% 4.3f Z:% 4.3f W: %4.3f%s", dDx, dDy, dDz, dDw, CRLF ? "\n\r" : " ");
    
    u2out(str);                                                     //output to UART2

}

/** get_S5_keypress
* @note	Wait forever for S5 push-button press before returning 
* @return 
*/ 
void get_S5_keypress()
{
	UINT8 key_id = 0;


	while(1)
	{
		if (SW_S5_LOW)
			PB_EVT = _SW_S5_SLCT;                               // If the select button (which has no interrupt) is brought low (pressed)
		key_id = isPressed(); 
		if (key_id == _SW_S5_SLCT)
			break;
	}
}

/** delay
* @note	Millisecond delay function based on Timer1 (1ms interrupts)
* @param usMs Number of milliseconds to wait
* @return
*/
void delay(UINT16 usMs)
{
    TMR1_Start();
    
    while (usMs)
    {
        if (TMR1_GetElapsedThenClear())
            usMs--;
    }
    return;
}
