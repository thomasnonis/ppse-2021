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
/** @file  mmain.c
*****************************************************************************
*   MM7150 with Explorer 16 Development Board Demo and Sample Code main file
*
*   Company : Microchip Technology Inc.
*
*   File name : mmain.c
*
*   Summary : Main module for MM7150 Sample code with system init
*           calls, main menu, and sensor data output
*
*   Functions : main
*               display_menu
*
*   Revisions : 0.11 27-11-17 A21444 - ver 1.4.1 
*             : 0.10 10-6-16 A21444 - ver 1.4.0 - use MCC, added flash corruption recovery
*             : 0.9 9-29-15 C21674 - ver 1.3.5
*             : 0.8 7-9-15 C21674  - ver 1.3.4
*                                  - added free-fall demo 
*                                  - added MM7150 F/W build display
*                                  - one-time calibration mode (saved to MM7150 Flash) 
*                                  - bug fixes (HID_INT stuck low, Flash Config non-binary file hang)
*             : 0.7 5-02-15 C21674 - added SSC7150 FW Configuration update mode
*                                  -  added Calibration mode
*                                  -  modified DataRate for all sensors = 10 (max rate = 10 ms) 
*             : 0.6 3-16-15 C21674 - customer discovered issue with COMPASS (incorrect readings). Changed sensitivity 1->0 to fix
*             : 0.5 2-04-15 C21674 - ver change 1.3.2 adds support for modified VREG registers
*                                     (& subsequent remapping of existing VREGs)
*                                   - Flash Update support added
*                                   - modified menues for flash update support
*                                   - sleep/wake support
*             : 0.4 9-18-14 C21674  - ver change 1.3.1 adds support for error recovery/display, 
*                                     disable each sensor by putting into LOW POWER mode on exit, added
*                                     support for multi-threaded approach to VREGs
*             : 0.3 8-26-14 C21674  - ver change 1.3.0 adds support for SSC7150 F/W changes (removed backward compatibility)
*             : 0.2 8-26-14 C21674  - ver change 1.2.1 adds backward compatibility support for SSC7150 F/W changes
*             : 0.1 8-04-14 C21674  - aesthetic changes, debouncing improved
*             : 0.0 7-01-14 C16368  - Initial version created
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
extern UINT32 POR_TIMER;
extern volatile BOOL bFF_INT;
extern volatile UINT8 PB_EVT;                                       // push button event
extern UINT16 seq_number;
extern BOOL process_packet;                                         // UART packet processing pending flag
extern BOOL img_wr_done;
UINT8 FLASH_BUF[BUF_150] = {0};

double prev_dat[4];                                                 // Used to hold previous data readings to preclude redundant data output
char COM_title_buf[] = "MICROCHIP Motion Module Demo: MM7150 + Explorer 16 Board v1.4.1\n\r\0";
UINT8 display_menu(void);                                           // Displays and handles menu selections
BOOL bWakeExpected = FALSE;


/** mmain
* @note	First executed, initializes system, loops forever on menu, error exit only 
* @param 
* @return error status
*/ 
int mmain(void) 
{
    UINT8 ucRetStat;                                                // return status value
    UINT8 ucSens_slct;                                              // sensor device selected via menu
    UINT16 wdata;                                                   // pre-exponent-scaled data
    double dDataX = 0, dDataY = 0, dDataZ = 0, dDataW = 0;          // Data field variables
    double dMult = 1;                                               // Multiplier for data fields : sets them to appropriate order of magnitude
    UINT16 wRMWdata;                                                // temporary data for Read-Modify-Write VREG data
    char u2out_buf[32];
    BOOL bFlshHdr = FALSE;
    UINT16 FF_Flag;
    BOOL bIn_FF_flag = FALSE;
    UINT16 wDataX, wDataY, wDataZ, wTmpX, wTmpY, wTmpZ;
    UINT16 wFFPos, wPosImpact;
    BOOL bFFall_set = FALSE;
    BOOL bImpact_set = FALSE;
    BOOL bS4_Pressed = FALSE;
    UINT16 i;
    UINT16 m,n;
    UINT8 *pbuf;
                            
    #define MAX_FF_ARRAY    40
    struct FreeFall													// parameters for individual sensors
    {
       double dFF_X;
       double dFF_Y;
       double dFF_Z;
       UINT16 usMs_time_stamp;
       UINT8 ucFFstate;												//0 = not FreeFall, 1=FF but previous not FF, 2= FF and previous FF
       BOOL bLinearFreeFall;
       BOOL bImpact;
    } FF_ENTRY[MAX_FF_ARRAY];


    sys_init();                                                     // initialize Explorer 16 environment (LEDs, LCDs, push buttons, UART2)     
    u2out(COM_title_buf);                                           // output title string to UART2
    
    ucRetStat = VREG_init();                                        // initialize VREG functions (NOTE: if the SF board does not initialize, end the program)
    switch( ucRetStat )
    {
        case SUCCESS:
            break;
        case REP_PARS_FAIL:
            if ( !flash_info_command(FLASH_BUF, FLASH_UPDATE_BLDR_INFO_CMD) && //check that the SSC7150 is indeed in boot loader mode
                 (FLASH_BUF[5] == FLASH_HDR_VAL) && (FLASH_BUF[4] && FLASH_HDR_VAL) && (FLASH_BUF[3] == FLASH_HDR_VAL) )
            {
                goto DO_FLASH_RECOVERY;
            }
        default:
            error_handler("Vini",0,ucRetStat);                      // error display
    }
	
    if ( (ucRetStat = display_FW_buildnum()) )						//read/display (to UART port) SSC7150 Firmware Build revision 
        error_handler("misc",0, ucRetStat);
   

    while(1)                                                        // main demo loop begins here (does NOT exit)
    {
        bWakeExpected = FALSE;

        switch (ucSens_slct = display_menu())                       // display the menu and operate on its return value
        {   
            case ACCEL_MENU_SLCT:                                   // Accelerometer has been selected

                u2out("Accelerometer has been selected.\n\r");
                u2out("Displaying accelerometer data:\n\r");

                // Enable the Accelerometer
                // To support multi-threaded applications, Read-Modify-Write the VREG:
                if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_SHC, &wRMWdata)) == SUCCESS )  // Read data from the Sensor Hub Configuration register
                {
                    if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_SHC, wRMWdata | VREG_SHC_ACC_EN)) ) // Enable the accelerometer (ACC_EN)
                        error_handler("VRWr",VREG_SHC, ucRetStat);  // error display
                }
                else
                    error_handler("VRRd",VREG_SHC,ucRetStat);       // error display

               if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_ACSEN, 1)) ) // Change Accelerometer sensitivity
                    error_handler("VRWr",VREG_ACSEN,ucRetStat);     // error display

                if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_ACDXR, MAX_DATA_RATE)) ) // Change acc data rate
                    error_handler("VRWr",VREG_ACDXR,ucRetStat);     // error display

                dMult = ret_exponent(ACCEL_VREG_OFFSET);            // Get unit exponent for this sensor's data
                
                while(1)
                {                                                   // get acc readings
                    if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_ACXD, &wdata)) )  // Read data from the accelerometer datax register
                        error_handler("VRRd",VREG_ACXD,ucRetStat);  // error display

                    dDataX = (int)wdata * dMult;                    // Multiply this by the exponent scaler to get floating point value
 
                    if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_ACYD, &wdata)) )  // Read data from the accelerometer datay register
                        error_handler("VRRd",VREG_ACYD,ucRetStat);  // error display

                    dDataY = (int)wdata * dMult;                    // Multiply this by the exponent scaler to get floating point value

                    if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_ACZD, &wdata)) )  // Read data from the accelerometer dataz register
                        error_handler("VRRd",VREG_ACZD,ucRetStat);  // error display

                    dDataZ = (int)wdata * dMult;                    // Multiply this by the exponent scaler to get floating point value
                    
                    if (prev_dat[0] != dDataX || prev_dat[1] != dDataY || prev_dat[2] != dDataZ) // Ensure that this data has not yet been displayed
                    {                       
                        prev_dat[0] = dDataX;                       // Store the current data values for later reference
                        prev_dat[1] = dDataY;
                        prev_dat[2] = dDataZ;
                        lcd_put_data(dDataX, dDataY, dDataZ,0, 3, dMult); // Send data to LCD
                        uart_put_data(dDataX, dDataY, dDataZ,0, 3, TRUE); // Send data to uart
                    }   //NOTE: If data does not change in response to movement of sensor, SSC7150 not sending EC_DATA_AVAIL interrupts and 'may' need POR
                
                    if ( isPressed() == _SW_S4_RST )
                    {
                        //Disable the Accelerometer on exit
                        if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_SHC, &wRMWdata)) == SUCCESS )  // Read data from the Sensor Hub Configuration register
                        {
                            if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_SHC, wRMWdata & ~VREG_SHC_ACC_EN)) ) // Disable the accelerometer (clear the ACC_EN bit)
                                error_handler("VRWr",VREG_SHC, ucRetStat); // error display
                        }
                        else
                            error_handler("VRRd",VREG_SHC,ucRetStat);   // error display
                                
                        break;                                      // If S4 reset was pressed, escape this loop and return to main menu
                    }
                }
                break;

            case GYRO_MENU_SLCT:                                    // Gyrometer has been selected
                u2out("Gyrometer has been selected.\n\r");
                u2out("Displaying gyrometer data:\n\r");
                
                // Enable the Gyrometer
                // To support multi-threaded applications, Read-Modify-Write the VREG:
                if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_SHC, &wRMWdata)) == SUCCESS )  // Read data from the Sensor Hub Configuration register
                {
                    if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_SHC, wRMWdata | VREG_SHC_GYRO_EN)) ) // Enable the Gyrometer (GYRO_EN)
                        error_handler("VRWr",VREG_SHC, ucRetStat);  // error display
                }
                else
                    error_handler("VRRd",VREG_SHC,ucRetStat);       // error display

                if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_GYSEN, 0x01 )) )  // Change gyro sensitivity
                    error_handler("VRWr",VREG_GYSEN,ucRetStat);     // error display

                if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_GYDR, MAX_DATA_RATE)) )    // Change gyro data rate
                    error_handler("VRWr",VREG_GYDR,ucRetStat);      // error display
				
                dMult = ret_exponent(GYRO_VREG_OFFSET);             // Get unit exponent for this sensor's data

                while(1)
                {                                                   // get gyro readings
                    if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_GYXD, &wdata)) )  // Read data from the gyrometer datax register
                        error_handler("VRRd",VREG_GYXD,ucRetStat);  // error display

                    dDataX = (int)wdata * dMult;                    // Multiply this by the exponent scaler to get floating point value
 
                    if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_GYYD, &wdata)) )  // Read data from the gyrometer datay register
                        error_handler("VRRd",VREG_GYYD,ucRetStat);  // error display

                    dDataY = (int)wdata * dMult;                    // Multiply this by the exponent scaler to get floating point value

                    if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_GYZD, &wdata)) )  // Read data from the gyrometer dataz register
                        error_handler("VRRd",VREG_GYZD,ucRetStat);  // error display

                    dDataZ = (int)wdata * dMult;                    // Multiply this by the exponent scaler to get floating point value
                    
                    if (prev_dat[0] != dDataX || prev_dat[1] != dDataY || prev_dat[2] != dDataZ) // Ensure that this data has not yet been displayed
                    {
                        prev_dat[0] = dDataX;                       // Store the current data values for later reference
                        prev_dat[1] = dDataY;
                        prev_dat[2] = dDataZ;
                        lcd_put_data(dDataX, dDataY, dDataZ,0, 3, dMult); // Send data to LCD
                        uart_put_data(dDataX, dDataY, dDataZ,0, 3, TRUE); // Send data to uart
                    }   //NOTE: If data does not change in response to movement of sensor, SSC7150 not sending EC_DATA_AVAIL interrupts and 'may' need POR
                    
                    if ( isPressed() == _SW_S4_RST )
                    {
                        //Disable the Gyrometer on exit
                        if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_SHC, &wRMWdata)) == SUCCESS )  // Read data from the Sensor Hub Configuration register
                        {
                            if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_SHC, wRMWdata & ~VREG_SHC_GYRO_EN)) ) // Disable the Gyrometer (clear the GYRO_EN bit)
                                error_handler("VRWr",VREG_SHC, ucRetStat); // error display
                        }
                        else
                            error_handler("VRRd",VREG_SHC,ucRetStat); // error display
                                
                        break;                                      // If S4 reset was pressed, escape this loop and return to main menu
                    }   
                 }
                break;

            case COMP_MENU_SLCT:                                    // Compass has been selected
                u2out("Compass has been selected.\n\r");
                u2out("Displaying compass data:\n\r");
               
                // Enable the Compass
                // To support multi-threaded applications, Read-Modify-Write the VREG :
                if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_SHC, &wRMWdata)) == SUCCESS )  // Read data from the Sensor Hub Configuration register
                {
                    if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_SHC, wRMWdata | VREG_SHC_CMP_EN)) ) // Enable the compass (CMP_EN)
                         error_handler("VRWr",VREG_SHC, ucRetStat);  // error display
                }
                else
                    error_handler("VRRd",VREG_SHC,ucRetStat);       // error display
                               
                if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_CMSEN, MAX_SENSITIVITY)) )   // Change CMP sensitivity FOR CALIBRATION
                    error_handler("VRWr",VREG_CMSEN,ucRetStat);     // error display

                if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_CMDR, MAX_DATA_RATE)) )    // Change CMP data rate
                    error_handler("VRWr",VREG_CMDR,ucRetStat);      // error display

                dMult = ret_exponent(CMP_VREG_OFFSET);              // Get unit exponent for this sensor's data

                while(1)
                {
                    if ( ( ucRetStat = HOST_SF_LIB_VREG_read(VREG_CMD, &wdata)) ) // Read data from the compass data register
                        error_handler("VRRd",VREG_CMD,ucRetStat);   // error display

                    dDataX = (int)wdata * dMult;                    // Note: for the compass there is only one data field to be retrieved
                    
                    if (prev_dat[0] != dDataX)                      // Ensure that this data has not yet been displayed on LCD
                    {
                        prev_dat[0] = dDataX;                       // Store the current data value for later reference
                        lcd_put_data(dDataX,0,0,0, 1, dMult);
                        uart_put_data(dDataX,0,0,0, 1, TRUE); 
                    }   //NOTE: If data does not change in response to movement of sensor, SSC7150 not sending EC_DATA_AVAIL interrupts and 'may' need POR                    
                    
                    if ( isPressed() == _SW_S4_RST )
                    {
                        //Disable the Compass on exit
                        if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_SHC, &wRMWdata)) == SUCCESS )  // Read data from the Sensor Hub Configuration register
                        {
                            if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_SHC, wRMWdata & ~VREG_SHC_CMP_EN)) ) // Disable the compass (clear the CMP_EN bit)
                                error_handler("VRWr",VREG_SHC, ucRetStat); // error display
                        }
                        else
                            error_handler("VRRd",VREG_SHC,ucRetStat); // error display
                                
                        break;                                      // If S4 reset was pressed, escape this loop and return to main menu
                    }

                }
                break;

            case ORI_MENU_SLCT:                                     // Orientation sensor has been selected
                u2out("Orientation sensor has been selected.\n\r");
                u2out("Displaying orientation data:\n\r");

                // Enable the Orientation sensor
                // To support multi-threaded applications, Read-Modify-Write the VREG:
                if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_SHC, &wRMWdata)) == SUCCESS )  // Read data from the Sensor Hub Configuration register
                {
                    if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_SHC, wRMWdata | VREG_SHC_ORI_EN)) ) // Enable the orientation (ORI_EN)
                        error_handler("VRWr",VREG_SHC, ucRetStat);  // error display
                }
                else
                    error_handler("VRRd",VREG_SHC,ucRetStat);       // error display

                if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_ORSEN, 0xC8)) )    // Change ORI sensitivity
                    error_handler("VRWr",VREG_ORSEN,ucRetStat);     // error display

                if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_ORDR, MAX_DATA_RATE)) )     // Change ORI data rate
                    error_handler("VRWr",VREG_ORDR,ucRetStat);      // error display

                dMult = ret_exponent(ORI_VREG_OFFSET);              // Get unit exponent for this sensor's data

                while(1)
                {
                    if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_ORXD, &wdata)) ) // Read data from the orientation sensor datax register
                        error_handler("VRRd",VREG_ORXD,ucRetStat);  // error display

                    dDataX = (int)wdata * dMult;                    // Multiply this by the exponent scaler to get floating point value

                    if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_ORYD, &wdata)) ) // Read data from the orientation sensor datay register
                        error_handler("VRRd",VREG_ORYD,ucRetStat);  // error display

                    dDataY = (int)wdata * dMult;                    // Multiply this by the exponent scaler to get floating point value

                    if ( ( ucRetStat = HOST_SF_LIB_VREG_read(VREG_ORZD, &wdata)) ) // Read data from the orientation sensor dataz register
                        error_handler("VRRd",VREG_ORZD,ucRetStat);  // error display

                    dDataZ = (int)wdata * dMult;                    // Multiply this by the exponent scaler to get floating point value

                    if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_ORWD, &wdata)) )  // Read data from the orientation sensor dataw register
                        error_handler("VRRd",VREG_ORWD,ucRetStat);  //error display

                    dDataW = (int)wdata * dMult;                    // Multiply this by the exponent scaler to get floating point value
                    
                    if (prev_dat[0] != dDataX || prev_dat[1] != dDataY || prev_dat[2] != dDataZ || prev_dat[3] != dDataW) // Ensure that this data has not yet been displayed
                    {
                        prev_dat[0] = dDataX;                       // Store the current data values for later reference
                        prev_dat[1] = dDataY;
                        prev_dat[2] = dDataZ;
                        prev_dat[3] = dDataW;
                        lcd_put_data(dDataX, dDataY, dDataZ, dDataW, 4, dMult); // Send data to LCD
                        uart_put_data(dDataX, dDataY, dDataZ, dDataW, 4, TRUE); // Send data to uart
                    }   //NOTE: If data does not change in response to movement of sensor, SSC7150 not sending EC_DATA_AVAIL interrupts and 'may' need POR                    

                    if ( isPressed() == _SW_S4_RST )
                    {
                        //Disable the orientation sensor on exit
                        if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_SHC, &wRMWdata)) == SUCCESS )  // Read data from the Sensor Hub Configuration register
                        {
                            if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_SHC, wRMWdata & ~VREG_SHC_ORI_EN)) ) // Disable the orientation sensor (clear the ORI_EN bit)
                                error_handler("VRWr",VREG_SHC, ucRetStat);      // error display
                        }
                        else
                            error_handler("VRRd",VREG_SHC,ucRetStat); // error display
                                
                        break;                                      //If S4reset was pressed, escape this loop and return to main menu
                    }
                }
                break;
                
            case INCL_MENU_SLCT:                                    // Inclinometer has been selected
                u2out("Inclinometer has been selected.\n\r");
                u2out("Displaying inclinometer data:\n\r");

                // Enable the Inclinometer
                // To support multi-threaded applications, Read-Modify-Write the VREG:
                if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_SHC, &wRMWdata)) == SUCCESS )  // Read data from the Sensor Hub Configuration register
                {
                    if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_SHC, wRMWdata | VREG_SHC_INC_EN)) ) // Enable the Inclinometer (INC_EN)
                        error_handler("VRWr",VREG_SHC, ucRetStat);  // error display
                }
                else
                    error_handler("VRRd",VREG_SHC,ucRetStat);       // error display

                if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_INSEN, 0x01)) )    // Change sensitivity
                    error_handler("VRWr",VREG_INSEN,ucRetStat);     // error display

                if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_INDR, MAX_DATA_RATE)) )    // Change data rate
                    error_handler("VRWr",VREG_INDR,ucRetStat);      // error display

                dMult = ret_exponent(INCL_VREG_OFFSET);             // Get unit exponent for this sensor's data

                while(1)
                {
                    if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_INXD, &wdata)) ) // Read data from the inclinometer datax register
                        error_handler("VRRd",VREG_INXD,ucRetStat);  // error display

                    dDataX = (int)wdata * dMult;                    // Multiply this by the exponent scaler to get floating point value

                    if (( ucRetStat = HOST_SF_LIB_VREG_read(VREG_INYD, &wdata)) )  // Read data from the inclinometer datay register
                        error_handler("VRRd",VREG_INYD,ucRetStat);  // error display

                    dDataY = (int)wdata * dMult;                    // Multiply this by the exponent scaler to get floating point value

                    if ( ( ucRetStat = HOST_SF_LIB_VREG_read(VREG_INZD, &wdata)) )  // Read data from the inclinometer dataz register
                        error_handler("VRRd",VREG_INZD,ucRetStat);  // error display

                    dDataZ = (int)wdata * dMult;                    // Multiply this by the exponent scaler to get floating point value
                    
                    if(prev_dat[0] != dDataX || prev_dat[1] != dDataY || prev_dat[2] != dDataZ) // Ensure that this data has not yet been displayed
                    {
                        prev_dat[0] = dDataX;                       // Store the current data values for later reference
                        prev_dat[1] = dDataY;
                        prev_dat[2] = dDataZ;
                        lcd_put_data(dDataX, dDataY, dDataZ,0, 3, dMult); // Send data to LCD
                        uart_put_data(dDataX, dDataY, dDataZ,0, 3, TRUE); // Send data to uart
                    }     //NOTE: If data does not change in response to movement of sensor, SSC7150 not sending EC_DATA_AVAIL interrupts and 'may' need POR

                    if ( isPressed() == _SW_S4_RST )
                    {
                        // Disable the Inclinometer on exit
                        if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_SHC, &wRMWdata)) == SUCCESS )  // Read data from the Sensor Hub Configuration register
                        {
                            if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_SHC, wRMWdata & ~VREG_SHC_INC_EN)) ) // Disable the Inclinometer (clear the INC_EN bit)
                                error_handler("VRWr",VREG_SHC, ucRetStat);      // error display
                        }
                        else
                            error_handler("VRRd",VREG_SHC,ucRetStat); // error display
                                
                        break;                                      // If S4 reset was pressed, escape this loop and return to main menu
                    }
                }
                break;

            case RAW_ACC_MENU_SLCT:                                 // Raw Accelerometer data has been selected              
                u2out("Raw accelerometer has been selected.\n\r");
                u2out("Displaying raw accelerometer data:\n\r");

				// Enable the Raw Accelerometer
                // To support multi-threaded applications, Read-Modify-Write the VREG:
                if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_SHC, &wRMWdata)) == SUCCESS )  // Read data from the Sensor Hub Configuration register
                {
                    if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_SHC, wRMWdata | VREG_SHC_RAWACC_EN | VREG_SHC_ACC_EN)) ) // Enable the raw accelerometer (RAWACC_EN)& corresponding ACCEL sensor (to allow wake from D2-polled every 60 ms)
									//rev 0.10 spec : 'for any raw sensor to be enabled properly, the corresponding 
									//sensors must also be enabled. For RAW ACCEL corresponding sensor : ACCEL

                        error_handler("VRWr",VREG_SHC, ucRetStat);  // error display
                }
                else
                    error_handler("VRRd",VREG_SHC,ucRetStat);       // error display


                if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_RACSEN, MAX_SENSITIVITY)) ) // Set Raw Acc (and corresponding sensor (Acc)) sensitivity to "0" to 
																							//achieve the desired data rate for raw sensors (see spec).
                    error_handler("VRWr",VREG_RACSEN,ucRetStat);    // error display

                if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_ACSEN, MAX_SENSITIVITY)) ) // Set Acc sensitivity to "0" to achieve the desired data rates for raw sensors.
                    error_handler("VRWr",VREG_ACSEN,ucRetStat);    // error display
               
				if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_RACDR, MAX_DATA_RATE)) ) // Change Raw Acc data rate
                    error_handler("VRWr",VREG_RACDR,ucRetStat);     //error display

                dMult = ret_exponent(RAW_ACC_VREG_OFFSET);          // Get unit exponent for this sensor's data

                while(1)
                {
                    if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_RACXD, &wdata)) )  // Read data from the raw accelerometer datax register
                        error_handler("VRRd",VREG_RACXD,ucRetStat); // error display

                    dDataX = (int)wdata * dMult;                    // Multiply this by the exponent scaler to get floating point value

                    if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_RACYD, &wdata)) ) // Read data from the raw accelerometer datay register
                        error_handler("VRRd",VREG_RACYD,ucRetStat); // error display

                    dDataY = (int)wdata * dMult;                    // Multiply this by the exponent scaler to get floating point value

                    if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_RACZD, &wdata) ) )  // Read data from the raw accelerometer dataz register
                        error_handler("VRRd",VREG_RACZD,ucRetStat); // error display

                    dDataZ = (int)wdata * dMult;                    // Multiply this by the exponent scaler to get floating point value
                    
                    if(prev_dat[0] != dDataX || prev_dat[1] != dDataY || prev_dat[2] != dDataZ) // Ensure that this data has not yet been displayed
                    {
                        prev_dat[0] = dDataX;                           // Store the current data values for later reference
                        prev_dat[1] = dDataY;
                        prev_dat[2] = dDataZ;
                        lcd_put_data(dDataX, dDataY, dDataZ,0, 3, dMult); // Send data to LCD
                        uart_put_data(dDataX, dDataY, dDataZ,0, 3, TRUE); // Send data to uart
                    }    //NOTE: If data does not change in response to movement of sensor, SSC7150 not sending EC_DATA_AVAIL interrupts and 'may' need POR

                    if ( isPressed() == _SW_S4_RST )
                    {
                        // Disable the Raw Accelerometer & corresponding ACCEL sensor on exit
                        if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_SHC, &wRMWdata)) == SUCCESS )  // Read data from the Sensor Hub Configuration register
                        {
                            if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_SHC, wRMWdata & ~(VREG_SHC_RAWACC_EN | VREG_SHC_ACC_EN) )) ) // Disable the raw accelerometer (clear the RAWACC_EN bit) & corresponding ACC_EN
                                error_handler("VRWr",VREG_SHC, ucRetStat);      // error display
                        }
                        else
                            error_handler("VRRd",VREG_SHC,ucRetStat); // error display
                                
                        break;                                      // If S4 reset was pressed, escape this loop and return to main menu
                    }
                }
                break;

            case RAW_MAG_MENU_SLCT:                                 // Raw Magnometer data has been selected
                u2out("Raw magnometer has been selected.\n\r");
                u2out("Displaying raw magnometer data:\n\r");

                // Enable the Raw Magnometer
                // To support multi-threaded applications, Read-Modify-Write the VREG :
                if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_SHC, &wRMWdata)) == SUCCESS )  // Read data from the Sensor Hub Configuration register
                {
                    if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_SHC, wRMWdata | VREG_SHC_RAWMAG_EN | VREG_SHC_CMP_EN)) ) // Enable the  Raw Magnometer (RAWMAG_EN) & corresponding CMP sensor (to allow wake from D2-polled every 60 ms)
									//rev 0.10 spec: 'for any raw sensor to be enabled properly, the corresponding 
									//sensors must also be enabled. For RAW MAG corresponding sensor : CMP
                        error_handler("VRWr",VREG_SHC, ucRetStat);  // error display
                }
                else
                    error_handler("VRRd",VREG_SHC,ucRetStat);       // error display
               
                if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_RMGSEN, MAX_SENSITIVITY)) )  // Set Raw Acc (and corresponding sensor (Cmp)) sensitivity to "0" to 
																							//achieve the desired data rate for raw sensors (see spec).
                    error_handler("VRWr",VREG_RMGSEN,ucRetStat);    // error display

                if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_CMSEN, MAX_SENSITIVITY)) ) // Set Cmp sensitivity to "0" to achieve the desired data rates for raw sensors.
                    error_handler("VRWr",VREG_CMSEN,ucRetStat);    // error display

				if ((ucRetStat = HOST_SF_LIB_VREG_write(VREG_RMGDR, MAX_DATA_RATE)) )  // Change data rate
                    error_handler("VRWr",VREG_RMGDR,ucRetStat);     // error display
               
                dMult = ret_exponent(RAW_MAG_VREG_OFFSET);          // Get unit exponent for this sensor's data

                while(1)
                {
                    if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_RMGXD, &wdata)) )  // Read data from the raw magnometer datax register
                        error_handler("VRRd",VREG_RMGXD,ucRetStat); // error display

                    dDataX = (int)wdata * dMult;                    // Multiply this by the exponent scaler to get floating point value

                    if ((ucRetStat = HOST_SF_LIB_VREG_read(VREG_RMGYD, &wdata)) )  // Read data from the raw magnometer datay register
                        error_handler("VRRd",VREG_RMGYD,ucRetStat); // error display

                    dDataY = (int)wdata * dMult;                    // Multiply this by the exponent scaler to get floating point value

                    if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_RMGZD, &wdata)) )  // Read data from the raw magnometer dataz register
                        error_handler("VRRd",VREG_RMGZD,ucRetStat); // error display

                    dDataZ = (int)wdata * dMult;                    // Multiply this by the exponent scaler to get floating point value
                    
                    if (prev_dat[0] != dDataX || prev_dat[1] != dDataY || prev_dat[2] != dDataZ) // Ensure that this data has not yet been displayed
                    {
                        prev_dat[0] = dDataX;                       // Store the current data values for later reference
                        prev_dat[1] = dDataY;
                        prev_dat[2] = dDataZ;
                        lcd_put_data(dDataX, dDataY, dDataZ,0, 3, dMult);  // Send data to LCD
                        uart_put_data(dDataX, dDataY, dDataZ,0, 3, TRUE); // Send data to uart
                    }    //NOTE: If data does not change in response to movement of sensor, SSC7150 not sending EC_DATA_AVAIL interrupts and 'may' need POR

                    if ( isPressed() == _SW_S4_RST )
                    {
                        //Disable the Raw Magnometer on exit
                        if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_SHC, &wRMWdata)) == SUCCESS )  // Read data from the Sensor Hub Configuration register
                        {
                            if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_SHC, wRMWdata & ~(VREG_SHC_RAWMAG_EN | VREG_SHC_CMP_EN)) ) ) // Disable the Raw Magnometer (clear the RAWMAG_EN bit) & corresponding CMP_EN
                                error_handler("VRWr",VREG_SHC, ucRetStat);      // error display
                        }
                        else
                            error_handler("VRRd",VREG_SHC,ucRetStat); // error display
                                
                        break;                                      // If S4 reset was pressed, escape this loop and return to main menu
                    }
               }
                break;

            case RAW_GYRO_MENU_SLCT:                                // Raw Gyrometer data has been selected                
                u2out("Raw gyrometer has been selected.\n\r");
                u2out("Displaying raw gyrometer data:\n\r");

                // Enable the Raw Gyrometer
                // To support multi-threaded applications, Read-Modify-Write the VREG:
                if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_SHC, &wRMWdata)) == SUCCESS )  // Read data from the Sensor Hub Configuration register
                {
                    if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_SHC, wRMWdata | VREG_SHC_RAWGYR_EN | VREG_SHC_GYRO_EN)) ) // Enable the Raw Gyrometer (GYR_EN) & corresponding GYRO sensor (to allow wake from D2-polled every 60 ms)
												//rev 0.10 spec: 'for any raw sensor to be enabled properly, the corresponding 
												//sensors must also be enabled. For RAW GYRO corresponding sensor : GYRO
                        error_handler("VRWr",VREG_SHC, ucRetStat);  // error display
                }
                else
                    error_handler("VRRd",VREG_SHC,ucRetStat);       // error display

                if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_RGYSEN, MAX_SENSITIVITY)) ) // Set Raw Gyro (and corresponding sensor (Gyro)) sensitivity to "0" to 
																							//achieve the desired data rate for raw sensors (see spec).
                    error_handler("VRWr",VREG_RACSEN,ucRetStat);    // error display

                if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_GYSEN, MAX_SENSITIVITY)) ) // Set Gyro sensitivity to "0" to achieve the desired data rates for raw sensors.
                    error_handler("VRWr",VREG_GYSEN,ucRetStat);    // error display
               
                
                if ( ( ucRetStat = HOST_SF_LIB_VREG_write(VREG_RGYDR, MAX_DATA_RATE)) )  // Change data rate
                    error_handler("VRRd",VREG_RGYDR,ucRetStat);     // error display

                dMult = ret_exponent(RAW_GYR_VREG_OFFSET);          // Get unit exponent for this sensor's data

                while(1)
                {
                    if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_RGYXD, &wdata)) ) // Read data from the raw gyrometer datax register
                        error_handler("VRRd",VREG_RGYXD,ucRetStat); // error display

                    dDataX = (int)wdata * dMult;                    // Multiply this by the exponent scaler to get floating point value

                    if (( ucRetStat = HOST_SF_LIB_VREG_read(VREG_RGYYD, &wdata)) ) // Read data from the raw gyrometer datay register
                        error_handler("VRRd",VREG_RGYYD,ucRetStat); // error display

                    dDataY = (int)wdata * dMult;                    // Multiply this by the exponent scaler to get floating point value

                    if ((ucRetStat = HOST_SF_LIB_VREG_read(VREG_RGYZD, &wdata)) ) // Read data from the raw gyrometer dataz register
                        error_handler("VRRd",VREG_RGYZD,ucRetStat); // error display

                    dDataZ = (int)wdata * dMult;                    // Multiply this by the exponent scaler to get floating point value
                    
                    if (prev_dat[0] != dDataX || prev_dat[1] != dDataY || prev_dat[2] != dDataZ) // Ensure that this data has not yet been displayed
                    {
                        prev_dat[0] = dDataX;                       // Store the current data values for later reference
                        prev_dat[1] = dDataY;
                        prev_dat[2] = dDataZ;
                        lcd_put_data(dDataX, dDataY, dDataZ,0, 3, dMult); // Send data to LCD
                        uart_put_data(dDataX, dDataY, dDataZ,0, 3, TRUE); // Send data to uart
                    }   //NOTE: If data does not change in response to movement of sensor, SSC7150 not sending EC_DATA_AVAIL interrupts and 'may' need POR

                    if ( isPressed() == _SW_S4_RST )
                    {
                        //Disable the Raw Gyro on exit
                        if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_SHC, &wRMWdata)) == SUCCESS )  // Read data from the Sensor Hub Configuration register
                        {
                            if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_SHC, wRMWdata & ~(VREG_SHC_RAWGYR_EN | VREG_SHC_GYRO_EN )) ) ) // Disable the raw gyro (clear the GYR_EN bit) & corresponding GYRO_EN
                                error_handler("VRWr",VREG_SHC, ucRetStat);      //error display
                        }
                        else
                            error_handler("VRRd",VREG_SHC,ucRetStat); // error display
                                
                        break;                                      // If S4 reset was pressed, escape this loop and return to main menu
                    }
                }
                break;
           
            case SLP_MENU_SLCT:
                u2out("\n\rSleep has been selected\n\r");
                LCD_ClearScreen();
                LCD_PutString("Sleep ...       ", 16);
                
                // Enable Sleep
                // To support multi-threaded applications, Read-Modify-Write the VREG:
                if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_SHC, &wRMWdata)) == SUCCESS )  // Read data from the Sensor Hub Configuration register
                {
                    if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_SHC, wRMWdata | VREG_SHC_SLP_BIT )) ) // Set sleep bit
                        error_handler("VRWr",VREG_SHC, ucRetStat);  // error display
                }
                else
                    error_handler("VRRd",VREG_SHC,ucRetStat);       // error display
				
                u2out("\n\rMM7150 is Sleeping....\n\r");
                u2out("\n\rPress S4 to continue \n\r");             //keep the LCD and UART menus
                LCD_ClearScreen();
                LCD_PutString("MM7150 Sleeping ", 16);
                LCD_PutString("Press S4 ...    ", 16);

                while ( isPressed() != _SW_S4_RST );                //wait here for S4
                    break;

            case WAKE_MENU_SLCT:
                u2out("\n\rWake has been selected\n\r");
                LCD_ClearScreen();
                LCD_PutString("Wake ...        ", 16);
                
                bWakeExpected = TRUE;
				// Enable Wake
                // To support multi-threaded applications, Read-Modify-Write the VREG:
                if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_SHC, &wRMWdata)) == SUCCESS )  // Read data from the Sensor Hub Configuration register
                {
                    if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_SHC, wRMWdata | VREG_SHC_WAKE_BIT )) ) // Set wake bit
                        error_handler("VRWr",VREG_SHC, ucRetStat);  // error display
                }
                else
                    error_handler("VRRd",VREG_SHC,ucRetStat);       // error display
                
                u2out("\n\rWake has completed\n\r");
                u2out("\n\rPress S4 to continue \n\r");             //keep the LCD and UART menus
                LCD_ClearScreen();
                LCD_PutString("Wake Completed  ", 16);
                
                LCD_PutString("Press S4 ...    ", 16);

                while ( isPressed() != _SW_S4_RST );                //wait here for S4
                break;
                
            case FLASH_UPDATE_MENU_SLCT:                            // Flash update has been selected              
               
                u2out("\n\rFlash update has been selected\n\r");                
                if (flash_info_command(FLASH_BUF, FLASH_UPDATE_RESET_CMD)) //issue reset command to put SSC7150 into boot loader mode
                {
                    u2out("\n\n\r***Module RESET failed*** \n\n\r");
                    LCD_ClearScreen();
                    LCD_PutString("**Reset Failed**", 16);
				}	
                else
                {
                    u2out("\n\n\rRESET module to BootLoader Mode successfully\n\n\r");
                    
                    //delay for SSC7150 to go from SF application mode to boot loader mode
                    delay(200);                                     //200 ms
  
                    // Check if the SSC7150 is in BootLoader mode
                    if (!flash_info_command(FLASH_BUF, FLASH_UPDATE_BLDR_INFO_CMD))	//check that the SSC7150 is indeed in boot loader mode
                    {
                        if ((FLASH_BUF[5] != FLASH_HDR_VAL) || (FLASH_BUF[4] != FLASH_HDR_VAL) || (FLASH_BUF[3] != FLASH_HDR_VAL))
                        {
                            // SSC7150 is not in BootLoader mode
                            u2out("\n\rSSC7150 is not in BootLoader mode\n\r");
                            LCD_ClearScreen();
                            LCD_PutString("**Reset Failed**", 16);
                        }
                        else
                        {
DO_FLASH_RECOVERY:
                            //SSC7150 is in BootLoader mode and ready to start flash update process
                            u2out("\n\rSSC7150 is in BootLoader mode\n\r");
                            LCD_ClearScreen();
                            LCD_PutString("Flash Update ...", 16);
                            LCD_PutString("In progress     ", 16);

                            u2out("\n\n\rSelect the BIN file to download or Press S4 to exit\n\r");
                            //initialize variables used by UART Rx handler to download binary from HOST
                            m = 0;
                            pbuf = FLASH_BUF;
                            seq_number = 0;
                            process_packet = FALSE;
                            img_wr_done = FALSE;
                            while(1)
                            {
                                if (process_packet == FALSE)
                                {
                                    n = UART2_ReadBuffer(pbuf, PACKET_LENGTH - m);
                                    m += n;
                                    pbuf += n;
                                    if(m==PACKET_LENGTH)
                                    {
                                        process_packet = TRUE;
                                        m = 0;
                                        pbuf = FLASH_BUF;
                                    }
                                }
                                //wait for an entire packet of data received from Host
                                if (process_packet == TRUE)
                                {
                                    process_packet = FALSE;
                                    // Perform next flash write
                                    if (flash_write_command(FLASH_BUF, FLASH_WRITE_CMD))
                                    {
                                        u2out("\n\n\rFlash Write Error: ");
                                        u2out((char*)&FLASH_BUF[0]);
                                        u2out("\n\n\r POR Exp16 hardware \n\n\r");
                                        u2out("\n\n\r");

                                        LCD_ClearScreen();
                                        LCD_PutString("**Update ERROR**", 16);
                                        LCD_PutString("POR Exp16 board ", 16);
                                        while(1);                   //hang here waiting for Power on Reset
                                    }

                                    else
                                    {
                                        u2out("\rPackets written: ");
                                        u2out((char*)&FLASH_BUF[0]);
                                    }
								}
	                                                        
                                if (img_wr_done == TRUE)
                                {                                   //If all packets written, get flash info
                                    img_wr_done = FALSE;
                                    u2out("\n\n\rImage write completed!! \n\n\r");
                                    LCD_ClearScreen();
	                                
                                    delay(DELAY100);                //SSC7150 needs time to do CRC calculation
	                                
                                    if ( !flash_verify_command(FLASH_BUF)) //Verify the programmed image
                                    {
                                        memset(FLASH_BUF, 0, sizeof(FLASH_BUF));
	                                    
                                        flash_info_command(FLASH_BUF, FLASH_UPDATE_RESET_CMD);
	                                    
                                        u2out("\n\n\rVerification Pass: ");
                                        u2out("\n\n\rNEW SF label: ");
                                        u2out((char*)&FLASH_BUF[LABEL_INFO_INDEX]);
                                        u2out("\n\n\r POR Exp16 hardware \n\n\r");
	                                    
                                        LCD_PutString("Image VERIFIED  ", 16);
                                        LCD_PutString("POR Exp16 board ", 16);
	                                                                      
                                        while(1);                   //Flash update success, requires Power On Reset of SSC7150 (which means POR of Exp16)
                                    }
                                    else
                                    {
                                        u2out("\n\n\r***Verification Failed*** \n\n\r");
                                        LCD_PutString("**Verify FAILED*", 16);
                                    }

                                    break;
                                }
                                if ( isPressed() == _SW_S4_RST )	//give user a way to escape without programming flash
                                {
                                    u2out("\n\n\r Flash Update process ABORTED by user\n\n\r");
                                    LCD_ClearScreen();
                                    LCD_PutString("Update Aborted  ", 16);
                                    break;
                                }
                            }
                        }
                    }
                }		
                
                u2out("\n\n\r POR Exp16 with MM7150-PICtail hardware \n\n\r");
                LCD_PutString("POR Exp16 board ", 16);
                while(1);

                break;                                              //end of flash_update

            case FLASH_CONF_MENU_SLCT:								//Flash configuration update has been selected 

                u2out("\n\rFlash Configuration update has been selected\n\r");
                if (flash_info_command(FLASH_BUF, FLASH_UPDATE_RESET_CMD)) //issue reset command to put SSC7150 into boot loader mode
                {
                    u2out("\n\n\r***Module RESET failed*** \n\n\r");
                    LCD_ClearScreen();
                    LCD_PutString("**Reset Failed**", 16);
				}			
                else
                {
                    u2out("\n\n\rRESET module to BootLoader Mode successfully\n\n\r");
                    //delay for SSC7150 to go from SF application mode to boot loader mode
                    delay(200);                                     //200 ms
  
                    // Check if the SSC7150 is in BootLoader mode
                    if (!flash_info_command(FLASH_BUF, FLASH_UPDATE_BLDR_INFO_CMD))	//check that the SSC7150 is indeed in boot loader mode
                    {
                        if ((FLASH_BUF[5] != FLASH_HDR_VAL) || (FLASH_BUF[4] != FLASH_HDR_VAL) || (FLASH_BUF[3] != FLASH_HDR_VAL))
                        {
                            // SSC7150 is not in BootLoader mode
                            u2out("\n\rSSC7150 is not in BootLoader mode\n\r");
                            LCD_ClearScreen();
                            LCD_PutString("**Reset Failed**", 16);
                        }
                        else
                        {
                            //SSC7150 is in BootLoader mode and ready to start flash update process
                            u2out("\n\rSSC7150 is in BootLoader mode\n\r");

                            delay(200);                             //200 ms
                            memset(FLASH_BUF, 0, sizeof(FLASH_BUF));
                            if (flash_info_command(FLASH_BUF, FLASH_GET_INFO_CMD))	 //issue getinfo command to prepare to write configuration table
                            {
                                u2out("\n\n\r***Module INFO failed*** \n\n\r");
                                LCD_ClearScreen();
                                LCD_PutString("**Info  Failed**", 16);
                            }			
                            else
                            {
                                u2out("\n\n\rGet Info from MM7150 successful\n\n\r");
                                if ((FLASH_BUF[5] != FLASH_HDR_VAL) || (FLASH_BUF[4] != FLASH_HDR_VAL) || (FLASH_BUF[3] != FLASH_HDR_VAL))
                                {
                                    // SSC7150 is not in BootLoader mode
                                    u2out("\n\rSSC7150 is not in BootLoader mode\n\r");
                                    LCD_ClearScreen();
                                    LCD_PutString("**Info  Failed**", 16);
                                }
                                else
                                {
                                    //SSC7150 is in BootLoader mode and ready to start flash update process
                                    memset(FLASH_BUF, 0, sizeof(FLASH_BUF));
                                    u2out("\n\rFlash Configuration Update in progress...\n\r");
                                    LCD_ClearScreen();
                                    LCD_PutString("FlshCfg Updt ...", 16);
                                    LCD_PutString("In progress     ", 16);

                                    u2out("\n\n\rSelect the BIN configuration file to download or Press S4 to exit\n\r");

                                    //initialize variables used by UART Rx handler to download binary from HOST
                                    m = 0;
                                    pbuf = FLASH_BUF;
                                    seq_number = 0;
                                    process_packet = FALSE;
                                    img_wr_done = FALSE;
                                    bFlshHdr = FALSE;
                                    
                                    //wait for an entire "valid" packet of data received from Host
                                    while (1)
                                    {
                                        if (process_packet == FALSE)
                                        {
                                            n = UART2_ReadBuffer(pbuf, PACKET_LENGTH - m);
                                            m += n;
                                            pbuf += n;
                                            if(m==PACKET_LENGTH)
                                            {
                                                process_packet = TRUE;
                                                m = 0;
                                                pbuf = FLASH_BUF;
                                            }
                                        }
                                        if (!bFlshHdr && (m >= FLASH_HDR_SIZE) && !process_packet)	//check that the expected header has arrived, otherwise if "binary" not selected can hang here waiting full packet  
                                        {															
                                            for (i = 0; i < FLASH_HDR_SIZE; i++)  //check 1st 4 header bytes for known value
                                            {
                                                if (FLASH_BUF[i] != FLASH_HDR_VAL) 
                                                {
                                                    u2out("\n\n\r**Flash Config Download Error: Incorrect Header**\n\r");
                                                    u2out("\n\n\r POR Exp16 hardware \n\n\r");
                                                    LCD_ClearScreen();
                                                    LCD_PutString("**Update ERROR**", 16);
                                                    LCD_PutString("POR Exp16 board ", 16);
                                                    while(1);       //hang here waiting for Power on Reset
                                                }
                                            }
                                            bFlshHdr = TRUE;		//already checked this header
                                        }

                                        if (process_packet == TRUE) //received the entire packet
                                        {
                                            process_packet = FALSE;

                                            ucRetStat = flash_write_sector_command(FLASH_BUF, FLASH_WRITE_SECTOR_CMD); //write packet to specified flash sector

                                            if (ucRetStat)
                                            {
                                                u2out("\n\n\r**Flash Write Sector Error**: ");
                                                sprintf(u2out_buf, "error code = 0x%X\n\r", ucRetStat);
                                                u2out(u2out_buf);
                                                LCD_ClearScreen();
                                                LCD_PutString("**Update ERROR**", 16);
                                                LCD_PutString("POR Exp16 board ", 16);
                                                while(1);               //hang here waiting for Power on Reset
                                            }
                                            else
                                            {
                                                delay(200);             //200 ms
                                                if (!flash_read_sector_command(FLASH_BUF))  //read back to verify
                                                {
                                                    u2out("\n\n\r***Configuration Image VERIFIED*** \n\n\r");
                                                    u2out("\n\n\r POR Exp16 hardware \n\n\r");
                                                    LCD_PutString("Image VERIFIED  ", 16);
                                                    LCD_PutString("POR Exp16 board ", 16);

                                                    while(1);           // Flash update success, requires Power On Reset of SSC7150 (which means POR of Exp16)
                                                }
                                                else
                                                {
                                                    u2out("\n\n\r***Verification Failed*** \n\n\r");
                                                    LCD_PutString("**Verify FAILED*", 16);
                                                }
                                            }
                                        }

                                        if ( isPressed() == _SW_S4_RST )	//give user a way to escape without programming flash
                                        {
                                            u2out("\n\n\r Flash Update process ABORTED by user\n\n\r");
                                            LCD_ClearScreen();
                                            LCD_PutString("Update Aborted  ", 16);
                                            break;
                                        }
                                    }
                                }		
                            }
                        }
                    }  
                } 

                u2out("\n\n\r POR Exp16 with MM7150-PICtail hardware \n\n\r");
                LCD_PutString("POR Exp16 board ", 16);

                while(1);
                break;                                              //end of flash_configuration update
			
            case CALI_MENU_SLCT:

                u2out("\n\rCalibration has been selected\n\r");
                LCD_ClearScreen();
                LCD_PutString("Calibration ... ", 16);
                
                // Enable the Inclinometer for calibration mode because it uses AGM raw sensors
                // To support multi-threaded applications, Read-Modify-Write the VREG:
                if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_SHC, &wRMWdata)) == SUCCESS )  // Read data from the Sensor Hub Configuration register
                {
                    if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_SHC, wRMWdata | VREG_SHC_INC_EN)) ) // Enable the Inclinometer (INC_EN)
                        error_handler("VRWr",VREG_SHC, ucRetStat);  //error display
                }
                else
                    error_handler("VRRd",VREG_SHC,ucRetStat);       //error display

                if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_INSEN, MAX_SENSITIVITY)) )    //Change sensitivity=0 as per spec for calibration 
                    error_handler("VRWr",VREG_INSEN,ucRetStat);     // error display

                if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_INDR, MAX_DATA_RATE)) )    //Change data rate= 10 (max data rate @10ms)
                    error_handler("VRWr",VREG_INDR,ucRetStat);      // error display			
				
				u2out("\n\rPress S4 when completed \n\r");          //keep the LCD and UART menus
                LCD_PutString("Press S4 ...    ", 16);

                while ( isPressed() != _SW_S4_RST );                //wait here for S4
                
                // Disable the Inclinometer on exit
                if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_SHC, &wRMWdata)) == SUCCESS )  // Read data from the Sensor Hub Configuration register
                {
                    if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_SHC, wRMWdata & ~VREG_SHC_INC_EN)) ) // Disable the Inclinometer (clear the INC_EN bit)
                        error_handler("VRWr",VREG_SHC, ucRetStat);  //error display
                }
                else
                    error_handler("VRRd",VREG_SHC,ucRetStat);       //error display
                   
                break;                                              //end of calibration mode

           
            case CALI_SAVE_MENU_SLCT:
                //NOTE:  Exercise caution calling this command too often as the MM7150 has LIMITED LIFECYCLE for NVM updates. 
                u2out("\n\r***CAUTION: ONE-TIME (NVM-save) calibration has been selected\n\r");
                LCD_ClearScreen();
                LCD_PutString("Calibration->NVM", 16);
                
                //check if NVM write flag already set in Diagnostic status register. If it is set then need to do POR to clear it.
                if ( (ucRetStat = diag_info_command(FLASH_BUF)) )	// read diagnostic report ID to check if NVM flag already set
                    error_handler("misc",0, ucRetStat);				//no return from error handler

                if ((FLASH_BUF[DIAG_ACCU_STAT] & (1<<DIAG_NVM_BIT) ) != 0)
                {
                    u2out("\n\r***Coefficients in NVM bit already set***\n\r");
                    u2out("\n\r***POR Explorer16 with MM7150-PICtail required to reset***\n\r");

                    LCD_ClearScreen ( ) ;
                    LCD_PutString("Calibration Err ", 16);
                    LCD_PutString("POR Exp16 Board ", 16);
                    while (1);										//hang here, need Power On Reset
                }

                // Enable the Inclinometer for calibration mode because it uses AGM raw sensors
                // To support multi-threaded applications, Read-Modify-Write the VREG:
                if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_SHC, &wRMWdata)) == SUCCESS )  // Read data from the Sensor Hub Configuration register
                {
                    if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_SHC, wRMWdata | VREG_SHC_INC_EN)) ) // Enable the Inclinometer (INC_EN)
                        error_handler("VRWr",VREG_SHC, ucRetStat);  //error display
                }
                else
                    error_handler("VRRd",VREG_SHC,ucRetStat);       //error display

                if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_INSEN, MAX_SENSITIVITY)) ) //Change sensitivity=0 as per spec for calibration 
                    error_handler("VRWr",VREG_INSEN,ucRetStat);     //error display

                if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_INDR, MAX_DATA_RATE)) )   //Change data rate= 10 (max data rate @10ms)
                    error_handler("VRWr",VREG_INDR,ucRetStat);      //error display			
				
                u2out("\n\rPress S4 to start calibration\n\r"); 
                LCD_PutString("Push S4-MoveSens", 16);
                
                while ( isPressed() != _SW_S4_RST );
				
                u2out("\n\rBegin calibration movements for at least 5 seconds\n\r");
                u2out("      until HIGH accuracy flag set\n\r");
                u2out("         or Press S4 again to exit\n\r"); 

                POR_TIMER = 0;	//reset the 10 ms timer counter;
                while(1)
                {
                    if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_INXD, &wdata)) ) // Read data from the inclinometer datax register
                        error_handler("VRRd",VREG_INXD,ucRetStat);  // error display

                    if (( ucRetStat = HOST_SF_LIB_VREG_read(VREG_INYD, &wdata)) )  // Read data from the inclinometer datay register
                        error_handler("VRRd",VREG_INYD,ucRetStat);  // error display

                    if ( ( ucRetStat = HOST_SF_LIB_VREG_read(VREG_INZD, &wdata)) )  // Read data from the inclinometer dataz register
                        error_handler("VRRd",VREG_INZD,ucRetStat);  // error display


                    if (POR_TIMER >= DELAY5K)						//so calibration motion for at least 3 seconds
                    {
                        //check that accuracy flag status = "high accuracy"
                        if ( (ucRetStat = diag_info_command(FLASH_BUF)) ) //read diagnostic report ID to skip 1 min requirement for NVM
                            error_handler("misc",0, ucRetStat);			//no return from error handler

                        if ((FLASH_BUF[DIAG_ACCU_STAT] & 0xF) == HIGH_ACCURACY) //there's a problem, the accuracy flag should = 3 
                        {
                            u2out("\n\r***Accuracy flag set, stop calibration movements***\n\r");
                            break;
                        }
                    }

                    if ( isPressed() == _SW_S4_RST )
                        break;
                }

                //do deep sleep to store new calibration to MM7150 NVM
                //At this point need to do SLEEP, wait 1 sec, WAKE and check NVM write success flag

                bWakeExpected = TRUE;
				// Disable ALL sensors (only the Inclinometer s/b enabled at this point) in order to go to Deep Sleep
                if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_SHC, &wRMWdata)) == SUCCESS )  // Read data from the Sensor Hub Configuration register
                {
                    if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_SHC, wRMWdata & ~VREG_SHC_INC_EN)) ) // Disable the Inclinometer (clear the INC_EN bit)
                        error_handler("VRWr",VREG_SHC, ucRetStat);  //error display
                }
                else
                    error_handler("VRRd",VREG_SHC,ucRetStat);       //error display

                // Enable Sleep
                // To support multi-threaded applications, Read-Modify-Write the VREG:
                if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_SHC, &wRMWdata)) == SUCCESS )  // Read data from the Sensor Hub Configuration register
                {
                    if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_SHC, wRMWdata | VREG_SHC_SLP_BIT )) ) // Set sleep bit
                        error_handler("VRWr",VREG_SHC, ucRetStat);  //error display
                }
                else
                    error_handler("VRRd",VREG_SHC,ucRetStat);       //error display

                u2out("\n\rSaving calibration to NVM, please wait....\n\r");
                LCD_ClearScreen ( ) ;
                LCD_PutString("Saving Cali     ", 16);
                LCD_PutString("wait...         ", 16);

                delay (DELAY1K);                                    //wait 1 second for MM7150 F/W

                // Enable Wake
                // To support multi-threaded applications, Read-Modify-Write the VREG:
                if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_SHC, wRMWdata | VREG_SHC_WAKE_BIT )) ) // Set wake bit
                    error_handler("VRWr",VREG_SHC, ucRetStat);      //error display

                //Send Diagnostic command again and check the NVM bit status (indicates coefficient Flash update successful)
                if ( (ucRetStat = diag_info_command(FLASH_BUF)) )	//read diagnostic report ID to skip 1 min requirement for NVM
                    error_handler("misc",0, ucRetStat);				//no return from error handler

                //check that Coefficients in NVM bit set following sleep/wake cycle
                if ((FLASH_BUF[DIAG_ACCU_STAT] & (1<<DIAG_NVM_BIT) ) == 0)
                {
                    u2out("\n\r***Coefficients in NVM bit incorrect, calibration failed***\n\r");

                    LCD_ClearScreen ( ) ;
                    LCD_PutString("Calibration Err ", 16);
                }

                u2out("\n\rCalibration saved to Non-Volatile-Memory\n\r"); 
                u2out("\n\rPress S4 to exit\n\r"); 			
                LCD_ClearScreen ( ) ;
                LCD_PutString("Save Completed  ", 16);
                LCD_PutString("Press S4 ...    ", 16);

                while ( isPressed() != _SW_S4_RST );                //wait here for S4

                break;                                              //end of calibration mode
 
            case FREE_FALL_MENU_SLCT:

                u2out("FreeFall mode has been selected.\n\r");
                u2out("Disabling UART and LCD updates temporarily to achieve maximum sensor rate.\n\r");
                
                u2out("\n\rExp16 board LED D9 indicates FREE-FALL, LED D8 indicates IMPACT with surface\n\r");
                u2out("\n\rDrop sensor or Press S4 to exit\n\r");

                LCD_ClearScreen ( ) ;
                LCD_PutString("FreeFall Demo   ", 16);
                LCD_PutString("Press S4 to exit", 16);

                memset(&FF_ENTRY, 0x00, sizeof(FF_ENTRY));

                // Enable the Accelerometer with max sensitivity and data rate in order to detect free-fall motion
                // To support multi-threaded applications, Read-Modify-Write the VREG:
                if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_SHC, &wRMWdata)) == SUCCESS )  // Read data from the Sensor Hub Configuration register
                {
                    if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_SHC, wRMWdata | VREG_SHC_ACC_EN)) ) // Enable the accelerometer (ACC_EN)
                        error_handler("VRWr",VREG_SHC, ucRetStat);  //error display
                }
                else
                    error_handler("VRRd",VREG_SHC,ucRetStat);       //error display

                if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_ACSEN, MAX_SENSITIVITY)) ) // Change Accelerometer sensitivity
                    error_handler("VRWr",VREG_ACSEN,ucRetStat);     //error display

                if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_ACDXR, MAX_DATA_RATE)) ) // Change Accelerometer data rate
                    error_handler("VRWr",VREG_ACDXR,ucRetStat);     //error display

                dMult = ret_exponent(ACCEL_VREG_OFFSET);            //Get unit exponent for this sensor's data
                
                bS4_Pressed = FALSE;
                bIn_FF_flag = FALSE;
                FF_Flag = 0;
                bFFall_set = FALSE;
                bImpact_set = FALSE;
                wFFPos = 160;										//Free-fall range : if X,Y,Z reading within +/- 160 -> 0.160 of 0.000
                wPosImpact = 1500;									//Impact range : if X,Y,Z reading within +/- 1500 -> 1.500	

                //turn off freefall/impact LED indicators on Exp16 board
                PORTA &= ~(LED_D09 | LED_D08);

                while(1)
                {                                                   //get Accelerometer readings
 
                    if (bFF_INT)
                    {                                               //only read sensor data when the interrupt presents itself
                        bFF_INT = FALSE;

                        if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_ACXD, &wDataX)) )  // Read data from the accelerometer datax register
                            error_handler("VRRd",VREG_ACXD,ucRetStat);  // error display
                        if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_ACYD, &wDataY)) )  // Read data from the accelerometer datay register
                            error_handler("VRRd",VREG_ACYD,ucRetStat);  // error display
                        if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_ACZD, &wDataZ)) )  // Read data from the accelerometer dataz register
                            error_handler("VRRd",VREG_ACZD,ucRetStat);  //error display
						                                           
                        wTmpX = wDataX;
                        cnv_neg (&wTmpX);                           //absolute value conversion
					
                        wTmpY = wDataY;
                        cnv_neg (&wTmpY);                           //absolute value conversion

                        wTmpZ = wDataZ;
                        cnv_neg (&wTmpZ);                           //absolute value conversion
                        
                        bFFall_set = FALSE;

                        if (bImpact_set == FALSE)                   //check for impact range readings
                        {
                            if ((( wTmpX > wPosImpact) || ( wTmpY > wPosImpact) || ( wTmpZ > wPosImpact)) && FF_Flag)		//this can't be the 1st data point
                            {
                                bImpact_set = TRUE;                               
                                PORTA |= LED_D08;                   //turn on impact LED_D08 indicator
                            }					

                            if (!bImpact_set)
                            {
                                if ( ( wTmpX < wFFPos) && ( wTmpY < wFFPos) && ( wTmpZ < wFFPos) ) 
                                {
                                    //if this is the 1st data point then can't accept all 0's because that condition can happen when initializing sensor to new params
                                    if (!(FF_Flag) && (!wDataX) && (!wDataY) && (!wDataZ))
                                        bFFall_set = FALSE;

                                    else bFFall_set = TRUE;
                                }
                                else bFFall_set = FALSE;
                            }   

                            //check for free-fall condition
                            if ( bFFall_set || bIn_FF_flag)			//all 3-axis must be within selected range 
                            {	
                                //free-fall has been initiated							
                                if (FF_Flag == 0)					//1st time the free-fall conditions have been met
                                {
                                    FF_ENTRY[FF_Flag].usMs_time_stamp = POR_TIMER = 0;	//reset the 10 ms timer counter;
                                    FF_ENTRY[FF_Flag].bLinearFreeFall = FALSE;			//haven't determined linear freefall yet
                                    FF_ENTRY[FF_Flag].ucFFstate = 1; //can only get here if bFFall_set = TRUE
                                }
                                else
                                {
                                    FF_ENTRY[FF_Flag].usMs_time_stamp = (UINT16)POR_TIMER;	//save current timer count to this event

                                    if (POR_TIMER - FF_ENTRY[FF_Flag-1].usMs_time_stamp > 11) //check if this event is < 11ms since last determined freefall event
                                    {
                                        FF_ENTRY[FF_Flag].bLinearFreeFall = FALSE;
                                        FF_ENTRY[FF_Flag].ucFFstate = 0;
                                    }
                                    else							//freefall determined within 10 msec of last event
                                    {
                                        if ( FF_ENTRY[FF_Flag-1].ucFFstate != 0) //if previous event was a freefall event                                     
                                        {
                                            if (bFFall_set)
                                            {
                                                FF_ENTRY[FF_Flag].bLinearFreeFall = TRUE; // then this is also based on freefall flag 
                                                FF_ENTRY[FF_Flag].ucFFstate = 2;
                                                PORTA |= LED_D09;	//turn on freefall LED_D09 indicator
                                            }
                                            else
                                            {
                                                FF_ENTRY[FF_Flag].bLinearFreeFall = FALSE; //this is NOT a freefall event based on previous 
                                                FF_ENTRY[FF_Flag].ucFFstate = 0;
                                            }
                                        }
                                        else 
                                        {
                                            FF_ENTRY[FF_Flag].bLinearFreeFall = FALSE; //this is NOT a freefall event based on previous 
                                            if (bFFall_set)
                                            {
                                                FF_ENTRY[FF_Flag].ucFFstate = 1;
                                            }
                                            else
                                            {
                                                FF_ENTRY[FF_Flag].ucFFstate = 0;
                                            }
                                        }							
                                    }
                                    FF_ENTRY[FF_Flag].bImpact = bImpact_set; //indicate impact
                                }

                                FF_ENTRY[FF_Flag].dFF_X = (int)wDataX * dMult; //log the data point
                                FF_ENTRY[FF_Flag].dFF_Y = (int)wDataY * dMult;
                                FF_ENTRY[FF_Flag].dFF_Z = (int)wDataZ * dMult;

                                FF_Flag++;
                                bIn_FF_flag = TRUE;                 //currently in free-fall
                            }				
                        }        
                    }							

                    if (FF_Flag >= MAX_FF_ARRAY)                    //we collected maximum amount of allowable data 
                        break;

                    if (bImpact_set && FF_Flag)						//ignore readings before getting 1st freefall indication
                        break;

                    if ( isPressed() == _SW_S4_RST )                //button S4 to exit
                    {
                        bS4_Pressed = TRUE;                         //user aborted operation                      
                        break;
                    }
                }

                //handle case of pending-unread data
                if (bFF_INT)								//check initial polarity of HIDI2C_HOST_INT on RE8/INT1 							 
                {
                    if ( HID_I2C_completion())				//if HID_INT signal stuck low, need to read un-read MM7150 data to clear it
                        return HID_INT_FAIL;
                }
                //Disable the Accelerometer on exit
                if ( (ucRetStat = HOST_SF_LIB_VREG_read(VREG_SHC, &wRMWdata)) == SUCCESS )  // Read data from the Sensor Hub Configuration register
                {
                    if ( (ucRetStat = HOST_SF_LIB_VREG_write(VREG_SHC, wRMWdata & ~VREG_SHC_ACC_EN)) ) // Disable the accelerometer (clear the ACC_EN bit)
                        error_handler("VRWr",VREG_SHC, ucRetStat);  //error display
                }
                else
                    error_handler("VRRd",VREG_SHC,ucRetStat);       //error display


                if (FF_Flag)                                        //there's data events collected, output them now that we're done collecting data
                {	
                    for (i = 0; i < FF_Flag; i++ )
                    {
                        uart_put_data(FF_ENTRY[i].dFF_X, FF_ENTRY[i].dFF_Y, FF_ENTRY[i].dFF_Z,0, 3, FALSE); // Send data to uart

                        sprintf ( u2out_buf, "  @ %d msec ",(UINT16)FF_ENTRY[i].usMs_time_stamp);
                        u2out(u2out_buf);

                        if (FF_ENTRY[i].bLinearFreeFall)
                        {
                            u2out(" ***Free-falling \n\r"); 
                        }
                        else if (FF_ENTRY[i].bImpact)
                        {
                            u2out(" ***Impact \n\r"); 
                        }
                        else u2out("\n\r");
                    }                   
                }
						
                if (!bS4_Pressed)                                   //user hasn't bailed-out prematurely
                {
                    u2out("\n\r Press S4 button to exit this mode \n\r"); 
                    while ( isPressed() != _SW_S4_RST );            //wait here for S4                                    
                }
                //turn off LED indicators
                PORTA &= ~(LED_D09 | LED_D08);

                break;

            default: break;
        }
    }
    return (EXIT_SUCCESS);
}

/** display_menu
* @note	Sensor type selection menu displayed to LCD and UART2 
* @param 
* @return curr_row pointer value for menu item selected 
*/ 
UINT8 display_menu(void)
{
    int iCurr_row = 1;
    int iPrev_row = iCurr_row - 1;
    UINT8 ucButton;
    char menu[MENU_ITEMS][MENU_STR_LEN] = { "MM7150 Demo v141\n\r\0", 
                                            "  Select mode:  \n\r\0",
                                            "  ACCELEROMETER \n\r\0",
                                            "  GYROMETER     \n\r\0",
                                            "  COMPASS       \n\r\0",
                                            "  ORIENTATION   \n\r\0",
                                            "  INCLINOMETER  \n\r\0",
                                            "  RAW ACCELEROM \n\r\0",
                                            "  RAW MAGNETO   \n\r\0",
                                            "  RAW GYRO      \n\r\0",
                                            "  Sleep         \n\r\0",
                                            "  Wake          \n\r\0",
                                            "  Flash Update  \n\r\0",
                                            "  FlshCnfgUpdate\n\r\0",
                                            "  Calibration   \n\r\0",
                                            "  1x Calibrate  \n\r\0",
                                            "  FREEFALL      \n\r\0",
                                            "                \n\r\0"};
    
	
    while(1)                                                        //menu loop
    {
        LCD_ClearScreen();                                          // Clear LCD screen
        LCD_PutString(menu[iPrev_row], 16);                         // output string to 1st row of LCD
        LCD_PutString(menu[iCurr_row], 16);                         // output string to 2nd row of LCD
        u2out(menu[iCurr_row]);                                     // output strings to COM port thru UART2

        while(1)                                                    // wait for a button to be pressed
        {                                                           // NOTE: S5 has no interrupt associated with it so this "while loop" is necessary
 
            if (SW_S5_LOW)
                PB_EVT = _SW_S5_SLCT;                               // If the select button (which has no interrupt) is brought low (pressed)

            ucButton = isPressed();                                 //check for debounced button press
            
            if (ucButton != 0xFF )                                  // if any button pressed
                break;	

        }

        if ((ucButton == _SW_S5_SLCT) && (iCurr_row >= MIN_MENU_SLCT)) // selection has been made so break the menu loop
            break;

        if (ucButton == _SW_S6_DWN)                                 // If the down button was pressed
        {
            iPrev_row++;                                            //down button increments the row pointers
            iCurr_row++;

            if (iPrev_row >= MIN_MENU_SLCT)                         // Clear the selection symbol from the previous menu selection
                menu[iPrev_row][0] = ' ';

            if (iCurr_row >= MIN_MENU_SLCT && iCurr_row <= MAX_MENU_SLCT) // Set a selection indicator for the next item in the menu
                menu[iCurr_row][0] = '>';

            if (iPrev_row == MAX_MENU_SLCT)
            {
                menu[iPrev_row][0] = ' ';
                iPrev_row = 1;
                iCurr_row = iPrev_row + 1;
                menu[iCurr_row][0] = '>';
            }                 
        }                                                           //end of DOWN button condition

        if (ucButton == _SW_S3_UP)                                  // If the UP button was pressed
        {
            iPrev_row--;                                            //up button decrements the row pointers
            iCurr_row--;

            if ( (iPrev_row == 0) || (iCurr_row == 0) )				// If we are at the beginning of the menu, loop to the end of the menu
            {
                iPrev_row = MAX_MENU_SLCT - 1;                      //adjust row pointers
                iCurr_row = MAX_MENU_SLCT;
            }

            if (iPrev_row >= MIN_MENU_SLCT)                         // Clear the selection symbol from the previous menu selection
                menu[iCurr_row+1][0] = ' ';
            
            if(iCurr_row >= MIN_MENU_SLCT && iCurr_row <=MAX_MENU_SLCT)
                menu[iCurr_row][0] = '>';
        }                                                           //end of UP button condition
        
        if (ucButton == _SW_S4_RST)                                 //Reset-return to main menu button pressed
        {
            ucButton = 0xFF;                                        //nullify the button press
            iCurr_row = 1;                                          //reset row parameters to initial values
            iPrev_row = iCurr_row - 1;
        }	

    }                                                               //end of menu loop
        
    return ((UINT8)iCurr_row);                                      //return pointer value for the menu item selected
}

