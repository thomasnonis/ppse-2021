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
/** @file  utils.c
*   Miscellaneous Utility functions 
*****************************************************************************
*   MM7150 with Explorer 16 Development Board Demo and Sample Code system file
*
*   Company : Microchip Technology Inc.
*
*   File name : utils.c
*
*   Summary : Module for MM7150 Sample Code demo code 
*
*   Functions : HID_I2C_completion
*               diag_info_command
*               display_FW_buildnum
*               cnv_neg
*
*   Revisions : 0.2 10-6-16 A21444 - use MCC
*             : 0.1 7-9-15 C21674  - Initial version created
*               
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



//*****************************************************************************
//*****************************************************************************
//  Section : Code
//*****************************************************************************
//*****************************************************************************

/** HID_I2C_completion
* @note	Check for HID_I2C_INT stuck low, issue I2C write-read command to read any pending data
* @return completion_status 0=SUCCESS, failure= non-zero
*/
UINT8 HID_I2C_completion()
{
    UINT8 ucRx_data[BUF_40];
    UINT8 ucRet = FALSE;

    if (I2C_ALERT_GetValue()==0)
    {
        ucRet = i2c_cmd_WrRd (READ,                                 // Read the data from the SSC7150
                            0,                                      //num of cmd bytes
                            0,                                      //cmd buf (ignored)
                            BYTE_ADJ_VAL,                           //num of bytes to read
                            ucRx_data,                              //recv buf 
                            TRUE);                                  //actual # of bytes SSC7150 returns is in 1st two bytes of read packet, this flag(=TRUE) means "use the 1st two bytes as the actual read packet length"

    }
    return (ucRet);
}

/** display_FW_buildnum
* @note	Uses FlashInfo command to obtain SSC7150 FW build string 
* @return completion_status 0=SUCCESS, failcode: 0x41=FLSH_INFO_ERR
*/
UINT8 display_FW_buildnum()
{
    UINT8 local_buf [BUF_70] = {0};
	

    if (flash_info_command(local_buf, FLASH_GET_INFO_CMD))			//issue firmware info command to SSC7150 to get firmware revision number
    {
		u2out("\n\n\r***Firmware Revision Info failed*** \n\n\r");
		return FLSH_INFO_ERR;
	}
	else
	{
		u2out("\n\r    SSC7150 F/W Revision : ");
        u2out((char*)&local_buf[FW_BUILD_STR_STRT]);				//point to start of FW build string in buffer
        u2out("\n\n\r");
	}

	return SUCCESS;
}

/** diag_info_command
* @note	Implements Diagnostic Info command 
* @param ucCmdDatbuf pointer to data to be sent as part of diag command
* @return completion_status 0=SUCCESS, failcode: 16h=GET_FEAT_FAIL, 41h=FLSH_INFO_ERR, 51h=DIAG_INFO_ERR
*/
UINT8 diag_info_command(UINT8 *ucCmdDatbuf)
{
   
	if (!ucCmdDatbuf)                                              //sanity check on the buffer
        return DIAG_INFO_ERR;

    ucCmdDatbuf[0] = DIAG_RPT_ID;									// set the report id

	//Get the response from the device       
    if ( HOST_SF_LIB_HID_Get_Report(GET_RPT_FEAT, ucCmdDatbuf, 0) )  //get response parameters from SSC7150 via Get Feature 
        return GET_FEAT_FAIL;
     
    if (ucCmdDatbuf[0] == DIAG_INFO_SIZE)                           // Check the expected size in returned packet
        return SUCCESS;

    else
        return FLSH_INFO_ERR;
}
/** cnv_neg
* @note	Implements negative number conversion w/o using math library
* @param usVal pointer to value to be converted to positive number 
*/
void cnv_neg ( UINT16 *usVal )
{
	UINT16 usTmp;

	if (*usVal & 0x8000)											// is it a negative value?
	{
		usTmp = ~(*usVal);
		*usVal = usTmp + 1;
	}
}
