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
/** @file  flashupdate.c
*   Flash Update functions 
*****************************************************************************
*   MM7150 with Explorer 16 Development Board Demo and Sample Code system file
*
*   Company : Microchip Technology Inc.
*
*   File name : flashupdate.c
*
*   Summary : Module for MM7150 Sample Code demo code which implements 
*           Flash Update process 
*
*   Functions : flash_info_command
*               flash_write_command
*               flash_verify_command
*               flash_read_sector_command
*               flash_write_sector_command               
*
*   Revisions : 0.1 2-04-15 C21674  - Initial version created
*               0.2 5-02-15 c21674  - added sector write/read commands
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
extern HID_DESCRIPTOR_TABLE HID_FIELD;                              //structure for HID Descriptor


UINT16 seq_number = 0;
UINT32 image_crc = 0;
BOOL img_wr_done = FALSE;


char ErrorString[][50] =                                            //List of errors returned by SSC7150 firmware
{
	"SPI Communication failure",
	"Flash Protect Error",
	"Flash Write Error",
	"Flash Read Error",
	"Decrypt Signature Error",
	"Packet Framing Error",
	"Packet Checksum Error",
	"Header verification Error",
	"Verification Failure",
	"Command Timeout",
	"Wrong SP Parmeters",
	"Wrong Flash parameters",
	"TAG Write Error",
	"Invalid TAGs \nPlease verify the input file",
	"Image Length Error",
	"Invalid CRC Error",
	"Invalid Flash Address",
	"Invalid Data Length",
	"Unknown Error, Error code out of range"
};

//*****************************************************************************
//*****************************************************************************
//  Section : Code
//*****************************************************************************
//*****************************************************************************

/** flash_info_command
* @note	Implements flash info(+ reset) command during flash update
* @param ucCmdDatbuf pointer to data to be sent as part of flash info command
* @param cmd command byte for reset or bootloader_info sub-command
* @return completion_status 0=SUCCESS, failcode: 0x17=SET_FEAT_FAIL,0x16=GET_FEAT_FAIL, 0x41=FLSH_INFO_ERR
*/
UINT8 flash_info_command(UINT8 *ucCmdDatbuf, UINT8 cmd)
{
   
     if (!ucCmdDatbuf)                                              //sanity check on the buffer
        return FLSH_INFO_ERR;

    /* Form the SET FEAT packet */
    ucCmdDatbuf[0] = FLASH_UPDATE_RPT_ID;                           // set the report id
    ucCmdDatbuf[1] = 0x12;                                          // Data length LSB
    ucCmdDatbuf[2] = 0;                                             // Data length MSB
    ucCmdDatbuf[3] = FLASH_UPDATE_RPT_ID;                           // Report ID
    ucCmdDatbuf[4] = FLASH_INFO_CMD;                                // flash update command
    
    ucCmdDatbuf[5] = cmd;                                           //Reset or BtLdr Info command

    if ( HOST_SF_LIB_HID_Set_Report(SET_RPT_FEAT, ucCmdDatbuf, ucCmdDatbuf[1]))  //send parameters to SSC7150 via Set Feature command packet
        return SET_FEAT_FAIL;

    //Get the response from the device    
    ucCmdDatbuf[0] = FLASH_UPDATE_RPT_ID;                           // set the report id
    
    if ( HOST_SF_LIB_HID_Get_Report(GET_RPT_FEAT, ucCmdDatbuf, 0) )  //get response parameters from SSC7150 via Get Feature 
        return GET_FEAT_FAIL;

        
    if (ucCmdDatbuf[9] == TRUE)                                     // Check the status code in returned packet
        return SUCCESS;

    else
    {
        // If any error
        memcpy(ucCmdDatbuf, ErrorString[ucCmdDatbuf[10] - 0xA1], 50);
        return FLSH_INFO_ERR;
    }        
}

/** flash_write_command
* @note	Implements flash write command during flash update
* @param ucCmdDatbuf data buffer to write to device 
* @param cmd command write/sector write to execute
* @return completion_status 0=SUCCESS, failcode: 0x19=SET_RPT_FAIL,0x41=FLSH_WRITE_ERR
*/
UINT8 flash_write_command(UINT8 *ucCmdDatbuf, UINT8 cmd)
{
    UINT8 i, length;    
    UINT16 chksum = 0;
	UINT8 hid_i2c_rpt_buf[BUF_150];    

    if (!ucCmdDatbuf)                                               //sanity check on the buffer
        return FLSH_WRITE_ERR;

    seq_number++;
    
    // Form the SET FEAT packet 
    hid_i2c_rpt_buf[0] = FLASH_UPDATE_RPT_ID;                       // set the report id
    hid_i2c_rpt_buf[1] = 0x8F;                                      // Data length LSB
    hid_i2c_rpt_buf[2] = 0;                                         // Data length MSB
    hid_i2c_rpt_buf[3] = FLASH_UPDATE_RPT_ID;                       // Report ID
    hid_i2c_rpt_buf[4] = cmd;										//flash update write (or sector write) command
    hid_i2c_rpt_buf[5] = seq_number & 0x00FF;                       // sequence number LSB
    hid_i2c_rpt_buf[6] = (seq_number & 0xFF00) >> 8;                // sequence number MSB
    hid_i2c_rpt_buf[7] = 0;
    hid_i2c_rpt_buf[8] = 0;

    // byte 7 & 8 are checksum values, data starts from byte 9 
    memcpy(&hid_i2c_rpt_buf[FLSH_WR_DATA_INDEX], ucCmdDatbuf, PACKET_LENGTH);

    length = FLSH_WR_DATA_INDEX + PACKET_LENGTH;

    // For first packet we need to add total packet numbers 
    if (seq_number == 1)
    {
        hid_i2c_rpt_buf[++length] = TOTAL_64BYTE_PKTS & 0x00FF;         // sequence number LSB
        hid_i2c_rpt_buf[++length] = (TOTAL_64BYTE_PKTS & 0xFF00) >> 8;  // sequence number MSB
    
        // First packet contains CRC of unencrypted image we need to store it for verification purpose in future
        image_crc = ucCmdDatbuf[19];
        image_crc = ((image_crc << 8) | ucCmdDatbuf[18]);
        image_crc = ((image_crc << 8) | ucCmdDatbuf[17]);
        image_crc = ((image_crc << 8) | ucCmdDatbuf[16]);
    }

    // Calculate checksum
    for(i=3; i<=(length); i++)
    {
        chksum += hid_i2c_rpt_buf[i];
    }

    // Append checksum
    hid_i2c_rpt_buf[7] = chksum & 0x00FF;                           // checkkum LSB
    hid_i2c_rpt_buf[8] = (chksum & 0xFF00) >> 8;                    // checksum MSB

    // Send SET OUT Report to module 
    if (HOST_SF_LIB_HID_Set_Report(SET_RPT_OUT, hid_i2c_rpt_buf, length))    
    {
        memcpy(ucCmdDatbuf, "SET_RPT_FAIL", 12);                    //return error string
        return SET_RPT_FAIL;
    }

    //Get the response from the device   
    hid_i2c_rpt_buf[0] = FLASH_UPDATE_RPT_ID;                       // set the report id again as it is overwritten by last transaction

    
    if ( HOST_SF_LIB_HID_Get_Report(GET_RPT_FEAT, hid_i2c_rpt_buf,0))
    {
        memcpy(ucCmdDatbuf, "SET_RPT2_FAIL", 12);                   //return error string
        return SET_RPT_FAIL;
    }

    // Check the status code 
    if (hid_i2c_rpt_buf[5] == TRUE)    
    {
        sprintf((char *)ucCmdDatbuf, "%d", seq_number);
        
        if (seq_number >= (TOTAL_64BYTE_PKTS/2))                    // If all packets sent 
            img_wr_done = TRUE;
        return SUCCESS;
    }
    else
    {           //error
        seq_number = 1;
        memcpy(ucCmdDatbuf, ErrorString[hid_i2c_rpt_buf[6] - 0xA1], 50); //get error msg from F/W
        return FLSH_WRITE_ERR;
    } 
}


/** flash_verify_command
* @note	Implements flash verify command during flash update
* @param ucCmdDatbuf contains the CRC received from image
* @return completion_status 0=SUCCESS, failcode: 0x1C=HID_GET_RPT_INPT_FAIL,0x19=SET_RPT_FAIL,0x43=FLSH_VERIFY_ERR
*/
UINT8 flash_verify_command(UINT8 *ucCmdDatbuf)
{
    UINT32 prgm_crc;
	UINT8 hid_i2c_rpt_buf[BUF_150];    
    
   
    // Form the SET FEAT packet 
    hid_i2c_rpt_buf[0] = FLASH_UPDATE_RPT_ID;                       // set the report id
    hid_i2c_rpt_buf[1] = 18;                                        // Data length lower byte
    hid_i2c_rpt_buf[2] = 0;                                         // Data length upper byte
    hid_i2c_rpt_buf[3] = FLASH_UPDATE_RPT_ID;                       // Report ID
    hid_i2c_rpt_buf[4] = FLASH_READ_CMD;                            // Actual flash update command

    if (HOST_SF_LIB_HID_Set_Report(SET_RPT_FEAT, hid_i2c_rpt_buf, 5))
      return SET_RPT_FAIL;

    hid_i2c_rpt_buf[0] = FLASH_UPDATE_RPT_ID;                       // set the report id again as it is overwritten by last transaction

    if ( HOST_SF_LIB_HID_Get_Report(GET_RRT_INPT, hid_i2c_rpt_buf,0))
        return HID_GET_RPT_INPT_FAIL;

    if (hid_i2c_rpt_buf[5] == TRUE)
    {
        prgm_crc = hid_i2c_rpt_buf[12];
        prgm_crc = ((prgm_crc << 8) | hid_i2c_rpt_buf[11]);
        prgm_crc = ((prgm_crc << 8) | hid_i2c_rpt_buf[10]);
        prgm_crc = ((prgm_crc << 8) | hid_i2c_rpt_buf[9]);

        if (image_crc == prgm_crc)
            return SUCCESS;
        else
            return FLSH_CRC_ERR;                                    //CRC_ERROR
    }
    
    else
    {
        // If any error
        memcpy(ucCmdDatbuf, ErrorString[hid_i2c_rpt_buf[10] - 0xA1], 50);
        return FLSH_VERIFY_ERR;
    }        
}

/** flash_read_sector_command
* @note	Implements flash sector verify command during flash configuration update
* @param ucCmdDatbuf contains the CRC received from image
* @return completion_status 0=SUCCESS, failcode: 1Ch=HID_GET_RPT_INPT_FAIL,19h=SET_RPT_FAIL,43h=FLSH_VERIFY_ERR,44h=FLSH_CRC_ERR
*/
UINT8 flash_read_sector_command(UINT8 *ucCmdDatbuf)
{
    UINT8 i, length, seq, pkt_size, cmd_cnt = 0;
	UINT8 hid_i2c_rpt_buf[BUF_150]={0};    
 	UINT8 read_pkt_buf[BUF_150]={0};    
   
	
	seq = 1;
	length = 18;
    // Form the SET FEAT packet 
    hid_i2c_rpt_buf[0] = FLASH_UPDATE_RPT_ID;                       // set the report id
    hid_i2c_rpt_buf[1] = length;                                    // Data length lower byte
    hid_i2c_rpt_buf[2] = 0;                                         // Data length upper byte
    hid_i2c_rpt_buf[3] = FLASH_UPDATE_RPT_ID;                       // Report ID
    hid_i2c_rpt_buf[4] = FLASH_READ_SECTOR_CMD;                     // Actual flash sector read command
	hid_i2c_rpt_buf[5] = seq;	//LSB of sequence number
	hid_i2c_rpt_buf[6] = 0; //MSB of sequence number
	hid_i2c_rpt_buf[7] = 1; //status of last packet
	hid_i2c_rpt_buf[8] = 0;	//LSB of length of data to be read 
	hid_i2c_rpt_buf[9] = 4;	//MSB of length of data to be read 
	hid_i2c_rpt_buf[10] = 0x00;	//LSB of config sector address
	hid_i2c_rpt_buf[11] = 0x6C;	//LSB+1 config addr
	hid_i2c_rpt_buf[12] = 0x00;	//LSB+2 config addr
	hid_i2c_rpt_buf[13] = 0x9D;	//MSB config addr


    if (HOST_SF_LIB_HID_Set_Report(SET_RPT_FEAT, hid_i2c_rpt_buf, length))
      return SET_RPT_FAIL;

    read_pkt_buf[0] = FLASH_UPDATE_RPT_ID;                       // set the report id again as it is overwritten by last transaction

    if ( HOST_SF_LIB_HID_Get_Report(GET_RRT_INPT, read_pkt_buf,0))
        return HID_GET_RPT_INPT_FAIL;

	//confirm successful read sector completion
	if (read_pkt_buf[2] == FLASH_READ_SECTOR_CMD)
	{
		if (read_pkt_buf[5] != TRUE)
			return(FLSH_VERIFY_ERR);
	}
	else return FLSH_VERIFY_ERR;

	//compare to ucCmdDatbuf contents
	pkt_size = read_pkt_buf[0]-2;									//throw away last 2 bytes of read packet
    
	if (!pkt_size)													//size of packet in byte 0
        return FLSH_VERIFY_ERR;
	
    for (i = FLSH_WR_DATA_INDEX; i < pkt_size; i++)
	{
		if (ucCmdDatbuf[cmd_cnt++] != read_pkt_buf[i])
			return FLSH_VERIFY_ERR;
	}
	
	//get next packet for this sector
	seq++;
    hid_i2c_rpt_buf[0] = FLASH_UPDATE_RPT_ID;                       // set the report id
    hid_i2c_rpt_buf[1] = length;                                    // Data length lower byte
    hid_i2c_rpt_buf[2] = 0;                                         // Data length upper byte
    hid_i2c_rpt_buf[3] = FLASH_UPDATE_RPT_ID;                       // Report ID
    hid_i2c_rpt_buf[4] = FLASH_READ_SECTOR_CMD;                     // Actual flash sector read command
	hid_i2c_rpt_buf[5] = seq;	//LSB of sequence number
	hid_i2c_rpt_buf[6] = 0; //MSB of sequence number
	hid_i2c_rpt_buf[7] = 1; //status of last packet
	hid_i2c_rpt_buf[8] = 0;	//LSB of length of data to be read 
	hid_i2c_rpt_buf[9] = 0;	//MSB of length of data to be read 
	hid_i2c_rpt_buf[10] = 0;	//LSB of config sector address
	hid_i2c_rpt_buf[11] = 0;	//LSB+1 config addr
	hid_i2c_rpt_buf[12] = 0;	//LSB+2 config addr
	hid_i2c_rpt_buf[13] = 0;	//MSB config addr

    if (HOST_SF_LIB_HID_Set_Report(SET_RPT_FEAT, hid_i2c_rpt_buf, length))
      return SET_RPT_FAIL;

    read_pkt_buf[0] = FLASH_UPDATE_RPT_ID;                          // set the report id again as it is overwritten by last transaction

    if ( HOST_SF_LIB_HID_Get_Report(GET_RRT_INPT, read_pkt_buf,0))
        return HID_GET_RPT_INPT_FAIL;

	//confirm successful read sector completion
	if (read_pkt_buf[2] == FLASH_READ_SECTOR_CMD)
	{
		if (read_pkt_buf[5] != TRUE)
			return(FLSH_VERIFY_ERR);
	}
	else return FLSH_VERIFY_ERR;

	//compare to ucCmdDatbuf contents
	pkt_size = read_pkt_buf[0]-2;
 
    if (!pkt_size)                                                  //size of packet in byte 0
        return FLSH_VERIFY_ERR;
	
    for (i = FLSH_WR_DATA_INDEX; i < read_pkt_buf[0]; i++)
	{
		if (ucCmdDatbuf[cmd_cnt++] != read_pkt_buf[i])
			return FLSH_VERIFY_ERR;
	}
	
    return(SUCCESS);
}

/** flash_write_sector_command
* @note	Implements flash write command during flash update
* @param ucCmdDatbuf data buffer to write to device 
* @param cmd command write/sector write to execute
* @return completion_status 0=SUCCESS, failcode: 19h=SET_RPT_FAIL,1Ch=HID_GET_RPT_INPT_FAIL,41h=FLSH_WRITE_ERR, 
*/
UINT8 flash_write_sector_command(UINT8 *ucCmdDatbuf, UINT8 cmd)
{
    UINT8 i, length;    
    UINT16 chksum;
	UINT8 hid_i2c_rpt_buf[BUF_150]={0};    


    if (!ucCmdDatbuf)                                               //sanity check on the buffer
        return FLSH_WRITE_ERR;

    seq_number++;
    length = 143;                                                   //packet length expected by SSC7150 F/W
    // Form the SET FEAT packet 
    hid_i2c_rpt_buf[0] = FLASH_UPDATE_RPT_ID;                       // set the report id
    hid_i2c_rpt_buf[1] = length;                                    // Data length LSB
    hid_i2c_rpt_buf[2] = 0;                                         // Data length MSB
    hid_i2c_rpt_buf[3] = FLASH_UPDATE_RPT_ID;                       // Report ID
    hid_i2c_rpt_buf[4] = cmd;										//flash update write (or sector write) command
    hid_i2c_rpt_buf[5] = seq_number & 0x00FF;                       // sequence number LSB
    hid_i2c_rpt_buf[6] = (seq_number & 0xFF00) >> 8;                // sequence number MSB
    hid_i2c_rpt_buf[7] = 0;			                                //crc LSB - initially = 0, calculate crc from byte[3] to end 
    hid_i2c_rpt_buf[8] = 0;			                                //crc MSB

    // byte 7 & 8 are checksum values for this entire transaction, data starts from byte 9
	//however, the config binary has its own crc that we need to calculate first and put into bytes 0x4C(LSB) 0x4D(MSB)
    
	
	//clear any existing crc in bytes 0x4C/0x4D
	ucCmdDatbuf[CFG_CRC_LSB] = 0;
	ucCmdDatbuf[CFG_CRC_MSB] = 0;
    
	// Calculate checksum of config binary ONLY
	chksum = 0;

	for(i=0; i<CFG_CRC_LSB; i++)
    {
        chksum += ucCmdDatbuf[i];
    }

	//update the crc for this config data
	ucCmdDatbuf[CFG_CRC_LSB] = chksum & 0x00FF;;
	ucCmdDatbuf[CFG_CRC_MSB] = (chksum & 0xFF00) >> 8;
	
	//transfer the configuration binary packet that we downloaded and updated crc
    for (i = 0; i < 128; i++)
    {
        hid_i2c_rpt_buf[FLSH_WR_DATA_INDEX + i] = ucCmdDatbuf[i];
    }

	//let's put some known values into the packet
	hid_i2c_rpt_buf[PAD_FF] = 0xFF;
	hid_i2c_rpt_buf[PACKET_LSB] = 2;
	hid_i2c_rpt_buf[PACKET_MSB] = 0;
	hid_i2c_rpt_buf[CFG_SECT_LSB] = 0x00;
	hid_i2c_rpt_buf[CFG_SECT_LSB_1] = 0x6C;
	hid_i2c_rpt_buf[CFG_SECT_LSB_2] = 0x0;
	hid_i2c_rpt_buf[CFG_SECT_MSB] = 0x9D;


	//now calculate crc for the transaction packet starting with hid_i2c_rpt_buf[3]
	chksum = 0;

	for(i = 3; i < (length+1); i++)
    {
        chksum += hid_i2c_rpt_buf[i];
    }
	
    //update the crc for the entire packet
	hid_i2c_rpt_buf[7] = chksum & 0x00FF;
	hid_i2c_rpt_buf[8] = (chksum & 0xFF00) >> 8;


    if (HOST_SF_LIB_HID_Set_Report(SET_RPT_OUT, hid_i2c_rpt_buf, length))
      return SET_RPT_FAIL;

    hid_i2c_rpt_buf[0] = FLASH_UPDATE_RPT_ID;                       // set the report id again as it is overwritten by last transaction

    if ( HOST_SF_LIB_HID_Get_Report(GET_RPT_FEAT, hid_i2c_rpt_buf,0))
        return HID_GET_RPT_INPT_FAIL;

	//confirm successful write sector completion
	if (hid_i2c_rpt_buf[2] == FLASH_WRITE_SECTOR_CMD)
	{
		if (hid_i2c_rpt_buf[5] == TRUE)
			return(SUCCESS);
	}
	else return FLSH_WRITE_ERR;
    
    return (SUCCESS);
}
