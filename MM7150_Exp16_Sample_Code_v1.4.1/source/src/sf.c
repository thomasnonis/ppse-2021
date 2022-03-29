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
/** @file  sf.c
*   Sensor functions, HID protocol calls, HID Report Descriptor parser functions 
*****************************************************************************
*   MM7150 with Explorer 16 Development Board Sample Code system file
*
*   Company : Microchip Technology Inc.
*
*   File name : sf.c
*
*   Summary : Module for MM7150 demo code which implements 
*           HID protocols and parses Report Descriptor 
*
*   Functions : report_parse
*               hid_i2c_descriptor_handler
*               hid_i2c_cmd_process            
*               ret_exponent 
*
*   Revisions : 0.4 2-04-15 C21674 - added HOST_SF_LIB_HID_Set_Report
*                                  - added HOST_SF_LIB_HID_Get_Report
*             : 0.3 9-30-14 C21674 - error handling, SET Feature byte length updated from Get Feature
*             : 0.2 9-4-14 - (removed backward compatibilty) SSC7150 build0600 and later
*             : 0.1 8-4-14 C21674  - added backward compatibility for SSC7150 F/W changes 
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

SF_SENSOR SENSOR[NUM_SENS];                                         //structure of individual sensors
SF_SENSOR TEMP_SF[NUM_SENS];                                        //temporary structure of individual sensors for "bookkeeping"
HID_DESCRIPTOR_TABLE HID_FIELD;                                     //structure for HID Descriptor 
UINT8 HID_DESC[HID_CNFG_LEN];                                       //buffer to store the HID config descriptor table
UINT8 RPT_DESC[HID_RPT_LEN];                                        //buffer to store HID Report Descriptor table
UINT8 RAW_SENSOR_CNT = 0;                                           //for backward compatibility to F/W with all-in-one RAW sensor data

extern SF_VREGS VREGS;                                              //structure containing the VREG registers 
extern volatile BOOL TIMER_1MS_FLG;                                 // Timer counter variable 
extern volatile BOOL EC_DATA_AVAIL;                                 //HIDI2_HOST_INT indicates EC data available


//*****************************************************************************
//*****************************************************************************
//  Section : Code
//*****************************************************************************
//*****************************************************************************

/** hid_i2c_descriptor_handler
* @note	Retrieve either the descriptor or report tables from the SSC7150 device Note: call GET_HID_DESC 1st
* @param ucCmd_req command to execute (GET_HID_DESC/GET_RPT_DESC)
* @return completion_status 0=SUCCESS, 0x10=ID_FAIL, 0x11=HID_DESC_FAIL, 0x12=RPT_DESC_FAIL, 0x14=REP_PARS_FAIL
*/
UINT16 hid_i2c_descriptor_handler(UINT8 ucCmd_req)
{
    UINT8 ucRet = FALSE;
    UINT8 ucTx_data[BUF_40];                                        // buffer for general writes to i2c slave

    
    switch(ucCmd_req)
    {  
        case GET_HID_DESC:                                          //read the HID Config Descriptor from SSC7150
            ucTx_data[0] = HID_DESC_CMD_LSB;                        // HID descriptor table request is 0x00 01
            ucTx_data[1] = HID_DESC_CMD_MSB;

            ucRet = i2c_cmd_WrRd ( WR_RD,                           //read the HID Config Descriptor from SSC7150
                                DESC_CMD_LEN,                       //num of cmd bytes
                                ucTx_data,                          //cmd buf 
                                HID_CNFG_LEN,                       //num of bytes to read
                                HID_DESC,                           //recv buf
                                FALSE);                             //flag indicating that we specified the number of bytes to read explicitly
            if (ucRet)
                return HID_DESC_FAIL;                               //read HID Config Descriptor failed
     
            HID_FIELD.wHIDDescLen = ( (HID_DESC[1] << BYTE_SHIFT) | HID_DESC[0] );  // As an error check, parse out descriptor table length
                
            if (HID_FIELD.wHIDDescLen != HID_CNFG_LEN)              // If descriptor table length != 30 there is a problem with the transmission
                return HID_DESC_FAIL;
                
            HID_FIELD.wRepDescLen = ( (HID_DESC[5] << BYTE_SHIFT) | HID_DESC[4] );  // Parse out report descriptor length and store in appropriate variable
                
            if ( !HID_FIELD.wRepDescLen || HID_FIELD.wRepDescLen == 0xFFFF || (HID_FIELD.wRepDescLen > HID_RPT_LEN) ) // Check to see if the report descriptor size is valid
                return HID_DESC_FAIL;
                
            HID_FIELD.wRepDescReg = ( (HID_DESC[6] << BYTE_SHIFT) | HID_DESC[7] ); // Parse out report descriptor register and store in appropriate variable

            HID_FIELD.wCmdReg = ( (HID_DESC[17] << BYTE_SHIFT) | HID_DESC[16] ); // Parse out command register and store in appropriate variable

            HID_FIELD.wDatReg = ( (HID_DESC[19] << BYTE_SHIFT) | HID_DESC[18] ); // Parse out data register and store in appropriate variable

            if ( !( HID_FIELD.wVenID = ( (HID_DESC[21] << BYTE_SHIFT) | HID_DESC[20] )) ) // Parse out vendor ID and store in appropriate variable
                return ID_FAIL;

            if ( !( HID_FIELD.wProdID = ( (HID_DESC[23] << BYTE_SHIFT) | HID_DESC[22] )) ) // Parse out product ID and store in appropriate variable
                return ID_FAIL;
                
            HID_FIELD.wVerID = ( (HID_DESC[25] << BYTE_SHIFT) | HID_DESC[24] ); // Parse out version ID and store in appropriate variable
             
            VREGS.IDs.PID = HID_FIELD.wProdID;                      // Update virtual register fields
            VREGS.IDs.VID = HID_FIELD.wVenID;
            VREGS.IDs.DID = HID_FIELD.wVerID;

            break;

        case GET_RPT_DESC:                                          //read HID Report Descriptor table from SSC7150
            
            ucTx_data[0] = (HID_FIELD.wRepDescReg >> BYTE_SHIFT);   // Report table request is 0x02 00
            ucTx_data[1] = HID_FIELD.wRepDescReg;

            ucRet = i2c_cmd_WrRd (WR_RD,                            //read the HID Report Descriptor from SSC7150
                                DESC_CMD_LEN,                       //num of cmd bytes
                                ucTx_data,                          //cmd buf
                                HID_FIELD.wRepDescLen,              //num of bytes to read
                                RPT_DESC,                           //recv buf
                                FALSE);                             //flag indicating that we specified the number of bytes to read explicitly
            if (ucRet)
                return RPT_DESC_FAIL;                               // Attempt to acquire the report descriptor table from the HID device
            
            if (report_parse(RPT_DESC))                             // Parse the descriptor for sensor information
                return REP_PARS_FAIL;  
            
            break;
    }

    return SUCCESS;
}


/** hid_i2c_cmd_process
* @note	Send commands or retrieve data to/from SSC7150 device.   Note: calls to GET_HID_DESC/GET_RPT_DESC must already have completed
* @param ucCmdDatbuf ptr to cmd or data buf
* @param ucCmd_req command 
* @param ucReport_id sensor id 
* @return completion_status 0=SUCCESS, failcode: 0x16=GET_FEAT_FAIL, 0x17=SET_FEAT_FAIL, 0x18=RESET_FAIL, 0x19=SET_RPT_FAIL, 0x1A=POWER_ON_FAIL, 0x1B=SLEEP_CMD_FAIL, 0x1C=HID_GET_RPT_INPT_FAIL, 0x1D=HID_GET_RPT_FEAT_FAIL 
*/
UINT8 hid_i2c_cmd_process(UINT8 *ucCmdDatbuf, UINT8 ucCmd_req, UINT8 ucReport_id)
{
    UINT8 ucRetStat;
    UINT8 ucTx_data[BUF_40];                   
    UINT8 ucSensPtr, ucTmpPtr;
    UINT8 ucCmdBufMaxSize = 0;
    UINT16 usTimeout;                           
    UINT16 *usVREGSptr = (UINT16 *)&VREGS;                          // Pointer to individual virtual registers; initialized to start of VREGS struct

    
    switch(ucCmd_req)
    {
        
        case RESET_DEV_CMD:                                         //HID Reset command                      
            ucTx_data[0] = HID_FIELD.wCmdReg;                       //command field bytes from HID config table                    
            ucTx_data[1] = (HID_FIELD.wCmdReg >> BYTE_SHIFT);       
            ucTx_data[2] = RESET_CMD_LSB;                           //HID Reset command opcode low byte              
            ucTx_data[3] = RESET_CMD_MSB;                           //HID Reset command opcode high byte

            // send the reset command to SSC7150
            ucRetStat = i2c_cmd_WrRd (WRITE,                        //WRITE command packet to SSC7150 
                                CMD_LEN,                            //num of cmd bytes 
                                ucTx_data,                          //cmd buf 
                                0,                                  //num of bytes to read             
                                ucCmdDatbuf,                        //recv buf
                                FALSE);                             //flag indicating that we specified the number of bytes to read explicitly
            
            if (ucRetStat != SUCCESS)
                return RESET_FAIL;

            TIMER_1MS_FLG = 0;                                      // Prepare timer1 for counting
            usTimeout = TIMEOUT_5SEC;                               // 5 sec (as per HID spec) timeout for reset command 
            while (usTimeout)                                       // wait up to API spec timeout to respond with EC_DATA avail interrupt   
            {    
                if (EC_DATA_AVAIL)                                  // EC interrupt asserted (data is available)
                    break; 
					
                if (TIMER_1MS_FLG)
                { 
                    TIMER_1MS_FLG = 0;
                    usTimeout--;                                    // 1 msec expired, reduce counter
                }
            }

            if (!usTimeout) 
                return RESET_FAIL;                                  // timeout occured without device responding with interrupt
                
            ucRetStat = i2c_cmd_WrRd (READ,                         // EC_DATA_AVAIL flag was set indicating SSC7150 has data available to be read in response to the RESET CMD
                                    0,                              //num of cmd bytes
                                    ucTx_data,                      //cmd buf
                                    2,                              //num of bytes to read
                                    ucCmdDatbuf,                    //recv buf 
                                    FALSE);                         //flag indicating that we specified the number of bytes to read explicitly
                
            if (ucRetStat != SUCCESS)
                return RESET_FAIL; 
            
            if (ucCmdDatbuf[0] != 0 && ucCmdDatbuf[1] != 0)         // expect 1st two bytes of data packet from SSC7150 in response to RESET command to be "00 00"
                return RESET_FAIL;                                  // invalid data found
 
            VREGS.SHC.reset = VREG_RESET_SUCCESS;                   // Clear the reset VREG to indicate successful 
                   
            break;


        case POWER_ON:      
            ucTx_data[0] = HID_FIELD.wCmdReg;                       //command field bytes from HID config table 
            ucTx_data[1] = (HID_FIELD.wCmdReg >> BYTE_SHIFT);                              
            ucTx_data[2] = POWER_CMD_LSB;                           //HID Power command opcode low  byte for POWER Device ON
            ucTx_data[3] = POWER_CMD_MSB;                           //HID Power command opcode high byte

            ucRetStat = i2c_cmd_WrRd (WRITE,                        // Issue power on command to SSC7150
                                CMD_LEN,                            //num of cmd bytes
                                ucTx_data,                          //cmd buf
                                0,                                  //num of bytes to read
                                ucCmdDatbuf,                        //recv buf
                                FALSE);                             //flag indicating that we specified the number of bytes to read explicitly

            if (ucRetStat != SUCCESS)
                return POWER_ON_FAIL;                               // command failed

            break;


        case SLEEP:         

            ucTx_data[0] = HID_FIELD.wCmdReg;                       //command field bytes from HID config table
            ucTx_data[1] = (HID_FIELD.wCmdReg >> BYTE_SHIFT);            
            ucTx_data[2] = SLEEP_ON_LSB;                            //HID Power command opcode low byte for Device SLEEP
            ucTx_data[3] = POWER_CMD_MSB;                           //HID Power command opcode high byte

            ucRetStat = i2c_cmd_WrRd (WRITE,                        // Issue sleep command to SSC7150
                                CMD_LEN,                            //num of cmd bytes
                                ucTx_data,                          //cmd buf 
                                0,                                  //num of bytes to read
                                ucCmdDatbuf,                        //recv buf
                                FALSE);                             //flag indicating that we specified the number of bytes to read explicitly

            if (ucRetStat != SUCCESS)
                return SLEEP_CMD_FAIL;                              // command failed         
            
            break;

        case HID_GET_RPT_INPT:
			
            ucCmdDatbuf[0] = ucReport_id;                           //the HOST_SF_LIB_HID_Get_Report expects the sensor id in byte[0] of passed buffer
			
            if ( HOST_SF_LIB_HID_Get_Report(GET_RRT_INPT, ucCmdDatbuf, 0) )
                return HID_GET_RPT_FEAT_FAIL;                       // command failed
			 
            break;


        case HID_GET_RPT_FEAT:

            ucCmdDatbuf[0] = ucReport_id;                           //the HOST_SF_LIB_HID_Get_Report expects the sensor id in byte[0] of passed buffer
			
            if ( HOST_SF_LIB_HID_Get_Report(GET_RPT_FEAT, ucCmdDatbuf, 0) )
                return HID_GET_RPT_FEAT_FAIL;                       // command failed

            if ((ucCmdDatbuf[2] == 0) || (ucCmdDatbuf[2] > NUM_SENS) ) // check for valid ID range in feature report
                return HID_GET_RPT_FEAT_FAIL;                       // invalid reportID
                       
            //now parse the parameters in returned report feature based on offsets derived earlier from parsing the HID Report Descriptor
            for (ucSensPtr = 0; ucSensPtr < NUM_SENS; ucSensPtr++)  // traverse through the sensor list looking for matching sesnor id
            {          
                if (SENSOR[ucSensPtr].id == ucReport_id)
                {        
                    usVREGSptr = &VREGS.sensitivity.ACSEN + ucSensPtr; // Set pointer to appropriate sensitivity register (as offset from accelerometer sensitivity VREG02)
                    
                    //retrieve offset pointer to sensitivty parameters for this sensor device (from the Report Descriptor table). Add offset to initial pointer for the actual desired "ACSEN" data within the feature report packet and store this value in VREG register 
                    *usVREGSptr = ( (ucCmdDatbuf[SENSOR[ucSensPtr].SensOffset + GF_SENS_OFFSET_MSB] << BYTE_SHIFT) | ucCmdDatbuf[SENSOR[ucSensPtr].SensOffset + GF_SENS_OFFSET_LSB]); // Use info about offsets to set VREGS to appropriate data
                    
                    usVREGSptr = &VREGS.data_rt.ACDXR + ucSensPtr;  // Set pointer to appropriate data rate register
                    
                    *usVREGSptr = ((ucCmdDatbuf[SENSOR[ucSensPtr].DatRtOffset + GF_DATR_OFFSET_MSB] << BYTE_SHIFT) | ucCmdDatbuf[SENSOR[ucSensPtr].DatRtOffset + GF_DATR_OFFSET_LSB]); //point to appropriate data fields within the feature report and store in VREG reg

                    usVREGSptr = (UINT16 *)&(VREGS.expo.exp1);      //point to appropriate data fields within the feature report and store in VREG reg  NOTE: each exponent VREG holds 4 devices' 4bit exponent value fields
                    if (ucSensPtr > 3)                              // sensor indexes from 4 to 7 = sensors that have unit exponents in the 2nd exponent register (VREG37)
                        usVREGSptr++;                               // Increment ptr to VREG exponent register to be searched

                    if (ucSensPtr > 7)                              // sensor indexes > 7 = sensors that have unit exponents in the 3rd exponent register (VREG38)
                        usVREGSptr++;                               // Increment ptr to VREG exponent register to be searched

                    ucTmpPtr= ucSensPtr % 4;                        // Find the appropriate offset for this sensor's unit exponent

                    *usVREGSptr &= ~(0xF << (4*ucTmpPtr));          // Clear unit exponent data   

                    if (SENSOR[ucSensPtr].DatExp)
                    {
                       *usVREGSptr |= (SENSOR[ucSensPtr].DatExp << (4*ucTmpPtr));  // Set the unit exponent data to the appropriate VREG  
                    }

                    break;              
                }   
            }			

            break;
     
        case HID_SET_RPT_FEAT:
            //for non-Vendor commands, the sensor id is NOT passed in byte[0] to 'HOST_SF_LIB_HID_Set_Report'
            ucCmdBufMaxSize = ucCmdDatbuf[RPT_SIZE_LSB];            //get size of GetReportFeature Packet
            
            if ((ucCmdBufMaxSize == 0) || (ucCmdBufMaxSize == 0xFF)) //is the size reasonable?
                return SET_FEAT_FAIL;
					
            if ( HOST_SF_LIB_HID_Set_Report(SET_RPT_FEAT, ucCmdDatbuf, ucCmdBufMaxSize) )
                return SET_FEAT_FAIL;                               // command failed

            break;

        default:
            break;
    }

    return SUCCESS;
}

/** report_parse
* @note	Parse data retrieved from SSC7150 HID Report Descriptor, calls and populate required local structs
* @param *ucBuf Buffer holding the report descriptor table
* @return completion_status 0=SUCCESS, failcodes: 0x14=REP_PARS_FAIL, 0x15=NO_EOC_FAIL 
*/
UINT8 report_parse(UINT8 *ucBuf)
{
    UINT8 *ucBptr = ucBuf;                                          // HID Descriptor byte pointer
    UINT8 *ucStrt = ucBptr;                                         // Pointer to hold start address for later reference
    UINT8 ucSensptr = 0;                                            // sensor number variable for struct array traverse
    UINT8 ucUsageOffset;                                            // offset of desired HID_USAGE_SENSOR_PROERTY field in report descriptor for later use in finding parameters within the GetReportFeature data
    BOOL bFlag = FALSE;                                             // Flag variable to notify that desired fields have been reached

   

    memset(TEMP_SF, 0xFF, sizeof(TEMP_SF));                         // Set all of the structs in the TEMP array to 0xFF so we can know if a sensor wasn't found in the report
    
    while( (ucBptr - ucStrt) <= HID_FIELD.wRepDescLen )             // Continue looping until end of report
    {
        if ( *ucBptr == HID_COLLECTION )                            // Look for HID_COLLECTION(Physical) which should be start of REPORT ID (sensor device) info in report descriptor table  
        {
            ucBptr++;                                               // Increment the pointer to look at the next byte
            
            if ( *(ucBptr++) == HID_PHYSICAL && *(ucBptr++) == HID_REPORT_ID ) // Check if the next two bytes meet the next required identifier needs   
            {
                ucUsageOffset = 0;                                  // clear offset of desired HID_USAGE_SENSOR_PROERTY field in report descriptor for later use in finding parameters within the GetReportFeature data                                    
                TEMP_SF[ucSensptr].id = *ucBptr;                    // Store first sensor device ID number in out temp struct
                
                while(*(ucBptr++) != HID_USAGE_SENSOR_TYPE);        // Parse until sensor type indicator found  (in HID table: HID_USAGE_SENSOR_TYPE_MOTION_ACCELEROMETER_3D)
                
                TEMP_SF[ucSensptr].type = *ucBptr;                  // Store sensor type in struct      
                bFlag = TRUE;                                       // Set the flag to show we are in a field of data that we desire
            }
        }
        
        if ( *ucBptr == HID_END_COLLECTION )                        // Look for end of this sensor's collection 
        {
            ucBptr++;
            
            if( *ucBptr == HID_END_COLLECTION ) 
                break;                                              // end of the report has occurred
            
            else if(*(ucBptr++) == HID_REPORT_ID)                   // Next report ID has been found
            {   
                ucUsageOffset = 0;                                  // clear offset of desired HID_USAGE_SENSOR_PROERTY field in report descriptor for later use in finding parameters within the GetReportFeature data
                TEMP_SF[ucSensptr].id = *ucBptr;                    // Store location of sensor ID within the HID Report in struct
                
                while(*(ucBptr++) != HID_USAGE_SENSOR_TYPE);        // Look for sensor type identifier (ie HID_USAGE_SENSOR_TYPE_MOTION_ACCELEROMETER_3D)
                
                TEMP_SF[ucSensptr].type = *(ucBptr++);              // Store location of sensor type within the HID Report in struct   
                bFlag = TRUE;                                       // Set the flag to show we are in a field of data that we desire
            }
        }

        if (bFlag)                                                  // Check if we should proceed parsing within the HID_USAGE_SENSOR_Properties or simply continue incrementing until a new ID is found
        {
            while((ucBptr - ucStrt) <= HID_FIELD.wRepDescLen)       // Search for relevant features HID_USAGE_SENSOR_Properties
            {
                if (*ucBptr == HID_USAGE_SENSOR_PROPERTY)             
                {
                    ucBptr++;
                    if (*ucBptr == HID_USAGE_SENSOR_PROPERTY_CONN_TYPE) // Seach for sensor connection type and increment offset variable because this field is undesired  (in HID table:HID_USAGE_SENSOR_PROPERTY_SENSOR_CONNECTION_TYPE)
                    {
                        ucBptr++;
                        if (*(ucBptr++) == 0x03)                    // last parameter of HID_USAGE_SENSOR_PROPERTY_SENSOR_CONNECTION_TYPE (0x0A,0x09,0x03) 
                            ucUsageOffset++;                        //increment HID_USAGE_SENSOR_PROPERTY_... offset "pointer" (for later use in finding parameters within the GetReportFeature data)
                    }
                    
                    if (*ucBptr == HID_USAGE_SENSOR_PROPERTY_RPT_STATE)  // Search for reporting state and increment offset variable because this field is undesired  (in HID table:HID_USAGE_SENSOR_PROPERTY_REPORTING_STATE)
                    {
                        ucBptr++;
                        if (*(ucBptr++) == 0x03)                    // last parameter of HID_USAGE_SENSOR_PROPERTY_REPORT_INTERVAL (0x0A,0x0E,0x03) 
                            ucUsageOffset++;                        //increment HID_USAGE_SENSOR_PROPERTY_... offset "pointer" (for later use in finding parameters within the GetReportFeature data)
                    }

                    if (*ucBptr == HID_USAGE_SENSOR_PROPERTY_PWR_STATE)  // Search for power state and increment offset variable because this field is undesired  (in HID table : HID_USAGE_SENSOR_PROPERTY_POWER_STATE)
                    {
                        ucBptr++;
                        if (*(ucBptr++) == 0x03)                    // last parameter of HID_USAGE_SENSOR_PROPERTY_POWER_STATE (0x0A,0x19,0x03) 
                            ucUsageOffset++;                        //increment HID_USAGE_SENSOR_PROPERTY_... offset "pointer" (for later use in finding parameters within the GetReportFeature data)
                     }

                    if (*ucBptr == HID_USAGE_SENSOR_STATE_1)        // Search for sensor state and increment offset variable because this field is undesired  (in HID table:HID_USAGE_SENSOR_STATE)
                    {
                        ucBptr++;
                        if (*(ucBptr++) == 0x02)                    // last parameter of HID_USAGE_SENSOR_STATE (0x0A,0x01,0x02) 
                            ucUsageOffset++;                        //increment HID_USAGE_SENSOR_PROPERTY_... offset "pointer" (for later use in finding parameters within the GetReportFeature data)
                     }

                    if (*ucBptr == HID_USAGE_SENSOR_PROPERTY_RPT_INT)  // Search for reporting interval. We desire this value so store it in our struct and then increment the offset  (in HID table:HID_USAGE_SENSOR_PROPERTY_REPORT_INTERVAL)
                    {
                        ucBptr++;
                        if (*(ucBptr++) == 0x03)                    // last parameter of HID_USAGE_SENSOR_PROPERTY_REPORT_INTERVAL (0x0A,0x0E,0x03)
                            TEMP_SF[ucSensptr].DatRtOffset = ucUsageOffset++; //save & increment HID_USAGE_SENSOR_PROPERTY_... offset "pointer" (for later use in finding parameters within the GetReportFeature data)
                    }

                    ucBptr++;                                       //increment position ptr

                    // NOTE: There is a different identifier for RAW data and that is why two identifiers are checked for here
                    if ( (*ucBptr == HID_USAGE_SENSOR_DATA_ACCU) || (*ucBptr == HID_USAGE_SENSOR_DATA_RAW_ACCU) )  // Search for sensor accuracy & increment offset variable because this field is undesired (in HID table:HID_USAGE_SENSOR_DATA(HID_USAGE_SENSOR_DATA_MOTION_ACCELERATION,HID_USAGE_SENSOR_DATA_MOD_ACCURACY))
                        ucUsageOffset++;                            //increment HID_USAGE_SENSOR_PROPERTY_... offset "pointer" (for later use in finding parameters within the GetReportFeature data)

                    if ( (*ucBptr == HID_USAGE_SENSOR_DATA_RES) || (*ucBptr == HID_USAGE_SENSOR_DATA_RAW_RES) )    // Search for sensor resolution and increment offset variable because this field is undesired (in HID table: HID_USAGE_SENSOR_DATA(HID_USAGE_SENSOR_DATA_MOTION_ACCELERATION,HID_USAGE_SENSOR_DATA_MOD_RESOLUTION))
                        ucUsageOffset++;                            //increment HID_USAGE_SENSOR_PROPERTY_... offset "pointer" (for later use in finding parameters within the GetReportFeature data)

                    if ( (*ucBptr == HID_USAGE_SENSOR_DATA_MOD_SENS) || (*ucBptr == HID_USAGE_SENSOR_DATA_RAW_MOD_SENS) )  // Search for sensor sensitivity (HID_USAGE_SENSOR_DATA(HID_USAGE_SENSOR_DATA_MOTION_ACCELERATION,HID_USAGE_SENSOR_DATA_MOD_CHANGE_SENSITIVITY_ABS))
                    {
                        TEMP_SF[ucSensptr].SensOffset = ucUsageOffset++; //save & increment HID_USAGE_SENSOR_PROPERTY_... offset "pointer" (for later use in finding parameters within the GetReportFeature data)                         
                        
                        while(*(ucBptr++) != HID_UNIT_EXP);         // Increment until exponent value of the data is found
                        
                        TEMP_SF[ucSensptr].SensExp = *ucBptr;       // Store this value in the temp struct
                            break;
                    }

                    if ( (*ucBptr == HID_USAGE_SENSOR_DATA_MOD_MAX) || (*ucBptr == HID_USAGE_SENSOR_DATA_RAW_MOD_MAX) )   // Search for sensor MAX val and increment offset variable because this field is undesired HID_USAGE_SENSOR_DATA(HID_USAGE_SENSOR_DATA_MOTION_ACCELERATION,HID_USAGE_SENSOR_DATA_MOD_MAX)
                        ucUsageOffset++;                            //increment HID_USAGE_SENSOR_PROPERTY_... offset "pointer" (for later use in finding parameters within the GetReportFeature data)
                    
                    if ( (*ucBptr == HID_USAGE_SENSOR_DATA_MOD_MIN)  || (*ucBptr == HID_USAGE_SENSOR_DATA_RAW_MOD_MIN) )  // Search for sensor MIN val and increment offset variable because this field is undesired HID_USAGE_SENSOR_DATA(HID_USAGE_SENSOR_DATA_MOTION_ACCELERATION,HID_USAGE_SENSOR_DATA_MOD_MIN)
                        ucUsageOffset++;                            //increment HID_USAGE_SENSOR_PROPERTY_... offset "pointer" (for later use in finding parameters within the GetReportFeature data)
                }
                
                else ucBptr++;                                      // If a new identifier has not yet been reached, continue traversing report descriptor 
            }
            
            ucUsageOffset = 0;                                      //reset HID_USAGE_SENSOR_PROPERTY_... offset "pointer" (for later use in finding parameters within the GetReportFeature data)
            
            while((ucBptr - ucStrt) <= HID_FIELD.wRepDescLen)       // Search for relevant input features
            {
                if(*ucBptr == HID_USAGE_SENSOR_)                         
                {
                    ucBptr++;                                       // Continue to next byte
                    
                    if (*ucBptr == HID_USAGE_SENSOR_STATE_1)                  
                    {
                        ucBptr++;
                        if(*(ucBptr++) == HID_USAGE_SENSOR_STATE_2) // Search for HID usage sensor state and increment offset variable because this field is undesired
                            ucUsageOffset++;                        //increment HID_USAGE_SENSOR_PROPERTY_... offset "pointer" (for later use in finding parameters within the GetReportFeature data)
                    }

                    if (*ucBptr == HID_USAGE_SENSOR_EVENT_1)               
                    {
                        ucBptr++;                                   // Continue to next byte
                        if (*(ucBptr++) == HID_USAGE_SENSOR_EVENT_2)   // Search for HID usage sensor event and increment the offset variable
                        {    
                            ucUsageOffset++;                        //increment HID_USAGE_SENSOR_PROPERTY_... offset "pointer" (for later use in finding parameters within the GetReportFeature data)
                            while(*(ucBptr++) != HID_END_COLLECTION); // end of the sensor event field signifies the start of desired input data
                            
                            TEMP_SF[ucSensptr].DatOffset = ucUsageOffset + 3; //save HID_USAGE_SENSOR_PROPERTY_... offset "pointer" (for later use in finding parameters within the GetReportFeature data) NOTE: offset an additional 3 for extra data received on GPIO interrupt
                            
                            while(*(ucBptr++) != HID_UNIT_EXP);     // Search for, and store, the unit exponent value for the input data
                            
                            TEMP_SF[ucSensptr].DatExp = *ucBptr;
                            
                            break;
                        }
                    }
                }
                else ucBptr++;                                      // If desired identifiers haven't been reached, continue traversing HID Report Descriptor
            }
            
            bFlag = FALSE;                                          // Reset the flag to 0 to show that we are done with descriptor data from this report ID
            ucSensptr++;                                            // Increment to the next sensor in the struct array of sensors
        }

        else ucBptr++;                                              // If desired identifiers haven't been reached, continue traversing HID Report Descriptor
    }

    if ((ucBptr - ucStrt) > (HID_FIELD.wRepDescLen + 1))            // Sanity check to ensure the loop exited at the end of the report descriptor
        return NO_EOC_FAIL;

    for(ucSensptr = 0; ucSensptr < NUM_SENS; ucSensptr++)           // Sanity check to see if the desired fields were parsed correctly
    {
        if (TEMP_SF[ucSensptr].id == 0xFF)                          // If the ID == 0xFF it means that the sensor isn't present in the device
            continue;
        if (TEMP_SF[ucSensptr].id != (ucSensptr+1))                 // Make sure every filled struct has a valid sensor ID
             return REP_PARS_FAIL; 
    }

    memset(&SENSOR, 0xFF, sizeof(SENSOR));                          // Init & Rearrange structs into order compatible with VREGS structure            

    for (ucSensptr = 0; ucSensptr < NUM_SENS; ucSensptr++)          // go through the entire sensor list looking for sensor types
    {
        if (TEMP_SF[ucSensptr].type == ACCEL_SENSOR_TYPE)           // Sensor type identifier for accelerometer
        {
            SENSOR[ACCEL_VREG_OFFSET] = TEMP_SF[ucSensptr];         // Store this struct as the first in the new order
            VREGS.SL.accel = TRUE;                                  // VREG01 accelerometer available bit(0) set
        }
        
        if (TEMP_SF[ucSensptr].type == GYRO_SENSOR_TYPE)            // Sensor type identifier for gyrometer
        {
            SENSOR[GYRO_VREG_OFFSET] = TEMP_SF[ucSensptr];          // Store this struct as the second in the new order
            VREGS.SL.gyro = TRUE;                                   // VREG01 gyrometer available bit(1) set
        }
        
        if (TEMP_SF[ucSensptr].type == CMP_SENSOR_TYPE)             // Sensor type identifier for compass
        {
            SENSOR[CMP_VREG_OFFSET] = TEMP_SF[ucSensptr];           // Store this struct as the third in the new order
            VREGS.SL.cmp = TRUE;                                    // VREG01 compass available bit(2) set
        }
        
        if (TEMP_SF[ucSensptr].type == ORI_SENSOR_TYPE)             // Sensor type identifier for orientation sensor
        {
            SENSOR[ORI_VREG_OFFSET] = TEMP_SF[ucSensptr];           // Store this struct as the fourth in the new order
            VREGS.SL.ori = TRUE;                                    // VREG01 orientation available bit(3) set
        }
        
        if (TEMP_SF[ucSensptr].type == INCL_SENSOR_TYPE)            // Sensor type identifier for inclinometer
        {
            SENSOR[INCL_VREG_OFFSET] = TEMP_SF[ucSensptr];          // Store this struct as the fifth in the new order
            VREGS.SL.incl = TRUE;                                   // VREG01 gyrometer available bit(4) set
        }
        
        if (TEMP_SF[ucSensptr].type == RAW_SENSOR_TYPE)             // Sensor type identifier for raw data
        {                                                           // NOTE: there is no VREG01 bit for raw sensors available 
            SENSOR[RAW_VREG_OFFSET+RAW_SENSOR_CNT] = TEMP_SF[ucSensptr]; // Store this struct as the 8th/9th/10th in the new order
            RAW_SENSOR_CNT++;                                       // NOTE: pre-production SSC7150 F/W only enumerates 1 RAW sensor, so keep running count
        }     
    }
    
    return SUCCESS;
}


/** ret_exponent
* @note	Returns the exponent scaler for a requested data field
* @param ucAdj_SensN sensor num of interest
* @return dMult multiplier value
*/ 
double ret_exponent (UINT8 ucAdj_SensN)
{
    UINT8 ucVreg_ptr = VREG_EXP1;                                   // VREG36 is start of exponent values
    UINT8 ucRet;                                                    // Function status variable
    double dMult = 1;                                               // Unit exponent scaler to be returned
    UINT16 wExpo;                                                   // 'Buffer' to hold the exponent register's contents
    UINT8 ucTemp_expo = 0;                                          // Variable to hold the 4 bit exponent value


    if (ucAdj_SensN > 3)                                            // sensor indexes from 4 to 7 = sensors that have unit exponents in the 2nd exponent register (VREG37)
        ucVreg_ptr++;                                               // Increment ptr to VREG exponent register to be searched

    if (ucAdj_SensN > 7)                                            // sensor indexes > 7 = sensors that have unit exponents in the 3rd exponent register (VREG38)
        ucVreg_ptr++;                                               // Increment ptr to VREG exponent register to be searched
      
    ucAdj_SensN= ucAdj_SensN % 4;                                   // Find the appropriate offset for this sensor's unit exponent


    ucRet = HOST_SF_LIB_VREG_read (ucVreg_ptr, &wExpo);             // Read this sesnor's data unit exponent value from the VREG
    
    ucTemp_expo = (UINT8)((wExpo >> (4 * ucAdj_SensN)) & 0xF);       // get the 4 exp bits for desired sensor

    if (ucTemp_expo >= 0 && ucTemp_expo <= 7)                        // These values are all positive exponents
        dMult = pow(10, ucTemp_expo);

    if(ucTemp_expo >= 8 && ucTemp_expo <= 0x0F)                      // These values are all negative exponents (ie. to right of decimal place)
        dMult = pow(10, (int)(-16 + ucTemp_expo));

    return dMult;                                                    //return Unit exponent scaler
}

/** HOST_SF_LIB_HID_Set_Report
* @note	Sends the HID I2C set report command to device
* @param type refers to feature or output report; type = 3 - feature, 2 - output
* @param ReportBuffer pointer to data that needs to be sent to the device
* @param size specifies the size, in bytes,of the report buffer 
* @return completion_status 0=SUCCESS, failcode: 0x19=SET_RPT_FAIL
*/
UINT8 HOST_SF_LIB_HID_Set_Report(UINT8 type, UINT8 *ReportBuffer, UINT8 size)
{
    UINT8 ucTx_data[BUF_150];
    UINT8 ucRetStat;
    UINT8 ucCmdBytePtr;
    BOOL bVendorCmd = FALSE;


    if (size <= 0 || size > (BUF_150 - RPT_LEN))
        return SET_RPT_FAIL;
    
    if (ReportBuffer[0] == FLASH_UPDATE_RPT_ID)                     // the flash update (Vendor) commands have a different format
        bVendorCmd = TRUE;
    
    ucTx_data[0] = HID_FIELD.wCmdReg;                               //command field bytes from HID config table
    ucTx_data[1] = (HID_FIELD.wCmdReg >> BYTE_SHIFT);               // Command register MSB
    if (bVendorCmd)
        ucTx_data[2] = type | ReportBuffer[0];                      // HID Set command opcode low byte which includes the sensor's ReportID, high byte report type
    else
        ucTx_data[2] = type | ReportBuffer[2];
    ucTx_data[3] = SET_OPCODE;                                      // HID SetReport command opcode high byte
    ucTx_data[4] = HID_FIELD.wDatReg;                               //data field bytes from HID config table
    ucTx_data[5] = (HID_FIELD.wDatReg >> BYTE_SHIFT);

    ucCmdBytePtr = RPT_LEN;
    
    if (bVendorCmd)                                                 // flash update (vendor) commands have a different format
        ReportBuffer++;                                             //skips 1st byte of input buffer
    else size++;

    while (size--)
    {
        ucTx_data[ucCmdBytePtr++] = *(ReportBuffer++);              // Append input to the command
    }

    ucRetStat = i2c_cmd_WrRd (WRITE,                                //issue SetReportOutput command to SSC7150
                        ucCmdBytePtr,                               //num of cmd bytes
                        ucTx_data,                                  //cmd buf
                        0,                                          //num of bytes to read initially
                        0,                                          //recv buf
                        FALSE);                                     //actual # of bytes SSC7150 returns is in 1st two bytes of read packet, this flag(=TRUE) means "use the 1st two bytes as the actual read packet length"

    if (ucRetStat != SUCCESS)
    {
        return SET_RPT_FAIL;
    }

    return ucRetStat;
}

/** HOST_SF_LIB_HID_Get_Report
* @note	Sends the HID I2C Get Report command to device
* @param type refers to feature or input report; type = 3 - feature, 1 - input
* @param ReportBuffer pointer towhich the feature report data is read into 
* @param size specifies the size, in bytes, of the report buffer 
* @return completion_status 0=I2C_SUCCESS, failcode: 0x1C=HID_GET_RPT_INPT_FAIL,0x1D=HID_GET_RPT_FEAT_FAIL
*/
UINT8 HOST_SF_LIB_HID_Get_Report(UINT8 type, UINT8* ReportBuffer, UINT8 size)
{
    UINT8 ucTx_data[BUF_150];
    UINT8 ucRetStat;
    UINT8 ucCmdBytePtr;

    ucTx_data[0] = HID_FIELD.wCmdReg;                               //command field bytes from HID config table
    ucTx_data[1] = (HID_FIELD.wCmdReg >> BYTE_SHIFT);               // Command register MSB
    ucTx_data[2] = type | ReportBuffer[0];                          // HID Get command opcode low byte which includes the sensor's ReportID, high byte report type
    ucTx_data[3] = GET_RPT_CMD_MSB;                                 // HID GetReport command opcode high byte
    ucTx_data[4] = HID_FIELD.wDatReg;                               //data field bytes from HID config table
    ucTx_data[5] = (HID_FIELD.wDatReg >> BYTE_SHIFT);

    ucCmdBytePtr = RPT_LEN;

    ucRetStat = i2c_cmd_WrRd (WR_RD,                                 //issue SetReportOutput command to SSC7150
                        ucCmdBytePtr,                               //num of cmd bytes
                        ucTx_data,                                  //cmd buf
                        BYTE_ADJ_VAL,                               //num of bytes to read initially
                        ReportBuffer,                               //recv buf
                        TRUE);                                      //actual # of bytes SSC7150 returns is in 1st two bytes of read packet, this flag(=TRUE) means "use the 1st two bytes as the actual read packet length"

    if (ucRetStat != SUCCESS)
    {
        if (type == GET_RPT_FEAT)
            return HID_GET_RPT_FEAT_FAIL;
        else
            return HID_GET_RPT_INPT_FAIL;
    }

    return ucRetStat;
}


