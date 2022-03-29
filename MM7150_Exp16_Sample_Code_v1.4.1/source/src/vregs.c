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
/** @file  vregs.c
*   Sensor Virtual register implementation functions 
*****************************************************************************
*   MM7150 with Explorer 16 Development Board Sample Code system file
*
*   Company : Microchip Technology Inc.
*
*   File name : vregs.c
*
*   Summary : VREG functions module for MM7150 demo code 
*
*   Functions:  VREG_init
*               set_state_data
*               parse_update_VREG_data
*               HOST_SF_LIB_VREG_read
*               HOST_SF_LIB_VREG_write
*
*   Revisions : 0.7 10-6-16 A21444 - added flash corruption recovery
*             : 0.6 7-9-15 C21674 - added error/check recovery to VREG_init() for HID_INT sigal stuck low
*             : 0.5 5-02-15 C21674 - moved MagFlux readings to Compass from RawMag
*             : 0.4 2-04-15 C21674 - added wake support
*             : 0.3 9-30-14 C21674 - error handling, sleep implementation : all
*                                    sensors must be disabled before sleeping (per API spec)
*             : 0.2 9-4-14 - (removed backward compatibilty) SSC7150 build0600 and later
*             : 0.1 8-4-14 C21674  - added backward compatibility for SSC7150 F/W
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

SF_VREGS VREGS;                                                     // structure of Sensor VREG registers

GET_SET_PARAMS SET_PARAMS;                                          //structure of SetFeature parameters
extern SF_SENSOR SENSOR[NUM_SENS];                                  // structure of individual sensors
extern volatile BOOL EC_DATA_AVAIL;                                 // flag to indicate when EC has data available to be read
UINT16 usPREV_SHC_STATE;                                            // UINT16 buffer to store previous SHC VREG config for comparison
extern UINT32 POR_TIMER;											// amount of elapsed time (in ms) since POR 

//*****************************************************************************
//*****************************************************************************
//  Section : Code
//*****************************************************************************
//*****************************************************************************

/** VREG_init
* @note	Inits VREG registers, get HID config & report descriptor tables, retrieves all device features
* @param 
* @return error status 0=SUCCESS, failcodes: 0x11=HID_DESC_FAIL, 0x12=RPT_DESC_FAIL, 0x18=RESET_FAIL 
*/ 
UINT8 VREG_init() 
{
    UINT8 ucBuf[BUF_40];                                           
    UINT8 ucSensor_num;
    UINT16 err;

	
    memset(&VREGS, 0x00, sizeof(VREGS));                            // Initialize VREG registers 

    while (POR_TIMER < I2C_POR_TIMEOUT);							//wait here for 2 seconds elapsed since POR for MM7150 i2c engine to be up and running
 	
    if (HID_I2C_completion())										// check initial polarity of HIDI2C_HOST_INT on RE8/INT1 if this signal is LOW (ASSERTED) to start, then issue a HID_READ command to try and clear it (MM7150 finish sensor reading that was interrupted by POR)
        return HID_INT_FAIL;

    if ( hid_i2c_descriptor_handler(GET_HID_DESC) )                 // get HID descriptor from SSC7150
    {
        VREGS.stat.stat4.SHStartStatus = VREG_SHSTART_FAIL;         // update status register (VREG 0x3F) for failure to get HID descriptor
        return HID_DESC_FAIL;
    }        
    
    hid_i2c_cmd_process(ucBuf, POWER_ON, ARB_ID);                   // Issue HID Power ON command to SSC7150 (NOTE: 'ucBuf' and 'ARB_ID' are don't cares for POWER_ON command)
    
    VREGS.SHC.reset = VREG_RESET_INIT;                              //set the SHC reset bit to indicate this operation has not yet completed successfully
    if ( hid_i2c_cmd_process (ucBuf, RESET_DEV_CMD, ARB_ID) )       // Issue HID Reset command  (NOTE: 'ucBuf' and 'ARB_ID' are don't cares for RESET_REG command)
    {
        VREGS.stat.stat4.SHStartStatus = VREG_SHSTART_FAIL;         // If HID Reset fails, update status register (VREG 0x3F)
        return RESET_FAIL;
    }    
    
    if ( (err = hid_i2c_descriptor_handler(GET_RPT_DESC)) != SUCCESS ) // Get HID Report descriptor from SSC7150 
    {
        VREGS.stat.stat4.SHStartStatus = VREG_SHSTART_FAIL;         // HID report descriptor error, update status register (VREG 0x3F)
        return (err);
    }   
    
    for (ucSensor_num = 0; ucSensor_num <= NUM_SENS; ucSensor_num++) //go through the sensor list but only GetFeatureReport for valid ids   
    {
        if (SENSOR[ucSensor_num].id != 0xFF && SENSOR[ucSensor_num].id != 0)
        {
            hid_i2c_cmd_process(ucBuf, HID_GET_RPT_FEAT, SENSOR[ucSensor_num].id);  // Get feature report for all valid sensors and update VREGS register
        }
    }

    
    VREGS.stat.stat4.SHStartStatus = VREG_SHSTART_SUCCESS;          // HID startup successful, update status register (VREG 0x3F) 

    VREGS.SHC.SHwake = TRUE;                                        // HID device is now awake and ready for operation
    VREGS.SHC.SHstart = TRUE;

    return SUCCESS;
}


/** set_state_data
* @note	Set and confirm new features for a device 
* @param GET_SET_PARAMS structure containing  power state (=2: FULL, =3: LOW),sensor ID, New data rate,New sensitivity
* @return error status 0=SUCCESS, failcodes: 0x17=SET_FEAT_FAIL 
*/ 
UINT8 set_state_data(GET_SET_PARAMS SET_PARAMS)    
{
    UINT8 ucGetFeatBuf[BUF_40];                                     // GetFeature report buffer
    UINT8 ucSetFeatBuf[BUF_40];                                     // SetFeature report buffer
    UINT8 ucRetryCnt, ucGSptr, ucBufSize;


    if ( hid_i2c_cmd_process(ucGetFeatBuf, HID_GET_RPT_FEAT, SET_PARAMS.ucid) ) // Issue a get report feature command and store the get features in ucGetFeatBuf
        return SET_FEAT_FAIL;  
    
    memcpy(&ucSetFeatBuf, &ucGetFeatBuf, BUF_40);                   // Copy GetFeatBuff to bufs to use for set feature command

    // Modify fields that we would like to set in bufs (sensor state, power state, data rate, sensitivity)
    ucSetFeatBuf[RPT_REPORT_STATE] = HID_USAGE_SENSOR_PROPERTY_REPORTING_STATE_ALL_EVENTS_ENUM;   
    ucSetFeatBuf[RPT_PWR_STATE] = SET_PARAMS.ucPowerState;          //update the Power state parameter

    if (SET_PARAMS.bDataRateValid)                                  // update data rate value?
    {
        ucSetFeatBuf[RPT_REPORT_INTVAL_LSB] = (UINT8) SET_PARAMS.usDataRateVal; //update 16 bit data rate value
        ucSetFeatBuf[RPT_REPORT_INTVAL_MSB] = (UINT8) (SET_PARAMS.usDataRateVal >> BYTE_SHIFT);
    }
    
    if (SET_PARAMS.bSensitivityValid)                               // update sensitivity value?
    {
        ucSetFeatBuf[RPT_CHG_SENS_LSB] = (UINT8) SET_PARAMS.usSensitivityVal; //update 16 bit sensitivity value
        ucSetFeatBuf[RPT_CHG_SENS_MSB] = (UINT8) (SET_PARAMS.usSensitivityVal >> BYTE_SHIFT);
    }

    for (ucRetryCnt = 0; ucRetryCnt < MAX_RETRIES; ucRetryCnt++)    // API spec requires 3 attempts at setting new features. 
    {
        hid_i2c_cmd_process(ucSetFeatBuf, HID_SET_RPT_FEAT, SET_PARAMS.ucid);  // Send HID_SetFeature command to SSC7150 to update new features to the device
        hid_i2c_cmd_process(ucGetFeatBuf, HID_GET_RPT_FEAT, SET_PARAMS.ucid);  // Send HID_GetFeature command to SSC7150 to check if new feature request to device were updated
        
        ucBufSize = ucGetFeatBuf[RPT_SIZE_LSB];                     // size (in bytes) of packet is in 1st byte
        for (ucGSptr = 0; ucGSptr < ucBufSize; ucGSptr++)           // Check to see if the features have been updated
        {
            if ( ucGetFeatBuf[ucGSptr] != ucSetFeatBuf[ucGSptr] ) 
                break;                                              //no they weren't, try again
        }

        if ( ucGSptr == ucBufSize )                                 //that's the whole packet, ALL data matches
            break;                                                  //we're done
    }

    if (ucRetryCnt == MAX_RETRIES)
        return SET_FEAT_FAIL;                                       // Return error code
    
    else return SUCCESS;                                            // Return successful
}

/** parse_update_VREG_data
* @note	Parse input report, update data VREGS 
* @param ucSensorNum Sensor data struct checked for unit exponent value
* @param ucInpRep Pointer to input report data buffer
* @return 
*/ 
void parse_update_VREG_data (UINT8 ucSensorNum, UINT8 *ucInpRep)
{
    
    switch (ucSensorNum)                                            //update VREG data registers with their respective data.
    {     
        case ACCEL_VREG_OFFSET:                                     // Accelerometer data has been read
 
            VREGS.data.ACXD = ((ucInpRep[SENSOR[ACCEL_VREG_OFFSET].DatOffset + 1] << BYTE_SHIFT) | ucInpRep[SENSOR[ACCEL_VREG_OFFSET].DatOffset]);    // parse data from data offset found in report descriptor
            VREGS.data.ACYD = ((ucInpRep[SENSOR[ACCEL_VREG_OFFSET].DatOffset + 3] << BYTE_SHIFT) | ucInpRep[SENSOR[ACCEL_VREG_OFFSET].DatOffset + 2]);
            VREGS.data.ACZD = ((ucInpRep[SENSOR[ACCEL_VREG_OFFSET].DatOffset + 5] << BYTE_SHIFT) | ucInpRep[SENSOR[ACCEL_VREG_OFFSET].DatOffset + 4]);
            break;

        case GYRO_VREG_OFFSET:                                      // Gyrometer data has been read
 
            VREGS.data.GYXD = ((ucInpRep[SENSOR[GYRO_VREG_OFFSET].DatOffset + 1] << BYTE_SHIFT) | ucInpRep[SENSOR[GYRO_VREG_OFFSET].DatOffset]);
            VREGS.data.GYYD = ((ucInpRep[SENSOR[GYRO_VREG_OFFSET].DatOffset + 3] << BYTE_SHIFT) | ucInpRep[SENSOR[GYRO_VREG_OFFSET].DatOffset + 2]);
            VREGS.data.GYZD = ((ucInpRep[SENSOR[GYRO_VREG_OFFSET].DatOffset + 5] << BYTE_SHIFT) | ucInpRep[SENSOR[GYRO_VREG_OFFSET].DatOffset + 4]);
            break;

        case CMP_VREG_OFFSET:                                       // Compass data has been read

            VREGS.data.CMD = ((ucInpRep[SENSOR[CMP_VREG_OFFSET].DatOffset + 1] << BYTE_SHIFT) | ucInpRep[SENSOR[CMP_VREG_OFFSET].DatOffset]);
            
            VREGS.data.MGFXD = ((ucInpRep[SENSOR[CMP_VREG_OFFSET].DatOffset + 3] << BYTE_SHIFT) | ucInpRep[SENSOR[CMP_VREG_OFFSET].DatOffset + 2]);		//Magnetic Flux are associated with Compass readings 
            VREGS.data.MGFYD = ((ucInpRep[SENSOR[CMP_VREG_OFFSET].DatOffset + 5] << BYTE_SHIFT) | ucInpRep[SENSOR[CMP_VREG_OFFSET].DatOffset + 4]);
            VREGS.data.MGFZD = ((ucInpRep[SENSOR[CMP_VREG_OFFSET].DatOffset + 7] << BYTE_SHIFT) | ucInpRep[SENSOR[CMP_VREG_OFFSET].DatOffset + 6]);

            break;
        
        case ORI_VREG_OFFSET:                                       // Orientation data has been read

            VREGS.data.ORXD = ((ucInpRep[SENSOR[ORI_VREG_OFFSET].DatOffset + 1] << BYTE_SHIFT) | ucInpRep[SENSOR[ORI_VREG_OFFSET].DatOffset]);
            VREGS.data.ORYD = ((ucInpRep[SENSOR[ORI_VREG_OFFSET].DatOffset + 3] << BYTE_SHIFT) | ucInpRep[SENSOR[ORI_VREG_OFFSET].DatOffset + 2]);
            VREGS.data.ORZD = ((ucInpRep[SENSOR[ORI_VREG_OFFSET].DatOffset + 5] << BYTE_SHIFT) | ucInpRep[SENSOR[ORI_VREG_OFFSET].DatOffset + 4]);
            VREGS.data.ORWD = ((ucInpRep[SENSOR[ORI_VREG_OFFSET].DatOffset + 7] << BYTE_SHIFT) | ucInpRep[SENSOR[ORI_VREG_OFFSET].DatOffset + 6]);
            break;
       
        case INCL_VREG_OFFSET:                                      // Inclinometer data has been read

            VREGS.data.INXD = ((ucInpRep[SENSOR[INCL_VREG_OFFSET].DatOffset + 1] << BYTE_SHIFT) | ucInpRep[SENSOR[INCL_VREG_OFFSET].DatOffset]);
            VREGS.data.INYD = ((ucInpRep[SENSOR[INCL_VREG_OFFSET].DatOffset + 3] << BYTE_SHIFT) | ucInpRep[SENSOR[INCL_VREG_OFFSET].DatOffset + 2]);
            VREGS.data.INZD = ((ucInpRep[SENSOR[INCL_VREG_OFFSET].DatOffset + 5] << BYTE_SHIFT) | ucInpRep[SENSOR[INCL_VREG_OFFSET].DatOffset + 4]);
            break;

        case RAW_ACC_VREG_OFFSET:                                   // Raw data has been read
            
            VREGS.data.RACXD = ((ucInpRep[SENSOR[RAW_VREG_OFFSET].DatOffset + 1] << BYTE_SHIFT) | ucInpRep[SENSOR[RAW_VREG_OFFSET].DatOffset]);
            VREGS.data.RACYD = ((ucInpRep[SENSOR[RAW_VREG_OFFSET].DatOffset + 3] << BYTE_SHIFT) | ucInpRep[SENSOR[RAW_VREG_OFFSET].DatOffset + 2]);
            VREGS.data.RACZD = ((ucInpRep[SENSOR[RAW_VREG_OFFSET].DatOffset + 5] << BYTE_SHIFT) | ucInpRep[SENSOR[RAW_VREG_OFFSET].DatOffset + 4]);
            break;
       
        case RAW_MAG_VREG_OFFSET:                                   // Raw MAG data has been read
            
            VREGS.data.RMGXD = ((ucInpRep[SENSOR[RAW_MAG_VREG_OFFSET].DatOffset + 1] << BYTE_SHIFT) | ucInpRep[SENSOR[RAW_MAG_VREG_OFFSET].DatOffset]);
            VREGS.data.RMGYD = ((ucInpRep[SENSOR[RAW_MAG_VREG_OFFSET].DatOffset + 3] << BYTE_SHIFT) | ucInpRep[SENSOR[RAW_MAG_VREG_OFFSET].DatOffset + 2]);
            VREGS.data.RMGZD = ((ucInpRep[SENSOR[RAW_MAG_VREG_OFFSET].DatOffset + 5] << BYTE_SHIFT) | ucInpRep[SENSOR[RAW_MAG_VREG_OFFSET].DatOffset + 4]);
            break;
        
        case RAW_GYR_VREG_OFFSET:                                   // Raw GYRO data has been read
            
            VREGS.data.RGYXD = ((ucInpRep[SENSOR[RAW_GYR_VREG_OFFSET].DatOffset + 1] << BYTE_SHIFT) | ucInpRep[SENSOR[RAW_GYR_VREG_OFFSET].DatOffset]);
            VREGS.data.RGYYD = ((ucInpRep[SENSOR[RAW_GYR_VREG_OFFSET].DatOffset + 3] << BYTE_SHIFT) | ucInpRep[SENSOR[RAW_GYR_VREG_OFFSET].DatOffset + 2]);
            VREGS.data.RGYZD = ((ucInpRep[SENSOR[RAW_GYR_VREG_OFFSET].DatOffset + 5] << BYTE_SHIFT) | ucInpRep[SENSOR[RAW_GYR_VREG_OFFSET].DatOffset + 4]);
            break;

        default:
            break;
    }
    return;
}


/** HOST_SF_LIB_VREG_read
* @note	Reads & returns contents of requested virtual (VREG) register while updating the SHC VREG to reflect the appropriate sensor. Also checks for input data & parses it into data VREGS. 
* @param ucRegOffset VREG register of interest
* @param usData Ptr for 16 bit value
* @return completion status 0=SUCCESS, failcodes: 0x31=HID_INT_FAIL, 0x33=VREG_OFFSET_ERR,  
*/ 
UINT8 HOST_SF_LIB_VREG_read(UINT8 ucRegOffset, UINT16 *usData)
{
    UINT8 ucRet = FALSE;
    UINT8 ucRx_data[BUF_40];
    UINT8 ucSensorNum;
    UINT16 *usRegPtr;                                               // Pointer to individual register

    
    if (ucRegOffset > VREG_MAX_OFFSET)                              //make sure this is within the VREG register set 
        return VREG_OFFSET_ERR;

    usRegPtr = (UINT16 *)&VREGS.SHC;                                // Set the pointer to SHC (VREG00 register)

    if (EC_DATA_AVAIL)                                              //SSC7150 sensor has data available for us to read
    {       
        ucRet = i2c_cmd_WrRd (READ,                                  // Read the data from the SSC7150
                            0,                                      //num of cmd bytes
                            0,                                      //cmd buf (ignored)
                            BYTE_ADJ_VAL,                           //num of bytes to read
                            ucRx_data,                              //recv buf 
                            TRUE);                                  //actual # of bytes SSC7150 returns is in 1st two bytes of read packet, this flag(=TRUE) means "use the 1st two bytes as the actual read packet length"
		
        if (ucRet) 
            return HID_INT_FAIL;

        for (ucSensorNum = 0; ucSensorNum < NUM_SENS; ucSensorNum++) //go through the sensor list to see where to put the received data 
        {              
            if (ucRx_data[2] == SENSOR[ucSensorNum].id)          //1st two bytes of read data packet hold size of packet, then data begins at byte 2
            {
                if (*usRegPtr & (1 << (ucSensorNum + VREG_SHC_ACC_EN_VAL))) // If the incoming data belongs to an enabled sensor (in SHC (VREGS00))
                {
                    parse_update_VREG_data(ucSensorNum, ucRx_data); //store data in appropriate VREG data registers
                    break;                                          //found correct enabled sensor, no reason to look further
                }
            }            
        }   
    }
    
    usRegPtr += (UINT16) ucRegOffset;                               //point to VREG specified by input parameter 'ucRegOffset'
    *usData = *usRegPtr;                                            //return the register contents specified by the read command
    return SUCCESS;
}

/** HOST_SF_LIB_VREG_write
* @note	Write commands to specified VREG. Reads & returns contents of requested virtual (VREG) register 
*        while updating the SHC VREG to reflect the appropriate sensor. Also checks for input data & parses it into data VREGS. 
* @param ucRegOffset VREG register of interest
* @param usData Data to be written to device
* @return completion status 0=SUCCESS, failcodes: 0x32=VREG_ACCESS_ERR, 0x33=VREG_OFFSET_ERR, 0x31=HID_INT_FAIL, 0x17=SET_FEAT_FAIL, 0x18=RESET_FAIL, 0x1E=WAKE_CMD_FAIL, 0x1B=SLEEP_CMD_FAIL 
*/ 
UINT8 HOST_SF_LIB_VREG_write(UINT8 ucRegOffset, UINT16 usData)
{
    UINT8 ucRet = FALSE;
    UINT16 *usRegPtr, *usStat_ptr, *usSHC_ptr;                      // Pointer to individual register
    UINT16 usMask;
    UINT8 ucRx_data[BUF_40];                                        // local buffer for i2c comm
    UINT8 ucId = 0xFF;
    UINT8 ucBitOffset, ucAdjustedPtr, ucSensorNum, usTmpStatptr, ucTmpStatVal, ucTmpPtr;


    if (ucRegOffset > VREG_MAX_OFFSET)                              //make sure this is within the VREG register set 
        return VREG_OFFSET_ERR;

    if ( (ucRegOffset == VREG_SL || (ucRegOffset >= VREG_ACXD && ucRegOffset <= VREG_EXPCSS3) ) ) // If the requested register is READ ONLY, return an error
         return VREG_ACCESS_ERR;

    usSHC_ptr = (UINT16 *)&VREGS.SHC;                               // Set pointers to SHC (VREG00 register)
  
    usPREV_SHC_STATE = *usSHC_ptr;                                  //save previous state of SHC VREG contents for comparison 

    usRegPtr = usSHC_ptr + (UINT16)ucRegOffset;                     //point to VREG specified by input parameter 'ucRegOffset'

    if (VREGS.SHC.SHstart)                                          // this bit set during VREG_init and after RESET (no user access to this bit)
    { 
        SET_PARAMS.ucPowerState = FULL_POWER;                       //initialize parameters for SetFeature call (NOTE: set to LOW_POWER when disabling sensor)
        SET_PARAMS.bSensitivityValid = FALSE;
        SET_PARAMS.bDataRateValid = FALSE;
        
        if (ucRegOffset == VREG_SHC)                                // If the command will be written to the SHC register
        {
            usStat_ptr = (UINT16 *)&VREGS.stat;                     // Set the status register pointer to the first status register

       /*****************RESET COMMAND********************/
            if (usData & VREG_SHC_RST_BIT)                          // Reset bit has been set
            {
                if (!hid_i2c_cmd_process(0, RESET_DEV_CMD, ARB_ID)) // Issue the reset command (parameters 1 and 3 are not used)
                {
                    memset(&VREGS, 0x00, sizeof(VREGS));            // Initialize VREG registers to POR values
                    VREGS.stat.stat4.ResetStatus = VREG_RST_CMD_SUCCESS; // Status update to notify command set succesful
                    VREGS.SHC.SHstart = TRUE;
                    VREGS.SHC.SHwake = TRUE;
                    return SUCCESS;                                 //return success (all VREG bits have been reset to POR values)
                }
                else
                {
                    VREGS.stat.stat4.ResetStatus = VREG_RESET_CMD_FAIL; // Status update to notify command set error occurred
                    return RESET_FAIL;
                }
            }

       /*****************WAKE COMMAND********************/
            if (usData & VREG_SHC_WAKE_BIT)                         // wake bit set?
            {
                if ((usPREV_SHC_STATE & VREG_SHC_WAKE_BIT) == 0)    // this bit was not previously set, issue wake command
                {
                    Wake_signal();                                   //assert wake signal (1 ms toggle of RE9 signal to SSC150)

                    delay(12);                                      //wait 12 ms (11 ms min per spec) after wake signal and before sending POWER_ON command to SSC7150
					
                    if (!hid_i2c_cmd_process(0, POWER_ON, ARB_ID))  // Issue the wake command (parameters 1 and 3 are not used)
                    {
                        VREGS.SHC.sleep = FALSE;                    //clear the sleep bit as per API spec
                        VREGS.SHC.SHwake = TRUE;
                        VREGS.stat.stat4.ShSleepWakeStatus = VREG_WAKE_SUCCESS; // Status update to notify command set succesful
						
                        //spec says must wait a minimum of 30 ms before next command to SSC7150, so let's wait here...
                        delay(31);                                  //delay 31 ms (30 ms min per spec)
                    }
                    else
                    {
                        VREGS.stat.stat4.ShSleepWakeStatus = VREG_WAKE_FAIL; // Status update to notify command set error occurred
                        return WAKE_CMD_FAIL;
                    }
                }
            }
            
            /*****************SLEEP COMMAND********************/
            if (usData & VREG_SHC_SLP_BIT)                          //sleep bit set?
            {
                //check if any sensors are enabled, if they are then CLEAR SHC Sleep bit and return without setting status bit as per API spec
                if ( (usData & ~(VREG_SHC_SLP_BIT | VREG_SHC_START_BIT) ) > (VREG_SHC_SLP_BIT | VREG_SHC_START_BIT | VREG_SHC_WAKE_BIT) )
                {
                    VREGS.SHC.sleep = FALSE;
                    return SUCCESS;                                 //although the sleep command wasn't issued, don't return any error because user shouldn't issue sleep with any sensors enabled
                }

                if ((usPREV_SHC_STATE & VREG_SHC_SLP_BIT) == 0)     // this bit was not previously set, issue sleep command
                {
                    if (!hid_i2c_cmd_process(0, SLEEP, ARB_ID))     // Issue the sleep command (parameters 1 and 3 are not used)
                    {
                        VREGS.stat.stat4.ShSleepWakeStatus = VREG_SLEEP_SUCCESS; // Status update to notify command set succesful
                        VREGS.SHC.sleep = TRUE;                     // Set SHC (VREG00) to show SSC7150 device is asleep   
                        VREGS.SHC.SHwake = FALSE;                   // Clear SHC (VREG00) bit that shows device is not awake
                        
			//spec says must wait a minimum of 70 ms before wake command, so let's wait here...
                        delay(71);                                  //delay 71 ms (70 ms min per spec)
						
                        return SUCCESS;                             // no need to continue checking (since we already checked if any sensors were enabled) and we won't wake & sleep at the same time
                   }
                   else
                   {
                        VREGS.stat.stat4.ShSleepWakeStatus = VREG_SLEEP_FAIL;  // Status update to notify command set error occurred
                        return SLEEP_CMD_FAIL;
                   }
                }
            }

        /*****************ENABLE SENSOR COMMANDS********************/
            ucTmpStatVal = 0;                                       //initialize for the case of disabling ALL sensors
            
            for (ucBitOffset = VREG_SHC_ACC_EN_VAL; ucBitOffset <= VREG_SHC_RAWGYR_EN_VAL; ucBitOffset++) // Runs through bits relevant to sensor enable/disable (in VREG00) and executes command
            {
                ucAdjustedPtr = ucBitOffset;
            
                if (ucBitOffset == VREG_SHC_RAWMAG_EN_VAL)          // bit 12 in VREG00 corresponds to Raw Mag. This is bit0 of VREG3D (Stat2)
                    usStat_ptr++;                                   //  so we need to increment the status reg pointer
            
                if ( ((1 << ucBitOffset) & usData) || (usPREV_SHC_STATE & (1 << ucBitOffset)) ) //is enable bit set now OR was it previously set?
                {

                    usTmpStatptr = ucBitOffset;                     // do some pre-calculation for sensor stat bit position

                    if (usTmpStatptr >= VREG_SHC_RAWMAG_EN_VAL)     // Set appropriate offset if the sensor falls in stat2
                        usTmpStatptr = ucBitOffset - 8;
                    ucTmpStatVal = VREG_ENABLE_SUCCESS;             // success status value for VREG status reg


                    if ((1 << ucBitOffset) & usData)                // this sensor's enable bit is set
                    {
                        if ((usPREV_SHC_STATE & (1 << ucBitOffset)) == 0) // this bit was not previously set, need to set this sensor to FULL_POWER      
                        {                           
                            SET_PARAMS.ucid = SENSOR[ucAdjustedPtr - VREG_SHC_ACC_EN_VAL].id;
                            
                            if (set_state_data(SET_PARAMS))         // Set this sensor to FULL POWER
                                ucTmpStatVal = VREG_ENA_DIS_FAIL;   // Error occurred, write an error code to the sensor status register

                            *usStat_ptr &= ~(STATUS1_MASK << (2*(usTmpStatptr - VREG_SHC_ACC_EN_VAL))); //clear stat bits before updating
                            *usStat_ptr |= (ucTmpStatVal << (2*(usTmpStatptr - VREG_SHC_ACC_EN_VAL)));  // Write applicable status value to VREG status register
                            *usSHC_ptr |= (1 << ucBitOffset);       // Set the sensor enable bit in VREG00          
                        }
                        //else if this sensor's enable bit was previously set, no need to do anything 
                    }

                    else if (usPREV_SHC_STATE & (1 << ucBitOffset)) // this sensor's enable bit is not set now, but it was enabled previously, send LOW POWER command 
                    {
                        ucTmpStatVal = VREG_DISABLE_SUCCESS;        //successful disable status value

                        SET_PARAMS.ucid = SENSOR[ucAdjustedPtr - VREG_SHC_ACC_EN_VAL].id;
                        SET_PARAMS.ucPowerState = LOW_POWER;        //set sensor to LOW POWER
                        if (set_state_data(SET_PARAMS))             // Sensor should be disabled
                            ucTmpStatVal = VREG_ENA_DIS_FAIL;       // Error occurred, write an error code to the sensor status register

                        *usStat_ptr &= ~(STATUS1_MASK << (2*(usTmpStatptr - VREG_SHC_ACC_EN_VAL))); //clear stat bits before updating
                        *usStat_ptr |= (ucTmpStatVal << (2*(usTmpStatptr - VREG_SHC_ACC_EN_VAL))); // Write appropriate status value to the status register
                        *usSHC_ptr &= ~(1 << ucBitOffset);          // Clear the sensor enable bit in VREG00        
                    }
                }
            }
            
            if (ucTmpStatVal == VREG_ENA_DIS_FAIL) 
                return SET_FEAT_FAIL;
        }   //end of SHC register

    /*****************SENSITIVITY-CHANGE COMMAND********************/
        if (ucRegOffset >= VREG_ACSEN && ucRegOffset <= VREG_RGYSEN) // data will be written to VREG sensitivity register
        {
            usStat_ptr = (UINT16 *)&VREGS.stat;                     // Reset the status register pointer to the first status register (VREG3C)
            usStat_ptr++;                                           // Increment to status register (VREG3D) for change in sensitivity (CS) stat bits
        
            for (ucTmpPtr = VREG_ACSEN; ucTmpPtr <= VREG_RGYSEN; ucTmpPtr++) // Loop through the VREG sensitivity registers
            {
                if ( (ucRegOffset == ucTmpPtr) &&                   // If current register matches desired one, and that sensor is enabled...
                    (*usSHC_ptr & (1 << (ucTmpPtr + VREG_ACSEN))) )
                {
                    ucAdjustedPtr = ucTmpPtr - VREG_ACSEN;          //offset from first sensitivity VREG
                    
                    ucId = SENSOR[ucAdjustedPtr].id;                // Set variable equal to id of current sensor
                    break;
                }
            }
            
            if ((ucTmpPtr - VREG_ACSEN) < 4)                        // Sensitivity status of accel, gyro, compass, and orientation, are in VREG 0x3D (3-bits each) starting at bit 4
                ucAdjustedPtr = (3*ucAdjustedPtr) + 4;              // calculate proper ptr value
                    
            else if ((ucTmpPtr - VREG_ACSEN) >= 4 && (ucTmpPtr - VREG_ACSEN) < 9) //Sensitivity status of inclinometer, 2 reserved sensors, raw accelerometr, and raw manometer are in VREG 0x3E
            {
                usStat_ptr++;                                       // point to next VREG status register (VREG3E)
                ucAdjustedPtr = 3 * (-(4 - (ucTmpPtr - VREG_ACSEN))); // calculate proper ptr value
            }

            else                                                    
            {                                                       // Last sensor (raw gyrometer) is in VREG 0x3F
                usStat_ptr += 2;                                    // pointer to last VREG status register
                ucAdjustedPtr = 0;
            }

            SET_PARAMS.ucid = ucId;
            SET_PARAMS.bSensitivityValid = TRUE;                    //update with valid sensitivity value
            SET_PARAMS.usSensitivityVal = usData;
 
            if (set_state_data(SET_PARAMS))                         // If sensor is enabled, sensitivity register will be written to
            {
                *usStat_ptr &= ~(STATUS2_MASK << ucAdjustedPtr);    // Clear status bits before writing to them 
                *usStat_ptr |= (VREG_UPDATE_SENS_FAIL << ucAdjustedPtr);   // Set status bits with Update Sensitivity error (one-shot status that is only valid immediately after a VREGWrite to Sensitivity update because it is shared with Data Rate update)
                return SET_FEAT_FAIL;
            }
            
            else
            {
                *usRegPtr = usData;                                 // write succesful, update new sensitivity value in sensitivity register
                *usStat_ptr &= ~(STATUS2_MASK << ucAdjustedPtr);    // Clear status bits before writing to them            
                *usStat_ptr |= (VREG_UPDATE_SENS_SUCCESS << ucAdjustedPtr); // set status bits with Update Sensitivity Success status (one-shot status that is only valid immediately after a VREGWrite to Sensitivity update because it is shared with Data Rate update)
            }
        }                                                           //end of SENSITIVITY command
    
    /*****************DATA-RATE-CHANGE COMMAND********************/
        if (ucRegOffset >= VREG_ACDXR && ucRegOffset <= VREG_RGYDR) // data will be written to VREG data rate register
        {
            usStat_ptr = (UINT16 *)&VREGS.stat;                     // Set the status register pointer to the first status register
            usStat_ptr++;                                           // Increment to status register (VREG3D) for data rate stat bits

            for(ucTmpPtr = VREG_ACDXR; ucTmpPtr <= VREG_RGYDR; ucTmpPtr++) // Loop through VREG data rate registers
            {
                if ( (ucRegOffset == ucTmpPtr) &&                   // If current register matches desired one, and that sensor is enabled...
                    (*usSHC_ptr & (1 << (ucTmpPtr - 8))) )
                {
                    ucAdjustedPtr = ucTmpPtr - VREG_ACDXR;          //offset from first data rate VREG
                    

                    ucId = SENSOR[ucAdjustedPtr].id;                // Set variable equal to id of current sensor
                    break;
                }
            }
            
            if ((ucTmpPtr - VREG_ACDXR) < 4)                        // Data rate status of accel, gyro, compass, and orientation, are in VREG 0x3D (3-bits each) starting at bit 4
                ucAdjustedPtr = (3*ucAdjustedPtr) + 4;              // calculate proper ptr value
                
            else if ((ucTmpPtr - VREG_ACDXR) >= 4 && (ucTmpPtr - VREG_ACDXR) < 9) //Data rate status of inclinometer, 2 reserved sensors, raw accelerometr, and raw manometer are found in VREG 0x3E
            {
                usStat_ptr++;                                       // point to next VREG status register
                ucAdjustedPtr = 3 * (-(4 - (ucTmpPtr - VREG_ACDXR))); // calculate proper ptr value
            }
              
            else            
            {                                                       // Last sensor (raw gyrometer) is in VREG 0x3F
                usStat_ptr += 2;                                    // pointer to last VREG status register
                ucAdjustedPtr = 0;
            }
                
            SET_PARAMS.ucid = ucId;
            SET_PARAMS.bDataRateValid = TRUE;                       //update sensor with valid data rate value
            SET_PARAMS.usDataRateVal = usData;
            
            if (set_state_data(SET_PARAMS))                         // If sensor is enabled, data rate register will be written to
            {
                *usStat_ptr &= ~(STATUS2_MASK << ucAdjustedPtr);    // Clear status bits before writing to them   
                *usStat_ptr |= (VREG_UPDATE_DATARATE_FAIL << ucAdjustedPtr); // Update status bits with error (one-shot status that is only valid immediately after a VREGWrite to Data Rate update because it is shared with Sensitivity update)
                return SET_FEAT_FAIL;
            }
            
            else
            {
                *usRegPtr = usData;                                 // write succesful, update new sensitivity value in sensitivity register
                *usStat_ptr &= ~(STATUS2_MASK << ucAdjustedPtr);    // Clear status bits before writing to them    
                *usStat_ptr |= (VREG_UPDATE_DATARATE_SUCCESS << ucAdjustedPtr); // Update status bits with success status (one-shot status that is only valid immediately after a VREGWrite to Data Rate update because it is shared with Sensitivity update)
            }
        }                                                           //end of DATA_RATE command

    /*****************STATUS-BIT-CLEAR COMMAND********************/
        if (ucRegOffset >= VREG_STAT1 && ucRegOffset <= VREG_STAT4) // offset is in status registers field
        {
            usStat_ptr = (UINT16 *)&VREGS.stat;                     // Reset the status register pointer to the first status register
		
            for (ucTmpPtr = 0; ucTmpPtr < 8; ucTmpPtr++)            //validate current status field = enable_disable_failed
            {
                usMask = (VREG_ENA_DIS_FAIL << (ucTmpPtr * 2) );    //mask individual sensor status fields
                if ( ( usData & usMask) == 0 )                      //clear these stat bits from input data if valid fields exist in status register
                {
                    if ( ucRegOffset == VREG_STAT1)                 //all 8 sensor status fields are valid for possilbe update
                    {
                        if (( *usStat_ptr & usMask) == usMask )     //valid status field contents
                        {
                            *usStat_ptr &= ~usMask;                 //clear these two bits of status field preserving the rest of the register
                        }
                    }
                    else if ((ucRegOffset == VREG_STAT2) && (ucTmpPtr < 2)) // only 1st two status fields of VREG_STAT2 are valid for possible update
                    {
                        if (( *(usStat_ptr+1) & usMask) == usMask ) //valid status field contents
                        {
                            *(usStat_ptr+1) &= ~usMask;             //clear these two bits of status field preserving the rest of the register
                        }
                    }
                }
            }
        }
    }

    if (EC_DATA_AVAIL)                                              // Check if the device has new data
    {                                                              
         ucRet = i2c_cmd_WrRd (READ,                                // Read the data from the SSC7150
                            0,                                      //num of cmd bytes
                            0,                                      //cmd buf (ignored)
                            BYTE_ADJ_VAL,                           //num of bytes to read
                            ucRx_data,                              //recv buf 
                            TRUE);                                  //actual # of bytes SSC7150 returns is in 1st two bytes of read packet, this flag(=TRUE) means "use the 1st two bytes as the actual read packet length"

       if (ucRet) 
           return HID_INT_FAIL;

        for (ucSensorNum = 0; ucSensorNum < NUM_SENS; ucSensorNum++) //go through the sensor list to see where to put the received data
        {          
            if (ucRx_data[2] == SENSOR[ucSensorNum].id)           //1st two bytes of read data packet hold size of packet, then data begins at byte 2
            {
                if (*usSHC_ptr & (1 << (ucTmpPtr + VREG_SHC_ACC_EN_VAL))) // If the incoming data belongs to an enabled sensor, store it in VREG data registers
                { 
                    parse_update_VREG_data(ucSensorNum, ucRx_data);
                    break;
                }
            }
        }
    }
    
    return SUCCESS;
}
