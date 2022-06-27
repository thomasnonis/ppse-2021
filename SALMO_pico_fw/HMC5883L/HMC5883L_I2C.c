#include "HMC5883L_I2C.h"
#include "hardware/i2c.h"

#include <stdio.h>
#include "pico/stdlib.h"

void HMC5883L_I2C_ByteWrite(uint8_t slaveAddr, uint8_t* pBuffer, uint8_t writeAddr){
    uint8_t buffer[2];
    buffer[0]=writeAddr;
    buffer[1]=*pBuffer;
    
    int ret=0;
    ret=i2c_write_timeout_us(I2C_PERIPHERAL,slaveAddr, buffer, 2, false, 1000); //1ms timeout max
    if(ret==PICO_ERROR_GENERIC||ret==PICO_ERROR_TIMEOUT){
        printf("HMC5883L_I2C_ByteWrite: device not present or timeout reached\n");
    }
};
void HMC5883L_I2C_BufferRead(uint8_t slaveAddr,uint8_t* pBuffer, uint8_t readAddr, uint16_t NumByteToRead){
    int ret=0;

    // before read the protocol needs to send the register address to the slave (see i2c protocol)
    ret=i2c_write_timeout_us(I2C_PERIPHERAL, slaveAddr, &readAddr, 1, true, 1000); //1ms timeout max
    if(ret==PICO_ERROR_GENERIC||ret==PICO_ERROR_TIMEOUT){
        printf("HMC5883L_I2C_BufferRead: device not present or timeout reached\n");
    }
    ret=i2c_read_timeout_us(I2C_PERIPHERAL, slaveAddr, pBuffer, NumByteToRead, false, 1000); //1ms timeout max
};