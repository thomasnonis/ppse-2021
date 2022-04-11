#ifndef _MPU6050_BSP_H_
#define _MPU6050_BSP_H_

#include <stdio.h>
#include "hardware/i2c.h"

void MPU6050_I2C_Init(){
    // Initialize I2C
    i2c_init(i2c_default, 400 * 1000);
    
};
void MPU6050_I2C_ByteWrite(uint8_t slaveAddr, uint8_t* pBuffer, uint8_t writeAddr){
    i2c_write_blocking(i2c_default, addr, buf, 2, false);
};
void MPU6050_I2C_BufferRead(uint8_t slaveAddr,uint8_t* pBuffer, uint8_t readAddr, uint16_t NumByteToRead){

};

#endif