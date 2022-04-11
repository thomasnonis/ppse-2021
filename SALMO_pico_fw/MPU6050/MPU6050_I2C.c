#ifndef _MPU6050_I2C_H_
#define _MPU6050_I2C_H_

#include "MPU6050_I2C.h"

void MPU6050_I2C_ByteWrite(uint8_t slaveAddr, uint8_t* pBuffer, uint8_t writeAddr){
    i2c_write_blocking(I2C_PERIPHERAL,slaveAddr, pBuffer, 1, false);
};
void MPU6050_I2C_BufferRead(uint8_t slaveAddr,uint8_t* pBuffer, uint8_t readAddr, uint16_t NumByteToRead){
    // before read the protocol needs to send the register address to the slave (see i2c protocol)
    i2c_write_blocking(I2C_PERIPHERAL, slaveAddr, &readAddr, 1, true);
    i2c_read_blocking(I2C_PERIPHERAL, slaveAddr, pBuffer, NumByteToRead, false);
};

#endif