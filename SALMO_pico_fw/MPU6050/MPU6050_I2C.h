#ifndef I2C_MPU6050_H_
#define I2C_MPU6050_H_

#include "hardware/i2c.h"
#include "MPU6050.h"
#include <stdio.h>

#define I2C_PERIPHERAL (i2c1)

void MPU6050_I2C_ByteWrite(uint8_t slaveAddr, uint8_t* pBuffer, uint8_t writeAddr);
void MPU6050_I2C_BufferRead(uint8_t slaveAddr,uint8_t* pBuffer, uint8_t readAddr, uint16_t NumByteToRead);

#endif /* BSP_MPU6050_H_ */
