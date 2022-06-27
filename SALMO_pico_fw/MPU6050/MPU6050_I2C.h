#ifndef _MPU6050_I2C_H_
#define _MPU6050_I2C_H_

#include <stdio.h>

#define I2C_PERIPHERAL (i2c1)

void MPU6050_I2C_ByteWrite(uint8_t slaveAddr, uint8_t* pBuffer, uint8_t writeAddr);
void MPU6050_I2C_BufferRead(uint8_t slaveAddr,uint8_t* pBuffer, uint8_t readAddr, uint16_t NumByteToRead);

#endif /* _MPU6050_I2C_H_ */
