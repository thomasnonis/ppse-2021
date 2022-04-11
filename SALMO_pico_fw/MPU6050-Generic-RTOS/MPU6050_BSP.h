#ifndef BSP_MPU6050_H_
#define BSP_MPU6050_H_

#include <stdint.h>

void MPU6050_I2C_Init();
void MPU6050_I2C_ByteWrite(uint8_t slaveAddr, uint8_t* pBuffer, uint8_t writeAddr);
void MPU6050_I2C_BufferRead(uint8_t slaveAddr,uint8_t* pBuffer, uint8_t readAddr, uint16_t NumByteToRead);

#endif /* BSP_MPU6050_H_ */
