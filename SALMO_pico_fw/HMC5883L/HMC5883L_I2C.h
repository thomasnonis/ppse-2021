#ifndef _HMC5883L_I2C_H_
#define _HMC5883L_I2C_H_

#include <stdio.h>

#define I2C_PERIPHERAL (i2c1)

void HMC5883L_I2C_ByteWrite(uint8_t slaveAddr, uint8_t* pBuffer, uint8_t writeAddr);
void HMC5883L_I2C_BufferRead(uint8_t slaveAddr,uint8_t* pBuffer, uint8_t readAddr, uint16_t NumByteToRead);

#endif /* _HMC5883L_I2C_H_ */
