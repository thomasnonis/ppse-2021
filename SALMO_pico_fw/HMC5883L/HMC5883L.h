// I2Cdev library collection - HMC5883L I2C device class header file
// Based on Honeywell HMC5883L datasheet, 10/2010 (Form #900405 Rev B)
// 6/12/2012 by Jeff Rowberg <jeff@rowberg.net>
// 6/6/2015 by Andrey Voloshin <voloshin@think.in.ua>
// 03/28/2017 by Kamnev Yuriy <kamnev.u1969@gmail.com>
// 11/04/2022 by Tollardo Simone, Tommaso Canova, Lisa Santarossa, Gabriele Berretta
//
// Changelog:
//     2022-04-11 - ported to RP2040
//     2017-03-28 - ported to STM32 using Keil MDK Pack
//     2015-06-06 - ported to STM32 HAL library from Arduino code
//     2012-06-12 - fixed swapped Y/Z axes
//     2011-08-22 - small Doxygen comment fixes
//     2011-07-31 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef _HMC5883L_H_
#define _HMC5883L_H_

#include "hardware/i2c.h"
#include <stdbool.h>

//ID registers
uint8_t HMC5883L_getIDA();
uint8_t HMC5883L_getIDB();
uint8_t HMC5883L_getIDC();

void HMC5883L_initialize();
//True if device ID is HMC5883L ID
bool isHMC();

// CONFIG_A register
uint8_t HMC5883L_getSampleAveraging();
void HMC5883L_setSampleAveraging(uint8_t averaging);
uint8_t HMC5883L_getDataRate();
void HMC5883L_setDataRate(uint8_t rate);
uint8_t HMC5883L_getMeasurementBias();
void HMC5883L_setMeasurementBias(uint8_t bias);

// CONFIG_B register
uint8_t HMC5883L_getGain();
void HMC5883L_setGain(uint8_t gain);

// MODE register
uint8_t HMC5883L_getMode();
void HMC5883L_setMode(uint8_t mode);

// DATA* registers
void HMC5883L_getHeading(int16_t *x, int16_t *y, int16_t *z);
int16_t HMC5883L_getHeadingX();
int16_t HMC5883L_getHeadingY();
int16_t HMC5883L_getHeadingZ();

// STATUS register
bool HMC5883L_getLockStatus();
bool HMC5883L_getReadyStatus();

#endif /* _HMC5883L_H_ */
