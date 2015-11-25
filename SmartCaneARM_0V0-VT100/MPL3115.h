/*******************************************************************************
 *	@file   MPL3115.h
 *	@device MPU-9250 I2C 9-Axis Inertial Sensor
 *	@brief  Header file for MPL3115 Device Driver.
 *	@author Shun Bai (wanyancan@gmail.com)
********************************************************************************

Copyright (c) <2015> <Shun Bai (wanyancan at gmail)>

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

 */
#ifndef MPL3115_H
#define MPL3115_H
#include <stdint.h>  // for uint32_t etc.
#include <stdbool.h>

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define MPL3115A2_ADDRESS                       (0x60)    // 1100000
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    #define MPL3115A2_REGISTER_STATUS               (0x00)
    #define MPL3115A2_REGISTER_STATUS_TDR 0x02
    #define MPL3115A2_REGISTER_STATUS_PDR 0x04
    #define MPL3115A2_REGISTER_STATUS_PTDR 0x08

    #define MPL3115A2_REGISTER_PRESSURE_MSB         (0x01)
    #define MPL3115A2_REGISTER_PRESSURE_CSB         (0x02)
    #define MPL3115A2_REGISTER_PRESSURE_LSB         (0x03)

    #define MPL3115A2_REGISTER_TEMP_MSB             (0x04)
    #define MPL3115A2_REGISTER_TEMP_LSB             (0x05)

    #define MPL3115A2_REGISTER_DR_STATUS            (0x06)

    #define MPL3115A2_OUT_P_DELTA_MSB               (0x07)
    #define MPL3115A2_OUT_P_DELTA_CSB               (0x08)
    #define MPL3115A2_OUT_P_DELTA_LSB               (0x09)

    #define MPL3115A2_OUT_T_DELTA_MSB               (0x0A)
    #define MPL3115A2_OUT_T_DELTA_LSB               (0x0B)

    #define MPL3115A2_WHOAMI                        (0x0C)

#define MPL3115A2_PT_DATA_CFG 0x13
#define MPL3115A2_PT_DATA_CFG_TDEFE 0x01
#define MPL3115A2_PT_DATA_CFG_PDEFE 0x02
#define MPL3115A2_PT_DATA_CFG_DREM 0x04

#define MPL3115A2_CTRL_REG1                     (0x26)
#define MPL3115A2_CTRL_REG1_SBYB 0x01
#define MPL3115A2_CTRL_REG1_OST 0x02
#define MPL3115A2_CTRL_REG1_RST 0x04
#define MPL3115A2_CTRL_REG1_OS1 0x00
#define MPL3115A2_CTRL_REG1_OS2 0x08
#define MPL3115A2_CTRL_REG1_OS4 0x10
#define MPL3115A2_CTRL_REG1_OS8 0x18
#define MPL3115A2_CTRL_REG1_OS16 0x20
#define MPL3115A2_CTRL_REG1_OS32 0x28
#define MPL3115A2_CTRL_REG1_OS64 0x30
#define MPL3115A2_CTRL_REG1_OS128 0x38
#define MPL3115A2_CTRL_REG1_RAW 0x40
#define MPL3115A2_CTRL_REG1_ALT 0x80
#define MPL3115A2_CTRL_REG1_BAR 0x00
#define MPL3115A2_CTRL_REG2                     (0x27)
#define MPL3115A2_CTRL_REG3                     (0x28)
#define MPL3115A2_CTRL_REG4                     (0x29)
#define MPL3115A2_CTRL_REG5                     (0x2A)

#define MPL3115A2_REGISTER_STARTCONVERSION      (0x12)
/*=========================================================================*/

bool MPL3115A2_init(void);
float MPL3115A2_getPressure(void);
float MPL3115A2_getAltitude(void);
float MPL3115A2_getTemperature(void);

uint8_t MPL3115A2_read8(uint8_t a);
bool MPL3115A2_write8(uint8_t a, uint8_t d);

#endif 
