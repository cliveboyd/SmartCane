/*******************************************************************************
 *	@file   MPU9150.h
 *	@device MPU-9250 I2C 9-Axis Inertial Sensor
 *	@brief  Header file for MPU9150 Device Driver.
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
 
#ifndef MPU9250_H

#define MPU9250_H

#include <stdint.h>  			// for uint32_t etc.
#include <stdbool.h>

   
//// Set initial input parameters
typedef enum {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
} Ascale_t;
 
typedef enum {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
} Gscale_t;

typedef enum {
  MFS_14BITS = 0,
  MFS_16BITS
} Mscale_t;

// uint8_t Mmode = 0x02;        			// 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read

void readAccelFloatMG(float *xyz);
void readGyroFloatDeg(float *xyz);
void readMagFloatUT(float *xyz);

void readAccelData(int16_t * destination);
void readGyroData(int16_t * destination);
void readMagData(int16_t * destination);

void readQuaternion(float * q);

void initAK8963(float * destination);

void resetMPU9250(void);
void initMPU9250(void);
void MPU9250SelfTest(float * destination);

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

void accelgyrocalMPU9250(float * dest1, float * dest2);
	
int16_t MPU9250_get_temperature(void);
int16_t readTempData(void);
uint8_t MPU9250_WhoAmI(void);

void calibrateMPU9250(float[],float[]);
void MPU9250_Setup(void);

void magcalMPU9250(float * dest1, float * dest2, uint16_t count);
void CalibrateMagnetometer(uint16_t count);

uint8_t MPU9250_WhoAmI(void);


void getMres(void);
void getGres(void);
void getAres(void);

void MPU9250_setup(void);

void readMagTest(float * destination);
void MPU9250_Timed_Interupt(void);
void MPU9250_Get_Euler(float pitch, float roll, float yaw);

#endif

