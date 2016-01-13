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
 
#ifndef MPU9150_H
#define MPU9150_H

#include <stdint.h>  			// for uint32_t etc.
#include <stdbool.h>

 // Reference: 
 // product spec
 // http://www.inertialelements.com/docs/PS-MPU-9150A-00v4_3.pdf 
 
 // register map
 // http://www.inertialelements.com/docs/RM-MPU-9150A-00v4_2.pdf

 // AD0 Pin 
 // the eight-bit device address, so shift seven-bit addresses left by one!

#define AD0 0
#if AD0
#define MPU9150_ADDRESS			0x69  		// Device address when AD0 = 1, 1101001
#else
#define MPU9150_ADDRESS			0x68  		// Device address when AD0 = 0, 1101000
#endif 

#define AK8975A_ADDRESS			0x0C 		// deprecated in Rev 4.2

// Magnetometer Registers
#define WHO_AM_I_AK8975A 		0x00 		// should return 0x48
#define INFO             		0x01
#define AK8975A_ST1      		0x02  		// data ready status bit 0
#define AK8975A_XOUT_L   		0x03  		// data
#define AK8975A_XOUT_H   		0x04
#define AK8975A_YOUT_L   		0x05
#define AK8975A_YOUT_H  		0x06
#define AK8975A_ZOUT_L   		0x07
#define AK8975A_ZOUT_H   		0x08
#define AK8975A_ST2      		0x09  		// Data overflow bit 3 and data read error status bit 2
#define AK8975A_CNTL     		0x0A  		// Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0

#define AK8975A_CNTL_POWERDOWN_MODE 	0x00
#define AK8975A_CNTL_SINGLE_MODE 		0x01
#define AK8975A_CNTL_SELFTEST_MODE 		0x08
#define AK8975A_CNTL_FUSEROM_MODE 		0x0F
#define AK8975A_CNTL_16BIT_MODE 		0x10

#define AK8975A_CNTL2	 		0x0B
#define AK8975A_CNTL2_SRESET	0x01

#define AK8975A_ASTC			0x0C  		// Self test control
#define AK8975A_ASTC_NOTEST		0x0 		// NORMAL
#define AK8975A_ASTC_GENTEST	0x40 		// NORMAL

#define AK8975A_ASAX			0x10  		// Fuse ROM x-axis sensitivity adjustment value
#define AK8975A_ASAY			0x11  		// Fuse ROM y-axis sensitivity adjustment value
#define AK8975A_ASAZ			0x12  		// Fuse ROM z-axis sensitivity adjustment value
 
// deprecated in Rev 4.2
#define XGOFFS_TC        		0x00 		// Bit 7 PWR_MODE, bits 6:1 XG_OFFS_TC, bit 0 OTP_BNK_VLD                 
#define YGOFFS_TC        		0x01                                                                          
#define ZGOFFS_TC        		0x02
#define X_FINE_GAIN      		0x03 		// [7:0] fine gain
#define Y_FINE_GAIN      		0x04
#define Z_FINE_GAIN      		0x05
#define XA_OFFSET_H      		0x06 		// User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   		0x07
#define YA_OFFSET_H      		0x08
#define YA_OFFSET_L_TC   		0x09
#define ZA_OFFSET_H      		0x0A
#define ZA_OFFSET_L_TC   		0x0B

// Gyro and Acc registers
#define SELF_TEST_X				0x0D
#define SELF_TEST_Y				0x0E    
#define SELF_TEST_Z				0x0F
#define SELF_TEST_A				0x10
#define XG_OFFSET_H				0x13	// User-defined trim values for gyroscope, populate with calibration routine
#define XG_OFFSET_L				0x14	// deprecated in Rev 4.2
#define YG_OFFSET_H				0x15	// deprecated in Rev 4.2
#define YG_OFFSET_L				0x16	// deprecated in Rev 4.2
#define ZG_OFFSET_H				0x17	// deprecated in Rev 4.2
#define ZG_OFFSET_L				0x18	// deprecated in Rev 4.2
#define SMPLRT_DIV				0x19
#define CONFIG					0x1A
#define GYRO_CONFIG				0x1B
#define ACCEL_CONFIG			0x1C
#define ACCEL_CONFIG2			0x1D
#define FF_THR					0x1D	// Free-fall // deprecated in Rev 4.2
#define FF_DUR					0x1E	// Free-fall // deprecated in Rev 4.2
#define MOT_THR					0x1F	// deprecated in Rev 4.2 // Motion detection threshold bits [7:0]
#define MOT_DUR					0x20	// deprecated in Rev 4.2 // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR				0x21	// deprecated in Rev 4.2 // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR				0x22	// deprecated in Rev 4.2 // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms
#define FIFO_EN					0x23
#define I2C_MST_CTRL			0x24   
#define I2C_SLV0_ADDR			0x25
#define I2C_SLV0_REG			0x26
#define I2C_SLV0_CTRL			0x27
#define I2C_SLV1_ADDR			0x28
#define I2C_SLV1_REG			0x29
#define I2C_SLV1_CTRL			0x2A
#define I2C_SLV2_ADDR			0x2B
#define I2C_SLV2_REG			0x2C
#define I2C_SLV2_CTRL			0x2D
#define I2C_SLV3_ADDR			0x2E
#define I2C_SLV3_REG			0x2F
#define I2C_SLV3_CTRL			0x30
#define I2C_SLV4_ADDR			0x31
#define I2C_SLV4_REG			0x32
#define I2C_SLV4_DO				0x33
#define I2C_SLV4_CTRL			0x34
#define I2C_SLV4_DI				0x35
#define I2C_MST_STATUS			0x36
#define INT_PIN_CFG				0x37
#define INT_ENABLE				0x38
#define DMP_INT_STATUS			0x39	// Check DMP interrupt
#define INT_STATUS				0x3A
#define ACCEL_XOUT_H			0x3B
#define ACCEL_XOUT_L			0x3C
#define ACCEL_YOUT_H			0x3D
#define ACCEL_YOUT_L			0x3E
#define ACCEL_ZOUT_H			0x3F
#define ACCEL_ZOUT_L			0x40
#define TEMP_OUT_H				0x41
#define TEMP_OUT_L				0x42
#define GYRO_XOUT_H				0x43
#define GYRO_XOUT_L				0x44
#define GYRO_YOUT_H				0x45
#define GYRO_YOUT_L				0x46
#define GYRO_ZOUT_H				0x47
#define GYRO_ZOUT_L				0x48
#define EXT_SENS_DATA_00		0x49
#define EXT_SENS_DATA_01		0x4A
#define EXT_SENS_DATA_02		0x4B
#define EXT_SENS_DATA_03		0x4C
#define EXT_SENS_DATA_04		0x4D
#define EXT_SENS_DATA_05		0x4E
#define EXT_SENS_DATA_06		0x4F
#define EXT_SENS_DATA_07		0x50
#define EXT_SENS_DATA_08		0x51
#define EXT_SENS_DATA_09		0x52
#define EXT_SENS_DATA_10		0x53
#define EXT_SENS_DATA_11		0x54
#define EXT_SENS_DATA_12		0x55
#define EXT_SENS_DATA_13		0x56
#define EXT_SENS_DATA_14		0x57
#define EXT_SENS_DATA_15		0x58
#define EXT_SENS_DATA_16		0x59
#define EXT_SENS_DATA_17		0x5A
#define EXT_SENS_DATA_18		0x5B
#define EXT_SENS_DATA_19		0x5C
#define EXT_SENS_DATA_20		0x5D
#define EXT_SENS_DATA_21		0x5E
#define EXT_SENS_DATA_22		0x5F
#define EXT_SENS_DATA_23		0x60

//#define MOT_DETECT_STATUS 0x61 		// deprecated in Rev 4.2
#define I2C_SLV0_DO				0x63
#define I2C_SLV1_DO				0x64
#define I2C_SLV2_DO				0x65
#define I2C_SLV3_DO				0x66
#define I2C_MST_DELAY_CTRL		0x67
#define SIGNAL_PATH_RESET		0x68

//#define MOT_DETECT_CTRL		0x69	// deprecated in Rev 4.2
#define USER_CTRL				0x6A  
#define PWR_MGMT_1       		0x6B	// Device defaults to the SLEEP mode
#define PWR_MGMT_2       		0x6C

//#define DMP_BANK        		0x6D	// deprecated in Rev 4.2// Activates a specific bank in the DMP
//#define DMP_RW_PNT       		0x6E	// deprecated in Rev 4.2// Set read/write pointer to a specific start address in specified DMP bank
//#define DMP_REG       		0x6F	// deprecated in Rev 4.2// Register in DMP from which to read or to which to write
//#define DMP_REG_1        		0x70
//#define DMP_REG_2        		0x71

#define FIFO_COUNTH      		0x72
#define FIFO_COUNTL      		0x73
#define FIFO_R_W         		0x74
#define WHO_AM_I_MPU9150 		0x75	// Should return 0x68, 0x0 110100 0 , as device address
 
  
// Set initial input parameters
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

//uint8_t Mmode = 0x02;        			// 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read

void readAccelFloatMG(float *xyz);
void readGyroFloatDeg(float *xyz);
void readMagFloatUT(float *xyz);
void readAccelData(int16_t * destination);
void readGyroData(int16_t * destination);
void readMagData(int16_t * destination);
void readQuaternion(float * q);
void initAK8975A(float * destination);
int16_t readTempData(void);
void resetMPU9150(void);
void initMPU9150(void);
void MPU9150SelfTest(float * destination);
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

void calibrateMPU9150(float[],float[]);
void MPU9250_Setup(void);

#endif

