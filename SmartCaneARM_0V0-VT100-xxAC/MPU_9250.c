/* MPU9250_MS5637_t3 Basic Example Code
 by: Kris Winer
 date: April 1, 2014
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 Demonstrate basic MPU-9250 functionality including parameterizing the register addresses, initializing the sensor, 
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to 
 allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and 
 Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.
 
 This sketch is intended specifically for the MPU9250+MS5637 Add-on shield for the Teensy 3.1.
 It uses SDA/SCL on pins 17/16, respectively, and it uses the Teensy 3.1-specific Wire library i2c_t3.h.
 The MS5637 is a simple but high resolution pressure sensor, which can be used in its high resolution
 mode but with power consumption of 20 microAmp, or in a lower resolution mode with power consumption of
 only 1 microAmp. The choice will depend on the application.
 
 SDA and SCL should have external pull-up resistors (to 3.3V).
 4K7 resistors are on the MPU9250+MS5637 breakout board.
 
 Hardware setup:
 MPU9250 Breakout --------- Arduino
 VDD ---------------------- 3.3V
 VDDI --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND
 
 Note: The MPU9250 is an I2C sensor and uses the Arduino Wire library. 
 Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
 We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
 */
 


#include <stdint.h>  			// for uint32_t etc.
#include <stdbool.h>
#include <stdio.h>
#include <MPU_9250.h>
#include "Communication.h"
#include "math.h"
#include <nrf_delay.h>
#include "global.h"

Ascale_t Ascale = AFS_2G;     									// AFS_2G, AFS_4G, AFS_8G, AFS_16G
Gscale_t Gscale = GFS_1000DPS; 									// GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
Mscale_t Mscale = MFS_16BITS;

// See also MPU-9250 Register Map and Descriptions, Revision 4.0, RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 for registers not listed in 
// above document; the MPU9250 and MPU9150 are virtually identical but the latter has a different register map

//Magnetometer Registers
#define AK8963_ADDRESS   0x0C
#define WHO_AM_I_AK8963  0x00	// should return 0x48

#define INFO             0x01
#define AK8963_ST1       0x02	// data ready status bit 0

#define AK8963_XOUT_L	 0x03	// data
#define AK8963_XOUT_H	 0x04
#define AK8963_YOUT_L	 0x05
#define AK8963_YOUT_H	 0x06
#define AK8963_ZOUT_L	 0x07
#define AK8963_ZOUT_H	 0x08

#define AK8963_ST2       0x09	// Data overflow bit 3 and data read error status bit 2

#define AK8963_CNTL      0x0A	// Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C	// Self test control
#define AK8963_I2CDIS    0x0F	// I2C disable

#define AK8963_ASAX      0x10	// Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11	// Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12	// Fuse ROM z-axis sensitivity adjustment value

#define SELF_TEST_X_GYRO 0x00                  
#define SELF_TEST_Y_GYRO 0x01                                                                          
#define SELF_TEST_Z_GYRO 0x02

#define X_FINE_GAIN		 0x03	// [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05

//#define XA_OFFSET_H      0x06	// User-defined trim values for accelerometer
//#define XA_OFFSET_L_TC   0x07
//#define YA_OFFSET_H      0x08
//#define YA_OFFSET_L_TC   0x09
//#define ZA_OFFSET_H      0x0A
//#define ZA_OFFSET_L_TC   0x0B

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E    
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18

#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E   
#define WOM_THR          0x1F   

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23

#define I2C_MST_CTRL     0x24   
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36

#define INT_piN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  								// Check DMP interrupt
#define INT_STATUS       0x3A

#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40

#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42

#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48

#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60

#define MOT_DETECT_STATUS 0x61

#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_delay_msec_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69

#define USER_CTRL        0x6A									// Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B									// Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C

#define DMP_BANK         0x6D									// Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E									// Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F									// Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71 

#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74


#define WHO_AM_I_MPU9250 0x75 									// Should return 0x71

#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E

// Using the MPU9250Teensy 3.1 Add-On shield, ADO is set to 0 
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
#define ADO 0
#if ADO
#define MPU9250_ADDRESS 0x69  										// Device address when ADO = 1
#define AK8963_ADDRESS 0x0C   										// Address of magnetometer
#else
#define MPU9250_ADDRESS 0x68  										// Device address when ADO = 0
#define AK8963_ADDRESS 0x0C											// Address of magnetometer
#endif  

#define SerialDebug false  											// set to true to get Serial output for debugging
#define VT100Debug  false
	
const float pix = 3.14159265358979323846f;

#define ADC_256  0x00 												// define pressure and temperature conversion rates
#define ADC_512  0x02
#define ADC_1024 0x04
#define ADC_2048 0x06
#define ADC_4096 0x08

#define ADC_8192 0x0A
#define ADC_D1   0x40
#define ADC_D2   0x50

//	Specify sensor full scale
uint8_t OSR = ADC_8192;     										// set pressure amd temperature oversample rate

uint8_t Mmode = 0x02;        										// 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
float aRes, gRes, mRes;      										// scale resolutions per LSB for the sensors
  
int16_t accelCount[3];  											// Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   											// Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    											// Stores the 16-bit signed magnetometer sensor output

float magCalibration[3] = {0, 0, 0};								// Factory mag calibration and mag bias
float gyroBias[3]	= {0, 0, 0}	;									// Bias corrections for gyro 
float accelBias[3]	= {0, 0, 0};									// Bias corrections for accelerometer
float magBias[3]	= {0, 0, 0};									// Bias corrections for magnetometer
float magScale[3]	= {0, 0, 0};
int16_t tempCount;            										// Temperature raw count output
float temperature;													// Stores the MPU9250 gyro internal chip temperature in degrees Celsius
float SelfTest[6];													// holds results of gyro and accelerometer self test

//	Global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = 3.1415926 * (40.0f / 180.0f);					// gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = 3.1415926 * (0.0f  / 180.0f);					// gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)

/*	There is a tradeoff in the beta parameter between accuracy and response speed.
	In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
	However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
	Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
	By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
	I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a piD control sense; 
	the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
	In any case, this is the free parameter in the Madgwick filtering and fusion scheme. */
	
float beta = 0.60459979f;											// compute beta, ---> beta = sqrt(3.0f / 4.0f) * GyroMeasError
float zeta = 0;				   										// compute zeta, ---> zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift
																	// the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f 												// these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

float deltat = 0.100; 			         							// integration interval for both filter schemes --> main.sysTimerHandler 100msec

float ax, ay, az, gx, gy, gz, mx, my, mz; 							// variables to hold latest sensor data values 

float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    							// vector to hold quaternion

float eInt[3] = {0.0f, 0.0f, 0.0f};       							// vector to hold integral error for Mahony method

void delay_msec(float mSec) {										// delay_msec in mseconds
	nrf_delay_us(1000.0*mSec);
}

void MPU9250_writeByte_(uint8_t address, uint8_t subAddress, uint8_t data, bool loop) {
   unsigned char data_write[2];
   data_write[0] = subAddress;
   data_write[1] = data;
	
	for(int i=0;i<5;i++) {
		if(I2C_Write(address,
              data_write,
              2,
              0))  																				// no stop bit
			break;
		if (!loop) break;
	}
}


void MPU9250_writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
	MPU9250_writeByte_(address, subAddress, data, true);
}

void MPU9250_readBytes_(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest, bool loop) {     
	int i=0;
	for(;i<5;i++) {
		if(I2C_Write(address,
				  (unsigned char*)&subAddress,
				  1,
				  0))
				  break; 																		// if written length > 0
			if (!loop) break;
	}
	if (i>=5) 
		return;
	for(i=0;i<5;i++) {
		if(I2C_Read(address,
             dest,
             count,
             1))
			break;  																			// if read length > 0
		if (!loop) break;
	}
}

void MPU9250_readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) {     
	MPU9250_readBytes_(address, subAddress, count, dest, true);
} 

unsigned char MPU9250_readByte_(uint8_t address, uint8_t subAddress, bool loop) {
    unsigned char data[1]; 																		// `data` will store the register data     
    MPU9250_readBytes_(address, subAddress, 1, data, loop);
    return data[0]; 
}

unsigned char MPU9250_readByte(uint8_t address, uint8_t subAddress) {
    unsigned char data[1]; 																		// `data` will store the register data     
	MPU9250_readBytes(address, subAddress, 1, data);
    return data[0]; 
}

void MPU9250_setup() {
	int8_t c = MPU9250_readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);	//	Read WHO_AM_I register for MPU-9250
	if (c == 0x71) {												//	WHO_AM_I should always be 0x71 for MPU9250  
		MPU9250SelfTest(SelfTest);									//	Start by performing self test and reporting acceleration trim values
		
		getAres();													//	Get sensor resolutions, only need to do this once
		getGres();
		getMres();

//		printf("Calibrate gyroBias (deg/sec) and accelBias (mg)");
		accelgyrocalMPU9250(gyroBias, accelBias); 					//	Calibrate gyro and accelerometers, load biases in bias registers

		initMPU9250(); 												/*	Initialize device for active mode read of 
																		acclerometer, gyroscope, and temperature */
//		Read the WHO_AM_I register of the magnetometer.
		int8_t d = MPU9250_readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);	//	Read magnetometer WHO_AM_I for AK8963 ---> Should be 0x48

//		Get magnetometer calibration from AK8963 ROM
		initAK8963(magCalibration); 								//	CSB ---> WORKS TO HERE RETURNS MAG CAL VALUES
	
//		Initialize device for active mode read of magnetometer
		magcalMPU9250(magBias, magScale,  8);						//	AK8963 magBias (mGause) and magScale (mGause)

	} 
	else {
//	printf("Could not connect to MPU9250: ");
//	while(1); { 													//	CSB... Don't Loop forever if communication doesn't happen
	}
}

void CalibrateMagnetometer(uint16_t count) {;
	magcalMPU9250(magBias, magScale, count);							//	AK8963 magBias (mGause) and magScale (mGause)
}

void MPU9250_Timed_Interupt() {  									/*	This Pipe needs to be transfered to interupt service
																		routine	based on MPU9250 int pin assertion*/
//	If int pin goes high, all data registers have new data
	if (MPU9250_readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {		//	Check if data ready interrupt

//		if(digitalRead(intpin)) {  									//	Upon interrupt, read data
		readAccelData(accelCount);  								//	Read the xyz adc values
 
//		Now we'll calculate the accleration value into actual g's
		ax = (float)accelCount[0]*aRes - accelBias[0];				//	Get actual g value, this depends on scale being set
		ay = (float)accelCount[1]*aRes - accelBias[1];   
		az = (float)accelCount[2]*aRes - accelBias[2];  

		readGyroData(gyroCount);  									//	Read the xyz adc values

//		Calculate the gyro value into actual degrees per second
		gx = (float)gyroCount[0]*gRes;  							//	Get actual gyro value, this depends on scale being set
		gy = (float)gyroCount[1]*gRes;
		gz = (float)gyroCount[2]*gRes;

		readMagData(magCount);  									// Read the xy/z adc values
		if (magCount[0]+magCount[1]+magCount[2]!=0){
/*			Calculate the magnetometer values in milliGauss
			Include factory calibration per data sheet and user environmental corrections */
			mx = (float)magCount[0]*mRes*magCalibration[0] - magBias[0];  	//	Get actual magnetometer value, this depends on scale being set
			my = (float)magCount[1]*mRes*magCalibration[1] - magBias[1];
			mz = (float)magCount[2]*mRes*magCalibration[2] - magBias[2];

			mx *= magScale[0];
			my *= magScale[1];
			mz *= magScale[2];
	}
}
  
  
/*	Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
	the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
	We have to make some allowance for this orientationmismatch in feeding the output to the quaternion filter.
	For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
	in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
	This is ok by aircraft orientation standards!  
	Pass gyro rate as rad/s */
	
	MadgwickQuaternionUpdate(ax, ay, az, gx*pix/180.0f, gy*pix/180.0f, gz*pix/180.0f,  my,  mx, mz);

//	MahonyQuaternionUpdate(ax, ay, az, gx*pi/180.0f, gy*pi/180.0f, gz*pi/180.0f, my, mx, mz);


/*	Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  
	In this coordinate system, the positive z-axis is down toward Earth. 
		
	Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, 
	looking down on the sensor positive yaw is counterclockwise.

	Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
	
	mpu_roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
	
	These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
	Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations
	must be applied in the correct order which for this configuration is yaw, pitch, and then roll.
	
	For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.*/
	
	yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
	pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
	roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);

	yaw   *= 180.0f / pix; 
	yaw   += 5.125; 					// 5.125 deg TrueNorth Compensation ---> Melbourne Australia on 2014-04-04
	pitch *= 180.0f / pix;
	roll  *= 180.0f / pix;
		 
		
/*	With these settings the filter is updating at a ~145 Hz rate using the Madgwick scheme and 
	>200 Hz using the Mahony scheme.
	
	The filter update rate is determined mostly by the mathematical steps in the respective algorithms, 
	the processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
	an ODR of 10 Hz for the magnetometer produce the above rates, maximum magnetometer ODR of 100 Hz produces
	filter update rates of 36-145 and ~38 Hz for the Madgwick and Mahony schemes, respectively. 
	
	This is presumably because the magnetometer read takes longer than the gyro or accelerometer reads.
	This filter update rate should be fast enough to maintain accurate platform orientation for 
	stabilization control of a fast-moving robot or quadcopter. 
	
	Compare to the update rate of 200 Hz produced by the on-board Digital Motion Processor of Invensense's 
	MPU6050 6DoF and MPU9150 9DoF sensors. */
}

int16_t MPU9250_get_temperature() {
	return readTempData();
}

int16_t readTempData() {											// Read temperature Data from Inertial Sensor
  uint8_t rawData[2];  																// Temperature data stored here
  MPU9250_readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  					// Read the two raw data registers sequentially into data array 
  return (((int16_t)rawData[0] << 8) | rawData[1])/100 ;  							// Turn the MSB and LSB into a 16-bit value
}

void getMres() {
  switch (Mscale) {
 	// Possible magnetometer scales (and their register bit settings) are:
	// 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
          mRes = 10.*4912./8190.; 									// Proper scale to return milliGauss
          break;
    case MFS_16BITS:
          mRes = 10.*4912./32760.0; 								// Proper scale to return milliGauss
          break;
  }
}

void getGres() {
  switch (Gscale) {
 	// Possible gyro scales (and their register bit settings) are:
	// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
          gRes = 250.0/32768.0;
          break;
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 2000.0/32768.0;
          break;
  }
}

void getAres() {
  switch (Ascale)
  {
 	// Possible accelerometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
          aRes = 2.0/32768.0;
          break;
    case AFS_4G:
          aRes = 4.0/32768.0;
          break;
    case AFS_8G:
          aRes = 8.0/32768.0;
          break;
    case AFS_16G:
          aRes = 16.0/32768.0;
          break;
  }
}

uint8_t MPU9250_WhoAmI() {
	uint8_t  d = MPU9250_readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);		// Should Return 0x71
	return d;
}

void readAccelData(int16_t * destination) {
  uint8_t rawData[6];  														// xyz accel register data stored here
  MPU9250_readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);			// Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;				// Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}

void readGyroData(int16_t * destination) {
	uint8_t rawData[6];  													// xyz gyro register data stored here
	MPU9250_readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);		// Read the six raw data registers sequentially into data array

	destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;				// Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
	destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}

void readMagData(int16_t * destination) {

	uint8_t rawData[7];								// xyz gyro register data + ST2 register stored here, must read ST2 at end of data acquisition
	
	destination[0] = 0;
	destination[1] = 0;
	destination[2] = 0;
	
	if(MPU9250_readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { 				// wait for magnetometer data ready bit to be set, Bypass if not ready
		
		delay_msec(10);
		
		MPU9250_readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  	// Read the six raw data and ST2 registers sequentially into data array
		
//		MPU9250_readBytes(AK8963_ADDRESS, AK8963_ST2, 1, &rawData[6]);		// End data read by reading ST2 register
		
		uint8_t c = rawData[6]; 											
		
		if(!(c & 0x08)) { 													// Check if magnetic sensor overflow set, if not then continue
			if(!(c & 0x04)) {												// Check if magnetic sensor read error,   if not then load mag data
				destination[0] = ((int16_t) rawData[1] << 8) | rawData[0];	// Turn the MSB and LSB into a signed 16-bit value & stored as little Endian
				destination[1] = ((int16_t) rawData[3] << 8) | rawData[2];
				destination[2] = ((int16_t) rawData[5] << 8) | rawData[4]; 				
			}
		}
	}
}

void initAK8963(float * destination) {
  // First extract the factory calibration for each magnetometer axis
	
	uint8_t rawData[3];  													// xyz magnetometer calibration data stored here
	MPU9250_writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); 					// Power down magnetometer  
	delay_msec(10);
	
	MPU9250_writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); 					// Enter Fuse ROM access mode
	delay_msec(10);
	
	MPU9250_readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  		// Read the x, y and z-axis calibration values
	
	destination[0] =  (float)(rawData[0] - 128)/256. + 1.;   				// Return xyz axis sensitivity adjustment values, etc.
	destination[1] =  (float)(rawData[1] - 128)/256. + 1.;  
	destination[2] =  (float)(rawData[2] - 128)/256. + 1.; 
	
	MPU9250_writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); 					// Power down magnetometer  
	delay_msec(10);

	// Configure the magnetometer for continuous read and highest resolution
	// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates

	MPU9250_writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode);	// Set magnetometer data resolution and sample ODR
	delay_msec(10);
}

void initMPU9250(void) {
//	wake up device
	MPU9250_writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); 					// Clear sleep mode bit (6), enable all sensors 
	delay_msec(100); 														// Wait for all registers to reset 

//	get stable time source
	MPU9250_writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  					// Auto select clock source to be PLL gyroscope reference if ready else
	delay_msec(200); 
  
/*	Configure Gyro and Thermometer
	Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
	minimum delay_msec time for this setting is 5.9 ms, which means sensor fusion update rates cannot
	be higher than 1 / 0.0059 = 170 Hz
	DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
	With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz */
	MPU9250_writeByte(MPU9250_ADDRESS, CONFIG, 0x03);  

//	Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	MPU9250_writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  					// Use a 200 Hz rate; a rate consistent with the filter update rate 
																			// determined inset in CONFIG above
/*	Set gyroscope full scale range
	Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3 */
	uint8_t c = MPU9250_readByte(MPU9250_ADDRESS, GYRO_CONFIG);
//  writeRegister(GYRO_CONFIG, c & ~0xE0); 									// Clear self-test bits [7:5] 
	MPU9250_writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0x02);				// Clear Fchoice bits [1:0] 
	MPU9250_writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0x18);				// Clear AFS bits [4:3]
	MPU9250_writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c | Gscale << 3); 		// Set full scale range for the gyro
//	writeRegister(GYRO_CONFIG, c | 0x00); 									// Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  
//	Set accelerometer full-scale range configuration
	c = MPU9250_readByte(MPU9250_ADDRESS, ACCEL_CONFIG);
//  writeRegister(ACCEL_CONFIG, c & ~0xE0);									// Clear self-test bits [7:5] 
	MPU9250_writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c & ~0x18); 			// Clear AFS bits [4:3]
	MPU9250_writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c | Ascale << 3);		// Set full scale range for the accelerometer 

/*	Set accelerometer sample rate configuration
	It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz */
	c = MPU9250_readByte(MPU9250_ADDRESS, ACCEL_CONFIG2);
	MPU9250_writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c & ~0x0F);			// Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
	MPU9250_writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c | 0x03);			// Set accelerometer rate to 1 kHz and bandwidth to 41 Hz

//	The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
//	but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

/*	Configure Interrupts and Bypass Enable
	Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
	clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
	can join the I2C bus and all can be controlled by the Arduino as master */
	MPU9250_writeByte(MPU9250_ADDRESS, INT_piN_CFG, 0x22);    
	MPU9250_writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);					// Enable data ready (bit 0) interrupt
	delay_msec(100);
}

/*	Function which accumulates gyro and accelerometer data after device initialization. 
	It calculates the average of the at-rest readings and then loads the resulting offsets 
	into accelerometer and gyro bias registers. */
void accelgyrocalMPU9250(float * dest1, float * dest2) {  
	uint8_t data[12];												// data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  
//	reset device
	MPU9250_writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80);					// Write a one to bit 7 reset bit; toggle reset device
	delay_msec(100);
   
/*	Get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
	else use the internal oscillator, bits 2:0 = 001*/
	MPU9250_writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  
	MPU9250_writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
	delay_msec(200);                                    

//	Configure device for bias calculation
	MPU9250_writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);					// Disable all interrupts
	MPU9250_writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);						// Disable FIFO
	MPU9250_writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);					// Turn on internal clock source
	MPU9250_writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00);					// Disable I2C master
	MPU9250_writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);					// Disable FIFO and I2C master modes
	MPU9250_writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);					// Reset FIFO and DMP
	delay_msec(15);
  
//	Configure MPU6050 gyro and accelerometer for bias calculation
	MPU9250_writeByte(MPU9250_ADDRESS, CONFIG, 0x01);						// Set low-pass filter to 188 Hz
	MPU9250_writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);					// Set sample rate to 1 kHz
	MPU9250_writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);					// Set gyro full-scale to 250 degrees per second, maximum sensitivity
	MPU9250_writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);					// Set accelerometer full-scale to 2 g, maximum sensitivity
 
	uint16_t  gyrosensitivity  = 131;   									// = 131 LSB/degrees/sec
	uint16_t  accelsensitivity = 16384;  									// = 16384 LSB/g

//	Configure FIFO to capture accelerometer and gyro data for bias calculation
	MPU9250_writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   					// Enable FIFO  
	MPU9250_writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     					// Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	delay_msec(40); 														// accumulate 40 samples in 40 milliseconds = 480 bytes

//	At end of sample accumulation, turn off FIFO sensor read
	MPU9250_writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);        				// Disable gyro and accelerometer sensors for FIFO
	MPU9250_readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); 			// read FIFO sample count
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	packet_count = fifo_count/12;											// How many sets of full gyro and accelerometer data for averaging

	for (ii = 0; ii < packet_count; ii++) {
		int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
		MPU9250_readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]);			// read data for averaging
		accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  );	// Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  );
		accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  );
		
		gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  );
		gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  );
		gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]);

		accel_bias[0] += (int32_t) accel_temp[0]; 							// Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
		
		gyro_bias[0]  += (int32_t) gyro_temp[0];
		gyro_bias[1]  += (int32_t) gyro_temp[1];
		gyro_bias[2]  += (int32_t) gyro_temp[2];
	}

	accel_bias[0] /= (int32_t) packet_count; 								// Normalize sums to get average count biases
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	
	gyro_bias[0]  /= (int32_t) packet_count;
	gyro_bias[1]  /= (int32_t) packet_count;
	gyro_bias[2]  /= (int32_t) packet_count;
    
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  	// Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}
   
/*	Construct the gyro biases for push to the hardware gyro bias registers, 
	which are reset to zero upon device startup */
	data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; 						// Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0]/4)       & 0xFF; 						// Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  
	data[3] = (-gyro_bias[1]/4)       & 0xFF;
	data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	data[5] = (-gyro_bias[2]/4)       & 0xFF;
  
//	Push gyro biases to hardware registers
	MPU9250_writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
	MPU9250_writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
  
	MPU9250_writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
	MPU9250_writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
  
	MPU9250_writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
	MPU9250_writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
  
//	Output scaled gyro biases for display in the main program
	dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;  
	dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
	dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

/*	Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	the accelerometer biases calculated above must be divided by 8.  */

	int32_t accel_bias_reg[3] = {0, 0, 0};								// A place to hold the factory accelerometer trim biases
	MPU9250_readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]);		// Read factory accelerometer trim values
	accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
	
	MPU9250_readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
	
	MPU9250_readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  
	uint32_t mask = 1uL;										// Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = {0, 0, 0};							// Define array to hold mask bit for each accelerometer bias axis

	for(ii = 0; ii < 3; ii++) {
		if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01;	// If temperature compensation bit is set, record that fact in mask_bit
	}

/*	Construct total accelerometer bias, 
	including calculated average accelerometer bias from above */
	accel_bias_reg[0] -= (accel_bias[0]/8);						// Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1]/8);
	accel_bias_reg[2] -= (accel_bias[2]/8);

	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0])      & 0xFF;
	data[1] = data[1] | mask_bit[0]; 							// preserve temperature compensation bit when writing back to accelerometer bias registers

	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;

	data[3] = (accel_bias_reg[1])      & 0xFF;
	data[3] = data[3] | mask_bit[1]; 							// preserve temperature compensation bit when writing back to accelerometer bias registers

	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;

	data[5] = (accel_bias_reg[2])      & 0xFF;
	data[5] = data[5] | mask_bit[2]; 							// preserve temperature compensation bit when writing back to accelerometer bias registers
 
/*	WARNING Apparently this is not working for the acceleration biases in the MPU-9250
	Are we handling the temperature correction bit properly?
	
	Push accelerometer biases to hardware registers */

	MPU9250_writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
	MPU9250_writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
	MPU9250_writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
	MPU9250_writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
	MPU9250_writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
	MPU9250_writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);

//	Output scaled accelerometer biases for display in the main program
	dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
	dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
	dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

void magcalMPU9250(float * dest1, float * dest2, uint16_t sample_count) {
	uint16_t ii = 0; //, sample_count = 0;
	int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
	int16_t mag_max[3]  = {0xFF, 0xFF, 0xFF}, mag_min[3] = {0x7F, 0x7F, 0x7F}, mag_temp[3] = {0, 0, 0};

	delay_msec(100);														// Was 4000

//	sample_count = 32;														// Probably need to sample in many orthagonal spatial alignments ??????? CSB
	for(ii = 0; ii < sample_count; ii++) {
		readMagData(mag_temp);  											// Read the mag data   
		for (int jj = 0; jj < 3; jj++) {
			if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
			if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
		}
		if(Mmode == 0x02) delay_msec(135);  								// at 8 Hz ODR, new mag data is available every 125 ms
		if(Mmode == 0x06) delay_msec(12);									// at 100 Hz ODR, new mag data is available every 10 ms
	}

//	Get hard iron correction
	mag_bias[0]  = (mag_max[0] + mag_min[0])/2;								// get average x mag bias in counts
	mag_bias[1]  = (mag_max[1] + mag_min[1])/2;								// get average y mag bias in counts
	mag_bias[2]  = (mag_max[2] + mag_min[2])/2;
	
//	Get average z mag bias in counts
    
    dest1[0] = (float) mag_bias[0]*mRes*magCalibration[0];					// save mag biases in G for main program
    dest1[1] = (float) mag_bias[1]*mRes*magCalibration[1];   
    dest1[2] = (float) mag_bias[2]*mRes*magCalibration[2];
       
//	Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;							// get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;							// get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;							// get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    dest2[0] = avg_rad/((float)mag_scale[0]);
    dest2[1] = avg_rad/((float)mag_scale[1]);
    dest2[2] = avg_rad/((float)mag_scale[2]);
}

void MPU9250SelfTest(float * destination) {									/*	Should return percent deviation from factory trim 
																				values, +/- 14 or less deviation is a pass */
	uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
	uint8_t selfTest[6];
	uint8_t FS = 0;	
	int16_t gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];
	float factoryTrim[6];

	MPU9250_writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);					// Set gyro sample rate to 1 kHz
	MPU9250_writeByte(MPU9250_ADDRESS, CONFIG, 0x02);						// Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	MPU9250_writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 1<<FS);					// Set full scale range for the gyro to 250 dps
	MPU9250_writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02);				// Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	MPU9250_writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 1<<FS);				// Set full scale range for the accelerometer to 2 g

	for( int ii = 0; ii < 50; ii++) {										// get average current values of gyro and acclerometer
		MPU9250_readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);	// Read the six raw data registers into data array
		aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);		// Turn the MSB and LSB into a signed 16-bit value
		aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);  
		aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]); 

		MPU9250_readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);	// Read the six raw data registers sequentially into data array
		gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);		// Turn the MSB and LSB into a signed 16-bit value
		gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);  
		gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]); 
	}
  
	for (int ii =0; ii < 3; ii++) {  										// Get average of 50 values and store as average current readings
	  aAvg[ii] /= 50;
	  gAvg[ii] /= 50;
	}
  
//	Configure the accelerometer for self-test
	MPU9250_writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0);					// Enable self test on all three axes and set accelerometer range to +/- 2 g
	MPU9250_writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0xE0);					// Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	delay_msec(25); 														// delay_msec a while to let the device stabilize

	for( int ii = 0; ii < 50; ii++) {										// get average self-test values of gyro and acclerometer  
		MPU9250_readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);	// Read the six raw data registers into data array
		aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);	// Turn the MSB and LSB into a signed 16-bit value
		aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);  
		aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]); 

		MPU9250_readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);	// Read the six raw data registers sequentially into data array
		gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);	// Turn the MSB and LSB into a signed 16-bit value
		gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);  
		gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]); 
	}
  
	for (int ii =0; ii < 3; ii++) {  										// Get average of 50 values and store as average self-test readings
		aSTAvg[ii] /= 50;
		gSTAvg[ii] /= 50;
	}   
  
//	Configure the gyro and accelerometer for normal operation
	MPU9250_writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);  
	MPU9250_writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0x00);  
	delay_msec(25);															// delay_msec a while to let the device stabilize
   
//	Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	selfTest[0] = MPU9250_readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL);		// X-axis accel self-test results
	selfTest[1] = MPU9250_readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL);		// Y-axis accel self-test results
	selfTest[2] = MPU9250_readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL);		// Z-axis accel self-test results
	selfTest[3] = MPU9250_readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO);		// X-axis gyro self-test results
	selfTest[4] = MPU9250_readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);		// Y-axis gyro self-test results
	selfTest[5] = MPU9250_readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);		// Z-axis gyro self-test results

//	Retrieve factory self-test value from self-test code reads
	factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
	factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
	factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
	factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
	factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
	factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

/*	Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
	To get percent, must multiply by 100 */
   for (int i = 0; i < 3; i++) {
		destination[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i];   	// Report percent differences
		destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3]; 	// Report percent differences
   } 
}

void readAccelFloatMG(float *xyz) { 												// in milli g
	int16_t Acc[3];
	readAccelData(Acc);
	getAres();
	for(int i=0;i<3;i++) 
		xyz[i] = aRes*Acc[i];
}

void readGyroFloatDeg(float *xyz) {					  								// in degree
	
	int16_t Gyro[3];
	readGyroData(Gyro);
	getGres();
	for(int i=0;i<3;i++) 
		xyz[i] = gRes*Gyro[i];
}

void readMagFloatUT(float *xyz) {					 								// in micro tesla
	int16_t Mag[3];
	
	readMagData(Mag);
	if(Mag[0]+Mag[1]+Mag[2]!=0) {
		for(int i=0;i<3;i++) {
			xyz[i] = 1229.0*Mag[i]/4095.0 * magCalibration[i];
		}
	}
	else {
		xyz[0]=0;
		xyz[1]=0;
		xyz[2]=0;
	}

}

void readMagTest(float *xyz){														// Read Calibrated Magnetometer (mGauss)
    int16_t Mag[3];

	readMagData(Mag);
   
/*	Calculate the magnetometer values in milliGauss
	Include factory calibration per data sheet and user environmental corrections */
	if(Mag[0]+Mag[1]+Mag[2]!=0) {
		mx = (float)Mag[0];
		mx = mx*mRes*magCalibration[0] - magBias[0];  						// Get actual magnetometer value, this depends on scale being set
		my = (float)Mag[1];
		my = my*mRes*magCalibration[1] - magBias[1];  
		mz = (float)Mag[2];
		mz = mz*mRes*magCalibration[2] - magBias[2];  

		mx *= magScale[0];
		my *= magScale[1];
		mz *= magScale[2]; 

		xyz[0]=mx;
		xyz[1]=my;
		xyz[2]=mz;
	}
	else {
		xyz[0]=0;
		xyz[1]=0;
		xyz[2]=0;
	 }
		 
}


void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
/*	Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
	(see http://www.x-io.co.uk/category/open-source/ for examples and more details)
	which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
	device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
	The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
	but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz! */
	
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   						// Short name local variable for readability
	float norm;
	float hx, hy, _2bx, _2bz;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;


	float _2q1mx;																// Auxiliary variables to avoid repeated arithmetic
	float _2q1my;
	float _2q1mz;
	float _2q2mx;
	float _4bx;
	float _4bz;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;
	float _2q1q3 = 2.0f * q1 * q3;
	float _2q3q4 = 2.0f * q3 * q4;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

//	Normalise accelerometer measurement
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; 														// handle NaN
	norm = 1.0f/norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

//	Normalise magnetometer measurement
	norm = sqrt(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return; 														// handle NaN
	norm = 1.0f/norm;
	mx *= norm;
	my *= norm;
	mz *= norm;

//	Reference direction of Earth's magnetic field
	_2q1mx = 2.0f * q1 * mx;
	_2q1my = 2.0f * q1 * my;
	_2q1mz = 2.0f * q1 * mz;
	_2q2mx = 2.0f * q2 * mx;
	
	hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
	hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
	_2bx = sqrt(hx * hx + hy * hy);
	_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;

//	Gradient decent algorithm corrective step
	s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s2 =  _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s4 =  _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	
	norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    							// Normalise step magnitude
	norm = 1.0f/norm;
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

//	Compute rate of change of quaternion
	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
	qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
	qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
	qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

//	Integrate to yield quaternion
	q1 += qDot1 * deltat;
	q2 += qDot2 * deltat;
	q3 += qDot3 * deltat;
	q4 += qDot4 * deltat;
	
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    							// Normalise quaternion
	norm = 1.0f/norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;
}
  
void readQuaternion(float *quaternion) {											// Read Quaternion
	quaternion[0] = q[0];
	quaternion[1] = q[1];
	quaternion[2] = q[2];
	quaternion[3] = q[3];
}  
  
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
	
/*	Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors 
	and measured vectors.*/
	
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   							// Short name local variable for readability
	float norm;
	float hx, hy, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;
	float pa, pb, pc;


	float q1q1 = q1 * q1;															// Auxiliary variables to avoid repeated arithmetic
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;   

//	Normalise accelerometer measurement
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return;														// handle NaN
	norm = 1.0f / norm;																// use reciprocal for division
	ax *= norm;
	ay *= norm;
	az *= norm;

//	Normalise magnetometer measurement
	norm = sqrt(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return;														// handle NaN
	norm = 1.0f / norm;																// use reciprocal for division
	mx *= norm;
	my *= norm;
	mz *= norm;

//	Reference direction of Earth's magnetic field
	hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
	hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
	bx = sqrt((hx * hx) + (hy * hy));
	bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

//	Estimated direction of gravity and magnetic field
	vx = 2.0f * (q2q4 - q1q3);
	vy = 2.0f * (q1q2 + q3q4);
	vz = q1q1 - q2q2 - q3q3 + q4q4;
	wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
	wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
	wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);  

//	Error is cross product between estimated direction and measured direction of gravity
	ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
	
	if (Ki > 0.0f) {
		eInt[0] += ex;															// Accumulate integral error
		eInt[1] += ey;
		eInt[2] += ez;
	} 
	else {
		eInt[0] = 0.0f;															// Prevent integral wind up
		eInt[1] = 0.0f;
		eInt[2] = 0.0f;
	}


	gx = gx + Kp * ex + Ki * eInt[0];											// Apply feedback terms
	gy = gy + Kp * ey + Ki * eInt[1];
	gz = gz + Kp * ez + Ki * eInt[2];


	pa = q2;																	// Integrate rate of change of quaternion
	pb = q3;
	pc = q4;
	q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
	q2 = pa + ( q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
	q3 = pb + ( q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
	q4 = pc + ( q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);


	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);							// Normalise quaternion
	norm = 1.0f / norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;
}

