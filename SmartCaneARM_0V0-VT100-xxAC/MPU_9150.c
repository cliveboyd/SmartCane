/* Copyright (c) <2015> <Shun Bai (wanyancan at gmail)>

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
 
#include "nrf_delay.h"

#include "MPU_9150.h"
#include "Communication.h"
#include "math.h"


#define MAX_TIMEOUT_LOOPS (500UL) 								/**< MAX while loops to wait for MPU event */

Ascale_t Ascale = AFS_2G;     									// AFS_2G, AFS_4G, AFS_8G, AFS_16G
Gscale_t Gscale = GFS_1000DPS; 									// GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
Mscale_t Mscale = MFS_16BITS;

float aRes, gRes, mRes;      									// Scale resolutions per LSB for the sensors
 
int16_t accelCount[3];  										// Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   										// Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    										// Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = {0, 0, 0};							// Factory mag calibration 
float magBias[3] = {0, 0, 0};  									// Factory mag bias
float magScale[3] = {0, 0, 0};
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}; 		// Bias corrections for gyro and accelerometer
float ax, ay, az, gx, gy, gz, mx, my, mz; 						// Variables to hold latest sensor data values 
int16_t tempCount;   											// Stores the real internal chip temperature in degrees Celsius
float temperature;
float SelfTest[6];
 
int delt_t = 0; 												// Used to control display output rate
int count = 0;  												// Used to control display output rate
 

//static const float PI = 3.14159265358979323846f;
//static const float GyroMeasError = PI * (60.0f / 180.0f);     // Gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3

static const float beta = 0.906899682117108925297f;  			// Compute beta

//static const float GyroMeasDrift = PI * (1.0f / 180.0f);      // Gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
//static const float zeta = 0.0151149947019518154f;  			// Compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

static const float Kp = 2.0f * 5.0f; 							// Free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
static const float Ki = 0.0f ;
 
float pitch, yaw, roll;
float deltat = 0.100f;											// Hard set Quaternion filter sample rate called via SYSTEM_TIMER_INTERVAL=100ms via main
// float deltat = 0.0f;                             			// Integration interval for both filter schemes
//int lastUpdate = 0, firstUpdate = 0, Now = 0;    				// Used to calculate integration interval

float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};           				// Vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};              				// Vector to hold integral error for Mahony method

void MPU9150_writeByte_(uint8_t address, uint8_t subAddress, uint8_t data, bool loop) {
   unsigned char data_write[2];
   data_write[0] = subAddress;
   data_write[1] = data;
	
	for(int i=0;i<5;i++)
	{
		if(I2C_Write(address,
              data_write,
              2,
              0))  // no stop bit
			break;
		if (!loop) break;
	}
}
void MPU9150_writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
	MPU9150_writeByte_(address, subAddress, data, true);
}
void MPU9150_readBytes_(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest, bool loop) {     
	int i=0;
	for(;i<5;i++)
	{
		if(I2C_Write(address,
				  (unsigned char*)&subAddress,
				  1,
				  0))
				  break; 														// if written length > 0
			if (!loop) break;
	}
	if (i>=5) 
		return;
	for(i=0;i<5;i++) {
		if(I2C_Read(address,
             dest,
             count,
             1))
			break;  // if read length > 0
		if (!loop) break;
	}
}

void MPU9150_readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) {     
	MPU9150_readBytes_(address, subAddress, count, dest, true);
} 
unsigned char MPU9150_readByte_(uint8_t address, uint8_t subAddress, bool loop) {
    unsigned char data[1]; // `data` will store the register data     
    MPU9150_readBytes_(address, subAddress, 1, data,loop);
    return data[0]; 
}
unsigned char MPU9150_readByte(uint8_t address, uint8_t subAddress) {
    unsigned char data[1]; // `data` will store the register data     
    MPU9150_readBytes(address, subAddress, 1, data);
    return data[0]; 
}

void getMres() {															// Possible magnetometer scales register bit settings are: 14 bit (0) and 16 bit (1)
  switch (Mscale) {
    case MFS_14BITS:
          mRes = 10.*4912./8190.;											// Proper scale to return milliGauss
          break;
    case MFS_16BITS:
          mRes = 10.*4912./32760.0; 										// Proper scale to return milliGauss
          break;
	}
}
void getGres() {															// Possible gyro scales (and their register bit settings) are:
																			// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
																			// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:  
	switch (Gscale) {
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
void getAres() {															// Possible accelerometer scales (and their register bit settings) are:
																			// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
																			// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:	
  switch (Ascale) {												
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
void wait(float Sec) {																// Delay in Seconds
	nrf_delay_us(1000000.0*Sec);
}

void readQuaternion(float *quaternion) {											// Read Quaternion
	float Acc[3];
	float Gyro[3];
	float Mag[3];
	readAccelFloatMG(Acc);
	readGyroFloatDeg(Gyro);
	readMagFloatUT(Mag);
	
	MadgwickQuaternionUpdate(Acc[0],Acc[1],Acc[2],Gyro[0],Gyro[1],Gyro[2],Mag[0],Mag[1],Mag[2]);
	
	quaternion[0] = q[0];
	quaternion[1] = q[1];
	quaternion[2] = q[2];
	quaternion[3] = q[3];
}
void readAccelFloatMG(float *xyz) { 												// in mili G
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
	
	for(int i=0;i<3;i++) {
		xyz[i] = 1229.0*Mag[i]/4095.0 * magCalibration[i];
	}
}
void readAccelData(int16_t * destination) {											// Read Accelerometer
	uint8_t rawData[6];  															// xyz accel register data stored here
	MPU9150_readBytes(MPU9150_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  				// Read the six raw data registers into data array
	destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;			// Turn the MSB and LSB into a signed 16-bit value
	destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
	destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
}
 
void readGyroData(int16_t * destination) {											// Read Gyro
	uint8_t rawData[6];  															// xyz gyro register data stored here
	MPU9150_readBytes(MPU9150_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  				// Read the six raw data registers sequentially into data array
	destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;			// Turn the MSB and LSB into a signed 16-bit value
	destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
	destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
}
 
int readMagData(int16_t * destination) {											// Initalise Magnetometer
	uint8_t rawData[6];  															// xyz magnetometer register data stored here
	uint8_t c;
	
	MPU9150_writeByte(AK8975A_ADDRESS, AK8975A_CNTL, AK8975A_CNTL_POWERDOWN_MODE);	// Toggle enable data read from magnetometer, no continuous read mode!
	
	wait(0.01);
	
	MPU9150_readBytes(AK8975A_ADDRESS, AK8975A_XOUT_L, 6, &rawData[0]);				// Read the six raw data registers sequentially into data array

	c = MPU9150_readByte(AK8975A_ADDRESS, AK8975A_ST2);	
	c = MPU9150_readByte(AK8975A_ADDRESS, AK8975A_ST1);	
//	MPU9150_readBytes(MPU9150_ADDRESS, EXT_SENS_DATA_01, 6, &rawData[0]);			// Read the six raw data registers sequentially into data array

	MPU9150_writeByte(AK8975A_ADDRESS, 0x0c, 0x00);  								// self test
	MPU9150_writeByte(AK8975A_ADDRESS, AK8975A_CNTL, AK8975A_CNTL_SINGLE_MODE); 	// toggle enable data read from magnetometer, no continuous read mode!

	MPU9150_writeByte(AK8975A_ADDRESS, AK8975A_CNTL, AK8975A_CNTL_SINGLE_MODE); 	// toggle enable data read from magnetometer, no continuous read mode!
	wait(0.01);
	
/*	Only accept a new magnetometer data read if the data ready bit is set and 
	if there are no sensor overflow or data read errors */
	uint32_t timeout = MAX_TIMEOUT_LOOPS; 															/* max loops to wait for RXDREADY event*/
	while(!(MPU9150_readByte(AK8975A_ADDRESS, AK8975A_ST1) & 0x01) && (--timeout)) {				// wait for magnetometer data ready bit to be set
		nrf_delay_us(100);
	}
	if(timeout==0) return 1;
	
	MPU9150_readBytes(AK8975A_ADDRESS, AK8975A_XOUT_L, 6, &rawData[0]);  			// Read the six raw data registers sequentially into data array
	destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  					// Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
	destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
	
	MPU9150_readBytes(AK8975A_ADDRESS, AK8975A_XOUT_L, 6, &rawData[0]);				// Read the six raw data registers sequentially into data array
	destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;						// Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
	destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;

	c = MPU9150_readByte(AK8975A_ADDRESS, 0x00);	
	c = MPU9150_readByte(AK8975A_ADDRESS, AK8975A_ST2);	
	
	if(c&0x0C) {																	// overflow 
		MPU9150_writeByte(AK8975A_ADDRESS, AK8975A_CNTL, AK8975A_CNTL_SINGLE_MODE); // toggle enable data read from magnetometer, no continuous read mode!
		
		timeout = MAX_TIMEOUT_LOOPS; 
		while(!(MPU9150_readByte(AK8975A_ADDRESS, AK8975A_ST1) & 0x01) && (--timeout)) {
			nrf_delay_us(1000);}
		
		MPU9150_readBytes(AK8975A_ADDRESS, AK8975A_XOUT_L, 6, &rawData[0]);  		// Read the six raw data registers sequentially into data array
		destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  				// Turn the MSB and LSB into a signed 16-bit value
		destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
		destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
		c = MPU9150_readByte(AK8975A_ADDRESS, AK8975A_ST2);
	}
	if(timeout==0) return 1;
	c = MPU9150_readByte(AK8975A_ADDRESS, 0x0c);	
	c +=1;
	return 0;
}
 
void initAK8975A(float * destination) {												// Magnetometer INIT
	uint8_t rawData[3];  															// xyz magnetometer register data stored here
	MPU9150_writeByte(AK8975A_ADDRESS, AK8975A_CNTL, AK8975A_CNTL_POWERDOWN_MODE);	// Power down
	wait(0.01);
	
//  MPU9150_writeByte(AK8975A_ADDRESS, AK8975A_CNTL, AK8975A_CNTL_FUSEROM_MODE);	// Enter Fuse ROM access mode
 
	wait(0.01);
	MPU9150_readBytes(AK8975A_ADDRESS, AK8975A_ASAX, 3, &rawData[0]);				// Read the xyz-axis calibration values
	destination[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;						// Return x-axis sensitivity adjustment values
	destination[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;  
	destination[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f; 
}

void initAK8963(float * destination) {												// Alternative Magnetometer INIT Under Investigation CSB
	uint8_t rawData[3];  															// First extract the factory calibration for each magnetometer axis														// x/y/z gyro calibration data stored here
	
	MPU9150_writeByte(AK8975A_ADDRESS, AK8975A_CNTL, 0x00); 						// Power down magnetometer  
	wait(0.01);
	
	MPU9150_writeByte(AK8975A_ADDRESS, AK8975A_CNTL, 0x0F); 						// Enter Fuse ROM access mode
	wait(0.01);
	
	MPU9150_readBytes(AK8975A_ADDRESS, AK8975A_CNTL, 3, &rawData[0]);				// Read the xyz-axis calibration values
	destination[0] =  (float)(rawData[0] - 128)/256. + 1.;							// Return x-axis sensitivity adjustment values, etc.
	destination[1] =  (float)(rawData[1] - 128)/256. + 1.;  
	destination[2] =  (float)(rawData[2] - 128)/256. + 1.; 
	
	MPU9150_writeByte(AK8975A_ADDRESS, AK8975A_CNTL, 0x00);							// Power down magnetometer  
	wait(0.01);

/*	Configure the magnetometer for continuous read and highest resolution
	Set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	and enable continuous mode data acquisition.
	Mmode (bits [3:0]), 0010 for 8Hz and 0110 for 100Hz sample rate. */

//MPU9150_writeByte(AK8975A_ADDRESS, AK8975A_CNTL, Mscale << 4 | Mmode);			// Set magnetometer data resolution and sample ODR
	MPU9150_writeByte(AK8975A_ADDRESS, AK8975A_CNTL, Mscale << 4 | 0x02);			// Set magnetometer data resolution and sample ODR
	wait(0.01);
}

int16_t MPU9150_get_temperature() {
	return readTempData();
}

int16_t readTempData() {															// Read temperature Data from Inertial Sensor
  uint8_t rawData[2];  																// Temperature data stored here
  MPU9150_readBytes(MPU9150_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  					// Read the two raw data registers sequentially into data array 
  return (((int16_t)rawData[0] << 8) | rawData[1])/100 ;  							// Turn the MSB and LSB into a 16-bit value
}
 
void resetMPU9150() {																// Reset device

	MPU9150_writeByte(MPU9150_ADDRESS, PWR_MGMT_1, 0x80); 							// Write a one to bit 7 reset bit; toggle reset device
	wait(0.2);
  }
    
int initMPU9150() {																// Inertial Sensor
//	unsigned char status = 0;
    uint8_t c, temp;
    uint8_t chkRegs[] = {SMPLRT_DIV, CONFIG, GYRO_CONFIG, ACCEL_CONFIG, FIFO_EN, I2C_MST_CTRL, I2C_SLV0_ADDR, 
						I2C_SLV0_REG, I2C_SLV0_CTRL, I2C_SLV4_REG, I2C_SLV4_DO, I2C_SLV4_CTRL, I2C_SLV4_DI, 
						I2C_MST_STATUS, INT_PIN_CFG, INT_ENABLE, INT_STATUS, I2C_MST_DELAY_CTRL, SIGNAL_PATH_RESET, USER_CTRL,
						PWR_MGMT_1, PWR_MGMT_2, 0x75};
	
	unsigned char status;
	while(!status) status = I2C_Init();
	resetMPU9150();
	
//	Initialize MPU9150 device and wake up device
	MPU9150_writeByte(MPU9150_ADDRESS, PWR_MGMT_1, 0x00); 							// Clear sleep mode bit (6), enable all sensors 
	wait(0.1);																		// Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt  

	
//	Get stable time source
	MPU9150_writeByte(MPU9150_ADDRESS, PWR_MGMT_1, 0x01);  							// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

	for(int i=0; i < sizeof(chkRegs)/sizeof(uint8_t); i++) {
		c = MPU9150_readByte(MPU9150_ADDRESS, chkRegs[i]);
		c +=1;
	}


	MPU9150_writeByte(MPU9150_ADDRESS, INT_PIN_CFG, 0x02);
	c = MPU9150_readByte(MPU9150_ADDRESS, USER_CTRL); 
	if (c & 0x20) MPU9150_writeByte(MPU9150_ADDRESS, USER_CTRL, c ^ 0x20);
	wait(2);
	
	for(int i=0; i < 0x13; i++) {
		temp = i;
		c = MPU9150_readByte(AK8975A_ADDRESS, i);
		temp += 1;
	}
	
	MPU9150_writeByte(MPU9150_ADDRESS, FIFO_EN, 0xF8); 							// 0x23

	c = MPU9150_readByte(AK8975A_ADDRESS, 0x00);								// Check magnetometer id
	if (c == 0x48) initAK8975A(magCalibration); 								// Load the sensitivity from rom

/*	Connfigure Gyro and Accelerometer
	Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively; 
	DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
	Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate. */
	MPU9150_writeByte(MPU9150_ADDRESS, CONFIG, 0x03);  
	c = MPU9150_readByte(MPU9150_ADDRESS, CONFIG);

 
//	Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	MPU9150_writeByte(MPU9150_ADDRESS, SMPLRT_DIV, 0x04);						// Use a 200 Hz rate; the same rate set in CONFIG above
	c = MPU9150_readByte(MPU9150_ADDRESS, SMPLRT_DIV);

/*	Set gyroscope full scale range
	Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3 */
	c =  MPU9150_readByte(MPU9150_ADDRESS, GYRO_CONFIG);
	while(c!=(Gscale<<3)) {
		MPU9150_writeByte(MPU9150_ADDRESS, GYRO_CONFIG, (Gscale<<3));			// Use a 200 Hz rate; the same rate set in CONFIG above
		c =  MPU9150_readByte(MPU9150_ADDRESS, GYRO_CONFIG);	
	}
//	Set accelerometer configuration
	c =  MPU9150_readByte(MPU9150_ADDRESS, ACCEL_CONFIG);
	while(c!=(Ascale<<3)) {
		MPU9150_writeByte(MPU9150_ADDRESS, ACCEL_CONFIG, (Ascale<<3));  		// Use a 200 Hz rate; the same rate set in CONFIG above
		c =  MPU9150_readByte(MPU9150_ADDRESS, ACCEL_CONFIG);
	}
  
	 
//	Configure Magnetometer for FIFO
//	Initialize AK8975A for write
//	MPU9150_writeByte(MPU9150_ADDRESS, INT_PIN_CFG, 0x00);						// 0x37
//	MPU9150_writeByte(MPU9150_ADDRESS, USER_CTRL, 0x00);						// 0x6a
//	
//	MPU9150_writeByte(MPU9150_ADDRESS, I2C_MST_CTRL, 0x40);						// WAIT_FOR_ES = 1
//	MPU9150_writeByte(MPU9150_ADDRESS, I2C_SLV0_ADDR, 0x8C);					// Enable and read address (0x0C) of the AK8975A // 0x25, read = 1
//	MPU9150_writeByte(MPU9150_ADDRESS, I2C_SLV0_REG, 0x02);						// Register within AK8975A from which to start data read, // 0x26, 0x3 - 0x8 are mag measurement data
//	MPU9150_writeByte(MPU9150_ADDRESS, I2C_SLV0_CTRL, 0x88);					// Read six bytes and swap bytes // 0x27, num_read = 6 bytes

//	
//	MPU9150_writeByte(MPU9150_ADDRESS, I2C_SLV1_ADDR, 0x0C);					// Write address of AK8975A  // 0x28 
//	MPU9150_writeByte(MPU9150_ADDRESS, I2C_SLV1_REG, 0x0A);						// Register from within the AK8975 to which to write  // 0x29
//	MPU9150_writeByte(MPU9150_ADDRESS, I2C_SLV1_CTRL, 0x81);					// Enable Slave 1 // 0x2A  slave1_en = 1, slv1_len = 1
//	MPU9150_writeByte(MPU9150_ADDRESS, I2C_SLV1_DO, 0x01);						// Register that holds output data written into Slave 1 when in write mode //0x64

//	MPU9150_writeByte(MPU9150_ADDRESS, I2C_MST_DELAY_CTRL, 0x03);				// 0x67 // Enable delay of external sensor data until all data registers have been read

	MPU9150_writeByte(MPU9150_ADDRESS, 0x01, 0x80); 							// unknown 
	
//	MPU9150_writeByte(MPU9150_ADDRESS, 0x34, 0x04);								// unknown
//	MPU9150_writeByte(MPU9150_ADDRESS, 0x64, 0x00);								// unknown 
//	MPU9150_writeByte(MPU9150_ADDRESS, 0x6A, 0x00);								// unknown
//	MPU9150_writeByte(MPU9150_ADDRESS, 0x64, 0x01); 							// unknown 
//	MPU9150_writeByte(MPU9150_ADDRESS, 0x6A, 0x20); 							// unknown 
//	MPU9150_writeByte(MPU9150_ADDRESS, 0x34, 0x13); 							// unknown 
	
//	Set up auxilliary communication with AK8975A for FIFO read
   
//	Configure FIFO
//	MPU9150_writeByte(MPU9150_ADDRESS, INT_ENABLE, 0x00); 						// Disable all interrupts // 0x38
//	MPU9150_writeByte(MPU9150_ADDRESS, FIFO_EN, 0x00);    						// Disable FIFO   // 0x23
//	MPU9150_writeByte(MPU9150_ADDRESS, USER_CTRL, 0x01);  						// Reset I2C master and FIFO and DMP // 0x6a
//	MPU9150_writeByte(MPU9150_ADDRESS, USER_CTRL, 0x00);  						// Disable FIFO 
//	wait(0.1);


/*	Configure Interrupts and Bypass Enable
	Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips 
	can join the I2C bus and all can be controlled by the master. */

//	MPU9150_writeByte(MPU9150_ADDRESS, INT_PIN_CFG, 0x32);   					// 0x37 , INT active high, push-pull,  
//	MPU9150_writeByte(MPU9150_ADDRESS, INT_ENABLE, 0x01);  						// 0x38 // Enable data ready (bit 0) interrupt

	wait(0.1);
	for(int i=0; i < sizeof(chkRegs)/sizeof(uint8_t); i++) {
		c = MPU9150_readByte(MPU9150_ADDRESS, chkRegs[i]);
		c +=1;
	}
	for(int i=0; i < 0x13; i++) {
		temp = i;
		c = MPU9150_readByte(AK8975A_ADDRESS, i);
		temp += 1;
	}
	
	int16_t out[3];
	int ret = ReadMagData(out);
	return ret;
	
	return 0;
}

void initMPU9250() {  
//	Wake up device
	MPU9150_writeByte(MPU9150_ADDRESS, PWR_MGMT_1, 0x00); 						// Clear sleep mode bit (6), enable all sensors 
	wait(0.100); 																// Wait for all registers to reset 

//	Get stable time source
	MPU9150_writeByte(MPU9150_ADDRESS, PWR_MGMT_1, 0x01);  						// Auto select clock source to be PLL gyroscope reference if ready else
	wait(0.200); 
  
/*	Configure Gyro and Thermometer
	Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
	minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
	be higher than 1 / 0.0059 = 170 Hz
	DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
	With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz */
	MPU9150_writeByte(MPU9150_ADDRESS, CONFIG, 0x03);  

//	Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	MPU9150_writeByte(MPU9150_ADDRESS, SMPLRT_DIV, 0x04);  						// Use a 200 Hz rate; a rate consistent with the filter update rate 
																				// determined inset in CONFIG above
/*	Set gyroscope full scale range
	Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3 */
	uint8_t c = MPU9150_readByte(MPU9150_ADDRESS, GYRO_CONFIG);
//  writeRegister(GYRO_CONFIG, c & ~0xE0); 										// Clear self-test bits [7:5] 
	MPU9150_writeByte(MPU9150_ADDRESS, GYRO_CONFIG, c & ~0x02);					// Clear Fchoice bits [1:0] 
	MPU9150_writeByte(MPU9150_ADDRESS, GYRO_CONFIG, c & ~0x18);					// Clear AFS bits [4:3]
	MPU9150_writeByte(MPU9150_ADDRESS, GYRO_CONFIG, c | Gscale << 3); 			// Set full scale range for the gyro
//	writeRegister(GYRO_CONFIG, c | 0x00); 										// Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  
//	Set accelerometer full-scale range configuration
	c = MPU9150_readByte(MPU9150_ADDRESS, ACCEL_CONFIG);
//  writeRegister(ACCEL_CONFIG, c & ~0xE0);										// Clear self-test bits [7:5] 
	MPU9150_writeByte(MPU9150_ADDRESS, ACCEL_CONFIG, c & ~0x18); 				// Clear AFS bits [4:3]
	MPU9150_writeByte(MPU9150_ADDRESS, ACCEL_CONFIG, c | Ascale << 3);			// Set full scale range for the accelerometer 

/*	Set accelerometer sample rate configuration
	It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz */
	c = MPU9150_readByte(MPU9150_ADDRESS, ACCEL_CONFIG2);
	MPU9150_writeByte(MPU9150_ADDRESS, ACCEL_CONFIG2, c & ~0x0F);				// Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
	MPU9150_writeByte(MPU9150_ADDRESS, ACCEL_CONFIG2, c | 0x03);				// Set accelerometer rate to 1 kHz and bandwidth to 41 Hz

 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

/*	Configure Interrupts and Bypass Enable
	Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
	clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
	can join the I2C bus and all can be controlled by the Arduino as master */
	MPU9150_writeByte(MPU9150_ADDRESS, INT_PIN_CFG, 0x22);    
	MPU9150_writeByte(MPU9150_ADDRESS, INT_ENABLE, 0x01);						// Enable data ready (bit 0) interrupt
	wait(0.100);
}

/*	Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
	of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers. */
void accelgyrocalMPU9250(float * dest1, float * dest2) {  
	uint8_t data[12];															// data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  
//	reset device
	MPU9150_writeByte(MPU9150_ADDRESS, PWR_MGMT_1, 0x80);						// Write a one to bit 7 reset bit; toggle reset device
	wait(0.100);
   
/*	Get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
	else use the internal oscillator, bits 2:0 = 001*/
	MPU9150_writeByte(MPU9150_ADDRESS, PWR_MGMT_1, 0x01);  
	MPU9150_writeByte(MPU9150_ADDRESS, PWR_MGMT_2, 0x00);
	wait(0.200);                                    

//	Configure device for bias calculation
	MPU9150_writeByte(MPU9150_ADDRESS, INT_ENABLE, 0x00);						// Disable all interrupts
	MPU9150_writeByte(MPU9150_ADDRESS, FIFO_EN, 0x00);							// Disable FIFO
	MPU9150_writeByte(MPU9150_ADDRESS, PWR_MGMT_1, 0x00);						// Turn on internal clock source
	MPU9150_writeByte(MPU9150_ADDRESS, I2C_MST_CTRL, 0x00);						// Disable I2C master
	MPU9150_writeByte(MPU9150_ADDRESS, USER_CTRL, 0x00);						// Disable FIFO and I2C master modes
	MPU9150_writeByte(MPU9150_ADDRESS, USER_CTRL, 0x0C);						// Reset FIFO and DMP
	wait(0.015);
  
//	Configure MPU6050 gyro and accelerometer for bias calculation
	MPU9150_writeByte(MPU9150_ADDRESS, CONFIG, 0x01);							// Set low-pass filter to 188 Hz
	MPU9150_writeByte(MPU9150_ADDRESS, SMPLRT_DIV, 0x00);						// Set sample rate to 1 kHz
	MPU9150_writeByte(MPU9150_ADDRESS, GYRO_CONFIG, 0x00);						// Set gyro full-scale to 250 degrees per second, maximum sensitivity
	MPU9150_writeByte(MPU9150_ADDRESS, ACCEL_CONFIG, 0x00);						// Set accelerometer full-scale to 2 g, maximum sensitivity
 
	uint16_t  gyrosensitivity  = 131;   										// = 131 LSB/degrees/sec
	uint16_t  accelsensitivity = 16384;  										// = 16384 LSB/g

//	Configure FIFO to capture accelerometer and gyro data for bias calculation
	MPU9150_writeByte(MPU9150_ADDRESS, USER_CTRL, 0x40);   						// Enable FIFO  
	MPU9150_writeByte(MPU9150_ADDRESS, FIFO_EN, 0x78);     						// Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	wait(0.040); 																// accumulate 40 samples in 40 milliseconds = 480 bytes

//	At end of sample accumulation, turn off FIFO sensor read
	MPU9150_writeByte(MPU9150_ADDRESS, FIFO_EN, 0x00);        					// Disable gyro and accelerometer sensors for FIFO
	MPU9150_readBytes(MPU9150_ADDRESS, FIFO_COUNTH, 2, &data[0]); 				// read FIFO sample count
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	packet_count = fifo_count/12;												// How many sets of full gyro and accelerometer data for averaging

	for (ii = 0; ii < packet_count; ii++) {
		int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
		MPU9150_readBytes(MPU9150_ADDRESS, FIFO_R_W, 12, &data[0]);				// read data for averaging
		accel_temp[0] = (int16_t) (((int16_t)data[0] << 8)  | data[1] );		// Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t) (((int16_t)data[2] << 8)  | data[3] );
		accel_temp[2] = (int16_t) (((int16_t)data[4] << 8)  | data[5] ); 
		
		gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8)  | data[7] );
		gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8)  | data[9] );
		gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]);

		accel_bias[0] += (int32_t) accel_temp[0]; 								// Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
		
		gyro_bias[0]  += (int32_t) gyro_temp[0];
		gyro_bias[1]  += (int32_t) gyro_temp[1];
		gyro_bias[2]  += (int32_t) gyro_temp[2];
	}

	accel_bias[0] /= (int32_t) packet_count; 									// Normalize sums to get average count biases
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	
	gyro_bias[0]  /= (int32_t) packet_count;
	gyro_bias[1]  /= (int32_t) packet_count;
	gyro_bias[2]  /= (int32_t) packet_count;
    
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  		// Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}
   
/*	Construct the gyro biases for push to the hardware gyro bias registers, 
	which are reset to zero upon device startup */
	data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; 									// Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0]/4)       & 0xFF; 									// Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
	data[3] = (-gyro_bias[1]/4)       & 0xFF;
	data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	data[5] = (-gyro_bias[2]/4)       & 0xFF;
  
//	Push gyro biases to hardware registers
	MPU9150_writeByte(MPU9150_ADDRESS, XG_OFFSET_H, data[0]);
	MPU9150_writeByte(MPU9150_ADDRESS, XG_OFFSET_L, data[1]);
	MPU9150_writeByte(MPU9150_ADDRESS, YG_OFFSET_H, data[2]);
	MPU9150_writeByte(MPU9150_ADDRESS, YG_OFFSET_L, data[3]);
	MPU9150_writeByte(MPU9150_ADDRESS, ZG_OFFSET_H, data[4]);
	MPU9150_writeByte(MPU9150_ADDRESS, ZG_OFFSET_L, data[5]);
  
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
	MPU9150_readBytes(MPU9150_ADDRESS, XA_OFFSET_H, 2, &data[0]);		// Read factory accelerometer trim values
	accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
	
	MPU9150_readBytes(MPU9150_ADDRESS, YA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
	
	MPU9150_readBytes(MPU9150_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  
	uint32_t mask = 1uL;										// Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = {0, 0, 0};							// Define array to hold mask bit for each accelerometer bias axis

	for(ii = 0; ii < 3; ii++) {
		if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01;	// If temperature compensation bit is set, record that fact in mask_bit
	}
  
//	Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0]/8);						// Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1]/8);
	accel_bias_reg[2] -= (accel_bias[2]/8);

	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0])      & 0xFF;
	data[1] = data[1] | mask_bit[0]; 							// Preserve temperature compensation bit when writing back to accelerometer bias registers

	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;

	data[3] = (accel_bias_reg[1])      & 0xFF;
	data[3] = data[3] | mask_bit[1]; 							// Preserve temperature compensation bit when writing back to accelerometer bias registers

	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;

	data[5] = (accel_bias_reg[2])      & 0xFF;
	data[5] = data[5] | mask_bit[2]; 							// Preserve temperature compensation bit when writing back to accelerometer bias registers
 
/*	Apparently this is not working for the acceleration biases in the MPU-9250
	Are we handling the temperature correction bit properly?
	Push accelerometer biases to hardware registers */

/*  MPU9150_writeByte(MPU9150_ADDRESS, XA_OFFSET_H, data[0]);
	MPU9150_writeByte(MPU9150_ADDRESS, XA_OFFSET_L, data[1]);
	
	MPU9150_writeByte(MPU9150_ADDRESS, YA_OFFSET_H, data[2]);
	MPU9150_writeByte(MPU9150_ADDRESS, YA_OFFSET_L, data[3]);
	
	MPU9150_writeByte(MPU9150_ADDRESS, ZA_OFFSET_H, data[4]);
	MPU9150_writeByte(MPU9150_ADDRESS, ZA_OFFSET_L, data[5]);
*/

//	Output scaled accelerometer biases for display in the main program
	dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
	dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
	dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}


void magcalMPU9250(float * dest1, float * dest2) {
	uint16_t ii = 0, sample_count = 0;
	int32_t mag_bias[3]		= {0, 0, 0};
	int32_t mag_scale[3]	= {0, 0, 0};
	int16_t mag_max[3]		= {0xFF, 0xFF, 0xFF};
	int16_t mag_min[3]		= {0x7F, 0x7F, 0x7F};
	int16_t mag_temp[3]		= {0, 0, 0};

	wait(4.0);

	sample_count = 128;
	for(ii = 0; ii < sample_count; ii++) {
		readMagData(mag_temp);  											// Read the mag data   
		for (int jj = 0; jj < 3; jj++) {
			if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
			if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
		}
		 wait(0.135); 
//		if(Mmode == 0x02) wait(0.135);  									// at 8 Hz ODR, new mag data is available every 125 ms
//		if(Mmode == 0x06) wait(0.012);										// at 100 Hz ODR, new mag data is available every 10 ms
	}

//	Get hard iron correction
	mag_bias[0]  = (mag_max[0] + mag_min[0])/2;								// Get average x mag bias in counts
	mag_bias[1]  = (mag_max[1] + mag_min[1])/2;								// Get average y mag bias in counts
	mag_bias[2]  = (mag_max[2] + mag_min[2])/2; 							// Get average z mag bias in counts
    
    dest1[0] = (float) mag_bias[0]*mRes*magCalibration[0];					// Save mag biases in G for main program
    dest1[1] = (float) mag_bias[1]*mRes*magCalibration[1];   
    dest1[2] = (float) mag_bias[2]*mRes*magCalibration[2];  
       
//	Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;							// Get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;							// Get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;							// Get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    dest2[0] = avg_rad/((float)mag_scale[0]);
    dest2[1] = avg_rad/((float)mag_scale[1]);
    dest2[2] = avg_rad/((float)mag_scale[2]);
}

void calibrateMPU9150(float * dest1, float * dest2) {						// Calibrate Gyro and Accelerometers while at rest 
	
/*	Function which accumulates gyro and accelerometer data after device initialization. 
	It calculates the average of the at-rest readings and then loads the resulting offsets
	into accelerometer and gyro bias registers. */
	
	uint8_t data[12];														// data array to hold accelerometer and gyro x, y, z, data	
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  
//	reset device, reset all registers, clear gyro and accelerometer bias registers
	MPU9150_writeByte(MPU9150_ADDRESS, PWR_MGMT_1, 0x80);  					// Write a one to bit 7 reset bit; toggle reset device
	wait(0.1);  
   
//	get stable time source
//	Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	MPU9150_writeByte(MPU9150_ADDRESS, PWR_MGMT_1, 0x01);  
	MPU9150_writeByte(MPU9150_ADDRESS, PWR_MGMT_2, 0x00); 
	wait(0.2);
  
//	Configure device for bias calculation
	MPU9150_writeByte(MPU9150_ADDRESS, INT_ENABLE,   0x00);   				// Disable all interrupts
	MPU9150_writeByte(MPU9150_ADDRESS, FIFO_EN,      0x00);    				// Disable FIFO
	MPU9150_writeByte(MPU9150_ADDRESS, PWR_MGMT_1,   0x00);    				// Turn on internal clock source
	MPU9150_writeByte(MPU9150_ADDRESS, I2C_MST_CTRL, 0x00);  				// Disable I2C master
	MPU9150_writeByte(MPU9150_ADDRESS, USER_CTRL,    0x00);     			// Disable FIFO and I2C master modes
	MPU9150_writeByte(MPU9150_ADDRESS, USER_CTRL,    0x0C);     			// Reset FIFO and DMP
	wait(0.015);
  
//	Configure MPU9150 gyro and accelerometer for bias calculation
	MPU9150_writeByte(MPU9150_ADDRESS, CONFIG,       0x01);    				// Set low-pass filter to 188 Hz
	MPU9150_writeByte(MPU9150_ADDRESS, SMPLRT_DIV,   0x00);  				// Set sample rate to 1 kHz
	MPU9150_writeByte(MPU9150_ADDRESS, GYRO_CONFIG,  0x00);  				// Set gyro full-scale to 250 degrees per second, maximum sensitivity
	MPU9150_writeByte(MPU9150_ADDRESS, ACCEL_CONFIG, 0x00); 				// Set accelerometer full-scale to 2 g, maximum sensitivity

	uint16_t  gyrosensitivity  = 131;   									// = 131 LSB/degrees/sec
	uint16_t  accelsensitivity = 16384;  									// = 16384 LSB/g
 
//	Configure FIFO to capture accelerometer and gyro data for bias calculation
	MPU9150_writeByte(MPU9150_ADDRESS, USER_CTRL,   0x40);   				// Enable FIFO  , I2C_MST_EN is disabled, and I2C_MST_RESET for once
	MPU9150_writeByte(MPU9150_ADDRESS, FIFO_EN,     0x78);     				// Enable gyro and accelerometer sensors for FIFO (max size 1024 bytes in MPU9150)
	wait(0.08); 															// accumulate 80 samples in 80 milliseconds = 960 bytes
 
//	At end of sample accumulation, turn off FIFO sensor read
	MPU9150_writeByte(MPU9150_ADDRESS, FIFO_EN, 0x00);        				// Disable gyro and accelerometer sensors for FIFO
	MPU9150_readBytes(MPU9150_ADDRESS, FIFO_COUNTH, 2, &data[0]); 			// Read FIFO sample count
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	packet_count = fifo_count/12;											// How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    MPU9150_readBytes(MPU9150_ADDRESS, FIFO_R_W, 12, &data[0]); 			// Read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8)  | data[1] );		// Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8)  | data[3] );
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8)  | data[5] );  
	
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8)  | data[7] );
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8)  | data[9] );
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]);
    
    accel_bias[0] += (int32_t) accel_temp[0]; 								// Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
	
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
}
  
    accel_bias[0] /= (int32_t) packet_count;								// Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;
    
	if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  	// Remove gravity from the z-axis accelerometer bias calculation
	else {accel_bias[2] += (int32_t) accelsensitivity;}
 
//	Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; 								// Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0]/4)       & 0xFF; 								// Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
	data[3] = (-gyro_bias[1]/4)       & 0xFF;
	data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	data[5] = (-gyro_bias[2]/4)       & 0xFF;
 
//	Push gyro biases to hardware registers 									// Deprecated in Rev 4.2
//	MPU9150_writeByte(MPU9150_ADDRESS, XG_OFFS_USRH, data[0]); 
//	MPU9150_writeByte(MPU9150_ADDRESS, XG_OFFS_USRL, data[1]);
//	MPU9150_writeByte(MPU9150_ADDRESS, YG_OFFS_USRH, data[2]);
//	MPU9150_writeByte(MPU9150_ADDRESS, YG_OFFS_USRL, data[3]);
//	MPU9150_writeByte(MPU9150_ADDRESS, ZG_OFFS_USRH, data[4]);
//	MPU9150_writeByte(MPU9150_ADDRESS, ZG_OFFS_USRL, data[5]);
 
	dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;				// construct gyro bias in deg/s for later manual subtraction
	dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
	dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;
	
/*	Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	the accelerometer biases calculated above must be divided by 8.*/
 
	int32_t accel_bias_reg[3] = {0, 0, 0};									// A place to hold the factory accelerometer trim biases
	
//  MPU9150_readBytes(MPU9150_ADDRESS, XA_OFFSET_H, 2, &data[0]); 			// Read factory accelerometer trim values
	accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
	
//  MPU9150_readBytes(MPU9150_ADDRESS, YA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
	
//  MPU9150_readBytes(MPU9150_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  
	uint32_t mask = 1uL;													// Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = {0, 0, 0}; 										// Define array to hold mask bit for each accelerometer bias axis

	for(ii = 0; ii < 3; ii++) {
		if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; 					// If temperature compensation bit is set, record that fact in mask_bit
	}	
 
//	Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0]/8); 								// Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1]/8);
	accel_bias_reg[2] -= (accel_bias[2]/8);

	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0])      & 0xFF;
	data[1] = data[1] | mask_bit[0]; 										// preserve temperature compensation bit when writing back to accelerometer bias registers
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1])      & 0xFF;
	data[3] = data[3] | mask_bit[1]; 										// preserve temperature compensation bit when writing back to accelerometer bias registers
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2])      & 0xFF;
	data[5] = data[5] | mask_bit[2]; 										// preserve temperature compensation bit when writing back to accelerometer bias registers

	
//	Apparently this is not working for the acceleration biases in the MPU-9250
//	Are we handling the temperature correction bit properly?
	
//	Push accelerometer biases to hardware registers
//	MPU9150_writeByte(MPU9150_ADDRESS, XA_OFFSET_H,    data[0]);  
//	MPU9150_writeByte(MPU9150_ADDRESS, XA_OFFSET_L_TC, data[1]);
//	MPU9150_writeByte(MPU9150_ADDRESS, YA_OFFSET_H,    data[2]);
//	MPU9150_writeByte(MPU9150_ADDRESS, YA_OFFSET_L_TC, data[3]);
//	MPU9150_writeByte(MPU9150_ADDRESS, ZA_OFFSET_H,    data[4]);
//	MPU9150_writeByte(MPU9150_ADDRESS, ZA_OFFSET_L_TC, data[5]);
 
//	Output scaled accelerometer biases for manual subtraction in the main program
	dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
	dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
	dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}
 
void MPU9250_Setup () {
	getAres();																// Get sensor resolutions, only need to do this once
	getGres();
	getMres();

	accelgyrocalMPU9250(gyroBias, accelBias); 								// Calibrate gyro and accelerometers, load biases in bias registers
	wait(1);
	initMPU9250(); 
	uint8_t  d = MPU9150_readByte(AK8975A_ADDRESS, WHO_AM_I_AK8975A);		// Read WHO_AM_I register for AK8975A
	wait(1); 
	initAK8963(magCalibration);												// Initialize device for active mode read of magnetometer
//	magcalMPU9250(magBias, magScale);
}

uint8_t MPU9250_WhoAmI() {
	uint8_t  d = MPU9150_readByte(MPU9150_ADDRESS, WHO_AM_I_MPU9250);		// Should Return 0x71
	return d;
}
	
void MPU9150SelfTest(float * destination) { 								// Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
																			// Accelerometer and gyroscope self test; check calibration wrt factory settings   
	uint8_t rawData[4] = {0, 0, 0, 0};
	uint8_t selfTest[6];
	float factoryTrim[6];
   
//	Configure the accelerometer for self-test
	MPU9150_writeByte(MPU9150_ADDRESS, ACCEL_CONFIG, 0xF0); 				// Enable self test on all three axes and set accelerometer range to +/- 8 g
	MPU9150_writeByte(MPU9150_ADDRESS, GYRO_CONFIG,  0xE0); 				// Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	wait(0.25);  															// Delay a while to let the device execute the self-test

	rawData[0] = MPU9150_readByte(MPU9150_ADDRESS, SELF_TEST_X); 			// X-axis self-test results
	rawData[1] = MPU9150_readByte(MPU9150_ADDRESS, SELF_TEST_Y); 			// Y-axis self-test results
	rawData[2] = MPU9150_readByte(MPU9150_ADDRESS, SELF_TEST_Z); 			// Z-axis self-test results
	rawData[3] = MPU9150_readByte(MPU9150_ADDRESS, SELF_TEST_A); 			// Mixed-axis self-test results
	
//	Extract the acceleration test results first
	selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ; 			// XA_TEST result is a five-bit unsigned integer

	selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 4 ; 			// YA_TEST result is a five-bit unsigned integer
	selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) >> 4 ; 			// ZA_TEST result is a five-bit unsigned integer

//	Extract the gyration test results first
	selfTest[3] = rawData[0]  & 0x1F ; 										// XG_TEST result is a five-bit unsigned integer
	selfTest[4] = rawData[1]  & 0x1F ; 										// YG_TEST result is a five-bit unsigned integer
	selfTest[5] = rawData[2]  & 0x1F ; 										// ZG_TEST result is a five-bit unsigned integer   
   
//	Process results to allow final comparison with factory set values
	factoryTrim[0] = (4096.0f*0.34f)*(pow( (0.92f/0.34f) , ((selfTest[0] - 1.0f)/30.0f)));	// FT[Xa] factory trim calculation
	factoryTrim[1] = (4096.0f*0.34f)*(pow( (0.92f/0.34f) , ((selfTest[1] - 1.0f)/30.0f)));	// FT[Ya] factory trim calculation
	factoryTrim[2] = (4096.0f*0.34f)*(pow( (0.92f/0.34f) , ((selfTest[2] - 1.0f)/30.0f)));	// FT[Za] factory trim calculation
	factoryTrim[3] =  ( 25.0f*131.0f)*(pow( 1.046f , (selfTest[3] - 1.0f) ));				// FT[Xg] factory trim calculation
	factoryTrim[4] =  (-25.0f*131.0f)*(pow( 1.046f , (selfTest[4] - 1.0f) ));				// FT[Yg] factory trim calculation
	factoryTrim[5] =  ( 25.0f*131.0f)*(pow( 1.046f , (selfTest[5] - 1.0f) ));				// FT[Zg] factory trim calculation
   
//  Output self-test results and factory trim calculation if desired
//  Serial.println(selfTest[0]); Serial.println(selfTest[1]); Serial.println(selfTest[2]);
//  Serial.println(selfTest[3]); Serial.println(selfTest[4]); Serial.println(selfTest[5]);
//  Serial.println(factoryTrim[0]); Serial.println(factoryTrim[1]); Serial.println(factoryTrim[2]);
//  Serial.println(factoryTrim[3]); Serial.println(factoryTrim[4]); Serial.println(factoryTrim[5]);
 
/*	Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
	To get to percent, must multiply by 100 and subtract result from 100 */
	for (int i = 0; i < 6; i++) {
		destination[i] = 100.0f + 100.0f*(selfTest[i] - factoryTrim[i])/factoryTrim[i]; 		// Report percent differences
	}
}
 


void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
/*	Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
	(see http://www.x-io.co.uk/category/open-source/ for examples and more details)
	which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
	device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
	The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
	but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz! */
	
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   					// Short name local variable for readability
	float norm;
	float hx, hy, _2bx, _2bz;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;


	float _2q1mx;															// Auxiliary variables to avoid repeated arithmetic
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
	if (norm == 0.0f) return; 												// handle NaN
	norm = 1.0f/norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

//	Normalise magnetometer measurement
	norm = sqrt(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return; 												// handle NaN
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
	s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
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
	
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
	norm = 1.0f/norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;
}
  
  
  
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
	
/*	Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors 
	and measured vectors.*/
	
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   				// short name local variable for readability
	float norm;
	float hx, hy, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;
	float pa, pb, pc;

//	Auxiliary variables to avoid repeated arithmetic
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
	if (norm == 0.0f) return;											// handle NaN
	norm = 1.0f / norm;													// use reciprocal for division
	ax *= norm;
	ay *= norm;
	az *= norm;

//	Normalise magnetometer measurement
	norm = sqrt(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return;											// handle NaN
	norm = 1.0f / norm;													// use reciprocal for division
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
		eInt[0] += ex;										// accumulate integral error
		eInt[1] += ey;
		eInt[2] += ez;
	} 
	else {
		eInt[0] = 0.0f;										// prevent integral wind up
		eInt[1] = 0.0f;
		eInt[2] = 0.0f;
	}

//	Apply feedback terms
	gx = gx + Kp * ex + Ki * eInt[0];
	gy = gy + Kp * ey + Ki * eInt[1];
	gz = gz + Kp * ez + Ki * eInt[2];

//	Integrate rate of change of quaternion
	pa = q2;
	pb = q3;
	pc = q4;
	q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
	q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
	q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
	q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

//	Normalise quaternion
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
	norm = 1.0f / norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;
}

