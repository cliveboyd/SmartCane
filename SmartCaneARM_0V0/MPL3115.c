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
 
#include "MPL3115.h"
#include "nrf_delay.h"
#include "Communication.h"

/**************************************************************************/
/*!
    @brief  Setups the HW (reads coefficients values, etc.)
*/
/**************************************************************************/
bool MPL3115A2_init(void) {
  uint8_t whoami = MPL3115A2_read8(MPL3115A2_WHOAMI);
  while (whoami != 0xC4) {
    whoami = MPL3115A2_read8(MPL3115A2_WHOAMI);
  }

  MPL3115A2_write8(MPL3115A2_CTRL_REG1, // 0x26
	 MPL3115A2_CTRL_REG1_OS128 |
	 MPL3115A2_CTRL_REG1_ALT);   // 0xB8

  MPL3115A2_write8(MPL3115A2_PT_DATA_CFG,   // 0x13
	 MPL3115A2_PT_DATA_CFG_TDEFE |
	 MPL3115A2_PT_DATA_CFG_PDEFE |
	 MPL3115A2_PT_DATA_CFG_DREM);   // 0x07
  return true;
}

/*********************************************************************/
bool MPL3115A2_readBytes_(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest, bool loop)
{     
	int i=0;
	for(;i<5;i++)
	{
		if(I2C_Write(address,
				  (unsigned char*)&subAddress,
				  1,
				  0))
				  break; // if written length > 0
			if (!loop) break;
	}
	if (i>=5) 
		return false;
	for(i=0;i<5;i++)
	{
		if(I2C_Read(address,
             dest,
             count,
             1))
			break;  // if read length > 0
		if (!loop) break;
	}
	if (i<5)
		return true;
	return false;
}

uint8_t MPL3115A2_read8(uint8_t a) {
	int i=0;
	uint8_t out;
	for(;i<5;i++)
	{
		if(I2C_Write(MPL3115A2_ADDRESS,
				  (unsigned char*)&a,
				  1,
				  0))
				  break; // if written length > 0
	}
	if (i>=5) 
		return 0xff;  // failed
	for(i=0;i<5;i++)
	{
		if(I2C_Read(MPL3115A2_ADDRESS,
             &out,
             1,
             0))
			break;  // if read length > 0
	}
	if (i<5)
		return out;
	return 0xff;
}

bool MPL3115A2_write8(uint8_t a, uint8_t d) {
	unsigned char data_write[2];
	data_write[0] = a;
	data_write[1] = d;
	int i;
	for(i=0;i<5;i++)
	{
		if(I2C_Write(MPL3115A2_ADDRESS,
              data_write,
              2,
              1))  // no stop bit
			break;
	}
	if(i<5) {
		d = MPL3115A2_read8(a);
		while(d != data_write[1])
		{
			if(I2C_Write(MPL3115A2_ADDRESS,
              data_write,
              2,
              1))  // no stop bit
			{
				d = MPL3115A2_read8(a);
				if (d == data_write[1])
					return true;
			}
			i++;
			if (i>5)
				break;
		}
		return true;
	}
		return true;
	return false;
}

/**************************************************************************/
/*!
    @brief  Gets the floating-point temperature in Centigrade
*/
/**************************************************************************/
float MPL3115A2_getTemperature() {
	uint16_t t;

	uint8_t sta = 0;
	while (! (sta & MPL3115A2_REGISTER_STATUS_TDR)) {
		sta = MPL3115A2_read8(MPL3115A2_REGISTER_STATUS);
		nrf_delay_ms(10);
	}
  
	uint8_t out[2];
	MPL3115A2_readBytes_(MPL3115A2_ADDRESS, MPL3115A2_REGISTER_TEMP_MSB, 2, out, true);

	t = (out[0]<<8)|out[1];
	t >>= 4;

	float temp = t;
	temp /= 16.0;
	return temp;
}

float MPL3115A2_getAltitude() {
	int32_t alt;

	MPL3115A2_write8(MPL3115A2_CTRL_REG1, // 0x26
		MPL3115A2_CTRL_REG1_SBYB |
		MPL3115A2_CTRL_REG1_OS128 |
		MPL3115A2_CTRL_REG1_ALT);  // 0xB9

	uint8_t sta = 0;
	sta = MPL3115A2_read8(MPL3115A2_CTRL_REG1);
	sta = MPL3115A2_read8(MPL3115A2_REGISTER_STATUS);
	while (! (sta & (MPL3115A2_REGISTER_STATUS_PDR|MPL3115A2_REGISTER_STATUS_PTDR)) ) {
		//sta = MPL3115A2_read8(MPL3115A2_WHOAMI);
		nrf_delay_ms(200);
		sta = MPL3115A2_read8(MPL3115A2_REGISTER_STATUS);
		if ((sta & (MPL3115A2_REGISTER_STATUS_PDR|MPL3115A2_REGISTER_STATUS_PTDR))) 
			break;
		MPL3115A2_write8(MPL3115A2_CTRL_REG1, // 0x26
		MPL3115A2_CTRL_REG1_SBYB |
		MPL3115A2_CTRL_REG1_OS128 |
		MPL3115A2_CTRL_REG1_ALT);  // 0xB9
		sta = MPL3115A2_read8(MPL3115A2_REGISTER_STATUS);
	}
	uint8_t out[3];
	MPL3115A2_readBytes_(MPL3115A2_ADDRESS, MPL3115A2_REGISTER_PRESSURE_MSB, 3, out, true);

	alt = (out[0]<<16)|(out[1]<<8)|out[2]; // receive DATA
	alt >>= 4;

	if (alt & 0x800000) {
		alt |= 0xFF000000;
	}

	float altitude = alt;
	altitude /= 16.0;
	return altitude;
}

/**************************************************************************/
/*!
    @brief  Gets the floating-point pressure level in kPa
*/
/**************************************************************************/
float MPL3115A2_getPressure() {
	uint32_t pressure;

	MPL3115A2_write8(MPL3115A2_CTRL_REG1, 
		MPL3115A2_CTRL_REG1_SBYB |
		MPL3115A2_CTRL_REG1_OS128 |
		MPL3115A2_CTRL_REG1_BAR);

	uint8_t sta = 0;
	while (! (sta & MPL3115A2_REGISTER_STATUS_PDR)) {
		sta = MPL3115A2_read8(MPL3115A2_REGISTER_STATUS);
		nrf_delay_ms(10);
	}
  
  	uint8_t out[3];
	MPL3115A2_readBytes_(MPL3115A2_ADDRESS, MPL3115A2_REGISTER_PRESSURE_MSB, 3, out, true);

	pressure = (out[0]<<16)|(out[1]<<8)|out[2]; // receive DATA
	pressure >>= 4;

	float baro = pressure;
	baro /= 4.0;
	return baro;
}
