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
 
#ifndef _COMMUNICATION_H_
#define _COMMUNICATION_H_



/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/
#include <stdint.h>  // for uint32_t etc.
#include <stdbool.h>
#include "spi_master.h"

#ifndef u8
typedef				uint8_t     u8;
typedef 			uint16_t    u16;
typedef 			uint32_t    u32;
typedef				uint64_t    u64;

typedef volatile	u8			vu8;
typedef volatile	u32			vu32;
typedef volatile	u64			vu64;
#endif

/******************************************************************************/
/* Functions Prototypes                                                       */
/******************************************************************************/

/* Initializes the I2C communication peripheral. */
char SPI_write(	const spi_master_hw_instance_t	spi_master_hw_instance,
						uint8_t * const					p_tx_data,
						uint8_t * const					p_rx_data,
						const uint16_t					len);

char AT45_spi_write(uint8_t Byte);
						
uint16_t AT45_SPIM0_write_16b(unsigned short regValue);

uint16_t A2035H_SPIM0_Write_16b(unsigned short regValue);
						
unsigned char SPI_Init(void);


						

/* Initializes the I2C communication peripheral. */
unsigned char I2C_Init(void);

/* Writes data to a slave device. */
unsigned char I2C_Write(unsigned char slaveAddress,
                        unsigned char *dataBuffer,
                        unsigned char bytesNumber,
                        unsigned char stopBit);

/* Reads data from a slave device. */
unsigned char I2C_Read(unsigned char slaveAddress,
                       unsigned char *dataBuffer,
                       unsigned char bytesNumber,
                       unsigned char stopBit);

#endif // _COMMUNICATION_H
