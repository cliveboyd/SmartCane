/***************************************************************************//**
 *	@file   A2035H.h
 *	@device GPS Multi Sateliite Hybrid RF Receiver UART-I2C-SPI
 *	@brief  Header file of A2035H Driver.
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
 
 
 /*
 MANUFACTURERS NOTES... Clarification from Ambigueous and Flakey Application Note
 
 GSD4e-Host-Platform-Interfaces-Application-Note-CS-211776-AN-3.pdf
 
 In general the A2035 supports SSPI (Slave SPI).
 The A2035-H is an SPI SLAVE (The HOST is the MASTER)
- Your µP (the host) is the master.
- The Host SPI_CLK (is an output)
- The Host SPI_CS (is an output)


SPI-CS  on the A2035 ..	Pin 8,	input,	1V8 (3V3 tolerant)
SPI-TX  on the A2035 ..	Pin 20,	output,	3V3
SPI-RX  on the A2035 ..	Pin 21,	Input,	1V8 (3V3 tolerant)
SPI-CLK on the A2035 ..	Pin 7,	Input,	1V8 (3V3 tolerant)

Leave the A2305H nRST open (triState) upon Power UP NOT pulled high to 3V3 or 1V8.
Leave the A2305H nCS  open (triState) upon Power UP NOT pulled high to 3V3 or 1V8.
 
 */
 
 
/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/
#include <stdint.h>  // for uint32_t etc.
#include <stdbool.h>

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

/* Toggle the Start On-OFF Pin. */
void initA2035H(void);

/* Initializes the SPI communication peripheral and reset the part. */
void A2035H_Init_IO(void);

/* Reset the A2035H. */
void A2035H_Reset(void);

/* Writes a 16bit value to the A2035. */
void A2035H_SPI_WriteReg(unsigned short regValue);


/* Read a single 8bit value from the A2035. */
char A2035H_SPI_ReadByte(void);

/* Read a A Packet of characters from the A2035. */
void A2035H_SPI_ReadPacket(int *RxPacket, int ByteCount);

/* Scheduled Read of A2035 GPS SPI - Load 32 byte packet and send on to NEMA parser*/
void A2035H_Sheduled_SPI_Read(void);

void A2035H_Toggle_ONOFF(void);

void A2035H_RESET_ON(void);

void A2035H_RESET_OFF(void);

void A2035H_POWER_OFF(void);

void A2035H_POWER_ON(void);

void A2035H_tristate_SPI_Bus_Pins(void);			// Also tristates A2035 NRST Reset ---. re manufacture suggestion upon 3V3GPS power-up

void A2035H_Configure_Non_SPI_IOPins(void);
