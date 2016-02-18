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
 
 
/*	1-Wire SHA-1 Authenticated 1Kb EEPROM with 1V8 Operation
 
	1024 Bits of EEPROM Memory Partitioned Into Four Pages of 256 Bits
	
	On-Chip 512-Bit SHA-1 Engine to Compute 160bit Message Authentication Codes (MACs) and to
	Generate Secure Store.
	
	Dedicated 64-Bit Write-Only Secret with a Feature to Extend the Secret Size to 320 Bits by Setting a
	256bit Page as Both Read and Write Protected.
	
	5-Byte Challenge Size for Read Authenticated Page with Optional “Anonymous” Mode
	
	Write Access Requires Knowledge of the Secret and the Capability of Computing and Transmitting
	a 160-Bit MAC as Authorization.
	
	Communicates to Host with a Single Digital Signal at 12.5kbps or 35.7kbps Using 1-Wire Protocol
	
	Reads and Writes Over 1V75V to 3V65V Voltage
	
	Temperatur Range from -20°C to +85°C
	
	Memory Structure....
	00h–1Fh R/W Data Memory Page 0
	20h–3Fh R/W Data Memory Page 1
	40h–5Fh R/W Data Memory Page 2
	60h–7Fh (R)/(W) Data Memory Page 3
	80h–87h W Secrets Memory, No Read Access
	88h R/(W) Write Protect Secret and Memory Range 8Ch to 8Fh/User Byte
	89h R/(W) Write Protect Pages 0 to 3/User Byte
	8Ah R/(W) User Byte, Write Protects Itself if Programmed to Either 55h or AAh
	8Bh Read Factory Byte
	8Ch R/(W) EPROM Mode Control for Page 1/User Byte
	8Dh R/(W) Write Protect Page 0/User Byte
	8Eh–8Fh R/(W) User Bytes/Manufacturer ID. Function depends on 8Bh factory byte.
	90h–97h Read 64-Bit Registration Number, Memory-Mapped Readout
	98h R/(W) Write Protect Page 3/User Byte
	99h R/(W) Read Protect Page 3/User Byte
	9Ah–9Fh Read Factory Bytes. Data varies and is not relevant for user.
	
*/
 
 
/*
	UNDER DEVELOPEMENT - Preliminary Routines Only
	Requires All Drivers to be Done.
 
*/
 
 
#include <stdint.h>  // for uint32_t etc.

#include <string.h>

#include "nrf_delay.h"
#include "nrf_gpio.h"

#include "ds28e02_config.h"
#include "ds28e02.h"

unsigned char ds28e02_id[8];

/* 1-wire is at defined in ds28e02_config.h */
#define SET_PIN_INPUT()  do { NRF_GPIO->DIRCLR = (1UL << One_Wire_Interface_1WI_PIN_NUMBER);  } while(0)   /*!< Configures 1-wire pin as input  */
#define SET_PIN_OUTPUT() do { NRF_GPIO->DIRSET = (1UL << One_Wire_Interface_1WI_PIN_NUMBER);  } while(0)   /*!< Configures 1-wire pin as output  */

#define OUTP_0() do { NRF_GPIO->OUTCLR = (1UL << One_Wire_Interface_1WI_PIN_NUMBER);  } while(0)   			/*!< Pulls 1-wire pin low  */
#define OUTP_1() do { NRF_GPIO->OUTSET = (1UL << One_Wire_Interface_1WI_PIN_NUMBER);  } while(0)   			/*!< Pulls 1-wire pin high */

#define PIN_INIT() do{						\
						SET_PIN_INPUT();	\
						OUTP_0();			\
					} while(0)


/* Drive the one wire interface low */
#define OW_DRIVE() do {						\
						SET_PIN_OUTPUT();	\
						OUTP_0();			\
                   } while (0)

/* Release the one wire by turning on the internal pull-up. */
#define OW_RELEASE() do {					\
						SET_PIN_INPUT();    \
						OUTP_1();           \
					} while (0)

/* Read one bit. */
#define INP()  ((NRF_GPIO->IN >> One_Wire_Interface_1WI_PIN_NUMBER) & 0x1UL)                     /*!< Reads current state of 1-wire pin */
 

/* 
 * Reference: 
 * Device driver for the Dallas Semiconductor DS2401 chip. Heavily
 * based on the application note 126 "1-Wire Communications Through
 * Software".
 *
 * http://www.maximintegrated.com/en/app-notes/index.mvp/id/126
*/
					 
/*
 * Delay times in us.
 */
#define tA 6          				/* min-5,	recommended-6,		max-15 */              
#define tB 64         				/* min-59,	recommended-64,		max-N/A */
#define tC 60         				/* min-60,	recommended-60,		max-120 */                     
#define tD 10         				/* min-5.3,	recommended-10,		max-N/A */
#define tE 9         				/* min-0.3,	recommended-9,		max-9.3 */
#define tF 55        				/* min-50,	recommended-55,		max-N/A */
#define tG 0         				/* min-0,	recommended-0,		max-0 	*/
#define tH 480        				/* min-480,	recommended-480,	max-640 */
#define tI 70         				/* min-60.3,recommended-70,		max-75.3 */
#define tJ 410        				/* min-410,	recommended-410,	max-N/A */
/*---------------------------------------------------------------------------*/
#define udelay(u) nrf_delay_us((u))

/*---------------------------------------------------------------------------*/
static uint8_t ds28_reset(void) {
	uint8_t result;
	OW_DRIVE();
	udelay(500);     					/* 480 < tH < 640 */
	OW_RELEASE();    					/* Releases the bus */
	udelay(tI);
	result = INP();
	udelay(tJ);
	return result;
}

/*---------------------------------------------------------------------------*/
static void ds28_write_byte(uint8_t byte) {
	uint8_t i = 7;
	
	do {
		if (byte & 0x01) {
			OW_DRIVE();
			udelay(tA);
			OW_RELEASE();					/* Releases the bus */
			udelay(tB);
		} 
	else {
		OW_DRIVE();
		udelay(tC);
		OW_RELEASE();						/* Releases the bus */
		udelay(tD);
	}
	if (i == 0) return;
	i--;
	byte >>= 1;
	} while (1);
}

/*---------------------------------------------------------------------------*/
static unsigned ds28_read_byte(void) {
	unsigned result = 0;
	int i = 7;
	do {
		OW_DRIVE();
		udelay(tA);
		OW_RELEASE();					/* Releases the bus */
		udelay(tE);
		if (INP()) result |= 0x80;		/* LSbit first */
		udelay(tF);
		if (i == 0) return result;
		i--;
		result >>= 1;
	} while (1);
}

/*---------------------------------------------------------------------------*/
/* Polynomial ^8 + ^5 + ^4 + 1 		ie -->	POLYNOMIAL = X8 + X5 + X4 + 1 */
static unsigned crc8_add(unsigned acc, unsigned byte) {
	int i;
	acc ^= byte;
	for (i = 0; i < 8; i++)
		if (acc & 1) acc = (acc >> 1) ^ 0x8c;
		else acc >>= 1;
		return acc;
}

int ds28e02_Read_Factory(void) {

	unsigned factory;

	NRF_GPIO->PIN_CNF[One_Wire_Interface_1WI_PIN_NUMBER] =			\
		  (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)	\
		| (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos)	\
		| (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)	\
		| (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)	\
		| (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos);	

	PIN_INIT();

if (ds28_reset() == 0) {
		ds28_write_byte(0x8b);    						/* Read Factory Byte at address 0x8b  --> returns 0x55 or 0xAA*/   
		factory = ds28_read_byte();
	return factory;  									/* Note: Expect Value of 0x55 or 0xAA */
	}
	return 1;											// fail
}

int ds28e02_initAndRead() {
  int i;
  unsigned family, crc, acc;

	NRF_GPIO->PIN_CNF[One_Wire_Interface_1WI_PIN_NUMBER] =			\
		  (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)	\
		| (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos)	\
		| (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)	\
		| (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)	\
		| (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos);	

	PIN_INIT();

	// clear id first
	memset(ds28e02_id, 0x0, sizeof(ds28e02_id));

if (ds28_reset() == 0) {
	
	ds28_write_byte(0x90);    							/* Read ROM Registration (64bits) command. Adrr=90_to_97 family id at lower byte 90 or 97 ??*/   
	
	family = ds28_read_byte();
	
	for (i = 7; i >= 2; i--) ds28e02_id[i] = ds28_read_byte(); // Get 48 bits ID --> Actual EEPROM ID size == 64bits
	
	crc = ds28_read_byte();

	if(family != 0x4f) 	goto fail;					// Family code always 0x4fh == DS28E02 --> Seeing 0x46 ?????
		
	acc = crc8_add(0x0, family);
	for (i = 7; i >= 2; i--) acc = crc8_add(acc, ds28e02_id[i]);
	
	if (acc == crc) return 0; 						/* Success! */
} 
	return 1;  										/* Fail! */

fail:
	  memset(ds28e02_id, 0x0, sizeof(ds28e02_id));	// clear the buffer upon fail
	  return 1;  									/* Fail! */

}



int ds28e02_write_ScratchPad (void){				// cmd = 0x0F   (Scratchpad Size==64-bit)
	
	return 1;  										/* Fail! */
}

int ds28e02_read_ScratchPad (void){					// cmd = 0xAA   (Scratchpad Size==64-bit)
	
	return 1;  										/* Fail! */
}

void ds28e02_refresh_scratchpad (void) {			// cmd =    (32bits) (8bytes)

}


void ds28e02_read_page (int page, int addr) {		// cmd =     (Four pages of 256bits) (4x 32bytes)

}

void ds28e02_write_page (int page, int addr) {		// cmd =     (Four pages of 256bits) (4x 32bytes)

}

void ds28e02_read_authentication_page (void) {		// cmd =
}

void ds28e02_copy_scratchpad(void){					// cmd = 
}

void ds28e02_load_first_secret(void){
}

void ds28e02_compute_next_secret(){
}

void ds28e02_read_ROM(void){
}

void ds28e02_match_ROM(void){
}

void ds28e02_search_ROM(void){
}

void ds28e02_skip_ROM(void){
}

void ds28e02_resume_communication(void){
}

void ds28e02_overdrive_skip_ROM(void){
}

void ds28e02_overdrive_Match_ROM(void){
}

void ds28e02_read_secret_memory(void){						// 64-bits (8bytes)
}

void ds28e02_read_SHA1(void){								// Secure Hash 512bit
}
