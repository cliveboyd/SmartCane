/*	AT45DB Currently Bit Crunched SPI MASTER Serial Flash primarilly targeting AT45DB161E with SPI IO Lines shared with A2035H GPS Module

*	Author...	Clive S. Boyd
*	Date...		February 2016

*	Modification of Original Code from mbed AT45 Library, for driving the Atmel AT45 series Dataflash with Serial Interface (SPI)

*	Copyright (c) 2012, Created by Steen Joergensen (stjo2809) inspired by Chris Styles AT45 library
*
*	Permission is hereby granted, free of charge, to any person obtaining a copy
*	of this software and associated documentation files (the "Software"), to deal
*	in the Software without restriction, including without limitation the rights
*	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*	copies of the Software, and to permit persons to whom the Software is
*	furnished to do so, subject to the following conditions:
*
*	The above copyright notice and this permission notice shall be included in
*	all copies or substantial portions of the Software.
*
*	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
*	THE SOFTWARE.
*/
 
/*	Note:	The AT45DB161 SFLASH SPI IO Lines are shared with the A2035-H GPS Module.
 			
			Both the SFLASH  AT45DB161 and GPS A2035-H Operate in Slave Mode with Host as MASTER
						
*/
 
 
 
 /*
 ToDo	Convert SPI Bit Crunch Routine to SPI-Master Hardware....
  
 */
 
#include <AT45.h>

#include <stdbool.h> 
#include <stdint.h>
#include <stdio.h>

#include "nrf_gpio.h"
#include "app_error.h"
#include "app_util_platform.h"

#include "nrf_delay.h"

#include "A2035H.h"																// Include to allow access to A2035H_RESET_ON();
#include "global.h"

#define SPI_BCM_SCK_PIN       (19U)     										//	AT45DB161E SFLASH BIT CRUNCH MASTER SCLK  
#define SPI_BCM_MISO_PIN      (20U)     										//	AT45DB161E SFLASH BIT CRUNCH MASTER MOSI
#define SPI_BCM_MOSI_PIN      (17U)     										//	AT45DB161E SFLASH BIT CRUNCH MASTER MISO
#define SPI_BCM_SS_PIN        (18U)  											//	AT45DB161E SFLASH BIT CRUNCH MASTER nCS


//	WARNING...	A2035H GPS Module needs to be in Reset State during operation of AT45DB161E SFLASH

#define AT45DB_NCS_HIGH		NRF_GPIO->OUTSET = (1UL << SPI_BCM_SS_PIN)			// Macro to Assert   AT45DB161 NCS HIGH (OFF)
#define AT45DB_NCS_LOW		NRF_GPIO->OUTCLR = (1UL << SPI_BCM_SS_PIN)			// Macro to Deassert AT45DB161 NCS LOW  (ON)

#define AT45DB_SCLK_HIGH	NRF_GPIO->OUTSET = (1UL << SPI_BCM_SCK_PIN)			// Macro to Deassert AT45DB161 SCLK HIGH
#define AT45DB_SCLK_LOW		NRF_GPIO->OUTCLR = (1UL << SPI_BCM_SCK_PIN)			// Macro to Deassert AT45DB161 NCS LOW

#define AT45DB_MOSI_HIGH	NRF_GPIO->OUTSET = (1UL << SPI_BCM_MOSI_PIN)		// Macro to Deassert AT45DB161 MOSI PIN HIGH
#define AT45DB_MOSI_LOW		NRF_GPIO->OUTCLR = (1UL << SPI_BCM_MOSI_PIN)		// Macro to Deassert AT45DB161 MOSI PIN LOW

#define AT45_MISO_READ() 	((NRF_GPIO->IN >> SPI_BCM_MISO_PIN) & 0x1UL)		//  Reads current state of MISO Pin 

//=============================================================================
// Public functions
//=============================================================================

int pages		= 4096;															// Number of pages 
int pagesize	= 528;															// Size of pages -->  Expected Default page size == 528 ---> 256/264, 512/528, 1024/1056 bytes
int devicesize	= 512;															// In bytes
int blocks		= 512;															// Number of blocks 512 in AT45DB161E (One Block == 4096/4224 bytes)
int sectors		= 17;															// Number of sectors 0_to_17 (One sector == 256 pages (135168 bytes))
																				// Note Special Case... Sector [0a=8pages (4224 bytes)] and [Secor0b=248pages (130994 bytes)]

bool deep_down = true;															// Variable for deep power down function (awake ?)
bool deep_down_onoff = false;													// Variable for deep power down function (On/Off) 
 
void AT45DB161E_init(void) {													// AT45DB161E == 16Mbit --> http://www.adestotech.com/wp-content/uploads/doc8782.pdf
	
	A2035H_RESET_ON();
	nrf_delay_us(250);
	A2035H_POWER_OFF();															// RESET ASSERTED and 3V3GPS Power Maintained Forced OFF
	nrf_delay_us(100000);
	
	AT45_Congigure_IO_Pins();													// ToDo Sort Out PIN Config at A2035H.c
	
	AT45_initialize();
	
	nrf_delay_us(100000);														// 100ms Delay required to return correct during initial read of WhoAmI Value!!!
}

char AT45_read_byte(int address) {												// Return byte from address	---> This function returns the char
	return (AT45_memread( address ));
}  

int AT45_read_page(char* data, int page) {
	int address = -1;

	if(pagesize == 256)			address = page * 256;
	else if(pagesize == 264)	address = page * 512;
	else if(pagesize == 512)	address = page * 512;
	else if(pagesize == 528)	address = page * 1024;
	else if(pagesize == 1024)	address = page * 1024;
	else if(pagesize == 1056)	address = page * 2048;
	else {
		return (-1);															 // Something isn't configured right ---> Exit
	}
				
	AT45_busy();        
	AT45_flashread(1,address); 													// Read the first page of the block into SRAM buffer 1
	AT45_busy();               													// Wait until First page has loaded into buffer 1
		
	// This is done directly as we are reading back an entire buffer, and this can be done more optimally than using _sramread

	AT45_select();
	spi_bcm_write(0xd4);
	AT45_sendaddr (0x0);
	spi_bcm_write (0x0);														// Don't care byte
		
	for(int i=0; i<pagesize ;i++) {
		data[i] = spi_bcm_write (0x0);  
	}
	AT45_deselect();            
	return (0);																	// Exit OK
}
																				// This function writes the char to the address supplied
																				// Note : We pass the raw address to the underlying functions
void AT45_write_byte(int address, char data) {
	AT45_busy();   
	AT45_flashread(1,address);            										// Read the Flash page into SRAM buffer
	AT45_busy();																// Wait for the read to complete

	AT45_sramwrite(1,address,data);        										// Write new data into SRAM
	AT45_busy();                           										// Make sure flash isn't busy

	AT45_flashwrite(1,address);            										// Write back to the page address
}       
      
int AT45_write_page(char* data, int page) {
	int address = -1;

	if(pagesize == 256)			address = page * 256;
	else if(pagesize == 264)	address = page * 512;
	else if(pagesize == 512)	address = page * 512;
	else if(pagesize == 528)	address = page * 1024;
	else if(pagesize == 1024)	address = page * 1024;
	else if(pagesize == 1056)	address = page * 2048;

	else return (-1);															// Something isn't configured right ---> Exit

	AT45_select();

	spi_bcm_write(0x84);														// Writing to buffer #1

	AT45_sendaddr (0);															// We are writing to the entire buffer

	for(int i=0; i<pagesize ;i++) {
		spi_bcm_write ((char) data[i]);  
	}

	//   AT45_deselect();        
	//   AT45_busy();																// Make sure the Flash isn't busy
																				// Issue command to write buffer 1 to the appropraite flash page
	AT45_select();  
	spi_bcm_write (0x83);  														// SRAM Buffer 1 to SFLASH Main Memory (Pg Size = 512 byte)
	AT45_sendaddr (AT45_getpaddr(address));										// Get Page Address
	AT45_deselect();

	return (0);   
}


int AT45_write_block(char *data[], int block) {									// !!!! ToDo... Under construction F&#65533; CHECK AF MIC OG LERCHE 
	char temp_data[pagesize];
	int page_start;

	if(block < blocks || block == 0) {
		page_start = block * 8;

		for(int i=0; i<8 ;i++) {
			
			for(int z=0; z<pagesize ;z++) {
			   temp_data[z] = data[i][z];    
			}   
			
			AT45_write_page(temp_data, page_start);
			page_start = page_start + 1;
		}
	} 
	else {																		// Do nothing
	}

	return (0);
}
 
int AT45_FAT_read(char* data, int page) {										// Read FAT --> File Allocation Table

	if((pagesize == 256) || (pagesize == 264)) {								// Note: For 256 byte pages, we read two pages
		int address = page * 512;												// This is the start address of the 512 byte block
		   
		AT45_flashread(1, address);												// Read the first page of the block into SRAM buffer 1
		AT45_busy();															// Wait until First page has loaded into buffer 1

		// This is done directly as we are reading back an entire buffer, and this can be done more optimally as a Flash read rather then using _sramread
		AT45_select();
		spi_bcm_write (0xd4);
		AT45_sendaddr (0x00);
		spi_bcm_write (0x00);													// Don't care byte

		for(int i=0;i<256;i++) {
			data[i] = spi_bcm_write (0x00);  
		}
		AT45_deselect();            
		
		AT45_flashread(2,address+256);											// Read the second page of the block into SRAM buffer 2
		AT45_busy();															// Wait until second page has loaded into buffer 2
		
		// Now the second page is loaded, pull this out into the second half of the data buffer
		// This is done directly as we are reading back an entire buffer, and this can be done more optimally than using _sramread
		AT45_select();
		spi_bcm_write (0xd4);
		AT45_sendaddr (0x00);
		spi_bcm_write (0x00);													// Don't care byte
		
		for(int i=0;i<256;i++) {
			data[256+i] = spi_bcm_write (0x0);  
		}
		AT45_deselect();                
		return (0);
	}

	// For 512 byte pages, we read just the single page, transfer it    
	else if((pagesize == 512) || (pagesize == 528)) {
		int address = page * 512;												// This is the start address of the 512 byte block

		AT45_busy();															// Wait until First page has loaded into buffer 1
		AT45_flashread(1, address);												// Read the first page of the block into SRAM buffer 1
		AT45_busy();															// Wait until First page has loaded into buffer 1

		// Now the page has loaded, simply transfer it from the sram buffer to the data array
		// This is done directly as we are reading back an entire buffer, and this can be done more optimally than using _sramread
		AT45_select();
		spi_bcm_write (0xd4);
		AT45_sendaddr (0x00);
		spi_bcm_write (0x00);													// Don't care byte

		for(int i=0;i<512;i++) {
			data[i] = spi_bcm_write (0x00);  
		}
		AT45_deselect();            
		return (0);
	}

	// For 1024 byte pages, we read just a single page, transfer half of it
	else if((pagesize == 1024) || (pagesize == 1056)) {
		int address = AT45_getpaddr(page * 512);								// This is the start address of the 512 byte block

		AT45_busy();															// Wait until First page has loaded into buffer 1

		AT45_flashread(1,address);												// Read the first page of the block into SRAM buffer 1

		AT45_busy();															// Wait until First page has loaded into buffer 1

		// Now the page has loaded, simply transfer it from the sram buffer to the data array
		// This is done directly as we are reading back an entire buffer, and this can be done more optimally than using _sramread

		AT45_select();
		spi_bcm_write(0xd4);

		if (page %2) {															// Odd numbered block, read from adress 0x200
			AT45_sendaddr (0x200);
		} 
	else {																		// Even numbered block, then we are reading from sram buffer 0x0 
			AT45_sendaddr (0x00);
		}

		spi_bcm_write (0x00);   													// Don't care byte

		for(int i=0;i<512;i++) {
			data[i] = spi_bcm_write (0x00);  
		}
		AT45_deselect();            
		return (0);
	} 
	else {
		return (-1); 					// Something isn't configured right
	}
}

int AT45_FAT_write(char* data, int page) {

	if((pagesize == 256) || (pagesize == 264)) {    							// For 256 byte pages, we overwrite two pages
		
																				// Fill the first buffer with the first half of the block
																				// Do this directly, for better performance
		AT45_select();
		spi_bcm_write(0x84);													// Writing to buffer #1
		AT45_sendaddr (0);														// We are writing to the entire buffer

		for(int i=0;i<256;i++) {
			spi_bcm_write (data[i]);  
		}
		AT45_deselect();        
				
		AT45_flashwrite(1,(page*512));

																				// Fill the buffer with the second half of the block
																				// Do this directly, for better performance
		AT45_select();
		spi_bcm_write(0x84);													// Writing to buffer #1
		AT45_sendaddr (0);														// We are writing to the entire buffer

		for(int i=0;i<256;i++) {
			spi_bcm_write (data[256+i]);  
		}
		AT45_deselect();        

		AT45_flashwrite(1,((page*512)+256));
	}
		
																				// For 512 byte pages, we overwrite a single page
	else if((pagesize == 512) || (pagesize == 528)) {

																				// Fill the first buffer with the block data
																				// Do this directly, for better performance
		AT45_select();
		spi_bcm_write(0x84);													// Writing to buffer #1
		AT45_sendaddr (0);														// We are writing to the entire buffer

		for(int i=0;i<512;i++) {
			spi_bcm_write (data[i]);  
		}

		AT45_deselect();        
		AT45_busy();															// Make sure the flash isn't busy
				
																				// Issue command to write buffer 1 to the appropraite flash page
		AT45_select();  
		spi_bcm_write (0x83);  
		AT45_sendaddr (AT45_getpaddr(page * 512));  
		AT45_deselect();                
	}

																				// For 1024 byte pages, we do a read modify write
																				// Must make sure we overwrite the right half of the page!
	else if((pagesize == 1024) || (pagesize == 1056)) {

		AT45_busy();															// Make sure the flash isn't busy

		int address = AT45_getpaddr(page*512);

		AT45_flashread(1,address);  											// Read the page into sram
		
																				// Wait for this operation to complete
		AT45_busy();
		
																				// Overwrite the appropriate half
																				// Do this directly, for better performance
		AT45_select();	
		spi_bcm_write(0x84);													// Writing to buffer #1

		if(page%2) {															// This is an odd block number, overwrite second half of buffer 
			AT45_sendaddr (0x200); 												// We are writing to the entire buffer
		} 
	else {       																// This is an even block, overwrite the first half
			AT45_sendaddr (0x0);												// We are writing to the entire buffer
		}

		for(int i=0;i<512;i++) {
			spi_bcm_write (data[i]);  
		}
		AT45_deselect();        
		
																				// Write the page back
		AT45_busy();
		AT45_flashwrite(1,address); 
	} 
	else {

		return (-1); 															// Something has gone wrong
	}

	return (0);
}
       
void AT45_chip_erase(void) { 													// Erase the entire chip
	AT45_busy();																// Make sure flash isn't already in busy.

	AT45_select();
																				// Four (4) byte command sequence
	spi_bcm_write(0xc7);
	spi_bcm_write(0x94);
	spi_bcm_write(0x80);
	spi_bcm_write(0x9a);
	AT45_deselect();  

	AT45_busy();																// Make erase a blocking function
}        
 
void AT45_(int block) {															// Erase one block  
	int address = -1;
																				// Calculate page addresses
	if(block < blocks || block == 0) {
		if(pagesize == 256)			address = block * 2048;
		else if(pagesize == 264)	address = block * 4096;
		else if(pagesize == 512)	address = block * 4096;
		else if(pagesize == 528)	address = block * 8192;
		else if(pagesize == 1024)	address = block * 8192;
		else if(pagesize == 1056)	address = block * 16384;
				
		AT45_busy();
		AT45_select();
		spi_bcm_write(0x50);
		AT45_sendaddr (address);
		AT45_deselect();
		AT45_busy();   
	} 
	else {
																				// Do nothing   
	}
}
 			
void AT45_page_erase(int erase_page) {											// Erase one page  AT45DB161 ---> 528 bytes     
	int address = -1;

	// Calculate page addresses
	if(erase_page < pages || erase_page == 0) {
		if(pagesize == 256)			{address = erase_page * 256;}
		else if(pagesize == 264)	{address = erase_page * 512;}
		else if(pagesize == 512)	{address = erase_page * 512;}
		else if(pagesize == 528)	{address = erase_page * 1024;}
		else if(pagesize == 1024)	{address = erase_page * 1024;}
		else if(pagesize == 1056)	{address = erase_page * 2048;}
		
		AT45_busy();
		AT45_select();
		spi_bcm_write(0x81);
		AT45_sendaddr (address);
		AT45_deselect();
		AT45_busy();   

	} 
	else {
																				// Do nothing   
	}
}
 
void AT45_sector_erase(int erase_sector) {										// Erase a Sector 0-15... One Sector = 128kbytes
	int address = -1;

	// Calculate page addresses
	if(erase_sector < sectors || erase_sector != 0) {							// skip sector 0 as it it is split into 0a and 0b 1_to_16 OK
		if(pagesize == 256)			{address = erase_sector * 65536;}
		else if(pagesize == 264)	{address = erase_sector * 67584;}
		else if(pagesize == 512)	{address = erase_sector * 131072;}
		else if(pagesize == 528)	{address = erase_sector * 135168;}			// Expected for Standard AT45DB161E pagesize==528
		else if(pagesize == 1024)	{address = erase_sector * 262144;}
		else if(pagesize == 1056)	{address = erase_sector * 524288;}
		
		AT45_busy();
		AT45_select();
		spi_bcm_write(0x7c);													// Erase sector command
		AT45_sendaddr (address);
		AT45_deselect();
		AT45_busy();   
	} 
	else {
																				// Do nothing   
	}	
}


void AT45_block_erase(int erase_block) {										// Erase a Block... One block = 4kbytes
	int address = -1;

	// Calculate page addresses
	if(erase_block < blocks || erase_block == 0) {								// skip sector 0 as it it is split into 0a and 0b 1_to_16 OK
		if(pagesize == 256)			{address = erase_block * 4096;}
		else if(pagesize == 264)	{address = erase_block * 4224;}
		else if(pagesize == 512)	{address = erase_block * 4096;}
		else if(pagesize == 528)	{address = erase_block * 4224;}				// Expected for Standard AT45DB161E pagesize==528
		else if(pagesize == 1024)	{address = erase_block * 4096;}
		else if(pagesize == 1056)	{address = erase_block * 4224;}
		
		AT45_busy();
		AT45_select();
		spi_bcm_write(0x50);													// Erase Block Command
		AT45_sendaddr (address);
		AT45_deselect();
		AT45_busy();   
	} 
	else {
																				// Do nothing   
	}	
}	


int AT45_device_size(void) { 													// ToDo Return the size of the part in bytes
	return devicesize;			// Hardset
}       
 
int AT45_pages(void) { 															// ToDo Return the numbers of pages
	return pages;				// Hardset
}       
       
int AT45_pagesize(void) { 														// ToDo Return the page size of the part in bytes
	return pagesize;			// Hardset
}       
       									
void AT45_setpagesize_to_binary(void) {											// A one-time programmable configuration
	AT45_busy();																// Make sure flash isn't already in busy.

	AT45_select();																// Four (4) byte command sequence
	spi_bcm_write(0x3d);
	spi_bcm_write(0x2a);
	spi_bcm_write(0x80);
	spi_bcm_write(0xa6);

	AT45_deselect();  
	AT45_busy();																// Make erase a blocking function   
}
 
															
int AT45DB161E_blocks (void) {													// Return the number of blocks in this device in accordance with the datasheet
    return blocks;				// Hardset										// ToDo read device
}

int AT45_WhoAmI(void) { 
	int id = 0;
	AT45_select();
	spi_bcm_write(0x9f);														// cmd to read Manufactures id ==0x1f, Device id Byte 1 == 0x26h, device id byte 2 == 0x00
	id = (spi_bcm_write(0x00) << 8);
	id |= spi_bcm_write(0x00);
	AT45_deselect();
	return id;
}

int AT45_id(void) { 															// Return id of the part == 0x1f26
	int id = 0;
	AT45_select();
	spi_bcm_write(0x9f);														// cmd to read Manufactures id ==0x1f, Device id Byte 1 == 0x26h, device id byte 2 == 0x00
	id = (spi_bcm_write(0x00) << 8);
	id |= spi_bcm_write(0x00);
	AT45_deselect();
	return id;
}

/*	To read the Status Register, the nCS pin must first be asserted 
	and then the opcode D7h must be clocked into the device.
	After the opcode has been clocked in, the device will begin outputting 
	Status Register data on the SO pin during every subsequent clock cycle.
*/

int AT45_status(void) { 														// Return the Status  ready xx.7=1 busy xx.7=0
    int status = 0;
    AT45_select();
	spi_bcm_write(0xd7);														// cmd to read status register
    status = (spi_bcm_write(0x00));												// write out dummy and read back first(1st) byte of status
    AT45_deselect();            
    return status; 
}

int AT45_test_pagesize_528(void) { 												// Test Page Size via status.0 (xx.0=1 --> Page Size=512    xx.0=0 ---> Page Size=528
    int status = AT45_status();
    status = status && 0x01;
    if (status==0) return 0;
	else return -1; 
}

int AT45_test_pagesize_512(void) { 												// Test Page Size via status.0 (xx.0=1 --> Page Size=512    xx.0=0 ---> Page Size=528
	int status = AT45_status();
	status = status && 0x01;
	if (status==1) return 0;
	else return -1; 
}

void AT45_deep_power_down(bool deep_down_onoff) {								// Wake up from deep power down
	if(deep_down_onoff == false) {				
		AT45_select();
		spi_bcm_write(0xab);
		AT45_deselect();
		deep_down = true;
//		nrf_delay_us(35);														// Remember to wait 35us before reusing the device.          
	}

	else if(deep_down_onoff == true) { 											// Go to deep power down
		AT45_busy();
		AT45_select();
		spi_bcm_write(0xb9);
		AT45_deselect();
		deep_down = false;
	} 
	else {
																				// Do nothing   
	}       
}
 
bool AT45_is_it_awake(void) {
	return deep_down;
}
 
//=============================================================================
//		Private functions
//=============================================================================

int AT45_initialize(void) {
	int id = 0;
	int status = 0;

	if (AT45_test_pagesize_528==0) pagesize=528;								// Do a quick check on manufacturer pagesize configuration (Default should be 528)
	else pagesize=512;

	id = AT45_id();
	status = AT45_status();

	if ((id & 0x1f) == 0x3) {													// 2Mbits 
		devicesize = 262144; 													// Size in bytes
		pages = 1024;       													// Number of pages
		blocks = 128;        													// Number of blocks
		if (status & 0x1)	pagesize = 256;
		else				pagesize = 264;
	}

	else if ((id & 0x1f) == 0x4) {												// 4Mbits 
		devicesize = 524288;
		pages = 2048;
		blocks = 256;        
		if (status & 0x1)	pagesize = 256;
		else				pagesize = 264; 
	}

	else if ((id & 0x1f) == 0x5) {												// 8Mbits 
		devicesize = 1048576;
		pages = 4096;
		blocks = 512;        
		if (status & 0x1)	pagesize = 256;
		else				pagesize = 264;
	}

	else if ((id & 0x1f) == 0x6) {												// 16Mbits 
		devicesize = 2097152;
		pages = 4096;
		blocks = 512;        
		if (status & 0x1)	pagesize = 512;
		else				pagesize = 528;
	}

	else if ((id & 0x1f) == 0x7) {												// 32Mbits 
		devicesize = 4194304;
		pages = 8192;
		blocks = 1024;        
		if (status & 0x1)	pagesize = 512;
		else				pagesize = 528;
	}

	else if ((id & 0x1f) == 0x8) {												// 64Mbits 
		devicesize = 8388608;
		pages = 8192;
		blocks = 1024;
		if (status & 0x1)	pagesize = 1024;
		else				pagesize = 1056;    
	}

	else {
		devicesize = -1;
		pages = -1;
		pagesize = -1;
		blocks = -1;
		return -1;																// Return RS
	}
	return 0;
}
 
void AT45_select(void) {
//	AT45_ncs = 0;
	AT45DB_NCS_LOW;
}

void AT45_deselect(void) {
//	AT45_ncs = 1;
	AT45DB_NCS_HIGH;
}
 
void AT45_busy(void) {
	volatile int iambusy = 1;  
	while (iambusy) {
		if ( AT45_status() & 0x80 ) {iambusy = 0;}								// If bit 7 is set, we can proceed
	}
}


void AT45_sramwrite(int buffer, int address, int data) {						// Write to an SRAM buffer
																				// Note: We create buffer and page addresses in _sram and _flash
 	int cmd = 0;
	int baddr = 0;

	baddr = AT45_getbaddr(address);

	AT45_busy();

	AT45_select();

	if (buffer == 1)	{cmd = 0x84;}											// SRAM Buffer#1 Write
	else 				{cmd = 0x87;}											// SRAM Buffer#2 Write

	spi_bcm_write(cmd);
	AT45_sendaddr (baddr);
	spi_bcm_write (data);  

	AT45_deselect();            
}
 


int AT45_sramread(int buffer, int address) {									// Read from an SRAM buffer
																				// Note : We create buffer and page addresses in _sram and _flash
	int cmd = 0;
	int baddr = 0;
	int bufdata = 0;

	baddr = AT45_getbaddr(address);

	AT45_select();

	if(buffer == 1)		{cmd = 0xd4;}											// SRAM Buffer#1 High Frequency Read 0xd4  -->  Buff#1 Alternative Low Frequency Read use 0xd1
	else				{cmd = 0xd6;}											// SRAM Buffer#2 High Frequency Read 0xd6  -->  Buff#2 Alternative Low Frequency Read use 0xd3

	spi_bcm_write(cmd);
	AT45_sendaddr (baddr);	
	spi_bcm_write (0x00);   													// Don't care byte
	bufdata = spi_bcm_write (0x00);  

	AT45_deselect();            

	return (bufdata);
}
 

void AT45_flashwrite (int buffer, int address) {								// Write and SRAM buffer to main memory
 
	int cmd = 0;
	int paddr = AT45_getpaddr(address);

	AT45_busy();																// Check flash is not busy 

	AT45_select();  

	if (buffer == 1)	{cmd = 0x83;}											// Write SRAM Buffer#1 to Main Memory Page Program with Built-In Erase
	else 		        {cmd = 0x86;}											// Write SRAM Buffer#2 to Main Memory Page Program with Built-In Erase
	  
	spi_bcm_write (cmd);  
	AT45_sendaddr (paddr);  
	AT45_deselect();     

	AT45_busy();																// Check flash is not busy 
}
 

void AT45_flashread (int buffer, int address) {									// Read from Flash memory into SRAM buffer

	int cmd = 0;
	int paddr = AT45_getpaddr(address);											// Calculate page address

	AT45_busy();  																// Wait for flash not busy
	AT45_select();
	  
	if (buffer == 1)	{cmd = 0x53;}											// Main Memory Page to Buffer 1 Transfer
	else 				{cmd = 0x55;}											// Main Memory Page to Buffer 2 Transfer

	spi_bcm_write (cmd);  
	AT45_sendaddr (paddr);
	AT45_deselect();            
}
 

int AT45_memread (int address) {												// Read directly from main memory
 
	int data = 0;        
	int addr;

	addr = AT45_getpaddr(address) | AT45_getbaddr(address);						// Define addr from recovered (Page Address) and (Buffer Address)

	AT45_busy();

	AT45_select();

	spi_bcm_write (0xd2);														// Direct main memory page read command
	AT45_sendaddr (addr);

	spi_bcm_write (0x00);														// 4 don't care bytes
	spi_bcm_write (0x00);  
	spi_bcm_write (0x00);  
	spi_bcm_write (0x00);

	data = spi_bcm_write (0x00);  												// Clock the data

	AT45_busy();   
	AT45_deselect();            

	return data;
}
 
// Work out the page address
// If we have a 2^N page size, it is just the top N bits
// If we have non-2^N, we use the shifted address
int AT45_getpaddr(int address) {												// Get the Page Address
 
	int paddr;

	if (pagesize == 256)		{paddr = address & 0xffffff00;}   
	else if (pagesize == 264)	{paddr = (address << 1) & 0xfffffe00;}
	else if (pagesize == 512)	{paddr = address & 0xfffffe00;}
	else if (pagesize == 528 )	{paddr = (address << 1) & 0xfffffc00;}
	else if (pagesize == 1024)	{paddr = address & 0xfffffc00;}
	else if (pagesize == 1056 )	{paddr = (address << 1) & 0xfffff800;}
	else {	
	paddr = -1;}
	return (paddr);
}
 
// Clean the buffer address. This is the 8/9/10 LSBs
int AT45_getbaddr(int address) {												// Get the Buffer Address
 
	int baddr;

	if ((pagesize == 256) || (pagesize == 264 ))		{baddr = address & 0xff;}
	else if ((pagesize == 512) || (pagesize == 528 ))	{baddr = address & 0x1ff;}
	else if ((pagesize == 1024) || (pagesize == 1056 ))	{baddr = address & 0x3ff;}
	else {
	baddr = -1;}
	return (baddr);
}
 
// Sends the three lest significant bytes of the supplied address
void AT45_sendaddr (int sendadd) {
    
	spi_bcm_write(sendadd >> 16);
	spi_bcm_write(sendadd >> 8);
	spi_bcm_write(sendadd);      
}


void AT45_disable_IO_Pins(void) {										// Disable the AT45 nCS SLCK MISO MOSI Lines
	
	NRF_GPIO->PIN_CNF[SPI_BCM_SS_PIN] = 							
	(GPIO_PIN_CNF_SENSE_Disabled		<< GPIO_PIN_CNF_SENSE_Pos)	
	| (GPIO_PIN_CNF_DRIVE_S0S1			<< GPIO_PIN_CNF_DRIVE_Pos)	
	| (GPIO_PIN_CNF_PULL_Disabled		<< GPIO_PIN_CNF_PULL_Pos)	
	| (GPIO_PIN_CNF_INPUT_Disconnect	<< GPIO_PIN_CNF_INPUT_Pos)	
	| (GPIO_PIN_CNF_DIR_Input			<< GPIO_PIN_CNF_DIR_Pos);		// Configure The SPI-CS  Line as InputIO Pin as Input	

	NRF_GPIO->PIN_CNF[SPI_BCM_SCK_PIN] =								
	  (GPIO_PIN_CNF_SENSE_Disabled		<< GPIO_PIN_CNF_SENSE_Pos)		
	| (GPIO_PIN_CNF_DRIVE_S0S1			<< GPIO_PIN_CNF_DRIVE_Pos)	
	| (GPIO_PIN_CNF_PULL_Disabled		<< GPIO_PIN_CNF_PULL_Pos)	
	| (GPIO_PIN_CNF_INPUT_Disconnect	<< GPIO_PIN_CNF_INPUT_Pos)	
	| (GPIO_PIN_CNF_DIR_Input			<< GPIO_PIN_CNF_DIR_Pos);		// Configure SPI1 Slave CLK Pin as Input


	NRF_GPIO->PIN_CNF[SPI_BCM_MISO_PIN] = 							
	  (GPIO_PIN_CNF_SENSE_Disabled		<< GPIO_PIN_CNF_SENSE_Pos)	
	| (GPIO_PIN_CNF_DRIVE_S0S1			<< GPIO_PIN_CNF_DRIVE_Pos)	
	| (GPIO_PIN_CNF_PULL_Disabled		<< GPIO_PIN_CNF_PULL_Pos)	
	| (GPIO_PIN_CNF_INPUT_Disconnect	<< GPIO_PIN_CNF_INPUT_Pos)	
	| (GPIO_PIN_CNF_DIR_Input			<< GPIO_PIN_CNF_DIR_Pos);		// Configure MISO IO Pin as Input

	NRF_GPIO->PIN_CNF[SPI_BCM_MOSI_PIN] = 							
	  (GPIO_PIN_CNF_SENSE_Disabled 		<< GPIO_PIN_CNF_SENSE_Pos)		
	| (GPIO_PIN_CNF_DRIVE_S0S1			<< GPIO_PIN_CNF_DRIVE_Pos)		
	| (GPIO_PIN_CNF_PULL_Disabled    	<< GPIO_PIN_CNF_PULL_Pos)	
	| (GPIO_PIN_CNF_INPUT_Disconnect	<< GPIO_PIN_CNF_INPUT_Pos)	
	| (GPIO_PIN_CNF_DIR_Input			<< GPIO_PIN_CNF_DIR_Pos);		// Configure The MOSI IO Pin as Input
  
	  
	SFLASH_GPS_flag=999;												// Bus released ---> IO Pins No longer defined --> Now Safe to Assert AT45 SFLASH Read
}

void AT45_Congigure_IO_Pins(void) {										// Configure the AT45 nCS SLCK MISO MOSI Lines
	
	NRF_GPIO->PIN_CNF[SPI_BCM_SS_PIN] = 							
	  (GPIO_PIN_CNF_SENSE_Disabled		<< GPIO_PIN_CNF_SENSE_Pos)	
	| (GPIO_PIN_CNF_DRIVE_S0S1			<< GPIO_PIN_CNF_DRIVE_Pos)	
	| (GPIO_PIN_CNF_PULL_Pullup			<< GPIO_PIN_CNF_PULL_Pos)	
	| (GPIO_PIN_CNF_INPUT_Disconnect	<< GPIO_PIN_CNF_INPUT_Pos)	
	| (GPIO_PIN_CNF_DIR_Output			<< GPIO_PIN_CNF_DIR_Pos);		// Configure The SPI-CS  Line as InputIO Pin as Output

	NRF_GPIO->PIN_CNF[SPI_BCM_SCK_PIN] =								
	  (GPIO_PIN_CNF_SENSE_Disabled		<< GPIO_PIN_CNF_SENSE_Pos)		
	| (GPIO_PIN_CNF_DRIVE_S0S1			<< GPIO_PIN_CNF_DRIVE_Pos)	
	| (GPIO_PIN_CNF_PULL_Pulldown		<< GPIO_PIN_CNF_PULL_Pos)	
	| (GPIO_PIN_CNF_INPUT_Disconnect	<< GPIO_PIN_CNF_INPUT_Pos)	
	| (GPIO_PIN_CNF_DIR_Output			<< GPIO_PIN_CNF_DIR_Pos);		// Configure SPI1 Slave CLKIO Pin as Output


	NRF_GPIO->PIN_CNF[SPI_BCM_MISO_PIN] = 								// MASTER IN
	  (GPIO_PIN_CNF_SENSE_Disabled		<< GPIO_PIN_CNF_SENSE_Pos)	
	| (GPIO_PIN_CNF_DRIVE_S0S1			<< GPIO_PIN_CNF_DRIVE_Pos)	
	| (GPIO_PIN_CNF_PULL_Disabled		<< GPIO_PIN_CNF_PULL_Pos)	
	| (GPIO_PIN_CNF_INPUT_Connect		<< GPIO_PIN_CNF_INPUT_Pos)	
	| (GPIO_PIN_CNF_DIR_Input			<< GPIO_PIN_CNF_DIR_Pos);		// Configure MISO IO Pin as Input



	NRF_GPIO->PIN_CNF[SPI_BCM_MOSI_PIN] = 								// MASTER OUT
	  (GPIO_PIN_CNF_SENSE_Disabled		<< GPIO_PIN_CNF_SENSE_Pos)		
	| (GPIO_PIN_CNF_DRIVE_S0S1			<< GPIO_PIN_CNF_DRIVE_Pos)		
	| (GPIO_PIN_CNF_PULL_Disabled		<< GPIO_PIN_CNF_PULL_Pos)	
	| (GPIO_PIN_CNF_INPUT_Disconnect	<< GPIO_PIN_CNF_INPUT_Pos)	
	| (GPIO_PIN_CNF_DIR_Output			<< GPIO_PIN_CNF_DIR_Pos);		// Configure The MOSI IO Pin as Output
  
	SFLASH_GPS_flag=2;													// AT45 has the Bus
}


//	#define AT45DB_NCS_HIGH		NRF_GPIO->OUTSET = (1UL << SPI_BCM_SS_PIN)			// Macro to Assert   AT45DB161 NCS HIGH (OFF)
//	#define AT45DB_NCS_LOW		NRF_GPIO->OUTCLR = (1UL << SPI_BCM_SS_PIN)			// Macro to Deassert AT45DB161 NCS LOW  (ON)

//	#define AT45DB_SCLK_HIGH	NRF_GPIO->OUTSET = (1UL << SPI_BCM_SCK_PIN)			// Macro to Deassert AT45DB161 SCLK HIGH
//	#define AT45DB_SCLK_LOW		NRF_GPIO->OUTCLR = (1UL << SPI_BCM_SCK_PIN)			// Macro to Deassert AT45DB161 NCS LOW

//	#define AT45DB_MOSI_HIGH	NRF_GPIO->OUTSET = (1UL << SPI_BCM_MOSI_PIN)		// Macro to Deassert AT45DB161 MOSI PIN HIGH
//	#define AT45DB_MOSI_LOW		NRF_GPIO->OUTCLR = (1UL << SPI_BCM_MOSI_PIN)		// Macro to Deassert AT45DB161 MOSI PIN LOW

//	#define AT45_MISO_READ() 	((NRF_GPIO->IN >> SPI_BCM_MISO_PIN) & 0x1UL)		//  Reads current state of MISO Pin 

int spi_bcm_write(int databyte){										// ToDo   SPI MASTER BIT CRUNCH Routine (Output a byte and get one back)
	uint8_t i=0;														// MSB First 76543210
	uint8_t dataout=0;
	
	int bit_in=0;

	AT45DB_NCS_LOW;														// Should Already be LOW upon Entry

	nrf_delay_us(5);

	int mask=0x80;
	for(i=0;i<8;i++){
//		int temptest=(databyte & mask);
		if((databyte & mask)==0)	bit_in = BCM_Out_Low();
		else 						bit_in = BCM_Out_High();
		
		dataout = dataout << 1;											// Shift Recovered byte to the left and add in new bit
		if(bit_in==1) 	dataout = dataout | 0x01;
		else			dataout = dataout & 0xFE;
		
		mask = mask >> 1;												// Shift mask right 1 bit  1000 0000 --> 0000 0001
}
	return dataout;
}


int BCM_Out_Low(void) {													// Strobe out a LOW bit and receive a bit in return
		
	AT45DB_MOSI_LOW;													// Data Out LOW Setup
		
	AT45DB_SCLK_HIGH;													// SCLK High_to_Low Primes new data from AT45 for 
	nrf_delay_us(10);													// Add a short delay

	uint32_t MISO_bit=((NRF_GPIO->IN >> SPI_BCM_MISO_PIN) & 0x1UL);		// Read MISO Input Pin

	AT45DB_SCLK_LOW;													// SCLK Edge Positive for AT45 to Latch Data Out bit
	nrf_delay_us(5);													// Add a short delay

	return MISO_bit;
}


int BCM_Out_High(void) {												// Strobe out a HIGH bit and receive a bit in return

	AT45DB_MOSI_HIGH;													// Setup Data Out bit
	
	AT45DB_SCLK_HIGH;													// SCLK High_to_Low Primes new data from AT45 for 
	nrf_delay_us(10);													// Add a short delay
	
	uint32_t MISO_bit=((NRF_GPIO->IN>> SPI_BCM_MISO_PIN) & 0x1UL);		// Read MISO Input Pin

	AT45DB_SCLK_LOW;													// SCLK Edge Positive for AT45 to Latch Data Out bit
	nrf_delay_us(5);													// Add a short delay

	return MISO_bit;
}

