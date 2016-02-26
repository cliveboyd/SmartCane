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


 /*
 MANUFACTURERS NOTES... Clarification from Ambigueous and Flakey Application Note
 
 Note...Current GPS StartUp Sequence Enables GPS SPI Mode. ---> Yeaaaaa!!!!!
 
 
 GSD4e-Host-Platform-Interfaces-Application-Note-CS-211776-AN-3.pdf
 
 In general the A2035 supports SSPI (Slave SPI).
 NOTE::: The A2035-H is an SPI SLAVE (The HOST is the MASTER)
- Your µP (the host) is the master.
- The Host SPI_CLK (is an output)
- The Host SPI_CS  (is an output)


SPI-CS  on the A2035 ..	Pin 8,	input,	1V8 (3V3 tolerant)  (Actually NCS as Asserted LOW)
SPI-TX  on the A2035 ..	Pin 20,	output,	3V3
SPI-RX  on the A2035 ..	Pin 21,	Input,	1V8 (3V3 tolerant)
SPI-CLK on the A2035 ..	Pin 7,	Input,	1V8 (3V3 tolerant)

Leave the A2305H nRST open (triState) upon Power UP NOT pulled high to 3V3 or 1V8.
Leave the A2305H nCS  open (triState) upon Power UP NOT pulled high to 3V3 or 1V8.
 
 */
 
/*
Note	The A2035-H SPI loads a fifo at the rate of 100bytes/second
		Need to reading fifo faster then the fill rate, however, pading bytes of 
		0xB4 0xA7 are automatically added.
		
		NOTE... The Host needs to send B4A4 padding bytes into the A2035H NOT Zero's
	
		Upon B4A7 data blocks a delay is planned to be added to allow fifo to fill.
		Probably a couple of lines.

		A NEMA parser NEMAParser.C will be used to extract formated data.
	
		A2035-H FIFO Buffer ==> 1024bytes refreshed at 100 bytes per sec

		A2035-H SPI Mode ---> Configuration SPI Mode 1 (CPOL:0,CPHA:1) --> MSB first

*/


/* 
NOTE:	The A2035H and the AT45DB161 share the SPi Clk MISO and MOSI signal lines
		
		To Place the A2035H in SSPI Mode ---> Upon 3V3GPS PowerUp
		
		GPIO7 = High (External Pull Up ---> None) Configured internally
		GPIO6 = Low  (External Pull Up ---> None) Configured internally
		
		A2035H.nRST ---> Open Circuit (Tristate at powerup)

*/



//#ifndef A2035H
//#define A2035H


/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/
#include <stdint.h>  										// for uint32_t etc.
#include <stdbool.h>

#include "A2035H.h"

#include "nrf_gpio.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "spi_master.h"										// AT45DB161 SFLASH   requires MASTER SPI0 Driver Defined under A2035.h

															// A2035-H GPS Module Requires MASTER SPI1
#include "NEMAParser.h"

#include "nrf_delay.h"
#include "global.h"
#include "Communication.h"									// SPI Master-0 Definitions

#define A2035H_ON_OFF_PIN		(13U)				/**<  GPS ON-OFF Toogle Control GPIO pin number. */
#define A2035H_NRST_PIN			(10U)				/**<  GPS nRESET GPIO pin number. */
#define A2035H_NEN_PIN			(04U)				/**<  GPS 3V3 Linear regulator nEnable GPIO pin number. */
#define A2035H_INT_PIN			(14U)				/**<  GPS int GPIO pin number. */

#define A2035H_MOSI_PIN			(17U)				/**<  GPS int GPIO pin number. */
#define A2035H_MISO_PIN			(20U)				/**<  GPS int GPIO pin number. */
#define A2035H_SCLK_PIN			(19U)				/**<  GPS int GPIO pin number. */
#define A2035H_NCS_PIN 			(15U)     			/**< A2035H SPI-M0-NCS GPIO pin number. */

#define AT45_ALT_NCS			(18U)				/**<  Alternate AT45 SFLASH Definition as double security to ensure chip is tristate when A2035 is enabled */



#define SPI_TO_USE SPI_MASTER_0

#define TX_RX_MSG_LENGTH   3

static uint8_t m_rx_data_spi[TX_RX_MSG_LENGTH]; 			/**< SPI RX data buffer. */

static volatile bool m_transfer_completed = true;

uint16_t	A2035H_RawDataInPointer=0;
uint16_t	A2035H_RawDataOutPointer=0;

uint16_t	SPI_TX_Count=0;

#define		SPI_DUMMY_BYTE0   0xB4
#define		SPI_DUMMY_BYTE1   0xA7
 
#define		A2035H_NCS_HIGH     NRF_GPIO->OUTSET = (1UL << A2035H_NCS_PIN)		// USED to drive A2035 SPI-CS during SPI MASTER tests
#define		A2035H_NCS_LOW      NRF_GPIO->OUTCLR = (1UL << A2035H_NCS_PIN)		// USED to drive A2035 SPI-CS during SPI MASTER tests

#define		TX_BUF_SIZE   4														/**< SPI TX buffer size. */      
#define		RX_BUF_SIZE   TX_BUF_SIZE											/**< SPI RX buffer size. */    

char 		A2035H_RawData[128];												/** Character Buffer for A2035-H Raw Data */

char 		A2035H_TxData[4];
char 		A2035H_RxData[4];


 
/*******************************************************************************
 * @brief Sets the Reset bit of the A2035H.
 *
 * @return None.
*******************************************************************************/
void A2035H_Reset(void) {
//    A2035H_SetRegisterValue(A2035H_REG_CMD | A2035H_RESET);
}


/*******************************************************************************
 * @brief Writes the value to a register.
 *
 * @param -  regValue - The value 16-bit to write to the register.
 *
 * @return  None.    
*******************************************************************************/
void A2035H_SetRegisterValue(unsigned short regValue) {		// 16bit
	unsigned char data[2] = {0x00, 0x00};
	
	data[0] = (unsigned char)((regValue & 0xFF00) >> 8);
	data[1] = (unsigned char)((regValue & 0x00FF) >> 0);
	A2035H_NCS_LOW;
	SPI_write(SPI_TO_USE, data, m_rx_data_spi ,2);
	A2035H_NCS_HIGH;
}


/*******************************************************************************
 * @brief	Send a dummy zero byte and Read a byte from SPI Port.
 *
 * @return	m_rx_data_spi[0]   
*******************************************************************************/
char A2035H_SPI_ReadByte(void) {						// 1x8bit
	
	unsigned char data[2] = {0xB4, 0xA7};				// Pad Characters
	SPI_TX_Count++;
	
	if (SPI_TX_Count>1) {
		SPI_TX_Count=0;
		data[0]=0xB4;
	}
	else data[0]=0xA7;
	
	A2035H_NCS_LOW;
	SPI_write(SPI_TO_USE, data , m_rx_data_spi ,1);
	A2035H_NCS_HIGH;

	return (char) m_rx_data_spi[0];
}

/*******************************************************************************
 * @brief	Send a dummy zero byte and Read a byte from SPI Port.
 *
 *	@Note	A2035_NCS Controlled outside of this routine to facilitate burst transfer
 *
 * @return	m_rx_data_spi[0]   
*******************************************************************************/
char A2035H_SPI_ReadByte_NOCS(void) {					// 1x8bit
	
	unsigned char TxData[2] = {0xB4, 0xA7};				// Pad Characters
	SPI_TX_Count++;
	
	if (SPI_TX_Count>1) {
		SPI_TX_Count=0;
		TxData[0]=0xB4;
	}
	else TxData[0]=0xA7;
	
	SPI_write(SPI_TO_USE, TxData , m_rx_data_spi ,1);

	return (char) m_rx_data_spi[0];
}


/*******************************************************************************
 * @brief	Send a dummy zero byte and Read a byte from SPI Port.
 *
 * @return	m_rx_data_spi[0]   
*******************************************************************************/
void A2035H_SPI_writeReg(uint16_t regValue) {			// 2x8bit
	
	unsigned char data[2] = {0x00, 0x00};	
	
	data[0] = (unsigned char)((regValue & 0xFF00) >> 8);		
	data[1] = (unsigned char)((regValue & 0x00FF) >> 0);

	A2035H_NCS_LOW;
	SPI_write(SPI_TO_USE, data , m_rx_data_spi ,2);
	A2035H_NCS_HIGH;
}


/******************************************************************************
 * @brief	Read and load A2035 32 SPI Port0 data bytes into A2035_RawData Buffer.
 *
 * @return	32 bytes indexed into A2035_RawData[128] by A2035_RawDataInPointer 
*******************************************************************************/
void A2035H_SPI_ReadAndLoad32Packets() {
	
	A2035H_NCS_LOW;
	
	for(int i=0;i<32-1;i++){											// Load 32 bytes into 128 byte circular char array
		A2035H_RawDataInPointer++;
		if (A2035H_RawDataInPointer==128) A2035H_RawDataInPointer=0;
		
		A2035H_RawData[A2035H_RawDataInPointer] = A2035H_SPI_ReadByte_NOCS();
	}
	
	A2035H_NCS_HIGH;
}


/******************************************************************************
 * @brief	Routine called from main() system interupt timer.
 *
 * @return	void ---> Loads data to GPS NEMAParser
*******************************************************************************/
void A2035H_Sheduled_SPI_Read() {
	
	A2035H_SPI_ReadAndLoad32Packets();
	
	while (A2035H_RawDataOutPointer!=A2035H_RawDataInPointer) {			// Loop until Out pointer catches up to in or until temp buffer full
		char temp[33];
		uint8_t count=0;
		uint8_t ParseCount=0;
		A2035H_RawDataOutPointer++;
		if (A2035H_RawDataOutPointer == 128) A2035H_RawDataOutPointer=0;
		while (count<32) {;
			count++;
			if (A2035H_RawData[A2035H_RawDataOutPointer]!= (char) 0xB4 || A2035H_RawData[A2035H_RawDataOutPointer]!= (char) 0xA7) {		// Skip pad characters
				ParseCount++;
				temp[ParseCount]=A2035H_RawData[A2035H_RawDataOutPointer];
			}
		}
		if (ParseCount>0) {
			Parse(temp, ParseCount);									// Load recursive data to NEMAParser
		}
	}
}


typedef enum {
	Pull_up,
	Pull_down,
	Pull_disable
} PullUpDown_t;


static __INLINE void pullupdown_gpio_cfg_output(uint32_t pin_number, PullUpDown_t pull ) {
	unsigned char pullset;
	switch(pull)
	{
		case Pull_up:
			pullset = (GPIO_PIN_CNF_PULL_Pullup		<< GPIO_PIN_CNF_PULL_Pos);
			break;
		
		case Pull_down:
			pullset = (GPIO_PIN_CNF_PULL_Pulldown	<< GPIO_PIN_CNF_PULL_Pos);
			break;
		
		case Pull_disable:
			pullset = (GPIO_PIN_CNF_PULL_Disabled	<< GPIO_PIN_CNF_PULL_Pos);
			break;
	}

    /*lint -e{845} 														// A zero has been given as right argument to operator '|'" */
    NRF_GPIO->PIN_CNF[pin_number] = (GPIO_PIN_CNF_SENSE_Disabled <<  GPIO_PIN_CNF_SENSE_Pos)
																 |  (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
																 |   pullset
																 |  (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
																 |  (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
}


void initA2035Hx(void) {

}

void A2035H_Toggle_ONOFF(void){
//	ON_OFF Strobe Low-High-low
	nrf_gpio_pin_clear(A2035H_ON_OFF_PIN);								// Assert ON_OFF Pin Low		
	nrf_delay_us(50000); 												// Delay 50ms
	
	nrf_gpio_pin_set(A2035H_ON_OFF_PIN);								// Assert ON_OFF Pin High
	nrf_delay_us(220000); 												// delay minimum 200ms ---> allow 220msec
	
	nrf_gpio_pin_clear(A2035H_ON_OFF_PIN);								// Assert ON_OFF Pin Low
	nrf_delay_us(50000); 												// Delay 50ms 
}

void A2035H_RESET_ON(void) {
	nrf_gpio_pin_clear(A2035H_NRST_PIN);								// A2035 Reset ---> Pull HIGH (external 33k 3V3 Pull-up removed. Rquired for SPI Mode Powerup)
	nrf_delay_us(50000); 												// Delay 50ms 
}

void A2035H_RESET_OFF(void) {
	nrf_gpio_pin_set(A2035H_NRST_PIN);									// A2035 Reset ---> Pull HIGH Also Pulled High via 33k
	nrf_delay_us(50000); 												// Delay 50ms 
}

void A2035H_POWER_OFF(void) {											// WARNING Abrust Power Down may corrupt FLASH
	A2035H_Toggle_ONOFF();
	nrf_delay_us(50000); 												// Delay 50ms	
	nrf_gpio_pin_clear(A2035H_NRST_PIN);								// A2035 Reset ---> Pull HIGH Also Pulled High via 33k
	nrf_delay_us(50000); 												// Delay 50ms	
	nrf_gpio_pin_clear(A2035H_NEN_PIN);									// A2035 NEN 3V3GPS OFF ---> ASSERT LOW
	nrf_delay_us(50000); 												// Delay 50ms
}


void A2035H_POWER_ON(void) {											// WARNING Abrust Power Down may corrupt FLASH
	pullupdown_gpio_cfg_output(A2035H_NEN_PIN, Pull_up);
	nrf_gpio_pin_set(A2035H_NEN_PIN);									// A2035 NEN 3V3GPS OFF ---> ASSERT LOW
}

void initA2035H(void) {
	
	A2035H_POWER_OFF();													// POWER OFF Strobes ON-OFF First then After Delay Kills Power
	A2035H_tristate_SPI_Bus_Pins();	
	
	nrf_delay_us(500000); 												// Wait for rails to settle ---> Delay 500ms
	
	A2035H_POWER_ON();													// Initiates spi_init after Power ON
	
	nrf_delay_us(1000000); 												// Wait 1 second
	A2035H_Configure_Non_SPI_IOPins();									// Gonfigures Inputs and Sets Output Defaults --> Reasserts 3vV3GPS Power, Asserts Reset HIGH, Sets ON-OFF to LOW
	
	SPI_Init();															// Restart the SPI Master ---> Required only after A2035 Powerup Note: A2035 Initialised First before AT45
	
	nrf_delay_us(500000); 												// Delay 500msec then initiate Off-On-Off
	A2035H_Toggle_ONOFF();
}

void A2035H_Configure_Non_SPI_IOPins(void) {
	pullupdown_gpio_cfg_output(A2035H_ON_OFF_PIN,	Pull_down);	
	pullupdown_gpio_cfg_output(A2035H_NRST_PIN,		Pull_up);	
	pullupdown_gpio_cfg_output(A2035H_NEN_PIN,		Pull_up);
	
	nrf_gpio_pin_clear(A2035H_ON_OFF_PIN);								// Assert Low  ---> ON-OFF Default LOW
	nrf_gpio_pin_set(A2035H_NRST_PIN);									// Assert High ---> RESET ALLOW NORMAL OPERATION
	nrf_gpio_pin_set(A2035H_NEN_PIN);									// Assert High ---> 3V3GPS ON
	
	
	NRF_GPIO->PIN_CNF[A2035H_INT_PIN] = 								
	(GPIO_PIN_CNF_SENSE_Disabled		<< GPIO_PIN_CNF_SENSE_Pos)	
	| (GPIO_PIN_CNF_DRIVE_S0S1			<< GPIO_PIN_CNF_DRIVE_Pos)	
	| (GPIO_PIN_CNF_PULL_Disabled		<< GPIO_PIN_CNF_PULL_Pos)	
	| (GPIO_PIN_CNF_INPUT_Disconnect	<< GPIO_PIN_CNF_INPUT_Pos)	
	| (GPIO_PIN_CNF_DIR_Input			<< GPIO_PIN_CNF_DIR_Pos);		// Configure SPI Pin as Input
}

void A2035H_tristate_SPI_Bus_Pins(void) {

	NRF_GPIO->PIN_CNF[A2035H_NCS_PIN] = 								
	(GPIO_PIN_CNF_SENSE_Disabled		<< GPIO_PIN_CNF_SENSE_Pos)	
	| (GPIO_PIN_CNF_DRIVE_S0S1			<< GPIO_PIN_CNF_DRIVE_Pos)	
	| (GPIO_PIN_CNF_PULL_Disabled		<< GPIO_PIN_CNF_PULL_Pos)	
	| (GPIO_PIN_CNF_INPUT_Disconnect	<< GPIO_PIN_CNF_INPUT_Pos)	
	| (GPIO_PIN_CNF_DIR_Input			<< GPIO_PIN_CNF_DIR_Pos);		// Configure SPI Pin as Input

	NRF_GPIO->PIN_CNF[A2035H_SCLK_PIN] =								
	(GPIO_PIN_CNF_SENSE_Disabled		<< GPIO_PIN_CNF_SENSE_Pos)		
	| (GPIO_PIN_CNF_DRIVE_S0S1			<< GPIO_PIN_CNF_DRIVE_Pos)	
	| (GPIO_PIN_CNF_PULL_Disabled		<< GPIO_PIN_CNF_PULL_Pos)	
	| (GPIO_PIN_CNF_INPUT_Disconnect	<< GPIO_PIN_CNF_INPUT_Pos)	
	| (GPIO_PIN_CNF_DIR_Input			<< GPIO_PIN_CNF_DIR_Pos);		// Configure SPI Pin as Input


	NRF_GPIO->PIN_CNF[A2035H_MISO_PIN] = 							
	(GPIO_PIN_CNF_SENSE_Disabled		<< GPIO_PIN_CNF_SENSE_Pos)	
	| (GPIO_PIN_CNF_DRIVE_S0S1			<< GPIO_PIN_CNF_DRIVE_Pos)	
	| (GPIO_PIN_CNF_PULL_Disabled		<< GPIO_PIN_CNF_PULL_Pos)	
	| (GPIO_PIN_CNF_INPUT_Disconnect	<< GPIO_PIN_CNF_INPUT_Pos)	
	| (GPIO_PIN_CNF_DIR_Input			<< GPIO_PIN_CNF_DIR_Pos);		// Configure SPI Pin as Input

	NRF_GPIO->PIN_CNF[A2035H_MOSI_PIN] = 							
	(GPIO_PIN_CNF_SENSE_Disabled		<< GPIO_PIN_CNF_SENSE_Pos)		
	| (GPIO_PIN_CNF_DRIVE_S0S1			<< GPIO_PIN_CNF_DRIVE_Pos)		
	| (GPIO_PIN_CNF_PULL_Disabled		<< GPIO_PIN_CNF_PULL_Pos)	
	| (GPIO_PIN_CNF_INPUT_Disconnect	<< GPIO_PIN_CNF_INPUT_Pos)	
	| (GPIO_PIN_CNF_DIR_Input			<< GPIO_PIN_CNF_DIR_Pos);		// Configure SPI Pin as Input
	
  	NRF_GPIO->PIN_CNF[A2035H_NRST_PIN] = 							
	(GPIO_PIN_CNF_SENSE_Disabled		<< GPIO_PIN_CNF_SENSE_Pos)		
	| (GPIO_PIN_CNF_DRIVE_S0S1			<< GPIO_PIN_CNF_DRIVE_Pos)		
	| (GPIO_PIN_CNF_PULL_Disabled		<< GPIO_PIN_CNF_PULL_Pos)	
	| (GPIO_PIN_CNF_INPUT_Disconnect	<< GPIO_PIN_CNF_INPUT_Pos)	
	| (GPIO_PIN_CNF_DIR_Input			<< GPIO_PIN_CNF_DIR_Pos);		// Configure A2035 NRST Pin as Tristate ---> Re Manufacturere SPI Config Requirements
	
}


