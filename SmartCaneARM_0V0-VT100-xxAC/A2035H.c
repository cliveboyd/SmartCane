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
Note	The A2035-H SPI loads a fifo at the rate of 100bytes/second
		Need to reading fifo faster then the fill rate, however, pading bytes of 
		0xB4 0xA7 are automatically added.
	
		Upon B4A7 data blocks a delay is planned to be added to allow fifo to fill.
		Probably a a couple of lines.

		A NEMA parser NEMAParser.C will be used to extract formated data.
	
		A2035-H FIFO Buffer ==> 1024bytes refreshed at 100 bytes per sec

		A2035-H SPI Mode ---> Configuration SPI Mode 1 (CPOL:0,CPHA:1) --> MSb first

*/


/* 
NOTE:	The A2035H and the AT45DB161 share the SPi Clk MISO and MOSI signal lines
		the A2035H needs to be configured as SPI-SLAVE while the AT45DB SFLASH requires SPI MASTER
		To Resolve this conflict SPI Lines will be configured as SPI-MASTER When the GPS A2035H is held in RESET
		
		After Reading the SPI-MASTER Serial Flash Memory the At45 will be placed in nCS and the SPI IO lines then 
		returned to SPI-SLAVE.  At this time the A2035 will be reactivated nRST=High and read.
	
		in order to facilitate the Dual SPI-SLAVE_SPI-MASTER function on Shared Io pins the A2035H and AT45DB drivers
		will be controlled via SPI-Master-Slave Controller

*/



//#ifndef A2035H
//#define A2035H


/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/
#include <stdint.h>  									// for uint32_t etc.
#include <stdbool.h>

#include "A2035H.h"

#include "nrf_gpio.h"
#include "app_error.h"
#include "app_util_platform.h"
//#include "spi_master.h"									// AT45DB161 SFLASH   requires MASTER SPI0 Driver Defined under A2035.h
#include "spi_slave.h"										// A2035-H GPS Module Requires SLAVE  SPI1
#include "NEMAParser.h"

#include "nrf_delay.h"
#include "global.h"

#define A2035H_ON_OFF_PIN_NUMBER		(13U)				/**<  GPS ON-OFF Toogle Control GPIO pin number. */
#define A2035H_NRST_PIN_NUMBER			(10U)				/**<  GPS nRESET GPIO pin number. */
#define A2035H_NEN_PIN_NUMBER			(04U)				/**<  GPS 3V3 Linear regulator nEnable GPIO pin number. */
#define A2035H_INT_PIN_NUMBER			(14U)				/**<  GPS int GPIO pin number. */

#define AT45_ALT_NCS					(18U)				/**<  ALternate AT45 SFLASH Definition as double sequrity to ensure chip is tristate when A2035 is enabled */

#define A2035H_CS_PIN 					(15U)     			/**< SPI1 SLAVE A2035H GPIO pin number. */
//#define AT45DB161E_CS_PINS			(18U)     			/**< SPI0 MASTER CS SFlash AT45DB161EGPIO pin number. */

#define SPI_TO_USE SPI_SLAVE_1

#define SPI_SLAVE_1_ENABLE		

#ifdef SPI_SLAVE_1_ENABLE									// SPI1 SLAVE ENABLE Definitions
	#define SPI1S_MISO_PIN				(17U)				/**< Definition Extracted for SPI Slave on SMARTCANE 1V0 >**/
	#define SPI1S_MOSI_PIN				(20U)				// Note for Slave mode.... A2035H ==> MASTER while the nRF51 HOST ==> SLAVE
	#define SPI1S_CSN_PIN				(15U)
	#define SPI1S_SCK_PIN      			(19U)
#endif														// SPI1 SLAVE ENABLE Definitions

#define SPIFREQ_TO_USE SPI_FREQUENCY_FREQUENCY_M1			// 1MHz ----> Maximum Speed suported is 6M8Hz on A2035-H

#define SPICPOL_TO_USE SPI_CONFIG_CPOL_ActiveLow
#define SPICPHA_TO_USE SPI_CONFIG_CPHA_Leading


#define TX_RX_MSG_LENGTH   4

static uint8_t m_rx_data_spi[TX_RX_MSG_LENGTH]; 			/**< SPI RX data buffer. */

static volatile bool m_transfer_completed = true;

char A2035_RawData[128];									/** Character Buffer for A2035-H Raw Data */
uint16_t	A2035_RawDataInPointer=0;
uint16_t	A2035_RawDataOutPointer=0;

#define SPI_DUMMY_BYTE   0x00
 
#define CS_HIGH     NRF_GPIO->OUTSET = (1UL << A2035H_CS_PIN)		// USED to drive A2035 SPI-CS during SPI MASTER tests
#define CS_LOW      NRF_GPIO->OUTCLR = (1UL << A2035H_CS_PIN)		// USED to drive A2035 SPI-CS during SPI MASTER tests
 

#define TX_BUF_SIZE   64u				/**< SPI TX buffer size. */      
#define RX_BUF_SIZE   TX_BUF_SIZE		/**< SPI RX buffer size. */      
#define DEF_CHARACTER 0xAAu				/**< SPI default character. Character clocked out in case of an ignored transaction. */      
#define ORC_CHARACTER 0x55u				/**< SPI over-read character. Character clocked out after an over-read of the transmit buffer. */      

static uint8_t m_tx_buf[TX_BUF_SIZE];	/**< SPI TX buffer. */      
static uint8_t m_rx_buf[RX_BUF_SIZE];	/**< SPI RX buffer. */          

/**@brief Function for initializing buffers.
 *
 * @param[in] p_tx_buf  Pointer to a transmit buffer.
 * @param[in] p_rx_buf  Pointer to a receive  buffer.
 * @param[in] len       Buffers length.
 */
static __INLINE void spi_slave_buffers_init(uint8_t * const p_tx_buf, uint8_t * const p_rx_buf, const uint16_t len) {
    uint16_t i;
    for (i = 0; i < len; i++)
    {
        p_tx_buf[i] = (uint8_t)('a' + i);
        p_rx_buf[i] = 0;
    }
}

/**@brief Function for checking if received data is valid.
 *
 * @param[in] p_rx_buf  Pointer to a receive  buffer.
 * @param[in] len       Buffers length.
 *
 * @retval true     Buffer contains expected data.
 * @retval false    Data in buffer are different than expected.
 */
static bool __INLINE spi_slave_buffer_check(uint8_t * const p_rx_buf, const uint16_t len) {
	uint16_t i;
	for (i = 0; i < len; i++) {
		if (p_rx_buf[i] != (uint8_t)('A' + i)) return false;
	}
	return true;
}

/**@brief Function for SPI slave event callback.
 *
 * Upon receiving an SPI transaction complete event, LED1 will blink and the buffers will be set.
 *
 * @param[in] event SPI slave driver event.
 */
static void spi_slave_event_handle(spi_slave_evt_t event) {
    uint32_t err_code=0;
    
    if (event.evt_type == SPI_SLAVE_XFER_DONE) {
//		err_code = bsp_indication_set(BSP_INDICATE_RCV_OK);
		APP_ERROR_CHECK(err_code);
//		Check if buffer size is the same as amount of received data.
		APP_ERROR_CHECK_BOOL(event.rx_amount == RX_BUF_SIZE);

//		Check if received data is valid.
		bool success = spi_slave_buffer_check(m_rx_buf, event.rx_amount);
		APP_ERROR_CHECK_BOOL(success);

//		Set buffers.
		err_code = spi_slave_buffers_set(m_tx_buf, m_rx_buf, sizeof(m_tx_buf), sizeof(m_rx_buf));
		APP_ERROR_CHECK(err_code);          
    }
}

/**@brief Function for initializing SPI slave.
 *
 *  Function configures a SPI slave and sets buffers.
 *
 * @retval NRF_SUCCESS  Initialization successful.
 */
uint32_t spi_slave_A2035H_init(void) {
    
	uint32_t           err_code;
    spi_slave_config_t spi_slave_config;
        
    err_code = spi_slave_evt_handler_register(spi_slave_event_handle);
    APP_ERROR_CHECK(err_code);    

    spi_slave_config.pin_miso         = SPI1S_MISO_PIN;
    spi_slave_config.pin_mosi         = SPI1S_MOSI_PIN;
    spi_slave_config.pin_sck          = SPI1S_SCK_PIN;
    spi_slave_config.pin_csn          = SPI1S_CSN_PIN;
    spi_slave_config.mode             = SPI_MODE_0;
    spi_slave_config.bit_order        = SPIM_LSB_FIRST;
    spi_slave_config.def_tx_character = DEF_CHARACTER;
    spi_slave_config.orc_tx_character = ORC_CHARACTER;
    
    err_code = spi_slave_init(&spi_slave_config);
    APP_ERROR_CHECK(err_code);
    
    //Initialize buffers.
    spi_slave_buffers_init(m_tx_buf, m_rx_buf, (uint16_t)TX_BUF_SIZE);
    
    //Set buffers.
    err_code = spi_slave_buffers_set(m_tx_buf, m_rx_buf, sizeof(m_tx_buf), sizeof(m_rx_buf));
    APP_ERROR_CHECK(err_code);            

    return NRF_SUCCESS;
}

  
/**@brief Function for initializing a SPI master driver.
 *
 * @param[in] spi_master_instance       An instance of SPI master module.
 * @param[in] spi_master_event_handler  An event handler for SPI master events.
 * @param[in] lsb                       Bits order LSB if true, MSB if false.
 */
//static void spi_master_init(spi_master_hw_instance_t   spi_master_instance,
//                            spi_master_event_handler_t spi_master_event_handler,
//                            const bool                 lsb)
//{
//    uint32_t err_code = NRF_SUCCESS;

//    // Configure SPI master.
//    spi_master_config_t spi_config = SPI_MASTER_INIT_DEFAULT;

//    switch (spi_master_instance) {
//        
//		#ifdef SPI_MASTER_0_ENABLE
//        case SPI_MASTER_0:
//        {
//            spi_config.SPI_Pin_SCK  = SPIM0_SCK_PIN;
//            spi_config.SPI_Pin_MISO = SPIM0_MISO_PIN;
//            spi_config.SPI_Pin_MOSI = SPIM0_MOSI_PIN;
//            spi_config.SPI_Pin_SS   = SPIM0_SS_PIN;
//        }
//        break;
//        #endif 										/* SPI_MASTER_0_ENABLE */
//		
//		case SPI_MASTER_1:
//			break;
//		
//        default:
//            break;
//    }
	
	// bit order
//    spi_config.SPI_CONFIG_ORDER = (lsb ? SPI_CONFIG_ORDER_LsbFirst : SPI_CONFIG_ORDER_MsbFirst);
//	spi_config.SPI_Freq = SPIFREQ_TO_USE;
//	spi_config.SPI_CONFIG_CPHA = SPICPHA_TO_USE;
//	spi_config.SPI_CONFIG_CPOL = SPICPOL_TO_USE;
//	spi_config.SPI_PriorityIRQ = APP_IRQ_PRIORITY_HIGH;
//	
//    err_code = spi_master_open(spi_master_instance, &spi_config);
//    
//	APP_ERROR_CHECK(err_code);

//    // Register event handler for SPI master.
//    spi_master_evt_handler_reg(spi_master_instance, spi_master_event_handler);
//}


/*******************************************************************************
 * @brief Initializes the SPI communication peripheral and resets the part.
 *
 * @return 1.
*******************************************************************************/
void A2035H_Init_SPIO(void) {
																				// Pull A2035H CS pin  
	 NRF_GPIO->PIN_CNF[A2035H_CS_PIN] = 							\
        (GPIO_PIN_CNF_SENSE_Disabled 	<< GPIO_PIN_CNF_SENSE_Pos)	\
      | (GPIO_PIN_CNF_DRIVE_H0D1     	<< GPIO_PIN_CNF_DRIVE_Pos)	\
      | (GPIO_PIN_CNF_PULL_Pullup    	<< GPIO_PIN_CNF_PULL_Pos)	\
      | (GPIO_PIN_CNF_INPUT_Disconnect  << GPIO_PIN_CNF_INPUT_Pos)	\
      | (GPIO_PIN_CNF_DIR_Output      	<< GPIO_PIN_CNF_DIR_Pos);	
		
		NRF_GPIO->DIRSET = (1UL << A2035H_CS_PIN);
		NRF_GPIO->OUTSET = (1UL << A2035H_CS_PIN); 
	
//	spi_master_init(SPI_TO_USE, NULL, false);  									// no handler, msb first

    A2035H_Reset();
}

/**@brief Function for sending and receiving data.
 *
 * @param[in]   spi_master_hw_instance  SPI master instance.
 * @param[in]   p_tx_data               A pointer to a buffer TX.
 * @param[out]  p_rx_data               A pointer to a buffer RX.
 * @param[in]   len                     A length of the data buffers.
 */
//static void SPI_Write(const spi_master_hw_instance_t spi_master_hw_instance,
//                          uint8_t * const                p_tx_data,
//                          uint8_t * const                p_rx_data,
//                          const uint16_t                 len)
//{
//		uint32_t err_code;
//		err_code = spi_master_send_recv(spi_master_hw_instance, p_tx_data, len, p_rx_data, len);    // Start transfer.
//		APP_ERROR_CHECK(err_code);
//}
 
 
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
//	unsigned char data[2] = {0x00, 0x00};
//	
//	data[0] = (unsigned char)((regValue & 0xFF00) >> 8);
//	data[1] = (unsigned char)((regValue & 0x00FF) >> 0);
//	CS_LOW;
////	SPI_Write(SPI_TO_USE, data, m_rx_data_spi ,2);
//	CS_HIGH;
}


/*******************************************************************************
 * @brief	Send a dummy zero byte and Read a byte from SPI Port.
 *
 * @return	m_rx_data_spi[0]   
*******************************************************************************/
char A2035H_SPI_ReadBytes(void) {		// 1x8bit
	
	CS_LOW;
//	SPI_Write(SPI_TO_USE, SPI_DUMMY_BYTE , m_rx_data_spi ,1);
	CS_HIGH;
	return (char) m_rx_data_spi[0];
}


/******************************************************************************
 * @brief	Read and load A2035 32 SPI Port0 data bytes into A2035_RawData Buffer.
 *
 * @return	32 bytes indexed into A2035_RawData[128] by A2035_RawDataInPointer 
*******************************************************************************/
void A2035H_SPI_ReadAndLoad32Packets() {
	
	for(int i=0;i<32-1;i++){											// Load 32 bytes into 128 byte circular char array
		A2035_RawDataInPointer++;
		if (A2035_RawDataInPointer==128) A2035_RawDataInPointer=0;
		A2035_RawData[A2035_RawDataInPointer]=A2035H_SPI_ReadBytes();
	}
}


/******************************************************************************
 * @brief	Routine called from main() system interupt timer.
 *
 * @return	void ---> Loads data to GPS NEMAParser
*******************************************************************************/
void A2035H_Sheduled_SPI_Read() {
	
	A2035H_SPI_ReadAndLoad32Packets();
	
	while (A2035_RawDataOutPointer!=A2035_RawDataInPointer) {			// Loop until Out pointer catches up to in or until temp buffer full
		char temp[33];
		uint8_t count=0;
		A2035_RawDataOutPointer++;
		if (A2035_RawDataOutPointer == 128) A2035_RawDataOutPointer=0;
		while (count<32) {;
			count++;
			if (A2035_RawData[A2035_RawDataOutPointer]!= (char) 0xB4 || A2035_RawData[A2035_RawDataOutPointer]!= (char) 0xA7) {		// Skip pad characters
				temp[count]=A2035_RawData[A2035_RawDataOutPointer];
			}
		}
		Parse(temp, count);												// Load recursive data to NEMAParser
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
//	Note Current SPI Configuration Enables 1V8 TM pulses after a few minutes if exposed to Sky
//	SPI A2035 outputs No Clock and No Data	SPI-CS --> 1V8

	
//	uint32_t	err_code;
	
	pullupdown_gpio_cfg_output(SPI1S_CSN_PIN, Pull_up);
	pullupdown_gpio_cfg_output(SPI1S_SCK_PIN, Pull_down);
	
	pullupdown_gpio_cfg_output(A2035H_ON_OFF_PIN_NUMBER, Pull_down);
	pullupdown_gpio_cfg_output(A2035H_NRST_PIN_NUMBER,   Pull_up);
	pullupdown_gpio_cfg_output(A2035H_INT_PIN_NUMBER,    Pull_down);	// Need to sort out the operation of this pin WARNING---> Pulled low via external 33k!!!
	pullupdown_gpio_cfg_output(A2035H_NEN_PIN_NUMBER,    Pull_down);	// Ensure 3V3GPS Powered OFF
	
//	nrf_gpio_pin_set(SPI1S_CSN_PIN);									// ASSERT HIGH Extract from A2035H eval User Guide Table 4: Swith Settings Pg:13
//	nrf_gpio_pin_clear(SPI1S_SCK_PIN);									// ASSERT  LOW Extract from A2035H eval User Guide Table 4: Swith Settings Pg:13
	
	nrf_gpio_pin_clear(A2035H_NRST_PIN_NUMBER);							// A2035 Reset      	---> ASSERT LOW Keep Low during POWER UP
	nrf_gpio_pin_clear(A2035H_ON_OFF_PIN_NUMBER);						// A2035 ONOFF 	
	
	nrf_delay_us(100000); 												// Delay 100ms 
	
	nrf_gpio_pin_clear(A2035H_NEN_PIN_NUMBER);							// A2035 NEN 3V3GPS OFF ---> ASSERT LOW
	
	nrf_delay_us(100000); 												// Delay 100ms 
	
	nrf_gpio_pin_set(A2035H_NEN_PIN_NUMBER);							// Enable 3V3GPS Supply Rail
	nrf_delay_us(250000); 												// Delay 250ms 
	
	
	nrf_gpio_pin_set(A2035H_NRST_PIN_NUMBER);							// A2035 Reset ---> Pull HIGH Also Pulled High via 33k
	nrf_delay_us(250000); 												// Delay 250ms 

	A2035H_Toggle_ONOFF(); 												// Delay 100ms 

//	err_code = spi_slave_A2035H_init();									// Initialise Slave SPI Event Handler Here
//	APP_ERROR_CHECK(err_code);

	pullupdown_gpio_cfg_output(SPI1S_CSN_PIN, Pull_up);
	pullupdown_gpio_cfg_output(A2035H_INT_PIN_NUMBER,    Pull_up);		// Test to check pin state Post nReset

//	A2035H_Init_SPI0();													// Initalise the SPI 0 Port used by GPS A2035H (Host == SLAVE)
																		// ---> Shared with SFLASH --> SPI0 Host = MASTER (reset A2035-H for SFALSH Use)
}

void A2035H_Toggle_ONOFF(void){
//	ON_OFF Strobe Low-High-low
	nrf_gpio_pin_clear(A2035H_ON_OFF_PIN_NUMBER);						// Assert ON_OFF Pin Low		
	nrf_delay_us(50000); 												// Delay 50ms
	
	nrf_gpio_pin_set(A2035H_ON_OFF_PIN_NUMBER);							// Assert ON_OFF Pin High
	nrf_delay_us(220000); 												// delay minimum 200ms ---> allow 220msec
	
	nrf_gpio_pin_clear(A2035H_ON_OFF_PIN_NUMBER);						// Assert ON_OFF Pin Low
	nrf_delay_us(50000); 												// Delay 50ms 
}

void A2035H_RESET_ON(void) {
	nrf_gpio_pin_clear(A2035H_NRST_PIN_NUMBER);							// A2035 Reset ---> Pull HIGH Also Pulled High via 33k
	nrf_delay_us(50000); 												// Delay 50ms 
}

void A2035H_RESET_OFF(void) {
	nrf_gpio_pin_set(A2035H_NRST_PIN_NUMBER);							// A2035 Reset ---> Pull HIGH Also Pulled High via 33k
	nrf_delay_us(50000); 												// Delay 50ms 
}

void A2035H_POWER_OFF(void) {											// WARNING Abrust Power Down may corrupt FLASH
	A2035H_Toggle_ONOFF();
	nrf_gpio_pin_clear(A2035H_NRST_PIN_NUMBER);							// A2035 Reset ---> Pull HIGH Also Pulled High via 33k
	nrf_delay_us(50000); 												// Delay 50ms
	
	nrf_gpio_pin_clear(A2035H_NEN_PIN_NUMBER);							// A2035 NEN 3V3GPS OFF ---> ASSERT LOW
	nrf_delay_us(50000); 												// Delay 50ms
}

void A2035H_POWER_ON(void) {											// WARNING Abrust Power Down may corrupt FLASH
	nrf_gpio_pin_set(A2035H_NEN_PIN_NUMBER);							// A2035 NEN 3V3GPS OFF ---> ASSERT LOW
	nrf_delay_us(200000); 												// Delay 200ms
}

void initA2035H(void) {
	init_A2035H_UART();
//	init_A2035H_SPI_SLAVE();
//	init_A2035H_SPI_MASTER();
}

void init_A2035H_UART(void) {
/*	NOTE	For SPI  Mode A2035.GPIO6.SPI_CLK Left OPEN   --> Weak 	 Internal pull DOWN
						  A2035.GPIO7.SPI_CS  Left OPEN   --> Weak   Internal pull UP (Rail Unknown Probably 1V8)
						  
   (A2235H) For I2C  Mode A2035.GPIO6.SPI_CLK Pulled HIGH --> Strong External 10k Pull UP to 1V8 rail (R27)
						  A2035.GPIO7.SPI_CS  Pulled LOW  --> Strong External 10k Pull Down  
	
			For UART Mode A2035.GPIO6.SPI_CLK Pulled HIGH --> Strong External 10k Pull UP to 1V8 rail (R27)
						  A2035.GPIO7.SPI_CS  Left   OPEN --> Weak   Internal pull UP (Rail Unknown Probably 1V8)  */

//	WORKING...	Current HARDWARE Config 10k pullups to 1V8 on GPIO6.SPI_CLK and GPIO7.SPI_CS
//				UART MODE OPERATIONAL RX=1V8 TX=0V_3V3(DATA) GPIO6.SPI_CLK=1V8 GPIO7.SPI_CS=1V8
	
//	uint32_t	err_code;
	
//	pullupdown_gpio_cfg_output(SPI1S_CSN_PIN, Pull_up);					// Doesn't Work --> UART Mode with internal 3V3 Pullups ()
//	pullupdown_gpio_cfg_output(SPI1S_SCK_PIN, Pull_down);				// Try and Force SPI Mode
	
//	pullupdown_gpio_cfg_output(SPI1S_CSN_PIN, Pull_up);					// Doesn't Work --> UART Mode with internal 3V3 Pullups ()
//	pullupdown_gpio_cfg_output(SPI1S_SCK_PIN, Pull_up);
	
	pullupdown_gpio_cfg_output(A2035H_ON_OFF_PIN_NUMBER, Pull_down);	// Drive all rails to ground to stop 3V3-1V8 bleed through
	nrf_gpio_pin_clear(A2035H_ON_OFF_PIN_NUMBER);						// A2035 ONOFF 	---> ASSERT LOW Keep Low during POWER UP
	
	pullupdown_gpio_cfg_output(A2035H_NRST_PIN_NUMBER,   Pull_up);
	nrf_gpio_pin_clear(A2035H_NRST_PIN_NUMBER);							// A2035 Reset	---> ASSERT LOW Keep Low during POWER UPpullupdown_gpio_cfg_output(A2035H_INT_PIN_NUMBER,    Pull_down);	// Need to sort out the operation of this pin WARNING---> Pulled low via external 33k!!!
	
	pullupdown_gpio_cfg_output(A2035H_NEN_PIN_NUMBER,    Pull_down);	// Ensure 3V3GPS Powered OFF
	nrf_gpio_pin_clear(A2035H_NEN_PIN_NUMBER);							// A2035 NEN 3V3GPS OFF ---> ASSERT LOW
	
//	nrf_gpio_pin_set(SPI1S_CSN_PIN);										// ASSERT HIGH Extract from A2035H eval User Guide Table 4: Swith Settings Pg:13
//	nrf_gpio_pin_clear(SPI1S_SCK_PIN);									// ASSERT  LOW Extract from A2035H eval User Guide Table 4: Swith Settings Pg:13
	
	nrf_delay_us(250000); 												// Delay 250ms Allow 3V#GPS rail to blead off residual power
	
	nrf_gpio_pin_set(A2035H_NEN_PIN_NUMBER);							// ASSERT NEN --> 3V3 Enable 3V3GPS Supply Rail
	nrf_gpio_pin_clear(A2035H_NRST_PIN_NUMBER);							// ASSERT Reset	
	nrf_delay_us(250000); 												// Delay 250ms 
	
	nrf_gpio_pin_set(A2035H_NRST_PIN_NUMBER);							// DEASSERT Reset ---> Pull HIGH Also Pulled High via 33k	
	nrf_delay_us(250000); 												// Delay 250ms 
	
//	pullupdown_gpio_cfg_output(SPI1S_SCK_PIN, Pull_disable);
//	pullupdown_gpio_cfg_output(SPI1S_SCK_PIN, Pull_up);

//	nrf_delay_us(800000); 												// Delay 800msec (may need up to 1 second if Wakeup not monitored)

	A2035H_Toggle_ONOFF(); 												// Start ---> OFF-ON-OFF Toggle Sequence

	nrf_delay_us(800000);
//	pullupdown_gpio_cfg_output(SPI1S_CSN_PIN, Pull_down);				// Test to see what happens Forces Io from 1V8 to 0V0

//	nrf_delay_us(800000);
//	pullupdown_gpio_cfg_output(SPI1S_MOSI_PIN, Pull_up);
//	pullupdown_gpio_cfg_output(SPI1S_CSN_PIN, Pull_down);
//	uint16_t kk=0;
//	uint16_t jj=0;
//	for(0;kk<50;kk++) {
//		jj=0;
//		for(0;jj<8;jj++) {
//			pullupdown_gpio_cfg_output(SPI1S_MOSI_PIN, Pull_up);
//			nrf_delay_us(20000);	
//			pullupdown_gpio_cfg_output(SPI1S_MOSI_PIN, Pull_down);
//			nrf_delay_us(20000);		
//		}
//	}
//	err_code = spi_slave_A2035H_init();									// Initialise Slave SPI Event Handler Here
//	APP_ERROR_CHECK(err_code);

//	pullupdown_gpio_cfg_output(SPI1S_CSN_PIN, Pull_up);
	
//	pullupdown_gpio_cfg_output(SPI1S_MOSI_PIN, Pull_up);				// Test Pullup up in case pin is an Open Drain
	
//	pullupdown_gpio_cfg_output(A2035H_INT_PIN_NUMBER, Pull_down);		// Test to check pin state Post nReset

//	A2035H_Init_SPI0();													// Initalise the SPI 0 Port used by GPS A2035H (Host == SLAVE)
																		// ---> Shared with SFLASH --> SPI0 Host = MASTER (reset A2035-H for SFALSH Use)
}

void A2035H_disable_SPI_Bus_Pins(void) {								// ToDo  Disable the A2035h nCS SLCK MISO MOSI Lines
	
	NRF_GPIO->PIN_CNF[A2035H_CS_PIN] = 								
	(GPIO_PIN_CNF_SENSE_Disabled		<< GPIO_PIN_CNF_SENSE_Pos)	
	| (GPIO_PIN_CNF_DRIVE_S0S1			<< GPIO_PIN_CNF_DRIVE_Pos)	
	| (GPIO_PIN_CNF_PULL_Disabled		<< GPIO_PIN_CNF_PULL_Pos)	
	| (GPIO_PIN_CNF_INPUT_Disconnect	<< GPIO_PIN_CNF_INPUT_Pos)	
	| (GPIO_PIN_CNF_DIR_Input			<< GPIO_PIN_CNF_DIR_Pos);		// Configure The SPI-CS  Line as InputIO Pin as Input	

	NRF_GPIO->PIN_CNF[SPI1S_SCK_PIN] =								
	(GPIO_PIN_CNF_SENSE_Disabled		<< GPIO_PIN_CNF_SENSE_Pos)		
	| (GPIO_PIN_CNF_DRIVE_S0S1			<< GPIO_PIN_CNF_DRIVE_Pos)	
	| (GPIO_PIN_CNF_PULL_Disabled		<< GPIO_PIN_CNF_PULL_Pos)	
	| (GPIO_PIN_CNF_INPUT_Disconnect	<< GPIO_PIN_CNF_INPUT_Pos)	
	| (GPIO_PIN_CNF_DIR_Input			<< GPIO_PIN_CNF_DIR_Pos);		// Configure SPI1 Slave CLKIO Pin as Input


	NRF_GPIO->PIN_CNF[SPI1S_MISO_PIN] = 							
	(GPIO_PIN_CNF_SENSE_Disabled		<< GPIO_PIN_CNF_SENSE_Pos)	
	| (GPIO_PIN_CNF_DRIVE_S0S1			<< GPIO_PIN_CNF_DRIVE_Pos)	
	| (GPIO_PIN_CNF_PULL_Disabled		<< GPIO_PIN_CNF_PULL_Pos)	
	| (GPIO_PIN_CNF_INPUT_Disconnect	<< GPIO_PIN_CNF_INPUT_Pos)	
	| (GPIO_PIN_CNF_DIR_Input			<< GPIO_PIN_CNF_DIR_Pos);		// Configure MISO IO Pin as Input

	NRF_GPIO->PIN_CNF[SPI1S_MOSI_PIN] = 							
	(GPIO_PIN_CNF_SENSE_Disabled		<< GPIO_PIN_CNF_SENSE_Pos)		
	| (GPIO_PIN_CNF_DRIVE_S0S1			<< GPIO_PIN_CNF_DRIVE_Pos)		
	| (GPIO_PIN_CNF_PULL_Disabled		<< GPIO_PIN_CNF_PULL_Pos)	
	| (GPIO_PIN_CNF_INPUT_Disconnect	<< GPIO_PIN_CNF_INPUT_Pos)	
	| (GPIO_PIN_CNF_DIR_Input			<< GPIO_PIN_CNF_DIR_Pos);		// Configure The MOSI IO Pin as Input
  
	  
	SFLASH_GPS_flag=999;												// Bus released ---> IO Pins No longer defined --> Now Safe to Assert AT45 SFLASH Read
}

void A2035H_enable_SPI_Bus_Pins(void) {									// ToDo  Reenable the A2035h nCS SLCK MISO MOSI Lines -- Check SPI Slave Init options
		
	pullupdown_gpio_cfg_output(AT45_ALT_NCS, Pull_up);					// PULL and ASSERT SFLASH NCS HIGH to Ensure No Bus Contention (Upon Entry --> Should Already be High!!!!)
	nrf_gpio_pin_set(AT45_ALT_NCS);							

	uint32_t err_code = spi_slave_A2035H_init();						// Initialise Slave SPI Event Handler Here
	APP_ERROR_CHECK(err_code);

	SFLASH_GPS_flag=4;													// SPI SLAVE Bus reasserted for use by A2035 (SFLASH Should not be asserted if SFLASH_GPS_flag=4
}

