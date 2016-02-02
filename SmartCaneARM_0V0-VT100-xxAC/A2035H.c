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
 
#ifndef A2035H
#define A2035H


/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/
#include <stdint.h>  									// for uint32_t etc.
#include <stdbool.h>

#include "A2035H.h"
#include "spi_master.h"
#include "nrf_gpio.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "spi_master.h"

#include "nrf_delay.h"


#define A2035H_ON_OFF_PIN_NUMBER	(13U)				/**<  GPS ON-OFF Toogle Control GPIO pin number. */
#define A2035H_NRST_PIN_NUMBER		(10U)				/**<  GPS nRESET GPIO pin number. */
#define A2035H_NEN_PIN_NUMBER		(04U)				/**<  GPS 3V3 Linear regulator nEnable GPIO pin number. */
#define A2035H_INT_PIN_NUMBER		(14U)				/**<  GPS int GPIO pin number. */


#define SPI_TO_USE SPI_MASTER_0

#define A2035H_CS_PIN 				(15U)     			/**< SPI0 A2035H GPIO pin number. */
#define AT45DB161E_CS_PINS			(18U)     			/**< SPI0 CS SFlash AT45DB161EGPIO pin number. */

#ifdef SPI_MASTER_0_ENABLE

#define SPIM0_SCK_PIN       		(19U)     			/**< SPI Clock GPIO pin number. */
#define SPIM0_MOSI_PIN      		(17U)     			/**< SPI Master Out Slave In GPIO pin number. */
#define SPIM0_MISO_PIN      		(20U)    			/**< SPI Master In Slave Out GPIO pin number. */

#define SPIM0_SS_PIN        		A2035H_CS_PIN     	/**< SPI Slave Select GPIO pin number.   re pin 15*/

#endif 	// SPI0 ENABLE Definitions

#endif	// A2035H definitions


#define SPIFREQ_TO_USE SPI_FREQUENCY_FREQUENCY_M1			// 1Mbps ----> Need to Check This????? WARNING WARNING May be to fast for A2035-H

#define SPICPOL_TO_USE SPI_CONFIG_CPOL_ActiveLow
#define SPICPHA_TO_USE SPI_CONFIG_CPHA_Leading


#define TX_RX_MSG_LENGTH         4

static uint8_t m_rx_data_spi[TX_RX_MSG_LENGTH]; 			/**< SPI master RX buffer. */
static volatile bool m_transfer_completed = true;


#define SPI_DUMMY_BYTE   0x00
 
#define CS_HIGH     NRF_GPIO->OUTSET = (1UL << A2035H_CS_PIN)
#define CS_LOW      NRF_GPIO->OUTCLR = (1UL << A2035H_CS_PIN)
 
 
/**@brief Function for initializing a SPI master driver.
 *
 * @param[in] spi_master_instance       An instance of SPI master module.
 * @param[in] spi_master_event_handler  An event handler for SPI master events.
 * @param[in] lsb                       Bits order LSB if true, MSB if false.
 */
static void spi_master_init(spi_master_hw_instance_t   spi_master_instance,
                            spi_master_event_handler_t spi_master_event_handler,
                            const bool                 lsb)
{
    uint32_t err_code = NRF_SUCCESS;

    // Configure SPI master.
    spi_master_config_t spi_config = SPI_MASTER_INIT_DEFAULT;

    switch (spi_master_instance) {
        
		#ifdef SPI_MASTER_0_ENABLE
        case SPI_MASTER_0:
        {
            spi_config.SPI_Pin_SCK  = SPIM0_SCK_PIN;
            spi_config.SPI_Pin_MISO = SPIM0_MISO_PIN;
            spi_config.SPI_Pin_MOSI = SPIM0_MOSI_PIN;
            spi_config.SPI_Pin_SS   = SPIM0_SS_PIN;
        }
        break;
        #endif /* SPI_MASTER_0_ENABLE */
		
		case SPI_MASTER_1:
			break;
		
        default:
            break;
    }
	
	// bit order
    spi_config.SPI_CONFIG_ORDER = (lsb ? SPI_CONFIG_ORDER_LsbFirst : SPI_CONFIG_ORDER_MsbFirst);
	spi_config.SPI_Freq = SPIFREQ_TO_USE;
	spi_config.SPI_CONFIG_CPHA = SPICPHA_TO_USE;
	spi_config.SPI_CONFIG_CPOL = SPICPOL_TO_USE;
	spi_config.SPI_PriorityIRQ = APP_IRQ_PRIORITY_HIGH;
	
    err_code = spi_master_open(spi_master_instance, &spi_config);
    
	APP_ERROR_CHECK(err_code);

    // Register event handler for SPI master.
    spi_master_evt_handler_reg(spi_master_instance, spi_master_event_handler);
}


/***************************************************************************//**
 * @brief Initializes the SPI communication peripheral and resets the part.
 *
 * @return 1.
*******************************************************************************/
void A2035H_Init_IO(void) {
																				// Pull A2035H CS pin  
	 NRF_GPIO->PIN_CNF[A2035H_CS_PIN] = 							\
        (GPIO_PIN_CNF_SENSE_Disabled 	<< GPIO_PIN_CNF_SENSE_Pos)	\
      | (GPIO_PIN_CNF_DRIVE_H0D1     	<< GPIO_PIN_CNF_DRIVE_Pos)	\
      | (GPIO_PIN_CNF_PULL_Pullup    	<< GPIO_PIN_CNF_PULL_Pos)	\
      | (GPIO_PIN_CNF_INPUT_Disconnect  << GPIO_PIN_CNF_INPUT_Pos)	\
      | (GPIO_PIN_CNF_DIR_Output      	<< GPIO_PIN_CNF_DIR_Pos);	
		
		NRF_GPIO->DIRSET = (1UL << A2035H_CS_PIN);
		NRF_GPIO->OUTSET = (1UL << A2035H_CS_PIN); 
	
	spi_master_init(SPI_TO_USE, NULL, false);  									// no handler, msb first

    A2035H_Reset();
}

/**@brief Function for sending and receiving data.
 *
 * @param[in]   spi_master_hw_instance  SPI master instance.
 * @param[in]   p_tx_data               A pointer to a buffer TX.
 * @param[out]  p_rx_data               A pointer to a buffer RX.
 * @param[in]   len                     A length of the data buffers.
 */
static void SPI_Write(const spi_master_hw_instance_t spi_master_hw_instance,
                          uint8_t * const                p_tx_data,
                          uint8_t * const                p_rx_data,
                          const uint16_t                 len)
{
    uint32_t err_code;
	err_code = spi_master_send_recv(spi_master_hw_instance, p_tx_data, len, p_rx_data, len);    // Start transfer.
    APP_ERROR_CHECK(err_code);
}
 
 
 
 
/***************************************************************************//**
 * @brief Sets the Reset bit of the A2035H.
 *
 * @return None.
*******************************************************************************/
void A2035H_Reset(void) {
//    A2035H_SetRegisterValue(A2035H_REG_CMD | A2035H_RESET);
	
}


/***************************************************************************//**
 * @brief Writes the value to a register.
 *
 * @param -  regValue - The value to write to the register.
 *
 * @return  None.    
*******************************************************************************/
void A2035H_SetRegisterValue(unsigned short regValue) {		// 16bit
	unsigned char data[2] = {0x00, 0x00};
	
	data[0] = (unsigned char)((regValue & 0xFF00) >> 8);
	data[1] = (unsigned char)((regValue & 0x00FF) >> 0);
	CS_LOW;
	SPI_Write(SPI_TO_USE, data, m_rx_data_spi ,2);
	CS_HIGH;
}


/***************************************************************************//**
 * @brief Writes to the frequency registers.
 *
 * @param -  reg - Frequence register to be written to.
 * @param -  val - The value to be written.
 *
 * @return  None.    
*******************************************************************************/
//void A2035H_SetFrequency(u16 reg, u32 val) {
//	unsigned short freqHi = reg;
//	unsigned short freqLo = reg;
//	
//	freqHi |= (val & 0xFFFC000) >> 14 ;
//	freqLo |= (val & 0x3FFF);
//	A2035H_SetRegisterValue(A2035H_B28 | A2035H_RESET);  // disable DAC at RESET bit first
//	A2035H_SetRegisterValue(freqLo);			// When B28 is on, first 14 bits is for LSB
//	A2035H_SetRegisterValue(freqHi);			// then is MSB 14bits
//}


/***************************************************************************//**
 * @brief Writes to the phase registers.
 *
 * @param -  reg - Phase register to be written to.
 * @param -  val - The value to be written.
 *
 * @return  None.    
*******************************************************************************/
//void A2035H_SetPhase(u16 reg, u16 val) {
//	unsigned short phase = reg;
//	phase |= val;
//	A2035H_SetRegisterValue(phase);
//}

/***************************************************************************//**
 * @brief Sets the type of waveform to be output.
 *
 * @param -  type - type of waveform to be output.
 *
 * @return  None.    
*******************************************************************************/
//void A2035H_SetWave(unsigned short type) {
//	A2035H_SetRegisterValue(type);
//}




// UART Code Depricated and replaced with SPI0

//void uart_event_handle_withBle(app_uart_evt_t * p_event) {	// is setup callback in initA2035H

//	static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];		// 20 bytes
//    static uint8_t index = 0;
//    uint32_t err_code;

//    switch (p_event->evt_type)
//    {
//        case APP_UART_DATA_READY:
//            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
//            index++;

//            if ((data_array[index - 1] == '\n') || (index >= (BLE_NUS_MAX_DATA_LEN)))
//            {
//                err_code = ble_nus_string_send(&m_nus, data_array, index);
//                if (err_code != NRF_ERROR_INVALID_STATE)
//                {
//                    APP_ERROR_CHECK(err_code);
//                }
//                
//                index = 0;
//            }
//            break;

//        case APP_UART_COMMUNICATION_ERROR:
//            APP_ERROR_HANDLER(p_event->data.error_communication);
//            break;

//        case APP_UART_FIFO_ERROR:
//            APP_ERROR_HANDLER(p_event->data.error_code);
//            break;

//        default:
//			UNUSED_VARIABLE(app_uart_get(&data_array[index]));
//            index++;

//            if ((data_array[index - 1] == '\n') || (index >= (BLE_NUS_MAX_DATA_LEN)))
//            {
//                //err_code = ble_nus_string_send(&m_nus, data_array, index);
//                if (err_code != NRF_ERROR_INVALID_STATE)
//                {
//                    APP_ERROR_CHECK(err_code);
//                }
//                
//                index = 0;
//            }
//            break;
//            }

//}


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
			pullset = (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);
			break;
		
		case Pull_down:
			pullset = (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos);
			break;
		
		case Pull_disable:
			pullset = (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos);
			break;
	}

    /*lint -e{845} 													// A zero has been given as right argument to operator '|'" */
    NRF_GPIO->PIN_CNF[pin_number] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                                            | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                                            | pullset
                                            | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
                                            | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
}

void initA2035H(void) {
	
	nrf_delay_us(1000000); 												// delay 1s
	
	pullupdown_gpio_cfg_output(A2035H_ON_OFF_PIN_NUMBER, Pull_down);
	nrf_delay_us(1000000); 												// delay 1s
	
	pullupdown_gpio_cfg_output(A2035H_NRST_PIN_NUMBER, Pull_up);
	nrf_delay_us(1000000); 												// delay 1s
	
	pullupdown_gpio_cfg_output(A2035H_NEN_PIN_NUMBER, Pull_up);
	nrf_delay_us(1000000); 												// delay 1s
	
	pullupdown_gpio_cfg_output(A2035H_INT_PIN_NUMBER, Pull_down);
	nrf_delay_us(1000000); 												// delay 1s
	
	
//	pullupdown_gpio_cfg_output(A2035H_TX_PIN_NUMBER, Pull_down);
	
//	nrf_gpio_cfg_output(A2035H_ON_OFF_PIN_NUMBER);
//	nrf_gpio_cfg_output(A2035H_NRST_PIN_NUMBER);
//	nrf_gpio_cfg_output(A2035H_RX_PIN_NUMBER);
//	nrf_gpio_cfg_output(A2035H_TX_PIN_NUMBER);

	nrf_gpio_pin_set(A2035H_NEN_PIN_NUMBER);			// Enable Aux 3V3 regulator
	nrf_gpio_pin_clear(A2035H_NRST_PIN_NUMBER);			// Toggle NRST
	
	nrf_delay_us(100000); 								// Delay 100ms 
	
	nrf_gpio_pin_set(A2035H_NRST_PIN_NUMBER);
	
	nrf_delay_us(100000); 								// Delay 100ms 

	nrf_gpio_pin_clear(A2035H_ON_OFF_PIN_NUMBER);		// Assert ON_OFF Pin Low
	nrf_delay_us(100000); 								// Delay 100ms
	
	nrf_gpio_pin_set(A2035H_ON_OFF_PIN_NUMBER);			// Assert ON_OFF Pin High
	nrf_delay_us(200000); 								// delay 200ms
	
	nrf_gpio_pin_clear(A2035H_ON_OFF_PIN_NUMBER);		// Assert ON_OFF Pin Low
	nrf_delay_us(100000); 								// Delay 100ms 
	
//	nrf_gpio_pin_clear(A2035H_RX_PIN_NUMBER);
//	nrf_gpio_pin_set(A2035H_TX_PIN_NUMBER);

	nrf_delay_us(1000);

//	Note as of Smartcane 1V0 the A2035H configured as SPI Slave ---> UART no longer used.

//	ToDo Sort out SPI and parse string




// $$$$$$ UART Rx Depricated and replaced with SPI0 Parser
//	uint32_t err_code;
//    const app_uart_comm_params_t comm_params =
//      {
//          A2035H_TX_PIN_NUMBER,  // need to swap RX / TX at controller side
//          A2035H_RX_PIN_NUMBER,
//          0U,
//          0U,
//          APP_UART_FLOW_CONTROL_DISABLED,
//          false,  // no parity
//          UART_BAUDRATE_BAUDRATE_Baud4800
//      };

//    APP_UART_FIFO_INIT(&comm_params,
//                         UART_RX_BUF_SIZE,
//                         UART_TX_BUF_SIZE,
//                         uart_event_handle_withBle,
//                         APP_IRQ_PRIORITY_LOW,
//                         err_code);

//    APP_ERROR_CHECK(err_code);
	  
//	printf("\n\rStart: \n\r");
//	
//	uint8_t cr;
//    while (true)
//    {
//        
//        while(app_uart_get(&cr) != NRF_SUCCESS);
//        //while(app_uart_put(cr) != NRF_SUCCESS);

//		cr += 1;
//        if (false) // (cr == 'q' || cr == 'Q')
//        {
//            printf(" \n\rExit!\n\r");

//            while (true)
//            {
//                // Do nothing.
//            }
//        }
//    }
	
}
