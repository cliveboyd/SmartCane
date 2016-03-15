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
/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/
#include "Communication.h"
#include "spi_master.h"
#include "twi_master.h"
#include "nrf_gpio.h"

#include "app_util_platform.h"

#include "global.h"


#define SPI_MISO_READ() 	((NRF_GPIO->IN >> SPI_MISO_PIN) & 0x1UL)		//  Reads current state of MISO Pin 

#define A2035H_NCS_PIN		(15U)
#define AT45_NCS_PIN		(18U)

#define SPIFREQ_TO_USE SPI_FREQUENCY_FREQUENCY_M1							// Clock == 1MHz
#define SPICPOL_TO_USE SPI_CONFIG_CPOL_ActiveHigh							// Configuration SPI Mode 1  CPOL:0
//#define SPICPOL_TO_USE SPI_CONFIG_CPOL_ActiveLow
#define SPICPHA_TO_USE SPI_CONFIG_CPHA_Trailing								// Configuration SPI Mode 1  CPHA:1
//#define SPICPHA_TO_USE SPI_CONFIG_CPHA_Leading 

// Note for GPS SPI Mode-1 CPOL:0 CPHA:1

#define TX_RX_MSG_LENGTH	4

static uint8_t m_rx_data_spi[TX_RX_MSG_LENGTH];								/**< SPI master RX buffer. */
static volatile bool m_transfer_completed = true;

#define SPI_DUMMY_BYTE		0xCC
#define SPI_DUMMY_BYTE0		0xAA
#define SPI_DUMMY_BYTE1		0x55
#define SPI_DUMMY_BYTE2		0xCC
 
#define AT45_NCS_HIGH	NRF_GPIO->OUTSET = (1UL << AT45_NCS_PIN)
#define AT45_NCS_LOW	NRF_GPIO->OUTCLR = (1UL << AT45_NCS_PIN)

#define A2035H_NCS_HIGH	NRF_GPIO->OUTSET = (1UL << A2035H_NCS_PIN)
#define A2035H_NCS_LOW	NRF_GPIO->OUTCLR = (1UL << A2035H_NCS_PIN)

#define SPI_TO_USE SPI_MASTER_0

uint16_t	SPI_Event=0;						// Dummy Variable

// MASTER-0 Shared between AT45 SFLASH and A2035H GPS    (Note: Different SPI Modes ---> Under Test)
#define SPIM0_SCK_PIN       (19U)     			/**< SPI clock GPIO pin number. */
#define SPIM0_MOSI_PIN      (17U)     			/**< SPI Master Out Slave In  GPIO pin number. */
#define SPIM0_MISO_PIN      (20U)    			/**< SPI Master In  Slave Out GPIO pin number. */

static uint16_t	CommunicationsFlag=0;			// AT45=2   A2035=4


/**@brief Function for initializing a SPI master driver For SPI Mode 1.    ---> SFLASH AT45
 *
 * @param[in] spi_master_instance       An instance of SPI master module.
 * @param[in] spi_master_event_handler  An event handler for SPI master events.
 * @param[in] lsb                       Bits order LSB if true, MSB if false.
 */
static void spi_master_mode0_init(spi_master_hw_instance_t	spi_master_instance,
							spi_master_event_handler_t 		spi_master_event_handler,
							const bool                 		lsb)
{
    uint32_t err_code = NRF_SUCCESS;

//	Configure SPI master.spi_master_init
	spi_master_config_t spi_config = SPI_MASTER_INIT_DEFAULT;

    switch (spi_master_instance) {
        
		#ifdef SPI_MASTER_0_ENABLE
        case SPI_MASTER_0:
        {
            spi_config.SPI_Pin_SCK  = SPIM0_SCK_PIN;
            spi_config.SPI_Pin_MISO = SPIM0_MISO_PIN;
            spi_config.SPI_Pin_MOSI = SPIM0_MOSI_PIN;
            spi_config.SPI_Pin_SS   = AT45_NCS_PIN;				// SPIM0_SS_PIN;
        }
        break;
        #endif /* SPI_MASTER_0_ENABLE */

        #ifdef SPI_MASTER_1_ENABLE								// ---> NOT USED
        case SPI_MASTER_1:
        {

        }
        break;
        #endif 	/* SPI_MASTER_1_ENABLE */

        default:
            break;
    }
	// bit order and Clocking SPI Mode 0 ---> AT45 SFLASH
    spi_config.SPI_CONFIG_ORDER	= SPI_CONFIG_ORDER_MsbFirst;					//	SPI_CONFIG_ORDER_MsbFirst ---> This is correct for AT45 re datasheet
	spi_config.SPI_Freq			= SPI_FREQUENCY_FREQUENCY_M2;					// 	500KHz chip rated to 66MHz
//	spi_config.SPI_Freq			= SPI_FREQUENCY_FREQUENCY_K500;
	spi_config.SPI_CONFIG_CPHA	= SPI_CONFIG_CPHA_Leading;						//  AT45 returns correct WhoAmI output as seen on Oscilliscope but Rx register not correct 0's or F's ??
	spi_config.SPI_CONFIG_CPOL	= SPI_CONFIG_CPOL_ActiveHigh;					//  AT45 returns correct WhoAmI output as seen on Oscilliscope but Rx register not correct 0's or F's ??
	


	
	spi_config.SPI_PriorityIRQ	= APP_IRQ_PRIORITY_HIGH;
	

	if (CommunicationsFlag == 0) {												// Test for 1st instance and bypass SPI_master_close
		err_code = spi_master_open(spi_master_instance, &spi_config);
		CommunicationsFlag=2;													// Flag as AT45 master
	}
	else {
		CommunicationsFlag=2;	
		spi_master_close(spi_master_instance);									// Close previous spi instatance
		err_code = spi_master_open(spi_master_instance, &spi_config);			// Start a new spi instance
	}
	APP_ERROR_CHECK(err_code);
	APP_ERROR_CHECK(err_code);

//	Register event handler for SPI master.
//	spi_master_evt_handler_reg(spi_master_instance, spi_master_event_handler);  // Not Used??
}

//void spi_master_event_handler(void){											// Dummy to see what happens
//	SPI_Event++;
//}


/**@brief Function for initializing a SPI master driver For SPI Mode 1.
 *
 * @param[in] spi_master_instance       An instance of SPI master module.
 * @param[in] spi_master_event_handler  An event handler for SPI master events.
 * @param[in] lsb                       Bits order LSB if true, MSB if false.
*/


static void spi_master_mode1_init(spi_master_hw_instance_t	spi_master_instance,
							spi_master_event_handler_t 		spi_master_event_handler,
							const bool                 		lsb)
{
    uint32_t err_code = NRF_SUCCESS;

//	Configure SPI master.spi_master_init
	spi_master_config_t spi_config = SPI_MASTER_INIT_DEFAULT;

    switch (spi_master_instance) {
        
		#ifdef SPI_MASTER_0_ENABLE
        case SPI_MASTER_0:
        {
            spi_config.SPI_Pin_SCK  = SPIM0_SCK_PIN;
            spi_config.SPI_Pin_MISO = SPIM0_MISO_PIN;
            spi_config.SPI_Pin_MOSI = SPIM0_MOSI_PIN;
            spi_config.SPI_Pin_SS   = A2035H_NCS_PIN;							// SPIM0_SS_PIN;
        }
        break;
        #endif /* SPI_MASTER_0_ENABLE */

        #ifdef SPI_MASTER_1_ENABLE												// ---> NOT USED
        case SPI_MASTER_1:
        {

        }
        break;
        #endif 	/* SPI_MASTER_1_ENABLE */

        default:
            break;
    }
	// bit order
    spi_config.SPI_CONFIG_ORDER	= (lsb ? SPI_CONFIG_ORDER_LsbFirst : SPI_CONFIG_ORDER_MsbFirst);  // lsb False ---> MSBFirst
	spi_config.SPI_Freq			= SPIFREQ_TO_USE;								//1MHz
	spi_config.SPI_CONFIG_CPHA	= SPI_CONFIG_CPHA_Trailing;
	spi_config.SPI_CONFIG_CPOL	= SPI_CONFIG_CPOL_ActiveHigh;
	spi_config.SPI_PriorityIRQ	= APP_IRQ_PRIORITY_HIGH;
	
	
	if (CommunicationsFlag == 0) {												// Test for 1st instance and bypass SPI_master_close
		err_code = spi_master_open(spi_master_instance, &spi_config);
		CommunicationsFlag=4;													// Flag as A2035 GPS master
	}
	else {
		CommunicationsFlag=4;	
		spi_master_close(spi_master_instance);									// Close previouse SPI instance before starting a new mode
		err_code = spi_master_open(spi_master_instance, &spi_config);			// start a new spi instance
	}
	APP_ERROR_CHECK(err_code);

//	Register event handler for SPI master.
//	spi_master_evt_handler_reg(spi_master_instance, spi_master_event_handler);
}



/***************************************************************************************
 *	@brief Reset NCS Pins on AT45 and A2035H and then Initializes the SPI communication 
 *	for NCS==A2035 with AT45 NCS==HIGH
 *
 *	@return 1.
****************************************************************************************/
unsigned char SPI_A2035H_Init(void) {
	
	// pull up all CS pins first
	NRF_GPIO->PIN_CNF[A2035H_NCS_PIN] =								\
	  (GPIO_PIN_CNF_SENSE_Disabled		<< GPIO_PIN_CNF_SENSE_Pos)	\
	| (GPIO_PIN_CNF_DRIVE_H0D1			<< GPIO_PIN_CNF_DRIVE_Pos)	\
	| (GPIO_PIN_CNF_PULL_Pullup			<< GPIO_PIN_CNF_PULL_Pos)	\
	| (GPIO_PIN_CNF_INPUT_Disconnect	<< GPIO_PIN_CNF_INPUT_Pos)	\
	| (GPIO_PIN_CNF_DIR_Output			<< GPIO_PIN_CNF_DIR_Pos);	
	
	NRF_GPIO->DIRSET = (1UL << A2035H_NCS_PIN);
	NRF_GPIO->OUTSET = (1UL << A2035H_NCS_PIN); 
	
//	Pull up this chip's CS pin as well
	NRF_GPIO->PIN_CNF[AT45_NCS_PIN] =								\
	  (GPIO_PIN_CNF_SENSE_Disabled		<< GPIO_PIN_CNF_SENSE_Pos)	\
	| (GPIO_PIN_CNF_DRIVE_H0D1			<< GPIO_PIN_CNF_DRIVE_Pos)	\
	| (GPIO_PIN_CNF_PULL_Pullup			<< GPIO_PIN_CNF_PULL_Pos)	\
	| (GPIO_PIN_CNF_INPUT_Disconnect	<< GPIO_PIN_CNF_INPUT_Pos)	\
	| (GPIO_PIN_CNF_DIR_Output			<< GPIO_PIN_CNF_DIR_Pos);	
		
	NRF_GPIO->DIRSET = (1UL << AT45_NCS_PIN);												// ASSERT Both NCS signals HIGH
	NRF_GPIO->OUTSET = (1UL << AT45_NCS_PIN); 
	
	spi_master_mode1_init(SPI_TO_USE, NULL, false);  										// NULL ---> No handler, msb first

//	AD9837_Reset();
//	AD9837_SetFrequency(FREQREG_TO_USE, freq);
//	AD9837_SetPhase(PHAREG_TO_USE, phase);
	
    return (1);
}


/***************************************************************************************
 *	@brief Reset NCS Pins on AT45 and A2035H and then Initializes the SPI communication 
 *	for NCS==A2035 with AT45 NCS==HIGH
 *
 *	@return 1.
****************************************************************************************/
unsigned char SPI_AT45_Init(void) {
	
	// pull up all CS pins first
	NRF_GPIO->PIN_CNF[AT45_NCS_PIN] =								\
	  (GPIO_PIN_CNF_SENSE_Disabled		<< GPIO_PIN_CNF_SENSE_Pos)	\
	| (GPIO_PIN_CNF_DRIVE_H0D1			<< GPIO_PIN_CNF_DRIVE_Pos)	\
	| (GPIO_PIN_CNF_PULL_Pullup			<< GPIO_PIN_CNF_PULL_Pos)	\
	| (GPIO_PIN_CNF_INPUT_Disconnect	<< GPIO_PIN_CNF_INPUT_Pos)	\
	| (GPIO_PIN_CNF_DIR_Output			<< GPIO_PIN_CNF_DIR_Pos);	
	
	NRF_GPIO->DIRSET = (1UL << AT45_NCS_PIN);
	NRF_GPIO->OUTSET = (1UL << AT45_NCS_PIN); 
	
//	Pull up this chip's CS pin as well
	NRF_GPIO->PIN_CNF[A2035H_NCS_PIN] =								\
	  (GPIO_PIN_CNF_SENSE_Disabled		<< GPIO_PIN_CNF_SENSE_Pos)	\
	| (GPIO_PIN_CNF_DRIVE_H0D1			<< GPIO_PIN_CNF_DRIVE_Pos)	\
	| (GPIO_PIN_CNF_PULL_Pullup			<< GPIO_PIN_CNF_PULL_Pos)	\
	| (GPIO_PIN_CNF_INPUT_Disconnect	<< GPIO_PIN_CNF_INPUT_Pos)	\
	| (GPIO_PIN_CNF_DIR_Output			<< GPIO_PIN_CNF_DIR_Pos);	
		
	NRF_GPIO->DIRSET = (1UL << A2035H_NCS_PIN);												// ASSERT Both NCS signals HIGH
	NRF_GPIO->OUTSET = (1UL << A2035H_NCS_PIN); 
	
	spi_master_mode0_init(SPI_TO_USE, NULL, false);  										// NULL ---> No handler, msb first

//	AD9837_Reset();
//	AD9837_SetFrequency(FREQREG_TO_USE, freq);
//	AD9837_SetPhase(PHAREG_TO_USE, phase);
	
    return (1);
}


/**@brief Function for sending and receiving data.
 *
 * @param[in]   spi_master_hw_instance  SPI master instance.
 * @param[in]   p_tx_data               A pointer to a buffer TX.
 * @param[out]  p_rx_data               A pointer to a buffer RX.
 * @param[in]   len                     A length of the data buffers.
 */
char SPI_write(const spi_master_hw_instance_t	spi_master_hw_instance,
				uint8_t * const					p_tx_data,
				uint8_t * const					p_rx_data,
				const uint16_t					len)
{
	
//	Start transfer.
	uint32_t err_code =
	spi_master_send_recv(spi_master_hw_instance, p_tx_data, len, p_rx_data, len);
	APP_ERROR_CHECK(err_code);
	return p_rx_data[0];
}


char AT45_spi_write(uint8_t Byte) {
	uint8_t TxByte=Byte;
	uint8_t RxByte=0;

	
	uint8_t * const p_tx_data = &TxByte; 
	uint8_t * const p_rx_data = &RxByte; 


//	Start transfer.
	AT45_NCS_LOW;														// Has to be Low so action nCS ---> also controlled within AT45.c	
	uint32_t err_code = spi_master_send_recv(SPI_TO_USE, p_tx_data, 1, p_rx_data, 1);
//	AT45_NCS_HIGH;														// nCS controlled within AT45.c	
	
	APP_ERROR_CHECK(err_code);
	return RxByte;														//p_rx_data[0];
}

char AT45_spi_write_Auto_nCS(uint8_t Byte) {
	uint8_t TxByte=Byte;
	uint8_t RxByte=0;

	
	uint8_t * const p_tx_data = &TxByte; 
	uint8_t * const p_rx_data = &RxByte; 


//	Start transfer.
	AT45_NCS_LOW;														
	uint32_t err_code = spi_master_send_recv(SPI_TO_USE, p_tx_data, 1, p_rx_data, 1);
	AT45_NCS_HIGH;														
	
	APP_ERROR_CHECK(err_code);
	return RxByte;														//p_rx_data[0];
}

uint16_t AT45_SPIM0_write_16b(unsigned short regValue) {
	unsigned char data[2] = {0x00, 0x00};
	uint16_t	RxByte;
	
	data[0] = (unsigned char)((regValue & 0xFF00) >> 8);
	data[1] = (unsigned char)((regValue & 0x00FF) >> 0);
	AT45_NCS_LOW;														// nCS controlled within AT45.c	
	SPI_write(SPI_TO_USE, data, m_rx_data_spi ,2);
	AT45_NCS_HIGH;														// nCS controlled within AT45.c	
	
	RxByte = m_rx_data_spi[1] << 8;
	RxByte = RxByte & m_rx_data_spi[0];
	
	return RxByte;
}

uint32_t AT45_SPIM0_write_32b(uint32_t Value) {
	unsigned char data[4];
	uint32_t	RxByte;
	
	data[0] = (unsigned char)((Value & 0xFF000000) >> 24);
	data[1] = (unsigned char)((Value & 0x00FF0000) >> 16);
	data[2] = (unsigned char)((Value & 0x0000FF00) >>  8);
	data[3] = (unsigned char)((Value & 0x000000FF) >>  0);
	
	AT45_NCS_LOW;														// nCS controlled within AT45.c	
	SPI_write(SPI_TO_USE, data, m_rx_data_spi ,4);
	AT45_NCS_HIGH;														// nCS controlled within AT45.c	
	
	RxByte = m_rx_data_spi[3] << 8;
	RxByte = m_rx_data_spi[2] << 8;
	RxByte = m_rx_data_spi[1] << 8;
	RxByte = RxByte & m_rx_data_spi[0];
	
	return RxByte;
}


uint16_t A2035H_SPIM0_Write_16b(unsigned short regValue) {
	unsigned char data[2] = {0x00, 0x00};
	uint16_t	RxByte;
	
	data[0] = (unsigned char)((regValue & 0xFF00) >> 8);
	data[1] = (unsigned char)((regValue & 0x00FF) >> 0);
	A2035H_NCS_LOW;
	SPI_write(SPI_TO_USE, data, m_rx_data_spi ,2);
	A2035H_NCS_HIGH;
	
	RxByte = m_rx_data_spi[1] << 8;
	RxByte = RxByte & m_rx_data_spi[0];
	
	return RxByte;
}

/***************************************************************************//**
 * @brief Sets the Reset bit of the AD9837.
 *
 * @return None.
*******************************************************************************/
////void AD9837_Reset(void) {
////    AD9837_SetRegisterValue(AD9837_REG_CMD | AD9837_RESET);
////}





/***************************************************************************//**
 * @brief Initializes the I2C communication peripheral.
 *
 * @return status - The result of the I2C initialization.
 *                  Example: 0 - Initialization failed;
 *                           1 - Initialization succeeded.
*******************************************************************************/
unsigned char I2C_Init(void)
{
	if (!twi_master_init()) return 0;

	return 1;
}

/***************************************************************************//**
 * @brief Writes data to a slave device.
 *
 * @param slaveAddress - Adress of the slave device.
 * @param dataBuffer - Pointer to a buffer storing the transmission data.
 * @param bytesNumber - Number of bytes to write.
 * @param stopBit - Stop condition control.
 *                  Example: 0 - A stop condition will not be sent;
 *                           1 - A stop condition will be sent.
 *
 * @return status - Number of written bytes.
*******************************************************************************/
unsigned char I2C_Write(unsigned char slaveAddress,
                        unsigned char *dataBuffer,
                        unsigned char bytesNumber,
                        unsigned char stopBit)
{
	unsigned char targetAddress = (slaveAddress << 1);
	
	if (twi_master_transfer(targetAddress, dataBuffer, bytesNumber, stopBit)) {
		return bytesNumber;
	}
	return 0;
}

/***************************************************************************//**
 * @brief Reads data from a slave device.
 *
 * @param slaveAddress - Adress of the slave device.
 * @param dataBuffer - Pointer to a buffer that will store the received data.
 * @param bytesNumber - Number of bytes to read.
 * @param stopBit - Stop condition control.
 *                  Example: 0 - A stop condition will not be sent;
 *                           1 - A stop condition will be sent.
 *
 * @return status - Number of read bytes.
*******************************************************************************/
unsigned char I2C_Read(unsigned char slaveAddress,
                       unsigned char* dataBuffer,
                       unsigned char bytesNumber,
                       unsigned char stopBit)
{
	unsigned char targetAddress = (slaveAddress << 1) | 1;
	
	if (twi_master_transfer(targetAddress, dataBuffer, bytesNumber, stopBit)) {
		return bytesNumber;
	}
	return 0;
}

