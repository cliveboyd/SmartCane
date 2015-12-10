/***************************************************************************//**
 *   @file   AD9837.c
 *   @brief  Implementation of AD9837 Driver.
 *   @author Mihai Bancisor
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: 560
*******************************************************************************/

/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/

#include "spi_master.h"
#include "nrf_gpio.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "spi_master.h"
#include "nrf_delay.h"


#include "AD9837.h"		// AD9837 definitions.

#define AD9837_CS_PIN (7U)
#define OTHER_CS_PINS {20U}
#define SPI_TO_USE SPI_MASTER_0
#define FREQREG_TO_USE AD9837_REG_FREQ0
#define PHAREG_TO_USE AD9837_REG_PHASE0

#ifdef SPI_MASTER_0_ENABLE
#define SPIM0_SCK_PIN       (8U)     /**< SPI clock GPIO pin number. */
#define SPIM0_MOSI_PIN      (9U)     /**< SPI Master Out Slave In GPIO pin number. */
#define SPIM0_MISO_PIN      (11U)    // dummy pin /**< SPI Master In Slave Out GPIO pin number. */
#define SPIM0_SS_PIN        AD9837_CS_PIN     /**< SPI Slave Select GPIO pin number. */
#endif // SPI_MASTER_0_ENABLE

#ifdef SPI_MASTER_1_ENABLE
#define SPIM1_SCK_PIN       (19U)     /**< SPI clock GPIO pin number. */
#define SPIM1_MOSI_PIN      (18U)     /**< SPI Master Out Slave In GPIO pin number. */
#define SPIM1_MISO_PIN      (17U)     /**< SPI Master In Slave Out GPIO pin number. */
#define SPIM1_SS_PIN        (20U)  	// flash spi instance bus CS     /**< SPI Slave Select GPIO pin number. */
#endif // SPI_MASTER_1_ENABLE


#define SPIFREQ_TO_USE SPI_FREQUENCY_FREQUENCY_M1
#define SPICPOL_TO_USE SPI_CONFIG_CPOL_ActiveLow
#define SPICPHA_TO_USE SPI_CONFIG_CPHA_Leading


#define TX_RX_MSG_LENGTH         3
static uint8_t m_rx_data_spi[TX_RX_MSG_LENGTH]; /**< SPI master RX buffer. */
static volatile bool m_transfer_completed = true;


#define SPI_DUMMY_BYTE   0xCC
 
#define CS_HIGH     NRF_GPIO->OUTSET = (1UL << AD9837_CS_PIN)
#define CS_LOW      NRF_GPIO->OUTCLR = (1UL << AD9837_CS_PIN)
 

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

    switch (spi_master_instance)
    {
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

        #ifdef SPI_MASTER_1_ENABLE
        case SPI_MASTER_1:
        {
            spi_config.SPI_Pin_SCK  = SPIM1_SCK_PIN;
            spi_config.SPI_Pin_MISO = SPIM1_MISO_PIN;
            spi_config.SPI_Pin_MOSI = SPIM1_MOSI_PIN;
            spi_config.SPI_Pin_SS   = SPIM1_SS_PIN;
        }
        break;
        #endif /* SPI_MASTER_1_ENABLE */

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
unsigned char AD9837_Init(u32 freq, u16 phase)
{
	
	u8 sharedCS[] = OTHER_CS_PINS;
	
	// pull up all CS pins first
	for(int i=0; i<sizeof(sharedCS)/sizeof(sharedCS[0]);i++) {
		    NRF_GPIO->PIN_CNF[sharedCS[i]] =     \
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) \
      | (GPIO_PIN_CNF_DRIVE_H0D1     << GPIO_PIN_CNF_DRIVE_Pos) \
      | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)  \
      | (GPIO_PIN_CNF_INPUT_Disconnect  << GPIO_PIN_CNF_INPUT_Pos) \
      | (GPIO_PIN_CNF_DIR_Output      << GPIO_PIN_CNF_DIR_Pos);	
		NRF_GPIO->DIRSET = (1UL << sharedCS[i]);
		NRF_GPIO->OUTSET = (1UL << sharedCS[i]); 
	}
	// pull up this chip's CS pin as well
	 NRF_GPIO->PIN_CNF[AD9837_CS_PIN] =     \
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) \
      | (GPIO_PIN_CNF_DRIVE_H0D1     << GPIO_PIN_CNF_DRIVE_Pos) \
      | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)  \
      | (GPIO_PIN_CNF_INPUT_Disconnect  << GPIO_PIN_CNF_INPUT_Pos) \
      | (GPIO_PIN_CNF_DIR_Output      << GPIO_PIN_CNF_DIR_Pos);	
		NRF_GPIO->DIRSET = (1UL << AD9837_CS_PIN);
		NRF_GPIO->OUTSET = (1UL << AD9837_CS_PIN); 
	
	spi_master_init(SPI_TO_USE, NULL, false);  // no handler, msb first

    AD9837_Reset();
	AD9837_SetFrequency(FREQREG_TO_USE, freq);
	AD9837_SetPhase(PHAREG_TO_USE, phase);
	
    return (1);
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
    // Start transfer.
    uint32_t err_code =
        spi_master_send_recv(spi_master_hw_instance, p_tx_data, len, p_rx_data, len);
    APP_ERROR_CHECK(err_code);
}

/***************************************************************************//**
 * @brief Sets the Reset bit of the AD9837.
 *
 * @return None.
*******************************************************************************/
void AD9837_Reset(void)
{
    AD9837_SetRegisterValue(AD9837_REG_CMD | AD9837_RESET);
}

/***************************************************************************//**
 * @brief Clears the Reset bit of the AD9837.
 * i.e. enable the DAC output
 * @return None.
*******************************************************************************/
void AD9837_Enable(bool bSq)
{
	if(bSq) 
		AD9837_SetRegisterValue(AD9837_REG_CMD | AD9837_B28 |AD9837_OPBITEN);
	else
		AD9837_SetRegisterValue(AD9837_REG_CMD | AD9837_B28 );
}
/***************************************************************************//**
 * @brief Writes the value to a register.
 *
 * @param -  regValue - The value to write to the register.
 *
 * @return  None.    
*******************************************************************************/
void AD9837_SetRegisterValue(unsigned short regValue)
{
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
void AD9837_SetFrequency(u16 reg, u32 val)
{
	unsigned short freqHi = reg;
	unsigned short freqLo = reg;
	
	freqHi |= (val & 0xFFFC000) >> 14 ;
	freqLo |= (val & 0x3FFF);
	AD9837_SetRegisterValue(AD9837_B28 | AD9837_RESET);  // disable DAC at RESET bit first
	AD9837_SetRegisterValue(freqLo);			// When B28 is on, first 14 bits is for LSB
	AD9837_SetRegisterValue(freqHi);			// then is MSB 14bits
}
/***************************************************************************//**
 * @brief Writes to the phase registers.
 *
 * @param -  reg - Phase register to be written to.
 * @param -  val - The value to be written.
 *
 * @return  None.    
*******************************************************************************/
void AD9837_SetPhase(u16 reg, u16 val)
{
	unsigned short phase = reg;
	phase |= val;
	AD9837_SetRegisterValue(phase);
}

/***************************************************************************//**
 * @brief Sets the type of waveform to be output.
 *
 * @param -  type - type of waveform to be output.
 *
 * @return  None.    
*******************************************************************************/
void AD9837_SetWave(unsigned short type)
{
	AD9837_SetRegisterValue(type);
}

