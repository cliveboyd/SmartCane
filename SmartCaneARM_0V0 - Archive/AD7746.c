/***************************************************************************//**
 *   @file   AD7746.c
 *   @brief  Implementation of AD7746 Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
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
 *   SVN Revision: 585
*******************************************************************************/

/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/
#include "AD7746.h"
#include "nrf_assert.h"
#include "nrf_gpio.h"

#define AD7746_RDY_BAR_PIN (4U)  // Assertion low RDY pin as input
/***************************************************************************//**
 * @brief Initializes the I2C communication peripheral.
 *
 * @return status - The result of the I2C initialization.
 *                  Example: 0 - Initialization failed;
 *                           1 - Initialization succeeded.
*******************************************************************************/
unsigned char AD7746_Init(void)
{
    unsigned char status = 0;
    
    status = I2C_Init();
	
#if defined(BOARD_CUSTOM)
	nrf_gpio_cfg_input(AD7746_RDY_BAR_PIN, NRF_GPIO_PIN_NOPULL );  // use RDY pin to sense data ready 
#endif
	
	AD7746_Reset();
    unsigned char config = 0;
	// set cap measurement
	AD7746_Read(AD7746_REG_CAP_SETUP, &config,1);
	ASSERT(config == 0);
		
		config = config | AD7746_CAPSETUP_CAPEN | AD7746_CAPSETUP_CACHOP;
	  AD7746_Write(AD7746_REG_CAP_SETUP, &config,1);
	
	AD7746_Read(AD7746_REG_CAP_SETUP, &config,1);
	ASSERT(config == (AD7746_CAPSETUP_CAPEN | AD7746_CAPSETUP_CACHOP));
	// set temp measurement	
		AD7746_Read(AD7746_REG_VT_SETUP, &config,1);
		
		config = config | AD7746_VTSETUP_VTEN | AD7746_VTSETUP_VTCHOP;
	  AD7746_Write(AD7746_REG_VT_SETUP, &config,1);
		AD7746_Read(AD7746_REG_VT_SETUP, &config,1);
	ASSERT(config == (AD7746_VTSETUP_VTEN | AD7746_VTSETUP_VTCHOP));
	// set config
		AD7746_Read(AD7746_REG_CFG, &config,1);
		
	// single conversion, slow conversion time (120ms for cap and temp)
		config = AD7746_CONF_MODE_CONT_CONV | AD7746_CONF_VTFS(0) | AD7746_CONF_CAPFS(0);
	  AD7746_Write(AD7746_REG_CFG, &config,1);
				AD7746_Read(AD7746_REG_CFG, &config,1);
	ASSERT(config == (AD7746_CONF_MODE_CONT_CONV | AD7746_CONF_VTFS(0) | AD7746_CONF_CAPFS(0)));
	
    return status;
}

unsigned long AD7746_getConfig(void)
{
	unsigned long ret = 0;
	    unsigned char config = 0;
	// set cap measurement
	AD7746_Read(AD7746_REG_CAP_SETUP, &config,1);
ASSERT(config == (AD7746_CAPSETUP_CAPEN | AD7746_CAPSETUP_CACHOP));
	ret = config << 16;
	// set temp measurement	
		AD7746_Read(AD7746_REG_VT_SETUP, &config,1);
			ASSERT(config == (AD7746_VTSETUP_VTEN | AD7746_VTSETUP_VTCHOP));
	ret += config << 8;
	// set config
		AD7746_Read(AD7746_REG_CFG, &config,1);
		ASSERT(config == (AD7746_CONF_MODE_CONT_CONV | AD7746_CONF_VTFS(3) | AD7746_CONF_CAPFS(7)));

				return config;

}

/***************************************************************************//**
 * @brief Writes data into AD7746 registers, starting from the selected
 *        register address pointer.
 *
 * @param subAddr - The selected register address pointer.
 * @param dataBuffer - Pointer to a buffer storing the transmission data.
 * @param bytesNumber - Number of bytes that will be sent.
 *
 * @return None.
*******************************************************************************/
void AD7746_Write(unsigned char subAddr,
                  unsigned char* dataBuffer,
                  unsigned char bytesNumber)
{
    unsigned char sendBuffer[10] = {0, };
    unsigned char byte = 0;
    
    sendBuffer[0] = subAddr;
    for(byte = 1; byte <= bytesNumber; byte++)
    {
        sendBuffer[byte] = dataBuffer[byte - 1];
    }
    while(!I2C_Write(AD7746_ADDRESS,
              sendBuffer,
              bytesNumber + 1,
              1));
}

/***************************************************************************//**
 * @brief Reads data from AD7746 registers, starting from the selected
 *        register address pointer.
 *
 * @param subAddr - The selected register address pointer.
 * @param dataBuffer - Pointer to a buffer that will store the received data.
 * @param bytesNumber - Number of bytes that will be read.
 *
 * @return None.
*******************************************************************************/
void AD7746_Read(unsigned char subAddr,
                 unsigned char* dataBuffer,
                 unsigned char bytesNumber)
{
    while(!I2C_Write(AD7746_ADDRESS,
              (unsigned char*)&subAddr,
              1,
              0));
    while(!I2C_Read(AD7746_ADDRESS,
             dataBuffer,
             bytesNumber,
             1));
}

/***************************************************************************//**
 * @brief Resets the AD7746.
 *
 * @return None
*******************************************************************************/
void AD7746_Reset(void)
{
    unsigned char cmd = 0;
    
    cmd = AD7746_RESET_CMD;
    while(!I2C_Write(AD7746_ADDRESS,
              (unsigned char*)&cmd,
              1,
              1));
}

/***************************************************************************//**
 * @brief Waits until a conversion on a voltage/temperature channel has been
 *        finished and returns the output data.
 *
 * @return capData - The content of the VT Data register.
*******************************************************************************/
unsigned long AD7746_GetVTData(void)
{
    unsigned char receiveBuffer[3] = {0, 0, 0};
    unsigned long vtData = 0;
    
    receiveBuffer[0] = AD7746_STATUS_RDYVT;
    while(!(receiveBuffer[0] & AD7746_STATUS_RDYVT))
    {
        AD7746_Read(AD7746_REG_STATUS,
                    receiveBuffer,
                    1);
    }
    AD7746_Read(AD7746_REG_VT_DATA_HIGH,
                receiveBuffer,
                3);
    vtData = ((unsigned long)receiveBuffer[0] << 16) +
             ((unsigned short)receiveBuffer[1] << 8) +
             receiveBuffer[2];
    
    return vtData;
}

/***************************************************************************//**
 * @brief Waits until a conversion on a capacitive channel has been finished and
 *        returns the output data.
 *
 * @return capData - The content of the Cap Data register.
*******************************************************************************/
unsigned long AD7746_GetCapData(void)
{
    unsigned char receiveBuffer[3] = {0, 0, 0};
    unsigned long capData = 0;
    
    receiveBuffer[0] = AD7746_STATUS_RDYCAP;
    while(!(receiveBuffer[0] & AD7746_STATUS_RDYCAP))
    {
        AD7746_Read(AD7746_REG_STATUS,
                    receiveBuffer,
                    1);
    }
    AD7746_Read(AD7746_REG_CAP_DATA_HIGH,
                receiveBuffer,
                3);
    capData = ((unsigned long)receiveBuffer[0] << 16) +
              ((unsigned short)receiveBuffer[1] << 8) +
              receiveBuffer[2];
    
    return capData;
}
