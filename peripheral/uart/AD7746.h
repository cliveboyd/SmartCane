/*******************************************************************************
 *	@file   AD7746.h
 *	@device AD7746 I2C Quad or (Dual Differential) Capacitor Sensor
 *	@brief  Header file of AD7746 Driver.
 *	@author DBogdan (dragos.bogdan@analog.com)
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
#ifndef _AD7746_H_
#define _AD7746_H_

/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/
#include "Communication.h"

/******************************************************************************/
/* AD7746                                                                     */
/******************************************************************************/
/* AD7746 Slave Address */
#define AD7746_ADDRESS                  0x48  // 0b01001000, I2C module will shift left by 1, 0x90 for write, 0x91 for read

/* AD7746 Slave Address */
#define AD7746_RESET_CMD                0xBF

/* AD7746 Register Definition */
#define AD7746_REG_STATUS				0
#define AD7746_REG_CAP_DATA_HIGH		1
#define AD7746_REG_CAP_DATA_MID			2
#define AD7746_REG_CAP_DATA_LOW			3
#define AD7746_REG_VT_DATA_HIGH			4
#define AD7746_REG_VT_DATA_MID			5
#define AD7746_REG_VT_DATA_LOW			6
#define AD7746_REG_CAP_SETUP			7
#define AD7746_REG_VT_SETUP				8
#define AD7746_REG_EXC_SETUP			9
#define AD7746_REG_CFG					10
#define AD7746_REG_CAPDACA				11
#define AD7746_REG_CAPDACB				12
#define AD7746_REG_CAP_OFFH				13
#define AD7746_REG_CAP_OFFL				14
#define AD7746_REG_CAP_GAINH			15
#define AD7746_REG_CAP_GAINL			16
#define AD7746_REG_VOLT_GAINH			17
#define AD7746_REG_VOLT_GAINL			18

/* AD7746_REG_STATUS bits */
#define AD7746_STATUS_EXCERR			(1 << 3)
#define AD7746_STATUS_RDY				(1 << 2)
#define AD7746_STATUS_RDYVT				(1 << 1)
#define AD7746_STATUS_RDYCAP			(1 << 0)

/* AD7746_REG_CAP_SETUP bits */
#define AD7746_CAPSETUP_CAPEN			(1 << 7)
#define AD7746_CAPSETUP_CIN2			(1 << 6)
#define AD7746_CAPSETUP_CAPDIFF			(1 << 5)
#define AD7746_CAPSETUP_CACHOP			(1 << 0)

/* AD7746_REG_VT_SETUP bits */
#define AD7746_VTSETUP_VTEN				(1 << 7)
#define AD7746_VTSETUP_VTMD_INT_TEMP	(0 << 5)
#define AD7746_VTSETUP_VTMD_EXT_TEMP	(1 << 5)
#define AD7746_VTSETUP_VTMD_VDD_MON		(2 << 5)
#define AD7746_VTSETUP_VTMD_EXT_VIN		(3 << 5)
#define AD7746_VTSETUP_EXTREF			(1 << 4)
#define AD7746_VTSETUP_VTSHORT			(1 << 1)
#define AD7746_VTSETUP_VTCHOP			(1 << 0)

/* AD7746_REG_EXC_SETUP bits */
#define AD7746_EXCSETUP_CLKCTRL			(1 << 7)
#define AD7746_EXCSETUP_EXCON			(1 << 6)
#define AD7746_EXCSETUP_EXCB			(1 << 5)
#define AD7746_EXCSETUP_NEXCB			(1 << 4)
#define AD7746_EXCSETUP_EXCA			(1 << 3)
#define AD7746_EXCSETUP_NEXCA			(1 << 2)
#define AD7746_EXCSETUP_EXCLVL(x)		(((x) & 0x3) << 0)

/* AD7746_REG_CFG bits */
#define AD7746_CONF_VTFS(x)				(((x) & 0x3) << 6)
#define AD7746_CONF_CAPFS(x)			(((x) & 0x7) << 3)
#define AD7746_CONF_MODE_IDLE			(0 << 0)
#define AD7746_CONF_MODE_CONT_CONV		(1 << 0)
#define AD7746_CONF_MODE_SINGLE_CONV	(2 << 0)
#define AD7746_CONF_MODE_PWRDN			(3 << 0)
#define AD7746_CONF_MODE_OFFS_CAL		(5 << 0)
#define AD7746_CONF_MODE_GAIN_CAL		(6 << 0)

/* AD7746_REG_CAPDACx bits */
#define AD7746_CAPDAC_DACEN				(1 << 7)
#define AD7746_CAPDAC_DACP(x)			((x) & 0x7F)

/******************************************************************************/
/* Functions Prototypes                                                       */
/******************************************************************************/

/* Initializes the I2C communication peripheral. */
unsigned char AD7746_Init(void);
/* Writes data into AD7746 registers, starting from the selected register
   address pointer. */
void AD7746_Write(unsigned char subAddr,
                  unsigned char* dataBuffer,
                  unsigned char bytesNumber);
/* Reads data from AD7746 registers, starting from the selected register
   address pointer. */
void AD7746_Read(unsigned char subAddr,
                 unsigned char* dataBuffer,
                 unsigned char bytesNumber);
/* Resets the AD7746. */
void AD7746_Reset(void);
/* Waits until a conversion on a voltage/temperature channel has been finished
   and returns the output data. */
unsigned long AD7746_GetVTData(void);
/* Waits until a conversion on a capacitive channel has been finished and
   returns the output data. */
unsigned long AD7746_GetCapData(void);

unsigned long AD7746_getConfig(void);
#endif // _AD7746_H
