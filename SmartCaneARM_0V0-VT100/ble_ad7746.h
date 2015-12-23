/***************************************************************************//**
 *	@file   ble_AD7746.h
 *	@device AD7746 I2C Quad (Dual Differential) Capacitor Sensor
 *	@brief  Header file of Blue tooth AD7746 Capacitor Sensor Interface Driver.
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
 
#ifndef BLE_AD7746_H__
#define BLE_AD7746_H__

#include <string.h>
#include "service_maker.h"
#include "ble.h"
#include "nordic_common.h"									// For UNUSED_PARAMETER


#define SERV_BASE AD7746									// TODO: Set the service base name
#define CHAR1 CONFIG
#define CHAR2 CAP
#define CHAR3 TEMP

//TODO-default:  set the service UUID base, 
MAKE_UUID_BASE(SERV_BASE) = {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00};

typedef enum {
	MAKE_UUID_SERVICE(SERV_BASE, 0x3456),					// TODO:  set the service UUID
	MAKE_UUID_CHAR_NOVALUE(SERV_BASE,CHAR1),
	MAKE_UUID_CHAR_NOVALUE(SERV_BASE,CHAR2),
	MAKE_UUID_CHAR_NOVALUE(SERV_BASE,CHAR3),
} MAKE_UUID_ENUM(SERV_BASE);

MAKE_SERVICE_STRUCT(SERV_BASE);								// Forward declaration for service struct ble_AD7746_t

MAKE_SERVICE_INIT_STRUCT(SERV_BASE);

typedef struct CONFIG_bytes_s {
	unsigned char configs[12];
} CONFIG_bytes_t;

typedef void (*on_char_write_callback_t) (MAKE_SERVICE_STRUCT_NAME(SERV_BASE) *, CONFIG_bytes_t*, uint8_t len);


MAKE_SERVICE_STRUCT_CONTENT(SERV_BASE, { \
	uint16_t service_handle; \
	uint8_t  uuid_type;  \
	uint16_t conn_handle; \
	ble_gatts_char_handles_t MAKE_CHAR_HANDLEID(CHAR1); \
	ble_gatts_char_handles_t MAKE_CHAR_HANDLEID(CHAR2); \
	ble_gatts_char_handles_t MAKE_CHAR_HANDLEID(CHAR3); \

	on_char_write_callback_t on_write_config_callback;				// TODO: add your customized struct contents below:
});

static MAKE_SERVICE_STRUCT_NAME(SERV_BASE) m_AD7746;


MAKE_SERVICE_INIT_STRUCT_CONTENT( SERV_BASE, { \
	on_char_write_callback_t on_write_config_callback;				// TODO: add your customized init struct contents below:
});

/**@brief Function for initializing the Service.
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
MAKE_SERVICE_INIT_CALL(SERV_BASE);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Service.
 */
MAKE_SERVICE_BLE_EVT_HANDLER(SERV_BASE);

uint32_t ble_AD7746_send_temp_notify(MAKE_SERVICE_STRUCT_NAME(SERV_BASE) *, float tempValue);

/* DEBUG: preprocessor generated codes: 
static unsigned char AD7746_UUID_BASE[] = {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00};

typedef enum {
	AD7746_UUID_SERVICE = 0x3456,   
	AD7746_UUID_CONFIG_CHAR,
	AD7746_UUID_CAP_CHAR,
	AD7746_UUID_TEMP_CHAR,
}AD7746_UUID_t;

typedef struct ble_AD7746_s ble_AD7746_t;
typedef struct ble_AD7746_init_s ble_AD7746_init_t;

typedef struct CONFIG_bytes_s {
	unsigned char configs[12];
}CONFIG_bytes_t;

typedef void (*on_char_write_callback_t) (ble_AD7746_t *, CONFIG_bytes_t*);

#line 64 "ble_ad7746.h"

typedef struct ble_AD7746_s { uint16_t service_handle; uint8_t uuid_type; uint16_t conn_handle; ble_gatts_char_handles_t CONFIG_char_handleID; ble_gatts_char_handles_t CAP_char_handleID; ble_gatts_char_handles_t TEMP_char_handleID; on_char_write_callback_t on_write_config_callback; } ble_AD7746_t;

typedef struct ble_AD7746_init_s { on_char_write_callback_t on_write_config_callback; } ble_AD7746_init_t;

*/

#endif // BLE_AD7746_H__
