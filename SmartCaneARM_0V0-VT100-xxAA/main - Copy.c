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
 
 
#include <stdint.h>  // for uint32_t etc.
#include <string.h>  // for memset()

#include "softdevice_handler.h"
#include "pstorage.h"
#include "ble_advertising.h"
#include "device_manager.h"
#include "ble_hci.h"  			// for set hci_status_code to sd_ble_gap_disconnect
#include "ble_conn_params.h"
#include "bsp.h" 						// for bsp_indication_set
#include "app_scheduler.h"  // for scheduler APP_SCHED_INIT
#include "app_timer_appsh.h"  // for app_timer_event_t

#include "nordic_common.h"  // for UNUSED_PARAMETER

#include "app_button.h"
#include "app_timer.h"  // for APP_TIMER_TICKS
#include "app_gpiote.h" // for APP_GPIOTE_INIT
#include "nrf_assert.h" // for ASSERT


#include "ble_ad7746.h"
#include "AD7746.h"

#include "nrf_temp.h"
#include "sensorADC.h"

#include "ds2401.h"
#include "AT45_Flash.h"
#include "MPU_9150.h"  
#include "inv_mpu.h"
#include "nrf_delay.h"

#include "A2035H.h"
//#include "ble_nus.h"  // for nordic uart service
#include "app_uart.h"
#include "MPL3115.h"
#include "ltc2943.h"


#define IS_SRVC_CHANGED_CHARACT_PRESENT  0                                          /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/


#define DEVICE_NAME                      "SmartCane"                               /**< Name of device. Will be included in the advertising data. */
#define DEVICE_APPEARANCE								BLE_APPEARANCE_UNKNOWN											/**< Device appearance **/
// for ble_conn_param_init param
#define APP_TIMER_PRESCALER              0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS             (6+BSP_APP_TIMERS_NUMBER)                  /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE          4                                          /**< Size of timer operation queues. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)/**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                          /**< Number of attempts before giving up the connection parameter negotiation. */

// for Service timer
#define CAP_MEAS_INTERVAL       APP_TIMER_TICKS(500, APP_TIMER_PRESCALER) /**< measurement interval (ticks). */
#define TEMP_MEAS_INTERVAL      APP_TIMER_TICKS(500, APP_TIMER_PRESCALER) /**< measurement interval (ticks). */


// for GAP param init
#define MIN_CONN_INTERVAL                MSEC_TO_UNITS(9, UNIT_1_25_MS)           /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL                MSEC_TO_UNITS(300, UNIT_1_25_MS)           /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                    0                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                 MSEC_TO_UNITS(4000, UNIT_10_MS)            /**< Connection supervisory timeout (4 seconds). */
#define TX_POWER_LEVEL                    (0)                                              /**< TX Power Level value. This will be set both in the TX Power service, in the advertising data, and also used to set the radio transmit power. */



// for on_adv_evt
// to use button, define BUTTONS_NUMBER in custom_board.h
#define WAKEUP_BUTTON_ID                 0                                          /**< Button used to wake up the application. */
#define BOND_DELETE_ALL_BUTTON_ID        1                                          /**< Button used for deleting all bonded centrals during startup. */


// for ble_advertising_init param
#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */
#define APP_ADV_GAP_FLAG				BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE  



// for app scheduler module
#define SCHED_MAX_EVENT_DATA_SIZE       sizeof(app_timer_event_t)                   /**< Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */
#define SCHED_QUEUE_SIZE                10                                          /**< Maximum number of events in the scheduler queue. */

// for device manager module
#define SEC_PARAM_BOND                   1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                                         /**< Maximum encryption key size. */


//static ble_uuid_t m_adv_uuids[1] = {{AD7746_UUID_SERVICE, BLE_UUID_TYPE_VENDOR_BEGIN}}; /**< Universally unique service identifiers. */

#define PSHOLD_PIN_NUMBER  (28U)	
#define MOTOR_PIN_NUMBER (7U)
																	 
static uint16_t m_conn_handle = 				BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */
static dm_application_instance_t         m_app_handle;                              /**< Application identifier allocated by device manager */

static app_timer_id_t                    m_cap_timer_id;                        /**<  timer. */
static app_timer_id_t                    m_temp_timer_id;                       /**<  timer. */
//static app_timer_id_t                    m_config_timer_id;                       /**<  timer. */
								

static bool finishedADC;


// this callback will be called when write requrest is received by BLE softdevice 
// to write config data to CONFIG_CHARacteristic
void ad7746_on_write_config_callback (ble_AD7746_t *p_ad7746, 
	CONFIG_bytes_t *p_config, uint8_t len)
{
	  // write bytes of config to device ad7746
}
static void button_event_handler(bsp_event_t event)
{
	unsigned long tempData;
	 switch(event)
	 {
		 case BSP_EVENT_KEY_0:
			// tempData = AD7746_GetVTData();
			
			 break;
		 case BSP_EVENT_KEY_1:
			//nrf_gpio_pin_toggle(MOTOR_PIN_NUMBER);
			nrf_gpio_pin_set(MOTOR_PIN_NUMBER);
			nrf_delay_ms(1000);
			nrf_gpio_pin_clear(MOTOR_PIN_NUMBER);
			break;
		 default:
			 break;
	 }
	 UNUSED_VARIABLE(tempData);
}


void someother_module_init()
{
	// LED setup
	uint32_t err_code;
	
	// for button interupt, must be called before 'app_button_init' inside bsp_init
	APP_GPIOTE_INIT(1);  // one user, no use yet unless app_button_enable() is called


	err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
					APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
					button_event_handler);  // test butten handler
	if (err_code == NRF_ERROR_INVALID_STATE)
	{
		err_code = NRF_ERROR_INVALID_STATE;
	}
    APP_ERROR_CHECK(err_code);
//		nrf_gpio_cfg_output(BSP_LED_0);
//		nrf_gpio_cfg_output(BSP_LED_1);
	nrf_gpio_cfg_output(MOTOR_PIN_NUMBER);
	
//		AD7746_Init();
	
//		nrf_temp_init();

		//nrf_gpio_cfg_output(7);
		
	
		unsigned char status = 0;
		while(!status)  status = I2C_Init();
		initMPU9150();
//		if(mpu_init(NULL));
//		long gyro,accel;
//		mpu_run_self_test(&gyro, &accel);
//		sensorADC_config();  // sensor adc init
		finishedADC = false;  // for sensor_adc only
	
// read compass
//		short data[3];
		
		unsigned long timestamp;
//		
//		nrf_delay_us(1000000);


//		while(mpu_get_compass_reg(data, &timestamp)) 
//			nrf_delay_us(1000);
//		while(1)
//		{
//			status = mpu_get_compass_reg(data, &timestamp);
//			if(data[0]>0 || status == 0)
//				break;
//		}
//	long temperature;
//	while(mpu_get_temperature(&temperature, &timestamp));
//	while(mpu_get_accel_reg(data,&timestamp));
//	while(mpu_get_gyro_reg(data,&timestamp));
//	
//	mpu_run_6500_self_test(&gyro, &accel,0);
		timestamp +=1;
		initA2035H();
	MPL3115A2_init();
	float f = MPL3115A2_getAltitude();
//	
	ltc294x_init();
	int temp;
	ltc294x_get_current(&temp);
	ltc294x_get_temperature(&temp);
	ltc294x_get_voltage(&temp);
	ltc294x_get_charge_counter(&temp);
	ltc294x_get_charge_counter(&temp);
}
static void capmeasure_timer_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);

}

int32_t readNRF_TEMP() {
		int32_t ret = 0;
    while (true)
    {
        NRF_TEMP->TASKS_START = 1; /** Start the temperature measurement. */

        /* Busy wait while temperature measurement is not finished, you can skip waiting if you enable interrupt for DATARDY event and read the result in the interrupt. */
        /*lint -e{845} // A zero has been given as right argument to operator '|'" */
        while (NRF_TEMP->EVENTS_DATARDY == 0)
        {
            // Do nothing.
        }
        NRF_TEMP->EVENTS_DATARDY = 0;

        /**@note Workaround for PAN_028 rev2.0A anomaly 29 - TEMP: Stop task clears the TEMP register. */
        ret = (nrf_temp_read() / 4);

        /**@note Workaround for PAN_028 rev2.0A anomaly 30 - TEMP: Temp module analog front end does not power down when DATARDY event occurs. */
        NRF_TEMP->TASKS_STOP = 1; /** Stop the temperature measurement. */

        return ret;
    }
}

#define MEASURESIZE 128
static float dataToSend[MEASURESIZE];
// TODO: why can't send bluetooth notify in this callback? 
static void blockSend(float *Data, unsigned int len) { 
	if (m_AD7746.conn_handle != BLE_CONN_HANDLE_INVALID) {
		for(unsigned int i=0; i<len; i++) {
				dataToSend[i] = Data[i];
		}
		finishedADC = true;
	}
}
typedef enum  {
	DataTypeNone = 0,
	DataTypeTEMP = 1,
}SendDataType;

static SendDataType resumetype;
unsigned int lastSendIndex;

unsigned int resumeSendData() {  // only called by on_ble_evt when BLE_EVT_TX_COMPLETE happens
	// return the index to current finished position in 'dataToSend' buffer
	uint32_t err_code = NRF_SUCCESS;
	if (lastSendIndex==(unsigned int)-1) {
		return (unsigned int)-1;
	}
	switch (resumetype) {  // depends on the data to send type
		case DataTypeTEMP:  // simulating as ADC, TODO: change to ADC service
			if (finishedADC) {
				unsigned int i=lastSendIndex;  // resume from this index
				float tempf;
				bool bNeedBreak = false;
				while(i<MEASURESIZE && !bNeedBreak) {  
					tempf = dataToSend[i];
					err_code = ble_AD7746_send_temp_notify(&m_AD7746, tempf);
					switch (err_code) {
						case NRF_SUCCESS:
						{
							i++;
							break;
						}
						case NRF_ERROR_INVALID_STATE:
						case BLE_ERROR_GATTS_SYS_ATTR_MISSING:
						{  // shouldn't come to here
							finishedADC = false;
							lastSendIndex = (unsigned int)-1;  
							bNeedBreak = true;
							break;
						}
						case BLE_ERROR_NO_TX_BUFFERS:
						{  // the main reason to split the TX process 
							bNeedBreak = true;
							lastSendIndex = i;	// resume from the index
							break; // resend 
						}
						default:  // shouldn't come to here
							APP_ERROR_HANDLER(err_code);
					}
				}
				if (i>=MEASURESIZE) 
				{
					finishedADC = false;
					lastSendIndex = 0;
				}
				return lastSendIndex;
			}
			break; // end of case DataTypeTEMP:
		case DataTypeNone:
			break;
		default:
			break;
	}
	return (unsigned int)-1; // Means invalid
}
static void tempmeasure_timer_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);

	uint32_t err_code = NRF_SUCCESS;
    short data[3];
	unsigned long timestamp;
//	int16_t temp; // = readTempData();
	
	float Acc[3];
//	readAccelFloatMG(Acc);
//	readGyroFloatDeg(Acc);

	readMagFloatUT(Acc);
	err_code = ble_AD7746_send_temp_notify(&m_AD7746, Acc[0]);
		if ((err_code != NRF_SUCCESS) &&
				(err_code != NRF_ERROR_INVALID_STATE) &&
				(err_code != BLE_ERROR_NO_TX_BUFFERS) &&
				(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
			)  // ignore these errors as they appear only during setup and are normal
		{
			APP_ERROR_HANDLER(err_code);
		}

	//readAccelData(data);
	// do measurement and notify
// real AD7746 temp data
//		unsigned long tempData = AD7746_GetVTData();
//		float tempf;
//		tempf = MPL3115A2_getAltitude();
//	
//		tempData = tempData - 0x800000;
//		tempf = tempData/2048.0;
	
//	mpu_get_compass_reg(data, &timestamp);
//		tempf = temp/340.0+35;
//		tempf = sizeof(short);

//float tempf;
//tempf = MPL3115A2_getAltitude();
//		err_code = ble_AD7746_send_temp_notify(&m_AD7746, tempf);
//		if ((err_code != NRF_SUCCESS) &&
//				(err_code != NRF_ERROR_INVALID_STATE) &&
//				(err_code != BLE_ERROR_NO_TX_BUFFERS) &&
//				(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
//			)  // ignore these errors as they appear only during setup and are normal
//		{
//			APP_ERROR_HANDLER(err_code);
//		}

//	int temp;
//	float tempf;
//	ltc294x_get_charge_counter(&temp);
//	tempf = temp;
//		err_code = ble_AD7746_send_temp_notify(&m_AD7746, tempf);
//		if ((err_code != NRF_SUCCESS) &&
//				(err_code != NRF_ERROR_INVALID_STATE) &&
//				(err_code != BLE_ERROR_NO_TX_BUFFERS) &&
//				(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
//			)  // ignore these errors as they appear only during setup and are normal
//		{
//			APP_ERROR_HANDLER(err_code);
//		}
	
// real onboard temp data
//		float tempf;
//		tempf = readNRF_TEMP(); // using internal temp sensor

		
// dummy ADC continuous measure		
//		float rawADCresult = sensorADC_singleMeasure();
//		if (finishedADC) {
			// data is finished measurement, waiting for transmit
//			resumetype = DataTypeTEMP;
//			lastSendIndex = 0;  // restart from 0 index
//			lastSendIndex = resumeSendData();
//		}else {
//			sensorADC_startSequenceMeasure(MEASURESIZE, blockSend);
//		}
		
    return ;
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers.
    err_code = app_timer_create(&m_cap_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                capmeasure_timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_temp_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                tempmeasure_timer_handler);
    APP_ERROR_CHECK(err_code);

}

/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    uint32_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_cap_timer_id, CAP_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_temp_timer_id, TEMP_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);


}

void app_on_connect(ble_evt_t* p_ble_evt)
{
	    uint32_t                         err_code = NRF_SUCCESS;

			err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
			APP_ERROR_CHECK(err_code);
//		nrf_gpio_pin_set(CONNECTED_LED_PIN_NO);
//		nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);

//		err_code = app_button_enable();
		nrf_gpio_pin_set(MOTOR_PIN_NUMBER);
		nrf_delay_ms(1000);
		nrf_gpio_pin_clear(MOTOR_PIN_NUMBER);
	
		APP_ERROR_CHECK(err_code);

}
void app_on_disconnect(ble_evt_t* p_ble_evt)
{
	 	    uint32_t                         err_code = NRF_SUCCESS;
//           nrf_gpio_pin_clear(CONNECTED_LED_PIN_NO);

//            err_code = app_button_disable();
            APP_ERROR_CHECK(err_code);
}

void app_on_adv_timeout(ble_evt_t* p_ble_evt)
{
		uint32_t                         err_code = NRF_SUCCESS;

		if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISING)
		{
        err_code = bsp_indication_set(BSP_INDICATE_USER_STATE_1);
        APP_ERROR_CHECK(err_code);
			// Configure buttons with sense level low as wakeup source.
			nrf_gpio_cfg_sense_input(BSP_BUTTON_0,
									BUTTON_PULL,
									NRF_GPIO_PIN_SENSE_LOW);
			// use STM6601 chip to turn power off completely
		 	NRF_GPIO->OUTCLR = (1UL << PSHOLD_PIN_NUMBER); 

			// Go to system-off mode (this function will not return; wakeup will cause a reset)                
			err_code = sd_power_system_off();
			APP_ERROR_CHECK(err_code);
		}
}

/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t             err_code;
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
		APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Application level BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
			      m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

						app_on_connect(p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
						m_conn_handle = BLE_CONN_HANDLE_INVALID;
            
						app_on_disconnect(p_ble_evt);
            break;

        case BLE_GAP_EVT_TIMEOUT:
						app_on_adv_timeout(p_ble_evt);
            break;
		case BLE_EVT_TX_COMPLETE:
			resumeSendData();
			break;
        default:
            // No implementation needed.
            break;
    }
		UNUSED_VARIABLE(err_code);
}
/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);

            // enable buttons to wake-up from power off
            err_code = bsp_buttons_enable( (1 << WAKEUP_BUTTON_ID)
                                         | (1 << BOND_DELETE_ALL_BUTTON_ID));
            APP_ERROR_CHECK(err_code);

		// use STM6601 chip to turn power off completely
		 	NRF_GPIO->OUTCLR = (1UL << PSHOLD_PIN_NUMBER); 

		
            // Go to system-off mode. This function will not return; wakeup will cause a reset.
            err_code = sd_power_system_off();
            APP_ERROR_CHECK(err_code);
            break;
        default:
            break;
    }
}

/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    dm_ble_evt_handler(p_ble_evt);  // manages Active and Bonded Peers
    ble_conn_params_on_ble_evt(p_ble_evt);
	ble_AD7746_on_ble_evt(&m_AD7746, p_ble_evt);    // TODO: customized service handler
	on_ble_evt(p_ble_evt);  											 	// TODO: common application hander for ble event
		ble_advertising_on_ble_evt(p_ble_evt);   // it will monitor and restart advertising on Disconnection
}

/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
	  pstorage_sys_event_handler(sys_evt);
    ble_advertising_on_sys_evt(sys_evt);
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_250MS_CALIBRATION,NULL); //NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);

#ifdef S110
    // Enable BLE stack.
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
#endif

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

static void DeviceNameFromID(char* name, int len)
{
	if(len<16) {
		strncpy(name, DEVICE_NAME, len);
	}else {
		while(!ds2401_initAndRead()) ;
		int j = 0;
		bool skipLeading = true;
		for(int i=0;i<6;i++)
		{
			if(ds2401_id[i]==0 && skipLeading) continue;
			else skipLeading = false;
			sprintf(name+j*2, "%02X", ds2401_id[i]);
			j++;
		}
		if (j*2<len) {
			strncpy(name+j*2, DEVICE_NAME, len-j*2);
		}
	}
	return;
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
	char IDName[16];
	DeviceNameFromID(IDName,16);
	
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)IDName,
                                          15);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(DEVICE_APPEARANCE);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
										  
    err_code = sd_ble_gap_tx_power_set(TX_POWER_LEVEL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_AD7746_init_t ad7746_init;
	memset(&ad7746_init, 0, sizeof(ad7746_init));
	
	ad7746_init.on_write_config_callback = ad7746_on_write_config_callback;
	
	err_code = ble_AD7746_init(&m_AD7746, &ad7746_init);
    APP_ERROR_CHECK(err_code);
	
//	ble_nus_init_t   nus_init;
//    memset(&nus_init, 0, sizeof(nus_init));
//    nus_init.data_handler = nus_data_handler;
//   
//    err_code = ble_nus_init(&m_nus, &nus_init);
//    APP_ERROR_CHECK(err_code);
}
/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
	ble_advdata_t scanrsp;
	
	// make sure service_init is called before advertising_init
	ble_uuid_t adv_uuids[] = {{AD7746_UUID_SERVICE, m_AD7746.uuid_type}, // uuid_type will be filled by ble_AD7746_init
								};
	//{BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE}};  
    
	// Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags                   = APP_ADV_GAP_FLAG;
//	advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
//	advdata.uuids_complete.p_uuids = adv_uuids;

	memset(&scanrsp,0,sizeof(scanrsp));	
	scanrsp.uuids_complete.uuid_cnt = sizeof(adv_uuids)/sizeof(adv_uuids[0]);
	scanrsp.uuids_complete.p_uuids = adv_uuids;
//	memset(&scanrsp,0,sizeof(scanrsp));	

    err_code = ble_advdata_set(&advdata, &scanrsp);
	if (err_code == NRF_ERROR_DATA_SIZE)
	{
		err_code = NRF_ERROR_DATA_SIZE;
	}
    APP_ERROR_CHECK(err_code);

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

/**@brief Function for handling the Device Manager events.
 *
 * @param[in] p_evt  Data associated to the device manager event.
 */
static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_event,
                                           ret_code_t        event_result)
{
    APP_ERROR_CHECK(event_result);

#ifdef BLE_DFU_APP_SUPPORT
    if (p_event->event_id == DM_EVT_LINK_SECURED)
    {
        app_context_load(p_handle);
    }
#endif // BLE_DFU_APP_SUPPORT

    return NRF_SUCCESS;
}
/**@brief Function for the Device Manager initialization.
 */
static void device_manager_init(void)
{
    uint32_t               err_code;
    dm_init_param_t        init_data;
    dm_application_param_t register_param;

    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    // Clear all bonded centrals if the Bonds Delete button is pushed.
    err_code = bsp_button_is_pressed(BOND_DELETE_ALL_BUTTON_ID,&(init_data.clear_persistent_data));
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_data);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
}

/**@brief    Function for handling the data from the Nordic UART Service.
 *
 * @details  This function will process the data received from the Nordic UART BLE Service and send
 *           it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        while(app_uart_put(p_data[i]) != NRF_SUCCESS);
    }
    while(app_uart_put('\n') != NRF_SUCCESS);
}
/**@snippet [Handling the data received over BLE] */




/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
#ifdef DEBUG_NRF_USER
/**@brief Function for error handling, which is called when an error has occurred.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name.
 */
 #include "ble_debug_assert_handler.h"
 
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    // This call can be used for debug purposes during application development.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover with a reset.
    //NVIC_SystemReset();
}
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
		if (line_num == 1371) {
				return;
		}
    app_error_handler(0xDEADBEAF, line_num, p_file_name);
}
#endif

int main(void)
{
    uint32_t err_code;

    // Initialize softdevice stack.
    ble_stack_init();
	
	// ps hold
		    NRF_GPIO->PIN_CNF[PSHOLD_PIN_NUMBER] =     \
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) \
      | (GPIO_PIN_CNF_DRIVE_S0S1     << GPIO_PIN_CNF_DRIVE_Pos) \
      | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)  \
      | (GPIO_PIN_CNF_INPUT_Disconnect  << GPIO_PIN_CNF_INPUT_Pos) \
      | (GPIO_PIN_CNF_DIR_Output      << GPIO_PIN_CNF_DIR_Pos);	
	
	NRF_GPIO->DIRSET = (1UL << PSHOLD_PIN_NUMBER); 
	NRF_GPIO->OUTSET = (1UL << PSHOLD_PIN_NUMBER); 

	
	
		// Initialize common modules
		scheduler_init();
		device_manager_init();
		timers_init(); ////////////////////////////////////

		// Initialize GAP and GATT 
		gap_params_init();
		// Initialize service
		services_init();	
		advertising_init();
	
		// Initialize application specific modules
		someother_module_init();
		
		conn_params_init();
	
		application_timers_start();
		
		// start advertising
		advertising_start();


		for(;;)
		{
			app_sched_execute();
			err_code = sd_app_evt_wait();
			APP_ERROR_CHECK(err_code);
		}
		
}
