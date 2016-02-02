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
	NORDIC nrf51822 QFN CHIP variants and S110 SoftDevice Configurations

	:::::: Nordic SoftDevice ---> S110 Ver 8.0.0

	nRF51822_xxAC IROM1 Start:0x18000 Size:0x28000(163k)   IRAM1... Start 0x20002000 Size:0x6000(32k)

	nRF51822_xxAA IROM1 Start:0x18000 Size:0x28000(163k)   IRAM1... Start 0x20002000 Size:0x4000(16k)

	Note: Soft device S110 Starts at Address 0x000000

	QFN nRF51822 Compatability ---> Hardware ID (HWID)
		
	QFAC Rev:3  FLASH:256k RAM:32k   HWID:0083
	
	QFAA Rev:1a FLASH:256k RAM:16k   HWID:001D
	QFAA Rev:2  FLASH:256k RAM:16k   HWID:0024 0044 003C
	QFAA Rev:3  FLASH:256k RAM:16k   HWID:0072
*/

/*
 ERROR...	180seconds Requires ---> Upon Expire Forces Power Down ---> tried to bump 2 Hours and Avoid Advertising Time Out Event however int > 180 causes a shutdown
 Note		Removed Powerdown option at Adv time Out however still forces shutdown upon event
 Note 		Commented out // advertising_start() to bypass shutdown issue (No bluetooth if NOT started

 ERROR		Added app_uart_fifo to run time environment (If buffer fills and not emptied quich enough the FIFO stops outputing and uart hangs)
 ERROR		After a period of time UART Hangs however App keeps working re push button interupt drives motor (REASON UNKNOWN)
 ERROR		Error Agrevated by FIFO Size may also be problem not using nrf51822xAC current processor is _AA
 
 ERROR		UART failure while in ATxx Command Mode. Note ---> No Screen Refresh UART FIFO que still dies after a period of time possibly ble related.
			Buttom interupt still runs ????

 ERROR		uP ADC Vbat rail initially reads high and slowly goes to steady state. C cludge facto applied to scale to measured DVM Battery voltage

 TODO...	Calibrate Inertial Sensors especially magnetometer
 
 TODO...	Inertial Quaternion needs to be scheduled with correct MPU_9150.DeltaT upadtes derived from time interval currently hard set to Zero

 TODO...	Run an interative LPF on accellerometer (Gravity=1) scaler ---> Maybe do the same on Magnetometer
 
 TODO...	Sort out Digital Motion Processor Routines including factory and user calibration
 
 TODO...	Driver for Serial Flash AT45DB161
 
 TODO...	Driver for SPI A2035 GPS Extraction and Parse NEMA (requires SPI Module variant shared with SFLASH SPI)
 
 TODO...	Fix gasGauge Manager (Readings need to be fixed) 
 
 TODO...	Generate PWM for Audio Tone Prompts
 
 TODO...	Define and Store GIS Way Points
 
 TODO...	Fix nRf51 processor Temperature Shutdown error is subroutine called

 TODO...	Iniatiate Watch Dog Timer and Pat_the_Dog in main()
 
 TODO...	Write Driver for SHA-1 EEPROM DS28e02 (Security Option APPLY - Hide from GitHub)
 
 TODO...	Do some further tests on Pressure/Altimeter and incorportae into GIS awarness tracking
 
 TODO...	Command-Control via Aux. USB-UART (Write PC app to Manage GIS and Client data sets)
 
 TODO...	SPI Parsed NEMA stream to UART at 4800Baud (Setup ATxx command for VT100 Overide/Initiation OF NEMA Stream)
 
 TODO...	Write a driver for SHA-1 EEPROM ds28e02
 Note		Security Restriction Apply ---> Keep Away from GitHub 
 
 TODO...	Investigate Bluetooth ble_nus Nordic UART service as alternate command control (Requires Further Applet Develeopement)
 
 TODO...	Expand Error Logging (Count errors) and display last active err_code or possibly show a small 1-5 que of last active errors
 
 TODO...	sensorADC_singleMeasure Battery Voltage Measurement stabalises after multiple reads --> Should be available upon first read ?????
 
 TODO...	iPhone Application via bluetooth (Fix up broadcast headers)
 
 */
  
//#define DEBUG							// Test to Place Application in DEBUG mode to allow error recording within app_error.h

//#define DEBUG_NRF_USER				// WARNING Disable in final product Release --- Forces endless loop trap upon system errors.

#define HARDWARE_1V0					// Current Hardware Version Dec-2015

#include <stdbool.h> 
#include <stdint.h>  					// for uint32_t etc.
#include <stdio.h>
#include <math.h>

#include "app_uart.h"
#include "app_error.h"
#include "app_button.h"
#include "app_timer.h"  				// for APP_TIMER_TICKS
#include "app_gpiote.h" 				// for APP_GPIOTE_INIT

#include "bsp.h"

#include <nrf_delay.h>					// ToDo Fix this error flag !!!!!!!!!!!!!!!!!!!! What is going on here????
#include "nrf.h"
#include "nrf_temp.h"
#include "nrf_soc.h"

//#include "nrf_drv_wdt.h"

#include "global.h"						// A set of global variable definitions
#include <string.h>  					// for memset()
#include "softdevice_handler.h"			// Bluetooth
#include "pstorage.h"
#include "ble_advertising.h"			// Bluetooth
#include "device_manager.h"
#include "ble_hci.h"  					// Bluetooth for set hci_status_code to sd_ble_gap_disconnect
#include "ble_conn_params.h"			// Bluetooth

#include "app_scheduler.h"  			// for scheduler APP_SCHED_INIT
#include "app_timer_appsh.h"  			// for app_timer_event_t

#include "nordic_common.h"  			// for UNUSED_PARAMETER

#include "nrf_assert.h" 				// for ASSERT

#include "ble_ad7746.h"					// Needed for ble functionallity. ---> Needs to be abstracted for general use.

//#include "AD7746.h"					// Differential Capacitor Sensor (Not avilable on this Hardware)

#include "sensorADC.h"					// ???????

#include "ds2401.h"						// Unique 1-Wire 48bit ID
#include "AT45_Flash.h"					// SPI Serial Flash 16Mbit
//#include "MPU_9150.h"  				// I2C Inertial Sensor 9-Axis Gyro Accel Magnetis + Temperature
//#include "inv_mpu.h"					// Inertial Sensor Drivers
#include "MPU_9250.h"  					// I2C Inertial Sensor 9-Axis Gyro Accel Magnetis + Temperature (EXTRACT FROM ADRINO SAMPLE CODE INCLUDES  AUX PRESS SENDSOR)

//#include "ble_nus.h"  				// Bluetooth nordic uart service currently not used may use later for GPS NEMA streaming etc

//#include "A2035H.h"						// GPS Hybrid Module ---> Configured for UART needs to be reworked for SPI operation

#include "MPL3115.h"					// I2C Peripheral Pressure, Altitude and Temperature Sensor
#include "ltc2943.h"					// I2C Gas Gauge Battery Monitor

#include "Communication.h"				// I2C and Other comm's

#define IS_SRVC_CHANGED_CHARACT_PRESENT  0                                          /**< Include or not the service_changed characteristic. */

#define DEVICE_NAME						"SCANE1V0"									/**< Name of device. Will be included in the advertising data ---> Append DS2401 ID */

#define DEVICE_APPEARANCE				 BLE_APPEARANCE_UNKNOWN						/**< Device appearance **/

// for ble_conn_param_init param
#define APP_TIMER_PRESCALER              0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS             (6+BSP_APP_TIMERS_NUMBER)                  /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE          4                                          /**< Size of timer operation queues. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to 
																						 first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)/**< Time between each call 30sec to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                          /**< Number of attempts before giving up the connection parameter negotiation. */

// for Service timer
#define SYSTEM_TIMER_INTERVAL			 APP_TIMER_TICKS(200, APP_TIMER_PRESCALER)	/**< sys measurement interval ticks === (200msec) 
																						 NOTE: If to small seems to screews up with TWI ???*/

// for GAP param init
#define MIN_CONN_INTERVAL                MSEC_TO_UNITS(9, UNIT_1_25_MS)           	/**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL                MSEC_TO_UNITS(300, UNIT_1_25_MS)           /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                    0                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                 MSEC_TO_UNITS(4000, UNIT_10_MS)            /**< Connection supervisory timeout (4 seconds). */
#define TX_POWER_LEVEL                   (0) 										/**< TX Power Level value. This will be set both in the TX Power service,
																						 in the advertising data, and also used to set the radio transmit power. */
//	to use button, define BUTTONS_NUMBER in custom_board.h
#define WAKEUP_BUTTON_ID                 0                                          /**< Button used to wake up the application. */
#define BOND_DELETE_ALL_BUTTON_ID        1                                          /**< Button used for deleting all bonded centrals during startup. */

//	ble_advertising_init param
#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */
#define APP_ADV_GAP_FLAG				BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE  

//	app scheduler module
#define SCHED_MAX_EVENT_DATA_SIZE       sizeof(app_timer_event_t)                   /**< Maximum size of scheduler events. Note that scheduler BLE stack events
																						 do not contain any data, as the events are being pulled from the stack in the event handler. */
#define SCHED_QUEUE_SIZE				10  										/**< Maximum number of events in the scheduler queue.  */

//	device manager module
#define SEC_PARAM_BOND					 1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES		BLE_GAP_IO_CAPS_NONE						/**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE			16											/**< Maximum encryption key size. */

#define MEASURESIZE						128											/**< Need to chase down Probably Diff Capacitor ADC or similar ??? > */
static float dataToSend[MEASURESIZE];

//static ble_uuid_t m_adv_uuids[1] = {{AD7746_UUID_SERVICE, BLE_UUID_TYPE_VENDOR_BEGIN}}; /**< Universally unique service identifiers. */

//	************************************************************
//
//		NRF51822 Port P0.xx IO Assignments SMARTCANE PCB 1V0
//
//	************************************************************

#define PSHOLD_PIN_NUMBER		(28U)				// Assert High 3V3 Power ---> Upon SW1 Long Push STM6601.pin4
#define SMART_RST_PIN_NUMBER	(01U)				// Assert HIGH Force a 3V3 Rail Shutdown of STM6601.pin2 via FET (Requires Physical Button Push to Iniatiate Power-Up)

#define PB_SW1_PIN_NUMBER		(06U)				// INPUT PBOUT SW1 Image of Push Button State (Normally High) STM6601.pin8
#define PB_SW2_PIN_NUMBER		(11U)				// INPUT SW2 Push Button (Normally High via 3V3 Pull-up) Active Low

#define MOTOR_PIN_NUMBER		(07U)				// Assert HIGH Enables 1V2 Linear Regulator to Drive Onboard 9000RPM Haptic Vibration Motor RT9030.pin3
#define AUX_MOTOR_PIN_NUMBER	(25U)				// Assert HIGH Provides Switched Ground Sink Path for 3V3 Rail via 33R Series Resistance

#define AUDIO_CNTRL_PIN_NUMBER	(23U)				// Assert LOW To ENABLE onboard Audio Amplifier used to Drive Off Board Speaker LM4800.pin5
#define AUDIO_SOURCE_PIN_NUMBER	(00U)				// Analog OUTPUT - AC Coupled Square Wave providing signal to Speaker Audio Amp LM4880.pin2

#define LED_RED_PIN_NUMBER		(30U)				// Assert LOW To Enable LED1 RED Diagnostic LED

#define GPS_NEN_PIN_NUMBER		(04U)				// Assert LOW GPS Not Enable - Disables Secondary Linear 3V3 Regulator running directly off VBAT
#define GPS_ONOFF_PIN_NUMBER	(13U)				// Assert HIGH-LOW-HIGH GPS Power Up-Down On-Off Sequencer A2035-H.pin18
#define GPS_NRST_PIN_NUMBER		(10U)				// Assert LOW  GPS Reset A2035-H.pin1

#define ADC_VBAT_PIN_NUMBER		(05U)				// Analog Signal tied to 150k//680k Voltage Divider Measuring VBAT ratio

#define GAUGE_NALCC_PIN_NUMBER	(12U)				// INPUT (INT) Gas Gauge Interupt I2C Programmable Alarm States LTC2943.pin6.
#define INERTIAL_INT_PIN_NUMBER	(03U)				// INPUT (INT) Inertial Sensor Interupt MPU9250.pin12
#define PRESS1_PIN_NUMBER		(29U)				// INPUT (INT) Pressure Sensor PRESS1 Interupt MPL3115.pin6
#define PRESS2_PIN_NUMBER		(02U)				// INPUT (INT) Pressure Sensor PRESS2 Interupt MPL3115.pin5
#define GPS_INT_PIN_NUMBER		(14U)				// INPUT (INT) GPS External Interupt A2035-H.pin19


static uint16_t m_conn_handle =  BLE_CONN_HANDLE_INVALID;			/** < Handle of the current connection. */
static dm_application_instance_t m_app_handle;						/** < Application identifier allocated by device manager */

static app_timer_id_t	m_sys_timer_id;								/** < system timer. */

#define MAX_TEST_DATA_BYTES		(4U)								/** < max number of test bytes per TX Burst to be used for tx and rx. ---> was 55 */
#define UART_TX_BUF_SIZE		128									/** < UART TX buffer size. Note 512 causes error in xxAA target ---> WARNING Must be 2^x */
#define UART_RX_BUF_SIZE		16									/** < UART RX buffer size.  ---> WARNING Must be 2^x  */

//nrf_drv_wdt_channel_id m_channel_id;
																	// Note... Melbourne == lat:-24.0000deg Lon:+135.0000deg Elev:0.00km

const float MagTNReference = 5.125;									// Australian Compass trueNorth magNorth field Compensation for Melbourne Australia
																	// ---> Add to Mag Compass bearing for True North
																	
const float MagDeclination = -12.000;								// Magnetic declination at Melbourne Australia ---> extracted from Map 

const float EarthMagnetic_uT = 60.00;								// Earths Magnetic field ranges from 25uT to 65uT ---> Melbourne=60uT

float	Magnetic[3];
float	MagMeanCal[3];
float	MagMaxCal[3];												// 
float	MagMinCal[3];												// 
float	MagCenterCal[3];											// Mean of (Max-Min)/2 ---> Axis Centroid based on Ambient Field

float	Gyro[3];
float	GyroIntegrator[3];
float	Gyro_ManualCal[3];											// The following registers are used within manual Gyro calibration

char	DS2401_ID[16];												// String to hold ---> DS2401 (4bits) + Device_ID
uint8_t MenuLevel=0;												// Initalise Default Menu Level to 00 --> Main Menu
float	Quaternion[4];												// Test Global Decleration to test Quaternion filter update operation
float	accel_ManualCal[3];											// The following registers are used within manual Acceleration calibration
float	ErrorCount[16];												// Background Error Counters
uint32_t sysTimeTick=0;												// Time Tick Counter Loaded within Timer Interupt ---> SYSTEM_TIMER_INTERVAL 
uint32_t sysLastTime=0;
uint32_t DogCount=0;												// DogCount ---> Background Heartbeat 
static bool finishedADC;
const float pi = 3.14159265358979323846f;
//uint32_t DiagCounter=0;

uint16_t ErrorCounter_UART_FIFO=0;
uint16_t ErrorCounter_UART_Comms=0;
uint16_t ErrorCounter[9];

uint32_t error_code_clone[3];

uint32_t line_num_clone=0;

uint16_t Count_VibroMotor=4;										// Auto Starts vibro motor upon sys timer startup for 400msec

char	ATxx[4];													// AT Command mode Ascii buffer

float pitch=0; float roll=0; float yaw=0;							// Also resides in global.h
uint16_t QuaternionCount=0;



int Process_0  = 0;													/* Initalise Bit Flags Process_0
	Process_0.0	= Flag to Activate Vibro Motor at Sys_Timer 200msec 
	Process_0.1	= Flag to Activate Vibro Motor at Sys_Timer 400msec 
	Process_0.1	= Flag to Activate Vibro Motor at Sys_Timer 800msec
	Process_0.3	= Flag to Activate Vibro Motor at Sys_Timer 1600msec 
	Process_0.4	= Flag to Activate Vibro Motor at Sys_Timer 3200msec 
	Process_0.5	= Flag to Activate Vibro Motor at Sys_Timer 6400msec
	Process_0.6	= 
	Process_0.7	= 
	Process_0.8	= LED Red Flag
	Process_0.9	= 
	Process_0.10 = 
	Process_0.11 = 
	Process_0.12 = 
	Process_0.13 = 
	Process_0.14 = 
	Process_0.15 = 
*/
int Process_1  = 0;													/* Initalise Bit Flags Process_1	
	Process_1.0	= Flag to bypass VT100 Mode and run ATxx cmd mode 
	Process_1.1	= Flag to transfer NEMA GPS Stream to UART
	Process_1.2	= 
	Process_1.3	= 
	Process_1.4	= 
	Process_1.5	= 
	Process_1.6	= 
	Process_1.7	= 
	Process_1.8	= 
	Process_1.9	= 
	Process_1.10 = 
	Process_1.11 = 
	Process_1.12 = 
	Process_1.13 = 
	Process_1.14 = 
	Process_1.15 = 
*/

int Process_2  = 0;													/* Initalise Bit Flags Process_2	
	Process_2.0	= PWM Tone Test Master Flag				
	Process_2.1	= PWM Tone Test 100msec
	Process_2.2	= 
	Process_2.3	= 
	Process_2.4	= 
	Process_2.5	= 
	Process_2.6	= Indicate       Revision 2 nRF51 Processor
	Process_2.7	= Indicate Valid Revision 3 nRF51 Processor			Note: Should Run with this Processor Revision
*/

int Process_3  = 0;													/* Initalise Bit Flags Process_3
	Process_3.0	= Flag to start ble advertising
	Process_3.1	= Flag to latch ble advertising event as oneshot
	Process_3.2	= 
	Process_3.3	= 
	Process_3.4	= 
	Process_3.5	= 
	Process_3.6	= 
	Process_3.7	= 
*/

int Process_4  = 0;													/* Initalise Bit Flags Process_4	
	Process_4.0	= Flag to indicate Button 0 Asserted
	Process_4.1	= Flag to indicate Button 1 Asserted
	Process_4.2	= 
	Process_4.3	= 
	Process_4.4	= 
	Process_4.5	= 
	Process_4.6	= 
	Process_4.7	= 
*/

int Process_5  = 0;													/* Initalise Bit Flags Process_5	
	Process_5.0	= Flag Processor xxAC Flash:256k Ram:32k
	Process_5.1	= Flag MPU9250 Inertial Sensor Present
	Process_5.2	= Flag MPL3115 Pressure Sensor Present
	Process_5.3	= Flag LTC2943 GasGauge Sensor Present
	Process_5.4	= Flag A2035 GPS Sensor Present
	Process_5.5	= Flag SFlash Memory Present
	Process_5.6	= Flag EEPROM Present
	Process_5.7	= Flag System Temperatures OK
*/
int Process_6  = 0;													/* Initalise Bit Flags Process_6	
	Process_6.0	= 
	Process_6.1	= 
	Process_6.2	= 
	Process_6.3	= 
	Process_6.4	= 
	Process_6.5	= 
	Process_6.6	= 
	Process_6.7	= 
*/

int Process_7  = 0;													/* Initalise Bit Flags Process_7

	Process_7.0	= Flag to indicate cane in NORMAL user allignment - Buttons UP and cane angled downwards at ~45deg
	Process_7.1	= Buttons 90deg to RIGHT
	Process_7.2	= Buttons 90deg to LEFT
	Process_7.3	= Buttons UP 
	Process_7.4	= Cane UP
	Process_7.5	= Cane DOWN
	Process_7.6	= 
	Process_7.7	= 
*/

int iProcess_0 = 0;													/* Initalise Bit Flags iProcess_0 ---> Reserved for use by INTERUPT service routines


	iProcess_0.0	= Active System Timer Interupt Handler. Used to trap and bypass multi-entry.
	iProcess_0.1	= 
	iProcess_0.2	= 
	iProcess_0.3	= 
	iProcess_0.4	= 
	iProcess_0.5	= 
	iProcess_0.6	= 
	iProcess_0.7	= 
	iProcess_0.8	= 
	iProcess_0.9	= 
	iProcess_0.10	= 
	iProcess_0.11	= 
	iProcess_0.12	= 
	iProcess_0.13	= 
	iProcess_0.14	= 
	iProcess_0.15	= 
*/

void setbit(int *Addr, uint8_t Operator) {							// Set   bit at Process_x based on Operator (0..7)---> Single bit
	if(Operator<=16) {
		*Addr  = *Addr | (int) pow(2, Operator);						// Bitwise OR
	}
}	

void clrbit(int *Addr, uint8_t Operator) {							// Clear bit at Process_x bassed on Operator (0..7)---> Single bit 
	if(Operator<=16){
		*Addr  = *Addr & ~((int)(pow(2, Operator)));					// Bitwise AND of inverted Operator
	}
}

bool testbit(int *Addr, uint8_t Operator) {							// Test  bit at Process_x bassed on Operator (0..7)---> Single bit return true/false
	if(Operator<=16){
		if (*Addr & (int) pow(2,Operator)) {
			return true;												// True
		} else {	
			return false;												// False
		}
	}
	return false;
}

float BearingTN(float lat1, float lon1, float lat2, float lon2) {	// Function to derive True-North Bearing between two Latitude-Londitude GPS coordinates

	lat1 *= pi/180;													// Convert degrees to radians
	lon1 *= pi/180;
	lat2 *= pi/180;
	lon2 *= pi/180;
	
	float x  = cos(lat2)*sin(lon2-lon1);
	float y = cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)* cos(lon2-lon1);

	//	printf("%0.6f  %0.6f", x,y);

	return atan2(x, y)*180/pi;										// Return True North Bearing in degrees
}
float BearingMag(float lat1, float lon1, float lat2, float lon2) {	// Function to derive Magnetic-North Bearing between two Latitude-Londitude GPS coordinates
	float BearingTrueNorth = BearingTN(lat1, lon1, lat2, lon2);		
	return BearingTrueNorth + MagTNReference;
}

float LPFilter(float Out, float In, float percent) {				// Low Pass Filter Function
	Out = percent/100*Out + (1-(percent/100))*In;
	return Out;
}

void led_red_ON(void){												// Function Shortcut to Assert   Diagnostic LED
	nrf_gpio_pin_set(BSP_LED_0);
	setbit(&Process_0,8);
	
}
void led_red_OFF(void){												// Function Shortcut to Deassert Diagnostic LED
	nrf_gpio_pin_clear(BSP_LED_0);
	clrbit(&Process_0,8);
}
void led_red_TOGGLE(void){											// Function Shortcut to Toggle Assertion of Diagnostic LED
	if(testbit(&Process_0,8)){
		led_red_OFF();
	}
	else { 
		led_red_ON();
	}
}
void ad7746_on_write_config_callback (ble_AD7746_t *p_ad7746, 
	CONFIG_bytes_t *p_config, uint8_t len) {						// Write bytes of config ble device AD7746 Capacitor Sensor --> (Redirected to other sensors)
		

/*	This callback will be active upon write request by BLE softdevice to write config data to CONFIG_CHARacteristic*/		
	  
}
	
static void button_event_handler(bsp_event_t event) {
	
/**	@brief 		Function to handle GPIO Button Interupts
	@details 	
	@note  		Currently two (2) Push buttons enabled on SmartCane_1V0
	@ref 		*/	


	 switch(event) {
		 case BSP_EVENT_KEY_0:										// SmartCane REAR SWITCH
			setbit(&Process_4,0);
			setbit(&Process_0,4);									// Flag to start Vibro Motor assert at System timer handler
			break;
		 
		 case BSP_EVENT_KEY_1:										// SmartCane FRONT SWITCH
			setbit(&Process_4,0);
		 
			setbit(&Process_0,2);									// Flag to start Vibro Motor assert at System timer handler
//			nrf_gpio_pin_set(MOTOR_PIN_NUMBER);		 				// nrf_gpio_pin_toggle(MOTOR_PIN_NUMBER);
//			nrf_delay_ms(1000);										// Commented to negate interupt stall
//			nrf_gpio_pin_clear(MOTOR_PIN_NUMBER);
			break;
		 
		 default:
			ErrorCounter[2] += 1;
			break;
	 }
}

void uart_error_handle (app_uart_evt_t * p_event) {					// UART event handler includes non Error Events
	
/**	@brief 		Function to handle UART error events 
	@details 	
	@note  		ToDo Should include some error counters here 
	@ref 		*/	

    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR) {		// UART Communications Error Event
        APP_ERROR_HANDLER(p_event->data.error_communication);
		ErrorCounter_UART_Comms += 1;
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR) {			// UART FIFO Error Event
        APP_ERROR_HANDLER(p_event->data.error_code);
		ErrorCounter_UART_FIFO += 1;
    }
	else if (p_event->evt_type == APP_UART_DATA_READY) {			// UART Rx Data Event
 	ErrorCounter[3] += 1;
    }
	else if (p_event->evt_type == APP_UART_TX_EMPTY) {				// UART Tx Empty Event
    }
	else if (p_event->evt_type == APP_UART_DATA) {					// UART Data Rx Event & Data present (NOT used in FIFO Mode)
    }
}

static void UART_VT100_Main_Menu() {								// $$$$$$ TOP LEVEL MENU-x or X								$$$$$$
	
/**	@brief 		Function to load a single top level VT100 Terminal Menu. 
	@details 	Transmitts VT100 ESC Sequenceone character to clear screen and load menu data.
	@note  		ASCII DEC 27 == x1B    VT100 ClrScreen == <ESC>[2J    Home == ESC[H    Cursor Location === <ESC>[{ROW};{COLUMN}H
	@ref 		*/		
	
	MenuLevel=00;

	printf("\r\n");
	printf("\x1B[2J");													// VT100 CLR SCREEN
	printf("\x1B[H");													// VT100 CURSOR HOME

	nrf_delay_ms(50);

	printf("\x1B[01;10H  GDV-UoM SMARTCANE MAIN MENU");
	printf("\x1B[01;45HDEVICE ID = ");
	printf("\x1B[02;45H   STATUS = ");

	nrf_delay_ms(50);

	printf("\x1B[04;10H  1... All Sensors");
	printf("\x1B[06;10H  2... GPS Global Position");
	printf("\x1B[08;10H  3... Inertial Sensors");
	printf("\x1B[10;10H  4... Altitude and Temperature");

	nrf_delay_ms(50);

	printf("\x1B[12;10H  5... Memory Functions");
	printf("\x1B[14;10H  6... Power Management");
	printf("\x1B[16;10H  7... Cane Diagnostics");
	
	nrf_delay_ms(50);
	
	printf("\x1B[18;10H  8... System Diagnostics");
	printf("\x1B[20;10H  9... Peripheral Diagnostics");

	nrf_delay_ms(50);
	
	printf("\x1B[22;10H  A... Bluetooth Diagnostics");

	printf("\x1B[24;10H  ?... Help");
}


static void UART_VT100_Menu_1()	{									// $$$$$$ ALL SENSORS MENU-1								$$$$$$
	
/** @brief 		Function to load Top level Main Menu to VT100 Screen. 
	@details 	Loads VT100 ESC Sequenceone character to clear screen and load menu data.
	@note  		ASCII DEC 27 == x1B    VT100 ClrScreen == <ESC>[2J    Home == ESC[H    Cursor Location === <ESC>[{ROW};{COLUMN}H
	@ref 		VT100 Terminal Emulation Nominal Baud:115200 */		
	
	MenuLevel=10;

	printf("\x1B[2J");														// VT100 CLR SCREEN
	printf("\x1B[H");														// VT100 CURSOR HOME
	printf("\x1B[01;05H GDV-UoM SMARTCANE MENU-1  ALL SENSORS and SYSTEMS");
	printf("\x1B[04;05H GRAVITY g");
	printf("\x1B[05;05H ACC-X = ");
	printf("\x1B[06;05H ACC-Y = ");
	printf("\x1B[07;05H ACC-Z = ");
	printf("\x1B[08;05H ACC-M = ");

	nrf_delay_ms(50);

	printf("\x1B[04;30H GYROSCOPE deg");
	printf("\x1B[05;30H GYRO-X = ");
	printf("\x1B[06;30H GYRO-Y = ");
	printf("\x1B[07;30H GYRO-Z = ");

	nrf_delay_ms(50);

	printf("\x1B[04;55H COMPASS ");
	printf("\x1B[05;55H Compass-X = ");
	printf("\x1B[06;55H Compass-Y = ");
	printf("\x1B[07;55H Compass-Z = ");

	nrf_delay_ms(50);
	 
	printf("\x1B[11;05H GPS");
	printf("\x1B[12;05H Latitude   = ");
	printf("\x1B[13;05H Longditude = ");
	printf("\x1B[14;05H Time       = ");

	nrf_delay_ms(50);
	 
	printf("\x1B[11;30H TEMPERATURE");
	printf("\x1B[12;30H ProcessorT = ");
	printf("\x1B[13;30H CompassT   = ");
	printf("\x1B[14;30H PressureT  = ");
	printf("\x1B[15;30H GasGaugeT  = ");
	
	nrf_delay_ms(50);
	
	printf("\x1B[11;55H HEIGHT");
	printf("\x1B[12;55H Pressure  = ");
	printf("\x1B[13;55H Altitude  = ");
	

	nrf_delay_ms(50);
		
	printf("\x1B[24;05H  X... exit    ?...Help");
}	


static void UART_VT100_Menu_2() {									// $$$$$$ GPS GLOBAL POSITION MENU-2  						$$$$$$
	
/** @brief 		Function to load GPS GLOBAL POSITION MENU to VT100 Screen. 
	@details 	Loads VT100 ESC Sequenceone character to clear screen and load menu data.
	@note  		ASCII DEC 27 == x1B    VT100 ClrScreen == <ESC>[2J    Home == ESC[H    Cursor Location === <ESC>[{ROW};{COLUMN}H
	@ref 		VT100 Terminal Emulation Nominal Baud:115200 */		
	
	MenuLevel=20;

	printf("\x1B[2J");													// VT100 CLR SCREEN
	printf("\x1B[H");													// VT100 CURSOR HOME
	printf("\x1B[01;05H GDV-UoM SMARTCANE MENU-2  GPS GLOBAL POSITION SENSOR");

	nrf_delay_ms(50);
			 
	printf("\x1B[04;05H GPS");
	printf("\x1B[05;05H Latitude   = ");
	printf("\x1B[06;05H Longditude = ");
	printf("\x1B[07;05H Height     = ");
	printf("\x1B[08;05H Time       = ");

	nrf_delay_ms(50);
	 
	printf("\x1B[05;30H SatNumber = ");
	printf("\x1B[06;30H Spare     = ");
	printf("\x1B[07;30H Spare     = ");
	printf("\x1B[08;30H Spare     = ");
	printf("\x1B[09;30H Spare     = ");
	
	nrf_delay_ms(50);
		
  printf("\x1B[24;05H  X... exit    ?...Help");
}	


static void UART_VT100_Menu_3()	{									// $$$$$$ INERTIAL SENSOR MENU-3  							$$$$$$
/** @brief 		Function to load INERTIAL SENSOR MENU to VT100 Screen. 
	@details 	Loads VT100 ESC Sequenceone character to clear screen and load menu data.
	@note  		ASCII DEC 27 == x1B    VT100 ClrScreen == <ESC>[2J    Home == ESC[H    Cursor Location === <ESC>[{ROW};{COLUMN}H
	@ref 		VT100 Terminal Emulation Nominal Baud:115200 */		
	
	MenuLevel=30;

	printf("\x1B[2J");														// VT100 CLR SCREEN
	printf("\x1B[H");														// VT100 CURSOR HOME
	printf("\x1B[01;05H GDV-UoM SMARTCANE MENU-3  INERTIAL SENSOR MENU");

	nrf_delay_ms(50);
			 
	printf("\x1B[03;05H MAGNETOMETER uT");
	printf("\x1B[04;05H Mag-X = ");
	printf("\x1B[05;05H Mag-Y = ");
	printf("\x1B[06;05H Mag-Z = ");
	printf("\x1B[07;05H Magxy = ");
	printf("\x1B[08;05H Magxxy= ");

	nrf_delay_ms(50);
	 
	printf("\x1B[03;30H GRAVITY g");
	printf("\x1B[04;30H Grav-X  = ");
	printf("\x1B[05;30H Grav-Y  = ");
	printf("\x1B[06;30H Grav-Z  = ");
	printf("\x1B[07;30H Grav-M  = ");

	nrf_delay_ms(50);

	printf("\x1B[03;55H GYROSCOPE deg/sec");
	printf("\x1B[04;55H Pitch  = ");
	printf("\x1B[05;55H Roll   = ");
	printf("\x1B[06;55H Yaw    = ");


	nrf_delay_ms(50);
	
	printf("\x1B[10;55H GYROSCOPE Integrator");
	printf("\x1B[11;55H Pitch  = ");
	printf("\x1B[12;55H Roll   = ");
	printf("\x1B[13;55H Yaw    = ");


	nrf_delay_ms(50);

	printf("\x1B[10;05H MADGWICK QUATERNION");
	printf("\x1B[11;05H Quat0 = ");
	printf("\x1B[12;05H Quat1 = ");
	printf("\x1B[13;05H Quat2 = ");
	printf("\x1B[14;05H Quat3 = ");
	printf("\x1B[15;05H QuatM = "); 

	nrf_delay_ms(50);

	printf("\x1B[10;30H QUATERNION P-R-Y");
	printf("\x1B[11;30H Q pitch = ");
	printf("\x1B[12;30H Q roll  = ");
	printf("\x1B[13;30H Q yaw   = ");

	nrf_delay_ms(50);
	
	printf("\x1B[17;05H COMPASS BEARING");
	printf("\x1B[18;05H MagDeg  = ");
	printf("\x1B[19;05H Bearing = ");

	nrf_delay_ms(50);	

	printf("\x1B[15;55H Gyro0  = ");
	printf("\x1B[16;55H Gyro1  = ");
	printf("\x1B[17;55H Gyro2  = ");
	
	nrf_delay_ms(50);
		
	printf("\x1B[19;55H ManCal0= ");
	printf("\x1B[20;55H ManCal1= ");
	printf("\x1B[21;55H ManCal2= ");
	
	printf("\x1B[23;05H  SysTimeTick = ");
	printf("\x1B[24;05H  X... exit    ?...Help");
}


static void UART_VT100_Menu_4() {									// $$$$$$ ALTITUDE AND PRESSURE MENU-4						$$$$$$
	
/**	@brief 		Function to load PRESSURE AND ALTITUDE MENU to VT100 Screen. 
	@details 	Loads VT100 ESC Sequenceone character to clear screen and load menu data.
	@note  		ASCII DEC 27 == x1B    VT100 ClrScreen == <ESC>[2J    Home == ESC[H    Cursor Location === <ESC>[{ROW};{COLUMN}H
	@ref 		VT100 Terminal Emulation Nominal Baud:115200 */		
		
	MenuLevel=40;
		
	printf("\x1B[2J");										// VT100 CLR SCREEN
	printf("\x1B[H");												// VT100 CURSOR HOME
	printf("\x1B[01;05H GDV-UoM SMARTCANE MENU-4  PRESSURE, ALTITUDE & TEMPERATURE");

	printf("\x1B[05;05H PRESSURE");
	printf("\x1B[06;05H Abs Pressure = ");
	printf("\x1B[07;05H ZeroRef      = ");
	printf("\x1B[08;05H Temperature  = ");

	nrf_delay_ms(55);

	printf("\x1B[05;40H ALTITUDE");
	printf("\x1B[06;40H Altitude    = ");
	printf("\x1B[07;40H ZeroRef     = ");
	printf("\x1B[08;40H 1HrRelative = ");

	nrf_delay_ms(55);

	printf("\x1B[25;05H  X... exit    ?...Help");
}	


static void UART_VT100_Menu_5()	{									// $$$$$$ SFLASH EEPROM SoC_FLASH MEMORY MANAGEMENT MENU-5	$$$$$$
	
/**	@brief 		Function to load  SFLASH EEPROM SoC_FLASH MEMORY MANAGEMENT MENU to VT100 Screen. 
	@details 	Loads VT100 ESC Sequenceone character to clear screen and load menu data.
	@note  		ASCII DEC 27 == x1B    VT100 ClrScreen == <ESC>[2J    Home == ESC[H    Cursor Location === <ESC>[{ROW};{COLUMN}H
	@ref 		VT100 Terminal Emulation Nominal Baud:115200 */		
		
	MenuLevel=50;
	
	printf("\x1B[2J");												// VT100 CLR SCREEN
	printf("\x1B[H");												// VT100 CURSOR HOME
	printf("\x1B[01;05H GDV-UoM SMARTCANE MENU-5 SFLASH EEPROM SoC_FLASH MEMORY MANAGEMENT MENU");
	 
	printf("\x1B[05;05H SFLASH");
	printf("\x1B[06;05H Device ID   = ");
	printf("\x1B[07;05H Percent Use = ");
	printf("\x1B[08;05H Spare       = ");

	nrf_delay_ms(50);
	 
	printf("\x1B[05;30H EEPROM");
	printf("\x1B[06;30H Device ID   = ");
	printf("\x1B[07;30H Percent Use = ");
	printf("\x1B[08;30H Spare       = ");

	nrf_delay_ms(50);

	printf("\x1B[05;55H ARM SoC");
	printf("\x1B[06;55H Device ID   = ");
	printf("\x1B[07;55H Percent Use = ");
	printf("\x1B[08;55H Spare       = ");

	nrf_delay_ms(50);		

	printf("\x1B[25;05H  X... exit    ?...Help");
}	




static void UART_VT100_Menu_6() {		    						// $$$$$$ POWER MANAGEMENT MENU-6							$$$$$$
	
/** @brief 		Function to load POWER MANAGEMENT MENU to VT100 Screen. 
	@details 	Loads VT100 ESC Sequenceone character to clear screen and load menu data.
	@note  		ASCII DEC 27 == x1B    VT100 ClrScreen == <ESC>[2J    Home == ESC[H    Cursor Location === <ESC>[{ROW};{COLUMN}H
	@ref 		VT100 Terminal Emulation Nominal Baud:11520	 */
		
	MenuLevel=60;
	
	printf("\x1B[2J");														// VT100 CLR SCREEN
	printf("\x1B[H");														// VT100 CURSOR HOME
	printf("\x1B[01;05H GDV-UoM SMARTCANE MENU-6 POWER MANAGEMENT MENU");

	nrf_delay_ms(50);
			 
	printf("\x1B[05;05H Gauge Measurements");
	printf("\x1B[06;05H Batt Voltage  = ");
	printf("\x1B[07;05H Batt Current  = ");
	printf("\x1B[08;05H Batt Status   = "); 
	printf("\x1B[09;05H Batt Capacity = ");
	printf("\x1B[10;05H Batt Temp     = ");

	nrf_delay_ms(50);
	 
	printf("\x1B[05;50H SoC ADC rail");
	printf("\x1B[06;50H Vbatt     = ");
	printf("\x1B[07;50H 3V3 Rail  = ");
	printf("\x1B[08;50H Spare     = ");

	nrf_delay_ms(50);

	printf("\x1B[25;05H  X... exit    ?...Help");
}	


static void UART_VT100_Menu_7() {		    						// $$$$$$ CANE DIAGNOSTIC MENU-7							$$$$$$
	
/**	@brief 		Function to load CANE DIAGNOSTIC MENU to VT100 Screen. 
	@details 	Loads VT100 ESC Sequenceone character to clear screen and load menu data.
	@note  		ASCII DEC 27 == x1B    VT100 ClrScreen == <ESC>[2J    Home == ESC[H    Cursor Location === <ESC>[{ROW};{COLUMN}H
	@ref 		VT100 Terminal Emulation Nominal Baud:115200 */	
		
	MenuLevel=70;

	printf("\x1B[2J");													// VT100 CLR SCREEN
	printf("\x1B[H");													// VT100 CURSOR HOME
	printf("\x1B[01;05H GDV-UoM SMARTCANE MENU-7 CANE DIAGNOSTIC MENU");
	 
	printf("\x1B[05;05H Configurations");
	printf("\x1B[06;05H Way Points  = ");
	printf("\x1B[07;05H Home        = ");
	printf("\x1B[08;05H Spare       = ");

	nrf_delay_ms(50);
	 
	printf("\x1B[05;30H SPARE");
	printf("\x1B[06;30H Spare = ");
	printf("\x1B[07;30H Spare = ");
	printf("\x1B[08;30H Spare = ");

	nrf_delay_ms(50);

	printf("\x1B[05;55H SPARE");
	printf("\x1B[06;55H Spare = ");
	printf("\x1B[07;55H Spare = ");
	printf("\x1B[08;55H Spare = ");

	nrf_delay_ms(50);		

	printf("\x1B[24;05H  X... exit    ?...Help");
}	


static void UART_VT100_Menu_8() {		    						// $$$$$$ SYSTEM DIAGNOSTIC MENU-8							$$$$$$
	
/**	@brief 		Function to load SYSTEM DIAGNOSTIC MENU to VT100 Screen. 
	@details 	Loads VT100 ESC Sequenceone character to clear screen and load menu data.
	@note  		ASCII DEC 27 == x1B    VT100 ClrScreen == <ESC>[2J    Home == ESC[H    Cursor Location === <ESC>[{ROW};{COLUMN}H
	@ref		*/
	
	MenuLevel=80;

	printf("\x1B[2J");														// VT100 CLR SCREEN
	printf("\x1B[H");														// VT100 CURSOR HOME
	printf("\x1B[01;05H GDV-UoM SMARTCANE MENU-8 SYSTEM DIAGNOSTIC MENU");

	nrf_delay_ms(50);

	printf("\x1B[05;05H KEY COMMANDS");
	printf("\x1B[06;05H a = calibrateMPU9250");
	printf("\x1B[07;05H b = ZERO Manual Cal's");
	printf("\x1B[08;05H c = Strobe Vibro 1sec");

	nrf_delay_ms(50);

	printf("\x1B[09;05H d = Start Timer");
	printf("\x1B[10;05H e = StrobeRedLED");
	printf("\x1B[11;05H f = Strobe PWM Tone");
	printf("\x1B[12;05H g = Bluetooth ADVERT");
	
	nrf_delay_ms(50);
	
	printf("\x1B[13;05H h = ATxx Command Menu");
	printf("\x1B[14;05H j = Clr Error Counters");
	printf("\x1B[15;05H k = Clr Gyro Integrators");
	printf("\x1B[16;05H l = Calibrate Magnetometer 25sec");
	
	nrf_delay_ms(50);
	 
	printf("\x1B[05;34H ERROR COUNTERS");
	printf("\x1B[06;34H ErrCount 00= ");
	printf("\x1B[07;34H ErrCount 01= ");
	printf("\x1B[08;34H ErrCount 02= ");
	printf("\x1B[09;34H ErrCount 03= ");
	
	nrf_delay_ms(50);

	printf("\x1B[10;34H ErrCount 04= ");
	printf("\x1B[11;34H ErrCount 05= ");
	printf("\x1B[12;34H ErrCount 06= ");
	printf("\x1B[13;34H ErrCount 07= ");
	printf("\x1B[14;34H ErrCount 08= ");

	nrf_delay_ms(50);
	
	printf("\x1B[16;34H UART Err  = ");
	printf("\x1B[17;34H UART FIFO = ");
	
	nrf_delay_ms(50);
	
	printf("\x1B[05;58H SPARE");
	printf("\x1B[06;58H Spare = ");
	printf("\x1B[07;58H Spare = ");
	printf("\x1B[08;58H Spare = ");

	nrf_delay_ms(10);		

	printf("\x1B[24;05H  X... exit    ?...Help");
}	

static void UART_VT100_Menu_9() {		    						// $$$$$$ PERIPHERAL DIAGNOSTIC MENU-9						$$$$$$
	
/**	@brief 		Function to load SYSTEM DIAGNOSTIC MENU to VT100 Screen. 
	@details 	Loads VT100 ESC Sequenceone character to clear screen and load menu data.
	@note  		ASCII DEC 27 == x1B    VT100 ClrScreen == <ESC>[2J    Home == ESC[H    Cursor Location === <ESC>[{ROW};{COLUMN}H
	@ref		*/
	
	MenuLevel=90;

	printf("\x1B[2J");														// VT100 CLR SCREEN
	printf("\x1B[H");														// VT100 CURSOR HOME
	printf("\x1B[01;05H GDV-UoM SMARTCANE MENU-8 PERIPHERAL DIAGNOSTIC MENU");

	nrf_delay_ms(50);

	printf("\x1B[05;05H nRF51822 ");
	printf("\x1B[06;05H QFN AC       ");
	if(testbit(&Process_5,0)) printf("Pass     ");
	else printf("---> FAIL");
	
	printf("\x1B[07;05H uP Flash     %dk", (uint16_t) NRF_FICR->CODESIZE);										// uP Flash size==0x0100 --> 256k
	
	printf("\x1B[08;05H uP Ram       %dk", (uint16_t) NRF_FICR->NUMRAMBLOCK * (NRF_FICR->SIZERAMBLOCKS / 1024));	// uP Ram size==0x0020 --> 32k

	nrf_delay_ms(50);

	printf("\x1B[09;05H Inertial     ");
	if(testbit(&Process_5,1)) printf("Pass     ");
	else printf("---> FAIL");

	printf("\x1B[10;05H Pressure     ");
	if(testbit(&Process_5,2)) printf("Pass     ");
	else printf("---> FAIL");
	
	printf("\x1B[11;05H Gas Gauge    ");
	if(testbit(&Process_5,3)) printf("Pass     ");
	else printf("---> FAIL ");
	
	printf("\x1B[12;05H GPS          ");
	if(testbit(&Process_5,4)) printf("Pass     ");
	else printf("---> FAIL");
	
	nrf_delay_ms(50);
	
	printf("\x1B[13;05H SFlash       ");
	if(testbit(&Process_5,5)) printf("Pass     ");
	else printf("---> FAIL");
	
	printf("\x1B[14;05H EEPROM       ");
	if(testbit(&Process_5,6)) printf("Pass     ");
	else printf("---> FAIL");
	
	printf("\x1B[15;05H Temperature  ");
	if(testbit(&Process_5,7)) printf("Pass     ");
	else printf("---> FAIL");

	nrf_delay_ms(50);		

	printf("\x1B[24;05H  X... exit    ?...Help");
}	
static void UART_VT100_Menu_A() {		    						// $$$$$$ BLUETOOTH DIAGNOSTIC MENU-A						$$$$$$
	
/*	@brief 		Function to load Bluetooth DIAGNOSTIC MENU to VT100 Screen. 
 *  @details 	Loads VT100 ESC Sequenceone character to clear screen and load menu data.
 *  @note  		ASCII DEC 27 == x1B    VT100 ClrScreen == <ESC>[2J    Home == ESC[H    Cursor Location === <ESC>[{ROW};{COLUMN}H
 *  @ref 		VT100 Terminal Emulation Nominal Baud:115200 */		
		
	MenuLevel=100;
	
	printf("\x1B[2J");														// VT100 CLR SCREEN
	printf("\x1B[H");														// VT100 CURSOR HOME
	printf("\x1B[01;05H GDV-UoM SMARTCANE MENU-9 BLUETOOTH DIAGNOSTIC MENU");
	
	nrf_delay_ms(50);

	printf("\x1B[05;05H EVENT COUNTERS");
	printf("\x1B[06;05H Event-1 = ");
	printf("\x1B[07;05H ");
	printf("\x1B[08;05H ");

	nrf_delay_ms(50);
	
	printf("\x1B[09;05H ");
	printf("\x1B[10;05H ");
	printf("\x1B[11;05H ");
	printf("\x1B[12;05H ");
	printf("\x1B[13;05H ");

	nrf_delay_ms(50);
	 
	printf("\x1B[05;34H ERROR COUNTERS");
	printf("\x1B[06;34H ErrCount 01= ");
	printf("\x1B[07;34H ErrCount 02= ");
	printf("\x1B[08;34H ErrCount 03= ");

	nrf_delay_ms(50);
	
	printf("\x1B[09;34H ErrCount 04= ");
	printf("\x1B[10;34H ErrCount 05= ");
	printf("\x1B[11;34H ErrCount 06= ");
	printf("\x1B[12;34H ErrCount 07= ");

	nrf_delay_ms(50);
	
	printf("\x1B[13;34H ErrCount 08= ");
	printf("\x1B[14;34H ErrCount 09= ");
	printf("\x1B[15;34H ErrCount 10= ");
	printf("\x1B[16;34H ErrCount 11= ");
	
	nrf_delay_ms(50);

	printf("\x1B[05;59H SPARE");
	printf("\x1B[06;59H Spare = ");
	printf("\x1B[07;59H Spare = ");
	printf("\x1B[08;59H Spare = ");

	nrf_delay_ms(50);		

	printf("\x1B[24;05H  X... exit    ?...Help");
}	


static void UART_VT100_Help_Menu() {								// $$$$$$ SCREEN HELP MENU-?								$$$$$$
	
/* @brief 		Function to load Screen Help MENU to VT100 Screen. 
 *  @details 	Loads VT100 ESC Sequenceone character to clear screen and load menu data.
 *  @note  		ASCII DEC 27 == x1B    VT100 ClrScreen == <ESC>[2J    Home == ESC[H    Cursor Location === <ESC>[{ROW};{COLUMN}H
 *  @ref 		
 */	
	MenuLevel=110;

	printf("\x1B[2J");													// VT100 CLR SCREEN
	printf("\x1B[H");													// VT100 CURSOR HOME
	printf("\x1B[01;05H GDV-UoM SMARTCANE MENU-? Help MENU");
	 
	printf("\x1B[05;05H HELP CODES");
	printf("\x1B[06;05H Way Points  = ");
	printf("\x1B[07;05H Home        = ");
	printf("\x1B[08;05H Spare       = ");

	nrf_delay_ms(50);
	 
	printf("\x1B[05;30H SPARE");
	printf("\x1B[06;30H Spare = ");
	printf("\x1B[07;30H Spare = ");
	printf("\x1B[08;30H Spare = ");

	nrf_delay_ms(50);

	printf("\x1B[05;55H SPARE");
	printf("\x1B[06;55H Spare = ");
	printf("\x1B[07;55H Spare = ");
	printf("\x1B[08;55H Spare = ");

	nrf_delay_ms(50);		

	printf("\x1B[24;05H  X... exit    ?...Help");
}	


static void UART_VT100_Refresh_Data_Main_Menu()	{					// $$$$$$ MAIN MENU						DATA REFRESH		$$$$$$
	
/** @brief 		Function to load MAIN MENU Refresh Data to VT100 Screen.  */	
	if (testbit(&Process_1, 0)==false) {
		printf("\x1B[01;58H %s %s", DEVICE_NAME, DS2401_ID);				// Device Name + Device ID

		printf("\x1B[02;58H READY");										// Self Test Status ToDo

		printf("\x1B[01;78H");												// Park VT100 cursor at row 01 column 78	// Park VT100 cursor at row 01 column 78

	}	
}


static void UART_VT100_Refresh_Data_All_Sensors() {					// $$$$$$ ALL SENSORS MENU 				DATA REFRESH MENU-1	$$$$$$
	
/** @brief 		Function to load ALL SENSOR Refresh Data MENU-1 to VT100 Screen.  */	
	
	if (testbit(&Process_1, 0)==false) {
		int value;
		float Acc[3];
		double MagGravity;
		float temp;
		
		readAccelFloatMG(Acc);													// ACCELERATION - GRAVITY

		printf("\x1B[05;14H%+2.2f  ", Acc[0]);
		printf("\x1B[06;14H%+2.2f  ", Acc[1]);
		printf("\x1B[07;14H%+2.2f  ", Acc[2]);									// Z Axis direction swap as chip mounted on PCB bottom

		MagGravity = sqrt(Acc[0]*Acc[0] + Acc[1]*Acc[1] + Acc[2]*Acc[2]);
		printf("\x1B[08;14H%+2.2f  ", MagGravity);
		
		nrf_delay_ms(50);

		readGyroFloatDeg(Acc);													// GYRO
		printf("\x1B[05;40H%+4.2f   ", Acc[0]);
		printf("\x1B[06;40H%+4.2f   ", Acc[1]);
		printf("\x1B[07;40H%+4.2f   ", Acc[2]);
		
		nrf_delay_ms(50);
		
//		readMagFloatUT(data);													// MAGNETIC Field in uTesla UT (updated at system timer
		printf("\x1B[05;68H%+4.2f   ", Magnetic[1]);							// X-Y Axis swap to allign with Accel-Gyro axis
		printf("\x1B[06;68H%+4.2f   ", Magnetic[0]);
		printf("\x1B[07;68H%+4.2f   ", Magnetic[2]);							// Z-Axis alligned in reverse direction to Accel-Gyro
		
		
		nrf_delay_ms(50);
		
		printf("\x1B[12;68H%+08.2f  ", (float)MPL3115A2_getPressure());
		
		temp=(float)MPL3115A2_getPressureSeaLevel();							// dummy read else Altitude reads incorectly probably as pressure???
		printf("\x1B[13;68H%+08.2f  ", (float)MPL3115A2_getAltitude());			// ERROR Returns a different value to the same function called at menu-4 ????? ERROR

		nrf_delay_ms(50);
		
//		TEMPERATURE ---> REAL TIME DEVICE UPDATES
		
		printf("\x1B[12;44H%+4.1f  ", (float) temperature_nRF51_get());			// nRF51 Processor Die Temperature
		
		printf("\x1B[13;44H%+4.1f  ", (float) readTempData()); 					// Inertial Temperature
		
		printf("\x1B[14;44H%+4.1f  ", (float) MPL3115A2_getTemperature());		// Pressure Sensor Temperature	

		if (!ltc294x_get_temperature(&value)) nrf_delay_ms(10);
		if (!ltc294x_get_temperature(&value)) {
			temp = value/100;	
			printf("\x1B[15;44H%+4.1f  ", temp);								// GasGauge Temperature
		}
		nrf_delay_ms(50);
	}

}

static void UART_VT100_Refresh_Data_GPS() {							// $$$$$$ GPS MENU 						DATA REFRESH MENU-2	$$$$$$
	
/** @brief 		Function to load GPS MENU Refresh Data MENU-2 to VT100 Screen.  */
		
	}	


static void UART_VT100_Refresh_Data_Inertial() {					// $$$$$$ INERTIAL MENU 				DATA REFRESH MENU-3	$$$$$$
	
/** @brief 		Function to load INERTIAL MENU Refresh Data MENU-3 to VT100 Screen.  */
	
if (testbit(&Process_1, 0)==false) {
	float Acc[3];
		double MagGravity, Magnitude, Bearing;
			
		float data[4];
		
		readAccelFloatMG(Acc);													// ACCELERATION-GRAVITY (g)
		printf("\x1B[04;41H%+6.3f    ", Acc[0]);
		printf("\x1B[05;41H%+6.3f    ", Acc[1]);
		printf("\x1B[06;41H%+6.3f    ", -1 * Acc[2]);							// Swap z Axis AS g typically lies on neg z-axis
		
		MagGravity = sqrt(Acc[0]*Acc[0] + Acc[1]*Acc[1] + Acc[2]*Acc[2]);
		printf("\x1B[07;41H%+6.3f   ",  (float) MagGravity);					// MAG X-Y-Z
		
		readGyroFloatDeg(Acc);													// GYROSCOPE Degrees
		printf("\x1B[04;65H%+06.1f    ", Gyro[0]);			//Acc[0]);
		printf("\x1B[05;65H%+06.1f    ", Gyro[1]);			//Acc[1]);
		printf("\x1B[06;65H%+06.1f    ", Gyro[2]);			//Acc[2]);
		
		nrf_delay_ms(50);
			
		printf("\x1B[15;65H%+06.2f    ", Acc[0]);
		printf("\x1B[16;65H%+06.2f    ", Acc[1]);
		printf("\x1B[17;65H%+06.2f    ", Acc[2]);
		
		nrf_delay_ms(50);
			
//		mpu_get_compass_reg(data, &timestamp);
//		readMagFloatUT(data);													// MAGNETIC FIELD uTesla
//		readMagTest(data);

		printf("\x1B[04;14H%+06.1f    ", Magnetic[1]);			// X			// Updated at System Timer
		printf("\x1B[05;14H%+06.1f    ", Magnetic[0]);			// Y
		printf("\x1B[06;14H%+06.1f    ", Magnetic[2]);			// Z

		nrf_delay_ms(100);
		
		Magnitude = sqrt(Magnetic[0]*Magnetic[0] + Magnetic[1]*Magnetic[1] + Magnetic[2]*Magnetic[2]);	// Total Magnetic Field Exposure ---> Earth + Other ????
		printf("\x1B[08;14H%+06.1f    ", Magnitude);
		
		nrf_delay_ms(50);
		
		Magnitude = sqrt(Magnetic[0]*Magnetic[0] + Magnetic[1]*Magnetic[1]);   					// Total X-Y Magnetic Field Exposure ---> Earth + Other ????
		printf("\x1B[07;14H%+06.1f    ", Magnitude);
		
		nrf_delay_ms(50);
			
		Bearing = 180/pi*atan(Magnetic[0]/Magnetic[1]);
		
		if(data[1]<0)  Bearing = Bearing + 180;
		printf("\x1B[18;14H%+06.1f   ", Bearing);
		
		printf("\x1B[19;14H       ");
		
		nrf_delay_ms(100);
		
		readQuaternion(Quaternion);												// QUARTERNION ---> Updated during sys Timer 200 msec
		
		printf("\x1B[11;14H%+07.4f ", Quaternion[0]);
		printf("\x1B[12;14H%+07.4f ", Quaternion[1]);
		printf("\x1B[13;14H%+07.4f ", Quaternion[2]);
		printf("\x1B[14;14H%+07.4f ", Quaternion[3]);
		
		Magnitude = sqrt( Quaternion[0] * Quaternion[0] + 
						  Quaternion[1] * Quaternion[1] + 
						  Quaternion[2] * Quaternion[2] +
						  Quaternion[3] * Quaternion[3]);						// Magnitude of Quaternion ????
							
		printf("\x1B[15;14H%+07.4f  ", Magnitude);
		
		nrf_delay_ms(100);
		
		printf("\x1B[11;65H%+06.1f    ", GyroIntegrator[0]);
		printf("\x1B[12;65H%+06.1f    ", GyroIntegrator[1]);
		printf("\x1B[13;65H%+06.1f    ", GyroIntegrator[2]);
		
		nrf_delay_ms(100);
			
		printf("\x1B[19;65H%+06.2f    ", Gyro_ManualCal[0]);
		printf("\x1B[20;65H%+06.2f    ", Gyro_ManualCal[1]);
		printf("\x1B[21;65H%+06.2f    ", Gyro_ManualCal[2]);
		
		printf("\x1B[23;21H%d", sysTimeTick);

		nrf_delay_ms(250);
				
	
/*	Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.

	In this coordinate system, the positive z-axis is down toward Earth. 
	Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
	Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
	Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
	These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
	Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
	applied in the correct order which for this configuration is yaw, pitch, and then roll.
	For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.*/
  
//		float yaw, pitch, roll, q[4];
//		
//		q[0] = Quaternion[0];
//		q[1] = Quaternion[1];
//		q[2] = Quaternion[2];
//		q[3] = Quaternion[3];

//		yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
//		pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
//		roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
//		
//		pitch *= 180.0f / pi;
//		yaw   *= 180.0f / pi; 
//		yaw   += MagTNReference; 								// True North Offset Melbourne Australia on 2014-04-04
//		roll  *= 180.0f / pi;
		
//		MPU9250_Get_Euler(pitch, roll, yaw);

		printf("\x1B[11;41H%+4.2f    ", pitch);
		printf("\x1B[12;41H%+4.2f    ", roll);
		printf("\x1B[13;41H%+4.2f    ", yaw);
		
		nrf_delay_ms(200);
	}
}	

static void UART_VT100_Refresh_Data_Altitude() {					// $$$$$$ ALTITUDE MENU 				DATA REFRESH MENU-4	$$$$$$
	
/** @brief 		Function to load ALTITUDE MENU Refresh Data MENU-4 to VT100 Screen.  */	
		
	// Note MPL3115A returns float ---> cast as int 

	if (testbit(&Process_1, 0)==false) {
		
		printf("\x1B[06;21H%+4.2f   ", (float)MPL3115A2_getPressure());
		
		printf("\x1B[07;21H%+4.2f   ", (float)MPL3115A2_getPressureSeaLevel()); 

		printf("\x1B[08;23H%+4.2f   ", (float)MPL3115A2_getTemperature());

		printf("\x1B[06;55H%+4.2f   ", (float)MPL3115A2_getAltitude());

	}
}	


static void UART_VT100_Refresh_Data_Memory() {						// $$$$$$ MEMORY MENU 					DATA REFRESH MENU-5	$$$$$$
	
/** @brief 		Function to load MEMORY MENU Refresh Data MENU-5 to VT100 Screen.  */	
	if (testbit(&Process_1, 0)==false) {
	}
}	

static void UART_VT100_Refresh_Data_Power()	{	    				// $$$$$$ POWER MANAGEMENT MENU			DATA REFRESH MENU-6	$$$$$$
	
/** @brief 		Function to load POWER MANAGEMENT MENU Refresh Data MENU-6 to VT100 Screen.  */
	if (testbit(&Process_1, 0)==false) {
		int value;
		float temp;
			
		if (!ltc294x_get_voltage(&value))											// WARNING GasGauge Intermitent Occassionally Returns 0000
		temp = (float) value;
		temp = temp / 1000000;
		printf("\x1B[6;22H%+4.2f   ", temp);										// GasGauge Battery Voltage 

		if (value>4100000)															// WARNING If battery emoved the Charging Supply Jumps to .GT.4V1 and incorrectly shows as charged
			printf("\x1B[08;22H CHARGED  ");										// Need to base battery of charge counter or on battery reaching correct capacity
		else if (value>3900000)						// 3V9  75% Capacity
			printf("\x1B[08;22H 75%%     ");
		else if (value>3750000)						// 3V75 50% Capacity
			printf("\x1B[08;22H 50%%     "); 
		else if (value>3700000)						// 3V70 25% Capacity
			printf("\x1B[08;22H 25%%     "); 
		else if (value<=3650000)					// 3V65 12% Capacity ----> ALARM ToDo Force a power down
			printf("\x1B[08;22H ALARM   "); 
		
		if (!ltc294x_get_current(&value))					
		temp = (float) value/1000;
		printf("\x1B[7;22H%+4.2f     ", temp);										// GasGauge Battery Current  FAULT ---> Not Returning Valid dada ??????????
		
		printf("\x1B[10;22H%+4.2f    ", temp);					
		
		if (!ltc294x_get_charge_counter(&value))
		temp = (float) value/10;
		printf("\x1B[09;22H%+4.2f     ", temp);										// GasGauge Charge Counter
		
		if (!ltc294x_get_temperature(&value))
		temp = (float) value/100;	
		printf("\x1B[10;22H%+4.2f     ", temp);										// GasGauge Temperature 
		
		temp = (float) sensorADC_singleMeasure()/39.58;								// Scalled to generate actual voltage at battery relative to CRO VRMS reading
		printf("\x1B[06;63H%+4.2f     ", temp);										// nRF51 Processor ADC.AIN6 Battery Voltage 
	}
}

static void UART_VT100_Refresh_Data_Cane() {						// $$$$$$ CANE DIAGNOSTICS MENU 		DATA REFRESH MENU-7	$$$$$$
	
/** @brief 		Function to load CANE DIAGNOSTICS MENU Refresh Data MENU-7 to VT100 Terminal Screen.  */	
	if (testbit(&Process_1, 0)==false) {								// Test for VT100 refresh Flag
	}
}	

static void UART_VT100_Refresh_Data_System() {						// $$$$$$ SYSTEM DIAGNOSTICS MENU		DATA REFRESH MENU-8	$$$$$$
	
/** @brief 		Function to load SYSTEM DIAGNOSTICS MENU Refresh Data MENU-8 to VT100 Screen.  */	
	if (testbit(&Process_1, 0)==false) {								// Test for VT100 refresh Flag
		
		
		printf("\x1B[06;47H %d   ", ErrorCounter[0]);					// System Error Counter
		printf("\x1B[07;47H %d   ", ErrorCounter[1]);					// System Error Counter
		printf("\x1B[08;47H %d   ", ErrorCounter[2]);					// System Error Counter
		printf("\x1B[09;47H %d   ", ErrorCounter[3]);					// System Error Counter
		printf("\x1B[10;47H %d   ", ErrorCounter[4]);					// System Error Counter
		
		nrf_delay_ms(100);
		
		printf("\x1B[11;47H %d   ", ErrorCounter[5]);					// System Error Counter
		printf("\x1B[12;47H %d   ", ErrorCounter[6]);					// System Error Counter
		printf("\x1B[13;47H %d   ", ErrorCounter[7]);					// System Error Counter
		printf("\x1B[14;47H %d   ", ErrorCounter[8]);					// System Error Counter

		nrf_delay_ms(100);
		
		printf("\x1B[16;47H %d   ", ErrorCounter_UART_FIFO);			// System Error Counter
		printf("\x1B[17;47H %d   ", ErrorCounter_UART_Comms);			// System Error Counter
		
		printf("\x1B[20;06H error_code t-2 = %d   ", error_code_clone[2]);	// System Error Clone ---> Last captured non zero error_codes
		printf("\x1B[21;06H error_code t-1 = %d   ", error_code_clone[1]);	// System Error Clone ---> Last captured non zero error_codes
		printf("\x1B[22;06H error_code t   = %d   ", error_code_clone[0]);	// System Error Clone ---> Last captured non zero error_codes
		
		nrf_delay_ms(100);
		
		printf("\x1B[22;47H SP = 0x%X ", Read32_ASM_SP());				// Current Stack Position
		
	}
}	
static void UART_VT100_Refresh_Data_Peripheral() {					// $$$$$$ PERIPHERALICS MENU			DATA REFRESH MENU-9	$$$$$$
	
/** @brief 		Function to load Peripheral DIAGNOSTICS MENU Refresh Data MENU-8 to VT100 Screen.  */	
	if (testbit(&Process_1, 0)==false) {								// Test for VT100 refresh Flag
		
		
	}
}	

static void UART_VT100_Refresh_Data_Bluetooth() {					// $$$$$$ BLUETOOTH DIAGNOSTICS MENU	DATA REFRESH MENU-A	$$$$$$
	
	
/** @brief 		Function to load SYSTEM DIAGNOSTICS MENU Refresh Data MENU-8 to VT100 Screen.  */	
	if (testbit(&Process_1, 0)==false) {								// Test for VT100 refresh Flag
	}
}	

static void VT100_Scan_Keyboard_All_Menues() {		    			// $$$$$$ LOAD ALL VT100 DATA REFRESH ACROSS ALL MENUs      $$$$$$
	
/** @brief 		Function to load ALL VT100 IMMEDIATE AND DATA REFRESH MENUEs to UART Screen.  */
	
	if (testbit(&Process_1, 0)==false) {											// Process_1.0 VT100 refresh=0 or ATxx Command Mode=1
		
		printf("\x1B[01;78H");														// Park VT100 cursor at row 01 column 78
		
		nrf_delay_ms(50);
		
		uint8_t cr;
		if (app_uart_get(&cr) == NRF_SUCCESS) {										// Test for UART receive character and action if available.
			
			switch (cr) {															// Upon Valid Rx Menu Character Load a new Background Menue
			
				case '1':
					UART_VT100_Menu_1();											// 1... All Sensors
					break;
				
				case '2':
					UART_VT100_Menu_2();											// 2... GPS Global Position
				break;
				
				case '3':
					UART_VT100_Menu_3();											// 3... Inertial Sensors
					break;
				
				case '4':
					UART_VT100_Menu_4();											// 4... Altitude and Temperature							
					break;
				
				case '5':
					 UART_VT100_Menu_5();											// 5... Memory Functions						
					break;
				
				case '6':
					 UART_VT100_Menu_6();											// 6... Power Management						
					break;
				
				case '7':
					 UART_VT100_Menu_7();											// 7... Cane Diagnostics					
					break;
				
				case '8':
					 UART_VT100_Menu_8();											// 8... System Diagnostics						
					break;
				
				case '9':
					 UART_VT100_Menu_9();											// 9... Peripheral Diagnostics						
					break;
				
				case 'A':
					 UART_VT100_Menu_A();											// A... Bluetooth Diagnostics						
					break;
				
				case 'x':
					UART_VT100_Main_Menu();											// 0... Main Menue
					break;
				
				case 'X':
					UART_VT100_Main_Menu();											// 0... Main Menue
					break;
				
				case 't':
					printf("\x1B[25;30H STATUS: Diagnostic Test              ");	// Diagnostic Test
					break;
				
				case '?':
					UART_VT100_Help_Menu();											// Help Menu
					break;
				
				case 'a':															// Grab Static Manual Calibration Variables
//					calibrateMPU9150(gyro_ManualCal, accel_ManualCal);
					Gyro_ManualCal[0]=Gyro[0];										// Warning... Zero ManualCal registers before reloading --> Interative
					Gyro_ManualCal[1]=Gyro[1];
					Gyro_ManualCal[2]=Gyro[2];
				
					printf("\x1B[25;30H SYSMSG: Load Gyro-Accel ManualCal   ");
					break;

				case 'b':
					if ( MenuLevel == 80) {											// Zero Manual Calibrations
					Gyro_ManualCal[0]=0;   Gyro_ManualCal[1]=0;  Gyro_ManualCal[2]=0;
					accel_ManualCal[0]=0; accel_ManualCal[1]=0; accel_ManualCal[2]=0;
					
					printf("\x1B[25;30H SYSMSG: Clr Gyro-Accel Zero         ");
					}
					break;
				
				case 'c':
					if ( MenuLevel == 80) { 										// Vibro Strobe
					printf("\x1B[25;30H SYSMSG: Vibro Strobe                ");
					setbit(&Process_0, 1);											// Flag to initiate Vibro Motor at 100msec+100msec
					}
					break;
				
				case 'd':
					if ( MenuLevel == 80) { 										// Spare Menu as Start timer
					printf("\x1B[25;30H SYSMSG: ASM register Test          ");
					}
					break;
				
				case 'e':
					if (MenuLevel == 80) { 											// LED Red Test
					printf("\x1B[25;30H SYSMSG: Toggle LED                 ");
					}
					led_red_TOGGLE();
					break;
				
				case 'f':
					if (MenuLevel == 80) { 											// PWM Tone Test (ToDo)
					setbit(&Process_2, 0);											// PWM Master On/Off Flag
					setbit(&Process_2, 1);											// Flag to initiate PWM Tone Test 100msec
					printf("\x1B[25;30H SYSMSG: Enable PWM TEST            ");
					}
					break;
				
				case 'g':
					if (MenuLevel == 80) { 											// Start Bluetooth Advertising will force hardware shutdown after 180sec upon No Connection time out event
					setbit(&Process_3, 0);											// Signal main() TSA to activate ble advertising
//					advertising_start();											// Start Advertising --> Starts Bluetooth advertising however also shuts down power to board after 180sec if no connection
					printf("\x1B[25;30H SYSMSG: Start Bluetooth Advert    ");
					}
					break;
				
				case 'h':
					if (MenuLevel == 80) { 											// Start ATxx Command Mode return with AT99...... VT100 Refresh Mode Bypassed
					setbit(&Process_1, 0);											
					printf("\x1B[2J");												// VT100 CLR SCREEN
					printf("\x1B[H");												// VT100 CURSOR HOME
					printf("SmartCane ATxx Command Mode\r\n");
					printf("Cmd = ");
					}
					break;
				
				case 'j':
					if (MenuLevel == 80) { 											// Clear Error Counters
					printf("\x1B[25;30H SYSMSG: Clear Error Counters      ");										
					
					ErrorCounter_UART_FIFO=0;
					ErrorCounter_UART_Comms=0;
						
					ErrorCounter[2]=0;
					ErrorCounter[3]=0;
					ErrorCounter[4]=0;
					ErrorCounter[5]=0;
					ErrorCounter[6]=0;
					ErrorCounter[7]=0;
					ErrorCounter[8]=0;
					}
					break;
				
				case 'k':
					if (MenuLevel == 80) { 											// Clear Gyro Integrators
					printf("\x1B[25;30H SYSMSG: Clear Gyro Integrators   ");										
					GyroIntegrator[0] = 0;
					GyroIntegrator[1] = 0;
					GyroIntegrator[2] = 0;
					}
					break;
				
				case 'l':
					if (MenuLevel == 80) { 											// Calibrate Magnetometer Average Min Max Readings 
					printf("\x1B[25;30H SYSMSG: Calibrate Magnetometer START ");										
					CalibrateMagnetometer(500);										// Average 500 readings
					printf("\x1B[25;30H SYSMSG: Calibrate Magnetometer END   ");
					
					float Acc[3];
					readMagTest(Acc);
					MagMeanCal[0] = Acc[0];			// Clear Auto Cal Registers Under developement at system timer handler
					MagMeanCal[1] = Acc[1];
					MagMeanCal[2] = Acc[2];
					
					MagMaxCal[0] = -999999;
					MagMaxCal[1] = -999999;
					MagMaxCal[2] = -999999;
					
					MagMinCal[0] = +999999;
					MagMinCal[1] = +999999;
					MagMinCal[2] = +999999;
					}
					break;	
				
				default:
					break;
			} 
		} else {
			
			nrf_delay_ms(25);
						
			switch (MenuLevel) {												// If Load UART Reresh data based on current Menu Level
			
				case 00:														// Main Menu
					UART_VT100_Refresh_Data_Main_Menu();
					break;		
				case 10:														// All Sensors
					UART_VT100_Refresh_Data_All_Sensors();
					break;		
				case 20:														// GPS Global Position
					UART_VT100_Refresh_Data_GPS();
					break;	
				case 30:														// Inertial Sensors
					UART_VT100_Refresh_Data_Inertial();	
					break;
				case 40:														// Altitude Pressure and Temperature
					UART_VT100_Refresh_Data_Altitude();
					break;		
				case 50:														// Memory Functions
					UART_VT100_Refresh_Data_Memory();
					break;		
				case 60:														// Power Management
					UART_VT100_Refresh_Data_Power();
					break;	
				case 70:														// Cane Diagnostcs
					UART_VT100_Refresh_Data_Cane();
					break;		
				case 80:														// System Diagnostcs - Including Error Counters
					UART_VT100_Refresh_Data_System();
					break;
				case 90:														// Peripheral Diagnostcs
					UART_VT100_Refresh_Data_Peripheral();
					break;
				case 100:														// Bluetooth Diagnostcs 
					UART_VT100_Refresh_Data_Bluetooth();
					break;
				default:
					break;		
			}
			
			if (testbit(&Process_1, 0)==false) {	
				printf("\x1B[01;78H");											// Park VT100 cursor at row 01 column 78
				nrf_delay_ms(50);
			}
		}
	} else {
		uint8_t cr;
		if (app_uart_get(&cr) == NRF_SUCCESS) {										// Test for UART ATxx Cmd Mode Characters ---> (VT100 Mode Bypassed).
			
			if (cr == 13){
				
				if ((ATxx[3]=='A') && (ATxx[2]=='T')){
					if (ATxx[1]>=48 && ATxx[1]<=57 && ATxx[0]>=48 && ATxx[0]<=57){	// Test for Numeric
						
						int8_t ATcode;
						ATcode=(ATxx[1]-48)*10+ATxx[0]-48;							// Convert double ascii into int8 number range 00--99
						
						switch (ATcode) {											// Load UART Reresh data based on current Menu Level
		
							case 00:												// AT00 (Help)
								printf("HELP... AT00 to AT99\r\n\n");
								printf("AT00 - Help \r\n");
								printf("AT01 - Display ASM Stack and PC \r\n");
								printf("AT02 - Test Magnetic Bearing b/n 2 Lat-Lon coordinates \r\n");
								printf("AT03 - Spare \r\n");
								printf("AT80 - GPS NEMA to UART \r\n");
								
								printf("AT99 - Start VT100 Mode \r\n");
								break;
							
							case 01:											// AT01 Command to read ASM SP Stack Pointer Register and output to UART
								printASMReg();
								break;
							
							case 02:											// AT02 Command to print Magnetic Bearing between two Lat-Lon Coordinates
								printf("\nMagnetic Bearing = %+06.1f \r\n", 
								BearingMag(39.099912,-94.581213,38.627089,-90.200203));		// Melbourne Magnetic
//								BearingTN(39.099912,-94.581213,38.627089,-90.200203));		// True North
								break;
							
							case 03:											// AT03
								break;
							
							case 80:											// Toggle Flag to Stream NEMA GPS to UART
								if(testbit(&Process_1, 1)==false) {
									setbit(&Process_1, 1);
								} else {
									clrbit(&Process_1, 1);
								}
								break;
							
							case 99:											// AT99 Enable VT100 Mode
								clrbit(&Process_1, 0);							// Return to VT100 Refresh Mode
								UART_VT100_Main_Menu();							// Reinitialise Default UART VT100 Main Menu
								break;
							
							default:
								break;
							
						}
					}
				}
				ATxx[3]=0;														// Clear the top Buffer --- Disable for cr cr cr repeats
				if (testbit(&Process_1, 0)==true) printf("\r\nCmd = ");
						
			} else {
				ATxx[3]=ATxx[2];												// Shuffle ASCII charcters into 4 char buffer ---> Expect "ATnn" upon cairrage return
				ATxx[2]=ATxx[1];
				ATxx[1]=ATxx[0];
				ATxx[0]=cr;
			}
		}
	}
	nrf_delay_ms(100);	
}

void APP_ERROR_CHECK_CSB(uint32_t error_code) {						// $$$$$$ Copy of APP_ERROR_CHECK to allow clone trap of error_code events
	
	if(error_code !=0) {												// Load error into array if not 0
		error_code_clone[2]  = error_code_clone[1];
		error_code_clone[1]  = error_code_clone[0];
		error_code_clone[0]  = error_code;
	}
	
	APP_ERROR_CHECK(error_code);
}

void SmartCane_peripheral_init() {									// $$$$$$ Initialisation of SmartCane Peripherals
	
/** @brief 		Function to initialise modules including button interupts
    @details 	
    @note  		
    @ref 		*/	
	
	uint32_t err_code;
	
	// NOTE... For button interupt, must be called before 'app_button_init' inside bsp_init
	
	APP_GPIOTE_INIT(1);  															// One user, no use yet unless app_button_enable() is called

	err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
					APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),						// 100msec scan
					button_event_handler);											// Button Interupt Handler
	if (err_code == NRF_ERROR_INVALID_STATE) ErrorCounter[4] += 1;

	APP_ERROR_CHECK_CSB(err_code);

	nrf_gpio_cfg_output(MOTOR_PIN_NUMBER);
	
//	LED setup ToDo
	nrf_gpio_cfg_output(BSP_LED_0);

//	nrf_temp_init();

//	nrf_gpio_cfg_output(7);											// Haptic Vibration Motor
	
	unsigned char status = 0;
	while(!status)  status = I2C_Init();
		
//	calibrateMPU9150()
	
	MPU9250_setup();		// Passes through however includes long with big Averaging Loop
	
//	initMPU9250();
	if(MPU9250_WhoAmI()==0x71) setbit(&Process_5,1);				// Process_5.1	= Flag MPU9250 Inertial Sensor Present
	
	float Acc[3];
	readGyroFloatDeg(Acc);											// Read xyz Raw GYRO registers deg/sec
	
	Gyro_ManualCal[0]=Acc[0];										// Initalise autozero registers
	Gyro_ManualCal[1]=Acc[1];
	Gyro_ManualCal[2]=Acc[2];
	
	sysLastTime = sysTimeTick;
	
	GyroIntegrator[0]=0;
	GyroIntegrator[1]=0;
	GyroIntegrator[2]=0;

	//readMagFloatUT(Acc);											// Read xyz Raw Magnetometer registers uTesla	
	readMagTest(Acc);
	MagMeanCal[0] = Acc[0];
	MagMeanCal[1] = Acc[1];
	MagMeanCal[2] = Acc[2];
	
	MagMaxCal[0] = -999999;
	MagMaxCal[1] = -999999;
	MagMaxCal[2] = -999999;
	
	MagMinCal[0] = +999999;
	MagMinCal[1] = +999999;
	MagMinCal[2] = +999999;
	
/*
//		if(mpu_init(NULL));
//		long gyro,accel;
//		mpu_run_self_test(&gyro, &accel);

//		sensorADC_config();  										// sensor adc init
		finishedADC = false;  										// for sensor_adc only
	
// read compass
//		short data[3];
		
//		unsigned long timestamp;
//		
//		nrf_delay_us(1000000);


//		while(mpu_get_compass_reg(data, &timestamp)) 
//			nrf_delay_us(1000);
//			while(1) {
//				status = mpu_get_compass_reg(data, &timestamp);
//				if(data[0]>0 || status == 0)
//				break;
//			}
//		long temperature;
//		while(mpu_get_temperature(&temperature, &timestamp));
//		while(mpu_get_accel_reg(data,&timestamp));
//		while(mpu_get_gyro_reg(data,&timestamp));
//	
//		mpu_run_6500_self_test(&gyro, &accel,0);

//		initA2035H();
*/

/*	Process_5.4	= Flag A2035 GPS Sensor Present

	Process_5.5	= Flag SFlash Memory Present
	Process_5.6	= Flag EEPROM Present
	Process_5.7	= uP and Peripheral temperatures OK ---> .LE.50 */

	MPL3115A2_init();
	if(MPL3115A2_WhoAmI()==0xC4) setbit(&Process_5,2);			// Process_5.2 = Flag MPL3115 Pressure Sensor Present
	
	float f = MPL3115A2_getAltitude();
	
	ltc294x_init();
	int temp;
	ltc294x_get_temperature(&temp);
	if(temp!=0) setbit(&Process_5,3);							// Process_5.3 = Flag LTC2943 GasGauge Sensor Present cludeged by reading temperature
	
	ltc294x_get_current(&temp);
	ltc294x_get_voltage(&temp);
	ltc294x_get_charge_counter(&temp);
}


int32_t temperature_nRF51_get(void) {								// $$$$$$ nRf51 Die Temperature
    int32_t temp;
    uint32_t err_code;
    int32_t temperature;
	
    err_code = sd_temp_get(&temp);
    APP_ERROR_CHECK(err_code);
    if (err_code) return 0;
	
	temperature = (double) temp;
    temperature = (temperature / 4);												// Die Temperature 0.25deg increments ---> Accuracy +/-4degC
    
 //   int8_t exponent = -2;
    return temperature; //((exponent & 0xFF) << 24) | (temp & 0x00FFFFFF);
}

int printASMReg(void) {												// $$$$$$ Function to access and Print ASM registers including Stack Pointer SP 
    
	int spReg, lrReg, pcReg;
	
	__asm {
		MOV spReg, __current_sp()
		MOV pcReg, __current_pc()
		MOV lrReg, __return_address()
	}

	printf("Stack Pointer SP = 0x%X\n\r",spReg);					// Read and Print _ASM SP, PC and Return Address Output Upon keyboard cmd AT01
	printf("      Program Counter PC = 0x%X\n\r",pcReg);
	printf("      Return  Address LR = 0x%X\n\r",lrReg);
	return 0;
}

int Read32_ASM_SP() {												// Read and return _ASM Stack Poiter SP Register
	int spReg;
	__asm {
		MOV spReg, __current_sp()
	}
	return spReg;
} 

/* _ASM Program Counter PC Register									// Bypassed while not in use


unsigned int Read32_ASM_PC() {										// Read and return _ASM Program Counter PC Register
	unsigned int pcReg;
	__asm MOV pcReg, __current_pc()
	return pcReg;
}*/

/*	static void blockSend(float *Data, unsigned int len) {			// ToDo   Bluetooth fails to send notify in this callback? ---> Leo


//	
// 		if (m_AD7746.conn_handle != BLE_CONN_HANDLE_INVALID) {
////			for(unsigned int i=0; i<len; i++) {
////			dataToSend[i] = Data[i];
////		}
////		finishedADC = true;
////	}
}*/


typedef enum {DataTypeNone = 0, DataTypeTEMP = 1} SendDataType;
static SendDataType resumetype;
unsigned int lastSendIndex;
unsigned int resumeSendData() {  									// Only called by on_ble_evt when BLE_EVT_TX_COMPLETE happens
																	// returns the index to current finished position in 'dataToSend' buffer
	uint32_t err_code = NRF_SUCCESS;
	
	ErrorCounter[5] += 1;
	
	if (lastSendIndex==(unsigned int)-1) {
		return (unsigned int)-1;
	}
	switch (resumetype) {											// depends on the data to send type
		case DataTypeTEMP:											// simulating as ADC, TODO: change to ADC service
			if (finishedADC) {
				unsigned int i=lastSendIndex;						// resume from this index
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
						{  											// shouldn't come to here
							finishedADC = false;
							lastSendIndex = (unsigned int)-1;  
							bNeedBreak = true;
							break;
						}
						case BLE_ERROR_NO_TX_BUFFERS:
						{  											// the main reason to split the TX process 
							bNeedBreak = true;
							lastSendIndex = i;						// resume from the index
							break; 									// resend 
						}
						default:  									// shouldn't come to here
							APP_ERROR_HANDLER(err_code);
					}
				}
				if (i>=MEASURESIZE) {
					finishedADC = false;
					lastSendIndex = 0;
				}
				return lastSendIndex;
			}
			break; 													// End of case DataTypeTEMP:
		case DataTypeNone:
			break;
		default:
			break;
	}
	return (unsigned int)-1; 										// Means invalid
}




/**
 * @brief WDT events handler.



//void bsp_event_callback(bsp_event_t event){
//
//	@brief BSP events callback.
//
//    switch(event) {
//        case BSP_EVENT_KEY_0:
//            nrf_drv_wdt_channel_feed(m_channel_id);
//            break;
//        
//        default :
//            //Do nothing.
//            break;
//    }
//} */

static void system_timer_handler(void * p_context) {				// Timer event to prime quaternion inertial filter 100msec rate
	
/** @brief 		Function to service background timer interupt 
    @details 	This routine is designed provide real time delay functions and load inertial sensor MPU9250
				and provide background scheduling of ble data transfer tasks.
	
    @note  		nRF51822 Anomaly Notice v3.0 ---> Use of an EVENT from any TIMER module  
				to trigger a TASK in GPIOTE or RTC using the PPI could fail under
				certain conditions.
    @ref  																			*/		

	uint32_t err_code = NRF_SUCCESS;    
			
	UNUSED_PARAMETER(p_context);
	
	float temp;
	float Acc[3];
	
	sysTimeTick += 1;														// sysTimeTick Counter in SysTimer Increments	 200msec
	
	if(!testbit(&iProcess_0,0)) {											// Trap and bypass in case handler busy with previous event 
		setbit(&iProcess_0,0);												// Set Flag for active system timer handler


		readGyroFloatDeg(Acc);												// Read xyz Raw GYRO registers deg/sec
		temp=Acc[0]+Acc[1]+Acc[2];
		if(temp != 0) {														// Skip if Acc[]=0
			Gyro[0] = 0.05*Gyro[0] + 0.94*((Acc[0] - Gyro_ManualCal[0]));	// Apply Leaky LP filtering to Manual Calibration (15% old + 84% new)
			Gyro[1] = 0.05*Gyro[1] + 0.94*((Acc[1] - Gyro_ManualCal[1]));	// Apply Leaky LP filtering
			Gyro[2] = 0.05*Gyro[2] + 0.94*((Acc[2] - Gyro_ManualCal[2]));	// Apply leaky LP filtering
			
			if(temp <= 10 || temp >= -10) {									// Trap out during big movements Auto cal when moving slow !!!
				Gyro_ManualCal[0]=LPFilter(Gyro_ManualCal[0],Acc[0],99.9);	// Slowly Zero ManualCal Registers when during slow or static movement
				Gyro_ManualCal[1]=LPFilter(Gyro_ManualCal[1],Acc[1],99.9);
				Gyro_ManualCal[2]=LPFilter(Gyro_ManualCal[2],Acc[2],99.9);				
			}
			
			if(sysLastTime==0 || sysLastTime>sysTimeTick) sysLastTime = sysTimeTick;
			float DeltaT = 0.2*(sysTimeTick-sysLastTime);					// 200msec increments

			if (Gyro[0]<-5 || Gyro[0]>5) GyroIntegrator[0]+=Gyro[0]*DeltaT;	// Integrate if outside deadband +/- 5degs/sec
																			// convert degs/sec to degrees ie Nominally divide by 100msec sys timer entry
			if (Gyro[1]<-5 || Gyro[1]>5) GyroIntegrator[1]+=Gyro[1]*DeltaT;
			if (Gyro[2]<-5 || Gyro[2]>5) GyroIntegrator[2]+=Gyro[2]*DeltaT;
					
			if(GyroIntegrator[0]>+180) GyroIntegrator[0]-=360;				// Reference Gyrointegrators in the range 0 to +/- 180
			if(GyroIntegrator[0]<-180) GyroIntegrator[0]+=360;		
			if(GyroIntegrator[1]>+180) GyroIntegrator[1]-=360;
			if(GyroIntegrator[1]<-180) GyroIntegrator[1]+=360;		
			if(GyroIntegrator[2]>+180) GyroIntegrator[2]-=360;
			if(GyroIntegrator[2]<-180) GyroIntegrator[2]+=360;
			
			GyroIntegrator[0] *= 0.999;										// Slowly Zero Gyro Integrators 
			GyroIntegrator[1] *= 0.999;
			GyroIntegrator[2] *= 0.999;
			
			sysLastTime = sysTimeTick;
		
		}
		

		MPU9250_Timed_Interupt();											// ToDo---> Maybe Dive via MPU9250 interupt to ensure correct data present !!!!!
		
		if(testbit(&Process_5,1)){												// Check if MPU9250 Inertial Sensor Present

			Acc[0]=Acc[1]=Acc[2]=0;
//			readMagFloatUT(Acc);												// Read xyz Raw Magnetometer uTesla registers
			readMagTest(Acc);
			temp=Acc[0]+Acc[1]+Acc[2];
			if(temp != 0) {														// Skip if Acc[]=0
				
				if(Acc[0]>MagMaxCal[0])	LPFilter(MagMaxCal[0],Acc[0], 85);		// Trap for Maximum Magnetic Field Values in each axis
				if(Acc[1]>MagMaxCal[1])	LPFilter(MagMaxCal[1],Acc[1], 85);			
				if(Acc[2]>MagMaxCal[2])	LPFilter(MagMaxCal[2],Acc[2], 85);			

				if(Acc[0]<MagMinCal[0])	LPFilter(MagMinCal[0],Acc[0], 85);		// Trap for Minimum Magnetic Field Values in each axis
				if(Acc[1]<MagMinCal[1])	LPFilter(MagMinCal[1],Acc[1], 85);			
				if(Acc[2]<MagMinCal[2])	LPFilter(MagMinCal[2],Acc[2], 85);			

				MagCenterCal[0] = LPFilter(MagCenterCal[0],(MagMaxCal[0]-MagMinCal[0])/2, 99);
				MagCenterCal[1] = LPFilter(MagCenterCal[1],(MagMaxCal[1]-MagMinCal[1])/2, 99);
				MagCenterCal[2] = LPFilter(MagCenterCal[2],(MagMaxCal[2]-MagMinCal[2])/2, 99);

				MagMeanCal[0] = LPFilter(MagMeanCal[0],Acc[0]-MagCenterCal[0], 95);
				MagMeanCal[1] = LPFilter(MagMeanCal[1],Acc[1]-MagCenterCal[1], 95);
				MagMeanCal[2] = LPFilter(MagMeanCal[2],Acc[2]-MagCenterCal[2], 95);

				MagMaxCal[0] *= 0.99999;										// Send to zero while static
				MagMaxCal[1] *= 0.99999;
				MagMaxCal[2] *= 0.99999;

				MagMinCal[0] *= 0.99999;										// Send to zero while static
				MagMinCal[1] *= 0.99999;
				MagMinCal[2] *= 0.99999;
				
				Magnetic[0]=Acc[0];		//-MagCenterCal[0];
				Magnetic[1]=Acc[1];		//-MagCenterCal[1];
				Magnetic[2]=Acc[2];		//-MagCenterCal[2];
			}
		}
		
		
		readAccelFloatMG(Acc);													// ACCELERATION-GRAVITY (g) ---> May Require LP Filtering --> Decide on test!!!!
		
		if(Acc[0]+Acc[1]+Acc[2]!=0) { 											// Chech for valid Acc Read
			if(Acc[1] < 1 ) {													// Check that Acc[1] is with Gravity Unit Circle 
				if(Acc[2] > 0.5) {												// Test Cane Handle Facing Up
					temp = acos(1/Acc[1])*180/pi;								// acos(1/y)
					if (temp>35 && temp<55) {									// Test if Cane Alignment Angled Down 35deg-to-55deg
						setbit(&Process_7,0);									// Flag Cane Alignment is NORMAL OPERATION
					} else clrbit(&Process_7,0);
				}
			}

		if(atan(Acc[2]/Acc[1])*180/pi>85) setbit(&Process_7,4);					// Test and Flag for Cane Vertical-UP (+15deg-to-15deg)  
		else clrbit(&Process_7,4);

		if(atan(Acc[2]/Acc[1])*180/pi<-85) setbit(&Process_7,5);				// Test and Flag for Cane Vertical-DOWN (+15deg-to-15deg)  
		else clrbit(&Process_7,5);


		if(Acc[0] > 0.4) setbit(&Process_7,2);									// Test and Flag for Cane Buttons RIGHT --> Process_7.2 Flags
		else clrbit(&Process_7,2);	
		
		if(Acc[0] < -0.4) setbit(&Process_7,3);									// Test and Flag for Cane Buttons LEFT  --> Process_7.3 Flags
		else clrbit(&Process_7,3);	
		
		}
	
		// Test for Vibration motor initiation and Delay off count
		if(testbit(&Process_0, 0)==true) {									// Background Test for Active Vibro Motor Flag Auto Countdown of 200msec
			Count_VibroMotor = 2;
			clrbit(&Process_0, 0);
		} 

		if(testbit(&Process_0, 1)==true) {									// Background Test for Active Vibro Motor Flag Auto Countdown of 400msec
			Count_VibroMotor = 4;
			clrbit(&Process_0, 1);
		}
		
		if(testbit(&Process_0, 2)==true) {									// Background Test for Active Vibro Motor Flag Auto Countdown of 800msec
			Count_VibroMotor = 8;
			clrbit(&Process_0, 2);
		}
		
		if(testbit(&Process_0, 3)==true) {									// Background Test for Active Vibro Motor Flag Auto Countdown of 1600msec
			Count_VibroMotor = 16;
			clrbit(&Process_0, 3);
		}
		
		if(testbit(&Process_0, 4)==true) {									// Background Test for Active Vibro Motor Flag Auto Countdown of 3200msec
			Count_VibroMotor = 32;
			clrbit(&Process_0, 4);
		}
		if(testbit(&Process_0, 5)==true) {									// Background Test for Active Vibro Motor Flag Auto Countdown of 6400msec
			Count_VibroMotor = 64;
			clrbit(&Process_0, 5);
		}
		if(Count_VibroMotor > 0){											// Note Will Assert Vibro Motor up to 2x500 ticks after process flag set in main()
			
			Count_VibroMotor -= 1;
			nrf_gpio_pin_set(MOTOR_PIN_NUMBER);
		} else {
			nrf_gpio_pin_clear(MOTOR_PIN_NUMBER);
		}


		
	//	readAccelFloatMG(Acc);			//[0]==Left=+ive    Right=-ive		[1]==Pitch Up=+ive  Down=-ive		[2]==Z Up=+ive       Z Down-ive
		readGyroFloatDeg(Acc);			//[0]==Pitch Up=+ive Down=-ive		[1]==Roll CW=+ive   ACW=-ive		[2]==Yaw Right=+ive  Left=-ive
	//	readMagFloatUT(Acc);			//[0]==Front=+ive    Rear=-ive		[1]==Right=+ive     Left=-ive		[2]==Z Down=+ive     Z Up=-ive
				
		err_code = ble_AD7746_send_temp_notify(&m_AD7746, Acc[2]);				// Currently ASSIGNED TO transmit Gyro Yaw to ble pipe when ble connected
		
		if ((err_code != NRF_SUCCESS) &&
				(err_code != NRF_ERROR_INVALID_STATE) &&
				(err_code != BLE_ERROR_NO_TX_BUFFERS) &&
				(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)) 				// ignore these errors as they appear only during setup and are normal
		
		{	APP_ERROR_HANDLER(err_code);
		}

/*
	//	short data[3];
	//	unsigned long timestamp;
	//	int16_t temp; // = readTempData();	
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
	//		} else {
	//			sensorADC_startSequenceMeasure(MEASURESIZE, blockSend);
	//		}

*/
	
	clrbit(&iProcess_0,0);											// Clear Interupt Flag ---> Indicates exit from of system timer handler
	return;
	}
}

static void timers_init(void) {										// Initalise Timers and setup event handlers
/**	@brief		Function for the Timer initialisation.
	@details	Initializes the timer module. This creates application timer event handlers.*/

	uint32_t err_code;


    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);		// Initialize Timer Module.


	err_code = app_timer_create(&m_sys_timer_id,    												// Create system_timer_handler == 100 msec
                                APP_TIMER_MODE_REPEATED,
                                system_timer_handler);
	
    APP_ERROR_CHECK_CSB(err_code);

}

static void application_timers_start(void) {						// Start Background Timers Serviced by interupt handlers defined at timers_init()
/**	@brief Function for starting application timers*/   
	
	uint32_t err_code;

    // Start application timers.
 	err_code = app_timer_start(m_sys_timer_id, SYSTEM_TIMER_INTERVAL, NULL);		// 500 ticks
    APP_ERROR_CHECK_CSB(err_code);   
	
//    err_code = app_timer_start(m_ble_timer_id, BLE_MEAS_INTERVAL, NULL);			// 500 ticks ???
//    APP_ERROR_CHECK_CSB(err_code);
}

void app_on_connect(ble_evt_t * p_ble_evt) {						// ble Event to detect start of Bluetooth connection
	    uint32_t err_code = NRF_SUCCESS;

		err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
		APP_ERROR_CHECK_CSB(err_code);
//		nrf_gpio_pin_set(CONNECTED_LED_PIN_NO);
//		nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);

//		err_code = app_button_enable();
		nrf_gpio_pin_set(MOTOR_PIN_NUMBER);
		nrf_delay_ms(1000);
		nrf_gpio_pin_clear(MOTOR_PIN_NUMBER);
	
		APP_ERROR_CHECK_CSB(err_code);

}
void app_on_disconnect(ble_evt_t* p_ble_evt) {						// ble Event to detect end   of Bluetooth connection
	 	    uint32_t err_code = NRF_SUCCESS;
//			nrf_gpio_pin_clear(CONNECTED_LED_PIN_NO);
			err_code = app_button_disable();
            APP_ERROR_CHECK_CSB(err_code);
}

void app_on_adv_timeout(ble_evt_t* p_ble_evt) {						// Event fired after predefined delay ---> 180sec then initiates power down!!!!
	
		uint32_t err_code = NRF_SUCCESS;

		if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISING) {
			err_code = bsp_indication_set(BSP_INDICATE_USER_STATE_1);
			APP_ERROR_CHECK_CSB(err_code);
			
//			nrf_gpio_cfg_sense_input(BSP_BUTTON_0,
//									BUTTON_PULL,
//									NRF_GPIO_PIN_SENSE_LOW);			// Configure buttons with sense level low as wakeup source.
			
//			err_code = sd_power_system_off();							// Go to system-off mode (this function will not return; wakeup will cause a reset)     
//			APP_ERROR_CHECK_CSB(err_code);
//		 	NRF_GPIO->OUTCLR = (1UL << PSHOLD_PIN_NUMBER); 				// Use STM6601 chip to turn power off completely
		}
		ErrorCounter[6] += 1;
}

static void advertising_start(void) {								// Starts broadcast of bluetooth identification
/**	@brief 		Function to start ble advertising.*/
	
	uint32_t err_code;
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
	APP_ERROR_CHECK_CSB(err_code);
}

static void on_ble_evt(ble_evt_t * p_ble_evt) {						// ble event handler
/**	@brief		Function for handling the Application level BLE Stack events.
	@snippet	[Handling the data received over BLE]
	@brief		Callback function for asserts in the SoftDevice.
	@details	This function will be called in case of an assert in the SoftDevice.
	@warning	This handler is an example only and does not fit a final product. You need to analyze
				how your product is supposed to react in case of Assert.
	
	@warning	On assert from the SoftDevice, the system can only recover on reset.
	@param[in]	line_num   Line number of the failing ASSERT call.
	@param[in]	file_name  File name of the failing ASSERT call.
	@param[in]	p_ble_evt   Bluetooth stack event.*/ 
	
	
	uint32_t err_code;

    switch (p_ble_evt->header.evt_id) {
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
			ErrorCounter[2] += 1;									// Monitor Unknowns Note: ErrorCounter[2] sheraed elsewhere
            break;
    }
		UNUSED_VARIABLE(err_code);
}
static void on_adv_evt(ble_adv_evt_t ble_adv_evt) {					// ble advertisement event handler
/**	@brief Function for handling advertising events.
	@details This function will be called for advertising events which are passed to the application.
	@param[in] ble_adv_evt  Advertising event.*/    
	
	uint32_t err_code;

    switch (ble_adv_evt) {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK_CSB(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK_CSB(err_code);

            // enable buttons to wake-up from power off
            err_code = bsp_buttons_enable( (1 << WAKEUP_BUTTON_ID)
                                         | (1 << BOND_DELETE_ALL_BUTTON_ID));
            APP_ERROR_CHECK_CSB(err_code);

		// use STM6601 chip to turn power off completely
		 	NRF_GPIO->OUTCLR = (1UL << PSHOLD_PIN_NUMBER); 

		
            // Go to system-off mode. This function will not return; wakeup will cause a reset.
            err_code = sd_power_system_off();
            APP_ERROR_CHECK_CSB(err_code);
            break;
        default:
            break;
    }
}
static void ble_evt_dispatch(ble_evt_t * p_ble_evt) {				// ble    event dispatcher
/**	@brief 		Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
	
	@details 	This function is called from the BLE Stack event interrupt handler after a BLE stack
				event has been received.
	
	@param[in]	p_ble_evt  Bluetooth stack event*/    
	
	dm_ble_evt_handler(p_ble_evt);  								// Manages Active and Bonded Peers
    ble_conn_params_on_ble_evt(p_ble_evt);
	
	ble_AD7746_on_ble_evt(&m_AD7746, p_ble_evt);    				// TODO: customized service handler
	
	on_ble_evt(p_ble_evt); 											// TODO: common application handler for ble event
	ble_advertising_on_ble_evt(p_ble_evt);   						// Monitor and restart advertising on Disconnection
	
	ErrorCounter[7] += 1;
}
static void sys_evt_dispatch(uint32_t sys_evt) {					// system event dispatcher
/**	@brief 		Function for dispatching a system event to interested modules.
	@details 	This function is called from the System event interrupt handler after a system
				event has been received.
	@param[in] 	sys_evt  System stack event.**/

	pstorage_sys_event_handler(sys_evt);
    
	ble_advertising_on_sys_evt(sys_evt);
	
	ErrorCounter[8] += 1;
}


static void ble_stack_init(void) {									// Initalise ble Bluetooth stack also initalises Low Frequency Clock Source --> LF RC or crystal
/**	@brief Function for initializing the BLE stack.
	@details Initializes the SoftDevice and the BLE event interrupt.*/    
	
	uint32_t err_code;

	SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_250MS_CALIBRATION,NULL);			// Low Frequency Clock Source --> Internal 32kHz RC Oscillator;

//	SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_TEMP_16000MS_CALIBRATION,NULL);	// Temperature compensated LF RC Osc calibration every 16seconds
	
// WARNING --> NOT USED ---> Alternative LF Clock Source 
//	SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM,NULL);						// Low Frequency Clock Source --> External Xtal/Resonator 32k682kHz

#ifdef S110																				// Initialize the SoftDevice handler module.
	ble_enable_params_t ble_enable_params;												// Enable BLE stack.
	memset(&ble_enable_params, 0, sizeof(ble_enable_params));
	ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
	APP_ERROR_CHECK_CSB(err_code);
#endif
  
	err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);						// Register with the SoftDevice ble event handler module for BLE events.
    APP_ERROR_CHECK_CSB(err_code);

    
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);						// Register with the SoftDevice system event handler module for BLE events.
    APP_ERROR_CHECK_CSB(err_code);
}

static void DeviceNameFromID(char* name, int len) {					// Redefine Device name with Device ID + Device Name based on DS2401 Address
	if(len<16) {
		strncpy(name, DEVICE_NAME, len);
	} else {
		while(!ds2401_initAndRead()) ;
		int j = 0;
		bool skipLeading = true;
		for(int i=0;i<8;i++)
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

static void GrabDeviceID(char* name, int len) {						// DS2401 xxxxxxxx
	if(len<16) {
		strncpy(name, DEVICE_NAME, len);
	} else {
		while(!ds2401_initAndRead()) ;
		int j = 0;
		bool skipLeading = true;
		for(int i=0;i<8;i++)
		{
			if(ds2401_id[i]==0 && skipLeading) continue;
			else skipLeading = false;
			sprintf(name+j*2, "%02X", ds2401_id[i]);
			j++;
		}
		if (j*2<len) {
			strncpy(name+j*2, "", len-j*2);
		}
	}
	return;
}
static void gap_params_init(void) {									// Initialise ble Generic Access Profile GAP parameters
/**	@brief 		Function for the GAP initialization.
	@details 	This function sets up all the necessary GAP (Generic Access Profile) parameters of the
				device including the device name, appearance, and the preferred connection parameters.*/    
	uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
	char IDName[16];
	DeviceNameFromID(IDName,16);
	
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)IDName,
                                          15);
    APP_ERROR_CHECK_CSB(err_code);

    err_code = sd_ble_gap_appearance_set(DEVICE_APPEARANCE);
    APP_ERROR_CHECK_CSB(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK_CSB(err_code);
										  
    err_code = sd_ble_gap_tx_power_set(TX_POWER_LEVEL);
    APP_ERROR_CHECK_CSB(err_code);
}
static void services_init(void) {									// Initialise Services Used within Application
/**	@brief Function for initializing services that will be used by the application.
	@details Initialize the Heart Rate, Battery and Device Information services.*/	
	
	uint32_t       err_code;
    ble_AD7746_init_t ad7746_init;
	memset(&ad7746_init, 0, sizeof(ad7746_init));
	
	ad7746_init.on_write_config_callback = ad7746_on_write_config_callback;
	
	err_code = ble_AD7746_init(&m_AD7746, &ad7746_init);
    APP_ERROR_CHECK_CSB(err_code);
	
//	ble_nus_init_t   nus_init;				//nus == Nordic UART Service
//	memset(&nus_init, 0, sizeof(nus_init));
//	nus_init.data_handler = nus_data_handler;
//   
//	err_code = ble_nus_init(&m_nus, &nus_init);
//	APP_ERROR_CHECK_CSB(err_code);
}
static void advertising_init(void) {								// Initalise ble Bluetooth Advertising 
/**	@brief 		Function for initializing the Advertising functionality.
	@details	Encodes the required advertising data and passes it to the stack.
				Also builds a structure to be passed to the stack when starting advertising.*/
	
	uint32_t      err_code;
    ble_advdata_t advdata;
	ble_advdata_t scanrsp;
	
	// make sure service_init is called before advertising_init
	ble_uuid_t adv_uuids[] = {{AD7746_UUID_SERVICE, m_AD7746.uuid_type},		// uuid_type will be filled by ble_AD7746_init
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
    APP_ERROR_CHECK_CSB(err_code);

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK_CSB(err_code);
}
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt) {		// Function to handle ble Bluetooth Connection Events as defined at conn_parms_init()
/**	@brief		Function for handling the Connection Parameters Module.
	
	@details	This function will be called for all events in the Connection Parameters Module which are passed to the application.
	@note		All this function does is to disconnect. This could have been done by simple setting the disconnect_on_fail config parameter, 
				but instead we use the event handler mechanism to demonstrate its use.
	@param[in]	p_evt  Event received from the Connection Parameters Module. */
    
	uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK_CSB(err_code);
    }
}


static void conn_params_error_handler(uint32_t nrf_error) {			// Function for handling a Connection Parameters error.
/**	@brief Function for handling a Connection Parameters error.
	@param[in] nrf_error  Error code containing information about what went wrong.*/	
    APP_ERROR_HANDLER(nrf_error);
	
	ErrorCounter[1] += 1;
}


static void conn_params_init(void) {								// Initalise ble Bluetooth Connection Parameters
/**	@brief Function for initialising the Connection Parameters module.  */	
	
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
    APP_ERROR_CHECK_CSB(err_code);
}

static void scheduler_init(void) {									// Initalise Scheduler
/**@brief Function for the Event Scheduler initialization.  */	

    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

static uint32_t device_manager_evt_handler							// ToDo ---> Empty
	(dm_handle_t const * p_handle,
	 dm_event_t  const * p_event,
     ret_code_t          event_result) {	
				   
/**	@brief		Function for handling the Device Manager events.
	@param[in]	p_evt  Data associated to the device manager event.*/											   
    
	APP_ERROR_CHECK_CSB(event_result);

#ifdef BLE_DFU_APP_SUPPORT
    if (p_event->event_id == DM_EVT_LINK_SECURED) {
        app_context_load(p_handle);
    }
#endif 																// BLE_DFU_APP_SUPPORT
		ErrorCounter[2] += 1;										// Note ErrorCounter[2] Shared elsewhere
    return NRF_SUCCESS;
}
static void device_manager_init(void) {
/**	@brief Function for the Device Manager initialization.  */
	
	uint32_t               err_code;
    dm_init_param_t        init_data;
    dm_application_param_t register_param;

    err_code = pstorage_init();    												// Initialise persistent storage module.
    APP_ERROR_CHECK_CSB(err_code);

    // Clear all bonded centrals if the Bonds Delete button is pushed.
    err_code = bsp_button_is_pressed(BOND_DELETE_ALL_BUTTON_ID,&(init_data.clear_persistent_data));
    APP_ERROR_CHECK_CSB(err_code);

    err_code = dm_init(&init_data);
    APP_ERROR_CHECK_CSB(err_code);

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
    
	APP_ERROR_CHECK_CSB(err_code);
}

	
/** @brief		Function for handling the data from the Nordic UART Service. ---> BYPASSED At This time
				static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length) {

	@snippet	[Handling the data received over BLE]
	
	@details	This function will process the data received from the Nordic UART BLE Service and send
				it to the UART module.

	@param[in]	p_nus    Nordic UART Service structure.
	@param[in]	p_data   Data to be send to UART module.
	@param[in]	length   Length of the data.

    for (uint32_t i = 0; i < length; i++) {
        while(app_uart_put(p_data[i]) != NRF_SUCCESS);
    }
    while(app_uart_put('\n') != NRF_SUCCESS);
}
*/

 
#ifdef DEBUG_NRF_USER												// ble debug error handler
/**	@brief		Function for error handling, which is called when an error has occurred.
	@warning	This handler is an example only and does not fit a final product. You need to analyze
				how your product is supposed to react in case of error.
				
	@param[in]	error_code  Error code supplied to the handler.
	@param[in]	line_num    Line number where the handler is called.
	@param[in]	p_file_name Pointer to the file name.*/

#include "ble_debug_assert_handler.h" 

void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name) {

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
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name) {
	return;		/////
		
//		if (line_num == 1371) {
//				return;
//		}
//    app_error_handler(0xDEADBEAF, line_num, p_file_name);
}

#endif	//  endif DEBUG_NRF_USER
void TestProcessorHWID(void) {										// Test nRF51822 Processor HWID ---> Under Developement requires SDK 8.1.0 or higher
	
	uint32_t ic_data;
	ic_data = (((*((uint32_t volatile *)0xF0000FE8)) & 0x000000FF) );
	if (ic_data==0x0084) setbit(&Process_5,0);													// HWID=84 ==> nRF51822 xxAC Flash:512k Ram:32k
																								// Note HWID reads 4C for nRF51822 xxAA Flash:256k Ram:16k
	
	uint16_t ram_size	= (uint16_t) NRF_FICR->NUMRAMBLOCK * (NRF_FICR->SIZERAMBLOCKS / 1024);	// FICR ---> Factory Information Config Register
	if (ram_size != 0x0020) clrbit(&Process_5,0);												// ram_size==0x0020 --> 32k  ram_size == 0x0010 --> 16k   
	
	uint16_t flash_size	= (uint16_t) NRF_FICR->CODESIZE;										// flash_size==0x0100 --> 256k
	if (flash_size != 0x0100) clrbit(&Process_5,0);
	
	ic_data = (((*((uint32_t volatile *)0xF0000FE8)) & 0x000000F0) >> 4);						// Check Hardware Revision
	
	switch (ic_data) {
       
		case 1: 						// IC revision 1	IC_REVISION_NRF51_REV1;
			setbit(&Process_0,4);																// Assert Vibro Motor to Signal Error
			break;

		case 4: 						// IC revision 2	IC_REVISION_NRF51_REV2;
			setbit(&Process_0,4);																// Assert Vibro Motor to Signal Error
			break;

		case 7:							// IC revision 3	Fall Through
		case 8:							// IC revision 3	Fall Through
		case 9:							// IC_REVISION_NRF51_REV3
			setbit(&Process_2,7);																// <-- Should End Up Here for Rev3
		break;

		default:						// IC_REVISION_NRF51_UNKNOWN;
			setbit(&Process_0,4);																// Assert Vibro Motor to Signal Error
			break;
    }
}

void Test_LiPo_Battery_for_ALARM_Shutdown() {						// Test Battery Voltage and Force Shutdown if b/n 1V5 and 3V6
	int value;
	float temp;
	
	if (!ltc294x_get_voltage(&value))								// ToDo ERROR GasGauge Voltage Intermitent Occassionally Returns 0000
	temp = (float) value;
	temp = temp / 1000000;
	if ((temp>=1.5) && (temp <=3.6)){								// LiPo Battery Voltage 4V1=Full  3V9=75%  3V8=50%   3V7=25%   3V65 = 12% == (ALARM-->ShutDown)
																	// Trap for temp Value less than 1 --> probably means a misread ToDO sort out this bug and fix
			
		nrf_gpio_pin_set(MOTOR_PIN_NUMBER);							// Strobe Vibro Motor 2000msec 
		nrf_delay_ms(1000);
		nrf_gpio_pin_clear(MOTOR_PIN_NUMBER);

		// use STM6601 chip to turn power off completely
		NRF_GPIO->OUTCLR = (1UL << PSHOLD_PIN_NUMBER); 				// Power Down --- ShutDown and Save Battery -- Vbatt < 3V6 is at Critical Supply Voltage --> Capacity < 12%
		}
	
	return;
	}

	
void WatchDog_init() {;												// ToDo nRF51822 Hardware WatchDog Initalisation
/**	@brief		Function WatchDog_init() used to Initalise hardware WatchDog.
	@details 	
	@note  		
	@ref 
	@todo		Write Routine to activate ARM nrf51822 WatchDog probably with 5 second delay */	
	
}

void PatTheDog() {													// ToDo Pat The Dog and WatchDog Timer Happy ---> nRF51 Hardware WatchDog
/**	@brief		Function PatTheDog() used to keep WatchDog Alive and Well.
	@details 	
	@note  		
	@ref 
	@todo		Write Routine to activate ARM nrf51822 WatchDog probably with 5 second delay */	
	
	}

void TestTemperatures () {											// Test uP and Peripheral Temperatures
/**	@brief		Function to test uP and peripheral temperatures ---> Clear Process_5.7 if Temperature High.
	@details 	
	@note  		
	@ref 
	@todo		
*/	
	int value;
	float temp;
	bool flag=true;

	// ToDo Test Ptrocess Temperature
	
	if( (float) MPU9250_get_temperature() > 50) flag=false;					// Inertial Sensor Temperature

	if((float) MPL3115A2_getTemperature() >50) flag=false;					// Pressure Sensor Temperature	

	if (!ltc294x_get_temperature(&value)) {									// GasGauge Temperature
		temp=value/100;
		if(temp> 50) flag=false;
	}
	
	if(flag==false) clrbit(&Process_5,7);									// if Process_5.7 ---> 
	else setbit(&Process_5,7);
}

void GPIO_config_all_init() {										// Configure nRF51 GPIO Pins as required.

/**	@brief		Function GPIO_config_all_init() Configures GPIO Pins.

	@details	
	@note  		
	@ref		
*/	

//	Configure Outputs.
	NRF_GPIO->PIN_CNF[PSHOLD_PIN_NUMBER] =     									// Configure PSU Hold as Output
		(GPIO_PIN_CNF_SENSE_Disabled 	<< GPIO_PIN_CNF_SENSE_Pos)
	  | (GPIO_PIN_CNF_DRIVE_S0S1     	<< GPIO_PIN_CNF_DRIVE_Pos)
	  | (GPIO_PIN_CNF_PULL_Pullup    	<< GPIO_PIN_CNF_PULL_Pos)
	  | (GPIO_PIN_CNF_INPUT_Disconnect  << GPIO_PIN_CNF_INPUT_Pos)
	  | (GPIO_PIN_CNF_DIR_Output      	<< GPIO_PIN_CNF_DIR_Pos);

	NRF_GPIO->DIRSET = (1UL << PSHOLD_PIN_NUMBER);
	NRF_GPIO->OUTSET = (1UL << PSHOLD_PIN_NUMBER);								// Assert HIGH 
	nrf_gpio_pin_set(AUDIO_CNTRL_PIN_NUMBER);									// Alternatae Re-Assert HIGH for PSU Hold --> Assert Low for ShutDown

	
	NRF_GPIO->PIN_CNF[SMART_RST_PIN_NUMBER] =     					
		(GPIO_PIN_CNF_SENSE_Disabled 	<< GPIO_PIN_CNF_SENSE_Pos)	
	  | (GPIO_PIN_CNF_DRIVE_S0S1     	<< GPIO_PIN_CNF_DRIVE_Pos)	
	  | (GPIO_PIN_CNF_PULL_Pulldown    	<< GPIO_PIN_CNF_PULL_Pos) 	
	  | (GPIO_PIN_CNF_INPUT_Disconnect  << GPIO_PIN_CNF_INPUT_Pos)	
	  | (GPIO_PIN_CNF_DIR_Output      	<< GPIO_PIN_CNF_DIR_Pos);				// Configure SMARTRST as Output

	NRF_GPIO->DIRSET = (1UL << SMART_RST_PIN_NUMBER);
	NRF_GPIO->OUTCLR = (1UL << SMART_RST_PIN_NUMBER);							// Assert Low 
	nrf_gpio_pin_clear(AUX_MOTOR_PIN_NUMBER);									// Alternate Re-Assert LOW --> Note: Asserted High forces Shutdown via SMART RESET
	

	NRF_GPIO->PIN_CNF[AUDIO_CNTRL_PIN_NUMBER] =									// Configure AUDIO_CNTRL_PIN_NUMBER as Output
		(GPIO_PIN_CNF_SENSE_Disabled 	<< GPIO_PIN_CNF_SENSE_Pos)
	  | (GPIO_PIN_CNF_DRIVE_S0S1     	<< GPIO_PIN_CNF_DRIVE_Pos)
	  | (GPIO_PIN_CNF_PULL_Disabled    	<< GPIO_PIN_CNF_PULL_Pos)
	  | (GPIO_PIN_CNF_INPUT_Disconnect  << GPIO_PIN_CNF_INPUT_Pos)
	  | (GPIO_PIN_CNF_DIR_Output      	<< GPIO_PIN_CNF_DIR_Pos);

	NRF_GPIO->DIRSET = (1UL << AUDIO_CNTRL_PIN_NUMBER);
	NRF_GPIO->OUTSET = (1UL << AUDIO_CNTRL_PIN_NUMBER);							// Assert HIGH for Standby Powerdown
	nrf_gpio_pin_set(AUDIO_CNTRL_PIN_NUMBER);									// Alternatae Re-Assert HIGH for Standby Powerdown	

	NRF_GPIO->PIN_CNF[AUDIO_SOURCE_PIN_NUMBER] =								// Configure AUDIO_CNTRL_PIN_NUMBER as Output
		(GPIO_PIN_CNF_SENSE_Disabled 	<< GPIO_PIN_CNF_SENSE_Pos)
	  | (GPIO_PIN_CNF_DRIVE_S0S1     	<< GPIO_PIN_CNF_DRIVE_Pos)
	  | (GPIO_PIN_CNF_PULL_Disabled   	<< GPIO_PIN_CNF_PULL_Pos)
	  | (GPIO_PIN_CNF_INPUT_Disconnect  << GPIO_PIN_CNF_INPUT_Pos)
	  | (GPIO_PIN_CNF_DIR_Output      	<< GPIO_PIN_CNF_DIR_Pos);

	NRF_GPIO->DIRSET = (1UL << AUDIO_SOURCE_PIN_NUMBER);
	NRF_GPIO->OUTCLR = (1UL << AUDIO_SOURCE_PIN_NUMBER);						// Assert LOW for Standby AC Coupled PWM Path
	nrf_gpio_pin_clear(AUDIO_SOURCE_PIN_NUMBER);			
	
	NRF_GPIO->PIN_CNF[AUX_MOTOR_PIN_NUMBER] =									// Configure AUX_MOTOR_PIN_NUMBER as Output
		(GPIO_PIN_CNF_SENSE_Disabled 	<< GPIO_PIN_CNF_SENSE_Pos)
	  | (GPIO_PIN_CNF_DRIVE_S0S1     	<< GPIO_PIN_CNF_DRIVE_Pos)
	  | (GPIO_PIN_CNF_PULL_Pulldown    	<< GPIO_PIN_CNF_PULL_Pos)
	  | (GPIO_PIN_CNF_INPUT_Disconnect  << GPIO_PIN_CNF_INPUT_Pos)
	  | (GPIO_PIN_CNF_DIR_Output      	<< GPIO_PIN_CNF_DIR_Pos);

	NRF_GPIO->DIRSET = (1UL << AUX_MOTOR_PIN_NUMBER);
	NRF_GPIO->OUTCLR = (1UL << AUX_MOTOR_PIN_NUMBER);							// Assert LOW Output Driven via N-CH FET
	nrf_gpio_pin_clear(AUX_MOTOR_PIN_NUMBER);									// Alternate Re-Assert LOW Output Driven via N-CH FET
	
	
	NRF_GPIO->PIN_CNF[SMART_RST_PIN_NUMBER] =     					
		(GPIO_PIN_CNF_SENSE_Disabled 	<< GPIO_PIN_CNF_SENSE_Pos)	
	  | (GPIO_PIN_CNF_DRIVE_S0S1     	<< GPIO_PIN_CNF_DRIVE_Pos)	
	  | (GPIO_PIN_CNF_PULL_Pulldown    	<< GPIO_PIN_CNF_PULL_Pos) 	
	  | (GPIO_PIN_CNF_INPUT_Disconnect  << GPIO_PIN_CNF_INPUT_Pos)	
	  | (GPIO_PIN_CNF_DIR_Output      	<< GPIO_PIN_CNF_DIR_Pos);				// Configure SMARTRST as Output

	NRF_GPIO->DIRSET = (1UL << SMART_RST_PIN_NUMBER);
	NRF_GPIO->OUTCLR = (1UL << SMART_RST_PIN_NUMBER);							// Assert Low 
	nrf_gpio_pin_clear(AUX_MOTOR_PIN_NUMBER);									// Alternate Re-Assert LOW --> Note: Asserted High forces Shutdown via SMART RESET


//	Configure Inputs with Weak Pullups or pulldowns as required.
	nrf_gpio_cfg_sense_input(BSP_BUTTON_0,
						BUTTON_PULL,
						NRF_GPIO_PIN_SENSE_LOW);								// Configure button-0 with sense level low as wakeup source.
	
	nrf_gpio_cfg_sense_input(BSP_BUTTON_1,
						BUTTON_PULL,
						NRF_GPIO_PIN_SENSE_LOW);								// Configure button-1 with sense level low as wakeup source.
						

//#define GAUGE_NALCC_PIN_NUMBER	(12U)				// INPUT (INT) Gas Gauge Interupt I2C Programmable Alarm States LTC2943.pin6
//#define INERTIAL_INT_PIN_NUMBER	(03U)				// INPUT (INT) Inertial Sensor Interupt MPU9250.pin12
//#define PRESS1_PIN_NUMBER			(29U)				// INPUT (INT) Pressure Sensor PRESS1 Interupt MPL3115.pin6
//#define PRESS2_PIN_NUMBER			(02U)				// INPUT (INT) Pressure Sensor PRESS2 Interupt MPL3115.pin5
//#define GPS_INT_PIN_NUMBER		(14U)				// INPUT (INT) GPS External Interupt A2035-H.pin19


/* TODO
	Pressure  Sensor interupts PRESS1 and PRESS2
	Speaker Audio Output Source and Audio Control
	Auxillary Motor Haptic
	LED_Red
	GPS interupt
	MPU interupt
*/

}

void WatchDogTimer_init(void){										// Cofigure WatchDog Timer ---> ToDo
//	uint32_t   err_code;
//   
//	//Configure WDT.
//    nrf_drv_wdt_config_t config = {
//	//	NRF_DRV_WDT_DEAFULT_CONFIG;
//	
//	
//	
//	};
//    err_code = nrf_drv_WatchDogTimer_init(&config, wdt_event_handler);
//    APP_ERROR_CHECK(err_code);
//    err_code = nrf_drv_wdt_channel_alloc(&m_channel_id);
//    APP_ERROR_CHECK(err_code);
//    nrf_drv_wdt_enable();

}

int main(void) {													// $$$$$$$$$$$  PROGRAM ENTRY ---> main()

/**	@brief		Function main() application entry.
	@details	
	@note  		
	@ref		
	@todo		*/
	
	uint32_t err_code;
	
	GrabDeviceID(DS2401_ID,16);										// Grab and store 8-bit Hex Device ID

	ble_stack_init();												// Initialize softdevice stack. ---> s110

	GPIO_config_all_init();											// Initialise GPIO Pins as Required Outputs and Inputs
	
	sensorADC_config();												// Initalise SensorADC for Battery Voltage Measurement at ADC.AIN6

//	Initialize Common modules
	scheduler_init();												// Initialise Scheduler
	device_manager_init();											// Initialise Device Manager  
	timers_init();													// Initialise Timers  
	gap_params_init();												// Initialise GAP and GATT 
	services_init();												// Initialise service
	advertising_init();												// Initialise Advertising
	conn_params_init();												// Initialise connection parameters
	application_timers_start();										// Start Timers
	
	SmartCane_peripheral_init();									// Initialise application specific modules
	
//	advertising_start();											// Starts Bluetooth advertising, however, also shuts down power to board after 180sec if no connection
																	// Note... Moved to VT100 System Menu 6 ---> G
	
	const app_uart_comm_params_t comm_params =	{					// Load UART Parameters ... USB UART Virtual COM port BAUD==11500
		  RX_PIN_NUMBER,
		  TX_PIN_NUMBER,
		  RTS_PIN_NUMBER,
		  CTS_PIN_NUMBER,
		  APP_UART_FLOW_CONTROL_DISABLED,
		  false,
		  UART_BAUDRATE_BAUDRATE_Baud115200
	  };

	APP_UART_FIFO_INIT(&comm_params,								// Initalise UART FIFO with defined setup parameters
		UART_RX_BUF_SIZE,
		UART_TX_BUF_SIZE,
		uart_error_handle,
		APP_IRQ_PRIORITY_LOW,
		err_code);
	  
	APP_ERROR_CHECK_CSB(err_code);									// Application Error Checker ---> error.h

	TestProcessorHWID();
	
	WatchDogTimer_init();
	  
	UART_VT100_Main_Menu();											// Initialise Default UART VT100 Main Menu
	
	int TSA_Count=0;												// Base TSA Initialisation Start Count = 0 of  TSA_Count 0..25	
	while (true) {													// TSA Time Slot Assigner ---> Endless Loop within main()
/**	@brief 		MASTER TIME SLOT ASSIGNER --> TSA
	@details 	This routine operates as a never ending loop within 
				main() and managers background tasks based on a 
				rotating counter TSA_Count.
	@note  		This while Loop Operates as a Background Task Manager */
		
		app_sched_execute();										// Execute Task Scheduler per loop
		
		TSA_Count +=1;												// Increment TSA loop count
		if (TSA_Count>=26) TSA_Count=0;								// Reset the TSA Count to 0

		switch (TSA_Count) {										// Background Task Scheduler
		
		case 0:														// Loads VT100 Rehresh Menus and test for UART Rx characters
			VT100_Scan_Keyboard_All_Menues();		
			break;
		
		case 1:														// ToDo Collect GPS Navigation Data and Load Bearing to Next Waypoint
			break;
		
		case 2:														// ToDo Upon Active GPS Waypoint Zone - Use Inertial/Compass Data to Signal Correct Bearing
			break;
		
		case 3:														// ToDo During Active Movement Collect User Parametrics. Walking Pace, Stick Movement, Gestures
			break;
		
		case 4:														// Test for Process Flags 
			if (testbit(&Process_3, 0)==true) {						// Process_3.0 flag to start ble advertising
				if(testbit(&Process_3, 1)==false) {
					advertising_start();							// Starts Bluetooth advertising, however,shutdown initiated after 180sec ---> Need to reschedule or STOP
					setbit(&Process_3, 1);
					clrbit(&Process_3, 0);					
				}
			}
			break;
			
		case 5:														// To Be Assigned 
			
		break;
			
		case 6:														// To Be Assigned 
			
			break;
		
		case 18:													// To Be Assigned
			break;
		
		case 19:													// System Monitoring and Health Checking
			Test_LiPo_Battery_for_ALARM_Shutdown();
			TestTemperatures();			
			break;
		
		case 20:													// ToDo State Machine to Control Haptic Transducer Vibration States and On-Off Sequences
			break;
			
		case 21:													// ToDo State Machine to Signal Audio Tone Sequences at Various Output Frequencies
			break;
		
		case 22:													// ToDo State Machine to start Audio Tone Sequence at Various Output Frequencies
			break;
			
		case 23:													// ToDo Monitor for physical movement and/or physical activity 
			break;
			
		case 24:													// ToDo Monitor Battery State and System Temperatures
			break;
			
		case 25:													// Pat the Dog every 50 loops of TSA
			DogCount += 1;	
			if (DogCount >= 50) {
				DogCount=0;
				PatTheDog();
			}
			
			break;
		
		default:
			break;
		}
	}
}
