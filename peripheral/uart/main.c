/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/* @file
 * @defgroup uart_example_main main.c
 * @{
 * @ingroup uart_example
 * @brief UART Example Application main file.
 *
 * This file contains the source code for a sample application using UART.
 * 
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>								//Required for sqrt and other high order functions

#include "app_error.h"
#include "app_uart.h"

#include "nrf_delay.h"
#include "nrf.h"
#include "nrf_temp.h"

#include "bsp.h"

#include "Communication.h"

#include "ds2401.h"

#include "MPU_9150.h"  

#include "inv_mpu.h"

#include "MPL3115.h"

#include "ltc2943.h"

#define DEVICE_NAME				"SCANE1V0"			//Keep under 8 characters to allow 8 char DS2401 ID to be pre appended (Max --> 16)

#define MAX_TEST_DATA_BYTES     (55U)               /**< max number of test bytes per TX Burst to be used for tx and rx. */

#define UART_TX_BUF_SIZE 		512                 /**< UART TX buffer size. ... Needs to be 512*/
#define UART_RX_BUF_SIZE 		16					/**< UART RX buffer size. Modified from 1 to 16 */


//	************************************************************
//
//		NRF51822 Port P0.xx IO Assignments SMARTCANE PCB 1V0
//
//	************************************************************

#define PSHOLD_PIN_NUMBER		(28U)				//Assert High 3V3 Power ---> Upon SW1 Long Push STM6601.pin4
#define SMART_RST_PIN_NUMBER	(01U)				//Assert HIGH Force a 3V3 Rail Shutdown of STM6601.pin2 via FET (Requires Physical Button Push to Iniatiate Power-Up)

#define PB_SW1_PIN_NUMBER		(06U)				//INPUT PBOUT SW1 Image of Push Button State (Normally High) STM6601.pin8
#define PB_SW2_PIN_NUMBER		(11U)				//INPUT SW2 Push Button (Normally High via 3V3 Pull-up) Active Low

#define MOTOR_PIN_NUMBER		(07U)				//Assert HIGH Enables 1V2 Linear Regulator to Drive Onboard 9000RPM Haptic Vibration Motor RT9030.pin3
#define AUX_MOTOR_PIN_NUMBER	(25U)				//Assert HIGH Provides Switched Ground Sink Path for 3V3 Rail via 33R Series Resistance

#define AUDIO_CNTRL_PIN_NUMBER	(23U)				//Assert LOW To ENABLE onboard Audio Amplifier used to Drive Off Board Speaker LM4800.pin5
#define AUDIO_SOURCE_PIN_NUMBER	(00U)				//Analog OUTPUT - AC Coupled Square Wave providing signal to Speaker Audio Amp LM4880.pin2

#define LED_RED_PIN_NUMBER		(30U)				//Assert LOW To Enable LED1 RED Diagnostic LED

#define GPS_NEN_PIN_NUMBER		(04U)				//Assert LOW GPS Not Enable - Disables Secondary Linear 3V3 Regulator running directly off VBAT
#define GPS_ONOFF_PIN_NUMBER	(13U)				//Assert HIGH-LOW-HIGH GPS Power Up-Down On-Off Sequencer A2035-H.pin18
#define GPS_NRST_PIN_NUMBER		(10U)				//Assert LOW  GPS Reset A2035-H.pin1

#define ADC_VBAT_PIN_NUMBER		(05U)				//Analog Signal tied to 150k//680k Voltage Divider Measuring VBAT ratio

#define GAUGE_NALCC_PIN_NUMBER	(12U)				//INPUT (INT) Gas Gauge Interupt I2C Programmable Alarm States LTC2943.pin6.
#define INERTIAL_INT_PIN_NUMBER	(03U)				//INPUT (INT) Inertial Sensor Interupt MPU9250.pin12
#define PRESS1_PIN_NUMBER		(29U)				//INPUT (INT) Pressure Sensor PRESS1 Interupt MPL3115.pin6
#define PRESS2_PIN_NUMBER		(02U)				//INPUT (INT) Pressure Sensor PRESS2 Interupt MPL3115.pin5
#define GPS_INT_PIN_NUMBER		(14U)				//INPUT (INT) GPS External Interupt A2035-H.pin19


char DS2401_ID[16];									// String to hold ---> DS2401 (4bits) + Device_ID
char cStatus[] = "Ready                        ";	// VT100 Status Updates
uint8_t MenuLevel=0;								// Initalise Default VT100 Menu Level to 00 --> Main Menu
float Quaternion[4];								// Use to hold Inertial Quaternion wxyz



void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

static void DeviceNameFromID(char* name, int len)	//Redefine Device name with Device ID + Device Name
{
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

static void GrabDeviceID(char* name, int len)		//DS2401 xxxxxxxx
{
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
/** @brief 		Function to load a single top level VT100 Terminal Menu. 
 *  @details 	Transmitts VT100 ESC Sequenceone character to clear screen and load menu data.
 *  @note  		ASCII DEC 27 == x1B    VT100 ClrScreen == <ESC>[2J    Home == ESC[H    Cursor Location === <ESC>[{ROW};{COLUMN}H
 *  @ref 		TX_PIN_NUMBER must be connected to @ref RX_PIN_NUMBER)
 */
static void UART_VT100_Main_Menu()						// $$$$$$ TOP LEVEL MENU $$$$$$
{
	
//  Static menu loaded once and refreshed with menu level values elswhere.
		MenuLevel=00;

		printf("\x1B[2J");								//VT100 CLR SCREEN
		printf("\x1B[H");								//VT100 CURSOR HOME
		printf("\x1B[01;10H  GDV-UoM SMARTCANE MAIN MENU");
		printf("\x1B[01;47HDEVICE ID = ");
		printf("\x1B[02;47H   STATUS = ");

		printf("\x1B[04;10H  1... All Sensors");
		printf("\x1B[06;10H  2... GPS Global Position");
		printf("\x1B[08;10H  3... Inertial Sensors");
		printf("\x1B[10;10H  4... Altitude and Temperature");
		printf("\x1B[12;10H  5... Memory Functions");
		printf("\x1B[14;10H  6... Power Management");
		printf("\x1B[16;10H  7... Cane Diagnostics");
		printf("\x1B[18;10H  8... System Diagnostics");
		printf("\x1B[22;10H  9... Shutdown");

		printf("\x1B[24;10H  ?... Help");
}

/** @brief 		Function to load top level Main Menu to VT100 Terminal Screen. 
 *  @details 	Loads VT100 ESC Sequenceone character to clear screen and load menu data.
 *  @note  		ASCII DEC 27 == x1B    VT100 ClrScreen == <ESC>[2J    Home == ESC[H    Cursor Location === <ESC>[{ROW};{COLUMN}H
 *  @ref 		VT100 Terminal Emulation Nominal Baud:38400
 */
static void UART_VT100_Menu_1()							// $$$$$$  ALL SENSORS  $$$$$$
{		MenuLevel=10;

		printf("\x1B[2J");								//VT100 CLR SCREEN
		printf("\x1B[H");								//VT100 CURSOR HOME
		printf("\x1B[01;05H GDV-UoM SMARTCANE MENU-1  ALL SENSORS and SYSTEMS");
		printf("\x1B[04;05H GRAVITY g");
		printf("\x1B[05;05H ACC-X = ");
		printf("\x1B[06;05H ACC-Y = ");
		printf("\x1B[07;05H ACC-Z = ");
		printf("\x1B[08;05H ACC-M = ");

		nrf_delay_ms(15);

		printf("\x1B[04;30H GYROSCOPE deg");
		printf("\x1B[05;30H GYRO-X = ");
		printf("\x1B[06;30H GYRO-Y = ");
		printf("\x1B[07;30H GYRO-Z = ");
		printf("\x1B[08;30H GYRO-M = ");
	
		nrf_delay_ms(15);

		printf("\x1B[04;55H COMPASS ");
		printf("\x1B[05;55H Compass-X = ");
		printf("\x1B[06;55H Compass-Y = ");
		printf("\x1B[07;55H Compass-Z = ");
		printf("\x1B[08;55H Compass-M = ");

		nrf_delay_ms(15);
		 
		printf("\x1B[11;05H GPS");
		printf("\x1B[12;05H Latitude   = ");
		printf("\x1B[13;05H Longditude = ");
		printf("\x1B[14;05H Time       = ");

		nrf_delay_ms(15);
		 
		printf("\x1B[11;30H TEMPERATURE");
		printf("\x1B[12;30H ProcessorT = ");
		printf("\x1B[13;30H CompassT   = ");
		printf("\x1B[14;30H PressureT  = ");
		printf("\x1B[15;30H GasGaugeT  = ");
		
		nrf_delay_ms(15);
		
		printf("\x1B[11;55H HEIGHT");
		printf("\x1B[12;55H Pressure  = ");
		printf("\x1B[13;55H Altitude  = ");
		

		nrf_delay_ms(15);
			
		printf("\x1B[24;05H  X... exit    ?...Help");
}	

/** @brief 		Function to load GPS GLOBAL POSITION MENU to VT100 Terminal Screen. 
 *  @details 	Loads VT100 ESC Sequenceone character to clear screen and load menu data.
 *  @note  		ASCII DEC 27 == x1B    VT100 ClrScreen == <ESC>[2J    Home == ESC[H    Cursor Location === <ESC>[{ROW};{COLUMN}H
 *  @ref 		VT100 Terminal Emulation Nominal Baud:38400
 */
static void UART_VT100_Menu_2()							// $$$$$$  GPS GLOBAL POSITION MENU  $$$$$$
{		MenuLevel=20;
	
		printf("\x1B[2J");			//VT100 CLR SCREEN
		printf("\x1B[H");			//VT100 CURSOR HOME
		printf("\x1B[01;05H GDV-UoM SMARTCANE MENU-2  GPS GLOBAL POSITION SENSOR");
		 
		printf("\x1B[04;05H GPS");
		printf("\x1B[05;05H Latitude   = ");
		printf("\x1B[06;05H Longditude = ");
		printf("\x1B[07;05H Height     = ");
		printf("\x1B[08;05H Time       = ");

		nrf_delay_ms(15);
		 
		printf("\x1B[05;30H SatNumber = ");
		printf("\x1B[06;30H Spare     = ");
		printf("\x1B[07;30H Spare     = ");
		printf("\x1B[08;30H Spare     = ");
		printf("\x1B[09;30H Spare     = ");
		
		nrf_delay_ms(15);
    	    
	  printf("\x1B[24;05H  X... exit    ?...Help");
}	

/** @brief 		Function to load INERTIAL SENSOR MENU to VT100 Terminal Screen. 
 *  @details 	Loads VT100 ESC Sequenceone character to clear screen and load menu data.
 *  @note  		ASCII DEC 27 == x1B    VT100 ClrScreen == <ESC>[2J    Home == ESC[H    Cursor Location === <ESC>[{ROW};{COLUMN}H
 *  @ref 		VT100 Terminal Emulation Nominal Baud:38400
 */
static void UART_VT100_Menu_3()							// $$$$$$  INERTIAL SENSOR MENU  $$$$$$
{		MenuLevel=30;
	
		printf("\x1B[2J");			//VT100 CLR SCREEN
		printf("\x1B[H");			//VT100 CURSOR HOME
		printf("\x1B[01;05H GDV-UoM SMARTCANE MENU-3  INERTIAL SENSOR MENU");
		 
		printf("\x1B[05;05H MAGNETOMETER uT");
		printf("\x1B[06;05H Mag-X = ");
		printf("\x1B[07;05H Mag-Y = ");
		printf("\x1B[08;05H Mag-Z = ");
		printf("\x1B[09;05H Magxy = ");
		printf("\x1B[10;05H Magxxy= ");
	
	
		nrf_delay_ms(15);
		 
		printf("\x1B[05;30H GRAVITY g");
		printf("\x1B[06;30H Grav-X  = ");
		printf("\x1B[07;30H Grav-Y  = ");
		printf("\x1B[08;30H Grav-Z  = ");
		printf("\x1B[09;30H Grav-M  = ");

		nrf_delay_ms(15);

		printf("\x1B[05;55H GYROSCOPE deg/sec");
		printf("\x1B[06;55H Pitch  = ");
		printf("\x1B[07;55H Roll   = ");
		printf("\x1B[08;55H Yaw    = ");

		
		nrf_delay_ms(15);
		
		printf("\x1B[12;05H MADGWICK QUATERNION");
		printf("\x1B[13;05H Quat0 = ");
		printf("\x1B[14;05H Quat1 = ");
		printf("\x1B[15;05H Quat2 = ");
		printf("\x1B[16;05H Quat3 = ");
    	printf("\x1B[17;05H QuatM = "); 

		printf("\x1B[12;30H QUATERNION P-R-Y");
		printf("\x1B[13;30H Q pitch = ");
		printf("\x1B[14;30H Q roll  = ");
		printf("\x1B[15;30H Q yaw   = ");
	  printf("\x1B[24;05H  X... exit    ?...Help");
}	

/** @brief 		Function to load PRESSURE AND ALTITUDE MENU to VT100 Terminal Screen. 
 *  @details 	Loads VT100 ESC Sequenceone character to clear screen and load menu data.
 *  @note  		ASCII DEC 27 == x1B    VT100 ClrScreen == <ESC>[2J    Home == ESC[H    Cursor Location === <ESC>[{ROW};{COLUMN}H
 *  @ref 		VT100 Terminal Emulation Nominal Baud:38400
 */
static void UART_VT100_Menu_4()							// $$$$$$  ALTITUDE AND PRESSURE MENU  $$$$$$
{		MenuLevel=40;
	
		printf("\x1B[2J");			//VT100 CLR SCREEN
		printf("\x1B[H");			//VT100 CURSOR HOME
		printf("\x1B[01;05H GDV-UoM SMARTCANE MENU-4  PRESSURE, ALTITUDE & TEMPERATURE");
		 
		printf("\x1B[05;05H PRESSURE");
		printf("\x1B[06;05H Abs Pressure = ");
		printf("\x1B[07;05H ZeroRef      = ");
		printf("\x1B[08;05H Temperature  = ");

		nrf_delay_ms(15);
		 
		printf("\x1B[05;40H ALTITUDE");
		printf("\x1B[06;40H Altitude    = ");
		printf("\x1B[07;40H ZeroRef     = ");
		printf("\x1B[08;40H 1HrRelative = ");
		
		nrf_delay_ms(15);
    	    
	  printf("\x1B[24;05H  X... exit    ?...Help");
}	

/** @brief 		Function to load  SFLASH EEPROM SoC_FLASH MEMORY MANAGEMENT MENU to VT100 Terminal Screen. 
 *  @details 	Loads VT100 ESC Sequenceone character to clear screen and load menu data.
 *  @note  		ASCII DEC 27 == x1B    VT100 ClrScreen == <ESC>[2J    Home == ESC[H    Cursor Location === <ESC>[{ROW};{COLUMN}H
 *  @ref 		VT100 Terminal Emulation Nominal Baud:38400
 */
static void UART_VT100_Menu_5()							// $$$$$$  SFLASH EEPROM SoC_FLASH MEMORY MANAGEMENT MENU  $$$$$$
{		MenuLevel=50;
	
		printf("\x1B[2J");								// VT100 CLR SCREEN
		printf("\x1B[H");								// VT100 CURSOR HOME
		printf("\x1B[01;05H GDV-UoM SMARTCANE MENU-5 SFLASH EEPROM SoC_FLASH MEMORY MANAGEMENT MENU");
		 
		printf("\x1B[05;05H SFLASH");
		printf("\x1B[06;05H Device ID   = ");
		printf("\x1B[07;05H Percent Use = ");
		printf("\x1B[08;05H Spare       = ");

		nrf_delay_ms(5);
		 
		printf("\x1B[05;30H EEPROM");
		printf("\x1B[06;30H Device ID   = ");
		printf("\x1B[07;30H Percent Use = ");
		printf("\x1B[08;30H Spare       = ");

		nrf_delay_ms(5);

		printf("\x1B[05;55H ARM SoC");
		printf("\x1B[06;55H Device ID   = ");
		printf("\x1B[07;55H Percent Use = ");
		printf("\x1B[08;55H Spare       = ");

		nrf_delay_ms(5);		

		printf("\x1B[24;05H  X... exit    ?...Help");
}	


/** @brief 		Function to load POWER MANAGEMENT MENU to VT100 Terminal Screen. 
 *  @details 	Loads VT100 ESC Sequenceone character to clear screen and load menu data.
 *  @note  		ASCII DEC 27 == x1B    VT100 ClrScreen == <ESC>[2J    Home == ESC[H    Cursor Location === <ESC>[{ROW};{COLUMN}H
 *  @ref 		VT100 Terminal Emulation Nominal Baud:38400
 */
static void UART_VT100_Menu_6()		    				// $$$$$$  POWER MANAGEMENT MENU  $$$$$$
{		MenuLevel=60;
	
		printf("\x1B[2J");								// VT100 CLR SCREEN
		printf("\x1B[H");								// VT100 CURSOR HOME
		printf("\x1B[01;05H GDV-UoM SMARTCANE MENU-6 POWER MANAGEMENT MENU");
		 
		printf("\x1B[05;05H GAUGE MEASUREMENTS");
		printf("\x1B[06;05H Batt Voltage  = ");
		printf("\x1B[07;05H Batt Current  = ");
		printf("\x1B[08;05H Batt Status   = "); 
		printf("\x1B[09;05H Batt Capacity = ");
		printf("\x1B[10;05H Batt Temp     = ");

		nrf_delay_ms(15);
		 
		printf("\x1B[05;50H SoC ADC RAILS");
		printf("\x1B[06;50H Vbatt    = ");
		printf("\x1B[07;50H 3V3 Rail = ");
		printf("\x1B[08;50H Spare    = ");
		
		nrf_delay_ms(15);

		printf("\x1B[24;05H  X... exit    ?...Help");
}	

/** @brief 		Function to load CANE DIAGNOSTIC MENU to VT100 Terminal Screen. 
 *  @details 	Loads VT100 ESC Sequenceone character to clear screen and load menu data.
 *  @note  		ASCII DEC 27 == x1B    VT100 ClrScreen == <ESC>[2J    Home == ESC[H    Cursor Location === <ESC>[{ROW};{COLUMN}H
 *  @ref 		VT100 Terminal Emulation Nominal Baud:38400
 */
static void UART_VT100_Menu_7()		    				// $$$$$$  CANE DIAGNOSTIC MENU $$$$$$
{		MenuLevel=70;
	
		printf("\x1B[2J");								// VT100 CLR SCREEN
		printf("\x1B[H");								// VT100 CURSOR HOME
		printf("\x1B[01;05H GDV-UoM SMARTCANE MENU-7 CANE DIAGNOSTIC MENU");
		 
		printf("\x1B[05;05H Configurations");
		printf("\x1B[06;05H Way Points  = ");
		printf("\x1B[07;05H Home        = ");
		printf("\x1B[08;05H Spare       = ");

		nrf_delay_ms(5);
		 
		printf("\x1B[05;30H SPARE");
		printf("\x1B[06;30H Spare = ");
		printf("\x1B[07;30H Spare = ");
		printf("\x1B[08;30H Spare = ");

		nrf_delay_ms(5);

		printf("\x1B[05;55H SPARE");
		printf("\x1B[06;55H Spare = ");
		printf("\x1B[07;55H Spare = ");
		printf("\x1B[08;55H Spare = ");

		nrf_delay_ms(5);		

		printf("\x1B[24;05H  X... exit    ?...Help");
}	

/** @brief 		Function to load SYSTEM DIAGNOSTIC MENU to VT100 Terminal Screen. 
 *  @details 	Loads VT100 ESC Sequenceone character to clear screen and load menu data.
 *  @note  		ASCII DEC 27 == x1B    VT100 ClrScreen == <ESC>[2J    Home == ESC[H    Cursor Location === <ESC>[{ROW};{COLUMN}H
 *  @ref 		VT100 Terminal Emulation Nominal Baud:38400
 */
static void UART_VT100_Menu_8()		    				// $$$$$$  SYSTEM DIAGNOSTIC MENU $$$$$$
{		MenuLevel=80;
	
		printf("\x1B[2J");								//VT100 CLR SCREEN
		printf("\x1B[H");								//VT100 CURSOR HOME
		printf("\x1B[01;05H GDV-UoM SMARTCANE MENU-8 SYSTEM DIAGNOSTIC MENU");
		 
		printf("\x1B[05;05H KEY COMMANDS");
		printf("\x1B[06;05H A = calibrateMPU9150");
		printf("\x1B[07;05H B = ZERO Manual Cal's");
		printf("\x1B[08;05H C = Strobe Vibro 1sec");
		printf("\x1B[09;05H D = Start Timer");
		printf("\x1B[10;05H E = StrobeRedLED");
		printf("\x1B[11;05H F = Strobe PWM Tone");
		printf("\x1B[12;05H G = Spare");
		printf("\x1B[13;05H H = Spare");
	

		nrf_delay_ms(15);
		 
		printf("\x1B[05;32H ERROR COUNTERS");
		printf("\x1B[06;32H ErrorCount 01= ");
		printf("\x1B[07;32H ErrorCount 02= ");
		printf("\x1B[08;32H ErrorCount 03= ");
		printf("\x1B[09;32H ErrorCount 04= ");
		printf("\x1B[10;32H ErrorCount 05= ");
		printf("\x1B[11;32H ErrorCount 06= ");
		printf("\x1B[12;32H ErrorCount 07= ");


		nrf_delay_ms(15);

		printf("\x1B[05;58H SPARE");
		printf("\x1B[06;58H Spare = ");
		printf("\x1B[07;58H Spare = ");
		printf("\x1B[08;58H Spare = ");

		nrf_delay_ms(15);		

		printf("\x1B[24;05H  X... exit    ?...Help");
}	
/** @brief 		Function to load Screen Help MENU to VT100 Terminal Screen. 
 *  @details 	Loads VT100 ESC Sequenceone character to clear screen and load menu data.
 *  @note  		ASCII DEC 27 == x1B    VT100 ClrScreen == <ESC>[2J    Home == ESC[H    Cursor Location === <ESC>[{ROW};{COLUMN}H
 *  @ref 			VT100 Terminal Emulation Nominal Baud:38400
 */
static void UART_VT100_Help_Menu()						// $$$$$$  SCREEN HELP MENU  $$$$$$
{		MenuLevel=100;

		printf("\x1B[2J");								// VT100 CLR SCREEN
		printf("\x1B[H");								// VT100 CURSOR HOME
		printf("\x1B[01;05H GDV-UoM SMARTCANE MENU-? Help MENU");
		 
		printf("\x1B[05;05H HELP CODES");
		printf("\x1B[06;05H Way Points  = ");
		printf("\x1B[07;05H Home        = ");
		printf("\x1B[08;05H Spare       = ");

		nrf_delay_ms(5);
		 
		printf("\x1B[05;30H SPARE");
		printf("\x1B[06;30H Spare = ");
		printf("\x1B[07;30H Spare = ");
		printf("\x1B[08;30H Spare = ");

		nrf_delay_ms(5);

		printf("\x1B[05;55H SPARE");
		printf("\x1B[06;55H Spare = ");
		printf("\x1B[07;55H Spare = ");
		printf("\x1B[08;55H Spare = ");

		nrf_delay_ms(5);		

		printf("\x1B[24;05H  X... exit    ?...Help");
}	

/** @brief		Function to load MAIN MENU Refresh Data to VT100 Terminal Screen.  */
static void UART_VT100_Display_Data_Main_Menu()			// $$$$$$  MAIN MENU DATA REFRESH  $$$$$$
{
		printf("\x1B[01;58H %s %s", DEVICE_NAME, DS2401_ID);			//Device Name + Device ID

		printf("\x1B[02;58H READY");									//Self Test Status ToDo
		
	
		printf("\x1B[01;78H");							//Park VT100 cursor at row 01 column 75	//Park VT100 cursor at row 01 column 78
}	

/** @brief 		Function to load ALL SENSOR Refresh Data to VT100 Terminal Screen.  */
static void UART_VT100_Display_Data_All_Sensors()		// $$$$$$  ALL SENSORS MENU DATA REFRESH  $$$$$$
{
	int value;
	float Acc[3];
	double Calc, MagGravity, MagCompass;
	float data[3], temp;
	
	readAccelFloatMG(Acc);								//ACCELERATION

	printf("\x1B[05;14H%+2.2f ", Acc[0]);
	printf("\x1B[06;14H%+2.2f ", Acc[1]);
	printf("\x1B[07;14H%+2.2f ", Acc[2]);
	
	MagGravity = sqrt(Acc[0]*Acc[0] + Acc[1]*Acc[1] + Acc[2]*Acc[2]);
	printf("\x1B[08;14H%+2.2f ", MagGravity);
	
	nrf_delay_ms(15);		
	
	readGyroFloatDeg(Acc);								//GYRO
	printf("\x1B[05;40H%+4.2f ", Acc[0]);
	printf("\x1B[06;40H%+4.2f ", Acc[1]);
	printf("\x1B[07;40H%+4.2f ", Acc[2]);
	
	Calc = sqrt(Acc[0]*Acc[0] + Acc[1]*Acc[1] + Acc[2]*Acc[2]);
	printf("\x1B[08;40H%+4.2f ", Calc);
	
	nrf_delay_ms(15);		
	
//	mpu_get_compass_reg(data, &timestamp);
	readMagFloatUT(data);								//MAGNETIC
	printf("\x1B[05;68H%+4.2f ", (float)data[0]);
	printf("\x1B[06;68H%+4.2f ", (float)data[1]);
	printf("\x1B[07;68H%+4.2f ", (float)data[2]);
	
	MagCompass = sqrt(data[0]*data[0] + data[1]*data[1] + data[2]*data[2]);   //Total Magnetic Field Exposure ---> Earth + Other ????
	printf("\x1B[08;68H%+4.2f ", MagCompass);
	
//	printf("\x1B[10;19H%10d ",(int)timestamp);
	
	nrf_delay_ms(15);		

	printf("\x1B[12;68H%+4.2f ", (float)MPL3115A2_getPressure());
	
 	printf("\x1B[13;68H%+4.2f ", (float)MPL3115A2_getAltitude());

	nrf_delay_ms(15);		

//	TEMPERATURE ---> REAL TIME DEVICE UPDATES
	
	printf("\x1B[12;44H%+4.1f ", (float) readNRF_TEMP());					// Processor Temperature
	
	printf("\x1B[13;44H%+4.1f ", (float) readTempData()); 					// Inertial Temperature
	
	printf("\x1B[14;44H%+4.1f ", (float) MPL3115A2_getTemperature());		// Pressure Sensor Temp	

	if (!ltc294x_get_temperature(&value))
	temp = value/100;	
	printf("\x1B[15;44H%+4.1f ", temp);										// GasGauge Temperature
	 
	
}	


/** @brief 		Function to load GPS MENU Refresh Data to VT100 Terminal Screen.  
*/
static void UART_VT100_Display_Data_GPS()					// $$$$$$ GPS MENU DATA REFRESH  $$$$$$
{
	int value;
		
	if (!ltc294x_get_voltage(&value))
		printf("\x1B[06;19H%10d ",value);

	
	 if (!ltc294x_get_temperature(&value))
		printf("\x1B[10;19H%10d",value);
	 
}	

/** @brief 		Function to load INERTIAL MENU Refresh Data to VT100 Terminal Screen.  */
static void UART_VT100_Display_Data_Inertial()				// $$$$$$  INERTIAL MPU9250 MENU DATA REFRESH  $$$$$$
	{
	float Acc[3];
	double MagGravity, Magnitude;
		
	float data[4];
	
	readAccelFloatMG(Acc);									// ACCELERATION-GRAVITY (g)
	printf("\x1B[06;41H%+2.2f ", Acc[0]);
	printf("\x1B[07;41H%+2.2f ", Acc[1]);
	printf("\x1B[08;41H%+2.2f ", Acc[2]);
	
	MagGravity = sqrt(Acc[0]*Acc[0] + Acc[1]*Acc[1] + Acc[2]*Acc[2]);
	printf("\x1B[09;41H%+2.4f ", (float) MagGravity);		// MAG X-Y-Z
	
	nrf_delay_ms(15);

	readGyroFloatDeg(Acc);									// GYROSCOPE Degrees
	printf("\x1B[06;65H%+4.2f ", Acc[0]);
	printf("\x1B[07;65H%+4.2f ", Acc[1]);
	printf("\x1B[08;65H%+4.2f ", Acc[2]);
	
	nrf_delay_ms(15);
	
//	mpu_get_compass_reg(data, &timestamp);
	readMagFloatUT(data);									// MAGNETIC FIELD uTesla
	printf("\x1B[06;14H%+4.2f ", data[0]);
	printf("\x1B[07;14H%+4.2f ", data[1]);
	printf("\x1B[08;14H%+4.2f ", data[2]);

	nrf_delay_ms(15);
	
	Magnitude = sqrt(data[0]*data[0] + data[1]*data[1] );   //Total X-Y Magnetic Field Exposure ---> Earth + Other ????
	printf("\x1B[09;14H%+4.4f ", Magnitude);
	
	nrf_delay_ms(15);
	
	Magnitude = sqrt(data[0]*data[0] + data[1]*data[1] + data[2]*data[2]);   //Total Magnetic Field Exposure ---> Earth + Other ????
	printf("\x1B[10;14H%+4.4f ", Magnitude);
	
	nrf_delay_ms(15);

	readQuaternion(Quaternion);								// QUARTERNION   ---> NOT UPDATING FILTER
	printf("\x1B[13;14H%+4.2f ", Quaternion[0]);
	printf("\x1B[14;14H%+4.2f ", Quaternion[1]);
	printf("\x1B[15;14H%+4.2f ", Quaternion[2]);
	printf("\x1B[16;14H%+4.2f ", Quaternion[3]);

	nrf_delay_ms(15);
	
	Magnitude = sqrt(	Quaternion[0]*Quaternion[0] + 
						Quaternion[1]*Quaternion[1] + 
						Quaternion[2]*Quaternion[2] +
						Quaternion[3]*Quaternion[3]);		// Magnitude of Quaternion ????
	printf("\x1B[17;14H%+4.4f ", Magnitude);
	
/*	Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
	In this coordinate system, the positive z-axis is down toward Earth. 
	Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
	Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
	Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
	These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
	Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
	applied in the correct order which for this configuration is yaw, pitch, and then roll.
	For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.*/
  
	float yaw, pitch, roll, q[4];
	float PI = 3.14159265358979323846f;
	
	q[0] = Quaternion[0];
	q[1] = Quaternion[1];
	q[2] = Quaternion[2];
	q[3] = Quaternion[3];

	yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
	pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
	roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
	pitch *= 180.0f / PI;
	yaw   *= 180.0f / PI; 
	yaw   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
	roll  *= 180.0f / PI;

	printf("\x1B[13;41H%+4.2f ", pitch);
	printf("\x1B[14;41H%+4.2f ", roll);
	printf("\x1B[15;41H%+4.2f ", yaw);
	}	


/** @brief 		Function to load ALTITUDE MENU Refresh Data to VT100 Terminal Screen.  */
static void UART_VT100_Display_Data_Altitude()			// $$$$$$  ALTITUDE MENU DATA REFRESH  $$$$$$
{
		// Note MPL3115A returns float ---> cast as int 

		printf("\x1B[06;21H%+4.2f ", (float)MPL3115A2_getPressure());
		
		printf("\x1B[07;21H%+4.2f ", (float)MPL3115A2_getPressureSeaLevel()); 

		printf("\x1B[08;23H%+4.2f ", (float)MPL3115A2_getTemperature());

		printf("\x1B[06;55H%+4.2f ", (float)MPL3115A2_getAltitude());
}	


/** @brief 		Function to load MEMORY MENU Refresh Data to VT100 Terminal Screen.  */
static void UART_VT100_Display_Data_Memory()			// $$$$$$  Memory MENU DATA REFRESH  $$$$$$
{
	int value;
		
	if (!ltc294x_get_voltage(&value))
		printf("\x1B[06;19H%10d ",value);

	
	 if (!ltc294x_get_temperature(&value))
		printf("\x1B[10;19H%10d",value);
}	



/** @brief 		Function to load POWER MANAGEMENT MENU Refresh Data to VT100 Terminal Screen.  */
static void UART_VT100_Display_Data_Power()		    	// $$$$$$  POWER MANAGEMENT MENU DATA REFRESH  $$$$$$
{
	int value;
	float temp;
		
	if (!ltc294x_get_voltage(&value))				// WARNING GasGauge Intermitent Occassionally Returns 0000
	temp = (float) value;
	temp = temp / 1000000;
	printf("\x1B[6;22H%+4.3f ", temp);				// GasGauge Battery Voltage 

	if (value>4000000)								// WARNING If battery emoved the Charging Supply Jumps to .GT.4V1 and incorrectly shows as charged
		printf("\x1B[08;21H CHARGED ");				// Need to base battery of charge counter or on battery reaching correct capacity
	else if (value>3500000)
		printf("\x1B[08;21H HIGH    ");
	else if (value>2500000)
		printf("\x1B[08;21H MEDIUM  "); 
	else if (value>2000000)
		printf("\x1B[08;21H LOW     "); 
	else if (value<=2000000)
		printf("\x1B[08;21H VERY LOW"); 
	
	if (!ltc294x_get_current(&value))					
	temp = (float) value/1000;
	printf("\x1B[7;22H%+4.2f ", temp);				// GasGauge Battery Current  FAULT ---> Not Returning Valid dada ??????????
	
	printf("\x1B[10;22H%+4.2f ", temp);					
	if (!ltc294x_get_charge_counter(&value))
	temp = (float) value/10;
	printf("\x1B[09;22H%+4.2f ", temp);				// GasGauge Charge Counter
	
	if (!ltc294x_get_temperature(&value))
	temp = (float) value/100;	
	printf("\x1B[10;22H%+4.2f ", temp);				// GasGauge Temperature 
	
	nrf_delay_ms(15);	

}	


/** @brief 		Function to load CANE DIAGNOSTICS MENU Refresh Data to VT100 Terminal Screen.  */
static void UART_VT100_Display_Data_Cane()				// $$$$$$  CANE DIAGNOSTICS MENU DATA REFRESH  $$$$$$
{
	int value;
		
	if (!ltc294x_get_voltage(&value))
		printf("\x1B[06;19H%10d ",value);

	
	 if (!ltc294x_get_temperature(&value))
		printf("\x1B[10;19H%10d",value);
}	

/** @brief 		Function to load SYSTEM DIAGNOSTICS MENU Refresh Data to VT100 Terminal Screen.  */
static void UART_VT100_Display_Data_System()			// $$$$$$  SYSTEM DIAGNOSTICS MENU DATA REFRESH  $$$$$$
{
	
}	

/** @brief		Function to load ALL VT100 IMMEDIATE AND DATA REFRESH MENUEs to UART Terminal Screen.  
*/
static void Load_VT100_All_Menues()		    			// $$$$$$  LOAD ALL VT100 IMMEDIATE AND DATA REFRESH MENUEs  $$$$$$
	{
	printf("\x1B[01;78H");								//Park VT100 cursor at row 01 column 75
	
	nrf_delay_ms(5);
	
	uint8_t ch;
	if(app_uart_get(&ch) == NRF_SUCCESS) 
	{
		switch (ch) 
		{
		case '1':
		UART_VT100_Menu_1();							//	1... All Sensors
		break;
		
		case '2':
			UART_VT100_Menu_2();						//	2... GPS Global Position
		break;
		
		case '3':
			UART_VT100_Menu_3();						//	3... Inertial Sensors
		break;
		
		case '4':
			UART_VT100_Menu_4();						//	4... Altitude and Temperature							
		break;
		
		case '5':
			 UART_VT100_Menu_5();						//	5... Memory Functions						
		break;
		
		case '6':
			 UART_VT100_Menu_6();						//	6... Power Management						
		break;
		
		case '7':
			 UART_VT100_Menu_7();						//	7... Cane Diagnostics					
		break;
		
		case '8':
			 UART_VT100_Menu_8();						//	8... System Diagnostics						
		break;
		
		case 'x':
			UART_VT100_Main_Menu();						//	0... Main Menue
		break;	
		
		case 'X':
			UART_VT100_Main_Menu();
		break;
		
		case 't':
			printf ("BCD");								//	$$$$$ printf Diagnostic Test
		break;
		
		case '?':
			UART_VT100_Help_Menu();						//	Help Menu
		break;
		
		case 'A':
			if ( MenuLevel == 80) 						// Grab Static Manual Calibration Variables
			{
//			calibrateMPU9150(gyro_ManualCal, accel_ManualCal);
//			char cStatus[] = "Manual Acc-Gyro Offsets";
			}
		break;
		
		case 'B':
			if ( MenuLevel == 80) 						// Zero Manual Calibrations
			{
////			gyro_ManualCal[0]=0;   gyro_ManualCal[1]=0;  gyro_ManualCal[2]=0;
////			accel_ManualCal[0]=0; accel_ManualCal[1]=0; accel_ManualCal[2]=0;
//				char cStatus[] = "Zero Acc-Gyro Offsets";
			}
		break;
		
		case 'C':
			if ( MenuLevel == 80) 					
			{
			nrf_gpio_pin_set(MOTOR_PIN_NUMBER);			// Strobe Vibro Motor 500msec
			nrf_delay_ms(500);
			nrf_gpio_pin_clear(MOTOR_PIN_NUMBER);
//			char cStatus[] = "Strobe Vibro Motor";
			}
		break;
		
		case 'D':
			if ( MenuLevel == 80) 						// Start Background Timer
			{
//			char cStatus[] = "System Spare D";
			}
		break;
		
		case 'E':
			if ( MenuLevel == 80) 						// LED Red Test
			{
//			char cStatus[] = "System Spare E";
			}
		break;
		
		case 'F':
			if ( MenuLevel == 80) 						// PWM Tone Test
			{
//			char cStatus[] = "System Spare F";
			}
		break;
		
		case 'G':
			if ( MenuLevel == 80) 						// Spare G
			{
//			char cStatus[] = "System Spare G";
			}
		break;
		
		case 'H':
			if ( MenuLevel == 80) 						// Spare H
			{
//			char cStatus[] = "System Spare H";
			}
		break;
		} 
	}		
	
	else 
	
	{
		nrf_delay_ms(25);
			
		switch (MenuLevel)								//Load UART Reresh data based on current Menu Level
		{
		case 00:										//Main Menu
			UART_VT100_Display_Data_Main_Menu();
		break;		
		
		case 10:										//All Sensors
			UART_VT100_Display_Data_All_Sensors();
		break;		
			
		case 20:										//GPS Global Position
			UART_VT100_Display_Data_GPS();
		break;	

		case 30:										//Inertial Sensors
			UART_VT100_Display_Data_Inertial();
		break;

		case 40:										//Altitude Pressure and Temperature
			UART_VT100_Display_Data_Altitude();
		break;		
			
		case 50:										//Memory Functions
			UART_VT100_Display_Data_Memory();
		break;		
		
		case 60:										//Power Management
			UART_VT100_Display_Data_Power();
		break;	

		case 70:										//Cane Diagnostcs
			UART_VT100_Display_Data_Cane();
		break;		
		
		case 80:										// System Diagnostcs - Including Error Counters
			UART_VT100_Display_Data_System();
		break;				
		}	
		
		printf("\x1B[35;45H STATUS: %c", cStatus);		// Display System Status Message
		
		printf("\x1B[01;78H");							// Park VT100 cursor at row 01 column 75
		
		nrf_delay_ms(5);
	}
	}

/** @brief 		Function to read internal nrf51822 temperature. */
int32_t readNRF_TEMP() 
{
	int32_t ret = 0;
    while (true)
    {
        NRF_TEMP->TASKS_START = 1; /** Start the temperature measurement. */

        /* Busy wait while temperature measurement is not finished, you can skip waiting if you enable interrupt for DATARDY event and read the result in the interrupt. */
        /*lint -e{845} // A zero has been given as right argument to operator '|'" */
        while (NRF_TEMP->EVENTS_DATARDY == 0)
        {
            // $$$$$$$$$$$$$$ WARNING Need to monitor this trap or setup a countdown and exit ---> Do nothing.  $$$$$$$$$
        }
        NRF_TEMP->EVENTS_DATARDY = 0;

        /**@note Workaround for PAN_028 rev2.0A anomaly 29 - TEMP: Stop task clears the TEMP register. */
        ret = (nrf_temp_read() / 4);

        /**@note Workaround for PAN_028 rev2.0A anomaly 30 - TEMP: Temp module analog front end does not power down when DATARDY event occurs. */
        NRF_TEMP->TASKS_STOP = 1; /** Stop the temperature measurement. */

        return ret;
    }
}


	
/** @brief 		Function for MAIN application entry.
 *  @details 	
 *  @note  		
 *  @ref 
 *  @todo
 */
int main(void)
{
    LEDS_CONFIGURE(LEDS_MASK);
    LEDS_OFF(LEDS_MASK);
	
	GrabDeviceID(DS2401_ID,16);
	
    uint32_t err_code;
    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          APP_UART_FLOW_CONTROL_DISABLED,
          false,
          UART_BAUDRATE_BAUDRATE_Baud38400
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOW,
                         err_code);

    APP_ERROR_CHECK(err_code);
	
	uint8_t status = 0;	  
	while(!status)  status = I2C_Init();

	  
	ltc294x_init();							// Power Battery Gauge I2C
	MPL3115A2_init();						// Pressure, Altitude and Temperature I2C
	MPU9150_init();							// Gyro, Accelerometer, Magnetometer (Compass) and Temperature I2C
//	initMPU9150_Alt();						// Test Initalised Accel and Gyro ---> Magtemoter returns zeros
	  
	UART_VT100_Main_Menu();					//Initialise Default UART VT100 Menu

	  
/** @brief 		MASTER TIME SLOT ASSIGNER.  
 *  @details 	This routine operates as a never ending loop within main() and managers background tasks based on a rotating counter MainCount
 *  @note  		This while Loop Operates as a Background Task Manager
 *  @ref 
 *  @todo
 */
	int TLA_Count=0;						//Base Start Counter TLA 0..25

	while (true)
	{
//			app_sched_execute();
//			err_code = sd_app_evt_wait();	// BYPASS Menu not working possible TRAP pending unknowns !!!!!!!
//			APP_ERROR_CHECK(err_code);		// BYPASS
	
		Load_VT100_All_Menues();			//Loads VT100 Default Power Up Menu
		
		TLA_Count=TLA_Count+1;
		if (TLA_Count>=25) TLA_Count=0;
	
		switch (TLA_Count)
		{
		case 0:								//Loads VT100 Static and Rehresh UART Menues (NOTE: msec Time Delays operate within this function)
			Load_VT100_All_Menues();		
			break;
		
		case 1:								//Collect GPS Navigation Data and Load Bearing to Next Weighpoint
			
			break;
		
		case 2:								//Upon Active GPS Weighpoint Zone - Use Inertial/Compass Data to Signal Correct Bearing
			
			break;
		
		case 3:								//During Active Movement Collect User Parametrics. Walking Pace, Stick Movement, Gestures
			
			break;
		
		case 4:								//Collect and Filter Inertial Data
			
			break;
			
		case 5:								//Collect and Filter Pressure/Altitude Data 
			
			break;
			
		case 6:								//To Be Assigned 
			
			break;
		
		case 7:								//To Be Assigned 
			
			break;
		

		case 19:							//System Monitoring and Health Checking
			
			break;
		
		case 20:							//State Machine to Control Haptic Transducer Vibration States and On-Off Sequences
			
			break;
			
		case 21:							//State machine to Signal Audio Tone Sequences at Various Output Frequencies
			
			break;
		
		case 22:							//State machine to start Audio Tone Sequence at Various Output Frequencies
			
			break;
			
		case 23:							//Monitor for physical movement and/or physical activity 
			
			break;
			
		case 24:							//Monitor Battery State and System Temperatures
			
			break;
			
		case 25:							//End of background tasker ---> delay if necessary
			
			break;
		};	//end switch
		
	}; // while loop

/** end main **/
}





