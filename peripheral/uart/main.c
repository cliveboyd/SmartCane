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
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h"
#include "ltc2943.h"
#include "Communication.h"


#define MAX_TEST_DATA_BYTES     (55U)               /**< max number of test bytes per TX Burst to be used for tx and rx. */
#define UART_TX_BUF_SIZE 		512                 /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 		1                   /**< UART RX buffer size. */

uint8_t MenuLevel=0;								//Initalise Default Menu Level to 00 --> Main Menu

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

/** @brief 		Function to load a single top level VT100 Terminal Menu. 
 *  @details 	Transmitts VT100 ESC Sequenceone character to clear screen and load menu data.
 *  @note  		ASCII DEC 27 == x1B    VT100 ClrScreen == <ESC>[2J    Home == ESC[H    Cursor Location === <ESC>[{ROW};{COLUMN}H
 *  @ref 		TX_PIN_NUMBER must be connected to @ref RX_PIN_NUMBER)
 */
static void UART_VT100_Main_Menu()						// $$$$$$ TOP LEVEL MENU $$$$$$
{
	
//  Static menu loaded once and refreshed with menu level values elswhere.
		MenuLevel=00;

		printf("\x1B[2J");		//VT100 CLR SCREEN
		printf("\x1B[H");			//VT100 CURSOR HOME
		printf("\x1B[01;10H  GDV-UoM SMARTCANE MAIN MENU");
		printf("\x1B[01;50HDEVICE ID = ");
		printf("\x1B[02;50H   STATUS = ");

		printf("\x1B[04;10H  1... All Sensors");
		printf("\x1B[06;10H  2... GPS Global Position");
		printf("\x1B[08;10H  3... Inertial Sensors");
		printf("\x1B[10;10H  4... Altitude and Temperature");
		printf("\x1B[12;10H  5... Memory Functions");
		printf("\x1B[14;10H  6... Power Management");
		printf("\x1B[16;10H  7... Cane Diagnostics");

		printf("\x1B[18;10H  9... Shutdown");

		printf("\x1B[24;10H  ?... Help");
}

/** @brief 		Function to load top level Main Menu to VT100 Terminal Screen. 
 *  @details 	Loads VT100 ESC Sequenceone character to clear screen and load menu data.
 *  @note  		ASCII DEC 27 == x1B    VT100 ClrScreen == <ESC>[2J    Home == ESC[H    Cursor Location === <ESC>[{ROW};{COLUMN}H
 *  @ref 		VT100 Terminal Emulation Nominal Baud:38400
 */
static void UART_VT100_Menu_1()							// $$$$$$  ALL SENSORS  $$$$$$
{
//  Static menu loaded once and refreshed with MenuLevel values elswhere.
		MenuLevel=10;

		printf("\x1B[2J");		//VT100 CLR SCREEN
		printf("\x1B[H");			//VT100 CURSOR HOME
		printf("\x1B[01;05H GDV-UoM SMARTCANE MENU-1  ALL SENSORS and SYSTEMS");
		printf("\x1B[04;05H Gravity");
		printf("\x1B[05;05H ACC-X = ");
		printf("\x1B[06;05H ACC-Y = ");
		printf("\x1B[07;05H ACC-Z = ");

		nrf_delay_ms(5);

		printf("\x1B[04;30H Gyroscope");
		printf("\x1B[05;30H GYRO-X = ");
		printf("\x1B[06;30H GYRO-Y = ");
		printf("\x1B[07;30H GYRO-Z = ");

		nrf_delay_ms(5);

		printf("\x1B[04;55H Compass");
		printf("\x1B[05;55H Compass-X = ");
		printf("\x1B[06;55H Compass-Y = ");
		printf("\x1B[07;55H Compass-Z = ");

		nrf_delay_ms(5);
		 
		printf("\x1B[10;05H GPS");
		printf("\x1B[11;05H Latitude   = ");
		printf("\x1B[12;05H Longditude = ");
		printf("\x1B[13;05H Time       = ");

		nrf_delay_ms(5);
		 
		printf("\x1B[10;30H Temperature");
		printf("\x1B[11;30H Processor = ");
		printf("\x1B[12;30H Compass   = ");
		printf("\x1B[13;30H Pressure  = ");

		printf("\x1B[10;55H Height");
		printf("\x1B[11;55H Pressure = ");
		printf("\x1B[12;55H Altitude = ");

		nrf_delay_ms(5);
			
		printf("\x1B[24;05H  X... exit    ?...Help");
}	

/** @brief 		Function to load GPS GLOBAL POSITION MENU to VT100 Terminal Screen. 
 *  @details 	Loads VT100 ESC Sequenceone character to clear screen and load menu data.
 *  @note  		ASCII DEC 27 == x1B    VT100 ClrScreen == <ESC>[2J    Home == ESC[H    Cursor Location === <ESC>[{ROW};{COLUMN}H
 *  @ref 		VT100 Terminal Emulation Nominal Baud:38400
 */
static void UART_VT100_Menu_2()							// $$$$$$  GPS GLOBAL POSITION MENU  $$$$$$
{
//  Static menu loaded once and refreshed with MenuLevel values elswhere.
		MenuLevel=20;
	
		printf("\x1B[2J");			//VT100 CLR SCREEN
		printf("\x1B[H");			//VT100 CURSOR HOME
		printf("\x1B[01;05H GDV-UoM SMARTCANE MENU-2  GPS GLOBAL POSITION SENSOR");
		 
		printf("\x1B[04;05H GPS");
		printf("\x1B[05;05H Latitude   = ");
		printf("\x1B[06;05H Longditude = ");
		printf("\x1B[07;05H Height     = ");
		printf("\x1B[08;05H Time       = ");

		nrf_delay_ms(5);
		 
		printf("\x1B[05;30H SatNumber = ");
		printf("\x1B[06;30H Spare     = ");
		printf("\x1B[07;30H Spare     = ");
		printf("\x1B[08;30H Spare     = ");
		printf("\x1B[09;30H Spare     = ");
		
		nrf_delay_ms(5);
    	    
	  printf("\x1B[24;05H  X... exit    ?...Help");
}	

/** @brief 		Function to load INERTIAL SENSOR MENU to VT100 Terminal Screen. 
 *  @details 	Loads VT100 ESC Sequenceone character to clear screen and load menu data.
 *  @note  		ASCII DEC 27 == x1B    VT100 ClrScreen == <ESC>[2J    Home == ESC[H    Cursor Location === <ESC>[{ROW};{COLUMN}H
 *  @ref 		VT100 Terminal Emulation Nominal Baud:38400
 */
static void UART_VT100_Menu_3()							// $$$$$$  INERTIAL SENSOR MENU  $$$$$$
{
//  Static menu loaded once and refreshed with MenuLevel values elswhere.
		MenuLevel=30;
	
		printf("\x1B[2J");			//VT100 CLR SCREEN
		printf("\x1B[H");			//VT100 CURSOR HOME
		printf("\x1B[01;05H GDV-UoM SMARTCANE MENU-3  INERTIAL SENSOR MENU");
		 
		printf("\x1B[05;05H Magnetometer");
		printf("\x1B[06;05H Mag-X     = ");
		printf("\x1B[07;05H Mag-Y     = ");
		printf("\x1B[08;05H Mag-Z     = ");
		printf("\x1B[09;05H Mag-M     = ");

		nrf_delay_ms(5);
		 
		printf("\x1B[05;30H Gravity");
		printf("\x1B[06;30H Grav-X    = ");
		printf("\x1B[07;30H Grav-Y    = ");
		printf("\x1B[08;30H Grav-Z    = ");
		printf("\x1B[09;30H Grav-M    = ");

		nrf_delay_ms(5);

		printf("\x1B[05;50H Gyroscope ");
		printf("\x1B[06;50H Gyro-X    = ");
		printf("\x1B[07;50H Gyro-Y    = ");
		printf("\x1B[08;50H Gyro-Z    = ");
		printf("\x1B[09;50H Gyro-M    = ");
		
		nrf_delay_ms(5);
    	        
	  printf("\x1B[24;05H  X... exit    ?...Help");
}	

/** @brief 		Function to load PRESSURE AND ALTITUDE MENU to VT100 Terminal Screen. 
 *  @details 	Loads VT100 ESC Sequenceone character to clear screen and load menu data.
 *  @note  		ASCII DEC 27 == x1B    VT100 ClrScreen == <ESC>[2J    Home == ESC[H    Cursor Location === <ESC>[{ROW};{COLUMN}H
 *  @ref 		VT100 Terminal Emulation Nominal Baud:38400
 */
static void UART_VT100_Menu_4()							// $$$$$$  ALTITUDE AND PRESSURE MENU  $$$$$$
{
//  Static menu loaded once and refreshed with MenuLevel values elswhere.
		MenuLevel=40;
	
		printf("\x1B[2J");			//VT100 CLR SCREEN
		printf("\x1B[H");			//VT100 CURSOR HOME
		printf("\x1B[01;05H GDV-UoM SMARTCANE MENU-4  PRESSURE and ALTITUDE SENSOR");
		 
		printf("\x1B[05;05H Pressure");
		printf("\x1B[06;05H Abs Pressure = ");
		printf("\x1B[07;05H ZeroRef      = ");
		printf("\x1B[08;05H Temperature  = ");

		nrf_delay_ms(5);
		 
		printf("\x1B[05;30H Altitude");
		printf("\x1B[06;30H Altitude     = ");
		printf("\x1B[07;30H ZeroRef      = ");
		printf("\x1B[08;30H 1HrRelative  = ");

		
		nrf_delay_ms(5);
    	    
	  printf("\x1B[24;05H  X... exit    ?...Help");
}	

/** @brief 		Function to load  SFLASH EEPROM SoC_FLASH MEMORY MANAGEMENT MENU to VT100 Terminal Screen. 
 *  @details 	Loads VT100 ESC Sequenceone character to clear screen and load menu data.
 *  @note  		ASCII DEC 27 == x1B    VT100 ClrScreen == <ESC>[2J    Home == ESC[H    Cursor Location === <ESC>[{ROW};{COLUMN}H
 *  @ref 		VT100 Terminal Emulation Nominal Baud:38400
 */
static void UART_VT100_Menu_5()							// $$$$$$  SFLASH EEPROM SoC_FLASH MEMORY MANAGEMENT MENU  $$$$$$
{
		MenuLevel=50;
	
		printf("\x1B[2J");			//VT100 CLR SCREEN
		printf("\x1B[H");			//VT100 CURSOR HOME
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
{
		MenuLevel=60;
	
		printf("\x1B[2J");			//VT100 CLR SCREEN
		printf("\x1B[H");			//VT100 CURSOR HOME
		printf("\x1B[01;05H GDV-UoM SMARTCANE MENU-6 POWER MANAGEMENT MENU");
		 
		printf("\x1B[05;05H Gauge Measurements");
		printf("\x1B[06;05H Batt Voltage  = ");
		printf("\x1B[07;05H Batt Current  = ");
		printf("\x1B[08;05H Batt Status   = "); 
		printf("\x1B[09;05H Batt Capacity = ");
		printf("\x1B[10;05H Batt Temp     = ");

		nrf_delay_ms(5);
		 
		printf("\x1B[05;50H SoC ADC rail");
		printf("\x1B[06;50H Vbatt     = ");
		printf("\x1B[07;50H 3V3 Rail  = ");
		printf("\x1B[08;50H Spare     = ");
		
		nrf_delay_ms(5);
    
	  
		printf("\x1B[24;05H  X... exit    ?...Help");
}	

/** @brief 		Function to load CANE DIAGNOSTIC MENU to VT100 Terminal Screen. 
 *  @details 	Loads VT100 ESC Sequenceone character to clear screen and load menu data.
 *  @note  		ASCII DEC 27 == x1B    VT100 ClrScreen == <ESC>[2J    Home == ESC[H    Cursor Location === <ESC>[{ROW};{COLUMN}H
 *  @ref 		VT100 Terminal Emulation Nominal Baud:38400
 */
static void UART_VT100_Menu_7()		    				// $$$$$$  CANE DIAGNOSTIC MENU $$$$$$
{
		MenuLevel=70;
	
		printf("\x1B[2J");			//VT100 CLR SCREEN
		printf("\x1B[H");			//VT100 CURSOR HOME
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

/** @brief 		Function to load Screen Help MENU to VT100 Terminal Screen. 
 *  @details 	Loads VT100 ESC Sequenceone character to clear screen and load menu data.
 *  @note  		ASCII DEC 27 == x1B    VT100 ClrScreen == <ESC>[2J    Home == ESC[H    Cursor Location === <ESC>[{ROW};{COLUMN}H
 *  @ref 			VT100 Terminal Emulation Nominal Baud:38400
 */
static void UART_VT100_Help_Menu()						// $$$$$$  SCREEN HELP MENU  $$$$$$
{
		MenuLevel=100;

		printf("\x1B[2J");					//VT100 CLR SCREEN
		printf("\x1B[H");					//VT100 CURSOR HOME
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

/** @brief 		Function to load MAIN MENU Refresh Data to VT100 Terminal Screen.  
*/
static void UART_VT100_Display_Data_Main_Menu()			// $$$$$$  MAIN MENU DATA REFRESH  $$$$$$
{
	    printf("\x1B[1;60H 00000000");					//Device ID ToDo
		printf("\x1B[2;60H READY");						//Self Test Status ToDo
		
		printf("\x1B[01;75H");							//Park VT100 cursor at row 01 column 75
}	

/** @brief 		Function to load ALL SENSOR Refresh Data to VT100 Terminal Screen.  
*/
static void UART_VT100_Display_Data_All_Sensors()		// $$$$$$  ALL SENSORS MENU DATA REFRESH  $$$$$$
{
	int value;
		
	if (!ltc294x_get_voltage(&value))
		printf("\x1B[06;19H%10d ",value);

	
	
	 if (!ltc294x_get_temperature(&value))
		printf("\x1B[10;19H%10d",value);
}	


/** @brief 		Function to load GPS MENU Refresh Data to VT100 Terminal Screen.  
*/
static void UART_VT100_Display_Data_GPS()				// $$$$$$ GPS MENU DATA REFRESH  $$$$$$
{
	int value;
		
	if (!ltc294x_get_voltage(&value))
		printf("\x1B[06;19H%10d ",value);

	
	 if (!ltc294x_get_temperature(&value))
		printf("\x1B[10;19H%10d",value);
}	

/** @brief 		Function to load INERTIAL MENU Refresh Data to VT100 Terminal Screen.  
*/
static void UART_VT100_Display_Data_Inertial()			// $$$$$$  INERTIAL MENU DATA REFRESH  $$$$$$
{
	int value;
		
	if (!ltc294x_get_voltage(&value))
		printf("\x1B[06;19H%10d ",value);

	
	 if (!ltc294x_get_temperature(&value))
		printf("\x1B[10;19H%10d",value);
}	

/** @brief 		Function to load ALTITUDE MENU Refresh Data to VT100 Terminal Screen.  
*/
static void UART_VT100_Display_Data_Altitude()			// $$$$$$  ALTITUDE MENU DATA REFRESH  $$$$$$
{
	int value;
		
	if (!ltc294x_get_voltage(&value))
		printf("\x1B[06;19H%10d ",value);

	
	 if (!ltc294x_get_temperature(&value))
		printf("\x1B[10;19H%10d",value);
}	


/** @brief 		Function to load MEMORY MENU Refresh Data to VT100 Terminal Screen.  
*/
static void UART_VT100_Display_Data_Memory()			// $$$$$$  Memory MENU DATA REFRESH  $$$$$$
{
	int value;
		
	if (!ltc294x_get_voltage(&value))
		printf("\x1B[06;19H%10d ",value);

	
	 if (!ltc294x_get_temperature(&value))
		printf("\x1B[10;19H%10d",value);
}	



/** @brief 		Function to load POWER MANAGEMENT MENU Refresh Data to VT100 Terminal Screen.  
*/
static void UART_VT100_Display_Data_Power()		    	// $$$$$$  POWER MANAGEMENT MENU DATA REFRESH  $$$$$$
{
	int value;
		
	if (!ltc294x_get_voltage(&value))
		printf("\x1B[06;21H%10d ",value);

	if (value>4000000)									// WARNING If batter Removed the Charging Supply Jumps to .GT.4V1 and incorrectly shows as charged
		printf("\x1B[08;21H   CHARGED");				// NEEd to base battery of charge counter or on battery reaching correct capacity
	else if (value>3500000)
		printf("\x1B[08;21H      HIGH");
	else if (value>2500000)
		printf("\x1B[08;21H    MEDIUM"); 
	else if (value>2000000)
		printf("\x1B[08;21H       LOW"); 
	else if (value<=2000000)
		printf("\x1B[08;21H  VERY LOW"); 
	
	if (!ltc294x_get_current(&value))					//FAULT ---> Not Returning Valid dada ??????????
		printf("\x1B[07;21H%10d",value);
	
	if (!ltc294x_get_charge_counter(&value))
		printf("\x1B[09;21H%10d",value);
	
	 if (!ltc294x_get_temperature(&value))
		printf("\x1B[10;21H%10d",value);
}	


/** @brief 		Function to load CANE DIAGNOSTICS MENU Refresh Data to VT100 Terminal Screen.  
*/
static void UART_VT100_Display_Data_Cane()				// $$$$$$  CANE DIAGNOSTICS MENU DATA REFRESH  $$$$$$
{
	int value;
		
	if (!ltc294x_get_voltage(&value))
		printf("\x1B[06;19H%10d ",value);

	
	 if (!ltc294x_get_temperature(&value))
		printf("\x1B[10;19H%10d",value);
}	

/** @brief 		Function to load ALL VT100 IMMEDIATE AND DATA REFRESH MENUEs to UART Terminal Screen.  
*/
static void Load_VT100_All_Menues()		    			// $$$$$$  LOAD ALL VT100 IMMEDIATE AND DATA REFRESH MENUEs  $$$$$$
	{
	printf("\x1B[01;75H");								//Park VT100 cursor at row 01 column 75
	
	nrf_delay_ms(5);
	
	uint8_t ch;
	if(app_uart_get(&ch) == NRF_SUCCESS) 
	{
		switch (ch) 
		{
		case '1':
			UART_VT100_Menu_1();
		break;
		
		case '2':
			UART_VT100_Menu_2();
		break;
		
		case '3':
			UART_VT100_Menu_3();
		break;
		
		case '4':
			UART_VT100_Menu_4();							
		break;
		
		case '5':
			 UART_VT100_Menu_5();							
		break;
		
		case '6':
			 UART_VT100_Menu_6();							
		break;
		
		case '7':
			 UART_VT100_Menu_7();							
		break;
		
		case 'x':
			UART_VT100_Main_Menu();
		break;
		
		case 'X':
			UART_VT100_Main_Menu();
		break;
		
		case '?':
			UART_VT100_Help_Menu();
		break;
		} 
	}		
	
	else 
	
	{
		nrf_delay_ms(25);
					
		switch (MenuLevel)				//Load UART Reresh data based on current Menu Level
		{
		case 00:		//Main Menu
			UART_VT100_Display_Data_Main_Menu();
		break;		
		
		case 10:		//All Sensors
			UART_VT100_Display_Data_All_Sensors();
		break;		
			
		case 20:		//GPS Global Position
			UART_VT100_Display_Data_GPS();
		break;	

		case 30:		//Inertial Sensors
			UART_VT100_Display_Data_Inertial();
		break;

		case 40:		//Altitude Pressure and Temperature
			UART_VT100_Display_Data_Altitude();
		break;		
			
		case 50:		//Memory Functions
			UART_VT100_Display_Data_Memory();
		break;		
		
		case 60:		//Power Management
			UART_VT100_Display_Data_Power();
		break;	

		case 70:		//Cane Diagnostcs
			UART_VT100_Display_Data_Cane();
		break;		
			
		}	
		printf("\x1B[01;75H");			//Park VT100 cursor at row 01 column 75
		nrf_delay_ms(5);
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
	ltc294x_init();

	UART_VT100_Main_Menu();					//Initialise Default UART VT100 Menu

	  
/** @brief 		MASTER TIME SLOT ASSIGNER.  
 *  @details 	This routine operates as a never ending loop within main() and managers foreground tasks based on a rotating counter MainCount
 *  @note  		This While Loop Operates as a Background Task Manager
 *  @ref 
 *  @todo
 */
	int MainCount=0;
	while (true) 
	{
		MainCount=MainCount++;
		if (MainCount>=25) MainCount=0;

		switch (MainCount)
		{
		case 0:
			Load_VT100_All_Menues();		//Loads VT100 Static and Rehresh UART Menues (NOTE: msec Time Delays operate within this function)
			break;
		
		case 1:
			//Load_VT100_All_Menues();
			break;
		
		case 2:
			//Load_VT100_All_Menues();
			break;
		
		case 3:
			//Load_VT100_All_Menues();
			break;
			
		
		case 25:
			//Load_VT100_All_Menues();
			break;
		}
	}; // while loop


/** end main **/
}





