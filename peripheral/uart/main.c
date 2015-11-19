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

/** @file
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


#define MAX_TEST_DATA_BYTES     (55U)               /**< max number of test bytes per TX Burst to be used for tx and rx. */
#define UART_TX_BUF_SIZE 512                        /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 1                          /**< UART RX buffer size. */

 uint8_t MenuLevel;
 uint8_t MenuLevel=0;

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
 *  @ref 			TX_PIN_NUMBER must be connected to @ref RX_PIN_NUMBER)
 */
static void uart_VT100_Main_Menu()
{
	
//  Static menu loaded once and refreshed with menu level values elswhere.
	  uint8_t MenuLevel=00;
	
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
    printf("\x1B[14;10H  6... Shutdown");
	
	  printf("\x1B[24;05H  ?...Help");
}	

static void uart_VT100_Menu_1()
{
//  Static menu loaded once and refreshed with menu level values elswhere.
		uint8_t MenuLevel=10;
	
		printf("\x1B[2J");		//VT100 CLR SCREEN
  	printf("\x1B[H");			//VT100 CURSOR HOME
    printf("\x1B[01;05H GDV-UoM SMARTCANE MENU-1  ALL SENSORS");
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


/**
 * @brief Function for main application entry.
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
			
		uart_VT100_Main_Menu();

			
			
    uint8_t i=2;
		do {
        printf("\x1B[01;75H");		//park VT100 cursor at row 01 column 75
			  
			  nrf_delay_ms(5);
			
			  uint8_t cr;
        while(app_uart_get(&cr) != NRF_SUCCESS);

				switch (cr)
							{
							case '1':
								printf("1");
								uart_VT100_Menu_1();
							break;
							
							case '2':
								printf("2");
							  //uart_VT100_Menu_2();
							break;
							
							case '3':
								printf("3");
								//uart_VT100_Menu_3();
							break;
							
							case '4':
								printf("4");
								//uart_VT100_Menu_4();							
							break;
							
							case '5':
								 printf("5");
								//uart_VT100_Menu_5();							
							break;
							
							case 'q':
								printf("q");	
							break;
								
							case 'Q':
								printf("Q");								
							break;
		
							case 'x':
								printf("x");
								uart_VT100_Main_Menu();
							break;
							
							case 'X':
								printf("X");
								uart_VT100_Main_Menu();
							break;
							
							case '?':
								printf("?");
				//			uart_VT100_Help_Menu();
							break;
							} 
					
			} while (i=2); 


			
/** end main **/
}





