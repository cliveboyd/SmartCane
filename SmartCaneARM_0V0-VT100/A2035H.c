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
 
#include "A2035H.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "app_uart.h"
#include <stdio.h>

#include "nordic_common.h"  								// for UNUSED_PARAMETER


#define MAX_TEST_DATA_BYTES     (15U)						/**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 16									/**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 32									/**< UART RX buffer size. */

static ble_nus_t	m_nus;									/**< Structure to identify the Nordic UART Service. */


void uart_event_handle_withBle(app_uart_evt_t * p_event)	// is setup callback in initA2035H
{
	static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];		// 20 bytes
    static uint8_t index = 0;
    uint32_t err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') || (index >= (BLE_NUS_MAX_DATA_LEN)))
            {
                err_code = ble_nus_string_send(&m_nus, data_array, index);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
                
                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
			UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') || (index >= (BLE_NUS_MAX_DATA_LEN)))
            {
                //err_code = ble_nus_string_send(&m_nus, data_array, index);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
                
                index = 0;
            }
            break;
            }

}


typedef enum
{
	Pull_up,
	Pull_down,
	Pull_disable
} PullUpDown_t;


static __INLINE void pullupdown_gpio_cfg_output(uint32_t pin_number, PullUpDown_t pull )
{
	unsigned char pullset;
	switch(pull)
	{
		case Pull_up:
			pullset = (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);
			break;
		case Pull_down:
			pullset = (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos);
			break;
		case Pull_disable:
			pullset = (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos);
			break;
	}
    /*lint -e{845} // A zero has been given as right argument to operator '|'" */
    NRF_GPIO->PIN_CNF[pin_number] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                                            | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                                            | pullset
                                            | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
                                            | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
}

void initA2035H(void)
{
	nrf_delay_us(1000000); // delay 1s 
	pullupdown_gpio_cfg_output(A2035H_ON_OFF_PIN_NUMBER, Pull_down);
	pullupdown_gpio_cfg_output(A2035H_NRST_PIN_NUMBER, Pull_up);
	pullupdown_gpio_cfg_output(A2035H_NEN_PIN_NUMBER, Pull_up);
	pullupdown_gpio_cfg_output(A2035H_INT_PIN_NUMBER, Pull_down);
	
//	pullupdown_gpio_cfg_output(A2035H_TX_PIN_NUMBER, Pull_down);
	
//	nrf_gpio_cfg_output(A2035H_ON_OFF_PIN_NUMBER);
//	nrf_gpio_cfg_output(A2035H_NRST_PIN_NUMBER);
//	nrf_gpio_cfg_output(A2035H_RX_PIN_NUMBER);
//	nrf_gpio_cfg_output(A2035H_TX_PIN_NUMBER);

	nrf_gpio_pin_set(A2035H_NEN_PIN_NUMBER);			// Enable Aux 3V3 regulator
	nrf_gpio_pin_clear(A2035H_NRST_PIN_NUMBER);			// Toggle NRST
	nrf_delay_us(100000); 								// Delay 100ms 
	nrf_gpio_pin_set(A2035H_NRST_PIN_NUMBER);
	nrf_delay_us(100000); 								// Delay 100ms 
	
	nrf_gpio_pin_clear(A2035H_ON_OFF_PIN_NUMBER);		// Assert ON_OFF Pin Low
	nrf_delay_us(100000); 								// Delay 100ms 
	nrf_gpio_pin_set(A2035H_ON_OFF_PIN_NUMBER);			// Assert ON_OFF Pin High
	nrf_delay_us(200000); 								// delay 200ms 
	nrf_gpio_pin_clear(A2035H_ON_OFF_PIN_NUMBER);		// Assert ON_OFF Pin Low
	nrf_delay_us(100000); 								// Delay 100ms 
	
//	nrf_gpio_pin_clear(A2035H_RX_PIN_NUMBER);
//	nrf_gpio_pin_set(A2035H_TX_PIN_NUMBER);
	nrf_delay_us(1000);

//	Note as of Smartcane 1V0 the A2035H configured as SPI Slave ---> UART no longer used.
//	ToDo Sort out SPI and parse string


//	uint32_t err_code;
//    const app_uart_comm_params_t comm_params =
//      {
//          A2035H_TX_PIN_NUMBER,  // need to swap RX / TX at controller side
//          A2035H_RX_PIN_NUMBER,
//          0U,
//          0U,
//          APP_UART_FLOW_CONTROL_DISABLED,
//          false,  // no parity
//          UART_BAUDRATE_BAUDRATE_Baud4800
//      };

//    APP_UART_FIFO_INIT(&comm_params,
//                         UART_RX_BUF_SIZE,
//                         UART_TX_BUF_SIZE,
//                         uart_event_handle_withBle,
//                         APP_IRQ_PRIORITY_LOW,
//                         err_code);

//    APP_ERROR_CHECK(err_code);
	  
//	printf("\n\rStart: \n\r");
//	
//	uint8_t cr;
//    while (true)
//    {
//        
//        while(app_uart_get(&cr) != NRF_SUCCESS);
//        //while(app_uart_put(cr) != NRF_SUCCESS);

//		cr += 1;
//        if (false) // (cr == 'q' || cr == 'Q')
//        {
//            printf(" \n\rExit!\n\r");

//            while (true)
//            {
//                // Do nothing.
//            }
//        }
//    }
	
}
