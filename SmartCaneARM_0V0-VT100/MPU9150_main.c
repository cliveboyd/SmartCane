/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
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

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "mpu9150.h"
#include "mpu9150_register_map.h"

/*Pins to connect MPU. */
#define MPU9150_TWI_SCL_PIN 1
#define MPU9150_TWI_SDA_PIN 2

/*UART buffer size. */
#define UART_TX_BUF_SIZE 256
#define UART_RX_BUF_SIZE 1
static const nrf_drv_twi_t m_twi_instance = NRF_DRV_TWI_INSTANCE(0);

/**
 * @brief UART events handler.
 */
static void uart_events_handler(app_uart_evt_t * p_event) {
    switch (p_event->evt_type) {
        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}


/**
 * @brief UART initialization.
 * Just the usual way. Nothing special here
 */
static void uart_config(void) {
    uint32_t  err_code;
	
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_events_handler,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);

    APP_ERROR_CHECK(err_code);
}


/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context) {   
   
    mpu9150_twi_event_handler(p_event);									// Pass TWI events down to the MPU driver.
}

/**
 * @brief TWI initialization.
 * Just the usual way. Nothing special here */
void twi_init(void) {
    ret_code_t err_code;
    
    const nrf_drv_twi_config_t twi_mpu_9150_config = {
       .scl                = MPU9150_TWI_SCL_PIN,
       .sda                = MPU9150_TWI_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH
    };
    
    err_code = nrf_drv_twi_init(&m_twi_instance, &twi_mpu_9150_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);
    
    nrf_drv_twi_enable(&m_twi_instance);
}

void mpu_init(void) {
    ret_code_t ret_code;
    // Initiate MPU9150 driver with TWI instance handler
    ret_code = mpu9150_init(&m_twi_instance);
    APP_ERROR_CHECK(ret_code);													// Check for errors in return value
    
    // Setup and configure the MPU9150 with intial values
    mpu9150_config_t p_mpu_config = MPU9150_DEFAULT_CONFIG();					// Load default values
    p_mpu_config.smplrt_div = 19;												// Change sampelrate. Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV). 19 gives a sample rate of 50Hz
    p_mpu_config.accel_config.afs_sel = AFS_2G;									// Set accelerometer full scale range to 2G
    ret_code = mpu9150_config(&p_mpu_config);									// Configure the MPU9150 with above values
    APP_ERROR_CHECK(ret_code);													// Check for errors in return value
    
    
    // This is a way to configure the interrupt pin behaviour
    // NOT YET IMPLEMENTED IN THIS EXAMPLE. YOU CAN DO IT YOURSELF BUT YOU WILL HAVE TO SET UP THE NRF51 INTERRUPT PINS AND IRQ ROUTINES YOURSELF
	
//    mpu9150_int_pin_cfg_t p_int_pin_cfg = MPU9150_DEFAULT_INT_PIN_CONFIG();	// Default configurations
//    p_int_pin_cfg.int_rd_clear = 1;											// When this bit is equal to 1, interrupt status bits are cleared on any read operation
//    ret_code = mpu9150_int_cfg_pin(&p_int_pin_cfg);							// Configure pin behaviour
//    APP_ERROR_CHECK(ret_code);												// Check for errors in return value
//    
//    // Enable the MPU interrupts
//    mpu9150_int_enable_t p_int_enable = MPU9150_DEFAULT_INT_ENABLE_CONFIG();
//    p_int_enable.data_rdy_en = 1;												// Trigger interrupt everytime new sensor values are available
//    ret_code = mpu9150_int_enable(&p_int_enable); 							// Configure interrupts
//    APP_ERROR_CHECK(ret_code);												// Check for errors in return value    
}

/**
 * @brief Function for main application entry.
 */
int main(void) {
    uart_config();
    printf("\033[2J\033[;H MPU9150 example. Compiled @ %s\r\n", __TIME__);
    
	twi_init();
    mpu_init();
    
    accel_values_t acc_values;
    uint32_t sample_number = 0;
    while(1) {
        
        mpu9150_read_accel(&acc_values);										// Read accelerometer sensor values
        
		// Clear terminal and print values
        printf("\033[2J\033[;HSample # %d\r\nX: %06d\r\nY: %06d\r\nZ: %06d", ++sample_number, acc_values.x, acc_values.y, acc_values.z);
        nrf_delay_ms(250);
    }
}

/** @} */

    Status API Training Shop Blog About Pricing 

    © 2016 GitHub, Inc. Terms Privacy Security Contact Help 

