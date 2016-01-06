/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
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

#include "twi_master.h"
#include "twi_master_config.h"
#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"

/* Max cycles approximately to wait on RXDREADY and TXDREADY event,
 * This is optimized way instead of using timers, this is not power aware. */
 
#define MAX_TIMEOUT_LOOPS (20000UL)					/**< MAX while loops to wait for RXD/TXD event */



static bool twi_master_write(uint8_t * data, uint8_t data_length, bool issue_stop_condition) {
    uint32_t timeout = MAX_TIMEOUT_LOOPS;			/* max loops to wait for EVENTS_TXDSENT event*/
	int BypassCount;
	
    if (data_length == 0)  return false;			/* Return false for requesting data of size 0 */

    NRF_TWI1->TXD           = *data++;
    NRF_TWI1->TASKS_STARTTX = 1;

    /** @snippet [TWI HW master write] */
    while (true) {
		BypassCount=50000;       
		while (NRF_TWI1->EVENTS_TXDSENT == 0 && NRF_TWI1->EVENTS_ERROR == 0 && (--timeout)) {
//            BypassCount -= 1;
//			if (BypassCount <=0) {
//				return false;						// CSB Bypass upon unresolved error
//			}
			
			// Wait here for awhile and do nothing.
        }

        if (timeout == 0 || NRF_TWI1->EVENTS_ERROR != 0) {
            // Recover the peripheral as indicated by PAN 56: "TWI: TWI module lock-up." found at
            // Product Anomaly Notification document found at 
            // https://www.nordicsemi.com/eng/Products/Bluetooth-R-low-energy/nRF51822/#Downloads
			
            NRF_TWI1->EVENTS_ERROR = 0;
            NRF_TWI1->ENABLE       = TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos; 
            NRF_TWI1->POWER        = 0;
            nrf_delay_us(5);
            NRF_TWI1->POWER        = 1;
            NRF_TWI1->ENABLE       = TWI_ENABLE_ENABLE_Enabled << TWI_ENABLE_ENABLE_Pos;

            (void)twi_master_init();

            return false;
        }
        NRF_TWI1->EVENTS_TXDSENT = 0;
        if (--data_length == 0) {
            break;
        }

        NRF_TWI1->TXD = *data++;
    }
    /** @snippet [TWI HW master write] */

    if (issue_stop_condition) {
        NRF_TWI1->EVENTS_STOPPED = 0;
        NRF_TWI1->TASKS_STOP     = 1;
       
		BypassCount=10000;
		while(NRF_TWI1->EVENTS_STOPPED == 0) {				 /* Wait until stop sequence is sent */ 
			BypassCount -= 1;
			if (BypassCount <=0) {
				return false;								// CSB Bypass upon unresolved error
			}
			
			// Wait here for awhile and do nothing.
		}
    }
    return true;
}


/** @brief Function for read by twi_master.  */
static bool twi_master_read(uint8_t * data, uint8_t data_length, bool issue_stop_condition) {
    uint32_t timeout = MAX_TIMEOUT_LOOPS;				/* max loops to wait for RXDREADY event*/
	int BypassCount;
	
    if (data_length == 0) {
		return false;									/* Return false for requesting data of size 0 */
	}
    
	else if (data_length == 1) {
        NRF_PPI->CH[0].TEP = (uint32_t)&NRF_TWI1->TASKS_STOP;
    } else {
        NRF_PPI->CH[0].TEP = (uint32_t)&NRF_TWI1->TASKS_SUSPEND;
    }

    NRF_PPI->CHENSET          = PPI_CHENSET_CH0_Msk;
    NRF_TWI1->EVENTS_RXDREADY = 0;
    NRF_TWI1->TASKS_STARTRX   = 1;

    /** @snippet [TWI HW master read] */
	   
	while (true) {
        BypassCount=1000;
		while (NRF_TWI1->EVENTS_RXDREADY == 0 && NRF_TWI1->EVENTS_ERROR == 0 && (--timeout)) {
            BypassCount -= 1;
			if (BypassCount <=0) {
				return false;						// CSB Bypass upon unresolved error
			}
			
			// Wait here for awhile and do nothing.
			
        }
        NRF_TWI1->EVENTS_RXDREADY = 0;

        if (timeout == 0 || NRF_TWI1->EVENTS_ERROR != 0) {
            // Recover the peripheral as indicated by PAN 56: "TWI: TWI module lock-up." found at
            // Product Anomaly Notification document found at
            // https://www.nordicsemi.com/eng/Products/Bluetooth-R-low-energy/nRF51822/#Downloads
            NRF_TWI1->EVENTS_ERROR = 0;
            NRF_TWI1->ENABLE       = TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos;
            NRF_TWI1->POWER        = 0;
            nrf_delay_us(5);
            NRF_TWI1->POWER        = 1;
            NRF_TWI1->ENABLE       = TWI_ENABLE_ENABLE_Enabled << TWI_ENABLE_ENABLE_Pos;

            (void)twi_master_init();

            return false;
        }

        *data++ = NRF_TWI1->RXD;

        /* Configure PPI to stop TWI master before we get last BB event */
        if (--data_length == 1) {
            NRF_PPI->CH[0].TEP = (uint32_t)&NRF_TWI1->TASKS_STOP;
        }

        if (data_length == 0) {
            break;
        }

        // Recover the peripheral as indicated by PAN 56: "TWI: TWI module lock-up." found at
        // Product Anomaly Notification document found at
        // https://www.nordicsemi.com/eng/Products/Bluetooth-R-low-energy/nRF51822/#Downloads
		
        nrf_delay_us(20);
        NRF_TWI1->TASKS_RESUME = 1;
    }
    /** @snippet [TWI HW master read] */

    /* Wait until stop sequence is sent ---> CSB 1000 delay count Trap Added during debug testing*/
    BypassCount=1000;
	while (NRF_TWI1->EVENTS_STOPPED == 0) {
        BypassCount -= 1;
		if (BypassCount <=0) {
			return false;
		}
		
		// Wait here for awhile and o nothing during BypassCount countdown
	}
    NRF_TWI1->EVENTS_STOPPED = 0;

    NRF_PPI->CHENCLR = PPI_CHENCLR_CH0_Msk;
    return true;
}


/**
 * @brief Function for detecting stuck slaves (SDA = 0 and SCL = 1) and tries to clear the bus.
 *
 * @return
 * @retval false Bus is stuck.
 * @retval true Bus is clear.
 */
static bool twi_master_clear_bus(void) {
    uint32_t twi_state;
    bool     bus_clear;
    uint32_t clk_pin_config;
    uint32_t data_pin_config;

    // Save and disable TWI hardware so software can take control over the pins.
    twi_state        = NRF_TWI1->ENABLE;
    NRF_TWI1->ENABLE = TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos;

    clk_pin_config = \
        NRF_GPIO->PIN_CNF[TWI_MASTER_CONFIG_CLOCK_PIN_NUMBER];
    NRF_GPIO->PIN_CNF[TWI_MASTER_CONFIG_CLOCK_PIN_NUMBER] =      \
        (GPIO_PIN_CNF_SENSE_Disabled  << GPIO_PIN_CNF_SENSE_Pos) \
      | (GPIO_PIN_CNF_DRIVE_S0D1    << GPIO_PIN_CNF_DRIVE_Pos)   \
      | (GPIO_PIN_CNF_PULL_Pullup   << GPIO_PIN_CNF_PULL_Pos)    \
      | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)   \
      | (GPIO_PIN_CNF_DIR_Output    << GPIO_PIN_CNF_DIR_Pos);

    data_pin_config = \
        NRF_GPIO->PIN_CNF[TWI_MASTER_CONFIG_DATA_PIN_NUMBER];
    NRF_GPIO->PIN_CNF[TWI_MASTER_CONFIG_DATA_PIN_NUMBER] =       \
        (GPIO_PIN_CNF_SENSE_Disabled  << GPIO_PIN_CNF_SENSE_Pos) \
      | (GPIO_PIN_CNF_DRIVE_S0D1    << GPIO_PIN_CNF_DRIVE_Pos)   \
      | (GPIO_PIN_CNF_PULL_Pullup   << GPIO_PIN_CNF_PULL_Pos)    \
      | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)   \
      | (GPIO_PIN_CNF_DIR_Output    << GPIO_PIN_CNF_DIR_Pos);

    TWI_SDA_HIGH();
    TWI_SCL_HIGH();
    TWI_DELAY();

    if ((TWI_SDA_READ() == 1) && (TWI_SCL_READ() == 1)) {
        bus_clear = true;
    } else {
        uint_fast8_t i;
        bus_clear = false;

        // Clock max 18 pulses worst case scenario(9 for master to send the rest of command and 9
        // for slave to respond) to SCL line and wait for SDA come high.
        for (i=18; i--;) {
            TWI_SCL_LOW();
            TWI_DELAY();
            TWI_SCL_HIGH();
            TWI_DELAY();

            if (TWI_SDA_READ() == 1) {
                bus_clear = true;
                break;
            }
        }
    }

    NRF_GPIO->PIN_CNF[TWI_MASTER_CONFIG_CLOCK_PIN_NUMBER] = clk_pin_config;
    NRF_GPIO->PIN_CNF[TWI_MASTER_CONFIG_DATA_PIN_NUMBER]  = data_pin_config;

    NRF_TWI1->ENABLE = twi_state;

    return bus_clear;
}


/** @brief Function for initializing the twi_master.
 */
bool twi_master_init(void) {
    /* To secure correct signal levels on the pins used by the TWI
       master when the system is in OFF mode, and when the TWI master is
       disabled, these pins must be configured in the GPIO peripheral.
    */
    NRF_GPIO->PIN_CNF[TWI_MASTER_CONFIG_CLOCK_PIN_NUMBER] =     \
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) \
      | (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos) \
      | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)  \
      | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos) \
      | (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos);

    NRF_GPIO->PIN_CNF[TWI_MASTER_CONFIG_DATA_PIN_NUMBER] =      \
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) \
      | (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos) \
      | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)  \
      | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos) \
      | (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos);

    NRF_TWI1->EVENTS_RXDREADY = 0;
    NRF_TWI1->EVENTS_TXDSENT  = 0;
    NRF_TWI1->PSELSCL         = TWI_MASTER_CONFIG_CLOCK_PIN_NUMBER;
    NRF_TWI1->PSELSDA         = TWI_MASTER_CONFIG_DATA_PIN_NUMBER;
    NRF_TWI1->FREQUENCY       = TWI_FREQUENCY_FREQUENCY_K100 << TWI_FREQUENCY_FREQUENCY_Pos;
    NRF_PPI->CH[0].EEP        = (uint32_t)&NRF_TWI1->EVENTS_BB;
    NRF_PPI->CH[0].TEP        = (uint32_t)&NRF_TWI1->TASKS_SUSPEND;
    NRF_PPI->CHENCLR          = PPI_CHENCLR_CH0_Msk;
    NRF_TWI1->ENABLE          = TWI_ENABLE_ENABLE_Enabled << TWI_ENABLE_ENABLE_Pos;

    return twi_master_clear_bus();
}


/** @brief  Function for transfer by twi_master.
 */
bool twi_master_transfer(uint8_t   address,
                         uint8_t * data,
                         uint8_t   data_length,
                         bool      issue_stop_condition) {
    bool transfer_succeeded = false;
    if (data_length > 0 && twi_master_clear_bus()) {
        NRF_TWI1->ADDRESS = (address >> 1);

        if ((address & TWI_READ_BIT)) {
            transfer_succeeded = twi_master_read(data, data_length, issue_stop_condition);
        } else {
            transfer_succeeded = twi_master_write(data, data_length, issue_stop_condition);
        }
    }
    return transfer_succeeded;
}

/*lint --flb "Leave library region" */
