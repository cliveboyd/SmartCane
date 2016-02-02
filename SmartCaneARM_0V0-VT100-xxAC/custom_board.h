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
#ifndef CUSTOM_BOARD_H
#define CUSTOM_BOARD_H

// Definitions for SmartCane Hardware Revision 1V0

#define LEDS_NUMBER    1							// One Red Diagnostic LED is defined on SmartCane 1V0

#define LED_START      30
#define LED_RGB_RED    30
#define LED_RGB_GREEN  30
#define LED_RGB_BLUE   30
#define LED_STOP       30

#define LED_RGB_RED_MASK    (1<<LED_RGB_RED)
#define LED_RGB_GREEN_MASK  (1<<LED_RGB_GREEN)
#define LED_RGB_BLUE_MASK   (1<<LED_RGB_BLUE)

#define LEDS_LIST { LED_RGB_RED, LED_RGB_GREEN, LED_RGB_BLUE}

// defining RGB led as 3 single LEDs
#define BSP_LED_0 LED_RGB_RED
#define BSP_LED_1 LED_RGB_GREEN
#define BSP_LED_2 LED_RGB_BLUE

#define BSP_LED_0_MASK    (1<<BSP_LED_0)
#define BSP_LED_1_MASK    (1<<BSP_LED_1)
#define BSP_LED_2_MASK    (1<<BSP_LED_2)

#define LEDS_MASK      (BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK)

//defines which LEDs are lit when signal is low
#define LEDS_INV_MASK  LEDS_MASK



// Two Push Buttons are defined here.

#define BUTTON_START   6
#define BUTTON_0       6						// Rear  Button
#define BUTTON_1       11						// Front Button
#define BUTTON_STOP    11
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BSP_BUTTON_0   BUTTON_0					// Rear  Handle Push Button
#define BSP_BUTTON_1   BUTTON_1					// Front Handle Push Button

#define BSP_BUTTON_0_MASK (1<<0)
#define BSP_BUTTON_1_MASK (1<<1)

#define BSP_BUTTON_2_MASK (1<<2)
#define BSP_BUTTON_3_MASK (1<<3)
#define BSP_BUTTON_4_MASK (1<<4)
#define BSP_BUTTON_5_MASK (1<<5)
#define BSP_BUTTON_6_MASK (1<<6)
#define BSP_BUTTON_7_MASK (1<<7)

#define BUTTONS_NUMBER 2
#define BUTTONS_LIST {BUTTON_0,BUTTON_1}
#define BUTTONS_MASK   0x000000FF


// UART Tx/Rx pins are interfaced via USB-UART Hub ---> CP2103
#define RX_PIN_NUMBER  26					// UART via USB CP2103
#define TX_PIN_NUMBER  27					// UART via USB CP2130

#define CTS_PIN_NUMBER 22					// Spare IO Pin ---> CTS-RTS not connected.
#define RTS_PIN_NUMBER 22					// Spare IO Pin

#define HWFC           true
#define NHWFC          false
	
#endif
