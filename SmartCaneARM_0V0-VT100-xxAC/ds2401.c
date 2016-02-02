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

#include <string.h>

#include "nrf_delay.h"
#include "nrf_gpio.h"

#include "ds2401_config.h"
#include "ds2401.h"

unsigned char ds2401_id[8];

/* 1-wire is at defined in ds2401_config.h */
#define SET_PIN_INPUT()  do { NRF_GPIO->DIRCLR = (1UL << DS2401_SERIAL_ID_PIN_NUMBER);  } while(0)   /*!< Configures 1-wire pin as input  */
#define SET_PIN_OUTPUT() do { NRF_GPIO->DIRSET = (1UL << DS2401_SERIAL_ID_PIN_NUMBER);  } while(0)   /*!< Configures 1-wire pin as output  */

#define OUTP_0() do { NRF_GPIO->OUTCLR = (1UL << DS2401_SERIAL_ID_PIN_NUMBER);  } while(0)   /*!< Pulls 1-wire pin low  */
#define OUTP_1() do { NRF_GPIO->OUTSET = (1UL << DS2401_SERIAL_ID_PIN_NUMBER);  } while(0)   /*!< Pulls 1-wire pin high */

#define PIN_INIT() do{  \
                     SET_PIN_INPUT();    \
                     OUTP_0();           \
                   } while(0)


/* Drive the one wire interface low */
#define OW_DRIVE() do {                    \
                     SET_PIN_OUTPUT();     \
                     OUTP_0();             \
                   } while (0)

/* Release the one wire by turning on the internal pull-up. */
#define OW_RELEASE() do {                  \
                       SET_PIN_INPUT();    \
                       OUTP_1();           \
                     } while (0)

/* Read one bit. */
#define INP()  ((NRF_GPIO->IN >> DS2401_SERIAL_ID_PIN_NUMBER) & 0x1UL)                     /*!< Reads current state of 1-wire pin */

					 

/* 
 * Reference: 
 * Device driver for the Dallas Semiconductor DS2401 chip. Heavily
 * based on the application note 126 "1-Wire Communications Through
 * Software".
 *
 * http://www.maximintegrated.com/en/app-notes/index.mvp/id/126
*/
					 
/*
 * Delay times in us.
 */
#define tA 6          				/* min-5,	recommended-6,		max-15 */              
#define tB 64         				/* min-59,	recommended-64,		max-N/A */
#define tC 60         				/* min-60,	recommended-60,		max-120 */                     
#define tD 10         				/* min-5.3,	recommended-10,		max-N/A */
#define tE 9         				/* min-0.3,	recommended-9,		max-9.3 */
#define tF 55        				/* min-50,	recommended-55,		max-N/A */
#define tG 0         				/* min-0,	recommended-0,		max-0 	*/
#define tH 480        				/* min-480,	recommended-480,	max-640 */
#define tI 70         				/* min-60.3,recommended-70,		max-75.3 */
#define tJ 410        				/* min-410,	recommended-410,	max-N/A */
/*---------------------------------------------------------------------------*/
#define udelay(u) nrf_delay_us((u))
/*---------------------------------------------------------------------------*/
static uint8_t
reset(void) {
  uint8_t result;
  OW_DRIVE();
  udelay(500);     					/* 480 < tH < 640 */
  OW_RELEASE();    					/* Releases the bus */
  udelay(tI);
  result = INP();
  udelay(tJ);
  return result;
}
/*---------------------------------------------------------------------------*/
static void
write_byte(uint8_t byte) {
  uint8_t i = 7;
  do {
    if (byte & 0x01) {
      OW_DRIVE();
      udelay(tA);
      OW_RELEASE();    				/* Releases the bus */
      udelay(tB);
    } 
	else {
      OW_DRIVE();
      udelay(tC);
      OW_RELEASE();    				/* Releases the bus */
      udelay(tD);
    }
    if (i == 0)
      return;
    i--;
    byte >>= 1;
  } while (1);
}
/*---------------------------------------------------------------------------*/
static unsigned
read_byte(void)
{
  unsigned result = 0;
  int i = 7;
  do {
    OW_DRIVE();
    udelay(tA);
    OW_RELEASE();       			/* Releases the bus */
    udelay(tE);
    if (INP())
      result |= 0x80;   			/* LSbit first */
    udelay(tF);
    if (i == 0)
      return result;
    i--;
    result >>= 1;
  } while (1);
}
/*---------------------------------------------------------------------------*/
/* Polynomial ^8 + ^5 + ^4 + 1 */
static unsigned
crc8_add(unsigned acc, unsigned byte)
{
  int i;
  acc ^= byte;
  for (i = 0; i < 8; i++)
    if (acc & 1)
      acc = (acc >> 1) ^ 0x8c;
    else
      acc >>= 1;

  return acc;
}
/*---------------------------------------------------------------------------*/
int
ds2401_initAndRead()
{
  int i;
  unsigned family, crc, acc;

    NRF_GPIO->PIN_CNF[DS2401_SERIAL_ID_PIN_NUMBER] =     \
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) \
      | (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos) \
      | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)  \
      | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos) \
      | (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos);	
	
  PIN_INIT();
	
	// clear id first
   memset(ds2401_id, 0x0, sizeof(ds2401_id));

  if (reset() == 0) {
    write_byte(0x33);    			/* Read ROM command. */   
    family = read_byte();
    for (i = 7; i >= 2; i--) {  	// get 48 bits ID
      ds2401_id[i] = read_byte();
    }
    crc = read_byte();

    if(family != 0x01) { 			// family code always 0x01h
      goto fail;
    }
    acc = crc8_add(0x0, family);
    for (i = 7; i >= 2; i--) {
      acc = crc8_add(acc, ds2401_id[i]);
    }
    if (acc == crc) {
      return 1; 					/* Success! */
    }
  } 

fail:
  memset(ds2401_id, 0x0, sizeof(ds2401_id));
  return 0;  						/* Fail! */
}

