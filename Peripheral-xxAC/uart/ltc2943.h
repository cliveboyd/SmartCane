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

#ifndef LTC2943_H
#define LTC2943_H


#include <stdint.h>  // for uint32_t etc.
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

int ltc294x_init(void);
int ltc294x_get_temperature(int *val);
int ltc294x_get_current( int *val);
int ltc294x_get_voltage(int *val);
int ltc294x_get_charge_counter(int *val);
int ltc294x_set_charge_now(int val);
int ltc294x_get_charge_now(int *val);
int ltc294x_read_charge_register(void);
int ltc294x_reset( int prescaler_exp);
void ltc294x_update(void);
#endif
