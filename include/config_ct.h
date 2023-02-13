/*
 * MIT License
 *
 * Copyright (c) 2022 Christos Zosimidis
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 *
 * debug_ct.h
 *
 *  Created on: 24.10.2022
 *      Author: Christos Zosimidis
 */

#ifndef CRAZET_DEBUG_CT_H_
#define CRAZET_DEBUG_CT_H_

/* Nordic Semiconductor include. */
#include <nrf.h>

/* Standard library includes. */
#include <stdio.h>

#ifndef CRAZET_TIMER_CONFIG
	#define CRAZET_TIMER_CONFIG 0
#endif

#if CRAZET_TIMER_CONFIG == 0
	#define CRAZET_TIMER NRF_TIMER0
#elif CRAZET_TIMER_CONFIG == 1
	#define CRAZET_TIMER NRF_TIMER1
#elif CRAZET_TIMER_CONFIG == 2
	#define CRAZET_TIMER NRF_TIMER2
#endif /* CRAZET_TIMER_CONFIG */

#ifndef CRAZET_DEBUG_PRINT
	#define CRAZET_DEBUG_PRINT 0
#endif /* CRAZET_DEBUG_PRINT */

#ifndef CRAZET_DEBUG_LED
	#define CRAZET_DEBUG_LED 0
#endif /* CRAZET_DEBUG_LED */

#if CRAZET_DEBUG_PRINT == 1
	void crazet_print(const char* msg);
	#define CRAZET_PRINT(...)           \
		{					            \
			char msg[64];               \
			sprintf(msg, __VA_ARGS__);  \
			crazet_print(msg);          \
		}
#else
	#define CRAZET_PRINT(...)
#endif /* CRAZET_DEBUG_PRINT == 1 */

#if CRAZET_DEBUG_LED == 1
	void crazetToggleRxLED();
	void crazetToggleTxLED();
#define CRAZET_RX_LED_TOGGLE() \
		crazetToggleRxLED()
#define CRAZET_TX_LED_TOGGLE() \
		crazetToggleTxLED()
#else
	#define CRAZET_RX_LED_TOGGLE()
	#define CRAZET_TX_LED_TOGGLE()
#endif /* CRAZET_DEBUG_LED */


#endif /* CRAZET_DEBUG_CT_H_ */
