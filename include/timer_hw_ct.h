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
 * timer_hw_ct.h
 *
 *  Created on: 23.08.2022
 *      Author: Christos Zosimidis
 */

#ifndef CRAZET_TIMER_HW_CT_H_
#define CRAZET_TIMER_HW_CT_H_

/* Nordic Semiconductors includes. */
#include <nrf.h>

/* Standard libraries includes. */
#include <stdint.h>

/******************************************************************************
 * TIMER configuration types
 ******************************************************************************/
#define CRAZET_TIMER NRF_TIMER0

#define CRAZET_TIMER_DEFAULT_PRESCALER (4U)

typedef enum {
	CRAZET_TIMER_CC0_REG = 0,
	CRAZET_TIMER_CC1_REG,
	CRAZET_TIMER_CC2_REG,
	CRAZET_TIMER_CC3_REG
} CrazetTimerCC;

typedef void(*TimerIRQEventHandler)(void);
/******************************************************************************/

/******************************************************************************
 * Global TIMER variables
 ******************************************************************************/

/* Global function pointer for TIMER event handlers. */
extern TimerIRQEventHandler TimerOnCompare0EventHandler;
extern TimerIRQEventHandler TimerOnCompare1EventHandler;
extern TimerIRQEventHandler TimerOnCompare2EventHandler;
extern TimerIRQEventHandler TimerOnCompare3EventHandler;

/******************************************************************************
 * TIMER configuration functions
 ******************************************************************************/
#define CRAZET_TIMER_ENABLE_IRQ()  NVIC_EnableIRQ(TIMER0_IRQn)
#define CRAZET_TIMER_DISABLE_IRQ() NVIC_DisableIRQ(TIMER0_IRQn)

void setCrazetTimerPrescaler(uint8_t prescale);
void setCrazetTimerCCReg(CrazetTimerCC ccreg, uint32_t value);

void shutdownCrazetTimer();
void startCrazetTimer();

/******************************************************************************/

#endif /* CRAZET_TIMER_HW_CT_H_ */
