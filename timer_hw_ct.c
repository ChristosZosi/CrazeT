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
 * timer_hw_ct.c
 *
 *  Created on: 23.08.2022
 *      Author: Christos Zosimidis
 */

/* CrazeT includes. */
#include "timer_hw_ct.h"
#include "debug_ct.h"

/* Nordic Semiconductors includes. */
#include <nrf.h>

/* Standard libraries includes. */
#include <stdint.h>

/******************************************************************************
 * Timer private functions
 ******************************************************************************/
static void interruptStubHandler() {
	/* TODO: print also interrupt ID */
	CRAZET_WARN("No handler registered!\r\n");
}

void TIMER0_IRQHandler() {

	if(CRAZET_TIMER->EVENTS_COMPARE[0] && (CRAZET_TIMER->INTENSET & TIMER_INTENSET_COMPARE0_Msk)) {

		if(TimerOnCompare0EventHandler) {
			TimerOnCompare0EventHandler();
		}

	}

	if(CRAZET_TIMER->EVENTS_COMPARE[1] && (CRAZET_TIMER->INTENSET & TIMER_INTENSET_COMPARE1_Msk)) {

		if(TimerOnCompare1EventHandler) {
			TimerOnCompare1EventHandler();
		}

	}

	if(CRAZET_TIMER->EVENTS_COMPARE[2] && (CRAZET_TIMER->INTENSET & TIMER_INTENSET_COMPARE2_Msk)) {

		if(TimerOnCompare2EventHandler) {
			TimerOnCompare2EventHandler();
		}

	}

	if(CRAZET_TIMER->EVENTS_COMPARE[3] && (CRAZET_TIMER->INTENSET & TIMER_INTENSET_COMPARE3_Msk)) {

		if(TimerOnCompare3EventHandler) {
			TimerOnCompare3EventHandler();
		}

	}

}
/******************************************************************************/

/******************************************************************************
 * Global Timer static initializations
 ******************************************************************************/

/* Global function pointer or TIMER event handlers. */
TimerIRQEventHandler TimerOnCompare0EventHandler = interruptStubHandler;
TimerIRQEventHandler TimerOnCompare1EventHandler = interruptStubHandler;
TimerIRQEventHandler TimerOnCompare2EventHandler = interruptStubHandler;
TimerIRQEventHandler TimerOnCompare3EventHandler = interruptStubHandler;
/******************************************************************************/

/******************************************************************************
 * RADIO configuration public functions
 ******************************************************************************/

void setCrazetTimerPrescaler(uint8_t prescale) {
	CRAZET_TIMER->PRESCALER = prescale;
}

void setCrazetTimerCCReg(CrazetTimerCC ccreg, uint32_t value) {
	CRAZET_TIMER->CC[ccreg] = value;
}

void shutdownCrazetTimer() {
	CRAZET_TIMER->TASKS_SHUTDOWN = 1UL;
}

void startCrazetTimer() {
	CRAZET_TIMER->TASKS_START = 1UL;
}
/******************************************************************************/
