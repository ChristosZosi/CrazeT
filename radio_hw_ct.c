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
 * crazet_hw_ct.c
 *
 *  Created on: 23.08.2022
 *      Author: Christos Zosimidis
 */

/* CrazeT includes. */
#include "radio_hw_ct.h"
#include "debug_ct.h"

/* Nordic Semiconductors includes. */
#include <nrf.h>

/* Standard libraries includes. */
#include <stdint.h>

/******************************************************************************
 * Radio private functions
 ******************************************************************************/

static void interruptStubHandler() {
	/* TODO: print also interrupt ID */
	CRAZET_WARN("No handler registered!\r\n");
}

void RADIO_IRQHandler() {
	/* TODO: Maybe turn LEDS on/off for debugging features. */
	if(CRAZET_RADIO->EVENTS_DISABLED && (CRAZET_RADIO->INTENSET & RADIO_INTENSET_DISABLED_Msk)) {

		switch(crazetRadioState) {

		case CRAZET_RADIO_RX_STATE:

			if(CRAZET_RADIO->CRCSTATUS !=  0UL) {

				if(RadioOnDisabledRXEventHandler) {
					RadioOnDisabledRXEventHandler();
				}

			} else {
				CRAZET_ERROR("CRAZET_RADIO_RX_STATE: CRC failed.");
			}
			break;

		case CRAZET_RADIO_TX_STATE:

			if(RadioOnDisabledTXEventHandler) {
				RadioOnDisabledTXEventHandler();
			}
			break;

		default:
			CRAZET_ERROR("Unknown CRAZET RADIO state\r\n");
		}
	}

	if(CRAZET_RADIO->EVENTS_RSSIEND && (CRAZET_RADIO->INTENSET & RADIO_INTENSET_RSSIEND_Msk)) {

		if(RadioOnRSSIEndEventHandler) {
			RadioOnRSSIEndEventHandler();
		}
	}
}
/******************************************************************************/

/******************************************************************************
 * Global RADIO static initializations
 ******************************************************************************/

CrazetRadioState crazetRadioState = CRAZET_RADIO_RX_STATE;

/* Global function pointers for RADIO event handlers. */
RadioIRQEventHandler RadioOnDisabledRXEventHandler = interruptStubHandler;
RadioIRQEventHandler RadioOnDisabledTXEventHandler = interruptStubHandler;
RadioIRQEventHandler RadioOnRSSIEndEventHandler    = interruptStubHandler;
/******************************************************************************/

/******************************************************************************
 * RADIO configuration public functions
 ******************************************************************************/

void setCrazetRadioBitRate(CrazetRadioBitrate bitrate) {
	CRAZET_RADIO->MODE = (bitrate << RADIO_MODE_MODE_Pos);
}

void setCrazetRadioTXPower(CrazetRadioTXPower txpower) {
	CRAZET_RADIO->TXPOWER = (txpower << RADIO_TXPOWER_TXPOWER_Pos);
}

void setCrazetRadioChannel(CrazetRadioChannel channel) {
	CRAZET_RADIO->FREQUENCY = channel;
}

void setCrazetRadioCRC(CrazetRadioCrc crc) {
	CRAZET_RADIO->CRCCNF = (crc << RADIO_CRCCNF_LEN_Pos);
	switch(crc) {
	case RADIO_CRCCNF_LEN_One:
		CRAZET_RADIO->CRCINIT = 0xFFUL;	// Initial value
		CRAZET_RADIO->CRCPOLY = 0x107UL;	// CRC poly: x^8+x^2^x^1+1
		break;
	case RADIO_CRCCNF_LEN_Two:
		CRAZET_RADIO->CRCINIT = 0xFFFFUL;	// Initial value
		CRAZET_RADIO->CRCPOLY = 0x11021UL;	// CRC poly: x^16+x^12^x^5+1
		break;
	case RADIO_CRCCNF_LEN_Disabled:
	default:
		break;
	}
}

void configureCrazetRadioAirPacket(uint8_t onAirAddrLen, uint8_t onAirPayloadLen) {

	/* On-air packet configuration */
	/* | PREAMBLE  | BASE      | PREFIX   | S0      | LENGTH   | S1      | PAYLOAD     | CRC     | */
	/* |-----------|-----------|----------|---------|----------|---------|-------------|---------| */
	/* | 1 bytes   | 2-4 bytes | 2 bytes  | 0 bytes | 1 bytes  | 0 bytes | 0-253 bytes | 2 bytes | */

	CRAZET_RADIO->PCNF0 = (0UL << RADIO_PCNF0_S0LEN_Pos) | /* S1 field 0 bytes */
						  (8UL << RADIO_PCNF0_LFLEN_Pos) | /* LENGTH field 1 bytes */
						  (0UL << RADIO_PCNF0_S1LEN_Pos);  /* S0 field 0 bytes */

	CRAZET_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos) | /* Data whitening of on air packet disabled */
						  (RADIO_PCNF1_ENDIAN_Big 		<< RADIO_PCNF1_ENDIAN_Pos)  | /* On air endiannes of packet, little endian */
						  (onAirAddrLen 				<< RADIO_PCNF1_BALEN_Pos)   | /* length of base address of an on air packet */
						  (0UL 							<< RADIO_PCNF1_STATLEN_Pos) | /* static length of an on air packet payload */
						  (onAirPayloadLen 				<< RADIO_PCNF1_MAXLEN_Pos);   /* max. length of an on air packet payload */
}

void setCrazetRadioBase0Addr(uint32_t base0) {
	CRAZET_RADIO->BASE0 = base0;
}

void setCrazetRadioBase1Addr(uint32_t base1) {
	CRAZET_RADIO->BASE1 = base1;
}

void setCrazetRadioPrefix0Addr(uint32_t prefix0) {
	CRAZET_RADIO->PREFIX0 = prefix0;
}

void setCrazetRadioPrefix1Addr(uint32_t prefix1) {
	CRAZET_RADIO->PREFIX1 = prefix1;
}
/******************************************************************************/
