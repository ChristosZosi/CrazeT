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
 * crazet.c
 *
 *  Created on: 24.08.2022
 *      Author: Christos Zosimidis
 */

/* CrazeT includes. */
#include <config_ct.h>
#include "crazet.h"
#include "radio_hw_ct.h"
#include <nrf.h>

/* Standard libraries includes. */
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

/******************************************************************************
 * CRAZET constants
 ******************************************************************************/

#define LOGICAL_SELF_NODE_ADDRR  CRAZET_RADIO_LOGICAL_ADDRR_0
#define LOGICAL_BROADCAST_ADDRR  CRAZET_RADIO_LOGICAL_ADDRR_1
#define LOGICAL_JOIN_ADDRR       CRAZET_RADIO_LOGICAL_ADDRR_2
#define LOGICAL_TX_ADDRR         CRAZET_RADIO_LOGICAL_ADDRR_7

#define DEFAULT_BRAODCAST_ADDRR (0xFF)
#define DEFAULT_JOIN_ADDR       (0xFE)
#define DEFAULT_TX_ADDRR        DEFAULT_JOIN_ADDR

#define CRAZET_RADIO_DEFAULT_SHORTS (RADIO_SHORTS_READY_START_Enabled       << RADIO_SHORTS_READY_START_Pos       | \
									 RADIO_SHORTS_ADDRESS_RSSISTART_Enabled << RADIO_SHORTS_ADDRESS_RSSISTART_Pos | \
									 RADIO_SHORTS_END_DISABLE_Enabled       << RADIO_SHORTS_END_DISABLE_Pos       | \
									 RADIO_SHORTS_DISABLED_RSSISTOP_Enabled << RADIO_SHORTS_DISABLED_RSSISTOP_Pos)

#define CRAZET_TIMER_DEFAULT_SHORTS (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos | \
									 TIMER_SHORTS_COMPARE0_STOP_Enabled  << TIMER_SHORTS_COMPARE0_STOP_Pos)

#define CRAZET_TIMER_CC0_REG 0

/* CrazeT RX and TX FIFO queue sizes */
#define CRAZET_RX_FIFO_QUEUE_SIZE 32
#define CRAZET_TX_FIFO_QUEUE_SIZE 32
/******************************************************************************/

/******************************************************************************
 * CRAZET PPI system
 ******************************************************************************/

#define PPI NRF_PPI

#define PPI_RADIO_DISABLE_ON_CC0           (0)
#define PPI_TIMER_SHUTDOWN_ON_RADIO_ADDR   (1)

static void enableCrazetPPISystem() {

	PPI->CH[PPI_RADIO_DISABLE_ON_CC0].EEP  = (uint32_t)&CRAZET_TIMER->EVENTS_COMPARE[CRAZET_TIMER_CC0_REG];
	PPI->CH[PPI_RADIO_DISABLE_ON_CC0].TEP  = (uint32_t)&CRAZET_RADIO->TASKS_DISABLE;

	PPI->CH[PPI_TIMER_SHUTDOWN_ON_RADIO_ADDR].EEP = (uint32_t)&CRAZET_RADIO->EVENTS_END;
	PPI->CH[PPI_TIMER_SHUTDOWN_ON_RADIO_ADDR].TEP = (uint32_t)&CRAZET_TIMER->TASKS_STOP;
}

/******************************************************************************/

/******************************************************************************
 * CRAZET module static variable
 ******************************************************************************/

static CrazetConfig crazetConfig;

/* CrazeT RX FIFO-Queue */
static CrazetPacket rxQueue[CRAZET_RX_FIFO_QUEUE_SIZE];
static volatile uint8_t rxQueueHead = 0;
static volatile uint8_t rxQueueTail = 0;

/* CrazeT TX FIFO-Queue */
static CrazetPacket txQueue[CRAZET_TX_FIFO_QUEUE_SIZE];
static volatile uint8_t txQueueHead = 0;
static volatile uint8_t txQueueTail = 0;
/******************************************************************************/

/******************************************************************************
 * CRAZET private inline functions
 ******************************************************************************/

static inline void updateSelfNodeAddress(uint8_t addrr) {
	CRAZET_RADIO->PREFIX0 =
			(CRAZET_RADIO->PREFIX0 & 0xFFFFFF00) |
			(addrr << 0);
}

static inline void updateBroadcastAddress(uint8_t addrr) {
	CRAZET_RADIO->PREFIX0 =
			(CRAZET_RADIO->PREFIX0 & 0xFFFF00FF) |
			(addrr << 8);
}

static inline void updateJoinAddress(uint8_t addrr) {
	CRAZET_RADIO->PREFIX0 =
			(CRAZET_RADIO->PREFIX0 & 0xFF00FFFF) |
			(addrr << 16);
}

static inline void updateTXAddress(uint8_t addrr) {
	CRAZET_RADIO->PREFIX1 =
			(CRAZET_RADIO->PREFIX1 & 0xFFFFFF00) |
			(addrr << 0);
}

static inline bool isRXQueueFull() {
	return ((rxQueueHead + 1) % CRAZET_RX_FIFO_QUEUE_SIZE) == rxQueueTail;
}

static inline bool isRXQueueEmpty() {
	return rxQueueTail == rxQueueHead;
}

static inline void rXQueuePush() {
	rxQueueHead = (rxQueueHead + 1) % CRAZET_RX_FIFO_QUEUE_SIZE;
}

static inline void rXQueuePop() {
	rxQueueTail = (rxQueueTail + 1) % CRAZET_RX_FIFO_QUEUE_SIZE;
}

static inline bool isTXQueueFull() {
	return ((txQueueHead + 1) % CRAZET_TX_FIFO_QUEUE_SIZE) == txQueueTail;
}

static inline bool isTXQueueEmpty() {
	return txQueueTail == txQueueHead;
}

static inline void tXQueuePush() {
	txQueueHead = (txQueueHead + 1) % CRAZET_TX_FIFO_QUEUE_SIZE;
}

static inline void tXQueuePop() {
	txQueueTail = (txQueueTail + 1) % CRAZET_TX_FIFO_QUEUE_SIZE;
}
/******************************************************************************/

/******************************************************************************
 * CRAZET finite state machine (FSM)
 ******************************************************************************/

/* Handler to be called when a packet has been received. */
static void radioPacketReceivedHandler();
/* Handler to be called when a packet has been sent. */
static void radioPacketSentHandler();

void radioPacketReceivedHandler() {

}

static void radioPacketSentHandler() {

}
/******************************************************************************/


/******************************************************************************
 * CRAZET public functions
 ******************************************************************************/

bool initCrazet(const CrazetConfig * const config) {

	/* Copy CrazetConfig to module configuration. */
	memcpy(&crazetConfig, config, sizeof(CrazetConfig));

	/* Enable the RADIO module. */
	CRAZET_RADIO_ENABLE();

	/* Set the RADIO Bitrate. */
	setCrazetRadioBitRate(crazetConfig.bitrate);

	/* Set the RADIO transmit power .*/
	setCrazetRadioTXPower(crazetConfig.txpower);

	/* Set the RADIO frequency channel. */
	setCrazetRadioChannel(crazetConfig.channel);

	/* Enable the RADIO CRC check for data integrity. */
	setCrazetRadioCRC(crazetConfig.crc);

	/* Configure the RADIO on air packet. */
	configureCrazetRadioAirPacket(CRAZET_RADIO_ON_AIR_PACKET_BASE_ADDR_LEN,
			CRAZET_RADIO_ON_AIR_PACKET_PAYLOAD_LEN);

	/* Set the RADIO base and prefix addresses. */
	setCrazetRadioBase0Addr(CRAZET_RADIO_DEFAULT_BASE_0_ADDRR);
	setCrazetRadioBase1Addr(CRAZET_RADIO_DEFAULT_BASE_1_ADDRR);
	setCrazetRadioPrefix0Addr(CRAZET_RADIO_DEFAULT_PREFIX_0_ADDRR);
	setCrazetRadioPrefix1Addr(CRAZET_RADIO_DEFAULT_PREFIX_1_ADDRR);

	/* Set the CRAZET logical addresses. */
	updateBroadcastAddress(DEFAULT_BRAODCAST_ADDRR);
	updateJoinAddress(DEFAULT_JOIN_ADDR);
	updateTXAddress(DEFAULT_JOIN_ADDR);

	/* Set the CRAZET self node address. */
	updateSelfNodeAddress(crazetConfig.selfNodeAddress);

	/* Initialize the CRAZET PPI system. */
	enableCrazetPPISystem();

	/* Initialize the CRAZET FSM. */
	crazetRadioState = CRAZET_RADIO_RX_STATE;
	RadioOnDisabledRXEventHandler = radioPacketReceivedHandler;
	RadioOnDisabledTXEventHandler = radioPacketSentHandler;

	return true;
}

bool readCrazetPacket(CrazetPacket *packet) {
	if(!isRXQueueEmpty()) {
		memcpy(packet, &rxQueue[rxQueueTail], sizeof(CrazetPacket));
		rXQueuePop();
		return true;
	}
	return false;
}

bool sendCrazetPacket(const CrazetPacket * const packet) {
	if(!isTXQueueFull()) {
		memcpy(&txQueue[txQueueHead], packet, sizeof(CrazetPacket));
		tXQueuePush();
		return true;
	}
	return false;
}
/******************************************************************************/
