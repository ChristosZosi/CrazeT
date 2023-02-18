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

#define CRAZET_ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

#define LOGICAL_SELF_NODE_ADDRR  CRAZET_RADIO_LOGICAL_ADDRR_0
#define LOGICAL_BROADCAST_ADDRR  CRAZET_RADIO_LOGICAL_ADDRR_1
#define LOGICAL_JOIN_ADDRR       CRAZET_RADIO_LOGICAL_ADDRR_2
#define LOGICAL_TX_ADDRR         CRAZET_RADIO_LOGICAL_ADDRR_7

#define DEFAULT_BRAODCAST_ADDRR (0xFF)
#define DEFAULT_JOIN_ADDR       (0xFE)
#define DEFAULT_TX_ADDRR        DEFAULT_JOIN_ADDR

#define CRAZET_STOP_RX_JOIN_ADDR()                                   \
		{	CRAZET_RADIO->RXADDRESSES &= ~(1 << LOGICAL_JOIN_ADDRR); }

#define CRAZET_START_RX_JOIN_ADDR()                                  \
		{	CRAZET_RADIO->RXADDRESSES |= (1 << LOGICAL_JOIN_ADDRR); }

#define CRAZET_RADIO_DEFAULT_SHORTS (RADIO_SHORTS_READY_START_Enabled       << RADIO_SHORTS_READY_START_Pos       | \
									 RADIO_SHORTS_ADDRESS_RSSISTART_Enabled << RADIO_SHORTS_ADDRESS_RSSISTART_Pos | \
									 RADIO_SHORTS_END_DISABLE_Enabled       << RADIO_SHORTS_END_DISABLE_Pos       | \
									 RADIO_SHORTS_DISABLED_RSSISTOP_Enabled << RADIO_SHORTS_DISABLED_RSSISTOP_Pos)

#define CRAZET_TIMER_DEFAULT_SHORTS (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos | \
									 TIMER_SHORTS_COMPARE0_STOP_Enabled  << TIMER_SHORTS_COMPARE0_STOP_Pos)

#define CRAZET_TIMER_CC0_REG 0

#define CRAZET_TOKEN_CLOCKWISE_DIRECTION      (0)
#define CRAZET_TOKEN_ANTI_CLOCKWISE_DIRECTION (1)

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
 * CRAZET packet and frame types
 ******************************************************************************/
typedef enum {
	TOKEN_FRAME      = 0,
	CTS_FRAME        = 1,
	RTS_FRAME        = 2,
	DATA_FRAME       = 3,
	DATA_ACK_FRAME   = 4,
	INVITE_FRAME     = 5,
	INVITE_ACK_FRAME = 6
} CrazetFrame;

#define ON_AIR_PACKET_HEADER_SIZE (5)
#define ON_AIR_PACKET_DATA_SIZE \
	(CRAZET_RADIO_ON_AIR_PACKET_PAYLOAD_LEN - ON_AIR_PACKET_HEADER_SIZE)
typedef struct crazet_on_air_packet {
	uint8_t packetSize;
	uint8_t frameType;
	uint8_t source_id;
	uint8_t target_id;
	uint8_t data_size;
	uint8_t data[ON_AIR_PACKET_DATA_SIZE];
}__attribute__((packed, aligned(1))) CrazetOnAirPacket;

#define DATA_FRAME_HEADER_SIZE (2)
#define DATA_FRAME_DATA_SIZE   (ON_AIR_PACKET_DATA_SIZE - DATA_FRAME_HEADER_SIZE)
typedef struct crazet_data_frame {
	uint8_t ack;
	uint8_t broadcast;
	uint8_t data[DATA_FRAME_DATA_SIZE];
}__attribute__((packed, aligned(1))) CrazetDataFrame;

#define TOKEN_FRAME_SIZE (8)
typedef struct crazet_token_frame {
	uint32_t tokenID;
	uint8_t tokenDirection;
	uint8_t failed_node_id;
	uint8_t failed_node_pred;
	uint8_t new_node_successor;
}__attribute__((packed, aligned(1))) CrazetTokenFrame;

typedef struct crazet_queue_packet {
	uint32_t crc;
	uint8_t rssi;
	uint8_t logicalAddres;
	CrazetOnAirPacket onAirPacket;
}__attribute__((packed, aligned(1))) CrazetQueuePacket;

/******************************************************************************/

/******************************************************************************
 * CRAZET module static variable
 ******************************************************************************/

static CrazetConfig ctConfig;

/* CrazeT RX FIFO-Queue */
static CrazetQueuePacket rxQueue[CRAZET_RX_FIFO_QUEUE_SIZE];
static volatile uint8_t rxQueueHead = 0;
static volatile uint8_t rxQueueTail = 0;

/* CrazeT TX FIFO-Queue */
static CrazetQueuePacket txQueue[CRAZET_TX_FIFO_QUEUE_SIZE];
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

typedef enum {
	TIMEOUT_EVENT          = -1,
	TOKEN_FRAME_EVENT      = TOKEN_FRAME,
	CTS_FRAME_EVENT        = CTS_FRAME,
	RTS_FRAME_EVENT        = RTS_FRAME,
	DATA_FRAME_EVENT       = DATA_FRAME,
	DATA_ACK_FRAME_EVENT   = DATA_ACK_FRAME,
	INVITE_FRAME_EVENT     = INVITE_FRAME,
	INVITE_ACK_FRAME_EVENT = INVITE_ACK_FRAME
} CrazetEvent;

typedef enum {
	IDLE_STATE = 0,
	RX_WAIT_CTS_STATE,
	RX_WAIT_RTS_STATE,
	RX_WAIT_DATA_ACK_STATE,
	RX_WAIT_INVITE_ACK_STATE
} CrazetState;

typedef struct crazet_finite_state_machine_data {
	uint8_t successor_id;
	uint8_t predecessor_id;

	uint8_t token_retries;
	uint8_t rts_retries;
	uint8_t data_retries;

	uint32_t data_frame_crc;
	uint8_t data_frame_src;

	CrazetState state;
	CrazetState lastState;

	bool is_network_active;
	uint8_t token_direction;
	bool is_token_direction_iverted;
	uint32_t token_id;
	uint32_t lastInviteToken_id;
	bool amITokenCreator;

	CrazetTokenFrame tokenFrame;
	CrazetOnAirPacket* retransmitPacket;
} CrazetFSMData;

static CrazetFSMData fsmData;
static CrazetOnAirPacket servicePacket;

static void resetCrazetFSMData(CrazetFSMData* data) {
	data->successor_id   = ctConfig.selfNodeAddress;
	data->predecessor_id = ctConfig.selfNodeAddress;

	data->token_retries = 0;
	data->rts_retries   = 0;
	data->data_retries  = 0;

	data->data_frame_crc = 0;
	data->data_frame_src = ctConfig.selfNodeAddress;

	data->state     = IDLE_STATE;
	data->lastState = IDLE_STATE;

	data->is_network_active         = false;
	data->token_direction           = 0;
	data->token_id                  = 0;
	data->lastInviteToken_id        = 0;
	data->amITokenCreator           = false;
	data->is_token_direction_iverted = false;

	data->retransmitPacket = &servicePacket;
}

static void initServicePacket(CrazetOnAirPacket* pk) {
	pk->packetSize = ON_AIR_PACKET_HEADER_SIZE;
	pk->source_id  = ctConfig.selfNodeAddress;
	pk->target_id  = ctConfig.selfNodeAddress;
}

static void sendRadioPacketAuto(const CrazetOnAirPacket * const txPk, uint32_t logTxAddres, uint32_t timeout) {
	CRAZET_RADIO->PACKETPTR = (uint32_t)txPk;
	CRAZET_RADIO->TXADDRESS = logTxAddres;

	CRAZET_RADIO->SHORTS = CRAZET_RADIO_DEFAULT_SHORTS |
			RADIO_SHORTS_DISABLED_RXEN_Enabled << RADIO_SHORTS_DISABLED_RXEN_Pos;

	crazetRadioState = CRAZET_RADIO_TX_STATE;

	CRAZET_TIMER->CC[CRAZET_TIMER_CC0_REG] = timeout;

	CRAZET_RADIO->TASKS_TXEN = 1U;
}

static void sendRadioPacketManual(const CrazetOnAirPacket * const txPk, uint32_t logTxAddres) {
	CRAZET_RADIO->PACKETPTR = (uint32_t)txPk;
	CRAZET_RADIO->TXADDRESS = logTxAddres;

	CRAZET_RADIO_DISABLE_IRQ();
	PPI->CHENCLR = 0U;

	CRAZET_RADIO->TASKS_TXEN = 1U;

	while(CRAZET_RADIO->EVENTS_DISABLED == 0U);

	CRAZET_RADIO_ENABLE_IRQ();
	PPI->CHENSET = (1 << PPI_RADIO_DISABLE_ON_CC0) |
				   (1 << PPI_TIMER_SHUTDOWN_ON_RADIO_ADDR);
}

static void rumpUpRadioInRx() {
 	CRAZET_RADIO->TASKS_RXEN = 1U;
 	CRAZET_TIMER->TASKS_START = 1U;
}

/******************************************************************************
 * IDLE_STATE Event handler functions
 ******************************************************************************/
static void on_idle_timeout_event(const CrazetQueuePacket * const rxPk) {
	(void)rxPk;

	CrazetOnAirPacket* txPk = &servicePacket;
	txPk->frameType  = INVITE_FRAME;
	txPk->packetSize = ON_AIR_PACKET_HEADER_SIZE;

	fsmData.state             = RX_WAIT_INVITE_ACK_STATE;
	fsmData.lastState         = IDLE_STATE;

	sendRadioPacketAuto(txPk, LOGICAL_JOIN_ADDRR, ctConfig.invitationTimeoutUs);
}

static void on_idle_token_frame_event(const CrazetQueuePacket * const rxPk) {
	const CrazetTokenFrame* rxTokenFrame = (const CrazetTokenFrame*)rxPk->onAirPacket.data;

	CrazetOnAirPacket* txPk = &servicePacket;

	fsmData.amITokenCreator = false;

	if(fsmData.is_network_active) {

		if(rxTokenFrame->tokenID == fsmData.token_id) {
			fsmData.token_id++;
			fsmData.amITokenCreator = true;
		} else {

			if(rxTokenFrame->tokenID < fsmData.token_id && rxTokenFrame != 0) {
				/* Received token is not valid. Just send RTS frame
				 * to sender, to destroy invalid token. */
				txPk->target_id = rxPk->onAirPacket.source_id;
				updateTXAddress(txPk->target_id);
				sendRadioPacketAuto(txPk, LOGICAL_TX_ADDRR, ctConfig.networkIdleTimeoutUs);
				return;
			}
			fsmData.token_id = rxTokenFrame->tokenID;
		}

	} else {
		fsmData.is_network_active = true;
		CRAZET_STOP_RX_JOIN_ADDR();

		fsmData.token_id        = rxTokenFrame->tokenID;
		fsmData.token_direction = rxTokenFrame->tokenDirection;

		fsmData.successor_id   = rxTokenFrame->new_node_successor;
		fsmData.predecessor_id = rxPk->onAirPacket.source_id;
	}

	if(rxTokenFrame->tokenDirection != fsmData.token_direction) {
		fsmData.token_direction = rxTokenFrame->tokenDirection;

		if(rxTokenFrame->failed_node_id == fsmData.successor_id) {
			fsmData.successor_id = rxTokenFrame->failed_node_pred;

			fsmData.tokenFrame.failed_node_pred = ctConfig.selfNodeAddress;
			fsmData.tokenFrame.failed_node_id  = ctConfig.selfNodeAddress;

		} else {
			fsmData.successor_id = fsmData.predecessor_id;

			fsmData.tokenFrame.failed_node_pred = rxTokenFrame->failed_node_pred;
			fsmData.tokenFrame.failed_node_id  = rxTokenFrame->failed_node_id;
		}
	}

	fsmData.tokenFrame.tokenID        = fsmData.token_id;
	fsmData.tokenFrame.tokenDirection = fsmData.token_direction;

	fsmData.data_frame_crc = 0;
	fsmData.data_frame_src = ctConfig.selfNodeAddress;

	fsmData.predecessor_id = rxPk->onAirPacket.source_id;

	txPk->packetSize = ON_AIR_PACKET_HEADER_SIZE;
	txPk->frameType  = CTS_FRAME;
	txPk->target_id  = fsmData.predecessor_id;

	fsmData.retransmitPacket = txPk;
	fsmData.rts_retries = 0;

	fsmData.state     = RX_WAIT_CTS_STATE;
	fsmData.lastState = IDLE_STATE;

	updateTXAddress(txPk->target_id);
	sendRadioPacketAuto(txPk, LOGICAL_TX_ADDRR, ctConfig.rtsFrameRetransmitUsDelay);
}

static void on_idle_data_frame_event(const CrazetQueuePacket * const rxPk) {
	/* Check if received DATA_FRAME is a new one. */
	if(rxPk->crc == fsmData.data_frame_crc &&
			rxPk->onAirPacket.source_id == fsmData.data_frame_src) {

		/* Update data of the DATA_FRAME. */
		fsmData.data_frame_crc = rxPk->crc;
		fsmData.data_frame_src = rxPk->onAirPacket.source_id;

		if(!isRXQueueFull()) {
			// Push new DATA_FRAME in the rxQueue;
			rXQueuePush();
		} else {
			// TODO: Raise rxQueue full
		}


	} else {
		// TODO: Raise duplicated DATA_FRAME
	}

	const CrazetDataFrame* rxDataFrame = (const CrazetDataFrame*)&rxPk->onAirPacket.data;
	if(rxDataFrame->broadcast == 1 || rxDataFrame->ack != 1) {
		/* Received DATA_FRAME is either broadcast or no ack. */
		rumpUpRadioInRx();
		return;
	}

	CrazetOnAirPacket *txPk = &servicePacket;
	txPk->packetSize = ON_AIR_PACKET_HEADER_SIZE;
	txPk->target_id = txPk->source_id;
	txPk->frameType = DATA_ACK_FRAME;

	updateTXAddress(txPk->target_id);
	sendRadioPacketAuto(txPk, LOGICAL_TX_ADDRR, ctConfig.networkIdleTimeoutUs);
}

static void on_idle_rts_frame_event(const CrazetQueuePacket * const rxPk) {
	(void)rxPk;
	// TODO: Assert pk->onAirPacket.source_id == fsmData.successor_id
	sendRadioPacketAuto(fsmData.retransmitPacket, LOGICAL_TX_ADDRR, ctConfig.networkIdleTimeoutUs);
}

static void on_idle_invite_frame_event(const CrazetQueuePacket * const rxPk) {
	(void)rxPk;
	CrazetOnAirPacket *txPk = &servicePacket;
	txPk->packetSize = ON_AIR_PACKET_HEADER_SIZE;
	txPk->target_id = txPk->source_id;
	txPk->frameType = INVITE_ACK_FRAME;

	updateTXAddress(txPk->source_id);
	sendRadioPacketAuto(txPk, LOGICAL_TX_ADDRR, ctConfig.networkIdleTimeoutUs);
}
/******************************************************************************/

static bool shouldInvite() {
	if(fsmData.amITokenCreator &&
			((fsmData.token_id - fsmData.lastInviteToken_id) > ctConfig.invitationRate)) {

		fsmData.lastInviteToken_id = fsmData.token_id;
		return true;
	}
	return false;
}

static void sendInviteFrame() {
	CrazetOnAirPacket* txPk = &servicePacket;
	txPk->frameType  = INVITE_FRAME;
	txPk->packetSize = ON_AIR_PACKET_HEADER_SIZE;

	fsmData.state = RX_WAIT_INVITE_ACK_STATE;
	sendRadioPacketAuto(txPk, LOGICAL_JOIN_ADDRR, ctConfig.invitationTimeoutUs);
}

static void sendTokenFrame() {
	CrazetOnAirPacket* txPk = &servicePacket;
	txPk->frameType  = TOKEN_FRAME;
	txPk->data_size  = TOKEN_FRAME_SIZE;
	txPk->packetSize = txPk->data_size + ON_AIR_PACKET_HEADER_SIZE;
	txPk->target_id  = fsmData.successor_id;

	memcpy(txPk->data, &fsmData.tokenFrame, TOKEN_FRAME_SIZE);

	fsmData.token_retries    = 0;
	fsmData.retransmitPacket = txPk;
	fsmData.state            = RX_WAIT_RTS_STATE;

	updateTXAddress(txPk->target_id);
	sendRadioPacketAuto(txPk, LOGICAL_TX_ADDRR, ctConfig.tokenFrameRetransmitUsDelay);
}

/******************************************************************************
 * RX_WAIT_CTS_STATE Event handler functions
 ******************************************************************************/
static void on_wait_cts_next_action() {

	fsmData.lastState = RX_WAIT_CTS_STATE;

	if(!isTXQueueEmpty()) {
		CrazetOnAirPacket* txPk = &txQueue[txQueueHead].onAirPacket;
		const CrazetDataFrame* txDataFrame = (const CrazetDataFrame*)&txPk->data;

		// TODO: Add define for ack and broadcast
		if(txDataFrame->ack == 1) {

			fsmData.retransmitPacket = txPk;
			fsmData.data_retries = 0;

			fsmData.state = RX_WAIT_DATA_ACK_STATE;
			updateTXAddress(txPk->target_id);
			sendRadioPacketAuto(txPk, LOGICAL_TX_ADDRR, ctConfig.dataFrameRetransmitUsDelay);
			return;
		}

		uint8_t txLogAddr;

		if(txDataFrame->broadcast == 1) {
			txLogAddr = LOGICAL_BROADCAST_ADDRR;
		} else {
			updateTXAddress(txPk->target_id);
			txLogAddr = LOGICAL_TX_ADDRR;
		}
		sendRadioPacketManual(txPk, txLogAddr);
		tXQueuePop();
	}

	if(shouldInvite()) {
		sendInviteFrame();
		return;
	}
	sendTokenFrame();
}

static void on_wait_cts_timeout_event(const CrazetQueuePacket * const rxPk) {
	(void)rxPk;
	if(fsmData.rts_retries < ctConfig.rtsFrameRetransmitAttempts) {
		fsmData.rts_retries++;
		sendRadioPacketAuto(fsmData.retransmitPacket, LOGICAL_TX_ADDRR, ctConfig.rtsFrameRetransmitUsDelay);
		return;
	}
	//TODO: Raise error CTS_FRAME failed to receive
	on_wait_cts_next_action();
}

static void on_wait_cts_token_frame_event(const CrazetQueuePacket * const rxPk) {
	(void)rxPk;
	fsmData.rts_retries = 0;
	sendRadioPacketAuto(fsmData.retransmitPacket, LOGICAL_TX_ADDRR, ctConfig.dataFrameRetransmitUsDelay);
}

static void on_wait_cts_cts_frame_event(const CrazetQueuePacket * const rxPk) {
	(void)rxPk;
	on_wait_cts_next_action();
}
/******************************************************************************/

/******************************************************************************
 * RX_WAIT_DATA_ACK Event handler functions
 ******************************************************************************/
static void on_wait_data_ack_next_action() {
	tXQueuePush();
	fsmData.lastState = RX_WAIT_DATA_ACK_STATE;
	if(shouldInvite()) {
		sendInviteFrame();
		return;
	}
	sendTokenFrame();
}

static void on_wait_data_ack_timeout_event(const CrazetQueuePacket * const rxPk) {
	(void)rxPk;
	if(fsmData.data_retries < ctConfig.dataFrameRetransmitAttempts) {
		fsmData.data_retries++;
		sendRadioPacketAuto(fsmData.retransmitPacket, LOGICAL_TX_ADDRR, ctConfig.dataFrameRetransmitUsDelay);
		return;
	}
	// TODO: Raise an error
	on_wait_data_ack_next_action();
}

static void on_wait_data_ack_data_ack_frame_event(const CrazetQueuePacket * const rxPk) {
	(void)rxPk;
	on_wait_data_ack_next_action();
}
/******************************************************************************/

/******************************************************************************
 * RX_WAIT_INVITE_ACK Event handler functions
 ******************************************************************************/
static void on_wait_invite_ack_timeout_event(const CrazetQueuePacket * const rxPk) {
	(void)rxPk;
	if(fsmData.lastState == IDLE_STATE) {
		fsmData.state = IDLE_STATE;
		fsmData.lastState = RX_WAIT_INVITE_ACK_STATE;

		fsmData.is_network_active = false;
		CRAZET_START_RX_JOIN_ADDR();

		CRAZET_TIMER->CC[CRAZET_TIMER_CC0_REG] = ctConfig.networkIdleTimeoutUs;
		rumpUpRadioInRx();
		return;
	}
	fsmData.lastState = RX_WAIT_INVITE_ACK_STATE;
	sendTokenFrame();
}

static void on_wait_invite_ack_invite_ack_frame_event(const CrazetQueuePacket * const rxPk) {

	if(fsmData.lastState == IDLE_STATE) {
		fsmData.token_id++;
		fsmData.tokenFrame.tokenID = fsmData.token_id;
		fsmData.is_network_active = true;
		CRAZET_STOP_RX_JOIN_ADDR();
	}

	fsmData.tokenFrame.new_node_successor = fsmData.successor_id;
	fsmData.successor_id = rxPk->onAirPacket.source_id;

	fsmData.lastState = RX_WAIT_INVITE_ACK_STATE;
	sendTokenFrame();
}
/******************************************************************************/


/******************************************************************************
 * RX_WAIT_RTS_STATE Event handler functions
 ******************************************************************************/
static void on_wait_rts_timeout_event(const CrazetQueuePacket * const rxPk) {
	(void)rxPk;
	if(fsmData.token_retries < ctConfig.tokenFrameRetransmitAttempts) {
		fsmData.token_retries++;
		sendRadioPacketAuto(fsmData.retransmitPacket, LOGICAL_TX_ADDRR, ctConfig.tokenFrameRetransmitUsDelay);
		return;
	}

	if(fsmData.is_token_direction_iverted) {
		fsmData.is_token_direction_iverted = false;
		fsmData.is_network_active = false;
		CRAZET_START_RX_JOIN_ADDR();

		fsmData.state     = IDLE_STATE;
		fsmData.lastState = RX_WAIT_RTS_STATE;

		CRAZET_TIMER->CC[CRAZET_TIMER_CC0_REG] = ctConfig.networkIdleTimeoutUs;
		rumpUpRadioInRx();
		return;
	}

	fsmData.is_token_direction_iverted = true;
	if(fsmData.token_direction == CRAZET_TOKEN_CLOCKWISE_DIRECTION) {
		fsmData.token_direction = CRAZET_TOKEN_ANTI_CLOCKWISE_DIRECTION;
	} else {
		fsmData.token_direction = CRAZET_TOKEN_CLOCKWISE_DIRECTION;
	}

	fsmData.tokenFrame.failed_node_id   = fsmData.successor_id;
	fsmData.tokenFrame.failed_node_pred = ctConfig.selfNodeAddress;

	fsmData.successor_id = fsmData.predecessor_id;

	sendTokenFrame();
}

static void on_wait_rts_rts_frame_event(const CrazetQueuePacket * const rxPk) {
	(void)rxPk;
	CrazetOnAirPacket* txPk = &servicePacket;

	txPk->packetSize = ON_AIR_PACKET_HEADER_SIZE;
	txPk->data_size  = 0;
	txPk->frameType  = CTS_FRAME;
	txPk->target_id  = fsmData.successor_id;

	fsmData.is_token_direction_iverted = false;

	fsmData.state     = IDLE_STATE;
	fsmData.lastState = RX_WAIT_RTS_STATE;

	updateTXAddress(txPk->target_id);
	sendRadioPacketAuto(txPk, LOGICAL_TX_ADDRR, ctConfig.networkIdleTimeoutUs);
}
/******************************************************************************/


static void crazet_stub_handler(const CrazetQueuePacket * const rxPk) {
	(void)rxPk;
	/* TODO: Raise an error */
	CRAZET_RADIO->TASKS_RXEN = 1U;
	CRAZET_TIMER->TASKS_START = 1U;
}

typedef void(*CrazetEventHandler)(const CrazetQueuePacket * const rxPk);

typedef struct event_handlers {
	CrazetEvent event;
	CrazetEventHandler handler;
} EventHandlers;

static const EventHandlers idleStateHandlers[] = {
		{TIMEOUT_EVENT,      on_idle_timeout_event     },
		{TOKEN_FRAME_EVENT,  on_idle_token_frame_event },
		{DATA_FRAME_EVENT,   on_idle_data_frame_event  },
		{RTS_FRAME_EVENT,    on_idle_rts_frame_event   },
		{INVITE_FRAME_EVENT, on_idle_invite_frame_event}
};

static const EventHandlers rxWaitCtsStateHandlers[] = {
		{TIMEOUT_EVENT,     on_wait_cts_timeout_event    },
		{TOKEN_FRAME_EVENT, on_wait_cts_token_frame_event},
		{CTS_FRAME_EVENT,   on_wait_cts_cts_frame_event  }
};

static const EventHandlers rxWaitRtsStateHandlers[] = {
		{TIMEOUT_EVENT,   on_wait_rts_timeout_event  },
		{RTS_FRAME_EVENT, on_wait_rts_rts_frame_event}
};

static const EventHandlers rxWaitDataAckStateHandlers[] = {
		{TIMEOUT_EVENT,        on_wait_data_ack_timeout_event       },
		{DATA_ACK_FRAME_EVENT, on_wait_data_ack_data_ack_frame_event}
};

static const EventHandlers rxWaitInviteAckStateHandlers[] = {
		{TIMEOUT_EVENT,          on_wait_invite_ack_timeout_event         },
		{INVITE_ACK_FRAME_EVENT, on_wait_invite_ack_invite_ack_frame_event}
};

typedef struct state {
	int handlerCounts;
	const EventHandlers *handlers;
} StateMachine;

static const StateMachine stateMachine[] = {
	[IDLE_STATE]               = {CRAZET_ARRAY_SIZE(idleStateHandlers)            ,idleStateHandlers},
	[RX_WAIT_CTS_STATE]        = {CRAZET_ARRAY_SIZE(rxWaitCtsStateHandlers)       ,rxWaitCtsStateHandlers},
	[RX_WAIT_RTS_STATE]        = {CRAZET_ARRAY_SIZE(rxWaitRtsStateHandlers)       ,rxWaitRtsStateHandlers},
	[RX_WAIT_DATA_ACK_STATE]   = {CRAZET_ARRAY_SIZE(rxWaitDataAckStateHandlers)   ,rxWaitDataAckStateHandlers},
	[RX_WAIT_INVITE_ACK_STATE] = {CRAZET_ARRAY_SIZE(rxWaitInviteAckStateHandlers) ,rxWaitInviteAckStateHandlers},
};

/* Handler to be called when a packet has been received. */
static void radioPacketReceivedHandler() {

	CrazetEvent event;
	CrazetQueuePacket* rxPk;

	if(CRAZET_TIMER->EVENTS_COMPARE[CRAZET_TIMER_CC0_REG]) {
		/* If a timeout occurred. */
		CRAZET_TIMER->EVENTS_COMPARE[CRAZET_TIMER_CC0_REG] = 0U;
		event = TIMEOUT_EVENT;
		rxPk = NULL;
	} else if(CRAZET_RADIO->EVENTS_END && CRAZET_RADIO->CRCSTATUS != 0) {
		/* If a new VALID packet has been received. */
		rxPk = &rxQueue[rxQueueHead];

		rxPk->crc = CRAZET_RADIO->RXCRC;
		rxPk->rssi = CRAZET_RADIO->RSSISAMPLE;
		rxPk->logicalAddres = CRAZET_RADIO->RXMATCH;

		event = rxPk->onAirPacket.frameType;
	} else {
		/*
		 * It is quite possible that a corrupted packet has
		 * been received and the CRAZET_RADIO interrupt
		 * kicked. In this case we just start the CRAZET_RADIO
		 * and CRAZET_TIMER and leave the CRAZET state machine
		 * as it is.
		 */
		rumpUpRadioInRx();
		return;
	}

	/* Call the handler of the event handler of the current state. */
	CrazetEventHandler handler = crazet_stub_handler;
	for(int i = 0; i < stateMachine[fsmData.state].handlerCounts; i++) {
		if(event == stateMachine[fsmData.state].handlers[i].event) {
			handler = stateMachine[fsmData.state].handlers[i].handler;
			break;
		}
	}
	handler(rxPk);
}

/* Handler to be called when a packet has been sent. */
static void radioPacketSentHandler() {
	/*
	 * CRAZET_RADIO is already starting to rump in in RX state,
	 * because of the SHORT RADIO_SHORTS_DISABLED_TXEN_Enabled.
	 */
	CRAZET_RADIO->SHORTS = CRAZET_RADIO_DEFAULT_SHORTS;
	CRAZET_RADIO->PACKETPTR = (uint32_t)&rxQueue[rxQueueHead].onAirPacket;

	crazetRadioState = CRAZET_RADIO_RX_STATE;

	/* In RX state the timer should always run. */
	CRAZET_TIMER->TASKS_START = 1U;
}

/******************************************************************************/


/******************************************************************************
 * CRAZET public functions
 ******************************************************************************/

bool initCrazet(const CrazetConfig * const config) {

	/* Copy CrazetConfig to module configuration. */
	memcpy(&ctConfig, config, sizeof(CrazetConfig));

	/* Enable the RADIO module. */
	CRAZET_RADIO_ENABLE();

	/* Set the RADIO Bitrate. */
	setCrazetRadioBitRate(ctConfig.bitrate);

	/* Set the RADIO transmit power .*/
	setCrazetRadioTXPower(ctConfig.txpower);

	/* Set the RADIO frequency channel. */
	setCrazetRadioChannel(ctConfig.channel);

	/* Enable the RADIO CRC check for data integrity. */
	setCrazetRadioCRC(ctConfig.crc);

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
	updateSelfNodeAddress(ctConfig.selfNodeAddress);

	/* Initialize the CRAZET PPI system. */
	enableCrazetPPISystem();

	/* Set the CRAZET_TIMER configuration. */
	CRAZET_TIMER->PRESCALER = 4U;
	CRAZET_TIMER->BITMODE = 3;

	/* Initialize the CRAZET FSM. */
	crazetRadioState = CRAZET_RADIO_RX_STATE;
	RadioOnDisabledRXEventHandler = radioPacketReceivedHandler;
	RadioOnDisabledTXEventHandler = radioPacketSentHandler;

	resetCrazetFSMData(&fsmData);
	initServicePacket(&servicePacket);

	CRAZET_RADIO->RXADDRESSES = 1 << LOGICAL_SELF_NODE_ADDRR |
			1 << LOGICAL_BROADCAST_ADDRR |
			1 << LOGICAL_JOIN_ADDRR;

	CRAZET_RADIO->TXADDRESS = 1 << LOGICAL_TX_ADDRR;

	CRAZET_TIMER->CC[CRAZET_TIMER_CC0_REG] = ctConfig.networkIdleTimeoutUs;

	CRAZET_TIMER->SHORTS = CRAZET_TIMER_DEFAULT_SHORTS;
	CRAZET_RADIO->SHORTS = CRAZET_RADIO_DEFAULT_SHORTS;

	PPI->CHENCLR = 0U;
	PPI->CHENSET = (1 << PPI_RADIO_DISABLE_ON_CC0) |
				   (1 << PPI_TIMER_SHUTDOWN_ON_RADIO_ADDR);

	/* Enable interrupt routine. */
	CRAZET_RADIO->INTENSET = RADIO_INTENSET_DISABLED_Enabled << RADIO_INTENSET_DISABLED_Pos;
	NVIC_SetPriority(RADIO_IRQn, 1);
	CRAZET_RADIO_ENABLE_IRQ();

	NRF_RADIO->PACKETPTR = (uint32_t)&rxQueue[rxQueueHead].onAirPacket;

	/* Start the CRAZET_RADIO in RX mode. */
	CRAZET_RADIO->TASKS_RXEN = 1U;

	/* Start the CRAZET_TIMER. */
	CRAZET_TIMER->TASKS_START = 1U;

	return true;
}

bool readCrazetPacket(CrazetPacket *packet) {
	if(!isRXQueueEmpty()) {
		const CrazetQueuePacket* queuedPk = &rxQueue[rxQueueTail];
		packet->source_id = queuedPk->onAirPacket.source_id;
		packet->target_id = queuedPk->onAirPacket.target_id;
		packet->dataSize  = queuedPk->onAirPacket.data_size - 2;

		const CrazetDataFrame* dataFrame  = (const CrazetDataFrame*)&queuedPk->onAirPacket.data;
		packet->ack       = dataFrame->ack;
		packet->broadcast = dataFrame->broadcast;

		// TODO: Add assert (packet->dataSize <= CRAZET_MAX_PACKET_DATA_SIZE)
		memcpy(&packet->data, &dataFrame->data, packet->dataSize);

		rXQueuePop();
		return true;
	}
	return false;
}

bool sendCrazetPacket(const CrazetPacket * const packet) {
	if(!isTXQueueFull()) {
		CrazetQueuePacket* queuePk = &txQueue[txQueueHead];
		queuePk->onAirPacket.source_id = packet->source_id;
		queuePk->onAirPacket.target_id = packet->target_id;
		queuePk->onAirPacket.data_size = packet->dataSize + 2;

		queuePk->onAirPacket.packetSize = queuePk->onAirPacket.data_size + ON_AIR_PACKET_HEADER_SIZE;
		queuePk->onAirPacket.frameType = DATA_FRAME;

		CrazetDataFrame* dataFrame  = (CrazetDataFrame*)&queuePk->onAirPacket.data;
		dataFrame->ack       = packet->ack;
		dataFrame->broadcast = packet->broadcast;

		// TODO: Add assert (queuePk->onAirPacket.data_size <= ON_AIR_PACKET_DATA_SIZE)
		memcpy(&dataFrame->data, &packet->data, packet->dataSize);

		tXQueuePush();
		return true;
	}
	return false;
}
/******************************************************************************/
