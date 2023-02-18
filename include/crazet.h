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
 * crazet.h
 *
 *  Created on: 23.08.2022
 *      Author: Christos Zosimidis
 */

#ifndef CRAZET_CRAZET_H_
#define CRAZET_CRAZET_H_

/* CrazeT includes. */
#include "radio_hw_ct.h"

/* Standard libraries includes. */
#include <stdint.h>
#include <stdbool.h>

/******************************************************************************
 * CRAZET configuration types
 ******************************************************************************/
#define CRAZET_MAX_PACKET_SIZE			64
#define CRAZET_MAX_PACKET_DATA_SIZE		32 // TODO: set this to an actual value

typedef struct crazet_config_t {

	/* Crazet RADIO configuration. */
	CrazetRadioBitrate bitrate;
	CrazetRadioTXPower txpower;
	CrazetRadioChannel channel;
	CrazetRadioCrc crc;

	/* Crazet timings configuration. */
	uint16_t dataFrameRetransmitUsDelay;
	uint16_t tokenFrameRetransmitUsDelay;
	uint16_t rtsFrameRetransmitUsDelay;

	uint8_t dataFrameRetransmitAttempts;
	uint8_t tokenFrameRetransmitAttempts;
	uint8_t rtsFrameRetransmitAttempts;

	uint32_t networkIdleTimeoutUs;
	uint32_t invitationTimeoutUs;
	uint32_t invitationRate;

	uint8_t selfNodeAddress;
} CrazetConfig;

// TODO: Determine actual default values for the CrazetConfig
#define CRAZET_DEFAULT_CONFIG { .bitrate                      = RADIO_MODE_MODE_Nrf_2Mbit,  \
								.txpower                      = RADIO_TXPOWER_TXPOWER_0dBm, \
								.channel                      = 100,                        \
								.crc                          = RADIO_CRCCNF_LEN_Two,       \
								.dataFrameRetransmitUsDelay   = 2500,                       \
								.tokenFrameRetransmitUsDelay  = 2500,                       \
								.rtsFrameRetransmitUsDelay    = 2500,                       \
								.dataFrameRetransmitAttempts  = 100,                        \
								.tokenFrameRetransmitAttempts = 100,                        \
								.rtsFrameRetransmitAttempts   = 100,                        \
								.networkIdleTimeoutUs         = 1000000,                    \
								.invitationTimeoutUs          = 2500,                       \
								.invitationRate               = 50,                         \
								.selfNodeAddress              = 0x01                        \
}

typedef struct crazet_packet {
	uint8_t source_id;
	uint8_t target_id;
	uint8_t dataSize;
	bool ack;
	bool broadcast;
	uint8_t data[CRAZET_MAX_PACKET_DATA_SIZE];
} CrazetPacket;
/******************************************************************************/

/******************************************************************************
 * CRAZET public functions
 ******************************************************************************/

bool initCrazet(const CrazetConfig * const config);

bool readCrazetPacket(CrazetPacket* packet);
bool sendCrazetPacket(const CrazetPacket * const packet);

/******************************************************************************/

#endif /* CRAZET_CRAZET_H_ */
