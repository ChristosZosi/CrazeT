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
 * crazet_hw_ct.h
 *
 *  Created on: 23.08.2022
 *      Author: Christos Zosimidis
 */

#ifndef CRAZET_RADIO_HW_CT_H_
#define CRAZET_RADIO_HW_CT_H_

/* Nordic Semiconductors includes. */
#include <nrf.h>

/* Standard libraries includes. */
#include <stdint.h>

/******************************************************************************
 * RADIO configuration types
 ******************************************************************************/
#define CRAZET_RADIO NRF_RADIO

#define CRAZET_RADIO_ON_AIR_PACKET_BASE_ADDR_LEN (4UL)
#define CRAZET_RADIO_ON_AIR_PACKET_PAYLOAD_LEN   (64UL)

#define CRAZET_RADIO_DEFAULT_BASE_0_ADDRR            (0xF7F7F7F7)
#define CRAZET_RADIO_DEFAULT_BASE_1_ADDRR            (0xF7F7F7F7)
#define CRAZET_RADIO_DEFAULT_PREFIX_0_ADDRR          (0xC3C2C1C0)
#define CRAZET_RADIO_DEFAULT_PREFIX_1_ADDRR          (0xC7C6C5C4)

#define CRAZET_RADIO_LOGICAL_ADDRR_0 (0UL) /* BASE0 + PREFIX0.AP0 */
#define CRAZET_RADIO_LOGICAL_ADDRR_1 (1UL) /* BASE1 + PREFIX0.AP1 */
#define CRAZET_RADIO_LOGICAL_ADDRR_2 (2UL) /* BASE1 + PREFIX0.AP2 */
#define CRAZET_RADIO_LOGICAL_ADDRR_3 (3UL) /* BASE1 + PREFIX0.AP3 */

#define CRAZET_RADIO_LOGICAL_ADDRR_4 (4UL) /* BASE1 + PREFIX1.AP4 */
#define CRAZET_RADIO_LOGICAL_ADDRR_5 (5UL) /* BASE1 + PREFIX1.AP5 */
#define CRAZET_RADIO_LOGICAL_ADDRR_6 (6UL) /* BASE1 + PREFIX1.AP6 */
#define CRAZET_RADIO_LOGICAL_ADDRR_7 (7UL) /* BASE1 + PREFIX1.AP7 */

typedef enum {
#ifdef NRF51
	CRAZET_RADIO_BITRATE_250KBPS = RADIO_MODE_MODE_Nrf_250Kbit,
#endif
	CRAZET_RADIO_BITRATE_1MBPS = RADIO_MODE_MODE_Nrf_1Mbit,
	CRAZET_RADIO_BITRATE_2MBPS = RADIO_MODE_MODE_Nrf_2Mbit
} CrazetRadioBitrate;

typedef enum {
#ifdef NRF52840_XXAA
	CRAZET_RADIO_TXPOWER_8DBM = RADIO_TXPOWER_TXPOWER_Pos8dBm,
	CRAZET_RADIO_TXPOWER_7DBM = RADIO_TXPOWER_TXPOWER_Pos7dBm,
	CRAZET_RADIO_TXPOWER_6DBM = RADIO_TXPOWER_TXPOWER_Pos6dBm,
	CRAZET_RADIO_TXPOWER_5DBM = RADIO_TXPOWER_TXPOWER_Pos5dBm,
	CRAZET_RADIO_TXPOWER_3DBM = RADIO_TXPOWER_TXPOWER_Pos3dBm,
	CRAZET_RADIO_TXPOWER_2DBM = RADIO_TXPOWER_TXPOWER_Pos2dBm,
#endif
	CRAZET_RADIO_TXPOWER_0DBM = RADIO_TXPOWER_TXPOWER_0dBm,
	CRAZET_RADIO_TXPOWER_NEG4DBM = RADIO_TXPOWER_TXPOWER_Neg4dBm,
	CRAZET_RADIO_TXPOWER_NEG8DBM = RADIO_TXPOWER_TXPOWER_Neg8dBm,
	CRAZET_RADIO_TXPOWER_NEG12DBM = RADIO_TXPOWER_TXPOWER_Neg12dBm,
	CRAZET_RADIO_TXPOWER_NEG16DBM = RADIO_TXPOWER_TXPOWER_Neg16dBm,
	CRAZET_RADIO_TXPOWER_NEG20DBM = RADIO_TXPOWER_TXPOWER_Neg20dBm,
	CRAZET_RADIO_TXPOWER_NEG30DBM = RADIO_TXPOWER_TXPOWER_Neg30dBm,
	CRAZET_RADIO_TXPOWER_NEG40DBM = RADIO_TXPOWER_TXPOWER_Neg40dBm
} CrazetRadioTXPower;

typedef uint8_t CrazetRadioChannel;

typedef enum {
	CRAZET_RADIO_CRC_16BIT = RADIO_CRCCNF_LEN_Two,
	CRAZET_RADIO_CRC_8BIT = RADIO_CRCCNF_LEN_One,
	CRAZET_RADIO_CRC_DISABLED = RADIO_CRCCNF_LEN_Disabled
} CrazetRadioCrc;

typedef enum {
	CRAZET_RADIO_RX_STATE,
	CRAZET_RADIO_TX_STATE
} CrazetRadioState;

typedef void(*RadioIRQEventHandler)(void);
/******************************************************************************/

/******************************************************************************
 * Global RADIO variables
 ******************************************************************************/

extern CrazetRadioState crazetRadioState;

/* Global function pointers for RADIO event handlers. */
extern RadioIRQEventHandler RadioOnDisabledRXEventHandler;
extern RadioIRQEventHandler RadioOnDisabledTXEventHandler;
extern RadioIRQEventHandler RadioOnRSSIEndEventHandler;
/******************************************************************************/

/******************************************************************************
 * RADIO configuration functions
 ******************************************************************************/
#define CRAZET_RADIO_ENABLE()  (CRAZET_RADIO->POWER = 1u)
#define CRAZET_RADIO_DISABLE() (CRAZET_RADIO->POWER = 0u)

#define CRAZET_RADIO_ENABLE_IRQ()  NVIC_DisableIRQ(RADIO_IRQn)
#define CRAZET_RADIO_DISABLE_IRQ() NVIC_EnableIRQ(RADIO_IRQn)

void setCrazetRadioBitRate(CrazetRadioBitrate bitrate);
void setCrazetRadioTXPower(CrazetRadioTXPower txpower);
void setCrazetRadioChannel(CrazetRadioChannel channel);
void setCrazetRadioCRC(CrazetRadioCrc crc);

void configureCrazetRadioAirPacket(uint8_t onAirAddrLen, uint8_t onAirPayloadLen);

void setCrazetRadioBase0Addr(uint32_t base0);
void setCrazetRadioBase1Addr(uint32_t base1);
void setCrazetRadioPrefix0Addr(uint32_t prefix0);
void setCrazetRadioPrefix1Addr(uint32_t prefix1);
/******************************************************************************/

#endif /* CRAZET_RADIO_HW_CT_H_ */
