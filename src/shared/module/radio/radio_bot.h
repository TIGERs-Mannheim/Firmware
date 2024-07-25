#pragma once

#include "radio_module.h"
#include "radio_buf.h"
#include "radio_pkt.h"
#include "util/shell_cmd.h"
#include "log_msgs.h"
#include "ch.h"

#define RADIO_BOT_EVENT_PACKET_RECEIVED		EVENT_MASK(0)
#define RADIO_BOT_EVENT_PACKET_TRANSMITTED	EVENT_MASK(1)

typedef struct _RadioClientRx
{
	RadioBuffer buf;

	uint8_t expectedToggleBit;
	uint32_t rxPacketsLost;	// identified by incorrect toggle bits

	uint32_t lastReceivedTime_us;
	int32_t avgRxRssi_mdBm;

	uint8_t isOnline;
} RadioClientRx;

typedef struct _RadioClientTx
{
	RadioBuffer buf;
} RadioClientTx;

typedef struct _RadioBotStats
{
	volatile uint8_t isBaseOnline;
	volatile uint32_t baseTimeouts;
	volatile uint32_t baseCycleTime_us;
	volatile uint32_t rxInvalidHeader;
} RadioBotStats;

typedef struct _RadioBotData
{
	RadioModule* pModule;
	IRQn_Type lowPrioIRQn;
	uint8_t clientId;
	uint32_t baseTimeout_us;
} RadioBotData;

typedef struct _RadioBot
{
	RadioBotData data;

	RadioClientRx fromBots[RADIO_NUM_ROBOT_CLIENTS];
	RadioClientRx fromBase[RADIO_NUM_TOTAL_CLIENTS];
	RadioClientTx toBase;

	SX1280LLDPacketStatus rxPacketStatus;

	RadioMode mode;
	uint8_t isTxRequested;
	uint8_t headerToggleBit;
	uint32_t lastBaseReceivedTime_us;
	uint32_t lastAddressedTime_us;
	volatile uint32_t newFrequency;

	RadioBotStats stats;

	volatile eventflags_t queuedEvents;
	event_source_t eventSource;

	ShellCmdHandler cmdHandler;
} RadioBot;

void RadioBotInit(RadioBot* pBot, RadioBotData* pInit);
void RadioBotLowPrioIRQ(RadioBot* pRadio);
void RadioBotSetMode(RadioBot* pRadio, RadioMode mode);
void RadioBotSetChannel(RadioBot* pRadio, uint8_t channel);
int16_t RadioBotSetBotId(RadioBot* pRadio, uint8_t botId);
uint8_t RadioBotIsTxEmpty(RadioBot* pRadio);
int16_t RadioBotSendPacket(RadioBot* pRadio, const uint8_t* pData, uint32_t dataSize);
int16_t RadioBotGetMyPacketFromBase(RadioBot* pRadio, uint8_t* pDst, uint32_t dstSize, uint32_t* pPktSize);
int16_t RadioBotGetPacketFromBase(RadioBot* pRadio, uint8_t dstId, uint8_t* pDst, uint32_t dstSize, uint32_t* pPktSize);
int16_t RadioBotGetPacketFromBot(RadioBot* pRadio, uint8_t srcId, uint8_t* pDst, uint32_t dstSize, uint32_t* pPktSize);
void RadioBotGetNetStats(RadioBot* pRadio, NetStats* pStats);
