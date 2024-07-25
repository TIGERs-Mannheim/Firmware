#pragma once

#include "radio_module.h"
#include "radio_buf.h"
#include "radio_pkt.h"
#include "util/shell_cmd.h"
#include "util/flash_fs.h"
#include "radio_settings.h"
#include "ch.h"

#define RADIO_BASE_EVENT_PREPARE_BROADCAST	EVENT_MASK(0)
#define RADIO_BASE_EVENT_PACKET_RECEIVED	EVENT_MASK(1)

#define RADIO_BASE_MAX_SCHEDULER_ENTRIES (RADIO_NUM_ROBOT_CLIENTS*2 + RADIO_NUM_BROADCAST_TIMESLOTS_MAX)

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

	uint8_t headerToggleBit;
} RadioClientTx;

typedef struct _RadioBaseStats
{
	volatile uint32_t rxInvalidHeader;
	volatile uint32_t rxFromOtherBase;
	volatile uint8_t timeslotsUsed;
	volatile uint32_t cycleDuration_us;
} RadioBaseStats;

typedef struct _RadioBaseTimeslot
{
	uint8_t targetId;
	uint16_t duration_us;
} RadioBaseTimeslot;

typedef struct _RadioBaseCycle
{
	RadioBaseTimeslot slots[RADIO_BASE_MAX_SCHEDULER_ENTRIES];
	RadioBaseTimeslot* pActiveSlot;
	RadioBaseTimeslot* pEndSlot;
} RadioBaseSchedule;

typedef struct _RadioBaseConfig
{
	uint8_t channel;		// = frequency
	uint8_t maxBots;
	uint8_t fixedRuntime;
	uint8_t numBroadcastSlots;
} RadioBaseConfig;

typedef struct _RadioBaseData
{
	RadioModule* pModule;
	IRQn_Type lowPrioIRQn;
	uint32_t clientTimeout_us;
} RadioBaseData;

typedef struct _RadioBase
{
	RadioBaseData data;

	RadioBaseConfig config;
	FlashFile* pCfgFile;

	RadioClientRx baseIn[RADIO_NUM_ROBOT_CLIENTS];
	RadioClientTx baseOut[RADIO_NUM_TOTAL_CLIENTS];

	SX1280LLDPacketStatus rxPacketStatus;

	RadioBaseSchedule schedule[2];
	RadioBaseSchedule* pActiveSchedule;
	RadioBaseSchedule* pOtherSchedule;

	RadioMode mode;
	uint8_t nextIdToCheck;
	uint8_t lastOpWasTx;
	uint8_t lastTxPrepId;
	volatile uint32_t newFrequency;

	RadioBaseStats stats;

	volatile eventflags_t queuedEvents;
	event_source_t eventSource;

	volatile eventflags_t eventFlagsPrepareTx;
	event_source_t eventSourcePrepareTx;

	ShellCmdHandler cmdHandler;
} RadioBase;

void RadioBaseInit(RadioBase* pBase, RadioBaseData* pInit);
void RadioBaseLowPrioIRQ(RadioBase* pBase);

void RadioBaseSetMode(RadioBase* pBase, RadioMode mode);
void RadioBaseSetChannel(RadioBase* pBase, uint8_t channel);
void RadioBaseSetMaxBots(RadioBase* pBase, uint8_t maxBots);
void RadioBaseSetNumBroadcastSlots(RadioBase* pBase, uint8_t numBroadcastSlots);
void RadioBaseSetFixedRuntime(RadioBase* pBase, uint8_t fixed);
void RadioBaseSaveConfig(RadioBase* pBase);

uint8_t RadioBaseGetNumBotsOnline(RadioBase* pBase);
float RadioBaseGetLinkQuality(RadioBase* pBase);

int16_t RadioBaseGetPacket(RadioBase* pRadio, uint8_t srcId, uint8_t* pDst, uint32_t dstSize, uint32_t* pPktSize);
int16_t RadioBaseSendPacket(RadioBase* pRadio, uint8_t dstId, const uint8_t* pData, uint32_t dataSize);
uint8_t RadioBaseIsTxEmpty(RadioBase* pRadio, uint8_t dstId);
