#include "radio_base.h"
#include "hal/sys_time.h"
#include "radio_phy.h"
#include "util/crc.h"
#include "math/map_to_range.h"
#include "errors.h"
#include "sx1280_def.h"

static void phyOpDoneBase(const RadioPhyOp* pOp, void* pUser);
static void phyOpPrepareNextBase(RadioPhyOp* pOp, void* pUser);
static void registerShellCommands(ShellCmdHandler* pHandler);

void RadioBaseLowPrioIRQ(RadioBase* pBase)
{
	chSysLockFromISR();

	if(pBase->queuedEvents)
	{
		chEvtBroadcastFlagsI(&pBase->eventSource, pBase->queuedEvents);
		pBase->queuedEvents = 0;
	}

	if(pBase->eventFlagsPrepareTx)
	{
		chEvtBroadcastFlagsI(&pBase->eventSourcePrepareTx, pBase->eventFlagsPrepareTx);
		pBase->eventFlagsPrepareTx = 0;
	}

	chSysUnlockFromISR();
}

void RadioBaseInit(RadioBase* pBase, RadioBaseData* pInit)
{
	pBase->data = *pInit;

	for(size_t i = 0; i < 2; i++)
	{
		pBase->schedule[i].pEndSlot = pBase->schedule[i].slots;
		pBase->schedule[i].pActiveSlot = pBase->schedule[i].slots;
	}

	pBase->pActiveSchedule = &pBase->schedule[0];
	pBase->pOtherSchedule = &pBase->schedule[1];

	chEvtObjectInit(&pBase->eventSource);
	chEvtObjectInit(&pBase->eventSourcePrepareTx);

	pBase->config.channel = 0;
	pBase->config.fixedRuntime = 0;
	pBase->config.maxBots = 12;
	pBase->config.numBroadcastSlots = 0;

	FlashFSOpenOrCreate("radio/cfg", 0, &pBase->config, sizeof(RadioBaseConfig), &pBase->pCfgFile);

	if(pBase->config.maxBots < 1)
		pBase->config.maxBots = 1;

	for(size_t i = 0; i < RADIO_NUM_ROBOT_CLIENTS; i++)
		RadioBufInit(&pBase->baseIn[i].buf, 1);

	for(size_t i = 0; i < RADIO_NUM_TOTAL_CLIENTS; i++)
		RadioBufInit(&pBase->baseOut[i].buf, 0);

	pBase->data.pModule->data.pPhy->pUser = pBase;
	pBase->data.pModule->data.pPhy->opDoneCallback = &phyOpDoneBase;
	pBase->data.pModule->data.pPhy->opPrepareNextCallback = &phyOpPrepareNextBase;

	RadioBaseSetChannel(pBase, pBase->config.channel);

	ShellCmdHandlerInit(&pBase->cmdHandler, pBase);
	registerShellCommands(&pBase->cmdHandler);
}

void RadioBaseSaveConfig(RadioBase* pBase)
{
	FlashFSWrite(&pBase->pCfgFile, &pBase->config, sizeof(RadioBaseConfig));
}

void RadioBaseSetMode(RadioBase* pBase, RadioMode mode)
{
	pBase->mode = mode;
}

void RadioBaseSetChannel(RadioBase* pBase, uint8_t channel)
{
	uint32_t freq = channel;
	pBase->newFrequency = (freq + 2300)*1000000UL;

	pBase->config.channel = channel;
}

void RadioBaseSetMaxBots(RadioBase* pBase, uint8_t maxBots)
{
	if(maxBots < 1)
		maxBots = 1;
	else if(maxBots > RADIO_NUM_ROBOT_CLIENTS)
		maxBots = RADIO_NUM_ROBOT_CLIENTS;

	pBase->config.maxBots = maxBots;
}

void RadioBaseSetNumBroadcastSlots(RadioBase* pBase, uint8_t numBroadcastSlots)
{
	if(numBroadcastSlots > RADIO_NUM_BROADCAST_TIMESLOTS_MAX)
		numBroadcastSlots = RADIO_NUM_BROADCAST_TIMESLOTS_MAX;

	pBase->config.numBroadcastSlots = numBroadcastSlots;
}

void RadioBaseSetFixedRuntime(RadioBase* pBase, uint8_t fixed)
{
	pBase->config.fixedRuntime = fixed;
}

uint8_t RadioBaseGetNumBotsOnline(RadioBase* pBase)
{
	uint8_t bots = 0;

	for(uint8_t i = 0; i < RADIO_NUM_ROBOT_CLIENTS; i++)
	{
		if(pBase->baseIn[i].isOnline)
			bots++;
	}

	return bots;
}

float RadioBaseGetLinkQuality(RadioBase* pBase)
{
	uint8_t bots = 0;
	float link = 0.0f;

	for(uint8_t i = 0; i < RADIO_NUM_ROBOT_CLIENTS; i++)
	{
		if(pBase->baseIn[i].isOnline)
		{
			link += pBase->baseIn[i].avgRxRssi_mdBm * 0.001f;
			bots++;
		}
	}

	if(bots == 0)
		return 0.0f;

	float dbm = link/(float)bots;

	return MapToRangef32(-90.0f, -12.0f, 0.0f, 99.9f, dbm);
}

int16_t RadioBaseGetPacket(RadioBase* pRadio, uint8_t srcId, uint8_t* pDst, uint32_t dstSize, uint32_t* pPktSize)
{
	if(srcId >= RADIO_NUM_ROBOT_CLIENTS)
		return ERROR_INVALID_PARAMETER;

	*pPktSize = RadioBufGetNextAppPacket(&pRadio->baseIn[srcId].buf, pDst, dstSize);
	if(*pPktSize == 0)
		return ERROR_FIFO_EMPTY;

	return 0;
}

int16_t RadioBaseSendPacket(RadioBase* pRadio, uint8_t dstId, const uint8_t* pData, uint32_t dataSize)
{
	if(dstId >= RADIO_NUM_TOTAL_CLIENTS)
		return ERROR_INVALID_PARAMETER;

	if(RadioBufEnqueueAppPacket(&pRadio->baseOut[dstId].buf, pData, dataSize) == 0)
		return ERROR_FIFO_FULL;

	return 0;
}

uint8_t RadioBaseIsTxEmpty(RadioBase* pRadio, uint8_t dstId)
{
	if(dstId >= RADIO_NUM_TOTAL_CLIENTS)
		return 1;

	return RadioBufferIsEmpty(&pRadio->baseOut[dstId].buf);
}

static void rescheduleBaseSequence(RadioBase* pBase)
{
	RadioBaseSchedule* pOtherSchedule = pBase->pOtherSchedule;

	// Gather timing statistics
	uint8_t slotsUsedLastCycle = 0;
	uint32_t lastCycleDurationSum_us = 0;
	for(RadioBaseTimeslot* pSlot = pOtherSchedule->slots; pSlot < pOtherSchedule->pEndSlot; pSlot++)
	{
		slotsUsedLastCycle++;
		lastCycleDurationSum_us += pSlot->duration_us;
	}

	pBase->stats.timeslotsUsed = slotsUsedLastCycle;
	pBase->stats.cycleDuration_us = lastCycleDurationSum_us;

	// Figure out schedule for next cycle
	uint8_t robotsUsed = 0;

	for(uint8_t i = 0; i < RADIO_NUM_ROBOT_CLIENTS; i++)
	{
		RadioClientRx* pClient = &pBase->baseIn[i];

		// Update online state of client based on last reception time
		uint32_t tNow_us = SysTimeUSec();
		uint32_t timeSinceLastReceived = tNow_us - pClient->lastReceivedTime_us;

		if(pClient->isOnline && timeSinceLastReceived > pBase->data.clientTimeout_us)
			pClient->isOnline = 0;

		if(!pClient->isOnline)
		{
			if(timeSinceLastReceived < pBase->data.clientTimeout_us)
				pClient->isOnline = 1;
			else
				pClient->lastReceivedTime_us = tNow_us - pBase->data.clientTimeout_us;
		}

		// Add all online clients to schedule
		if(pClient->isOnline && robotsUsed < pBase->config.maxBots)
		{
			pOtherSchedule->slots[robotsUsed*2].targetId = i | RADIO_HEADER_DIR_FROM_BASE;
			pOtherSchedule->slots[robotsUsed*2+1].targetId = i;
			robotsUsed++;
		}
	}

	// Check additional client(s) which is not online to check if it is online now
	while(robotsUsed < pBase->config.maxBots)
	{
		for(size_t i = 0; i < RADIO_NUM_ROBOT_CLIENTS; i++)
		{
			if(!pBase->baseIn[pBase->nextIdToCheck].isOnline)
				break;

			++pBase->nextIdToCheck;
			pBase->nextIdToCheck %= RADIO_NUM_ROBOT_CLIENTS;
		}

		pOtherSchedule->slots[robotsUsed*2].targetId = pBase->nextIdToCheck | RADIO_HEADER_DIR_FROM_BASE;
		pOtherSchedule->slots[robotsUsed*2+1].targetId = pBase->nextIdToCheck;
		robotsUsed++;
		++pBase->nextIdToCheck;
		pBase->nextIdToCheck %= RADIO_NUM_ROBOT_CLIENTS;

		if(!pBase->config.fixedRuntime)
			break;
	}

	// Put in specified number of broadcast slots
	for(uint8_t i = 0; i < pBase->config.numBroadcastSlots; i++)
		pOtherSchedule->slots[robotsUsed*2 + i].targetId = RADIO_HEADER_BROADCAST | RADIO_HEADER_DIR_FROM_BASE;

	pOtherSchedule->pActiveSlot = pOtherSchedule->slots;
	pOtherSchedule->pEndSlot = pOtherSchedule->slots +  + robotsUsed*2 + pBase->config.numBroadcastSlots;
}

static void phyOpDoneBase(const RadioPhyOp* pOp, void* pUser)
{
	RadioBase* pBase = (RadioBase*)pUser;

	if(pOp->type == RADIO_PHY_OP_RX && pOp->rx.isPacketReceived)
	{
		SX1280LLDParsePacketStatus(&pOp->rx.rxStatus, pBase->data.pModule->data.packetType, &pBase->rxPacketStatus);

		RadioModuleAirPacket* pPacket = (RadioModuleAirPacket*)pOp->rx.data;
		if(pPacket->header == (pPacket->invHeader ^ 0xFF))
		{
			if(pPacket->header & RADIO_HEADER_DIR_FROM_BASE)
			{
				pBase->stats.rxFromOtherBase++;
			}
			else
			{
				uint8_t clientId = pPacket->header & RADIO_HEADER_CLIENT_ID_MASK;

				RadioClientRx* pClient = &pBase->baseIn[clientId];

				if(pClient->expectedToggleBit != (pPacket->header & RADIO_HEADER_SEQ_TOGGLE))
					pClient->rxPacketsLost++;

				pClient->lastReceivedTime_us = SysTimeUSec();
				pClient->avgRxRssi_mdBm = (pClient->avgRxRssi_mdBm * 3 + pBase->rxPacketStatus.rssi_mdBm) >> 2;

				if(RadioBufPutRxAirPacket(&pClient->buf, pPacket->payload, sizeof(pPacket->payload)))
				{
					pBase->queuedEvents |= RADIO_BASE_EVENT_PACKET_RECEIVED;
					NVIC_SetPendingIRQ(pBase->data.lowPrioIRQn);
				}

				pClient->expectedToggleBit = (pPacket->header & RADIO_HEADER_SEQ_TOGGLE) ^ RADIO_HEADER_SEQ_TOGGLE;
			}
		}
		else
		{
			pBase->stats.rxInvalidHeader++;
		}
	}

	if(pOp->type == RADIO_PHY_OP_RX || pOp->type == RADIO_PHY_OP_TX)
	{
		pBase->pActiveSchedule->pActiveSlot->duration_us = pOp->pTrace->tEnd_us - pOp->pTrace->tStart_us;
		pBase->pActiveSchedule->pActiveSlot++;
	}
}

static void phyOpPrepareNextBase(RadioPhyOp* pOp, void* pUser)
{
	RadioBase* pBase = (RadioBase*)pUser;

	size_t slotsDone = pBase->pActiveSchedule->pActiveSlot - pBase->pActiveSchedule->slots;
	size_t totalSlots = pBase->pActiveSchedule->pEndSlot - pBase->pActiveSchedule->slots;

	// Half of the schedule done check
	if(slotsDone == totalSlots/2)
		rescheduleBaseSequence(pBase);

	// Full schedule done check
	if(slotsDone == totalSlots)
	{
		RadioBaseSchedule* pTmp = pBase->pActiveSchedule;
		pBase->pActiveSchedule = pBase->pOtherSchedule;
		pBase->pOtherSchedule = pTmp;
	}

	RadioBaseTimeslot* pActiveSlot = pBase->pActiveSchedule->pActiveSlot;
	RadioBaseTimeslot* pNextSlot = pBase->pActiveSchedule->pActiveSlot + 1;
	if(pNextSlot == pBase->pActiveSchedule->pEndSlot)
		pNextSlot = pBase->pOtherSchedule->slots;

	if(pNextSlot->targetId & RADIO_HEADER_DIR_FROM_BASE)
	{
		uint8_t dstId = pNextSlot->targetId & (RADIO_HEADER_CLIENT_ID_MASK | RADIO_HEADER_BROADCAST);
		if(dstId != pBase->lastTxPrepId)
		{
			if(pNextSlot->targetId & RADIO_HEADER_BROADCAST)
			{
				pBase->queuedEvents |= RADIO_BASE_EVENT_PREPARE_BROADCAST;
				NVIC_SetPendingIRQ(pBase->data.lowPrioIRQn);
			}
			else
			{
				pBase->eventFlagsPrepareTx |= (1 << dstId);
				NVIC_SetPendingIRQ(pBase->data.lowPrioIRQn);
			}

			pBase->lastTxPrepId = dstId;
		}
	}

	// Figure out next slot
	uint8_t nextId = pActiveSlot->targetId;

	RadioModuleOpTrace* pTrace = RadioModuleGetNextTrace(pBase->data.pModule);
	pTrace->id = nextId;

	pOp->pTrace = (RadioPhyOpTrace*)pTrace;

	if(pBase->newFrequency)
	{
		pOp->type = RADIO_PHY_OP_CFG;
		pOp->cfg.cmd.cmd = CMD_SET_RFFREQUENCY;
		pOp->cfg.cmd.setRfFrequency.frequency_Hz = pBase->newFrequency;
		pBase->newFrequency = 0;
		pBase->lastOpWasTx = 0;
	}
	else if(pBase->mode == RADIO_MODE_OFF)
	{
		pOp->type = RADIO_PHY_OP_IDLE;
		pBase->lastOpWasTx = 0;
	}
	else if((nextId & RADIO_HEADER_DIR_FROM_BASE) && pBase->mode == RADIO_MODE_ACTIVE)
	{
		// Base TX slot
		uint8_t clientId = nextId & (RADIO_HEADER_CLIENT_ID_MASK | RADIO_HEADER_BROADCAST);
		RadioClientTx* pClient = &pBase->baseOut[clientId];

		pOp->type = RADIO_PHY_OP_TX;
		pOp->tx.size = RADIO_AIR_PACKET_SIZE;
		pOp->tx.useTxDelay = pBase->lastOpWasTx;

		RadioModuleAirPacket* pPacket = (RadioModuleAirPacket*)pOp->tx.data;
		pPacket->header = nextId | pClient->headerToggleBit;
		pPacket->invHeader = pPacket->header ^ 0xFF;
		RadioBufGetTxAirPacket(&pClient->buf, pPacket->payload, sizeof(pPacket->payload));

		pClient->headerToggleBit ^= RADIO_HEADER_SEQ_TOGGLE;
		pBase->lastOpWasTx = 1;
	}
	else
	{
		// Base RX slot
		pOp->type = RADIO_PHY_OP_RX;
		pOp->rx.size = RADIO_AIR_PACKET_SIZE;

		if(pBase->mode == RADIO_MODE_SNIFFER)
			pOp->rx.isLongWait = 1;
		else
			pOp->rx.isLongWait = 0;

		pBase->lastOpWasTx = 0;
	}
}

SHELL_CMD(trace, "Print radio driver trace buffer");

SHELL_CMD_IMPL(trace)
{
	(void)argc; (void)argv;
	RadioBase* pRadio = (RadioBase*)pUser;

	RadioModulePrintTrace(pRadio->data.pModule);
}

SHELL_CMD(stat, "Print radio stats");

SHELL_CMD_IMPL(stat)
{
	(void)argc; (void)argv;
	RadioBase* pRadio = (RadioBase*)pUser;

	uint32_t tNow = SysTimeUSec();

	printf("Timeslots used:     %hu (%uus, %.2fHz)\r\n", (uint16_t)pRadio->stats.timeslotsUsed, pRadio->stats.cycleDuration_us, 1e6f/pRadio->stats.cycleDuration_us);
	printf("RX invalid header:  %u\r\n", pRadio->stats.rxInvalidHeader);
	printf("RX from other base: %u\r\n", pRadio->stats.rxFromOtherBase);

	printf("From bots:\r\n");
	printf("ID LastRx       RSSI     rxPTot  rxPLos rxBDisc rxBDOvr rxCDErr  rxBOvr rxBGood rxAppBy\r\n");
	for(size_t i = 0; i < RADIO_NUM_ROBOT_CLIENTS; i++)
	{
		RadioClientRx* pClient = &pRadio->baseIn[i];
		if(!pClient->isOnline)
			continue;

		RadioBufferStats* pStats = &pClient->buf.stats;

		tNow = SysTimeUSec();

		printf("%2u %4ums % 7.2fdBm %10u %7u %7u %7u %7u %7u %7u %7u\r\n", i, (tNow - pClient->lastReceivedTime_us)/1000, pClient->avgRxRssi_mdBm * 0.001f,
				pStats->rxPacketsTotal, pClient->rxPacketsLost, pStats->rxBytesDiscarded, pStats->rxBufferDataOverflows, pStats->rxCobsDecodingErrors,
				pStats->rxBufferOverruns, pStats->rxBytesGood, pStats->rxAppBytes);
	}
}

SHELL_CMD(config, "Show radio configuration");

SHELL_CMD_IMPL(config)
{
	(void)argc; (void)argv;
	RadioBase* pRadio = (RadioBase*)pUser;

	printf("--- Radio Configuration ---\r\n");
	printf("Channel:    %hu (2.%03huGHz)\r\n", (uint16_t)pRadio->config.channel, ((uint16_t)pRadio->config.channel)+300);
	printf("Max. Bots:  %hu\r\n", (uint16_t)pRadio->config.maxBots);
	printf("Runtime:    %s\r\n", pRadio->config.fixedRuntime ? "fixed" : "variable");
	printf("Broadcasts: %hu\r\n", pRadio->config.numBroadcastSlots);
}

SHELL_CMD(ch, "Set channel",
	SHELL_ARG(channel, "New channel (0-255)")
);

SHELL_CMD_IMPL(ch)
{
	(void)argc;
	RadioBase* pRadio = (RadioBase*)pUser;

	int channel = atoi(argv[1]);

	if(channel < 0 || channel > 255)
	{
		fprintf(stderr, "Invalid channel");
		return;
	}

	printf("New channel: %d\r\n", channel);

	RadioBaseSetChannel(pRadio, channel);
	RadioBaseSaveConfig(pRadio);
}

SHELL_CMD(bots, "Set maximum number of bots",
	SHELL_ARG(max, "Max. bots (1-32)")
);

SHELL_CMD_IMPL(bots)
{
	(void)argc;
	RadioBase* pRadio = (RadioBase*)pUser;

	int bots = atoi(argv[1]);

	if(bots < 1 || bots > 32)
	{
		fprintf(stderr, "Invalid number of maximum bots");
		return;
	}

	printf("New bot limit: %d\r\n", bots);

	RadioBaseSetMaxBots(pRadio, bots);
	RadioBaseSaveConfig(pRadio);
}

SHELL_CMD(bc, "Set number of broadcast slots",
	SHELL_ARG(slots, "Number of broadcast slots (0-8)")
);

SHELL_CMD_IMPL(bc)
{
	(void)argc;
	RadioBase* pRadio = (RadioBase*)pUser;

	int slots = atoi(argv[1]);

	if(slots < 0 || slots > 8)
	{
		fprintf(stderr, "Invalid number of broadcast slots");
		return;
	}

	printf("New broadcast slots: %d\r\n", slots);

	RadioBaseSetNumBroadcastSlots(pRadio, slots);
	RadioBaseSaveConfig(pRadio);
}

SHELL_CMD(rt, "Select fixed or variable runtime mode",
	SHELL_ARG(mode, "fixed or variable")
);

SHELL_CMD_IMPL(rt)
{
	(void)argc;
	RadioBase* pRadio = (RadioBase*)pUser;

	if(*argv[1] == 'f')
	{
		printf("Selecting fixed runtime\r\n");
		RadioBaseSetFixedRuntime(pRadio, 1);
		RadioBaseSaveConfig(pRadio);
	}
	else if(*argv[1] == 'v')
	{
		printf("Selecting variable runtime\r\n");
		RadioBaseSetFixedRuntime(pRadio, 0);
		RadioBaseSaveConfig(pRadio);
	}
	else
	{
		fprintf(stderr, "Unknown runtime mode");
	}
}

static void registerShellCommands(ShellCmdHandler* pHandler)
{
	ShellCmdAdd(pHandler, stat_command);
	ShellCmdAdd(pHandler, trace_command);
	ShellCmdAdd(pHandler, config_command);
	ShellCmdAdd(pHandler, ch_command);
	ShellCmdAdd(pHandler, bots_command);
	ShellCmdAdd(pHandler, bc_command);
	ShellCmdAdd(pHandler, rt_command);
}
