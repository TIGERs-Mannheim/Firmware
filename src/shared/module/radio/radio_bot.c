#include "radio_bot.h"
#include "util/crc.h"
#include "hal/sys_time.h"
#include "errors.h"
#include "sx1280_def.h"
#include "struct_ids.h"

static void phyOpDoneClient(const RadioPhyOp* pOp, void* pUser);
static void phyOpPrepareNextClient(RadioPhyOp* pOp, void* pUser);
static void registerShellCommands(ShellCmdHandler* pHandler);

void RadioBotLowPrioIRQ(RadioBot* pRadio)
{
	chSysLockFromISR();
	chEvtBroadcastFlagsI(&pRadio->eventSource, pRadio->queuedEvents);
	pRadio->queuedEvents = 0;
	chSysUnlockFromISR();
}

void RadioBotInit(RadioBot* pRadio, RadioBotData* pInit)
{
	pRadio->data = *pInit;

	chEvtObjectInit(&pRadio->eventSource);

	for(size_t i = 0; i < RADIO_NUM_ROBOT_CLIENTS; i++)
		RadioBufInit(&pRadio->fromBots[i].buf, 1);

	for(size_t i = 0; i < RADIO_NUM_TOTAL_CLIENTS; i++)
		RadioBufInit(&pRadio->fromBase[i].buf, 1);

	RadioBufInit(&pRadio->toBase.buf, 0);

	pRadio->data.pModule->data.pPhy->pUser = pRadio;
	pRadio->data.pModule->data.pPhy->opDoneCallback = &phyOpDoneClient;
	pRadio->data.pModule->data.pPhy->opPrepareNextCallback = &phyOpPrepareNextClient;

	ShellCmdHandlerInit(&pRadio->cmdHandler, pRadio);
	registerShellCommands(&pRadio->cmdHandler);
}

void RadioBotGetNetStats(RadioBot* pRadio, NetStats* pStats)
{
	const RadioClientRx* pRx = &pRadio->fromBase[pRadio->data.clientId];
	const RadioClientTx* pTx = &pRadio->toBase;

	pStats->header.type = SID_NETSTATS;
	pStats->header.length = sizeof(NetStats);
	pStats->header.timestamp = SysTimeUSec();
	pStats->rxPeriod = 1e6f/pRadio->stats.baseCycleTime_us;
	pStats->rssi = pRadio->fromBase[pRadio->data.clientId].avgRxRssi_mdBm * 0.001f;
	pStats->rxPackets = pRx->buf.stats.rxPacketsTotal;
	pStats->rxBytes = pRx->buf.stats.rxBytesGood;
	pStats->txPackets = pTx->buf.stats.txPacketsTotal;
	pStats->txBytes = pTx->buf.stats.txBytesTotal;
	pStats->rxPacketsLost = pRx->rxPacketsLost;
	pStats->rxHLPacketsLost = pRx->buf.stats.rxCobsDecodingErrors + pRx->buf.stats.rxCrcErrors;
	pStats->txHLPacketsLost = pTx->buf.stats.txBufferUnderrun;
}

void RadioBotSetMode(RadioBot* pRadio, RadioMode mode)
{
	pRadio->mode = mode;
}

void RadioBotSetChannel(RadioBot* pRadio, uint8_t channel)
{
	uint32_t freq = channel;
	pRadio->newFrequency = (freq + 2300)*1000000UL;
}

int16_t RadioBotSetBotId(RadioBot* pRadio, uint8_t botId)
{
	if(botId >= RADIO_NUM_ROBOT_CLIENTS)
		return ERROR_INVALID_PARAMETER;

	pRadio->data.clientId = botId;
	return 0;
}

uint8_t RadioBotIsTxEmpty(RadioBot* pRadio)
{
	return RadioBufferIsEmpty(&pRadio->toBase.buf);
}

int16_t RadioBotGetPacketFromBase(RadioBot* pRadio, uint8_t dstId, uint8_t* pDst, uint32_t dstSize, uint32_t* pPktSize)
{
	if(dstId >= RADIO_NUM_TOTAL_CLIENTS)
		return ERROR_INVALID_PARAMETER;

	*pPktSize = RadioBufGetNextAppPacket(&pRadio->fromBase[dstId].buf, pDst, dstSize);
	if(*pPktSize == 0)
		return ERROR_FIFO_EMPTY;

	return 0;
}

int16_t RadioBotGetPacketFromBot(RadioBot* pRadio, uint8_t srcId, uint8_t* pDst, uint32_t dstSize, uint32_t* pPktSize)
{
	if(srcId >= RADIO_NUM_ROBOT_CLIENTS)
		return ERROR_INVALID_PARAMETER;

	*pPktSize = RadioBufGetNextAppPacket(&pRadio->fromBots[srcId].buf, pDst, dstSize);
	if(*pPktSize == 0)
		return ERROR_FIFO_EMPTY;

	return 0;
}

int16_t RadioBotGetMyPacketFromBase(RadioBot* pRadio, uint8_t* pDst, uint32_t dstSize, uint32_t* pPktSize)
{
	return RadioBotGetPacketFromBase(pRadio, pRadio->data.clientId, pDst, dstSize, pPktSize);
}

int16_t RadioBotSendPacket(RadioBot* pRadio, const uint8_t* pData, uint32_t dataSize)
{
	if(RadioBufEnqueueAppPacket(&pRadio->toBase.buf, pData, dataSize) == 0)
		return ERROR_FIFO_FULL;

	return 0;
}

static void phyOpDoneClient(const RadioPhyOp* pOp, void* pUser)
{
	RadioBot* pRadio = (RadioBot*)pUser;

	if(pOp->type == RADIO_PHY_OP_RX && pOp->rx.isPacketReceived)
	{
		SX1280LLDParsePacketStatus(&pOp->rx.rxStatus, pRadio->data.pModule->data.packetType, &pRadio->rxPacketStatus);

		RadioModuleAirPacket* pPacket = (RadioModuleAirPacket*)pOp->rx.data;
		if(pPacket->header == (pPacket->invHeader ^ 0xFF))
		{
			uint8_t clientId = pPacket->header & RADIO_HEADER_CLIENT_ID_MASK;

			if(pPacket->header & RADIO_HEADER_DIR_FROM_BASE)
			{
				// Data from base station
				RadioClientRx* pClient = &pRadio->fromBase[clientId];
				if(pPacket->header & RADIO_HEADER_BROADCAST)
					pClient = &pRadio->fromBase[RADIO_NUM_TOTAL_CLIENTS-1];

				pClient->avgRxRssi_mdBm = (pClient->avgRxRssi_mdBm * 3 + pRadio->rxPacketStatus.rssi_mdBm) >> 2;
				pClient->lastReceivedTime_us = SysTimeUSec();
				pClient->isOnline = 1;

				if(pClient->expectedToggleBit != (pPacket->header & RADIO_HEADER_SEQ_TOGGLE))
					pClient->rxPacketsLost++;

				pRadio->lastBaseReceivedTime_us = SysTimeUSec();
				pRadio->stats.isBaseOnline = 1;

				if(RadioBufPutRxAirPacket(&pClient->buf, pPacket->payload, sizeof(pPacket->payload)))
				{
					pRadio->queuedEvents |= RADIO_BOT_EVENT_PACKET_RECEIVED;
					NVIC_SetPendingIRQ(pRadio->data.lowPrioIRQn);
				}

				if(clientId == pRadio->data.clientId && (pPacket->header & RADIO_HEADER_BROADCAST) == 0)
				{
					pRadio->isTxRequested = 1;
					pRadio->stats.baseCycleTime_us = pOp->pTrace->tReceive_us - pRadio->lastAddressedTime_us;
					pRadio->lastAddressedTime_us = pOp->pTrace->tReceive_us;
				}

				pClient->expectedToggleBit = (pPacket->header & RADIO_HEADER_SEQ_TOGGLE) ^ RADIO_HEADER_SEQ_TOGGLE;
			}
			else
			{
				RadioClientRx* pClient = &pRadio->fromBots[clientId];
				pClient->avgRxRssi_mdBm = (pClient->avgRxRssi_mdBm * 3 + pRadio->rxPacketStatus.rssi_mdBm) >> 2;
				pClient->lastReceivedTime_us = SysTimeUSec();
				pClient->isOnline = 1;

				if(pClient->expectedToggleBit != (pPacket->header & RADIO_HEADER_SEQ_TOGGLE))
					pClient->rxPacketsLost++;

				if(RadioBufPutRxAirPacket(&pClient->buf, pPacket->payload, sizeof(pPacket->payload)))
				{
					pRadio->queuedEvents |= RADIO_BOT_EVENT_PACKET_RECEIVED;
					NVIC_SetPendingIRQ(pRadio->data.lowPrioIRQn);
				}

				pClient->expectedToggleBit = (pPacket->header & RADIO_HEADER_SEQ_TOGGLE) ^ RADIO_HEADER_SEQ_TOGGLE;
			}

			if(pOp->pTrace)
			{
				RadioModuleOpTrace* pTrace = (RadioModuleOpTrace*)pOp->pTrace;
				pTrace->id = pPacket->header & (RADIO_HEADER_CLIENT_ID_MASK | RADIO_HEADER_DIR_FROM_BASE | RADIO_HEADER_BROADCAST);
			}
		}
		else
		{
			pRadio->stats.rxInvalidHeader++;
		}
	}

	uint32_t tNow_us = SysTimeUSec();

	if(pRadio->stats.isBaseOnline && (tNow_us - pRadio->lastBaseReceivedTime_us) > pRadio->data.baseTimeout_us)
	{
		pRadio->stats.isBaseOnline = 0;
		pRadio->stats.baseTimeouts++;
	}

	for(size_t i = 0; i < RADIO_NUM_TOTAL_CLIENTS; i++)
	{
		if(tNow_us - pRadio->fromBase[i].lastReceivedTime_us > pRadio->data.baseTimeout_us)
			pRadio->fromBase[i].isOnline = 0;
	}

	for(size_t i = 0; i < RADIO_NUM_ROBOT_CLIENTS; i++)
	{
		if(tNow_us - pRadio->fromBots[i].lastReceivedTime_us > pRadio->data.baseTimeout_us)
			pRadio->fromBots[i].isOnline = 0;
	}
}

static void phyOpPrepareNextClient(RadioPhyOp* pOp, void* pUser)
{
	RadioBot* pRadio = (RadioBot*)pUser;

	RadioModuleOpTrace* pTrace = RadioModuleGetNextTrace(pRadio->data.pModule);
	pTrace->id = pRadio->data.clientId;

	pOp->pTrace = (RadioPhyOpTrace*)pTrace;

	if(pRadio->newFrequency && !pRadio->isTxRequested)
	{
		pOp->type = RADIO_PHY_OP_CFG;
		pOp->cfg.cmd.cmd = CMD_SET_RFFREQUENCY;
		pOp->cfg.cmd.setRfFrequency.frequency_Hz = pRadio->newFrequency;
		pRadio->newFrequency = 0;
	}
	else if(pRadio->mode == RADIO_MODE_OFF)
	{
		pOp->type = RADIO_PHY_OP_IDLE;
	}
	else if(pRadio->isTxRequested && pRadio->stats.isBaseOnline && pRadio->mode == RADIO_MODE_ACTIVE)
	{
		pOp->type = RADIO_PHY_OP_TX;
		pOp->tx.useTxDelay = 0;
		pOp->tx.size = sizeof(RadioModuleAirPacket);
		RadioModuleAirPacket* pPacket = (RadioModuleAirPacket*)pOp->tx.data;
		pPacket->header = pRadio->data.clientId | pRadio->headerToggleBit;
		pPacket->invHeader = pPacket->header ^ 0xFF;
		if(RadioBufGetTxAirPacket(&pRadio->toBase.buf, pPacket->payload, sizeof(pPacket->payload)))
		{
			pRadio->queuedEvents |= RADIO_BOT_EVENT_PACKET_TRANSMITTED;
			NVIC_SetPendingIRQ(pRadio->data.lowPrioIRQn);
		}

		pRadio->headerToggleBit ^= RADIO_HEADER_SEQ_TOGGLE;
	}
	else
	{
		pOp->type = RADIO_PHY_OP_RX;
		pOp->rx.size = RADIO_AIR_PACKET_SIZE;
		pOp->rx.isLongWait = 1;
	}

	pRadio->isTxRequested = 0;
}

SHELL_CMD(trace, "Print radio driver trace buffer");

SHELL_CMD_IMPL(trace)
{
	(void)argc; (void)argv;
	RadioBot* pRadio = (RadioBot*)pUser;

	RadioModulePrintTrace(pRadio->data.pModule);
}

SHELL_CMD(stat, "Print radio stats");

SHELL_CMD_IMPL(stat)
{
	(void)argc; (void)argv;
	RadioBot* pRadio = (RadioBot*)pUser;

	uint32_t tNow = SysTimeUSec();

	printf("Client ID: %hu\r\n", (uint16_t)pRadio->data.clientId);
	printf("Sync: %hu, losses: %u, cycle time: %uus (%.2fHz)\r\n", (uint16_t)pRadio->stats.isBaseOnline, pRadio->stats.baseTimeouts, pRadio->stats.baseCycleTime_us, 1e6f/pRadio->stats.baseCycleTime_us);
	printf("RX invalid header:  %u\r\n", pRadio->stats.rxInvalidHeader);
	printf("Time since last RX: %uus\r\n", tNow - pRadio->lastBaseReceivedTime_us);

	printf("From base:\r\n");
	printf("ID LastRx       RSSI     rxPTot  rxPLos rxBDisc rxBDOvr rxCDErr  rxBOvr rxBGood rxAppBy\r\n");
	for(size_t i = 0; i < RADIO_NUM_TOTAL_CLIENTS; i++)
	{
		RadioClientRx* pClient = &pRadio->fromBase[i];
		if(!pClient->isOnline)
			continue;

		RadioBufferStats* pStats = &pClient->buf.stats;

		tNow = SysTimeUSec();

		printf("%2u %4ums % 7.2fdBm %10u %7u %7u %7u %7u %7u %7u %7u\r\n", i, (tNow - pClient->lastReceivedTime_us)/1000, pClient->avgRxRssi_mdBm * 0.001f,
				pStats->rxPacketsTotal, pClient->rxPacketsLost, pStats->rxBytesDiscarded, pStats->rxBufferDataOverflows, pStats->rxCobsDecodingErrors,
				pStats->rxBufferOverruns, pStats->rxBytesGood, pStats->rxAppBytes);
	}

	printf("From bots:\r\n");
	printf("ID LastRx       RSSI     rxPTot  rxPLos rxBDisc rxBDOvr rxCDErr  rxBOvr rxBGood rxAppBy\r\n");
	for(size_t i = 0; i < RADIO_NUM_ROBOT_CLIENTS; i++)
	{
		RadioClientRx* pClient = &pRadio->fromBots[i];
		if(!pClient->isOnline)
			continue;

		RadioBufferStats* pStats = &pClient->buf.stats;

		tNow = SysTimeUSec();

		printf("%2u %4ums % 7.2fdBm %10u %7u %7u %7u %7u %7u %7u %7u\r\n", i, (tNow - pClient->lastReceivedTime_us)/1000, pClient->avgRxRssi_mdBm * 0.001f,
				pStats->rxPacketsTotal, pClient->rxPacketsLost, pStats->rxBytesDiscarded, pStats->rxBufferDataOverflows, pStats->rxCobsDecodingErrors,
				pStats->rxBufferOverruns, pStats->rxBytesGood, pStats->rxAppBytes);
	}
}

static void registerShellCommands(ShellCmdHandler* pHandler)
{
	ShellCmdAdd(pHandler, stat_command);
	ShellCmdAdd(pHandler, trace_command);
}
