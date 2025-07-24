#include "radio_bot.h"
#include "util/crc.h"
#include "hal/sys_time.h"
#include "errors.h"
#include "sx1280_def.h"
#include "struct_ids.h"

#include <math.h>

#define NEW_CHANNEL_CHANNEL_MASK 0xFF
#define NEW_CHANNEL_PENDING_BIT 0x100

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
	pRadio->newChannel = (uint32_t)channel | NEW_CHANNEL_PENDING_BIT;
}

void RadioBotStartScan(RadioBot* pRadio, uint8_t lastChannel)
{
	pRadio->lastScanChannel = lastChannel;
	pRadio->startScan = 1;
}

uint8_t RadioBotIsScanComplete(RadioBot* pRadio)
{
	return pRadio->startScan == 0 && pRadio->isScanning == 0;
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

	if(pOp->type == RADIO_PHY_OP_RX)
	{
		if(pRadio->startScan)
		{
			pRadio->isScanning = 1;
			pRadio->startScan = 0;
			RadioBotSetChannel(pRadio, pRadio->activeChannel+1);
			pRadio->stats.isBaseOnline = 0;
			pRadio->isTxRequested = 0;
		}
		else if(pOp->rx.isPacketReceived)
		{
			SX1280LLDParsePacketStatus(&pOp->rx.rxStatus, pRadio->data.pModule->data.packetType, &pRadio->rxPacketStatus);

			if(pRadio->rxPacketStatus.flrc.rx.syncError || pRadio->rxPacketStatus.gfsk.rx.syncError)
			{
				pRadio->stats.rxSyncError++;

				if(pOp->pTrace)
				{
					RadioModuleOpTrace* pTrace = (RadioModuleOpTrace*)pOp->pTrace;
					pTrace->id = 0xFF;
				}
			}
			else if(pRadio->rxPacketStatus.flrc.rx.crcError || pRadio->rxPacketStatus.gfsk.rx.crcError)
			{
				pRadio->stats.rxCrcError++;

				if(pOp->pTrace)
				{
					RadioModuleOpTrace* pTrace = (RadioModuleOpTrace*)pOp->pTrace;
					pTrace->id = 0xFF;
				}
			}
			else
			{
				pRadio->stats.rxPacketsGood++;

				RadioModuleAirPacket* pPacket = (RadioModuleAirPacket*)pOp->rx.data;
				uint8_t clientId = pPacket->header & RADIO_HEADER_CLIENT_ID_MASK;

				if(pPacket->header & RADIO_HEADER_DIR_FROM_BASE)
				{
					// Data from base station
					RadioClientRx* pClient = &pRadio->fromBase[clientId];
					if(pPacket->header & RADIO_HEADER_BROADCAST)
						pClient = &pRadio->fromBase[RADIO_NUM_TOTAL_CLIENTS-1];

					if(pClient->expectedSeq != 0)
						pClient->rxPacketsLost += (uint8_t)(pPacket->seq - pClient->expectedSeq);

					pClient->txPacketsLost += (uint8_t)(pPacket->rxSeqLoss - pClient->lastRxSeqLoss);

					pClient->avgRxRssi_mdBm = (pClient->avgRxRssi_mdBm * 3 + pRadio->rxPacketStatus.rssi_mdBm) / 4;
					pClient->lastReceivedTime_us = SysTimeUSec();
					pClient->isOnline = 1;

					pRadio->stats.lastReceivedRssiFromBaseStation_mdBm = pRadio->rxPacketStatus.rssi_mdBm;

					if(pRadio->isScanning)
					{
						pRadio->queuedEvents |= RADIO_BOT_EVENT_SCAN_COMPLETE;
						NVIC_SetPendingIRQ(pRadio->data.lowPrioIRQn);
					}

					pRadio->lastBaseReceivedTime_us = SysTimeUSec();
					pRadio->stats.isBaseOnline = 1;
					pRadio->isScanning = 0;

					if(RadioBufPutRxAirPacket(&pClient->buf, pPacket->payload, sizeof(pPacket->payload)))
					{
						pRadio->queuedEvents |= RADIO_BOT_EVENT_PACKET_RECEIVED_FROM_BASE;
						NVIC_SetPendingIRQ(pRadio->data.lowPrioIRQn);
					}

					if(clientId == pRadio->data.clientId && (pPacket->header & RADIO_HEADER_BROADCAST) == 0)
					{
						pRadio->isTxRequested = 1;
						uint32_t newCycleTime_us = pOp->pTrace->tReceive_us - pRadio->lastAddressedTime_us;
						pRadio->stats.baseCycleTime_us = (7*pRadio->stats.baseCycleTime_us + newCycleTime_us) / 8;
						pRadio->lastAddressedTime_us = pOp->pTrace->tReceive_us;
					}

					pClient->expectedSeq = pPacket->seq + 1;
					pClient->lastRxSeqLoss = pPacket->rxSeqLoss;
				}
				else
				{
					// Data from other robot
					RadioClientRx* pClient = &pRadio->fromBots[clientId];

					if(pClient->expectedSeq != 0)
						pClient->rxPacketsLost += (uint8_t)(pPacket->seq - pClient->expectedSeq);

					pClient->txPacketsLost += (uint8_t)(pPacket->rxSeqLoss - pClient->lastRxSeqLoss);

					pClient->avgRxRssi_mdBm = (pClient->avgRxRssi_mdBm * 3 + pRadio->rxPacketStatus.rssi_mdBm) / 4;
					pClient->lastReceivedTime_us = SysTimeUSec();
					pClient->isOnline = 1;

					if(RadioBufPutRxAirPacket(&pClient->buf, pPacket->payload, sizeof(pPacket->payload)))
					{
						pRadio->queuedEvents |= RADIO_BOT_EVENT_PACKET_RECEIVED_FROM_BOT;
						NVIC_SetPendingIRQ(pRadio->data.lowPrioIRQn);
					}

					pClient->expectedSeq = pPacket->seq + 1;
					pClient->lastRxSeqLoss = pPacket->rxSeqLoss;
				}

				if(pOp->pTrace)
				{
					RadioModuleOpTrace* pTrace = (RadioModuleOpTrace*)pOp->pTrace;
					pTrace->id = pPacket->header & (RADIO_HEADER_CLIENT_ID_MASK | RADIO_HEADER_DIR_FROM_BASE | RADIO_HEADER_BROADCAST);
				}
			}
		}
		else if(pRadio->isScanning)
		{
			if(pRadio->activeChannel == pRadio->lastScanChannel)
			{
				pRadio->isScanning = 0;
				pRadio->queuedEvents |= RADIO_BOT_EVENT_SCAN_COMPLETE;
				NVIC_SetPendingIRQ(pRadio->data.lowPrioIRQn);
			}
			else
			{
				RadioBotSetChannel(pRadio, pRadio->activeChannel+1);
			}
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
		if(tNow_us - pRadio->fromBase[i].lastReceivedTime_us > pRadio->data.baseTimeout_us || pRadio->isScanning)
			pRadio->fromBase[i].isOnline = 0;
	}

	for(size_t i = 0; i < RADIO_NUM_ROBOT_CLIENTS; i++)
	{
		if(tNow_us - pRadio->fromBots[i].lastReceivedTime_us > pRadio->data.baseTimeout_us || pRadio->isScanning)
			pRadio->fromBots[i].isOnline = 0;
	}
}

static void phyOpPrepareNextClient(RadioPhyOp* pOp, void* pUser)
{
	RadioBot* pRadio = (RadioBot*)pUser;

	RadioModuleOpTrace* pTrace = RadioModuleGetNextTrace(pRadio->data.pModule);
	pTrace->id = pRadio->data.clientId;

	pOp->pTrace = (RadioPhyOpTrace*)pTrace;

	if(pRadio->newChannel && !pRadio->isTxRequested)
	{
		uint32_t newChannel = pRadio->newChannel & NEW_CHANNEL_CHANNEL_MASK;

		pOp->type = RADIO_PHY_OP_CFG;
		pOp->cfg.cmd.cmd = CMD_SET_RFFREQUENCY;
		pOp->cfg.cmd.timeout_us = 10000;
		pOp->cfg.cmd.setRfFrequency.frequency_Hz = (2300UL + newChannel) * 1000000UL;
		pRadio->activeChannel = newChannel;
		pRadio->newChannel = 0;
	}
	else if(pRadio->mode == RADIO_MODE_OFF)
	{
		pOp->type = RADIO_PHY_OP_IDLE;
	}
	else if(pRadio->isTxRequested && pRadio->stats.isBaseOnline && pRadio->mode == RADIO_MODE_ACTIVE)
	{
		pOp->type = RADIO_PHY_OP_TX;
		pOp->tx.useTxDelay = 0;
		if(pRadio->data.pModule->data.packetType == PACKET_TYPE_GFSK)
		{
			pOp->tx.txPeriodBase = PERIOD_TICK_SIZE_15625_NS;
			pOp->tx.txPeriodCount = pRadio->data.pModule->data.settings.gfsk.longPreambleCount;
		}
		else
		{
			pOp->tx.txPeriodBase = PERIOD_TICK_SIZE_1000_US;
			pOp->tx.txPeriodCount = 0;
		}

		pOp->tx.size = sizeof(RadioModuleAirPacket);
		RadioModuleAirPacket* pPacket = (RadioModuleAirPacket*)pOp->tx.data;
		pPacket->header = pRadio->data.clientId;
		pPacket->seq = pRadio->nextSeqNumber;
		pPacket->rxSeqLoss = pRadio->fromBase[pRadio->data.clientId].rxPacketsLost;
		if(RadioBufGetTxAirPacket(&pRadio->toBase.buf, pPacket->payload, sizeof(pPacket->payload)))
		{
			pRadio->queuedEvents |= RADIO_BOT_EVENT_PACKET_TRANSMITTED;
			NVIC_SetPendingIRQ(pRadio->data.lowPrioIRQn);
		}

		pRadio->nextSeqNumber++;
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

	uint32_t totalPacketsLost = 0;
	float baseAvgRssi_dBm = 0.0f;
	size_t numClients = 0;

	for(size_t i = 0; i < RADIO_NUM_TOTAL_CLIENTS; i++)
	{
		totalPacketsLost += pRadio->fromBase[i].rxPacketsLost;

		if(!pRadio->fromBase[i].isOnline)
			continue;

		baseAvgRssi_dBm += pRadio->fromBase[i].avgRxRssi_mdBm * 1e-3f;
		numClients++;
	}

	baseAvgRssi_dBm /= numClients;
	if(numClients == 0)
		baseAvgRssi_dBm = 0;

	printf("Client ID: %hu\r\n", (uint16_t)pRadio->data.clientId);
	printf("Channel: %hu, mode: %hu, scanning: %hu\r\n", (uint16_t)pRadio->activeChannel, (uint16_t)pRadio->mode, (uint16_t)pRadio->isScanning);
	printf("Sync: %hu, losses: %u, cycle time: %uus (%.2fHz)\r\n", (uint16_t)pRadio->stats.isBaseOnline, pRadio->stats.baseTimeouts, pRadio->stats.baseCycleTime_us, 1e6f/pRadio->stats.baseCycleTime_us);
	printf("Good: %u, Sync: %u, CRC: %u, Lost: %u\r\n", pRadio->stats.rxPacketsGood, pRadio->stats.rxSyncError, pRadio->stats.rxCrcError, totalPacketsLost);
	float lossRate = (float)totalPacketsLost / (float)(pRadio->stats.rxPacketsGood + totalPacketsLost);
	printf("Loss Rate: %.2f%%\r\n", lossRate*100.0f);
	printf("Base RSSI: %.2fdBm\r\n", baseAvgRssi_dBm);
	printf("Time since last RX: %uus\r\n", tNow - pRadio->lastBaseReceivedTime_us);

	printf("To base:\r\n");
	printf("    txPTot  txPLos  txPUdr     txBTot\r\n");

	if(pRadio->data.clientId < RADIO_NUM_TOTAL_CLIENTS)
	{
		RadioClientRx* pClientRx = &pRadio->fromBase[pRadio->data.clientId];
		RadioBufferStats* pStatsOut = &pRadio->toBase.buf.stats;

		printf("%10u %7u %7u %10u\r\n", pStatsOut->txPacketsTotal, pClientRx->txPacketsLost, pStatsOut->txBufferUnderrun, pStatsOut->txBytesTotal);
	}

	printf("From base:\r\n");
	printf("ID LastRx       RSSI     rxPTot  rxPLos rxBDisc rxBDOvr rxCDErr rxCRCErr rxBOvr rxBGood rxAppBy\r\n");
	for(size_t i = 0; i < RADIO_NUM_TOTAL_CLIENTS; i++)
	{
		RadioClientRx* pClientRx = &pRadio->fromBase[i];
		if(!pClientRx->isOnline)
			continue;

		RadioBufferStats* pStatsRx = &pClientRx->buf.stats;

		tNow = SysTimeUSec();

		printf("%2u %4ums % 7.2fdBm %10u %7u %7u %7u %7u %7u %7u %7u %7u\r\n", i, (tNow - pClientRx->lastReceivedTime_us)/1000, pClientRx->avgRxRssi_mdBm * 0.001f,
				pStatsRx->rxPacketsTotal, pClientRx->rxPacketsLost, pStatsRx->rxBytesDiscarded, pStatsRx->rxBufferDataOverflows, pStatsRx->rxCobsDecodingErrors,
				pStatsRx->rxCrcErrors, pStatsRx->rxBufferOverruns, pStatsRx->rxBytesGood, pStatsRx->rxAppBytes);
	}

	printf("From bots:\r\n");
	printf("ID LastRx       RSSI     rxPTot  rxPLos rxBDisc rxBDOvr rxCDErr rxCRCErr rxBOvr rxBGood rxAppBy\r\n");
	for(size_t i = 0; i < RADIO_NUM_ROBOT_CLIENTS; i++)
	{
		RadioClientRx* pClient = &pRadio->fromBots[i];
		if(!pClient->isOnline)
			continue;

		RadioBufferStats* pStats = &pClient->buf.stats;

		tNow = SysTimeUSec();

		printf("%2u %4ums % 7.2fdBm %10u %7u %7u %7u %7u %7u %7u %7u %7u\r\n", i, (tNow - pClient->lastReceivedTime_us)/1000, pClient->avgRxRssi_mdBm * 0.001f,
				pStats->rxPacketsTotal, pClient->rxPacketsLost, pStats->rxBytesDiscarded, pStats->rxBufferDataOverflows, pStats->rxCobsDecodingErrors,
				pStats->rxCrcErrors, pStats->rxBufferOverruns, pStats->rxBytesGood, pStats->rxAppBytes);
	}
}

SHELL_CMD(clear, "Clear statistics");

SHELL_CMD_IMPL(clear)
{
	(void)argc; (void)argv;
	RadioBot* pRadio = (RadioBot*)pUser;

	pRadio->stats.rxCrcError = 0;
	pRadio->stats.rxSyncError = 0;
	pRadio->stats.rxPacketsGood = 0;

	for(size_t i = 0; i < RADIO_NUM_TOTAL_CLIENTS; i++)
	{
		pRadio->fromBase[i].rxPacketsLost = 0;
		pRadio->fromBase[i].txPacketsLost = 0;
		pRadio->fromBase[i].expectedSeq = 0;
	}

	for(size_t i = 0; i < RADIO_NUM_ROBOT_CLIENTS; i++)
	{
		pRadio->fromBots[i].rxPacketsLost = 0;
		pRadio->fromBots[i].txPacketsLost = 0;
		pRadio->fromBots[i].expectedSeq = 0;
	}

	printf("Radio statistics cleared\r\n");
}

SHELL_CMD(scan, "Scan for an active base station");

SHELL_CMD_IMPL(scan)
{
	(void)argc; (void)argv;
	RadioBot* pRadio = (RadioBot*)pUser;

	if(RadioBotIsScanComplete(pRadio))
	{
		pRadio->startScan = 1;
		printf("Scan started\r\n");
	}
	else
	{
		fprintf(stderr, "Scan already in progress\r\n");
	}
}

static void registerShellCommands(ShellCmdHandler* pHandler)
{
	ShellCmdAdd(pHandler, stat_command);
	ShellCmdAdd(pHandler, trace_command);
	ShellCmdAdd(pHandler, clear_command);
	ShellCmdAdd(pHandler, scan_command);
}
