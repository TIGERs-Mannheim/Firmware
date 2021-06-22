/*
 * rf_queue.h
 *
 *  Created on: 27.09.2017
 *      Author: AndreR
 */

#pragma once

#include "commands.h"
#include "fifo_lin.h"

typedef void(*RFOutgoingCmdFunc)(void* pUserData, uint8_t* pData, uint32_t pktSize);

typedef struct PACKED _RFQueueStats // 22
{
	// LL traffic (RF I/O)
	struct PACKED // 8
	{
		uint16_t txPackets;
		uint16_t txBytes;

		uint16_t rxPackets;
		uint16_t rxBytes;
	} rfIO;

	// LL feedback, packet-based (lost RX due to sequence, tx ACK, rx MaxRT)
	struct PACKED // 2
	{
		uint16_t rxPacketsLost;	// identified by sequence gaps

		// only used by TX nRF24 (= base station)
		uint16_t _deprecated1;
		uint16_t _deprecated2;
	} rfFeedback;

	// HL traffic (Queue I/O), measured at RF processing, NOT at user input
	struct PACKED // 8
	{
		uint16_t txPackets;
		uint16_t txBytes;

		uint16_t rxPackets;
		uint16_t rxBytes;
	} queueIO;

	// HL feedback, packet-based (queues full)
	struct PACKED // 4
	{
		uint16_t txPacketsLost;
		uint16_t rxPacketsLost;
	} queueFeedback;
} RFQueueStats;

typedef struct _RFQueue
{
	uint16_t maxPacketLength;
	uint16_t maxMessageSize;
	uint16_t maxEncodedMsgSize;
	uint16_t rxTxBufSize;

	FifoLin rxFifo;
	FifoLin txFifo;

	uint8_t* rxBuf;
	uint8_t* txBuf;

	uint32_t rxBufUsed;
	uint32_t txBufUsed;

	uint8_t txSeq;
	uint8_t rxSeq;

	RFQueueStats stats;

	RFOutgoingCmdFunc pOutgoindCmdCb;
	void* pUserData;
} RFQueue;

void	RFQueueInit(RFQueue* pQueue, uint8_t* pDataTx, uint16_t txSize, uint8_t* pDataRx, uint16_t rxSize, uint16_t maxPacketLength, uint16_t maxMessageSize);
void	RFQueueSetOutgoingCallback(RFQueue* pQueue, RFOutgoingCmdFunc func, void* pUserData);
int16_t	RFQueueGetPacket(RFQueue* pQueue, uint8_t* pDst, uint32_t dstSize, uint32_t* pPktSize);
int16_t	RFQueueSend(RFQueue* pQueue, const uint8_t* pData, uint32_t dataSize);
void	RFQueueClear(RFQueue* pQueue);
uint32_t RFQueueFetchTxPacket(RFQueue* pQueue, uint8_t* pDataDst);
int16_t RFQueueFeedRxPacket(RFQueue* pQueue, const uint8_t* pData, uint32_t size);
