/*
 * ext.h
 *
 *  Created on: 25.06.2019
 *      Author: AndreR
 */

#pragma once

#include "util/uart_fifo.h"
#include "util/fifo_lin.h"
#include "util/cobs.h"
#include "commands.h"

#define EXT_TX_BUF_SIZE 2048
#define EXT_RX_BUF_SIZE 2048
#define EXT_MAX_PACKET_SIZE 512

typedef struct _EXTStats
{
	uint32_t rxWire;
	uint32_t rxPayload;
	uint32_t txWire;
	uint32_t txPayload;
	float rxUsage;
	float txUsage;
} EXTStats;

typedef struct _Ext
{
	uint8_t installed;

	uint32_t baudrate;

	binary_semaphore_t dataAvailable;

	uint8_t rxProcessingBuf[COBSMaxStuffedSize(EXT_MAX_PACKET_SIZE)+1];
	uint16_t rxProcBufUsed;

	uint8_t rxDecodeBuf[EXT_MAX_PACKET_SIZE];

	uint8_t txBuf[EXT_TX_BUF_SIZE];
	uint8_t rxBuf[2][EXT_RX_BUF_SIZE];

	uint8_t txAssemblyBuf[EXT_MAX_PACKET_SIZE];

	mutex_t txMutex;

	DMA_Stream_TypeDef* pRxStream;
	DMA_Stream_TypeDef* pTxStream;
	volatile uint32_t* pRxIFCR;
	volatile uint32_t* pTxIFCR;
	uint32_t rxIfcrClearAll;
	uint32_t txIfcrClearAll;

	int32_t curTxTransferSize;

	EXTStats liveStats;
	EXTStats stats;
} Ext;

extern Ext ext;

void ExtInit(uint32_t baudrate);
void ExtTask(void* params);
int16_t ExtSendPacket(PacketHeader* pHeader, const void* _pData, uint16_t dataLength);
