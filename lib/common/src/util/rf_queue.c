/*
 * rf_queue.c
 *
 *  Created on: 27.09.2017
 *      Author: AndreR
 */

#include "rf_queue.h"
#include "cobs.h"
#include "errors.h"
#include "log.h"
#include <string.h>

void RFQueueInit(RFQueue* pQueue, uint8_t* pDataTx, uint16_t txSize, uint8_t* pDataRx, uint16_t rxSize, uint16_t maxPacketLength, uint16_t maxMessageSize)
{
	pQueue->maxPacketLength = maxPacketLength;
	pQueue->maxMessageSize = maxMessageSize;
	pQueue->maxEncodedMsgSize = COBSMaxStuffedSize(maxMessageSize);
	pQueue->rxTxBufSize = maxPacketLength + pQueue->maxEncodedMsgSize;

	pQueue->rxBuf = pDataRx;
	pQueue->txBuf = pDataTx;

	pDataRx += pQueue->rxTxBufSize;
	pDataTx += pQueue->rxTxBufSize;
	rxSize -= pQueue->rxTxBufSize;
	txSize -= pQueue->rxTxBufSize;

	FifoLinInit(&pQueue->rxFifo, pDataRx, rxSize);
	FifoLinInit(&pQueue->txFifo, pDataTx, txSize);

	pQueue->rxBufUsed = 0;
	pQueue->txBufUsed = 0;
	pQueue->rxSeq = 0;
	pQueue->txSeq = 0;

	memset(&pQueue->stats, 0, sizeof(pQueue->stats));
}

void RFQueueSetOutgoingCallback(RFQueue* pQueue, RFOutgoingCmdFunc func, void* pUserData)
{
	pQueue->pOutgoindCmdCb = func;
	pQueue->pUserData = pUserData;
}

int16_t RFQueueGetPacket(RFQueue* pQueue, uint8_t* pDst, uint32_t dstSize, uint32_t* pPktSize)
{
	uint8_t* pSrc;
	*pPktSize = 0;

	if(FifoLinGet(&pQueue->rxFifo, &pSrc, pPktSize))
		return ERROR_RF_QUEUE_EMPTY;

	if(*pPktSize > dstSize)
	{
		LogErrorC("Dst mem too small", *pPktSize | dstSize << 16);
		return ERROR_NOT_ENOUGH_MEMORY;
	}

	memcpy(pDst, pSrc, *pPktSize);

	FifoLinDelete(&pQueue->rxFifo);

	return 0;
}


/**
 * Put data in an RF queue.
 *
 * @param pQueue the queue
 * @param pData data to send
 * @param dataSize datasize
 * @return 0 on success, errorcode otherwise
 */
int16_t RFQueueSend(RFQueue* pQueue, const uint8_t* pData, uint32_t dataSize)
{
	uint8_t* pDst;
	int16_t result;

	if(dataSize > pQueue->maxMessageSize)
		return ERROR_INVALID_PARAMETER;

	result = FifoLinReserve(&pQueue->txFifo, dataSize, &pDst);
	if(result)
	{
		++pQueue->stats.queueFeedback.txPacketsLost;
		return result;
	}

	memcpy(pDst, pData, dataSize);

	FifoLinCommit(&pQueue->txFifo, dataSize);

	return 0;
}

void RFQueueClear(RFQueue* pQueue)
{
	FifoLinClear(&pQueue->txFifo);
	FifoLinClear(&pQueue->rxFifo);
}

uint32_t RFQueueFetchTxPacket(RFQueue* pQueue, uint8_t* pDataDst)
{
	uint8_t* pData;
	uint32_t pktSize;
	uint32_t bytesWritten;

	++pQueue->txSeq;
	pQueue->txSeq &= 0x7F;

	uint8_t header = pQueue->txSeq;
	if(pQueue->txBufUsed != 0)
		header |= 0x80;	// flag signals data continuation

	while(pQueue->txBufUsed < pQueue->maxPacketLength-1)
	{
		if(FifoLinGet(&pQueue->txFifo, &pData, &pktSize))
			break;

		if(pktSize > pQueue->maxMessageSize)	// assert
		{
			LogErrorC("Oversized HL message", pktSize);
			FifoLinDelete(&pQueue->txFifo);
			continue;
		}

		if(pQueue->pOutgoindCmdCb)
			(*pQueue->pOutgoindCmdCb)(pQueue->pUserData, pData, pktSize);

		COBSEncode(pData, pktSize, &pQueue->txBuf[pQueue->txBufUsed], pQueue->maxEncodedMsgSize, &bytesWritten);
		FifoLinDelete(&pQueue->txFifo);

		pQueue->txBufUsed += bytesWritten;
		pQueue->txBuf[pQueue->txBufUsed++] = 0;

		++pQueue->stats.queueIO.txPackets;
		pQueue->stats.queueIO.txBytes += pktSize;
	}

	uint32_t bytesToReturn = pQueue->txBufUsed;

	if(bytesToReturn > pQueue->maxPacketLength-1)
		bytesToReturn = pQueue->maxPacketLength-1;

	pDataDst[0] = header;
	if(bytesToReturn > 0)
	{
		memcpy(pDataDst+1, pQueue->txBuf, bytesToReturn);
		pQueue->txBufUsed -= bytesToReturn;
		memmove(pQueue->txBuf, &pQueue->txBuf[bytesToReturn], pQueue->txBufUsed);
	}

	++pQueue->stats.rfIO.txPackets;
	pQueue->stats.rfIO.txBytes += bytesToReturn+1;

	return bytesToReturn+1;
}

static void shiftOutRxData(RFQueue* pQueue, uint32_t nullPos)
{
	if(pQueue->rxBufUsed < nullPos+1)	// assert
	{
		LogErrorC("Invalid RX shift", nullPos);
		pQueue->rxBufUsed = 0;
		return;
	}

	pQueue->rxBufUsed -= nullPos+1;
	memmove(pQueue->rxBuf, &pQueue->rxBuf[nullPos+1], pQueue->rxBufUsed);
}

static void feedRxData(RFQueue* pQueue, const uint8_t* pData, uint32_t size)
{
	uint32_t nullPos;
	int16_t result;

	if(size == 0)
		return;

	if(size > pQueue->maxPacketLength-1)	// assert
	{
		LogErrorC("Oversized LL packet", size);
		return;
	}

	// append received data to rxBuf
	memcpy(&pQueue->rxBuf[pQueue->rxBufUsed], pData, size);
	pQueue->rxBufUsed += size;

	while(pQueue->rxBufUsed > 0)
	{
		// search for packet delimiter
		for(nullPos = 0; nullPos < pQueue->rxBufUsed; nullPos++)
		{
			if(pQueue->rxBuf[nullPos] == 0)
				break;
		}

		if(nullPos == pQueue->rxBufUsed)
		{
			if(pQueue->rxBufUsed > pQueue->maxEncodedMsgSize)
			{
				// we must have missed something, max message size reached and no null found
				pQueue->rxBufUsed = 0;
			}

			break;	// no packet delimiter found
		}

		if(nullPos > pQueue->maxEncodedMsgSize)
		{
			// we must also have missed something here, that message is too big
			shiftOutRxData(pQueue, nullPos);
			continue;
		}

		uint8_t* pDst;
		uint32_t bytesWritten;
		result = FifoLinReserve(&pQueue->rxFifo, pQueue->maxMessageSize, &pDst);
		if(result)
		{
			// no memory to save all eventual data
			shiftOutRxData(pQueue, nullPos);
			++pQueue->stats.queueFeedback.rxPacketsLost;
			LogErrorC("FifoResv error", pQueue->rxFifo.numPackets | FifoLinFreeMem(&pQueue->rxFifo) << 16);
			continue;
		}

		result = COBSDecode(pQueue->rxBuf, nullPos, pDst, pQueue->maxMessageSize, &bytesWritten);
		if(result)
		{
			// looks like we received garbage
			shiftOutRxData(pQueue, nullPos);
			++pQueue->stats.queueFeedback.rxPacketsLost;
			LogErrorC("COBS error", result);
			continue;
		}

		if(bytesWritten >= PACKET_HEADER_SIZE)
		{
			FifoLinCommit(&pQueue->rxFifo, bytesWritten);

			++pQueue->stats.queueIO.rxPackets;
			pQueue->stats.queueIO.rxBytes += bytesWritten;
		}

		// shift out processed data
		shiftOutRxData(pQueue, nullPos);
	}
}

int16_t RFQueueFeedRxPacket(RFQueue* pQueue, const uint8_t* pData, uint32_t size)
{
	if(size > pQueue->maxPacketLength)
		return ERROR_INVALID_PARAMETER;

	++pQueue->stats.rfIO.rxPackets;
	pQueue->stats.rfIO.rxBytes += size;

	uint8_t header = pData[0];

	const uint8_t* pRxPayload = pData + 1;
	size--;

	// process first byte with sequence and data continuation information
	++pQueue->rxSeq;
	pQueue->rxSeq &= 0x7F;

	uint8_t dataContinuation = header & 0x80;
	uint8_t recvSeq = header & 0x7F;

	if(recvSeq == pQueue->rxSeq)
	{
		// everything is fine, sequence as expected, no packet loss
		feedRxData(pQueue, pRxPayload, size);
		return 0;
	}

	// we have a sequence mismatch => packet loss
	LogWarnC("Seq mismatch", pQueue->rxSeq | ((uint32_t )recvSeq) << 16);

	pQueue->stats.rfFeedback.rxPacketsLost += (recvSeq - pQueue->rxSeq) & 0x7F;

	// packet lost, everything in the rx buffer is invalid
	pQueue->rxBufUsed = 0;

	// adjust to new sequence number
	pQueue->rxSeq = recvSeq;

	if(dataContinuation == 0)
	{
		// with this packet a new command starts, process it as normal
		feedRxData(pQueue, pRxPayload, size);
		return 0;
	}

	// packet lost and continuation from the last packet is signaled, drop until next zero
	uint16_t nullPos;
	for(nullPos = 0; nullPos < size; nullPos++)
	{
		if(pRxPayload[nullPos] == 0)
			break;
	}

	if(nullPos < size - 1)
	{
		// there starts another packet, process it
		feedRxData(pQueue, &pRxPayload[nullPos + 1], size - nullPos - 1);
	}

	return 0;
}
