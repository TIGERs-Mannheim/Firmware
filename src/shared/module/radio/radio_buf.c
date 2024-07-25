#include "radio_buf.h"
#include "errors.h"
#include "util/crc.h"
#include <string.h>

void RadioBufInit(RadioBuffer* pBuf, uint8_t isRx)
{
	pBuf->isRxBuf = isRx;
	chMtxObjectInit(&pBuf->accessMtx);

	if(isRx)
	{
		for(size_t i = 0; i < RADIO_APP_MAX_BUFFERED_PACKETS; i++)
		{
			pBuf->entries[i].isOwnedByRadio = 1;
		}
	}
}

uint8_t RadioBufferIsEmpty(RadioBuffer* pBuf)
{
	if(pBuf->isRxBuf)
	{
		for(size_t i = 0; i < RADIO_APP_MAX_BUFFERED_PACKETS; i++)
		{
			if(pBuf->entries[i].isOwnedByRadio == 0)
				return 0;
		}
	}
	else
	{
		for(size_t i = 0; i < RADIO_APP_MAX_BUFFERED_PACKETS; i++)
		{
			if(pBuf->entries[i].isOwnedByRadio == 1)
				return 0;
		}
	}

	return 1;
}

uint32_t RadioBufPutRxAirPacket(RadioBuffer* pRxBuf, const uint8_t* pData, size_t dataSize)
{
	if(dataSize == 0)
		return 0;

	uint32_t numDecodedAppPackets = 0;
	uint32_t payloadLeft = dataSize;

	pRxBuf->stats.rxPacketsTotal++;

	while(payloadLeft)
	{
		RadioBufferEntry* pBuf = &pRxBuf->entries[pRxBuf->nextRadioEntry];
		if(!pBuf->isOwnedByRadio)
		{
			// RX overrun, no buffer free
			pRxBuf->stats.rxBufferOverruns++;
			pRxBuf->stats.rxBytesDiscarded += dataSize;
			return numDecodedAppPackets;
		}

		// search for packet delimiter
		uint8_t delimiterFound = 0;
		uint32_t copySize;
		for(copySize = 0; copySize < payloadLeft; copySize++)
		{
			if(pData[copySize] == 0)
			{
				delimiterFound = 1;
				copySize++;
				break;
			}
		}

		if(pBuf->dataSize + copySize > sizeof(pBuf->data))
		{
			// Buffer memory overflowed, probably missed packet(s)
			pRxBuf->stats.rxBufferDataOverflows++;
			pRxBuf->stats.rxBytesDiscarded += pBuf->dataSize;
			pBuf->dataSize = 0;
		}

		// Copy data, including possible zero
		memcpy(pBuf->data + pBuf->dataSize, pData, copySize);
		pBuf->dataSize += copySize;
		payloadLeft -= copySize;
		pData += copySize;

		if(delimiterFound)
		{
			if(pBuf->dataSize <= 1)
			{
				pBuf->dataSize = 0;
			}
			else
			{
				pRxBuf->nextRadioEntry = (pRxBuf->nextRadioEntry + 1) % RADIO_APP_MAX_BUFFERED_PACKETS;
				pBuf->isOwnedByRadio = 0;
				numDecodedAppPackets++;
			}
		}
	}

	return numDecodedAppPackets;
}

uint32_t RadioBufGetTxAirPacket(RadioBuffer* pTxBuf, uint8_t* pData, size_t dataSize)
{
	if(dataSize == 0)
		return 0;

	uint32_t numEncodedAppPackets = 0;

	size_t payloadBytesFree = dataSize;

	while(payloadBytesFree)
	{
		RadioBufferEntry* pBuf = &pTxBuf->entries[pTxBuf->nextRadioEntry];
		if(pBuf->isOwnedByRadio)
		{
			size_t entryBytesLeft = pBuf->dataSize - pBuf->bytesProcessed;
			size_t copySize = payloadBytesFree;
			if(copySize > entryBytesLeft)
				copySize = entryBytesLeft;

			memcpy(pData, pBuf->data + pBuf->bytesProcessed, copySize);
			pData += copySize;
			pBuf->bytesProcessed += copySize;
			payloadBytesFree -= copySize;

			if(pBuf->bytesProcessed >= pBuf->dataSize)
			{
				pTxBuf->nextRadioEntry = (pTxBuf->nextRadioEntry + 1) % RADIO_APP_MAX_BUFFERED_PACKETS;
				pBuf->isOwnedByRadio = 0;
				numEncodedAppPackets++;
			}
		}
		else
		{
			memset(pData, 0, payloadBytesFree);
			break;
		}
	}

	pTxBuf->stats.txPacketsTotal++;
	pTxBuf->stats.txBytesTotal += dataSize - payloadBytesFree;

	return numEncodedAppPackets;
}

size_t RadioBufGetNextAppPacket(RadioBuffer* pRxBuf, uint8_t* pData, size_t dataSizeMax)
{
	chMtxLock(&pRxBuf->accessMtx);

	RadioBufferEntry* pEntry = &pRxBuf->entries[pRxBuf->nextAppEntry];
	if(pEntry->isOwnedByRadio || dataSizeMax < RADIO_APP_MAX_PACKET_SIZE+2)
	{
		chMtxUnlock(&pRxBuf->accessMtx);
		return 0;
	}

	uint32_t bytesDecoded = 0;
	int16_t result = COBSDecode(pEntry->data, pEntry->dataSize-1, pData, dataSizeMax, &bytesDecoded);
	if(result || bytesDecoded < 2)
	{
		pRxBuf->stats.rxCobsDecodingErrors++;
		pRxBuf->stats.rxBytesDiscarded += pEntry->dataSize;

		pRxBuf->nextAppEntry = (pRxBuf->nextAppEntry + 1) % RADIO_APP_MAX_BUFFERED_PACKETS;
		pEntry->dataSize = 0;
		pEntry->isOwnedByRadio = 1;
		chMtxUnlock(&pRxBuf->accessMtx);
		return RadioBufGetNextAppPacket(pRxBuf, pData, dataSizeMax);
	}

	uint16_t crc = CRC16CalcChecksum(pData, bytesDecoded - sizeof(uint16_t));
	uint16_t* pPacketCrc = (uint16_t*)(pData + bytesDecoded - sizeof(uint16_t));
	if(*pPacketCrc != crc)
	{
		pRxBuf->stats.rxCrcErrors++;
		pRxBuf->stats.rxBytesDiscarded += pEntry->dataSize;

		pRxBuf->nextAppEntry = (pRxBuf->nextAppEntry + 1) % RADIO_APP_MAX_BUFFERED_PACKETS;
		pEntry->dataSize = 0;
		pEntry->isOwnedByRadio = 1;
		chMtxUnlock(&pRxBuf->accessMtx);
		return RadioBufGetNextAppPacket(pRxBuf, pData, dataSizeMax);
	}

	pRxBuf->stats.rxBytesGood += pEntry->dataSize;
	pRxBuf->stats.rxAppBytes += bytesDecoded - sizeof(uint16_t);

	pRxBuf->nextAppEntry = (pRxBuf->nextAppEntry + 1) % RADIO_APP_MAX_BUFFERED_PACKETS;
	pEntry->dataSize = 0;
	pEntry->isOwnedByRadio = 1;

	chMtxUnlock(&pRxBuf->accessMtx);

	return bytesDecoded - sizeof(uint16_t);
}

size_t RadioBufEnqueueAppPacket(RadioBuffer* pTxBuf, const uint8_t* pData, size_t dataSize)
{
	chMtxLock(&pTxBuf->accessMtx);

	RadioBufferEntry* pEntry = &pTxBuf->entries[pTxBuf->nextAppEntry];
	if(pEntry->isOwnedByRadio || dataSize > RADIO_APP_MAX_PACKET_SIZE)
	{
		pTxBuf->stats.txBufferUnderrun++;
		chMtxUnlock(&pTxBuf->accessMtx);
		return 0;
	}

	uint16_t crc = CRC16CalcChecksum(pData, dataSize);

	COBSState cobs;
	uint32_t encodedSize = 0;
	COBSStartEncode(&cobs, dataSize + sizeof(uint16_t), pEntry->data, sizeof(pEntry->data));
	COBSFeedEncode(&cobs, pData, dataSize);
	COBSFeedEncode(&cobs, &crc, sizeof(uint16_t));
	COBSFinalizeEncode(&cobs, &encodedSize);

	pEntry->data[encodedSize++] = 0;
	pEntry->dataSize = encodedSize;
	pEntry->bytesProcessed = 0;

	pTxBuf->nextAppEntry = (pTxBuf->nextAppEntry + 1) % RADIO_APP_MAX_BUFFERED_PACKETS;
	pEntry->isOwnedByRadio = 1;

	chMtxUnlock(&pTxBuf->accessMtx);

	return dataSize;
}
