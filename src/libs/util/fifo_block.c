/*
 * fifo_block.c
 *
 *  Created on: 01.09.2014
 *      Author: AndreR
 */

#include "fifo_block.h"
#include "errors.h"
#include "util/console.h"
#include <string.h>

void FifoBlockInit(FifoBlock* pFifo, uint32_t numBuf, uint32_t bufSize, uint8_t* pBufData)
{
	chMtxObjectInit(&pFifo->writeMutex);
	pFifo->pData = pBufData;
	pFifo->blockSize = bufSize+sizeof(FifoBlockBuffer);
	pFifo->activeBlock = 0;
	pFifo->numBlocks = numBuf;

	for(uint32_t buf = 0; buf < numBuf; buf++)
	{
		FifoBlockBuffer* pBuf = (FifoBlockBuffer*)pBufData;
		pBufData += sizeof(FifoBlockBuffer);
		pBuf->pFirst = pBufData;
		pBuf->pWrite = pBufData;
		pBufData += bufSize;
		pBuf->pLast = pBufData;
		pBuf->state = FIFO_BLOCK_STATE_EMPTY;
	}
}

static FifoBlockBuffer* getBuffer(FifoBlock* pFifo, uint32_t index)
{
	return (FifoBlockBuffer*)(pFifo->pData+(index*pFifo->blockSize));
}

int16_t FifoBlockGetData(FifoBlock* pFifo, uint32_t index, uint8_t** ppData, uint32_t* pDataSize)
{
	FifoBlockBuffer* pBuf = getBuffer(pFifo, index);
	if(pBuf->state != FIFO_BLOCK_STATE_FULL)
		return ERROR_FIFO_BLOCK_NOT_FULL;

	*ppData = pBuf->pFirst;
	*pDataSize = (uint32_t)(pBuf->pWrite-pBuf->pFirst);

	return 0;
}

int16_t FifoBlockFreeBuffer(FifoBlock* pFifo, uint32_t index)
{
	FifoBlockBuffer* pBuf = getBuffer(pFifo, index);
	if(pBuf->state != FIFO_BLOCK_STATE_FULL)
		return ERROR_FIFO_BLOCK_NOT_FULL;

	pBuf->pWrite = pBuf->pFirst;
	pBuf->state = FIFO_BLOCK_STATE_EMPTY;

	return 0;
}

void FifoBlockFlush(FifoBlock* pFifo)
{
	chMtxLock(&pFifo->writeMutex);

	FifoBlockBuffer* pBuffer = getBuffer(pFifo, pFifo->activeBlock);

	if(pBuffer->state != FIFO_BLOCK_STATE_FULL)
	{
		pBuffer->state = FIFO_BLOCK_STATE_FULL;

		// issue write event
		if(pFifo->fullCb)
			(*pFifo->fullCb)(pFifo->activeBlock);
	}

	while(pBuffer->state != FIFO_BLOCK_STATE_EMPTY)
		chThdSleepMilliseconds(1);

	pFifo->activeBlock = 0;

	chMtxUnlock(&pFifo->writeMutex);
}

int16_t FifoBlockPush(FifoBlock* pFifo, const void* _pData, uint32_t size, uint8_t noBlock)
{
	int16_t result = 0;
	uint32_t bytesWritten = 0;
	uint32_t sizeLeft = size;
	const uint8_t* pData = (uint8_t*)_pData;

	chMtxLock(&pFifo->writeMutex);

	uint32_t bytesFree = 0;

	FifoBlockBuffer* pBuffer;
	for(uint32_t i = 0; i < pFifo->numBlocks; i++)
	{
		pBuffer = getBuffer(pFifo, (pFifo->activeBlock+i)%pFifo->numBlocks);
		bytesFree += pBuffer->pLast-pBuffer->pWrite;
	}

	if(noBlock && bytesFree < size)
	{
		chMtxUnlock(&pFifo->writeMutex);
		return ERROR_NOT_ENOUGH_MEMORY;
	}

	while(bytesWritten < size)
	{
		pBuffer = getBuffer(pFifo, pFifo->activeBlock);

		while(pBuffer->state == FIFO_BLOCK_STATE_FULL)
		{
			result = ERROR_FIFO_BLOCK_DELAYED;
			chThdSleepMilliseconds(1);
		}

		if(pBuffer->pWrite == pBuffer->pFirst)
			pBuffer->state = FIFO_BLOCK_STATE_ACTIVE;

		uint32_t writeSize = pBuffer->pLast-pBuffer->pWrite;
		if(writeSize > sizeLeft)
			writeSize = sizeLeft;

		memcpy((void*)pBuffer->pWrite, pData, writeSize);
		pBuffer->pWrite += writeSize;
		pData += writeSize;
		bytesWritten += writeSize;
		sizeLeft -= writeSize;

		if(pBuffer->pWrite == pBuffer->pLast)
		{
			pBuffer->state = FIFO_BLOCK_STATE_FULL;

			pFifo->fullCbActive = 1;

			// issue write event
			if(pFifo->fullCb)
				(*pFifo->fullCb)(pFifo->activeBlock);

			pFifo->fullCbActive = 0;

			++pFifo->activeBlock;
			pFifo->activeBlock %= pFifo->numBlocks;
		}
	}

	chMtxUnlock(&pFifo->writeMutex);

	return result;
}

void FifoBlockDebug(FifoBlock* pFifo)
{
	ConsolePrint("activeBlock: %u, fullCbActive: %hu\r\n", pFifo->activeBlock, pFifo->fullCbActive);

	ConsolePrint("pFirst     pLast      pWrite     state\r\n");
	for(uint32_t i = 0; i < pFifo->numBlocks; i++)
	{
		FifoBlockBuffer* pBuffer = getBuffer(pFifo, i);
		ConsolePrint("0x%08X 0x%08X 0x%08X %hu\r\n", pBuffer->pFirst, pBuffer->pLast, pBuffer->pWrite, (uint16_t)pBuffer->state);
	}
}
