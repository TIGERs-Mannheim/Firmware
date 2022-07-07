/*
 * fifo_block.h
 *
 *  Created on: 01.09.2014
 *      Author: AndreR
 */

#pragma once

#include "ch.h"

#define FIFO_BLOCK_STATE_EMPTY	0
#define FIFO_BLOCK_STATE_ACTIVE	1
#define FIFO_BLOCK_STATE_FULL	2

typedef void(*FifoBlockFullCb)(uint32_t);

typedef struct _FifoBlockBuffer
{
	uint8_t* pFirst;
	uint8_t* pLast;
	volatile uint8_t* pWrite;
	volatile uint8_t state;
} FifoBlockBuffer;

#define FIFO_BLOCK_DATA(numBuf, bufSize) (sizeof(FifoBlockBuffer)*numBuf + bufSize*numBuf)

typedef struct _FifoBlock
{
	uint8_t* pData;
	uint32_t blockSize;	// including Header
	uint32_t activeBlock;
	uint32_t numBlocks;
	mutex_t writeMutex;

	FifoBlockFullCb fullCb;

	uint16_t fullCbActive;
} FifoBlock;

void	FifoBlockInit(FifoBlock* pFifo, uint32_t numBuf, uint32_t bufSize, uint8_t* pBufData);

int16_t FifoBlockPush(FifoBlock* pFifo, const void* _pData, uint32_t size, uint8_t noBlock);
void	FifoBlockFlush(FifoBlock* pFifo);

int16_t FifoBlockGetData(FifoBlock* pFifo, uint32_t index, uint8_t** ppData, uint32_t* pDataSize);
int16_t FifoBlockFreeBuffer(FifoBlock* pFifo, uint32_t index);

void FifoBlockDebug(FifoBlock* pFifo);
