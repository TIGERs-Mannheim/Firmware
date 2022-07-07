/*
 * fifo_lin.h
 *
 *  Created on: 19.09.2013
 *      Author: AndreR
 *
 *  Linear FIFO which stores blocks of data which are always continuous in memory.
 *  In worst case scenarios the maximum unused memory is equal to the maximum packet size.
 *  Packets bigger than half the size of total memory are not guaranteed to fit,
 *  although enough memory might be free.
 *
 *  Synchronization behavior:
 *  - Reserve and Commit (incl. Allocate) are synchronized on task level
 *  - Get and Delete are unprotected
 *
 *  Intended R/W-Model:
 *  Multiple Write Single Read (MWSR)
 */

#ifndef FIFO_LIN_H_
#define FIFO_LIN_H_

#include <stdint.h>

#include "ch.h"

typedef struct _LinFifo
{
	uint8_t* pFirst;	// first writable byte
	uint8_t* pLast;	// one byte past the last writable byte
	uint8_t* volatile pRead;	// start of next block to read
	uint8_t* volatile pWrite;	// next free byte to start writing

	volatile uint32_t numPackets;

	mutex_t reserveMutex;
	mutex_t accessMutex;
} FifoLin;

void FifoLinInit(FifoLin* pFifo, uint8_t* pData, uint32_t size);
int16_t FifoLinAllocate(FifoLin* pFifo, uint32_t size, uint8_t** ppData);
int16_t FifoLinReserve(FifoLin* pFifo, uint32_t maxSize, uint8_t** ppData);
int16_t FifoLinCommit(FifoLin* pFifo, uint32_t size);
int16_t FifoLinGet(FifoLin* pFifo, uint8_t** ppData, uint32_t* pSize);
int16_t FifoLinDelete(FifoLin* pFifo);
uint32_t FifoLinFreeMem(FifoLin* pFifo);
void FifoLinClear(FifoLin* pFifo);

#endif /* FIFO_LIN_H_ */
