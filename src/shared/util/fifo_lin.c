/*
 * fifo_lin.c
 *
 *  Created on: 20.09.2013
 *      Author: AndreR
 */

#include "fifo_lin.h"
#include "errors.h"

typedef uint32_t hdr_t;

void FifoLinInit(FifoLin* pFifo, uint8_t* pData, uint32_t size)
{
	pFifo->pFirst = pData;
	pFifo->pLast = pData+size;
	pFifo->pWrite = pData;
	pFifo->pRead = pData;
	pFifo->numPackets = 0;

	chMtxObjectInit(&pFifo->reserveMutex);
	chMtxObjectInit(&pFifo->accessMutex);
}

int16_t FifoLinAllocate(FifoLin* pFifo, uint32_t size, uint8_t** ppData)
{
	int16_t result;

	result = FifoLinReserve(pFifo, size, ppData);
	if(result)
		return result;

	result = FifoLinCommit(pFifo, size);

	return result;
}

int16_t FifoLinReserve(FifoLin* pFifo, uint32_t maxSize, uint8_t** ppData)
{
	if(pFifo->reserveMutex.owner != chThdGetSelfX())
	{
		// we do not own the mutex, so we need to acquire it
		chMtxLock(&pFifo->reserveMutex);
	}

	chMtxLock(&pFifo->accessMutex);

	// check if used space wraps around end or if its placed mid-buffer
	if(pFifo->pRead < pFifo->pWrite || (pFifo->pRead == pFifo->pWrite && pFifo->numPackets == 0))
	{
		// calculate free space before and after used area
		uint32_t sizeAfter = pFifo->pLast - pFifo->pWrite;
		uint32_t sizeBefore = pFifo->pRead - pFifo->pFirst;

		// check if space after used area is sufficient
		if(sizeAfter >= maxSize+2*sizeof(hdr_t))
		{
			*((hdr_t*)pFifo->pWrite) = maxSize;
			*ppData = pFifo->pWrite+sizeof(hdr_t);

			chMtxUnlock(&pFifo->accessMutex);
			return 0;
		}

		if(sizeBefore >= maxSize+sizeof(hdr_t))
		{
			*((hdr_t*)pFifo->pWrite) = 0;
			*((hdr_t*)pFifo->pFirst) = maxSize;
			*ppData = pFifo->pFirst+sizeof(hdr_t);

			chMtxUnlock(&pFifo->accessMutex);
			return 0;
		}
	}
	else // used space wraps
	{
		uint32_t sizeAvail = pFifo->pRead - pFifo->pWrite;

		// is there enough space available?
		if(sizeAvail >= maxSize+sizeof(hdr_t))
		{
			*((hdr_t*)pFifo->pWrite) = maxSize;
			*ppData = pFifo->pWrite+sizeof(hdr_t);

			chMtxUnlock(&pFifo->accessMutex);
			return 0;
		}
	}

	chMtxUnlock(&pFifo->accessMutex);
	chMtxUnlock(&pFifo->reserveMutex);

	return ERROR_FIFO_LIN_NOMEM;
}

int16_t FifoLinCommit(FifoLin* pFifo, uint32_t size)
{
	if(pFifo->reserveMutex.owner != chThdGetSelfX())
		return ERROR_FIFO_LIN_NOT_OWNED;

	if(size == 0)
	{
		chMtxUnlock(&pFifo->reserveMutex);
		return 0;
	}

	chMtxLock(&pFifo->accessMutex);

	// insert at pFirst or pWrite?
	if(*((hdr_t*)pFifo->pWrite) == 0)
	{
		// planned to insert at pFirst
		if(size > *((hdr_t*)pFifo->pFirst))
		{
			chMtxUnlock(&pFifo->accessMutex);
			chMtxUnlock(&pFifo->reserveMutex);
			return ERROR_FIFO_LIN_EXCEED_RESV;
		}

		*((hdr_t*)pFifo->pFirst) = size;
		pFifo->pWrite = pFifo->pFirst+size+sizeof(hdr_t);
	}
	else
	{
		// insert at pWrite
		if(size > *((hdr_t*)pFifo->pWrite))
		{
			chMtxUnlock(&pFifo->accessMutex);
			chMtxUnlock(&pFifo->reserveMutex);
			return ERROR_FIFO_LIN_EXCEED_RESV;
		}

		*((hdr_t*)pFifo->pWrite) = size;
		pFifo->pWrite += size+sizeof(hdr_t);
	}

	++pFifo->numPackets;

	if(pFifo->numPackets > (pFifo->pLast-pFifo->pFirst)/sizeof(hdr_t))
	{
		asm volatile("BKPT #02");
	}

	chMtxUnlock(&pFifo->accessMutex);
	chMtxUnlock(&pFifo->reserveMutex);

	return 0;
}

int16_t FifoLinGet(FifoLin* pFifo, uint8_t** ppData, uint32_t* pSize)
{
	chMtxLock(&pFifo->accessMutex);

	if(pFifo->numPackets == 0)
	{
		*ppData = 0;
		chMtxUnlock(&pFifo->accessMutex);
		return ERROR_FIFO_LIN_EMPTY;
	}

	*pSize = *((uint32_t*)pFifo->pRead);

	if(*pSize == 0)
	{
		*pSize = *((uint32_t*)pFifo->pFirst);
		*ppData = pFifo->pFirst+sizeof(hdr_t);
	}
	else
	{
		*ppData = pFifo->pRead+sizeof(hdr_t);
	}

	chMtxUnlock(&pFifo->accessMutex);

	return 0;
}

int16_t FifoLinDelete(FifoLin* pFifo)
{
	chMtxLock(&pFifo->accessMutex);

	if(pFifo->numPackets == 0)
	{
		chMtxUnlock(&pFifo->accessMutex);
		return ERROR_FIFO_LIN_EMPTY;
	}

	uint32_t size;

	size = *((uint32_t*)pFifo->pRead);

	if(size == 0)
	{
		pFifo->pRead = pFifo->pFirst;
		size = *((uint32_t*)pFifo->pRead);
	}

	pFifo->pRead += size+sizeof(hdr_t);

	--pFifo->numPackets;

	chMtxUnlock(&pFifo->accessMutex);

	return 0;
}

uint32_t FifoLinFreeMem(FifoLin* pFifo)
{
	chMtxLock(&pFifo->accessMutex);

	// check if used space wraps around end of if its placed mid-buffer
	if(pFifo->pRead <= pFifo->pWrite)
	{
		// calculate free space before and after used area
		uint32_t sizeAfter = pFifo->pLast - pFifo->pWrite;
		uint32_t sizeBefore = pFifo->pRead - pFifo->pFirst;

		chMtxUnlock(&pFifo->accessMutex);

		if(sizeAfter < 2*sizeof(hdr_t))
			sizeAfter = 0;
		else
			sizeAfter -= 2*sizeof(hdr_t);

		if(sizeBefore < sizeof(hdr_t))
			sizeBefore = 0;
		else
			sizeBefore -= sizeof(hdr_t);

		if(sizeAfter >= sizeBefore)
			return sizeAfter;
		else
			return sizeBefore;
	}
	else // used space wraps
	{
		uint32_t sizeAvail = pFifo->pRead - pFifo->pWrite;

		if(sizeAvail < sizeof(hdr_t))
			sizeAvail = 0;
		else
			sizeAvail -= sizeof(hdr_t);

		chMtxUnlock(&pFifo->accessMutex);

		return sizeAvail;
	}
}

void FifoLinClear(FifoLin* pFifo)
{
	chMtxLock(&pFifo->accessMutex);
	chMtxLock(&pFifo->reserveMutex);

	pFifo->pWrite = pFifo->pFirst;
	pFifo->pRead = pFifo->pFirst;
	pFifo->numPackets = 0;

	chMtxUnlock(&pFifo->reserveMutex);
	chMtxUnlock(&pFifo->accessMutex);
}
