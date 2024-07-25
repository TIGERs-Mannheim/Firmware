#include "fifo.h"
#include <string.h>

void FifoInit(Fifo* pFifo, void* pData, size_t dataSize)
{
	pFifo->pDataBegin = pData;
	pFifo->pDataEnd = pFifo->pDataBegin + dataSize;
	pFifo->pRead = pFifo->pDataBegin;
	pFifo->pWrite = pFifo->pDataBegin;
	pFifo->counter = 0;
}

void FifoReset(Fifo* pFifo)
{
	pFifo->pRead = pFifo->pDataBegin;
	pFifo->pWrite = pFifo->pDataBegin;
	pFifo->counter = 0;
}

size_t FifoGetSize(Fifo* pFifo)
{
	return pFifo->pDataEnd - pFifo->pDataBegin;
}

size_t FifoGetFullSpace(Fifo* pFifo)
{
	return pFifo->counter;
}

size_t FifoGetLinearFullSpace(Fifo* pFifo)
{
	if(pFifo->counter == 0)
		return 0;

	if(pFifo->pWrite <= pFifo->pRead)
		return pFifo->pDataEnd - pFifo->pRead;

	return pFifo->pWrite - pFifo->pRead;
}

size_t FifoGetEmptySpace(Fifo* pFifo)
{
	return FifoGetSize(pFifo) - pFifo->counter;
}

size_t FifoGetLinearEmptySpace(Fifo* pFifo)
{
	if(FifoGetEmptySpace(pFifo) == 0)
		return 0;

	if(pFifo->pRead <= pFifo->pWrite)
		return pFifo->pDataEnd - pFifo->pWrite;

	return pFifo->pRead - pFifo->pWrite;
}

size_t FifoRead(Fifo* pFifo, void* pTarget, size_t targetSize)
{
	if(targetSize == 0 || pFifo->counter == 0)
		return 0;

    if(targetSize > pFifo->counter)
        targetSize = pFifo->counter;

    if(pFifo->pRead < pFifo->pWrite)
    {
    	memcpy(pTarget, pFifo->pRead, targetSize);
    	pFifo->pRead += targetSize;
    }
    else
    {
    	size_t bytesToEnd = pFifo->pDataEnd - pFifo->pRead;
    	if(bytesToEnd > targetSize)
    	{
    		memcpy(pTarget, pFifo->pRead, targetSize);
    		pFifo->pRead += targetSize;
    	}
    	else
    	{
    		memcpy(pTarget, pFifo->pRead, bytesToEnd);
    		memcpy((uint8_t*)pTarget + bytesToEnd, pFifo->pDataBegin, targetSize - bytesToEnd);
    		pFifo->pRead = pFifo->pDataBegin + targetSize - bytesToEnd;
    	}
    }

	pFifo->counter -= targetSize;

    return targetSize;
}

size_t FifoPeek(Fifo* pFifo, void* pTarget, size_t targetSize)
{
	size_t counter = pFifo->counter;
	uint8_t* pRead = pFifo->pRead;

	size_t result = FifoRead(pFifo, pTarget, targetSize);

	pFifo->counter = counter;
	pFifo->pRead = pRead;

	return result;
}

size_t FifoDrain(Fifo* pFifo, size_t drainSize)
{
	if(drainSize == 0 || pFifo->counter == 0)
		return 0;

    if(drainSize > pFifo->counter)
    	drainSize = pFifo->counter;

    if(pFifo->pRead < pFifo->pWrite)
    {
    	pFifo->pRead += drainSize;
    }
    else
    {
    	size_t bytesToEnd = pFifo->pDataEnd - pFifo->pRead;
    	if(bytesToEnd > drainSize)
    	{
    		pFifo->pRead += drainSize;
    	}
    	else
    	{
    		pFifo->pRead = pFifo->pDataBegin + drainSize - bytesToEnd;
    	}
    }

	pFifo->counter -= drainSize;

	return drainSize;
}

size_t FifoWrite(Fifo* pFifo, const void* pData, size_t dataSize)
{
    if(dataSize > FifoGetEmptySpace(pFifo) || dataSize == 0)
        return 0;

    if(pFifo->pWrite < pFifo->pRead)
    {
    	memcpy(pFifo->pWrite, pData, dataSize);
    	pFifo->pWrite += dataSize;
    }
    else
    {
    	size_t bytesToEnd = pFifo->pDataEnd - pFifo->pWrite;
    	if(bytesToEnd > dataSize)
    	{
    		memcpy(pFifo->pWrite, pData, dataSize);
    		pFifo->pWrite += dataSize;
    	}
    	else
    	{
    		memcpy(pFifo->pWrite, pData, bytesToEnd);
    		memcpy(pFifo->pDataBegin, (uint8_t*)pData + bytesToEnd, dataSize - bytesToEnd);
    		pFifo->pWrite = pFifo->pDataBegin + dataSize - bytesToEnd;
    	}
    }

	pFifo->counter += dataSize;

    return dataSize;
}

size_t FifoInject(Fifo* pFifo, size_t dataSize)
{
    if(dataSize > FifoGetEmptySpace(pFifo) || dataSize == 0)
        return 0;

    if(pFifo->pWrite < pFifo->pRead)
    {
    	pFifo->pWrite += dataSize;
    }
    else
    {
    	size_t bytesToEnd = pFifo->pDataEnd - pFifo->pWrite;
    	if(bytesToEnd > dataSize)
    	{
    		pFifo->pWrite += dataSize;
    	}
    	else
    	{
    		pFifo->pWrite = pFifo->pDataBegin + dataSize - bytesToEnd;
    	}
    }

	pFifo->counter += dataSize;

    return dataSize;
}
