#include "queue.h"

void QueueInit(Queue* pQueue, uint8_t* pData, size_t dataSize)
{
    pQueue->pDataBegin = pData;
    pQueue->pDataEnd = pData + dataSize;
    pQueue->pRead = pData;
    pQueue->pWrite = pData;
    pQueue->counter = 0;
}

void QueueReset(Queue* pQueue)
{
    pQueue->pRead = pQueue->pDataBegin;
    pQueue->pWrite = pQueue->pDataBegin;
    pQueue->counter = 0;
}

size_t QueueGetFullSpace(Queue* pQueue)
{
    return pQueue->counter;
}

size_t QueueGetEmptySpace(Queue* pQueue)
{
    return (size_t)(pQueue->pDataEnd - pQueue->pDataBegin) - pQueue->counter;
}

size_t QueueRead(Queue* pQueue, uint8_t* pTarget, size_t targetSize)
{
    if(targetSize > pQueue->counter)
        targetSize = pQueue->counter;

    for(size_t i = 0; i < targetSize; i++)
    {
        *pTarget++ = *pQueue->pRead++;
        if(pQueue->pRead >= pQueue->pDataEnd)
            pQueue->pRead = pQueue->pDataBegin;
    }

    if(targetSize > 0)
    {
		syssts_t sts = chSysGetStatusAndLockX();

		pQueue->counter -= targetSize;

		chSysRestoreStatusX(sts);
    }

    return targetSize;
}

size_t QueuePeek(const Queue* pQueue, uint8_t* pTarget, size_t targetSize)
{
    if(targetSize > pQueue->counter)
        targetSize = pQueue->counter;

    volatile uint8_t* pPeek = pQueue->pRead;

    for(size_t i = 0; i < targetSize; i++)
    {
        *pTarget++ = *pPeek++;
        if(pPeek >= pQueue->pDataEnd)
            pPeek = pQueue->pDataBegin;
    }

    return targetSize;
}

size_t QueueDrain(Queue* pQueue, size_t drainSize)
{
    if(drainSize > pQueue->counter)
        drainSize = pQueue->counter;

    for(size_t i = 0; i < drainSize; i++)
    {
        pQueue->pRead++;

        if(pQueue->pRead >= pQueue->pDataEnd)
            pQueue->pRead = pQueue->pDataBegin;
    }

    if(drainSize > 0)
    {
		syssts_t sts = chSysGetStatusAndLockX();

		pQueue->counter -= drainSize;

		chSysRestoreStatusX(sts);
    }

    return drainSize;
}

size_t QueueWrite(Queue* pQueue, const uint8_t* pData, size_t dataSize)
{
    if(dataSize > QueueGetEmptySpace(pQueue))
        return 0;

    for(size_t i = 0; i < dataSize; i++)
    {
        *pQueue->pWrite++ = *pData++;
        if(pQueue->pWrite >= pQueue->pDataEnd)
            pQueue->pWrite = pQueue->pDataBegin;
    }

    if(dataSize > 0)
    {
		syssts_t sts = chSysGetStatusAndLockX();

		pQueue->counter += dataSize;

		chSysRestoreStatusX(sts);
    }

    return dataSize;
}
