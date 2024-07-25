#pragma once

#include "ch.h"

/**
 * Thread-safe queue for passing around bytes between
 * exactly two threads or one thread and one ISR.
 */
typedef struct _Queue
{
    volatile size_t counter;
    uint8_t* pDataBegin;
    uint8_t* pDataEnd;
    volatile uint8_t* pWrite;
    volatile uint8_t* pRead;
} Queue;

void QueueInit(Queue* pQueue, uint8_t* pData, size_t dataSize);
void QueueReset(Queue* pQueue);
size_t QueueGetFullSpace(Queue* pQueue);
size_t QueueGetEmptySpace(Queue* pQueue);
size_t QueueRead(Queue* pQueue, uint8_t* pTarget, size_t targetSize);
size_t QueuePeek(const Queue* pQueue, uint8_t* pTarget, size_t targetSize);
size_t QueueDrain(Queue* pQueue, size_t drainSize);
size_t QueueWrite(Queue* pQueue, const uint8_t* pData, size_t dataSize);
