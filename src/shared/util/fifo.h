#pragma once

#include <stdint.h>
#include <stddef.h>

typedef struct _Fifo
{
	size_t counter;
    uint8_t* pDataBegin;
    uint8_t* pDataEnd;
    uint8_t* pWrite;
    uint8_t* pRead;
} Fifo;

/**
 * Initialize a FIFO structure.
 *
 * @param pFifo
 * @param pData Storage location.
 * @param dataSize Storage size.
 */
void FifoInit(Fifo* pFifo, void* pData, size_t dataSize);

/**
 * Clear all data and reset struct members to initial values.
 *
 * @param pFifo
 */
void FifoReset(Fifo* pFifo);

/// Get number of bytes which can be read
size_t FifoGetFullSpace(Fifo* pFifo);

/// Get number of bytes which can be read from continuous memory, starting at pFifo->pRead
size_t FifoGetLinearFullSpace(Fifo* pFifo);

/// Get number of bytes which can be written
size_t FifoGetEmptySpace(Fifo* pFifo);

/// Number of bytes which can be written to continuous memory, starting at pFifo->pWrite
size_t FifoGetLinearEmptySpace(Fifo* pFifo);

/**
 * Read data from FIFO.
 * If there is not enough data in the FIFO or target memory is too small a partial read is performed.
 *
 * @param pFifo
 * @param pTarget Target memory.
 * @param targetSize Size of target memory.
 * @return Number of bytes actually read.
 */
size_t FifoRead(Fifo* pFifo, void* pTarget, size_t targetSize);

/// Same as FifoRead, but data is not removed from the FIFO
size_t FifoPeek(Fifo* pFifo, void* pTarget, size_t targetSize);

/**
 * Drop data from FIFO.
 *
 * @param pFifo
 * @param drainSize Number of bytes to drop.
 * @return Number of bytes actually dropped.
 */
size_t FifoDrain(Fifo* pFifo, size_t drainSize);

/**
 * Write data to FIFO.
 * If there is not enough space to store all data, no data is stored at all.
 *
 * @param pFifo
 * @param pData Data to write.
 * @param dataSize Number of bytes to write.
 * @return Number of bytes actually written.
 */
size_t FifoWrite(Fifo* pFifo, const void* pData, size_t dataSize);

/**
 * Advance the FIFO write pointer and number of bytes stored.
 * This function is used if data has been externally written to the FIFO (e.g. DMA).
 * If there is not enough space to advance by the specified number of bytes, no operation is done.
 *
 * @param pFifo
 * @param dataSize Number of bytes written.
 * @return Number of bytes actually written.
 */
size_t FifoInject(Fifo* pFifo, size_t dataSize);
