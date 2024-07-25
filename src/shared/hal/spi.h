/*
 * OS SPI layer.
 *
 * Offers synchronous SPI transfers with timeouts.
 * Safe to use from multiple tasks but will block
 * if bus is already acquired.
 */
#pragma once

#include "spi_lld.h"
#include "ch.h"

typedef struct _SPISlave SPISlave;

typedef struct _SPI
{
	SPILLD* pBus;
	SPISlave* pLastActiveSlave;

	binary_semaphore_t transferSem;
	mutex_t busMutex;
} SPI;

typedef struct _SPISlave
{
	SPI* pBus;

	GPIOPin csPin;

	uint32_t prescaler;
	uint32_t cpol;
	uint32_t cpha;

	volatile uint32_t tTransferStart;
	volatile uint32_t transferTime_us;

	uint32_t timeoutTicks;
} SPISlave;

void	SPIInit(SPI* pSPI, SPILLD* pSPILLD);
void	SPISlaveInit(SPI* pSPI, SPISlave* pSlave);
uint8_t SPIHasEnoughMemory(SPISlave* pSlave, uint32_t requiredSize);

void	SPIAcquire(SPISlave* pSlave, void* ppTxBuf, void* ppRxBuf);
int16_t SPITransfer(SPISlave* pSlave, uint32_t size);
void	SPIRelease(SPISlave* pSlave);

static inline uint32_t SPIGetTransferTime(SPISlave* pSlave) { return pSlave->transferTime_us; }
