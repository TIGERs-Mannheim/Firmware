#include "spi.h"
#include "errors.h"
#include "util/log.h"
#include "hal/sys_time.h"

static void spiDoneCallback(void* pData)
{
	if(!pData)
	{
		LogWarn("SPI done callback w/o user data");
		return;
	}

	SPISlave* pSlave = (SPISlave*)pData;
	SPI* pSPI = pSlave->pBus;

	pSlave->transferTime_us = SysTimeCycleCounterDiffUSec(pSlave->tTransferStart, SysTimeCycleCounter());

	chSysLockFromISR();
	chBSemSignalI(&pSPI->transferSem);
	chSysUnlockFromISR();
}

void SPIInit(SPI* pSPI, SPILLD* pSPILLD)
{
	pSPI->pBus = pSPILLD;
	pSPILLD->doneCallback = &spiDoneCallback;

	chBSemObjectInit(&pSPI->transferSem, FALSE);
	chMtxObjectInit(&pSPI->busMutex);
}

void SPISlaveInit(SPI* pSPI, SPISlave* pSlave)
{
	SPILLDConfigureCsPin(pSlave->csPin);

	if(pSlave->timeoutTicks == 0)
		pSlave->timeoutTicks = TIME_MS2I(100);

	pSlave->pBus = pSPI;
}

void SPIAcquire(SPISlave* pSlave, void* ppTxBuf, void* ppRxBuf)
{
	SPI* pSPI = pSlave->pBus;

	chMtxLock(&pSPI->busMutex);

	pSPI->pBus->pCallbackData = pSlave;

	if(ppTxBuf)
		*((void**)ppTxBuf) = SPILLDGetTxBuf(pSPI->pBus);

	if(ppRxBuf)
		*((void**)ppRxBuf) = SPILLDGetRxBuf(pSPI->pBus);
}

void SPIRelease(SPISlave* pSlave)
{
	SPI* pSPI = pSlave->pBus;

	pSPI->pBus->pCallbackData = 0;

	chMtxUnlock(&pSPI->busMutex);
}

int16_t SPITransfer(SPISlave* pSlave, uint32_t size)
{
	int16_t result = 0;
	SPI* pSPI = pSlave->pBus;

	// Clear semaphore which indicates transfer is complete
	chBSemWaitTimeout(&pSPI->transferSem, TIME_IMMEDIATE);

	// Reconfigure bus if required (slave changed)
	if(pSlave->pBus->pLastActiveSlave != pSlave)
	{
		result = SPILLDReconfigure(pSPI->pBus, pSlave->prescaler, pSlave->cpol, pSlave->cpha);
		if(result)
			return result;

		pSlave->pBus->pLastActiveSlave = pSlave;
	}

	// Start actual transfer
	pSlave->tTransferStart = SysTimeCycleCounter();

	result = SPILLDStartTransfer(pSPI->pBus, pSlave->csPin, size);
	if(result)
		return result;

	// wait for transfer complete semaphore
	msg_t waitResult = chBSemWaitTimeout(&pSPI->transferSem, pSlave->timeoutTicks);
	if(waitResult != MSG_OK)
	{
		SPILLDAbortTransfer(pSPI->pBus);
		LogError("SPITimeout");
		return ERROR_WAIT_TIMEOUT;
	}

	return 0;
}

uint8_t SPIHasEnoughMemory(SPISlave* pSlave, uint32_t requiredSize)
{
	if(pSlave->pBus->pBus->data.dmaBufSize < requiredSize)
		return 0;

	return 1;
}
