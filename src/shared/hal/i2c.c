#include "i2c.h"
#include "errors.h"
#include "util/log.h"

static void transferDoneCallback(int16_t result, void* pData)
{
	I2C* pI2C = (I2C*)pData;

	pI2C->transferResult = result;

	chSysLockFromISR();
	chBSemSignalI(&pI2C->transferSem);
	chSysUnlockFromISR();
}

void I2CInit(I2C* pI2C, I2CLLD* pDriver)
{
	pI2C->pDriver = pDriver;
	pI2C->pDriver->doneCallback = &transferDoneCallback;
	pI2C->pDriver->pCallbackData = pI2C;

	chBSemObjectInit(&pI2C->transferSem, FALSE);
	chMtxObjectInit(&pI2C->busMutex);

	pI2C->timeoutTicks = TIME_MS2I(100);
}

uint8_t I2CHasEnoughMemory(I2C* pI2C, uint32_t requiredSize)
{
	if((*pI2C->pDriver->pVTable->getBufSize)(pI2C->pDriver) < requiredSize)
		return 0;

	return 1;
}

void I2CAcquire(I2C* pI2C, void* ppTxBuf, void* ppRxBuf)
{
	chMtxLock(&pI2C->busMutex);

	if(ppTxBuf)
		*((void**)ppTxBuf) = (*pI2C->pDriver->pVTable->getTxBuf)(pI2C->pDriver);

	if(ppRxBuf)
		*((void**)ppRxBuf) = (*pI2C->pDriver->pVTable->getRxBuf)(pI2C->pDriver);
}

void I2CRelease(I2C* pI2C)
{
	chMtxUnlock(&pI2C->busMutex);
}

int16_t I2CTransfer(I2C* pI2C, uint8_t addr, uint8_t txLength, uint8_t rxLength)
{
	if(pI2C->busMutex.owner != chThdGetSelfX())
		chSysHalt("I2CTransfer call without bus being acquired.");

	// Clear semaphore which indicates transfer is complete
	chBSemWaitTimeout(&pI2C->transferSem, TIME_IMMEDIATE);

	// Start transfer
	int16_t result = (*pI2C->pDriver->pVTable->transfer)(pI2C->pDriver, addr, txLength, rxLength);
	if(result)
		return result;

	// wait for transfer complete semaphore
	msg_t waitResult = chBSemWaitTimeout(&pI2C->transferSem, pI2C->timeoutTicks);
	if(waitResult != MSG_OK)
	{
		(*pI2C->pDriver->pVTable->abort)(pI2C->pDriver);
		LogError("I2CTimeout");
		return ERROR_WAIT_TIMEOUT;
	}

	return pI2C->transferResult;
}

int16_t I2CWrite(I2C* pI2C, uint8_t addr, uint8_t txLength)
{
	return I2CTransfer(pI2C, addr, txLength, 0);
}

int16_t I2CRead(I2C* pI2C, uint8_t addr, uint8_t rxLength)
{
	return I2CTransfer(pI2C, addr, 0, rxLength);
}
