#pragma once

#include "i2c_lld.h"
#include "ch.h"

typedef struct _I2C
{
	I2CLLD* pDriver;

	binary_semaphore_t transferSem;
	mutex_t busMutex;

	volatile int16_t transferResult;

	sysinterval_t timeoutTicks;
} I2C;

void	I2CInit(I2C* pI2C, I2CLLD* pDriver);
uint8_t	I2CHasEnoughMemory(I2C* pI2C, uint32_t requiredSize);

void	I2CAcquire(I2C* pI2C, void* ppTxBuf, void* ppRxBuf);
int16_t	I2CTransfer(I2C* pI2C, uint8_t addr, uint8_t txLength, uint8_t rxLength);
int16_t	I2CWrite(I2C* pI2C, uint8_t addr, uint8_t txLength);
int16_t	I2CRead(I2C* pI2C, uint8_t addr, uint8_t rxLength);
void	I2CRelease(I2C* pI2C);
