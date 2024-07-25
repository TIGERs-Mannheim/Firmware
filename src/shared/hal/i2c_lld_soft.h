/*
 * Low-Level I2C HAL interface based on software implementation
 * with a fast timer IRQ.
 */
#pragma once

#include "hal/i2c_lld.h"
#include "hal/init_hal.h"

typedef struct _I2CLLDSoftData
{
	TIM_TypeDef* pTimer;
	GPIOPin sclPin;
	GPIOPin sdaPin;

	uint8_t* pBufTx;
	uint8_t* pBufRx;
	uint32_t bufSize;
} I2CLLDSoftData;

typedef struct _I2CSoft
{
	I2CLLD base;

	I2CLLDSoftData data;

	volatile uint8_t addr;
	volatile const uint8_t* pTxData;
	volatile const uint8_t* pTxDataEnd;
	volatile uint8_t* pRxData;
	volatile const uint8_t* pRxDataEnd;

	volatile uint8_t hlState;
	volatile uint8_t opState;
	int8_t bitCounter;
	volatile  int16_t transferResult;

	volatile uint8_t abortPending;
} I2CLLDSoft;

void I2CLLDSoftTick(I2CLLDSoft* pI2C);
void I2CLLDSoftInit(I2CLLDSoft* pI2C, I2CLLDSoftData* pInit);
