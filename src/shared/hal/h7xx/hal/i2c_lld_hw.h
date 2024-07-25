/*
 * Low-Level I2C HAL interface based on specialized peripheral.
 */
#pragma once

#include "hal/i2c_lld.h"
#include "hal/init_hal.h"

typedef struct _I2CLLDHWData
{
	I2C_TypeDef* pRegister;

	GPIOPinAlt sclPin;
	GPIOPinAlt sdaPin;

	uint8_t* pBufTx;
	uint8_t* pBufRx;
	uint32_t bufSize;
} I2CLLDHWData;

typedef struct _I2CLLDHW
{
	I2CLLD base;

	I2CLLDHWData data;

	const uint8_t* pTxData;
	uint8_t* pRxData;
    uint32_t rxDataLength;

    volatile uint8_t isActive;
} I2CLLDHW;

void I2CLLDHWEventIRQ(I2CLLDHW* pI2C);
void I2CLLDHWErrorIRQ(I2CLLDHW* pI2C);
void I2CLLDHWInit(I2CLLDHW* pI2C, I2CLLDHWData* pInit);
