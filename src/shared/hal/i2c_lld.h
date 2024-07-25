/*
 * Low-Level I2C HAL interface.
 *
 * Driver must not use any OS functions and use a callback method.
 */
#pragma once

#include <stdint.h>

#define I2C_RESULT_TRANSFER_COMPLETE	0
#define I2C_RESULT_NACK 				1
#define I2C_RESULT_ERROR				2

typedef void(*I2CLLDDoneCallback)(int16_t result, void* pData);

typedef struct _I2CLLDVTable I2CLLDVTable;

typedef struct _I2CLLD
{
	const I2CLLDVTable* pVTable;

	I2CLLDDoneCallback doneCallback;
	void* pCallbackData;
} I2CLLD;

typedef struct _I2CLLDVTable
{
	void*	(*getTxBuf)(I2CLLD* pI2C);
	void*	(*getRxBuf)(I2CLLD* pI2C);
	uint32_t(*getBufSize)(I2CLLD* pI2C);
	int16_t (*transfer)(I2CLLD* pI2C, uint8_t addr, uint8_t txLength, uint8_t rxLength);
	void    (*abort)(I2CLLD* pI2C);
	uint8_t (*isActive)(I2CLLD* pI2C);
} I2CLLDVTable;
