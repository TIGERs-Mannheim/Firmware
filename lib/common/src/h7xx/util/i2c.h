/*
 * i2c.h
 *
 *  Created on: 06.01.2019
 *      Author: AndreR
 */

#pragma once

#include "ch.h"

typedef struct _I2C
{
	I2C_TypeDef* pRegister;

	mutex_t mtxInUse;

	mailbox_t eventQueue;
	msg_t eventQueueData[10];

	uint8_t* pTxData;
	uint8_t* pRxData;
} I2C;

void I2CEventIRQ(I2C* pI2C);
void I2CErrorIRQ(I2C* pI2C);
void I2CInit(I2C* pI2C, I2C_TypeDef* pI2CRegister);
int16_t I2CWrite(I2C* pI2C, uint8_t addr, uint8_t length, uint8_t* pData);
int16_t I2CRead(I2C* pI2C, uint8_t addr, uint8_t length, uint8_t* pData);
