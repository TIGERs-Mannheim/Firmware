/*
 * i2c_soft.h
 *
 *  Created on: 13.03.2020
 *      Author: AndreR
 */

#pragma once

#include "ch.h"
#include "util/init_hal.h"

typedef struct _I2CSoft
{
	binary_semaphore_t semTick;

	GPIO_TypeDef* pSCLPort;
	uint16_t sclPin;

	GPIO_TypeDef* pSDAPort;
	uint16_t sdaPin;

	uint8_t timeoutOccured;
} I2CSoft;

void I2CSoftTick(I2CSoft* pI2C);
void I2CSoftInit(I2CSoft* pI2C, GPIOPin sclPin, GPIOPin sdaPin);
int16_t I2CSoftWrite(I2CSoft* pI2C, uint8_t addr, uint8_t length, uint8_t* pData);
int16_t I2CSoftRead(I2CSoft* pI2C, uint8_t addr, uint8_t length, uint8_t* pData);
