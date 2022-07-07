/*
 * tca9548a.h
 *
 *  Created on: 13.03.2020
 *      Author: AndreR
 *
 * I2C multiplexer TCA9548A driver.
 */

#pragma once

#include "i2c_soft.h"

typedef struct _TCA9548A
{
	I2CSoft* pI2C;

	uint8_t channels;
} TCA9548A;

void TCA9548AInit(TCA9548A* pTca, I2CSoft* pI2C);
int16_t TCA9548ASetChannels(TCA9548A* pTca, uint8_t ch);
