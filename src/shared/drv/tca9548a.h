/*
 * Driver for I2C multiplexer TCA9548A.
 */

#pragma once

#include "hal/i2c.h"

typedef struct _TCA9548A
{
	I2C* pI2C;

	uint8_t channels;
} TCA9548A;

void TCA9548AInit(TCA9548A* pTca, I2C* pI2C);
int16_t TCA9548ASetChannels(TCA9548A* pTca, uint8_t ch);
