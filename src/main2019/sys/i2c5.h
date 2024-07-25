#pragma once

#include "hal/i2c.h"

extern I2C i2c5;

void I2C5Init(uint32_t irqLevel);
