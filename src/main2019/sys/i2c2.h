#pragma once

#include "hal/i2c.h"

extern I2C i2c2;

void I2C2Init(uint32_t irqLevel);
