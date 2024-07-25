#pragma once

#include "drv/pwr_ina226.h"

extern PwrINA226 devPowerMon;

void DevPowerMonInit(I2C* pI2C, tprio_t taskPrio);
