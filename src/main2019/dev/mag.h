#pragma once

#include "drv/mag_lis3.h"

extern MagLIS3 devMag;

void DevMagInit(SPI* pSPI, tprio_t taskPrio);
