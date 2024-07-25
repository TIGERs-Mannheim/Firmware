#pragma once

#include "drv/mpu_ext.h"

extern MpuExt devRaspberryPi;

void DevRaspberryPiInit(UartDma* pUart, tprio_t taskPrio);
