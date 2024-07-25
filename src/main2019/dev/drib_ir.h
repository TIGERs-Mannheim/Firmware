#pragma once

#include "drv/mcu_dribbler.h"

extern McuDribbler devDribIr;

void DevDribIrInit(UARTFifo* pUart, GPIOPinInterface* pRstPin, GPIOPinInterface* pBootPin, tprio_t taskPrio);
