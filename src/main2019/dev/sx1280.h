#pragma once

#include "drv/sx1280_lld.h"

extern SX1280LLD devSX1280;

void DevSX1280Init(SPILLD* pSPI, TimerSimpleLLD* pTimer1us, uint32_t irqLevelHighPrio, uint32_t irqLevelPin);
