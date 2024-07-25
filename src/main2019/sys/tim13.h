#pragma once

#include "hal/timer_opm.h"

extern TimerOpm tim13;

void TIM13Init(uint32_t irqLevel);
