#pragma once

#include "hal/timer_opm.h"

extern TimerOpm tim14;

void TIM14Init(uint32_t irqLevel);
