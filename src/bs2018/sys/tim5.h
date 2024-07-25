#pragma once

#include "hal/timer_simple_lld.h"

extern TimerSimpleLLD tim5;

void TIM5Init(uint32_t irqLevel);
