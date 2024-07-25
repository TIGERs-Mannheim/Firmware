#pragma once

#include "hal/init_hal.h"
#include "ch.h"

typedef struct _TimerOpm
{
	TIM_TypeDef* pTim;

	event_source_t eventSource;
} TimerOpm;

void TimerOpmInit(TimerOpm* pTimer, TIM_TypeDef* pTim, GPIOPinAlt pin, uint32_t prescaler);
void TimerOpmIRQ(TimerOpm* pTimer);
void TimerOpmPulse(TimerOpm* pTimer, uint32_t duration);
