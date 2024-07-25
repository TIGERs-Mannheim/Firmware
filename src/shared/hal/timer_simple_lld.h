/*
 * Simple timer which triggers after a defined time or periodically.
 *
 * Does not use any OS functions.
 */
#pragma once

#include "hal/init_hal.h"

typedef void (*TimerSimpleLLDCallback)(void*);

typedef struct _TimerSimpleLLD
{
	TIM_TypeDef* pTim;

	TimerSimpleLLDCallback pCallback;
	void* pUser;
} TimerSimpleLLD;

void TimerSimpleInit(TimerSimpleLLD* pTimer, TIM_TypeDef* pTim, uint32_t prescaler);
void TimerSimpleIRQ(TimerSimpleLLD* pTimer);
void TimerSimpleStartPulse(TimerSimpleLLD* pTimer, uint32_t duration);
void TimerSimpleStartPeriodic(TimerSimpleLLD* pTimer, uint32_t duration);
void TimerSimpleStartPeriodicEx(TimerSimpleLLD* pTimer, uint32_t duration, uint32_t durationToFirstTrigger);
void TimerSimpleStop(TimerSimpleLLD* pTimer);
