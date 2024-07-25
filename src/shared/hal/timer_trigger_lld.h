/*
 * Simple timer which triggers after a defined time.
 *
 * Does not use any OS functions.
 */
#pragma once

#include "hal/init_hal.h"

typedef void (*TimerTriggerLLDCallback)(void*);

typedef struct _TimerTriggerLLD
{
	TIM_TypeDef* pTim;

	TimerTriggerLLDCallback pCallback;
	void* pUser;
} TimerTriggerLLD;

void TimerTriggerInit(TimerTriggerLLD* pTimer, TIM_TypeDef* pTim, uint32_t prescaler);
void TimerTriggerIRQ(TimerTriggerLLD* pTimer);
void TimerTriggerStart(TimerTriggerLLD* pTimer, uint32_t duration);
void TimerTriggerAbort(TimerTriggerLLD* pTimer);
