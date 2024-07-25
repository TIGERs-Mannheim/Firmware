#include "timer_simple_lld.h"

void TimerSimpleInit(TimerSimpleLLD* pTimer, TIM_TypeDef* pTim, uint32_t prescaler)
{
	pTimer->pTim = pTim;

	pTim->CR1 = 0;
	pTim->PSC = prescaler;
	pTim->ARR = 0xFFFF;
	pTim->CCMR1 = 0;
	pTim->CCR1 = 0;
	pTim->CCER = 0;
	pTim->EGR = TIM_EGR_UG;
	pTim->SR = 0;
	pTim->DIER = TIM_DIER_UIE;
}

void TimerSimpleIRQ(TimerSimpleLLD* pTimer)
{
	if(pTimer->pTim->SR)
	{
		pTimer->pTim->SR = 0;

		if(pTimer->pCallback)
			(*pTimer->pCallback)(pTimer->pUser);
	}
}

void TimerSimpleStartPulse(TimerSimpleLLD* pTimer, uint32_t duration)
{
	if(duration > 1)
		duration--;

	TIM_TypeDef* pTim = pTimer->pTim;
	pTim->CR1 = 0;
	pTim->CNT = 0;
	pTim->ARR = duration;
	pTim->CR1 = TIM_CR1_OPM | TIM_CR1_CEN;
}

void TimerSimpleStartPeriodic(TimerSimpleLLD* pTimer, uint32_t duration)
{
	if(duration > 1)
		duration--;

	TIM_TypeDef* pTim = pTimer->pTim;
	pTim->CR1 = 0;
	pTim->CNT = 0;
	pTim->ARR = duration;
	pTim->CR1 = TIM_CR1_CEN;
}

void TimerSimpleStartPeriodicEx(TimerSimpleLLD* pTimer, uint32_t duration, uint32_t durationToFirstTrigger)
{
	if(durationToFirstTrigger == 0)
		durationToFirstTrigger = 1;
	else if(durationToFirstTrigger > 1)
		durationToFirstTrigger--;

	if(duration > 1)
		duration--;

	TIM_TypeDef* pTim = pTimer->pTim;
	pTim->CR1 = TIM_CR1_ARPE;
	pTim->DIER = 0;
	pTim->CNT = 0;
	pTim->ARR = durationToFirstTrigger;
	pTim->EGR = TIM_EGR_UG;
	pTim->ARR = duration;
	pTim->SR = 0;
	pTim->DIER = TIM_DIER_UIE;
	pTim->CR1 |= TIM_CR1_CEN;
}

void TimerSimpleStop(TimerSimpleLLD* pTimer)
{
	TIM_TypeDef* pTim = pTimer->pTim;
	pTim->CR1 = 0;
}
