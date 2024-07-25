#include "timer_trigger_lld.h"

void TimerTriggerInit(TimerTriggerLLD* pTimer, TIM_TypeDef* pTim, uint32_t prescaler)
{
	pTimer->pTim = pTim;

	pTim->CR1 = TIM_CR1_OPM;	// one pulse mode
	pTim->PSC = prescaler;
	pTim->ARR = 0xFFFF;
	pTim->CCMR1 = 0;
	pTim->CCR1 = 0;
	pTim->CCER = 0;
	pTim->EGR = TIM_EGR_UG;
	pTim->SR = 0;
	pTim->DIER = TIM_DIER_UIE;
}

void TimerTriggerIRQ(TimerTriggerLLD* pTimer)
{
	pTimer->pTim->SR = 0;

	if(pTimer->pCallback)
		(*pTimer->pCallback)(pTimer->pUser);
}

void TimerTriggerStart(TimerTriggerLLD* pTimer, uint32_t duration)
{
	TIM_TypeDef* pTim = pTimer->pTim;
	pTim->CR1 &= ~TIM_CR1_CEN;
	pTim->CNT = 0;
	pTim->ARR = duration;
	pTim->CR1 |= TIM_CR1_CEN;
}

void TimerTriggerAbort(TimerTriggerLLD* pTimer)
{
	TIM_TypeDef* pTim = pTimer->pTim;
	pTim->CR1 &= ~TIM_CR1_CEN;
}
