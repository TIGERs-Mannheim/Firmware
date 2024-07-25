#include "timer_opm.h"

#define OCM_DISABLED (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_0)
#define OCM_PWM (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1)

void TimerOpmInit(TimerOpm* pTimer, TIM_TypeDef* pTim, GPIOPinAlt pin, uint32_t prescaler)
{
	GPIOInitData gpioInit;

	chEvtObjectInit(&pTimer->eventSource);

	pTimer->pTim = pTim;

	pTim->CR1 = TIM_CR1_OPM;	// one pulse mode
	pTim->PSC = prescaler;
	pTim->ARR = 0xFFFF;
	pTim->CCMR1 = OCM_DISABLED;
	pTim->CCR1 = 1;
	pTim->CCER = TIM_CCER_CC1E;
	pTim->EGR = TIM_EGR_UG;
	pTim->SR = 0;
	pTim->DIER = TIM_DIER_UIE;

	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.pupd = GPIO_PUPD_UP;
	gpioInit.ospeed = GPIO_OSPEED_25MHZ;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.alternate = pin.alternate;
	GPIOInit(pin.pPort, pin.pin, &gpioInit);
}

void TimerOpmIRQ(TimerOpm* pTimer)
{
	pTimer->pTim->SR = 0;

	chSysLockFromISR();
	chEvtBroadcastI(&pTimer->eventSource);
	chSysUnlockFromISR();
}

void TimerOpmPulse(TimerOpm* pTimer, uint32_t duration)
{
	TIM_TypeDef* pTim = pTimer->pTim;

	pTim->CCMR1 = OCM_PWM;
	pTim->ARR = duration;
	pTim->CR1 |= TIM_CR1_CEN;
}
