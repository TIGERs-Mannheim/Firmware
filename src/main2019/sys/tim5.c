#include "tim5.h"
#include "ch.h"

TimerSimpleLLD tim5;

CH_FAST_IRQ_HANDLER(Vector108) // TIM5
{
	TimerSimpleIRQ(&tim5);
}

void TIM5Init(uint32_t irqLevel)
{
	RCC->APB1LENR |= RCC_APB1LENR_TIM5EN;
	__DSB();

	TimerSimpleInit(&tim5, TIM5, 99); // => results in 1us timer resolution

	NVICEnableIRQ(TIM5_IRQn, irqLevel);
}
