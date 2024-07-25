#include "tim2.h"
#include "ch.h"

TimerSimpleLLD tim2;

void TIM2Init()
{
	RCC->APB1LENR |= RCC_APB1LENR_TIM2EN;
	__DSB();

	TimerSimpleInit(&tim2, TIM2, 99); // => results in 1us timer resolution
}
