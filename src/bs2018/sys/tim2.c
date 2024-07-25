#include "tim2.h"
#include "ch.h"

TimerSimpleLLD tim2;

void TIM2Init()
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	TimerSimpleInit(&tim2, TIM2, 215); // => results in 1us timer resolution
}
