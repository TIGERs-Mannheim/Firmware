#include "tim13.h"

TimerOpm tim13;

CH_IRQ_HANDLER(VectorF0) // TIM8_UP_TIM13, TIM13
{
	CH_IRQ_PROLOGUE();
	TimerOpmIRQ(&tim13);
	CH_IRQ_EPILOGUE();
}

void TIM13Init(uint32_t irqLevel)
{
	RCC->APB1LENR |= RCC_APB1LENR_TIM13EN;
	__DSB();

	TimerOpmInit(&tim13, TIM13, (GPIOPinAlt){ GPIOF, GPIO_PIN_8, 9}, 99);

	NVICEnableIRQ(TIM8_UP_TIM13_IRQn, irqLevel);
}
