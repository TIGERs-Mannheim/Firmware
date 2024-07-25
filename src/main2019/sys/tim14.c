#include "tim14.h"

TimerOpm tim14;

CH_IRQ_HANDLER(VectorF4) // TIM8_TRG_COM_TIM14, TIM14
{
	CH_IRQ_PROLOGUE();
	TimerOpmIRQ(&tim14);
	CH_IRQ_EPILOGUE();
}

void TIM14Init(uint32_t irqLevel)
{
	RCC->APB1LENR |= RCC_APB1LENR_TIM14EN;
	__DSB();

	TimerOpmInit(&tim14, TIM14, (GPIOPinAlt){ GPIOF, GPIO_PIN_9, 9}, 99);

	NVICEnableIRQ(TIM8_TRG_COM_TIM14_IRQn, irqLevel);
}
