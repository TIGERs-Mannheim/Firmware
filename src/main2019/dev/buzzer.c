#include "buzzer.h"

Buzzer devBuzzer;

void DevBuzzerInit()
{
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;	// enable TIM1 clock
	__DSB();

	BuzzerData init;
	init.outputPin = (GPIOPinAlt){ GPIOA, GPIO_PIN_8, 1 };
	init.pTim = TIM1;
	init.psc1MHz = 99;
	init.pCCR = &TIM1->CCR1;
	init.pCCMR = &TIM1->CCMR1;
	init.ccmrBuzzerOn = TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	init.ccmrBuzzerOff = TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_2;

	BuzzerInit(&devBuzzer, &init);
}
