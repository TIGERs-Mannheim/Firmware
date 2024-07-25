#include "leds_front.h"

LEDRGBW devLedFrontLeft;
LEDRGBW devLedFrontRight;

void DevLedsFrontInit()
{
	RCC->APB1LENR |= RCC_APB1LENR_TIM3EN | RCC_APB1LENR_TIM4EN;
	__DSB();

	LEDRGBWData init;
	init.pTim = TIM3;
	init.red = (LEDRGBWChannel){ GPIOC, GPIO_PIN_6, 2, &TIM3->CCR4 };
	init.green = (LEDRGBWChannel){ GPIOC, GPIO_PIN_7, 2, &TIM3->CCR3 };
	init.blue = (LEDRGBWChannel){ GPIOB, GPIO_PIN_0, 2, &TIM3->CCR2 };
	init.white = (LEDRGBWChannel){ GPIOB, GPIO_PIN_1, 2, &TIM3->CCR1 };
	init.psc100MHz = 0;
	init.scaling = 0.1f;

	LEDRGBWInit(&devLedFrontLeft, &init);

	init.pTim = TIM4;
	init.red = (LEDRGBWChannel){ GPIOD, GPIO_PIN_12, 2, &TIM4->CCR4 };
	init.green = (LEDRGBWChannel){ GPIOD, GPIO_PIN_13, 2, &TIM4->CCR3 };
	init.blue = (LEDRGBWChannel){ GPIOB, GPIO_PIN_8, 2, &TIM4->CCR2 };
	init.white = (LEDRGBWChannel){ GPIOB, GPIO_PIN_9, 2, &TIM4->CCR1 };
	init.psc100MHz = 0;
	init.scaling = 0.1f;

	LEDRGBWInit(&devLedFrontRight, &init);
}
