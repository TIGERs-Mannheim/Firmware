#include "led_rgbw.h"

void LEDRGBWInit(LEDRGBW* pLED, LEDRGBWData* pInit)
{
	pLED->pRed = pInit->red.pCCR;
	pLED->pGreen = pInit->green.pCCR;
	pLED->pBlue = pInit->blue.pCCR;
	pLED->pWhite = pInit->white.pCCR;
	pLED->scaling = pInit->scaling;

	TIM_TypeDef* pTim = pInit->pTim;

	pTim->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 // PWM mode 1
			| TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 // PWM mode 1
			| TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE; // preload enabled on CCR1 and CCR2
	pTim->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 // PWM mode 1
			| TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 // PWM mode 1
			| TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE; // preload enabled on CCR3 and CCR4
	pTim->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
	pTim->PSC = pInit->psc100MHz; // => 100MHz
	pTim->ARR = 9999; // => 10kHz
	pTim->CCR1 = 0;
	pTim->CCR2 = 0;
	pTim->CCR3 = 0;
	pTim->CCR4 = 0;
	pTim->CR1 = TIM_CR1_CEN;

	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.pupd = GPIO_PUPD_NONE;

	gpioInit.alternate = pInit->red.alternate;
	GPIOInit(pInit->red.pPort, pInit->red.pin, &gpioInit);

	gpioInit.alternate = pInit->green.alternate;
	GPIOInit(pInit->green.pPort, pInit->green.pin, &gpioInit);

	gpioInit.alternate = pInit->blue.alternate;
	GPIOInit(pInit->blue.pPort, pInit->blue.pin, &gpioInit);

	gpioInit.alternate = pInit->white.alternate;
	GPIOInit(pInit->white.pPort, pInit->white.pin, &gpioInit);
}

void LEDRGBWSet(LEDRGBW* pLED, float red, float green, float blue, float white)
{
	red *= pLED->scaling;
	green *= pLED->scaling;
	blue *= pLED->scaling;
	white *= pLED->scaling;

	float sum = red + green + blue + white;

	// scale if sum exceeds 1
	if(sum > 1.0f)
	{
		float scale = 1.0f/sum;
		red *= scale;
		green *= scale;
		blue *= scale;
		white *= scale;
	}

	*pLED->pWhite = (uint16_t)(white*9999.0f);
	*pLED->pBlue = (uint16_t)(blue*9999.0f);
	*pLED->pGreen = (uint16_t)(green*9999.0f);
	*pLED->pRed = (uint16_t)(red*9999.0f);
}
