#include "led_mono.h"

void LEDMonoInit(LEDMono* pLED, GPIOPin pin, uint8_t isActiveHigh)
{
	pLED->isActiveHigh = isActiveHigh;
	pLED->pin = pin;

	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_OUTPUT;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;

	if(isActiveHigh)
	{
		gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
		gpioInit.pupd = GPIO_PUPD_NONE;
		GPIOPinReset(pin);
	}
	else
	{
		gpioInit.otype = GPIO_OTYPE_OPEN_DRAIN;
		gpioInit.pupd = GPIO_PUPD_UP;
		GPIOPinSet(pin);
	}

	GPIOInit(pin.pPort, pin.pin, &gpioInit);
}

void LEDMonoSet(LEDMono* pLED, uint8_t enable)
{
	if((enable && pLED->isActiveHigh) || (!enable && !pLED->isActiveHigh))
		GPIOPinSet(pLED->pin);
	else
		GPIOPinReset(pLED->pin);
}

uint8_t LEDMonoGet(LEDMono* pLED)
{
	uint8_t isSet = GPIOPinIsSet(pLED->pin);
	if((pLED->isActiveHigh && isSet) || (!pLED->isActiveHigh && !isSet))
		return 1;

	return 0;
}

void LEDMonoToggle(LEDMono* pLED)
{
	pLED->pin.pPort->ODR ^= pLED->pin.pin;
}
