/*
 * led.c
 *
 *  Created on: 23.10.2017
 *      Author: AndreR
 */

#include "led.h"
#include "util/init_hal.h"

typedef struct _LEDGlobal
{
	uint8_t state;
} LEDGlobal;

static LEDGlobal led;

void LEDInit()
{
	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_OUTPUT;
	gpioInit.otype = GPIO_OTYPE_OPEN_DRAIN;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;
	gpioInit.pupd = GPIO_PUPD_UP;
	GPIOInit(GPIOA, GPIO_PIN_4 | GPIO_PIN_5, &gpioInit); // LED0, LED1

	LEDSet(0);
}

void LEDSet(uint8_t state)
{
	if(state & LED_GREEN)
		GPIOA->BSRR = GPIO_PIN_4 << 16;
	else
		GPIOA->BSRR = GPIO_PIN_4;

	if(state & LED_RED)
		GPIOA->BSRR = GPIO_PIN_5 << 16;
	else
		GPIOA->BSRR = GPIO_PIN_5;

	led.state = state;
}

void LEDMustOn(uint8_t on)
{
	LEDSet(led.state | on);
}

void LEDMustOff(uint8_t off)
{
	LEDSet(led.state & ~(off));
}

void LEDToggle(uint8_t toggle)
{
	LEDSet(led.state ^ toggle);
}

uint8_t LEDGetState()
{
	return led.state;
}
