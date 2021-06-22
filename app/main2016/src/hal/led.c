/*
 * led.c
 *
 *  Created on: 24.10.2015
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
	GPIOInit(GPIOF, GPIO_PIN_3 | GPIO_PIN_15, &gpioInit); // LED1, LED4
	GPIOInit(GPIOC, GPIO_PIN_15, &gpioInit); // LED2
	GPIOInit(GPIOG, GPIO_PIN_15, &gpioInit); // LED3)

	LEDSet(0);
}

void LEDSet(uint8_t state)
{
	if(state & LED_LEFT_RED)
		GPIOF->BSRR = GPIO_PIN_3 << 16;
	else
		GPIOF->BSRR = GPIO_PIN_3;

	if(state & LED_LEFT_GREEN)
		GPIOC->BSRR = GPIO_PIN_15 << 16;
	else
		GPIOC->BSRR = GPIO_PIN_15;

	if(state & LED_RIGHT_RED)
		GPIOF->BSRR = GPIO_PIN_15 << 16;
	else
		GPIOF->BSRR = GPIO_PIN_15;

	if(state & LED_RIGHT_GREEN)
		GPIOG->BSRR = GPIO_PIN_15 << 16;
	else
		GPIOG->BSRR = GPIO_PIN_15;

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
