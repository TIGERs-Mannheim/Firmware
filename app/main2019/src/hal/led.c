/*
 * led.c
 *
 *  Created on: 30.12.2018
 *      Author: AndreR
 */

#include "led.h"
#include "ch.h"
#include "util/init_hal.h"
#include <math.h>

typedef struct _LEDGlobal
{
	uint16_t state;
} LEDGlobal;

static LEDGlobal led;

void LEDInit()
{
	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_OUTPUT;
	gpioInit.otype = GPIO_OTYPE_OPEN_DRAIN;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;
	gpioInit.pupd = GPIO_PUPD_UP;
	GPIOInit(GPIOC, GPIO_PIN_0 | GPIO_PIN_3, &gpioInit); // Mainboard LEDs

	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.pupd = GPIO_PUPD_NONE;
	GPIOInit(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_9, &gpioInit);
	GPIOInit(GPIOC, GPIO_PIN_6 | GPIO_PIN_7, &gpioInit);
	GPIOInit(GPIOD, GPIO_PIN_12 | GPIO_PIN_13, &gpioInit);

	LEDSet(0);

	RCC->APB1LENR |= RCC_APB1LENR_TIM3EN | RCC_APB1LENR_TIM4EN;
	__DSB();

	TIM3->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 // PWM mode 1
			| TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 // PWM mode 1
			| TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE; // preload enabled on CCR1 and CCR2
	TIM3->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 // PWM mode 1
			| TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 // PWM mode 1
			| TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE; // preload enabled on CCR3 and CCR4
	TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
	TIM3->PSC = 0; // => 100MHz
	TIM3->ARR = 9999; // => 10kHz
	TIM3->CCR1 = 0;
	TIM3->CCR2 = 0;
	TIM3->CCR3 = 0;
	TIM3->CCR4 = 0;
	TIM3->CR1 = TIM_CR1_CEN;

	TIM4->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 // PWM mode 1
			| TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 // PWM mode 1
			| TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE; // preload enabled on CCR1 and CCR2
	TIM4->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 // PWM mode 1
			| TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 // PWM mode 1
			| TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE; // preload enabled on CCR3 and CCR4
	TIM4->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
	TIM4->PSC = 0; // => 100MHz
	TIM4->ARR = 9999; // => 10kHz
	TIM4->CCR1 = 0;
	TIM4->CCR2 = 0;
	TIM4->CCR3 = 0;
	TIM4->CCR4 = 0;
	TIM4->CR1 = TIM_CR1_CEN;

	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.pupd = GPIO_PUPD_NONE;
	gpioInit.alternate = 2;

	GPIOInit(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_9, &gpioInit);
	GPIOInit(GPIOC, GPIO_PIN_6 | GPIO_PIN_7, &gpioInit);
	GPIOInit(GPIOD, GPIO_PIN_12 | GPIO_PIN_13, &gpioInit);
}

void LEDSet(uint16_t state)
{
	if(state & LED_MAIN_RED)
		GPIOReset(GPIOC, GPIO_PIN_3);
	else
		GPIOSet(GPIOC, GPIO_PIN_3);

	if(state & LED_MAIN_GREEN)
		GPIOReset(GPIOC, GPIO_PIN_0);
	else
		GPIOSet(GPIOC, GPIO_PIN_0);

	led.state = state;
}

void LEDFrontSet(uint16_t state)
{
	uint16_t leftLED = state & LED_FRONT_LEFT_MASK;
	uint16_t rightLED = state & LED_FRONT_RIGHT_MASK;

	float leftRGBW[4] = { 0.0f };
	float rightRGBW[4] = { 0.0f };
	float onValue = 1.0f;

	if(leftLED == LED_FRONT_LEFT_RED)
		leftRGBW[0] = onValue;
	else
		leftRGBW[0] = 0.0f;

	if(leftLED == LED_FRONT_LEFT_GREEN)
		leftRGBW[1] = onValue;
	else
		leftRGBW[1] = 0.0f;

	if(leftLED == LED_FRONT_LEFT_BLUE)
		leftRGBW[2] = onValue;
	else
		leftRGBW[2] = 0.0f;

	if(leftLED == LED_FRONT_LEFT_WHITE)
		leftRGBW[3] = onValue;
	else
		leftRGBW[3] = 0.0f;

	if(rightLED == LED_FRONT_RIGHT_RED)
		rightRGBW[0] = onValue;
	else
		rightRGBW[0] = 0.0f;

	if(rightLED == LED_FRONT_RIGHT_GREEN)
		rightRGBW[1] = onValue;
	else
		rightRGBW[1] = 0.0f;

	if(rightLED == LED_FRONT_RIGHT_BLUE)
		rightRGBW[2] = onValue;
	else
		rightRGBW[2] = 0.0f;

	if(rightLED == LED_FRONT_RIGHT_WHITE)
		rightRGBW[3] = onValue;
	else
		rightRGBW[3] = 0.0f;

	LEDLeftSet(leftRGBW[0], leftRGBW[1], leftRGBW[2], leftRGBW[3]);
	LEDRightSet(rightRGBW[0], rightRGBW[1], rightRGBW[2], rightRGBW[3]);
}

void LEDToggle(uint16_t toggle)
{
	LEDSet(led.state ^ toggle);
}

void LEDLeftSet(float red, float green, float blue, float white)
{
	// TODO: make this scaling configurable
	red *= 0.1f;
	green *= 0.1f;
	blue *= 0.1f;
	white *= 0.1f;

	float sum = red + green + blue + white;

	// scale if maximum power exceeds 1W
	if(sum > 1.0f)
	{
		float scale = 1.0f/sum;
		red *= scale;
		green *= scale;
		blue *= scale;
		white *= scale;
	}

	TIM3->CCR1 = (uint16_t)(white*9999.0f);
	TIM3->CCR2 = (uint16_t)(blue*9999.0f);
	TIM3->CCR3 = (uint16_t)(green*9999.0f);
	TIM3->CCR4 = (uint16_t)(red*9999.0f);
}

void LEDRightSet(float red, float green, float blue, float white)
{
	red *= 0.1f;
	green *= 0.1f;
	blue *= 0.1f;
	white *= 0.1f;

	float sum = red + green + blue + white;

	// scale if maximum power exceeds 1W
	if(sum > 1.0f)
	{
		float scale = 1.0f/sum;
		red *= scale;
		green *= scale;
		blue *= scale;
		white *= scale;
	}

	TIM4->CCR1 = (uint16_t)(white*9999.0f);
	TIM4->CCR2 = (uint16_t)(blue*9999.0f);
	TIM4->CCR3 = (uint16_t)(green*9999.0f);
	TIM4->CCR4 = (uint16_t)(red*9999.0f);
}

void LEDDemo()
{
//	for(float x = 0.0f; x < 3.14f*10; x += 3.14f/10000.0f)
//	{
//		LEDLeftSet(fabsf(sinf(x))*0.1f, 0.0f, 0.0f, 0.0f);
//		chThdSleepMilliseconds(1);
//	}

	LEDFrontSet(LED_FRONT_LEFT_RED | LED_FRONT_RIGHT_RED);
	chThdSleepMilliseconds(500);
	LEDFrontSet(LED_FRONT_LEFT_GREEN | LED_FRONT_RIGHT_GREEN);
	chThdSleepMilliseconds(500);
	LEDFrontSet(LED_FRONT_LEFT_BLUE | LED_FRONT_RIGHT_BLUE);
	chThdSleepMilliseconds(500);
	LEDFrontSet(LED_FRONT_LEFT_WHITE | LED_FRONT_RIGHT_WHITE);
	chThdSleepMilliseconds(500);

	for(uint8_t i = 0; i < 10; i++)
	{
		LEDFrontSet(LED_FRONT_LEFT_RED);
		chThdSleepMilliseconds(100);
		LEDFrontSet(LED_FRONT_RIGHT_RED);
		chThdSleepMilliseconds(100);
	}

	for(uint8_t i = 0; i < 10; i++)
	{
		LEDFrontSet(LED_FRONT_LEFT_GREEN);
		chThdSleepMilliseconds(100);
		LEDFrontSet(LED_FRONT_RIGHT_GREEN);
		chThdSleepMilliseconds(100);
	}

	for(uint8_t i = 0; i < 10; i++)
	{
		LEDFrontSet(LED_FRONT_LEFT_BLUE);
		chThdSleepMilliseconds(100);
		LEDFrontSet(LED_FRONT_RIGHT_BLUE);
		chThdSleepMilliseconds(100);
	}

	for(uint8_t i = 0; i < 10; i++)
	{
		LEDFrontSet(LED_FRONT_LEFT_WHITE);
		chThdSleepMilliseconds(100);
		LEDFrontSet(LED_FRONT_RIGHT_WHITE);
		chThdSleepMilliseconds(100);
	}

	for(uint8_t i = 0; i < 10; i++)
	{
		LEDFrontSet(LED_FRONT_LEFT_RED | LED_FRONT_RIGHT_GREEN);
		chThdSleepMilliseconds(200);
		LEDFrontSet(LED_FRONT_LEFT_GREEN | LED_FRONT_RIGHT_RED);
		chThdSleepMilliseconds(200);
	}

	for(uint8_t i = 0; i < 10; i++)
	{
		LEDFrontSet(LED_FRONT_LEFT_BLUE | LED_FRONT_RIGHT_WHITE);
		chThdSleepMilliseconds(200);
		LEDFrontSet(LED_FRONT_LEFT_WHITE | LED_FRONT_RIGHT_BLUE);
		chThdSleepMilliseconds(200);
	}

	for(uint8_t i = 0; i < 20; i++)
	{
		LEDFrontSet(LED_FRONT_LEFT_RED | LED_FRONT_RIGHT_GREEN);
		chThdSleepMilliseconds(50);
		LEDFrontSet(LED_FRONT_LEFT_GREEN | LED_FRONT_RIGHT_BLUE);
		chThdSleepMilliseconds(50);
		LEDFrontSet(LED_FRONT_LEFT_BLUE | LED_FRONT_RIGHT_WHITE);
		chThdSleepMilliseconds(50);
		LEDFrontSet(LED_FRONT_LEFT_WHITE | LED_FRONT_RIGHT_RED);
		chThdSleepMilliseconds(50);
	}
}
