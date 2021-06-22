/*
 * system_init.h
 *
 *  Created on: 14.01.2019
 *      Author: AndreR
 */

#pragma once

#include "stm32f031x6.h"

void SystemInit(uint32_t systemClock, uint32_t sysTickFreq);
void SystemWaitMs(uint32_t timeMs);
uint32_t SystemMeasureIdleIncrements();
uint32_t SystemWaitUntilSystick();

//#################### GPIO ###################
#define GPIO_PIN_0			0x0001
#define GPIO_PIN_1			0x0002
#define GPIO_PIN_2			0x0004
#define GPIO_PIN_3			0x0008
#define GPIO_PIN_4			0x0010
#define GPIO_PIN_5			0x0020
#define GPIO_PIN_6			0x0040
#define GPIO_PIN_7			0x0080
#define GPIO_PIN_8			0x0100
#define GPIO_PIN_9			0x0200
#define GPIO_PIN_10			0x0400
#define GPIO_PIN_11			0x0800
#define GPIO_PIN_12			0x1000
#define GPIO_PIN_13			0x2000
#define GPIO_PIN_14			0x4000
#define GPIO_PIN_15			0x8000

#define GPIO_MODE_INPUT		0
#define GPIO_MODE_OUTPUT	1
#define GPIO_MODE_AF		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_EXTI		4
#define GPIO_MODE_EXTI_AF	5

#define GPIO_OTYPE_PUSH_PULL	0
#define GPIO_OTYPE_OPEN_DRAIN	1

#define GPIO_OSPEED_2MHZ	0
#define GPIO_OSPEED_10MHZ	1
#define GPIO_OSPEED_50MHZ	3

#define GPIO_PUPD_NONE		0
#define GPIO_PUPD_UP		1
#define GPIO_PUPD_DOWN		2

#define GPIO_EXTI_TRIG_RISING 	0x01
#define GPIO_EXTI_TRIG_FALLING	0x02
#define GPIO_EXTI_TRIG_RISING_FALLING	(GPIO_EXTI_TRIG_RISING | GPIO_EXTI_TRIG_FALLING)

typedef struct _GPIOPin
{
	GPIO_TypeDef* pPort;
	uint16_t pin;
} GPIOPin;

typedef struct _GPIOInit
{
	uint32_t mode;
	uint32_t otype;
	uint32_t ospeed;	// not used in input and analog mode
	uint32_t pupd;
	uint32_t alternate;
	uint32_t extiTrigger;
} GPIOInitData;

void GPIOInit(GPIO_TypeDef* pGPIO, uint16_t pins, GPIOInitData* pInit);
#define GPIOSet(pGPIO, mask) (pGPIO->BSRR = ((mask) & 0xFFFF))
#define GPIOReset(pGPIO, mask) (pGPIO->BSRR = (((uint32_t)mask) << 16))
