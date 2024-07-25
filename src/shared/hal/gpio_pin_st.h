#pragma once

#include "gpio_pin.h"
#include "hal/init_hal.h"

typedef struct _GPIOPinSt
{
	GPIOPinInterface pin;

	GPIO_TypeDef* pPort;
	uint16_t pinMask;
} GPIOPinSt;

void GPIOPinStInit(GPIOPinSt* pPin, GPIO_TypeDef* pPort, uint16_t pinMask);
