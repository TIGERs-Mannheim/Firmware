#pragma once

#include <stdint.h>

typedef struct _GPIOPinInterface GPIOPinInterface;

typedef struct _GPIOPinInterface
{
	void (*write)(GPIOPinInterface* pGPIO, uint8_t level);
	uint8_t (*read)(GPIOPinInterface* pGPIO);
	void (*configure)(GPIOPinInterface* pGPIO, uint8_t isOutput);
} GPIOPinInterface;
