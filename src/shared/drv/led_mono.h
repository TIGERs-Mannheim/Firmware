#pragma once

#include "hal/init_hal.h"

typedef struct _LEDMono
{
	GPIOPin pin;
	uint8_t isActiveHigh;
} LEDMono;

void LEDMonoInit(LEDMono* pLED, GPIOPin pin, uint8_t isActiveHigh);
void LEDMonoSet(LEDMono* pLED, uint8_t enable);
uint8_t LEDMonoGet(LEDMono* pLED);
void LEDMonoToggle(LEDMono* pLED);
