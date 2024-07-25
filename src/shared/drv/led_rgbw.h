#pragma once

#include "hal/init_hal.h"

typedef struct _LEDRGBWChannel
{
	GPIO_TypeDef* pPort;
	uint16_t pin;
	uint16_t alternate;
	volatile uint32_t* pCCR;
} LEDRGBWChannel;

typedef struct _LEDRGBWData
{
	TIM_TypeDef* pTim;
	LEDRGBWChannel red;
	LEDRGBWChannel green;
	LEDRGBWChannel blue;
	LEDRGBWChannel white;
	uint32_t psc100MHz;
	float scaling;
} LEDRGBWData;

typedef struct _LEDRGBW
{
	float scaling;

	volatile uint32_t* pRed;
	volatile uint32_t* pGreen;
	volatile uint32_t* pBlue;
	volatile uint32_t* pWhite;
} LEDRGBW;

void LEDRGBWInit(LEDRGBW* pLED, LEDRGBWData* pInit);
void LEDRGBWSet(LEDRGBW* pLED, float red, float green, float blue, float white);
