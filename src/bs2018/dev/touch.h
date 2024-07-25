#pragma once

#include "drv/touch_ad7843.h"

typedef struct _Touch
{
	TouchAD7843 sensor;
	SPILLD spiDriver;
	SPI bus;
} Touch;

extern Touch touch;

void TouchInit();
void TouchUpdate();
void TouchTask(void*);
