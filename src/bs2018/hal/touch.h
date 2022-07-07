/*
 * touch.h
 *
 *  Created on: 02.11.2017
 *      Author: AndreR
 */

#pragma once

#include "util/spi_sync.h"
#include "util/ad7843.h"

typedef struct _Touch
{
	AD7843 sensor;
	SPISync bus;
} Touch;

extern Touch touch;

void TouchInit();
void TouchUpdate();
void TouchTask(void* params);
