#pragma once

#include "hal/spi.h"
#include "util/device_profiler.h"

#define AD7843_TOUCH_SAMPLES 16

typedef struct _TouchAD7843Measurement
{
	uint32_t timestamp_us;

	uint8_t pressed;
	uint16_t x;
	uint16_t y;
} TouchAD7843Measurement;

typedef struct _TouchAD7843
{
	SPISlave spiSlave;
	GPIOPin irqPin;

	uint8_t tx[2*2*AD7843_TOUCH_SAMPLES+2];
	uint8_t inv;

	TouchAD7843Measurement meas;

	DeviceProfiler profiler;

	THD_WORKING_AREA(waTask, 256);
	thread_t* pTask;

	mutex_t measMutex;
} TouchAD7843;

void TouchAD7843Init(TouchAD7843* pTouch, SPI* pSPI, GPIOPin csPin, GPIOPin irqPin, uint32_t prescaler, tprio_t prio);
void TouchAD7843Get(TouchAD7843* pTouch, TouchAD7843Measurement* pMeas);
