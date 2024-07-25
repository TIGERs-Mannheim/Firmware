#pragma once

#include "hal/spi.h"
#include "util/device_profiler.h"

#define ADC_KICKER_SAMPLES 16

typedef struct _ADCKickerMeasurement
{
	uint32_t timestamp_us;

	float capLevel_V;
	float boardTemp_degC;
} ADCKickerMeasurement;

typedef struct _ADCKicker
{
	SPISlave spiSlave;
	uint8_t tx[4*ADC_KICKER_SAMPLES];

	ADCKickerMeasurement meas;
	mutex_t measMutex;

	DeviceProfiler profiler;

	THD_WORKING_AREA(waTask, 256);
	thread_t* pTask;

	event_source_t eventSource;
} ADCKicker;


void ADCKickerInit(ADCKicker* pAdc, SPI* pSPI, GPIOPin csPin, uint32_t prescaler, tprio_t prio);
void ADCKickerGet(ADCKicker* pAdc, ADCKickerMeasurement* pMeas);
