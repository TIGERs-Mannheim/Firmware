#pragma once

#include "hal/spi.h"
#include "util/device_profiler.h"

typedef struct _MagLIS3Measurement
{
	uint32_t timestamp_us;

	float strength_uT[3];	// [uT]
	float temp_degC;		// [deg C]
} MagLIS3Measurement;

typedef struct _MagLIS3
{
	SPISlave spiSlave;
	MagLIS3Measurement meas;

	DeviceProfiler profiler;

	THD_WORKING_AREA(waTask, 256);
	thread_t* pTask;

	mutex_t measMutex;
} MagLIS3;

void MagLIS3Init(MagLIS3* pMag, SPI* pSPI, GPIOPin csPin, uint32_t prescaler, tprio_t prio);
void MagLIS3Get(MagLIS3* pMag, MagLIS3Measurement* pMeas);
