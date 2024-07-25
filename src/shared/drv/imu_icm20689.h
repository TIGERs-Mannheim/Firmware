#pragma once

#include "hal/spi.h"
#include "util/device_profiler.h"

typedef struct _ImuICM20689Measurement
{
	uint32_t timestamp_us;

	float acc_mDs2[3];	// [m/s^2]
	float gyr_radDs[3];	// [rad/s]
	float temp_degC; 	// [deg C]
} ImuICM20689Measurement;

typedef struct _ImuICM20689
{
	SPISlave spiSlave;
	ImuICM20689Measurement meas;

	DeviceProfiler profiler;

	THD_WORKING_AREA(waTask, 256);
	thread_t* pTask;

	mutex_t measMutex;
} ImuICM20689;

void ImuICM20689Init(ImuICM20689* pImu, SPI* pSPI, GPIOPin csPin, uint32_t prescaler, tprio_t prio);
void ImuICM20689Get(ImuICM20689* pImu, ImuICM20689Measurement* pMeas);
