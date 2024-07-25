#pragma once

#include "hal/i2c.h"
#include "util/device_profiler.h"

#define PWR_INA226_EVENT_MEAS_UPDATED EVENT_MASK(0)

typedef struct _PwrINA226Measurement
{
	uint32_t timestamp_us;

	float shunt_mV;
	float bus_V;
} PwrINA226Measurement;

typedef struct _PwrINA226
{
	I2C* pBus;
	PwrINA226Measurement meas;

	DeviceProfiler profiler;

	THD_WORKING_AREA(waTask, 256);
	thread_t* pTask;

	mutex_t measMutex;

	event_source_t eventSource;
} PwrINA226;

void PwrINA226Init(PwrINA226* pPwr, I2C* pBus, tprio_t prio);
void PwrINA226Get(PwrINA226* pPwr, PwrINA226Measurement* pMeas);
