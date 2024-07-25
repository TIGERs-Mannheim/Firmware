#pragma once

#include <stdint.h>

typedef struct _DeviceProfiler
{
	// Configuration
	float refreshPeriod_s;

	// Temporary values
	uint32_t tStart;
	uint32_t numUpdates;
	uint32_t tLastRateUpdate_us;

	// Result
	uint32_t ioDuration_us;
	float updateRate_Hz;
} DeviceProfiler;

void DeviceProfilerInit(DeviceProfiler* pProfiler, float refreshPeriod_s);
void DeviceProfilerBegin(DeviceProfiler* pProfiler);
void DeviceProfilerEnd(DeviceProfiler* pProfiler);
