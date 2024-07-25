#include "device_profiler.h"
#include "hal/sys_time.h"

void DeviceProfilerInit(DeviceProfiler* pProfiler, float refreshPeriod_s)
{
	pProfiler->refreshPeriod_s = refreshPeriod_s;
	pProfiler->tLastRateUpdate_us = SysTimeUSec();
	pProfiler->numUpdates = 0;
}

void DeviceProfilerBegin(DeviceProfiler* pProfiler)
{
	pProfiler->tStart = SysTimeCycleCounter();
}

void DeviceProfilerEnd(DeviceProfiler* pProfiler)
{
	++pProfiler->numUpdates;

	uint32_t tNow_us = SysTimeUSec();
	float dt_s = (tNow_us - pProfiler->tLastRateUpdate_us)*1e-6f;
	if(dt_s >= pProfiler->refreshPeriod_s)
	{
		pProfiler->tLastRateUpdate_us = tNow_us;

		pProfiler->updateRate_Hz = (float)pProfiler->numUpdates / dt_s;
		pProfiler->numUpdates = 0;
	}

	pProfiler->ioDuration_us = SysTimeCycleCounterDiffUSec(pProfiler->tStart, SysTimeCycleCounter());
}
