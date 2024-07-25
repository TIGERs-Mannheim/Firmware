#include "sys_time.h"
#include "hal/init_hal.h"

SysTimeGlobal sysTime;

void SysTimeInit(TimerSimpleLLD* pTimer1us)
{
	sysTime.pTimer1us = pTimer1us;
	sysTime.cpuClock_MHz = systemClockInfo.SYSClk / 1000000UL;

	TimerSimpleStartPeriodic(pTimer1us, 0xFFFFFFFF);
}

// microsecond precision, integer arithmetic => fast + recommended
unsigned long SysTimeUSec()
{
	return sysTime.pTimer1us->pTim->CNT; // [us]
}

uint32_t SysTimeUnix()
{
	return TIME_I2S(chVTGetSystemTimeX()) + sysTime.unixOffset;
}

// converts SysTimeUSec to seconds (float)
float SysTime()
{
	return SysTimeUSec()*1e-6f;
}

uint32_t SysTimeCycleCounter()
{
	return DWT->CYCCNT;
}

uint32_t SysTimeCycleCounterDiffUSec(uint32_t tStart, uint32_t tEnd)
{
	return (tEnd - tStart)/sysTime.cpuClock_MHz;
}
