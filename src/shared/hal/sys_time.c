#include "sys_time.h"
#include "hal/init_hal.h"

SysTimeGlobal sysTime;

static void millisecondTimerIrq(virtual_timer_t*, void*)
{
	chSysLockFromISR();
	++sysTime.millisecondCounter;
	chSysUnlockFromISR();
}

void SysTimeInit(TimerSimpleLLD* pTimer1us)
{
	chVTObjectInit(&sysTime.millisecondTimer);

	sysTime.pTimer1us = pTimer1us;
	sysTime.cpuClock_MHz = systemClockInfo.SYSClk / 1000000UL;

	TimerSimpleStartPeriodic(pTimer1us, 0xFFFFFFFF);
	chVTSetContinuous(&sysTime.millisecondTimer, TIME_MS2I(1), millisecondTimerIrq, 0);
}

uint32_t SysTimeUSec()
{
	return sysTime.pTimer1us->pTim->CNT; // [us]
}

uint64_t SysTimeMonotonic_ms()
{
	chSysLock();
	uint64_t millis = sysTime.millisecondCounter;
	chSysUnlock();

	return millis;
}

uint32_t SysTimeMonotonic_s()
{
	return SysTimeMonotonic_ms()/1000;
}

uint32_t SysTimeUnix_s()
{
	return SysTimeMonotonic_s() + sysTime.unixOffset;
}

void SysTimeSetUnixTime(uint32_t unixTime)
{
	sysTime.unixOffset = unixTime - SysTimeMonotonic_s();
}

uint32_t SysTimeCycleCounter()
{
	return DWT->CYCCNT;
}

uint32_t SysTimeCycleCounterDiffUSec(uint32_t tStart, uint32_t tEnd)
{
	return (tEnd - tStart)/sysTime.cpuClock_MHz;
}
