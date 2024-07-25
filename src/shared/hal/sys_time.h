#pragma once

#include "ch.h"
#include "timer_simple_lld.h"

#define NTP_OFFSET 2208988800UL

#define NTP_TO_UNIX(ntp) (((uint32_t)ntp)-NTP_OFFSET)
#define UNIX_TO_NTP(unix) (((uint32_t)unix)+NTP_OFFSET)

typedef struct _SysTime
{
	TimerSimpleLLD* pTimer1us; // 32-bit timer preferred
	uint32_t cpuClock_MHz;
	uint32_t unixOffset;	// time from 1.1.1900 to start of microcontroller
} SysTimeGlobal;

extern SysTimeGlobal sysTime;

void SysTimeInit(TimerSimpleLLD* pTimer1us);

// microsecond precision, integer arithmetic => fast + recommended
unsigned long SysTimeUSec();
uint32_t SysTimeUnix();	// takes 12days before overflow occurs

// converts SysTimeUSec to seconds (float)
float SysTime();

// Super precise cycle counter, overflows within a few seconds!
uint32_t SysTimeCycleCounter();
uint32_t SysTimeCycleCounterDiffUSec(uint32_t tStart, uint32_t tEnd);
