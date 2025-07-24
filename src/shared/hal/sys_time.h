/**
 * Note on SysTime clocks:
 * - SysTimeUSec, SysTimeMonotonic, SysTimeUnix, SysTimeCycleCounter
 *   may all use different time bases and offsets.
 *   Do not compare times from different clocks!
 * - For short (less than an hour) timespans it is recommended to use SysTimeUSec.
 * - For long timespans use SysTimeMonotonic_s.
 * - Only if you need a proper Unix timestamp use SysTimeUnix_s.
 *   The Unix time offset may be unknown and change suddenly.
 * - SysTimeCycleCounter is for precise profiling only.
 */
#pragma once

#include "ch.h"
#include "timer_simple_lld.h"

typedef struct _SysTime
{
	TimerSimpleLLD* pTimer1us; // 32-bit timer preferred
	uint32_t cpuClock_MHz;
	uint32_t unixOffset;	// time from 1.1.1970 to start of microcontroller

	virtual_timer_t millisecondTimer;
	uint64_t millisecondCounter;
} SysTimeGlobal;

extern SysTimeGlobal sysTime;

void SysTimeInit(TimerSimpleLLD* pTimer1us);

// microsecond precision, integer arithmetic => fast + recommended, takes ~4295s (or ~71.6mins) before overflow occurs
uint32_t SysTimeUSec();

// Monotonic clock with huge range, milliseconds precision, takes around half a billion years to overflow.
// Cannot be called from ISRs! Will lock system briefly.
uint64_t SysTimeMonotonic_ms();

// Monotonic clock with reduced range, second precision, overflows after ~136 years, useful for network stack.
// Cannot be called from ISRs! Will lock system briefly.
uint32_t SysTimeMonotonic_s();

// Unix system time in seconds. This clock may jump.
// Cannot be called from ISRs! Will lock system briefly.
uint32_t SysTimeUnix_s();
void SysTimeSetUnixTime(uint32_t unixTime);

// Super precise cycle counter, overflows within a few seconds!
uint32_t SysTimeCycleCounter();
uint32_t SysTimeCycleCounterDiffUSec(uint32_t tStart, uint32_t tEnd);
