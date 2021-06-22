/*
 * sys_time.h
 *
 *  Created on: 09.04.2014
 *      Author: AndreR
 */

#pragma once

#include "ch.h"

#define NTP_OFFSET 2208988800UL

#define NTP_TO_UNIX(ntp) (((uint32_t)ntp)-NTP_OFFSET)
#define UNIX_TO_NTP(unix) (((uint32_t)unix)+NTP_OFFSET)

typedef struct _SysTime
{
	uint32_t unixOffset;	// time from 1.1.1900 to start of microcontroller
} SysTimeGlobal;

extern SysTimeGlobal sysTime;

void SysTimeInitCorrection(int32_t transmissionTime);

// microsecond precision, integer arithmetic => fast + recommended
unsigned long SysTimeUSec();
uint32_t SysTimeUnix();	// takes 12days before overflow occurs

// converts SysTimeUSec to seconds (float)
float SysTime();

// maximum precision, uses floating point arithmetic => slow
float SysTimePrec();

// Super precise cycle counter, overflows within a few seconds!
uint32_t SysTimeCycleCounter();
uint32_t SysTimeCycleCounterDiffUSec(uint32_t tStart, uint32_t tEnd);
