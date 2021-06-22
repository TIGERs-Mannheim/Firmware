/*
 * sys_time.c
 *
 *  Created on: 25.10.2014
 *      Author: AndreR
 */

#include "sys_time.h"

#define NVIC_SYSTICK_CNT ( ( volatile unsigned long * ) 0xE000E018 )

#ifdef STM32H7XX
#define CPU_CLOCK_HZ 400000000UL
#endif

#ifdef STM32F7XX
#define CPU_CLOCK_HZ 216000000UL
#endif

#define F_CPU_CLOCK_HZ ((float)CPU_CLOCK_HZ)
#define F_TICK_RATE_HZ ((float)CH_CFG_ST_FREQUENCY)
#define SYSTICK_CNT_MAX ((CPU_CLOCK_HZ / CH_CFG_ST_FREQUENCY) - 1UL)
#define CNT_CLK_1MHZ (CPU_CLOCK_HZ/1000000UL)
#define USEC_PER_TICK (1000000UL/CH_CFG_ST_FREQUENCY)

SysTimeGlobal sysTime;

// microsecond precision, integer arithmetic => fast + recommended
unsigned long SysTimeUSec()
{
	return chVTGetSystemTimeX()*USEC_PER_TICK + (SYSTICK_CNT_MAX-*NVIC_SYSTICK_CNT)/CNT_CLK_1MHZ; // [us]
}

uint32_t SysTimeUnix()
{
	return ST2S(chVTGetSystemTimeX()) + sysTime.unixOffset;
}

// converts SysTimeUSec to seconds (float)
float SysTime()
{
	return SysTimeUSec()*1e-6f;
}

// maximum precision, uses floating point arithmetic => slow
float SysTimePrec()
{
	return chVTGetSystemTimeX()/F_TICK_RATE_HZ + (SYSTICK_CNT_MAX-*NVIC_SYSTICK_CNT)/F_CPU_CLOCK_HZ;
}

uint32_t SysTimeCycleCounter()
{
	return DWT->CYCCNT;
}

uint32_t SysTimeCycleCounterDiffUSec(uint32_t tStart, uint32_t tEnd)
{
	return (tEnd - tStart)/CNT_CLK_1MHZ;
}
