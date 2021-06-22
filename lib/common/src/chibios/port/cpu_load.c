/*
 * cpu_load.c
 *
 *  Created on: 13.05.2014
 *      Author: AndreR
 */

#include "cpu_load.h"
#include "util/sys_time.h"
#include "util/console.h"
#include "util/init_hal.h"
#include "ch.h"

#pragma GCC optimize ("Os")

uint64_t cputime;
CPULoad cpuLoad;

static const char* tStateNames[] = { CH_STATE_NAMES };

void CPULoadPrintTasks()
{
	ConsolePrint("Name            rPrio\tCPU Usage\tStack Free\tState\t\tSData\t\tMutices\r\n");

	for(thread_t* pThread = chRegFirstThread(); pThread != 0; pThread = chRegNextThread(pThread))
	{
		uint16_t cpuTime = (uint16_t)((float)pThread->p_ctime/(float)cputime*10000.0f);

		const char* pName;
		if(pThread->p_name)
			pName = pThread->p_name;
		else
			pName = "Unknown";

		ConsolePrint("%-16s%u\t%3hu.%02hu\t\t%u\t\t%-10s\t0x%08X\t", pName,
				pThread->p_realprio, cpuTime/100, cpuTime%100, chThdGetFreeStack(pThread),
				tStateNames[pThread->p_state], (uint32_t)pThread->p_u.rdymsg);

		for(mutex_t* pMutex = pThread->p_mtxlist; pMutex != 0; pMutex = pMutex->m_next)
		{
			ConsolePrint("0x%08X ", pMutex);
		}

		ConsolePrint("\r\n");

		chThdSleepMilliseconds(10);
	}
}

void CPULoadPrintUsage()
{
	uint16_t cpuTime = (uint16_t)(cpuLoad.usage*10000.0f);

	ConsolePrint("CPU Usage: %3hu.%02hu\r\n", cpuTime/100, cpuTime%100);
}

void CPULoadUpdateUsage()
{
	static uint64_t lastIdleCTime;
	static uint64_t lastCpuTime;

	if(chVTTimeElapsedSinceX(cpuLoad.lastUsageUpdateTime) < MS2ST(1000))
		return;

	cpuLoad.lastUsageUpdateTime = chVTGetSystemTimeX();

	thread_t* pIdle = chSysGetIdleThreadX();

	uint32_t idleTicks = pIdle->p_ctime-lastIdleCTime;
	lastIdleCTime = pIdle->p_ctime;

	uint32_t cpuTicks = cputime-lastCpuTime;
	lastCpuTime = cputime;

	float idleMillPercent = (float)idleTicks/(float)cpuTicks;
	cpuLoad.usage = 1.0f - idleMillPercent;
}
