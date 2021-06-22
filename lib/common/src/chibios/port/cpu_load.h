/*
 * cpu_load.h
 *
 *  Created on: 13.05.2014
 *      Author: AndreR
 */

#ifndef CPU_LOAD_H_
#define CPU_LOAD_H_

#include <stdint.h>

typedef struct _CPULoad
{
	float usage; // 0.0 - 1.0

	uint32_t lastUsageUpdateTime;
} CPULoad;

extern CPULoad cpuLoad;

void CPULoadPrintTasks();
void CPULoadUpdateUsage();
void CPULoadPrintUsage();

#endif /* CPU_LOAD_H_ */
