#pragma once

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
