#pragma once

#include <stdint.h>
#include "util/shell_cmd.h"

typedef struct _RTC
{
	ShellCmdHandler cmdHandler;
} RTCGlobal;

extern RTCGlobal rtc;

void RTCInit();

// Set time in binary coded decimal format (HHmmss)
void RTCSetTimeBCD(uint32_t time);

// Set date in binary coded decimal format (YYmmdd)
void RTCSetDateBCD(uint32_t date);

uint32_t RTCGetUnixTimestamp();
void RTCSetUnixTimestamp(int32_t time);

// Adjust RTC clock by -487.1 - 488.5ppm
void RTCSetCalib(float ppm);
