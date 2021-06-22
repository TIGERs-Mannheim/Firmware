/*
 * power.h
 *
 *  Created on: 24.10.2015
 *      Author: AndreR
 */

#pragma once

#include "ch.h"

#define POWER_EVENT_QUEUE_SIZE 4

typedef struct _Power
{
	mailbox_t eventQueue;
	msg_t eventQueueData[POWER_EVENT_QUEUE_SIZE];

	float vBat;
	float iCur;
	uint32_t measTimestamp;
	uint8_t batEmpty;
	uint8_t exhausted;

	uint8_t batCells;
} Power;

extern Power power;

void PowerInit();
void PowerTask(void* params);
void PowerShutdown();
