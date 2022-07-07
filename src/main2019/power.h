/*
 * power.h
 *
 *  Created on: 01.01.2019
 *      Author: AndreR
 */

#pragma once

#include "ch.h"
#include "util/i2c.h"

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
	float cellMin;
	float cellEnergetic;
	float cellMax;

	uint8_t usbPowered;

	I2C* pI2C;
} Power;

extern Power power;

void PowerInit(I2C* pI2C);
void PowerTask(void* params);
void PowerShutdown();
