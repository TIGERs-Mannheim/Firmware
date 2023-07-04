/*
 * dribbler_ctrl.h
 *
 *  Created on: 22.08.2020
 *      Author: AndreR
 */

#pragma once

#include <stdint.h>

typedef struct _DribblerCtrlData
{
	uint8_t lastHallPos;

	int32_t currentOffset;

	uint8_t sampleCounter;

	int32_t voltageQSum;

	int32_t filtOutVoltage;
} DribblerCtrlData;

extern DribblerCtrlData dribblerCtrl;

void DribblerCtrlUpdate();
void DribblerCtrlPerPWMCycle();
