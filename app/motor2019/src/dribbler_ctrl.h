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
	int32_t hallDeltas[8];

	int32_t currentOffset;
	int32_t currentDirection;
} DribblerCtrlData;

extern DribblerCtrlData dribblerCtrl;

void DribblerCtrlUpdate();
void DribblerCtrlPerPWMCycle();
