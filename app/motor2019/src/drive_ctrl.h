/*
 * drive_ctrl.c
 *
 *  Created on: 16.03.2019
 *      Author: AndreR
 */

#pragma once

#include <stdint.h>

typedef struct _DriveCtrlData
{
	uint8_t lastHallPos;
	int32_t hallDeltas[8];

	int32_t encSum;
	uint16_t lastEncPos;

	uint8_t sampleCounter;

	int32_t voltageDQSum[2];
} DriveCtrlData;

extern DriveCtrlData driveCtrl;

void DriveCtrlInit();
void DriveCtrlUpdate();
void DriveCtrlUpdatePerPWMCycle();
