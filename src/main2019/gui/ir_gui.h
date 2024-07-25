#pragma once

#include "gfx.h"
#include "log_msgs.h"

typedef struct _IrGuiData
{
	// From RobotCtrlState:
	uint8_t filteredBallPosValid;
	float filteredBallPos_m[2];

	// From RobotSensors:
	uint8_t irBallDetected;
	float irEstimatedBallPosition_mm[2];

	// From McuDribbler:
	float vLateral[4][5];
} IrGuiData;

GHandle IrGuiCreate();
void IrGuiUpdate(const IrGuiData* pData);
