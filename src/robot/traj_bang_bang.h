/*
 * traj_bang_bang.h
 *
 *  Created on: 06.08.2020
 *      Author: AndreR
 */

#pragma once

#include "log_msgs.h"
#include "math/traj_generator.h"
#include "math/lag_element.h"

typedef struct _TrajBangBang
{
	TrajGenerator trajGen;

	float lastTargetPos[3];

	LagElementPT1 orientTargetLag;
} TrajBangBang;

void TrajBangBangInit(TrajBangBang* pTraj);
void TrajBangBangUpdate(TrajBangBang* pTraj, const RobotCtrlState* pState, const DriveInput* pDrive, const RobotDriveLimits* pLimits, RobotCtrlReference* pReference, float centAccMax);
void TrajBangBangResetState(TrajBangBang* pTraj, const RobotCtrlState* pState);
