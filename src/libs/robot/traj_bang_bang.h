/*
 * traj_bang_bang.h
 *
 *  Created on: 06.08.2020
 *      Author: AndreR
 */

#pragma once

#include "util/traj_generator.h"
#include "util/lag_element.h"
#include "ctrl.h"

typedef struct _TrajBangBang
{
	TrajGenerator trajGen;

	float lastTargetPos[3];

	LagElementPT1 orientTargetLag;
} TrajBangBang;

void TrajBangBangInit(TrajBangBang* pTraj);
void TrajBangBangUpdate(TrajBangBang* pTraj, const RobotCtrlState* pState, const DriveInput* pDrive, RobotCtrlReference* pReference, float centAccMax);
void TrajBangBangResetState(TrajBangBang* pTraj, const RobotCtrlState* pState);
