/*
 * robot_status.h
 *
 *  Created on: 02.11.2017
 *      Author: AndreR
 */

#pragma once

#include "gfx.h"
#include "../presenter.h"

#define MOVE_MODE_OFF 0
#define MOVE_MODE_VEL_XY 1
#define MOVE_MODE_VEL_YW 2
#define MOVE_MODE_POS 3

typedef struct _MoveControlState
{
	uint8_t mode;
	float x;
	float y;
	float w;
} MoveControlState;

#define KICK_MODE_DISARM 0
#define KICK_MODE_ARM_STRAIGHT 1
#define KICK_MODE_ARM_CHIP 2

typedef struct _KickControlState
{
	uint8_t mode;
	float speed;
} KickControlState;

typedef struct _ControlPanelLimits
{
	float maxVelXY;
	float maxVelW;
	float maxAccXY;
	float maxAccW;
	float kickSpeed;
	float dribbleSpeed;
} ControlPanelLimits;

typedef struct _RobotStatus
{
	uint8_t manualControlOn;
	uint16_t selectedBotId;
	MoveControlState ultraState;
	KickControlState kickState;
	int16_t dribbleState;
	ControlPanelLimits limits;
} RobotStatus;

extern RobotStatus robotStatus;

void RobotStatusPositionFrameUpdate();
GHandle RobotStatusCreate();
void RobotStatusUpdate(PresenterRobotInfo* pRobots, uint8_t visionAvailable);
uint8_t RobotStatusIsChargeKicker();
