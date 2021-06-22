/*
 * ctrl_motor.c
 *
 *  Created on: 08.03.2013
 *      Author: AndreR
 */

#include <main/ctrl_motor.h>
#include "struct_ids.h"
#include "util/angle_math.h"
#include "util/arm_mat_util_f32.h"

#include <string.h>

static void init(const float* pDefaultPos);
static void controller(const RobotCtrlState* pState, const SkillOutput* pInput, RobotCtrlOutput* pOutput, RobotCtrlReference* pReference);
static void stateEstimation(const RobotSensors* pSensors, const RobotCtrlOutput* pOutput, RobotCtrlState* pState);

CtrlMotor ctrlMotor;

CtrlInstance ctrlMotorInstance = {
	.pInit = &init,
	.pExit = 0,
	.pStateEstimationFunc = &stateEstimation,
	.pControllerFunc = &controller,
};

static void init(const float* pDefaultPos)
{
	memcpy(ctrlMotor.defaultPos, pDefaultPos, sizeof(float)*3);
	ctrlMotor.firstRun = 1;
}

static void stateEstimation(const RobotSensors* pSensors, const RobotCtrlOutput* pOutput, RobotCtrlState* pState)
{
	(void)pOutput;

	// Encoders
	float localVel[3];
	CtrlUtilMotorVelToLocalVel(pSensors->enc.vel, localVel);

	CtrlUtilTurnLocal2Global(pState->pos[2], localVel[0], localVel[1], pState->vel, pState->vel+1);
	pState->vel[2] = localVel[2];
	pState->velUpdated = 1;

	// Acc update
	for(uint8_t i = 0; i < 3; i++)
	{
		pState->acc[i] = (pState->vel[i]-ctrlMotor.lastGlobalVel[i])/CTRL_DELTA_T;
		ctrlMotor.lastGlobalVel[i] = pState->vel[i];
	}
	pState->accUpdated = 1;

	if(pSensors->vision.updated)
	{
		// position update (feed-through)
		memcpy(pState->pos, pSensors->vision.pos, sizeof(float)*3);
		pState->posUpdated = 1;
	}
	else
	{
		// integrate encoders if no pos update is available
		pState->pos[0] += pState->vel[0]*CTRL_DELTA_T;
		pState->pos[1] += pState->vel[1]*CTRL_DELTA_T;
		pState->pos[2] = AngleNormalize(pState->pos[2] + pState->vel[2]*CTRL_DELTA_T);
		pState->posUpdated = 1;
	}

	if(ctrlMotor.firstRun)
	{
		ctrlMotor.firstRun = 0;
		memcpy(pState->pos, ctrlMotor.defaultPos, sizeof(float)*3);
		pState->posUpdated = 1;
	}
}

static void controller(const RobotCtrlState* pState, const SkillOutput* pInput, RobotCtrlOutput* pOutput, RobotCtrlReference* pReference)
{
	(void)pState;
	(void)pReference;

	const DriveInput* pDrive = &pInput->drive;

	float outVel[3] = {0, 0, 0};

	if(pDrive->modeXY == DRIVE_MODE_LOCAL_VEL)
	{
		outVel[0] = pDrive->localVel[0];
		outVel[1] = pDrive->localVel[1];
	}

	if(pDrive->modeW == DRIVE_MODE_LOCAL_VEL)
	{
		outVel[2] = pDrive->localVel[2];
	}

	// transform to motor velocities
	CtrlUtilLocalVelToMotorVel(outVel, pOutput->motorVel, 1);
	pOutput->ctrlMode = MOT_CTRL_VELOCITY;

	if(pDrive->modeXY == DRIVE_MODE_WHEEL_VEL || pDrive->modeW == DRIVE_MODE_WHEEL_VEL)
	{
		for(uint8_t i = 0; i < 4; i++)
			pOutput->motorVel[i] += pDrive->wheelVel[i]*CTRL_WHEEL_TO_MOTOR_RATIO;
	}

	if(pDrive->modeXY == DRIVE_MODE_OFF && pDrive->modeW == DRIVE_MODE_OFF)
	{
		memset(pOutput->motorVel, 0, sizeof(float)*4);

		pOutput->ctrlMode = MOT_CTRL_OFF;
	}
}
