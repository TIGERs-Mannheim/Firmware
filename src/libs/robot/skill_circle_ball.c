/*
 * skill_circle_ball.c
 *
 *  Created on: 19.02.2017
 *      Author: AndreR
 */

#include "skill_circle_ball.h"
#include "skill_basics.h"
#include "util/traj_2order.h"
#include "util/angle_math.h"

static void circleBallRun(const SkillInput* pInput, SkillOutput* pOutput);
static void circleBallInit(const SkillInput* pInput);

SkillInstance skillCircleBall = { &circleBallInit, &circleBallRun, 0 };

typedef struct PACKED _CircleBallInput
{
	int16_t speed; // [mm/s]
	int16_t circleRadius; // [mm]
	int16_t targetAngle; // [rad*1e-4]
	uint8_t mu; // friction coefficient

	uint8_t accMaxXY;
	uint8_t accMaxW;
	uint8_t jerkMaxXY;
	uint8_t jerkMaxW;

	BasicKDInput kd;
} CircleBallInput;

static uint8_t atTargetAngle;

static void circleBallInit(const SkillInput* pInput)
{
	(void)pInput;

	atTargetAngle = 0;
}

static void circleBallRun(const SkillInput* pInput, SkillOutput* pOutput)
{
	CircleBallInput* pCircle = (CircleBallInput*)pInput->pData;

	SkillBasicsParseKDInput(pInput, &pCircle->kd, pOutput);

	if(pCircle->accMaxXY == 0)
		pCircle->accMaxXY = 1;

	if(pCircle->accMaxW == 0)
		pCircle->accMaxW = 1;

	if(pCircle->jerkMaxXY == 0)
		pCircle->jerkMaxXY = 1;

	if(pCircle->jerkMaxW == 0)
		pCircle->jerkMaxW = 1;

	pOutput->drive.limits.velMaxXY = GLOBAL_POS_MAX_VEL_XY;
	pOutput->drive.limits.velMaxW = GLOBAL_POS_MAX_VEL_W;
	pOutput->drive.limits.accMaxXY = ((float)pCircle->accMaxXY)*(LOCAL_VEL_MAX_ACC_XY/255.0f);
	pOutput->drive.limits.accMaxW = ((float)pCircle->accMaxW)*(LOCAL_VEL_MAX_ACC_W/255.0f);

	pOutput->drive.modeXY = DRIVE_MODE_LOCAL_VEL;
	pOutput->drive.modeW = DRIVE_MODE_LOCAL_VEL;

	float radius = pCircle->circleRadius*0.001f;
	float speed = pCircle->speed*0.001f;
	float targetAngle = pCircle->targetAngle*1e-4f;
	float mu = pCircle->mu*1.0f/256.0f;

	float curSpeed[2];
	memcpy(curSpeed, pInput->pLastCtrlRef->trajVelLocal, sizeof(float)*2);
	float curSpeedAbs = sqrtf(curSpeed[0]*curSpeed[0]+curSpeed[1]*curSpeed[1]);

	if(fabsf(radius) < 0.01f)
	{
		pOutput->drive.localVel[0] = 0;
		pOutput->drive.localVel[1] = 0;
		pOutput->drive.localVel[2] = 0;
		return;
	}

	float theta = atanf((curSpeedAbs*curSpeedAbs)/radius*9.81f*mu);


	float angDiff = AngleNormalize(targetAngle-pInput->pState->pos[2]);
	if(fabsf(angDiff) < 0.01)
		atTargetAngle = 1;

	if(atTargetAngle == 0)
	{
		pOutput->kicker.mode = KICKER_MODE_DISARM;
	}

//	if(atTargetAngle)
//	{
//		pOutput->drive.localVel[0] = 0;
//		pOutput->drive.localVel[1] = speed;
//		pOutput->drive.localVel[2] = 0;
//	}
//	else
	{
		pOutput->drive.localVel[0] = arm_sin_f32(theta)*speed;
		pOutput->drive.localVel[1] = arm_cos_f32(-theta)*speed;
		pOutput->drive.localVel[2] = curSpeedAbs/radius;
	}
}
