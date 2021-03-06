/*
 * skill_basics.c
 *
 *  Created on: 24.06.2015
 *      Author: AndreR
 */

#include "robot/ctrl.h"
#include "robot/skill_basics.h"
#include "robot/network.h"
#include "util/angle_math.h"
#include "commands.h"

static void emergencyInit(const SkillInput* pInput);
static void emergencyRun(const SkillInput* pInput, SkillOutput* pOutput);
static void localVelRun(const SkillInput* pInput, SkillOutput* pOutput);
static void globalVelRun(const SkillInput* pInput, SkillOutput* pOutput);
static void globalPosRun(const SkillInput* pInput, SkillOutput* pOutput);
static void wheelVelRun(const SkillInput* pInput, SkillOutput* pOutput);
static void globalVelAndOrient(const SkillInput* pInput, SkillOutput* pOutput);
static void localForceRun(const SkillInput* pInput, SkillOutput* pOutput);
static void atTargetAngleInit(const SkillInput* pInput);

SkillInstance skillEmergency = { &emergencyInit, &emergencyRun, 0 };
SkillInstance skillLocalVel = { &atTargetAngleInit, &localVelRun, 0 };
SkillInstance skillGlobalVel = { &atTargetAngleInit, &globalVelRun, 0 };
SkillInstance skillGlobalPos = { &atTargetAngleInit, &globalPosRun, 0 };
SkillInstance skillWheelVel = { &atTargetAngleInit, &wheelVelRun, 0 };
SkillInstance skillGlobalVelAndOrient = {&atTargetAngleInit, &globalVelAndOrient, 0 };
SkillInstance skillLocalForce = { &atTargetAngleInit, &localForceRun, 0 };

typedef struct PACKED _GlobalVelXYPosWInput
{
	int16_t velXY[2];
	int16_t orient;

	uint8_t accMaxXY;
	uint8_t jerkMaxXY;
	uint8_t velMaxW;
	uint8_t accMaxW;

	BasicKDInput kd;
} GlobalVelXYPosWInput;

typedef struct PACKED _WheelVelInput
{
	int16_t wheelVel[4];

	BasicKDInput kd;
} WheelVelInput;

typedef struct PACKED _LocalForceInput
{
	int16_t xyw[3];

	BasicKDInput kd;
} ForceInput;

static uint8_t atTargetAngle;

void SkillBasicsSetKDInputKicker(BasicKDInput* pKD, uint8_t kickMode, uint8_t kickDevice, float kickSpeed)
{
	pKD->flags &= ~(BASIC_KD_FLAGS_KICK_DEVICE_MASK | BASIC_KD_FLAGS_KICK_MODE_MASK);

	pKD->flags |= kickDevice | (kickMode << 4);
	pKD->kickSpeed = (uint8_t)(kickSpeed*25.0f);
}

void SkillBasicsSetKDInputDribbler(BasicKDInput* pKD, float dribbleSpeed_rpm, float maxCurrent)
{
	int32_t speed = ((int32_t)(dribbleSpeed_rpm + 500.0f)) / 1000UL;
	int32_t current = (int32_t)((maxCurrent + 0.25f) * 2.0f);

	if(speed < 0)
		speed = 0;

	if(current < 0)
		current = 0;

	pKD->dribbler = (speed >> 1) & 0x0F;
	pKD->dribbler |= (current << 3) & 0xF0;

	pKD->flags &= ~(BASIC_KD_FLAGS_DRIB_SPEED_LSB | BASIC_KD_FLAGS_DRIB_CUR_LSB);

	if(speed & 0x01)
		pKD->flags |= BASIC_KD_FLAGS_DRIB_SPEED_LSB;

	if(current & 0x01)
		pKD->flags |= BASIC_KD_FLAGS_DRIB_CUR_LSB;
}

void SkillBasicsParseKDInput(const SkillInput* pInput, const BasicKDInput* pKD, SkillOutput* pOutput)
{
	pOutput->kicker.device = pKD->flags & BASIC_KD_FLAGS_KICK_DEVICE_MASK;
	pOutput->kicker.mode = (pKD->flags & BASIC_KD_FLAGS_KICK_MODE_MASK) >> 4;

	if(pOutput->kicker.mode == KICKER_MODE_ARM_TIME)
	{
		uint32_t timeHighBits = (pKD->flags & BASIC_KD_FLAGS_EXTRA_MASK) >> 1;
		uint32_t time10Us = (timeHighBits << 8) + pKD->kickSpeed;
		pOutput->kicker.speed = time10Us*1e-5f; // convert to [s]
	}
	else
	{
		// convert [0.04 m/s] to float
		float kickSpeed = pKD->kickSpeed*0.04f;
		pOutput->kicker.speed = kickSpeed;
	}

	if(pOutput->kicker.mode == KICKER_MODE_ARM_AIM && pOutput->drive.modeW == DRIVE_MODE_GLOBAL_POS)
	{
		float angDiff = AngleNormalize(pOutput->drive.pos[2] - pInput->pState->pos[2]);
		if(fabsf(angDiff) < 0.005f)
			atTargetAngle = 1;
		if(fabsf(angDiff) > 0.01f)
			atTargetAngle = 0;

		if(atTargetAngle == 0)
			pOutput->kicker.mode = KICKER_MODE_DISARM;
		else
			pOutput->kicker.mode = KICKER_MODE_ARM;
	}

	uint8_t dribblerSpeed = (pKD->dribbler & 0x0F) << 1;
	uint8_t dribblerCurrent = (pKD->dribbler & 0xF0) >> 3;

	if(pOutput->kicker.mode != KICKER_MODE_ARM_TIME)
	{
		if(pKD->flags & BASIC_KD_FLAGS_DRIB_SPEED_LSB)
			dribblerSpeed |= 0x01;

		if(pKD->flags & BASIC_KD_FLAGS_DRIB_CUR_LSB)
			dribblerCurrent |= 0x01;
	}

	pOutput->dribbler.speed = dribblerSpeed*1000.0f * 2.0f*M_PI/60.0f; // convert to [rad/s]
	pOutput->dribbler.maxCurrent = dribblerCurrent*0.5f;

	if(pOutput->dribbler.speed < 200.0f && pInput->pState->dribblerVel < 200.0f)
		pOutput->dribbler.mode = DRIBBLER_MODE_OFF;
	else
		pOutput->dribbler.mode = DRIBBLER_MODE_SPEED;
}

static void atTargetAngleInit(const SkillInput* pInput)
{
	(void)pInput;

	atTargetAngle = 0;
}

static void localVelRun(const SkillInput* pInput, SkillOutput* pOutput)
{
	VelInput* pVel = (VelInput*)pInput->pData;

	pOutput->drive.localVel[0] = pVel->xyw[0]*0.001f;
	pOutput->drive.localVel[1] = pVel->xyw[1]*0.001f;
	pOutput->drive.localVel[2] = pVel->xyw[2]*0.001f;

	if(pVel->accMaxXY == 0)
		pVel->accMaxXY = 1;

	if(pVel->accMaxW == 0)
		pVel->accMaxW = 1;

	if(pVel->jerkMaxXY == 0)
		pVel->jerkMaxXY = 1;

	if(pVel->jerkMaxW == 0)
		pVel->jerkMaxW = 1;

	pOutput->drive.limits.velMaxXY = GLOBAL_POS_MAX_VEL_XY;
	pOutput->drive.limits.velMaxW = GLOBAL_POS_MAX_VEL_W;
	pOutput->drive.limits.accMaxXY = ((float)pVel->accMaxXY)*(LOCAL_VEL_MAX_ACC_XY/255.0f);
	pOutput->drive.limits.accMaxW = ((float)pVel->accMaxW)*(LOCAL_VEL_MAX_ACC_W/255.0f);

	pOutput->drive.modeXY = DRIVE_MODE_LOCAL_VEL;
	pOutput->drive.modeW = DRIVE_MODE_LOCAL_VEL;

	SkillBasicsParseKDInput(pInput, &pVel->kd, pOutput);
}

static void localForceRun(const SkillInput* pInput, SkillOutput* pOutput)
{
	ForceInput* pForce = (ForceInput*)pInput->pData;

	pOutput->drive.localForce[0] = pForce->xyw[0]*0.01f;
	pOutput->drive.localForce[1] = pForce->xyw[1]*0.01f;
	pOutput->drive.localForce[2] = pForce->xyw[2]*0.001f;

	pOutput->drive.limits.velMaxXY = GLOBAL_POS_MAX_VEL_XY;
	pOutput->drive.limits.velMaxW = GLOBAL_POS_MAX_VEL_W;
	pOutput->drive.limits.accMaxXY = LOCAL_VEL_MAX_ACC_XY;
	pOutput->drive.limits.accMaxW = LOCAL_VEL_MAX_ACC_W;

	pOutput->drive.modeXY = DRIVE_MODE_LOCAL_FORCE;
	pOutput->drive.modeW = DRIVE_MODE_LOCAL_FORCE;

	SkillBasicsParseKDInput(pInput, &pForce->kd, pOutput);
}

static void globalVelRun(const SkillInput* pInput, SkillOutput* pOutput)
{
	VelInput* pVel = (VelInput*)pInput->pData;

	pOutput->drive.localVel[0] = pVel->xyw[0]*0.001f;
	pOutput->drive.localVel[1] = pVel->xyw[1]*0.001f;
	pOutput->drive.localVel[2] = pVel->xyw[2]*0.001f;

	if(pVel->accMaxXY == 0)
		pVel->accMaxXY = 1;

	if(pVel->accMaxW == 0)
		pVel->accMaxW = 1;

	if(pVel->jerkMaxXY == 0)
		pVel->jerkMaxXY = 1;

	if(pVel->jerkMaxW == 0)
		pVel->jerkMaxW = 1;

	pOutput->drive.limits.velMaxXY = GLOBAL_POS_MAX_VEL_XY;
	pOutput->drive.limits.velMaxW = GLOBAL_POS_MAX_VEL_W;
	pOutput->drive.limits.accMaxXY = ((float)pVel->accMaxXY)*(LOCAL_VEL_MAX_ACC_XY/255.0f);
	pOutput->drive.limits.accMaxW = ((float)pVel->accMaxW)*(LOCAL_VEL_MAX_ACC_W/255.0f);

	CtrlUtilTurnGlobal2Local(pInput->pState->pos[2], pOutput->drive.localVel[0], pOutput->drive.localVel[1], pOutput->drive.localVel, pOutput->drive.localVel+1);

	pOutput->drive.modeXY = DRIVE_MODE_LOCAL_VEL;
	pOutput->drive.modeW = DRIVE_MODE_LOCAL_VEL;

	SkillBasicsParseKDInput(pInput, &pVel->kd, pOutput);
}

static void globalPosRun(const SkillInput* pInput, SkillOutput* pOutput)
{
	GlobalPosInput* pPos = (GlobalPosInput*)pInput->pData;

	pOutput->drive.pos[0] = pPos->xyw[0]*0.001f;
	pOutput->drive.pos[1] = pPos->xyw[1]*0.001f;
	pOutput->drive.pos[2] = pPos->xyw[2]*0.001f;

	if(pPos->velMaxXY == 0)
		pPos->velMaxXY = 1;

	if(pPos->velMaxW == 0)
		pPos->velMaxW = 1;

	if(pPos->accMaxXY == 0)
		pPos->accMaxXY = 1;

	if(pPos->accMaxW == 0)
		pPos->accMaxW = 1;

	pOutput->drive.limits.velMaxXY = ((float)pPos->velMaxXY)*(GLOBAL_POS_MAX_VEL_XY/255.0f);
	pOutput->drive.limits.velMaxW = ((float)pPos->velMaxW)*(GLOBAL_POS_MAX_VEL_W/255.0f);
	pOutput->drive.limits.accMaxXY = ((float)pPos->accMaxXY)*(GLOBAL_POS_MAX_ACC_XY/255.0f);
	pOutput->drive.limits.accMaxW = ((float)pPos->accMaxW)*(GLOBAL_POS_MAX_ACC_W/255.0f);

	if(pPos->primaryDirection == -128)
	{
		pOutput->drive.modeXY = DRIVE_MODE_GLOBAL_POS;
	}
	else
	{
		pOutput->drive.modeXY = DRIVE_MODE_GLOBAL_POS_ASYNC;
		pOutput->drive.primaryDirection = ((float)pPos->primaryDirection) * (1.0f/127.0f) * M_PI;
	}

	pOutput->drive.modeW = DRIVE_MODE_GLOBAL_POS;

	SkillBasicsParseKDInput(pInput, &pPos->kd, pOutput);
}

static void wheelVelRun(const SkillInput* pInput, SkillOutput* pOutput)
{
	WheelVelInput* pVel = (WheelVelInput*)pInput->pData;

	for(uint8_t i = 0; i < 4; i++)
	{
		pOutput->drive.wheelVel[i] = pVel->wheelVel[i]*0.005f;
	}

	pOutput->drive.modeXY = DRIVE_MODE_WHEEL_VEL;
	pOutput->drive.modeW = DRIVE_MODE_WHEEL_VEL;

	SkillBasicsParseKDInput(pInput, &pVel->kd, pOutput);
}

static void globalVelAndOrient(const SkillInput* pInput, SkillOutput* pOutput)
{
	GlobalVelXYPosWInput* pVel = (GlobalVelXYPosWInput*)pInput->pData;

	pOutput->drive.localVel[0] = pVel->velXY[0]*0.001f;
	pOutput->drive.localVel[1] = pVel->velXY[1]*0.001f;
	pOutput->drive.pos[2] = pVel->orient*0.001f;

	if(pVel->accMaxXY == 0)
		pVel->accMaxXY = 1;

	if(pVel->accMaxW == 0)
		pVel->accMaxW = 1;

	if(pVel->jerkMaxXY == 0)
		pVel->jerkMaxXY = 1;

	if(pVel->velMaxW == 0)
		pVel->velMaxW = 1;

	pOutput->drive.limits.velMaxXY = GLOBAL_POS_MAX_VEL_XY;
	pOutput->drive.limits.accMaxXY = ((float)pVel->accMaxXY)*(LOCAL_VEL_MAX_ACC_XY/255.0f);
	pOutput->drive.limits.velMaxW = ((float)pVel->velMaxW)*(GLOBAL_POS_MAX_VEL_W/255.0f);
	pOutput->drive.limits.accMaxW = ((float)pVel->accMaxW)*(GLOBAL_POS_MAX_ACC_W/255.0f);

	CtrlUtilTurnGlobal2Local(pInput->pState->pos[2], pOutput->drive.localVel[0], pOutput->drive.localVel[1], pOutput->drive.localVel, pOutput->drive.localVel+1);

	pOutput->drive.modeXY = DRIVE_MODE_LOCAL_VEL;
	pOutput->drive.modeW = DRIVE_MODE_GLOBAL_POS;

	SkillBasicsParseKDInput(pInput, &pVel->kd, pOutput);
}

SkillEmergencyData emergency = {
	.decrement = {0.1f, 0.1f, 0.1f},
};

static void emergencyInit(const SkillInput* pInput)
{
	float localVel[3];

	CtrlUtilMotorVelToLocalVel(pInput->pSensors->enc.vel, localVel);

	float absVel = sqrtf(localVel[0]*localVel[0]+localVel[1]*localVel[1]);
	if(absVel > 0.01f)
	{
		emergency.decrement[0] = -localVel[0]/absVel*CTRL_DELTA_T*8.0f;
		emergency.decrement[1] = -localVel[1]/absVel*CTRL_DELTA_T*8.0f;
	}
	else
	{
		emergency.decrement[0] = 0.1f;
		emergency.decrement[1] = 0.1f;
	}

	if(localVel[2] > 0)
	{
		emergency.decrement[2] = -CTRL_DELTA_T*50.0f;
	}
	else
	{
		emergency.decrement[2] = CTRL_DELTA_T*50.0f;
	}

	memcpy(emergency.outVelLocal, localVel, sizeof(float)*3);
}

static void emergencyRun(const SkillInput* pInput, SkillOutput* pOutput)
{
	(void)pInput;

	uint8_t allStop = 1;

	for(uint8_t i = 0; i < 3; i++)
	{
		emergency.outVelLocal[i] += emergency.decrement[i];
		if(fabsf(emergency.outVelLocal[i]) < 10*fabsf(emergency.decrement[i]))
		{
			emergency.outVelLocal[i] = 0;
		}
		else
		{
			allStop = 0;
		}
	}

	if(allStop)
	{
		pOutput->drive.modeXY = DRIVE_MODE_OFF;
		pOutput->drive.modeW = DRIVE_MODE_OFF;
	}
	else
	{
		pOutput->drive.modeXY = DRIVE_MODE_WHEEL_VEL;
		pOutput->drive.modeW = DRIVE_MODE_WHEEL_VEL;

		CtrlUtilLocalVelToMotorVel(emergency.outVelLocal, pOutput->drive.wheelVel, 0);

		for(uint8_t i = 0; i < 4; i++)
			pOutput->drive.wheelVel[i] *= CTRL_MOTOR_TO_WHEEL_RATIO;
	}

// this is just used for roll-out testing
//	pOutput->drive.modeXY = DRIVE_MODE_OFF;
//	pOutput->drive.modeW = DRIVE_MODE_OFF;

//	pOutput->drive.localForce[0] = 0.0f;
//	pOutput->drive.localForce[1] = 0.0f;
//	pOutput->drive.localForce[2] = 0.0f;
//	pOutput->drive.modeXY = DRIVE_MODE_LOCAL_FORCE;
//	pOutput->drive.modeW = DRIVE_MODE_LOCAL_FORCE;

	pOutput->dribbler.mode = DRIBBLER_MODE_OFF;
	pOutput->kicker.mode = KICKER_MODE_DISARM;
}
