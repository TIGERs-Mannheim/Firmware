/*
 * skill_penalty_shoot.c
 *
 *  Created on: Nov 27, 2016
 *      Author: ArneS
 */

#include "skill_penalty_shoot.h"
#include "commands.h"
#include "util/sys_time.h"
#include "util/angle_math.h"
#include "skill_basics.h"

#define PENALTY_SHOOTER_SLOW_MOVE 0
#define PENALTY_SHOOTER_THREATENING_REST 1
#define PENALTY_SHOOTER_TURN 2
#define PENALTY_SHOOTER_KICK 3

typedef struct PACKED _SkillPenaltyShootInput
{
	int16_t  targetAngle;
	uint8_t  timeToShoot;
	uint16_t approachSpeed;
	int8_t sppedInTurnX;
	int8_t sppedInTurnY;
	int16_t  rotationSpeed;
	uint8_t  penaltyKickSpeed;
	uint8_t  dribberSpeed;
	uint8_t  accMax;
	uint8_t  accMaxW;
	uint8_t  jerkMax;
	uint8_t  jerkMaxW;
} SkillPenaltyShootInput;

static void penaltyShootRun(const SkillInput* pInput, SkillOutput* pOutput);
static void penaltyShootInit(const SkillInput* pInput);
static void penaltyShootSlowMove(const SkillPenaltyShootInput* pData, SkillOutput* pOutput);
static void penaltyShootThreateningRest(SkillOutput* pOutput);
static void penaltyShootTurnQuickly(const SkillPenaltyShootInput* pData, SkillOutput* pOutput);
static void penaltyShootPerformKick(const SkillPenaltyShootInput* pData, SkillOutput* pOutput);

SkillInstance skillPenaltyShoot =
	{ &penaltyShootInit, &penaltyShootRun, 0 };

static uint32_t startTime = 0;
static float currentAngleDifference = 0;
static uint8_t state = PENALTY_SHOOTER_SLOW_MOVE;
static uint16_t kickStateCounter = 0;

static void penaltyShootInit(const SkillInput* pInput)
{
	(void) pInput;
	state = PENALTY_SHOOTER_SLOW_MOVE;
	float targetAngle = ((SkillPenaltyShootInput*) pInput->pData)->targetAngle * 1e-3f;
	currentAngleDifference = AngleNormalize(targetAngle - pInput->pState->pos[2]);

}

static void penaltyShootRun(const SkillInput* pInput, SkillOutput* pOutput)
{

	SkillPenaltyShootInput* pPenaltyShootParam = (SkillPenaltyShootInput*) pInput->pData;
	pOutput->kicker.device = KICKER_DEVICE_STRAIGHT;
	pOutput->kicker.mode = KICKER_MODE_DISARM;
	pOutput->dribbler.mode = DRIBBLER_MODE_OFF;
	pOutput->dribbler.speed = pPenaltyShootParam->dribberSpeed * 1e3f * 2.0f*M_PI/60.0f;
	pOutput->dribbler.maxCurrent = 3.0f;
	pOutput->drive.limits.accMaxXY = ((float) pPenaltyShootParam->accMax) * (LOCAL_VEL_MAX_ACC_XY / 255.0f);
	pOutput->drive.limits.accMaxW = ((float) pPenaltyShootParam->accMaxW) * (LOCAL_VEL_MAX_ACC_W / 255.0f);

	switch(state)
	{
		case PENALTY_SHOOTER_SLOW_MOVE:
			penaltyShootSlowMove(pPenaltyShootParam, pOutput);
			if( pInput->pState->dribblerVel < pOutput->dribbler.speed*0.8 &&
				pInput->pSensors->ir.interrupted)
			{
				startTime = (uint32_t)(SysTimeUSec() / 1000);
				state = PENALTY_SHOOTER_THREATENING_REST;
			}
			break;
		case PENALTY_SHOOTER_THREATENING_REST:
			penaltyShootThreateningRest(pOutput);
			uint32_t timeToShoot = pPenaltyShootParam->timeToShoot;
			timeToShoot *= 100;
			if((SysTimeUSec() / 1000) - startTime > timeToShoot)
			{
				state = PENALTY_SHOOTER_TURN;
			}
			break;
		case PENALTY_SHOOTER_TURN:
			kickStateCounter = 0;
			penaltyShootTurnQuickly(pPenaltyShootParam, pOutput);
			float targetAngle = pPenaltyShootParam->targetAngle * 1e-3f;
			float newAngleDifference = AngleNormalize(targetAngle - pInput->pState->pos[2]);
			if(currentAngleDifference * newAngleDifference < .0f)
			{
				state = PENALTY_SHOOTER_KICK;
			}
			currentAngleDifference = newAngleDifference;
			break;
		case PENALTY_SHOOTER_KICK:
			penaltyShootPerformKick(pPenaltyShootParam, pOutput);
			if(++kickStateCounter >= 500)
			{
				state = PENALTY_SHOOTER_SLOW_MOVE;
			}
			break;
	}

	pOutput->drive.limits.strictVelLimit = 0;
}

static void penaltyShootSlowMove(const SkillPenaltyShootInput* pData, SkillOutput* pOutput)
{
	pOutput->drive.modeXY = DRIVE_MODE_LOCAL_VEL;
	pOutput->drive.modeW = DRIVE_MODE_LOCAL_VEL;
	pOutput->dribbler.mode = DRIBBLER_MODE_SPEED;
	pOutput->drive.localVel[0] = 0;
	pOutput->drive.localVel[1] = ((float) pData->approachSpeed) * 1e-3f;
	pOutput->drive.localVel[2] = 0;
}

static void penaltyShootTurnQuickly(const SkillPenaltyShootInput* pData, SkillOutput* pOutput)
{
	pOutput->drive.modeXY = DRIVE_MODE_LOCAL_VEL;
	pOutput->drive.modeW = DRIVE_MODE_LOCAL_VEL;
	pOutput->dribbler.mode = DRIBBLER_MODE_SPEED;
	pOutput->drive.localVel[0] = ((float) pData->sppedInTurnX) * 1e-2f;
	pOutput->drive.localVel[1] = ((float) pData->sppedInTurnY) * 1e-2f;
	pOutput->drive.localVel[2] = ((float) pData->rotationSpeed) * 1e-3f;
}

static void penaltyShootThreateningRest(SkillOutput* pOutput)
{
	pOutput->drive.modeXY = DRIVE_MODE_LOCAL_VEL;
	pOutput->drive.modeW = DRIVE_MODE_LOCAL_VEL;
	pOutput->dribbler.mode = DRIBBLER_MODE_SPEED;
	pOutput->drive.localVel[0] = 0;
	pOutput->drive.localVel[1] = 0;
	pOutput->drive.localVel[2] = 0;
}

static void penaltyShootPerformKick(const SkillPenaltyShootInput* pData, SkillOutput* pOutput)
{
	pOutput->drive.modeXY = DRIVE_MODE_OFF;
	pOutput->drive.modeW = DRIVE_MODE_OFF;
	pOutput->kicker.device = KICKER_DEVICE_STRAIGHT;
	pOutput->kicker.mode = KICKER_MODE_ARM;
	pOutput->kicker.speed = pData->penaltyKickSpeed * 0.04f;
}
