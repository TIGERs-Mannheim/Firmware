#include <util/line.h>
#include "skill_get_ball.h"
#include "commands.h"
#include "skill_basics.h"
#include "util/sys_time.h"
#include "util/angle_math.h"

typedef struct PACKED _SkillGetBallInput
{
	int16_t searchOrigin[2]; // current pos used if == 0x7FFF
	uint16_t searchRadius; // not used if == 0

	uint8_t velMaxXY;
	uint8_t velMaxW;
	uint8_t accMaxXY;
	uint8_t accMaxW;

	BasicKDInput kd;

	uint8_t rotationSpeed;
	uint8_t dockSpeed;
	uint8_t aimSpeed;
} SkillGetBallInput;

static struct _SkillGetBall
{
	uint8_t state;
	uint32_t stopBeginTime;
	uint32_t kickCounterStart;
	uint32_t numGoalDetections;
	uint32_t aimStartTime;
	float aimTargetAngle;
} getBall;

static void getBallInit(const SkillInput *pInput);
static void getBallRun(const SkillInput *pInput, SkillOutput *pOutput);
static void getBallStateRotate(const SkillInput *pInput, SkillOutput *pOutput);
static void getBallStateCenter(const SkillInput *pInput, SkillOutput *pOutput);
static void getBallStateApproach(const SkillInput *pInput, SkillOutput *pOutput);
static void getBallStateDocking(const SkillInput *pInput, SkillOutput *pOutput);
static void getBallStateStop(const SkillInput *pInput, SkillOutput *pOutput);
static void getBallStateSecure(const SkillInput *pInput, SkillOutput *pOutput);
static void getBallStateFindGoal(const SkillInput *pInput, SkillOutput *pOutput);
static void getBallStateAim(const SkillInput *pInput, SkillOutput *pOutput);

SkillInstance skillGetBall = { &getBallInit, &getBallRun, 0 };

#define STATE_ROTATE	0
#define STATE_CENTER	1
#define STATE_APPROACH	2
#define STATE_DOCKING	3
#define STATE_STOP		4
#define STATE_SECURE	5
#define STATE_FIND_GOAL	6
#define STATE_AIM		7

static void getBallInit(const SkillInput *pInput)
{
	(void) pInput;
	getBall.state = STATE_ROTATE;
	getBall.stopBeginTime = 0;
}

static void getBallRun(const SkillInput *pInput, SkillOutput *pOutput)
{
	SkillGetBallInput* pGet = (SkillGetBallInput*)pInput->pData;

	if(pGet->velMaxXY == 0)
		pGet->velMaxXY = 1;

	if(pGet->velMaxW == 0)
		pGet->velMaxW = 1;

	if(pGet->accMaxXY == 0)
		pGet->accMaxXY = 1;

	if(pGet->accMaxW == 0)
		pGet->accMaxW = 1;

	pOutput->drive.limits.velMaxXY = ((float)pGet->velMaxXY)*(GLOBAL_POS_MAX_VEL_XY/255.0f);
	pOutput->drive.limits.velMaxW = ((float)pGet->velMaxW)*(GLOBAL_POS_MAX_VEL_W/255.0f);
	pOutput->drive.limits.accMaxXY = ((float)pGet->accMaxXY)*(GLOBAL_POS_MAX_ACC_XY/255.0f);
	pOutput->drive.limits.accMaxW = ((float)pGet->accMaxW)*(GLOBAL_POS_MAX_ACC_W/255.0f);

	pOutput->kicker.mode = KICKER_MODE_DISARM;

	switch(getBall.state)
	{
		case STATE_ROTATE:
			getBallStateRotate(pInput, pOutput);
			break;
		case STATE_CENTER:
			getBallStateCenter(pInput, pOutput);
			break;
		case STATE_APPROACH:
			getBallStateApproach(pInput, pOutput);
			break;
		case STATE_DOCKING:
			getBallStateDocking(pInput, pOutput);
			break;
		case STATE_STOP:
			getBallStateStop(pInput, pOutput);
			break;
		case STATE_SECURE:
			getBallStateSecure(pInput, pOutput);
			break;
		case STATE_FIND_GOAL:
			getBallStateFindGoal(pInput, pOutput);
			break;
		case STATE_AIM:
			getBallStateAim(pInput, pOutput);
			break;
	}
}

// initial state
// rotate slowly until ball is visible, then proceed to centering state
static void getBallStateRotate(const SkillInput *pInput, SkillOutput *pOutput)
{
	SkillGetBallInput* pGet = (SkillGetBallInput*)pInput->pData;

	pOutput->drive.modeXY = DRIVE_MODE_LOCAL_VEL;
	pOutput->drive.modeW = DRIVE_MODE_LOCAL_VEL;
	pOutput->dribbler.mode = DRIBBLER_MODE_OFF;

	pOutput->drive.localVel[0] = 0.0f;
	pOutput->drive.localVel[1] = 0.0f;
	pOutput->drive.localVel[2] = ((float)pGet->rotationSpeed)*(GLOBAL_POS_MAX_VEL_W/255.0f);

	if(isfinite(pInput->pSensors->ball.pos[0]))
	{
		float orientationToBall = Vector2fLookAtGetAngle(Vector2fFromData(pInput->pState->pos), Vector2fFromData(pInput->pSensors->ball.pos));
		float orientDiff = AngleDiffAbs(pInput->pState->pos[2], orientationToBall);

//		float distanceToBall = GeometryDistanceBetweenTwoPoints(GeometryVector2f(pInput->pState->pos), GeometryVector2f(pInput->pSensors->ball.pos));

		if(pGet->searchRadius > 0)
		{
			Vector2f origin;

			if(pGet->searchOrigin[0] == 0x7FFF)
			{
				origin.x = pInput->pState->pos[0];
				origin.y = pInput->pState->pos[1];
			}
			else
			{
				origin.x = pGet->searchOrigin[0] * 0.001f;
				origin.y = pGet->searchOrigin[1] * 0.001f;
			}

			float distanceToBall = Vector2fGetDistanceBetween(origin, Vector2fFromData(pInput->pSensors->ball.pos));
			if(distanceToBall < pGet->searchRadius*0.001f)
			{
				getBall.state = STATE_CENTER;
				pOutput->drive.localVel[2] = 0.0f;
			}
		}
		else if(orientDiff < AngleDeg2Rad(15.0f)/* && distanceToBall < 10.0f*/)
		{
			getBall.state = STATE_CENTER;
			pOutput->drive.localVel[2] = 0.0f;
		}
	}
}

// Rotate until ball is centered with a 10� margin.
// Max. rotation velocity is used.
static void getBallStateCenter(const SkillInput *pInput, SkillOutput *pOutput)
{
	if(!isfinite(pInput->pSensors->ball.pos[0]))
	{
		getBall.state = STATE_ROTATE;
		return;
	}

	// rotate until ball in 10� to center angle

	pOutput->drive.modeXY = DRIVE_MODE_LOCAL_VEL;
	pOutput->drive.modeW = DRIVE_MODE_GLOBAL_POS;

	float targetOrientation = Vector2fLookAtGetAngle(Vector2fFromData(pInput->pState->pos), Vector2fFromData(pInput->pSensors->ball.pos));

	pOutput->drive.localVel[0] = 0.0f;
	pOutput->drive.localVel[1] = 0.0f;
	pOutput->drive.pos[2] = targetOrientation;

	if(AngleDiffAbs(pInput->pState->pos[2],  targetOrientation) < AngleDeg2Rad(10.0f))
	{
		getBall.state = STATE_APPROACH;
	}
}

// Approach ball and keep it focused at center. Max. local velocity is used.
// A brake trajectory is computed to stop with some distance to the ball.
static void getBallStateApproach(const SkillInput *pInput, SkillOutput *pOutput)
{
	if(!isfinite(pInput->pSensors->ball.pos[0]))
	{
		getBall.state = STATE_ROTATE;
		return;
	}

	// focus ball
	pOutput->drive.modeW = DRIVE_MODE_GLOBAL_POS;
	float targetOrientation = Vector2fLookAtGetAngle(Vector2fFromData(pInput->pState->pos), Vector2fFromData(pInput->pSensors->ball.pos));
	pOutput->drive.pos[2] = targetOrientation;

	// drive towards ball
	pOutput->drive.modeXY = DRIVE_MODE_LOCAL_VEL;
	pOutput->drive.localVel[0] = 0.0f;

	float curSpeed = Vector2fGetLength(Vector2fFromData(pInput->pState->vel));
	float brakeTime = curSpeed / pOutput->drive.limits.accMaxXY;
	float brakeDistance = 0.5f*curSpeed*brakeTime;

	float distanceToBall = Vector2fGetDistanceBetween(Vector2fFromData(pInput->pState->pos), Vector2fFromData(pInput->pSensors->ball.pos));

	if(distanceToBall < brakeDistance + 0.3f)
	{
		pOutput->drive.localVel[1] = 0.0f;

		if(curSpeed < 0.1f)
		{
			getBall.state = STATE_DOCKING;
		}
	}
	else
	{
		pOutput->drive.localVel[1] = pOutput->drive.limits.velMaxXY;
	}
}

// Slowly creep forward until ball interrupts IR barrier.
// Then go to stop mode.
static void getBallStateDocking(const SkillInput *pInput, SkillOutput *pOutput)
{
	SkillGetBallInput* pGet = (SkillGetBallInput*)pInput->pData;

	if(!isfinite(pInput->pSensors->ball.pos[0]))
	{
		getBall.state = STATE_ROTATE;
		return;
	}

	// enable dribbler
	SkillBasicsParseKDInput(pInput, &pGet->kd, pOutput);

	// focus ball
	pOutput->drive.modeW = DRIVE_MODE_GLOBAL_POS;
	float targetOrientation = Vector2fLookAtGetAngle(Vector2fFromData(pInput->pState->pos), Vector2fFromData(pInput->pSensors->ball.pos));
	pOutput->drive.pos[2] = targetOrientation;

	if(pInput->pSensors->ir.interrupted)
	{
		getBall.stopBeginTime = SysTimeUSec();

		if(pGet->aimSpeed)
			getBall.state = STATE_SECURE;
		else
			getBall.state = STATE_STOP;
	}
	else
	{
		pOutput->drive.modeXY = DRIVE_MODE_LOCAL_VEL;
		pOutput->drive.localVel[0] = 0.0f;
		pOutput->drive.localVel[1] = ((float)pGet->dockSpeed)*(GLOBAL_POS_MAX_VEL_XY/255.0f);
	}
}

// Dribble with active dribbler to secure ball
static void getBallStateSecure(const SkillInput *pInput, SkillOutput *pOutput)
{
	SkillGetBallInput* pGet = (SkillGetBallInput*)pInput->pData;

	if(!isfinite(pInput->pSensors->ball.pos[0]))
	{
		getBall.state = STATE_ROTATE;
		return;
	}

	uint32_t timeSinceStartMs = (SysTimeUSec() - getBall.stopBeginTime) / 1000;

	pOutput->drive.modeXY = DRIVE_MODE_LOCAL_VEL;
	pOutput->drive.modeW = DRIVE_MODE_LOCAL_VEL;

	pOutput->drive.localVel[0] = 0.0f;
	pOutput->drive.localVel[1] = ((float)pGet->dockSpeed)*(GLOBAL_POS_MAX_VEL_XY/255.0f);
	pOutput->drive.localVel[2] = 0.0f;

	if(timeSinceStartMs < 200)
	{
		// enable dribbler
		SkillBasicsParseKDInput(pInput, &pGet->kd, pOutput);

		// move forward, no more ball focus
		pOutput->drive.localVel[0] = 0.0f;
		pOutput->drive.localVel[1] = ((float)pGet->dockSpeed)*(GLOBAL_POS_MAX_VEL_XY/255.0f);
		pOutput->drive.localVel[2] = 0.0f;
	}
	else
	{
		// now find the goal
		getBall.kickCounterStart = pInput->pSensors->kicker.kickCounter;
		getBall.numGoalDetections = 0;
		getBall.state = STATE_FIND_GOAL;
	}
}

static void getBallStateFindGoal(const SkillInput *pInput, SkillOutput *pOutput)
{
	SkillGetBallInput* pGet = (SkillGetBallInput*)pInput->pData;

	if(!isfinite(pInput->pSensors->ball.pos[0]))
	{
		getBall.state = STATE_ROTATE;
		return;
	}

	// enable dribbler
	SkillBasicsParseKDInput(pInput, &pGet->kd, pOutput);

	// rotate slowly
	pOutput->drive.modeXY = DRIVE_MODE_LOCAL_VEL;
	pOutput->drive.modeW = DRIVE_MODE_LOCAL_VEL;

	pOutput->drive.localVel[0] = 0.0f;
	pOutput->drive.localVel[1] = 0.0f;
	pOutput->drive.localVel[2] = ((float)pGet->aimSpeed)*(GLOBAL_POS_MAX_VEL_W/255.0f);

	uint8_t goalFound = pInput->pSensors->pointDist.validColumns > 10 && pInput->pSensors->pointDist.isMostlyWhite
			&& pInput->pSensors->pointDist.updated;

	if(goalFound)
	{
//		float goalCenterAngleRelative = 0.5f*(pInput->pSensors->goal.rightAngle + pInput->pSensors->goal.leftAngle);
		float orientationAtFrameTime = pInput->pState->pos[2];// - pInput->pState->vel[2] * ((SysTimeUSec() - pInput->pSensors->goal.time) * 1e-6f) * 0.5f;

		getBall.aimTargetAngle = orientationAtFrameTime/* - goalCenterAngleRelative*/;
		getBall.numGoalDetections++;
	}

	//if(getBall.numGoalDetections >= 3 || (getBall.numGoalDetections > 0 && pInput->pSensors->goal.updated && !goalFound))
	if(getBall.numGoalDetections > 0)
	{
		getBall.aimStartTime = SysTimeUSec();
		getBall.state = STATE_AIM;
	}
}

static void getBallStateAim(const SkillInput *pInput, SkillOutput *pOutput)
{
	SkillGetBallInput* pGet = (SkillGetBallInput*)pInput->pData;

//	if(!isfinite(pInput->pSensors->ball.pos[0]))
//	{
//		getBall.state = STATE_ROTATE;
//		return;
//	}

	uint8_t goalFound = pInput->pSensors->pointDist.validColumns > 10 && pInput->pSensors->pointDist.isMostlyWhite
			&& pInput->pSensors->pointDist.updated;

	// refresh target angle whenever the robot stands still
	if(fabsf(pInput->pLastCtrlRef->trajVelLocal[2]) < 0.01f && goalFound)
	{
//		float goalCenterAngleRelative = 0.5f*(pInput->pSensors->goal.rightAngle + pInput->pSensors->goal.leftAngle);
//		float orientationAtFrameTime = pInput->pState->pos[2];
//
//		getBall.aimTargetAngle = orientationAtFrameTime - goalCenterAngleRelative;
	}

	// enable dribbler
	SkillBasicsParseKDInput(pInput, &pGet->kd, pOutput);

	pOutput->drive.modeW = DRIVE_MODE_GLOBAL_POS;
	pOutput->drive.pos[2] = getBall.aimTargetAngle;

	// Shoot after one second of aiming
	if(SysTimeUSec() - getBall.aimStartTime > 1000000)
	{
		pOutput->kicker.speed = 5.0f;
		pOutput->kicker.mode = KICKER_MODE_ARM;
		pOutput->kicker.device = KICKER_DEVICE_STRAIGHT;
	}

	if(getBall.kickCounterStart != pInput->pSensors->kicker.kickCounter)
	{
		getBall.state = STATE_STOP;
	}
}

// Dribble 0.2s with active dribbler to snap ball.
// Then stop and disable dribbler (until 0.5s).
// Then disable all motors (after 0.5s).
static void getBallStateStop(const SkillInput *pInput, SkillOutput *pOutput)
{
	SkillGetBallInput* pGet = (SkillGetBallInput*)pInput->pData;

	uint32_t timeSinceStartMs = (SysTimeUSec() - getBall.stopBeginTime) / 1000;

	pOutput->drive.modeXY = DRIVE_MODE_LOCAL_VEL;
	pOutput->drive.modeW = DRIVE_MODE_LOCAL_VEL;

	pOutput->drive.localVel[0] = 0.0;
	pOutput->drive.localVel[1] = ((float)pGet->dockSpeed)*(GLOBAL_POS_MAX_VEL_XY/255.0f);
	pOutput->drive.localVel[2] = 0.0f;

	if(timeSinceStartMs < 300)
	{
		// enable dribbler
		SkillBasicsParseKDInput(pInput, &pGet->kd, pOutput);

		// move forward, no more ball focus
		pOutput->drive.localVel[0] = 0.0;
		pOutput->drive.localVel[1] = ((float)pGet->dockSpeed)*(GLOBAL_POS_MAX_VEL_XY/255.0f);
		pOutput->drive.localVel[2] = 0.0f;
	}
	else if(timeSinceStartMs < 500)
	{
		pOutput->dribbler.speed = 0.0f;
		pOutput->drive.localVel[0] = 0.0;
		pOutput->drive.localVel[1] = 0.0f;
		pOutput->drive.localVel[2] = 0.0f;
	}
	else
	{
		pOutput->dribbler.mode = DRIBBLER_MODE_OFF;
		pOutput->drive.modeXY = DRIVE_MODE_OFF;
		pOutput->drive.modeW = DRIVE_MODE_OFF;
	}
}
