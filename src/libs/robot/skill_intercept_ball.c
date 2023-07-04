#include "skill_intercept_ball.h"
#include "commands.h"
#include "util/line.h"
#include "util/sys_time.h"
#include "util/angle_math.h"
#include "skill_basics.h"

#define BALL_VELOCITY_THRESHOLD 0.5f
#define INTERCEPT_ANGLE_THRESHOLD_DEG 45.0f
#define LOCK_DISTANCE_THRESHOLD 0.4f

typedef struct PACKED _SkillInterceptBallInput
{
	int16_t xyw[3]; // position is in front of dribbler

	uint8_t velMaxXY;
	uint8_t velMaxW;
	uint8_t accMaxXY;
	uint8_t accMaxW;

	BasicKDInput kd;

	uint16_t moveRadius;
} SkillInterceptBallInput;

static struct _SkillInterceptBall
{
	uint8_t state;
	Vector2f targetPositionDribbler;
	float targetOrientation;
	Vector2f interceptPoint; // bot center position, not dribbler
} interceptBall;

static void interceptBallInit(const SkillInput* pInput);
static void interceptBallRun(const SkillInput* pInput, SkillOutput* pOutput);
static void stateWaitForMovement(const SkillInput *pInput);
static void stateInterceptDynamic(const SkillInput *pInput);
static void stateInterceptLocked(const SkillInput *pInput);

SkillInstance skillInterceptBall = { &interceptBallInit, &interceptBallRun, 0 };

#define STATE_WAIT_FOR_MOVEMENT		0
#define STATE_INTERCEPT_DYNAMIC		1
#define STATE_INTERCEPT_LOCKED		2

static Vector2f getBotPositionFromDribblerPosition(Vector2f ballAtDribblerPos, float orientation, float distCenterToDribbler)
{
	Vector2f ballToBotCenter = Vector2fFromAngleLength(orientation, -distCenterToDribbler);

	return Vector2fAdd(ballAtDribblerPos, ballToBotCenter);
}

static Vector2f getDribblerPositionFromBotPosition(Vector2f botPos, float orientation, float distCenterToDribbler)
{
	Vector2f botCenterToBall = Vector2fFromAngleLength(orientation, distCenterToDribbler);

	return Vector2fAdd(botPos, botCenterToBall);
}

static void interceptBallInit(const SkillInput *pInput)
{
	interceptBall.state = STATE_WAIT_FOR_MOVEMENT;

	Vector2f curPos = Vector2fFromData(pInput->pState->pos);
	Vector2f dribPos = getDribblerPositionFromBotPosition(curPos, pInput->pState->pos[2], pInput->pAux->physical.dribblerDistance);

	interceptBall.targetPositionDribbler.x = dribPos.x;
	interceptBall.targetPositionDribbler.y = dribPos.y;

	interceptBall.interceptPoint.x = pInput->pState->pos[0];
	interceptBall.interceptPoint.y = pInput->pState->pos[1];

	interceptBall.targetOrientation = pInput->pState->pos[2];
}

static void updateInterceptionPoint(const SkillInput* pInput)
{
	SkillInterceptBallInput* pIntercept = (SkillInterceptBallInput*)pInput->pData;

	Line2f ballTravelLine;
	ballTravelLine.origin = Vector2fFromData(pInput->pSensors->ball.linePos);
	ballTravelLine.dir = Vector2fFromData(pInput->pSensors->ball.lineDir);

	Vector2f interceptPoint = Line2fNearestPointOnLine(ballTravelLine, interceptBall.targetPositionDribbler);

	Vector2f desired2Intercept = Vector2fFromPoints(interceptBall.targetPositionDribbler, interceptPoint);
	float dist2Intercept = Vector2fGetLength(desired2Intercept);
	float moveRadius = pIntercept->moveRadius*0.001f;

	if(moveRadius > 0.0f && dist2Intercept > moveRadius)
	{
		desired2Intercept.x *= moveRadius / dist2Intercept;
		desired2Intercept.y *= moveRadius / dist2Intercept;
	}

	Vector2f adjustedInterceptPoint = Vector2fAdd(interceptBall.targetPositionDribbler, desired2Intercept);

	interceptBall.interceptPoint = getBotPositionFromDribblerPosition(adjustedInterceptPoint,
			interceptBall.targetOrientation, pInput->pAux->physical.dribblerDistance);
}

static void interceptBallRun(const SkillInput* pInput, SkillOutput* pOutput)
{
	SkillInterceptBallInput* pIntercept = (SkillInterceptBallInput*)pInput->pData;

	if(pIntercept->velMaxXY == 0)
		pIntercept->velMaxXY = 1;

	if(pIntercept->velMaxW == 0)
		pIntercept->velMaxW = 1;

	if(pIntercept->accMaxXY == 0)
		pIntercept->accMaxXY = 1;

	if(pIntercept->accMaxW == 0)
		pIntercept->accMaxW = 1;

	pOutput->drive.limits.velMaxXY = ((float)pIntercept->velMaxXY)*(GLOBAL_POS_MAX_VEL_XY/255.0f);
	pOutput->drive.limits.velMaxW = ((float)pIntercept->velMaxW)*(GLOBAL_POS_MAX_VEL_W/255.0f);
	pOutput->drive.limits.accMaxXY = ((float)pIntercept->accMaxXY)*(GLOBAL_POS_MAX_ACC_XY/255.0f);
	pOutput->drive.limits.accMaxW = ((float)pIntercept->accMaxW)*(GLOBAL_POS_MAX_ACC_W/255.0f);

	SkillBasicsParseKDInput(pInput, &pIntercept->kd, pOutput);

	if(pIntercept->xyw[0] != 0x7FFF)
	{
		interceptBall.targetPositionDribbler.x = pIntercept->xyw[0] * 0.001f;
		interceptBall.targetPositionDribbler.y = pIntercept->xyw[1] * 0.001f;
		interceptBall.targetOrientation = pIntercept->xyw[2] * 0.001f;
	}

	pOutput->drive.pos[0] = interceptBall.interceptPoint.x;
	pOutput->drive.pos[1] = interceptBall.interceptPoint.y;
	pOutput->drive.modeXY = DRIVE_MODE_GLOBAL_POS;

	pOutput->drive.pos[2] = interceptBall.targetOrientation;
	pOutput->drive.modeW = DRIVE_MODE_GLOBAL_POS;

	switch(interceptBall.state)
	{
		case STATE_WAIT_FOR_MOVEMENT:
			stateWaitForMovement(pInput);
			break;
		case STATE_INTERCEPT_DYNAMIC:
			stateInterceptDynamic(pInput);
			break;
		case STATE_INTERCEPT_LOCKED:
			stateInterceptLocked(pInput);
			break;
	}
}

//initial state
//waits until ball is visible and then proceeds to intercept state
static void stateWaitForMovement(const SkillInput *pInput)
{
	if(isfinite(pInput->pSensors->ball.pos[0]))
	{
		Vector2f ballPos = Vector2fFromData(pInput->pSensors->ball.pos);
		Vector2f ballVel = Vector2fFromData(pInput->pSensors->ball.vel);
		float ballVelAbs = Vector2fGetLength(ballVel);

		Vector2f ballToDesired = Vector2fFromPoints(ballPos, interceptBall.targetPositionDribbler);

		float angle = Vector2fGetAngleBetween(ballToDesired, ballVel);

		if(ballVelAbs > BALL_VELOCITY_THRESHOLD && angle < AngleDeg2Rad(INTERCEPT_ANGLE_THRESHOLD_DEG))
		{
			// once valid interception found => go to next state and catch/redirect
			updateInterceptionPoint(pInput);
			interceptBall.state = STATE_INTERCEPT_DYNAMIC;
		}
	}
}

static void stateInterceptDynamic(const SkillInput *pInput)
{
	if(isfinite(pInput->pSensors->ball.pos[0]))
	{
		Vector2f ballVel = Vector2fFromData(pInput->pSensors->ball.vel);
		float ballVelAbs = Vector2fGetLength(ballVel);

		if(ballVelAbs > BALL_VELOCITY_THRESHOLD)
		{
			updateInterceptionPoint(pInput);
		}

		// use extrapolation with ball vel
		float latency = (SysTimeUSec() - pInput->pSensors->ball.time) * 1e-6f;
		Vector2f ballPos = Vector2fFromData(pInput->pSensors->ball.pos);

		Vector2f futurePos;
		futurePos.x = ballPos.x + ballVel.x*latency;
		futurePos.y = ballPos.y + ballVel.y*latency;

		float distanceToBall = Vector2fGetDistanceBetween(futurePos, interceptBall.interceptPoint);
		if(distanceToBall < LOCK_DISTANCE_THRESHOLD)
		{
			interceptBall.state = STATE_INTERCEPT_LOCKED;
		}
	}
	else
	{
		// ball was not visible for some time, go back to desired intercept pos
		interceptBall.interceptPoint = getBotPositionFromDribblerPosition(interceptBall.targetPositionDribbler,
				interceptBall.targetOrientation, pInput->pAux->physical.dribblerDistance);
		interceptBall.state = STATE_WAIT_FOR_MOVEMENT;
	}
}

static void stateInterceptLocked(const SkillInput *pInput)
{
	uint8_t leaveState = 0;

	if(isfinite(pInput->pSensors->ball.pos[0]))
	{
		Vector2f ballPos = Vector2fFromData(pInput->pSensors->ball.pos);
		float distanceToBall = Vector2fGetDistanceBetween(ballPos, interceptBall.interceptPoint);
		if(distanceToBall > LOCK_DISTANCE_THRESHOLD*2.0f)
		{
			leaveState = 1;
		}
	}
	else
	{
		leaveState = 1;
	}

	if(leaveState)
	{
		interceptBall.interceptPoint = getBotPositionFromDribblerPosition(interceptBall.targetPositionDribbler,
				interceptBall.targetOrientation, pInput->pAux->physical.dribblerDistance);
		interceptBall.state = STATE_WAIT_FOR_MOVEMENT;
	}
}
