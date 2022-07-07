#include "skill_get_ball.h"
#include "commands.h"
#include "skill_basics.h"
#include "util/sys_time.h"
#include "util/geometry.h"

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
} SkillGetBallInput;

static struct _SkillGetBall
{
	uint8_t state;
	uint32_t stopBeginTime;
} getBall;

static void getBallInit(const SkillInput *pInput);
static void getBallRun(const SkillInput *pInput, SkillOutput *pOutput);
static void getBallStateRotate(const SkillInput *pInput, SkillOutput *pOutput);
static void getBallStateCenter(const SkillInput *pInput, SkillOutput *pOutput);
static void getBallStateApproach(const SkillInput *pInput, SkillOutput *pOutput);
static void getBallStateDocking(const SkillInput *pInput, SkillOutput *pOutput);
static void getBallStateStop(const SkillInput *pInput, SkillOutput *pOutput);

SkillInstance skillGetBall = { &getBallInit, &getBallRun, 0 };

#define STATE_ROTATE	0
#define STATE_CENTER	1
#define STATE_APPROACH	2
#define STATE_DOCKING	3
#define STATE_STOP		4

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

			float distanceToBall = GeometryDistanceBetweenTwoPoints(origin, GeometryVector2f(pInput->pSensors->ball.pos));
			if(distanceToBall < pGet->searchRadius*0.001f)
			{
				getBall.state = STATE_CENTER;
				pOutput->drive.localVel[2] = 0.0f;
			}
		}
		else
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

	float targetOrientation = GeometryLookAt(GeometryVector2f(pInput->pState->pos), GeometryVector2f(pInput->pSensors->ball.pos));

	pOutput->drive.localVel[0] = 0.0f;
	pOutput->drive.localVel[1] = 0.0f;
	pOutput->drive.pos[2] = targetOrientation;

	if(GeometryAngleDiffAbs(pInput->pState->pos[2],  targetOrientation) < GeometryDeg2Rad(10.0f))
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
	float targetOrientation = GeometryLookAt(GeometryVector2f(pInput->pState->pos), GeometryVector2f(pInput->pSensors->ball.pos));
	pOutput->drive.pos[2] = targetOrientation;

	// drive towards ball
	pOutput->drive.modeXY = DRIVE_MODE_LOCAL_VEL;
	pOutput->drive.localVel[0] = 0.0f;

	float curSpeed = GeometryNorm(GeometryVector2f(pInput->pState->vel));
	float brakeTime = curSpeed / pOutput->drive.limits.accMaxXY;
	float brakeDistance = 0.5f*curSpeed*brakeTime;

	float distanceToBall = GeometryDistanceBetweenTwoPoints(GeometryVector2f(pInput->pState->pos), GeometryVector2f(pInput->pSensors->ball.pos));

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
	float targetOrientation = GeometryLookAt(GeometryVector2f(pInput->pState->pos), GeometryVector2f(pInput->pSensors->ball.pos));
	pOutput->drive.pos[2] = targetOrientation;

	if(pInput->pSensors->ir.interrupted)
	{
		getBall.state = STATE_STOP;
		getBall.stopBeginTime = SysTimeUSec();
	}
	else
	{
		pOutput->drive.modeXY = DRIVE_MODE_LOCAL_VEL;
		pOutput->drive.localVel[0] = 0.0;
		pOutput->drive.localVel[1] = ((float)pGet->dockSpeed)*(GLOBAL_POS_MAX_VEL_XY/255.0f);
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

	if(timeSinceStartMs < 200)
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
