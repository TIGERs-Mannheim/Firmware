#include "skill_tc_2022.h"
#include "commands.h"
#include "skill_basics.h"
#include "util/traj_2order.h"
#include "robot.h"
#include "util/sys_time.h"
#include "util/angle_math.h"
#include "util/vector.h"
#include <float.h>
#include <stdlib.h>

typedef struct PACKED _SkillTc2022Input
{
	int16_t targetPos[2]; // Stage 3: target, Stage 4: ball pos seed
	int16_t startPos[2]; // Stage 3: start pos seed, Stage: bot pos (fix)

	uint8_t velMaxXY;
	uint8_t velMaxW;
	uint8_t accMaxXY;
	uint8_t accMaxW;

	BasicKDInput kd;

	uint8_t rotationSpeed;
} SkillTc2022Input;

static struct _SkillTcData
{
	uint8_t stage;

	uint8_t stage12State;
	uint8_t getBallState;

	uint8_t kickCounterStart;
} tcData;

static void tc2022Init(const SkillInput *pInput);
static void tc2022Run(const SkillInput *pInput, SkillOutput *pOutput);
static void tcStateStage1and2(const SkillInput *pInput, SkillOutput *pOutput);
static void tcStateGetBall(const SkillInput *pInput, SkillOutput *pOutput);

SkillInstance skillTc2022 = { &tc2022Init, &tc2022Run, 0 };

#define STAGE_1 0
#define STAGE_2 1
#define STAGE_3 2
#define STAGE_4 3

#define GET_BALL_STATE_ENTRY 0
#define GET_BALL_STATE_ROTATE 1
#define GET_BALL_STATE_CENTER 2
#define GET_BALL_STATE_APPROACH 3
#define GET_BALL_STATE_DOCK 4
#define GET_BALL_STATE_TURN_COMPLETE 5
#define GET_BALL_STATE_FOUND 6

#define STAGE12_STATE_ENTRY 0
#define STAGE12_STATE_LOCATE 1
#define STAGE12_STATE_ROTATE 2
#define STAGE12_STATE_GET_BALL 3
#define STAGE12_STATE_COMPLETE 4
#define STAGE12_STATE_AIM 5

static void tc2022Init(const SkillInput *pInput)
{
	(void)pInput;

	srand(SysTimeUSec());

	tcData.stage = skills.input.skillId - 14;

	if(tcData.stage == STAGE_1 || tcData.stage == STAGE_2)
	{
		tcData.stage12State = STAGE12_STATE_ENTRY;
	}
}

static void tc2022Run(const SkillInput *pInput, SkillOutput *pOutput)
{
	SkillTc2022Input* pTc = (SkillTc2022Input*)pInput->pData;

	if(pTc->velMaxXY == 0)
		pTc->velMaxXY = 1;

	if(pTc->velMaxW == 0)
		pTc->velMaxW = 1;

	if(pTc->accMaxXY == 0)
		pTc->accMaxXY = 1;

	if(pTc->accMaxW == 0)
		pTc->accMaxW = 1;

	pOutput->drive.limits.velMaxXY = ((float)pTc->velMaxXY)*(GLOBAL_POS_MAX_VEL_XY/255.0f);
	pOutput->drive.limits.velMaxW = ((float)pTc->velMaxW)*(GLOBAL_POS_MAX_VEL_W/255.0f);
	pOutput->drive.limits.accMaxXY = ((float)pTc->accMaxXY)*(GLOBAL_POS_MAX_ACC_XY/255.0f);
	pOutput->drive.limits.accMaxW = ((float)pTc->accMaxW)*(GLOBAL_POS_MAX_ACC_W/255.0f);

	pOutput->kicker.mode = KICKER_MODE_DISARM;

	if(tcData.stage == STAGE_1 || tcData.stage == STAGE_2)
	{
		tcStateStage1and2(pInput, pOutput);
	}
}

static void tcStateStage1and2(const SkillInput *pInput, SkillOutput *pOutput)
{
	SkillTc2022Input* pTc = (SkillTc2022Input*)pInput->pData;

	static uint32_t aimTicks = 0;

	switch(tcData.stage12State)
	{
		case STAGE12_STATE_ENTRY:
		{
			tcData.stage12State = STAGE12_STATE_GET_BALL;
			tcData.getBallState = GET_BALL_STATE_ENTRY;

			RobotImplSetEnabledDetectors(EXT_STEP_MASK_BALL_LOC | EXT_STEP_MASK_DIST_SENSOR);
		}
		break;
		case STAGE12_STATE_LOCATE:
		{
			// drive forward until a wall is found
			pOutput->drive.modeXY = DRIVE_MODE_LOCAL_VEL;
			pOutput->drive.modeW = DRIVE_MODE_LOCAL_VEL;

			pOutput->drive.localVel[0] = 0.0f;
			pOutput->drive.localVel[1] = ((float)pTc->velMaxXY)*(GLOBAL_POS_MAX_VEL_XY/255.0f);
			pOutput->drive.localVel[2] = 0.0f;

			SkillBasicsParseKDInput(pInput, &pTc->kd, pOutput);

			pOutput->dribbler.mode = DRIBBLER_MODE_OFF;
			pOutput->dribbler.speed = 0.0f;

			// if a wall is found rotate randomly by 90-180deg
			if(pInput->pSensors->pointDist.avgHeight > 0.04f)
			{
				pOutput->drive.modeW = DRIVE_MODE_GLOBAL_POS;

				float offset = rand() % 90;
				pOutput->drive.pos[2] = pInput->pState->pos[2] + AngleDeg2Rad(90.0f) + offset;

				pOutput->drive.localVel[0] = 0.0f;
				pOutput->drive.localVel[1] = 0.0f;

				tcData.stage12State = STAGE12_STATE_ROTATE;
			}

			if(isfinite(pInput->pSensors->ball.pos[0]))
			{
				tcData.stage12State = STAGE12_STATE_GET_BALL;
				tcData.getBallState = GET_BALL_STATE_ENTRY;
			}
		}
		break;
		case STAGE12_STATE_ROTATE:
		{
			if(AngleDiffAbs(pOutput->drive.pos[2], pInput->pState->pos[2]) < AngleDeg2Rad(10.0f))
			{
				tcData.stage12State = STAGE12_STATE_LOCATE;
			}

			if(isfinite(pInput->pSensors->ball.pos[0]))
			{
				tcData.stage12State = STAGE12_STATE_GET_BALL;
				tcData.getBallState = GET_BALL_STATE_ENTRY;
			}
		}
		break;
		case STAGE12_STATE_GET_BALL:
		{
			tcStateGetBall(pInput, pOutput);

			if(tcData.getBallState == GET_BALL_STATE_FOUND)
			{
				if(tcData.stage == STAGE_1)
				{
					pOutput->drive.modeXY = DRIVE_MODE_LOCAL_VEL;
					pOutput->drive.modeW = DRIVE_MODE_LOCAL_VEL;

					pOutput->drive.localVel[0] = 0.0f;
					pOutput->drive.localVel[1] = 0.0f;
					pOutput->drive.localVel[2] = 0.0f;

					SkillBasicsParseKDInput(pInput, &pTc->kd, pOutput);

					pOutput->dribbler.mode = DRIBBLER_MODE_OFF;
					pOutput->dribbler.speed = 0.0f;

					tcData.stage12State = STAGE12_STATE_COMPLETE;
				}
				else // in stage 2: aim on goal
				{
					// rotate slowly
					pOutput->drive.modeXY = DRIVE_MODE_LOCAL_VEL;
					pOutput->drive.modeW = DRIVE_MODE_LOCAL_VEL;

					pOutput->drive.localVel[0] = 0.0f;
					pOutput->drive.localVel[1] = 0.5f;
					pOutput->drive.localVel[2] = 1.0f;

					tcData.kickCounterStart = pInput->pSensors->kicker.kickCounter;
					aimTicks = 0;

					tcData.stage12State = STAGE12_STATE_AIM;
				}
			}

			if(tcData.getBallState == GET_BALL_STATE_TURN_COMPLETE)
			{
				tcData.stage12State = STAGE12_STATE_LOCATE;
			}
		}
		break;
		case STAGE12_STATE_AIM:
		{
			aimTicks++;

			uint8_t goalFound = pInput->pSensors->pointDist.validColumns > 10 && pInput->pSensors->pointDist.isMostlyWhite
					&& pInput->pSensors->pointDist.updated;

			// enable dribbler
			SkillBasicsParseKDInput(pInput, &pTc->kd, pOutput);

			if(goalFound)
			{
				pOutput->kicker.speed = 6.0f;
				pOutput->kicker.mode = KICKER_MODE_ARM;
				pOutput->kicker.device = KICKER_DEVICE_STRAIGHT;
			}

			if(aimTicks > 5000 && pInput->pSensors->ir.interrupted == 0)
			{
				tcData.stage12State = STAGE12_STATE_GET_BALL;
				tcData.getBallState = GET_BALL_STATE_ENTRY;
			}

			if(tcData.kickCounterStart != pInput->pSensors->kicker.kickCounter)
			{
				pOutput->drive.modeXY = DRIVE_MODE_LOCAL_VEL;
				pOutput->drive.modeW = DRIVE_MODE_LOCAL_VEL;

				pOutput->drive.localVel[0] = 0.0f;
				pOutput->drive.localVel[1] = 0.0f;
				pOutput->drive.localVel[2] = 0.0f;

				SkillBasicsParseKDInput(pInput, &pTc->kd, pOutput);

				pOutput->dribbler.mode = DRIBBLER_MODE_OFF;
				pOutput->dribbler.speed = 0.0f;

				tcData.stage12State = STAGE12_STATE_COMPLETE;
			}
		}
		break;
	}
}

static void tcStateGetBall(const SkillInput *pInput, SkillOutput *pOutput)
{
	static uint32_t ticks2TurnComplete = 0;
	static uint32_t ticksDone = 0;
	static uint32_t ticksDocking = 0;
	static uint32_t ticksContact = 0;

	SkillTc2022Input* pTc = (SkillTc2022Input*)pInput->pData;

	switch(tcData.getBallState)
	{
		case GET_BALL_STATE_ENTRY:
		{
			ticksDocking = 0;
			ticksContact = 0;

			pOutput->drive.modeXY = DRIVE_MODE_LOCAL_VEL;
			pOutput->drive.modeW = DRIVE_MODE_LOCAL_VEL;
			pOutput->dribbler.mode = DRIBBLER_MODE_OFF;

			pOutput->drive.localVel[0] = 0.0f;
			pOutput->drive.localVel[1] = 0.0f;
			pOutput->drive.localVel[2] = ((float)pTc->rotationSpeed)*(GLOBAL_POS_MAX_VEL_W/255.0f);

			TrajSecOrder1D traj;
			TrajSecOrder1DCreate(&traj, 0.0f, 0.0f, 2.0f*M_PI, pOutput->drive.limits.velMaxW, pOutput->drive.limits.accMaxW);
			float tRotate = TrajSecOrder1DGetTotalTime(&traj) + 0.5f;

			ticks2TurnComplete = (uint32_t)(tRotate * 1000.0f);
			ticksDone = 0;

			tcData.getBallState = GET_BALL_STATE_ROTATE;
		}
		break;
		case GET_BALL_STATE_ROTATE:
		{
			ticksDone++;

			if(ticksDone > ticks2TurnComplete)
			{
				tcData.getBallState = GET_BALL_STATE_TURN_COMPLETE;
				pOutput->drive.localVel[2] = 0.0f;
			}

			if(isfinite(pInput->pSensors->ball.pos[0]))
			{
				float orientationToBall = Vector2fLookAtGetAngle(Vector2fFromData(pInput->pState->pos), Vector2fFromData(pInput->pSensors->ball.pos));
				float orientDiff = AngleDiffAbs(pInput->pState->pos[2], orientationToBall);

				if(orientDiff < AngleDeg2Rad(15.0f))
				{
					tcData.getBallState = GET_BALL_STATE_CENTER;
					pOutput->drive.localVel[2] = 0.0f;
				}
			}
		}
		break;
		case GET_BALL_STATE_CENTER:
		{
			if(isfinite(pInput->pSensors->ball.pos[0]))
			{
				// rotate until ball in 10deg to center angle

				pOutput->drive.modeXY = DRIVE_MODE_LOCAL_VEL;
				pOutput->drive.modeW = DRIVE_MODE_GLOBAL_POS;

				float targetOrientation = Vector2fLookAtGetAngle(Vector2fFromData(pInput->pState->pos), Vector2fFromData(pInput->pSensors->ball.pos));

				pOutput->drive.localVel[0] = 0.0f;
				pOutput->drive.localVel[1] = 0.0f;
				pOutput->drive.pos[2] = targetOrientation;

				if(AngleDiffAbs(pInput->pState->pos[2],  targetOrientation) < AngleDeg2Rad(10.0f))
				{
					tcData.getBallState = GET_BALL_STATE_APPROACH;
				}
			}
			else
			{
				tcData.getBallState = GET_BALL_STATE_ENTRY;
			}
		}
		break;
		case GET_BALL_STATE_APPROACH:
		{
			if(isfinite(pInput->pSensors->ball.pos[0]))
			{
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
						tcData.getBallState = GET_BALL_STATE_DOCK;
					}
				}
				else
				{
					pOutput->drive.localVel[1] = pOutput->drive.limits.velMaxXY;
				}
			}
			else
			{
				tcData.getBallState = GET_BALL_STATE_ENTRY;
			}
		}
		break;
		case GET_BALL_STATE_DOCK:
		{
			ticksDocking++;

			// enable dribbler
			SkillBasicsParseKDInput(pInput, &pTc->kd, pOutput);

			// focus ball
			if(isfinite(pInput->pSensors->ball.pos[0]) && ticksContact == 0)
			{
				pOutput->drive.modeW = DRIVE_MODE_GLOBAL_POS;
				float targetOrientation = Vector2fLookAtGetAngle(Vector2fFromData(pInput->pState->pos), Vector2fFromData(pInput->pSensors->ball.pos));
				pOutput->drive.pos[2] = targetOrientation;
			}

			if(pInput->pSensors->ir.interrupted)
			{
				ticksContact++;
			}
			else
			{
				pOutput->drive.modeXY = DRIVE_MODE_LOCAL_VEL;
				pOutput->drive.localVel[0] = 0.0f;
				pOutput->drive.localVel[1] = pOutput->drive.limits.velMaxXY * 0.33f;

				if(ticksDocking > 5000)
				{
					tcData.getBallState = GET_BALL_STATE_TURN_COMPLETE;
				}
			}

			if(ticksContact > 500)
			{
				tcData.getBallState = GET_BALL_STATE_FOUND;
			}
		}
		break;
		case GET_BALL_STATE_TURN_COMPLETE:
		{
		}
		break;
		case GET_BALL_STATE_FOUND:
		{
		}
		break;
	}
}
