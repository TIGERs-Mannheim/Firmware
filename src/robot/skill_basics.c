#include "robot/skill_basics.h"
#include "robot/network.h"
#include "robot.h"
#include "math/vector.h"
#include "commands.h"
#include "util/bits.h"

#define ENABLE_ROLL_OUT_TESTING				0

static void emergencyInit(const SkillInput* pInput);
static void emergencyRun(const SkillInput* pInput, SkillOutput* pOutput);
static void localVelRun(const SkillInput* pInput, SkillOutput* pOutput);
static void globalVelRun(const SkillInput* pInput, SkillOutput* pOutput);
static void globalPosRun(const SkillInput* pInput, SkillOutput* pOutput);
static void wheelVelRun(const SkillInput* pInput, SkillOutput* pOutput);
static void globalVelAndOrient(const SkillInput* pInput, SkillOutput* pOutput);
static void localForceRun(const SkillInput* pInput, SkillOutput* pOutput);

SkillInstance skillEmergency = { &emergencyInit, &emergencyRun, 0 };
SkillInstance skillLocalVel = { 0, &localVelRun, 0 };
SkillInstance skillGlobalVel = { 0, &globalVelRun, 0 };
SkillInstance skillGlobalPos = { 0, &globalPosRun, 0 };
SkillInstance skillWheelVel = { 0, &wheelVelRun, 0 };
SkillInstance skillGlobalVelAndOrient = {0, &globalVelAndOrient, 0 };
SkillInstance skillLocalForce = { 0, &localForceRun, 0 };

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

void SkillBasicsSetKDInputKicker(BasicKDInput* pKD, uint8_t kickMode, uint8_t kickDevice, float kickSpeed)
{
	int32_t kickSpeedBits = (int32_t)(kickSpeed*50.0f + 0.5f);
	if(kickSpeedBits < 0)
		kickSpeedBits = 0;
	else if(kickSpeedBits > 0x1FF)
		kickSpeedBits = 0x1FF;

	kickDevice &= 0x01;
	kickMode &= 0x0F;

	BitsPack(pKD->data, 0, 9, kickSpeedBits);
	BitsPack(pKD->data, 9, 1, kickDevice);
	BitsPack(pKD->data, 10, 2, kickMode);
}

void SkillBasicsSetKDInputDribbler(BasicKDInput* pKD, float dribbleSpeed_mDs, float maxForce_N)
{
	int32_t speed = (int32_t)(dribbleSpeed_mDs * 8.0f + 0.5f);
	int32_t force = (int32_t)(maxForce_N * 4.0f + 0.5f);

	if(speed < 0)
		speed = 0;
	else if(speed > 0x3F)
		speed = 0x3F;

	if(force < 0)
		force = 0;
	else if(force > 0x3F)
		force = 0x3F;

	BitsPack(pKD->data, 12, 6, speed);
	BitsPack(pKD->data, 18, 6, force);
}

void SkillBasicsParseKDInput(const BasicKDInput* pKD, SkillOutput* pOutput)
{
	pOutput->kicker.device = BitsUnpack(pKD->data, 9, 1);
	pOutput->kicker.mode = BitsUnpack(pKD->data, 10, 2);

	uint16_t kickBits = BitsUnpack(pKD->data, 0, 9);
	if(pOutput->kicker.mode == KICKER_MODE_ARM_TIME)
		pOutput->kicker.speed = kickBits*25e-6f; // convert to [s]
	else
		pOutput->kicker.speed = kickBits*0.02f; // convert to [m/s]

	uint8_t dribblerSpeedBits = BitsUnpack(pKD->data, 12, 6);
	uint8_t dribblerForceBits = BitsUnpack(pKD->data, 18, 6);

	pOutput->dribbler.mode = DRIBBLER_MODE_SPEED;
	pOutput->dribbler.velocity = dribblerSpeedBits*0.125f;
	pOutput->dribbler.maxForce = dribblerForceBits*0.25f;
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

	SkillBasicsParseKDInput(&pVel->kd, pOutput);
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

	SkillBasicsParseKDInput(&pForce->kd, pOutput);
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

	Vector2fTurnGlobal2Local(pInput->pState->pos[2], pOutput->drive.localVel[0], pOutput->drive.localVel[1], pOutput->drive.localVel, pOutput->drive.localVel+1);

	pOutput->drive.modeXY = DRIVE_MODE_LOCAL_VEL;
	pOutput->drive.modeW = DRIVE_MODE_LOCAL_VEL;

	SkillBasicsParseKDInput(&pVel->kd, pOutput);
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

	SkillBasicsParseKDInput(&pPos->kd, pOutput);
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

	SkillBasicsParseKDInput(&pVel->kd, pOutput);
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

	Vector2fTurnGlobal2Local(pInput->pState->pos[2], pOutput->drive.localVel[0], pOutput->drive.localVel[1], pOutput->drive.localVel, pOutput->drive.localVel+1);

	pOutput->drive.modeXY = DRIVE_MODE_LOCAL_VEL;
	pOutput->drive.modeW = DRIVE_MODE_GLOBAL_POS;

	SkillBasicsParseKDInput(&pVel->kd, pOutput);
}

SkillEmergencyData emergency = {
	.decrement = {0.1f, 0.1f, 0.1f},
};

static void emergencyInit(const SkillInput* pInput)
{
	float localVel[3];

	RobotMathMotorVelToLocalVel(pInput->pSensors->enc.vel, localVel);

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

		RobotMathLocalVelToMotorVel(emergency.outVelLocal, pOutput->drive.wheelVel);

		for(uint8_t i = 0; i < 4; i++)
			pOutput->drive.wheelVel[i] *= CTRL_MOTOR_TO_WHEEL_RATIO;
	}

#if ENABLE_ROLL_OUT_TESTING
	pOutput->drive.modeXY = DRIVE_MODE_OFF;
	pOutput->drive.modeW = DRIVE_MODE_OFF;
#endif

	pOutput->dribbler.mode = DRIBBLER_MODE_OFF;
	pOutput->kicker.mode = KICKER_MODE_DISARM;
}
