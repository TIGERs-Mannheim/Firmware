/*
 * skill_fast_pos.c
 *
 *  Created on: 04.10.2016
 *      Author: AndreR
 */

#include "skill_fast_pos.h"
#include "skill_basics.h"
#include "util/traj_2order.h"
#include "util/angle_math.h"
#include "struct_ids.h"

static void fastPosRun(const SkillInput* pInput, SkillOutput* pOutput);
static void fastPosInit(const SkillInput* pInput);

SkillFastPos skillFastPos = {
	.instance = { &fastPosInit, &fastPosRun, 0 },
	.config = { 1 },
};

static const ConfigFileDesc configFileDescFastPos =
	{ SID_CFG_SKILL_FAST_POS, 0, "skill/fastPos", 1, (ElementDesc[]) {
		{ UINT8, "backward_only", "", "backward_only" },
	} };

typedef struct PACKED _FastPosInput
{
	int16_t xyw[3];

	uint8_t velMaxXY;
	uint8_t velMaxW;
	uint8_t accMaxXY;
	uint8_t accMaxW;
	uint8_t accMaxXYFast;

	BasicKDInput kd;
} FastPosInput;

static uint8_t fastPosMode = 0;

void SkillFastPosConfigInit()
{
	skillFastPos.pConfigFile = ConfigOpenOrCreate(&configFileDescFastPos, &skillFastPos.config,
			sizeof(SkillFastPosConfig), 0, 0);
}

static void fastPosInit(const SkillInput* pInput)
{
	(void)pInput;

	fastPosMode = 0;
}

static void fastPosRun(const SkillInput* pInput, SkillOutput* pOutput)
{
	FastPosInput* pPos = (FastPosInput*)pInput->pData;

	SkillBasicsParseKDInput(pInput, &pPos->kd, pOutput);

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
	pOutput->drive.limits.strictVelLimit = 0;

	pOutput->drive.pos[0] = pPos->xyw[0]*0.001f;
	pOutput->drive.pos[1] = pPos->xyw[1]*0.001f;

	pOutput->drive.modeXY = DRIVE_MODE_GLOBAL_POS;

	float diffToTrg[2];
	diffToTrg[0] = pOutput->drive.pos[0]-pInput->pLastCtrlRef->trajPos[0];
	diffToTrg[1] = pOutput->drive.pos[1]-pInput->pLastCtrlRef->trajPos[1];

	float distToTarget = sqrtf(diffToTrg[0]*diffToTrg[0] + diffToTrg[1]*diffToTrg[1]);

	const float* pTrajVel = pInput->pLastCtrlRef->trajVelLocal;
	float trajVelAbs = sqrtf(pTrajVel[0]*pTrajVel[0]+pTrajVel[1]*pTrajVel[1]);

	if(distToTarget < 0.05f && trajVelAbs < 0.5f)
	{
		// almost at target position
		pOutput->drive.pos[2] = pPos->xyw[2]*0.001f;
		pOutput->drive.modeW = DRIVE_MODE_GLOBAL_POS;

		fastPosMode = 0;
	}
	else
	{
		if(trajVelAbs < 0.01f)
		{
			// orientation not stable over atan2 if too slow, keep orientation
			pOutput->drive.localVel[2] = 0;
			pOutput->drive.modeW = DRIVE_MODE_LOCAL_VEL;
		}
		else
		{
			float trajVelGlobal[2];
			CtrlUtilTurnLocal2Global(pInput->pLastCtrlRef->trajPos[2], pInput->pLastCtrlRef->trajVelLocal[0],
					pInput->pLastCtrlRef->trajVelLocal[1], trajVelGlobal, trajVelGlobal+1);

			float velOrient = atan2f(trajVelGlobal[1], trajVelGlobal[0]);
			float targetOrient = velOrient + PI; // assume going backward first
			float orientDiff = AngleNormalize(targetOrient-AngleNormalize(pInput->pLastCtrlRef->trajPos[2]));

			if(fabsf(orientDiff) > PI/2.0f && skillFastPos.config.backwardOnly == 0)
			{
				// more oriented with the front in driving direction, go forward
				targetOrient -= PI;
				AngleNormalizePtr(&targetOrient);
				orientDiff = AngleNormalize(targetOrient-AngleNormalize(pInput->pLastCtrlRef->trajPos[2]));
			}

			if(fabsf(orientDiff) < 0.1f && fastPosMode == 0)
			{
				fastPosMode = 1;
			}

			pOutput->drive.pos[2] = targetOrient;
			pOutput->drive.modeW = DRIVE_MODE_GLOBAL_POS;
		}
	}

	if(fastPosMode)
	{
		pOutput->drive.limits.accMaxXY = ((float)pPos->accMaxXYFast)*(GLOBAL_POS_MAX_ACC_XY/255.0f);
	}
}
