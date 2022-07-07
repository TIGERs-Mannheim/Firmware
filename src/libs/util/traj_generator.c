/*
 * traj_generator.c
 *
 *  Created on: 26.05.2016
 *      Author: AndreR
 */

#include "traj_generator.h"
#include "robot/util.h"
#include <string.h>
#include <math.h>

#define MODE_RESET					0
#define MODE_LOCAL_VEL				1
#define MODE_GLOBAL_POS				2
#define MODE_GLOBAL_POS_ASYNC		3

static void changeModeXY(TrajGenerator* pGen, uint8_t newMode)
{
	if(pGen->modeXY == newMode)
		return;

	switch(pGen->modeXY)
	{
		case MODE_RESET:
		{
			// out of reset, we only have the filtered data available
			memcpy(pGen->trajGlobalPos, pGen->filterGlobalPos, sizeof(float)*2);
			memcpy(pGen->trajGlobalVel, pGen->filterGlobalVel, sizeof(float)*2);
			memset(pGen->trajGlobalAcc, 0, sizeof(float)*2);

			CtrlUtilTurnGlobal2Local(pGen->filterGlobalPos[2], pGen->filterGlobalVel[0], pGen->filterGlobalVel[1], pGen->trajLocalVel, pGen->trajLocalVel+1);
			memset(pGen->trajLocalAcc, 0, sizeof(float)*2);
		}
		break;
		case MODE_GLOBAL_POS_ASYNC:
		case MODE_GLOBAL_POS:
		{
			switch(newMode)
			{
				case MODE_LOCAL_VEL:
				{
					CtrlUtilTurnGlobal2Local(pGen->trajGlobalPos[2], pGen->trajGlobalVel[0], pGen->trajGlobalVel[1], pGen->trajLocalVel, pGen->trajLocalVel+1);
					CtrlUtilTurnGlobal2Local(pGen->trajGlobalPos[2], pGen->trajGlobalAcc[0], pGen->trajGlobalAcc[1], pGen->trajLocalAcc, pGen->trajLocalAcc+1);
				}
				break;
			}
		}
		break;
		case MODE_LOCAL_VEL:
		{
			switch(newMode)
			{
				case MODE_GLOBAL_POS_ASYNC:
				case MODE_GLOBAL_POS:
				{
					memcpy(pGen->trajGlobalPos, pGen->filterGlobalPos, sizeof(float)*2);

					CtrlUtilTurnLocal2Global(pGen->filterGlobalPos[2], pGen->trajLocalVel[0], pGen->trajLocalVel[1], pGen->trajGlobalVel, pGen->trajGlobalVel+1);
					CtrlUtilTurnLocal2Global(pGen->filterGlobalPos[2], pGen->trajLocalAcc[0], pGen->trajLocalAcc[1], pGen->trajGlobalAcc, pGen->trajGlobalAcc+1);
				}
				break;
			}
		}
		break;
	}

	pGen->modeXY = newMode;
}

static void changeModeW(TrajGenerator* pGen, uint8_t newMode)
{
	if(pGen->modeW == newMode)
		return;

	switch(pGen->modeW)
	{
		case MODE_RESET:
		{
			// out of reset, we only have the filtered data available
			pGen->trajGlobalPos[2] = pGen->filterGlobalPos[2];
			pGen->trajGlobalVel[2] = pGen->filterGlobalVel[2];
			pGen->trajGlobalAcc[2] = 0;

			pGen->trajLocalVel[2] = pGen->filterGlobalVel[2];
			pGen->trajLocalAcc[2] = 0;
		}
		break;
		case MODE_GLOBAL_POS:
		{
			switch(newMode)
			{
				case MODE_LOCAL_VEL:
				{
					pGen->trajLocalVel[2] = pGen->trajGlobalVel[2];
					pGen->trajLocalAcc[2] = pGen->trajGlobalAcc[2];
				}
				break;
			}
		}
		break;
		case MODE_LOCAL_VEL:
		{
			switch(newMode)
			{
				case MODE_GLOBAL_POS:
				{
					pGen->trajGlobalPos[2] = pGen->filterGlobalPos[2];
					pGen->trajGlobalVel[2] = pGen->trajLocalVel[2];
					pGen->trajGlobalAcc[2] = pGen->trajLocalAcc[2];
				}
				break;
			}
		}
		break;
	}

	pGen->modeW = newMode;
}

void TrajGeneratorInit(TrajGenerator* pGen)
{
	memset(pGen, 0, sizeof(*pGen));

	pGen->modeXY = MODE_RESET;
	pGen->modeW = MODE_RESET;

	pGen->vMaxXY = 2;
	pGen->aMaxXY = 3;
	pGen->vMaxW = 10;
	pGen->aMaxW = 20;
}

void TrajGeneratorReset(TrajGenerator* pGen)
{
	pGen->modeXY = MODE_RESET;
	pGen->modeW = MODE_RESET;
}

void TrajGeneratorResetXY(TrajGenerator* pGen)
{
	pGen->modeXY = MODE_RESET;
}

void TrajGeneratorResetW(TrajGenerator* pGen)
{
	pGen->modeW = MODE_RESET;
}

void TrajGeneratorSetState(TrajGenerator* pGen, const float* pGlobalPos, const float* pGlobalVel)
{
	memcpy(pGen->filterGlobalPos, pGlobalPos, sizeof(float)*3);
	memcpy(pGen->filterGlobalVel, pGlobalVel, sizeof(float)*3);
}

float TrajGeneratorGetTotalTimeXY(TrajGenerator* pGen)
{
	switch(pGen->modeXY)
	{
		case MODE_GLOBAL_POS: return TrajSecOrder2DGetTotalTime(&pGen->trajPosXY);
		case MODE_GLOBAL_POS_ASYNC: return TrajSecOrder2DAsyncGetTotalTime(&pGen->trajPosXYAsync);
		case MODE_LOCAL_VEL: return TrajFirstOrder2DGetTotalTime(&pGen->trajVelXY);
		case MODE_RESET:
		default:
			return 0.0f;
	}
}

void TrajGeneratorCreateGlobalPosXY(TrajGenerator* pGen, const float* pPosTarget)
{
	changeModeXY(pGen, MODE_GLOBAL_POS);

	TrajSecOrder2DCreate(&pGen->trajPosXY, pGen->trajGlobalPos, pGen->trajGlobalVel, pPosTarget, pGen->vMaxXY, pGen->aMaxXY);
}

void TrajGeneratorCreateGlobalPosXYAsync(TrajGenerator* pGen, const float* pPosTarget, float rotation)
{
	changeModeXY(pGen, MODE_GLOBAL_POS_ASYNC);

	TrajSecOrder2DAsyncCreate(&pGen->trajPosXYAsync, pGen->trajGlobalPos, pGen->trajGlobalVel, pPosTarget, pGen->vMaxXY, pGen->aMaxXY, rotation);
}

void TrajGeneratorCreateGlobalPosW(TrajGenerator* pGen, float orientTarget)
{
	changeModeW(pGen, MODE_GLOBAL_POS);

	TrajSecOrder1DCreate(&pGen->trajPosW, pGen->trajGlobalPos[2], pGen->trajGlobalVel[2], orientTarget, pGen->vMaxW, pGen->aMaxW);
}

void TrajGeneratorCreateGlobalPosWDynamicAcc(TrajGenerator* pGen, float orientTarget, float accMin, float precision)
{
	changeModeW(pGen, MODE_GLOBAL_POS);

	const float totalTimeXY = TrajGeneratorGetTotalTimeXY(pGen);

	// test if we are already too slow with maximum acceleration
	TrajSecOrder1DCreate(&pGen->trajPosW, pGen->trajGlobalPos[2], pGen->trajGlobalVel[2], orientTarget, pGen->vMaxW, pGen->aMaxW);

	if(TrajSecOrder1DGetTotalTime(&pGen->trajPosW) > totalTimeXY)
		return;

	// if not, find best acceleration
	float acc = (accMin + pGen->aMaxW)*0.5f;
	float inc = acc*0.5f;

	// binary search, some iterations (fixed)
	while(inc > precision)
	{
		TrajSecOrder1DCreate(&pGen->trajPosW, pGen->trajGlobalPos[2], pGen->trajGlobalVel[2], orientTarget, pGen->vMaxW, acc);

		if(TrajSecOrder1DGetTotalTime(&pGen->trajPosW) > totalTimeXY)
			acc += inc;
		else
			acc -= inc;

	    inc *= 0.5f;
	}
}

void TrajGeneratorCreateLocalVelXY(TrajGenerator* pGen, const float* pVelTarget)
{
	changeModeXY(pGen, MODE_LOCAL_VEL);

	TrajFirstOrder2DCreate(&pGen->trajVelXY, pGen->trajLocalVel, pVelTarget, pGen->aMaxXY);
}

void TrajGeneratorCreateLocalVelW(TrajGenerator* pGen, float velTargetW)
{
	changeModeW(pGen, MODE_LOCAL_VEL);

	TrajFirstOrder1DCreate(&pGen->trajVelW, pGen->trajLocalVel[2], velTargetW, pGen->aMaxW);
}

void TrajGeneratorStepXY(TrajGenerator* pGen, float tStep)
{
	switch(pGen->modeXY)
	{
		case MODE_GLOBAL_POS:
		{
			TrajSecOrder2DValuesAtTime(&pGen->trajPosXY, tStep, pGen->trajGlobalPos, pGen->trajGlobalVel, pGen->trajGlobalAcc);
		}
		break;
		case MODE_GLOBAL_POS_ASYNC:
		{
			TrajSecOrder2DAsyncValuesAtTime(&pGen->trajPosXYAsync, tStep, pGen->trajGlobalPos, pGen->trajGlobalVel, pGen->trajGlobalAcc);
		}
		break;
		case MODE_LOCAL_VEL:
		{
			float posDiff[2];
			posDiff[0] = (pGen->trajLocalVel[0] + 0.5f*pGen->trajLocalAcc[0]*tStep)*tStep;
			posDiff[1] = (pGen->trajLocalVel[1] + 0.5f*pGen->trajLocalAcc[1]*tStep)*tStep;

			CtrlUtilTurnLocal2Global(pGen->trajGlobalPos[2], posDiff[0], posDiff[1], posDiff, posDiff+1);

			pGen->trajGlobalPos[0] += posDiff[0];
			pGen->trajGlobalPos[1] += posDiff[1];

			TrajFirstOrder2DValuesAtTime(&pGen->trajVelXY, tStep, pGen->trajLocalVel, pGen->trajLocalAcc);
		}
		break;
	}
}

void TrajGeneratorStepW(TrajGenerator* pGen, float tStep)
{
	switch(pGen->modeW)
	{
		case MODE_GLOBAL_POS:
		{
			TrajSecOrder1DValuesAtTime(&pGen->trajPosW, tStep, pGen->trajGlobalPos+2, pGen->trajGlobalVel+2, pGen->trajGlobalAcc+2);
		}
		break;
		case MODE_LOCAL_VEL:
		{
			float orientDiff = (pGen->trajLocalVel[2] + 0.5f*pGen->trajLocalAcc[2]*tStep)*tStep;
			pGen->trajGlobalPos[2] += orientDiff;

			TrajFirstOrder1DValuesAtTime(&pGen->trajVelW, tStep, pGen->trajLocalVel+2, pGen->trajLocalAcc+2);
		}
		break;
	}
}
