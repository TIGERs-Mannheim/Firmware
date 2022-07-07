/*
 * traj_generator.h
 *
 *  Created on: 26.05.2016
 *      Author: AndreR
 */

#pragma once

#include "traj_1order.h"
#include "traj_2order.h"

typedef struct _TrajGenerator
{
	float vMaxXY;
	float aMaxXY;
	float vMaxW;
	float aMaxW;

	uint8_t modeXY;
	uint8_t modeW;

	float filterGlobalPos[3];
	float filterGlobalVel[3];

	float trajGlobalPos[3];
	float trajGlobalVel[3];
	float trajGlobalAcc[3];

	TrajSecOrder2D trajPosXY;
	TrajSecOrder2DAsync trajPosXYAsync;
	TrajSecOrder1D trajPosW;

	float trajLocalVel[3];
	float trajLocalAcc[3];

	TrajFirstOrder2D trajVelXY;
	TrajFirstOrder1D trajVelW;
} TrajGenerator;

void TrajGeneratorInit(TrajGenerator* pGen);
void TrajGeneratorReset(TrajGenerator* pGen);
void TrajGeneratorResetXY(TrajGenerator* pGen);
void TrajGeneratorResetW(TrajGenerator* pGen);
void TrajGeneratorSetState(TrajGenerator* pGen, const float* pGlobalPos, const float* pGlobalVel);
float TrajGeneratorGetTotalTimeXY(TrajGenerator* pGen);
void TrajGeneratorCreateGlobalPosXY(TrajGenerator* pGen, const float* pPosTarget);
void TrajGeneratorCreateGlobalPosXYAsync(TrajGenerator* pGen, const float* pPosTarget, float rotation);
void TrajGeneratorCreateGlobalPosW(TrajGenerator* pGen, float orientTarget);
void TrajGeneratorCreateGlobalPosWDynamicAcc(TrajGenerator* pGen, float orientTarget, float accMin, float precision);
void TrajGeneratorCreateLocalVelXY(TrajGenerator* pGen, const float* pVelTarget);
void TrajGeneratorCreateLocalVelW(TrajGenerator* pGen, float velTargetW);
void TrajGeneratorStepXY(TrajGenerator* pGen, float tStep);
void TrajGeneratorStepW(TrajGenerator* pGen, float tStep);
