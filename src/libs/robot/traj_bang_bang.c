/*
 * traj_bang_bang.c
 *
 *  Created on: 06.08.2020
 *      Author: AndreR
 */

#include "traj_bang_bang.h"
#include "util/angle_math.h"

void TrajBangBangInit(TrajBangBang* pTraj)
{
	TrajGeneratorInit(&pTraj->trajGen);

	LagElementPT1Init(&pTraj->orientTargetLag, 1.0f, 0.1f, CTRL_DELTA_T);
}

void TrajBangBangUpdate(TrajBangBang* pTraj, const RobotCtrlState* pState, const DriveInput* pDrive, RobotCtrlReference* pReference, float centAccMax)
{
	// Configure trajectory generator
	pTraj->trajGen.vMaxXY = pDrive->limits.velMaxXY;
	pTraj->trajGen.aMaxXY = pDrive->limits.accMaxXY;

	// create local drive input with multi-turn orientation
	DriveInput drive;
	memcpy(&drive, pDrive, sizeof(DriveInput));
	drive.pos[2] = pState->pos[2] + AngleNormalize(drive.pos[2] - AngleNormalize(pState->pos[2]));

	// set current filter state on traj generator
	TrajGeneratorSetState(&pTraj->trajGen, pState->pos, pState->vel);

	// run trajectory generator

	const float dT = CTRL_DELTA_T;

	switch(pDrive->modeXY)
	{
		case DRIVE_MODE_GLOBAL_POS_ASYNC:
		case DRIVE_MODE_GLOBAL_POS:
		{
			if(pDrive->modeXY == DRIVE_MODE_GLOBAL_POS)
				TrajGeneratorCreateGlobalPosXY(&pTraj->trajGen, pDrive->pos);
			else
				TrajGeneratorCreateGlobalPosXYAsync(&pTraj->trajGen, pDrive->pos, pDrive->primaryDirection);

			const float* trgPos = pTraj->trajGen.trajGlobalPos;
			const float* trgVel = pTraj->trajGen.trajGlobalVel;
			const float* trgAcc = pTraj->trajGen.trajGlobalAcc;

			memcpy(pReference->trajPos, trgPos, sizeof(float)*2);

			CtrlUtilTurnGlobal2Local(trgPos[2], trgVel[0], trgVel[1], pReference->trajVelLocal, pReference->trajVelLocal+1);
			CtrlUtilTurnGlobal2Local(trgPos[2], trgAcc[0], trgAcc[1], pReference->trajAccLocal, pReference->trajAccLocal+1);

			TrajGeneratorStepXY(&pTraj->trajGen, dT);
		}
		break;
		case DRIVE_MODE_LOCAL_VEL:
		{
			TrajGeneratorCreateLocalVelXY(&pTraj->trajGen, pDrive->localVel);

			memcpy(pReference->trajPos, pTraj->trajGen.trajGlobalPos, sizeof(float)*2);
			memcpy(pReference->trajVelLocal, pTraj->trajGen.trajLocalVel, sizeof(float)*2);
			memcpy(pReference->trajAccLocal, pTraj->trajGen.trajLocalAcc, sizeof(float)*2);

			TrajGeneratorStepXY(&pTraj->trajGen, dT);
		}
		break;
		default:
		{
			TrajGeneratorResetXY(&pTraj->trajGen);
		}
		break;
	}

	// compute dynamic vel limit for orientation
	float velXYNorm = sqrtf(pReference->trajVelLocal[0]*pReference->trajVelLocal[0] + pReference->trajVelLocal[1]*pReference->trajVelLocal[1]);

	// a_cent = v_xy * v_w
	float vMaxW = centAccMax/velXYNorm;
	if(vMaxW > pDrive->limits.velMaxW)
		vMaxW = pDrive->limits.velMaxW;

	pTraj->trajGen.vMaxW = vMaxW;
	pTraj->trajGen.aMaxW = pDrive->limits.accMaxW;

	switch(pDrive->modeW)
	{
		case DRIVE_MODE_GLOBAL_POS:
		{
			if(fabsf(drive.pos[2] - pTraj->lastTargetPos[2]) > 0.05f)
				LagElementPT1SetState(&pTraj->orientTargetLag, drive.pos[2]);
			else
				LagElementPT1Process(&pTraj->orientTargetLag, drive.pos[2]);

			TrajGeneratorCreateGlobalPosW(&pTraj->trajGen, pTraj->orientTargetLag.lastOut);

			// use numeric difference for acceleration, can lead to discretization issues otherwise
			float lastVel = pReference->trajVelLocal[2];
			float curVel = pTraj->trajGen.trajGlobalVel[2];

			float deltaVel = curVel - lastVel;
			float accW = deltaVel * 1.0f/dT;

			if(accW > pDrive->limits.accMaxW)
				accW = pDrive->limits.accMaxW;
			else if(accW < -pDrive->limits.accMaxW)
				accW = -pDrive->limits.accMaxW;

			pReference->trajPos[2] = pTraj->trajGen.trajGlobalPos[2];
			pReference->trajVelLocal[2] = pTraj->trajGen.trajGlobalVel[2];
			pReference->trajAccLocal[2] = accW;

			TrajGeneratorStepW(&pTraj->trajGen, dT);
		}
		break;
		case DRIVE_MODE_LOCAL_VEL:
		{
			// Trajectory Update
			TrajGeneratorCreateLocalVelW(&pTraj->trajGen, pDrive->localVel[2]);

			// use numeric difference for acceleration, can lead to discretization issues otherwise
			float lastVel = pReference->trajVelLocal[2];
			float curVel = pTraj->trajGen.trajLocalVel[2];

			float deltaVel = curVel - lastVel;
			float accW = deltaVel * 1.0f/dT;

			if(accW > pDrive->limits.accMaxW)
				accW = pDrive->limits.accMaxW;
			else if(accW < -pDrive->limits.accMaxW)
				accW = -pDrive->limits.accMaxW;

			pReference->trajPos[2] = pTraj->trajGen.trajGlobalPos[2];
			pReference->trajVelLocal[2] = pTraj->trajGen.trajLocalVel[2];
			pReference->trajAccLocal[2] = accW;

			TrajGeneratorStepW(&pTraj->trajGen, dT);
		}
		break;
		default:
		{
			TrajGeneratorResetW(&pTraj->trajGen);
		}
		break;
	}

	memcpy(pTraj->lastTargetPos, drive.pos, sizeof(float)*3);
}

void TrajBangBangResetState(TrajBangBang* pTraj, const RobotCtrlState* pState)
{
	TrajGeneratorSetState(&pTraj->trajGen, pState->pos, pState->vel);
	TrajGeneratorReset(&pTraj->trajGen);
}
