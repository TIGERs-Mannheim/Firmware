/*
 * trajectory_orient.h
 *
 *  Created on: 26.05.2015
 *      Author: AndreR
 */

#ifndef TRAJECTORY_ORIENT_H_
#define TRAJECTORY_ORIENT_H_

#include "trajectory.h"

#define TRAJECTORY_ORIENT_PARTS 12

typedef struct _TrajectoryInputOrient
{
	float s0;
	float s1;
	float w0;
	float accW;
	float brkW;
	float wMax;
	float xyMax;
} TrajectoryInputOrient;

typedef struct _TrajectoryOrient
{
	TrajectoryPart parts[TRAJECTORY_ORIENT_PARTS];
	TrajectoryInputOrient input;
	uint8_t numParts;
} TrajectoryOrient;

void TrajectoryOrientCreate(TrajectoryOrient* pTrajW, float s0, float s1, float w0, float accW, float brkW,
		float wMax, const Trajectory2D* pTrajXY, const float xyMax);
void TrajectoryOrientCreateAngles(TrajectoryOrient* pTrajW, float s0, float s1, float w0, float accW, float brkW,
		float wMax, const Trajectory2D* pTrajXY, const float xyMax);
void TrajectoryOrientValuesAtEnd(TrajectoryOrient* pTraj, float* pAcc, float* pVel, float* pPos);
void TrajectoryOrientValuesAtTime(TrajectoryOrient* pTraj, float t, float* pAcc, float* pVel, float* pPos);
float TrajectoryOrientGetTotalTime(TrajectoryOrient* pTraj);
void TrajectoryOrientPrint(TrajectoryOrient* pTrajW);

#endif /* TRAJECTORY_ORIENT_H_ */
