/*
 * trajectory.h
 *
 *  Created on: 10.03.2015
 *      Author: AndreR
 */

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include <stdint.h>

#define TRAJECTORY_PARTS	4

typedef struct _TrajectoryInput2D
{
	float pS0[3];
	float pS1[3];
	float pV0[3];
	float acc;
	float brk;
	float vmax;
} TrajectoryInput2D;

typedef struct _TrajectoryPart
{
	float tEnd;

	// a(t) = acc
	// v(t) = acc*t + v0
	// s(t) = 0.5f*acc*t^2 + v0*t + s0
	float acc;
	float v0;
	float s0;
} TrajectoryPart;

typedef struct _Trajectory1D
{
	TrajectoryPart parts[TRAJECTORY_PARTS];
} Trajectory1D;

typedef struct _Trajectory2D
{
	TrajectoryPart x[TRAJECTORY_PARTS];
	TrajectoryPart y[TRAJECTORY_PARTS];
	TrajectoryInput2D input;
} Trajectory2D;

void TrajectoryPartValues(const TrajectoryPart* pParts, uint8_t numParts, float trajTime, float* pAcc, float* pVel, float* pPos);

void Trajectory1DCreate(Trajectory1D* pTraj, float s0, float s1, float v0, float acc, float brk, float vmax);
void Trajectory1DCreateCircleAware(Trajectory1D* pTraj, float s0, float s1, float v0, float acc, float brk, float vmax);
void Trajectory1DValuesAtTime(Trajectory1D* pTraj, float t, float* pAcc, float* pVel, float* pPos);
void Trajectory1DValuesAtEnd(Trajectory1D* pTraj, float* pAcc, float* pVel, float* pPos);
float Trajectory1DGetTotalTime(const Trajectory1D* pTraj);

void Trajectory2DCreate(Trajectory2D* pTraj, float* pS0, float* pS1, float* pV0, float acc, float brk, float vmax);
void Trajectory2DValuesAtTime(Trajectory2D* pTraj, float t, float* pAcc, float* pVel, float* pPos);
void Trajectory2DValuesAtEnd(Trajectory2D* pTraj, float* pAcc, float* pVel, float* pPos);
float Trajectory2DGetTotalTime(const Trajectory2D* pTraj);

#endif /* TRAJECTORY_H_ */
