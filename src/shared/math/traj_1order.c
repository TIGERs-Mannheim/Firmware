/*
 * traj_1order.c
 *
 *  Created on: 28.08.2020
 *      Author: AndreR
 */

#include "traj_1order.h"
#include <math.h>

void TrajFirstOrder1DCreate(TrajFirstOrder1D* pTraj, float x0, float xTrg, float xdMax)
{
	float toTargetVel = xTrg - x0;

	float toTargetNorm = fabsf(toTargetVel);

	float scale;
	if(toTargetNorm < 1e-6)
		scale = 0.0f;
	else
		scale = 1.0f/toTargetNorm * xdMax;

	pTraj->x0 = x0;
	pTraj->xd = toTargetVel * scale;

	pTraj->tEnd = toTargetNorm/xdMax;
}

void TrajFirstOrder1DValuesAtTime(const TrajFirstOrder1D* pTraj, float t, float* pX, float* pXd)
{
	if(t > pTraj->tEnd)
	{
		*pX = pTraj->x0 + pTraj->xd * pTraj->tEnd;
		*pXd = 0;
	}
	else
	{
		*pX = pTraj->x0 + pTraj->xd * t;
		*pXd = pTraj->xd;
	}
}

float TrajFirstOrder1DGetTotalTime(const TrajFirstOrder1D* pTraj)
{
	return pTraj->tEnd;
}

void TrajFirstOrder2DCreate(TrajFirstOrder2D* pTraj, const float* pX0, const float* pXTrg, float xdMax)
{
	float toTargetVel[2];
	toTargetVel[0] = pXTrg[0] - pX0[0];
	toTargetVel[1] = pXTrg[1] - pX0[1];

	float toTargetNorm = sqrtf(toTargetVel[0]*toTargetVel[0] + toTargetVel[1]*toTargetVel[1]);

	float scale;
	if(toTargetNorm < 1e-6)
		scale = 0.0f;
	else
		scale = 1.0f/toTargetNorm * xdMax;

	pTraj->x0[0] = pX0[0];
	pTraj->x0[1] = pX0[1];
	pTraj->xd[0] = toTargetVel[0] * scale;
	pTraj->xd[1] = toTargetVel[1] * scale;

	pTraj->tEnd = toTargetNorm/xdMax;
}

void TrajFirstOrder2DValuesAtTime(const TrajFirstOrder2D* pTraj, float t, float* pX, float* pXd)
{
	if(t > pTraj->tEnd)
	{
		pX[0] = pTraj->x0[0] + pTraj->xd[0] * pTraj->tEnd;
		pX[1] = pTraj->x0[1] + pTraj->xd[1] * pTraj->tEnd;
		pXd[0] = 0;
		pXd[1] = 0;
	}
	else
	{
		pX[0] = pTraj->x0[0] + pTraj->xd[0] * t;
		pX[1] = pTraj->x0[1] + pTraj->xd[1] * t;
		pXd[0] = pTraj->xd[0];
		pXd[1] = pTraj->xd[1];
	}
}

float TrajFirstOrder2DGetTotalTime(const TrajFirstOrder2D* pTraj)
{
	return pTraj->tEnd;
}
