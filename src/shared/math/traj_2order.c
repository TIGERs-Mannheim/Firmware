/*
 * traj_2order.c
 *
 *  Created on: 21.05.2016
 *      Author: AndreR
 */

#include "traj_2order.h"
#include "arm_math.h"
#include "vector.h"

#define FIND_XY_DISTRI_PRECISION 0.0000001f

#include <math.h>

typedef struct _TrajSecOrder1DValue
{
	float xdd;
	float xd;
	float x;
} TrajSecOrder1DValue;

static void partValues(const TrajSecOrderPart* pParts, uint8_t numParts, float trajTime, TrajSecOrder1DValue* pVal)
{
	uint8_t i;
	float t;
	const TrajSecOrderPart* pPiece = 0;
	float tPieceStart = 0;

	for(i = 0; i < numParts; i++)
	{
		pPiece = &pParts[i];
		if(trajTime < pPiece->tEnd)
			break;
	}

	if(i == numParts)
	{
		t = pPiece->tEnd-pParts[numParts-2].tEnd;	// trajectory complete, use end time
		pVal->xdd = 0;
	}
	else
	{
		if(i > 0)
			tPieceStart = pParts[i-1].tEnd;

		t = trajTime-tPieceStart;
		pVal->xdd = pPiece->xdd;
	}

	pVal->xd = pPiece->xd0 + pPiece->xdd*t;
	pVal->x = pPiece->x0 + pPiece->xd0*t + 0.5f*pPiece->xdd*t*t;
}

static inline float velChangeToZero(float s0, float v0, float aMax)
{
	float a;

	if(0 >= v0)
		a = aMax;
	else
		a = -aMax;

	float t = -v0/a;
	float s1 = s0 + 0.5f*v0*t;

	return s1;
}

static inline float velTriToZero(float s0, float v0, float v1, float aMax)
{
	float a1;
	float a2;

	if(v1 >= v0)
	{
		a1 = aMax;
		a2 = -aMax;
	}
	else
	{
		a1 = -aMax;
		a2 = aMax;
	}

	float t1 = (v1-v0)/a1;
	float s1 = s0 + 0.5f*(v0+v1)*t1;

	float t2 = -v1/a2;
	float s2 = s1 + 0.5f*v1*t2;

	return s2;
}

static inline void calcTri(TrajSecOrder1D* pTraj,
		float s0, float v0, float s2, float a, uint8_t isPos)
{
	float T2;
	float v1;
	float T1;
	float s1;

	if(isPos)
	{
		// + -
		float sq = (a*(s2-s0)+0.5f*v0*v0)/(a*a);
		if(sq > 0.0f)
			T2 = sqrtf(sq);
		else
			T2 = 0;
		v1 = a*T2;
		T1 = (v1-v0)/a;
		s1 = s0 + (v0+v1)*0.5f*T1;
	}
	else
	{
		// - +
		float sq = (a*(s0-s2)+0.5f*v0*v0)/(a*a);
		if(sq > 0.0f)
			T2 = sqrtf(sq);
		else
			T2 = 0;
		v1 = -a*T2;
		T1 = (v1-v0)/-a;
		s1 = s0 + (v0+v1)*0.5f*T1;
		a *= -1;
	}

	pTraj->parts[0].tEnd = T1;
	pTraj->parts[0].xdd = a;
	pTraj->parts[0].xd0 = v0;
	pTraj->parts[0].x0 = s0;

	pTraj->parts[1].tEnd = T1;
	pTraj->parts[1].xdd = a;
	pTraj->parts[1].xd0 = v1;
	pTraj->parts[1].x0 = s1;

	pTraj->parts[2].tEnd = T1+T2;
	pTraj->parts[2].xdd = -a;
	pTraj->parts[2].xd0 = v1;
	pTraj->parts[2].x0 = s1;
}

static inline void calcTrapz(TrajSecOrder1D* pTraj,
		float s0, float v0, float v1, float s3, float aMax)
{
	float a1;
	float a3;
	float T1;
	float T2;
	float T3;
	float v2;
	float s1;
	float s2;

	if(v0 > v1)
		a1 = -aMax;
	else
		a1 = aMax;

	if(v1 > 0)
		a3 = -aMax;
	else
		a3 = aMax;

	T1 = (v1-v0)/a1;
	v2 = v1;
	T3 = -v2/a3;

	s1 = s0 + 0.5*(v0+v1)*T1;
	s2 = s3 - 0.5*v2*T3;
	T2 = (s2-s1)/v1;

	pTraj->parts[0].tEnd = T1;
	pTraj->parts[0].xdd = a1;
	pTraj->parts[0].xd0 = v0;
	pTraj->parts[0].x0 = s0;

	pTraj->parts[1].tEnd = T1+T2;
	pTraj->parts[1].xdd = 0;
	pTraj->parts[1].xd0 = v1;
	pTraj->parts[1].x0 = s1;

	pTraj->parts[2].tEnd = T1+T2+T3;
	pTraj->parts[2].xdd = a3;
	pTraj->parts[2].xd0 = v2;
	pTraj->parts[2].x0 = s2;
}

void TrajSecOrder1DCreate(TrajSecOrder1D* pTraj, float x0, float xd0, float xTrg, float xdMax, float xddMax)
{
	float sAtZeroAcc = velChangeToZero(x0, xd0, xddMax);

	if(sAtZeroAcc <= xTrg)
	{
		float sEnd = velTriToZero(x0, xd0, xdMax, xddMax);

		if(sEnd >= xTrg)
		{
			// Triangular profile
			calcTri(pTraj, x0, xd0, xTrg, xddMax, 1);
		}
		else
		{
			// Trapezoidal profile
			calcTrapz(pTraj, x0, xd0, xdMax, xTrg, xddMax);
		}
	}
	else
	{
		// even with a full brake we miss xTrg
		float sEnd = velTriToZero(x0, xd0, -xdMax, xddMax);

		if(sEnd <= xTrg)
		{
			// Triangular profile
			calcTri(pTraj, x0, xd0, xTrg, xddMax, 0);
		}
		else
		{
			// Trapezoidal profile
			calcTrapz(pTraj, x0, xd0, -xdMax, xTrg, xddMax);
		}
	}
}

void TrajSecOrder1DValuesAtTime(const TrajSecOrder1D* pTraj, float t, float* pX, float* pXd, float* pXdd)
{
	TrajSecOrder1DValue val;

	partValues(pTraj->parts, TRAJ_2ORDER_PARTS, t, &val);

	if(pX)
		*pX = val.x;
	if(pXd)
		*pXd = val.xd;
	if(pXdd)
		*pXdd = val.xdd;
}

float TrajSecOrder1DGetTotalTime(const TrajSecOrder1D* pTraj)
{
	return pTraj->parts[TRAJ_2ORDER_PARTS-1].tEnd;
}

float TrajSecOrder1DGetFinalX(const TrajSecOrder1D* pTraj)
{
    float t = pTraj->parts[2].tEnd - pTraj->parts[1].tEnd;

    return pTraj->parts[2].x0 + pTraj->parts[2].xd0*t + 0.5*pTraj->parts[2].xdd*t*t;
}

void TrajSecOrder2DCreate(TrajSecOrder2D* pTraj, const float* pX0, const float* pXd0, const float* pXTrg, float xdMax, float xddMax)
{
	TrajSecOrder1D* pPosX = (TrajSecOrder1D*)pTraj->x;
	TrajSecOrder1D* pPosY = (TrajSecOrder1D*)pTraj->y;

	float inc = M_PI/8.0f;
	float alpha = M_PI/4.0f;

	// binary search, some iterations (fixed)
	while(inc > FIND_XY_DISTRI_PRECISION)
	{
	    float cA = arm_cos_f32(alpha);
	    float sA = arm_sin_f32(alpha);

	    TrajSecOrder1DCreate(pPosX, pX0[0], pXd0[0], pXTrg[0], xdMax*cA, xddMax*cA);
	    TrajSecOrder1DCreate(pPosY, pX0[1], pXd0[1], pXTrg[1], xdMax*sA, xddMax*sA);

	    if(pPosX->parts[TRAJ_2ORDER_PARTS-1].tEnd > pPosY->parts[TRAJ_2ORDER_PARTS-1].tEnd)
	        alpha = alpha-inc;
	    else
	        alpha = alpha+inc;

	    inc *= 0.5f;
	}
}

void TrajSecOrder2DValuesAtTime(const TrajSecOrder2D* pTraj, float t, float* pX, float* pXd, float* pXdd)
{
	TrajSecOrder1DValue x;
	TrajSecOrder1DValue y;

	partValues(pTraj->x, TRAJ_2ORDER_PARTS, t, &x);
	partValues(pTraj->y, TRAJ_2ORDER_PARTS, t, &y);

	if(pX)
	{
		pX[0] = x.x;
		pX[1] = y.x;
	}

	if(pXd)
	{
		pXd[0] = x.xd;
		pXd[1] = y.xd;
	}

	if(pXdd)
	{
		pXdd[0] = x.xdd;
		pXdd[1] = y.xdd;
	}
}

float TrajSecOrder2DGetTotalTime(const TrajSecOrder2D* pTraj)
{
	float xEnd = pTraj->x[TRAJ_2ORDER_PARTS-1].tEnd;
	float yEnd = pTraj->y[TRAJ_2ORDER_PARTS-1].tEnd;

	if(xEnd > yEnd)
		return xEnd;
	else
		return yEnd;
}

void TrajSecOrder2DAsyncCreate(TrajSecOrder2DAsync* pTraj, const float* pX0, const float* pXd0, const float* pXTrg, float xdMax, float xddMax, float rotation)
{
	pTraj->initialPos[0] = pX0[0];
	pTraj->initialPos[1] = pX0[1];
	pTraj->rotation = rotation;

	float startToTarget[2];
	startToTarget[0] = pXTrg[0] - pX0[0];
	startToTarget[1] = pXTrg[1] - pX0[1];
	Vector2fRotatePtr(-rotation, startToTarget[0], startToTarget[1], startToTarget);

	float xd0Rotated[2];
	Vector2fRotatePtr(-rotation, pXd0[0], pXd0[1], xd0Rotated);

	TrajSecOrder1D* pPosX = (TrajSecOrder1D*)pTraj->x;
	TrajSecOrder1D* pPosY = (TrajSecOrder1D*)pTraj->y;

	float inc = M_PI/8.0f;
	float alpha = M_PI/4.0f;

	// binary search, some iterations (fixed)
	while(inc > FIND_XY_DISTRI_PRECISION)
	{
		float asyncAlpha = alpha + ((M_PI_2 - alpha) * 0.5f);
	    float cA = arm_cos_f32(asyncAlpha);
	    float sA = arm_sin_f32(asyncAlpha);

	    TrajSecOrder1DCreate(pPosX, 0, xd0Rotated[0], startToTarget[0], xdMax*cA, xddMax*cA);
	    TrajSecOrder1DCreate(pPosY, 0, xd0Rotated[1], startToTarget[1], xdMax*sA, xddMax*sA);

	    if(pPosX->parts[TRAJ_2ORDER_PARTS-1].tEnd > pPosY->parts[TRAJ_2ORDER_PARTS-1].tEnd)
	        alpha = alpha-inc;
	    else
	        alpha = alpha+inc;

	    inc *= 0.5f;
	}
}

void TrajSecOrder2DAsyncValuesAtTime(const TrajSecOrder2DAsync* pTraj, float t, float* pX, float* pXd, float* pXdd)
{
	TrajSecOrder1DValue x;
	TrajSecOrder1DValue y;

	partValues(pTraj->x, TRAJ_2ORDER_PARTS, t, &x);
	partValues(pTraj->y, TRAJ_2ORDER_PARTS, t, &y);

	if(pX)
	{
		Vector2fRotatePtr(pTraj->rotation, x.x, y.x, pX);
		pX[0] += pTraj->initialPos[0];
		pX[1] += pTraj->initialPos[1];
	}

	if(pXd)
	{
		Vector2fRotatePtr(pTraj->rotation, x.xd, y.xd, pXd);
	}

	if(pXdd)
	{
		Vector2fRotatePtr(pTraj->rotation, x.xdd, y.xdd, pXdd);
	}
}

float TrajSecOrder2DAsyncGetTotalTime(const TrajSecOrder2DAsync* pTraj)
{
	float xEnd = pTraj->x[TRAJ_2ORDER_PARTS-1].tEnd;
	float yEnd = pTraj->y[TRAJ_2ORDER_PARTS-1].tEnd;

	if(xEnd > yEnd)
		return xEnd;
	else
		return yEnd;
}
