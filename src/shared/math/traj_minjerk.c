/*
 * traj_minjerk.c
 *
 *  Created on: 21.05.2016
 *      Author: AndreR
 */

#include "traj_minjerk.h"
#include <string.h>

static inline void poly5(float* pOut, float si, float vi, float ai, float sf, float vf, float t)
{
	pOut[0] = si;
	pOut[1] = vi;
	pOut[2] = ai*0.5f;

    float t2 = t*t;
    float t3 = t2*t;
    float t4 = t3*t;
    float t5 = t4*t;

    float b1 = sf - (si + vi*t + 0.5*ai*t2);
    float b2 = vf - (vi + ai*t);
    float b3 = -ai;

    float div1 = (2*t5);
    float div2 = (4*t3);
    float div3 = (6*t);

    if(div1 != 0.0f)
    	pOut[5] = (b3*t2-6*b2*t+12*b1)/div1;
    else
    	pOut[5] = 0;

    if(div2 != 0.0f)
    	pOut[4] = -(10*pOut[5]*t4-b3*t+2*b2)/div2;
    else
    	pOut[4] = 0;

    if(div3 != 0.0f)
    	pOut[3] = -(20*pOut[5]*t3+12*pOut[4]*t2-b3)/div3;
    else
    	pOut[3] = 0;
}

static inline void poly5Value(const float* pPoly, float t, float* pS, float* pV, float* pA, float* pJ)
{
    float t2 = t*t;
    float t3 = t2*t;
    float t4 = t3*t;
    float t5 = t4*t;

	if(pS)
		*pS = pPoly[0] + pPoly[1]*t + pPoly[2]*t2 + pPoly[3]*t3 + pPoly[4]*t4 + pPoly[5]*t5;

	if(pV)
		*pV = pPoly[1] + 2*pPoly[2]*t + 3*pPoly[3]*t2 + 4*pPoly[4]*t3 + 5*pPoly[5]*t4;

	if(pA)
		*pA = 2*pPoly[2] + 6*pPoly[3]*t + 12*pPoly[4]*t2 + 20*pPoly[5]*t3;

	if(pJ)
		*pJ = 6*pPoly[3] + 24*pPoly[4]*t + 60*pPoly[5]*t2;
}

void TrajMinJerk1DCreate(TrajMinJerk1D* pTraj, float s0, float v0, float a0, float sTrg, float vMax, float aAvg)
{
	TrajSecOrder1DCreate(&pTraj->traj, s0, v0, sTrg, vMax, aAvg);

    poly5(pTraj->polyA, s0, v0, a0, pTraj->traj.parts[1].x0, pTraj->traj.parts[1].xd0, pTraj->traj.parts[0].tEnd);

    if(pTraj->traj.parts[1].tEnd < 1e-3f)
    {
    	pTraj->traj.parts[0].tEnd = 0;
    	pTraj->traj.parts[1].tEnd = 0;

		poly5(pTraj->polyB, s0, v0, a0, TrajSecOrder1DGetFinalX(&pTraj->traj),
				0, pTraj->traj.parts[2].tEnd - pTraj->traj.parts[1].tEnd);
    }
    else
    {
		poly5(pTraj->polyB, pTraj->traj.parts[2].x0, pTraj->traj.parts[2].xd0, 0, TrajSecOrder1DGetFinalX(&pTraj->traj),
				0, pTraj->traj.parts[2].tEnd - pTraj->traj.parts[1].tEnd);
    }
}

void TrajMinJerk1DCreateEx(TrajMinJerk1D* pTraj, float bbS0, float bbV0, float mjS0, float mjV0, float mjA0, float sTrg, float vMax, float aAvg)
{
	TrajSecOrder1DCreate(&pTraj->traj, bbS0, bbV0, sTrg, vMax, aAvg);

    poly5(pTraj->polyA, mjS0, mjV0, mjA0, pTraj->traj.parts[1].x0, pTraj->traj.parts[1].xd0, pTraj->traj.parts[0].tEnd);

    if(pTraj->traj.parts[1].tEnd < 1e-3f)
    {
    	pTraj->traj.parts[0].tEnd = 0;
    	pTraj->traj.parts[1].tEnd = 0;

		poly5(pTraj->polyB, mjS0, mjV0, mjA0, TrajSecOrder1DGetFinalX(&pTraj->traj),
				0, pTraj->traj.parts[2].tEnd - pTraj->traj.parts[1].tEnd);
    }
    else
    {
		poly5(pTraj->polyB, pTraj->traj.parts[2].x0, pTraj->traj.parts[2].xd0, 0, TrajSecOrder1DGetFinalX(&pTraj->traj),
				0, pTraj->traj.parts[2].tEnd - pTraj->traj.parts[1].tEnd);
    }
}

static inline void createFromSecOrder(TrajMinJerk1D* pTraj, const TrajSecOrder1D* pSecOrder, float s0, float v0, float a0)
{
	memcpy(&pTraj->traj, pSecOrder, sizeof(TrajSecOrder1D));

    poly5(pTraj->polyA, s0, v0, a0, pTraj->traj.parts[1].x0, pTraj->traj.parts[1].xd0, pTraj->traj.parts[0].tEnd);

    if(pTraj->traj.parts[1].tEnd < 1e-3f)
    {
    	pTraj->traj.parts[0].tEnd = 0;
    	pTraj->traj.parts[1].tEnd = 0;

		poly5(pTraj->polyB, s0, v0, a0, TrajSecOrder1DGetFinalX(&pTraj->traj),
				0, pTraj->traj.parts[2].tEnd - pTraj->traj.parts[1].tEnd);
    }
    else
    {
		poly5(pTraj->polyB, pTraj->traj.parts[2].x0, pTraj->traj.parts[2].xd0, 0, TrajSecOrder1DGetFinalX(&pTraj->traj),
				0, pTraj->traj.parts[2].tEnd - pTraj->traj.parts[1].tEnd);
    }
}

void TrajMinJerk1DValuesAtTime(const TrajMinJerk1D* pTraj, float t, float* pS, float* pV, float* pA, float* pJ)
{
	if(t < pTraj->traj.parts[0].tEnd)
	{
		poly5Value(pTraj->polyA, t, pS, pV, pA, pJ);
	}
	else if(t < pTraj->traj.parts[1].tEnd)
	{
		TrajSecOrder1DValuesAtTime(&pTraj->traj, t, pS, pV, pA);

		if(pJ)
			*pJ = 0;
	}
	else if(t < pTraj->traj.parts[2].tEnd)
	{
		poly5Value(pTraj->polyB, t - pTraj->traj.parts[1].tEnd, pS, pV, pA, pJ);
	}
	else
	{
		if(pS)
			*pS = TrajSecOrder1DGetFinalX(&pTraj->traj);

		if(pV)
			*pV = 0;

		if(pA)
			*pA = 0;

		if(pJ)
			*pJ = 0;
	}
}

void TrajMinJerk1DBBValuesAtTime(const TrajMinJerk1D* pTraj, float t, float* pS, float* pV, float* pA)
{
	TrajSecOrder1DValuesAtTime(&pTraj->traj, t, pS, pV, pA);
}

float TrajMinJerk1DGetTotalTime(const TrajMinJerk1D* pTraj)
{
	return pTraj->traj.parts[TRAJ_2ORDER_PARTS-1].tEnd;
}

void TrajMinJerk2DCreate(TrajMinJerk2D* pTraj, float* pS0, float* pV0, float* pA0, float* pSTrg, float vMax, float aAvg)
{
	TrajSecOrder2D xyTraj;

	TrajSecOrder2DCreate(&xyTraj, pS0, pV0, pSTrg, vMax, aAvg);

	createFromSecOrder(&pTraj->x, (TrajSecOrder1D*)xyTraj.x, pS0[0], pV0[0], pA0[0]);
	createFromSecOrder(&pTraj->y, (TrajSecOrder1D*)xyTraj.y, pS0[1], pV0[1], pA0[1]);
}

void TrajMinJerk2DCreateEx(TrajMinJerk2D* pTraj, float* pBBS0, float* pBBV0, float* pMJS0, float* pMJV0, float* pMJA0, float* pSTrg, float vMax, float aAvg)
{
	TrajSecOrder2D xyTraj;

	TrajSecOrder2DCreate(&xyTraj, pBBS0, pBBV0, pSTrg, vMax, aAvg);

	createFromSecOrder(&pTraj->x, (TrajSecOrder1D*)xyTraj.x, pMJS0[0], pMJV0[0], pMJA0[0]);
	createFromSecOrder(&pTraj->y, (TrajSecOrder1D*)xyTraj.y, pMJS0[1], pMJV0[1], pMJA0[1]);
}

void TrajMinJerk2DValuesAtTime(const TrajMinJerk2D* pTraj, float t, float* pS, float* pV, float* pA, float* pJ)
{
	TrajMinJerk1DValuesAtTime(&pTraj->x, t, pS, pV, pA, pJ);

	if(pS != 0)
		++pS;

	if(pV != 0)
		++pV;

	if(pA != 0)
		++pA;

	if(pJ != 0)
		++pJ;

	TrajMinJerk1DValuesAtTime(&pTraj->y, t, pS, pV, pA, pJ);
}

void TrajMinJerk2DBBValuesAtTime(const TrajMinJerk2D* pTraj, float t, float* pS, float* pV, float* pA)
{
	TrajSecOrder1DValuesAtTime(&pTraj->x.traj, t, pS, pV, pA);

	if(pS != 0)
		++pS;

	if(pV != 0)
		++pV;

	if(pA != 0)
		++pA;

	TrajSecOrder1DValuesAtTime(&pTraj->y.traj, t, pS, pV, pA);
}

float TrajMinJerk2DGetTotalTime(const TrajMinJerk2D* pTraj)
{
	float tx = pTraj->x.traj.parts[TRAJ_2ORDER_PARTS-1].tEnd;
	float ty = pTraj->y.traj.parts[TRAJ_2ORDER_PARTS-1].tEnd;

	if(tx > ty)
		return tx;
	else
		return ty;
}
