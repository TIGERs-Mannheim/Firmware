/*
 * traj_minjerk.h
 *
 *  Created on: 21.05.2016
 *      Author: AndreR
 */

#ifndef TRAJ_MINJERK_H_
#define TRAJ_MINJERK_H_

#include "traj_2order.h"

typedef struct _TrajMinJerk1D
{
	TrajSecOrder1D traj;
	float polyA[6];
	float polyB[6];
} TrajMinJerk1D;

typedef struct _TrajMinJerk2D
{
	TrajMinJerk1D x;
	TrajMinJerk1D y;
} TrajMinJerk2D;

void	TrajMinJerk1DCreate(TrajMinJerk1D* pTraj, float s0, float v0, float a0, float sTrg, float vMax, float aAvg);
void	TrajMinJerk1DCreateEx(TrajMinJerk1D* pTraj, float bbS0, float bbV0, float mjS0, float mjV0, float mjA0, float sTrg, float vMax, float aAvg);
void	TrajMinJerk1DValuesAtTime(const TrajMinJerk1D* pTraj, float t, float* pS, float* pV, float* pA, float* pJ);
void	TrajMinJerk1DBBValuesAtTime(const TrajMinJerk1D* pTraj, float t, float* pS, float* pV, float* pA);
float	TrajMinJerk1DGetTotalTime(const TrajMinJerk1D* pTraj);

void	TrajMinJerk2DCreate(TrajMinJerk2D* pTraj, float* pS0, float* pV0, float* pA0, float* pSTrg, float vMax, float aAvg);
void	TrajMinJerk2DCreateEx(TrajMinJerk2D* pTraj, float* pBBS0, float* pBBV0, float* pMJS0, float* pMJV0, float* pMJA0, float* pSTrg, float vMax, float aAvg);
void	TrajMinJerk2DValuesAtTime(const TrajMinJerk2D* pTraj, float t, float* pS, float* pV, float* pA, float* pJ);
void	TrajMinJerk2DBBValuesAtTime(const TrajMinJerk2D* pTraj, float t, float* pS, float* pV, float* pA);
float	TrajMinJerk2DGetTotalTime(const TrajMinJerk2D* pTraj);

#endif /* TRAJ_MINJERK_H_ */
