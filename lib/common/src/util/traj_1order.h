/*
 * traj_1order.h
 *
 *  Created on: 28.08.2020
 *      Author: AndreR
 */

#pragma once

typedef struct _TrajFirstOrder1D
{
	float tEnd;

	// xd(t) = xd
	// x(t) = x0 + xd*t
	float xd;	// x dot, control signal
	float x0;	// x initial value
} TrajFirstOrder1D;

typedef struct _TrajFirstOrder2D
{
	float tEnd;
	float xd[2];
	float x0[2];
} TrajFirstOrder2D;

void	TrajFirstOrder1DCreate(TrajFirstOrder1D* pTraj, float x0, float xTrg, float xdMax);
void	TrajFirstOrder1DValuesAtTime(const TrajFirstOrder1D* pTraj, float t, float* pX, float* pXd);
float	TrajFirstOrder1DGetTotalTime(const TrajFirstOrder1D* pTraj);

void	TrajFirstOrder2DCreate(TrajFirstOrder2D* pTraj, const float* pX0, const float* pXTrg, float xdMax);
void	TrajFirstOrder2DValuesAtTime(const TrajFirstOrder2D* pTraj, float t, float* pX, float* pXd);
float	TrajFirstOrder2DGetTotalTime(const TrajFirstOrder2D* pTraj);
