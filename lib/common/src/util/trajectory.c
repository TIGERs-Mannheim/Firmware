/*
 * trajectory.c
 *
 *  Created on: 10.03.2015
 *      Author: AndreR
 */

#include "trajectory.h"
#include "sys_time.h"
#include "util/console.h"
#include "arm_mat_util_f32.h"
#include "util/angle_math.h"

#include <math.h>
#include <string.h>

// this is actually not used
typedef enum _TrajectoryType
{
	TRAJ_TYPE_A,
	TRAJ_TYPE_B,
	TRAJ_TYPE_C,
	TRAJ_TYPE_D,
	TRAJ_TYPE_E,
	TRAJ_TYPE_F,
	TRAJ_TYPE_G
} ETrajectoryType;

static void calcVelPos(TrajectoryPart* pParts, float s0, float v0);
static ETrajectoryType traj1Dfast(TrajectoryPart* pParts, float v0, float s, float acc, float brk, float vmax);
static void traj2D(TrajectoryPart* pX, TrajectoryPart* pY, float* pV0, float* pS, float acc, float brk, float vmax);

void TrajectoryPartValues(const TrajectoryPart* pParts, uint8_t numParts, float trajTime, float* pAcc, float* pVel, float* pPos)
{
	uint8_t i;
	float t;
	const TrajectoryPart* pPiece = 0;
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
		if((uint32_t)pAcc > 4)
			*pAcc = 0;
	}
	else
	{
		if(i > 0)
			tPieceStart = pParts[i-1].tEnd;

		t = trajTime-tPieceStart;
		if((uint32_t)pAcc > 4)
			*pAcc = pPiece->acc;
	}

	if((uint32_t)pVel > 4)
		*pVel = pPiece->v0 + pPiece->acc*t;
	if((uint32_t)pPos > 4)
		*pPos = pPiece->s0 + pPiece->v0*t + 0.5f*pPiece->acc*t*t;
}

void Trajectory1DCreate(Trajectory1D* pTraj, float s0, float s1, float v0, float acc, float brk, float vmax)
{
	float sDiff = s1-s0;

	traj1Dfast(pTraj->parts, v0, sDiff, acc, brk, vmax);

	calcVelPos(pTraj->parts, s0, v0);
}

void Trajectory1DCreateCircleAware(Trajectory1D* pTraj, float s0, float s1, float v0, float acc, float brk, float vmax)
{
	float sDiffShort = s1-s0;
	AngleNormalizePtr(&sDiffShort);
	float sDiffLong;
	if(sDiffShort < 0)
		sDiffLong = sDiffShort+2*M_PI;
	else
		sDiffLong = sDiffShort-2*M_PI;

	Trajectory1D longTraj;

	traj1Dfast(pTraj->parts, v0, sDiffShort, acc, brk, vmax);
	traj1Dfast(longTraj.parts, v0, sDiffLong, acc, brk, vmax);

	if(Trajectory1DGetTotalTime(&longTraj) < Trajectory1DGetTotalTime(pTraj))
		memcpy(pTraj, &longTraj, sizeof(Trajectory1D));

	calcVelPos(pTraj->parts, s0, v0);
}

void Trajectory1DValuesAtEnd(Trajectory1D* pTraj, float* pAcc, float* pVel, float* pPos)
{
	TrajectoryPartValues(pTraj->parts, TRAJECTORY_PARTS, pTraj->parts[3].tEnd+0.1f, pAcc, pVel, pPos);
}

void Trajectory1DValuesAtTime(Trajectory1D* pTraj, float t, float* pAcc, float* pVel, float* pPos)
{
	TrajectoryPartValues(pTraj->parts, TRAJECTORY_PARTS, t, pAcc, pVel, pPos);
}

float Trajectory1DGetTotalTime(const Trajectory1D* pTraj)
{
	return pTraj->parts[TRAJECTORY_PARTS-1].tEnd;
}

void Trajectory2DCreate(Trajectory2D* pTraj, float* pS0, float* pS1, float* pV0, float acc, float brk, float vmax)
{
	memcpy(pTraj->input.pS0, pS0, sizeof(float) * 3);
	memcpy(pTraj->input.pS1, pS1, sizeof(float) * 3);
	memcpy(pTraj->input.pV0, pV0, sizeof(float) * 3);
	pTraj->input.acc = acc;
	pTraj->input.brk = brk;
	pTraj->input.vmax = vmax;

	float sDiff[2] = {pS1[0]-pS0[0], pS1[1]-pS0[1]};

	traj2D(pTraj->x, pTraj->y, pV0, sDiff, acc, brk, vmax);

	calcVelPos(pTraj->x, pS0[0], pV0[0]);
	calcVelPos(pTraj->y, pS0[1], pV0[1]);
}

void Trajectory2DValuesAtEnd(Trajectory2D* pTraj, float* pAcc, float* pVel, float* pPos)
{
	TrajectoryPartValues(pTraj->x, TRAJECTORY_PARTS, pTraj->x[3].tEnd+0.1f, pAcc+0, pVel+0, pPos+0);
	TrajectoryPartValues(pTraj->y, TRAJECTORY_PARTS, pTraj->y[3].tEnd+0.1f, pAcc+1, pVel+1, pPos+1);
}

void Trajectory2DValuesAtTime(Trajectory2D* pTraj, float t, float* pAcc, float* pVel, float* pPos)
{
	TrajectoryPartValues(pTraj->x, TRAJECTORY_PARTS, t, pAcc+0, pVel+0, pPos+0);
	TrajectoryPartValues(pTraj->y, TRAJECTORY_PARTS, t, pAcc+1, pVel+1, pPos+1);
}

float Trajectory2DGetTotalTime(const Trajectory2D* pTraj)
{
	float xEnd = pTraj->x[TRAJECTORY_PARTS-1].tEnd;
	float yEnd = pTraj->y[TRAJECTORY_PARTS-1].tEnd;

	if(xEnd > yEnd)
		return xEnd;
	else
		return yEnd;
}

// and here come the magic generation functions
// Warning: math ahead!

static float dist(float v0, float v1, float acc)
{
	float t = fabsf(v0-v1)/acc;
	return 0.5f*(v0+v1)*t;
}

#define neg(v0, v1) dist(v0, v1, brk)
#define pos(v0, v1) dist(v0, v1, acc)

static ETrajectoryType traj1Dfast(TrajectoryPart* pParts, float v0, float s, float acc, float brk, float vmax)
{
	ETrajectoryType type;

	if(s < 0)
	{
		type = traj1Dfast(pParts, -v0, -s, acc, brk, vmax);

		for(uint8_t i = 0; i < TRAJECTORY_PARTS; i++)
			pParts[i].acc = -pParts[i].acc;

		return type;
	}

	float T1 = 0;
	float T2 = 0;
	float T3 = 0;
	float T4 = 0;
	float* Q = 0;

	if(vmax == 0.0f)
	{
		type = TRAJ_TYPE_A;
		Q = (float[4]){0, 0, 0, 0};
	}
	else
	{
		if(v0 >= 0)
		{
			float s_n = neg(v0, 0);

			if(s_n > s)
			{
				float s_npn = neg(v0, 0) + pos(0, -vmax) + neg(-vmax, 0);
				if(s_npn > s)
				{
					// Case G
					type = TRAJ_TYPE_G;
					T1=v0/brk;
					T2=vmax/acc;
					T4=vmax/brk;
					T3=(v0*T1-vmax*T4-vmax*T2-2*s)/(2*vmax);
					Q = (float[4]){-brk, -acc, 0, brk};
				}
				else
				{
					// Case F
					type = TRAJ_TYPE_F;
					T1=v0/brk;
					T3=(acc*(v0*v0-2*brk*s))/(brk*brk*(brk+acc));
					if(T3 <= 0.0f)
					{
						T3 = 0.0f;
						T2 = 0.0f;
					}
					else
					{
						T3 = sqrtf(T3);
						float v1=-brk*T3;
						T2=-(v1*T3+v0*T1-2*s)/v1;
					}
					Q = (float[4]){-brk, -acc, brk, 0};
				}
			}
			else
			{
				if(v0 > vmax)
				{
					// Case C
					type = TRAJ_TYPE_C;
					T1=-(vmax-v0)/brk;
					T2=-(v0*v0-2*brk*s)/(2*brk*vmax);
					T3=vmax/brk;
					Q = (float[4]){-brk, 0, -brk, 0};
				}
				else
				{
					float s_pn = pos(v0, vmax) + neg(vmax, 0);
					if(s_pn > s)
					{
						// Case A
						type = TRAJ_TYPE_A;
						T2=sqrtf((v0*v0+2*acc*s)/(brk*(brk+acc)));
						float v1=brk*T2;
						T1=(v1-v0)/acc;
						Q = (float[4]){acc, -brk, 0, 0};
					}
					else
					{
						// Case B
						type = TRAJ_TYPE_B;
						T1=(vmax-v0)/acc;
						T3=vmax/brk;
						T2=-(vmax*T3+(vmax+v0)*T1-2*s)/(2*vmax);
						Q = (float[4]){acc, 0, -brk, 0};
					}
				}
			}
		}
		else
		{
			float s_npn = neg(v0, 0) + pos(0, vmax) + neg(vmax, 0);
			if(s_npn > s)
			{
				// Case D
				type = TRAJ_TYPE_D;
				T1=-v0/brk;
				T3=(acc*(v0*v0+2*brk*s))/(brk*brk*(brk+acc));

				if(T3 <= 0.0f)
				{
					T3 = 0.0f;
					T2 = 0.0f;
				}
				else
				{
					T3 = sqrtf(T3);
					float v1=brk*T3;
					T2=-(v1*T3+v0*T1-2*s)/v1;
				}
				Q = (float[4]){brk, acc, -brk, 0};
			}
			else
			{
				// Case E
				type = TRAJ_TYPE_E;
				T1=-v0/brk;
				T2=vmax/acc;
				T4=vmax/brk;
				T3=-(vmax*T4+vmax*T2+v0*T1-2*s)/(2*vmax);
				Q = (float[4]){brk, acc, 0, -brk};
			}
		}
	}

	pParts[0].acc = Q[0];
	pParts[0].tEnd = T1;
	pParts[1].acc = Q[1];
	pParts[1].tEnd = T1+T2;
	pParts[2].acc = Q[2];
	pParts[2].tEnd = T1+T2+T3;
	pParts[3].acc = Q[3];
	pParts[3].tEnd = T1+T2+T3+T4;

	return type;
}

static void traj2D(TrajectoryPart* pX, TrajectoryPart* pY, float* pV0, float* pS, float acc, float brk, float vmax)
{
	float inc = M_PI/8.0f;
	float alpha = M_PI/4.0f;

	// binary search, some iterations (fixed)
	while(inc > 0.000001f)
	{
	    float cA = arm_cos_f32(alpha);
	    float sA = arm_sin_f32(alpha);

	    traj1Dfast(pX, pV0[0], pS[0], acc*cA, brk*cA, vmax*cA);
	    traj1Dfast(pY, pV0[1], pS[1], acc*sA, brk*sA, vmax*sA);

	    if(pX[TRAJECTORY_PARTS-1].tEnd > pY[TRAJECTORY_PARTS-1].tEnd)
	        alpha = alpha-inc;
	    else
	        alpha = alpha+inc;

	    inc *= 0.5f;
	}
}

static void calcVelPos(TrajectoryPart* pParts, float s0, float v0)
{
	TrajectoryPart* pFirst = pParts;

	pFirst->v0 = v0;
	pFirst->s0 = s0;

	for(uint8_t i = 1; i < TRAJECTORY_PARTS; i++)
	{
		TrajectoryPart* pCur = &pParts[i];
		TrajectoryPart* pPrev = &pParts[i-1];

		float tStart = 0;
		if(i > 1)
			tStart = pParts[i-2].tEnd;

		float dT = pPrev->tEnd - tStart;

		pCur->v0 = pPrev->v0 + pPrev->acc*dT;
		pCur->s0 = pPrev->s0 + pPrev->v0*dT + 0.5f*pPrev->acc*dT*dT;
	}
}
