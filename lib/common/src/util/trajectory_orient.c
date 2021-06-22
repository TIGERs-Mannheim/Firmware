/*
 * trajectory_orient.c
 *
 *  Created on: 26.05.2015
 *      Author: AndreR
 */

#include "ch.h"
#include <string.h>

#include "trajectory_orient.h"
#include "arm_mat_util_f32.h"
#include "util/angle_math.h"
#include "util/console.h"

#define COMBINE_THRESHOLD 0.01f

static void calcOrient(TrajectoryOrient* pTrajW, float s0);
static void getOrientLimit(const Trajectory2D* pTrajXY, float accW, float brkW, float wMax,
		float xyMax, arm_matrix_instance_f32* pMatLimit, uint8_t* pLimUsed);
static void generateTrajectoryOrient(TrajectoryOrient* pTrajW, arm_matrix_instance_f32* pMatVLimit, uint8_t vLimUsed,
		float accW, float brkW, float w0, float s);
//static void printTrajW(TrajectoryOrient* pTrajW);

static float vLimit[4*10];
static TrajectoryOrient trajWLong;
static float brkLimit[4*10];
static MUTEX_DECL(createMtx);

void TrajectoryOrientCreate(TrajectoryOrient* pTrajW, float s0, float s1, float w0, float accW, float brkW,
		float wMax, const Trajectory2D* pTrajXY, const float xyMax)
{
	uint8_t vLimUsed;
	arm_matrix_instance_f32 matVLimit = (arm_matrix_instance_f32){10, 4, vLimit};

	chMtxLock(&createMtx);

	memset(vLimit, 0, sizeof(vLimit));
	memset(brkLimit, 0, sizeof(brkLimit));
	memset(pTrajW, 0, sizeof(TrajectoryOrient));

	getOrientLimit(pTrajXY, accW, brkW, wMax, xyMax, &matVLimit, &vLimUsed);

//	ConsolePrint("vLimUsed: %hu\r\n", (uint16_t)vLimUsed);

	generateTrajectoryOrient(pTrajW, &matVLimit, vLimUsed, accW, brkW, w0, s1-s0);

	calcOrient(pTrajW, s0);

	chMtxUnlock(&createMtx);

//	chThdSleepMilliseconds(20);
//	ConsolePrint("trajUsed: %hu\r\n", (uint16_t)pTrajW->numParts);
//	printTrajW(pTrajW);
}

void TrajectoryOrientPrint(TrajectoryOrient* pTrajW)
{
	ConsolePrint("s0\tv0\tacc\ttEnd\r\n");

	for(uint8_t i = 0; i < pTrajW->numParts; i++)
	{
		ConsolePrint("%.6f %.6f %.6f %.6f\r\n", pTrajW->parts[i].s0, pTrajW->parts[i].v0, pTrajW->parts[i].acc, pTrajW->parts[i].tEnd);
		chThdSleepMilliseconds(5);
	}
}

//static void printAll(TrajectoryOrient* pTrajW)
//{
//	ConsolePrint("s0\tv0\tacc\ttEnd\r\n");
//
//	for(uint8_t i = 0; i < TRAJECTORY_ORIENT_PARTS; i++)
//	{
//		ConsolePrint("%.6f %.6f %.6f %.6f\r\n", pTrajW->parts[i].s0, pTrajW->parts[i].v0, pTrajW->parts[i].acc, pTrajW->parts[i].tEnd);
//		chThdSleepMilliseconds(5);
//	}
//}

// Note: not re-entrant due to static variables
void TrajectoryOrientCreateAngles(TrajectoryOrient* pTrajW, float s0, float s1, float w0, float accW, float brkW,
		float wMax, const Trajectory2D* pTrajXY, const float xyMax)
{
	uint8_t vLimUsed;
	arm_matrix_instance_f32 matVLimit = (arm_matrix_instance_f32){10, 4, vLimit};

	chMtxLock(&createMtx);

	memset(vLimit, 0, sizeof(vLimit));
	memset(brkLimit, 0, sizeof(brkLimit));
	memset(&trajWLong, 0, sizeof(TrajectoryOrient));
	memset(pTrajW, 0, sizeof(TrajectoryOrient));

	pTrajW->input.s0 = s0;
	pTrajW->input.s1 = s1;
	pTrajW->input.w0 = w0;
	pTrajW->input.accW = accW;
	pTrajW->input.brkW = brkW;
	pTrajW->input.wMax = wMax;
	pTrajW->input.xyMax = xyMax;

	getOrientLimit(pTrajXY, accW, brkW, wMax, xyMax, &matVLimit, &vLimUsed);
//	ConsolePrint("vLimUsed: %hu\r\n", (uint16_t)vLimUsed);

	float sDiffShort = s1-s0;
	AngleNormalizePtr(&sDiffShort);
	float sDiffLong;
	if(sDiffShort < 0)
		sDiffLong = sDiffShort+2*M_PI;
	else
		sDiffLong = sDiffShort-2*M_PI;

	generateTrajectoryOrient(pTrajW, &matVLimit, vLimUsed, accW, brkW, w0, sDiffShort);
	generateTrajectoryOrient(&trajWLong, &matVLimit, vLimUsed, accW, brkW, w0, sDiffLong);

	if(TrajectoryOrientGetTotalTime(&trajWLong) < TrajectoryOrientGetTotalTime(pTrajW))
		memcpy(pTrajW, &trajWLong, sizeof(TrajectoryOrient));

	calcOrient(pTrajW, s0);

	chMtxUnlock(&createMtx);

//	chThdSleepMilliseconds(20);
//	ConsolePrint("trajUsed: %hu\r\n", (uint16_t)pTrajW->numParts);
//	printTrajW(pTrajW);
}

void TrajectoryOrientValuesAtEnd(TrajectoryOrient* pTraj, float* pAcc, float* pVel, float* pPos)
{
	TrajectoryOrientValuesAtTime(pTraj, pTraj->parts[pTraj->numParts-1].tEnd+0.1f, pAcc, pVel, pPos);
}

void TrajectoryOrientValuesAtTime(TrajectoryOrient* pTraj, float t, float* pAcc, float* pVel, float* pPos)
{
	TrajectoryPartValues(pTraj->parts, pTraj->numParts, t, pAcc, pVel, pPos);
	AngleNormalizePtr(pPos);
}

float TrajectoryOrientGetTotalTime(TrajectoryOrient* pTraj)
{
	return pTraj->parts[pTraj->numParts-1].tEnd;
}

static void getOrientLimit(const Trajectory2D* pTrajXY, float accW, float brkW, float wMax,
		float xyMax, arm_matrix_instance_f32* pMatLimit, uint8_t* pLimUsed)
{
	arm_matrix_instance_f32 matBrkLimit = (arm_matrix_instance_f32){10, 4, brkLimit};

	arm_mat_zero_f32(pMatLimit);
	arm_mat_zero_f32(&matBrkLimit);

	xyMax += 0.1f;

	float xyToW = wMax/xyMax;

	const TrajectoryPart* pX = pTrajXY->x;
	const TrajectoryPart* pY = pTrajXY->y;

	// first element is reserved in case the first velocity is above maximum

	// process first element
	float vBegin = sqrtf(pTrajXY->x[0].v0*pTrajXY->x[0].v0 + pTrajXY->y[0].v0*pTrajXY->y[0].v0);
	float wBegin = (xyMax-vBegin)*xyToW;
	if(wBegin < 1e-3f)
		wBegin = 0;
	MAT_ELEMENT(*pMatLimit, 1, 1) = wBegin;

	// calculate limits from 2D trajectory
	uint8_t vLimUsed = 2;
	uint8_t x = 0;
	uint8_t y = 0;
	float t1 = -1;
	for(uint8_t i = 2; i < 10; i++)
	{
		float tNow;
		float vx;
		float vy;

		if(x == 4)
		{
			tNow = pY[y].tEnd;
			++y;
		}
		else if(y == 4)
		{
			tNow = pX[x].tEnd;
			++x;
		}
		else
		{
			if(pX[x].tEnd < pY[y].tEnd)
			{
				tNow = pX[x].tEnd;
				++x;
			}
			else
			{
				tNow = pY[y].tEnd;
				++y;
			}
		}

//		ConsolePrint("x: %hu, y: %hu, t1: %f, t2: %f\r\n", (uint16_t)x, (uint16_t)y, t1, tNow);

		if(tNow-t1 > COMBINE_THRESHOLD)
		{
			t1 = tNow;
		}
		else
		{
			if ((tNow < COMBINE_THRESHOLD) && (Trajectory2DGetTotalTime(pTrajXY) > COMBINE_THRESHOLD))
				continue;

			--vLimUsed;
		}

		TrajectoryPartValues(pTrajXY->x, TRAJECTORY_PARTS, tNow, 0, &vx, 0);
		TrajectoryPartValues(pTrajXY->y, TRAJECTORY_PARTS, tNow, 0, &vy, 0);

		float wNow = (xyMax-sqrtf(vx*vx + vy*vy))*xyToW;
		if(wNow < 1e-3f)
			wNow = 0;
		float wLast = MAT_ELEMENT(*pMatLimit, vLimUsed-1, 1);
		float tLast = MAT_ELEMENT(*pMatLimit, vLimUsed-2, 0);
		float aNow = (wNow-wLast)/(tNow-tLast);
		if(tNow-tLast == 0.0f)
			aNow = 0;

		if (fabsf(aNow) < 1e-3f)
		{
			wNow = wLast;
			aNow = 0;
		}

		float vt0 = wLast-aNow*tLast;

		MAT_ELEMENT(*pMatLimit, vLimUsed-1, 0) = tNow;
		MAT_ELEMENT(*pMatLimit, vLimUsed, 1) = wNow;
		MAT_ELEMENT(*pMatLimit, vLimUsed-1, 2) = aNow;
		MAT_ELEMENT(*pMatLimit, vLimUsed-1, 3) = vt0;

		++vLimUsed;
	}

	// check if xy limit is exceeded, prepend a zero element in vLimit
	if(MAT_ELEMENT(*pMatLimit, 1, 1) < 0)
	{
		float tZero = -MAT_ELEMENT(*pMatLimit, 1, 3)/MAT_ELEMENT(*pMatLimit, 1, 2);

		MAT_ELEMENT(*pMatLimit, 0, 0) = tZero;
		MAT_ELEMENT(*pMatLimit, 1, 1) = 0;
	}

	MAT_ELEMENT(*pMatLimit, vLimUsed-1, 0) = MAT_ELEMENT(*pMatLimit, vLimUsed-2, 0)+1e6f;
	MAT_ELEMENT(*pMatLimit, vLimUsed-1, 1) = wMax;
	MAT_ELEMENT(*pMatLimit, vLimUsed-1, 3) = wMax;

//	arm_mat_print(pMatLimit);
//	ConsolePrint("\r\n");

	// vLimUsed-1 is the last element (the extended wMax one)

	MAT_ELEMENT(matBrkLimit, TRAJECTORY_ORIENT_PARTS-1, 0) = MAT_ELEMENT(*pMatLimit, vLimUsed-1, 0);
	MAT_ELEMENT(matBrkLimit, TRAJECTORY_ORIENT_PARTS-1, 1) = MAT_ELEMENT(*pMatLimit, vLimUsed-1, 1);
	MAT_ELEMENT(matBrkLimit, TRAJECTORY_ORIENT_PARTS-1, 2) = MAT_ELEMENT(*pMatLimit, vLimUsed-1, 2);
	MAT_ELEMENT(matBrkLimit, TRAJECTORY_ORIENT_PARTS-1, 3) = MAT_ELEMENT(*pMatLimit, vLimUsed-1, 3);

	uint8_t brkLimFree = TRAJECTORY_ORIENT_PARTS-2;

	// we start with vLimUsed-2, it is the first one that can violate brkW
	for(int8_t i = vLimUsed-2; i >= 0; i--)
	{
		if(MAT_ELEMENT(matBrkLimit, brkLimFree, 0) == 0)
		{
			MAT_ELEMENT(matBrkLimit, brkLimFree, 0) = MAT_ELEMENT(*pMatLimit, i, 0);
		}

		float a1 = MAT_ELEMENT(*pMatLimit, i, 2);
		if(a1 >= -brkW)
		{
			MAT_ELEMENT(matBrkLimit, brkLimFree, 1) = MAT_ELEMENT(*pMatLimit, i, 1);
			MAT_ELEMENT(matBrkLimit, brkLimFree, 2) = MAT_ELEMENT(*pMatLimit, i, 2);
			MAT_ELEMENT(matBrkLimit, brkLimFree, 3) = MAT_ELEMENT(*pMatLimit, i, 3);
			--brkLimFree;
			continue;
		}

		float v1 = MAT_ELEMENT(*pMatLimit, i+1, 1);
		float t1 = MAT_ELEMENT(*pMatLimit, i, 0);
		float vz1 = v1+brkW*t1;

//		ConsolePrint("i: %hu, v1: %f, t1: %f, vz1: %f\r\n", (uint16_t)i, v1, t1, vz1);

		float vInter = 0;
		float tInter = 0;
		uint8_t j;
		for(j = i-1; j > 0; j--)
		{
			float a0 = MAT_ELEMENT(*pMatLimit, j, 2);
			float vz0 = MAT_ELEMENT(*pMatLimit, j, 3);

			tInter = -(vz0-vz1)/(a0+brkW);

//			ConsolePrint("j: %hu, tInter: %f, a0: %f, vz0: %f\r\n", (uint16_t)j, tInter, a0, vz0);

			if(tInter >= MAT_ELEMENT(*pMatLimit, j-1, 0) && tInter <= MAT_ELEMENT(*pMatLimit, j, 0))
			{
				// intersection found
				vInter = a0*tInter+vz0;
//				ConsolePrint("j: %hu, tInter: %f, vInter: %f\r\n", (uint16_t)j, tInter, vInter);
				break;
			}
		}

		if(j > 0)
		{
			MAT_ELEMENT(matBrkLimit, brkLimFree-1, 0) = tInter;
			MAT_ELEMENT(matBrkLimit, brkLimFree, 1) = vInter;
			MAT_ELEMENT(matBrkLimit, brkLimFree, 2) = -brkW;
			MAT_ELEMENT(matBrkLimit, brkLimFree, 3) = vz1;
			--brkLimFree;
			i = j+1;
		}
		else
		{
			// no intersecting element, use t=0
			MAT_ELEMENT(matBrkLimit, brkLimFree-1, 0) = 0;
			MAT_ELEMENT(matBrkLimit, brkLimFree, 1) = vz1;
			MAT_ELEMENT(matBrkLimit, brkLimFree, 2) = -brkW;
			MAT_ELEMENT(matBrkLimit, brkLimFree, 3) = vz1;
			brkLimFree -= 2;
			break;
		}
	}

//	arm_mat_print(&matBrkLimit);
//	ConsolePrint("\r\n");

//	uint8_t brkLimUsed = 10-(brkLimFree+1);
//	ConsolePrint("brkLimUsed: %hu\r\n", (uint16_t)brkLimUsed);

	arm_mat_zero_f32(pMatLimit);
	vLimUsed = 0;
	for(uint8_t i = brkLimFree+1; i < TRAJECTORY_ORIENT_PARTS; i++)
//	for(uint8_t i = 0; i < brkLimUsed; i++)
	{
		if(MAT_ELEMENT(*pMatLimit, vLimUsed, 1) == 0)
		{
			MAT_ELEMENT(*pMatLimit, vLimUsed, 1) = MAT_ELEMENT(matBrkLimit, i, 1);
		}

		float a1 = MAT_ELEMENT(matBrkLimit, i, 2);
		if(a1 <= accW)
		{
			MAT_ELEMENT(*pMatLimit, vLimUsed, 0) = MAT_ELEMENT(matBrkLimit, i, 0);
			MAT_ELEMENT(*pMatLimit, vLimUsed, 2) = MAT_ELEMENT(matBrkLimit, i, 2);
			MAT_ELEMENT(*pMatLimit, vLimUsed, 3) = MAT_ELEMENT(matBrkLimit, i, 3);
			++vLimUsed;
			continue;
		}

		float t1 = 0;
		if(i > 0)
			t1 = MAT_ELEMENT(matBrkLimit, i-1, 0);

		float v1 = MAT_ELEMENT(matBrkLimit, i, 1);
		float vz1 = v1-accW*t1;

//		ConsolePrint("i: %hu, v1: %f, t1: %f, vz1: %f\r\n", (uint16_t)i, v1, t1, vz1);

		float vInter = 0.0f;
		float tInter = 0.0f;
		uint8_t j;
		for(j = i+1; j < TRAJECTORY_ORIENT_PARTS; j++)
		{
			float a0 = MAT_ELEMENT(matBrkLimit, j, 2);
			float vz0 = MAT_ELEMENT(matBrkLimit, j, 3);

			tInter = -(vz0-vz1)/(a0-accW);

//			ConsolePrint("j: %hu, tInter: %f, a0: %f, vz0: %f\r\n", (uint16_t)j, tInter, a0, vz0);

			if(tInter >= (MAT_ELEMENT(matBrkLimit, j-1, 0) - 1e-4f) && tInter <= MAT_ELEMENT(matBrkLimit, j, 0))
			{
				// intersection found
				vInter = a0*tInter+vz0;
//				ConsolePrint("j: %hu, tInter: %f, vInter: %f\r\n", (uint16_t)j, tInter, vInter);
				break;
			}
		}

		if(j == TRAJECTORY_ORIENT_PARTS)
		{
			MAT_ELEMENT(*pMatLimit, vLimUsed, 0) = MAT_ELEMENT(matBrkLimit, i, 0);
			MAT_ELEMENT(*pMatLimit, vLimUsed, 2) = MAT_ELEMENT(matBrkLimit, i, 2);
			MAT_ELEMENT(*pMatLimit, vLimUsed, 3) = MAT_ELEMENT(matBrkLimit, i, 3);
		}
		else
		{
			// intersection found
			MAT_ELEMENT(*pMatLimit, vLimUsed, 2) = accW;
			MAT_ELEMENT(*pMatLimit, vLimUsed, 3) = vz1;
			MAT_ELEMENT(*pMatLimit, vLimUsed, 0) = tInter;
			MAT_ELEMENT(*pMatLimit, vLimUsed+1, 1) = vInter;
			i = j-1;
		}

		++vLimUsed;
	}

	*pLimUsed = vLimUsed;

//	arm_mat_print(pMatLimit);
//	ConsolePrint("\r\n");
}

//static void printTrajW(TrajectoryOrient* pTrajW)
//{
//	ConsolePrint("a       v0      s0      tEnd\r\n");
//	for(uint8_t i = 0; i < 10; i++)
//	{
//		ConsolePrint("% 7.5f % 7.5f % 7.5f % 7.5f\r\n", pTrajW->parts[i].acc, pTrajW->parts[i].v0, pTrajW->parts[i].s0, pTrajW->parts[i].tEnd);
//	}
//}

static void generateTrajectoryOrient(TrajectoryOrient* pTrajW, arm_matrix_instance_f32* pMatVLimit, uint8_t vLimUsed,
		float accW, float brkW, float w0, float s)
{
	float t = 0;

	uint8_t trajUsed = 0;
	pTrajW->parts[0].v0 = w0;

	if(fabsf(s) < 1e-4f)
		return;

	float sDriven = 0;
	float sDrivenLast = 0;

	float tBrk = w0/brkW;
	float sBrk = 0.5f*w0*tBrk;

	uint8_t intersectionFound = 0;
	if(s > 0 && w0 < 0)
	{
//		ConsolePrint("A\r\n");
		s += sBrk;
		w0 = 0;
		t = -tBrk;
		pTrajW->parts[0].acc = brkW;
		pTrajW->parts[0].tEnd = t;
		pTrajW->parts[1].v0 = 0;
		++trajUsed;
	}
	else if(s < 0 && w0 > 0)
	{
//		ConsolePrint("B\r\n");
		s -= sBrk;
		w0 = 0;
		t = tBrk;
		pTrajW->parts[0].acc = -brkW;
		pTrajW->parts[0].tEnd = t;
		pTrajW->parts[1].v0 = 0;
		++trajUsed;
	}
	else if(sBrk >= fabsf(s))
	{
		if(w0 > 0)
		{
//			ConsolePrint("C1\r\n");
			pTrajW->parts[0].acc = -brkW;
			s -= sBrk;
		}
		else
		{
//			ConsolePrint("C2\r\n");
			pTrajW->parts[0].acc = brkW;
			s += sBrk;
		}
		w0 = 0;
		t = fabsf(tBrk);
		pTrajW->parts[0].tEnd = t;
		pTrajW->parts[1].v0 = 0;
		++trajUsed;
	}
	else
	{
		if(w0 > 0)
		{
//			ConsolePrint("D1\r\n");
			pTrajW->parts[0].acc = -brkW;
		}
		else
		{
//			ConsolePrint("D2\r\n");
			pTrajW->parts[0].acc = brkW;
		}

		// check for backward intersection: t=0, v0=w0, a0=+-brk
		float a0 = pTrajW->parts[0].acc;
		float v0 = w0;
		float vInter;
		float tInter;
		int8_t i;
		for(i = vLimUsed-1; i >= 1; i--)
		{
			// calculate intersection point
			float a1 = MAT_ELEMENT(*pMatVLimit, i, 2);
			float vz1 = MAT_ELEMENT(*pMatVLimit, i, 3);

			if(a0 > 0)
			{
				a1 *= -1;
				vz1 *= -1;
			}

			if (a0 == a1)
			{
				// there is no intersection points if the lines have the same slope
				continue;
			}

			tInter = -(v0-vz1)/(a0-a1);

			float tLeft = 0;
			if(i > 0)
				tLeft = MAT_ELEMENT(*pMatVLimit, i-1, 0);

			if (i == 1 && v0 == vz1)
			{
				vInter = v0;
				intersectionFound = 1;
				break;
			}

			if(tInter >= (tLeft-2e-4f) && tInter <= MAT_ELEMENT(*pMatVLimit, i, 0) && tInter > 0)
			{
				vInter = a0*tInter+v0;
				intersectionFound = 1;
				break;
			}
		}

		if(intersectionFound)
		{
			// intersection found
//			ConsolePrint("E\r\n");
			float sTmp = 0.5f*(w0+vInter)*tInter;
			sDriven += sTmp;
			w0 = vInter;
			t = tInter;
			pTrajW->parts[0].tEnd = t;
			pTrajW->parts[1].v0 = w0;
			++trajUsed;
		}
	}

//	chThdSleepMilliseconds(20);
//	ConsolePrint("1:\r\n");
//	printAll(pTrajW);

	if(!intersectionFound)
	{
		if(s-sDriven > 0)
		{
//			ConsolePrint("F1\r\n");
			pTrajW->parts[trajUsed].acc = accW;
		}
		else
		{
//			ConsolePrint("F2\r\n");
			pTrajW->parts[trajUsed].acc = -accW;
		}

		// check for foward intersection: t=t, v0=w0, a0=+-acc
		float a0 = pTrajW->parts[trajUsed].acc;
		float v0 = w0;
		float tInter = 0.0f;
		float vInter = 0.0f;
		for(uint8_t i = 1; i < vLimUsed; i++)
		{
			float a1 = MAT_ELEMENT(*pMatVLimit, i, 2);
			float vz1 = MAT_ELEMENT(*pMatVLimit, i, 3);

			if(a0 < 0)
			{
				a1 *= -1;
				vz1 *= -1;
			}

			if (a0 == a1)
			{
				continue;
			}

			float vz0 = v0-a0*t;
			tInter = -(vz0-vz1)/(a0-a1);

			float tLeft = 0;
			if(i > 0)
				tLeft = MAT_ELEMENT(*pMatVLimit, i-1, 0);

			if(tInter >= tLeft && tInter <= MAT_ELEMENT(*pMatVLimit, i, 0)+1e-4f && tInter > t-1e-4f)
			{
				if(tInter < t)
					tInter = t;
				vInter = a0*tInter+vz0;
				break;
			}
		}

		float tLeft = 0;
		if(trajUsed > 0)
			tLeft = pTrajW->parts[trajUsed-1].tEnd;

		float sTmp = 0.5f*(w0+vInter)*(tInter-tLeft);
		sDrivenLast = sDriven;
		sDriven += sTmp;
		w0 = vInter;
		t = tInter;
		pTrajW->parts[trajUsed].tEnd = t;
		pTrajW->parts[trajUsed+1].v0 = w0;
		++trajUsed;
	}

//	chThdSleepMilliseconds(20);
//	ConsolePrint("2:\r\n");
//	printAll(pTrajW);

	uint8_t i;
	for(i = 0; i < vLimUsed-1; i++)
	{
		float tLeft = 0;
		if(i > 0)
			tLeft = MAT_ELEMENT(*pMatVLimit, i-1, 0);

		if(pTrajW->parts[trajUsed-1].tEnd >= tLeft && pTrajW->parts[trajUsed-1].tEnd <= MAT_ELEMENT(*pMatVLimit, i, 0))
		{
			if(s < 0)
				pTrajW->parts[trajUsed].acc = -MAT_ELEMENT(*pMatVLimit, i, 2);
			else
				pTrajW->parts[trajUsed].acc = MAT_ELEMENT(*pMatVLimit, i, 2);

			break;
		}
	}

//	chThdSleepMilliseconds(20);
//	ConsolePrint("3:\r\n");
//	printAll(pTrajW);

	for(; i <= vLimUsed; i++)
	{
		// central point
		float tc = pTrajW->parts[trajUsed-1].tEnd;
		float vc = pTrajW->parts[trajUsed].v0;

		float tBrk1 = fabsf(vc)/brkW;
		float sBrk1 = 0.5f*vc*tBrk1;

		float sBrkNow = sDriven+sBrk1;

//		ConsolePrint("i: %hu, s: %f, sBrkNow: %f, vc: %f, tc: %f\r\n", (uint16_t)i, s, sBrkNow, vc, tc);

		if((s >= 0 && sBrkNow > s ) || (s <= 0 && sBrkNow < s))
		{
//			chThdSleepMilliseconds(20);
//			ConsolePrint("4:\r\n");
//			printAll(pTrajW);

			float a1 = pTrajW->parts[trajUsed-1].acc;
			float v0 = pTrajW->parts[trajUsed-1].v0;
			float t0 = 0;
			if(trajUsed > 1)
				t0 = pTrajW->parts[trajUsed-2].tEnd;

			float a2;
			if(s < 0)
				a2 = brkW;
			else
				a2 = -brkW;

			float d = s-sDrivenLast;
			float t1;

			if(fabsf(a1) < 5e-4f)
			{
				if(fabsf(v0) < 1e-3f)
					t1 = 0;
				else
					t1 = (v0*v0+2*a2*d)/(2*a2*v0);
//				ConsolePrint("B - a1: %f, a2: %f, v0: %f, d: %f, t1: %f\r\n", a1, a2, v0, d, t1);
//				chThdSleepMilliseconds(20);
			}
			else
			{
				float C = a2-a1;
				float sqr = a2*C*(v0*v0+2*a1*d);
				if(sqr > 0)
					t1 = -(sqrtf(sqr)+C*v0)/(a1*C);
				else
					t1 = v0/a1;

				t1 = fabsf(t1);
//				ConsolePrint("A - a1: %f, a2: %f, v0: %f, d: %f, t1: %f\r\n", a1, a2, v0, d, t1);
			}

			float v1 = t1*a1+v0;
			float s1 = (v0+v1)*0.5f*t1;
			sDrivenLast = sDriven;
			sDriven += s1;

			pTrajW->parts[trajUsed-1].tEnd = t0+t1;
			pTrajW->parts[trajUsed].v0 = v1;
			pTrajW->parts[trajUsed].acc = a2;

			float t2 = -v1/a2;

			pTrajW->parts[trajUsed].tEnd = t0+t1+t2;

			++trajUsed;

//			chThdSleepMilliseconds(20);
//			ConsolePrint("5: %hu, %hu\r\n", (uint16_t)i, (uint16_t)trajUsed);
//			printAll(pTrajW);

			break;
		}
		else
		{
			// right point
			float tr = MAT_ELEMENT(*pMatVLimit, i, 0);
			float vr;
			float ar;
			if(i < vLimUsed-1)
			{
				vr = MAT_ELEMENT(*pMatVLimit, i+1, 1);
				ar = MAT_ELEMENT(*pMatVLimit, i+1, 2);
			}
			else
			{
				vr = MAT_ELEMENT(*pMatVLimit, i, 1);
				ar = 0;
			}

			if(s < 0)
			{
				vr *= -1;
				ar *= -1;
			}

			float t2 = tr-tc;
			float s2 = 0.5f*(vr+vc)*t2;
			sDrivenLast = sDriven;
			sDriven += s2;

			pTrajW->parts[trajUsed].tEnd = tr;
			if(trajUsed < TRAJECTORY_ORIENT_PARTS-1)
			{
				pTrajW->parts[trajUsed+1].v0 = vr;
				pTrajW->parts[trajUsed+1].acc = ar;
			}
			else
			{
				break;
			}

			++trajUsed;
		}
	}

	pTrajW->numParts = trajUsed;
}

static void calcOrient(TrajectoryOrient* pTrajW, float s0)
{
	TrajectoryPart* pParts = pTrajW->parts;
	TrajectoryPart* pFirst = pParts;

	pFirst->s0 = s0;

	for(uint8_t i = 1; i < pTrajW->numParts; i++)
	{
		TrajectoryPart* pCur = &pParts[i];
		TrajectoryPart* pPrev = &pParts[i-1];

		float tStart = 0;
		if(i > 1)
			tStart = pParts[i-2].tEnd;

		float dT = pPrev->tEnd - tStart;

		pCur->s0 = pPrev->s0 + pPrev->v0*dT + 0.5f*pPrev->acc*dT*dT;
	}
}
