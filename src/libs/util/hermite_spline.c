/*
 * hermite_spline.c
 *
 *  Created on: 27.03.2013
 *      Author: AndreR
 */

#include "hermite_spline.h"

#include "errors.h"
#include "arm_math.h"

#include <string.h>

void HermiteSplineInit(HermiteSpline* pSpline, float a[], float tEnd)
{
	for(uint8_t i=0;i<SPLINE_SIZE;i++)
		pSpline->a[i] = a[i];
	pSpline->tEnd = tEnd;
}

void HermiteSplineCreate(HermiteSpline* pSpline, float p0, float p1, float m0, float m1, float tEnd)
{
	float t2 = tEnd * tEnd;
	float t3 = t2 * tEnd;
	pSpline->a[5] = 0;
	pSpline->a[4] = 0;
	pSpline->a[3] = (((2 * p0) / t3) - ((2 * p1) / t3)) + (m0 / t2) + (m1 / t2);
	pSpline->a[2] = ((3 * p1) / t2) - ((3 * p0) / t2) - ((2 * m0) / tEnd) - (m1 / tEnd);
	pSpline->a[1] = m0;
	pSpline->a[0] = p0;
	pSpline->tEnd = tEnd;
}

float HermiteSplineValue(HermiteSpline* pSpline, float t)
{
	if (t < 0)
		t = 0.0f;
	if (t > pSpline->tEnd)
		t = pSpline->tEnd;

	return (pSpline->a[5] * t * t * t * t * t) + (pSpline->a[4] * t * t * t * t) +
			(pSpline->a[3] * t * t * t) + (pSpline->a[2] * t * t) + (pSpline->a[1] * t) + pSpline->a[0];
}

float HermiteSplineFirstDerivative(HermiteSpline* pSpline, float t)
{
	if (t < 0)
		t = 0.0f;
	if (t > pSpline->tEnd)
		t = pSpline->tEnd;

	return ((pSpline->a[5] * 5.0f * t * t * t * t) + (pSpline->a[4] * 4.0f * t * t * t) +
			(pSpline->a[3] * 3.0f * t * t) + (pSpline->a[2] * 2.0f * t) + pSpline->a[1]);
}

float HermiteSplineSecondDerivative(HermiteSpline* pSpline, float t)
{
	if (t < 0)
		t = 0.0f;
	if (t > pSpline->tEnd)
		t = pSpline->tEnd;

	return (pSpline->a[5] * 20 * t * t * t) + (pSpline->a[4] * 12 * t * t) + (pSpline->a[3] * 6 * t) + (pSpline->a[2] * 2);
}

float HermiteSplineMaxFirstDerivative(HermiteSpline* pSpline)
{
	float p = HermiteSplineFirstDerivative(pSpline, 0);
	float q = 0.0f;
	float r = HermiteSplineFirstDerivative(pSpline, pSpline->tEnd);

	if (pSpline->a[3] != 0.0f) // maximum existent?
	{
		float t = -(pSpline->a[2] * 2) / (pSpline->a[3] * 6); // time at maximum
		if ((t > 0) && (t < pSpline->tEnd))
		{
			q = HermiteSplineFirstDerivative(pSpline, t);
		}
	}

	p = fabsf(p);
	q = fabsf(q);
	r = fabsf(r);

	if(q > p && q > r)
		return q;
	if(p > q && p > r)
		return p;
	return r;
}


/**
 * Calculate maximum of second derivative.
 * Always positive.
 *
 * @return max
 */
float HermiteSplineMaxSecondDerivative(HermiteSpline* pSpline)
{
	// maximum of second derivative is at begin or end
	float q = HermiteSplineSecondDerivative(pSpline, 0);
	float p = HermiteSplineSecondDerivative(pSpline, pSpline->tEnd);

	q = fabsf(q);
	p = fabsf(p);

	if(p > q)
		return p;
	return q;
}


/**
 * Calculate maximum value of first derivative.
 * Always positive.
 *
 * @return max
 */
float HermiteSpline2DMaxFirstDerivative(HermiteSpline* pA, HermiteSpline* pB)
{
	Vector2f t0 = Vector2fFromXY(HermiteSplineFirstDerivative(pA, 0), HermiteSplineFirstDerivative(pB, 0));
	Vector2f tEnd = Vector2fFromXY(HermiteSplineFirstDerivative(pA, pA->tEnd), HermiteSplineFirstDerivative(pB, pB->tEnd));

	float a = Vector2fGetLength(t0);
	float b = 0.0f;
	float c = Vector2fGetLength(tEnd);

	if ((pA->a[3] + pB->a[3]) != 0.0f) // maximum existent?
	{
		float t = -(pB->a[2] + pA->a[2]) / ((3 * pB->a[3]) + (3 * pA->a[3])); // time at maximum
		if ((t > 0) && (t < pA->tEnd))
		{
			Vector2f B = Vector2fFromXY(HermiteSplineFirstDerivative(pA, t), HermiteSplineFirstDerivative(pB, t));
			b = Vector2fGetLength(B);
		}
	}

	if(a > b && a > c)
		return a;
	if(b > a && b > c)
		return b;

	return c;
}


/**
 * Calculate maximum of second derivative.
 * Always positive.
 *
 * @return max
 */
float HermiteSpline2DMaxSecondDerivative(HermiteSpline* pA, HermiteSpline* pB)
{
	// maximum of second derivative is at begin or end
	Vector2f t0 = Vector2fFromXY(HermiteSplineSecondDerivative(pA, 0), HermiteSplineSecondDerivative(pB, 0));
	Vector2f tEnd = Vector2fFromXY(HermiteSplineSecondDerivative(pA, pA->tEnd), HermiteSplineSecondDerivative(pB, pB->tEnd));

	float a = Vector2fGetLength(t0);
	float b = Vector2fGetLength(tEnd);

	if(a > b)
		return a;

	return b;
}

void HermiteSpline2DGenerate(Vector2f initialPos, Vector2f finalPos, Vector2f initialVelocity,
		Vector2f finalVelocity, float maxVelocity, float maxAcc, HermiteSpline* pOutX, HermiteSpline* pOutY)
{
	// catch some invalid velocities
	if (Vector2fGetLength(initialVelocity) > maxVelocity)
	{
		maxVelocity = Vector2fGetLength(initialVelocity);
	}

	if (Vector2fGetLength(finalVelocity) > maxVelocity)
	{
		maxVelocity = Vector2fGetLength(finalVelocity);
	}

//	maxVelocity += 0.1f;
//	maxAcc +=  0.1f;

	// generate initial guess based on distance and max velocity
	float d = Vector2fGetLength(Vector2fSubtract(finalPos, initialPos));
	// t will always be a too short time, that's good
	float t = d / maxVelocity;
	// and this will avoid nulls and NaNs in the following calculation
	t += 0.01f;

	HermiteSplineCreate(pOutX, initialPos.x, finalPos.x, initialVelocity.x, finalVelocity.x, t);
	HermiteSplineCreate(pOutY, initialPos.y, finalPos.y, initialVelocity.y, finalVelocity.y, t);

	float velDiff = maxVelocity - HermiteSpline2DMaxFirstDerivative(pOutX, pOutY);

	// optimize for velocity first
	while (velDiff < -0.01f)
	{
		t += 0.1f;

		HermiteSplineCreate(pOutX, initialPos.x, finalPos.x, initialVelocity.x, finalVelocity.x, t);
		HermiteSplineCreate(pOutY, initialPos.y, finalPos.y, initialVelocity.y, finalVelocity.y, t);

		velDiff = maxVelocity - HermiteSpline2DMaxFirstDerivative(pOutX, pOutY);
	}

	// now optimize for acceleration
	float accDiff = maxAcc - HermiteSpline2DMaxSecondDerivative(pOutX, pOutY);

	while (accDiff < -0.01f)
	{
		t += 0.1f;

		HermiteSplineCreate(pOutX, initialPos.x, finalPos.x, initialVelocity.x, finalVelocity.x, t);
		HermiteSplineCreate(pOutY, initialPos.y, finalPos.y, initialVelocity.y, finalVelocity.y, t);

		accDiff = maxAcc - HermiteSpline2DMaxSecondDerivative(pOutX, pOutY);;
	}
}

static float getShortestRotation(float angle1, float angle2)
{
	float rotateDist = 0;

	rotateDist = angle2 - angle1;
	if (rotateDist < -PI)
	{
		rotateDist = 2*PI + rotateDist;
	}
	if (rotateDist > PI)
	{
		rotateDist -= 2*PI;
	}
	return rotateDist;
}

void HermiteSplineGenerate(float initialPos, float finalPos, float initialVelocity,
		float finalVelocity, float t, float maxAngularVelocity, float maxAngularAcceleration,
		uint8_t isAngle, HermiteSpline* pOut)
{
	// catch some invalid velocities
	if (fabsf(initialVelocity) > maxAngularVelocity)
	{
		maxAngularVelocity = fabsf(initialVelocity);
	}

	if (fabsf(finalVelocity) > maxAngularVelocity)
	{
		maxAngularVelocity = fabsf(finalVelocity);
	}

	maxAngularVelocity += 0.1f;
	maxAngularAcceleration += 0.1f;

	if (isAngle)
	{
		finalPos = initialPos + getShortestRotation(initialPos, finalPos);
	}

	HermiteSplineCreate(pOut, initialPos, finalPos, initialVelocity, finalVelocity, t);

	float velDiff = maxAngularVelocity - HermiteSplineMaxFirstDerivative(pOut);

	// optimize for velocity first
	while (velDiff < -0.01f)
	{
		t += 0.1f;

		HermiteSplineCreate(pOut, initialPos, finalPos, initialVelocity, finalVelocity, t);

		velDiff = maxAngularVelocity - HermiteSplineMaxFirstDerivative(pOut);
	}

	// now optimize for acceleration
	float accDiff = maxAngularAcceleration - HermiteSplineMaxSecondDerivative(pOut);

	while (accDiff < -0.01f)
	{
		t += 0.1f;

		HermiteSplineCreate(pOut, initialPos, finalPos, initialVelocity, finalVelocity, t);

		accDiff = maxAngularAcceleration - HermiteSplineMaxSecondDerivative(pOut);
	}
}


// SPLINE LIST
void HermiteSplineListInit(HermiteSplineList* pList, HermiteSplineListItem* pData, uint16_t maxItems)
{
	pList->pItems = pData;
	pList->maxItems = maxItems;
	pList->numItems = 0;
	pList->totalTime = 0.0f;

	memset(pData, 0, sizeof(HermiteSplineListItem)*maxItems);
}

HermiteSplineListItem* HermiteSplineListAllocate(HermiteSplineList* pList)
{
	if(pList->numItems >= pList->maxItems)
		return 0;

	return &pList->pItems[pList->numItems++];
}

void HermiteSplineListUpdate(HermiteSplineList* pList)
{
	pList->totalTime = 0.0f;

	for(uint16_t i = 0; i < pList->numItems; i++)
	{
		pList->pItems[i].tStart = pList->totalTime;
		pList->totalTime += pList->pItems[i].spline.tEnd;
		pList->pItems[i].tEnd = pList->totalTime;
	}
}

HermiteSplineListItem* HermiteSplineListGetItem(HermiteSplineList* pList, float t)
{
	if (t < 0)
		t = 0.0f;
	if(t > pList->totalTime)
		t = pList->totalTime;

	for(uint16_t i = 0; i < pList->numItems; i++)
	{
		if(t >= pList->pItems[i].tStart && t < pList->pItems[i].tEnd)
		{
			return &pList->pItems[i];
		}
	}

	return &pList->pItems[pList->numItems-1];
}

void HermiteSplineListClear(HermiteSplineList* pList)
{
	pList->totalTime = 0.0f;
	pList->numItems = 0;
}
