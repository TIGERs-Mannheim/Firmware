/*
 * hermite_spline_cubic.c
 *
 *  Created on: 09.04.2015
 *      Author: AndreR
 */

#include "hermite_spline_cubic.h"

#include "errors.h"
#include "arm_math.h"
#include "angle_math.h"

#include <string.h>

void HermiteSplineCubicInit(HermiteSplineCubic* pSpline, float* pA)
{
	memcpy(pSpline->a, pA, sizeof(float)*4);
}

void HermiteSplineCubicCreate(HermiteSplineCubic* pSpline, float p0, float p1, float m0, float m1)
{
	pSpline->a[3] = 2.0f*p0 - 2.0f*p1 + m0 + m1;
	pSpline->a[2] = 3.0f*p1 - 3.0f*p0 - 2.0f*m0 - m1;
	pSpline->a[1] = m0;
	pSpline->a[0] = p0;
}

float HermiteSplineCubicValue(HermiteSplineCubic* pSpline, float t)
{
	if (t < 0)
		t = 0.0f;
	if (t > 1.0f)
		t = 1.0f;

	return (pSpline->a[3] * t * t * t) + (pSpline->a[2] * t * t) + (pSpline->a[1] * t) + pSpline->a[0];
}

float HermiteSplineCubicFirstDerivative(HermiteSplineCubic* pSpline, float t)
{
	if (t < 0)
		t = 0.0f;
	if (t > 1.0f)
		t = 1.0f;

	return (pSpline->a[3] * 3.0f * t * t) + (pSpline->a[2] * 2.0f * t) + pSpline->a[1];
}

float HermiteSplineCubicSecondDerivative(HermiteSplineCubic* pSpline, float t)
{
	if (t < 0)
		t = 0.0f;
	if (t > 1.0f)
		t = 1.0f;

	return (pSpline->a[3] * 6 * t) + (pSpline->a[2] * 2);
}

float HermiteSplineCubicMaxFirstDerivative(HermiteSplineCubic* pSpline)
{
	float p = HermiteSplineCubicFirstDerivative(pSpline, 0);
	float q = 0.0f;
	float r = HermiteSplineCubicFirstDerivative(pSpline, 1.0f);

	if (pSpline->a[3] != 0.0f) // maximum existent?
	{
		float t = -(pSpline->a[2] * 2) / (pSpline->a[3] * 6); // time at maximum
		if ((t > 0) && (t < 1.0f))
		{
			q = HermiteSplineCubicFirstDerivative(pSpline, t);
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
float HermiteSplineCubicMaxSecondDerivative(HermiteSplineCubic* pSpline)
{
	// maximum of second derivative is at begin or end
	float q = HermiteSplineCubicSecondDerivative(pSpline, 0);
	float p = HermiteSplineCubicSecondDerivative(pSpline, 1.0f);

	q = fabsf(q);
	p = fabsf(p);

	if(p > q)
		return p;
	return q;
}

void HermiteSplineCubic2DCreate(HermiteSplineCubic2D* pSpline, float* pStartPos, float* pEndPos, float* pStartVel, float* pEndVel)
{
	HermiteSplineCubicCreate(&pSpline->x, pStartPos[0], pEndPos[0], pStartVel[0], pEndVel[0]);
	HermiteSplineCubicCreate(&pSpline->y, pStartPos[1], pEndPos[1], pStartVel[1], pEndVel[1]);
}

void HermiteSplineCubic2DValues(HermiteSplineCubic2D* pSpline, float t, float* pValues)
{
	pValues[0] = HermiteSplineCubicValue(&pSpline->x, t);
	pValues[1] = HermiteSplineCubicValue(&pSpline->y, t);
}

void HermiteSplineCubic2DFirstDerivative(HermiteSplineCubic2D* pSpline, float t, float* pValues)
{
	pValues[0] = HermiteSplineCubicFirstDerivative(&pSpline->x, t);
	pValues[1] = HermiteSplineCubicFirstDerivative(&pSpline->y, t);
}

void HermiteSplineCubic2DSecondDerivative(HermiteSplineCubic2D* pSpline, float t, float* pValues)
{
	pValues[0] = HermiteSplineCubicSecondDerivative(&pSpline->x, t);
	pValues[1] = HermiteSplineCubicSecondDerivative(&pSpline->y, t);
}

float HermiteSplineCubic2DArcLength(HermiteSplineCubic2D* pSpline, uint32_t steps)
{
	float pVel[2];
	float stepSize = 1.0f/steps;

	float length = 0;

	for(float t = 0.0f; t < 1.0f; t += stepSize)
	{
		HermiteSplineCubic2DFirstDerivative(pSpline, t, pVel);

		length += sqrtf(pVel[0]*pVel[0] + pVel[1]*pVel[1])*stepSize;
	}

	return length;
}

float HermiteSplineCubic2DCurvature(HermiteSplineCubic2D* pSpline, float t, float stepSize)
{
	float v0[2];
	float v1[2];

	HermiteSplineCubic2DFirstDerivative(pSpline, t-stepSize*0.5f, v0);
	HermiteSplineCubic2DFirstDerivative(pSpline, t+stepSize*0.5f, v1);

	float l0 = sqrtf(v0[0]*v0[0] + v0[1]*v0[1]);
	float l1 = sqrtf(v1[0]*v1[0] + v1[1]*v1[1]);

	return (v0[0]*v1[0]+v0[1]*v1[1])/(l0*l1+0.001f);
}

float HermiteSplineCubic2DAngleChange(HermiteSplineCubic2D* pSpline, float t, float stepSize)
{
	float pVel1[2];
	float pVel2[2];

	HermiteSplineCubic2DFirstDerivative(pSpline, t, pVel1);
	HermiteSplineCubic2DFirstDerivative(pSpline, t+stepSize, pVel2);

	float l1Square = (pVel1[0]*pVel1[0] + pVel1[1]*pVel1[1]);
	float l2Square = (pVel2[0]*pVel2[0] + pVel2[1]*pVel2[1]);

	if(l1Square < 1e-3f || l2Square < 1e-3f )
		return 0.0f;

	float angles[2];
	angles[0] = atan2f(pVel1[1], pVel1[0]);
	angles[1] = atan2f(pVel2[1], pVel2[0]);

	return AngleNormalize(angles[1]-angles[0]);
}

float HermiteSplineCubic2DGetTime(HermiteSplineCubic2D* pSpline, float startTime, float requiredLength, float initialStepSize, float precision)
{
	float stepSize = initialStepSize;
	float length = 0;
	float pVel[2];

	float t = startTime;

	while(t < 1.0f)
	{
		HermiteSplineCubic2DFirstDerivative(pSpline, t, pVel);

		float lengthInThisStep = sqrtf(pVel[0]*pVel[0] + pVel[1]*pVel[1])*stepSize;

		length += lengthInThisStep;

		if(fabsf(length-requiredLength) < precision)
			return t+stepSize;

		if(length > requiredLength)
		{
			length -= lengthInThisStep;
			stepSize *= 0.5f;
			continue;
		}

		t += stepSize;
	}

	return 1.0f;
}
