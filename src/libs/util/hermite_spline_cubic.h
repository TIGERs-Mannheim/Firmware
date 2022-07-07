/*
 * hermite_spline_cubic.h
 *
 *  Created on: 09.04.2015
 *      Author: AndreR
 */

#ifndef HERMITE_SPLINE_CUBIC_H_
#define HERMITE_SPLINE_CUBIC_H_

#include <stdint.h>
#include "vector.h"

typedef struct _HermiteSplineCubic
{
	float a[4];
} HermiteSplineCubic;

typedef struct _HermiteSplineCubic2D
{
	HermiteSplineCubic x;
	HermiteSplineCubic y;
} HermiteSplineCubic2D;

void	HermiteSplineCubicInit(HermiteSplineCubic* pSpline, float a[]);
void	HermiteSplineCubicCreate(HermiteSplineCubic* pSpline, float p0, float p1, float m0, float m1);
float	HermiteSplineCubicValue(HermiteSplineCubic* pSpline, float t);
float	HermiteSplineCubicFirstDerivative(HermiteSplineCubic* pSpline, float t);
float	HermiteSplineCubicSecondDerivative(HermiteSplineCubic* pSpline, float t);
float	HermiteSplineCubicMaxFirstDerivative(HermiteSplineCubic* pSpline);
float	HermiteSplineCubicMaxSecondDerivative(HermiteSplineCubic* pSpline);

void HermiteSplineCubic2DCreate(HermiteSplineCubic2D* pSpline, float* pStartPos, float* pEndPos, float* pStartVel, float* pEndVel);
void HermiteSplineCubic2DValues(HermiteSplineCubic2D* pSpline, float t, float* pValues);
void HermiteSplineCubic2DFirstDerivative(HermiteSplineCubic2D* pSpline, float t, float* pValues);
void HermiteSplineCubic2DSecondDerivative(HermiteSplineCubic2D* pSpline, float t, float* pValues);
float HermiteSplineCubic2DArcLength(HermiteSplineCubic2D* pSpline, uint32_t steps);
float HermiteSplineCubic2DCurvature(HermiteSplineCubic2D* pSpline, float t, float stepSize);
float HermiteSplineCubic2DAngleChange(HermiteSplineCubic2D* pSpline, float t, float stepSize);
float HermiteSplineCubic2DGetTime(HermiteSplineCubic2D* pSpline, float startTime, float requiredLength, float initialStepSize, float precision);

#endif /* HERMITE_SPLINE_CUBIC_H_ */
