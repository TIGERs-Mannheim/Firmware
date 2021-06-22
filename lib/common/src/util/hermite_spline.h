/*
 * hermite_spline.h
 *
 *  Created on: 27.03.2013
 *      Author: AndreR
 */

#ifndef HERMITE_SPLINE_H_
#define HERMITE_SPLINE_H_

#include <stdint.h>
#include "vector.h"

#define SPLINE_SIZE 6

typedef struct _HermiteSpline
{
	float a[SPLINE_SIZE];
	float tEnd;
} HermiteSpline;

void	HermiteSplineInit(HermiteSpline* pSpline, float a[], float tEnd);
void	HermiteSplineCreate(HermiteSpline* pSpline, float p0, float p1, float m0, float m1, float tEnd);
float	HermiteSplineValue(HermiteSpline* pSpline, float t);
float	HermiteSplineFirstDerivative(HermiteSpline* pSpline, float t);
float	HermiteSplineSecondDerivative(HermiteSpline* pSpline, float t);
float	HermiteSplineMaxFirstDerivative(HermiteSpline* pSpline);
float	HermiteSplineMaxSecondDerivative(HermiteSpline* pSpline);
float	HermiteSpline2DMaxFirstDerivative(HermiteSpline* pA, HermiteSpline* pB);
float	HermiteSpline2DMaxSecondDerivative(HermiteSpline* pA, HermiteSpline* pB);
void	HermiteSpline2DGenerate(Vector2f initialPos, Vector2f finalPos, Vector2f initialVelocity,
			Vector2f finalVelocity, float maxVelocity, float maxAcc, HermiteSpline* pOutX, HermiteSpline* pOutY);
void	HermiteSplineGenerate(float initialPos, float finalPos, float initialVelocity,
			float finalVelocity, float t, float maxAngularVelocity, float maxAngularAcceleration,
			uint8_t isAngle, HermiteSpline* pOut);

typedef struct _HermiteSplineListItem
{
	HermiteSpline spline;
	float tStart;
	float tEnd;
} HermiteSplineListItem;

typedef struct _HermiteSplineList
{
	HermiteSplineListItem* pItems;
	uint16_t maxItems;
	uint16_t numItems;
	float totalTime;
} HermiteSplineList;

void 					HermiteSplineListInit(HermiteSplineList* pList, HermiteSplineListItem* pData, uint16_t maxItems);
HermiteSplineListItem*	HermiteSplineListAllocate(HermiteSplineList* pList);
void					HermiteSplineListUpdate(HermiteSplineList* pList);
HermiteSplineListItem*	HermiteSplineListGetItem(HermiteSplineList* pList, float t);
void					HermiteSplineListClear(HermiteSplineList* pList);

#endif /* HERMITE_SPLINE_H_ */
