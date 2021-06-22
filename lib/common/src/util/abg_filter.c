/*
 * abg_filter.c
 *
 *  Created on: 05.11.2015
 *      Author: AndreR
 */

#include "abg_filter.h"
#include "arm_math.h"
#include "float.h"

#define ABG_FILTER_CRITICALLY_DAMPED 0
#define ABG_FILTER_BUTTERWORTH 1
#define ABG_FILTER_MINIMUM_IAE 2

void ABGFilterInit(ABGFilter* pFilter, float updateRate, float filterFreq, uint8_t filterType)
{
	float dT = 1.0f/updateRate;
	float omega = 2.0f*PI*filterFreq;

	switch(filterType)
	{
		case ABG_FILTER_BUTTERWORTH:
		{
			pFilter->K[0] = 2.0f*omega*dT;
			pFilter->K[1] = 2.0f*omega*omega*dT;
			pFilter->K[2] = omega*omega*omega*dT;
		}
		break;
		case ABG_FILTER_MINIMUM_IAE:
		{
			pFilter->K[0] = 2.089582f*omega*dT;
			pFilter->K[1] = 1.479222f*omega*omega*dT;
			pFilter->K[2] = omega*omega*omega*dT;
		}
		break;
		default:
		{
			pFilter->K[0] = 3.0f*omega*dT;
			pFilter->K[1] = 3.0f*omega*omega*dT;
			pFilter->K[2] = omega*omega*omega*dT;
		}
		break;
	}

	pFilter->dT = dT;

	pFilter->x[0] = 0;
	pFilter->x[1] = 0;
	pFilter->x[2] = 0;

	pFilter->x0Limit = FLT_MAX;
}

static inline float normalize(float limit, float in)
{
	while(in > limit)
		in -= 2*limit;
	while(in < -limit)
		in += 2*limit;

	return in;
}

void ABGFilterUpdate(ABGFilter* pFilter, float meas)
{
	float dT = pFilter->dT;
//	float x0p = 0.5f*pFilter->x[2]*dT*dT + pFilter->x[1]*dT + pFilter->x[0];
	float x0p = ((0.5f*pFilter->x[2]*dT)*dT + pFilter->x[1])*dT + pFilter->x[0];

	float err = meas - x0p;

	err = normalize(pFilter->x0Limit, err);
	x0p = normalize(pFilter->x0Limit, x0p);

	pFilter->x[0] = x0p + pFilter->K[0]*err;
	pFilter->x[1] += pFilter->x[2]*dT + pFilter->K[1]*err;
	pFilter->x[2] += pFilter->K[2]*err;
}
