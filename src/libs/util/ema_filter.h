/*
 * ema_filter.h
 *
 *  Created on: 18.03.2015
 *      Author: AndreR
 */

#ifndef EMA_FILTER_H_
#define EMA_FILTER_H_

#include <stdint.h>

/**
 * The Exponential Moving Average (EMA) filter is an Infinite Impulse Response (IIR) filter
 * with exponentially decreasing weighting factors for each sample
 *
 * y_k = a*y_(k-1) + (1-a)*sample
 */
typedef struct _EMAFilter
{
	float alpha;
	float value;
} EMAFilter;

void EMAFilterInit(EMAFilter* pEMA, const float alpha);
float EMAFilterUpdate(EMAFilter* pEMA, const float sample);

#endif /* EMA_FILTER_H_ */
