/*
 * ema_filter.c
 *
 *  Created on: 18.03.2015
 *      Author: AndreR
 */

#include "ema_filter.h"

void EMAFilterInit(EMAFilter* pEMA, const float alpha)
{
	pEMA->value = 0;
	pEMA->alpha = alpha;
}

float EMAFilterUpdate(EMAFilter* pEMA, const float sample)
{
	pEMA->value = pEMA->alpha*pEMA->value + (1.0f-pEMA->alpha)*sample;
	return pEMA->value;
}
