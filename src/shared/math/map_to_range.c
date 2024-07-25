/*
 * map_to_range.c
 *
 *  Created on: 11.12.2017
 *      Author: AndreR
 */

#include "map_to_range.h"

float MapToRangef32(float inMin, float inMax, float outMin, float outMax, float value)
{
	if(value < inMin)
		return outMin;

	if(value > inMax)
		return outMax;

	float slope = (outMax-outMin)/(inMax-inMin);
	return outMin + slope*(value - inMin);
}
