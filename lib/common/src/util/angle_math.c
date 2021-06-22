/*
 * angle_math.c
 *
 *  Created on: 07.06.2013
 *      Author: AndreR
 */

#include "angle_math.h"

#include "arm_math.h"

static float mod(float x, float y)
{
	if(y == 0.0f)
		return x;

	float m = x - y * floorf(x / y);

	// handle boundary cases resulted from floating-point cut off:
	if(y > 0) // modulo range: [0..y)
	{
		if(m >= y)
			return 0;

		if(m < 0)
		{
			if(y + m == y)
				return 0;
			else
				return y + m;
		}
	}
	else // modulo range: (y..0]
	{
		if(m <= y)
			return 0;

		if(m > 0)
		{
			if(y + m == y)
				return 0;
			else
				return y + m;
		}
	}

	return m;
}

// wrap [rad] angle to [-PI..PI)
float AngleNormalize(float a)
{
	return mod(a + M_PI, M_TWOPI) - M_PI;
}

void AngleNormalizePtr(float* a)
{
	*a = mod(*a + M_PI, M_TWOPI) - M_PI;
}
