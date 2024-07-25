#include "clamp.h"

#include <math.h>

float Clampf(float in, float limit)
{
	if(in > limit)
		return limit;

	if(in < -limit)
		return -limit;

	return in;
}

void Clamp2Df(float* pIn, float limit)
{
	const float inAbs = sqrtf(pIn[0]*pIn[0] + pIn[1]*pIn[1]);

	if(inAbs > limit)
	{
		const float scale = limit/inAbs;

		pIn[0] *= scale;
		pIn[1] *= scale;
	}
}
