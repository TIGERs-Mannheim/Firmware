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

float ClampMinMaxf(float in, float min, float max)
{
	if(in > max)
		return max;

	if(in < -min)
		return -min;

	return in;
}

int32_t ClampMinMaxToIntf(float in, float min, float max)
{
	return lroundf(ClampMinMaxf(in, min, max));
}

int16_t ClampToInt16f(float in)
{
	int32_t rounded = lroundf(in);
	if(rounded < INT16_MIN)
		return INT16_MIN;

	if(rounded > INT16_MAX)
		return INT16_MAX;

	return rounded;
}

uint8_t ClampToUint8f(float in)
{
	int32_t rounded = lroundf(in);
	if(rounded < 0)
		return 0;

	if(rounded > UINT8_MAX)
		return UINT8_MAX;

	return rounded;
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
