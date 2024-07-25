/*
 * fixed_point.c
 *
 *  Created on: 11.03.2019
 *      Author: AndreR
 */

#include "fixed_point.h"
#include <math.h>

uint32_t FixedPointUnsignedFromFloat(float input, uint8_t fractionBits)
{
	return (uint32_t)(input * (1 << fractionBits));
}

int32_t FixedPointSignedFromFloat(float input, uint8_t fractionBits)
{
	return (int32_t)(input * (1 << fractionBits));
}

float FixedPointUnsignedToFloat(uint32_t input, uint8_t fractionBits)
{
	return (float)input/(float)(1 << fractionBits);
}

float FixedPointSignedToFloat(int32_t input, uint8_t fractionBits)
{
	return (float)input/(float)(1 << fractionBits);
}

float FixedPointGetEps(uint8_t fractionBits)
{
	return 1.0f/(float)(1 << fractionBits);
}

float FixedPointGetMax(uint8_t integerBits, uint8_t fractionBits)
{
	const float fractionEps = 1.0f/(float)(1 << fractionBits);
	const float integerMax = (float)(1 << integerBits);

	return integerMax - fractionEps;
}

uint8_t FixedPointIsEqual(float a, float b, uint8_t fractionBits)
{
	float eps = FixedPointGetEps(fractionBits);

	return fabsf(a-b) < (eps*0.5f);
}

float FixedPointUnsignedSaturate(float input, uint8_t integerBits, uint8_t fractionBits)
{
	const float representableMax = FixedPointGetMax(integerBits, fractionBits);

	if(input > representableMax)
		input = representableMax;
	else if(input < 0)
		input = 0;

	return input;
}

float FixedPointSignedSaturate(float input, uint8_t integerBits, uint8_t fractionBits)
{
	const float representableMax = FixedPointGetMax(integerBits, fractionBits);

	if(input > representableMax)
		input = representableMax;
	else if(input < -representableMax)
		input = -representableMax;

	return input;
}

uint32_t FixedPointUnsignedFromFloatSaturate(float input, uint8_t integerBits, uint8_t fractionBits)
{
	const float representableMax = FixedPointGetMax(integerBits, fractionBits);

	if(input > representableMax)
		input = representableMax;
	else if(input < 0)
		input = 0;

	return FixedPointUnsignedFromFloat(input, fractionBits);
}

int32_t FixedPointSignedFromFloatSaturate(float input, uint8_t integerBits, uint8_t fractionBits)
{
	const float representableMax = FixedPointGetMax(integerBits, fractionBits);

	if(input > representableMax)
		input = representableMax;
	else if(input < -representableMax)
		input = -representableMax;

	return FixedPointSignedFromFloat(input, fractionBits);
}
