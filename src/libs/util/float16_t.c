/*
 * float16_t.c
 *
 *  Created on: 26.03.2013
 *      Author: AndreR
 */

#include "float16_t.h"

typedef union _Float
{
	float f;
	uint32_t bits;
} Float;

float halfFloatToFloat(float16_t f16)
{
	Float f;
	f.bits = f16;

	uint32_t mant = f.bits & 0x03ff; // 10 bits mantissa
	uint32_t exp = f.bits & 0x7c00; // 5 bits exponent
	if (exp == 0x7c00)
	{
		exp = 0x3fc00; // -> NaN/Inf
	} else if (exp != 0) // normalized value
	{
		exp += 0x1c000; // exp - 15 + 127
		if ((mant == 0) && (exp > 0x1c400))
		{
			f.bits = (((f.bits & 0x8000) << 16) | (exp << 13) | 0x3ff);
			return f.f;
		}
	} else if (mant != 0) // && exp==0 -> subnormal
	{
		exp = 0x1c400; // make it normal
		do
		{
			mant <<= 1; // mantissa * 2
			exp -= 0x400; // decrease exp by 1
		} while ((mant & 0x400) == 0); // while not normal
		mant &= 0x3ff; // discard subnormal bit
	} // else +/-0 -> +/-0

	f.bits = (((f.bits & 0x8000) << 16) | ((exp | mant) << 13));

	return f.f;
}

float16_t floatToHalfFloat(float f32)
{
	Float f;
	f.f = f32;

	uint32_t sign = (f.bits >> 16) & 0x8000; // sign only
	uint32_t val = (f.bits & 0x7fffffff) + 0x1000; // rounded value

	if (val >= 0x47800000) // might be or become NaN/Inf
	{ // avoid Inf due to rounding
		if ((f.bits & 0x7fffffff) >= 0x47800000)
		{ // is or must become NaN/Inf
			if (val < 0x7f800000)
			{
				return sign | 0x7c00; // make it +/-Inf
			}
			return sign | 0x7c00 | ((f.bits & 0x007fffff) >> 13); // keep NaN (and Inf) bits
		}
		return sign | 0x7bff; // unrounded not quite Inf
	}
	if (val >= 0x38800000)
	{
		return sign | ((val - 0x38000000) >> 13); // exp - 127 + 15
	}
	if (val < 0x33000000)
	{
		return sign; // becomes +/-0
	}
	val = (f.bits & 0x7fffffff) >> 23; // tmp exp for subnormal calc
	return sign | ((((f.bits & 0x7fffff) | 0x800000) + (0x800000 >> (val - 102))) >> (126 - val));

}
