/*
 * vector.c
 *
 *  Created on: 28.06.2013
 *      Author: AndreR
 */

#include "vector.h"
#include "arm_mat_util_f32.h"

Vector2f Vector2fFromData(const float* pData)
{
	Vector2f vec = {{ pData[0], pData[1] }};
	return vec;
}

Vector2f Vector2fFromXY(float x, float y)
{
	return (Vector2f){{ x, y }};
}

Vector2f Vector2fFromPoints(Vector2f from, Vector2f to)
{
	return Vector2fSubtract(to, from);
}

Vector2f Vector2fFromAngleLength(float rad, float length)
{
	Vector2f out;
	out.x = arm_cos_f32(rad) * length;
	out.y = arm_sin_f32(rad) * length;
	return out;
}

Vector2f Vector2fAdd(Vector2f a, Vector2f b)
{
	return (Vector2f){{ a.x + b.x, a.y + b.y }};
}

Vector2f Vector2fSubtract(Vector2f a, Vector2f b)
{
	return (Vector2f){{ a.x - b.x, a.y - b.y }};
}

Vector2f Vector2fScale(Vector2f a, float factor)
{
	return (Vector2f){{ a.x*factor, a.y*factor }};
}

Vector2f Vector2fScaleTo(Vector2f a, float length)
{
	float oldLength = Vector2fGetLength(a);

	if(oldLength > 0.0f)
		return Vector2fScale(a, length / oldLength);

	return a;
}

Vector2f Vector2fNormalize(Vector2f v)
{
	return Vector2fScaleTo(v, 1.0f);
}

Vector2f Vector2fRotate(Vector2f v, float rad)
{
	float sA = arm_sin_f32(rad);
	float cA = arm_sin_f32(rad);

	return Vector2fFromXY(v.x*cA - v.y*sA, v.y*cA + v.x*sA);
}

Vector2f Vector2fGetNormalVector(Vector2f v)
{
	return (Vector2f){{ -v.y, v.x }};
}

float Vector2fDotProduct(Vector2f a, Vector2f b)
{
	return a.x*b.x + a.y*b.y;
}

float Vector2fGetLength(Vector2f v)
{
	return sqrtf(v.x*v.x + v.y*v.y);
}

float Vector2fGetAngle(Vector2f v)
{
	return arm_atan2_f32(v.y, v.x);
}

float Vector2fLookAtGetAngle(Vector2f from, Vector2f to)
{
	return Vector2fGetAngle(Vector2fFromPoints(from, to));
}

float Vector2fGetAngleBetween(Vector2f a, Vector2f b)
{
	float aNorm = Vector2fGetLength(a);
	float bNorm = Vector2fGetLength(b);
	float aDotB = Vector2fDotProduct(a, b);

	return acosf(aDotB/(aNorm*bNorm));
}

float Vector2fGetDistanceBetween(Vector2f from, Vector2f to)
{
	return Vector2fGetLength(Vector2fFromPoints(from, to));
}

uint8_t Vector2fIsFinite(Vector2f v)
{
	return isfinite(v.x) && isfinite(v.y);
}
