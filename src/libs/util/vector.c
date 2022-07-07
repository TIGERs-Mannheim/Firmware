/*
 * vector.c
 *
 *  Created on: 28.06.2013
 *      Author: AndreR
 */

#include "vector.h"
#include "angle_math.h"

float Vector2fLength(Vector2f v)
{
	return sqrtf(v.x*v.x + v.y*v.y);
}

float Vector3fLength(Vector3f v)
{
	return sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
}

Vector2f Vector2fCreate(float x, float y)
{
	Vector2f v;
	v.x = x;
	v.y = y;

	return v;
}

Vector3f Vector3fCreate(float x, float y, float z)
{
	Vector3f v;
	v.x = x;
	v.y = y;
	v.z = z;

	return v;
}

Vector2f Vector2fSubtract(Vector2f a, Vector2f b)
{
	Vector2f v;
	v.x = a.x-b.x;
	v.y = a.y-b.y;

	return v;
}

Vector2f Vector2fAdd(Vector2f a, Vector2f b)
{
	Vector2f v;
	v.x = a.x+b.x;
	v.y = a.y+b.y;

	return v;
}

Vector2f Vector2fMultiply(Vector2f a, float factor)
{
	Vector2f v;
	v.x = a.x*factor;
	v.y = a.y*factor;

	return v;
}

Vector2f Vector2fScale(Vector2f a, float length)
{
	float oldLength = Vector2fLength(a);
	if (oldLength != 0)
	{
		return Vector2fMultiply(a, (length / oldLength));
	}
	// You tried to scale a null-vector to a non-zero length! Vector stays unaffected.
	// but this is normal Math. if vector is zero, result is zero too
	return Vector2fCreate(a.x, a.y);
}

float Vector2fDistance(Vector2f a, Vector2f b)
{
	return fabsf(Vector2fLength(a) - Vector2fLength(b));
}

float Vector3fDistance(Vector3f a, Vector3f b)
{
	return fabsf(Vector3fLength(a) - Vector3fLength(b));
}

float Vector2fAngle(Vector2f a)
{
	float result = acosf(a.x / Vector2fLength(a));
	result = AngleNormalize(result);
	if (a.y < 0)
	{
		result = -result;
	}
	return result;
}
