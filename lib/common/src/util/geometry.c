/*
 * geometry.c
 *
 *  Created on: 10.01.2021
 *      Author: AndreR
 */

#include "geometry.h"
#include "arm_math.h"
#include "util/arm_mat_util_f32.h"
#include "util/angle_math.h"

Vector2f GeometryVector2f(const float* pData)
{
	Vector2f vec = {{ pData[0], pData[1] }};
	return vec;
}

Vector2f GeometryVectorAdd(Vector2f a, Vector2f b)
{
	return (Vector2f){{ a.x + b.x, a.y + b.y }};
}

Vector2f GeometryVectorSubtract(Vector2f a, Vector2f b)
{
	return (Vector2f){{ a.x - b.x, a.y - b.y }};
}

Vector2f GeometryCreateVectorFromAToB(Vector2f from, Vector2f to)
{
	return GeometryVectorSubtract(to, from);
}

Vector2f GeometryCreateVectorFromAngle(float rad, float length)
{
	Vector2f out;
	out.x = arm_cos_f32(rad) * length;
	out.y = arm_sin_f32(rad) * length;
	return out;
}

float GeometryLookAt(Vector2f from, Vector2f to)
{
	Vector2f dir = GeometryCreateVectorFromAToB(from, to);

	return arm_atan2_f32(dir.y, dir.x);
}

// shortest angle from a to b in radians
float GeometryAngleDiff(float a, float b)
{
	return AngleNormalize(AngleNormalize(b) - AngleNormalize(a));
}

// shortest absolute angle between a and b
float GeometryAngleDiffAbs(float a, float b)
{
	return fabsf(GeometryAngleDiff(a, b));
}

float GeometryDeg2Rad(float deg)
{
	return deg * ((float)M_PI)/180.0f;
}

float GeometryRad2Deg(float rad)
{
	return rad * (180.0f/(float)M_PI);
}

float GeometryNorm(Vector2f vec)
{
	return sqrtf(vec.x*vec.x + vec.y*vec.y);
}

float GeometryDistanceBetweenTwoPoints(Vector2f from, Vector2f to)
{
	Vector2f diff = GeometryCreateVectorFromAToB(from, to);

	return GeometryNorm(diff);
}

float GeometryDotProduct(Vector2f a, Vector2f b)
{
	return a.x*b.x + a.y*b.y;
}

float GeometryPointToLineIntersectionParameter(Line2f line, Vector2f point)
{
	Vector2f fromP2Origin = GeometryCreateVectorFromAToB(line.origin, point);
	return GeometryDotProduct(fromP2Origin, line.dir);
}

Vector2f GeometryClosestPointOnLine(Line2f line, Vector2f point)
{
	float param = GeometryPointToLineIntersectionParameter(line, point);
	Vector2f closest;
	closest.x = line.dir.x*param + line.origin.x;
	closest.y = line.dir.y*param + line.origin.y;
	return closest;
}

float GeometryAngleBetweenVectors(Vector2f a, Vector2f b)
{
	float aNorm = GeometryNorm(a);
	float bNorm = GeometryNorm(b);
	float aDotB = GeometryDotProduct(a, b);

	return acosf(aDotB/(aNorm*bNorm));
}
