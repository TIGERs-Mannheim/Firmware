/*
 * line.c
 *
 *  Created on: 10.01.2021
 *      Author: AndreR
 */

#include "line.h"
#include "arm_math.h"
#include "angle_math.h"

Line2f Line2fFromDirection(Vector2f support, Vector2f dir)
{
	Line2f line;
	line.dir = dir;
	line.origin = support;

	return line;
}

Line2f Line2fFromPoints(Vector2f a, Vector2f b)
{
	return Line2fFromDirection(a, Vector2fFromPoints(a, b));
}

static float pointToLineIntersectionParameter(Line2f line, Vector2f point)
{
	Vector2f fromP2Origin = Vector2fFromPoints(line.origin, point);
	return Vector2fDotProduct(fromP2Origin, line.dir);
}

Vector2f Line2fNearestPointOnLine(Line2f line, Vector2f point)
{
	float param = pointToLineIntersectionParameter(line, point);
	Vector2f closest;
	closest.x = line.dir.x*param + line.origin.x;
	closest.y = line.dir.y*param + line.origin.y;
	return closest;
}

float Line2fIntersectLine2fLambda(Line2f reference, Line2f other)
{
	if(Vector2fIsParallelTo(reference.dir, other.dir))
		return NAN;

	const float s1 = reference.origin.x;
	const float s2 = reference.origin.y;
	const float d1 = reference.dir.x;
	const float d2 = reference.dir.y;

	const float x1 = other.origin.x;
	const float x2 = other.origin.y;
	const float r1 = other.dir.x;
	const float r2 = other.dir.y;

	const float detRS = (r1 * s2) - (r2 * s1);
	const float detRX = (r1 * x2) - (r2 * x1);
	const float detDR = (d1 * r2) - (d2 * r1);

	return (detRS - detRX) / detDR;
}

Vector2f Line2fIntersectLine2f(Line2f a, Line2f b)
{
	return Vector2fAdd(a.origin, Vector2fScale(a.dir, Line2fIntersectLine2fLambda(a, b)));
}
