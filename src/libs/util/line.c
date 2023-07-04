/*
 * line.c
 *
 *  Created on: 10.01.2021
 *      Author: AndreR
 */

#include <util/line.h>
#include "arm_math.h"
#include "util/arm_mat_util_f32.h"
#include "util/angle_math.h"

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
