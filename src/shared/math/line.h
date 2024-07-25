/*
 * line.h
 *
 *  Created on: 10.01.2021
 *      Author: AndreR
 */

#pragma once

#include "vector.h"

typedef struct _Line2f
{
	Vector2f origin;
	Vector2f dir;
} Line2f;

Line2f Line2fFromDirection(Vector2f support, Vector2f dir);
Line2f Line2fFromPoints(Vector2f a, Vector2f b);
Vector2f Line2fNearestPointOnLine(Line2f line, Vector2f point);
Vector2f Line2fIntersectLine2f(Line2f a, Line2f b);
float Line2fIntersectLine2fLambda(Line2f reference, Line2f other);
