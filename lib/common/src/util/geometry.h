/*
 * geometry.h
 *
 *  Created on: 10.01.2021
 *      Author: AndreR
 */

#pragma once

typedef union _Vector2f
{
	struct
	{
		float x;
		float y;
	};

	float data[2];
} Vector2f;

typedef struct _Line2f
{
	Vector2f origin;
	Vector2f dir;
} Line2f;

Vector2f GeometryVector2f(const float* pData);
Vector2f GeometryCreateVectorFromAToB(Vector2f from, Vector2f to);
Vector2f GeometryCreateVectorFromAngle(float rad, float length);
Vector2f GeometryVectorAdd(Vector2f a, Vector2f b);
Vector2f GeometryVectorSubtract(Vector2f a, Vector2f b);
float GeometryLookAt(Vector2f from, Vector2f to);
float GeometryNorm(Vector2f vec);
float GeometryDistanceBetweenTwoPoints(Vector2f from, Vector2f to);
float GeometryDotProduct(Vector2f a, Vector2f b);
float GeometryPointToLineIntersectionParameter(Line2f line, Vector2f point);
Vector2f GeometryClosestPointOnLine(Line2f line, Vector2f point);
float GeometryAngleBetweenVectors(Vector2f a, Vector2f b);

float GeometryAngleDiff(float a, float b);
float GeometryAngleDiffAbs(float a, float b);
float GeometryDeg2Rad(float deg);
float GeometryRad2Deg(float rad);
