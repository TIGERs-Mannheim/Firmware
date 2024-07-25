/*
 * vector.h
 *
 *  Created on: 26.12.2012
 *      Author: AndreR
 */

#pragma once

#include <stdint.h>

typedef union _Vector2f
{
	float data[2];

	struct
	{
		float x;
		float y;
	};
} Vector2f;

Vector2f Vector2fFromData(const float* pData);
Vector2f Vector2fFromXY(float x, float y);
Vector2f Vector2fFromPoints(Vector2f from, Vector2f to);
Vector2f Vector2fFromAngleLength(float rad, float length);

Vector2f Vector2fAdd(Vector2f a, Vector2f b);
Vector2f Vector2fSubtract(Vector2f a, Vector2f b);
Vector2f Vector2fScale(Vector2f a, float factor);
Vector2f Vector2fScaleTo(Vector2f a, float length);
Vector2f Vector2fNormalize(Vector2f v);
Vector2f Vector2fRotate(Vector2f v, float rad);
Vector2f Vector2fGetNormalVector(Vector2f v);
float	Vector2fDotProduct(Vector2f a, Vector2f b);

float	Vector2fGetLength(Vector2f v);
float	Vector2fGetAngle(Vector2f v);
float	Vector2fLookAtGetAngle(Vector2f from, Vector2f to);
float	Vector2fGetAngleBetween(Vector2f a, Vector2f b);

float	Vector2fGetDistanceBetween(Vector2f from, Vector2f to);

uint8_t	Vector2fIsFinite(Vector2f v);
uint8_t Vector2fIsParallelTo(Vector2f a, Vector2f b);

void Vector2fTurnLocal2Global(float globalOrientation, float localX, float localY, float* pGlobalX, float* pGlobalY);
void Vector2fTurnGlobal2Local(float globalOrientation, float globalX, float globalY, float* pLocalX, float* pLocalY);
void Vector2fRotatePtr(float angle, float inX, float inY, float* pOut);
