/*
 * vector.h
 *
 *  Created on: 26.12.2012
 *      Author: AndreR
 */

#ifndef VECTOR_H_
#define VECTOR_H_

#include "arm_math.h"

typedef union _Vector3f
{
	float data[3];

	struct
	{
		float x;
		float y;
		union
		{
			float z;
			float w;
		};
	};
} Vector3f;

typedef union _Vector2f
{
	float data[2];

	struct
	{
		float x;
		float y;
	};
} Vector2f;

Vector2f Vector2fCreate(float x, float y);
Vector3f Vector3fCreate(float x, float y, float z);
float Vector2fLength(Vector2f v);
float Vector3fLength(Vector3f v);
Vector2f Vector2fSubtract(Vector2f a, Vector2f b);
Vector2f Vector2fAdd(Vector2f a, Vector2f b);
Vector2f Vector2fMultiply(Vector2f a, float factor);
Vector2f Vector2fScale(Vector2f a, float length);
float Vector2fDistance(Vector2f a, Vector2f b);
float Vector3fDistance(Vector3f a, Vector3f b);
float Vector2fAngle(Vector2f a);

#endif /* VECTOR_H_ */
