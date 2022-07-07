/*
 * quat.h
 *
 *  Created on: 22.07.2020
 *      Author: AndreR
 */

#pragma once

typedef struct _Quaterniond
{
	double x;
	double y;
	double z;
	double w;
} Quaterniond;

Quaterniond QuatGyroUpdate(Quaterniond quat, double* pGyro, const double dt);

// quaternion dot product. is cosine of angle between them.
double QuatDot(Quaterniond a, Quaterniond b);

// normalize a quaternion.
// typically used to mitigate precision errors.
Quaterniond QuatNormalize(Quaterniond q);
