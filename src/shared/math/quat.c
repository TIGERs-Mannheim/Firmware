/*
 * quat.c
 *
 *  Created on: 22.07.2020
 *      Author: AndreR
 */

#include "quat.h"
#include <math.h>

// Quaternion stuff from: https://github.com/jpreiss/cmath3d/blob/master/math3d.h
Quaterniond QuatGyroUpdate(Quaterniond quat, double* pGyro, const double dt)
{
	// from "Indirect Kalman Filter for 3D Attitude Estimation", N. Trawny, 2005
	Quaterniond q1;
	float const r = 0.5 * dt * pGyro[0];
	float const p = 0.5 * dt * pGyro[1];
	float const y = 0.5 * dt * pGyro[2];

	q1.x =    quat.x + y*quat.y - p*quat.z + r*quat.w;
	q1.y = -y*quat.x +   quat.y + r*quat.z + p*quat.w;
	q1.z =  p*quat.x - r*quat.y +   quat.z + y*quat.w;
	q1.w = -r*quat.x - p*quat.y - y*quat.z +   quat.w;

	return q1;
}

// quaternion dot product. is cosine of angle between them.
double QuatDot(Quaterniond a, Quaterniond b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

// normalize a quaternion.
// typically used to mitigate precision errors.
Quaterniond QuatNormalize(Quaterniond q)
{
	double s = 1.0 / sqrt(QuatDot(q, q));

	return (Quaterniond){s*q.x, s*q.y, s*q.z, s*q.w};
}
