/*
 * angle_math.h
 *
 *  Created on: 07.06.2013
 *      Author: AndreR
 */

#pragma once

/**
 * Wrap [rad] angle to [-PI..PI)
 * @param a Input angle.
 * @return Wrapped angle.
 */
float	AngleNormalize(float a);

/**
 * Wrap angle pointed to by a.
 * @param a Input/output angle.
 * @see AngleNormalize
 */
void	AngleNormalizePtr(float* a);

/**
 * Shortest angle from a to b in radians. Sign indicates direction.
 */
float	AngleDiff(float a, float b);

/**
 * Shortest angle from a to b in radians, absolute value.
 */
float	AngleDiffAbs(float a, float b);

/**
 * Convert degree to rad.
 */
float	AngleDeg2Rad(float deg);

/**
 * Convert rad to degree.
 */
float	AngleRad2Deg(float rad);
