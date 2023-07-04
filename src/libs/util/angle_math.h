/*
 * angle_math.h
 *
 *  Created on: 07.06.2013
 *      Author: AndreR
 */

#pragma once

float	AngleNormalize(float a);
void	AngleNormalizePtr(float* a);

float	AngleDiff(float a, float b);
float	AngleDiffAbs(float a, float b);

float	AngleDeg2Rad(float deg);
float	AngleRad2Deg(float rad);
