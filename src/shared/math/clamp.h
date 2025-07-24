#pragma once

#include <stdint.h>

/**
 * Values above limit are set to limit. Values below -limit are set to -limit.
 *
 * @param in Input value.
 * @param limit Limit for clamping.
 * @return Clamped value if in is beyond limit, otherwise unmodified in value.
 */
float Clampf(float in, float limit);

/**
 * Values above max or below are set to their respective limits.
 *
 * @param in Input value.
 * @param min Minimum value.
 * @param max Maximum value.
 * @return Clamped value if outside min/max, otherwise unmodified value.
 */
float ClampMinMaxf(float in, float min, float max);

// Same as ClampMinMaxf but result rounded to nearest integer.
int32_t ClampMinMaxToIntf(float in, float min, float max);

// Clamp float to int16_t range.
int16_t ClampToInt16f(float in);

// Clamp float to uint8_t range.
uint8_t ClampToUint8f(float in);

/**
 * Clamp length of 2D vector.
 *
 * @param pIn Treated as two dimensional array, modified if norm is greater than limit.
 * @param limit Norm of vector used as limit.
 */
void Clamp2Df(float* pIn, float limit);
