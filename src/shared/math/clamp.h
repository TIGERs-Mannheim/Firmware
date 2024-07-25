#pragma once

/**
 * Values above limit are set to limit. Values below -limit are set to -limit.
 *
 * @param in Input value.
 * @param limit Limit for clamping.
 * @return Clamped value if in is beyond limit, otherwise unmodified in value.
 */
float Clampf(float in, float limit);

/**
 * Clamp length of 2D vector.
 *
 * @param pIn Treated as two dimensional array, modified if norm is greater than limit.
 * @param limit Norm of vector used as limit.
 */
void Clamp2Df(float* pIn, float limit);
