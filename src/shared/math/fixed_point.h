/*
 * fixed_point.h
 *
 *  Created on: 11.03.2019
 *      Author: AndreR
 */

#pragma once

#include <stdint.h>

uint32_t	FixedPointUnsignedFromFloat(float input, uint8_t fractionBits);
int32_t		FixedPointSignedFromFloat(float input, uint8_t fractionBits);

float		FixedPointGetEps(uint8_t fractionBits);
float		FixedPointGetMax(uint8_t integerBits, uint8_t fractionBits);
uint8_t		FixedPointIsEqual(float a, float b, uint8_t fractionBits);

float		FixedPointUnsignedSaturate(float input, uint8_t integerBits, uint8_t fractionBits);
float		FixedPointSignedSaturate(float input, uint8_t integerBits, uint8_t fractionBits);

uint32_t	FixedPointUnsignedFromFloatSaturate(float input, uint8_t integerBits, uint8_t fractionBits);

// Sign bit is not included, so integerBits+fractionBits = max. 31
int32_t		FixedPointSignedFromFloatSaturate(float input, uint8_t integerBits, uint8_t fractionBits);


float		FixedPointUnsignedToFloat(uint32_t input, uint8_t fractionBits);
float		FixedPointSignedToFloat(int32_t input, uint8_t fractionBits);
