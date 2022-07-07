/*
 * float16_t.h
 *
 *  Created on: 26.03.2013
 *      Author: AndreR
 */

#ifndef FLOAT16_T_H_
#define FLOAT16_T_H_

#include <stdint.h>

typedef uint16_t float16_t;

float halfFloatToFloat(float16_t f16);
float16_t floatToHalfFloat(float f32);

#endif /* FLOAT16_T_H_ */
