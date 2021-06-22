/*
 * lag_element.h
 *
 *  Created on: 01.01.2015
 *      Author: AndreR
 */

#pragma once

#include "arm_math.h"

typedef struct _LagElementPT1
{
	arm_biquad_casd_df1_inst_f32 filter;
	float32_t state[4];
	float32_t coeff[5];
	float32_t lastOut;
} LagElementPT1;

void LagElementPT1Init(LagElementPT1* pPT1, float32_t K, float32_t T, float32_t sampleTime);
void LagElementPT1DerivativeInit(LagElementPT1* pPT1, float32_t K, float32_t T, float32_t sampleTime);
float32_t LagElementPT1Process(LagElementPT1* pPT1, float32_t input);
void LagElementPT1SetState(LagElementPT1* pPT1, float32_t set);
