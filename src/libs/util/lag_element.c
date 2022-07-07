/*
 * lag_element.c
 *
 *  Created on: 01.01.2015
 *      Author: AndreR
 */

#include "lag_element.h"

void LagElementPT1Init(LagElementPT1* pPT1, float32_t K, float32_t T, float32_t sampleTime)
{
	float a = (2*T - sampleTime) / (2*T + sampleTime);
	float b = K*sampleTime / (2*T + sampleTime);

	pPT1->coeff[0] = b;
	pPT1->coeff[1] = b;
	pPT1->coeff[2] = 0.0f;
	pPT1->coeff[3] = a;
	pPT1->coeff[4] = 0.0f;

	arm_biquad_cascade_df1_init_f32(&pPT1->filter, 1, pPT1->coeff, pPT1->state);
}

void LagElementPT1DerivativeInit(LagElementPT1* pPT1, float32_t K, float32_t T, float32_t sampleTime)
{
	float a = (2*T - sampleTime) / (2*T + sampleTime);
	float c = 2*K / (2*T + sampleTime);

	pPT1->coeff[0] = c;
	pPT1->coeff[1] = -c;
	pPT1->coeff[2] = 0.0f;
	pPT1->coeff[3] = a;
	pPT1->coeff[4] = 0.0f;

	arm_biquad_cascade_df1_init_f32(&pPT1->filter, 1, pPT1->coeff, pPT1->state);
}

float32_t LagElementPT1Process(LagElementPT1* pPT1, float32_t input)
{
	arm_biquad_cascade_df1_f32(&pPT1->filter, &input, &pPT1->lastOut, 1);

	return pPT1->lastOut;
}

void LagElementPT1SetState(LagElementPT1* pPT1, float32_t set)
{
	pPT1->state[0] = set;
	pPT1->state[1] = set;
	pPT1->state[2] = set;
	pPT1->state[3] = set;

	pPT1->lastOut = set;
}
