/*
 * model_enc.c
 *
 *  Created on: 05.08.2020
 *      Author: AndreR
 */

#include "model_enc.h"
#include <math.h>
#include <stdint.h>

void ModelEncInit(ModelEnc* pModel, ModelEncConfig* pConfig, float dt)
{
	pModel->dt = dt;
	pModel->pConfig = pConfig;
}

void ModelEncStep(ModelEnc* pModel, const float* pInput)
{
//	float velXYNorm = sqrtf(pInput[0]*pInput[0] + pInput[1]*pInput[1]);

	for(uint8_t i = 0; i < 2; i++)
	{
//		const float scaledT = pModel->pConfig->T[i] * velXYNorm + 0.001f;
		const float scaledT = pModel->pConfig->T[i] * fabsf(pInput[i]) + 0.001f;
		const float KoverT = pModel->pConfig->K[i]/scaledT;
		const float oneOverT = 1.0f/scaledT;

		float vDot = KoverT*pInput[i] - oneOverT*pModel->state[i];
		pModel->state[i] += vDot*pModel->dt;
	}
}

void ModelEncInvert(const ModelEnc* pModel, const float* pDesiredVel, float* pOutput)
{
	const uint8_t optimizationSteps = 10;

//	float velXYNorm = sqrtf(pDesiredVel[0]*pDesiredVel[0] + pDesiredVel[1]*pDesiredVel[1]);

	for(uint8_t i = 0; i < 2; i++)
	{
//		const float scaledT = pModel->pConfig->T[i] * velXYNorm + 0.001f;
		const float scaledT = pModel->pConfig->T[i] * fabsf(pDesiredVel[i]) + 0.001f;
		const float KoverT = pModel->pConfig->K[i]/scaledT;
		const float oneOverT = 1.0f/scaledT;

		float optVel = pDesiredVel[i];

		for(uint8_t j = 0; j < optimizationSteps; j++)
		{
			float vDot = KoverT*optVel - oneOverT*pModel->state[i];
			float futureVel = pModel->state[i] + vDot*pModel->dt;
			float err = pDesiredVel[i] - futureVel;
			optVel += err;
		}

		pOutput[i] = optVel;
	}
}
