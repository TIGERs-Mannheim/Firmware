/*
 * model_enc.h
 *
 *  Created on: 05.08.2020
 *      Author: AndreR
 *
 * Encoder velocity modelling:
 * Measured encoder velocity => actual robot velocity
 *
 * Approximately a first order lag element with a total velocity dependent time constant.
 */

#pragma once

typedef struct __attribute__ ((packed)) _ModelEncConfig
{
	float K[2];
	float T[2];
} ModelEncConfig;

typedef struct _ModelEnc
{
	ModelEncConfig* pConfig;

	float state[2];
	float dt;
} ModelEnc;

void ModelEncInit(ModelEnc* pModel, ModelEncConfig* pConfig, float dt);
void ModelEncStep(ModelEnc* pModel, const float* pInput);
void ModelEncInvert(const ModelEnc* pModel, const float* pDesiredVel, float* pOutput);
