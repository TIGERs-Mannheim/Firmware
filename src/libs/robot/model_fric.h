/*
 * model_fric.h
 *
 *  Created on: 05.08.2020
 *      Author: AndreR
 *
 * Friction modeling with coulomb and viscous friction compensation.
 */

#pragma once

typedef struct __attribute__ ((packed)) _ModelFricConfig
{
	float coulombXY;
	float viscousXY;
	float coulombW;
	float viscousW;
	float efficiencyXY;
	float efficiencyW;
} ModelFricConfig;

void ModelFricGetXY(const ModelFricConfig* pConfig, const float* pVel, float* pFrictionForce);
float ModelFricGetW(const ModelFricConfig* pConfig, float vel);
