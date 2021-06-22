/*
 * model_fric.c
 *
 *  Created on: 05.08.2020
 *      Author: AndreR
 */

#include "model_fric.h"
#include "ctrl.h"
#include "util/arm_mat_util_f32.h"

static float getFrictionFactor(const float* pVel)
{
	const float abs = sqrtf(pVel[0]*pVel[0] + pVel[1]*pVel[1]);
	if(abs < 1e-3f)
		return 1.0f;

	float local[3] = {pVel[0]/abs, pVel[1]/abs, 0};
	arm_matrix_instance_f32 matLocal = {3, 1, local};
	arm_matrix_instance_f32 matSubwheel = {4, 1, (float[4]){}};

	arm_mat_mult_f32(&ctrl.matXYW2Subwheel, &matLocal, &matSubwheel);

	float sum = 0;
	float sumX = 0;
	for(uint8_t i = 0; i < 4; i++)
	{
		sum += fabsf(matSubwheel.pData[i]);
		sumX += fabsf(MAT_ELEMENT(ctrl.matXYW2Subwheel, i, 0));
	}

	return sum/sumX;
}

void ModelFricGetXY(const ModelFricConfig* pConfig, const float* pVel, float* pFrictionForce)
{
	float frictionDir[2] = {0, 0};
	const float trgVelLocalAbs = sqrtf(pVel[0]*pVel[0] + pVel[1]*pVel[1]);
	if(trgVelLocalAbs > 0.01f)
	{
		frictionDir[0] = pVel[0]/trgVelLocalAbs;
		frictionDir[1] = pVel[1]/trgVelLocalAbs;
	}

	// calculate friction compensation force
	const float coulombFriction = pConfig->coulombXY;
	const float viscousFriction = pConfig->viscousXY;

	float frictionFactor = getFrictionFactor(frictionDir);

	pFrictionForce[0] = frictionDir[0] * frictionFactor * (coulombFriction + viscousFriction * fabsf(pVel[0]));
	pFrictionForce[1] = frictionDir[1] * frictionFactor * (coulombFriction + viscousFriction * fabsf(pVel[1]));
}

float ModelFricGetW(const ModelFricConfig* pConfig, float vel)
{
	float frictionDir = 0;
	const float trgVelLocalAbs = fabsf(vel);
	if(trgVelLocalAbs > 1e-3f)
	{
		frictionDir = vel/trgVelLocalAbs;
	}

	// calculate friction compensation force
	const float coulombFriction = pConfig->coulombW;
	const float viscousFriction = pConfig->viscousW;

	return frictionDir * (coulombFriction + viscousFriction * fabsf(vel));
}
