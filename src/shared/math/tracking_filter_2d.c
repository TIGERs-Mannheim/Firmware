/*
 * tracking_filter_2d.c
 *
 *  Created on: 09.05.2021
 *      Author: AndreR
 */

#include "tracking_filter_2d.h"
#include "arm_mat_util_f32.h"

void TrackingFilter2DInit(TrackingFilter2D* pFilter, float dt, const float* pInitialPos, float initialCovariance, float modelError, const float* pMeasError)
{
	pFilter->dt = dt;

	KFInit(&pFilter->kf, TF2D_NUM_STATES, TF2D_NUM_CONTROL, TF2D_NUM_MEAS, pFilter->kfData);

	MAT_ELEMENT(pFilter->kf.x, 0, 0) = pInitialPos[0];
	MAT_ELEMENT(pFilter->kf.x, 1, 0) = pInitialPos[1];

	arm_mat_scale_f32(&pFilter->kf.Sigma, initialCovariance, &pFilter->kf.Sigma);

	arm_mat_identity_f32(&pFilter->kf.A);
	MAT_ELEMENT(pFilter->kf.A, 0, 2) = dt;
	MAT_ELEMENT(pFilter->kf.A, 1, 3) = dt;

	MAT_ELEMENT(pFilter->kf.C, 0, 0) = 1.0f;
	MAT_ELEMENT(pFilter->kf.C, 1, 1) = 1.0f;

	TrackingFilter2DSetMeasError(pFilter, pMeasError);
	TrackingFilter2DSetModelError(pFilter, modelError);
}

void TrackingFilter2DSetMeasError(TrackingFilter2D* pFilter, const float* pMeasStdDev)
{
	MAT_ELEMENT(pFilter->kf.Ez, 0, 0) = pMeasStdDev[0] * pMeasStdDev[0];
	MAT_ELEMENT(pFilter->kf.Ez, 1, 1) = pMeasStdDev[1] * pMeasStdDev[1];
}

void TrackingFilter2DSetModelError(TrackingFilter2D* pFilter, float modelStdDev)
{
	float dt = pFilter->dt;

	float dt3 = (1.0f / 3.0f) * dt * dt * dt * modelStdDev * modelStdDev;
	float dt2 = (1.0f / 2.0f) * dt * dt * modelStdDev * modelStdDev;
	float dt1 = dt * modelStdDev * modelStdDev;

	MAT_ELEMENT(pFilter->kf.Ex, 0, 0) = dt3;
	MAT_ELEMENT(pFilter->kf.Ex, 1, 1) = dt3;
	MAT_ELEMENT(pFilter->kf.Ex, 2, 2) = dt1;
	MAT_ELEMENT(pFilter->kf.Ex, 3, 3) = dt1;
	MAT_ELEMENT(pFilter->kf.Ex, 0, 2) = dt2;
	MAT_ELEMENT(pFilter->kf.Ex, 1, 3) = dt2;
	MAT_ELEMENT(pFilter->kf.Ex, 2, 0) = dt2;
	MAT_ELEMENT(pFilter->kf.Ex, 3, 1) = dt2;
}

void TrackingFilter2DPredict(TrackingFilter2D* pFilter)
{
	KFPredict(&pFilter->kf);
}

void TrackingFilter2DUpdate(TrackingFilter2D* pFilter, const float* pPosition)
{
	MAT_ELEMENT(pFilter->kf.z, 0, 0) = pPosition[0];
	MAT_ELEMENT(pFilter->kf.z, 1, 0) = pPosition[1];
	KFUpdate(&pFilter->kf);
}

void TrackingFilter2DLimitPosition(TrackingFilter2D* pFilter, float limit)
{
	float* pPos = pFilter->kf.x.pData;

	if(pPos[0] > limit)
		pPos[0] = limit;
	else if(pPos[0] < -limit)
		pPos[0] = -limit;

	if(pPos[1] > limit)
		pPos[1] = limit;
	else if(pPos[1] < -limit)
		pPos[1] = -limit;
}

void TrackingFilter2DLimitVelocity(TrackingFilter2D* pFilter, float limit)
{
	float* pVel = pFilter->kf.x.pData + 2;

	if(pVel[0] > limit)
		pVel[0] = limit;
	else if(pVel[0] < -limit)
		pVel[0] = -limit;

	if(pVel[1] > limit)
		pVel[1] = limit;
	else if(pVel[1] < -limit)
		pVel[1] = -limit;
}

void TrackingFilter2DGetPosition(TrackingFilter2D* pFilter, float* pPosition)
{
	pPosition[0] = MAT_ELEMENT(pFilter->kf.x, 0, 0);
	pPosition[1] = MAT_ELEMENT(pFilter->kf.x, 1, 0);
}

void TrackingFilter2DGetVelocity(TrackingFilter2D* pFilter, float* pVelocity)
{
	pVelocity[0] = MAT_ELEMENT(pFilter->kf.x, 2, 0);
	pVelocity[1] = MAT_ELEMENT(pFilter->kf.x, 3, 0);
}
