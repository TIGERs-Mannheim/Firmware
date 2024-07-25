/*
 * tracking_filter_2d.h
 *
 *  Created on: 09.05.2021
 *      Author: AndreR
 */

#pragma once

#include "kf.h"

#define TF2D_NUM_STATES 4
#define TF2D_NUM_MEAS 2
#define TF2D_NUM_CONTROL 1

typedef struct _TrackingFilter2D
{
	KF kf;
	float kfData[KF_DATA_SIZE(TF2D_NUM_STATES, TF2D_NUM_CONTROL, TF2D_NUM_MEAS, TF2D_NUM_STATES)];
	float dt;
} TrackingFilter2D;

void TrackingFilter2DInit(TrackingFilter2D* pFilter, float dt, const float* pInitialPos,
		float initialCovariance, float modelError, const float* pMeasError);
void TrackingFilter2DSetMeasError(TrackingFilter2D* pFilter, const float* pMeasStdDev);
void TrackingFilter2DSetModelError(TrackingFilter2D* pFilter, float modelStdDev);
void TrackingFilter2DPredict(TrackingFilter2D* pFilter);
void TrackingFilter2DUpdate(TrackingFilter2D* pFilter, const float* pPosition);
void TrackingFilter2DGetPosition(TrackingFilter2D* pFilter, float* pPosition);
void TrackingFilter2DGetVelocity(TrackingFilter2D* pFilter, float* pVelocity);
void TrackingFilter2DLimitPosition(TrackingFilter2D* pFilter, float limit);
void TrackingFilter2DLimitVelocity(TrackingFilter2D* pFilter, float limit);
