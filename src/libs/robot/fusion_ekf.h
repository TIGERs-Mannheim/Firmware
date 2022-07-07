/*
 * fusion_ekf.h
 *
 *  Created on: 03.08.2020
 *      Author: AndreR
 */

#pragma once

#include "util/ekf.h"
#include "util/config.h"
#include "util/tracking_filter_2d.h"
#include "util/lag_element.h"
#include "model_enc.h"

typedef struct __attribute__ ((packed)) _FusionEKFConfig
{
	float posNoiseXY;
	float posNoiseW;
	float velNoiseXY;

	float visNoiseXY;
	float visNoiseW;

	float outlierMaxVelXY;
	float outlierMaxVelW;

	float trackingCoeff;

	uint8_t visCaptureDelay;
	uint8_t fusionHorizon;

	uint16_t visionTimeoutMs;

	float emaAccelT;

	float irPosNoise;
	float irMeasNoiseX;
	float irMeasNoiseY;
	float irOutlierMaxVel;
	uint16_t irTimeoutMs;
} FusionEKFConfig;

typedef struct _FusionEKFTimeSlot
{
	struct _meas
	{
		float gyrAcc[3];
		float pos[3];
		uint8_t posUpdated;
	} meas;

	float insState[5];
} FusionEKFTimeSlot;

#define FUSION_EKF_MAX_DELAY 128 // must be power of 2

typedef struct _FusionEKF
{
	// EKF state vector: [ px py pw vx vy ]
	// input vector: [ gyr_w acc_x acc_y ]
	// measurement vector: [ px py pw ]
	EKF ekf;
	float ekfData[EKF_DATA_SIZE(5, 3, 3, 5)];

	float encGyrPos[3];

	struct
	{
		uint16_t online;
		uint32_t timeLastValidSample;
		int32_t turns;
		float lastOrient;
		uint32_t numLateMeasurements;
	} vision;

	FusionEKFConfig* pConfig;

	FusionEKFTimeSlot timeSlots[FUSION_EKF_MAX_DELAY];
	uint32_t timeSlotNow;

	ModelEnc modelEnc;

	LagElementPT1 lagAccel[2];

	struct
	{
		float filteredCurrent;
	} dribbler;

	TrackingFilter2D ballFilter;
	uint32_t ballLastVisibleTimestamp;
	uint32_t ballLastUsedTimestamp;
} FusionEKF;

extern FusionEKF fusionEKF;
extern ConfigFileDesc fusionEKFConfigDesc;

void FusionEKFInit(FusionEKFConfig* pConfigEkf, ModelEncConfig* pConfigModel);
void FusionEKFUpdate(const RobotSensors* pSensors, const RobotCtrlOutput* pOutput, RobotCtrlState* pState);
void FusionEKFSetState(const float* pPos, const float* pVel);
void FusionEKFConfigUpdateCallback(uint16_t cfgId);
