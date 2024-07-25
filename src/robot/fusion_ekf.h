/*
 * fusion_ekf.h
 *
 *  Created on: 03.08.2020
 *      Author: AndreR
 */

#pragma once

#include "math/ekf.h"
#include "math/tracking_filter_2d.h"
#include "math/lag_element.h"
#include "util/config.h"
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

	float ballCenteringFactor;
	float ballCenteringVelLimit;

	uint8_t dribblerStrongOn; // 0-100
	uint8_t dribblerStrongOff; // 0-100

	uint16_t ballTimeoutMs;
	uint16_t activeDribblingForce_mN;
} FusionEKFConfig;

typedef struct _FusionEKFDribblerIdleCurTable
{
	uint16_t speed_mmDs[15];
	uint16_t cur_mA[15];
	uint8_t usedPoints;
} FusionEKFDribblerIdleCurTable;

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
		LagElementPT1 lagCurrent;

		uint8_t isStrongDribbling;
		uint32_t strongTicks;

		FusionEKFDribblerIdleCurTable idleCurrentTable;
		FlashFile* pIdleCurrentFile;
	} dribbler;

	float ballPosGlobal[2];
	float ballVelGlobal[2];
	uint32_t ballLastDetectedTimestamp;
} FusionEKF;

extern FusionEKF fusionEKF;
extern ConfigFileDesc fusionEKFConfigDesc;

void FusionEKFInit(FusionEKFConfig* pConfigEkf, ModelEncConfig* pConfigModel);
void FusionEKFUpdate(const RobotSensors* pSensors, const RobotCtrlOutput* pLastCtrlOut, const RobotCtrlReference* pLastCtrlRef, RobotCtrlState* pState);
void FusionEKFSetState(const float* pPos, const float* pVel);
void FusionEKFConfigUpdateCallback(uint16_t cfgId);
