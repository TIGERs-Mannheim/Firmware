/*
 * ctrl_panthera.h
 *
 *  Created on: 14.07.2020
 *      Author: AndreR
 */

#pragma once

#include "model_enc.h"
#include "model_fric.h"
#include "traj_bang_bang.h"
#include "fusion_ekf.h"
#include "util/config.h"

#pragma pack(push, 1)

typedef struct _CtrlPantheraConfigCtrlDrive
{
	struct
	{
		float k;
		float max;
		float antiJitterTheshold;
	} posXY;

	struct
	{
		float k;
		float max;
		float antiJitterTheshold;
	} posW;

	struct
	{
		float k;
		float maxCentrifugalAcc;
	} velW;

	float torqueVectoringStrength;
} CtrlPantheraConfigCtrlDrive;

typedef struct _CtrlPantheraConfigCtrlDribbler
{
	// Dynamic dribbler force target value
	int16_t forceLimitAngle; // [deg], usually negative (-20..-70)
	uint16_t forceFromFriction; // [mN]
	uint16_t forceDropRate; // [mN/s] Force reduction rate to drop ball
	uint16_t forceMinVal_mN; // Dribbler force below this value signals drop complete, output below this will be set to this

	// Automatic velocity increase to increase dribbler force
	uint16_t velLimit; // [mm/s]
	uint16_t velInc; // [mm/s^2]
	uint16_t velDec; // [mm/s^2]

	// Acceleration reduction if dribbler force not achieved
	uint16_t achievedFractionMin; // Minimum scale for acceleration
	uint16_t achievedFractionMax; // Above this fraction full acceleration is used
} CtrlPantheraConfigCtrlDribbler;

typedef struct _CtrlPantheraConfigModel
{
	ModelFricConfig fric;
	ModelEncConfig enc;
} CtrlPantheraConfigModel;

#pragma pack(pop)

typedef struct _CtrlPanthera
{
	float outVelModel[2];
	float outTorque[4];

	float dribblerVel;
	float dribblerLastInputVel;
	float dropRateForce;
	float dropRateSpeed;

	CtrlPantheraConfigModel* pConfigModel;
	CtrlPantheraConfigCtrlDrive* pConfigCtrlDrive;
	CtrlPantheraConfigCtrlDribbler* pConfigCtrlDribbler;

	ModelEnc modelEnc;

	TrajBangBang trajBB;

	uint16_t lastVisionState;
} CtrlPanthera;

extern CtrlPanthera ctrlPanthera;
extern ConfigFileDesc ctrlPantheraConfigModelDesc;
extern ConfigFileDesc ctrlPantheraConfigCtrlDriveDesc;
extern ConfigFileDesc ctrlPantheraConfigCtrlDribblerDesc;

void CtrlPantheraInit(CtrlPantheraConfigCtrlDrive* pConfigCtrlDrive, CtrlPantheraConfigCtrlDribbler* pConfigCtrlDribbler,
		CtrlPantheraConfigModel* pConfigModel, FusionEKFConfig* pConfigEkf, const float* pDefaultPos);
void CtrlPantheraUpdate(const RobotCtrlState* pState, const SkillOutput* pInput, RobotCtrlOutput* pOutput, RobotCtrlReference* pReference);
