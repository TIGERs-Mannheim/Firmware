/*
 * ctrl_panthera.h
 *
 *  Created on: 14.07.2020
 *      Author: AndreR
 */

#pragma once

#include "ctrl.h"
#include "model_enc.h"
#include "model_fric.h"
#include "traj_bang_bang.h"
#include "fusion_ekf.h"
#include "util/config.h"

#pragma pack(push, 1)

typedef struct _CtrlPantheraConfigCtrl
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
} CtrlPantheraConfigCtrl;

typedef struct _CtrlPantheraModelConfig
{
	ModelFricConfig fric;
	ModelEncConfig enc;
} CtrlPantheraConfigModel;

#pragma pack(pop)

typedef struct _CtrlPanthera
{
	float outVelModel[2];
	float outTorque[4];

	CtrlPantheraConfigModel* pConfigModel;
	CtrlPantheraConfigCtrl* pConfigCtrl;

	ModelEnc modelEnc;

	TrajBangBang trajBB;

	uint16_t lastVisionState;
} CtrlPanthera;

extern CtrlPanthera ctrlPanthera;
extern CtrlInstance ctrlPantheraInstance;
extern ConfigFileDesc ctrlPantheraConfigModelDesc;
extern ConfigFileDesc ctrlPantheraConfigCtrlDesc;

void CtrlPantheraInit(CtrlPantheraConfigCtrl* pConfigCtrl, CtrlPantheraConfigModel* pConfigModel, FusionEKFConfig* pConfigEkf);
