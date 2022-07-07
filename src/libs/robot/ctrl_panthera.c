/*
 * ctrl_panthera.c
 *
 *  Created on: 14.07.2020
 *      Author: AndreR
 */

#include "ctrl_panthera.h"
#include "util/angle_math.h"
#include "util/arm_mat_util_f32.h"
#include "util/map_to_range.h"
#include "struct_ids.h"
#include <math.h>

static void init(const float* pDefaultPos);
static void controller(const RobotCtrlState* pState, const DriveInput* pDrive, RobotCtrlOutput* pOutput, RobotCtrlReference* pReference);

CtrlPanthera ctrlPanthera;

CtrlInstance ctrlPantheraInstance = {
	.pInit = &init,
	.pExit = 0,
	.pStateEstimationFunc = &FusionEKFUpdate,
	.pControllerFunc = &controller,
	.pLog = 0
};

ConfigFileDesc ctrlPantheraConfigModelDesc =
	{ 0, 0, "", 10, (ElementDesc[]) {
		{ FLOAT, "xy_coulomb", "N", "xy/coulomb" },
		{ FLOAT, "xy_viscous", "Ns/m", "xy/viscous" },
		{ FLOAT, "w_coulomb", "N", "w/coulomb" },
		{ FLOAT, "w_viscous", "Ns/m", "w/viscous" },
		{ FLOAT, "xy_efficiency", "-", "xy/efficiency" },
		{ FLOAT, "w_efficiency", "-", "w/efficiency" },
		{ FLOAT, "enc_Kx", "-", "enc/gain/x" },
		{ FLOAT, "enc_Ky", "-", "enc/gain/y" },
		{ FLOAT, "enc_Tx", "s", "enc/time/x" },
		{ FLOAT, "enc_Ty", "s", "enc/time/y" },
	} };

ConfigFileDesc ctrlPantheraConfigCtrlDesc =
	{ 0, 0, "", 8, (ElementDesc[]) {
		{ FLOAT, "pos_xy_k", "", "pos/xy/k" },
		{ FLOAT, "pos_xy_max", "m/s", "pos/xy/max" },
		{ FLOAT, "pos_xy_ajitter", "", "pos/xy/anti_jitter" },
		{ FLOAT, "pos_w_k", "", "pos/w/k" },
		{ FLOAT, "pos_w_max", "rad/s", "pos/w/max" },
		{ FLOAT, "pos_w_ajitter", "", "pos/w/anti_jitter" },
		{ FLOAT, "vel_w_k", "", "vel/w/k" },
		{ FLOAT, "vel_w_cent_acc", "m/s^2", "vel/w/cent_acc" },
	} };

void CtrlPantheraInit(CtrlPantheraConfigCtrl* pConfigCtrl, CtrlPantheraConfigModel* pConfigModel, FusionEKFConfig* pConfigEkf)
{
	ctrlPanthera.pConfigCtrl = pConfigCtrl;
	ctrlPanthera.pConfigModel = pConfigModel;

	ModelEncInit(&ctrlPanthera.modelEnc, &ctrlPanthera.pConfigModel->enc, CTRL_DELTA_T);
	FusionEKFInit(pConfigEkf, &pConfigModel->enc);
	TrajBangBangInit(&ctrlPanthera.trajBB);
}

static void init(const float* pDefaultPos)
{
	float zeroVel[2] = {0, 0};
	FusionEKFSetState(pDefaultPos, zeroVel);
}

static float clampf(float in, float limit)
{
	if(in > limit)
		return limit;

	if(in < -limit)
		return -limit;

	return in;
}

static void clamp2Df(float* pIn, float limit)
{
	const float inAbs = sqrtf(pIn[0]*pIn[0] + pIn[1]*pIn[1]);

	if(inAbs > limit)
	{
		const float scale = limit/inAbs;

		pIn[0] *= scale;
		pIn[1] *= scale;
	}
}

static void controlXY(const DriveInput* pDrive, const RobotCtrlReference* pReference,
		const RobotCtrlState* pState, float* pOutVel, float* pOutAcc)
{
	pOutVel[0] = 0;
	pOutVel[1] = 0;
	pOutAcc[0] = 0;
	pOutAcc[1] = 0;

	switch(pDrive->modeXY)
	{
		case DRIVE_MODE_GLOBAL_POS:
		case DRIVE_MODE_GLOBAL_POS_ASYNC:
		{
			// use current state to rotate traj output
			CtrlUtilRotate(pReference->trajPos[2]-pState->pos[2], pReference->trajVelLocal[0], pReference->trajVelLocal[1], pOutVel);
			CtrlUtilRotate(pReference->trajPos[2]-pState->pos[2], pReference->trajAccLocal[0], pReference->trajAccLocal[1], pOutAcc);

			// XY position control
			float ctrl[2];
			float xErr = pReference->trajPos[0] - pState->pos[0];
			float yErr = pReference->trajPos[1] - pState->pos[1];
			ctrl[0] = xErr * ctrlPanthera.pConfigCtrl->posXY.k;
			ctrl[1] = yErr * ctrlPanthera.pConfigCtrl->posXY.k;
			clamp2Df(ctrl, ctrlPanthera.pConfigCtrl->posXY.max);

			float totalErr = sqrtf(xErr*xErr + yErr*yErr);
			float scale = MapToRangef32(0.0f, ctrlPanthera.pConfigCtrl->posXY.antiJitterTheshold, 0.0f, 1.0f, totalErr);
			ctrl[0] *= scale;
			ctrl[1] *= scale;

			// convert to local pos PID output
			float localPosPidOutput[2];
			CtrlUtilTurnGlobal2Local(pState->pos[2], ctrl[0], ctrl[1], localPosPidOutput, localPosPidOutput+1);

			pOutVel[0] += localPosPidOutput[0];
			pOutVel[1] += localPosPidOutput[1];
		}
		break;
		case DRIVE_MODE_LOCAL_VEL:
		{
			memcpy(pOutVel, pReference->trajVelLocal, sizeof(float)*2);
			memcpy(pOutAcc, pReference->trajAccLocal, sizeof(float)*2);
		}
		break;
		default:
		{
			return;
		}
	}

	// respect strict velocity limit if present
	if(pDrive->limits.strictVelLimit)
		clamp2Df(pOutVel, pDrive->limits.velMaxXY);

	// compensate rotation
	float rot = -pState->vel[2]*CTRL_DELTA_T*0.5f;
	CtrlUtilRotate(rot, pOutVel[0], pOutVel[1], pOutVel);
	CtrlUtilRotate(rot, pOutAcc[0], pOutAcc[1], pOutAcc);
}

static void controlW(const DriveInput* pDrive, const RobotCtrlReference* pReference,
		const RobotCtrlState* pState, float* pOutVel, float* pOutAcc)
{
	pOutVel[2] = pReference->trajVelLocal[2];
	pOutAcc[2] = pReference->trajAccLocal[2];

	switch(pDrive->modeW)
	{
		case DRIVE_MODE_GLOBAL_POS:
		case DRIVE_MODE_GLOBAL_POS_ASYNC:
		case DRIVE_MODE_LOCAL_VEL:
		{
			float outScale = 1.0f;

			if(pDrive->modeW == DRIVE_MODE_GLOBAL_POS || pDrive->modeW == DRIVE_MODE_GLOBAL_POS_ASYNC)
			{
				// P control on position
				const float orientError = pReference->trajPos[2] - pState->pos[2];
				pOutVel[2] += clampf(orientError * ctrlPanthera.pConfigCtrl->posW.k, ctrlPanthera.pConfigCtrl->posW.max);

				// gain scheduling based on distance to target orientation
				outScale = MapToRangef32(0.0f, ctrlPanthera.pConfigCtrl->posW.antiJitterTheshold, 0.0f, 1.0f, fabsf(orientError));
			}

			// P control on velocity (based on gyro)
			const float angVelError = pReference->trajVelLocal[2] - pState->vel[2];
			pOutVel[2] += angVelError * ctrlPanthera.pConfigCtrl->velW.k * outScale;

			pOutAcc[2] *= outScale;
		}
		break;
		default:
		{
			return;
		}
	}
}

static void controller(const RobotCtrlState* pState, const DriveInput* pDrive, RobotCtrlOutput* pOutput, RobotCtrlReference* pReference)
{
	if(ctrlPanthera.lastVisionState == 0 && fusionEKF.vision.online)
	{
		TrajBangBangResetState(&ctrlPanthera.trajBB, pState);
	}

	ctrlPanthera.lastVisionState = fusionEKF.vision.online;

	TrajBangBangUpdate(&ctrlPanthera.trajBB, pState, pDrive, pReference, ctrlPanthera.pConfigCtrl->velW.maxCentrifugalAcc);

	float outAcc[3];
	float outVel[3];

	controlXY(pDrive, pReference, pState, outVel, outAcc);
	controlW(pDrive, pReference, pState, outVel, outAcc);

	// coulomb & viscous friction compensation
	float frictionForce[3];
	ModelFricGetXY(&ctrlPanthera.pConfigModel->fric, outVel, frictionForce);
	frictionForce[2] = ModelFricGetW(&ctrlPanthera.pConfigModel->fric, outVel[2]);

	float frictionAcc[3];
	frictionAcc[0] = frictionForce[0]/botParams.physical.mass;
	frictionAcc[1] = frictionForce[1]/botParams.physical.mass;
	frictionAcc[2] = frictionForce[2]/ctrl.momentOfInertia;

	for(uint8_t i = 0; i < 3; i++)
		outAcc[i] += frictionAcc[i];

	// compensate for output efficiency
	outAcc[0] *= 1.0f/ctrlPanthera.pConfigModel->fric.efficiencyXY;
	outAcc[1] *= 1.0f/ctrlPanthera.pConfigModel->fric.efficiencyXY;
	outAcc[2] *= 1.0f/ctrlPanthera.pConfigModel->fric.efficiencyW;

	// run non-linear model in parallel and optimize output vel to cancel dynamics
	float outVelComp[3];

	ModelEncInvert(&ctrlPanthera.modelEnc, outVel, outVelComp);
	ModelEncStep(&ctrlPanthera.modelEnc, outVelComp);

	outVelComp[2] = outVel[2];

	// transform acceleration to motor feedforward torque
	float motorTorque[4];
	CtrlUtilLocalAccToMotorTorque(outAcc, motorTorque);

	// limit motor current slope
	const float maxCurrentIncPerMs = 0.5f; // [A]
	const float maxTorqueSlope = maxCurrentIncPerMs * botParams.driveTrain.motor.Km * 1.0f/CTRL_DELTA_T;

	for(uint8_t i = 0; i < 4; i++)
	{
		float toTargetTorque = motorTorque[i] - ctrlPanthera.outTorque[i];
		float toTargetTorqueDeriv = fabsf(toTargetTorque) * 1.0f/CTRL_DELTA_T;

		float scale = 1.0f;
		if(toTargetTorqueDeriv > maxTorqueSlope)
			scale = maxTorqueSlope/toTargetTorqueDeriv;

		toTargetTorque *= scale;

		ctrlPanthera.outTorque[i] += toTargetTorque;
	}

	memcpy(pOutput->motorTorque, ctrlPanthera.outTorque, sizeof(float)*4);

	// transform velocity to motor velocities
	CtrlUtilLocalVelToMotorVel(outVelComp, pOutput->motorVel, 0);
	pOutput->ctrlMode = MOT_CTRL_VEL_TORQUE;

	if(pDrive->modeXY == DRIVE_MODE_LOCAL_FORCE || pDrive->modeW == DRIVE_MODE_LOCAL_FORCE)
	{
		CtrlUtilLocalForceToMotorTorque((float*)pDrive->localForce, pOutput->motorTorque);
		memcpy(ctrlPanthera.outTorque, pOutput->motorTorque, sizeof(float)*4);
		pOutput->ctrlMode = MOT_CTRL_TORQUE;
	}

	if(pDrive->modeXY == DRIVE_MODE_WHEEL_VEL || pDrive->modeW == DRIVE_MODE_WHEEL_VEL)
	{
		for(uint8_t i = 0; i < 4; i++)
			pOutput->motorVel[i] += pDrive->wheelVel[i]*CTRL_WHEEL_TO_MOTOR_RATIO;
	}

	if(pDrive->modeXY == DRIVE_MODE_OFF && pDrive->modeW == DRIVE_MODE_OFF)
	{
		memset(pOutput->motorVel, 0, sizeof(float)*4);

		pOutput->ctrlMode = MOT_CTRL_OFF;
	}
}
