/*
 * ctrl_tigga.c
 *
 *  Created on: 26.02.2015
 *      Author: AndreR
 */

#include <main/ctrl.h>
#include <main/ctrl_tigga.h>
#include "struct_ids.h"
#include "util/arm_mat_util_f32.h"
#include "util/angle_math.h"
#include "util/sys_time.h"
#include "util/log.h"
#include "util/console.h"
#include "util/map_to_range.h"
#include <float.h>

// DEFINITIONS

// FORWARD DECLARATIONS
static void init(const float* pDefaultPos);
static void controller(const RobotCtrlState* pState, const SkillOutput* pInput, RobotCtrlOutput* pOutput, RobotCtrlReference* pReference);
static void sensorFusion(const RobotSensors* pSensors, const RobotCtrlOutput* pOutput, RobotCtrlState* pState);

static void ekfStateFunc(arm_matrix_instance_f32* pX, const arm_matrix_instance_f32* pU);
static void ekfStateJacobianFunc(const arm_matrix_instance_f32* pX, const arm_matrix_instance_f32* pU, arm_matrix_instance_f32* pF);
static void ekfMeasFunc(const arm_matrix_instance_f32* pX, arm_matrix_instance_f32* pY);
static void ekfMeasJacobianFunc(const arm_matrix_instance_f32* pX, arm_matrix_instance_f32* pH);

static inline const float* visionProc(uint8_t updated, int32_t tDelayUs, const float* pVisPos);

// GLOBAL DATA
CtrlTigga ctrlTigga = {
	.configFilter = {
		.posNoiseXY = 0.001f,
		.posNoiseW = 0.001f,
		.velNoiseXY = 0.03f,
		.visNoiseXY = 0.01f,
		.visNoiseW = 0.1f,
		.outlierMaxVelXY = 3.0f,
		.outlierMaxVelW = 3.0f,
		.trackingCoeff = 2.0f,
		.visCaptureDelay = 30,
		.visProcessingDelayMax = 20,
		.ctrlToGyrAccDelay = 10,
		.visionTimeoutMs = 1000,
	},
	.configPid = {
		.posXY = {150.0f, 10.0f, 0.02f},
		.posW =  {  1.5f,  0.5f, 0.1f},
		.velXY = { 60.0f, 10.0f},
		.velW =  {  0.5f,  0.8f},
		.rotationScaling = {0, 500, 600},
	},
	.configModel = {
		// determined by roll out in Y direction (no rotation compensation, torque == 0), must have plateau phase
		// required data: motor set torque (XYW local output force / absolute force), XY state local velocity (absolute velocity)
		.coulombXY = 7.67f,
		.viscousXY = 3.09f,
		.coulombW = 0.21f, // determined by roll out in W direction (torque == 0), must have plateau phase
		.viscousW = 0.0098f,
		.efficiencyXY = 0.75f, // roll out in Y
		.efficiencyW = 0.36f, // roll out in W
		// determined by controlled move in X and Y direction, with plateau phase, no wheel slipping
		// required data: efficiencyXY, local state velocity, local encoder velocity, local output force
		.xNum = 6.05f,
		.xDenom = 6.47f,
		.yNum = 5.11f,
		.yDenom = 4.62f,
	},
	.configCompass = {
		.hardIronX = -25.639f,
		.hardIronY = -5.025f,
		.softIronCos = 0.8717f,
		.softIronSin = -0.4900f,
		.softEllipseEcc = 0.8608f,
		.fieldOffset = -0.8390f,
	},
};

CtrlInstance ctrlTiggaInstance = {
	.pInit = &init,
	.pExit = 0,
	.pStateEstimationFunc = &sensorFusion,
	.pControllerFunc = &controller,
	.pLog = 0,
};

// LOCAL DATA - CONFIGS
static const ConfigFileDesc configFileDescFilter =
	{ SID_CFG_CTRL_TIGGA_FILTER, 36, "ctrl_tigga/filter", 12, (ElementDesc[]) {
		{ FLOAT, "noise_pos_xy", "m", "noise/pos/xy" },
		{ FLOAT, "noise_pos_w", "rad", "noise/pos/w" },
		{ FLOAT, "noise_vel_xy", "m/s", "noise/vel/xy" },
		{ FLOAT, "noise_vis_xy", "m", "noise/vis/xy" },
		{ FLOAT, "noise_vis_w", "rad", "noise/vis/w" },
		{ FLOAT, "outlier_max_vel_xy", "m/s", "outlier/vel/xy" },
		{ FLOAT, "outlier_max_vel_w", "rad/s", "outlier/vel/w" },
		{ FLOAT, "tracking_coeff", "", "tracking/coeff" },
		{ UINT8, "vis_capture_delay", "ms", "vision/capture/delay" },
		{ UINT8, "vis_prc_delay_max", "ms", "vision/proc/delay (max)" },
		{ UINT8, "ctrl_to_gyr_acc_delay", "ms", "Ctrl to GyrAcc delay" },
		{ UINT16, "vision_timeout_ms", "ms", "vision/timeout" },
	} };

static const ConfigFileDesc configFileDescPid =
	{ SID_CFG_CTRL_TIGGA_PID, 10, "ctrl_tigga/pid", 13, (ElementDesc[]) {
		{ FLOAT, "pos_xy_k", "", "pos/xy/k" },
		{ FLOAT, "pos_xy_max", "", "pos/xy/max" },
		{ FLOAT, "pos_xy_jit_thresh", "m", "pos/xy/anti_jitter_thresh" },
		{ FLOAT, "pos_w_k", "", "pos/w/k" },
		{ FLOAT, "pos_w_max", "", "pos/w/max" },
		{ FLOAT, "pos_w_jit_thresh", "rad", "pos/w/anti_jitter_thresh" },
		{ FLOAT, "vel_xy_k", "", "vel/xy/k" },
		{ FLOAT, "vel_xy_max", "", "vel/xy/max" },
		{ FLOAT, "vel_w_k", "", "vel/w/k" },
		{ FLOAT, "vel_w_max", "", "vel/w/max" },
		{ UINT8, "scale_rotation_out", "", "Scale Rotation Output" },
		{ UINT16, "start_vel", "mm/s", "Start scaling velocity" },
		{ UINT16, "range_vel", "mm/s", "Range of scaling velocity" },
	} };

static const ConfigFileDesc configFileDescModel =
	{ SID_CFG_CTRL_TIGGA_MODEL, 1, "ctrl_tigga/model", 10, (ElementDesc[]) {
		{ FLOAT, "xy_coulomb", "N", "xy/coulomb" },
		{ FLOAT, "xy_viscous", "Ns/m", "xy/viscous" },
		{ FLOAT, "w_coulomb", "N", "w/coulomb" },
		{ FLOAT, "w_viscous", "Ns/m", "w/viscous" },
		{ FLOAT, "xy_efficiency", "-", "xy/efficiency" },
		{ FLOAT, "w_efficiency", "-", "w/efficiency" },
		{ FLOAT, "x_num", "-", "x/numerator" },
		{ FLOAT, "x_denom", "-", "x/denominator" },
		{ FLOAT, "y_num", "-", "y/numerator" },
		{ FLOAT, "y_denom", "-", "y/denominator" },
	} };

static const ConfigFileDesc configFileDescCompass =
	{ SID_CFG_CTRL_TIGGA_COMPASS, 1, "ctrl_tigga/compass", 6, (ElementDesc[]) {
		{ FLOAT, "hard_iron_x", "mT", "hard/x"},
		{ FLOAT, "hard_iron_y", "mT", "hard/y"},
		{ FLOAT, "soft_iron_cos", "-", "soft/cos"},
		{ FLOAT, "soft_iron_sin", "-", "soft/sin"},
		{ FLOAT, "soft_ellipse_ecc", "-", "ellipse_ecc"},
		{ FLOAT, "field_offset", "rad", "field_offset"},
	} };

static void configUpdateCallback(uint16_t cfgId)
{
	switch(cfgId)
	{
		case SID_CFG_CTRL_TIGGA_FILTER:
		{
			uint16_t bufSize = (uint16_t)ctrlTigga.configFilter.visCaptureDelay + (uint16_t)ctrlTigga.configFilter.visProcessingDelayMax;
			if(bufSize > CTRL_TIGGA_MAX_BUFFER_SIZE)
				bufSize = CTRL_TIGGA_MAX_BUFFER_SIZE;

			uint16_t ctrlOutDelay = ctrlTigga.configFilter.ctrlToGyrAccDelay;
			if(ctrlOutDelay > CTRL_TIGGA_MAX_BUFFER_SIZE)
				ctrlOutDelay = CTRL_TIGGA_MAX_BUFFER_SIZE;

			DelayElementSetNumElements(&ctrlTigga.delayGyrAcc, bufSize);
			DelayElementSetNumElements(&ctrlTigga.delayIns, bufSize);
			DelayElementSetNumElements(&ctrlTigga.delayVision, bufSize);
			DelayElementSetNumElements(&ctrlTigga.delayCtrlOut, ctrlOutDelay);

			if(ctrlTigga.ekf.Ex.pData) // pData is null when ctrl_tigga has not been initialized yet
			{
				MAT_ELEMENT(ctrlTigga.ekf.Ex, 0, 0) = ctrlTigga.configFilter.posNoiseXY*ctrlTigga.configFilter.posNoiseXY;
				MAT_ELEMENT(ctrlTigga.ekf.Ex, 1, 1) = ctrlTigga.configFilter.posNoiseXY*ctrlTigga.configFilter.posNoiseXY;
				MAT_ELEMENT(ctrlTigga.ekf.Ex, 2, 2) = ctrlTigga.configFilter.posNoiseW*ctrlTigga.configFilter.posNoiseW;
				MAT_ELEMENT(ctrlTigga.ekf.Ex, 3, 3) = ctrlTigga.configFilter.velNoiseXY*ctrlTigga.configFilter.velNoiseXY;
				MAT_ELEMENT(ctrlTigga.ekf.Ex, 4, 4) = ctrlTigga.configFilter.velNoiseXY*ctrlTigga.configFilter.velNoiseXY;

				MAT_ELEMENT(ctrlTigga.ekf.Ez, 0, 0) = ctrlTigga.configFilter.visNoiseXY*ctrlTigga.configFilter.visNoiseXY;
				MAT_ELEMENT(ctrlTigga.ekf.Ez, 1, 1) = ctrlTigga.configFilter.visNoiseXY*ctrlTigga.configFilter.visNoiseXY;
				MAT_ELEMENT(ctrlTigga.ekf.Ez, 2, 2) = ctrlTigga.configFilter.visNoiseW*ctrlTigga.configFilter.visNoiseW;
			}
		}
		break;
	}
}

static void initEKF()
{
	EKFInit(&ctrlTigga.ekf, 5, 3, 3, ctrlTigga.ekfData);
	arm_mat_scale_f32(&ctrlTigga.ekf.Sigma, 0.001f, &ctrlTigga.ekf.Sigma);
	ctrlTigga.ekf.pState = &ekfStateFunc;
	ctrlTigga.ekf.pStateJacobian = &ekfStateJacobianFunc;
	ctrlTigga.ekf.pMeas = &ekfMeasFunc;
	ctrlTigga.ekf.pMeasJacobian = &ekfMeasJacobianFunc;

	arm_mat_identity_f32(&ctrlTigga.ekf.Ex);
	arm_mat_identity_f32(&ctrlTigga.ekf.Ez);
}

static void init(const float* pDefaultPos)
{
	ctrlTigga.visionOnline = 0;
	ctrlTigga.lastVisionTime = 0;
	ctrlTigga.noVisionCounter = 0;
	ctrlTigga.motorOffCounter = 0;

	ctrlTigga.numLateVisionMeasurements = 0;
	ctrlTigga.maxVisionDelay = 0;

	ctrlTigga.lastInitTime = chVTGetSystemTimeX();

	memset(ctrlTigga.mechState, 0, sizeof(float)*8);
	memset(ctrlTigga.encState, 0, sizeof(float)*8);
	memcpy(ctrlTigga.mechState, pDefaultPos, sizeof(float)*3);
	memcpy(ctrlTigga.encState, pDefaultPos, sizeof(float)*3);

	memset(&ctrlTigga.vision, 0, sizeof(ctrlTigga.vision));
	memset(ctrlTigga.bias, 0, sizeof(ctrlTigga.bias));
	memset(ctrlTigga.virtualVelOutput, 0, sizeof(ctrlTigga.virtualVelOutput));

	// Delay elements
	DelayElementInit(&ctrlTigga.delayGyrAcc, 1, CTRL_TIGGA_MAX_BUFFER_SIZE, sizeof(float)*3, ctrlTigga.delayGyrAccData);
	DelayElementInit(&ctrlTigga.delayIns, 1, CTRL_TIGGA_MAX_BUFFER_SIZE, sizeof(float)*5, ctrlTigga.delayInsData);
	DelayElementInit(&ctrlTigga.delayVision, 1, CTRL_TIGGA_MAX_BUFFER_SIZE, sizeof(CtrlTiggaVisionEntry), ctrlTigga.delayVisionData);
	DelayElementInit(&ctrlTigga.delayCtrlOut, 1, CTRL_TIGGA_MAX_BUFFER_SIZE, sizeof(float)*3, ctrlTigga.delayCtrlOutData);

	// FILTER
	initEKF();

	memcpy(ctrlTigga.ekf.x.pData, pDefaultPos, sizeof(float)*3);

	// CONTROL
	EMAFilterInit(&ctrlTigga.emaBias[0], 0.98f);
	EMAFilterInit(&ctrlTigga.emaBias[1], 0.98f);
	EMAFilterInit(&ctrlTigga.emaBias[2], 0.98f);

	TrajGeneratorInit(&ctrlTigga.trajGen);

	ConfigNotifyUpdate(ctrlTigga.pConfigFileFilter);
	ConfigNotifyUpdate(ctrlTigga.pConfigFilePid);
}

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

static float getFrictionFactorInvScaled(const float* pDir, const float offsetAtZero, const float scaleZeroTo90Deg)
{
	float sumX = 0;
	float sumY = 0;
	for(uint8_t i = 0; i < 4; i++)
	{
		sumX += fabsf(MAT_ELEMENT(ctrl.matXYW2Subwheel, i, 0));
		sumY += fabsf(MAT_ELEMENT(ctrl.matXYW2Subwheel, i, 1));
	}

	float yPeak = 1.0f - sumY/sumX;

	float factor = getFrictionFactor(pDir);

	return (1.0f - factor)/yPeak * scaleZeroTo90Deg + offsetAtZero;
}

static float getEncoderScalingFromForce(const float* pForce)
{
	const float xNum = ctrlTigga.configModel.xNum;
	const float xDenom = ctrlTigga.configModel.xDenom;
	const float yNum = ctrlTigga.configModel.yNum;
	const float yDenom = ctrlTigga.configModel.yDenom;

	float forceAbs = sqrtf(pForce[0]*pForce[0] + pForce[1]*pForce[1]);

	float xScale = xNum / (forceAbs + xDenom);
	float yScale = yNum / (forceAbs + yDenom);

	float scale = yScale - xScale;
	if(scale < 0.01f)
		return 1.0f;

	return getFrictionFactorInvScaled(pForce, xScale, scale);
}

static void getFrictionCompensationXY(const float* pVel, float* pFrictionForce)
{
	float frictionDir[2] = {0, 0};
	const float trgVelLocalAbs = sqrtf(pVel[0]*pVel[0] + pVel[1]*pVel[1]);
	if(trgVelLocalAbs > 1e-3f)
	{
		frictionDir[0] = pVel[0]/trgVelLocalAbs;
		frictionDir[1] = pVel[1]/trgVelLocalAbs;
	}

	// calculate friction compensation force
	const float coulombFriction = ctrlTigga.configModel.coulombXY;
	const float viscousFriction = ctrlTigga.configModel.viscousXY;

	float frictionFactor = getFrictionFactor(frictionDir);

	pFrictionForce[0] = frictionDir[0] * frictionFactor * (coulombFriction + viscousFriction * fabsf(pVel[0]));
	pFrictionForce[1] = frictionDir[1] * frictionFactor * (coulombFriction + viscousFriction * fabsf(pVel[1]));
}

static float getFrictionCompensationW(float vel)
{
	float frictionDir = 0;
	const float trgVelLocalAbs = fabsf(vel);
	if(trgVelLocalAbs > 1e-3f)
	{
		frictionDir = vel/trgVelLocalAbs;
	}

	// calculate friction compensation force
	const float coulombFriction = ctrlTigga.configModel.coulombW;
	const float viscousFriction = ctrlTigga.configModel.viscousW;

	return frictionDir * (coulombFriction + viscousFriction * fabsf(vel));
}

static float dist2f(const float* pA, const float* pB)
{
	const float diff[2] = {pA[0] - pB[0], pA[1] - pB[1]};
	return sqrtf(diff[0]*diff[0] + diff[1]*diff[1]);
}

static void trajGen(const DriveInput* pDrive, RobotCtrlReference* pRef)
{
	const float dT = CTRL_DELTA_T;

	static float lastTargetPos[2] = {0, 0};
	static float largePosChange = 0;

	switch(pDrive->modeXY)
	{
		case DRIVE_MODE_GLOBAL_POS_ASYNC:
		case DRIVE_MODE_GLOBAL_POS:
		{
			if(dist2f(lastTargetPos, pDrive->pos) > ctrlTigga.configPid.posXY.antiJitterThreshold)
				largePosChange = 1;

			memcpy(lastTargetPos, pDrive->pos, sizeof(float)*2);

			if(pDrive->modeXY == DRIVE_MODE_GLOBAL_POS)
			{
				TrajGeneratorCreateGlobalPosXY(&ctrlTigga.trajGen, pDrive->pos);
			}
			else
			{
				TrajGeneratorCreateGlobalPosXYAsync(&ctrlTigga.trajGen, pDrive->pos, pDrive->primaryDirection);
			}

			const float* trgPos = ctrlTigga.trajGen.trajGlobalPos;
			const float* trgVel = ctrlTigga.trajGen.trajGlobalVel;
			const float* trgAcc = ctrlTigga.trajGen.trajGlobalAcc;

			memcpy(pRef->trajPos, trgPos, sizeof(float)*2);

			CtrlUtilTurnGlobal2Local(trgPos[2], trgVel[0], trgVel[1], pRef->trajVelLocal, pRef->trajVelLocal+1);
			CtrlUtilTurnGlobal2Local(trgPos[2], trgAcc[0], trgAcc[1], pRef->trajAccLocal, pRef->trajAccLocal+1);

			if(dist2f(pRef->trajPos, pDrive->pos) < 0.001f)
				largePosChange = 0;

			if(largePosChange == 0)
			{
				pRef->trajAccLocal[0] = 0.0f;
				pRef->trajAccLocal[1] = 0.0f;
			}

			TrajGeneratorStepXY(&ctrlTigga.trajGen, dT);
		}
		break;
		case DRIVE_MODE_LOCAL_VEL:
		{
			TrajGeneratorCreateLocalVelXY(&ctrlTigga.trajGen, pDrive->localVel);

			memcpy(pRef->trajPos, ctrlTigga.trajGen.trajGlobalPos, sizeof(float)*2);
			memcpy(pRef->trajVelLocal, ctrlTigga.trajGen.trajLocalVel, sizeof(float)*2);
			memcpy(pRef->trajAccLocal, ctrlTigga.trajGen.trajLocalAcc, sizeof(float)*2);

			memcpy(lastTargetPos, pRef->trajPos, sizeof(float)*2);
			largePosChange = 1;

			TrajGeneratorStepXY(&ctrlTigga.trajGen, dT);
		}
		break;
		default:
		{
			TrajGeneratorResetXY(&ctrlTigga.trajGen);
		}
		break;
	}

	static float lastTargetOrient = 0.0f;
	static uint8_t largeOrientChange = 0;

	switch(pDrive->modeW)
	{
		case DRIVE_MODE_GLOBAL_POS:
		{
			if(fabsf(lastTargetOrient - pDrive->pos[2]) > ctrlTigga.configPid.posW.antiJitterThreshold)
				largeOrientChange = 1;

			lastTargetOrient = pDrive->pos[2];

			TrajGeneratorCreateGlobalPosWDynamicAcc(&ctrlTigga.trajGen, pDrive->pos[2], 1.0f, 0.1f);


			pRef->trajPos[2] = ctrlTigga.trajGen.trajGlobalPos[2];
			pRef->trajVelLocal[2] = ctrlTigga.trajGen.trajGlobalVel[2];
			pRef->trajAccLocal[2] = ctrlTigga.trajGen.trajGlobalAcc[2];

			if(fabsf(pRef->trajPos[2] - pDrive->pos[2]) < 0.01f)
				largeOrientChange = 0;

			if(largeOrientChange == 0)
			{
				pRef->trajAccLocal[2] = 0.0f;
			}

			TrajGeneratorStepW(&ctrlTigga.trajGen, dT);
		}
		break;
		case DRIVE_MODE_LOCAL_VEL:
		{
			// Trajectory Update
			TrajGeneratorCreateLocalVelW(&ctrlTigga.trajGen, pDrive->localVel[2]);

			pRef->trajPos[2] = ctrlTigga.trajGen.trajGlobalPos[2];
			pRef->trajVelLocal[2] = ctrlTigga.trajGen.trajLocalVel[2];
			pRef->trajAccLocal[2] = ctrlTigga.trajGen.trajLocalAcc[2];

			lastTargetOrient = pRef->trajPos[2];
			largeOrientChange = 1;

			TrajGeneratorStepW(&ctrlTigga.trajGen, dT);
		}
		break;
		default:
		{
			TrajGeneratorResetW(&ctrlTigga.trajGen);
		}
		break;
	}
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
		const RobotCtrlState* pState, float* pLocalOutForce)
{
	const float dT = CTRL_DELTA_T;

	float trgVelLocal[2];
	float trgAccLocal[2];

	pLocalOutForce[0] = 0;
	pLocalOutForce[1] = 0;

	switch(pDrive->modeXY)
	{
		case DRIVE_MODE_GLOBAL_POS:
		case DRIVE_MODE_GLOBAL_POS_ASYNC:
		case DRIVE_MODE_LOCAL_VEL:
		{
			// use current state to rotate traj output
			CtrlUtilRotate(pReference->trajPos[2]-pState->pos[2], pReference->trajVelLocal[0], pReference->trajVelLocal[1], trgVelLocal);
			CtrlUtilRotate(pReference->trajPos[2]-pState->pos[2], pReference->trajAccLocal[0], pReference->trajAccLocal[1], trgAccLocal);

			// XY position control
			float ctrl[2];
			ctrl[0] = (pReference->trajPos[0] - pState->pos[0]) * ctrlTigga.configPid.posXY.k;
			ctrl[1] = (pReference->trajPos[1] - pState->pos[1]) * ctrlTigga.configPid.posXY.k;
			clamp2Df(ctrl, ctrlTigga.configPid.posXY.max);

			// convert to local pos PID output
			float localPosPidOutput[2];
			CtrlUtilTurnGlobal2Local(pState->pos[2], ctrl[0], ctrl[1], localPosPidOutput, localPosPidOutput+1);

			pLocalOutForce[0] += localPosPidOutput[0];
			pLocalOutForce[1] += localPosPidOutput[1];
		}
		break;
		default:
		{
			return;
		}
	}

	// respect strict velocity limit if present
	// we zero out position controller output, so that velocity controller handles our velocity violation
	float stateVelAbs = sqrtf(pState->vel[0]*pState->vel[0] + pState->vel[1]*pState->vel[1]);
	if(pDrive->limits.strictVelLimit && stateVelAbs > pDrive->limits.velMaxXY)
	{
		pLocalOutForce[0] = 0;
		pLocalOutForce[1] = 0;
	}

	// velocity control
	float velCtrl[2];
	velCtrl[0] = (trgVelLocal[0] - pState->vel[0]) * ctrlTigga.configPid.velXY.k;
	velCtrl[1] = (trgVelLocal[1] - pState->vel[1]) * ctrlTigga.configPid.velXY.k;
	clamp2Df(velCtrl, ctrlTigga.configPid.velXY.max);
	pLocalOutForce[0] += velCtrl[0];
	pLocalOutForce[1] += velCtrl[1];

	// calculate acceleration feedforward force
	float accForce[2];
	accForce[0] = trgAccLocal[0] * botParams.physical.mass;
	accForce[1] = trgAccLocal[1] * botParams.physical.mass;

	pLocalOutForce[0] += accForce[0];
	pLocalOutForce[1] += accForce[1];

	// local out force is complete but does not contain friction compensation
	// calculate next velocity from model
	float nextVel[2];
	nextVel[0] = pState->vel[0] + pLocalOutForce[0]/botParams.physical.mass*dT;
	nextVel[1] = pState->vel[1] + pLocalOutForce[1]/botParams.physical.mass*dT;

	// calculate friction compensation force
	float frictionForce[2];
	getFrictionCompensationXY(nextVel, frictionForce);

	// scale friction compensation force near zero velocity when decelerating
	float forceDotVel;
	arm_dot_prod_f32(pLocalOutForce, nextVel, 2, &forceDotVel);
	if(forceDotVel > 0)
	{
		// accelerating
	}
	else
	{
		// decelerating
		float absVel = sqrtf(pState->vel[0]*pState->vel[0] + pState->vel[1]*pState->vel[1]);
		float scale = MapToRangef32(0.01f, 0.1f, 0.0f, 1.0f, absVel);
		frictionForce[0] *= scale;
		frictionForce[1] *= scale;
	}

	// add friction force
	pLocalOutForce[0] += frictionForce[0];
	pLocalOutForce[1] += frictionForce[1];

	// compensate rotation
	float rot = -pState->vel[2]*dT*0.5f;
	CtrlUtilRotate(rot, pLocalOutForce[0], pLocalOutForce[1], pLocalOutForce);
}

static void controlW(const DriveInput* pDrive, const RobotCtrlReference* pReference,
		const RobotCtrlState* pState, float* pLocalOutForce)
{
	float trgVel = pReference->trajVelLocal[2];
	float trgAcc = pReference->trajAccLocal[2];

	pLocalOutForce[2] = 0;

	switch(pDrive->modeW)
	{
		case DRIVE_MODE_GLOBAL_POS:
		case DRIVE_MODE_GLOBAL_POS_ASYNC:
		case DRIVE_MODE_LOCAL_VEL:
		{
			const float orientError = pReference->trajPos[2] - pState->pos[2];
			pLocalOutForce[2] += clampf(orientError * ctrlTigga.configPid.posW.k, ctrlTigga.configPid.posW.max);
		}
		break;
		default:
		{
			return;
		}
	}

	// W velocity control
	pLocalOutForce[2] += clampf((trgVel - pState->vel[2]) * ctrlTigga.configPid.velW.k, ctrlTigga.configPid.velW.max);

	// calculate acceleration feedforward force
	float accForce = trgAcc * ctrl.momentOfInertia;
	pLocalOutForce[2] += accForce;

	// local out force is complete but does not contain friction compensation
	// calculate next velocity from model
	float nextVel;
	nextVel = pState->vel[2] + pLocalOutForce[2]/ctrl.momentOfInertia*CTRL_DELTA_T;

	// calculate friction compensation force
	float frictionForce = getFrictionCompensationW(nextVel);
	if(nextVel*pLocalOutForce[2] > 0)
	{
		// accelerating
	}
	else
	{
		// decelerating
		float absVel = fabsf(pState->vel[2]);
		float scale = MapToRangef32(0.1f, 0.5f, 0.0f, 1.0f, absVel);
		frictionForce *= scale;
	}

	// add friction force
	pLocalOutForce[2] += frictionForce;

	if(ctrlTigga.configPid.rotationScaling.enable)
	{
		const float startVel = ctrlTigga.configPid.rotationScaling.startVel*1e-3f;
		const float endVel = startVel + ctrlTigga.configPid.rotationScaling.range*1e-3f;

		// reduce output efforts during fast moves, orientation is not that important
		const float xyVel = sqrtf(pState->vel[0]*pState->vel[0] + pState->vel[1]*pState->vel[1]);
		pLocalOutForce[2] *= MapToRangef32(startVel, endVel, 1.0f, 0.1f, xyVel);
	}
}

static void controller(const RobotCtrlState* pState, const SkillOutput* pInput, RobotCtrlOutput* pOutput, RobotCtrlReference* pReference)
{
	(void)pState;

	const DriveInput* pDrive = &pInput->drive;

	// Configure trajectory generator
	ctrlTigga.trajGen.vMaxXY = pDrive->limits.velMaxXY;
	ctrlTigga.trajGen.vMaxW = pDrive->limits.velMaxW;
	ctrlTigga.trajGen.aMaxXY = pDrive->limits.accMaxXY;
	ctrlTigga.trajGen.aMaxW = pDrive->limits.accMaxW;

	// create predicted state for control
	RobotCtrlState ctrlState;
	if(ctrlTigga.visionOnline)
		memcpy(&ctrlState.pos, ctrlTigga.mechState, sizeof(float)*8);
	else
		memcpy(&ctrlState.pos, ctrlTigga.encState, sizeof(float)*8);

	ctrlState.acc[2] = 0;

	// create local drive input with multi-turn orientation
	DriveInput drive;
	memcpy(&drive, pDrive, sizeof(DriveInput));
	drive.pos[2] = ctrlState.pos[2] + AngleNormalize(drive.pos[2] - AngleNormalize(ctrlState.pos[2]));

	// set current filter state on traj generator
	float globalVel[3] = {0, 0, ctrlState.vel[2]};
	CtrlUtilTurnLocal2Global(ctrlState.pos[2], ctrlState.vel[0], ctrlState.vel[1], globalVel, globalVel+1);
	TrajGeneratorSetState(&ctrlTigga.trajGen, ctrlState.pos, globalVel);

	// run trajectory generator
	trajGen(&drive, pReference);

	// call controllers for XY and W
	float outForce[3] = {0, 0, 0};

	controlXY(&drive, pReference, &ctrlState, outForce);
	controlW(&drive, pReference, &ctrlState, outForce);

	DelayElementSetLast(&ctrlTigga.delayCtrlOut, outForce);
	DelayElementTick(&ctrlTigga.delayCtrlOut);

	if(ctrlTigga.visionOnline)
	{
		outForce[0] *= 1.0f/ctrlTigga.configModel.efficiencyXY;
		outForce[1] *= 1.0f/ctrlTigga.configModel.efficiencyXY;
		outForce[2] *= 1.0f/ctrlTigga.configModel.efficiencyW;

		CtrlUtilLocalForceToMotorTorque(outForce, pOutput->motorTorque);
		pOutput->ctrlMode = MOT_CTRL_TORQUE;

		memcpy(ctrlTigga.virtualVelOutput, ctrlState.vel, sizeof(float)*3);
	}
	else
	{
		float frictionForce[3];
		getFrictionCompensationXY(ctrlState.vel, frictionForce);
		frictionForce[2] = getFrictionCompensationW(ctrlState.vel[2]);

		float ctrlAcc[3];
		ctrlAcc[0] = (outForce[0] - frictionForce[0])/botParams.physical.mass;
		ctrlAcc[1] = (outForce[1] - frictionForce[1])/botParams.physical.mass;
		ctrlAcc[2] = (outForce[2] - frictionForce[2])/ctrl.momentOfInertia;

		ctrlTigga.virtualVelOutput[0] += ctrlAcc[0] * CTRL_DELTA_T;
		ctrlTigga.virtualVelOutput[1] += ctrlAcc[1] * CTRL_DELTA_T;
		ctrlTigga.virtualVelOutput[2] += ctrlAcc[2] * CTRL_DELTA_T;

		CtrlUtilLocalVelToMotorVel(ctrlTigga.virtualVelOutput, pOutput->motorVel, 1);
		pOutput->ctrlMode = MOT_CTRL_VELOCITY;
	}

	if(pDrive->modeXY == DRIVE_MODE_LOCAL_FORCE || pDrive->modeW == DRIVE_MODE_LOCAL_FORCE)
	{
		CtrlUtilLocalForceToMotorTorque(drive.localForce, pOutput->motorTorque);
		pOutput->ctrlMode = MOT_CTRL_TORQUE;
	}

	if(pDrive->modeXY == DRIVE_MODE_WHEEL_VEL || pDrive->modeW == DRIVE_MODE_WHEEL_VEL)
	{
		for(uint8_t i = 0; i < 4; i++)
			pOutput->motorVel[i] = pDrive->wheelVel[i]*CTRL_WHEEL_TO_MOTOR_RATIO;

		pOutput->ctrlMode = MOT_CTRL_VELOCITY;
	}

	if(pDrive->modeXY == DRIVE_MODE_OFF && pDrive->modeW == DRIVE_MODE_OFF)
	{
		memset(pOutput->motorVel, 0, sizeof(float)*4);
		memset(ctrlTigga.virtualVelOutput, 0, sizeof(float)*3);

		pOutput->ctrlMode = MOT_CTRL_OFF;
	}

	if(chVTTimeElapsedSinceX(ctrlTigga.lastInitTime) < MS2ST(500))
	{
		pOutput->ctrlMode = MOT_CTRL_OFF;
	}
	else
	{
		ctrlTigga.lastInitTime = chVTGetSystemTimeX() - MS2ST(1000);
	}

	if(pOutput->ctrlMode == MOT_CTRL_OFF)
	{
		++ctrlTigga.motorOffCounter;
	}
	else
	{
		ctrlTigga.motorOffCounter = 0;
	}

	if(ctrlTigga.motorOffCounter > 5000)
	{
		ctrlTigga.motorOffCounter = 0;

		float subtract = ctrlTigga.vision.turns*2.0f*M_PI;
		ctrlTigga.encState[2] -= subtract;
		ctrlTigga.mechState[2] -= subtract;
		ctrlTigga.ekf.x.pData[2] -= subtract;

		for(uint32_t i = 0; i < ctrlTigga.delayIns.numElements; i++)
		{
			float* pIns = (float*)DelayElementGet(&ctrlTigga.delayIns, i);
			pIns[2] -= subtract;
		}

		ctrlTigga.vision.turns = 0;
	}
}

static void mergeStateFilters(const float* pLateVis)
{
	if(ctrlTigga.visionOnline)
	{
		++ctrlTigga.noVisionCounter;

		if(pLateVis)
			ctrlTigga.noVisionCounter = 0;

		if(ctrlTigga.noVisionCounter > ctrlTigga.configFilter.visionTimeoutMs)
		{
			ctrlTigga.visionOnline = 0;

			memcpy(ctrlTigga.encState, ctrlTigga.mechState, sizeof(float)*5);
		}
	}
	else
	{
		if(pLateVis)
		{
			ctrlTigga.noVisionCounter = 0;
			ctrlTigga.visionOnline = 1;

			float newState[5];
			memcpy(&newState[0], pLateVis, sizeof(float)*3);
			memcpy(&newState[3], &ctrlTigga.encState[3], sizeof(float)*2);

			memcpy(ctrlTigga.ekf.x.pData, newState, sizeof(float)*5);

			memcpy(ctrlTigga.mechState, newState, sizeof(float)*5);

			for(uint16_t i = 0; i < ctrlTigga.delayIns.numElements; i++)
			{
				float* pIns = (float*)DelayElementGet(&ctrlTigga.delayIns, i);
				memcpy(pIns, newState, sizeof(float)*5);
			}

			float globalVel[3];
			CtrlUtilTurnLocal2Global(pLateVis[2], newState[3], newState[4], globalVel, globalVel+1);
			globalVel[2] = ctrlTigga.encState[5];

			TrajGeneratorSetState(&ctrlTigga.trajGen, newState, globalVel);
			TrajGeneratorReset(&ctrlTigga.trajGen);
		}
	}
}

static void sensorFusion(const RobotSensors* pSensors, const RobotCtrlOutput* pOutput, RobotCtrlState* pState)
{
	// GYRO + ACCELEROMETER
	float gyrAcc[3] = {pSensors->gyr.rotVel[2]-ctrlTigga.bias[0], pSensors->acc.linAcc[0]-ctrlTigga.bias[1], pSensors->acc.linAcc[1]-ctrlTigga.bias[2]};
	DelayElementSetLast(&ctrlTigga.delayGyrAcc, gyrAcc);
	const float* pLateGyrAcc = (const float*)DelayElementGetNow(&ctrlTigga.delayGyrAcc);
	DelayElementTick(&ctrlTigga.delayGyrAcc);

	// ENCODERS
	float encVelLocal[3];
	// get local velocity from encoders
	CtrlUtilMotorVelToLocalVel(pSensors->enc.vel, encVelLocal);

	// correct local velocity
	if(pOutput->ctrlMode == MOT_CTRL_TORQUE || pOutput->ctrlMode == MOT_CTRL_VEL_TORQUE)
	{
		float motForceOutLocal[3];

		float motTorque[4];
		for(uint8_t i = 0; i < 4; i++)
			motTorque[i] = (pSensors->motorVol.vol[i] - CTRL_MOTOR_KE*pSensors->enc.vel[i]) * CTRL_MOTOR_KM/CTRL_MOTOR_R;

		// get local force from previous output
		CtrlUtilMotorTorqueToLocalForce(motTorque, motForceOutLocal);

		motForceOutLocal[0] *= ctrlTigga.configModel.efficiencyXY;
		motForceOutLocal[1] *= ctrlTigga.configModel.efficiencyXY;

		float encScale = getEncoderScalingFromForce(motForceOutLocal);

		encVelLocal[0] *= encScale;
		encVelLocal[1] *= encScale;
	}

	// step encoder state
	float encVelGlobal[2];
	CtrlUtilTurnLocal2Global(ctrlTigga.encState[2], encVelLocal[0], encVelLocal[1], encVelGlobal, encVelGlobal+1);
	ctrlTigga.encState[0] += encVelGlobal[0] * CTRL_DELTA_T;
	ctrlTigga.encState[1] += encVelGlobal[1] * CTRL_DELTA_T;
	ctrlTigga.encState[2] += encVelLocal[2] * CTRL_DELTA_T;
	memcpy(&ctrlTigga.encState[3], encVelLocal, sizeof(float)*2);
	ctrlTigga.encState[5] = encVelLocal[2];
	memset(&ctrlTigga.encState[6], 0, sizeof(float)*2);


	// VISION
	const float* pLateVis = visionProc(pSensors->vision.updated, pSensors->vision.delay, pSensors->vision.pos);

	// BIAS updates (gyr/acc)
	float encVelXYAbs = sqrtf(encVelLocal[0]*encVelLocal[0] + encVelLocal[1]*encVelLocal[1]);
	if(encVelXYAbs < 1e-3f && fabsf(encVelLocal[2]) < 1e-3f && ctrlTigga.visionOnline)
	{
		EMAFilterUpdate(&ctrlTigga.emaBias[0], pSensors->gyr.rotVel[2]);
		EMAFilterUpdate(&ctrlTigga.emaBias[1], pSensors->acc.linAcc[0]);
		EMAFilterUpdate(&ctrlTigga.emaBias[2], pSensors->acc.linAcc[1]);

		ctrlTigga.bias[0] = ctrlTigga.emaBias[0].value;
		ctrlTigga.bias[1] = ctrlTigga.emaBias[1].value;
		ctrlTigga.bias[2] = ctrlTigga.emaBias[2].value;
	}

	// INERTIAL NAVIGATION SYSTEM (INS)
	arm_matrix_instance_f32 matInsState = {5, 1, (float[5]){}};
	memcpy(matInsState.pData, DelayElementGetPrevious(&ctrlTigga.delayIns), sizeof(float)*5);
	const arm_matrix_instance_f32 matInput = {3, 1, gyrAcc};

	if(ctrlTigga.visionOnline)
		(*ctrlTigga.ekf.pState)(&matInsState, &matInput);

	DelayElementSetLast(&ctrlTigga.delayIns, matInsState.pData);
	const float* pLateIns = (const float*)DelayElementGetNow(&ctrlTigga.delayIns);

	DelayElementTick(&ctrlTigga.delayIns);

	// EKF
	if(ctrlTigga.visionOnline)
	{
		memcpy(ctrlTigga.ekf.u.pData, pLateGyrAcc, sizeof(float)*3);
		EKFPredict(&ctrlTigga.ekf);

		if(pLateVis)
		{
			memcpy(ctrlTigga.ekf.z.pData, pLateVis, sizeof(float)*3);
			EKFUpdate(&ctrlTigga.ekf);
		}
	}

	if(arm_mat_is_nan_f32(&ctrlTigga.ekf.x))
	{
		initEKF();

		memset(ctrlTigga.delayInsData, 0, ctrlTigga.delayIns.elementSize*ctrlTigga.delayIns.numElementsMax);

		ctrlTigga.visionOnline = 0;
	}

	// TRACKING (INS follows EKF)
	float gain = CTRL_DELTA_T/(ctrlTigga.configFilter.trackingCoeff*ctrlTigga.delayIns.numElements*0.001f);
	float corr[5];
	for(uint8_t i = 0; i < 5; i++)
		corr[i] = (ctrlTigga.ekf.x.pData[i]-pLateIns[i])*gain;

	for(uint16_t i = 0; i < ctrlTigga.delayIns.numElements; i++)
	{
		float* pIns = (float*)DelayElementGet(&ctrlTigga.delayIns, i);
		for(uint16_t j = 0; j < 5; j++)
			pIns[j] += corr[j];
	}

	// GATHER STATES
	memcpy(ctrlTigga.mechState, DelayElementGetPrevious(&ctrlTigga.delayIns), sizeof(float)*5);
	ctrlTigga.mechState[5] = gyrAcc[0];
	ctrlTigga.mechState[6] = gyrAcc[1];
	ctrlTigga.mechState[7] = gyrAcc[2];

	// compensate delay to real state
	float* pMechPos = ctrlTigga.mechState;
	float* pMechVel = ctrlTigga.mechState+3;

	for(uint32_t i = 0; i < ctrlTigga.delayCtrlOut.numElements; i++)
	{
		const float* pCtrlOut = (const float*)DelayElementGet(&ctrlTigga.delayCtrlOut, i);

		float frictionForce[3];
		getFrictionCompensationXY(pMechVel, frictionForce);
		frictionForce[2] = getFrictionCompensationW(pMechVel[2]);

		float ctrlAcc[3];
		ctrlAcc[0] = (pCtrlOut[0] - frictionForce[0])/botParams.physical.mass;
		ctrlAcc[1] = (pCtrlOut[1] - frictionForce[1])/botParams.physical.mass;
		ctrlAcc[2] = (pCtrlOut[2] - frictionForce[2])/ctrl.momentOfInertia;

		pMechPos[0] += 0.5f * ctrlAcc[0] * CTRL_DELTA_T * CTRL_DELTA_T + pMechVel[0] * CTRL_DELTA_T;
		pMechPos[1] += 0.5f * ctrlAcc[1] * CTRL_DELTA_T * CTRL_DELTA_T + pMechVel[1] * CTRL_DELTA_T;
		pMechPos[2] += 0.5f * ctrlAcc[2] * CTRL_DELTA_T * CTRL_DELTA_T + pMechVel[2] * CTRL_DELTA_T;
		pMechVel[0] += ctrlAcc[0] * CTRL_DELTA_T;
		pMechVel[1] += ctrlAcc[1] * CTRL_DELTA_T;
		pMechVel[2] += ctrlAcc[2] * CTRL_DELTA_T;
	}

	// FILL GLOBAL STATE
	mergeStateFilters(pLateVis);


	const float* pStateToUse;
	if(ctrlTigga.visionOnline)
		pStateToUse = ctrlTigga.mechState;
	else
		pStateToUse = ctrlTigga.encState;

	memcpy(pState->pos, pStateToUse, sizeof(float)*3);
	pState->pos[2] = AngleNormalize(pStateToUse[2]);

	float globalVel[3] = {0, 0, pStateToUse[5]};
	CtrlUtilTurnLocal2Global(pStateToUse[2], pStateToUse[3], pStateToUse[4], globalVel, globalVel+1);
	memcpy(pState->vel, globalVel, sizeof(float)*3);

	float globalAcc[2];
	CtrlUtilTurnLocal2Global(pStateToUse[2], pStateToUse[6], pStateToUse[7], globalAcc, globalAcc+1);
	pState->acc[0] = globalAcc[0];
	pState->acc[1] = globalAcc[1];
	pState->acc[2] = 0;	// can't estimate that with this filter

	pState->posUpdated = 1;
	pState->velUpdated = 1;
	pState->accUpdated = 1;

	//mag
	float hardIronCompX = pSensors->mag.strength[0] - ctrlTigga.configCompass.hardIronX;
	float hardIronCompY = pSensors->mag.strength[1] - ctrlTigga.configCompass.hardIronY;
	float softIronCompX = (ctrlTigga.configCompass.softIronCos*hardIronCompX + ctrlTigga.configCompass.softIronSin*hardIronCompY) * ctrlTigga.configCompass.softEllipseEcc;
	float softIronCompY = ctrlTigga.configCompass.softIronCos*hardIronCompY - ctrlTigga.configCompass.softIronSin*hardIronCompX;
	pState->magZ = AngleNormalize(arm_atan2_f32(softIronCompX, softIronCompY) - ctrlTigga.configCompass.fieldOffset);
}

void CtrlTiggaInitCfg()
{
	ctrlTigga.pConfigFileFilter = ConfigOpenOrCreate(&configFileDescFilter, &ctrlTigga.configFilter, sizeof(CtrlTiggaConfigFilter), &configUpdateCallback, 0);
	ctrlTigga.pConfigFilePid = ConfigOpenOrCreate(&configFileDescPid, &ctrlTigga.configPid, sizeof(CtrlTiggaConfigPID), &configUpdateCallback, 0);
	ctrlTigga.pConfigFileModel = ConfigOpenOrCreate(&configFileDescModel, &ctrlTigga.configModel, sizeof(CtrlTiggaConfigModel), &configUpdateCallback, 0);
	ctrlTigga.pConfigFileCompass = ConfigOpenOrCreate(&configFileDescCompass, &ctrlTigga.configCompass, sizeof(CtrlTiggaConfigCompass), &configUpdateCallback, 0);
}

static inline const float* visionProc(uint8_t updated, int32_t tDelayUs, const float* pVisPos)
{
	float* pRet = 0;

	// store maximum encountered vision delay
	if(tDelayUs > ctrlTigga.maxVisionDelay)
		ctrlTigga.maxVisionDelay = tDelayUs;

	// check if a sample is in the buffer for the current time
	CtrlTiggaVisionEntry* pEntryNow = (CtrlTiggaVisionEntry*)DelayElementGetNow(&ctrlTigga.delayVision);
	if(pEntryNow->state == 1)
	{
		pRet = pEntryNow->pos;
	}

	// slot consumed
	pEntryNow->state = 0;

	// advance delay pointer
	DelayElementTick(&ctrlTigga.delayVision);

	if(updated)
	{
		tDelayUs += ((uint32_t)ctrlTigga.configFilter.visCaptureDelay)*1000;

		// calculate vision buffer insertion point
		int32_t visTime = ctrlTigga.delayVision.numElements*CTRL_DELTA_T_IN_US;
		int32_t insertPos = (visTime-tDelayUs+CTRL_DELTA_T_IN_US/2)/CTRL_DELTA_T_IN_US - 1;
		if(insertPos < 0)
		{
			// this sample is too late to be inserted at the correct location, log the error...
			++ctrlTigga.numLateVisionMeasurements;

			// ... and use it anyway with adjusted insertion pos. That's better than to discard it completely.
			insertPos = 0;
		}

		const float* pInsStateAtInsert = (const float*)DelayElementGet(&ctrlTigga.delayIns, insertPos);

		// validate sample, outlier rejection
		uint32_t measTime = SysTimeUSec()-tDelayUs;
		float visionDt = (measTime-ctrlTigga.lastVisionTime)*1e-6f;

		float searchRadiusXY = 100.0f*ctrlTigga.configFilter.visNoiseXY + ctrlTigga.configFilter.outlierMaxVelXY*visionDt;
		float searchRadiusW = 2.0f*ctrlTigga.configFilter.visNoiseW + ctrlTigga.configFilter.outlierMaxVelW*visionDt;

		float diffX = pVisPos[0] - pInsStateAtInsert[0];
		float diffY = pVisPos[1] - pInsStateAtInsert[1];
		float diffXY = sqrtf(diffX*diffX+diffY*diffY);
		float diffW = AngleNormalize(pVisPos[2] - AngleNormalize(pInsStateAtInsert[2]));

		if(diffXY > searchRadiusXY || fabsf(diffW) > searchRadiusW)
		{
			// this is an invalid sample, vel is impossible
			return pRet;
		}

		ctrlTigga.lastVisionTime = measTime;

		// multi-turn angle correction
		if(pVisPos[2] < -2.0f && ctrlTigga.vision.lastVisionOrient > 2.0f)
			++ctrlTigga.vision.turns;

		if(pVisPos[2] > 2.0f && ctrlTigga.vision.lastVisionOrient < -2.0f)
			--ctrlTigga.vision.turns;

		ctrlTigga.vision.lastVisionOrient = pVisPos[2];

		float orient = pVisPos[2] + ctrlTigga.vision.turns*2.0f*M_PI;

		// store position
		CtrlTiggaVisionEntry newEntry;
		newEntry.pos[0] = pVisPos[0];
		newEntry.pos[1] = pVisPos[1];
		newEntry.pos[2] = orient;
		newEntry.state = 1;

		DelayElementSet(&ctrlTigga.delayVision, insertPos, &newEntry);
	}

	return pRet;
}

static void ekfStateFunc(arm_matrix_instance_f32* pX, const arm_matrix_instance_f32* pU)
{
	const float dt = 0.001f;

	float gyr_w = MAT_ELEMENT(*pU, 0, 0);
	float acc_x = MAT_ELEMENT(*pU, 1, 0);
	float acc_y = MAT_ELEMENT(*pU, 2, 0);

	float p_x = MAT_ELEMENT(*pX, 0, 0);
	float p_y = MAT_ELEMENT(*pX, 1, 0);
	float p_w = MAT_ELEMENT(*pX, 2, 0);
	float v_x = MAT_ELEMENT(*pX, 3, 0);
	float v_y = MAT_ELEMENT(*pX, 4, 0);

	float v_w = gyr_w;
	float a = -M_PI_2 + p_w + dt*v_w*0.5f;
	float a_x = acc_x + v_y*v_w;
	float a_y = acc_y - v_x*v_w;

	float px1 = p_x + (arm_cos_f32(a)*v_x-arm_sin_f32(a)*v_y)*dt + (arm_cos_f32(a)*a_x-arm_sin_f32(a)*a_y)*0.5f*dt*dt;
	float py1 = p_y + (arm_sin_f32(a)*v_x+arm_cos_f32(a)*v_y)*dt + (arm_sin_f32(a)*a_x+arm_cos_f32(a)*a_y)*0.5f*dt*dt;
	float vx1 = v_x + a_x*dt;
	float vy1 = v_y + a_y*dt;
	float pw1 = p_w + v_w*dt;

	MAT_ELEMENT(*pX, 0, 0) = px1;
	MAT_ELEMENT(*pX, 1, 0) = py1;
	MAT_ELEMENT(*pX, 2, 0) = pw1;
	MAT_ELEMENT(*pX, 3, 0) = vx1;
	MAT_ELEMENT(*pX, 4, 0) = vy1;
}

static void ekfStateJacobianFunc(const arm_matrix_instance_f32* pX, const arm_matrix_instance_f32* pU, arm_matrix_instance_f32* pF)
{
	float gyr_w = MAT_ELEMENT(*pU, 0, 0);
	float acc_x = MAT_ELEMENT(*pU, 1, 0);
	float acc_y = MAT_ELEMENT(*pU, 2, 0);

	float p_w = MAT_ELEMENT(*pX, 2, 0);
	float v_x = MAT_ELEMENT(*pX, 3, 0);
	float v_y = MAT_ELEMENT(*pX, 4, 0);

    float SF1 = gyr_w/2000.0f + p_w - M_PI_2;
	float SF2 = arm_cos_f32(SF1)/1000.0f + (gyr_w * arm_sin_f32(SF1))/2000000.0f;
	float SF3 = (gyr_w * arm_cos_f32(SF1))/2000000.0f;
	float SF4 = arm_sin_f32(SF1);
	float SF5 = acc_y - gyr_w * v_x;
	float SF6 = arm_cos_f32(SF1);
	float SF7 = gyr_w/1000.0f;

	arm_mat_identity_f32(pF);

	MAT_ELEMENT(*pF, 0, 2) = - (SF5*SF6)/2000000.0f - (v_x*SF4)/1000.0f - (v_y*SF6)/1000.0f - (SF4*(acc_x + gyr_w*v_y))/2000000.0f;
	MAT_ELEMENT(*pF, 0, 3) = SF2;
	MAT_ELEMENT(*pF, 0, 4) = SF3 - SF4/1000.0f;

	MAT_ELEMENT(*pF, 1, 2) = (v_x*SF6)/1000.0f - (SF4*SF5)/2000000.0f - (v_y*SF4)/1000.0f + (SF6*(acc_x + gyr_w*v_y))/2000000.0f;
	MAT_ELEMENT(*pF, 1, 3) = SF4/1000.0f - SF3;
	MAT_ELEMENT(*pF, 1, 4) = SF2;

	MAT_ELEMENT(*pF, 3, 4) = SF7;

	MAT_ELEMENT(*pF, 4, 3) = -SF7;
}

static void ekfMeasFunc(const arm_matrix_instance_f32* pX, arm_matrix_instance_f32* pY)
{
	memcpy(pY->pData, pX->pData, sizeof(float)*3);
}

static void ekfMeasJacobianFunc(const arm_matrix_instance_f32* pX, arm_matrix_instance_f32* pH)
{
	(void)pX;

	arm_mat_zero_f32(pH);
	MAT_ELEMENT(*pH, 0, 0) = 1.0f;
	MAT_ELEMENT(*pH, 1, 1) = 1.0f;
	MAT_ELEMENT(*pH, 2, 2) = 1.0f;
}
