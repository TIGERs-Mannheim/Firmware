/*
 * fusion_ekf.c
 *
 *  Created on: 03.08.2020
 *      Author: AndreR
 */

#include "fusion_ekf.h"
#include "struct_ids.h"
#include "ctrl.h"
#include "util/arm_mat_util_f32.h"
#include "util/angle_math.h"
#include "util/sys_time.h"
#include <math.h>

static void ekfStateFunc(arm_matrix_instance_f32* pX, const arm_matrix_instance_f32* pU);
static void ekfStateJacobianFunc(const arm_matrix_instance_f32* pX, const arm_matrix_instance_f32* pU, arm_matrix_instance_f32* pF);
static void ekfMeasFunc(const arm_matrix_instance_f32* pX, arm_matrix_instance_f32* pY);
static void ekfMeasJacobianFunc(const arm_matrix_instance_f32* pX, arm_matrix_instance_f32* pH);
static void initEKF();
static void loadNoiseCovariancesFromConfig();

static uint8_t isVisionSampleValid(const float* pVisPos, const float* pInsPos, int32_t tDelayUs);
static float visionMultiTurnCorrection(float visionOrientation);

FusionEKF fusionEKF;

ConfigFileDesc fusionEKFConfigDesc =
	{ 0, 0, "", 17, (ElementDesc[]) {
		{ FLOAT, "noise_pos_xy", "m", "noise/pos/xy" },
		{ FLOAT, "noise_pos_w", "rad", "noise/pos/w" },
		{ FLOAT, "noise_vel_xy", "m/s", "noise/vel/xy" },
		{ FLOAT, "noise_vis_xy", "m", "noise/vis/xy" },
		{ FLOAT, "noise_vis_w", "rad", "noise/vis/w" },
		{ FLOAT, "outlier_max_vel_xy", "m/s", "outlier/vel/xy" },
		{ FLOAT, "outlier_max_vel_w", "rad/s", "outlier/vel/w" },
		{ FLOAT, "tracking_coeff", "", "tracking/coeff" },
		{ UINT8, "vis_capture_delay", "ms", "vision/capture/delay" },
		{ UINT8, "fusion_horizon", "ms", "fusion/horizon" },
		{ UINT16, "vision_timeout", "ms", "vision/timeout" },
		{ FLOAT, "ema_accel_t", "s", "ema/accel/T" },
		{ FLOAT, "ir_noise_pos", "mm", "ir/noise/pos" },
		{ FLOAT, "ir_noise_ir_x", "mm", "ir/noise/ir/x" },
		{ FLOAT, "ir_noise_ir_y", "mm", "ir/noise/ir/y" },
		{ FLOAT, "ir_outlier_max_vel", "mm/s", "ir/outlier/vel" },
		{ UINT16, "ir_timeout", "ms", "ir/timeout" },
	} };

void FusionEKFConfigUpdateCallback(uint16_t cfgId)
{
	if(cfgId >= SID_CFG_FUSION_EKF_V2016 && cfgId <= SID_CFG_FUSION_EKF_V_LAST)
	{
		loadNoiseCovariancesFromConfig();

		LagElementPT1Init(&fusionEKF.lagAccel[0], 1.0f, fusionEKF.pConfig->emaAccelT, CTRL_DELTA_T);
		LagElementPT1Init(&fusionEKF.lagAccel[1], 1.0f, fusionEKF.pConfig->emaAccelT, CTRL_DELTA_T);
	}
}

void FusionEKFInit(FusionEKFConfig* pConfigEkf, ModelEncConfig* pConfigModel)
{
	fusionEKF.pConfig = pConfigEkf;
	ModelEncInit(&fusionEKF.modelEnc, pConfigModel, CTRL_DELTA_T);
	initEKF();

	LagElementPT1Init(&fusionEKF.lagAccel[0], 1.0f, fusionEKF.pConfig->emaAccelT, CTRL_DELTA_T);
	LagElementPT1Init(&fusionEKF.lagAccel[1], 1.0f, fusionEKF.pConfig->emaAccelT, CTRL_DELTA_T);

	float ballInitPos[2] = { 0 };
	float ballMeasEror[] = { fusionEKF.pConfig->irMeasNoiseX, fusionEKF.pConfig->irMeasNoiseY };
	TrackingFilter2DInit(&fusionEKF.ballFilter, CTRL_DELTA_T, ballInitPos, 1e-3f, fusionEKF.pConfig->irPosNoise, ballMeasEror);
}

void FusionEKFUpdate(const RobotSensors* pSensors, const RobotCtrlOutput* pOutput, RobotCtrlState* pState)
{
	(void)pOutput;

	FusionEKFTimeSlot* pSlotNow = &fusionEKF.timeSlots[fusionEKF.timeSlotNow];
	FusionEKFTimeSlot* pSlotPrev = &fusionEKF.timeSlots[(fusionEKF.timeSlotNow-1)%FUSION_EKF_MAX_DELAY];
	FusionEKFTimeSlot* pSlotFusionHorizon = &fusionEKF.timeSlots[(fusionEKF.timeSlotNow-fusionEKF.pConfig->fusionHorizon)%FUSION_EKF_MAX_DELAY];

	// GYRO + ACCELEROMETER
	pSlotNow->meas.gyrAcc[0] = pSensors->gyr.rotVel[2];
	pSlotNow->meas.gyrAcc[1] = LagElementPT1Process(&fusionEKF.lagAccel[0], pSensors->acc.linAcc[0]);
	pSlotNow->meas.gyrAcc[2] = LagElementPT1Process(&fusionEKF.lagAccel[1], pSensors->acc.linAcc[1]);
	pSlotNow->meas.posUpdated = 0;

	// VISION
	if(pSensors->vision.updated)
	{
		int32_t tDelayUs =  pSensors->vision.delay + ((int32_t)fusionEKF.pConfig->visCaptureDelay)*1000;
		int32_t insertPos = (tDelayUs+CTRL_DELTA_T_IN_US/2)/CTRL_DELTA_T_IN_US;

		if(insertPos > fusionEKF.pConfig->fusionHorizon)
		{
			// this sample is too late to be inserted at the correct location, log the error...
			fusionEKF.vision.numLateMeasurements++;

			// ... and use it anyway with adjusted insertion pos. That's better than to discard it completely.
			insertPos = fusionEKF.pConfig->fusionHorizon;
		}

		FusionEKFTimeSlot* pSlotVision = &fusionEKF.timeSlots[(fusionEKF.timeSlotNow-insertPos)%FUSION_EKF_MAX_DELAY];

		if(isVisionSampleValid(pSensors->vision.pos, pSlotVision->insState, tDelayUs))
		{
			pSlotVision->meas.pos[0] = pSensors->vision.pos[0];
			pSlotVision->meas.pos[1] = pSensors->vision.pos[1];
			pSlotVision->meas.pos[2] = visionMultiTurnCorrection(pSensors->vision.pos[2]);
			pSlotVision->meas.posUpdated = 1;

			if(!fusionEKF.vision.online)
			{
				fusionEKF.vision.online = 1;

				// make sure EKF jumps immediately to new position
				FusionEKFSetState(pSlotVision->meas.pos, fusionEKF.modelEnc.state);
			}
		}
	}

	if((SysTimeUSec() - fusionEKF.vision.timeLastValidSample) > fusionEKF.pConfig->visionTimeoutMs*1000)
	{
		// make sure timeLastValidSample does not roll-over
		if((SysTimeUSec() - fusionEKF.vision.timeLastValidSample) > fusionEKF.pConfig->visionTimeoutMs*100000)
			fusionEKF.vision.timeLastValidSample = SysTimeUSec() - fusionEKF.pConfig->visionTimeoutMs*100000;

		if(fusionEKF.vision.online)
		{
			fusionEKF.vision.online = 0;

			// start encoder integration from current INS estimate
			memcpy(fusionEKF.encGyrPos, pSlotNow->insState, sizeof(float)*3);
		}
	}

	// ENCODERS
	float encVelLocal[3];
	CtrlUtilMotorVelToLocalVel(pSensors->enc.vel, encVelLocal);

	// use encoders for rotation only at standstill
	if(fabsf(encVelLocal[2]) > 0.01f)
		encVelLocal[2] = pSensors->gyr.rotVel[2];

	// Use encoder velocity model for X and Y
	ModelEncStep(&fusionEKF.modelEnc, encVelLocal);
	memcpy(encVelLocal, fusionEKF.modelEnc.state, sizeof(float)*2);

	// integrate velocity in global frame
	float encVelGlobal[2];
	CtrlUtilTurnLocal2Global(fusionEKF.encGyrPos[2], encVelLocal[0], encVelLocal[1], encVelGlobal, encVelGlobal+1);
	fusionEKF.encGyrPos[0] += encVelGlobal[0] * CTRL_DELTA_T;
	fusionEKF.encGyrPos[1] += encVelGlobal[1] * CTRL_DELTA_T;
	fusionEKF.encGyrPos[2] += encVelLocal[2] * CTRL_DELTA_T;

	if(!fusionEKF.vision.online)
	{
		memcpy(pSlotNow->meas.pos, fusionEKF.encGyrPos, sizeof(float)*3);
		pSlotNow->meas.posUpdated = 1;
	}

	// INERTIAL NAVIGATION SYSTEM (INS)
	arm_matrix_instance_f32 matInsState = {5, 1, (float[5]){}};
	memcpy(matInsState.pData, pSlotPrev->insState, sizeof(float)*5);
	const arm_matrix_instance_f32 matInput = {3, 1, pSlotNow->meas.gyrAcc};

	(*fusionEKF.ekf.pState)(&matInsState, &matInput);

	memcpy(pSlotNow->insState, matInsState.pData, sizeof(float)*5);

	// EKF
	memcpy(fusionEKF.ekf.u.pData, pSlotFusionHorizon->meas.gyrAcc, sizeof(float)*3);
	EKFPredict(&fusionEKF.ekf);

	if(pSlotFusionHorizon->meas.posUpdated)
	{
		memcpy(fusionEKF.ekf.z.pData, pSlotFusionHorizon->meas.pos, sizeof(float)*3);
		EKFUpdate(&fusionEKF.ekf);
	}

	if(arm_mat_is_nan_f32(&fusionEKF.ekf.x))
	{
		initEKF();
		memset(fusionEKF.timeSlots, 0, sizeof(fusionEKF.timeSlots));
		memset(fusionEKF.encGyrPos, 0, sizeof(fusionEKF.encGyrPos));
		fusionEKF.vision.online = 0;
	}

	// TRACKING (INS follows EKF)
	float gain = CTRL_DELTA_T/(fusionEKF.pConfig->trackingCoeff*fusionEKF.pConfig->fusionHorizon*0.001f);
	float corr[5];
	for(uint8_t i = 0; i < 5; i++)
		corr[i] = (fusionEKF.ekf.x.pData[i] - pSlotFusionHorizon->insState[i])*gain;

	for(uint16_t i = 0; i < fusionEKF.pConfig->fusionHorizon; i++)
	{
		float* pIns = fusionEKF.timeSlots[(fusionEKF.timeSlotNow-i)%FUSION_EKF_MAX_DELAY].insState;
		for(uint16_t j = 0; j < 5; j++)
			pIns[j] += corr[j];
	}

	// FILL GLOBAL STATE
	const float* pStateToUse = pSlotNow->insState;

	pState->pos[0] = pStateToUse[0];
	pState->pos[1] = pStateToUse[1];
	pState->pos[2] = pStateToUse[2];

	CtrlUtilTurnLocal2Global(pStateToUse[2], pStateToUse[3], pStateToUse[4], &pState->vel[0], &pState->vel[1]);
	pState->vel[2] = pSlotNow->meas.gyrAcc[0];

	CtrlUtilTurnLocal2Global(pStateToUse[2], pSlotNow->meas.gyrAcc[1], pSlotNow->meas.gyrAcc[2], &pState->acc[0], &pState->acc[1]);
	pState->acc[2] = 0;	// can't estimate that with this filter

	pState->posUpdated = 1;
	pState->velUpdated = 1;
	pState->accUpdated = 1;

	fusionEKF.timeSlotNow = (fusionEKF.timeSlotNow+1)%FUSION_EKF_MAX_DELAY;

	// DRIBBLER
	const float curLagT = 0.005f;
	fusionEKF.dribbler.filteredCurrent += CTRL_DELTA_T/(curLagT+CTRL_DELTA_T)*(pSensors->dribbler.current - fusionEKF.dribbler.filteredCurrent);

	pState->dribblerVel = pSensors->dribbler.auxSpeed * botParams.dribbler.motor2BarRatio;
	pState->dribblerCur = fusionEKF.dribbler.filteredCurrent;

	// BALL IR ARRAY
	float irVisibleTimeout = fusionEKF.pConfig->irTimeoutMs * 1e-3f;

	uint32_t tNow = SysTimeUSec();
	float timeSinceVisible = (tNow - fusionEKF.ballLastVisibleTimestamp) * 1e-6f;
	if(timeSinceVisible < irVisibleTimeout)
	{
		TrackingFilter2DPredict(&fusionEKF.ballFilter);
		TrackingFilter2DLimitPosition(&fusionEKF.ballFilter, 50.0f);
		TrackingFilter2DLimitVelocity(&fusionEKF.ballFilter, 2000.0f);
	}

	if(pSensors->ir.ballDetected)
	{
		fusionEKF.ballLastVisibleTimestamp = tNow;

		float filtPos[2];
		TrackingFilter2DGetPosition(&fusionEKF.ballFilter, filtPos);

		float diff[2] = { filtPos[0] - pSensors->ir.estimatedBallPosition[0], filtPos[1] - pSensors->ir.estimatedBallPosition[1] };
		float absDiff = sqrtf(diff[0] * diff[0] + diff[1] * diff[1]);

		float timeSinceUsed = (tNow - fusionEKF.ballLastUsedTimestamp) * 1e-6f;

		if(absDiff < timeSinceUsed * fusionEKF.pConfig->irOutlierMaxVel)
		{
			TrackingFilter2DUpdate(&fusionEKF.ballFilter, pSensors->ir.estimatedBallPosition);
			fusionEKF.ballLastUsedTimestamp = tNow;
		}
	}

	if(timeSinceVisible < irVisibleTimeout)
	{
		if(pSensors->ir.interrupted)
			pState->ballIrState = 2;
		else
			pState->ballIrState = 1;
	}
	else
	{
		pState->ballIrState = 0;
	}

	TrackingFilter2DGetPosition(&fusionEKF.ballFilter, pState->ballIrPos);
}

void FusionEKFSetState(const float* pPos, const float* pVel)
{
	memcpy(fusionEKF.ekf.x.pData, pPos, sizeof(float)*3);
	memcpy(fusionEKF.encGyrPos, pPos, sizeof(float)*3);

	for(uint32_t i = 0; i < FUSION_EKF_MAX_DELAY; i++)
	{
		memcpy(fusionEKF.timeSlots[i].insState, pPos, sizeof(float)*3);
		memcpy(fusionEKF.timeSlots[i].insState+3, pVel, sizeof(float)*2);
		memcpy(fusionEKF.timeSlots[i].meas.pos, pPos, sizeof(float)*3);
	}
}

static uint8_t isVisionSampleValid(const float* pVisPos, const float* pInsPos, int32_t tDelayUs)
{
	// validate sample, outlier rejection
	uint32_t measTime = SysTimeUSec()-tDelayUs;
	float visionDt = (measTime-fusionEKF.vision.timeLastValidSample)*1e-6f;

	float searchRadiusXY = 100.0f*fusionEKF.pConfig->visNoiseXY + fusionEKF.pConfig->outlierMaxVelXY*visionDt;
	float searchRadiusW = 2.0f*fusionEKF.pConfig->visNoiseW + fusionEKF.pConfig->outlierMaxVelW*visionDt;

	float diffX = pVisPos[0] - pInsPos[0];
	float diffY = pVisPos[1] - pInsPos[1];
	float diffXY = sqrtf(diffX*diffX+diffY*diffY);
	float diffW = AngleNormalize(pVisPos[2] - AngleNormalize(pInsPos[2]));

	if(diffXY > searchRadiusXY || fabsf(diffW) > searchRadiusW)
	{
		// this is an invalid sample, vel is impossible
		return 0;
	}

	fusionEKF.vision.timeLastValidSample = measTime;

	return 1;
}

static float visionMultiTurnCorrection(float visionOrientation)
{
	if(visionOrientation < -2.0f && fusionEKF.vision.lastOrient > 2.0f)
		++fusionEKF.vision.turns;

	if(visionOrientation > 2.0f && fusionEKF.vision.lastOrient < -2.0f)
		--fusionEKF.vision.turns;

	fusionEKF.vision.lastOrient = visionOrientation;

	return visionOrientation + fusionEKF.vision.turns*2.0f*M_PI;
}

static void loadNoiseCovariancesFromConfig()
{
	if(fusionEKF.ekf.Ex.pData) // pData is null when fusionEKF has not been initialized yet
	{
		MAT_ELEMENT(fusionEKF.ekf.Ex, 0, 0) = fusionEKF.pConfig->posNoiseXY*fusionEKF.pConfig->posNoiseXY;
		MAT_ELEMENT(fusionEKF.ekf.Ex, 1, 1) = fusionEKF.pConfig->posNoiseXY*fusionEKF.pConfig->posNoiseXY;
		MAT_ELEMENT(fusionEKF.ekf.Ex, 2, 2) = fusionEKF.pConfig->posNoiseW *fusionEKF.pConfig->posNoiseW;
		MAT_ELEMENT(fusionEKF.ekf.Ex, 3, 3) = fusionEKF.pConfig->velNoiseXY*fusionEKF.pConfig->velNoiseXY;
		MAT_ELEMENT(fusionEKF.ekf.Ex, 4, 4) = fusionEKF.pConfig->velNoiseXY*fusionEKF.pConfig->velNoiseXY;

		MAT_ELEMENT(fusionEKF.ekf.Ez, 0, 0) = fusionEKF.pConfig->visNoiseXY*fusionEKF.pConfig->visNoiseXY;
		MAT_ELEMENT(fusionEKF.ekf.Ez, 1, 1) = fusionEKF.pConfig->visNoiseXY*fusionEKF.pConfig->visNoiseXY;
		MAT_ELEMENT(fusionEKF.ekf.Ez, 2, 2) = fusionEKF.pConfig->visNoiseW *fusionEKF.pConfig->visNoiseW;
	}

	if(fusionEKF.ballFilter.kf.Ex.pData)
	{
	    float measError[2] = { fusionEKF.pConfig->irMeasNoiseX, fusionEKF.pConfig->irMeasNoiseY };
	    TrackingFilter2DSetMeasError(&fusionEKF.ballFilter, measError);
	    TrackingFilter2DSetModelError(&fusionEKF.ballFilter, fusionEKF.pConfig->irPosNoise);
	}
}

static void initEKF()
{
	EKFInit(&fusionEKF.ekf, 5, 3, 3, fusionEKF.ekfData);
	arm_mat_scale_f32(&fusionEKF.ekf.Sigma, 0.001f, &fusionEKF.ekf.Sigma);
	fusionEKF.ekf.pState = &ekfStateFunc;
	fusionEKF.ekf.pStateJacobian = &ekfStateJacobianFunc;
	fusionEKF.ekf.pMeas = &ekfMeasFunc;
	fusionEKF.ekf.pMeasJacobian = &ekfMeasJacobianFunc;

	arm_mat_identity_f32(&fusionEKF.ekf.Ex);
	arm_mat_identity_f32(&fusionEKF.ekf.Ez);

	loadNoiseCovariancesFromConfig();
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
