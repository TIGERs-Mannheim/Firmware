/*
 * fusion_ekf.c
 *
 *  Created on: 03.08.2020
 *      Author: AndreR
 */

#include "fusion_ekf.h"
#include "struct_ids.h"
#include "robot.h"
#include "math/arm_mat_util_f32.h"
#include "math/angle_math.h"
#include "math/map_to_range.h"
#include "math/clamp.h"
#include "math/vector.h"
#include "hal/sys_time.h"
#include <math.h>

static void ekfStateFunc(arm_matrix_instance_f32* pX, const arm_matrix_instance_f32* pU);
static void ekfStateJacobianFunc(const arm_matrix_instance_f32* pX, const arm_matrix_instance_f32* pU, arm_matrix_instance_f32* pF);
static void ekfMeasFunc(const arm_matrix_instance_f32* pX, arm_matrix_instance_f32* pY);
static void ekfMeasJacobianFunc(const arm_matrix_instance_f32* pX, arm_matrix_instance_f32* pH);
static void initEKF();
static void loadNoiseCovariancesFromConfig();

static void ballStateEstimation(const RobotSensors* pSensors, RobotCtrlState* pState);
static uint8_t isVisionSampleValid(const float* pVisPos, const float* pInsPos, int32_t tDelayUs);
static float visionMultiTurnCorrection(float visionOrientation);
static float lookupDribblerIdleCurrent(float speed);

FusionEKF fusionEKF;

ConfigFileDesc fusionEKFConfigDesc =
	{ 0, 0, "", 18, (ElementDesc[]) {
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
		{ FLOAT, "ball_centering_factor", "-", "ball/centering/factor" },
		{ FLOAT, "ball_centering_limit", "m/s", "ball/centering/limit" },
		{ UINT8, "dribbler_strong_on", "0-100", "dribbler/strong/on" },
		{ UINT8, "dribbler_strong_off", "0-100", "dribbler/strong/off" },
		{ UINT16, "ball_timeout", "ms", "ball/timeout" },
		{ UINT16, "active_drib_min_force", "mN", "dribbler/active_min_force" },
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

	FlashFSOpenOrCreate("drib/idle_cur", 1, &fusionEKF.dribbler.idleCurrentTable, sizeof(fusionEKF.dribbler.idleCurrentTable), &fusionEKF.dribbler.pIdleCurrentFile);

	LagElementPT1Init(&fusionEKF.lagAccel[0], 1.0f, fusionEKF.pConfig->emaAccelT, CTRL_DELTA_T);
	LagElementPT1Init(&fusionEKF.lagAccel[1], 1.0f, fusionEKF.pConfig->emaAccelT, CTRL_DELTA_T);

	LagElementPT1Init(&fusionEKF.dribbler.lagCurrent, 1.0f, 0.005f, CTRL_DELTA_T);
}

void FusionEKFUpdate(const RobotSensors* pSensors, const RobotCtrlOutput* pLastCtrlOut, const RobotCtrlReference* pLastCtrlRef, RobotCtrlState* pState)
{
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
	RobotMathMotorVelToLocalVel(pSensors->enc.vel, encVelLocal);

	// use encoders for rotation only at standstill
	if(fabsf(encVelLocal[2]) > 0.01f)
		encVelLocal[2] = pSensors->gyr.rotVel[2];

	// Use encoder velocity model for X and Y
	ModelEncStep(&fusionEKF.modelEnc, encVelLocal);
	memcpy(encVelLocal, fusionEKF.modelEnc.state, sizeof(float)*2);

	// integrate velocity in global frame
	float encVelGlobal[2];
	Vector2fTurnLocal2Global(fusionEKF.encGyrPos[2], encVelLocal[0], encVelLocal[1], encVelGlobal, encVelGlobal+1);
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

	Vector2fTurnLocal2Global(pStateToUse[2], pStateToUse[3], pStateToUse[4], &pState->vel[0], &pState->vel[1]);
	pState->vel[2] = pSlotNow->meas.gyrAcc[0];

	Vector2fTurnLocal2Global(pStateToUse[2], pSlotNow->meas.gyrAcc[1], pSlotNow->meas.gyrAcc[2], &pState->acc[0], &pState->acc[1]);
	pState->acc[2] = 0;	// can't estimate that with this filter

	pState->posUpdated = 1;
	pState->velUpdated = 1;
	pState->accUpdated = 1;

	fusionEKF.timeSlotNow = (fusionEKF.timeSlotNow+1)%FUSION_EKF_MAX_DELAY;

	// DRIBBLER

	// Slightly filter current measurements as they are very noisy
	pState->dribblerCur = LagElementPT1Process(&fusionEKF.dribbler.lagCurrent, pSensors->dribbler.current);

	// Fill basic state estimates
	pState->dribblerVel = pSensors->dribbler.auxSpeed * robot.specs.dribbler.motor2BarRatio * 0.5f*robot.specs.dribbler.barDiameter;

	// Lookup idle current drawn by dribbler when not in ball contact and compute friction constant from it
	float curIdle_A = lookupDribblerIdleCurrent(pState->dribblerVel);
	float frictionConstant_Nms = robot.specs.dribbler.motor.Km * curIdle_A / pSensors->dribbler.auxSpeed;

	if(isfinite(frictionConstant_Nms))
		pState->dribblerKf = frictionConstant_Nms;

	if(pState->dribblerKf < robot.specs.dribbler.motor.Kf*0.1f)
		pState->dribblerKf = robot.specs.dribbler.motor.Kf*0.1f;

	pState->dribblerForce = (robot.specs.dribbler.motor.Km * pState->dribblerCur - pState->dribblerKf * pSensors->dribbler.auxSpeed) * 1.0f/robot.specs.dribbler.motor2BarRatio * 2.0f/robot.specs.dribbler.barDiameter;

	// BALL STATE ESTIMATION
	ballStateEstimation(pSensors, pState);
	float activeDribblingMinForce_N = fusionEKF.pConfig->activeDribblingForce_mN * 1e-3f;

	// dribbler state update
	if(pLastCtrlRef->dribblerForce <= activeDribblingMinForce_N || pLastCtrlOut->dribblerMode == DRIBBLER_MODE_OFF)
	{
		fusionEKF.dribbler.isStrongDribbling = 0;
		pState->dribblerState = DRIBBLER_STATE_OFF;
	}
	else if(pState->ballState != BALL_STATE_ACTIVE_DRIBBLING)
	{
		fusionEKF.dribbler.isStrongDribbling = 0;
		pState->dribblerState = DRIBBLER_STATE_IDLE;
	}
	else
	{
		const float dribblerForceAchievedFraction = pState->dribblerForce / pLastCtrlRef->dribblerForce;

		if(fusionEKF.dribbler.isStrongDribbling && dribblerForceAchievedFraction < fusionEKF.pConfig->dribblerStrongOff*0.01f)
			fusionEKF.dribbler.isStrongDribbling = 0;
		else if(!fusionEKF.dribbler.isStrongDribbling && dribblerForceAchievedFraction > fusionEKF.pConfig->dribblerStrongOn*0.01f)
			fusionEKF.dribbler.isStrongDribbling = 1;

		if(fusionEKF.dribbler.isStrongDribbling)
			fusionEKF.dribbler.strongTicks++;
		else
			fusionEKF.dribbler.strongTicks = 0;

		if(fusionEKF.dribbler.strongTicks >= 100)
			pState->dribblerState = DRIBBLER_STATE_STRONG;
		else
			pState->dribblerState = DRIBBLER_STATE_WEAK;
	}
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

static void ballStateEstimation(const RobotSensors* pSensors, RobotCtrlState* pState)
{
	const uint32_t tNow = SysTimeUSec();
	const uint32_t prevBallState = pState->ballState;
	const float deadReckoningTimeout_s = fusionEKF.pConfig->ballTimeoutMs * 1e-3f;
	const float activeDribblingMinForce_N = fusionEKF.pConfig->activeDribblingForce_mN * 1e-3f;

	// Figure out basic ball state/mode
	if(pState->dribblerForce >= activeDribblingMinForce_N && pSensors->ir.interrupted)
	{
		pState->ballState = BALL_STATE_ACTIVE_DRIBBLING;
		fusionEKF.ballLastDetectedTimestamp = tNow;
	}
	else if(pSensors->ir.interrupted)
	{
		pState->ballState = BALL_STATE_AT_BARRIER;
		fusionEKF.ballLastDetectedTimestamp = tNow;
	}
	else if(pSensors->ball.updated && isfinite(pSensors->ball.pos[0]))
	{
		pState->ballState = BALL_STATE_FAR_DETECTION;
		fusionEKF.ballLastDetectedTimestamp = tNow;
	}
	else if((tNow - fusionEKF.ballLastDetectedTimestamp) * 1e-6f < deadReckoningTimeout_s)
	{
		pState->ballState = BALL_STATE_DEAD_RECKONING;
	}
	else
	{
		pState->ballState = BALL_STATE_UNKNOWN;
	}

	// Ball models for different states
	switch(pState->ballState)
	{
		// Dead reckoning and far detection use global coordinates
		case BALL_STATE_DEAD_RECKONING:
		{
			fusionEKF.ballPosGlobal[0] += fusionEKF.ballVelGlobal[0] * CTRL_DELTA_T;
			fusionEKF.ballPosGlobal[1] += fusionEKF.ballVelGlobal[1] * CTRL_DELTA_T;
		}
		break;
		case BALL_STATE_FAR_DETECTION:
		{
			memcpy(fusionEKF.ballPosGlobal, pSensors->ball.pos, sizeof(float)*2);
			memcpy(fusionEKF.ballVelGlobal, pSensors->ball.vel, sizeof(float)*2);
		}
		break;
		// Barrier and dribbling use dribbler coordinates
		case BALL_STATE_AT_BARRIER:
		case BALL_STATE_ACTIVE_DRIBBLING:
		{
			if(prevBallState != BALL_STATE_ACTIVE_DRIBBLING && prevBallState != BALL_STATE_AT_BARRIER)
			{
				// First time run after a switch to one of these states => assume centered ball
				pState->ballPos[0] = 0.0f;
				pState->ballPos[1] = pState->ballState == BALL_STATE_ACTIVE_DRIBBLING ? 0.0f : 1e-3f;
				pState->ballVel[0] = 0.0f;
				pState->ballVel[1] = 0.0f;
			}

			float botVelLocal[2];
			Vector2fTurnGlobal2Local(pState->pos[2], pState->vel[0], pState->vel[1], botVelLocal, botVelLocal+1);

			float ballVelLocal[2];
			ballVelLocal[0] = -botVelLocal[0] + pState->vel[2] * robot.specs.physical.dribblerDistance_m;
			ballVelLocal[1] = -botVelLocal[1];

			memcpy(pState->ballVel, ballVelLocal, sizeof(float)*2);

			const float centeringLimit = fusionEKF.pConfig->ballCenteringVelLimit;
			const float centeringMapping = centeringLimit * fusionEKF.pConfig->ballCenteringFactor;

			float centeringVelocity = MapToRangef32(0.0f, centeringLimit, 0.0f, centeringMapping, fabsf(pState->ballVel[1])); // assume Y movement slowly centers ball
			centeringVelocity *= MapToRangef32(-5e-3f, 5e-3f, 1.0f, -1.0f, pState->ballPos[0]); // reduce centering velocity near center (=

			pState->ballPos[0] += (ballVelLocal[0] + centeringVelocity) * CTRL_DELTA_T;

			const float barHalfWidth = robot.specs.physical.dribblerWidth_m * 0.5f;

			pState->ballPos[0] = Clampf(pState->ballPos[0], barHalfWidth);

			if(pState->ballState == BALL_STATE_ACTIVE_DRIBBLING)
			{
				pState->ballPos[1] = 0.0f;
			}
			else
			{
				pState->ballPos[1] = 1e-3f;
			}
		}
		break;
		default: break;
	}

	// Ensure global and dribbler frame data matches
	if(pState->ballState == BALL_STATE_AT_BARRIER || pState->ballState == BALL_STATE_ACTIVE_DRIBBLING)
	{
		// transform dribbler ball state to global state
		float ballPos_fBot[2];
		ballPos_fBot[0] = pState->ballPos[0];
		ballPos_fBot[1] = pState->ballPos[1] + robot.specs.physical.dribblerDistance_m;

		const float ballDistance = sqrtf(ballPos_fBot[0]*ballPos_fBot[0] + ballPos_fBot[1]*ballPos_fBot[1]);

		float ballPos_fMap[2];
		Vector2fTurnLocal2Global(pState->pos[2], ballPos_fBot[0], ballPos_fBot[1], ballPos_fMap, ballPos_fMap+1);

		ballPos_fMap[0] += pState->pos[0];
		ballPos_fMap[1] += pState->pos[1];

		float botVel_fBot[2];
		Vector2fTurnGlobal2Local(pState->pos[2], pState->vel[0], pState->vel[1], botVel_fBot, botVel_fBot+1);

		float ballVel_fBall[2];
		ballVel_fBall[0] = pState->ballVel[0] + botVel_fBot[0] - pState->vel[2] * ballDistance;
		ballVel_fBall[1] = pState->ballVel[1] + botVel_fBot[1];

		float ballVel_fMap[2];
		Vector2fTurnLocal2Global(pState->pos[2], ballVel_fBall[0], ballVel_fBall[1], ballVel_fMap, ballVel_fMap+1);

		memcpy(fusionEKF.ballPosGlobal, ballPos_fMap, sizeof(float)*2);
		memcpy(fusionEKF.ballVelGlobal, ballVel_fMap, sizeof(float)*2);
	}
	else
	{
		// transform global ball state to dribbler state
		float botToBall_fMap[2];
		botToBall_fMap[0] = fusionEKF.ballPosGlobal[0] - pState->pos[0];
		botToBall_fMap[1] = fusionEKF.ballPosGlobal[1] - pState->pos[1];

		const float ballDistance = sqrtf(botToBall_fMap[0]*botToBall_fMap[0] + botToBall_fMap[1]*botToBall_fMap[1]);

		float ballPos_fBot[2];
		Vector2fTurnGlobal2Local(pState->pos[2], botToBall_fMap[0], botToBall_fMap[1], ballPos_fBot, ballPos_fBot+1);

		ballPos_fBot[1] -= robot.specs.physical.dribblerDistance_m; // effectively ballPos_fDribbler now

		float velDiff_fMap[2];
		velDiff_fMap[0] = fusionEKF.ballVelGlobal[0] - pState->vel[0];
		velDiff_fMap[1] = fusionEKF.ballVelGlobal[1] - pState->vel[1];

		float ballVel_fBot[2];
		Vector2fTurnGlobal2Local(pState->pos[2], velDiff_fMap[0], velDiff_fMap[1], ballVel_fBot, ballVel_fBot+1);

		ballVel_fBot[0] += pState->vel[2] * ballDistance;

		memcpy(pState->ballPos, ballPos_fBot, sizeof(float)*2);
		memcpy(pState->ballVel, ballVel_fBot, sizeof(float)*2);
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

static float lookupDribblerIdleCurrent(float speed)
{
	float curIdle_A = 0.0f;

	const FusionEKFDribblerIdleCurTable* pTable = &fusionEKF.dribbler.idleCurrentTable;

	if(pTable->usedPoints >= 2)
	{
		float minSpeed = pTable->speed_mmDs[0]*1e-3f;
		float maxSpeed = pTable->speed_mmDs[pTable->usedPoints-1]*1e-3f;

		if(speed <= minSpeed)
		{
			curIdle_A = pTable->cur_mA[0]*1e-3f;
		}
		else if(speed >= maxSpeed)
		{
			curIdle_A = pTable->cur_mA[pTable->usedPoints-1]*1e-3f;
		}
		else
		{
			uint32_t upperIndex = 1;
			uint32_t lowerIndex = 0;

			for(uint32_t i = 1; i < pTable->usedPoints; i++)
			{
				if(speed < pTable->speed_mmDs[i]*1e-3f)
				{
					upperIndex = i;
					lowerIndex = i-1;
					break;
				}
			}

			curIdle_A = MapToRangef32(pTable->speed_mmDs[lowerIndex]*1e-3f, pTable->speed_mmDs[upperIndex]*1e-3f, pTable->cur_mA[lowerIndex]*1e-3f, pTable->cur_mA[upperIndex]*1e-3f, speed);
		}
	}

	return curIdle_A;
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
