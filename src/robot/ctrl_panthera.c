/*
 * ctrl_panthera.c
 *
 *  Created on: 14.07.2020
 *      Author: AndreR
 */

#include "ctrl_panthera.h"
#include "robot.h"
#include "math/angle_math.h"
#include "math/arm_mat_util_f32.h"
#include "math/map_to_range.h"
#include "math/clamp.h"
#include "math/vector.h"
#include "struct_ids.h"
#include <math.h>

CtrlPanthera ctrlPanthera;

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

ConfigFileDesc ctrlPantheraConfigCtrlDriveDesc =
	{ 0, 0, "", 9, (ElementDesc[]) {
		{  FLOAT, "pos_xy_k", "", "pos/xy/k" },
		{  FLOAT, "pos_xy_max", "m/s", "pos/xy/max" },
		{  FLOAT, "pos_xy_ajitter", "", "pos/xy/anti_jitter" },
		{  FLOAT, "pos_w_k", "", "pos/w/k" },
		{  FLOAT, "pos_w_max", "rad/s", "pos/w/max" },
		{  FLOAT, "pos_w_ajitter", "", "pos/w/anti_jitter" },
		{  FLOAT, "vel_w_k", "", "vel/w/k" },
		{  FLOAT, "vel_w_cent_acc", "m/s^2", "vel/w/cent_acc" },
		{  FLOAT, "torque_vect_strength", "0-1", "torque_vect_strength" },
	} };

ConfigFileDesc ctrlPantheraConfigCtrlDribblerDesc =
	{ 0, 0, "", 9, (ElementDesc[]) {
		{  INT16, "drib_f_lim_angle", "deg", "drib/force/angle" },
		{ UINT16, "drib_f_friction", "mN", "drib/force/fromFriction" },
		{ UINT16, "drib_f_drop_rate", "mN/s", "drib/force/dropRate" },
		{ UINT16, "drib_f_min", "mN", "drib/force/min" },
		{ UINT16, "drib_vel_limit", "mm/s", "drib/vel/limit" },
		{ UINT16, "drib_vel_inc", "mm/s^2", "drib/vel/inc" },
		{ UINT16, "drib_vel_dec", "mm/s^2", "drib/vel/dec" },
		{ UINT16, "drib_frac_min", "0-100", "drib/frac/min" },
		{ UINT16, "drib_frac_max", "0-100", "drib/frac/max" },
	} };

void CtrlPantheraInit(CtrlPantheraConfigCtrlDrive* pConfigCtrlDrive, CtrlPantheraConfigCtrlDribbler* pConfigCtrlDribbler,
		CtrlPantheraConfigModel* pConfigModel, FusionEKFConfig* pConfigEkf, const float* pDefaultPos)
{
	ctrlPanthera.pConfigCtrlDrive = pConfigCtrlDrive;
	ctrlPanthera.pConfigCtrlDribbler = pConfigCtrlDribbler;
	ctrlPanthera.pConfigModel = pConfigModel;

	ModelEncInit(&ctrlPanthera.modelEnc, &ctrlPanthera.pConfigModel->enc, CTRL_DELTA_T);
	FusionEKFInit(pConfigEkf, &pConfigModel->enc);
	TrajBangBangInit(&ctrlPanthera.trajBB);

	float zeroVel[2] = {0, 0};
	FusionEKFSetState(pDefaultPos, zeroVel);
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
			Vector2fRotatePtr(pReference->trajPos[2]-pState->pos[2], pReference->trajVelLocal[0], pReference->trajVelLocal[1], pOutVel);
			Vector2fRotatePtr(pReference->trajPos[2]-pState->pos[2], pReference->trajAccLocal[0], pReference->trajAccLocal[1], pOutAcc);

			// XY position control
			float ctrl[2];
			float xErr = pReference->trajPos[0] - pState->pos[0];
			float yErr = pReference->trajPos[1] - pState->pos[1];
			ctrl[0] = xErr * ctrlPanthera.pConfigCtrlDrive->posXY.k;
			ctrl[1] = yErr * ctrlPanthera.pConfigCtrlDrive->posXY.k;
			Clamp2Df(ctrl, ctrlPanthera.pConfigCtrlDrive->posXY.max);

			float totalErr = sqrtf(xErr*xErr + yErr*yErr);
			float scale = MapToRangef32(0.0f, ctrlPanthera.pConfigCtrlDrive->posXY.antiJitterTheshold, 0.0f, 1.0f, totalErr);
			ctrl[0] *= scale;
			ctrl[1] *= scale;

			// convert to local pos PID output
			float localPosPidOutput[2];
			Vector2fTurnGlobal2Local(pState->pos[2], ctrl[0], ctrl[1], localPosPidOutput, localPosPidOutput+1);

			pOutVel[0] += localPosPidOutput[0];
			pOutVel[1] += localPosPidOutput[1];
			pOutAcc[0] *= scale;
			pOutAcc[1] *= scale;
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
		Clamp2Df(pOutVel, pDrive->limits.velMaxXY);

	// compensate rotation
	float rot = -pState->vel[2]*CTRL_DELTA_T*0.5f;
	Vector2fRotatePtr(rot, pOutVel[0], pOutVel[1], pOutVel);
	Vector2fRotatePtr(rot, pOutAcc[0], pOutAcc[1], pOutAcc);
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
			float posCtrlOut = 0.0f;

			if(pDrive->modeW == DRIVE_MODE_GLOBAL_POS || pDrive->modeW == DRIVE_MODE_GLOBAL_POS_ASYNC)
			{
				// P control on position
				const float orientError = pReference->trajPos[2] - pState->pos[2];
				posCtrlOut = Clampf(orientError * ctrlPanthera.pConfigCtrlDrive->posW.k, ctrlPanthera.pConfigCtrlDrive->posW.max);

				// gain scheduling based on distance to target orientation
				outScale = MapToRangef32(0.0f, ctrlPanthera.pConfigCtrlDrive->posW.antiJitterTheshold, 0.0f, 1.0f, fabsf(orientError));
			}

			// P control on velocity (based on gyro)
			const float angVelError = pReference->trajVelLocal[2] - pState->vel[2];
			pOutVel[2] += (posCtrlOut + angVelError * ctrlPanthera.pConfigCtrlDrive->velW.k) * outScale;

			pOutAcc[2] *= outScale;
		}
		break;
		default:
		{
			return;
		}
	}
}

static void controlDribbler(const RobotCtrlState* pState, const DribblerInput* pDribbler, DriveInput* pDrive, RobotCtrlOutput* pOutput, RobotCtrlReference* pReference)
{
	// Compute dribbler target force based on ball velocity
	const float dribbleLimitAngle = ctrlPanthera.pConfigCtrlDribbler->forceLimitAngle * M_PI/180.0f;
	const float frictionDribblerForce = ctrlPanthera.pConfigCtrlDribbler->forceFromFriction * 1e-3f;
	const float forceMinVal_N = ctrlPanthera.pConfigCtrlDribbler->forceMinVal_mN * 1e-3f;

	const float zeroVelDribblerForce = pDribbler->maxForce;
	const float minDribblerForce = 1.0f;

	float ballVelLocal[2];
	memcpy(ballVelLocal, pState->ballVel, sizeof(float)*2);

	const float ballVelLocalNorm = sqrtf(ballVelLocal[0]*ballVelLocal[0] + ballVelLocal[1]*ballVelLocal[1]);
	const float ballVelScale = MapToRangef32(0.0f, 0.1f, 0.0f, 1.0f, ballVelLocalNorm);

	float ballVelLocalScaled[2];
	if(ballVelLocalNorm > 1e-6f)
	{
		ballVelLocalScaled[0] = ballVelScale * ballVelLocal[0] / ballVelLocalNorm;
		ballVelLocalScaled[1] = ballVelScale * ballVelLocal[1] / ballVelLocalNorm;
	}
	else
	{
		ballVelLocalScaled[0] = 0;
		ballVelLocalScaled[1] = 0;
	}

	const float yDiffToLimitLine = ballVelLocalScaled[1] - tanf(dribbleLimitAngle) * fabsf(ballVelLocalScaled[0]);

	float dribblerForceTarget = yDiffToLimitLine * frictionDribblerForce + zeroVelDribblerForce;
	if(dribblerForceTarget < minDribblerForce)
		dribblerForceTarget = minDribblerForce;

	if(pDribbler->velocity > 0.0f)
		pReference->dribblerForce = dribblerForceTarget;

	// Increase dribbler speed if achieved force is too low
	const float dribblerForceAchievedFraction = pState->dribblerForce / pReference->dribblerForce;

	const float dribblerVelLimit = ctrlPanthera.pConfigCtrlDribbler->velLimit * 1e-3f;
	const float dribblerIncPerSec = ctrlPanthera.pConfigCtrlDribbler->velInc * 1e-3f;
	const float dribblerDecPerSec = ctrlPanthera.pConfigCtrlDribbler->velDec * 1e-3f;

	if((ctrlPanthera.dropRateForce > 0.0f || ctrlPanthera.dropRateSpeed > 0.0f) && pDribbler->velocity <= 0.0f)
	{
		ctrlPanthera.dribblerVel -= ctrlPanthera.dropRateSpeed * CTRL_DELTA_T;
		pReference->dribblerForce -= ctrlPanthera.dropRateForce * CTRL_DELTA_T;

		if(pReference->dribblerForce <= forceMinVal_N)
			ctrlPanthera.dropRateForce = 0.0f;

		if(ctrlPanthera.dribblerVel <= 0.0f)
			ctrlPanthera.dropRateSpeed = 0.0f;
	}
	else if(pState->ballState == BALL_STATE_ACTIVE_DRIBBLING && dribblerForceAchievedFraction < 0.8f)
	{
		ctrlPanthera.dribblerVel += dribblerIncPerSec * CTRL_DELTA_T;
		if(ctrlPanthera.dribblerVel > dribblerVelLimit)
			ctrlPanthera.dribblerVel = dribblerVelLimit;
	}
	else
	{
		ctrlPanthera.dribblerVel -= dribblerDecPerSec * CTRL_DELTA_T;
		if(ctrlPanthera.dribblerVel < pDribbler->velocity)
			ctrlPanthera.dribblerVel = pDribbler->velocity;
	}

	pReference->dribblerVel = ctrlPanthera.dribblerVel;

	// Ball drop logic
	if(pDribbler->velocity <= 0.0f && ctrlPanthera.dribblerLastInputVel > 0.0f)
	{
		const float dropForceRate = ctrlPanthera.pConfigCtrlDribbler->forceDropRate * 1e-3f;

		float dropTime = pReference->dribblerForce / dropForceRate;
		ctrlPanthera.dropRateForce = dropForceRate;
		ctrlPanthera.dropRateSpeed = dropTime > 1e-3f ? (pReference->dribblerVel / dropTime)*0.9f : 1.0f;
	}

	ctrlPanthera.dribblerLastInputVel = pDribbler->velocity;

	// Set dribbler mode and output variables
	if(pDribbler->mode == DRIBBLER_MODE_SPEED)
	{
		if(pReference->dribblerVel < 0.1f && pState->dribblerVel < 0.1f)
			pOutput->dribblerMode = DRIBBLER_MODE_OFF;
		else
			pOutput->dribblerMode = DRIBBLER_MODE_SPEED;
	}
	else if(pDribbler->mode == DRIBBLER_MODE_VOLTAGE)
	{
		pOutput->dribblerMode = DRIBBLER_MODE_VOLTAGE;
	}
	else
	{
		pOutput->dribblerMode = DRIBBLER_MODE_OFF;
	}

	if(pReference->dribblerForce <= forceMinVal_N)
		pReference->dribblerForce = forceMinVal_N;

	pOutput->dribblerVoltage = pDribbler->voltage;
	pOutput->dribblerVel = pReference->dribblerVel * 2.0f/robot.specs.dribbler.barDiameter * robot.specs.dribbler.bar2MotorRatio;
	pOutput->dribblerTorque = pReference->dribblerForce * 0.5f*robot.specs.dribbler.barDiameter * 1.0f/robot.specs.dribbler.bar2MotorRatio;

	// Adjust drive limits if dribbling ball
	if(pState->ballState == BALL_STATE_ACTIVE_DRIBBLING)
	{
		const float maxAccXYLast = sqrtf(pReference->trajAccLocal[0]*pReference->trajAccLocal[0] + pReference->trajAccLocal[1]*pReference->trajAccLocal[1]);
		const float maxAccXFractionLast = maxAccXYLast > 1e-3f ? fabsf(pReference->trajAccLocal[0] / maxAccXYLast) : 0.0f;

		const float maxAccXLast = pDrive->limits.accMaxXY * maxAccXFractionLast;
		const float maxAccRotLast = fabsf(pReference->trajAccLocal[2]) > 1e-3f ? pDrive->limits.accMaxW * robot.specs.physical.dribblerDistance_m : 0.0f;
		const float maxAccTotalLast = maxAccXLast + maxAccRotLast;

		const float accSplitXFraction = maxAccTotalLast > 1e-3f ? (maxAccXLast / maxAccTotalLast) : 0.0f;
		const float accSplitRotFraction = maxAccTotalLast > 1e-3f ? (maxAccRotLast / maxAccTotalLast) : 0.0f;

		// Compute max. acceleration based on achieved dribbler force
		const float scaleMinInOut = ctrlPanthera.pConfigCtrlDribbler->achievedFractionMin * 0.01f;
		const float scaleMaxIn = ctrlPanthera.pConfigCtrlDribbler->achievedFractionMax * 0.01f;
		float accScaling = MapToRangef32(scaleMinInOut, scaleMaxIn, scaleMinInOut, 1.0f, dribblerForceAchievedFraction);
		float finalTotalAccMaxX = maxAccTotalLast * accScaling;

		if(maxAccXFractionLast > 1e-3f && accSplitXFraction > 1e-3f)
			pDrive->limits.accMaxXY = (finalTotalAccMaxX * accSplitXFraction) / maxAccXFractionLast;

		if(accSplitRotFraction > 1e-3f)
			pDrive->limits.accMaxW = (finalTotalAccMaxX * accSplitRotFraction) / robot.specs.physical.dribblerDistance_m;
	}
}

static void applyTorqueVectoring(const float* pOutVel, const float* pOutAcc, float* pMotorTorque, float strength)
{
	// Get wheel force over ground from motor torque (in XY plane)
	float wheelGroundForce[4];
	for(uint8_t i = 0; i < 4; i++)
		wheelGroundForce[i] = pMotorTorque[i] * 1.0f/(robot.specs.physical.wheelRadius_m*CTRL_MOTOR_TO_WHEEL_RATIO);

	// Compute reaction forces acting on mass from acceleration and centrifugal effects
	float reactionForceXY[2];
	reactionForceXY[0] = (-pOutAcc[0] + pOutVel[1] * pOutVel[2]) * robot.specs.physical.mass_kg;
	reactionForceXY[1] = (-pOutAcc[1] - pOutVel[0] * pOutVel[2]) * robot.specs.physical.mass_kg;

	// Get force in Z direction from mass and moments around X and Y axes (trying to tilt robot)
	float forceZMomentsXY[3];
	forceZMomentsXY[0] = robot.specs.physical.mass_kg * 9.81f;
	forceZMomentsXY[1] = reactionForceXY[1] * robot.specs.physical.centerOfGravity_m[2];
	forceZMomentsXY[2] = -reactionForceXY[0] * robot.specs.physical.centerOfGravity_m[2];

	// Given a force/moment equlibrium compute the forces acting from ground up on each wheel contact point
	float groundForces[4];
	RobotMathForceZMomentsXYToGroundForces(forceZMomentsXY, groundForces);

	// Rational: wheel forces (accelerating the robot) are adjusted according to ground forces at each wheel.
	// To redistribute the wheel forces the null vector from the motor torque to robot force matrix is used.
	// Any linear combination of the null vector will not alter the resulting robot force.
	// There are four possible candidates (DistriCandidate) for the scale factor of the linear combination.
	// The wheel forces are scaled four times, each time with a factor resulting in the wheel force being
	// equal to the ground contact force (in absolute values).
	// Afterwards, the scaled wheel forces are checked if any of them is exceeding the ground contact force
	// (which would result in slipping). Furthermore, the error norm is computed for each candidate
	// (scaled wheel forces to ground contact force).
	typedef struct
	{
		float scale;
		float scaledWheelForce[4];
		float error;
		uint8_t isExceedingDownForce;
	} DistriCandidate;

	DistriCandidate candidates[4];

	uint8_t areAllExceedingForce = 1;

	for(uint8_t i = 0; i < 4; i++)
	{
		DistriCandidate* pCand = &candidates[i];
		pCand->error = 0.0f;
		pCand->isExceedingDownForce = 0;

		pCand->scale = (groundForces[i] - fabsf(wheelGroundForce[i])) / robotMath.matForceNullDir.pData[i] * strength;

		for(uint8_t j = 0; j < 4; j++)
		{
			pCand->scaledWheelForce[j] = wheelGroundForce[j] + pCand->scale * robotMath.matForceNullDir.pData[j];

			float err = groundForces[j] - fabsf(pCand->scaledWheelForce[j]);
			pCand->error += err*err;

			if(pCand->scaledWheelForce[j] > groundForces[i])
				pCand->isExceedingDownForce = 1;
		}

		if(pCand->isExceedingDownForce == 0)
			areAllExceedingForce = 0;
	}

	// If all candidates exceed the ground contact force, the one with minimum error is taken. Otherwise,
	// out of all the ones not exceeding ground contact force, the one with minimum error is taken.
	DistriCandidate* pOptCandidate = 0;
	float minError = INFINITY;

	if(areAllExceedingForce)
	{
		for(uint8_t i = 0; i < 4; i++)
		{
			if(candidates[i].error < minError)
			{
				pOptCandidate = &candidates[i];
				minError = candidates[i].error;
			}
		}
	}
	else
	{
		for(uint8_t i = 0; i < 4; i++)
		{
			if(candidates[i].isExceedingDownForce)
				continue;

			if(candidates[i].error < minError)
			{
				pOptCandidate = &candidates[i];
				minError = candidates[i].error;
			}
		}
	}

	// Wheel forces were scaled to ground contact force, but this is different from the original
	// norm of the wheel forces given by the motor torques. Wheel forces are now scaled linearly
	// to bring the norm back to the original value and then transformed to motor torques again.
	float origSum = 0.0f;
	float optSum = 0.0f;

	for(uint8_t i = 0; i < 4; i++)
	{
		origSum += fabsf(wheelGroundForce[i]);
		optSum += fabsf(pOptCandidate->scaledWheelForce[i]);
	}

	float optScale = origSum / optSum;
	if(optSum < 1e-6f)
		optScale = 0.0f;

	float optWheelGroundForce[4];

	for(uint8_t i = 0; i < 4; i++)
	{
		optWheelGroundForce[i] = pOptCandidate->scaledWheelForce[i] * optScale;
		pMotorTorque[i] = optWheelGroundForce[i] * robot.specs.physical.wheelRadius_m*CTRL_MOTOR_TO_WHEEL_RATIO;
	}
}

void CtrlPantheraUpdate(const RobotCtrlState* pState, const SkillOutput* pInput, RobotCtrlOutput* pOutput, RobotCtrlReference* pReference)
{
	if(ctrlPanthera.lastVisionState == 0 && fusionEKF.vision.online)
	{
		TrajBangBangResetState(&ctrlPanthera.trajBB, pState);
	}

	ctrlPanthera.lastVisionState = fusionEKF.vision.online;

	if(fusionEKF.vision.online)
	{
		float xErr = pReference->trajPos[0] - pState->pos[0];
		float yErr = pReference->trajPos[1] - pState->pos[1];
		float totalErrXY = sqrtf(xErr*xErr + yErr*yErr);

		// Reset trajectory generator if position control output would be at maximum
		if(totalErrXY > (ctrlPanthera.pConfigCtrlDrive->posXY.max / ctrlPanthera.pConfigCtrlDrive->posXY.k))
		{
			TrajBangBangResetState(&ctrlPanthera.trajBB, pState);
		}
	}

	// Dribbler control (may modify drive values)
	DriveInput drive = pInput->drive;
	controlDribbler(pState, &pInput->dribbler, &drive, pOutput, pReference);

	// Trajectory update and control
	TrajBangBangUpdate(&ctrlPanthera.trajBB, pState, &drive, &drive.limits, pReference, ctrlPanthera.pConfigCtrlDrive->velW.maxCentrifugalAcc);

	float outAcc[3];
	float outVel[3];

	controlXY(&drive, pReference, pState, outVel, outAcc);
	controlW(&drive, pReference, pState, outVel, outAcc);

	// coulomb & viscous friction compensation
	float frictionForce[3];
	ModelFricGetXY(&ctrlPanthera.pConfigModel->fric, outVel, frictionForce);
	frictionForce[2] = ModelFricGetW(&ctrlPanthera.pConfigModel->fric, outVel[2]);

	float frictionAcc[3];
	frictionAcc[0] = frictionForce[0]/robot.specs.physical.mass_kg;
	frictionAcc[1] = frictionForce[1]/robot.specs.physical.mass_kg;
	frictionAcc[2] = frictionForce[2]/robotMath.momentOfInertia_kg_m2;

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
	RobotMathLocalAccToMotorTorque(outAcc, motorTorque);

	applyTorqueVectoring(outVel, outAcc, motorTorque, ctrlPanthera.pConfigCtrlDrive->torqueVectoringStrength);

	// limit motor current slope
	const float maxCurrentIncPerMs = 0.5f; // [A]
	const float maxTorqueSlope = maxCurrentIncPerMs * robot.specs.driveTrain.motor.Km * 1.0f/CTRL_DELTA_T;

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
	RobotMathLocalVelToMotorVel(outVelComp, pOutput->motorVel);
	pOutput->ctrlMode = MOT_CTRL_VEL_TORQUE;

	if(drive.modeXY == DRIVE_MODE_LOCAL_FORCE || drive.modeW == DRIVE_MODE_LOCAL_FORCE)
	{
		RobotMathLocalForceToMotorTorque((float*)drive.localForce, pOutput->motorTorque);
		memcpy(ctrlPanthera.outTorque, pOutput->motorTorque, sizeof(float)*4);
		pOutput->ctrlMode = MOT_CTRL_TORQUE;
	}

	if(drive.modeXY == DRIVE_MODE_WHEEL_VEL || drive.modeW == DRIVE_MODE_WHEEL_VEL)
	{
		for(uint8_t i = 0; i < 4; i++)
			pOutput->motorVel[i] = drive.wheelVel[i]*CTRL_WHEEL_TO_MOTOR_RATIO;

		pOutput->ctrlMode = MOT_CTRL_VELOCITY;
	}

	if(drive.modeXY == DRIVE_MODE_OFF && drive.modeW == DRIVE_MODE_OFF)
	{
		memset(pOutput->motorVel, 0, sizeof(float)*4);

		pOutput->ctrlMode = MOT_CTRL_OFF;
	}
}
