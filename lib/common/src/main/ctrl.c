/*
 * ctrl.c
 *
 *  Created on: 06.03.2013
 *      Author: AndreR
 */

#include <main/ctrl.h>
#include <main/skills.h>
#include "util/sys_time.h"
#include "struct_ids.h"
#include "util/arm_mat_util_f32.h"
#include "util/boot.h"

#include <string.h>

Ctrl ctrl;

static float pXYW2Motor[12];
static float pMotor2XYW[12];
static float pForceXYW2Motor[12];
static float pForceMotor2XYW[12];
static float pMotorNullDir[4];
static float pForceXYZ2DownForce[12];
static float pXYW2Subwheel[12];

static void updatePhysicalParameters();

static const ConfigFileDesc configFileDescPhysical =
	{ SID_CFG_CTRL_PHYSICAL_PARAMS, 3, "ctrl/physical", 6, (ElementDesc[]) {
		{ FLOAT, "wheel_radius", "m", "Wheel Radius" },
		{ FLOAT, "front_angle", "deg", "Front Angle" },
		{ FLOAT, "back_angle", "deg", "Back  Angle" },
		{ FLOAT, "bot_radius", "m", "Bot Radius" },
		{ FLOAT, "mass", "kg", "Mass" },
		{ FLOAT, "dribbler_distance", "m", "Robot Center to Ball Center" },
	} };

static const ConfigFileDesc configFileDescDriveTrain =
	{ SID_CFG_CTRL_DRIVE_TRAIN, 3, "ctrl/drive_train", 8, (ElementDesc[]) {
		{ FLOAT, "motor2wheel", "-", "Motor to Wheel Ratio" },
		{ FLOAT, "wheel2motor", "-", "Wheel to Motor Ratio" },
		{ FLOAT, "motor_km", "Nm/A", "Motor Torque Constant" },
		{ FLOAT, "motor_kn", "rad/(s*V)", "Motor Speed Constant" },
		{ FLOAT, "motor_ke", "V/rad", "Motor back-EMF constant" },
		{ FLOAT, "motor_kf", "Nm*s", "Motor Friction Constant" },
		{ FLOAT, "motor_r", "Ohm", "Motor Resistance" },
		{ FLOAT, "motor_l", "H", "Motor Inductance" },
	} };

static const ConfigFileDesc configFileDescDribbler =
	{ SID_CFG_CTRL_DRIBBLER, 1, "ctrl/dribbler", 9, (ElementDesc[]) {
		{ FLOAT, "motor2bar", "-", "Motor to Bar Ratio" },
		{ FLOAT, "bar2motor", "-", "Bar to Motor Ratio" },
		{ FLOAT, "bar_diameter", "m", "Dribbling bar diameter" },
		{ FLOAT, "motor_km", "Nm/A", "Motor Torque Constant" },
		{ FLOAT, "motor_kn", "rad/(s*V)", "Motor Speed Constant" },
		{ FLOAT, "motor_ke", "V/rad", "Motor back-EMF constant" },
		{ FLOAT, "motor_kf", "Nm*s", "Motor Friction Constant" },
		{ FLOAT, "motor_r", "Ohm", "Motor Resistance" },
		{ FLOAT, "motor_l", "H", "Motor Inductance" },
	} };

static void configUpdateCallback(uint16_t cfgId)
{
	switch(cfgId)
	{
		case SID_CFG_CTRL_PHYSICAL_PARAMS:
		{
			updatePhysicalParameters();

			// reload controller so that its init function can take the new parameters into account
			ctrl.forceControllerReload = 1;
		}
		break;
	}
}

void CtrlInit()
{
	// construct matrices
	ctrl.matXYW2Motor = (arm_matrix_instance_f32){4, 3, pXYW2Motor};
	ctrl.matMotor2XYW = (arm_matrix_instance_f32){3, 4, pMotor2XYW};
	ctrl.matForceXYW2Motor = (arm_matrix_instance_f32){4, 3, pForceXYW2Motor};
	ctrl.matForceMotor2XYW = (arm_matrix_instance_f32){3, 4, pForceMotor2XYW};
	ctrl.matMotorNullDir = (arm_matrix_instance_f32){4, 1, pMotorNullDir};
	ctrl.matForceXYZ2DownForce = (arm_matrix_instance_f32){4, 3, pForceXYZ2DownForce};
	ctrl.matXYW2Subwheel = (arm_matrix_instance_f32){4, 3, pXYW2Subwheel};

	ctrl.pBotParamsConfig = ConfigOpenOrCreate(&configFileDescPhysical, &botParams.physical, sizeof(PhysicalParams), &configUpdateCallback, 1);
	ctrl.pBotDriveTrainConfig = ConfigOpenOrCreate(&configFileDescDriveTrain, &botParams.driveTrain, sizeof(DriveTrainParams), &configUpdateCallback, 1);
	ctrl.pBotDribblerConfig = ConfigOpenOrCreate(&configFileDescDribbler, &botParams.dribbler, sizeof(DribblerParams), &configUpdateCallback, 1);

	ConfigNotifyUpdate(ctrl.pBotParamsConfig);
}

void CtrlStateEstimation(const RobotSensors* pSensors, const RobotCtrlOutput* pOutput, RobotCtrlState* pState)
{
	if(ctrl.pInstance && ctrl.pInstance->pStateEstimationFunc)
	{
		(*ctrl.pInstance->pStateEstimationFunc)(pSensors, pOutput, pState);
	}
}

void CtrlControl(const RobotCtrlState* pState, const SkillOutput* pInput, RobotCtrlOutput* pOutput, RobotCtrlReference* pRef)
{
	if(ctrl.pInstance && ctrl.pInstance->pControllerFunc)
	{
		(*ctrl.pInstance->pControllerFunc)(pState, pInput, pOutput, pRef);
	}

	CtrlCheckControllerChange(pOutput);
}

void CtrlUpdateBotId(uint8_t botId)
{
	CtrlUtilGetDefaultBotPosition(botId, ctrl.defaultPosition);
}

static void updatePhysicalParameters()
{
	arm_matrix_instance_f32 matCM2Wheel = {4, 3, (float[12]){}};
	const float centerOfMass[3] = {0, 0, 0.056};

	float frontAngle = botParams.physical.frontAngle;
	float backAngle = botParams.physical.backAngle;
	float botRadius = botParams.physical.botRadius;
	float mass = botParams.physical.mass;

	ctrl.momentOfInertia = 0.55f*mass*botRadius*botRadius;

	float alpha = frontAngle*PI/180.0f;
	float beta = backAngle*PI/180.0f;

	ctrl.theta[0] = alpha;
	ctrl.theta[1] = PI-alpha;
	ctrl.theta[2] = PI+beta;
	ctrl.theta[3] = 2*PI-beta;

	for(uint8_t i = 0; i < 4; i++)
	{
		// construct matrix for XYW velocity to motor velocity conversion
		MAT_ELEMENT(ctrl.matXYW2Motor, i, 0) = -sinf(ctrl.theta[i]);
		MAT_ELEMENT(ctrl.matXYW2Motor, i, 1) = cosf(ctrl.theta[i]);
		MAT_ELEMENT(ctrl.matXYW2Motor, i, 2) = botRadius;

		// construct matrix for XYW velocity to subwheel velocity conversion
		MAT_ELEMENT(ctrl.matXYW2Subwheel, i, 0) = cosf(ctrl.theta[i]);
		MAT_ELEMENT(ctrl.matXYW2Subwheel, i, 1) = sinf(ctrl.theta[i]);
		MAT_ELEMENT(ctrl.matXYW2Subwheel, i, 2) = 0;

		// construct matrix for motor forces to XYW forces/moment conversion
		MAT_ELEMENT(ctrl.matForceMotor2XYW, 0, i) = -sinf(ctrl.theta[i]);
		MAT_ELEMENT(ctrl.matForceMotor2XYW, 1, i) = cosf(ctrl.theta[i]);
		MAT_ELEMENT(ctrl.matForceMotor2XYW, 2, i) = botRadius;

		// construct vectors from center of mass to wheels
		MAT_ELEMENT(matCM2Wheel, i, 0) = cosf(ctrl.theta[i])*botParams.physical.botRadius - centerOfMass[0];
		MAT_ELEMENT(matCM2Wheel, i, 1) = sinf(ctrl.theta[i])*botParams.physical.botRadius - centerOfMass[1];
		MAT_ELEMENT(matCM2Wheel, i, 2) = -centerOfMass[2];
	}

	arm_mat_pinv(&ctrl.matXYW2Motor, &ctrl.matMotor2XYW);
	arm_mat_pinv(&ctrl.matForceMotor2XYW, &ctrl.matForceXYW2Motor);

	// construct matrix to calculate wheel down forces from XY acceleration
	arm_matrix_instance_f32 matQ = {3, 4, (float[12]){}};
	arm_mat_pinv(&matCM2Wheel, &matQ);

	for(uint8_t i = 0; i < 4; i++)
	{
		MAT_ELEMENT(ctrl.matForceXYZ2DownForce, i, 0) = MAT_ELEMENT(matCM2Wheel, i, 2)*MAT_ELEMENT(matQ, 0, i);
		MAT_ELEMENT(ctrl.matForceXYZ2DownForce, i, 1) = MAT_ELEMENT(matCM2Wheel, i, 2)*MAT_ELEMENT(matQ, 1, i);
		MAT_ELEMENT(ctrl.matForceXYZ2DownForce, i, 2) = MAT_ELEMENT(matCM2Wheel, i, 2)*MAT_ELEMENT(matQ, 2, i);
	}

	// Construct motor null vector
	// this simple method is only valid for robots with an equal angle between the side wheels
	pMotorNullDir[0] = MAT_ELEMENT(ctrl.matMotor2XYW, 1, 2);
	pMotorNullDir[1] = MAT_ELEMENT(ctrl.matMotor2XYW, 1, 3);
	pMotorNullDir[2] = MAT_ELEMENT(ctrl.matMotor2XYW, 1, 1);
	pMotorNullDir[3] = MAT_ELEMENT(ctrl.matMotor2XYW, 1, 0);
	float length;
	arm_power_f32(pMotorNullDir, 4, &length);
	length = sqrtf(length);
	arm_scale_f32(pMotorNullDir, 1/length, pMotorNullDir, 4);
}

void CtrlSetController(CtrlInstance* pInstance)
{
	ctrl.pNewInstance = pInstance;
}

void CtrlCheckControllerChange(RobotCtrlOutput* pOutput)
{
	if(ctrl.pInstance != ctrl.pNewInstance || ctrl.forceControllerReload)
	{
		// controller change requested
		ctrl.forceControllerReload = 0;

		if(ctrl.pInstance)
		{
			if(ctrl.pInstance->pExit)
				(*ctrl.pInstance->pExit)();
		}

		ctrl.pInstance = ctrl.pNewInstance;

		if(ctrl.pInstance)
		{
			if(ctrl.pInstance->pInit)
				(*ctrl.pInstance->pInit)(ctrl.defaultPosition);
		}
		else
		{
			pOutput->ctrlMode = MOT_CTRL_OFF;
		}
	}
}
