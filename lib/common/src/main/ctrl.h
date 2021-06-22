/*
 * ctrl.h
 *
 *  Created on: 06.03.2013
 *      Author: AndreR
 */

#pragma once

#include <main/util.h>
#include "commands.h"
#include "arm_math.h"
#include "util/config.h"
#include "util/ema_filter.h"
#include "util/lag_element.h"
#include "log_msgs.h"
#include "intercom_constants.h"

#define CTRL_DELTA_T_IN_US 1000
#define CTRL_DELTA_T (CTRL_DELTA_T_IN_US*1e-6f)

#define MOT_CTRL_OFF			0
#define MOT_CTRL_VELOCITY		1
#define MOT_CTRL_VOLTAGE		2
//#define MOT_CTRL_VEL_ACC		3 // deprecated
#define MOT_CTRL_TORQUE			4
#define MOT_CTRL_VEL_TORQUE		5
#define MOT_CTRL_ANGLE			6

// helper for better readability
#define CTRL_MOTOR_TO_WHEEL_RATIO botParams.driveTrain.motor2WheelRatio
#define CTRL_WHEEL_TO_MOTOR_RATIO botParams.driveTrain.wheel2MotorRatio
#define CTRL_MOTOR_KM botParams.driveTrain.motor.Km
#define CTRL_MOTOR_KN botParams.driveTrain.motor.Kn
#define CTRL_MOTOR_KE botParams.driveTrain.motor.Ke
#define CTRL_MOTOR_KF botParams.driveTrain.motor.Kf
#define CTRL_MOTOR_R botParams.driveTrain.motor.R
#define CTRL_MOTOR_L botParams.driveTrain.motor.L

typedef void(*CtrlInitFunction)(const float*);
typedef void(*CtrlStateFunction)(const RobotSensors*, const RobotCtrlOutput*, RobotCtrlState*);
typedef void(*CtrlFunction)(const RobotCtrlState*, const SkillOutput*, RobotCtrlOutput*, RobotCtrlReference*);
typedef void(*CtrlExitFunction)();

typedef struct _CtrlInstance
{
	CtrlInitFunction pInit;
	CtrlStateFunction pStateEstimationFunc;
	CtrlFunction pControllerFunc;
	CtrlExitFunction pExit;
	void* pLog;
} CtrlInstance;

// Drive Train parameters
typedef struct PACKED _DriveTrainParams
{
	float motor2WheelRatio;
	float wheel2MotorRatio;
	struct PACKED
	{
		float Km;	// motor torque constant [Nm/A]
		float Kn;	// motor speed constant [rad/(s*V)]
		float Ke;	// motor back-EMF constant [V/rad]
		float Kf;	// motor friction constant [Nm*s]
		float R;	// motor resistance [Ohm]
		float L;	// motor inductance [H]
	} motor;
} DriveTrainParams;

typedef struct PACKED _DribblerParams
{
	float motor2BarRatio;
	float bar2MotorRatio;
	float barDiameter;
	struct PACKED
	{
		float Km;	// motor torque constant [Nm/A]
		float Kn;	// motor speed constant [rad/(s*V)]
		float Ke;	// motor back-EMF constant [V/rad]
		float Kf;	// motor friction constant [Nm*s]
		float R;	// motor resistance [Ohm]
		float L;	// motor inductance [H]
	} motor;
} DribblerParams;

// physical bot params
typedef struct PACKED _PhysicalParams
{
	float wheelRadius;
	float frontAngle;
	float backAngle;
	float botRadius;
	float mass; // [kg]
	float dribblerDistance; // from center of robot to center of ball in front of robot
} PhysicalParams;

typedef struct PACKED _BotParameters
{
	DriveTrainParams driveTrain;
	DribblerParams dribbler;
	PhysicalParams physical;
	uint8_t v2016;
	uint8_t extInstalled;
} BotParameters;

extern BotParameters botParams;

typedef struct _Ctrl
{
	ConfigFile* pBotParamsConfig;
	ConfigFile* pBotDriveTrainConfig;
	ConfigFile* pBotDribblerConfig;

	CtrlInstance* pInstance;
	CtrlInstance* pNewInstance;
	uint8_t forceControllerReload;

	float momentOfInertia;
	float theta[4];	// wheel angles in [rad]

	arm_matrix_instance_f32 matXYW2Motor; // 4x3
	arm_matrix_instance_f32 matMotor2XYW; // 3x4

	arm_matrix_instance_f32 matXYW2Subwheel; // 4x3

	arm_matrix_instance_f32 matForceXYW2Motor; // 4x3
	arm_matrix_instance_f32 matForceMotor2XYW; // 3x4

	arm_matrix_instance_f32	matMotorNullDir; // 4x1, linear combinations of this motor vector do not produce any movement = null space of matMotor2XYW

	arm_matrix_instance_f32 matForceXYZ2DownForce; // 4x3

	float defaultPosition[3];
} Ctrl;

extern Ctrl ctrl;

void CtrlInit();
void CtrlStateEstimation(const RobotSensors* pSensors, const RobotCtrlOutput* pOutput, RobotCtrlState* pState);
void CtrlControl(const RobotCtrlState* pState, const SkillOutput* pInput, RobotCtrlOutput* pOutput, RobotCtrlReference* pRef);
void CtrlSetController(CtrlInstance* pInstance);
void CtrlUpdateBotId(uint8_t botId);
void CtrlCheckControllerChange(RobotCtrlOutput* pOutput);
