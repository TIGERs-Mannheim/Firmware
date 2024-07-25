#pragma once

#include "util/config.h"
#include "drv/mcu_motor.h"

#pragma pack(push, 1)

// Drive Train parameters
typedef struct _DriveTrainParams
{
	float motor2WheelRatio;
	float wheel2MotorRatio;
	MotorParams motor;
} DriveTrainParams;

typedef struct _DribblerParams
{
	float motor2BarRatio;
	float bar2MotorRatio;
	float barDiameter;
	MotorParams motor;
} DribblerParams;

// physical bot params
typedef struct _PhysicalParams
{
	float wheelRadius_m;
	float frontAngle_deg;
	float backAngle_deg;
	float botRadius_m; // Robot center to wheel/ground contact point
	float mass_kg;
	float dribblerDistance_m; // from center of robot to center of ball in front of robot
	float dribblerWidth_m; // width the ball can move while at the dribbler
	float centerOfGravity_m[3]; // measured from geometric robot center point on ground
	float massDistributionZ; // Factor between 0.5 (mass evenly distributed/solid cylinder) and 1.0 (all mass at outer radius/hollow shell), used for inertia calculation
} PhysicalParams;

typedef struct _RobotSpecs
{
	DriveTrainParams driveTrain;
	DribblerParams dribbler;
	PhysicalParams physical;
} RobotSpecs;

#pragma pack(pop)

extern ConfigFileDesc robotSpecsConfigDescPhysical;
extern ConfigFileDesc robotSpecsConfigDescDriveTrain;
extern ConfigFileDesc robotSpecsConfigDescDribbler;
