#pragma once

#include "robot_specs.h"
#include "arm_math.h"

typedef struct _RobotMath
{
	float momentOfInertia_kg_m2;
	float theta_rad[4];	// wheel angles in [rad]

	arm_matrix_instance_f32 matXYW2Motor; // 4x3
	arm_matrix_instance_f32 matMotor2XYW; // 3x4

	arm_matrix_instance_f32 matXYW2Subwheel; // 4x3

	arm_matrix_instance_f32 matForceXYW2Motor; // 4x3
	arm_matrix_instance_f32 matForceMotor2XYW; // 3x4

	arm_matrix_instance_f32	matMotorNullDir; // 4x1, linear combinations of this motor vector do not produce any movement = null space of matMotor2XYW
	arm_matrix_instance_f32 matForceNullDir; // 4x1

	arm_matrix_instance_f32 matGroundForces2ForceZMomentsXY; // 3x4, upward (Z+) forces at wheel contact points to resulting z force and xy moments
	arm_matrix_instance_f32 matForceZMomentsXY2GroundForces; // 4x3, Z force and XY moments to upward forces at wheel contact points
} RobotMath;

extern RobotMath robotMath;

void RobotMathUpdate(const RobotSpecs* pSpecs);

void RobotMathLocalVelToMotorVel(const float* pLocal, float* pMotor);
void RobotMathMotorVelToLocalVel(const float* pMotor, float* pLocal);

void RobotMathLocalForceToMotorTorque(float* pForceXYW, float* pMot);
void RobotMathLocalAccToMotorTorque(float* pAccXYW, float* pMot);
void RobotMathMotorTorqueToLocalForce(const float* pMot, float* pForceXYW);

void RobotMathAccelerationFromMotorVoltages(float* pVolMot, float* pAccXYW);
void RobotMathMotorVoltagesFromAcceleration(float* pAccXYW, float* pVolMot);
void RobotMathVelocityFromMotorVoltages(float* pVolMot, float* pVelXYZ);
void RobotMathMotorVoltagesFromVelocity(float* pVelXYZ, float* pVolMot);

void RobotMathForceZMomentsXYToGroundForces(const float* pForceZMomentsXY, float* pGroundForces);

void RobotMathGetDefaultBotPosition(uint8_t botNumber, float* pPos);
