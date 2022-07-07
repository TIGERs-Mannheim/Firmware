/*
 * ekf.h
 *
 *  Created on: 09.03.2017
 *      Author: AndreR
 */

#pragma once

#include "arm_math.h"

#define EKF_SIZE_A(f)		(f*f)
#define EKF_SIZE_C(h,f)		(h*f)
#define EKF_SIZE_EX(f)		(f*f)
#define EKF_SIZE_EZ(h)		(h*h)
#define EKF_SIZE_X(f)		(f)
#define EKF_SIZE_SIGMA(f)	(f*f)
#define EKF_SIZE_K(f,h)		(f*h)
#define EKF_SIZE_U(g)		(g)
#define EKF_SIZE_Z(h)		(h)
#define EKF_SIZE_MAX(max)	(max*max)

// state vector (x) rows: f
// control vector (u) rows: g
// sensor vector (z) rows: h
#define EKF_DATA_SIZE(f,g,h,max) \
	(EKF_SIZE_A(f)+EKF_SIZE_C(h,f)+EKF_SIZE_EX(f) \
	+EKF_SIZE_EZ(h)+EKF_SIZE_X(f)+EKF_SIZE_SIGMA(f)+EKF_SIZE_K(f,h)+EKF_SIZE_U(g) \
	+EKF_SIZE_Z(h)+EKF_SIZE_MAX(max)*3)

typedef void(*EKFStateFunc)(arm_matrix_instance_f32* pX, const arm_matrix_instance_f32* pU);
typedef void(*EKFStateJacobianFunc)(const arm_matrix_instance_f32* pX, const arm_matrix_instance_f32* pU, arm_matrix_instance_f32* pF);
typedef void(*EKFMeasFunc)(const arm_matrix_instance_f32* pX, arm_matrix_instance_f32* pY);
typedef void(*EKFMeasJacobianFunc)(const arm_matrix_instance_f32* pX, arm_matrix_instance_f32* pH);

typedef struct _EKF
{
	uint16_t f;
	uint16_t g;
	uint16_t h;

	// state and measurement functions
	EKFStateFunc pState;
	EKFStateJacobianFunc pStateJacobian;
	EKFMeasFunc pMeas;
	EKFMeasJacobianFunc pMeasJacobian;

	// user matrices
	arm_matrix_instance_f32 A;		// (f x f)
	arm_matrix_instance_f32 C;		// (h x f)
	arm_matrix_instance_f32 Ex;		// (f x f)
	arm_matrix_instance_f32 Ez;		// (h x h)

	// internal matrices
	arm_matrix_instance_f32 x;		// state (f x 1)
	arm_matrix_instance_f32 Sigma;	// uncertainty (f x f)
	arm_matrix_instance_f32 K;		// Kalman gain (f x h)

	// command input
	arm_matrix_instance_f32 u;		// (g x 1)

	// sensor input
	arm_matrix_instance_f32 z;		// (h x 1)

	// temporary calculation matrices
	arm_matrix_instance_f32 tmp1;
	arm_matrix_instance_f32 tmp2;
	arm_matrix_instance_f32 tmp3;
} EKF;

void EKFInit(EKF* pKF, uint16_t numStates, uint16_t numCtrl, uint16_t numSensors, float* pData);
void EKFPredict(EKF* pKF);
void EKFUpdate(EKF* pKF);
