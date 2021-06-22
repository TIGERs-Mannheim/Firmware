/*
 * ukf.h
 *
 *  Created on: 14.06.2015
 *      Author: AndreR
 *
 *  Unscented Kalman Filter implementation with additive noise.
 *  Taken from van der Merwe, Algorithm 8, p.108
 */

#ifndef UKF_H_
#define UKF_H_

#include "arm_math.h"

#define UKF_SIZE_WM(L)	(2*L+1)
#define UKF_SIZE_WC(L)	(2*L+1)
#define UKF_SIZE_Q(L)	(L*L)
#define UKF_SIZE_R(N)	(N*N)
#define UKF_SIZE_X(L)	(L)
#define UKF_SIZE_U(U)	(U)
#define UKF_SIZE_Y(N)	(N)
#define UKF_SIZE_PX(L)	(L*L)
#define UKF_SIZE_PY(N)	(N*N)
#define UKF_SIZE_PXY(L,N) (L*N)
#define UKF_SIZE_XK1(L) (L*(2*L+1))
#define UKF_SIZE_YK1(L,N) (N*(2*L+1))
#define UKF_SIZE_XK2(L) (L)
#define UKF_SIZE_YK2(N)	(N)
#define UKF_SIZE_K(L,N) (L*N)
#define UKF_SIZE_TMP(m) (m*m)

#define UKF_DATA_SIZE(L, N, U, max) ( \
	UKF_SIZE_WM(L) + UKF_SIZE_WC(L) + UKF_SIZE_Q(L) + UKF_SIZE_R(N) + \
	UKF_SIZE_X(L) + UKF_SIZE_U(U) + UKF_SIZE_Y(N) + UKF_SIZE_PX(L) + \
	UKF_SIZE_XK1(L) + UKF_SIZE_YK1(L,N) + UKF_SIZE_XK2(L) + UKF_SIZE_YK2(N) +\
	UKF_SIZE_K(L,N) + 2*UKF_SIZE_TMP(max) + UKF_SIZE_PY(N) + 2*UKF_SIZE_PXY(L,N) + L)

typedef void(*UKFStateFunc)(arm_matrix_instance_f32* pX, const arm_matrix_instance_f32* pU);
typedef void(*UKFMeasFunc)(const arm_matrix_instance_f32* pX, const arm_matrix_instance_f32* pU, arm_matrix_instance_f32* pY);

typedef struct _UKF
{
	UKFStateFunc pState;
	UKFMeasFunc pMeas;

	float alpha;
	float beta;
	float ki;

	uint16_t L; // number of states
	uint16_t N; // number of measurements
	uint16_t U; // number of control inputs

	// number of sigma points
	uint16_t S; // 2*L+1

	// sigma scaling factor
	float c;

	// weights for mean
	arm_matrix_instance_f32 Wm; // (1 x S)
	// weights for covariance
	arm_matrix_instance_f32 Wc;	// (1 x S)

	// process noise covariance
	arm_matrix_instance_f32 Q;	// (L x L)
	// measurement noise covariance
	arm_matrix_instance_f32 R;	// (N x N)

	// internal state
	arm_matrix_instance_f32 x;	// (L x 1)

	// control input
	arm_matrix_instance_f32 u;	// (U x 1)

	// sensor input
	arm_matrix_instance_f32 y;	// (N x 1)

	// state covariance
	arm_matrix_instance_f32 Px;	// (L x L)
	arm_matrix_instance_f32 Py;	// (N x N)
	arm_matrix_instance_f32 Pxy;// (L x N)
	arm_matrix_instance_f32 PxyT;// (N x L)

	// sigma points
	arm_matrix_instance_f32 Xk;	// (S x L) // original = (L x S)
	arm_matrix_instance_f32 Yk; // (S x N) // original = (N x S)

	// propagated means
	arm_matrix_instance_f32 xk;	// (L x 1)
	arm_matrix_instance_f32 yk;	// (N x 1)

	// Kalman gain
	arm_matrix_instance_f32 K;	// (L x N)

	arm_matrix_instance_f32 tmp;
	arm_matrix_instance_f32 tmp2;

	uint8_t* pNoUpdate;
} UKF;

void UKFInit(UKF* pUKF, uint16_t numStates, uint16_t numMeas, uint16_t numCtrl, float* pData, UKFStateFunc stateFunc, UKFMeasFunc measFunc);
void UKFConfigure(UKF* pUKF, float alpha, float beta, float ki);
void UKFPredict(UKF* pUKF);
void UKFPredictNoStateFunc(UKF* pUKF);
void UKFUpdate(UKF* pUKF);

#endif /* UKF_H_ */
