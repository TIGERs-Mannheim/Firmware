/*
 * srukf.h
 *
 *  Created on: 28.03.2016
 *      Author: AndreR
 *
 *  Square-Root Unscented Kalman Filter implementation with additive noise.
 *  Taken from van der Merwe, Algorithm 11, p.115
 */

#ifndef SRUKF_H_
#define SRUKF_H_

#include "arm_math.h"

#define SRUKF_SIZE_WM(L)	(2*L+1)
#define SRUKF_SIZE_WC(L)	(2*L+1)
#define SRUKF_SIZE_SV(L)	(L)
#define SRUKF_SIZE_SN(N)	(N)
#define SRUKF_SIZE_X(L)		(L)
#define SRUKF_SIZE_U(U)		(U)
#define SRUKF_SIZE_Y(N)		(N)
#define SRUKF_SIZE_SX(L)	(L*L)
#define SRUKF_SIZE_SY(N)	(N*N)
#define SRUKF_SIZE_PXY(L,N)	(L*N)
#define SRUKF_SIZE_XK1(L)	(L*(2*L+1))
#define SRUKF_SIZE_YK1(L,N)	(N*(2*L+1))
#define SRUKF_SIZE_XK2(L)	(L)
#define SRUKF_SIZE_YK2(N)	(N)
#define SRUKF_SIZE_K(L,N) 	(L*N)
#define SRUKF_SIZE_TMP(m) 	(3*m*3*m)

#define SRUKF_DATA_SIZE(L, N, U, max) ( \
	SRUKF_SIZE_WM(L) + SRUKF_SIZE_WC(L) + SRUKF_SIZE_SV(L) + SRUKF_SIZE_SN(N) + \
	SRUKF_SIZE_X(L) + SRUKF_SIZE_U(U) + SRUKF_SIZE_Y(N) + SRUKF_SIZE_SX(L) + \
	SRUKF_SIZE_SY(N) + SRUKF_SIZE_PXY(L,N) + SRUKF_SIZE_XK1(L) + SRUKF_SIZE_YK1(L,N) +\
	SRUKF_SIZE_XK2(L) + SRUKF_SIZE_YK2(N) + SRUKF_SIZE_K(L,N) + 3*SRUKF_SIZE_TMP(max) + N + L + N)

typedef void(*SRUKFStateFunc)(arm_matrix_instance_f32* pX, const arm_matrix_instance_f32* pU);
typedef void(*SRUKFMeasFunc)(const arm_matrix_instance_f32* pX, const arm_matrix_instance_f32* pU, arm_matrix_instance_f32* pY);

typedef struct _SRUKF
{
	SRUKFStateFunc pState;
	SRUKFMeasFunc pMeas;

	float alpha;
	float beta;
	float ki;

	uint16_t L; // number of states
	uint16_t N; // number of measurements
	uint16_t U; // number of control inputs

	// number of sigma points
	uint16_t numSigma; // 2*L+1

	// sigma scaling factor
	float gamma;

	// weights for mean
	arm_matrix_instance_f32 Wm; // (1 x S)
	// weights for covariance
	arm_matrix_instance_f32 Wc;	// (1 x S)

	// square-root of  process noise covariance
	float* pSvDiag; // only the diagonal of Sv, length L

	// square-root of measurement noise covariance
	float* pSnDiag; // only the diagonal of Sn, length N

	// internal state
	arm_matrix_instance_f32 x;	// (L x 1)

	// control input
	arm_matrix_instance_f32 u;	// (U x 1)

	// sensor input
	arm_matrix_instance_f32 y;	// (N x 1)

	// square-root of state covariance
	arm_matrix_instance_f32 Sx;	// (L x L)

	// square-root of measurement covariance
	arm_matrix_instance_f32 Sy;	// (N x N)

	// cross covariance
	arm_matrix_instance_f32 Pxy;// (L x N)

	// sigma points
	arm_matrix_instance_f32 Xk;	// (numSigma x L) // original = (L x numSigma)
	arm_matrix_instance_f32 Yk; // (numSigma x N) // original = (N x numSigma)

	// propagated means
	arm_matrix_instance_f32 xk;	// (L x 1)
	arm_matrix_instance_f32 yk;	// (N x 1)

	// Kalman gain
	arm_matrix_instance_f32 K;	// (L x N)

	// temp data
	arm_matrix_instance_f32 tmp;
	arm_matrix_instance_f32 tmp2;
	arm_matrix_instance_f32 qr; // (3*max x max)

	uint32_t* pUpdate;		// N x 1

	uint32_t* pAngularState;	// L x 1
	uint32_t* pAngularMeas;		// N x 1
} SRUKF;

void SRUKFInit(SRUKF* pUKF, uint16_t numStates, uint16_t numMeas, uint16_t numCtrl, float* pData,
		SRUKFStateFunc stateFunc, SRUKFMeasFunc measFunc, float* stateNoiseDiag, float* measNoiseDiag);
void SRUKFConfigure(SRUKF* pUKF, float alpha, float beta, float ki);
void SRUKFSetProcessNoise(SRUKF* pUKF, float* pQ);
void SRUKFSetMeasurementNoise(SRUKF* pUKF, float* pR);
void SRUKFPredict(SRUKF* pUKF);
void SRUKFUpdate(SRUKF* pUKF);

#endif /* SRUKF_H_ */
