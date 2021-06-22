/*
 * srukf.c
 *
 *  Created on: 28.03.2016
 *      Author: AndreR
 */

#include "srukf.h"
#include "arm_mat_util_f32.h"
#include "angle_math.h"

void SRUKFInit(SRUKF* pUKF, uint16_t numStates, uint16_t numMeas, uint16_t numCtrl, float* pData,
		SRUKFStateFunc stateFunc, SRUKFMeasFunc measFunc, float* stateNoiseDiag, float* measNoiseDiag)
{
	uint16_t L = numStates;
	uint16_t N = numMeas;
	uint16_t U = numCtrl;
	uint16_t S = 2*L+1;

	uint16_t max = L;
	if(N > max)
		max = N;
	if(U > max)
		max = U;

	memset(pData, 0, SRUKF_DATA_SIZE(L, N, U, max));

	pUKF->pState = stateFunc;
	pUKF->pMeas = measFunc;
	pUKF->L = L;
	pUKF->N = N;
	pUKF->U = U;
	pUKF->numSigma = 2*L+1;

	pUKF->Wm = (arm_matrix_instance_f32){1, S, pData};
	pData += SRUKF_SIZE_WM(L);

	pUKF->Wc = (arm_matrix_instance_f32){1, S, pData};
	pData += SRUKF_SIZE_WC(L);

	pUKF->pSvDiag = pData;
	pData += SRUKF_SIZE_SV(L);

	pUKF->pSnDiag = pData;
	pData += SRUKF_SIZE_SN(N);

	pUKF->x = (arm_matrix_instance_f32){L, 1, pData};
	pData += SRUKF_SIZE_X(L);

	pUKF->u = (arm_matrix_instance_f32){U, 1, pData};
	pData += SRUKF_SIZE_U(U);

	pUKF->y = (arm_matrix_instance_f32){N, 1, pData};
	pData += SRUKF_SIZE_Y(N);

	pUKF->Sx = (arm_matrix_instance_f32){L, L, pData};
	pData += SRUKF_SIZE_SX(L);

	pUKF->Sy = (arm_matrix_instance_f32){N, N, pData};
	pData += SRUKF_SIZE_SY(N);

	pUKF->Pxy = (arm_matrix_instance_f32){L, N, pData};
	pData += SRUKF_SIZE_PXY(L, N);

	pUKF->Xk = (arm_matrix_instance_f32){S, L, pData};
	pData += SRUKF_SIZE_XK1(L);

	pUKF->Yk = (arm_matrix_instance_f32){S, N, pData};
	pData += SRUKF_SIZE_YK1(L, N);

	pUKF->xk = (arm_matrix_instance_f32){L, 1, pData};
	pData += SRUKF_SIZE_XK2(L);

	pUKF->yk = (arm_matrix_instance_f32){N, 1, pData};
	pData += SRUKF_SIZE_YK2(N);

	pUKF->K = (arm_matrix_instance_f32){L, N, pData};
	pData += SRUKF_SIZE_K(L, N);

	pUKF->tmp = (arm_matrix_instance_f32){max, max, pData};
	pData += SRUKF_SIZE_TMP(max);

	pUKF->tmp2 = (arm_matrix_instance_f32){max, max, pData};
	pData += SRUKF_SIZE_TMP(max);

	pUKF->qr = (arm_matrix_instance_f32){max, max, pData};
	pData += SRUKF_SIZE_TMP(max);

	pUKF->pUpdate = (uint32_t*)pData;
	memset(pUKF->pUpdate, 1, sizeof(uint32_t)*pUKF->N);
	pData += pUKF->N;

	pUKF->pAngularState = (uint32_t*)pData;
	memset(pUKF->pAngularState, 0, sizeof(uint32_t)*pUKF->L);
	pData += pUKF->L;

	pUKF->pAngularMeas = (uint32_t*)pData;
	memset(pUKF->pAngularMeas, 0, sizeof(uint32_t)*pUKF->N);

	SRUKFSetProcessNoise(pUKF, stateNoiseDiag);
	SRUKFSetMeasurementNoise(pUKF, measNoiseDiag);

	for(uint16_t i = 0; i < numStates; i++)
		MAT_ELEMENT(pUKF->Sx, i, i) = pUKF->pSvDiag[i];

	SRUKFConfigure(pUKF, 0.1f, 2, 0);
}

void SRUKFConfigure(SRUKF* pUKF, float alpha, float beta, float ki)
{
	pUKF->alpha = alpha;
	pUKF->beta = beta;
	pUKF->ki = ki;

	float lambda = alpha*alpha*(pUKF->L+ki) - pUKF->L;
	float c = pUKF->L+lambda;
	pUKF->gamma = sqrtf(c);

	MAT_ELEMENT(pUKF->Wm, 0, 0) = lambda/c;
	MAT_ELEMENT(pUKF->Wc, 0, 0) = lambda/c + (1-alpha*alpha+beta);

	for(uint16_t i = 1; i < pUKF->numSigma; i++)
	{
		MAT_ELEMENT(pUKF->Wm, 0, i) = 0.5f/c;
		MAT_ELEMENT(pUKF->Wc, 0, i) = 0.5f/c;
	}
}

void SRUKFSetProcessNoise(SRUKF* pUKF, float* pQ)
{
	for(uint16_t i = 0; i < pUKF->L; i++)
		pUKF->pSvDiag[i] = sqrtf(pQ[i]);
}

void SRUKFSetMeasurementNoise(SRUKF* pUKF, float* pR)
{
	for(uint16_t i = 0; i < pUKF->N; i++)
		pUKF->pSnDiag[i] = sqrtf(pR[i]);
}

void SRUKFPredict(SRUKF* pUKF)
{
	// >>> (17) construct sigma points

	// fill Xk
	// first row
	memcpy(pUKF->Xk.pData, pUKF->x.pData, sizeof(float)*pUKF->L);

	// fill other rows, add and subtract Sx once and transpose
	for(uint16_t i = 0; i < pUKF->L; i++)
	{
		for(uint16_t n = 0; n < pUKF->L; n++)
		{
			MAT_ELEMENT(pUKF->Xk, i+1, n) = pUKF->x.pData[n] + pUKF->gamma*MAT_ELEMENT(pUKF->Sx, n, i);
			MAT_ELEMENT(pUKF->Xk, pUKF->L+i+1, n) = pUKF->x.pData[n] - pUKF->gamma*MAT_ELEMENT(pUKF->Sx, n, i);
		}
	}

	// >>> (18) call state function with sigma points, Xk will be updated
	if(pUKF->pState)
	{
		for(uint16_t i = 0; i < pUKF->numSigma; i++)
		{
			arm_matrix_instance_f32 x = {pUKF->L, 1, arm_mat_row_ptr(&pUKF->Xk, i)};
			(*pUKF->pState)(&x, &pUKF->u);
		}
	}

	// >>> (19) calculate mean
	arm_mat_zero_f32(&pUKF->xk);
	for(uint16_t i = 0; i < pUKF->numSigma; i++)
	{
		for(uint16_t c = 0; c < pUKF->L; c++)
		{
			MAT_ELEMENT(pUKF->xk, c, 0) += MAT_ELEMENT(pUKF->Wm, 0, i)*MAT_ELEMENT(pUKF->Xk, i, c);
		}
	}

	// >>> (20) calculate new square-root state covariance

	// form matrix for QR decomposition
	pUKF->qr.numRows = 3*pUKF->L;
	pUKF->qr.numCols = pUKF->L;
	float sqWc1 = sqrtf(pUKF->Wc.pData[1]);
	for(uint16_t r = 0; r < 2*pUKF->L; r++)
	{
		for(uint16_t c = 0; c < pUKF->L; c++)
		{
			MAT_ELEMENT(pUKF->qr, r, c) = sqWc1*(MAT_ELEMENT(pUKF->Xk, r+1, c) - pUKF->xk.pData[c]);
		}
	}

	memset(&MAT_ELEMENT(pUKF->qr, 2*pUKF->L, 0), 0, sizeof(float)*pUKF->L*pUKF->L);
	for(uint16_t d = 0; d < pUKF->L; d++)
	{
		MAT_ELEMENT(pUKF->qr, 2*pUKF->L+d, d) = pUKF->pSvDiag[d];
	}

	pUKF->tmp.numRows = 3*pUKF->L;
	pUKF->tmp.numCols = 3*pUKF->L;
	pUKF->tmp2.numRows = 3*pUKF->L;
	pUKF->tmp2.numCols = 3*pUKF->L;

	// perform decomposition, result is in upper part of qr
	arm_mat_qr(&pUKF->qr, 0, &pUKF->qr, &pUKF->tmp, &pUKF->tmp2);

	pUKF->qr.numRows = pUKF->L;
	pUKF->qr.numCols = pUKF->L;

	// >>> (21) Cholesky update of Sx
	float sqWc0 = sqrtf(fabsf(pUKF->Wc.pData[0]));
	for(uint16_t i = 0; i < pUKF->L; i++)
		pUKF->tmp.pData[i] = sqWc0*(MAT_ELEMENT(pUKF->Xk, 0, i) - pUKF->xk.pData[i]);

	if(pUKF->Wc.pData[0] < 0)
		arm_mat_choldowndate(&pUKF->qr, pUKF->tmp.pData);
	else
		arm_mat_cholupdate(&pUKF->qr, pUKF->tmp.pData);

	// change to lower triangular form
	arm_mat_trans_f32(&pUKF->qr, &pUKF->Sx);
}

void SRUKFUpdate(SRUKF* pUKF)
{
	// >>> (22) resample sigma points

	// fill Xk
	// first row
	memcpy(pUKF->Xk.pData, pUKF->x.pData, sizeof(float)*pUKF->L);

	// fill other rows, add and subtract Sx once and transpose
	for(uint16_t i = 0; i < pUKF->L; i++)
	{
		for(uint16_t n = 0; n < pUKF->L; n++)
		{
			MAT_ELEMENT(pUKF->Xk, i+1, n) = pUKF->x.pData[n] + pUKF->gamma*MAT_ELEMENT(pUKF->Sx, n, i);
			MAT_ELEMENT(pUKF->Xk, pUKF->L+i+1, n) = pUKF->x.pData[n] - pUKF->gamma*MAT_ELEMENT(pUKF->Sx, n, i);
		}
	}

	// >>> (23) call measurement function with sigma points
	for(uint16_t i = 0; i < pUKF->numSigma; i++)
	{
		arm_matrix_instance_f32 x = {pUKF->L, 1, arm_mat_row_ptr(&pUKF->Xk, i)};
		arm_matrix_instance_f32 y = {pUKF->N, 1, arm_mat_row_ptr(&pUKF->Yk, i)};
		(*pUKF->pMeas)(&x, &pUKF->u, &y);
	}

	// >>> (24) calculate mean
	arm_mat_zero_f32(&pUKF->yk);
	for(uint16_t i = 0; i < pUKF->numSigma; i++)
	{
		for(uint16_t c = 0; c < pUKF->N; c++)
		{
			MAT_ELEMENT(pUKF->yk, c, 0) += MAT_ELEMENT(pUKF->Wm, 0, i)*MAT_ELEMENT(pUKF->Yk, i, c);
		}
	}

	// >>> (25) calculate new square-root measurement covariance

	// form matrix for QR decomposition
	pUKF->qr.numRows = 2*pUKF->L+pUKF->N;
	pUKF->qr.numCols = pUKF->N;
	float sqWc1 = sqrtf(pUKF->Wc.pData[1]);
	for(uint16_t r = 0; r < 2*pUKF->L; r++)
	{
		for(uint16_t c = 0; c < pUKF->N; c++)
		{
			MAT_ELEMENT(pUKF->qr, r, c) = sqWc1*(MAT_ELEMENT(pUKF->Yk, r+1, c) - pUKF->yk.pData[c]);
		}
	}

	memset(&MAT_ELEMENT(pUKF->qr, 2*pUKF->L, 0), 0, sizeof(float)*pUKF->N*pUKF->N);
	for(uint16_t d = 0; d < pUKF->N; d++)
	{
		MAT_ELEMENT(pUKF->qr, 2*pUKF->L+d, d) = pUKF->pSnDiag[d];
	}

	pUKF->tmp.numRows = 2*pUKF->L+pUKF->N;
	pUKF->tmp.numCols = 2*pUKF->L+pUKF->N;
	pUKF->tmp2.numRows = 2*pUKF->L+pUKF->N;
	pUKF->tmp2.numCols = 2*pUKF->L+pUKF->N;

	// perform decomposition, result is in upper part of qr
	arm_mat_qr(&pUKF->qr, 0, &pUKF->qr, &pUKF->tmp, &pUKF->tmp2);

	pUKF->qr.numRows = pUKF->N;
	pUKF->qr.numCols = pUKF->N;

	// >>> (26) Cholesky update of Sy
	float sqWc0 = sqrtf(fabsf(pUKF->Wc.pData[0]));
	for(uint16_t i = 0; i < pUKF->N; i++)
		pUKF->tmp.pData[i] = sqWc0*(MAT_ELEMENT(pUKF->Yk, 0, i) - pUKF->yk.pData[i]);

	if(pUKF->Wc.pData[0] < 0)
		arm_mat_choldowndate(&pUKF->qr, pUKF->tmp.pData);
	else
		arm_mat_cholupdate(&pUKF->qr, pUKF->tmp.pData);

	// change to lower triangular form
	arm_mat_trans_f32(&pUKF->qr, &pUKF->Sy);

	// >>> (27) calculate cross-covariance
	arm_matrix_instance_f32 matYm = {pUKF->N, 1, pUKF->tmp.pData};
	arm_matrix_instance_f32 matXm = {pUKF->L, 1, pUKF->tmp.pData+pUKF->N};
	arm_mat_zero_f32(&pUKF->Pxy);
	for(uint16_t i = 0; i < pUKF->numSigma; i++)
	{
		arm_matrix_instance_f32 Yi = {pUKF->N, 1, arm_mat_row_ptr(&pUKF->Yk, i)};
		arm_mat_sub_f32(&Yi, &pUKF->yk, &matYm);
		float* ym = matYm.pData;

		arm_matrix_instance_f32 Xi = {pUKF->L, 1, arm_mat_row_ptr(&pUKF->Xk, i)};
		arm_mat_sub_f32(&Xi, &pUKF->xk, &matXm);
		float* xm = matXm.pData;

		for(uint16_t r = 0; r < pUKF->L; r++)
		{
			for(uint16_t c = 0; c < pUKF->N; c++)
			{
				MAT_ELEMENT(pUKF->Pxy, r, c) += MAT_ELEMENT(pUKF->Wc, 0, i)*xm[r]*ym[c];
			}
		}
	}

	// >>> (28) calculate Kalman gain
	pUKF->tmp.numRows = pUKF->L;
	pUKF->tmp.numCols = pUKF->N;
	pUKF->tmp2.numRows = pUKF->L;
	pUKF->tmp2.numCols = pUKF->N;
	arm_mat_tri_back_substitution(&pUKF->qr, &pUKF->Pxy, &pUKF->tmp);

	arm_mat_tri_forward_substitution(&pUKF->Sy, &pUKF->tmp, &pUKF->tmp2); // K is in tmp2

	// construct M matrix which indicates the measurements which have occurred
	pUKF->tmp.numRows = pUKF->N;
	pUKF->tmp.numCols = pUKF->N;

	arm_mat_identity_f32(&pUKF->tmp);

	for(uint16_t i = 0; i < pUKF->N; i++)
	{
		if(pUKF->pUpdate[i] == 0)
			MAT_ELEMENT(pUKF->tmp, i, i) = 0;
	}

	arm_mat_mult_f32(&pUKF->tmp2, &pUKF->tmp, &pUKF->K);

	// update state
	pUKF->tmp.numRows = pUKF->L;
	pUKF->tmp.numCols = 1;

	arm_mat_sub_f32(&pUKF->y, &pUKF->yk, &pUKF->yk);

	for(uint16_t i = 0; i < pUKF->N; i++)
	{
		if(pUKF->pAngularMeas[i])
			AngleNormalizePtr(&pUKF->yk.pData[i]);
	}

	arm_mat_mult_f32(&pUKF->K, &pUKF->yk, &pUKF->tmp);

	arm_mat_add_f32(&pUKF->xk, &pUKF->tmp, &pUKF->x);

	for(uint16_t i = 0; i < pUKF->L; i++)
	{
		if(pUKF->pAngularState[i])
			AngleNormalizePtr(&pUKF->x.pData[i]);
	}

	// >>> (29) construct U
	pUKF->tmp.numRows = pUKF->L;
	pUKF->tmp.numCols = pUKF->N;

	arm_mat_mult_f32(&pUKF->K, &pUKF->Sy, &pUKF->tmp);

	pUKF->tmp2.numRows = pUKF->N;
	pUKF->tmp2.numCols = pUKF->L;

	arm_mat_trans_f32(&pUKF->tmp, &pUKF->tmp2);

	// >>> (30) Cholesky downdates of Sx
	pUKF->tmp.numRows = pUKF->L;
	pUKF->tmp.numCols = pUKF->L;

	arm_mat_trans_f32(&pUKF->Sx, &pUKF->tmp);

	for(uint16_t i = 0; i < pUKF->N; i++)
	{
		arm_mat_choldowndate(&pUKF->tmp, arm_mat_row_ptr(&pUKF->tmp2, i));
	}

	arm_mat_trans_f32(&pUKF->tmp, &pUKF->Sx);
}
