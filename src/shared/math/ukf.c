/*
 * ukf.c
 *
 *  Created on: 14.06.2015
 *      Author: AndreR
 */

#include "ukf.h"
#include "arm_mat_util_f32.h"

void UKFInit(UKF* pUKF, uint16_t numStates, uint16_t numMeas, uint16_t numCtrl, float* pData, UKFStateFunc stateFunc, UKFMeasFunc measFunc)
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

	memset(pData, 0, UKF_DATA_SIZE(L, N, U, max));

	pUKF->pState = stateFunc;
	pUKF->pMeas = measFunc;
	pUKF->L = L;
	pUKF->N = N;
	pUKF->U = U;
	pUKF->S = 2*L+1;

	pUKF->Wm = (arm_matrix_instance_f32){1, S, pData};
	pData += UKF_SIZE_WM(L);

	pUKF->Wc = (arm_matrix_instance_f32){1, S, pData};
	pData += UKF_SIZE_WC(L);

	pUKF->Q = (arm_matrix_instance_f32){L, L, pData};
	pData += UKF_SIZE_Q(L);

	pUKF->R = (arm_matrix_instance_f32){N, N, pData};
	pData += UKF_SIZE_R(N);

	pUKF->x = (arm_matrix_instance_f32){L, 1, pData};
	pData += UKF_SIZE_X(L);

	pUKF->u = (arm_matrix_instance_f32){U, 1, pData};
	pData += UKF_SIZE_U(U);

	pUKF->y = (arm_matrix_instance_f32){N, 1, pData};
	pData += UKF_SIZE_Y(N);

	pUKF->Px = (arm_matrix_instance_f32){L, L, pData};
	pData += UKF_SIZE_PX(L);

	pUKF->Py = (arm_matrix_instance_f32){N, N, pData};
	pData += UKF_SIZE_PY(N);

	pUKF->Pxy = (arm_matrix_instance_f32){L, N, pData};
	pData += UKF_SIZE_PXY(L, N);

	pUKF->PxyT = (arm_matrix_instance_f32){N, L, pData};
	pData += UKF_SIZE_PXY(L, N);

	pUKF->Xk = (arm_matrix_instance_f32){S, L, pData};
	pData += UKF_SIZE_XK1(L);

	pUKF->Yk = (arm_matrix_instance_f32){S, N, pData};
	pData += UKF_SIZE_YK1(L, N);

	pUKF->xk = (arm_matrix_instance_f32){L, 1, pData};
	pData += UKF_SIZE_XK2(L);

	pUKF->yk = (arm_matrix_instance_f32){N, 1, pData};
	pData += UKF_SIZE_YK2(N);

	pUKF->K = (arm_matrix_instance_f32){L, N, pData};
	pData += UKF_SIZE_K(L, N);

	pUKF->tmp = (arm_matrix_instance_f32){max, max, pData};
	pData += UKF_SIZE_TMP(max);

	pUKF->tmp2 = (arm_matrix_instance_f32){max, max, pData};
	pData += UKF_SIZE_TMP(max);

	pUKF->pNoUpdate = (uint8_t*)pData;

	arm_mat_identity_f32(&pUKF->Q);
	arm_mat_identity_f32(&pUKF->R);
	arm_mat_identity_f32(&pUKF->Px);

	UKFConfigure(pUKF, 0.1f, 2, 0);
}

void UKFConfigure(UKF* pUKF, float alpha, float beta, float ki)
{
	pUKF->alpha = alpha;
	pUKF->beta = beta;
	pUKF->ki = ki;

	float lambda = alpha*alpha*(pUKF->L+ki) - pUKF->L;
	float c = pUKF->L+lambda;
	pUKF->c = sqrtf(c);

	MAT_ELEMENT(pUKF->Wm, 0, 0) = lambda/c;
	MAT_ELEMENT(pUKF->Wc, 0, 0) = lambda/c + (1-alpha*alpha+beta);

	for(uint16_t i = 1; i < pUKF->S; i++)
	{
		MAT_ELEMENT(pUKF->Wm, 0, i) = 0.5f/c;
		MAT_ELEMENT(pUKF->Wc, 0, i) = 0.5f/c;
	}
}

void UKFPredict(UKF* pUKF)
{
	pUKF->tmp.numRows = pUKF->Px.numRows;
	pUKF->tmp.numCols = pUKF->Px.numCols;

	// cholesky decomposition to tmp
	arm_mat_chol(&pUKF->Px, &pUKF->tmp);

	// scale with c
	arm_mat_scale_f32(&pUKF->tmp, pUKF->c, &pUKF->tmp);

	// transpose for more efficient access
	arm_mat_trans_f32(&pUKF->tmp, &pUKF->Px);

	// fill Xk with x
	for(uint16_t i = 0; i < pUKF->S; i++)
	{
		memcpy(arm_mat_row_ptr(&pUKF->Xk, i), pUKF->x.pData, sizeof(float)*pUKF->L);
	}

	// add and subtract cholesky of Px once
	arm_matrix_instance_f32 ap = {pUKF->L, pUKF->L, arm_mat_row_ptr(&pUKF->Xk, 1)};
	arm_matrix_instance_f32 am = {pUKF->L, pUKF->L, arm_mat_row_ptr(&pUKF->Xk, pUKF->L+1)};
	arm_mat_add_f32(&ap, &pUKF->Px, &ap);
	arm_mat_sub_f32(&am, &pUKF->Px, &am);

	// now call state function with sigma points
	if(pUKF->pState)
	{
		for(uint16_t i = 0; i < pUKF->S; i++)
		{
			arm_matrix_instance_f32 x = {pUKF->L, 1, arm_mat_row_ptr(&pUKF->Xk, i)};
			(*pUKF->pState)(&x, &pUKF->u);
		}
	}

	// calculate mean
	arm_mat_zero_f32(&pUKF->xk);
	for(uint16_t i = 0; i < pUKF->S; i++)
	{
		for(uint16_t c = 0; c < pUKF->L; c++)
		{
			MAT_ELEMENT(pUKF->xk, c, 0) += MAT_ELEMENT(pUKF->Wm, 0, i)*MAT_ELEMENT(pUKF->Xk, i, c);
		}
	}

	// calculate covariance
	pUKF->tmp.numRows = pUKF->L;
	pUKF->tmp.numCols = 1;
	arm_mat_zero_f32(&pUKF->Px);
	for(uint16_t i = 0; i < pUKF->S; i++)
	{
		arm_matrix_instance_f32 Xi = {pUKF->L, 1, arm_mat_row_ptr(&pUKF->Xk, i)};
		arm_mat_sub_f32(&Xi, &pUKF->xk, &pUKF->tmp);
		float* xm = pUKF->tmp.pData;

		for(uint16_t r = 0; r < pUKF->L; r++)
		{
			for(uint16_t c = 0; c < pUKF->L; c++)
			{
				MAT_ELEMENT(pUKF->Px, r, c) += MAT_ELEMENT(pUKF->Wc, 0, i)*xm[r]*xm[c];
			}
		}
	}

	arm_mat_add_f32(&pUKF->Px, &pUKF->Q, &pUKF->Px);
}

void UKFPredictNoStateFunc(UKF* pUKF)
{
	pUKF->tmp.numRows = pUKF->Px.numRows;
	pUKF->tmp.numCols = pUKF->Px.numCols;
	pUKF->tmp2.numRows = pUKF->Px.numRows;
	pUKF->tmp2.numCols = pUKF->Px.numCols;

	// cholesky decomposition to tmp
	arm_mat_chol(&pUKF->Px, &pUKF->tmp);

	// scale with c
	arm_mat_scale_f32(&pUKF->tmp, pUKF->c, &pUKF->tmp);

	// transpose for more efficient access
	arm_mat_trans_f32(&pUKF->tmp, &pUKF->tmp2);

	// fill Xk with x
	for(uint16_t i = 0; i < pUKF->S; i++)
	{
		memcpy(arm_mat_row_ptr(&pUKF->Xk, i), pUKF->x.pData, sizeof(float)*pUKF->L);
	}

	// add and subtract cholesky of Px once
	arm_matrix_instance_f32 ap = {pUKF->L, pUKF->L, arm_mat_row_ptr(&pUKF->Xk, 1)};
	arm_matrix_instance_f32 am = {pUKF->L, pUKF->L, arm_mat_row_ptr(&pUKF->Xk, pUKF->L+1)};
	arm_mat_add_f32(&ap, &pUKF->tmp2, &ap);
	arm_mat_sub_f32(&am, &pUKF->tmp2, &am);

	arm_mat_copy_f32(&pUKF->x, &pUKF->xk);

	// calculate covariance
	arm_mat_add_f32(&pUKF->Px, &pUKF->Q, &pUKF->Px);
}

void UKFUpdate(UKF* pUKF)
{
	// resample sigma points, alternative approach
	pUKF->tmp.numRows = pUKF->Px.numRows;
	pUKF->tmp.numCols = pUKF->Px.numCols;
	pUKF->tmp2.numRows = pUKF->Px.numRows;
	pUKF->tmp2.numCols = pUKF->Px.numCols;

	// cholesky decomposition to tmp
	arm_mat_chol(&pUKF->Px, &pUKF->tmp);

	// scale with c
	arm_mat_scale_f32(&pUKF->tmp, pUKF->c, &pUKF->tmp);

	// transpose for more efficient access
	arm_mat_trans_f32(&pUKF->tmp, &pUKF->tmp2);

	// fill Xk with x
	for(uint16_t i = 0; i < pUKF->S; i++)
	{
		memcpy(arm_mat_row_ptr(&pUKF->Xk, i), pUKF->x.pData, sizeof(float)*pUKF->L);
	}

	// add and subtract cholesky of Px once
	arm_matrix_instance_f32 ap = {pUKF->L, pUKF->L, arm_mat_row_ptr(&pUKF->Xk, 1)};
	arm_matrix_instance_f32 am = {pUKF->L, pUKF->L, arm_mat_row_ptr(&pUKF->Xk, pUKF->L+1)};
	arm_mat_add_f32(&ap, &pUKF->tmp2, &ap);
	arm_mat_sub_f32(&am, &pUKF->tmp2, &am);

	// call measurement function with sigma points
	for(uint16_t i = 0; i < pUKF->S; i++)
	{
		arm_matrix_instance_f32 x = {pUKF->L, 1, arm_mat_row_ptr(&pUKF->Xk, i)};
		arm_matrix_instance_f32 y = {pUKF->N, 1, arm_mat_row_ptr(&pUKF->Yk, i)};
		(*pUKF->pMeas)(&x, &pUKF->u, &y);
	}

	// calculate mean
	arm_mat_zero_f32(&pUKF->yk);
	for(uint16_t i = 0; i < pUKF->S; i++)
	{
		for(uint16_t c = 0; c < pUKF->N; c++)
		{
			MAT_ELEMENT(pUKF->yk, c, 0) += MAT_ELEMENT(pUKF->Wm, 0, i)*MAT_ELEMENT(pUKF->Yk, i, c);
		}
	}

	// calculate covariance
	pUKF->tmp.numRows = pUKF->N;
	pUKF->tmp.numCols = 1;
	arm_mat_zero_f32(&pUKF->Py);
	for(uint16_t i = 0; i < pUKF->S; i++)
	{
		arm_matrix_instance_f32 Yi = {pUKF->N, 1, arm_mat_row_ptr(&pUKF->Yk, i)};
		arm_mat_sub_f32(&Yi, &pUKF->yk, &pUKF->tmp);
		float* ym = pUKF->tmp.pData;

		for(uint16_t r = 0; r < pUKF->N; r++)
		{
			for(uint16_t c = 0; c < pUKF->N; c++)
			{
				MAT_ELEMENT(pUKF->Py, r, c) += MAT_ELEMENT(pUKF->Wc, 0, i)*ym[r]*ym[c];
			}
		}
	}

	arm_mat_add_f32(&pUKF->Py, &pUKF->R, &pUKF->Py);

	// calculate cross-covariance
	arm_matrix_instance_f32 matYm = {pUKF->N, 1, pUKF->tmp.pData};
	arm_matrix_instance_f32 matXm = {pUKF->L, 1, pUKF->tmp.pData+pUKF->N};
	arm_mat_zero_f32(&pUKF->Pxy);
	for(uint16_t i = 0; i < pUKF->S; i++)
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

	// calculate Kalman gain
	pUKF->tmp.numRows = pUKF->N;
	pUKF->tmp.numCols = pUKF->N;
	pUKF->tmp2.numRows = pUKF->L;
	pUKF->tmp2.numCols = pUKF->N;

	arm_mat_inverse_f32(&pUKF->Py, &pUKF->tmp);

	arm_mat_mult_f32(&pUKF->Pxy, &pUKF->tmp, &pUKF->K);

	arm_mat_mult_f32(&pUKF->Pxy, &pUKF->tmp, &pUKF->tmp2);

//	nanCheck(&pUKF->tmp2, 8);

	pUKF->tmp.numRows = pUKF->L;
	pUKF->tmp.numCols = pUKF->L;

	arm_mat_identity_f32(&pUKF->tmp);

	for(uint16_t i = 0; i < pUKF->L; i++)
	{
		if(pUKF->pNoUpdate[i])
		{
			MAT_ELEMENT(pUKF->tmp, i, i) = 0;
		}
	}

	arm_mat_mult_f32(&pUKF->tmp, &pUKF->tmp2, &pUKF->K);

	// calculate new estimate
	pUKF->tmp.numRows = pUKF->L;
	pUKF->tmp.numCols = 1;

	arm_mat_sub_f32(&pUKF->y, &pUKF->yk, &pUKF->yk);

	arm_mat_mult_f32(&pUKF->K, &pUKF->yk, &pUKF->tmp);

	arm_mat_add_f32(&pUKF->xk, &pUKF->tmp, &pUKF->x);

	// calculate new state covariance
	pUKF->tmp.numRows = pUKF->L;
	pUKF->tmp.numCols = pUKF->L;

	arm_mat_trans_f32(&pUKF->Pxy, &pUKF->PxyT);

	arm_mat_mult_f32(&pUKF->K, &pUKF->PxyT, &pUKF->tmp);

	arm_mat_sub_f32(&pUKF->Px, &pUKF->tmp, &pUKF->Px);
}
