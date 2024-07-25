/*
 * kf.c
 *
 *  Created on: 18.03.2013
 *      Author: AndreR
 *
 *  Funny enough this file is named KFC... watch out, incoming trademarks!
 */

#include "kf.h"

#include <string.h>
#include "angle_math.h"
#include "util/log.h"
#include "arm_mat_util_f32.h"

void KFInit(KF* pKF, uint16_t f, uint16_t g, uint16_t h, float* pData)
{
	uint16_t max = f;
	if(g > max)
		max = g;
	if(h > max)
		max = h;

	memset(pData, 0, KF_DATA_SIZE(f, g, h, max)*sizeof(float));

	pKF->f = f;
	pKF->g = g;
	pKF->h = h;

	pKF->A.numRows = f;
	pKF->A.numCols = f;
	pKF->A.pData = pData;
	pData += KF_SIZE_A(f);

	pKF->B.numRows = f;
	pKF->B.numCols = g;
	pKF->B.pData = pData;
	pData += KF_SIZE_B(f, g);

	pKF->C.numRows = h;
	pKF->C.numCols = f;
	pKF->C.pData = pData;
	pData += KF_SIZE_C(h, f);

	pKF->Ex.numRows = f;
	pKF->Ex.numCols = f;
	pKF->Ex.pData = pData;
	pData += KF_SIZE_EX(f);

	pKF->Ez.numRows = h;
	pKF->Ez.numCols = h;
	pKF->Ez.pData = pData;
	pData += KF_SIZE_EZ(h);

	pKF->x.numRows = f;
	pKF->x.numCols = 1;
	pKF->x.pData = pData;
	pData += KF_SIZE_X(f);

	pKF->Sigma.numRows = f;
	pKF->Sigma.numCols = f;
	pKF->Sigma.pData = pData;
	pData += KF_SIZE_SIGMA(f);

	pKF->K.numRows = f;
	pKF->K.numCols = h;
	pKF->K.pData = pData;
	pData += KF_SIZE_K(f,h);

	pKF->u.numRows = g;
	pKF->u.numCols = 1;
	pKF->u.pData = pData;
	pData += KF_SIZE_U(g);

	pKF->z.numRows = h;
	pKF->z.numCols = 1;
	pKF->z.pData = pData;
	pData += KF_SIZE_Z(h);

	pKF->tmp1.numRows = max;
	pKF->tmp1.numCols = max;
	pKF->tmp1.pData = pData;
	pData += KF_SIZE_MAX(max);

	pKF->tmp2.numRows = max;
	pKF->tmp2.numCols = max;
	pKF->tmp2.pData = pData;
	pData += KF_SIZE_MAX(max);

	pKF->tmp3.numRows = max;
	pKF->tmp3.numCols = max;
	pKF->tmp3.pData = pData;
	pData += KF_SIZE_MAX(max);

	pKF->pNormalizeAngle = (uint32_t*)pData;
	pData += KF_SIZE_X(f);

	arm_mat_identity_f32(&pKF->Sigma);
}

void KFSetOrientationComponent(KF* pKF, uint32_t stateIndex)
{
	pKF->pNormalizeAngle[stateIndex] = 1;
}

void KFStartLookAhead(KF* pKF)
{
	// save state
	pKF->tmp3.numRows = pKF->f;
	pKF->tmp3.numCols = 1;
	arm_mat_copy_f32(&pKF->x, &pKF->tmp3);
}

void KFDoLookAhead(KF* pKF)
{
	// >>> x = A*x + B*u
	// tmp1 = A*x (f x 1)
	pKF->tmp1.numRows = pKF->f;
	pKF->tmp1.numCols = 1;
	arm_mat_mult_f32(&pKF->A, &pKF->x, &pKF->tmp1);
	// tmp2 = B*u (f x 1)
	pKF->tmp2.numRows = pKF->f;
	pKF->tmp2.numCols = 1;
	arm_mat_mult_f32(&pKF->B, &pKF->u, &pKF->tmp2);
	// x = tmp1 + tmp2 (f x 1)
	arm_mat_add_f32(&pKF->tmp1, &pKF->tmp2, &pKF->x);
}

void KFFinishLookAhead(KF* pKF)
{
	arm_mat_copy_f32(&pKF->tmp3, &pKF->x);
}

void KFPredict(KF* pKF)
{
	// >>> x = A*x + B*u
	// tmp1 = A*x (f x 1)
	pKF->tmp1.numRows = pKF->f;
	pKF->tmp1.numCols = 1;
	arm_mat_mult_f32(&pKF->A, &pKF->x, &pKF->tmp1);
	// tmp2 = B*u (f x 1)
	pKF->tmp2.numRows = pKF->f;
	pKF->tmp2.numCols = 1;
	arm_mat_mult_f32(&pKF->B, &pKF->u, &pKF->tmp2);
	// x = tmp1 + tmp2 (f x 1)
	arm_mat_add_f32(&pKF->tmp1, &pKF->tmp2, &pKF->x);

	// >>> Sigma = A*Sigma*A^T + Ex
	// tmp1 = A*Sigma (f x f)
	pKF->tmp1.numRows = pKF->f;
	pKF->tmp1.numCols = pKF->f;
	arm_mat_mult_f32(&pKF->A, &pKF->Sigma, &pKF->tmp1);
	// tmp2 = A^T (f x f)
	pKF->tmp2.numRows = pKF->f;
	pKF->tmp2.numCols = pKF->f;
	arm_mat_trans_f32(&pKF->A, &pKF->tmp2);
	// Sigma = tmp1*tmp2 (f x f)
	arm_mat_mult_f32(&pKF->tmp1, &pKF->tmp2, &pKF->Sigma);
	// Sigma = Sigma + Ex (f x f)
	arm_mat_add_f32(&pKF->Sigma, &pKF->Ex, &pKF->Sigma);
}

void KFUpdate(KF* pKF)
{
	// >>> K = Sigma*C^T*(C*Sigma*C^T+Ez)^-1
	// tmp1 = C^T (f x h)
	pKF->tmp1.numRows = pKF->f;
	pKF->tmp1.numCols = pKF->h;
	arm_mat_trans_f32(&pKF->C, &pKF->tmp1);
	// tmp2 = Sigma*tmp1 (f x h)
	pKF->tmp2.numRows = pKF->f;
	pKF->tmp2.numCols = pKF->h;
	arm_mat_mult_f32(&pKF->Sigma, &pKF->tmp1, &pKF->tmp2);
	// tmp3 = C*tmp2 (h x h)
	pKF->tmp3.numRows = pKF->h;
	pKF->tmp3.numCols = pKF->h;
	arm_mat_mult_f32(&pKF->C, &pKF->tmp2, &pKF->tmp3);
	// tmp3 = tmp3+Ez (h x h)
	arm_mat_add_f32(&pKF->tmp3, &pKF->Ez, &pKF->tmp3);
	// tmp1 = tmp3^-1 (h x h)
	pKF->tmp1.numRows = pKF->h;
	pKF->tmp1.numCols = pKF->h;
	arm_status invStat;
	switch(pKF->h)
	{
		case 1:
			if(pKF->tmp3.pData[0] == 0.0f)
			{
				invStat = ARM_MATH_SINGULAR;
			}
			else
			{
				pKF->tmp1.pData[0] = 1.0f/pKF->tmp3.pData[0];
				invStat = ARM_MATH_SUCCESS;
			}
			break;
		case 2:
			invStat = arm_mat_inv_2x2_f32(&pKF->tmp3, &pKF->tmp1);
			break;
		case 3:
			invStat = arm_mat_inv_3x3_f32(&pKF->tmp3, &pKF->tmp1);
			break;
		default:
			invStat = arm_mat_inverse_f32(&pKF->tmp3, &pKF->tmp1);
			break;
	}
	if(invStat != ARM_MATH_SUCCESS)
		LogErrorC("Matrix inverse error", invStat);
	// K = tmp2*tmp1 (f x h)
	arm_mat_mult_f32(&pKF->tmp2, &pKF->tmp1, &pKF->K);

	// >>> x = x + K*(z - C*x)
	// tmp1 = C*x (h x 1)
	pKF->tmp1.numRows = pKF->h;
	pKF->tmp1.numCols = 1;
	arm_mat_mult_f32(&pKF->C, &pKF->x, &pKF->tmp1);
	// tmp2 = z - tmp1 (h x 1)
	pKF->tmp2.numRows = pKF->h;
	pKF->tmp2.numCols = 1;
	arm_mat_sub_f32(&pKF->z, &pKF->tmp1, &pKF->tmp2);
	// normalize angle of selected variables
	for(uint16_t i = 0; i < pKF->f; i++)
	{
		if(pKF->pNormalizeAngle[i])
			AngleNormalizePtr(&pKF->tmp2.pData[i]);
	}
	// tmp3 = K*tmp2 (f x 1)
	pKF->tmp3.numRows = pKF->f;
	pKF->tmp3.numCols = 1;
	arm_mat_mult_f32(&pKF->K, &pKF->tmp2, &pKF->tmp3);
	// x = x + tmp3 (f x 1)
	arm_mat_add_f32(&pKF->x, &pKF->tmp3, &pKF->x);

	// >>> Sigma = (I - K*C)*Sigma
	// tmp1 = K*C (f x f)
	pKF->tmp1.numRows = pKF->f;
	pKF->tmp1.numCols = pKF->f;
	arm_mat_mult_f32(&pKF->K, &pKF->C, &pKF->tmp1);
	// tmp2 = I (f x f)
	pKF->tmp2.numRows = pKF->f;
	pKF->tmp2.numCols = pKF->f;
	arm_mat_identity_f32(&pKF->tmp2);
	// tmp2 = tmp2 - tmp1 (f x f)
	arm_mat_sub_f32(&pKF->tmp2, &pKF->tmp1, &pKF->tmp2);
	// tmp3 = Sigma (f x f)
	arm_mat_copy_f32(&pKF->Sigma, &pKF->tmp3);
	// Sigma = tmp2*tmp3 (f x f)
	arm_mat_mult_f32(&pKF->tmp2, &pKF->tmp3, &pKF->Sigma);
}
