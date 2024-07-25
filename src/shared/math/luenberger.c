/*
 * luenberger.c
 *
 *  Created on: 24.02.2015
 *      Author: AndreR
 */

#include "luenberger.h"

void LuenbergerInit(Luenberger* pLB, uint16_t f, uint16_t g, uint16_t h, float* pData)
{
	uint16_t max = f;
	if(g > max)
		max = g;
	if(h > max)
		max = h;

	memset(pData, 0, LB_DATA_SIZE(f, g, h, max)*sizeof(float));

	pLB->f = f;
	pLB->g = g;
	pLB->h = h;

	pLB->A = (arm_matrix_instance_f32){f, f, pData};
	pData += LB_SIZE_A(f);

	pLB->B = (arm_matrix_instance_f32){f, g, pData};
	pData += LB_SIZE_B(f, g);

	pLB->C = (arm_matrix_instance_f32){h, f, pData};
	pData += LB_SIZE_C(h, f);

	pLB->Astar = (arm_matrix_instance_f32){f, f, pData};
	pData += LB_SIZE_A(f);

	pLB->l = (arm_matrix_instance_f32){f, h, pData};
	pData += LB_SIZE_L(f, h);

	pLB->x = (arm_matrix_instance_f32){f, 1, pData};
	pData += LB_SIZE_X(f);

	pLB->u = (arm_matrix_instance_f32){g, 1, pData};
	pData += LB_SIZE_U(g);

	pLB->z = (arm_matrix_instance_f32){h, 1, pData};
	pData += LB_SIZE_Z(h);

	pLB->tmp1 = (arm_matrix_instance_f32){max, max, pData};
	pData += LB_SIZE_MAX(max);

	pLB->tmp2 = (arm_matrix_instance_f32){max, max, pData};
	pData += LB_SIZE_MAX(max);

	pLB->tmp3 = (arm_matrix_instance_f32){max, max, pData};
	pData += LB_SIZE_MAX(max);
}

void LuenbergerUpdate(Luenberger* pLB)
{
	// >>> Astar = A - L*C
	// tmp1 = L*C
	pLB->tmp1.numRows = pLB->f;
	pLB->tmp1.numCols = pLB->f;
	arm_mat_mult_f32(&pLB->l, &pLB->C, &pLB->tmp1);
	// Astar = A - tmp1
	arm_mat_sub_f32(&pLB->A, &pLB->tmp1, &pLB->Astar);

	// >>> x = A*x + B*u + L*z
	// tmp1 = A*x
	pLB->tmp1.numRows = pLB->f;
	pLB->tmp1.numCols = 1;
	arm_mat_mult_f32(&pLB->A, &pLB->x, &pLB->tmp1);

	// tmp2 = B*u
	pLB->tmp2.numRows = pLB->f;
	pLB->tmp2.numCols = 1;
	arm_mat_mult_f32(&pLB->B, &pLB->u, &pLB->tmp2);

	// tmp3 = L*z
	pLB->tmp3.numRows = pLB->f;
	pLB->tmp3.numCols = 1;
	arm_mat_mult_f32(&pLB->l, &pLB->z, &pLB->tmp3);

	// x = tmp1 + tmp2
	arm_mat_mult_f32(&pLB->tmp1, &pLB->tmp2, &pLB->x);

	// x = x + tmp3
	arm_mat_mult_f32(&pLB->x, &pLB->tmp3, &pLB->x);
}

void LuenbergerSISO2SInit(LuenbergerSISO2S* pLB, const float* pA, const float* pB, const float* pC, const float* pL)
{
	pLB->pB = pB;
	pLB->pL = pL;

	pLB->Astar[0] = pA[0] - pL[0]*pC[0];
	pLB->Astar[1] = pA[1] - pL[0]*pC[1];
	pLB->Astar[2] = pA[2] - pL[1]*pC[0];
	pLB->Astar[3] = pA[3] - pL[1]*pC[1];

	pLB->x[0] = 0;
	pLB->x[1] = 0;
}

void LuenbergerSISO2SUpdate(LuenbergerSISO2S* pLB, const float output, const float sensor)
{
	const float* B = pLB->pB;
	const float* L = pLB->pL;
	const float* As = pLB->Astar;
	float u = output;
	float* x = pLB->x;
	float z = sensor;

	x[0] = As[0]*x[0]+As[1]*x[1] + B[0]*u + L[0]*z;
	x[1] = As[2]*x[0]+As[3]*x[1] + B[1]*u + L[1]*z;
}
