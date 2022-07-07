/*
 * luenberger.h
 *
 *  Created on: 24.02.2015
 *      Author: AndreR
 */

#ifndef LUENBERGER_H_
#define LUENBERGER_H_

#include "arm_math.h"

#define LB_SIZE_A(f)		(f*f)
#define LB_SIZE_B(f,g)		(f*g)
#define LB_SIZE_C(h,f)		(h*f)
#define LB_SIZE_L(f,h)		(f*h)
#define LB_SIZE_X(f)		(f)
#define LB_SIZE_U(g)		(g)
#define LB_SIZE_Z(h)		(h)
#define LB_SIZE_MAX(max)	(max*max)

#define LB_DATA_SIZE(f,g,h,max) \
	(LB_SIZE_A(f)*2 + LB_SIZE_B(f,g) + LB_SIZE_C(h,f) + \
	 LB_SIZE_L(f,h) + LB_SIZE_U(g) + LB_SIZE_Z(h) + \
	 LB_SIZE_X(f) + LB_SIZE_MAX(max)*3)

typedef struct _Luenberger
{
	uint16_t f;
	uint16_t g;
	uint16_t h;

	// user matrices
	arm_matrix_instance_f32 A;		// (f x f)
	arm_matrix_instance_f32 B;		// (f x g)
	arm_matrix_instance_f32 C;		// (h x f)

	// internal matrices
	arm_matrix_instance_f32 Astar;	// (f x f)
	arm_matrix_instance_f32 l;		// (f x h)
	arm_matrix_instance_f32 x;		// (f x 1)

	// command input
	arm_matrix_instance_f32 u;		// (g x 1)

	// sensor input
	arm_matrix_instance_f32 z;		// (h x 1)

	// temporary calculation matrices
	arm_matrix_instance_f32 tmp1;
	arm_matrix_instance_f32 tmp2;
	arm_matrix_instance_f32 tmp3;
} Luenberger;

// Luenberg observer for single-input, single-output, 2 states systems
typedef struct _LuenbergerSISO2S
{
	const float* pB;
	const float* pL;

	float Astar[4];

	float x[2];	// state
} LuenbergerSISO2S;

/**
 f	state vector rows
 g	control vector rows
 h	sensor vector rows
*/
void LuenbergerInit(Luenberger* pLB, uint16_t f, uint16_t g, uint16_t h, float* pData);
void LuenbergerUpdate(Luenberger* pLB);

void LuenbergerSISO2SInit(LuenbergerSISO2S* pLB, const float* pA, const float* pB, const float* pC, const float* pL);
void LuenbergerSISO2SUpdate(LuenbergerSISO2S* pLB, const float output, const float sensor);

#endif /* LUENBERGER_H_ */
