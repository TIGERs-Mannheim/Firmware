/*
 * piecewise_function.h
 *
 *  Created on: 20.01.2015
 *      Author: AndreR
 */

#ifndef PIECEWISE_FUNCTION_H_
#define PIECEWISE_FUNCTION_H_

#include <stdint.h>

typedef struct _LinFuncPiece
{
	float a[2];	// function coefficients

//	uint32_t tStart;	// [us]
	uint32_t tEnd;		// [us]
} LinFuncPiece;

typedef struct _PiecewiseFunction
{
	LinFuncPiece pieces[3];

	uint32_t tStart;	// [us], SysTime
} PiecewiseFunction;

void	PiecewiseFunctionStart(PiecewiseFunction* pFunc);
float	PiecewiseFunctionValue(PiecewiseFunction* pFunc);
void	PiecewiseFunctionTestTrajectory(PiecewiseFunction* pOut, float distance, float acceleration);

#endif /* PIECEWISE_FUNCTION_H_ */
