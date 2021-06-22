/*
 * piecewise_function.c
 *
 *  Created on: 20.01.2015
 *      Author: AndreR
 */

#include "piecewise_function.h"
#include "sys_time.h"

#include <math.h>

void PiecewiseFunctionStart(PiecewiseFunction* pFunc)
{
	pFunc->tStart = SysTimeUSec();
}

float PiecewiseFunctionValue(PiecewiseFunction* pFunc)
{
	uint32_t timeSys = (SysTimeUSec()-pFunc->tStart);

	LinFuncPiece* pPiece = 0;
	float tPieceStart = 0;
	uint8_t i;
	for(i = 0; i < 3; i++)
	{
		pPiece = &pFunc->pieces[i];
		if(timeSys < pPiece->tEnd)
			break;
	}

	if(i > 0)
		tPieceStart = pFunc->pieces[i-1].tEnd;

	if(i == 3)
		return 0;

	float t = (timeSys-tPieceStart)*1e-6f;

	return pPiece->a[0]+t*pPiece->a[1];
}

void PiecewiseFunctionTestTrajectory(PiecewiseFunction* pOut, float distance, float acceleration)
{
	float t1 = sqrtf(distance/(2*acceleration));
	float s1 = 0.5f*acceleration*t1*t1;
	float s2 = 2*s1;
	float t2 = s2/(acceleration*t1);

	LinFuncPiece* p1 = &pOut->pieces[0];
	LinFuncPiece* p2 = &pOut->pieces[1];
	LinFuncPiece* p3 = &pOut->pieces[2];

//	p1->tStart = 0;
	p1->tEnd = (uint32_t)(t1*1e6f);
	p1->a[0] = 0;
	p1->a[1] = acceleration;

//	p2->tStart = p1->tEnd;
	p2->tEnd = (uint32_t)((t1+t2)*1e6f);
	p2->a[0] = acceleration*t1;
	p2->a[1] = 0;

//	p3->tStart = p2->tEnd;
	p3->tEnd = (uint32_t)((2*t1+t2)*1e6f);
	p3->a[0] = acceleration*t1;
	p3->a[1] = -acceleration;
}

