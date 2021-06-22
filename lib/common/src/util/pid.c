/*
 * pid.c
 *
 *  Created on: 18.05.2014
 *      Author: AndreR
 */

#include "pid.h"
#include "util/angle_math.h"
#include <string.h>

void PIDUpdate(PID* pPID, float measured)
{
	if(!pPID->enabled)
	{
		pPID->lastMeasurement = measured;
		pPID->log.in = measured;
		return;
	}

	/*Compute all the working error variables*/
	float error = pPID->setpoint - measured;
	pPID->iTerm += (pPID->params.ki * error);
	if(pPID->iTerm > pPID->params.outputMax)
		pPID->iTerm = pPID->params.outputMax;
	else if (pPID->iTerm < pPID->params.outputMin)
		pPID->iTerm = pPID->params.outputMin;
	float dInput = (measured - pPID->lastMeasurement);

	dInput = LagElementPT1Process(&pPID->dLag, dInput);
	error = LagElementPT1Process(&pPID->pLag, error);

	/*Compute PID Output*/
	pPID->output = pPID->params.kp * error + pPID->iTerm - pPID->params.kd * dInput;
	if(pPID->output > pPID->params.outputMax)
	{
		pPID->output = pPID->params.outputMax;
		pPID->overload = 1;
	}
	else if(pPID->output < pPID->params.outputMin)
	{
		pPID->output = pPID->params.outputMin;
		pPID->overload= 1;
	}
	else
		pPID->overload = 0;

	/*Remember some variables for next time*/
	pPID->lastMeasurement = measured;

	pPID->log.in = measured;
	pPID->log.out = pPID->output;
	pPID->log.error = error;
	pPID->log.iTerm = pPID->iTerm;
	pPID->log.dTerm = dInput;
}

void PIDUpdateAngular(PID* pPID, float measured)
{
	if(!pPID->enabled)
	{
		pPID->lastMeasurement = measured;
		pPID->log.in = measured;
		return;
	}

	/*Compute all the working error variables*/
	float error = pPID->setpoint - measured;
	AngleNormalizePtr(&error);
	pPID->iTerm += (pPID->params.ki * error);
	if(pPID->iTerm > pPID->params.outputMax)
		pPID->iTerm = pPID->params.outputMax;
	else if (pPID->iTerm < pPID->params.outputMin)
		pPID->iTerm = pPID->params.outputMin;
	float dInput = (measured - pPID->lastMeasurement);
	AngleNormalizePtr(&dInput);

	/*Compute PID Output*/
	pPID->output = pPID->params.kp * error + pPID->iTerm - pPID->params.kd * dInput;
	if(pPID->output > pPID->params.outputMax)
	{
		pPID->output = pPID->params.outputMax;
		pPID->overload = 1;
	}
	else if(pPID->output < pPID->params.outputMin)
	{
		pPID->output = pPID->params.outputMin;
		pPID->overload= 1;
	}
	else
		pPID->overload = 0;

	/*Remember some variables for next time*/
	pPID->lastMeasurement = measured;

	pPID->log.in = measured;
	pPID->log.out = pPID->output;
	pPID->log.error = error;
	pPID->log.iTerm = pPID->iTerm;
	pPID->log.dTerm = dInput;
}

void PIDSetParamsStruct(PID* pPID, const PIDParams* pParams)
{
	PIDSetParams(pPID, pParams->kp, pParams->ki, pParams->kd);
	PIDSetLimits(pPID, pParams->outputMin, pParams->outputMax);
}

void PIDSetParams(PID* pPID, float kp, float ki, float kd)
{
	pPID->params.kp = kp;
	pPID->params.ki = ki*pPID->sampleTime;
	pPID->params.kd = kd/pPID->sampleTime;
	pPID->iTerm = 0;
}

void PIDSetDFilterParam(PID* pPID, float T)
{
	LagElementPT1Init(&pPID->dLag, 1.0f, T, pPID->sampleTime);
	pPID->params.dFilterTimeConstant = T;
}

void PIDSetPFilterParam(PID* pPID, float T)
{
	LagElementPT1Init(&pPID->pLag, 1.0f, T, pPID->sampleTime);
	pPID->params.pFilterTimeConstant = T;
}

void PIDSetSampleTime(PID* pPID, float newSampleTime)
{
	if(newSampleTime > 0)
	{
		float ratio = newSampleTime/pPID->sampleTime;
		pPID->params.ki *= ratio;
		pPID->params.kd /= ratio;
		pPID->sampleTime = newSampleTime;
		pPID->iTerm = 0;
	}
}

void PIDSetLimits(PID* pPID, float min, float max)
{
	if(min > max)
		return;

	pPID->params.outputMin = min;
	pPID->params.outputMax = max;

	if(pPID->output > pPID->params.outputMax)
	{
		pPID->output = pPID->params.outputMax;
		pPID->overload = 1;
	}
	else if(pPID->output < pPID->params.outputMin)
	{
		pPID->output = pPID->params.outputMin;
		pPID->overload = 1;
	}
	else
		pPID->overload = 0;

	if(pPID->iTerm > pPID->params.outputMax)
		pPID->iTerm = pPID->params.outputMax;
	else if(pPID->iTerm < pPID->params.outputMin)
		pPID->iTerm = pPID->params.outputMin;
}

void PIDEnable(PID* pPID, uint8_t enable)
{
	if(enable && !pPID->enabled)
	{
		// we just went from manual to auto
		pPID->iTerm = pPID->output;

		if(pPID->iTerm > pPID->params.outputMax)
		{
			pPID->iTerm = pPID->params.outputMax;
			pPID->overload = 1;
		}
		else if(pPID->iTerm < pPID->params.outputMin)
		{
			pPID->iTerm = pPID->params.outputMin;
			pPID->overload = 1;
		}
		else
			pPID->overload = 0;
	}

	pPID->enabled = enable;
}

void PIDInit(PID* pPID, PIDParams* pParams, float sampleTime, uint8_t enabled)
{
	memset(pPID, 0, sizeof(*pPID));
	PIDSetSampleTime(pPID, sampleTime);
	PIDSetParams(pPID, pParams->kp, pParams->ki, pParams->kd);
	PIDSetLimits(pPID, pParams->outputMin, pParams->outputMax);
	PIDSetDFilterParam(pPID, pParams->dFilterTimeConstant);
	PIDSetPFilterParam(pPID, pParams->pFilterTimeConstant);
	PIDEnable(pPID, enabled);
}
