/*
 * pid_s12.c
 *
 *  Created on: 06.03.2019
 *      Author: AndreR
 *
 *  Fixed-Point PI Controller
 *  Input/Output: S12.0
 */

#include "pi_ctrl_s12.h"

/**
 * measured: S12.0
 */
void PICtrlS12Update(PICtrlS12* pPI, int32_t measured)
{
	if(!pPI->enabled)
		return;

	if(measured > 4095)
		measured = 4095;
	if(measured < -4096)
		measured = -4096;

	// S13.0 = S12.0 - S12.0
	int32_t error = pPI->setpoint - measured;

	// S18.13 = S5.13 * S13.0
	int32_t iError = pPI->ki * error;

	// => S18.12
	iError >>= 1;

	// S19.12 = S12.12 + S18.12
	int32_t iTermNew = pPI->iTerm + iError;

	// => S12.12
	if(iTermNew > (pPI->outputMax << 12))
		iTermNew = pPI->outputMax << 12;
	if(iTermNew < (pPI->outputMin << 12))
		iTermNew = (pPI->outputMin << 12);

	pPI->iTerm = iTermNew;

	// S20.10 = S7.10 * S13.0
	int32_t pError = pPI->kp * error;

	// S21.10 = S20.10 + (S12.12 >> 2)
	int32_t output = pError + (iTermNew >> 2);

	// => S21.0
	output >>= 10;

	if(pPI->antiJitter)
	{
		int32_t errorAbs = error < 0 ? -error : error;

		if(errorAbs < pPI->antiJitter)
		{
			// S12.0 = (S0.12 * S13.0 * S12.0) >> 12;
			//          \___________/ => due to errorAbs < antiJitter this can be a max of 4096 (S0.12)
			output = (pPI->antiJitterReciprocal * errorAbs * output) >> 12;
		}
	}

	// => S12.0
	if(output > pPI->outputMax)
	{
		output = pPI->outputMax;
		pPI->overload = 1;
	}
	else if(output < pPI->outputMin)
	{
		output = pPI->outputMin;
		pPI->overload = 1;
	}
	else
		pPI->overload = 0;

	pPI->output = output;
}

void PICtrlS12Setpoint(PICtrlS12* pPI, int32_t setpoint)
{
	if(setpoint > 4095)
		setpoint = 4095;
	if(setpoint < -4096)
		setpoint = -4096;

	pPI->setpoint = setpoint;
}

void PICtrlS12Enable(PICtrlS12* pPI, uint8_t enable)
{
	if(enable && !pPI->enabled)
	{
		// we just went from manual to auto
		if(pPI->ki)
			pPI->iTerm = pPI->output << 17;
		else
			pPI->iTerm = 0;
	}

	pPI->enabled = enable;
}

void PICtrlS12SetAntiJitter(PICtrlS12* pPI, int32_t antiJitter_S12_0)
{
	if(pPI->antiJitter == antiJitter_S12_0)
		return;

	if(antiJitter_S12_0 > 4096)
		antiJitter_S12_0 = 4096;

	pPI->antiJitter = antiJitter_S12_0;

	if(pPI->antiJitter == 0)
		pPI->antiJitterReciprocal = 0;
	else
		pPI->antiJitterReciprocal = 4096/antiJitter_S12_0;
}

void PICtrlS12Init(PICtrlS12* pPI, int32_t kp, int32_t ki, int32_t outputMin, int32_t outputMax)
{
	pPI->kp = kp;
	pPI->ki = ki;
	pPI->outputMin = outputMin;
	pPI->outputMax = outputMax;
	pPI->antiJitter = 0;
	pPI->antiJitterReciprocal = 0;
	pPI->iTerm = 0;
	pPI->output = 0;
	pPI->overload = 0;
	pPI->setpoint = 0;
	pPI->enabled = 0;
}
