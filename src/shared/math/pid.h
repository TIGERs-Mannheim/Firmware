/*
 * pid.h
 *
 *  Created on: 18.05.2014
 *      Author: AndreR
 */

#ifndef PID_H_
#define PID_H_

#include <stdint.h>
#include "lag_element.h"

typedef struct _PIDParams
{
	float kp;
	float ki;
	float kd;
	float outputMax;
	float outputMin;
	float dFilterTimeConstant;
	float pFilterTimeConstant;
} PIDParams;

typedef struct __attribute__((packed)) _PIDLog
{
	float in;
	float out;
	float error;
	float iTerm;
	float dTerm;
} PIDLog;

typedef struct _PID
{
	// input
	float setpoint;

	// parameters
	volatile uint16_t enabled;
	float sampleTime;	// [s] fixed! needs to be equal to update frequency
	PIDParams params;

	// data storage
	float lastMeasurement;
	float iTerm;		// integral term

	// output
	float output;
	uint8_t overload;

	LagElementPT1 dLag;
	LagElementPT1 pLag;

	// logging
	PIDLog log;
} PID;

void PIDInit(PID* pPID, PIDParams* pParams, float sampleTime, uint8_t enabled);
void PIDUpdate(PID* pPID, float measured);
void PIDUpdateAngular(PID* pPID, float measured);
void PIDSetSampleTime(PID* pPID, float newSampleTime);
void PIDSetParams(PID* pPID, float kp, float ki, float kd);
void PIDSetParamsStruct(PID* pPID, const PIDParams* pParams);
void PIDSetPFilterParam(PID* pPID, float T);
void PIDSetDFilterParam(PID* pPID, float T);
void PIDSetLimits(PID* pPID, float min, float max);
void PIDEnable(PID* pPID, uint8_t enable);

#endif /* PID_H_ */
