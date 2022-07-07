/*
 * ctrl_motor.h
 *
 *  Created on: 08.03.2013
 *      Author: AndreR
 */

#ifndef CTRL_MOTOR_H_
#define CTRL_MOTOR_H_

#include "ctrl.h"
#include "util/trajectory_orient.h"

typedef struct _CtrlMotor
{
	float lastGlobalVel[3];

	uint8_t firstRun;
	float defaultPos[3];
} CtrlMotor;

extern CtrlMotor ctrlMotor;
extern CtrlInstance ctrlMotorInstance;

#endif /* CTRL_MOTOR_H_ */
