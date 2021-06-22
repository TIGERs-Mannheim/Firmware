/*
 * dribbler_motor.h
 *
 *  Created on: 21.08.2020
 *      Author: AndreR
 */

#pragma once

#include <stdint.h>

#define HALL_GET_POS() (GPIOA->IDR & 0x07)

typedef struct _DribblerMotorData
{
	uint8_t enabled;
	int8_t direction; // -1 reverse, 1 forward
} DribblerMotorData;

extern DribblerMotorData dribblerMotor;

void DribblerMotorInit();
void DribblerMotorSetOff();
void DribblerMotorSetVoltage(int32_t voltage);
