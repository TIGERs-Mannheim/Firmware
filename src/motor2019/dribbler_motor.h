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
	int8_t direction; // -1 reverse, 1 forward
	int32_t voltage;
} DribblerMotorData;

extern DribblerMotorData dribblerMotor;

void DribblerMotorInit();
void DribblerMotorSetOff();
void DribblerMotorSetPosition(int32_t voltage, uint8_t hallPos);
void DribblerMotorSetVoltage(int32_t voltage);
