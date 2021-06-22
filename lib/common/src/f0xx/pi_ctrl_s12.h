/*
 * pi_ctrl_s12.h
 *
 *  Created on: 06.03.2019
 *      Author: AndreR
 */

#pragma once

#include <stdint.h>

typedef struct _PICtrlS12
{
	int32_t setpoint; // S12.0

	int32_t kp; // S7.10
	int32_t ki; // S5.13

	int32_t iTerm; // S12.12

	int32_t output; // S12.0

	int32_t outputMin;
	int32_t outputMax;

	uint8_t overload;
	uint8_t enabled;
} PICtrlS12;

void PICtrlS12Init(PICtrlS12* pPI, int32_t kp, int32_t ki, int32_t outputMin, int32_t outputMax);
void PICtrlS12Enable(PICtrlS12* pPI, uint8_t enable);
void PICtrlS12Update(PICtrlS12* pPI, int32_t measured);
void PICtrlS12Setpoint(PICtrlS12* pPI, int32_t setpoint);
