/*
 * port_ex.h
 *
 *  Created on: 06.01.2019
 *      Author: AndreR
 *
 * TCA9539 IO Port Extension on Power Board
 */

#pragma once

#include "util/i2c.h"

#define I2C_ADDR_TCA9539	0x74

#define PORT_EX_TARGET_MOTOR1	0
#define PORT_EX_TARGET_MOTOR2	1
#define PORT_EX_TARGET_MOTOR3	2
#define PORT_EX_TARGET_MOTOR4	3
#define PORT_EX_TARGET_DRIBBLER	4
#define PORT_EX_TARGET_IR		5

void PortExInit(I2C* pI2C);
int16_t PortExResetPulse(uint8_t target);
int16_t PortExBootSet(uint8_t target, uint8_t enable);
