/*
 * drive_motor.h
 *
 *  Created on: 25.01.2019
 *      Author: AndreR
 */

#pragma once

#include <stdint.h>

#define HALL_GET_POS() (GPIOA->IDR & 0x07)

typedef struct _DriveMotorData
{
	volatile uint16_t dmaData[24]; // [CCR1 CCR2 CCR3] x 8
	volatile uint16_t encDmaData[8];

	uint8_t currentMode;
} DriveMotorData;

extern DriveMotorData driveMotor;

void DriveMotorInit();
void DriveMotorUpdate();
void DriveMotorSetAlphaBeta(int32_t alpha_S16_0, int32_t beta_S16_0);
