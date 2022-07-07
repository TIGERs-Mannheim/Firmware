/*
 * test_experimental.h
 *
 *  Created on: 21.05.2022
 *      Author: AndreR
 */

#pragma once

#include <stdint.h>

void TestMotorIdentDriveAll();
void TestCompassCalib();
void TestMotorTraction();
void TestRotationIdent();
void TestMotorPhaseResistance(uint8_t motorId);
void TestDribbleRotation(float dribblerCurrent, float dribblerSpeedRPM);
