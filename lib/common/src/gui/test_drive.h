/*
 * test_drive.h
 *
 *  Created on: 23.03.2019
 *      Author: AndreR
 */

#pragma once

#include "gfx.h"
#include "intercom_constants.h"

typedef void(*TestDriveScheduleTestCallback)(uint8_t, uint8_t);

GHandle TestDriveCreate(TestDriveScheduleTestCallback scheduleTestCallback);
void TestDriveProgress(TestProgress* pProgress);
void TestDriveDynamicResult(TestMotorDynamicsResult* pResult);
