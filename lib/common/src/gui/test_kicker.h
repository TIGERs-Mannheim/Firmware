/*
 * test_kicker.h
 *
 *  Created on: 20.01.2020
 *      Author: lixfel
 */

#pragma once

#include "gfx.h"
#include "intercom_constants.h"

typedef void(*TestKickerScheduleTestCallback)();

GHandle TestKickerCreate(TestKickerScheduleTestCallback scheduleTestCallback);
void TestKickerProgress(const char* pProgress);
void TestKickerResults(TestKickerResult* pResult);
