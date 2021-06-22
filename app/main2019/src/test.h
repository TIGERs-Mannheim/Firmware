/*
 * test.h
 *
 *  Created on: 23.03.2019
 *      Author: AndreR
 */

#pragma once

#include "ch.h"
#include "intercom_constants.h"

typedef void(*TestDriveProgressCb)(TestProgress*);

#define TEST_EVENT_QUEUE_SIZE 10

typedef struct _TestData
{
	mailbox_t eventQueue;
	msg_t eventQueueData[TEST_EVENT_QUEUE_SIZE];

	TestDriveProgressCb pDriveProg;
} TestData;

extern TestData test;

void TestInit();
void TestTask(void* params);
void TestScheduleMotorTest(uint8_t testId, uint8_t motorId);
void TestScheduleCompassCalib();
void TestScheduleImuCalib();
void TestScheduleKickerTest();

void TestMotorDynamics(uint8_t motorId, TestMotorDynamicsResult* pResult);
void TestMotorTraction();
