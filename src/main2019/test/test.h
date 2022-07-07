/*
 * test.h
 *
 *  Created on: 23.03.2019
 *      Author: AndreR
 */

#pragma once

#include "ch.h"
#include "intercom_constants.h"

typedef void(*TestProgressCb)(const TestProgress*);

#define TEST_EVENT_QUEUE_SIZE 20
#define TEST_MAX_PROGRESS_CALLBACKS 10

typedef struct _TestData
{
	mailbox_t eventQueue;
	msg_t eventQueueData[TEST_EVENT_QUEUE_SIZE];

	TestProgressCb progressCallbacks[TEST_MAX_PROGRESS_CALLBACKS];
	uint16_t usedProgressCallbacks;

	TestProgress progress;
	uint32_t testStartTime;
	uint32_t expectedTestTime;

	THD_WORKING_AREA(waProgressTask, 1024);
	thread_t* pProgressTask;
} TestData;

extern TestData test;

void TestInit();
void TestTask(void* params);
void TestAddProgressCallback(TestProgressCb cb);

void TestScheduleMotorTest(uint8_t testId, uint8_t motorId);
void TestScheduleCompassCalib();
void TestScheduleImuCalib();
void TestScheduleKickerTest();

void TestModeStartup();
void TestModeExit();
