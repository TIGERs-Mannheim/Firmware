/*
 * test.h
 *
 *  Created on: 27.06.2016
 *      Author: AndreR
 */

#ifndef SRC_TEST_H_
#define SRC_TEST_H_

#include "intercom_constants.h"
#include "ch.h"

typedef void(*TestDriveProgressCb)(TestProgress*);

#define TEST_EVENT_QUEUE_SIZE 10

#define TEST_REASONING_RESULT_OK 0
#define TEST_REASONING_RESULT_WARNING 1
#define TEST_REASONING_RESULT_BAD 2

typedef struct _TestData
{
	mailbox_t eventQueue;
	msg_t eventQueueData[TEST_EVENT_QUEUE_SIZE];

	TestDriveProgressCb pDriveProg;
} TestData;

typedef struct _TestDynamicsReasoningResult
{
	uint8_t damping;
	uint8_t inertia;
	uint8_t cur;
	uint8_t vel;
} TestDynamicsReasoningResult;

extern TestData test;

void TestInit();
void TestTask(void* params);
void TestScheduleMotorTest(uint8_t testId, uint8_t motorId);

void TestMotorDynamics(uint8_t motorId, TestMotorDynamicsResult* pResult);
void calculateReasoningOnTestResults(const TestMotorDynamicsResult* pResult, TestDynamicsReasoningResult* pReasoningResult);
void TestMotorTraction();

#endif /* SRC_TEST_H_ */
