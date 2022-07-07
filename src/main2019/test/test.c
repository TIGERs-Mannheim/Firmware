/*
 * test.c
 *
 *  Created on: 23.03.2019
 *      Author: AndreR
 */

#include "test.h"
#include "motors.h"
#include "util/console.h"
#include "util/sys_time.h"
#include "robot/robot.h"
#include "gui/test_drive.h"

#include "test/test_imu.h"
#include "test/test_kicker.h"
#include "test/test_motor.h"
#include "test/test_experimental.h"

TestData test;

static void testProgressTask(void* params);

void TestInit()
{
	chMBObjectInit(&test.eventQueue, test.eventQueueData, TEST_EVENT_QUEUE_SIZE);

	TestAddProgressCallback(&TestDriveProgress);
}

void TestAddProgressCallback(TestProgressCb cb)
{
	if(test.usedProgressCallbacks >= TEST_MAX_PROGRESS_CALLBACKS)
		return;

	test.progressCallbacks[test.usedProgressCallbacks] = cb;
	test.usedProgressCallbacks++;
}

void TestTask(void* params)
{
	(void)params;

	msg_t event;

	chRegSetThreadName("Test");

	while(1)
	{
		if(chMBFetch(&test.eventQueue, &event, TIME_INFINITE) != MSG_OK)
			continue;

		uint8_t testId = event & 0xFF;
		uint8_t motorId = event >> 8;

		test.progress.testId = testId;
		test.progress.motorId = motorId;
		test.testStartTime = SysTimeUSec();
		test.expectedTestTime = 0;

		test.pProgressTask = chThdCreateStatic(test.waProgressTask, sizeof(test.waProgressTask), LOWPRIO,
				&testProgressTask, 0);

		switch(testId)
		{
			case IC_TEST_MOT_TRACTION:
			{
				TestMotorTraction();
			}
			break;
			case IC_TEST_MOT_IDENT_ELEC:
			{
				TestMotorIdentElectrical(motorId);
			}
			break;
			case IC_TEST_MOT_DYN:
			case IC_TEST_MOT_IDENT_MECH:
			{
				TestMotorIdentMechanical(motorId);
			}
			break;
			case IC_TEST_MOT_IDENT_ALL:
			{
				TestMotorIdentDriveAll();
			}
			break;
			case IC_TEST_MOT_PHASE_RESISTANCE:
			{
				TestMotorPhaseResistance(motorId);
			}
			break;
			case IC_TEST_ROTATION_IDENT:
			{
				TestRotationIdent();
			}
			break;
			case IC_TEST_COMPASS_CALIB:
			{
				TestCompassCalib();
			}
			break;
			case IC_TEST_IMU_CALIB:
			{
				TestImuCalib();
			}
			break;
			case IC_TEST_KICKER:
			{
				TestKickerResult result;
				TestKicker(&result);
			}
			break;
		}

		ConsolePrint("Test completed in %ums\r\n", (SysTimeUSec()-test.testStartTime)/1000);

		test.progress.testId = 0xFF;

		chThdWait(test.pProgressTask);
	}
}

void TestSchedule(uint8_t testId)
{
	chMBPost(&test.eventQueue, testId, TIME_IMMEDIATE);
}

void TestScheduleWithParam(uint8_t testId, uint8_t param)
{
	msg_t event = param;
	event <<= 8;
	event |= testId;

	chMBPost(&test.eventQueue, event, TIME_IMMEDIATE);
}

void TestScheduleMotorTest(uint8_t testId, uint8_t motorId)
{
	msg_t event = motorId;
	event <<= 8;
	event |= testId;

	chMBPost(&test.eventQueue, event, TIME_IMMEDIATE);
}

void TestScheduleCompassCalib()
{
	chMBPost(&test.eventQueue, IC_TEST_COMPASS_CALIB, TIME_IMMEDIATE);
}

void TestScheduleKickerTest()
{
	chMBPost(&test.eventQueue, IC_TEST_KICKER, TIME_IMMEDIATE);
}

void TestScheduleImuCalib()
{
	chMBPost(&test.eventQueue, IC_TEST_IMU_CALIB, TIME_IMMEDIATE);
}

void TestModeStartup()
{
	// Go to test mode
	RobotTestModeEnable(1);
	RobotTestModeStopAll();

	chThdSleepMilliseconds(5);

	for(uint8_t i = 0; i < 5; i++)
		MotorsSetOff(i);
}

void TestModeExit()
{
	RobotTestModeEnable(0);
	chThdSleepMilliseconds(1000);
}

static void testProgressTask(void* params)
{
	(void)params;

	chRegSetThreadName("TestProgress");

	while(test.progress.testId < 0xFF)
	{
		uint32_t testRuntime = SysTimeUSec() - test.testStartTime;
		test.progress.percentComplete = (float)testRuntime/(float)test.expectedTestTime * 100.0f;

		if(test.progress.percentComplete > 99.0f)
			test.progress.percentComplete = 99.0f;

		for(uint16_t i = 0; i < test.usedProgressCallbacks; i++)
			(*test.progressCallbacks[i])(&test.progress);

		chThdSleepMilliseconds(50);
	}

	chThdExit(0);
}
