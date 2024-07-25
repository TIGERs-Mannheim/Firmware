#include "tests.h"
#include "util/test.h"
#include "test_motor.h"
#include "test_kicker.h"
#include "test_imu.h"
#include "test_experimental.h"

Tests tests;

void TestsInit(tprio_t taskPrio)
{
	TestInit(taskPrio);

	ShellCmdHandlerInit(&tests.cmdHandler, 0);

	TestMotorInit(&tests.cmdHandler);
	TestKickerInit(&tests.cmdHandler);
	TestImuInit(&tests.cmdHandler);
	TestExperimentalInit(&tests.cmdHandler);
}
