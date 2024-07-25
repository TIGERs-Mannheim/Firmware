#include "test_common.h"
#include "robot/robot.h"
#include "dev/motors.h"

void TestModeStartup()
{
	// Go to test mode
	RobotTestModeEnable(1);
	RobotTestModeStopAll();

	chThdSleepMilliseconds(5);

	for(uint8_t i = 0; i < 5; i++)
		McuMotorSetOff(&devMotors.mcu[i]);
}

void TestModeExit()
{
	RobotTestModeEnable(0);
	chThdSleepMilliseconds(1000);
}
