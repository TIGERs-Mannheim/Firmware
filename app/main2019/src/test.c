/*
 * test.c
 *
 *  Created on: 23.03.2019
 *      Author: AndreR
 */

#include "test.h"
#include "float.h"
#include "motors.h"
#include "hal/led.h"
#include "kicker.h"
#include "power.h"
#include "main/network.h"
#include "arm_math.h"
#include "spi4.h"
#include "util/arm_mat_util_f32.h"
#include "util/angle_math.h"
#include "util/console.h"
#include "util/sys_time.h"
#include "util/network_print.h"
#include "util/signal_statistics.h"
#include "main/robot.h"
#include "main/ctrl.h"
#include "gui/test_drive.h"
#include "gui/test_kicker.h"
#include "main/skill_basics.h"
#include "main/ctrl_tigga.h"

#define TEST_KICKER_TIMEOUT 10000 // in ms

static void printTestMotorDynamicsResult(TestMotorDynamicsResult* pResult);
static void doReasoningOnDriveMotor(TestMotorDynamicsResult* pResult);
static void doReasoningOnDribblerMotor(TestMotorDynamicsResult* pResult);
static void doReasoningOnMotor(TestMotorDynamicsResult* pResult);
static void printTestMotorDynamicsResult(TestMotorDynamicsResult* pResult);
static void testCompassCalib();
static void testKicker(TestKickerResult* pResult);
static void doReasoningOnKicker(TestKickerResult* pResult);
static float calculateDriveTrainInertia(const float* pSamples, uint16_t numSamples, float voltage, uint16_t motorId, float b);
static void testMotorCurrent(uint8_t motorId);
static void testMotorIdent(uint8_t motorId);
static void testMotorIdentDriveAll();
static void testMotorResistance(uint8_t motorId);
static void testRotationIdent();
static void testImuCalib();

TestData test;

void TestInit()
{
	chMBObjectInit(&test.eventQueue, test.eventQueueData, TEST_EVENT_QUEUE_SIZE);

	test.pDriveProg = &TestDriveProgress;
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

		switch(testId)
		{
			case IC_TEST_MOT_DYN:
			{
				TestMotorDynamicsResult result;
				TestMotorDynamics(motorId, &result);

				TestDriveDynamicResult(&result);
			}
			break;
			case IC_TEST_MOT_TRACTION:
			{
				TestMotorTraction();
			}
			break;
			case IC_TEST_MOT_CURRENT:
			{
				testMotorCurrent(motorId);
			}
			break;
			case IC_TEST_MOT_IDENT:
			{
				testMotorIdent(motorId);
			}
			break;
			case IC_TEST_MOT_IDENT_ALL:
			{
				testMotorIdentDriveAll();
			}
			break;
			case IC_TEST_MOT_RESISTANCE:
			{
				testMotorResistance(motorId);
			}
			break;
			case IC_TEST_ROTATION_IDENT:
			{
				testRotationIdent();
			}
			break;
			case IC_TEST_COMPASS_CALIB:
			{
				testCompassCalib();
			}
			break;
			case IC_TEST_IMU_CALIB:
			{
				testImuCalib();
			}
			break;
			case IC_TEST_KICKER:
			{
				TestKickerResult result;
				testKicker(&result);
				doReasoningOnKicker(&result);

				TestKickerResults(&result);
			}
			break;
		}
	}
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

static void testStartup()
{
	//Shutdown to test mode
	RobotTestModeEnable(1);
	RobotTestModeStopAll();
	chThdSleepMilliseconds(5);
	for(uint8_t i = 0; i < 5; i++)
		MotorsSetOff(i);
}

static void testStop()
{
	RobotTestModeEnable(0);
	chThdSleepMilliseconds(1000);
}

static void testCompassCalib()
{
	testStartup();

	//Calib compass
	RobotTestModeSetSystems(ROBOT_SYSTEM_CONTROL | ROBOT_SYSTEM_STATE_EST | ROBOT_SYSTEM_DRIVE);
	robot.skillOutput.drive.limits.velMaxXY = (GLOBAL_POS_MAX_VEL_XY/255.0f);
	robot.skillOutput.drive.limits.velMaxW = (GLOBAL_POS_MAX_VEL_W/255.0f);
	robot.skillOutput.drive.limits.accMaxXY = (LOCAL_VEL_MAX_ACC_XY/255.0f);
	robot.skillOutput.drive.limits.accMaxW = (LOCAL_VEL_MAX_ACC_W/255.0f);
	robot.skillOutput.drive.limits.strictVelLimit = 0;
	robot.skillOutput.drive.modeW = DRIVE_MODE_LOCAL_VEL;
	robot.skillOutput.drive.modeXY = DRIVE_MODE_LOCAL_VEL;
	robot.skillOutput.drive.localVel[0] = 0;
	robot.skillOutput.drive.localVel[1] = 0;
	robot.skillOutput.drive.localVel[2] = 1.0;

	float magMin[2] = {FLT_MAX, FLT_MAX};
	float magMax[2] = {FLT_MIN, FLT_MIN};
	for(uint16_t i = 0; i < 10000; i++) //10s
	{
		for(uint8_t axis = 0; axis < 2; axis++)
		{
			float mag = robot.sensors.mag.strength[axis];

			if(mag < magMin[axis])
				magMin[axis] = mag;
			else if(mag > magMax[axis])
				magMax[axis] = mag;

		}
		chThdSleepMilliseconds(1);
	}
	robot.skillOutput.drive.localVel[2] = 0;
	chThdSleepMilliseconds(200);

	ConsolePrint("Min1:%f Min2:%f Max1:%f Max2:%f\r\n", magMin[0], magMin[1], magMax[0], magMax[1]);

	ctrlTigga.configCompass.hardIronX = (magMin[0] + magMax[0]) * 0.5f;
	ctrlTigga.configCompass.hardIronY = (magMin[1] + magMax[1]) * 0.5f;

	float hardIronCompX = robot.sensors.mag.strength[0] - ctrlTigga.configCompass.hardIronX;
	float hardIronCompY = robot.sensors.mag.strength[1] - ctrlTigga.configCompass.hardIronY;
	float softIronCompX = (ctrlTigga.configCompass.softIronCos*hardIronCompX + ctrlTigga.configCompass.softIronSin*hardIronCompY) * ctrlTigga.configCompass.softEllipseEcc;
	float softIronCompY = ctrlTigga.configCompass.softIronCos*hardIronCompY - ctrlTigga.configCompass.softIronSin*hardIronCompX;
	/*
	 * Read https://www.fierceelectronics.com/components/compensating-for-tilt-hard-iron-and-soft-iron-effects
	 * for further information on the topic of compass calibration.
	 * (Tilt is irrelevant)
	 * Due to the complexity of the detection of ellipse parameters the soft iron calibration is not done dynamically
	 * (nor automatic). The parameters for the soft iron calibration have been determined manually for one bot with
	 * the use of matlab and been tested on another bot.
	 * */

	float fieldOffset = AngleNormalize(arm_atan2_f32(softIronCompX, softIronCompY) - robot.state.pos[2]);
	ConsolePrint("Field offset: %f, Vision rotation:  %f\r\n", fieldOffset, robot.state.pos[2]);

	ctrlTigga.configCompass.fieldOffset = fieldOffset;
	ConfigNotifyUpdate(ctrlTigga.pConfigFileCompass);

	testStop();
}

#define IMU_CALIB_NUM_SAMPLES 9

static void testImuCalib()
{
	static const char* positionNames[] = {
			"Standing", "Front-Right Wheel down", "Rear-Right Wheel down",
			"Rear-Left Wheel down", "Front-Left Wheel down", "Upside down",
			"Standing", "Upside down", "Standing" };

	testStartup();

	SignalStatistics statTempImu = { 0 };
	SignalStatistics statTempMag = { 0 };

	ConsolePrint("Waiting for IMU temperature to settle, this can take few minutes.\r\n");

	uint8_t tempUnstable = 1;
	while(tempUnstable)
	{
		systime_t prev = chVTGetSystemTimeX();
		systime_t next = prev+US2ST(1000);

		SignalStatisticsReset(&statTempImu);

		for(uint32_t t = 0; t < 10000; t++)
		{
			chThdSleepUntilWindowed(prev, next);
			prev = next;
			next += US2ST(1000);

			SignalStatisticsSample(&statTempImu, spi4.imu.temp);

			if(t % 1000 == 0)
				ConsolePrint(".");
		}

		SignalStatisticsUpdate(&statTempImu);

		ConsolePrint("      % .8f  %.12f  %.8f\r\n", statTempImu.mean, statTempImu.variance, statTempImu.standardDeviation);

		if(statTempImu.standardDeviation < 0.03)
			tempUnstable = 0;
	}

	ConsolePrint("Leave the robot steady, acquiring initial signal variances");

	// contains gyro and accelerometer mean during static phases
	static double records[IMU_CALIB_NUM_SAMPLES][9];
	memset(records, 0, sizeof(records));

	SignalStatistics stats[9] = { 0 };

	const uint32_t numSamples = 10000;
	const uint32_t intervalLengths = 3000;

	systime_t prev = chVTGetSystemTimeX();
	systime_t next = prev+US2ST(1000);

	for(uint32_t t = 0; t < numSamples; t++)
	{
		chThdSleepUntilWindowed(prev, next);
		prev = next;
		next += US2ST(1000);

		SignalStatisticsSample(&stats[0], spi4.imu.gyr[0]);
		SignalStatisticsSample(&stats[1], spi4.imu.gyr[1]);
		SignalStatisticsSample(&stats[2], spi4.imu.gyr[2]);
		SignalStatisticsSample(&stats[3], spi4.imu.acc[0]);
		SignalStatisticsSample(&stats[4], spi4.imu.acc[1]);
		SignalStatisticsSample(&stats[5], spi4.imu.acc[2]);
		SignalStatisticsSample(&statTempImu, spi4.imu.temp);
		SignalStatisticsSample(&stats[6], spi4.mag.strength[0]);
		SignalStatisticsSample(&stats[7], spi4.mag.strength[1]);
		SignalStatisticsSample(&stats[8], spi4.mag.strength[2]);
		SignalStatisticsSample(&statTempMag, spi4.mag.temp);

		if(t % 1000 == 0)
			ConsolePrint(".");
	}

	ConsolePrint("done\r\n");

	for(uint8_t i = 0; i < 9; i++)
	{
		SignalStatisticsUpdate(&stats[i]);
		records[0][i] = stats[i].mean;
	}

	SignalStatisticsUpdate(&statTempImu);
	SignalStatisticsUpdate(&statTempMag);

	ConsolePrint("Gyro      mean          var.        std.dev.\r\n");
	ConsolePrint("X     % .8f  %.12f  %.8f\r\n", stats[0].mean, stats[0].variance, stats[0].standardDeviation);
	ConsolePrint("Y     % .8f  %.12f  %.8f\r\n", stats[1].mean, stats[1].variance, stats[1].standardDeviation);
	ConsolePrint("Z     % .8f  %.12f  %.8f\r\n", stats[2].mean, stats[2].variance, stats[2].standardDeviation);

	ConsolePrint("\r\nAcc       mean          var.        std.dev.\r\n");
	ConsolePrint("X     % .8f  %.12f  %.8f\r\n", stats[3].mean, stats[3].variance, stats[3].standardDeviation);
	ConsolePrint("Y     % .8f  %.12f  %.8f\r\n", stats[4].mean, stats[4].variance, stats[4].standardDeviation);
	ConsolePrint("Z     % .8f  %.12f  %.8f\r\n", stats[5].mean, stats[5].variance, stats[5].standardDeviation);

	ConsolePrint("\r\nTemp      mean          var.        std.dev.\r\n");
	ConsolePrint("      % .8f  %.12f  %.8f\r\n", statTempImu.mean, statTempImu.variance, statTempImu.standardDeviation);

	ConsolePrint("\r\nMag       mean          var.        std.dev.\r\n");
	ConsolePrint("X     % .8f  %.12f  %.8f\r\n", stats[6].mean, stats[6].variance, stats[6].standardDeviation);
	ConsolePrint("Y     % .8f  %.12f  %.8f\r\n", stats[7].mean, stats[7].variance, stats[7].standardDeviation);
	ConsolePrint("Z     % .8f  %.12f  %.8f\r\n", stats[8].mean, stats[8].variance, stats[8].standardDeviation);

	ConsolePrint("\r\nTemp      mean          var.        std.dev.\r\n");
	ConsolePrint("      % .8f  %.12f  %.8f\r\n", statTempMag.mean, statTempMag.variance, statTempMag.standardDeviation);

	const double staticThresholdFactor = 8.0;
	const double staticVarianceThresholds[3] = {
			stats[3].variance*staticThresholdFactor,
			stats[4].variance*staticThresholdFactor,
			stats[5].variance*staticThresholdFactor };

	for(uint16_t sample = 1; sample < IMU_CALIB_NUM_SAMPLES; sample++)
	{
		ConsolePrint("\r\nPlace the robot in position: %s\r\n", positionNames[sample]);

		uint8_t isSteady = 1;
		while(isSteady)
		{
			for(uint8_t i = 0; i < 6; i++)
				SignalStatisticsReset(&stats[i]);

			systime_t prev = chVTGetSystemTimeX();
			systime_t next = prev+US2ST(1000);

			for(uint32_t t = 0; t < intervalLengths; t++)
			{
				chThdSleepUntilWindowed(prev, next);
				prev = next;
				next += US2ST(1000);

				SignalStatisticsSample(&stats[3], spi4.imu.acc[0]);
				SignalStatisticsSample(&stats[4], spi4.imu.acc[1]);
				SignalStatisticsSample(&stats[5], spi4.imu.acc[2]);
			}

			for(uint8_t i = 3; i < 6; i++)
				SignalStatisticsUpdate(&stats[i]);

			if( stats[3].variance > staticVarianceThresholds[0] &&
				stats[4].variance > staticVarianceThresholds[1] &&
				stats[5].variance > staticVarianceThresholds[2])
			{
				isSteady = 0;
			}
		}

		ConsolePrint("Movement detected, waiting for robot to stabilize...\r\n");

		uint8_t isMoving = 1;
		while(isMoving)
		{
			for(uint8_t i = 0; i < 9; i++)
				SignalStatisticsReset(&stats[i]);

			systime_t prev = chVTGetSystemTimeX();
			systime_t next = prev+US2ST(1000);

			for(uint32_t t = 0; t < intervalLengths; t++)
			{
				chThdSleepUntilWindowed(prev, next);
				prev = next;
				next += US2ST(1000);

				SignalStatisticsSample(&stats[0], spi4.imu.gyr[0]);
				SignalStatisticsSample(&stats[1], spi4.imu.gyr[1]);
				SignalStatisticsSample(&stats[2], spi4.imu.gyr[2]);
				SignalStatisticsSample(&stats[3], spi4.imu.acc[0]);
				SignalStatisticsSample(&stats[4], spi4.imu.acc[1]);
				SignalStatisticsSample(&stats[5], spi4.imu.acc[2]);
				SignalStatisticsSample(&stats[6], spi4.mag.strength[0]);
				SignalStatisticsSample(&stats[7], spi4.mag.strength[1]);
				SignalStatisticsSample(&stats[8], spi4.mag.strength[2]);
			}

			for(uint8_t i = 0; i < 9; i++)
				SignalStatisticsUpdate(&stats[i]);

			if( stats[3].variance < staticVarianceThresholds[0] &&
				stats[4].variance < staticVarianceThresholds[1] &&
				stats[5].variance < staticVarianceThresholds[2])
			{
				isMoving = 0;

				// store final state
				for(uint8_t i = 0; i < 9; i++)
					records[sample][i] = stats[i].mean;
			}
		}

		ConsolePrint("Sample acquired (%hu/%hu)\r\n", sample+1, IMU_CALIB_NUM_SAMPLES);
	}

	ConsolePrint("\r\nCalibration complete, raw data:\r\n");

	for(uint16_t sample = 0; sample < IMU_CALIB_NUM_SAMPLES; sample++)
	{
		ConsolePrint("% .8f, % .8f, % .8f, % .8f, % .8f, % .8f, % .8f, % .8f, % .8f\r\n",
			records[sample][0], records[sample][1], records[sample][2],
			records[sample][3], records[sample][4], records[sample][5],
			records[sample][6], records[sample][7], records[sample][8]);

		chThdSleepMilliseconds(100);
	}

	chThdSleepMilliseconds(100);

	// Compute acceleromter bias

	// Determine sphere from acceleration points (center == bias, magnitude should be ~9.81)
	//    (ax+bx)^2 + (ay+by)^2 + (az+bz)^2 = g^2
	// => bx^2 + by^2 + bz^2 - g^2 - 2*ax*bx - 2*ay*by - 2*az*bz = -(ax^2 + ay^2 + az^2)
	// set: P = bx^2 + by^2 + bz^2 - g^2, Q = -2*bx, R = -2*by, S = -2*bz
	// => P + Q*ax + R*ay + S*az = -(ax^2 + ay^2 + az^2)
	// vector x = [P Q R S]'
	// matrix A = [1 ax ay az; ..]
	// vector b = [-(ax^2 + ay^2 + az^2); ..]

	arm_matrix_instance_f32 matAccA = { IMU_CALIB_NUM_SAMPLES, 4, (float[IMU_CALIB_NUM_SAMPLES*4]){} };
	arm_matrix_instance_f32 matAccb = { IMU_CALIB_NUM_SAMPLES, 1, (float[IMU_CALIB_NUM_SAMPLES]){} };

	for(uint16_t sample = 0; sample < IMU_CALIB_NUM_SAMPLES; sample++)
	{
		MAT_ELEMENT(matAccA, sample, 0) = 1.0f;
		MAT_ELEMENT(matAccA, sample, 1) = records[sample][3];
		MAT_ELEMENT(matAccA, sample, 2) = records[sample][4];
		MAT_ELEMENT(matAccA, sample, 3) = records[sample][5];

		MAT_ELEMENT(matAccb, sample, 0) = -(records[sample][3]*records[sample][3] +
				records[sample][4]*records[sample][4] +
				records[sample][5]*records[sample][5]);
	}

	arm_matrix_instance_f32 matAccAInv = { 4, IMU_CALIB_NUM_SAMPLES, (float[IMU_CALIB_NUM_SAMPLES*4]){} };
	arm_mat_pinv(&matAccA, &matAccAInv);

	arm_matrix_instance_f32 matAccx = { 4, 1, (float[4]){} };
	arm_mat_mult_f32(&matAccAInv, &matAccb, &matAccx);

	float accBias[3];
	accBias[0] = -MAT_ELEMENT(matAccx, 1, 0) * 0.5f;
	accBias[1] = -MAT_ELEMENT(matAccx, 2, 0) * 0.5f;
	accBias[2] = -MAT_ELEMENT(matAccx, 3, 0) * 0.5f;

	float g2 = accBias[0]*accBias[0] + accBias[1]*accBias[1] + accBias[2]*accBias[2] - MAT_ELEMENT(matAccx, 0, 0);
	float g = sqrtf(fabsf(g2));

	chThdSleepMilliseconds(100);

	// Compute magnetometer bias
	for(uint16_t sample = 0; sample < IMU_CALIB_NUM_SAMPLES; sample++)
	{
		MAT_ELEMENT(matAccA, sample, 0) = 1.0f;
		MAT_ELEMENT(matAccA, sample, 1) = records[sample][6];
		MAT_ELEMENT(matAccA, sample, 2) = records[sample][7];
		MAT_ELEMENT(matAccA, sample, 3) = records[sample][8];

		MAT_ELEMENT(matAccb, sample, 0) = -(records[sample][6]*records[sample][6] +
				records[sample][7]*records[sample][7] +
				records[sample][8]*records[sample][8]);
	}

	arm_mat_pinv(&matAccA, &matAccAInv);
	arm_mat_mult_f32(&matAccAInv, &matAccb, &matAccx);

	float magBias[3];
	magBias[0] = -MAT_ELEMENT(matAccx, 1, 0) * 0.5f;
	magBias[1] = -MAT_ELEMENT(matAccx, 2, 0) * 0.5f;
	magBias[2] = -MAT_ELEMENT(matAccx, 3, 0) * 0.5f;

	float m2 = magBias[0]*magBias[0] + magBias[1]*magBias[1] + magBias[2]*magBias[2] - MAT_ELEMENT(matAccx, 0, 0);
	float m = sqrtf(fabsf(m2));

	chThdSleepMilliseconds(100);

	// Compute gyro bias (simple average)
	float gyroBias[3] = { 0 };

	for(uint16_t sample = 0; sample < IMU_CALIB_NUM_SAMPLES; sample++)
	{
		gyroBias[0] += records[sample][0];
		gyroBias[1] += records[sample][1];
		gyroBias[2] += records[sample][2];
	}

	gyroBias[0] /= (float)IMU_CALIB_NUM_SAMPLES;
	gyroBias[1] /= (float)IMU_CALIB_NUM_SAMPLES;
	gyroBias[2] /= (float)IMU_CALIB_NUM_SAMPLES;


	// gather data to determine IMU to body frame rotation
	memset(records, 0, sizeof(records));
	float gyroIntegrationZ = 0.0f;

#define IMU_CALIB_NUM_ROT_SAMPLES 4

	for(uint16_t sample = 0; sample < IMU_CALIB_NUM_ROT_SAMPLES; sample++)
	{
		ConsolePrint("\r\nPlease rotate robot by approx. 90 degree\r\n");

		uint8_t isSteady = 1;
		while(isSteady)
		{
			for(uint8_t i = 3; i < 6; i++)
				SignalStatisticsReset(&stats[i]);

			systime_t prev = chVTGetSystemTimeX();
			systime_t next = prev+US2ST(1000);

			for(uint32_t t = 0; t < intervalLengths; t++)
			{
				chThdSleepUntilWindowed(prev, next);
				prev = next;
				next += US2ST(1000);

				gyroIntegrationZ += (spi4.imu.gyr[2] - gyroBias[2])*0.001f;

				SignalStatisticsSample(&stats[3], spi4.imu.acc[0]);
				SignalStatisticsSample(&stats[4], spi4.imu.acc[1]);
				SignalStatisticsSample(&stats[5], spi4.imu.acc[2]);
			}

			for(uint8_t i = 3; i < 6; i++)
				SignalStatisticsUpdate(&stats[i]);

			if( stats[3].variance > staticVarianceThresholds[0] &&
				stats[4].variance > staticVarianceThresholds[1] &&
				stats[5].variance > staticVarianceThresholds[2])
			{
				isSteady = 0;
			}
		}

		ConsolePrint("Movement detected, waiting for robot to stabilize...\r\n");

		uint8_t isMoving = 1;
		while(isMoving)
		{
			for(uint8_t i = 0; i < 9; i++)
				SignalStatisticsReset(&stats[i]);

			systime_t prev = chVTGetSystemTimeX();
			systime_t next = prev+US2ST(1000);

			for(uint32_t t = 0; t < intervalLengths; t++)
			{
				chThdSleepUntilWindowed(prev, next);
				prev = next;
				next += US2ST(1000);

				gyroIntegrationZ += (spi4.imu.gyr[2] - gyroBias[2])*0.001f;

				SignalStatisticsSample(&stats[3], spi4.imu.acc[0] - accBias[0]);
				SignalStatisticsSample(&stats[4], spi4.imu.acc[1] - accBias[1]);
				SignalStatisticsSample(&stats[5], spi4.imu.acc[2] - accBias[2]);
			}

			for(uint8_t i = 3; i < 6; i++)
				SignalStatisticsUpdate(&stats[i]);

			if( stats[3].variance < staticVarianceThresholds[0] &&
				stats[4].variance < staticVarianceThresholds[1] &&
				stats[5].variance < staticVarianceThresholds[2])
			{
				isMoving = 0;

				// store final state
				records[sample][0] = gyroIntegrationZ;

				for(uint8_t i = 3; i < 6; i++)
					records[sample][i] = stats[i].mean;
			}
		}

		ConsolePrint("Sample acquired (%hu/%hu)\r\n", sample+1, 4);
	}

	ConsolePrint("\r\nRaw data: \r\n");

	for(uint16_t sample = 0; sample < IMU_CALIB_NUM_ROT_SAMPLES; sample++)
	{
		ConsolePrint("% .8f, % .8f, % .8f, % .8f, % .8f, % .8f, % .8f, % .8f, % .8f\r\n",
			records[sample][0], records[sample][1], records[sample][2],
			records[sample][3], records[sample][4], records[sample][5],
			records[sample][6], records[sample][7], records[sample][8]);

		chThdSleepMilliseconds(100);
	}

	matAccA.numRows = IMU_CALIB_NUM_ROT_SAMPLES*2;
	matAccb.numRows = IMU_CALIB_NUM_ROT_SAMPLES*2;
	matAccAInv.numCols = IMU_CALIB_NUM_ROT_SAMPLES*2;

	for(uint16_t sample = 0; sample < IMU_CALIB_NUM_ROT_SAMPLES; sample++)
	{
		MAT_ELEMENT(matAccA, sample*2+0, 0) = cosf(records[sample][0]);
		MAT_ELEMENT(matAccA, sample*2+0, 1) = sinf(records[sample][0]);
		MAT_ELEMENT(matAccA, sample*2+0, 2) = 1.0f;
		MAT_ELEMENT(matAccA, sample*2+0, 3) = 0.0f;
		MAT_ELEMENT(matAccA, sample*2+1, 0) = -sinf(records[sample][0]);
		MAT_ELEMENT(matAccA, sample*2+1, 1) = cosf(records[sample][0]);
		MAT_ELEMENT(matAccA, sample*2+1, 2) = 0.0f;
		MAT_ELEMENT(matAccA, sample*2+1, 3) = 1.0f;

		MAT_ELEMENT(matAccb, sample*2+0, 0) = records[sample][3];
		MAT_ELEMENT(matAccb, sample*2+1, 0) = records[sample][4];
	}

	arm_mat_pinv(&matAccA, &matAccAInv);
	arm_mat_mult_f32(&matAccAInv, &matAccb, &matAccx);

	float accTiltBias[2];
	accTiltBias[0] = MAT_ELEMENT(matAccx, 2, 0);
	accTiltBias[1] = MAT_ELEMENT(matAccx, 3, 0);

	// Output result
	ConsolePrint("\r\nGyro bias: %.8f, %.8f, %.8f\r\n", gyroBias[0], gyroBias[1], gyroBias[2]);
	ConsolePrint("Acc bias: %.8f, %.8f, %.8f (%.8f)\r\n", accBias[0], accBias[1], accBias[2], g);
	ConsolePrint("Acc tilt: %.8f, %.8f\r\n", accTiltBias[0], accTiltBias[1]);
	ConsolePrint("Temp: %.3f\r\n", statTempImu.mean);
	ConsolePrint("Mag bias: %.8f, %.8f, %.8f  (% .8f)\r\n", magBias[0], magBias[1], magBias[2], m);
	ConsolePrint("Temp: %.3f\r\n", statTempMag.mean);

	if(g > 9.81*1.01 || g < 9.81*0.99)
	{
		ConsolePrint("Accelerometer scale off by >1%%. This is BAD!\r\n");
	}

	chThdSleepMilliseconds(100);

	ConsolePrint("\r\n.imuCalib = {\r\n");
	ConsolePrint("\t.gyrBias = { %.8ff, %.8ff, %.8ff },\r\n", gyroBias[0], gyroBias[1], gyroBias[2]);
	ConsolePrint("\t.accBias = { %.8ff, %.8ff, %.8ff },\r\n", accBias[0], accBias[1], accBias[2], g);
	ConsolePrint("\t.accTiltBias = { %.8ff, %.8ff },\r\n", accTiltBias[0], accTiltBias[1]);
	ConsolePrint("\t.imuCalibTemp =  %.3ff,\r\n", statTempImu.mean);
	ConsolePrint("\t.magBias = { %.8ff, %.8ff, %.8ff },\r\n", magBias[0], magBias[1], magBias[2], m);
	ConsolePrint("\t.magCalibTemp = %.3ff,\r\n", statTempMag.mean);
	ConsolePrint("\t.calibrated = 1\r\n");
	ConsolePrint("}\r\n");

	ConsolePrint("\r\nThank you for running this calibration, good bye!\r\n");

	testStop();
}

static uint8_t dischargeKicker()
{
	TestKickerProgress("Discharging");
	KickerAutoDischarge();

	uint32_t time = 0;
	while(kicker.autoDischarge && ++time < TEST_KICKER_TIMEOUT)
		chThdSleepMilliseconds(1);

	if(time == TEST_KICKER_TIMEOUT)
	{
		TestKickerProgress("Discharging failed");
		testStop();
		return 0;
	}

	return 1;
}

static void testKicker(TestKickerResult* pResult)
{
	testStartup();
	pResult->chargingSpeed = 0.0f;
	pResult->straightVoltageDrop = 0.0f;
	pResult->chipVoltageDrop = 0.0f;

	LEDLeftSet(0.0f, 0.0f, 0.01f, 0.0f);
	LEDRightSet(0.0f, 0.0f, 0.01f, 0.0f);

	if(!dischargeKicker())
		return;

	LEDLeftSet(0.01f, 0.0f, 0.0f, 0.0f);
	LEDRightSet(0.01f, 0.0f, 0.0f, 0.0f);

	float maxVoltageBackup = kicker.config.maxVoltage;
	kicker.config.maxVoltage = 220.0f;
	float startVoltage = kicker.vCap;
	TestKickerProgress("Test: Charging");
	KickerEnableCharge(1);

	uint32_t time = 0;
	while(!KickerIsCharged() && ++time < TEST_KICKER_TIMEOUT)
		chThdSleepMilliseconds(1);

	if(time == TEST_KICKER_TIMEOUT)
	{
		dischargeKicker();
		TestKickerProgress("Charging failed");
		return;
	}

	pResult->chargingSpeed = (kicker.vCap - startVoltage) / (time * 1e-3f);

	TestKickerProgress("Test: Straight Kick");
	KickerFire(0.001f, KICKER_DEVICE_STRAIGHT);
	chThdSleepMilliseconds(100);
	pResult->straightVoltageDrop = kicker.vCapBeforeKick - kicker.vCap;

	TestKickerProgress("Recharging");
	KickerEnableCharge(1);
	time = 0;
	while(!KickerIsCharged() && ++time < TEST_KICKER_TIMEOUT)
		chThdSleepMilliseconds(1);

	if(time == TEST_KICKER_TIMEOUT)
	{
		dischargeKicker();
		TestKickerProgress("Recharging failed");
		return;
	}

	TestKickerProgress("Test: Chip Kick");
	KickerFire(0.001f, KICKER_DEVICE_CHIP);
	chThdSleepMilliseconds(100);
	pResult->chipVoltageDrop = kicker.vCapBeforeKick - kicker.vCap;

	LEDLeftSet(0.0f, 0.0f, 0.01f, 0.0f);
	LEDRightSet(0.0f, 0.0f, 0.01f, 0.0f);

	kicker.config.maxVoltage = maxVoltageBackup;
	if(!dischargeKicker())
		return;

	TestKickerProgress("Test complete");

	testStop();
}

static void doReasoningOnKicker(TestKickerResult* pResult)
{
	if(pResult->chargingSpeed < 15.0f)
		pResult->reasoning.charge = TEST_REASONING_RESULT_BAD;
	else if(pResult->chargingSpeed < 30.0f || pResult->chargingSpeed > 70.0f)
		pResult->reasoning.charge = TEST_REASONING_RESULT_WARNING;
	else
		pResult->reasoning.charge = TEST_REASONING_RESULT_OK;

	if(pResult->straightVoltageDrop < 5.0f)
		pResult->reasoning.straight = TEST_REASONING_RESULT_BAD;
	else if(pResult->straightVoltageDrop < 10.0f || pResult->straightVoltageDrop > 30.0f)
		pResult->reasoning.straight = TEST_REASONING_RESULT_WARNING;
	else
		pResult->reasoning.straight = TEST_REASONING_RESULT_OK;

	if(pResult->chipVoltageDrop < 5.0f)
		pResult->reasoning.chip = TEST_REASONING_RESULT_BAD;
	else if(pResult->chipVoltageDrop < 10.0f || pResult->chipVoltageDrop > 30.0f)
		pResult->reasoning.chip = TEST_REASONING_RESULT_WARNING;
	else
		pResult->reasoning.chip = TEST_REASONING_RESULT_OK;
}

static void testMotorCurrent(uint8_t motorId)
{
	// This test identifies the motor voltage to motor current FOPDT model
	// and recommends current controller gains for D and Q

	EMAFilter current;
	EMAFilterInit(&current, 0.95f);

	if(motorId > 4)
		return;

	MotorsSingle* pMotor = &motors.motor[motorId];

	if(power.usbPowered)
	{
		ConsolePrint("Motor current test cannot be performed in USB powered mode!\r\n");
		return;
	}

	ConsolePrint("\r\n### Motor %hu System Identification ###\r\n", (uint16_t)motorId);

	testStartup();

	// Step 1: increase voltage until current reaches 8A or 80% of supply voltage, use this voltage as step setpoint
	float stepVoltage = 0.0f;
	const float maxVoltage = power.vBat*0.8f;
	const float maxCurrent = 8.0f;

	while(stepVoltage < maxVoltage && current.value < maxCurrent)
	{
		stepVoltage += 0.1f;

		MotorsSetElectricalAngle(motorId, 0.0f, stepVoltage);

		for(uint16_t i = 0; i < 20; i++)
		{
			chThdSleepMilliseconds(1);
			float normCurrent = sqrtf(pMotor->avgCurrentDQ1ms[0]*pMotor->avgCurrentDQ1ms[0] + pMotor->avgCurrentDQ1ms[1]*pMotor->avgCurrentDQ1ms[1]);
			EMAFilterUpdate(&current, normCurrent);
		}
	}

	ConsolePrint("Step voltage: %.3fV\r\n", stepVoltage);

	// Step 2: use this voltage some more to get a stable average of current
	for(uint16_t i = 0; i < 500; i++)
	{
		chThdSleepMilliseconds(1);
		float normCurrent = sqrtf(pMotor->avgCurrentDQ1ms[0]*pMotor->avgCurrentDQ1ms[0] + pMotor->avgCurrentDQ1ms[1]*pMotor->avgCurrentDQ1ms[1]);
		EMAFilterUpdate(&current, normCurrent);
	}

	ConsolePrint("U: %.3fV, I: %.3fA\r\n", stepVoltage, current.value);

	const float stepCurrent = current.value;

	// Step 3: Perform step input and record data
	MotorsSetElectricalAngle(motorId, 0.0f, stepVoltage*0.25f);
	current.value *= 0.25f;
	for(uint16_t i = 0; i < 500; i++)
	{
		chThdSleepMilliseconds(1);
		float normCurrent = sqrtf(pMotor->avgCurrentDQ1ms[0]*pMotor->avgCurrentDQ1ms[0] + pMotor->avgCurrentDQ1ms[1]*pMotor->avgCurrentDQ1ms[1]);
		EMAFilterUpdate(&current, normCurrent);
	}

	const float startCurrent = current.value;

	ConsolePrint("U: %.3fV, I: %.3fA\r\n", stepVoltage*0.25f, startCurrent);

	MotorsDebugRecord(motorId, MOTOR_EXCHANGE_MOSI_RECORD_VOLT_AB_CUR_DQ);
	chThdSleepMilliseconds(50);

	MotorsSetElectricalAngle(motorId, 0.0f, stepVoltage);
	chThdSleepMilliseconds(500);

	for(uint8_t i = 0; i < 5; i++)
		MotorsSetOff(i);

	// Step 4: do system identification
	const float plantGain = stepCurrent / stepVoltage;
	const float plantDeadTime = 1.0f/20e3f;

	// current after two time constants, need to find corresponding time constant
	const float currentT2 = (0.865f*(stepCurrent-startCurrent) + startCurrent);

	arm_matrix_instance_q15 recordData;
	MotorsDebugGetRecord(&recordData);

	uint16_t indexStep = 0;
	uint16_t indexAboveT2 = 0;
	float currentNearT2[2] = {0};

	for(uint16_t row = 0; row < recordData.numRows; row++)
	{
		float dq = recordData.pData[row*recordData.numCols + 0];
		float setVoltage = recordData.pData[row*recordData.numCols + 1];

		dq = sqrtf(dq * (1 << 14)) * 1e-3f;
		setVoltage = sqrtf(setVoltage * (1 << 16)) * 1e-3f;

		if(setVoltage > 0.5f*stepVoltage && indexStep == 0)
			indexStep = row;

		if(dq > currentT2)
		{
			indexAboveT2 = row;
			currentNearT2[1] = dq;
			break;
		}

		currentNearT2[0] = dq;
	}

	if(indexStep == 0 || indexAboveT2 == 0)
	{
		ConsolePrint("Unable to identify step in recorded data\r\n");
		testStop();
		return;
	}

	const float dt = 1.0f/20e3f;

	float t2CrossingOffset = (currentT2-currentNearT2[0])/(currentNearT2[1]-currentNearT2[0])*dt;
	float t2Constant = (indexAboveT2-1)*dt + t2CrossingOffset - (indexStep+1)*dt;
	float plantTimeConstant = t2Constant*0.5f;

	float motorResistance = 1.0f/plantGain;
	float motorInductance = plantTimeConstant*motorResistance;

	ConsolePrint("Plant: K: %.3f, T: %.6fs, d: %.5fs\r\n", plantGain, plantTimeConstant, plantDeadTime);
	ConsolePrint("Motor: R: %.3f\u2126, L: %.6fH\r\n", motorResistance, motorInductance);

	// Step 5: compute controller gains
	float Kp[2] = { 0.55f*plantTimeConstant/(dt*plantGain), 0.2f*plantTimeConstant/(dt*plantGain) };
	float Ti[2] = { 1.0f*plantTimeConstant, 1.0f*plantTimeConstant };
	float Ki[2] = { Kp[0]/(Ti[0]*20e3f), Kp[1]/(Ti[1]*20e3f) };

	ConsolePrint("D Ctrl: Kp: %.3f, Ki: %.4f\r\n", Kp[0], Ki[0]);
	ConsolePrint("Q Ctrl: Kp: %.3f, Ki: %.4f\r\n", Kp[1], Ki[1]);

	testStop();
}

#define NUM_VOLTAGES_MOTOR_IDENT 4

static void testMotorIdent(uint8_t motorId)
{
	EMAFilter current;
	EMAFilter speed;
	EMAFilterInit(&current, 0.99f);
	EMAFilterInit(&speed, 0.95f);

	if(motorId > 3)
		return;

	MotorsSingle* pMotor = &motors.motor[motorId];

	if(power.usbPowered)
	{
		ConsolePrint("Motor current test cannot be performed in USB powered mode!\r\n");
		return;
	}

	ConsolePrint("\r\n### Motor %hu System Identification 2 ###\r\n", (uint16_t)motorId);

	testStartup();

	MotorsSetVoltageDQ(motorId, 0, 0);

	chThdSleepMilliseconds(500);

	ConsolePrint("It: %.3fA\r\n", power.iCur);

	arm_matrix_instance_f32 stepsA = { NUM_VOLTAGES_MOTOR_IDENT, 2, (float[NUM_VOLTAGES_MOTOR_IDENT*2]){} };
	arm_matrix_instance_f32 stepsb = { NUM_VOLTAGES_MOTOR_IDENT, 1, (float[NUM_VOLTAGES_MOTOR_IDENT]){} };

	float R = 0.341f; // TODO: take from somewhere

	float torqueConstant = 0.0f;
	float setVolt;

	for(int32_t i = 0; i < NUM_VOLTAGES_MOTOR_IDENT; i++)
	{
		setVolt = 1.0f + i;

		for(float f = 0; f < 1.0f; f += 0.01f)
		{
			MotorsSetCurrentDVoltageQ(motorId, 0.0f, setVolt+f-1.0f);

			chThdSleepMilliseconds(1);
		}

		chThdSleepMilliseconds(500);

		current.value = pMotor->avgCurrentDQ1ms[1];
		speed.value = pMotor->hallVelocity;

		for(uint16_t t = 0; t < 1000; t++)
		{
			EMAFilterUpdate(&current, pMotor->avgCurrentDQ1ms[1]);
			EMAFilterUpdate(&speed, pMotor->hallVelocity);

			chThdSleepMilliseconds(1);
		}

		float U = setVolt;
		float I = current.value;
		float wEl = speed.value * 8.0f; // 8 = number of pole pairs

		float K = (U - R*I)/wEl;
		torqueConstant += K;

		MAT_ELEMENT(stepsA, i, 0) = speed.value;
		MAT_ELEMENT(stepsA, i, 1) = 1;
		MAT_ELEMENT(stepsb, i, 0) = current.value;

		ConsolePrint("Uq: %.3fV, Iq: %.3fA, w: %.3frad/s, K: %.6f\r\n",
				setVolt, current.value, speed.value, K);
	}

	torqueConstant *= 1.0f/(float)NUM_VOLTAGES_MOTOR_IDENT;

	for(uint8_t i = 0; i < NUM_VOLTAGES_MOTOR_IDENT; i++)
	{
		MAT_ELEMENT(stepsb, i, 0) *= torqueConstant * 1.5f * 8.0f;
	}

	arm_matrix_instance_f32 stepsx = { 2, 1, (float[2]){} };

	arm_matrix_instance_f32 stepsAinv = { 2, NUM_VOLTAGES_MOTOR_IDENT, (float[NUM_VOLTAGES_MOTOR_IDENT*2]){} };
	arm_mat_pinv(&stepsA, &stepsAinv);
	arm_mat_mult_f32(&stepsAinv, &stepsb, &stepsx);

	float viscousFriction = MAT_ELEMENT(stepsx, 0, 0);
	float coulombFriction = MAT_ELEMENT(stepsx, 1, 0);

	ConsolePrint("K: %.6fNm/A, b: %.8fNm*s, C: %.6fNm\r\n", torqueConstant, viscousFriction, coulombFriction);

	MotorsSetCurrentDQ(motorId, 0.0f, 0.0f);

	while(pMotor->hallVelocity > 0.0f)
		chThdSleepMilliseconds(10);

	chThdSleepMilliseconds(500);

	const float iCoulomb = coulombFriction / (torqueConstant * 1.5f * 8.0f);

	MotorsSetCurrentDQ(motorId, 0.0f, current.value);
	uint32_t tUpStartUs = SysTimeUSec();
	while(pMotor->hallVelocity < speed.value*0.632f)
		chThdSleepMilliseconds(1);

	float tUp = (SysTimeUSec() - tUpStartUs) * 1e-6f;

	MotorsSetCurrentDVoltageQ(motorId, 0.0f, setVolt);
	while(pMotor->hallVelocity < speed.value*0.95f)
		chThdSleepMilliseconds(1);

	chThdSleepMilliseconds(1000);

	MotorsSetCurrentDQ(motorId, 0.0f, iCoulomb);
	uint32_t tDownStartUs = SysTimeUSec();
	while(pMotor->hallVelocity > speed.value*0.368f)
		chThdSleepMilliseconds(1);

	float tDown = (SysTimeUSec() - tDownStartUs) * 1e-6f;

	ConsolePrint("tUp: %.3fs, tDown: %.3fs\r\n", tUp, tDown);

	float K = speed.value / (current.value - iCoulomb);
	float T = (tUp + tDown) * 0.5f;

	float b = 1.0f / K;
	float J = T / K;

	ConsolePrint("Plant: K: %.3f, T: %.3fs\r\n", K, T);
	ConsolePrint("Load: b: %.8f, J: %.6f\r\n", b, J);

	MotorsSetOff(motorId);

	testStop();
}

#define NUM_VOLTAGES_IDENT_ALL 3

static void testMotorIdentDriveAll()
{
	EMAFilter current[4];
	EMAFilter speed[4];

	for(uint8_t i = 0; i < 4; i++)
	{
		EMAFilterInit(&current[i], 0.99f);
		EMAFilterInit(&speed[i], 0.95f);
	}

	if(power.usbPowered)
	{
		ConsolePrint("Motor current test cannot be performed in USB powered mode!\r\n");
		return;
	}

	ConsolePrint("\r\n### Parallel Motor System Identification ###\r\n");

	testStartup();

	for(uint8_t i = 0; i < 4; i++)
		MotorsSetVoltageDQ(i, 0, 0);

	chThdSleepMilliseconds(500);

	float speeds[4][8];
	float currents[4][4];

	float R = 0.341f; // TODO: take from somewhere

	float torqueConstant = 0.0f;
	float setVolt;
	float avgSpeed;

	for(int32_t voltInt = 0; voltInt < NUM_VOLTAGES_IDENT_ALL; voltInt++)
	{
		setVolt = 1.0f + voltInt;

		for(float f = 0; f < 1.0f; f += 0.01f)
		{
			for(uint8_t i = 0; i < 4; i++)
				MotorsSetCurrentDVoltageQ(i, 0.0f, setVolt+f-1.0f);

			chThdSleepMilliseconds(1);
		}

		chThdSleepMilliseconds(500);

		for(uint8_t i = 0; i < 4; i++)
		{
			current[i].value = motors.motor[i].avgCurrentDQ1ms[1];
			speed[i].value = motors.motor[i].hallVelocity;
		}

		systime_t prev = chVTGetSystemTimeX();
		systime_t next = prev+US2ST(1000);

		for(uint16_t t = 0; t < 1000; t++)
		{
			for(uint8_t i = 0; i < 4; i++)
			{
				EMAFilterUpdate(&current[i], motors.motor[i].avgCurrentDQ1ms[1]);
				EMAFilterUpdate(&speed[i], motors.motor[i].hallVelocity);
			}

			chThdSleepUntilWindowed(prev, next);
			prev = next;
			next += US2ST(1000);
		}

		avgSpeed = 0.0f;
		for(uint8_t i = 0; i < 4; i++)
		{
			currents[i][voltInt] = current[i].value;
			speeds[i][voltInt*2+0] = speed[i].value;
			avgSpeed += speed[i].value;

			float U = setVolt;
			float I = current[i].value;
			float wEl = speed[i].value * 8.0f; // 8 = number of pole pairs

			float K = (U - R*I)/wEl;
			torqueConstant += K;

			ConsolePrint("Uq: %.3fV, Iq: %.3fA, w: %.3frad/s, K: %.6f\r\n",
					setVolt, I, speed[i].value, K);
			NetworkPrint("Uq: %.3fV, Iq: %.3fA, w: %.3frad/s, K: %.6f\n",
					setVolt, I, speed[i].value, K);
		}

		avgSpeed *= 0.25f;
	}

	torqueConstant *= 1.0f/((float)NUM_VOLTAGES_IDENT_ALL*4.0f);

	ConsolePrint("K: %.6fNm/A\r\n", torqueConstant);
	NetworkPrint("K: %.6fNm/A\n", torqueConstant);

	arm_matrix_instance_f32 stepsA = { NUM_VOLTAGES_IDENT_ALL, 2, 0 };
	arm_matrix_instance_f32 stepsb = { NUM_VOLTAGES_IDENT_ALL, 1, 0 };

	float cFrictionAvg = 0.0f;
	float iCoulomb[4];

	for(uint8_t m = 0; m < 4; m++)
	{
		stepsA.pData = speeds[m];
		stepsb.pData = currents[m];
		for(uint8_t j = 0; j < NUM_VOLTAGES_IDENT_ALL; j++)
		{
			MAT_ELEMENT(stepsb, j, 0) *= torqueConstant * 1.5f * 8.0f;
			MAT_ELEMENT(stepsA, j, 1) = 1;
		}

		arm_matrix_instance_f32 stepsx = { 2, 1, (float[2]){} };
		arm_matrix_instance_f32 stepsAinv = { 2, NUM_VOLTAGES_IDENT_ALL, (float[NUM_VOLTAGES_IDENT_ALL*2]){} };

		arm_mat_pinv(&stepsA, &stepsAinv);
		arm_mat_mult_f32(&stepsAinv, &stepsb, &stepsx);

		float viscousFriction = MAT_ELEMENT(stepsx, 0, 0);
		float coulombFriction = MAT_ELEMENT(stepsx, 1, 0);

		cFrictionAvg += coulombFriction;
		iCoulomb[m] = coulombFriction / (torqueConstant * 1.5f * 8.0f);

		ConsolePrint("b: %.8fNm*s, C: %.6fNm, Ic: %.4fA\r\n", viscousFriction, coulombFriction, iCoulomb[m]);
		NetworkPrint("b: %.8fNm*s, C: %.6fNm, Ic: %.4fA\n", viscousFriction, coulombFriction, iCoulomb[m]);
	}

	cFrictionAvg *= 0.25f;

	ConsolePrint("C: %.6f, w: %.3f\r\n", cFrictionAvg, avgSpeed);
	NetworkPrint("C: %.6f, w: %.3f\n", cFrictionAvg, avgSpeed);

	for(uint8_t i = 0; i < 4; i++)
		MotorsSetCurrentDQ(i, 0.0f, iCoulomb[i]);

	float tDown[4];
	uint32_t tDownStartUs = SysTimeUSec();
	uint8_t fastMotors = 0x0F;
	float tDownAvg = 0.0f;

	while(fastMotors)
	{
		for(uint8_t i = 0; i < 4; i++)
		{
			if(motors.motor[i].hallVelocity < avgSpeed*0.368f && (fastMotors & (1 << i)))
			{
				tDown[i] = (SysTimeUSec() - tDownStartUs) * 1e-6f;
				tDownAvg += tDown[i];
				fastMotors &= ~(1 << i);
				ConsolePrint("M%u tDown: %.3f\r\n", i, tDown[i]);
				NetworkPrint("M%u tDown: %.3f\n", i, tDown[i]);
			}
		}

		chThdSleepMilliseconds(1);
	}

	tDownAvg *= 0.25f;
	ConsolePrint("T: %.3f\r\n", tDownAvg);
	NetworkPrint("T: %.3f\n", tDownAvg);

	for(uint8_t i = 0; i < 4; i++)
		MotorsSetVoltageDQ(i, 0.0f, 0.0f);

	chThdSleepMilliseconds(1000);

	for(uint8_t i = 0; i < 4; i++)
	{
		MotorsSetOff(i);
	}

	testStop();
}

static void testMotorResistance(uint8_t motorId)
{
	if(power.usbPowered)
	{
		ConsolePrint("Motor resistance test cannot be performed in USB powered mode!\r\n");
		return;
	}

	testStartup();

	ConsolePrint("Testing motor resistances on motor %hu\r\n", motorId);

	const float setVoltage = 2.0f;
	const MotorsSingle* pMotor = &motors.motor[motorId];

	float RphMin[3] = {INFINITY, INFINITY, INFINITY};
	float RphMax[3] = {0, 0, 0};

	MotorsSetElectricalAngle(motorId, 0, 0);

	chThdSleepMilliseconds(200);

	ConsolePrint("It: %.3f\r\n", power.iCur);

	for(float angle = 0.0f; angle < 360.0f; angle += 120.0f)
	{
		const float angleRad = angle * M_PI / 180.0f;

		MotorsSetElectricalAngle(motorId, angleRad, setVoltage);
		chThdSleepMilliseconds(1000);

		float Iph[3];
		memcpy(Iph, pMotor->avgCurrentUVW1ms, sizeof(float)*3);

		// Va = Valpha;
		// Vb = -1/2 * Valpha + sqrt(3)/2*Vbeta;
		// Vc = -1/2 * Valpha - sqrt(3)/2*Vbeta;

		float Vph[3];
		Vph[0] = pMotor->setVoltage[0];
		Vph[1] = -0.5f*pMotor->setVoltage[0] + 0.866f*pMotor->setVoltage[1];
		Vph[2] = -0.5f*pMotor->setVoltage[0] - 0.866f*pMotor->setVoltage[1];

		float Rph[3];

		for(uint8_t i = 0; i < 3; i++)
		{
			Rph[i] = Vph[i] / Iph[i];
			RphMin[i] = fminf(RphMin[i], Rph[i]);
			RphMax[i] = fmaxf(RphMax[i], Rph[i]);
		}

		ConsolePrint("Angle: %.1f\r\n", angle);
		ConsolePrint("vUVW: % .3fV / % .3fV / % .3fV\r\n", Vph[0], Vph[1], Vph[2]);
		ConsolePrint("iUVW: % .3fA / % .3fA / % .3fA\r\n", Iph[0], Iph[1], Iph[2]);
		ConsolePrint("rUVW: % .3f\u2126 / % .3f\u2126 / % .3f\u2126\r\n", Rph[0], Rph[1], Rph[2]);
		ConsolePrint("It: %.3f\r\n", power.iCur);
	}

	ConsolePrint("rSpr: % .3f\u2126 / % .3f\u2126 / % .3f\u2126\r\n",
			RphMax[0]-RphMin[0], RphMax[1]-RphMin[1], RphMax[2]-RphMin[2]);

	MotorsSetOff(motorId);

	testStop();
}

static float rotationDecelerationModel(float wStart, float wEnd, float viscous, float coulomb, float inertia)
{
	float wNow = wStart;
	float t = 0.0f;

	while(wNow > wEnd)
	{
		float wDot = (-viscous*wNow - coulomb)/inertia;
		wNow += wDot*0.001f;
		t += 0.001f;
	}

	return t;
}

static void testRotationIdent()
{
	if(power.usbPowered)
	{
		ConsolePrint("Motor resistance test cannot be performed in USB powered mode!\r\n");
		return;
	}

	testStartup();

	ConsolePrint("Identifying rotation model parameters\r\n");
	NetworkPrint("Identifying rotation model parameters");

#define NUM_VOLTAGES_ROT_IDENT 16
	arm_matrix_instance_f32 matA = { NUM_VOLTAGES_ROT_IDENT, 2, (float[NUM_VOLTAGES_ROT_IDENT*2]){} };
	arm_matrix_instance_f32 vecB = { NUM_VOLTAGES_ROT_IDENT, 1, (float[NUM_VOLTAGES_ROT_IDENT]){} };


	uint16_t row = 0;
	for(float vol = 0.4f; vol < 0.4f+(NUM_VOLTAGES_ROT_IDENT*0.1f); vol += 0.1f)
	{
		// set test voltage
		for(uint8_t i = 0; i < 4; i++)
			MotorsSetVoltageDQ(i, 0.0f, vol);

		// spin up
		systime_t prev = chVTGetSystemTimeX();
		systime_t next = prev+US2ST(1000);

		uint16_t t;
		for(t = 0; t < 2000; t++)
		{
			if(fabsf(robot.sensors.gyr.rotVel[2]) > 30.0f)
				break;

			chThdSleepUntilWindowed(prev, next);
			prev = next;
			next += US2ST(1000);
		}

		if(t < 2000)
		{
			// spin up incomplete, robot too fast
			NetworkPrint("Spinning too fast. STOP. (%.2f)", robot.sensors.gyr.rotVel[2]);
			break;
		}

		// take samples
		prev = chVTGetSystemTimeX();
		next = prev+US2ST(1000);

		float rotVelSum = 0.0f;
		float motCurSum = 0.0f;
		for(t = 0; t < 2000; t++)
		{
			rotVelSum += robot.sensors.gyr.rotVel[2];
			motCurSum += (motors.motor[0].avgCurrentDQ1ms[1] +
					motors.motor[1].avgCurrentDQ1ms[1] +
					motors.motor[2].avgCurrentDQ1ms[1] +
					motors.motor[3].avgCurrentDQ1ms[1]) * 0.25f;

			chThdSleepUntilWindowed(prev, next);
			prev = next;
			next += US2ST(1000);
		}

		const float rotVelAvg = rotVelSum * 1.0f/2000.0f;
		const float motCurAvg = motCurSum * 1.0f/2000.0f;

		if(fabsf(rotVelAvg) < 1.0f)
		{
			// robot did not rotate, maybe it's not on the ground?
			NetworkPrint("No rotation detected. SKIP. (%.3f)", rotVelAvg);
			continue;
		}

		NetworkPrint("%.1f, %.4f, %.3f", vol, motCurAvg, rotVelAvg);

		MAT_ELEMENT(matA, row, 0) = rotVelAvg;
		MAT_ELEMENT(matA, row, 1) = 1.0f;
		MAT_ELEMENT(vecB, row, 0) = motCurAvg * botParams.driveTrain.motor.Km
				* 1/botParams.physical.wheelRadius
				* botParams.physical.botRadius * 4.0f;

		row++;
	}

	if(row < 2)
	{
		// not enough samples
		for(uint8_t i = 0; i < 4; i++)
			MotorsSetOff(i);

		testStop();

		return;
	}

	// determine model friction parameters
	matA.numRows = row;
	vecB.numRows = row;

	arm_matrix_instance_f32 vecX = { 2, 1, (float[2]){} };
	arm_matrix_instance_f32 matAInv = { 2, NUM_VOLTAGES_ROT_IDENT, (float[NUM_VOLTAGES_ROT_IDENT*2]){} };
	matAInv.numCols = row;

	arm_mat_pinv(&matA, &matAInv);
	arm_mat_mult_f32(&matAInv, &vecB, &vecX);

	const float viscousFriction = MAT_ELEMENT(vecX, 0, 0);
	const float coulombFriction = MAT_ELEMENT(vecX, 1, 0);

	// set motors to zero toque
	for(uint8_t i = 0; i < 4; i++)
	{
		MotorsSetCurrentDQ(i, 0.0f, 0.0f);
	}

	// sample deceleration
	float tStart = SysTime();
	float startRotVel = robot.sensors.gyr.rotVel[2];

	while(fabsf(robot.sensors.gyr.rotVel[2]) > 2.0f)
	{
		chThdSleepMilliseconds(1);
	}

	float tEnd = SysTime();
	float endRotVel = robot.sensors.gyr.rotVel[2];

	// determine inertia
	float dt = (tEnd - tStart);
	float decel = (endRotVel - startRotVel)/dt;
	float inertia = -coulombFriction / decel; // iniatial guess without viscous friction

	// run a deceleration model with viscous friction
	float inertiaLow = inertia*0.9f;
	float inertiaHigh = inertia*1.1f;

	float tEnd1 = rotationDecelerationModel(startRotVel, endRotVel, viscousFriction, coulombFriction, inertiaLow);
	float tEnd2 = rotationDecelerationModel(startRotVel, endRotVel, viscousFriction, coulombFriction, inertiaHigh);

	// find optimal inertia which reaches 'we' after 'dt' seconds
	float inertiaOpt = (dt-tEnd1)/(tEnd2-tEnd1) * (inertiaHigh-inertiaLow) + inertiaLow;

	NetworkPrint("ws: %.3f, we: %.3f, dt: %.5f", startRotVel, endRotVel, (tEnd-tStart));
	NetworkPrint("Bw: %.7f, Cw: %.4f, I: %.6f (%.6f)", viscousFriction, coulombFriction, inertiaOpt, inertia);

	// disable motors
	for(uint8_t i = 0; i < 4; i++)
	{
		MotorsSetOff(i);
	}

	testStop();
}

#define TRACTION_SAMPLES 10

void TestMotorTraction()
{
	testStartup();

	const float dir[4] = {1, -1, 1, -1};

	float turnVol[4];
	float result[4] = {0, 0, 0, 0};

	for(uint16_t sample = 0; sample < TRACTION_SAMPLES; sample++)
	{
		memset(turnVol, 0, sizeof(float)*4);

		for(float setVol = 0.0f; setVol < 5.0f; setVol += 0.01f)
		{
			uint8_t motorMissing = 0;
			for(uint8_t mot = 0; mot < 4; mot++)
			{
				if(fabsf(motors.motor[mot].encoderVelocity) > 20.0f && turnVol[mot] == 0.0f)
					turnVol[mot] = setVol;

				if(turnVol[mot] == 0.0f)
					motorMissing = 1;

				MotorsSetVoltageDQ(mot, 0.0f, dir[mot]*setVol);
			}

			if(motorMissing == 0)
				break;

			chThdSleepMilliseconds(10);
		}

		ConsolePrint("Motor break-away voltages:\r\n");
		for(uint8_t mot = 0; mot < 4; mot++)
		{
			MotorsSetVoltageDQ(mot, 0, 0);
			ConsolePrint("M%hu: %.3fV\r\n", (uint16_t)mot, turnVol[mot]);
			result[mot] += turnVol[mot];
		}

		chThdSleepMilliseconds(200);
	}

	ConsolePrint("Motor break-away average voltages:\r\n");
	for(uint8_t mot = 0; mot < 4; mot++)
	{
		ConsolePrint("M%hu: %.3fV\r\n", (uint16_t)mot, result[mot]/TRACTION_SAMPLES);
	}

	testStop();
}

#define SPIN_UP_SAMPLES 300
#define NUM_VOLTAGES 8
const float testVoltages[NUM_VOLTAGES] = {-3.0f, -2.5f, -2.0f, -1.5f, 1.5f, 2.0f, 2.5f, 3.0f};

void TestMotorDynamics(uint8_t motorId, TestMotorDynamicsResult* pResult)
{
	EMAFilter current;
	EMAFilter speed;

	static float spinUp[SPIN_UP_SAMPLES];

	TestProgress prog;
	prog.testId = IC_TEST_MOT_DYN;
	prog.motorId = motorId;
	prog.percentComplete = 0;

	if(test.pDriveProg)
		(*test.pDriveProg)(&prog);

	pResult->motorId = motorId;

	testStartup();

	EMAFilterInit(&current, 0.9f);
	EMAFilterInit(&speed, 0.95f);

	for(uint16_t j = 0; j < 1000; j++)
	{
		EMAFilterUpdate(&current, power.iCur);
		chThdSleepMilliseconds(1);
	}

	float mainOffset = current.value;

	prog.percentComplete = 1/18.0f*100.0f;
	if(test.pDriveProg)
		(*test.pDriveProg)(&prog);

//	ConsolePrint("Main current offset: %.3fA\r\n", mainOffset);

	float avgDamping = 0.0f;
	for(int16_t i = 0; i < NUM_VOLTAGES; i++)
	{
		float volt = testVoltages[i];

		if(motorId > 3)
			volt *= 0.5f;

		MotorsSetVoltageDQ(motorId, 0.0f, volt);

		chThdSleepMilliseconds(300);

		for(uint16_t j = 0; j < 700; j++)
		{
			EMAFilterUpdate(&current, power.iCur-mainOffset);

			if(motorId > 3)
				EMAFilterUpdate(&speed, motors.motor[4].hallVelocity);
			else
				EMAFilterUpdate(&speed, motors.motor[motorId].encoderVelocity);

			chThdSleepMilliseconds(1);
		}

		prog.percentComplete = ((float)(i+1))/NUM_VOLTAGES*50.0f;
		if(test.pDriveProg)
			(*test.pDriveProg)(&prog);

		float K;
		float R;
		float w;
		if(motorId > 3)
		{
			K = 5.179e-3f;
			R = 0.257f;
			if(volt < 0)
				w = -speed.value;
			else
				w = speed.value;
		}
		else
		{
			K = botParams.driveTrain.motor.Ke;
			R = botParams.driveTrain.motor.R;
			w = speed.value;
		}

		float b = (volt - K*w)/(R*w)*K;

//		float Kn = w/volt;
//		ConsolePrint("U: %.3fV, I: %.3fA, v: %.3frad/s b: %.9f Kn: %.3f\r\n", volt, current.value, w, b, Kn);

		avgDamping += b;
	}

	MotorsSetVoltageDQ(motorId, 0, 0);

	avgDamping /= (float)NUM_VOLTAGES;

//	ConsolePrint("Avg. Damping: %.9f\r\n", avgDamping);

	chThdSleepMilliseconds(200);

//	ConsolePrint("Doing spin up tests\r\n");

	float avgInertia = 0.0f;

	for(int16_t i = 0; i < NUM_VOLTAGES; i++)
	{
		float volt = testVoltages[i];

		if(motorId > 3)
			volt *= 0.5f;

		MotorsSetVoltageDQ(motorId, 0.0f, volt);

		for(uint16_t j = 0; j < SPIN_UP_SAMPLES; j++)
		{
			if(motorId > 3)
			{
				if(volt < 0)
					spinUp[j] = -motors.motor[4].hallVelocity;
				else
					spinUp[j] = motors.motor[4].hallVelocity;
			}
			else
			{
				spinUp[j] = motors.motor[motorId].encoderVelocity;
			}

			chThdSleepMilliseconds(1);
		}

		MotorsSetVoltageDQ(motorId, 0, 0);

		avgInertia += calculateDriveTrainInertia(spinUp, SPIN_UP_SAMPLES, volt, motorId, avgDamping);

		chThdSleepMilliseconds(SPIN_UP_SAMPLES);

		prog.percentComplete = ((float)(i+1))/NUM_VOLTAGES*50.0f+50.0f;
		if(test.pDriveProg)
			(*test.pDriveProg)(&prog);
	}

	avgInertia /= (float)NUM_VOLTAGES;

//	ConsolePrint("Avg. Inertia: %.9f\r\n", avgInertia);

	pResult->damping = avgDamping;
	pResult->inertia = avgInertia;
	pResult->cur = current.value;
	pResult->speed = speed.value;

	prog.percentComplete = 100;
	if(test.pDriveProg)
		(*test.pDriveProg)(&prog);

	testStop();

//	MotorsSetDampingCoefficient(motorId, avgDamping);

	doReasoningOnMotor(pResult);

	printTestMotorDynamicsResult(pResult);
}

static float evalMotor(float R, float L, float K, float b, float J, float T, float U, const float* pW, uint16_t i)
{
	float alpha = ((R*T*T+2.0f*L*T)*b+(K*K*T*T+2.0f*J*R*T+4.0f*J*L));
	float beta = (2.0f*R*T*T*b+(2.0f*K*K*T*T-8.0f*J*L));
	float gamma = (R*T*T-2.0f*L*T)*b+(K*K*T*T-2.0f*J*R*T+4.0f*J*L);

	float w1 = pW[i-1];
	float w2 = pW[i-2];

	return K*T*T/alpha * U + 2.0f*K*T*T/alpha * U + K*T*T/alpha * U - beta/alpha * w1 - gamma/alpha * w2;
}

static float calculateDriveTrainInertia(const float* pSamples, uint16_t numSamples, float voltage, uint16_t motorId, float b)
{
	float K;
	float R;
	float L;
	float J;
	if(motorId > 3)
	{
		K = 5.179e-3f;
		R = 0.257f;
		L = 0.026e-3f;
		J = 4.8e-06f;
	}
	else
	{
		K = botParams.driveTrain.motor.Ke;
		R = botParams.driveTrain.motor.R;
		L = botParams.driveTrain.motor.L;
		J = 3.4e-05f;
	}
	const float T = 1e-3f;

	float lastDiff = 0;
	uint16_t eval;
	for(eval = 0; eval < 2000; eval++)
	{
		float diff = 0;
		for(uint16_t i = 2; i < numSamples; i++)
		{
			float w0 = evalMotor(R, L, K, b, J, T, voltage, pSamples, i);
			diff += pSamples[i] - w0;
		}

		if((diff > 0 && lastDiff < 0) || (diff < 0 && lastDiff > 0))
			break;

		if(voltage > 0)
		{
			if(diff > 0)
				J -= 2e-8f;
			else
				J += 2e-8f;
		}
		else
		{
			if(diff > 0)
				J += 2e-8f;
			else
				J -= 2e-8f;
		}

		lastDiff = diff;
	}

//	ConsolePrint("Optimization terminated after %hu calls, J: %.9f\r\n", eval, J);

	return J;
}

static void doReasoningOnDriveMotor(TestMotorDynamicsResult* pResult)
{
	const float damping = 164e-6f;
	const float inertia = 4.5e-6f;
	const float cur = 0.065f;
	const float speed = 132.0f;

	const float warn = 0.3f;
	const float err = 0.5f;

	const float warnLow = 1.0f - warn;
	const float warnHigh = 1.0f + warn;
	const float errLow = 1.0f - err;
	const float errHigh = 1.0f + err;

	if(pResult->damping < damping*warnLow || pResult->damping > damping*warnHigh)
		pResult->reasoning.damping = TEST_REASONING_RESULT_WARNING;

	if(pResult->damping < damping*errLow || pResult->damping > damping*errHigh)
		pResult->reasoning.damping = TEST_REASONING_RESULT_BAD;

	if(pResult->inertia < inertia*warnLow || pResult->inertia > inertia*warnHigh)
		pResult->reasoning.inertia = TEST_REASONING_RESULT_WARNING;

	if(pResult->inertia < inertia*errLow || pResult->inertia > inertia*errHigh)
		pResult->reasoning.inertia = TEST_REASONING_RESULT_BAD;

	if(pResult->cur < cur*warnLow || pResult->cur > cur*warnHigh)
		pResult->reasoning.cur = TEST_REASONING_RESULT_WARNING;

	if(pResult->cur < cur*errLow || pResult->cur > cur*errHigh)
		pResult->reasoning.cur = TEST_REASONING_RESULT_BAD;

	if(pResult->speed < speed*warnLow || pResult->speed > speed*warnHigh)
		pResult->reasoning.vel = TEST_REASONING_RESULT_WARNING;

	if(pResult->speed < speed*errLow || pResult->speed > speed*errHigh)
		pResult->reasoning.vel = TEST_REASONING_RESULT_BAD;
}

static void doReasoningOnDribblerMotor(TestMotorDynamicsResult* pResult)
{
	pResult->reasoning.damping = TEST_REASONING_RESULT_OK;
	pResult->reasoning.inertia = TEST_REASONING_RESULT_OK;

	if(pResult->cur < 0.18f || pResult->cur > 0.28f)
		pResult->reasoning.cur = TEST_REASONING_RESULT_WARNING;

	if(pResult->cur < 0.13f || pResult->cur > 0.33f)
		pResult->reasoning.cur = TEST_REASONING_RESULT_BAD;

	if(pResult->speed < 225.0f || pResult->speed > 325.0f)
		pResult->reasoning.vel = TEST_REASONING_RESULT_WARNING;

	if(pResult->speed < 175.0f || pResult->speed > 375.0f)
		pResult->reasoning.vel = TEST_REASONING_RESULT_BAD;
}

static void doReasoningOnMotor(TestMotorDynamicsResult* pResult)
{
	pResult->reasoning.cur = TEST_REASONING_RESULT_OK;
	pResult->reasoning.damping = TEST_REASONING_RESULT_OK;
	pResult->reasoning.inertia = TEST_REASONING_RESULT_OK;
	pResult->reasoning.vel = TEST_REASONING_RESULT_OK;

	if (pResult->motorId < 4)
		doReasoningOnDriveMotor(pResult);
	else
		doReasoningOnDribblerMotor(pResult);
}

static const char* getReasoningString(uint8_t result)
{
	if(result == TEST_REASONING_RESULT_OK)
	{
		return "OK";
	}
	else if(result == TEST_REASONING_RESULT_WARNING)
	{
		return "WARNING";
	}
	else if(result == TEST_REASONING_RESULT_BAD)
	{
		return "BAD";
	}

	return "UNKNOWN";
}

static void printTestMotorDynamicsResult(TestMotorDynamicsResult* pResult)
{
	ConsolePrint("Finished dynamics test on motor %hu", pResult->motorId);

	if(pResult->motorId < 4)
		ConsolePrint(" (drive motor)\r\n");
	else
		ConsolePrint(" (dribbler motor)\r\n");

	ConsolePrint("\tdamping = %.9f (%s)\r\n", pResult->damping, getReasoningString(pResult->reasoning.damping));
	ConsolePrint("\tinertia = %.9f (%s)\r\n", pResult->inertia, getReasoningString(pResult->reasoning.inertia));
	ConsolePrint("\tcurrent = %f (%s)\r\n", pResult->cur, getReasoningString(pResult->reasoning.cur));
	ConsolePrint("\tspeed = %f (%s)\r\n", pResult->speed, getReasoningString(pResult->reasoning.vel));
}
