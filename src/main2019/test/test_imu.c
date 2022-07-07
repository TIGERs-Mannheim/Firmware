/*
 * test_imu.c
 *
 *  Created on: 21.05.2022
 *      Author: AndreR
 */

#include "test_imu.h"

#include "test.h"
#include "arm_math.h"
#include "util/arm_mat_util_f32.h"
#include "util/signal_statistics.h"
#include "util/console.h"
#include "spi4.h"

#define IMU_CALIB_NUM_SAMPLES 9

void TestImuCalib()
{
	static const char* positionNames[] = {
			"Standing", "Front-Right Wheel down", "Rear-Right Wheel down",
			"Rear-Left Wheel down", "Front-Left Wheel down", "Upside down",
			"Standing", "Upside down", "Standing" };

	TestModeStartup();

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

	TestModeExit();
}
