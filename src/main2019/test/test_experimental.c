#include "test_experimental.h"
#include "tiger_bot.h"
#include "robot/robot.h"
#include "robot/skill_basics.h"
#include "robot/ctrl_panthera.h"
#include "util/network_print.h"
#include "hal/sys_time.h"
#include "math/ema_filter.h"
#include "math/arm_mat_util_f32.h"
#include "math/angle_math.h"
#include "dev/motors.h"
#include "dev/leds_front.h"
#include "test_common.h"
#include "test_data.h"
#include "util/test.h"
#include <float.h>
#include <stdio.h>

static void testMotorPhaseResistance(Test* pTest)
{
	const TestArgMot* pArg = (const TestArgMot*)TestGetArgument(pTest);
	uint8_t motorId = pArg->motorId;

	printf("Testing motor phase resistances on motor %hu\r\n", motorId);

	if(tigerBot.powerControl.state == POWER_CONTROL_STATE_USB_POWERED)
	{
		printf("Test cannot be performed in USB powered mode!\r\n");
		return;
	}

	if(motorId > 4)
		return;

	TestModeStartup();

	const float setVoltage = 2.0f;
	McuMotor* pMotor = &devMotors.mcu[motorId];

	float RphMin[3] = {INFINITY, INFINITY, INFINITY};
	float RphMax[3] = {0, 0, 0};

	McuMotorSetElectricalAngle(pMotor, 0, 0);

	chThdSleepMilliseconds(200);

	printf("It: %.3f\r\n", tigerBot.powerControl.iCur);

	for(float angle = 0.0f; angle < 360.0f; angle += 120.0f)
	{
		const float angleRad = angle * M_PI / 180.0f;

		McuMotorSetElectricalAngle(pMotor, angleRad, setVoltage);
		chThdSleepMilliseconds(1000);

		float Iph[3];
		memcpy(Iph, pMotor->meas.avgCurrentUVW1ms_A, sizeof(float)*3);

		// Va = Valpha;
		// Vb = -1/2 * Valpha + sqrt(3)/2*Vbeta;
		// Vc = -1/2 * Valpha - sqrt(3)/2*Vbeta;

		float Vph[3];
		Vph[0] = pMotor->setVoltage_V[0];
		Vph[1] = -0.5f*pMotor->setVoltage_V[0] + 0.866f*pMotor->setVoltage_V[1];
		Vph[2] = -0.5f*pMotor->setVoltage_V[0] - 0.866f*pMotor->setVoltage_V[1];

		float Rph[3];

		for(uint8_t i = 0; i < 3; i++)
		{
			Rph[i] = Vph[i] / Iph[i];
			RphMin[i] = fminf(RphMin[i], Rph[i]);
			RphMax[i] = fmaxf(RphMax[i], Rph[i]);
		}

		printf("Angle: %.1f\r\n", angle);
		printf("vUVW: % .3fV / % .3fV / % .3fV\r\n", Vph[0], Vph[1], Vph[2]);
		printf("iUVW: % .3fA / % .3fA / % .3fA\r\n", Iph[0], Iph[1], Iph[2]);
		printf("rUVW: % .3f\u2126 / % .3f\u2126 / % .3f\u2126\r\n", Rph[0], Rph[1], Rph[2]);
		printf("It: %.3f\r\n", tigerBot.powerControl.iCur);
	}

	printf("rSpr: % .3f\u2126 / % .3f\u2126 / % .3f\u2126\r\n",
			RphMax[0]-RphMin[0], RphMax[1]-RphMin[1], RphMax[2]-RphMin[2]);

	McuMotorSetOff(pMotor);

	TestModeExit();
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

static void testRotationIdent(Test* pTest)
{
	(void)pTest;

	if(tigerBot.powerControl.state == POWER_CONTROL_STATE_USB_POWERED)
	{
		printf("Rotation identification test cannot be performed in USB powered mode!\r\n");
		return;
	}

	TestModeStartup();

	printf("Identifying rotation model parameters\r\n");
	NetworkPrint("Identifying rotation model parameters");

#define NUM_VOLTAGES_ROT_IDENT 16
	arm_matrix_instance_f32 matA = { NUM_VOLTAGES_ROT_IDENT, 2, (float[NUM_VOLTAGES_ROT_IDENT*2]){} };
	arm_matrix_instance_f32 vecB = { NUM_VOLTAGES_ROT_IDENT, 1, (float[NUM_VOLTAGES_ROT_IDENT]){} };


	uint16_t row = 0;
	for(float vol = 0.4f; vol < 0.4f+(NUM_VOLTAGES_ROT_IDENT*0.1f); vol += 0.1f)
	{
		// set test voltage
		for(uint8_t i = 0; i < 4; i++)
			McuMotorSetVoltageDQ(&devMotors.mcu[i], 0.0f, vol);

		// spin up
		systime_t prev = chVTGetSystemTimeX();
		systime_t next = prev+TIME_US2I(1000);

		uint16_t t;
		for(t = 0; t < 2000; t++)
		{
			if(fabsf(robot.sensors.gyr.rotVel[2]) > 30.0f)
				break;

			chThdSleepUntilWindowed(prev, next);
			prev = next;
			next += TIME_US2I(1000);
		}

		if(t < 2000)
		{
			// spin up incomplete, robot too fast
			NetworkPrint("Spinning too fast. STOP. (%.2f)", robot.sensors.gyr.rotVel[2]);
			break;
		}

		// take samples
		prev = chVTGetSystemTimeX();
		next = prev+TIME_US2I(1000);

		float rotVelSum = 0.0f;
		float motCurSum = 0.0f;
		for(t = 0; t < 2000; t++)
		{
			rotVelSum += robot.sensors.gyr.rotVel[2];
			motCurSum += (devMotors.mcu[0].meas.avgCurrentDQ1ms_A[1] +
					devMotors.mcu[1].meas.avgCurrentDQ1ms_A[1] +
					devMotors.mcu[2].meas.avgCurrentDQ1ms_A[1] +
					devMotors.mcu[3].meas.avgCurrentDQ1ms_A[1]) * 0.25f;

			chThdSleepUntilWindowed(prev, next);
			prev = next;
			next += TIME_US2I(1000);
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
		MAT_ELEMENT(vecB, row, 0) = motCurAvg * robot.specs.driveTrain.motor.Km
				* 1/robot.specs.physical.wheelRadius_m
				* robot.specs.physical.botRadius_m * 4.0f;

		row++;
	}

	if(row < 2)
	{
		// not enough samples
		for(uint8_t i = 0; i < 4; i++)
			McuMotorSetOff(&devMotors.mcu[i]);

		TestModeExit();

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

	// set motors to zero torque
	for(uint8_t i = 0; i < 4; i++)
		McuMotorSetCurrentDQ(&devMotors.mcu[i], 0.0f, 0.0f);

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
	float inertia = -coulombFriction / decel; // initial guess without viscous friction

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
		McuMotorSetOff(&devMotors.mcu[i]);

	TestModeExit();
}

#define TRACTION_SAMPLES 10

static void testMotorTraction(Test* pTest)
{
	(void)pTest;

	TestModeStartup();

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
				if(fabsf(devMotors.mcu[mot].meas.encoderVelocity_radDs) > 20.0f && turnVol[mot] == 0.0f)
					turnVol[mot] = setVol;

				if(turnVol[mot] == 0.0f)
					motorMissing = 1;

				McuMotorSetVoltageDQ(&devMotors.mcu[mot], 0.0f, dir[mot]*setVol);
			}

			if(motorMissing == 0)
				break;

			chThdSleepMilliseconds(10);
		}

		printf("Motor break-away voltages:\r\n");
		for(uint8_t mot = 0; mot < 4; mot++)
		{
			McuMotorSetVoltageDQ(&devMotors.mcu[mot], 0, 0);
			printf("M%hu: %.3fV\r\n", (uint16_t)mot, turnVol[mot]);
			result[mot] += turnVol[mot];
		}

		chThdSleepMilliseconds(200);
	}

	printf("Motor break-away average voltages:\r\n");
	for(uint8_t mot = 0; mot < 4; mot++)
	{
		printf("M%hu: %.3fV\r\n", (uint16_t)mot, result[mot]/TRACTION_SAMPLES);
	}

	TestModeExit();
}

static float bufferedDribblerCurrent[200];

static void testDribbleRotation(Test* pTest)
{
	const TestArgDribbleRotation* pArg = (const TestArgDribbleRotation*)TestGetArgument(pTest);

	float dribblerCurrent = pArg->dribblerCurrent_A;
	float dribblerSpeedRPM = pArg->dribblerSpeed_rpm;

	// TODO: maybe make a skill from this?

	EMAFilter current;
	EMAFilterInit(&current, 0.99f);

	TestModeStartup();

	// Configure systems and prepare output
	RobotTestModeSetSystems(ROBOT_SYSTEM_CONTROL | ROBOT_SYSTEM_STATE_EST | ROBOT_SYSTEM_DRIVE | ROBOT_SYSTEM_DRIBBLER);

	robot.skillOutput.drive.limits.velMaxXY = GLOBAL_POS_MAX_VEL_XY;
	robot.skillOutput.drive.limits.velMaxW = GLOBAL_POS_MAX_VEL_W;
	robot.skillOutput.drive.limits.accMaxXY = LOCAL_VEL_MAX_ACC_XY;
	robot.skillOutput.drive.limits.accMaxW = 1.5f;
	robot.skillOutput.drive.limits.strictVelLimit = 0;
	robot.skillOutput.drive.modeW = DRIVE_MODE_LOCAL_VEL;
	robot.skillOutput.drive.modeXY = DRIVE_MODE_LOCAL_VEL;
	robot.skillOutput.drive.localVel[0] = 0;
	robot.skillOutput.drive.localVel[1] = 0;
	robot.skillOutput.drive.localVel[2] = 0;
	robot.skillOutput.kicker.mode = KICKER_MODE_DISARM;
	robot.skillOutput.dribbler.mode = DRIBBLER_MODE_SPEED;
	robot.skillOutput.dribbler.velocity = 3.0f;
	robot.skillOutput.dribbler.maxForce = dribblerCurrent * robot.specs.dribbler.motor.Km * 2.0f/(robot.specs.dribbler.barDiameter);

	// Wait until ball is at dribbler
	while(!robot.sensors.ir.interrupted)
		chThdSleepMilliseconds(10);

	// secure ball at dribbler
	chThdSleepMilliseconds(1000);

	current.value = devMotors.mcu[4].meas.avgCurrentDQ1ms_A[1];

	// Increase dribbler speed and start rotation
	robot.skillOutput.dribbler.velocity = dribblerSpeedRPM * 1.0f/60.0f * M_PI * robot.specs.dribbler.barDiameter;
	robot.skillOutput.drive.localVel[2] = GLOBAL_POS_MAX_VEL_W;

	uint32_t curIndex = 0;

	while(robot.sensors.ir.interrupted)
	{
		bufferedDribblerCurrent[curIndex++] = devMotors.mcu[4].meas.avgCurrentDQ1ms_A[1];

		if(curIndex >= 200)
			curIndex = 0;

		EMAFilterUpdate(&current, bufferedDribblerCurrent[curIndex]);
		chThdSleepMilliseconds(1);
	}

	float velW = robot.state.vel[2];

	printf("Ball lost at w: %.2frad/s, I: %.3fA\r\n", velW, current.value);
	NetworkPrint("Ball lost at w: %.2frad/s, I: %.3fA\r\n", velW, current.value);

	robot.skillOutput.dribbler.velocity = 0;
	robot.skillOutput.drive.limits.accMaxW = 5.0f;
	robot.skillOutput.drive.localVel[2] = 0;

	while(robot.state.vel[2] > 1.0f)
		chThdSleepMilliseconds(1);

	TestModeExit();
}

static void testBatteryDropUnderLoad(Test* pTest)
{
	(void)pTest;

	const float testVoltage_V = 3.2f;

	EMAFilter current;
	EMAFilter voltage;

	EMAFilterInit(&current, 0.99f);
	EMAFilterInit(&voltage, 0.99f);

	float avgMotorCur_A[4];

	printf("Testing battery voltage drop under load\r\n");

	if(tigerBot.powerControl.state == POWER_CONTROL_STATE_USB_POWERED)
	{
		printf("Test cannot be performed in USB powered mode!\r\n");
		return;
	}

	TestModeStartup();

	for(uint8_t id = 0; id < 4; id++)
		McuMotorSetOff(&devMotors.mcu[id]);

	voltage.value = tigerBot.powerControl.vBat;
	current.value = tigerBot.powerControl.iCur;

	for(uint32_t i = 0; i < 1000; i++)
	{
		EMAFilterUpdate(&voltage, tigerBot.powerControl.vBat);
		EMAFilterUpdate(&current, tigerBot.powerControl.iCur);
		chThdSleepMilliseconds(1);
	}

	float initialVoltage_V = voltage.value;
	float initialCurrent_A = current.value;

	for(uint8_t id = 0; id < 4; id++)
	{
		McuMotorSetElectricalAngle(&devMotors.mcu[id], 0, testVoltage_V);
		avgMotorCur_A[id] = 0.0f;
	}

	chThdSleepMilliseconds(500);

	voltage.value = tigerBot.powerControl.vBat;
	current.value = tigerBot.powerControl.iCur;

	for(uint32_t i = 0; i < 1000; i++)
	{
		EMAFilterUpdate(&voltage, tigerBot.powerControl.vBat);
		EMAFilterUpdate(&current, tigerBot.powerControl.iCur);
		for(uint8_t id = 0; id < 4; id++)
		{
			avgMotorCur_A[id] += devMotors.mcu[id].meas.avgCurrentDQ1ms_A[0];
		}

		chThdSleepMilliseconds(1);
	}

	for(uint8_t id = 0; id < 4; id++)
	{
		avgMotorCur_A[id] /= 1000.0f;
	}

	for(uint8_t id = 0; id < 4; id++)
		McuMotorSetOff(&devMotors.mcu[id]);

	float loadCurrent_A = current.value;
	float dischageRating_C = loadCurrent_A/1.3f;
	float internalResistance_R = (initialVoltage_V - voltage.value)/loadCurrent_A;
	float normInternalR_R = internalResistance_R / dischageRating_C;
	float normInternalCellR_R = normInternalR_R / (float)tigerBot.powerControl.batCells;

	printf("Initial:    %.3fV, %.3fA\r\n", initialVoltage_V, initialCurrent_A);
	printf("Under load: %.3fV, %.3fA\r\n", voltage.value, loadCurrent_A);
	printf("Voltage drop: %.3fV\r\n", initialVoltage_V - voltage.value);
	printf("Discharge C (@1.3Ah): %.3f\r\n", loadCurrent_A/1.3f);
	printf("Internal resistance:\r\n");
	printf("At load: %.3fR, 1C: %.3fmR, 1C single-cell: %.3fmR\r\n", internalResistance_R, normInternalR_R*1e3f, normInternalCellR_R*1e3f);
	for(uint8_t id = 0; id < 4; id++)
	{
		printf("Avg. motor current: %.3fA\r\n", avgMotorCur_A[id]);
	}

	TestModeExit();
}

static void testBatteryDischarge(Test* pTest)
{
	(void)pTest;

	const float dischargeCurrent = 3.0f;

	float motorVoltages[4] = { 1.0f, 1.0f, 1.0f, 1.0f };

	EMAFilter current;
	EMAFilter voltage;

	EMAFilterInit(&current, 0.99f);
	EMAFilterInit(&voltage, 0.99f);

	printf("Discharging battery to storage level.\r\n");

	if(tigerBot.powerControl.state == POWER_CONTROL_STATE_USB_POWERED)
	{
		printf("Test cannot be performed in USB powered mode!\r\n");
		return;
	}

	TestModeStartup();

	voltage.value = tigerBot.powerControl.vBat;
	current.value = tigerBot.powerControl.iCur;

	for(uint8_t id = 0; id < 4; id++)
		McuMotorSetElectricalAngle(&devMotors.mcu[id], 0.0f, motorVoltages[id]);

	chThdSleepMilliseconds(1000);

	uint32_t reportCounter = 0;

	const float targetVoltage = tigerBot.powerControl.batCells * 3.75f;

	float angle = 0.0f;

	while(1)
	{
		EMAFilterUpdate(&voltage, tigerBot.powerControl.vBat);
		EMAFilterUpdate(&current, tigerBot.powerControl.iCur);

		if(voltage.value < targetVoltage)
		{
			break;
		}

		for(uint8_t id = 0; id < 4; id++)
		{
			McuMotorSetElectricalAngle(&devMotors.mcu[id], angle, motorVoltages[id]);

			float curD = devMotors.mcu[id].meas.avgCurrentDQ1ms_A[0];
			float curQ = devMotors.mcu[id].meas.avgCurrentDQ1ms_A[1];
			float cur = sqrtf(curD*curD + curQ*curQ);

			if(cur < dischargeCurrent)
				motorVoltages[id] += 0.0001f;
			else
				motorVoltages[id] -= 0.0001f;

			if(motorVoltages[id] < 0.1f)
				motorVoltages[id] = 0.1f;
		}

		angle += 0.005f;
		AngleNormalizePtr(&angle);

		reportCounter++;
		if(reportCounter % 1000 == 0)
		{
			printf("%6.3fV of %6.3fV (%6.3fA)  %.2f, %.2f, %.2f, %.2f\r\n", voltage.value, targetVoltage, current.value,
					motorVoltages[0], motorVoltages[1], motorVoltages[2], motorVoltages[3]);
		}

		chThdSleepMilliseconds(1);
	}

	for(uint8_t id = 0; id < 4; id++)
		McuMotorSetOff(&devMotors.mcu[id]);

	chThdSleepMilliseconds(1000);

	PowerControlKill(&tigerBot.powerControl);

	TestModeExit();
}

static void testDribblerIdleCurrent(Test* pTest)
{
	pTest->expectedTestTime_s = 8.0f;

	EMAFilter current;

	EMAFilterInit(&current, 0.99f);

	printf("Testing dribbler current draw without ball\r\n");

	if(tigerBot.powerControl.state == POWER_CONTROL_STATE_USB_POWERED)
	{
		printf("Test cannot be performed in USB powered mode!\r\n");
		return;
	}

	TestModeStartup();

	for(uint8_t id = 0; id < 4; id++)
		McuMotorSetOff(&devMotors.mcu[id]);

	McuMotor* pDrib = &devMotors.mcu[4];

	FusionEKFDribblerIdleCurTable* pTable = &fusionEKF.dribbler.idleCurrentTable;
	pTable->usedPoints = 0;
	pTable->speed_mmDs[0] = 0;
	pTable->cur_mA[0] = 0;

	static const float sampleSpeeds[] = { 0.0f, 0.5f, 1.0f, 1.5f, 2.0f, 2.5f, 3.0f, 3.5f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f, 10.0f };
	size_t numSamples = sizeof(sampleSpeeds) / sizeof(sampleSpeeds[0]);
	size_t tableSamples = sizeof(pTable->speed_mmDs) / sizeof(pTable->speed_mmDs[0]);

	if(numSamples > tableSamples)
		numSamples = tableSamples;

	for(uint32_t sample = 1; sample < numSamples; sample++)
	{
		float speed = sampleSpeeds[sample];

		float dribblerVel = speed * 2.0f/robot.specs.dribbler.barDiameter * robot.specs.dribbler.bar2MotorRatio;

		McuMotorSetVelocity(pDrib, dribblerVel, 0.0f, 5.0f);

		for(uint32_t i = 0; i < 500; i++)
		{
			McuMotorMeasurement meas;
			McuMotorGet(pDrib, &meas);
			EMAFilterUpdate(&current, meas.avgCurrentDQ1ms_A[1]);
			chThdSleepMilliseconds(1);
		}

		pTable->speed_mmDs[sample] = speed*1e3f;
		pTable->cur_mA[sample] = current.value*1e3f;
	}

	McuMotorSetOff(pDrib);

	pTable->usedPoints = numSamples;

	FlashFSWrite(&fusionEKF.dribbler.pIdleCurrentFile, &fusionEKF.dribbler.idleCurrentTable, sizeof(fusionEKF.dribbler.idleCurrentTable));

	TestModeExit();

	printf("Dribbler idle current calibration updated. Samples: %u\r\n", pTable->usedPoints);
	for(size_t i = 0; i < tableSamples; i++)
	{
		printf("%2u %5hu %4hu\r\n", i, pTable->speed_mmDs[i], pTable->cur_mA[i]);
	}
}

#define LED_FRONT_LEFT_RED		0x0100
#define LED_FRONT_LEFT_GREEN	0x0200
#define LED_FRONT_LEFT_BLUE		0x0400
#define LED_FRONT_LEFT_WHITE	0x0800
#define LED_FRONT_LEFT_MASK		0x0F00
#define LED_FRONT_RIGHT_RED		0x1000
#define LED_FRONT_RIGHT_GREEN	0x2000
#define LED_FRONT_RIGHT_BLUE	0x4000
#define LED_FRONT_RIGHT_WHITE	0x8000
#define LED_FRONT_RIGHT_MASK	0xF000

static void ledFrontSet(uint16_t state)
{
	uint16_t leftLED = state & LED_FRONT_LEFT_MASK;
	uint16_t rightLED = state & LED_FRONT_RIGHT_MASK;

	float leftRGBW[4] = { 0.0f };
	float rightRGBW[4] = { 0.0f };
	float onValue = 1.0f;

	if(leftLED == LED_FRONT_LEFT_RED)
		leftRGBW[0] = onValue;
	else
		leftRGBW[0] = 0.0f;

	if(leftLED == LED_FRONT_LEFT_GREEN)
		leftRGBW[1] = onValue;
	else
		leftRGBW[1] = 0.0f;

	if(leftLED == LED_FRONT_LEFT_BLUE)
		leftRGBW[2] = onValue;
	else
		leftRGBW[2] = 0.0f;

	if(leftLED == LED_FRONT_LEFT_WHITE)
		leftRGBW[3] = onValue;
	else
		leftRGBW[3] = 0.0f;

	if(rightLED == LED_FRONT_RIGHT_RED)
		rightRGBW[0] = onValue;
	else
		rightRGBW[0] = 0.0f;

	if(rightLED == LED_FRONT_RIGHT_GREEN)
		rightRGBW[1] = onValue;
	else
		rightRGBW[1] = 0.0f;

	if(rightLED == LED_FRONT_RIGHT_BLUE)
		rightRGBW[2] = onValue;
	else
		rightRGBW[2] = 0.0f;

	if(rightLED == LED_FRONT_RIGHT_WHITE)
		rightRGBW[3] = onValue;
	else
		rightRGBW[3] = 0.0f;

	LEDRGBWSet(&devLedFrontLeft, leftRGBW[0], leftRGBW[1], leftRGBW[2], leftRGBW[3]);
	LEDRGBWSet(&devLedFrontRight, rightRGBW[0], rightRGBW[1], rightRGBW[2], rightRGBW[3]);
}

static void testFrontLEDs(Test* pTest)
{
	(void)pTest;

	printf("Showing off...\r\n");

	TestModeStartup();

	ledFrontSet(LED_FRONT_LEFT_RED | LED_FRONT_RIGHT_RED);
	chThdSleepMilliseconds(500);
	ledFrontSet(LED_FRONT_LEFT_GREEN | LED_FRONT_RIGHT_GREEN);
	chThdSleepMilliseconds(500);
	ledFrontSet(LED_FRONT_LEFT_BLUE | LED_FRONT_RIGHT_BLUE);
	chThdSleepMilliseconds(500);
	ledFrontSet(LED_FRONT_LEFT_WHITE | LED_FRONT_RIGHT_WHITE);
	chThdSleepMilliseconds(500);

	for(uint8_t i = 0; i < 10; i++)
	{
		ledFrontSet(LED_FRONT_LEFT_RED);
		chThdSleepMilliseconds(100);
		ledFrontSet(LED_FRONT_RIGHT_RED);
		chThdSleepMilliseconds(100);
	}

	for(uint8_t i = 0; i < 10; i++)
	{
		ledFrontSet(LED_FRONT_LEFT_GREEN);
		chThdSleepMilliseconds(100);
		ledFrontSet(LED_FRONT_RIGHT_GREEN);
		chThdSleepMilliseconds(100);
	}

	for(uint8_t i = 0; i < 10; i++)
	{
		ledFrontSet(LED_FRONT_LEFT_BLUE);
		chThdSleepMilliseconds(100);
		ledFrontSet(LED_FRONT_RIGHT_BLUE);
		chThdSleepMilliseconds(100);
	}

	for(uint8_t i = 0; i < 10; i++)
	{
		ledFrontSet(LED_FRONT_LEFT_WHITE);
		chThdSleepMilliseconds(100);
		ledFrontSet(LED_FRONT_RIGHT_WHITE);
		chThdSleepMilliseconds(100);
	}

	for(uint8_t i = 0; i < 10; i++)
	{
		ledFrontSet(LED_FRONT_LEFT_RED | LED_FRONT_RIGHT_GREEN);
		chThdSleepMilliseconds(200);
		ledFrontSet(LED_FRONT_LEFT_GREEN | LED_FRONT_RIGHT_RED);
		chThdSleepMilliseconds(200);
	}

	for(uint8_t i = 0; i < 10; i++)
	{
		ledFrontSet(LED_FRONT_LEFT_BLUE | LED_FRONT_RIGHT_WHITE);
		chThdSleepMilliseconds(200);
		ledFrontSet(LED_FRONT_LEFT_WHITE | LED_FRONT_RIGHT_BLUE);
		chThdSleepMilliseconds(200);
	}

	for(uint8_t i = 0; i < 20; i++)
	{
		ledFrontSet(LED_FRONT_LEFT_RED | LED_FRONT_RIGHT_GREEN);
		chThdSleepMilliseconds(50);
		ledFrontSet(LED_FRONT_LEFT_GREEN | LED_FRONT_RIGHT_BLUE);
		chThdSleepMilliseconds(50);
		ledFrontSet(LED_FRONT_LEFT_BLUE | LED_FRONT_RIGHT_WHITE);
		chThdSleepMilliseconds(50);
		ledFrontSet(LED_FRONT_LEFT_WHITE | LED_FRONT_RIGHT_RED);
		chThdSleepMilliseconds(50);
	}

	TestModeExit();
}

SHELL_CMD(mot_res, "[Experimental] Test motor phase resistances",
	SHELL_ARG(motorId, "Motor id (0-4)")
);

SHELL_CMD_IMPL(mot_res)
{
	(void)pUser; (void)argc;

	int motorId = atoi(argv[1]);

	if(motorId < 0 || motorId > 4)
	{
		fprintf(stderr, "Invalid motor ID");
		return;
	}

	TestArgMot arg;
	arg.motorId = motorId;

	TestSchedule(TEST_ID_MOT_PHASE_RES, &arg, 1);
}

SHELL_CMD(rot, "[Experimental] Identify rotational model parameters");

SHELL_CMD_IMPL(rot)
{
	(void)pUser; (void)argc; (void)argv;

	TestSchedule(TEST_ID_ROT_IDENT, 0, 1);
}

SHELL_CMD(traction, "[Experimental] Test robot traction on field");

SHELL_CMD_IMPL(traction)
{
	(void)pUser; (void)argc; (void)argv;

	TestSchedule(TEST_ID_MOT_TRACTION, 0, 1);
}

SHELL_CMD(drib_rot, "[Experimental] Test ball grip strength by rotating with ball",
	SHELL_ARG(current, "Dribbler current to use [A]"),
	SHELL_ARG(speed, "Dribbler speed to use [rpm]")
);

SHELL_CMD_IMPL(drib_rot)
{
	(void)pUser; (void)argc;

	TestArgDribbleRotation args;
	args.dribblerCurrent_A = atof(argv[1]);
	args.dribblerSpeed_rpm = atof(argv[2]);

	TestSchedule(TEST_ID_DRIBBLE_ROTATION, &args, 1);
}

SHELL_CMD(bat_res, "[Experimental] Measure internal battery resistance");

SHELL_CMD_IMPL(bat_res)
{
	(void)pUser; (void)argc; (void)argv;

	TestSchedule(TEST_ID_BAT_RES, 0, 1);
}

SHELL_CMD(bat_dis, "[Experimental] Discharge battery to storage level");

SHELL_CMD_IMPL(bat_dis)
{
	(void)pUser; (void)argc; (void)argv;

	TestSchedule(TEST_ID_BAT_DISCHARGE, 0, 1);
}

SHELL_CMD(drib_idle, "[Experimental] Determine dribbler current draw without ball");

SHELL_CMD_IMPL(drib_idle)
{
	(void)pUser; (void)argc; (void)argv;

	TestSchedule(TEST_ID_DRIBBLE_IDLE, 0, 1);
}

SHELL_CMD(leds, "[Experimental] Significantly improve party mood");

SHELL_CMD_IMPL(leds)
{
	(void)pUser; (void)argc; (void)argv;

	TestSchedule(TEST_ID_FRONT_LEDS, 0, 1);
}

void TestExperimentalInit(ShellCmdHandler* pCmdHandler)
{
	TestRegister(TEST_ID_MOT_PHASE_RES, &testMotorPhaseResistance, sizeof(TestArgMot), 0);
	TestRegister(TEST_ID_ROT_IDENT, &testRotationIdent, 0, 0);
	TestRegister(TEST_ID_MOT_TRACTION, &testMotorTraction, 0, 0);
	TestRegister(TEST_ID_DRIBBLE_ROTATION, &testDribbleRotation, sizeof(TestArgDribbleRotation), 0);
	TestRegister(TEST_ID_DRIBBLE_IDLE, &testDribblerIdleCurrent, 0, 0);
	TestRegister(TEST_ID_BAT_RES, &testBatteryDropUnderLoad, 0, 0);
	TestRegister(TEST_ID_BAT_DISCHARGE, &testBatteryDischarge, 0, 0);
	TestRegister(TEST_ID_FRONT_LEDS, &testFrontLEDs, 0, 0);

	if(pCmdHandler)
	{
		ShellCmdAdd(pCmdHandler, mot_res_command);
		ShellCmdAdd(pCmdHandler, rot_command);
		ShellCmdAdd(pCmdHandler, traction_command);
		ShellCmdAdd(pCmdHandler, drib_rot_command);
		ShellCmdAdd(pCmdHandler, drib_idle_command);
		ShellCmdAdd(pCmdHandler, bat_res_command);
		ShellCmdAdd(pCmdHandler, bat_dis_command);
		ShellCmdAdd(pCmdHandler, leds_command);
	}
}
