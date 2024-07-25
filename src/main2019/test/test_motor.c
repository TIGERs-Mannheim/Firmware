#include "test_motor.h"
#include "test_data.h"
#include "util/test.h"
#include "math/ema_filter.h"
#include "hal/sys_time.h"
#include "dev/motors.h"
#include "robot/robot.h"
#include "test_common.h"
#include "tiger_bot.h"
#include <stdio.h>

static void doReasoningOnMotor(TestResultMotIdentMech* pResult);
static void printTestResultMotIdentMech(TestResultMotIdentMech* pResult);

static void motorIdentElectricalAtAngle(uint8_t motorId, float electricalAngle_rad)
{
	EMAFilter current;
	EMAFilterInit(&current, 0.95f);

	float pwmFrequency = 20e3f;
	if(motorId > 3)
		pwmFrequency = 40e3f;

	McuMotor* pMotor = &devMotors.mcu[motorId];

	// Step 1: increase voltage until current reaches 8A or 80% of supply voltage, use this voltage as step setpoint
	float stepVoltage = 0.0f;
	const float maxVoltage = tigerBot.powerControl.vBat*0.8f;
	const float maxCurrent = 8.0f;

	while(stepVoltage < maxVoltage && current.value < maxCurrent)
	{
		stepVoltage += 0.1f;

		McuMotorSetElectricalAngle(pMotor, electricalAngle_rad, stepVoltage);

		for(uint16_t i = 0; i < 20; i++)
		{
			chThdSleepMilliseconds(1);
			float normCurrent = sqrtf(pMotor->meas.avgCurrentDQ1ms_A[0]*pMotor->meas.avgCurrentDQ1ms_A[0] + pMotor->meas.avgCurrentDQ1ms_A[1]*pMotor->meas.avgCurrentDQ1ms_A[1]);
			EMAFilterUpdate(&current, normCurrent);
		}
	}

	printf("\r\nStep voltage: %.3fV, angle: %.1f\r\n", stepVoltage, electricalAngle_rad*180.0f/M_PI);

	// Step 2: use this voltage some more to get a stable average of current
	for(uint16_t i = 0; i < 500; i++)
	{
		chThdSleepMilliseconds(1);
		float normCurrent = sqrtf(pMotor->meas.avgCurrentDQ1ms_A[0]*pMotor->meas.avgCurrentDQ1ms_A[0] + pMotor->meas.avgCurrentDQ1ms_A[1]*pMotor->meas.avgCurrentDQ1ms_A[1]);
		EMAFilterUpdate(&current, normCurrent);
	}

	printf("U: %.3fV, I: %.3fA\r\n", stepVoltage, current.value);

	const float stepCurrent = current.value;

	// Step 3: Perform step input and record data
	McuMotorSetElectricalAngle(pMotor, electricalAngle_rad, stepVoltage*0.25f);
	current.value *= 0.25f;
	for(uint16_t i = 0; i < 500; i++)
	{
		chThdSleepMilliseconds(1);
		float normCurrent = sqrtf(pMotor->meas.avgCurrentDQ1ms_A[0]*pMotor->meas.avgCurrentDQ1ms_A[0] + pMotor->meas.avgCurrentDQ1ms_A[1]*pMotor->meas.avgCurrentDQ1ms_A[1]);
		EMAFilterUpdate(&current, normCurrent);
	}

	const float startCurrent = current.value;

	printf("U: %.3fV, I: %.3fA\r\n", stepVoltage*0.25f, startCurrent);

	McuMotorDebugRecord(pMotor, MOTOR_EXCHANGE_MOSI_RECORD_VOLT_AB_CUR_DQ, 1);
	chThdSleepMilliseconds(50);

	McuMotorSetElectricalAngle(pMotor, electricalAngle_rad, stepVoltage);
	chThdSleepMilliseconds(500);

	McuMotorSetOff(pMotor);

	// Step 4: do system identification
	const float plantGain = stepCurrent / stepVoltage;
	const float plantDeadTime = 1.0f/pwmFrequency;

	// current after two time constants, need to find corresponding time where this is reached
	const float currentT2 = (0.865f*(stepCurrent-startCurrent) + startCurrent);

	arm_matrix_instance_q15 recordData;
	recordData.numRows = pMotor->recordCounter;
	recordData.numCols = 2;
	recordData.pData = pMotor->pRecordData;

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
		fprintf(stderr, "Unable to identify step in recorded data\r\n");
		TestModeExit();
		return;
	}

	const float dt = 1.0f/20e3f;

	float t2CrossingOffset = (currentT2-currentNearT2[0])/(currentNearT2[1]-currentNearT2[0])*dt;
	float t2Constant = (indexAboveT2-1)*dt + t2CrossingOffset - (indexStep+1)*dt;
	float plantTimeConstant = t2Constant*0.5f;

	float motorResistance = 1.0f/plantGain;
	float motorInductance = plantTimeConstant*motorResistance;

	printf("Plant: K: %.3f, T: %.7fs, d: %.6fs\r\n", plantGain, plantTimeConstant, plantDeadTime);
	printf("Motor: R: %.3f\u2126, L: %.8fH\r\n", motorResistance, motorInductance);

	// Step 5: compute controller gains
	float Kp[2] = { 0.55f*plantTimeConstant/(dt*plantGain), 0.2f*plantTimeConstant/(dt*plantGain) };
	float Ti[2] = { 1.0f*plantTimeConstant, 1.0f*plantTimeConstant };
	float Ki[2] = { Kp[0]/(Ti[0]*pwmFrequency), Kp[1]/(Ti[1]*pwmFrequency) };

	printf("D Ctrl: Kp: %.3f, Ki: %.4f\r\n", Kp[0], Ki[0]);
	printf("Q Ctrl: Kp: %.3f, Ki: %.4f\r\n", Kp[1], Ki[1]);
}

static void testMotorIdentElectrical(Test* pTest)
{
	// This test identifies the motor voltage to motor current FOPDT model
	// and recommends current controller gains for D and Q

	const TestArgMot* pArg = (const TestArgMot*)TestGetArgument(pTest);
	uint8_t motorId = pArg->motorId;
	pTest->expectedTestTime_s = 3.0f;

	if(motorId > 4)
		return;

	printf("\r\n### Motor %hu Electrical System Identification ###\r\n", (uint16_t)motorId);

	if(tigerBot.powerControl.state == POWER_CONTROL_STATE_USB_POWERED)
	{
		fprintf(stderr, "Test cannot be performed in USB powered mode!\r\n");
		return;
	}

	TestModeStartup();

	for(float angle = 0.0f; angle < M_PI; angle += 60.0*M_PI/180.0f)
		motorIdentElectricalAtAngle(motorId, angle);

	TestModeExit();
}

static void testMotorIdentMechanical(Test* pTest)
{
	// This test identifies the motor current to motor speed first order model.
	// It computes load inertia (J) and viscous friction (b) from the identified model.

	const TestArgMot* pArg = (const TestArgMot*)TestGetArgument(pTest);
	TestResultMotIdentMech* pResult = (TestResultMotIdentMech*)TestGetResult(pTest);
	uint8_t motorId = pArg->motorId;
	pTest->expectedTestTime_s = 13.0f;

	EMAFilter current;
	EMAFilter speed;
	EMAFilterInit(&current, 0.99f);
	EMAFilterInit(&speed, 0.95f);

	if(motorId > 4)
		return;

	McuMotor* pMotor = &devMotors.mcu[motorId];

	printf("\r\n### Motor %hu Mechanical System Identification ###\r\n", (uint16_t)motorId);

	if(tigerBot.powerControl.state == POWER_CONTROL_STATE_USB_POWERED)
	{
		fprintf(stderr, "Test cannot be performed in USB powered mode!\r\n");
		return;
	}

	float R;

	if(motorId > 3)
		R = robot.specs.dribbler.motor.R;
	else
		R = robot.specs.driveTrain.motor.R * 1.0f/1.5f;

	const float identVoltages[2] = { 2.0f, 8.0f };
	float identCurrent[2];

	TestModeStartup();

	McuMotorSetVoltageDQ(pMotor, 0, 0);

	chThdSleepMilliseconds(500);

	float torqueConstant = 0.0f;
	float setVolt = 0.0f;

	for(int32_t i = 0; i < 2; i++)
	{
		float newVolt = identVoltages[i];

		for(; setVolt < newVolt; setVolt += 0.01f)
		{
			McuMotorSetCurrentDVoltageQ(pMotor, 0.0f, setVolt);

			chThdSleepMilliseconds(1);
		}

		chThdSleepMilliseconds(1000);

		current.value = pMotor->meas.avgCurrentDQ1ms_A[1];
		speed.value = pMotor->meas.hallVelocity_radDs;

		for(uint16_t t = 0; t < 1000; t++)
		{
			EMAFilterUpdate(&current, pMotor->meas.avgCurrentDQ1ms_A[1]);
			EMAFilterUpdate(&speed, pMotor->meas.hallVelocity_radDs);

			chThdSleepMilliseconds(1);
		}

		float U = setVolt;
		float I = current.value;
		float w = speed.value;

		float K = (U - R*I)/w;
		torqueConstant += K;

		identCurrent[i] = I;

		printf("Uq: %.3fV, Iq: %.3fA, w: %.3frad/s, K: %.6f\r\n",
				setVolt, current.value, speed.value, K);
	}

	torqueConstant *= 0.5f;

	const float initialCurrent = identCurrent[0];
	const float stepCurrent = current.value;
	const float stepSpeed = speed.value;

	printf("I: %.3fA, w: %.3frad/s\r\n", stepCurrent, stepSpeed);

	// quickly reduce speed by using voltage ramp down
	for(float f = identVoltages[1]; f > identVoltages[0]; f -= 0.01f)
	{
		McuMotorSetCurrentDVoltageQ(pMotor, 0.0f, f);
		chThdSleepMilliseconds(1);
		EMAFilterUpdate(&speed, pMotor->meas.hallVelocity_radDs);
	}

	// Switch back to current control mode and wait until speed is stable
	McuMotorSetCurrentDQ(pMotor, 0.0f, initialCurrent);

	for(uint16_t i = 0; i < 2000; i++)
	{
		chThdSleepMilliseconds(1);
		EMAFilterUpdate(&speed, pMotor->meas.hallVelocity_radDs);
	}

	float initialSpeed = speed.value;

	printf("I: %.3fA, w: %.3frad/s\r\n", initialCurrent, initialSpeed);

	// Perform current step
	McuMotorSetCurrentDQ(pMotor, 0.0f, stepCurrent);

	// Wait until speed is at 63% of step size (one time constant)
	const float speedT2 = (0.63f*(stepSpeed-initialSpeed) + initialSpeed);

	speed.alpha = 0.8f;

	uint32_t tUpStartUs = SysTimeUSec();
	while(speed.value < speedT2)
	{
		EMAFilterUpdate(&speed, pMotor->meas.hallVelocity_radDs);
		chThdSleepMilliseconds(1);
	}

	const float plantGain = (stepSpeed-initialSpeed) / (stepCurrent-initialCurrent);
	float plantTimeConstant = (SysTimeUSec() - tUpStartUs) * 1e-6f;

	printf("Plant: K: %.3f, T: %.7fs\r\n", plantGain, plantTimeConstant);

	float b = torqueConstant/plantGain;
	float J = plantTimeConstant*b;

	printf("Kt: %.6f [Nm/A]\r\n", torqueConstant);
	printf(" b: %.6f [Nmm*s]\r\n", b*1e3f);
	printf(" J: %.6f [kg*mm^2]\r\n", J*1e6f);

	McuMotorSetOff(pMotor);

	TestModeExit();

	pResult->motorId = motorId;
	pResult->cur = stepCurrent;
	pResult->speed = stepSpeed;
	pResult->inertia = J;
	pResult->damping = b;

	doReasoningOnMotor(pResult);
	printTestResultMotIdentMech(pResult);
}

static void doReasoningOnMotor(TestResultMotIdentMech* pResult)
{
	pResult->reasoning.cur = TEST_REASONING_RESULT_OK;
	pResult->reasoning.damping = TEST_REASONING_RESULT_OK;
	pResult->reasoning.inertia = TEST_REASONING_RESULT_OK;
	pResult->reasoning.vel = TEST_REASONING_RESULT_OK;

	float damping = 9e-6f;
	float inertia = 36.0e-6f;
	float cur = 0.29f;
	float speed = 356.0f;

	if(pResult->motorId > 3)
	{
		damping = 9e-6f;
		inertia = 1.4e-6f;
		cur = 1.5f;
		speed = 935.0f;
	}

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

static const char* reasoningString[] = { "OK", "WARNING", "BAD" };

static const char* getReasoningString(uint8_t result)
{
	if(result > 3)
		return "UNKNOWN";

	return reasoningString[result];
}

static void printTestResultMotIdentMech(TestResultMotIdentMech* pResult)
{
	printf("Finished dynamics test on motor %hu", pResult->motorId);

	if(pResult->motorId < 4)
		printf(" (drive motor)\r\n");
	else
		printf(" (dribbler motor)\r\n");

	printf("  damping = %.9f (%s)\r\n", pResult->damping, getReasoningString(pResult->reasoning.damping));
	printf("  inertia = %.9f (%s)\r\n", pResult->inertia, getReasoningString(pResult->reasoning.inertia));
	printf("  current = %f (%s)\r\n", pResult->cur, getReasoningString(pResult->reasoning.cur));
	printf("  speed = %f (%s)\r\n", pResult->speed, getReasoningString(pResult->reasoning.vel));
}

SHELL_CMD(mot_elec, "Run electrical motor system identification test",
	SHELL_ARG(motorId, "Motor id (0-4)")
);

SHELL_CMD_IMPL(mot_elec)
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

	TestSchedule(TEST_ID_MOT_IDENT_ELEC, &arg, 1);
}

SHELL_CMD(mot_mech, "Run mechanical motor system identification test",
	SHELL_ARG(motorId, "Motor id (0-4)")
);

SHELL_CMD_IMPL(mot_mech)
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

	TestSchedule(TEST_ID_MOT_IDENT_MECH, &arg, 1);
}

void TestMotorInit(ShellCmdHandler* pCmdHandler)
{
	TestRegister(TEST_ID_MOT_IDENT_ELEC, &testMotorIdentElectrical, sizeof(TestArgMot), 0);
	TestRegister(TEST_ID_MOT_IDENT_MECH, &testMotorIdentMechanical, sizeof(TestArgMot), sizeof(TestResultMotIdentMech));

	if(pCmdHandler)
	{
		ShellCmdAdd(pCmdHandler, mot_elec_command);
		ShellCmdAdd(pCmdHandler, mot_mech_command);
	}
}
