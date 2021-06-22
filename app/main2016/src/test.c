/*
 * test.c
 *
 *  Created on: 27.06.2016
 *      Author: AndreR
 */

#include "test.h"
#include "hal/motors.h"
#include "hal/led.h"
#include "power.h"
#include "main/network.h"
#include "arm_math.h"
#include "util/arm_mat_util_f32.h"
#include "gui/test_drive.h"
#include "util/console.h"
#include "main/robot.h"
#include "main/ctrl.h"

void printTestMotorDynamicsResult(TestMotorDynamicsResult* pResult);
void calculateReasoningOnTestResultsDrive(const TestMotorDynamicsResult* pResult, TestDynamicsReasoningResult* pReasoningResult);
void calculateReasoningOnTestResultsDribbler(const TestMotorDynamicsResult* pResult, TestDynamicsReasoningResult* pReasoningResult);
void calculateReasoningOnTestResults(const TestMotorDynamicsResult* pResult, TestDynamicsReasoningResult* pReasoningResult);
void printReasoningResultConstant(uint8_t result);
void printTestMotorDynamicsResult(TestMotorDynamicsResult* pResult);

static float calculateDriveTrainInertia(const float* pSamples, uint16_t numSamples, float voltage, uint16_t motorId, float b);

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

#define TRACTION_SAMPLES 10

void TestMotorTraction()
{
	RobotTestModeEnable(1);
	RobotTestModeStopAll();

	chThdSleepMilliseconds(5);

	for(uint8_t i = 0; i < 4; i++)
		MotorsDriveSetVoltage(i, 0);

	MotorsDribblerSetVoltage(0);

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
				if(fabsf(motors.drive[mot].velocity) > 20.0f && turnVol[mot] == 0.0f)
					turnVol[mot] = setVol;

				if(turnVol[mot] == 0.0f)
					motorMissing = 1;

				MotorsDriveSetVoltage(mot, dir[mot]*setVol);
			}

			if(motorMissing == 0)
				break;

			chThdSleepMilliseconds(10);
		}

		ConsolePrint("Motor break-away voltages:\r\n");
		for(uint8_t mot = 0; mot < 4; mot++)
		{
			MotorsDriveSetVoltage(mot, 0);
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

	RobotTestModeEnable(0);

	chThdSleepMilliseconds(1000);
}

#define SPIN_UP_SAMPLES 300
#define NUM_VOLTAGES 8
const float testVoltages[NUM_VOLTAGES] = {-8.0f, -7.0f, -6.0f, -5.0f, 5.0f, 6.0f, 7.0f, 8.0f};

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

	RobotTestModeEnable(1);
	RobotTestModeStopAll();

	chThdSleepMilliseconds(5);

	EMAFilterInit(&current, 0.9f);
	EMAFilterInit(&speed, 0.95f);

	for(uint8_t i = 0; i < 4; i++)
		MotorsDriveSetVoltage(i, 0);

	MotorsDribblerSetVoltage(0);

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
			MotorsDribblerSetVoltage(fabsf(volt));
		else
			MotorsDriveSetVoltage(motorId, volt);

		chThdSleepMilliseconds(300);

		for(uint16_t j = 0; j < 700; j++)
		{
			EMAFilterUpdate(&current, power.iCur-mainOffset);

			if(motorId > 3)
				EMAFilterUpdate(&speed, motors.dribbler.velocity);
			else
				EMAFilterUpdate(&speed, motors.drive[motorId].velocity);

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
			K = 9.1e-3f;
			R = 0.955f;
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

	if(motorId > 3)
		MotorsDribblerSetVoltage(0);
	else
		MotorsDriveSetVoltage(motorId, 0);

	avgDamping /= (float)NUM_VOLTAGES;

//	ConsolePrint("Avg. Damping: %.9f\r\n", avgDamping);

	chThdSleepMilliseconds(200);

//	ConsolePrint("Doing spin up tests\r\n");

	float avgInertia = 0.0f;

	for(int16_t i = 0; i < NUM_VOLTAGES; i++)
	{
		float volt = testVoltages[i];

		if(motorId > 3)
			MotorsDribblerSetVoltage(fabsf(volt));
		else
			MotorsDriveSetVoltage(motorId, volt);

		for(uint16_t j = 0; j < SPIN_UP_SAMPLES; j++)
		{
			if(motorId > 3)
			{
				if(volt < 0)
					spinUp[j] = -motors.dribbler.velocity;
				else
					spinUp[j] = motors.dribbler.velocity;
			}
			else
			{
				spinUp[j] = motors.drive[motorId].velocity;
			}

			chThdSleepMilliseconds(1);
		}

		if(motorId > 3)
			MotorsDribblerSetVoltage(0);
		else
			MotorsDriveSetVoltage(motorId, 0);

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

	RobotTestModeEnable(0);

	chThdSleepMilliseconds(1000);

	MotorsSetDampingCoefficient(motorId, avgDamping);

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
		K = 9.1e-3f;
		R = 0.955f;
		L = 0.0498e-3f;
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

void calculateReasoningOnTestResultsDrive(const TestMotorDynamicsResult* pResult, TestDynamicsReasoningResult* pReasoningResult)
{
	if(pResult->damping < 40.0e-6f || pResult->damping > 100.0e-6f)
	{
		pReasoningResult->damping = TEST_REASONING_RESULT_WARNING;
	}

	if(pResult->damping < 1.0e-6f || pResult->damping > 150.0e-6f)
	{
		pReasoningResult->damping = TEST_REASONING_RESULT_BAD;
	}

	if(pResult->inertia < 20.0e-6f || pResult->inertia > 45.0e-6f)
	{
		pReasoningResult->inertia = TEST_REASONING_RESULT_WARNING;
	}

	if(pResult->inertia < 10.0e-6f || pResult->inertia > 60.0e-6f)
	{
		pReasoningResult->inertia = TEST_REASONING_RESULT_BAD;
	}

	if(pResult->cur < 0.06f || pResult->cur > 0.15f)
	{
		pReasoningResult->cur = TEST_REASONING_RESULT_WARNING;
	}

	if(pResult->cur < 0.03f || pResult->cur > 0.25f)
	{
		pReasoningResult->cur = TEST_REASONING_RESULT_BAD;
	}

	if(pResult->speed < 270 || pResult->speed > 330)
	{
		pReasoningResult->vel = TEST_REASONING_RESULT_WARNING;
	}

	if(pResult->speed < 210 || pResult->speed > 390)
	{
		pReasoningResult->vel = TEST_REASONING_RESULT_BAD;
	}
}

void calculateReasoningOnTestResultsDribbler(const TestMotorDynamicsResult* pResult, TestDynamicsReasoningResult* pReasoningResult)
{
	if(pResult->damping < 1.0e-6f || pResult->damping > 10.0e-6f)
	{
		pReasoningResult->damping = TEST_REASONING_RESULT_WARNING;
	}

	if(pResult->damping < 1.0e-7f || pResult->damping > 20.0e-6f)
	{
		pReasoningResult->damping = TEST_REASONING_RESULT_BAD;
	}

	if(pResult->inertia < 1.0e-6f || pResult->inertia > 10.0e-6f)
	{
		pReasoningResult->inertia = TEST_REASONING_RESULT_WARNING;
	}

	if(pResult->inertia < 1.0e-7f || pResult->inertia > 20.0e-6f)
	{
		pReasoningResult->inertia = TEST_REASONING_RESULT_BAD;
	}

	if(pResult->cur < 0.25f || pResult->cur > 0.42f)
	{
		pReasoningResult->cur = TEST_REASONING_RESULT_WARNING;
	}

	if(pResult->cur < 0.15f || pResult->cur > 0.6f)
	{
		pReasoningResult->cur = TEST_REASONING_RESULT_BAD;
	}

	if(pResult->speed < 820 || pResult->speed > 860)
	{
		pReasoningResult->vel = TEST_REASONING_RESULT_WARNING;
	}

	if(pResult->speed < 760 || pResult->speed > 920)
	{
		pReasoningResult->vel = TEST_REASONING_RESULT_BAD;
	}
}

void calculateReasoningOnTestResults(const TestMotorDynamicsResult* pResult, TestDynamicsReasoningResult* pReasoningResult)
{
	pReasoningResult->cur = TEST_REASONING_RESULT_OK;
	pReasoningResult->damping = TEST_REASONING_RESULT_OK;
	pReasoningResult->inertia = TEST_REASONING_RESULT_OK;
	pReasoningResult->vel = TEST_REASONING_RESULT_OK;

	if (pResult->motorId < 4)
		calculateReasoningOnTestResultsDrive(pResult, pReasoningResult);
	else
		calculateReasoningOnTestResultsDribbler(pResult, pReasoningResult);

}

inline void printReasoningResultConstant(uint8_t result)
{
	if(result == TEST_REASONING_RESULT_OK)
	{
		ConsolePrint("OK");
	}
	else if(result == TEST_REASONING_RESULT_WARNING)
	{
		ConsolePrint("WARNING");
	}
	else if(result == TEST_REASONING_RESULT_BAD)
	{
		ConsolePrint("BAD");
	}
}

inline void printTestMotorDynamicsResult(TestMotorDynamicsResult* pResult)
{
	ConsolePrint("Finished dynamics test on motor %hu", pResult->motorId);

	if(pResult->motorId < 4)
		ConsolePrint(" (drive motor)\r\n");
	else
		ConsolePrint(" (dribbler motor)\r\n");

	TestDynamicsReasoningResult reasoningResult;
	calculateReasoningOnTestResults(pResult, &reasoningResult);
	ConsolePrint("\tdamping = %.9f (", pResult->damping);
	printReasoningResultConstant(reasoningResult.damping);
	ConsolePrint(")\n\r");
	ConsolePrint("\tinertia = %.9f (", pResult->inertia);
	printReasoningResultConstant(reasoningResult.inertia);
	ConsolePrint(")\n\r");
	ConsolePrint("\tcurrent = %f (", pResult->cur);
	printReasoningResultConstant(reasoningResult.cur);
	ConsolePrint(")\n\r");
	ConsolePrint("\tspeed = %f (", pResult->speed);
	printReasoningResultConstant(reasoningResult.vel);
	ConsolePrint(")\n\r");
}
