/*
 * test_experimental.c
 *
 *  Created on: 21.05.2022
 *      Author: AndreR
 */

#include "test_experimental.h"
#include "test.h"
#include "power.h"
#include "motors.h"
#include "robot/robot.h"
#include "robot/ctrl.h"
#include "robot/skill_basics.h"
#include "robot/ctrl_tigga.h"
#include "robot/ctrl_panthera.h"
#include "util/ema_filter.h"
#include "util/console.h"
#include "util/sys_time.h"
#include "util/network_print.h"
#include "util/arm_mat_util_f32.h"
#include "util/angle_math.h"
#include <float.h>

#define NUM_VOLTAGES_IDENT_ALL 3

void TestMotorIdentDriveAll()
{
	EMAFilter current[4];
	EMAFilter speed[4];

	for(uint8_t i = 0; i < 4; i++)
	{
		EMAFilterInit(&current[i], 0.99f);
		EMAFilterInit(&speed[i], 0.95f);
	}

	ConsolePrint("\r\n### Parallel Motor System Identification ###\r\n");

	if(power.usbPowered)
	{
		ConsolePrint("Test cannot be performed in USB powered mode!\r\n");
		return;
	}

	TestModeStartup();

	for(uint8_t i = 0; i < 4; i++)
		MotorsSetVoltageDQ(i, 0, 0);

	chThdSleepMilliseconds(500);

	float speeds[4][8];
	float currents[4][4];

	float R = botParams.driveTrain.motor.R * 1.0f/1.5f;

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

	TestModeExit();
}

void TestCompassCalib()
{
	TestModeStartup();

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
	chThdSleepMilliseconds(200);

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
	chThdSleepMilliseconds(200);

	ctrlTigga.configCompass.fieldOffset = fieldOffset;
	ConfigNotifyUpdate(ctrlTigga.pConfigFileCompass);

	TestModeExit();
}

void TestMotorPhaseResistance(uint8_t motorId)
{
	ConsolePrint("Testing motor phase resistances on motor %hu\r\n", motorId);

	if(power.usbPowered)
	{
		ConsolePrint("Test cannot be performed in USB powered mode!\r\n");
		return;
	}

	if(motorId > 4)
		return;

	TestModeStartup();

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

void TestRotationIdent()
{
	if(power.usbPowered)
	{
		ConsolePrint("Rotation identification test cannot be performed in USB powered mode!\r\n");
		return;
	}

	TestModeStartup();

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
	{
		MotorsSetOff(i);
	}

	TestModeExit();
}

#define TRACTION_SAMPLES 10

void TestMotorTraction()
{
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

	TestModeExit();
}

static float bufferedDribblerCurrent[200];

void TestDribbleRotation(float dribblerCurrent, float dribblerSpeedRPM)
{
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
	robot.skillOutput.dribbler.speed = 500;
	robot.skillOutput.dribbler.maxCurrent = dribblerCurrent;

	// Wait until ball is at dribbler
	while(!robot.sensors.ir.interrupted)
		chThdSleepMilliseconds(10);

	// secure ball at dribbler
	chThdSleepMilliseconds(1000);

	current.value = motors.motor[4].avgCurrentDQ1ms[1];

	// Increase dribbler speed and start rotation
	robot.skillOutput.dribbler.speed = dribblerSpeedRPM * 2.0f*M_PI/60.0f;
	robot.skillOutput.drive.localVel[2] = GLOBAL_POS_MAX_VEL_W;

	uint32_t curIndex = 0;

	while(robot.sensors.ir.interrupted)
	{
		bufferedDribblerCurrent[curIndex++] = motors.motor[4].avgCurrentDQ1ms[1];

		if(curIndex >= 200)
			curIndex = 0;

		EMAFilterUpdate(&current, bufferedDribblerCurrent[curIndex]);
		chThdSleepMilliseconds(1);
	}

	float velW = robot.state.vel[2];

	ConsolePrint("Ball lost at w: %.2frad/s, I: %.3fA\r\n", velW, current.value);
	NetworkPrint("Ball lost at w: %.2frad/s, I: %.3fA\r\n", velW, current.value);

	robot.skillOutput.dribbler.speed = 0;
	robot.skillOutput.drive.limits.accMaxW = 5.0f;
	robot.skillOutput.drive.localVel[2] = 0;

	while(robot.state.vel[2] > 1.0f)
		chThdSleepMilliseconds(1);

	TestModeExit();
}

void TestBatteryDropUnderLoad()
{
	const float testVoltage_V = 3.2f;

	EMAFilter current;
	EMAFilter voltage;

	EMAFilterInit(&current, 0.99f);
	EMAFilterInit(&voltage, 0.99f);

	float avgMotorCur_A[4];

	ConsolePrint("Testing battery voltage drop under load\r\n");

	if(power.usbPowered)
	{
		ConsolePrint("Test cannot be performed in USB powered mode!\r\n");
		return;
	}

	TestModeStartup();

	for(uint8_t id = 0; id < 4; id++)
		MotorsSetOff(id);

	voltage.value = power.vBat;
	current.value = power.iCur;

	for(uint32_t i = 0; i < 1000; i++)
	{
		EMAFilterUpdate(&voltage, power.vBat);
		EMAFilterUpdate(&current, power.iCur);
		chThdSleepMilliseconds(1);
	}

	float initialVoltage_V = voltage.value;
	float initialCurrent_A = current.value;

	for(uint8_t id = 0; id < 4; id++)
	{
		MotorsSetElectricalAngle(id, 0, testVoltage_V);
		avgMotorCur_A[id] = 0.0f;
	}

	chThdSleepMilliseconds(500);

	voltage.value = power.vBat;
	current.value = power.iCur;

	for(uint32_t i = 0; i < 1000; i++)
	{
		EMAFilterUpdate(&voltage, power.vBat);
		EMAFilterUpdate(&current, power.iCur);
		for(uint8_t id = 0; id < 4; id++)
		{
			avgMotorCur_A[id] += motors.motor[id].avgCurrentDQ1ms[0];
		}

		chThdSleepMilliseconds(1);
	}

	for(uint8_t id = 0; id < 4; id++)
	{
		avgMotorCur_A[id] /= 1000.0f;
	}

	for(uint8_t id = 0; id < 4; id++)
		MotorsSetOff(id);

	float loadCurrent_A = current.value;
	float dischageRating_C = loadCurrent_A/1.3f;
	float internalResistance_R = (initialVoltage_V - voltage.value)/loadCurrent_A;
	float normInternalR_R = internalResistance_R / dischageRating_C;
	float normInternalCellR_R = normInternalR_R / (float)power.batCells;

	ConsolePrint("Initial:    %.3fV, %.3fA\r\n", initialVoltage_V, initialCurrent_A);
	ConsolePrint("Under load: %.3fV, %.3fA\r\n", voltage.value, loadCurrent_A);
	ConsolePrint("Voltage drop: %.3fV\r\n", initialVoltage_V - voltage.value);
	ConsolePrint("Discharge C (@1.3Ah): %.3f\r\n", loadCurrent_A/1.3f);
	ConsolePrint("Internal resistance:\r\n");
	ConsolePrint("At load: %.3fR, 1C: %.3fmR, 1C single-cell: %.3fmR\r\n", internalResistance_R, normInternalR_R*1e3f, normInternalCellR_R*1e3f);
	for(uint8_t id = 0; id < 4; id++)
	{
		ConsolePrint("Avg. motor current: %.3fA\r\n", avgMotorCur_A[id]);
	}

	TestModeExit();
}
