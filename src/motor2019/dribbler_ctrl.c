/*
 * dribbler_ctrl.c
 *
 *  Created on: 22.08.2020
 *      Author: AndreR
 */

#include "dribbler_ctrl.h"
#include "system_init.h"
#include "log_msgs.h"
#include "adc.h"
#include "main.h"
#include "dribbler_motor.h"

DribblerCtrlData dribblerCtrl = {
	.currentOffset = 10837,
	.currentDirection = 1
};

// forward order 3 2 6 4 5 1   3 2 6 4 5 1
static const int8_t hallDir[8][8] =
{ // 	  0   1   2   3   4   5   6  7
/*0*/	{ 0,  0,  0,  0,  0,  0,  0, 0 },
/*1*/	{ 0,  0,  1,  1, -1, -1,  0, 0 },
/*2*/	{ 0, -1,  0, -1,  1,  0,  1, 0 },
/*3*/	{ 0, -1,  1,  0,  0, -1,  1, 0 },
/*4*/	{ 0,  1, -1,  0,  0,  1, -1, 0 },
/*5*/	{ 0,  1,  0,  1, -1,  0, -1, 0 },
/*6*/	{ 0,  0, -1, -1,  1,  1,  0, 0 },
/*7*/	{ 0,  0,  0,  0,  0,  0,  0, 0 },
};

void DribblerCtrlPerPWMCycle()
{
	const int32_t curMult_Q3_15 = 175698L; // 5.361871705068514

	// current measurements
	int32_t avgCurrent_U12_0 = adc.dmaData[0];
	int32_t avgCurrent_S16_0 = ((curMult_Q3_15 * avgCurrent_U12_0) >> 15);
	int32_t currentQ_S16_0 = (avgCurrent_S16_0 - dribblerCtrl.currentOffset)*dribblerCtrl.currentDirection;

	data.sensors.adc.currentUVW_S15_15[0] = 0;
	data.sensors.adc.currentUVW_S15_15[1] = 0;
	data.sensors.adc.currentUVW_S15_15[2] = 0;

	data.sensors.adc.currentDQ_S16_0[0] = 0;

	// PWM cycle is 40kHz, logging is 20kHz, so we sum up two measurements for logging
	if(adc.logCounter == 0)
		data.sensors.adc.currentDQ_S16_0[1] = currentQ_S16_0;
	else
		data.sensors.adc.currentDQ_S16_0[1] = (data.sensors.adc.currentDQ_S16_0[1] + currentQ_S16_0) >> 1;

	// hall sensors
	uint16_t hallSR = TIM2->SR;
	uint8_t hallPos = HALL_GET_POS(); // current position

	data.sensors.hall.position = hallPos;

	if(TIM2->CNT > 16777216UL)
	{
		TIM2->CNT = 16777216UL;
		data.sensors.hall.commutationTime = INT32_MAX;
	}
	else if(hallSR & TIM_SR_CC1IF) // hall pos changed
	{
		// counter value at time of hall change => time since last change:
		int32_t hallDeltaCNT = TIM2->CCR1;

		int8_t hallDirMult = hallDir[dribblerCtrl.lastHallPos][hallPos];
		dribblerCtrl.lastHallPos = hallPos;

		dribblerCtrl.currentDirection = hallDirMult;

		if(hallDirMult == 0)
		{
			++data.sensors.hall.invalidTransitions;
		}
		else
		{
			// hall delta time computation
			data.sensors.hall.commutationTime = hallDeltaCNT * hallDirMult;
		}
	}
	else
	{
		// no update from hall sensors
	}

	// Adjust current measurement offset during idle
	if(data.motor.mode == MOTOR_MODE_OFF)
	{
		dribblerCtrl.currentOffset = (avgCurrent_S16_0 + 7*dribblerCtrl.currentOffset) >> 3;
	}

	// Control
	PICtrlS12Update(&data.ctrl.currentQ, (currentQ_S16_0*24855) >> 16);

	if(data.ctrl.currentQ.enabled)
		data.motor.Udq[1] = data.ctrl.currentQ.output << 3;
	else
		data.ctrl.currentQ.output = data.motor.Udq[1] >> 3;

	// Apply output
	if(data.motor.mode == MOTOR_MODE_OFF)
	{
		data.motor.Udq[1] = 0;

		DribblerMotorSetOff();
	}
	else if(data.motor.mode == MOTOR_MODE_VOLT_AB)
	{
		// derivation of correct hall position from AB values is too complex, use fixed position
		DribblerMotorSetPosition(data.motor.Uab[0], 1);
	}
	else
	{
		DribblerMotorSetVoltage(data.motor.Udq[1]);
	}

	// trigger commutation from control cycle to make sure it does not happen during current measurement
	TIM1->EGR = TIM_EGR_COMG;

	// Sum up average output voltage
	dribblerCtrl.voltageQSum += data.motor.Udq[1];

	++dribblerCtrl.sampleCounter;
	if(dribblerCtrl.sampleCounter == 40)
	{
		dribblerCtrl.sampleCounter = 0;

		data.ctrl.avgVoltageDQ[0] = 0;
		data.ctrl.avgVoltageDQ[1] = (dribblerCtrl.voltageQSum*205) >> 13;

		dribblerCtrl.voltageQSum = 0;
	}

	switch(data.command.recordMode)
	{
		case MOTOR_EXCHANGE_MOSI_RECORD_SET_Q_ENC_DETLA:
		case MOTOR_EXCHANGE_MOSI_RECORD_CUR_AB:
		{
			adc.pSampleCurrent->iMotor = 0;
			adc.pSampleCurrent->iMotor2 = 0;
		}
		break;
		case MOTOR_EXCHANGE_MOSI_RECORD_CUR_DQ:
		{
			adc.pSampleCurrent->iMotor = 0;
			adc.pSampleCurrent->iMotor2 = data.sensors.adc.currentDQ_S16_0[1];
		}
		break;
		case MOTOR_EXCHANGE_MOSI_RECORD_VOLT_AB_CUR_DQ:
		{
			adc.pSampleCurrent->iMotor = (data.sensors.adc.currentDQ_S16_0[1]*data.sensors.adc.currentDQ_S16_0[1]) >> 14;
			adc.pSampleCurrent->iMotor2 = (dribblerMotor.voltage*dribblerMotor.voltage) >> 16;
		}
		break;
		case MOTOR_EXCHANGE_MOSI_RECORD_CUR_Q_SET_MEAS:
		{
			adc.pSampleCurrent->iMotor = (data.ctrl.currentQ.setpoint * 10800) >> 12;
			adc.pSampleCurrent->iMotor2 = data.sensors.adc.currentDQ_S16_0[1];
		}
		break;
		default:
			break;
	}
}

void DribblerCtrlUpdate()
{
	if(data.timeMs - data.comm.lastCommandTimeMs > 200)
	{
		data.comm.timeout = 1;
		data.comm.lastCommandTimeMs = data.timeMs - 500;
	}
	else
	{
		data.comm.timeout = 0;
	}

	if(data.comm.timeout)
	{
		data.command.motorMode = MOTOR_MODE_OFF;
		data.command.input[0] = 0;
		data.command.input[1] = 0;
	}

	data.ctrl.hallOnly = data.command.flags & MOTOR_EXCHANGE_MOSI_FLAG_HALL_ONLY;

	data.motor.mode = data.command.motorMode;

	// Motor speed is computed from output current (measured) and voltage (controlled)
	int32_t avgVoltageQ_S15_0 = data.ctrl.avgVoltageDQ[1]; // [mV]
	int32_t avgCurrentQ_S14_0 = adc.avgCurrentDQ_S14_0[1]; // [mA]

	int32_t motorResistance_U4_6 = data.command.resistance;
	int32_t motorBackEmfConstantInv_U10_2 = data.command.backEmfConstantInv;

	int32_t resistiveLoss_S18_0 = (motorResistance_U4_6 * avgCurrentQ_S14_0) >> 6;
	int32_t backEmf_S15_0 = avgVoltageQ_S15_0 - resistiveLoss_S18_0; // [mV]

	if(backEmf_S15_0 > 32767)
		backEmf_S15_0 = 32767;

	if(backEmf_S15_0 < -32768)
		backEmf_S15_0 = -32768;

	int32_t modelSpeed_S25_0 = (backEmf_S15_0 * motorBackEmfConstantInv_U10_2) >> 2; // [mrad/s]
	int32_t modelSpeed_S15_0 = modelSpeed_S25_0 >> 10; // [1000/1024 rad/s]

	data.sensors.encoder.delta = modelSpeed_S15_0;

	switch(data.motor.mode)
	{
		case MOTOR_MODE_VOLT_AB:
		{
			data.motor.Uab[0] = data.command.input[0];
			data.motor.Uab[1] = data.command.input[1];

			data.motor.Udq[1] = 0;

			PICtrlS12Enable(&data.ctrl.currentQ, 0);

			data.ctrl.currentQ.output = 0;
		}
		break;
		case MOTOR_MODE_CUR_D_VOLT_Q:
		case MOTOR_MODE_VOLT_DQ:
		{
			data.motor.Udq[1] = data.command.input[1];

			PICtrlS12Enable(&data.ctrl.currentQ, 0);

			data.ctrl.speed.output = (data.sensors.adc.currentDQ_S16_0[1]*24855) >> 16;
			data.ctrl.currentQ.output = data.motor.Udq[1] >> 3;
		}
		break;
		case MOTOR_MODE_CUR_DQ:
		{
			int32_t q_S12_0 = data.command.input[1];
			q_S12_0 = (q_S12_0*24855) >> 16; // Input +-10800mA, scale down  to +-4096
			PICtrlS12Setpoint(&data.ctrl.currentQ, q_S12_0);

			PICtrlS12Enable(&data.ctrl.currentQ, 1);

			data.ctrl.speed.output = q_S12_0;
		}
		break;
		case MOTOR_MODE_SPEED:
		{
			// Q current input is used as limit for speed controller
			int32_t q_S12_0 = data.command.input[1];
			q_S12_0 = (q_S12_0*24855) >> 16; // Input +-10800mA, scale down  to +-4096
			if(q_S12_0 < 0)
			{
				data.ctrl.speed.outputMin = q_S12_0;
				data.ctrl.speed.outputMax = -q_S12_0;
			}
			else
			{
				data.ctrl.speed.outputMin = -q_S12_0;
				data.ctrl.speed.outputMax = q_S12_0;
			}

			// encDeltaSetpoint is used as hall speed setpoint
			PICtrlS12Setpoint(&data.ctrl.speed, data.command.encDeltaSetpoint);
			PICtrlS12Enable(&data.ctrl.speed, 1);
			PICtrlS12Update(&data.ctrl.speed, modelSpeed_S15_0);

			// Apply output to Q current controller input
			PICtrlS12Setpoint(&data.ctrl.currentQ, data.ctrl.speed.output);
			PICtrlS12Enable(&data.ctrl.currentQ, 1);
		}
		break;
		default:
		{
			data.motor.Uab[0] = 0;
			data.motor.Uab[1] = 0;

			data.motor.Udq[1] = 0;

			PICtrlS12Enable(&data.ctrl.currentQ, 0);

			data.ctrl.speed.output = 0;
			data.ctrl.currentQ.output = 0;
		}
		break;
	}

	data.ctrl.currentQ.outputMax = data.sensors.adc.vSupply_Q16_0 >> 3;
	data.ctrl.currentQ.outputMin = -data.ctrl.currentQ.outputMax;

	data.ctrl.currentQ.kp = data.command.curCtrlQKp;
	data.ctrl.currentQ.ki = data.command.curCtrlQKi;

	data.ctrl.speed.kp = data.command.velCtrlKp;
	data.ctrl.speed.ki = data.command.velCtrlKi;
}
