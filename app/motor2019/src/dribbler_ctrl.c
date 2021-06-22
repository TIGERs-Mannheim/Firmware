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
	int32_t avgCurrent_U12_0 = (adc.dmaData[0] + adc.dmaData[1] + adc.dmaData[2] + adc.dmaData[3]) >> 2;
	int32_t avgCurrent_S16_0 = ((curMult_Q3_15 * avgCurrent_U12_0) >> 15);

	data.sensors.adc.currentUVW_S15_15[0] = 0;
	data.sensors.adc.currentUVW_S15_15[1] = 0;
	data.sensors.adc.currentUVW_S15_15[2] = 0;

	data.sensors.adc.currentDQ_S16_0[0] = 0;

	// PWM cycle is 40kHz, logging is 20kHz, so we sum up two measurements for logging
	if(adc.logCounter == 0)
		data.sensors.adc.currentDQ_S16_0[1] = (avgCurrent_S16_0 - dribblerCtrl.currentOffset)*dribblerCtrl.currentDirection;
	else
		data.sensors.adc.currentDQ_S16_0[1] = (data.sensors.adc.currentDQ_S16_0[1] + ((avgCurrent_S16_0 - dribblerCtrl.currentOffset)*dribblerCtrl.currentDirection)) >> 1;

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
			dribblerCtrl.hallDeltas[hallPos] = hallDeltaCNT;

			int32_t hallDeltaAvg = 0;
			for(uint8_t i = 1; i < 7; i++)
				hallDeltaAvg += dribblerCtrl.hallDeltas[i];

			hallDeltaAvg = ((hallDeltaAvg >> 4) * 43) >> 4; // * 1/6

//			data.sensors.hall.commutationTime = hallDeltaAvg * hallDirMult;
			data.sensors.hall.commutationTime = hallDeltaCNT * hallDirMult;
		}
	}
	else
	{
		// no update from hall sensors
	}

	if(data.motor.mode == MOTOR_MODE_OFF)
	{
		dribblerCtrl.currentOffset = (avgCurrent_S16_0 + 7*dribblerCtrl.currentOffset) >> 3;
	}

	if(data.motor.mode == MOTOR_MODE_VOLT_DQ)
	{
		DribblerMotorSetVoltage(data.motor.Udq[1]);

		data.ctrl.avgVoltageDQ[1] = data.motor.Udq[1];
	}
	else
	{
		DribblerMotorSetOff();
	}

	// trigger commutation from control cycle to make sure it does not happen during current measurement
	TIM1->EGR = TIM_EGR_COMG;

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
			adc.pSampleCurrent->iMotor = data.sensors.adc.currentDQ_S16_0[0];
			adc.pSampleCurrent->iMotor2 = data.sensors.adc.currentDQ_S16_0[1];
		}
		break;
		case MOTOR_EXCHANGE_MOSI_RECORD_VOLT_AB_CUR_DQ:
		{
			adc.pSampleCurrent->iMotor = data.sensors.adc.currentDQ_S16_0[1];
			adc.pSampleCurrent->iMotor2 = data.motor.Udq[1];
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

	switch(data.motor.mode)
	{
		case MOTOR_MODE_VOLT_DQ:
		{
			data.motor.Udq[0] = data.command.input[0];
			data.motor.Udq[1] = data.command.input[1];

			PICtrlS12Enable(&data.ctrl.currentD, 0);
			PICtrlS12Enable(&data.ctrl.currentQ, 0);

			data.ctrl.speed.output = (data.sensors.adc.currentDQ_S16_0[1]*24855) >> 16;
			data.ctrl.currentD.output = data.motor.Udq[0] >> 3;
			data.ctrl.currentQ.output = data.motor.Udq[1] >> 3;
		}
		break;

		case MOTOR_MODE_VOLT_AB:
		case MOTOR_MODE_CUR_D_VOLT_Q:
		case MOTOR_MODE_CUR_DQ:
		case MOTOR_MODE_SPEED:
		default:
		{
			data.motor.Uab[0] = 0;
			data.motor.Uab[1] = 0;

			data.motor.Udq[0] = 0;
			data.motor.Udq[1] = 0;

			PICtrlS12Enable(&data.ctrl.currentD, 0);
			PICtrlS12Enable(&data.ctrl.currentQ, 0);

			data.ctrl.speed.output = 0;
			data.ctrl.currentD.output = 0;
			data.ctrl.currentQ.output = 0;
		}
		break;
	}

	data.ctrl.currentD.outputMax = data.sensors.adc.vSupply_Q16_0 >> 3;
	data.ctrl.currentD.outputMin = -data.ctrl.currentD.outputMax;

	data.ctrl.currentQ.outputMax = data.sensors.adc.vSupply_Q16_0 >> 3;
	data.ctrl.currentQ.outputMin = -data.ctrl.currentQ.outputMax;

	data.ctrl.currentD.kp = data.command.curCtrlDKp;
	data.ctrl.currentD.ki = data.command.curCtrlDKi;

	data.ctrl.currentQ.kp = data.command.curCtrlQKp;
	data.ctrl.currentQ.ki = data.command.curCtrlQKi;

	data.ctrl.speed.kp = data.command.velCtrlKp;
	data.ctrl.speed.ki = data.command.velCtrlKi;
}
