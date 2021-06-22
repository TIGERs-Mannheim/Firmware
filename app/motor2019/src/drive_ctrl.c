/*
 * drive_ctrl.c
 *
 *  Created on: 16.03.2019
 *      Author: AndreR
 *

                                                  ###############################
                                                  ##### Controller Overview #####
                                                  ###############################

                                                                            D Current Control
                                                                             +----+    +---+
                                                                             |    |    | 1 |
                                                                          +--+ Ki +--->+ - +--+
                                                                          |  |    |    | s |  |
                                                                          |  +----+    +---+  |
                                                                          |                   |
                                                    +-------+             |  +----+           v                     +-----------+
D Current Setpoint [mA]                             |  4096 |    / \      |  |    |          / \     +---+          |           |
+-------------------------------------------------->+ ----- +---> + +-----+->+ Kp +---------> + +--->+ 4 +--------->+           |
                                                    | 10800 |    \ /         |    |          \ /     +---+          |           |
                                                    +-------+     ^          +----+                                 |           |
                                                                  | -                                               |           |
                                                                  |                                                 |           |
                                                              +---+---+                                             |           |
                                                              |  4096 |   D Current Feedback [mA]                   |           |
Q Current Setpoint/Feedforward [mA]                           | ----- +<--------------------------------------------+           |
+---------------------------------------------------------+   | 10800 |                                             |           |
                                                          |   +-------+                                             |           |
                                 Speed Control            v                 Q Current Control                       |           |
                                +----+    +---+       +---+---+              +----+    +---+                        |           |
                                |    |    | 1 |       |  4096 |              |    |    | 1 |                        |           |
                             +--> Ki +--->+ - +--+    | ----- |           +--> Ki +--->+ - +--+                     |           |
                             |  |    |    | s |  |    | 10800 |           |  |    |    | s |  |                     |           |
                             |  +----+    +---+  |    +---+---+           |  +----+    +---+  |                     |   Plant   |
                             |                   |        |               |                   |                     |           |
Speed                        |  +----+           v        v               |  +----+           v                     |           |
Setpoint    +---+   / \      |  |    |          / \      / \              |  |    |          / \     +---+          |           |
+---------->+ 2 +--> + +-----+->+ Kp +---------> + +----> + +-------------+->+ Kp +---------> + +--->+ 4 +--------->+           |
[encDelta]  +---+   \ /         |    |          \ /      \ /                 |    |          \ /     +---+          |           |
per 1ms              ^          +----+                    ^                  +----+                                 |           |
                     | -                                  | -                                                       |           |
                     |                                    |                                                         |           |
                     |                                +---+---+                                                     |           |
                   +-+-+                              |  4096 |           Q Current Feedback [mA]                   |           |
                   | 2 |                              | ----- +<----------------------------------------------------+           |
                   +-+-+                              | 10800 |                                                     |           |
                     ^                                +-------+                                                     |           |
                     |                                                    Speed Feedback [encDelta]                 |           |
                     +----------------------------------------------------------------------------------------------+           |
                                                                                                                    |           |
                                                                                                                    +-----------+
PI Controller Input/Output: +-4096

D/Q Input: +-10800mA
D/Q Output: +-32768mV
D/Q Control Rate: 20kHz

Speed Input: +-2048 encoder ticks per 1ms
Speed Output: +-4096 (matches 10800mA Q input)
Speed Control Rate: 1kHz
 */

#include "main.h"
#include "adc.h"
#include "system_init.h"
#include "drive_ctrl.h"
#include "drive_motor.h"

DriveCtrlData driveCtrl;

/*
From To
    1   2   3   4   5   6
1   x   x 340   x 260   x
2   x   x  20   x   x 100
3 320  40   x   x   x   x
4   x   x   x   x 220 140
5 280   x   x 200   x   x
6   x  80   x 160   x   x
 */
static const uint16_t hallTransitionAngleTable_U16_0[8][8] =
{ //         0       1       2       3       4       5       6       7
/*0*/ { 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF },
/*1*/ { 0xFFFF, 0xFFFF, 0xFFFF,  61895, 0xFFFF,  47332, 0xFFFF, 0xFFFF },
/*2*/ { 0xFFFF, 0xFFFF, 0xFFFF,   3641, 0xFFFF, 0xFFFF,  18204, 0xFFFF },
/*3*/ { 0xFFFF,  58254,   7282, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF },
/*4*/ { 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,  40050,  25486, 0xFFFF },
/*5*/ { 0xFFFF,  50972, 0xFFFF, 0xFFFF,  36409, 0xFFFF, 0xFFFF, 0xFFFF },
/*6*/ { 0xFFFF, 0xFFFF,  14564, 0xFFFF,  29127, 0xFFFF, 0xFFFF, 0xFFFF },
/*7*/ { 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF },
};

// dynamic hall sensor position correction, only used if no encoder available and interpolation mode on
static int16_t hallTransitionAngleOffsetTable_I16_0[8][8];

/*
1: 300
2: 60
3: 0
4: 180
5: 240
6: 120
*/
static const uint16_t hallPositionAngleTable_U16_0[8] =
{
	0xFFFF, 54613, 10923, 0, 32768, 43691, 21845, 0xFFFF
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

static const int16_t sinCosTable[65][2];

static inline void sinCos_S0_15(uint16_t angle, int32_t* pSin, int32_t* pCos);

void DriveCtrlInit()
{
	PICtrlS12Init(&data.ctrl.currentD, 0, 0, -512, 512);
	PICtrlS12Init(&data.ctrl.currentQ, 0, 0, -512, 512);
	PICtrlS12Init(&data.ctrl.speed, 0, 0, -4096, 4095);

	driveCtrl.lastHallPos = HALL_GET_POS();

	// set initial encoder value to current hall sector
	TIM3->CNT = (((int32_t)hallPositionAngleTable_U16_0[driveCtrl.lastHallPos]) * 45) >> 10;

	driveCtrl.lastEncPos = TIM3->CNT;
}

void DriveCtrlUpdate()
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
		case MOTOR_MODE_VOLT_AB:
		{
			data.motor.Uab[0] = data.command.input[0];
			data.motor.Uab[1] = data.command.input[1];

			data.motor.Udq[0] = 0;
			data.motor.Udq[1] = 0;

			PICtrlS12Enable(&data.ctrl.currentD, 0);
			PICtrlS12Enable(&data.ctrl.currentQ, 0);

			data.ctrl.speed.output = 0;
			data.ctrl.currentD.output = 0;
			data.ctrl.currentQ.output = 0;
		}
		break;
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
		case MOTOR_MODE_CUR_D_VOLT_Q:
		{
			int32_t d_S12_0 = data.command.input[0];
			d_S12_0 = (d_S12_0*24855) >> 16; // Input +-10800mA, scale down  to +-4096
			PICtrlS12Setpoint(&data.ctrl.currentD, d_S12_0);

			data.motor.Udq[1] = data.command.input[1];

			PICtrlS12Enable(&data.ctrl.currentD, 1);
			PICtrlS12Enable(&data.ctrl.currentQ, 0);

			data.ctrl.speed.output = (data.sensors.adc.currentDQ_S16_0[1]*24855) >> 16;
			data.ctrl.currentQ.output = data.motor.Udq[1] >> 3;
		}
		break;
		case MOTOR_MODE_CUR_DQ:
		{
			int32_t d_S12_0 = data.command.input[0];
			d_S12_0 = (d_S12_0*24855) >> 16; // Input +-10800mA, scale down  to +-4096
			PICtrlS12Setpoint(&data.ctrl.currentD, d_S12_0);

			int32_t q_S12_0 = data.command.input[1];
			q_S12_0 = (q_S12_0*24855) >> 16; // Input +-10800mA, scale down  to +-4096
			PICtrlS12Setpoint(&data.ctrl.currentQ, q_S12_0);

			PICtrlS12Enable(&data.ctrl.currentD, 1);
			PICtrlS12Enable(&data.ctrl.currentQ, 1);

			data.ctrl.speed.output = q_S12_0;
		}
		break;
		case MOTOR_MODE_SPEED:
		{
			int32_t speed_S12_0 = data.command.encDeltaSetpoint;
			speed_S12_0 <<= 1; // Input +-2048, scale up to +-4096
			PICtrlS12Setpoint(&data.ctrl.speed, speed_S12_0);
			PICtrlS12Enable(&data.ctrl.speed, 1);
			PICtrlS12Update(&data.ctrl.speed, data.sensors.encoder.delta << 1);

			int32_t d_S12_0 = data.command.input[0];
			d_S12_0 = (d_S12_0*24855) >> 16; // Input +-10800mA, scale down  to +-4096
			PICtrlS12Setpoint(&data.ctrl.currentD, d_S12_0);

			int32_t q_S12_0 = data.command.input[1];
			q_S12_0 = (q_S12_0*24855) >> 16; // Input +-10800mA, scale down  to +-4096
			q_S12_0 += data.ctrl.speed.output;

			// limit Q setpoint to +-10.35A
			if(q_S12_0 > 3925)
				q_S12_0 = 3925;
			else if(q_S12_0 < -3925)
				q_S12_0 = -3925;

			PICtrlS12Setpoint(&data.ctrl.currentQ, q_S12_0);

			PICtrlS12Enable(&data.ctrl.currentD, 1);
			PICtrlS12Enable(&data.ctrl.currentQ, 1);
		}
		break;
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

void DriveCtrlUpdatePerPWMCycle()
{
	const int32_t curMult_Q3_15 = 175698L; // 5.361871705068514

	// current measurements
	int32_t curZeroAdc_U12_0 = (int32_t)adc.dmaData[6];
	int32_t curU0Adc_U12_0 = (int32_t)adc.dmaData[0];
	int32_t curV0Adc_U12_0 = (int32_t)adc.dmaData[1];
	int32_t curW0Adc_U12_0 = (int32_t)adc.dmaData[2];
	int32_t curW1Adc_U12_0 = (int32_t)adc.dmaData[3];
	int32_t curV1Adc_U12_0 = (int32_t)adc.dmaData[4];
	int32_t curU1Adc_U12_0 = (int32_t)adc.dmaData[5];

	uint32_t overcurrent = (TIM1->SR & TIM_SR_TIF) ? 1 : 0;
	TIM1->SR &= ~TIM_SR_TIF;

	data.motor.overcurrentDetected |= overcurrent;

	// hall and encoder
	uint16_t hallSR = TIM2->SR;
	uint8_t hallPos = HALL_GET_POS(); // current position
	uint32_t encoderPos_U12_0 = driveMotor.encDmaData[3]; // encoder position: 0 - 2879
	int32_t rotorPosElec_U16_0;

	data.sensors.hall.position = hallPos;

	if(data.ctrl.hallOnly)
	{
		rotorPosElec_U16_0 = hallPositionAngleTable_U16_0[hallPos];
	}
	else
	{
		// bring encoder mechanical 0-2879 to electrical 0-65535 range:
		rotorPosElec_U16_0 = (encoderPos_U12_0 * 372827UL) >> 14;
	}

	if(TIM2->CNT > 16777216UL)
	{
		// no movement
		if(!data.sensors.encoder.installed)
		{
			TIM3->PSC = 0xFFFF;
		}

		TIM2->CNT = 16777216UL;
		data.sensors.hall.commutationTime = INT32_MAX;
	}
	else if(hallSR & TIM_SR_CC1IF) // hall pos changed
	{
		// counter value at time of hall change => time since last change:
		int32_t hallDeltaCNT = TIM2->CCR1;

		// captured encoder position on hall sensor change:
		int32_t encAngle_U16_0 = (TIM3->CCR3 * 372827UL) >> 14;

		int32_t hallAngle_U16_0 = hallTransitionAngleTable_U16_0[driveCtrl.lastHallPos][hallPos];
		int8_t hallDirMult = hallDir[driveCtrl.lastHallPos][hallPos];
		driveCtrl.lastHallPos = hallPos;

		if(hallAngle_U16_0 == 0xFFFF)
		{
			++data.sensors.hall.invalidTransitions;
		}
		else
		{
			// hall delta time computation
			driveCtrl.hallDeltas[hallPos] = hallDeltaCNT;

			int32_t hallDeltaAvg = 0;
			for(uint8_t i = 1; i < 7; i++)
				hallDeltaAvg += driveCtrl.hallDeltas[i];

			hallDeltaAvg = ((hallDeltaAvg >> 4) * 43) >> 4; // * 1/6

			data.sensors.hall.commutationTime = hallDeltaAvg * hallDirMult;

			if(data.ctrl.hallOnly)
			{
				TIM3->CNT = (rotorPosElec_U16_0 * 45) >> 10;

				data.sensors.encoder.correction = 0;
//				data.sensors.encoder.correction = 1820*hallDirMult;
			}
			else if(!data.sensors.encoder.installed)
			{
				TIM3->PSC = (hallDeltaAvg * 17) >> 13;
				if(hallDirMult > 0)
					TIM3->CR1 &= ~TIM_CR1_DIR;
				else
					TIM3->CR1 |= TIM_CR1_DIR;

				TIM3->EGR = TIM_EGR_UG;

				int16_t hallOffset = hallTransitionAngleOffsetTable_I16_0[driveCtrl.lastHallPos][hallPos];
				hallAngle_U16_0 += hallOffset;

				// intentionally use 16 bit signed overflow here:
				int32_t encAngleCorrection = (int16_t)(hallAngle_U16_0 - encAngle_U16_0);
				if(encAngleCorrection > 0)
					hallOffset--;
				else
					hallOffset++;

				if(hallOffset > 1820) // 10°
					hallOffset = 1820;
				else if(hallOffset < -1820)
					hallOffset = -1820;

				hallTransitionAngleOffsetTable_I16_0[driveCtrl.lastHallPos][hallPos] = hallOffset;

				rotorPosElec_U16_0 = hallAngle_U16_0;
				TIM3->CNT = (hallAngle_U16_0 * 45) >> 10;
			}
			else
			{
				// intentionally use 16 bit signed overflow here:
				int32_t encAngleCorrection = (int16_t)(hallAngle_U16_0 - encAngle_U16_0);

				// exponential moving average filter:
				data.sensors.encoder.correction = (15*data.sensors.encoder.correction + encAngleCorrection) >> 4;

				// compensate for missing encoder ticks
				if(data.sensors.encoder.correction > 10923)
				{
					TIM3->ARR--;
					TIM3->CNT = (((int32_t)hallPositionAngleTable_U16_0[hallPos]) * 45) >> 10;
					data.sensors.encoder.correction = 0;
				}

				if(data.sensors.encoder.correction < -10923)
				{
					TIM3->ARR++;
					TIM3->CNT = (((int32_t)hallPositionAngleTable_U16_0[hallPos]) * 45) >> 10;
					data.sensors.encoder.correction = 0;
				}
			}
		}
	}
	else
	{
		// no update from hall sensors
	}

	int32_t encPosElecComp_U16_0 = rotorPosElec_U16_0 + data.sensors.encoder.correction;
	encPosElecComp_U16_0 &= 0xFFFF;

	// compute all phase currents
	data.sensors.adc.currentUVW_S15_15[0] = curMult_Q3_15 * (((curU0Adc_U12_0 + curU1Adc_U12_0) >> 1) - curZeroAdc_U12_0);
	data.sensors.adc.currentUVW_S15_15[1] = curMult_Q3_15 * (((curV0Adc_U12_0 + curV1Adc_U12_0) >> 1) - curZeroAdc_U12_0);
	data.sensors.adc.currentUVW_S15_15[2] = curMult_Q3_15 * (((curW0Adc_U12_0 + curW1Adc_U12_0) >> 1) - curZeroAdc_U12_0);

	// I_alpha = 2/3 * I_U - 1/3 * I_V - 1/3 * I_W
	// I_beta = 1/sqrt(3) * I_V - 1/sqrt(3) * I_W
	int32_t currentAlpha_Q16_0 = ((data.sensors.adc.currentUVW_S15_15[0] >> 12)*2731 - (data.sensors.adc.currentUVW_S15_15[1] >> 12)*1365 - (data.sensors.adc.currentUVW_S15_15[2] >> 12)*1365) >> 15;
	int32_t currentBeta_Q16_0 = ((data.sensors.adc.currentUVW_S15_15[1] >> 12)*2365 - (data.sensors.adc.currentUVW_S15_15[2] >> 12)*2365) >> 15;

	// power-invariant Clarke transform
//	int32_t currentAlpha_Q16_0 = ((ctrl.currentUVW_S15_15[0] >> 12)*3344 - (ctrl.currentUVW_S15_15[1] >> 12)*1672 - (ctrl.currentUVW_S15_15[2] >> 12)*1672) >> 15;
//	int32_t currentBeta_Q16_0 = ((ctrl.currentUVW_S15_15[1] >> 12)*2896 - (ctrl.currentUVW_S15_15[2] >> 12)*2896) >> 15;

	int32_t sin_S0_15;
	int32_t cos_S0_15;

	sinCos_S0_15(encPosElecComp_U16_0 >> 3, &sin_S0_15, &cos_S0_15);

	data.sensors.adc.currentDQ_S16_0[0] = (cos_S0_15*currentAlpha_Q16_0 + sin_S0_15*currentBeta_Q16_0) >> 15;
	data.sensors.adc.currentDQ_S16_0[1] = (-sin_S0_15*currentAlpha_Q16_0 + cos_S0_15*currentBeta_Q16_0) >> 15;

	if(overcurrent)
	{
		if(data.ctrl.hallOnly)
		{
			if(data.motor.mode == MOTOR_MODE_VOLT_DQ)
			{
				// convert Udq to Uab
				data.motor.Uab[0] = (cos_S0_15*data.motor.Udq[0] - sin_S0_15*data.motor.Udq[1]) >> 15;
				data.motor.Uab[1] = (sin_S0_15*data.motor.Udq[0] + cos_S0_15*data.motor.Udq[1]) >> 15;

				DriveMotorSetAlphaBeta(data.motor.Uab[0], data.motor.Uab[1]);
			}
			else if(data.motor.mode == MOTOR_MODE_VOLT_AB)
			{
				DriveMotorSetAlphaBeta(data.motor.Uab[0], data.motor.Uab[1]);
			}
			else
			{
				DriveMotorSetAlphaBeta(0, 0);
			}
		}
		else
		{
			DriveMotorSetAlphaBeta(0, 0);
		}
	}
	else
	{
		PICtrlS12Update(&data.ctrl.currentD, (data.sensors.adc.currentDQ_S16_0[0]*24855) >> 16);
		PICtrlS12Update(&data.ctrl.currentQ, (data.sensors.adc.currentDQ_S16_0[1]*24855) >> 16);

		if(data.ctrl.currentD.enabled)
			data.motor.Udq[0] = data.ctrl.currentD.output << 2;
		else
			data.ctrl.currentD.output = data.motor.Udq[0] >> 2;

		if(data.ctrl.currentQ.enabled)
			data.motor.Udq[1] = data.ctrl.currentQ.output << 2;
		else
			data.ctrl.currentQ.output = data.motor.Udq[1] >> 2;

		if(data.motor.mode != MOTOR_MODE_VOLT_AB)
		{
			// convert Udq to Uab
			data.motor.Uab[0] = (cos_S0_15*data.motor.Udq[0] - sin_S0_15*data.motor.Udq[1]) >> 15;
			data.motor.Uab[1] = (sin_S0_15*data.motor.Udq[0] + cos_S0_15*data.motor.Udq[1]) >> 15;
		}

		DriveMotorSetAlphaBeta(data.motor.Uab[0], data.motor.Uab[1]);
	}

	driveCtrl.voltageDQSum[0] += data.motor.Udq[0];
	driveCtrl.voltageDQSum[1] += data.motor.Udq[1];

	int32_t encDelta = (int32_t)encoderPos_U12_0 - (int32_t)driveCtrl.lastEncPos;
	if(encDelta > 1440)
		encDelta -= 2880;
	if(encDelta < -1440)
		encDelta += 2880;

	driveCtrl.lastEncPos = encoderPos_U12_0;
	driveCtrl.encSum += encDelta;

	++driveCtrl.sampleCounter;
	if(driveCtrl.sampleCounter == 20)
	{
		driveCtrl.sampleCounter = 0;

		data.sensors.encoder.delta = driveCtrl.encSum;
		data.sensors.encoder.position = encoderPos_U12_0;

		data.ctrl.avgVoltageDQ[0] = (driveCtrl.voltageDQSum[0]*205) >> 12; // => *1/20
		data.ctrl.avgVoltageDQ[1] = (driveCtrl.voltageDQSum[1]*205) >> 12;

		driveCtrl.encSum = 0;
		driveCtrl.voltageDQSum[0] = 0;
		driveCtrl.voltageDQSum[1] = 0;
	}

	switch(data.command.recordMode)
	{
		case MOTOR_EXCHANGE_MOSI_RECORD_CUR_AB:
		{
			adc.pSampleCurrent->iMotor = currentAlpha_Q16_0;
			adc.pSampleCurrent->iMotor2 = currentBeta_Q16_0;
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
			adc.pSampleCurrent->iMotor = (data.sensors.adc.currentDQ_S16_0[0]*data.sensors.adc.currentDQ_S16_0[0] + data.sensors.adc.currentDQ_S16_0[1]*data.sensors.adc.currentDQ_S16_0[1]) >> 14;
			adc.pSampleCurrent->iMotor2 = (data.motor.Uab[0]*data.motor.Uab[0] + data.motor.Uab[1]*data.motor.Uab[1]) >> 16;
		}
		break;
		case MOTOR_EXCHANGE_MOSI_RECORD_SET_Q_ENC_DETLA:
		{
			adc.pSampleCurrent->iMotor = (data.ctrl.currentQ.setpoint * 10800) >> 12;
			adc.pSampleCurrent->iMotor2 = encDelta;
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

// angle: 0-8191 (U13.0), sin/cos: S0.15
static inline void sinCos_S0_15(uint16_t angle, int32_t* pSin, int32_t* pCos)
{
	uint16_t lutIndex = angle >> 7; // U6.0
	int32_t angleFractionSecond = angle & 0x7F; // S0.7
	int32_t angleFractionFirst = 128 - angleFractionSecond;

	int32_t sin_S0_22 = (int32_t)sinCosTable[lutIndex][0]*angleFractionFirst + (int32_t)sinCosTable[lutIndex+1][0]*angleFractionSecond;
	int32_t cos_S0_22 = (int32_t)sinCosTable[lutIndex][1]*angleFractionFirst + (int32_t)sinCosTable[lutIndex+1][1]*angleFractionSecond;

	*pSin = sin_S0_22 >> 7;
	*pCos = cos_S0_22 >> 7;
}

// format: S0.15 => 32767 = 1.0
static const int16_t sinCosTable[65][2] = {
	{      0,  32767 }, 	{   3212,  32609 }, 	{   6393,  32137 }, 	{   9512,  31356 },
	{  12539,  30273 }, 	{  15446,  28898 }, 	{  18204,  27245 }, 	{  20787,  25329 },
	{  23170,  23170 }, 	{  25329,  20787 }, 	{  27245,  18204 }, 	{  28898,  15446 },
	{  30273,  12539 }, 	{  31356,   9512 }, 	{  32137,   6393 }, 	{  32609,   3212 },
	{  32767,      0 }, 	{  32609,  -3212 }, 	{  32137,  -6393 }, 	{  31356,  -9512 },
	{  30273, -12539 }, 	{  28898, -15446 }, 	{  27245, -18204 }, 	{  25329, -20787 },
	{  23170, -23170 }, 	{  20787, -25329 }, 	{  18204, -27245 }, 	{  15446, -28898 },
	{  12539, -30273 }, 	{   9512, -31356 }, 	{   6393, -32137 }, 	{   3212, -32609 },
	{      0, -32767 }, 	{  -3212, -32609 }, 	{  -6393, -32137 }, 	{  -9512, -31356 },
	{ -12539, -30273 }, 	{ -15446, -28898 }, 	{ -18204, -27245 }, 	{ -20787, -25329 },
	{ -23170, -23170 }, 	{ -25329, -20787 }, 	{ -27245, -18204 }, 	{ -28898, -15446 },
	{ -30273, -12539 }, 	{ -31356,  -9512 }, 	{ -32137,  -6393 }, 	{ -32609,  -3212 },
	{ -32767,      0 }, 	{ -32609,   3212 }, 	{ -32137,   6393 }, 	{ -31356,   9512 },
	{ -30273,  12539 }, 	{ -28898,  15446 }, 	{ -27245,  18204 }, 	{ -25329,  20787 },
	{ -23170,  23170 }, 	{ -20787,  25329 }, 	{ -18204,  27245 }, 	{ -15446,  28898 },
	{ -12539,  30273 }, 	{  -9512,  31356 }, 	{  -6393,  32137 }, 	{  -3212,  32609 },
	{      0,  32767 }
};
