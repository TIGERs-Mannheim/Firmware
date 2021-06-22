/*
 * motors.c
 *
 *  Created on: 02.11.2015
 *      Author: AndreR
 */

#include "motors.h"
#include "util/init_hal.h"
#include "util/console.h"
#include "util/sys_time.h"
#include "util/angle_math.h"
#include "../power.h"
#include "arm_math.h"
#include "main/ctrl.h"
#include "commands.h"
#include "struct_ids.h"

Motors motors = {
	.config = {
		.Kp = 0.1f,
		.Ki = 10.0f,
		.Kd = 0.00f,
		.Ktp = 0.0f,
		.Ktd = 0.0f,
		.K = 35.0f,
		.T = 0.1f,
	},
	.motDamping = { 6.7847e-5f, 6.7847e-5f, 6.7847e-5f, 6.7847e-5f },
};

static const ConfigFileDesc configFileDescMotors =
	{ SID_CFG_MOTORS, 2, "motors", 7, (ElementDesc[]) {
		{ FLOAT, "pid_p", "", "PID/p" },
		{ FLOAT, "pid_i", "", "PID/i" },
		{ FLOAT, "pid_d", "", "PID/d" },
		{ FLOAT, "pid_tp", "s", "PID/tp" },
		{ FLOAT, "pid_td", "s", "PID/td" },
		{ FLOAT, "model_K", "", "model/K" },
		{ FLOAT, "model_T", "s", "model/T" },
	} };


/**
 * FIR Lowpass Filter
 * Order: 20
 * Constrained Equiripple
 * Fs: 100Hz
 * Passband edge: 1Hz
 * Apass: 0.1dB, Astop: 40dB
 * Quantized to single-precision float
 */
static float dribLPCoeff[21] = {
   -0.00784308929,-0.005474002566,-0.002820767462, 0.005069722887,  0.01922049001,
    0.03935496882,  0.06363747269,  0.08884492517,   0.1109740734,   0.1261434853,
     0.1315418482,   0.1261434853,   0.1109740734,  0.08884492517,  0.06363747269,
    0.03935496882,  0.01922049001, 0.005069722887,-0.002820767462,-0.005474002566,
   -0.00784308929
};

static void configUpdateCallback(uint16_t cfgId)
{
	if(cfgId == SID_CFG_MOTORS)
	{
		for(uint8_t i = 0; i < 4; i++)
		{
			PIDSetParams(&motors.drive[i].pid, motors.config.Kp, motors.config.Ki, motors.config.Kd);
			PIDSetPFilterParam(&motors.drive[i].pid, motors.config.Ktp);
			PIDSetDFilterParam(&motors.drive[i].pid, motors.config.Ktd);
		}
	}
}

void MotorsInit()
{
	// init data struct
	motors.drive[0].pFaultPort = GPIOG;
	motors.drive[0].ff1 = GPIO_PIN_7;
	motors.drive[0].ff2 = GPIO_PIN_8;
	motors.drive[0].pCCR = &TIM5->CCR2;
	motors.drive[0].pEncCNT = &TIM1->CNT;
	motors.drive[0].pDirPort = GPIOD;
	motors.drive[0].dirPin = GPIO_PIN_6;

	motors.drive[1].pFaultPort = GPIOE;
	motors.drive[1].ff1 = GPIO_PIN_4;
	motors.drive[1].ff2 = GPIO_PIN_3;
	motors.drive[1].pCCR = &TIM5->CCR1;
	motors.drive[1].pEncCNT = &TIM3->CNT;
	motors.drive[1].pDirPort = GPIOC;
	motors.drive[1].dirPin = GPIO_PIN_14;

	motors.drive[2].pFaultPort = GPIOC;
	motors.drive[2].ff1 = GPIO_PIN_5;
	motors.drive[2].ff2 = GPIO_PIN_4;
	motors.drive[2].pCCR = &TIM5->CCR3;
	motors.drive[2].pEncCNT = &TIM4->CNT;
	motors.drive[2].pDirPort = GPIOB;
	motors.drive[2].dirPin = GPIO_PIN_0;

	motors.drive[3].pFaultPort = GPIOB;
	motors.drive[3].ff1 = GPIO_PIN_12;
	motors.drive[3].ff2 = GPIO_PIN_13;
	motors.drive[3].pCCR = &TIM5->CCR4;
	motors.drive[3].pEncCNT = &TIM8->CNT;
	motors.drive[3].pDirPort = GPIOD;
	motors.drive[3].dirPin = GPIO_PIN_11;

	FlashFSOpenOrCreate("motors/damping", 0, &motors.motDamping, sizeof(motors.motDamping), &motors.pMotDampingFile);

	PIDParams params;
	params.kp = 0.10f; // 0.15f
	params.ki = 10.0f; //15.0f
	params.kd = 0.0f;
	params.outputMin = -10.0f;	// [V]
	params.outputMax = 10.0f;	// [V]
	params.dFilterTimeConstant = 0.0f;
	params.pFilterTimeConstant = 0.0f;

	for(uint8_t i = 0; i < 4; i++)
	{
		PIDInit(&motors.drive[i].pid, &params, 0.001f, 1);
	}

	params.kp = 0.02f;
	params.ki = 0.2f;
	params.kd = 0.0f;
	params.outputMin = 0.0f;	// [V]
	PIDInit(&motors.dribbler.pid, &params, 0.01f, 1);

	arm_fir_init_f32(&motors.dribbler.velFilter, MOTORS_DRIB_FILT_NUM_TAPS, dribLPCoeff, motors.dribbler.velFiltState, 1);

	GPIOInitData gpioInit;

	// ### Configure GPIOs ###

	// Drive Reset pin - pull low to keep reset during init
	GPIOReset(GPIOG, GPIO_PIN_0);

	gpioInit.mode = GPIO_MODE_OUTPUT;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;
	gpioInit.pupd = GPIO_PUPD_NONE;
	GPIOInit(GPIOG, GPIO_PIN_0, &gpioInit);

	// Dribbler reset pin - keep low
	GPIOReset(GPIOF, GPIO_PIN_5);
	GPIOInit(GPIOF, GPIO_PIN_5, &gpioInit);

	// initialize direction outputs
	GPIOInit(GPIOB, GPIO_PIN_0, &gpioInit); // M3
	GPIOInit(GPIOC, GPIO_PIN_14, &gpioInit); // M2
	GPIOInit(GPIOD, GPIO_PIN_6 | GPIO_PIN_11, &gpioInit); // M1, M4
	GPIOInit(GPIOG, GPIO_PIN_9, &gpioInit);	// M5
	GPIOReset(GPIOG, GPIO_PIN_9);

	// initialize fault inputs
	gpioInit.mode = GPIO_MODE_INPUT;
	gpioInit.pupd = GPIO_PUPD_UP;
	GPIOInit(GPIOB, GPIO_PIN_12 | GPIO_PIN_13, &gpioInit); // M4
	GPIOInit(GPIOC, GPIO_PIN_4 | GPIO_PIN_5, &gpioInit); // M3
	GPIOInit(GPIOE, GPIO_PIN_3 | GPIO_PIN_4, &gpioInit); // M2
	GPIOInit(GPIOF, GPIO_PIN_6 | GPIO_PIN_7, &gpioInit); // M5
	GPIOInit(GPIOG, GPIO_PIN_7 | GPIO_PIN_8, &gpioInit); // M1

	// M5 direction input
	gpioInit.pupd = GPIO_PUPD_DOWN;
	GPIOInit(GPIOF, GPIO_PIN_4, &gpioInit);

	// mode outputs
	gpioInit.mode = GPIO_MODE_OUTPUT;
	gpioInit.pupd = GPIO_PUPD_NONE;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;
	GPIOInit(GPIOC, GPIO_PIN_13, &gpioInit); // dribbler mode
	GPIOInit(GPIOF, GPIO_PIN_10, &gpioInit); // drive mode
	GPIOSet(GPIOC, GPIO_PIN_13);
//	GPIOReset(GPIOF, GPIO_PIN_10);
	GPIOSet(GPIOF, GPIO_PIN_10);


	// M1 - M4 timer output
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_25MHZ;
	gpioInit.pupd = GPIO_PUPD_NONE;
	gpioInit.alternate = 2;
	GPIOInit(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, &gpioInit);

	// M5 timer output
	gpioInit.alternate = 3;
	GPIOInit(GPIOB, GPIO_PIN_9, &gpioInit);

	// Encoder inputs
	gpioInit.otype = GPIO_OTYPE_OPEN_DRAIN;
	gpioInit.alternate = 1;
	GPIOInit(GPIOA, GPIO_PIN_8 | GPIO_PIN_9, &gpioInit); // M1

	gpioInit.alternate = 2;
	GPIOInit(GPIOB, GPIO_PIN_5, &gpioInit); // M2
	GPIOInit(GPIOA, GPIO_PIN_6, &gpioInit);

	gpioInit.alternate = 3;
	GPIOInit(GPIOC, GPIO_PIN_6 | GPIO_PIN_7, &gpioInit); // M4

	gpioInit.alternate = 2;
	GPIOInit(GPIOD, GPIO_PIN_12 | GPIO_PIN_13, &gpioInit); // M3

	// M5 Tacho
	gpioInit.alternate = 1;
	GPIOInit(GPIOA, GPIO_PIN_5, &gpioInit);

	// ### Configure TIMs ###

	// M1 - M4
	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
	TIM5->CR1 = 0;
	TIM5->CR2 = 0;
	TIM5->PSC = 0; // = 216MHz
	TIM5->ARR = 7199; // = 30kHz
	// enable preload, PWM mode 1
	TIM5->CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
	TIM5->CCMR2 = TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC4PE | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;
	// output enable, active high
	TIM5->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
	TIM5->CCR1 = 0;
	TIM5->CCR2 = 0;
	TIM5->CCR3 = 0;
	TIM5->CCR4 = 0;
	TIM5->EGR = TIM_EGR_UG;
	TIM5->CR1 = TIM_CR1_CEN;
	TIM5->SR = 0;

	// M5
	RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
	TIM11->CR1 = 0;
	TIM11->PSC = 0; // = 216MHz
	TIM11->ARR = 7199; // = 30kHz
	TIM11->CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	TIM11->CCER = TIM_CCER_CC1E;
	TIM11->CCR1 = 0;
	TIM11->EGR = TIM_EGR_UG;
	TIM11->CR1 = TIM_CR1_CEN;
	TIM11->SR = 0;

	// Encoder 1
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	TIM1->CR1 = 0;
	TIM1->CR2 = 0;
	TIM1->PSC = 0;
	TIM1->ARR = 0xFFFF;
	TIM1->RCR = 0;
	// IC1 mapped to TI1, IC2 mapped to TI2, filter = 0x0F on both, no prescaler
	TIM1->CCMR1 = TIM_CCMR1_CC1S_0 | (0x0F << 4) | TIM_CCMR1_CC2S_0 | (0x0F << 12);
	TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;	// enable input capture, rising polarity
	TIM1->DIER = 0;
	TIM1->SMCR = 3;	// encoder mode 3, count all flanks
	TIM1->CR1 = TIM_CR1_CEN;

	// Encoder 2
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	TIM3->CR1 = 0;
	TIM3->CR2 = 0;
	TIM3->PSC = 0;
	TIM3->ARR = 0xFFFF;
	// IC1 mapped to TI1, IC2 mapped to TI2, filter = 0x0F on both, no prescaler
	TIM3->CCMR1 = TIM_CCMR1_CC1S_0 | (0x0F << 4) | TIM_CCMR1_CC2S_0 | (0x0F << 12);
	TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;	// enable input capture, rising polarity
	TIM3->DIER = 0;
	TIM3->SMCR = 3;	// encoder mode 3, count all flanks
	TIM3->CR1 = TIM_CR1_CEN;

	// Encoder 3
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	TIM4->CR1 = 0;
	TIM4->CR2 = 0;
	TIM4->PSC = 0;
	TIM4->ARR = 0xFFFF;
	// IC1 mapped to TI1, IC2 mapped to TI2, filter = 0x0F on both, no prescaler
	TIM4->CCMR1 = TIM_CCMR1_CC1S_0 | (0x0F << 4) | TIM_CCMR1_CC2S_0 | (0x0F << 12);
	TIM4->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;	// enable input capture, rising polarity
	TIM4->DIER = 0;
	TIM4->SMCR = 3;	// encoder mode 3, count all flanks
	TIM4->CR1 = TIM_CR1_CEN;

	// Encoder 4
	RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
	TIM8->CR1 = 0;
	TIM8->CR2 = 0;
	TIM8->PSC = 0;
	TIM8->ARR = 0xFFFF;
	TIM8->RCR = 0;
	// IC1 mapped to TI1, IC2 mapped to TI2, filter = 0x0F on both, no prescaler
	TIM8->CCMR1 = TIM_CCMR1_CC1S_0 | (0x0F << 4) | TIM_CCMR1_CC2S_0 | (0x0F << 12);
	TIM8->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;	// enable input capture, rising polarity
	TIM8->DIER = 0;
	TIM8->SMCR = 3;	// encoder mode 3, count all flanks
	TIM8->CR1 = TIM_CR1_CEN;

	// Motor 5 tacho input
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	TIM2->CR1 = 0;
	TIM2->CR2 = 0;
	TIM2->PSC = 0;	// = 216MHz
	TIM2->ARR = 0xFFFFFFFF;
	// IC1 and IC2 mapped to TI1
	TIM2->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_1 | (0x0F << 4) | (0x0F << 12);
	// TI1 rising edge, TI2 falling edge
	TIM2->CCER = TIM_CCER_CC2P;
	TIM2->DIER = 0;
	// slave mode not used
	TIM2->SMCR = 0;
	// enable inputs
	TIM2->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
	TIM2->CR1 = TIM_CR1_CEN;

	motors.pConfigFileMotors = ConfigOpenOrCreate(&configFileDescMotors, &motors.config, sizeof(MotorsConfig), &configUpdateCallback, 0);

	ConfigNotifyUpdate(motors.pConfigFileMotors);
}

static void updateFaultStatus(uint8_t motor)
{
	motors.drive[motor].faultStatus = 0;

	if(motors.drive[motor].pFaultPort->IDR & motors.drive[motor].ff1)
		motors.drive[motor].faultStatus |= 0x01;

	if(motors.drive[motor].pFaultPort->IDR & motors.drive[motor].ff2)
		motors.drive[motor].faultStatus |= 0x02;
}

static inline void driveEnable(uint8_t enable)
{
	if(enable)
		GPIOSet(GPIOG, GPIO_PIN_0);
	else
		GPIOReset(GPIOG, GPIO_PIN_0);
}

static inline void dribblerEnable(uint8_t enable)
{
	if(enable)
		GPIOSet(GPIOF, GPIO_PIN_5);
	else
		GPIOReset(GPIOF, GPIO_PIN_5);
}

static inline void driveSetVoltage(uint8_t motor, float vol)
{
	motors.drive[motor].outVoltage = vol;
	motors.drive[motor].outVoltageTimestamp = SysTimeUSec();

	int32_t ccr;
	if(GPIOF->ODR & GPIO_PIN_10)
	{
		ccr = (int32_t)(fabsf(vol)/power.vBat*7200.0f);
		if(vol < 0.0f)
			GPIOReset(motors.drive[motor].pDirPort, motors.drive[motor].dirPin);
		else
			GPIOSet(motors.drive[motor].pDirPort, motors.drive[motor].dirPin);
	}
	else
	{
		ccr = (int32_t)(-vol/power.vBat*3600.0f+3600.0f);
		GPIOReset(motors.drive[motor].pDirPort, motors.drive[motor].dirPin);
	}

	if(ccr < 5)
		ccr = 5;
	if(ccr > 7195)
		ccr = 7195;

	*motors.drive[motor].pCCR = ccr;
}

static inline void dribblerSetVoltage(float vol)
{
	motors.dribbler.outVoltage = vol;
	motors.dribbler.outVoltageTimestamp = SysTimeUSec();

	if(vol < 0)
	{
		GPIOG->ODR |= GPIO_PIN_9;
		vol *= -1.0f;
	}
	else
	{
		GPIOG->ODR &= ~GPIO_PIN_9;
	}

	int32_t ccr = (int32_t)(vol/power.vBat*7200.0f);
//	int32_t ccr = (int32_t)(-vol/power.vBat*3600.0f+3600.0f);

	if(ccr < 5)
		ccr = 5;
	if(ccr > 7195)
		ccr = 7195;

	TIM11->CCR1 = ccr;
}

static inline void modelFuncMotor(float stateVel, float ctrlVol, float* pVelOut, float* pAccOut)
{
	// PT1
	const float dT = 0.001f;
	const float T2 = 1.0f/((motors.config.T/dT)+1.0f);

	*pVelOut = T2*(motors.config.K*ctrlVol - stateVel) + stateVel;
	*pAccOut = (*pVelOut-stateVel)/dT;
}

static inline void driveCtrl()
{
	// Drive motors control
	if(motors.driveMode == MOT_CTRL_OFF)
		driveEnable(0);
	else
		driveEnable(1);

	for(uint16_t i = 0; i < 4; i++)
	{
		// Velocity estimation
		int16_t encPos = (int16_t)*motors.drive[i].pEncCNT;
		float encPosF = encPos;
		encPosF *= 1.0f/2048.0f*2.0f*PI;

		motors.drive[i].velocity = AngleNormalize(encPosF-motors.drive[i].lastEncPos)/0.001f;
		motors.drive[i].velocityTimestamp = SysTimeUSec();
		motors.drive[i].lastEncPos = encPosF;

		// Control

		// enable/disable PID depending on mode
		if(motors.driveMode == MOT_CTRL_VELOCITY || motors.driveMode == MOT_CTRL_VEL_TORQUE)
			PIDEnable(&motors.drive[i].pid, 1);
		else
			PIDEnable(&motors.drive[i].pid, 0);

		// always do PID control, output is not always applied depending on drive mode
		PIDSetLimits(&motors.drive[i].pid, -power.vBat, power.vBat);
		motors.drive[i].pid.setpoint = motors.drive[i].setVelocity;
		PIDUpdate(&motors.drive[i].pid, motors.drive[i].velocity);

		// set motor voltage depending on drive mode
		switch(motors.driveMode)
		{
			case MOT_CTRL_VELOCITY:
			{
				driveSetVoltage(i, motors.drive[i].pid.output);
			}
			break;
			case MOT_CTRL_VOLTAGE:
			{
				driveSetVoltage(i, motors.drive[i].setVoltage);
			}
			break;
			case MOT_CTRL_TORQUE:
			{
				float setVoltage = motors.drive[i].setTorque * CTRL_MOTOR_R/CTRL_MOTOR_KM + CTRL_MOTOR_KE*motors.drive[i].velocity;
				driveSetVoltage(i, setVoltage);
			}
			break;
			case MOT_CTRL_VEL_TORQUE:
			{
				float setVoltage = motors.drive[i].setTorque * CTRL_MOTOR_R/CTRL_MOTOR_KM + CTRL_MOTOR_KE*motors.drive[i].velocity;
				driveSetVoltage(i, setVoltage + motors.drive[i].pid.output);
			}
			break;
			default:
			{
				driveSetVoltage(i, 0);
			}
			break;
		}

		motors.drive[i].estCurrent = (motors.drive[i].outVoltage - CTRL_MOTOR_KE*motors.drive[i].velocity) * 1.0f/CTRL_MOTOR_R;
	}
}

static inline float dribblerModel(float U, const float curVel)
{
	// U1 = K0*U + K1*U^2 + K2*U^3 + K3*U^4
	// vel = T2*(U1 - prevVel) + prevVel

	if(U < 0)
		U = 0;

	const float K[4] = {98.41f, 3.972f, -0.57f, 0.0286f};
	const float T = 0.0563f;

	const float dt = 0.001f;
	const float T2 = 1.0f/(T/dt+1.0f);

	float vel = (K[0] + (K[1] + (K[2] + K[3]*U)*U)*U)*U;
	float pred = T2*(vel - curVel) + curVel;

	return pred;
}

static inline void dribblerCtrl()
{
	static uint8_t dribCnt = 0;

	// Dribbler velocity measurement & filter
	if(TIM2->SR & (TIM_SR_CC1IF | TIM_SR_CC2IF))
	{
		uint32_t cntNow = TIM2->CNT;
		uint32_t ccr1 = TIM2->CCR1;
		uint32_t ccr2 = TIM2->CCR2;

		uint32_t diff1 = cntNow-ccr1;
		uint32_t diff2 = cntNow-ccr2;
		uint32_t diff;
		if(diff1 < diff2)
		{
			diff = ccr1-ccr2;
		}
		else
		{
			diff = ccr2-ccr1;
		}

		float tCom = diff*1.0f/216e6f;
		float newVel = (1.0f/6.0f*2*PI)/tCom; // motor rad/s
//		uint8_t neg = ((GPIOF->IDR & GPIO_PIN_4) == 0) ? 1 : 0;
//		float mult = neg ? -1.0f : 1.0f;
//		newVel *= mult;

//			ConsolePrint("tCom: %f, mult: %f\r\n", tCom, mult);

		if(newVel < 2000.0f && newVel > -2000.0f)
		{
			motors.dribbler.lastVelMeas = newVel;
		}

		motors.dribbler.lastUpdateTime = chVTGetSystemTimeX();
		motors.dribbler.measurementCounter++;
	}

	if(chVTTimeElapsedSinceX(motors.dribbler.lastUpdateTime) > MS2ST(50))
	{
		motors.dribbler.lastVelMeas = 0;
		motors.dribbler.lastUpdateTime = chVTGetSystemTimeX();
	}

//	arm_fir_f32(&motors.dribbler.velFilter, &motors.dribbler.lastVelMeas, &motors.dribbler.velocity, 1);

	float pred = dribblerModel(motors.dribbler.outVoltage, motors.dribbler.velocity);

	const float alpha = 0.96f;
	const float aBias = 0.998f;

	motors.dribbler.velocity = (1.0f-alpha)*motors.dribbler.lastVelMeas + alpha*(pred+motors.dribbler.bias);
	float err = motors.dribbler.lastVelMeas - pred;
	motors.dribbler.bias = (1.0f-aBias)*err + aBias*motors.dribbler.bias;

	// dribbler control
	uint8_t pidRun = 0;

	++dribCnt;
	if(dribCnt == 10)
	{
		dribCnt = 0;
		pidRun = 1;
	}

	switch(motors.dribbler.mode)
	{
		case DRIBBLER_MODE_VOLTAGE:
		{
			dribblerSetVoltage(motors.dribbler.setVoltage);
		}
		break;
		case DRIBBLER_MODE_SPEED:
		{
			if(pidRun)
			{
				motors.dribbler.pid.setpoint = motors.dribbler.setVelocity;
				PIDUpdate(&motors.dribbler.pid, motors.dribbler.velocity);

				if(fabsf(motors.dribbler.setVelocity) < 1.0f)
				{
					if(fabsf(motors.dribbler.velocity) > 0.1f && ((GPIOF->IDR & GPIO_PIN_4)))
					{
						dribblerSetVoltage(-4.0f);
					}
					else
					{
						dribblerSetVoltage(0.0f);
					}
				}
				else
				{
					dribblerSetVoltage(motors.dribbler.pid.output);
				}
			}
		}
		break;
		default:
		{
			dribblerEnable(0);
			dribblerSetVoltage(0);
		}
		break;
	}

	if(motors.dribbler.outVoltage < 0.1f && fabsf(motors.dribbler.velocity) < 25.0f)
		dribblerEnable(0);
	else
		dribblerEnable(1);
}

void MotorsDriveOff()
{
	motors.driveMode = MOT_CTRL_OFF;
}

void MotorsDriveSetVoltage(uint8_t motor, float vol)
{
	motors.driveMode = MOT_CTRL_VOLTAGE;
	motors.drive[motor].setVoltage = vol;
}

void MotorsDriveSetVelocity(uint8_t motor, float vel)
{
	motors.driveMode = MOT_CTRL_VELOCITY;
	motors.drive[motor].setVelocity = vel;
}

void MotorsDribblerOff()
{
	motors.dribbler.mode = DRIBBLER_MODE_OFF;
}

void MotorsDribblerSetVoltage(float vol)
{
	if(vol > 12.0f)
		vol = 12.0f;

	if(vol < -12.0f)
		vol = -12.0f;

	motors.dribbler.mode = DRIBBLER_MODE_VOLTAGE;
	motors.dribbler.setVoltage = vol;
}

void MotorsDribblerSetVelocity(float velRPM)
{
	if(velRPM > 20000)
		velRPM = 20000;

	if(velRPM < -20000)
		velRPM = -20000;

	motors.dribbler.mode = DRIBBLER_MODE_SPEED;
	motors.dribbler.setVelocity = velRPM*MOTORS_DRIBBLER_BAR_RPM_TO_MOTOR_RADS;	// convert to motor rad/s
}

void MotorsDrivePrintFault()
{
	for(uint16_t i = 0; i < 4; i++)
	{
		updateFaultStatus(i);
		ConsolePrint("%hu: %hu\r\n", i, (uint16_t)motors.drive[i].faultStatus);
	}
}

void MotorsDrivePrintEncoders()
{
	for(uint16_t i = 0; i < 4; i++)
	{
		ConsolePrint("%hu: %hu\r\n", i, *motors.drive[i].pEncCNT);
	}
}

void MotorsSetDampingCoefficient(uint8_t motorId, float damp)
{
	if(motorId > 3)
		return;

	motors.motDamping[motorId] = damp;

	FlashFSWrite(&motors.pMotDampingFile, &motors.motDamping, sizeof(motors.motDamping));
}

void MotorsTask(void* params)
{
	(void)params;

	chRegSetThreadName("Motors");

	systime_t prev = chVTGetSystemTimeX();
	systime_t next = prev+US2ST(1000);

	while(1)
	{
		driveCtrl();

		dribblerCtrl();

		chThdSleepUntilWindowed(prev, next);
		prev = next;
		next += US2ST(1000);
	}
}
