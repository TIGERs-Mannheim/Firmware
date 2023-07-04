/*
 * motors_com.c
 *
 *  Created on: 05.01.2019
 *      Author: AndreR
 */

#include "motors.h"
#include "hal/port_ex.h"
#include "hal/buzzer.h"
#include "util/init_hal.h"
#include "util/fixed_point.h"
#include "util/console.h"
#include "util/log.h"
#include "util/angle_math.h"
#include "util/sys_time.h"
#include "constants.h"
#include "struct_ids.h"
#include "power.h"
#include "robot/ctrl.h"

// motor program code constants
#define MOTORS_TARGET_ADDRESS 0x08000000
#define MOTORS_SOURCE_ADDRESS 0x08180000
#define MOTORS_PROGRAM_SIZE (32*1024)

// convert hall commutation time to rad/s speed
// tCom = 48e6/(rads * 1/(2*float(%pi))*pp*6); (pp = 1 | 8)
#define HALL_COM_TIME_TO_RAD_PER_SEC_DRIVE_NUM (6283185.307179586f)
#define HALL_COM_TIME_TO_RAD_PER_SEC_DRIBBLER_NUM (5.026548245743668e7f)

// encoder speed conversion:
// encDelta = rads * 1/(2*float(%pi)) * 23040 * 1/1000;
// => encDelta = 3.666929888837269 * rad per sec
// => rad per sec = encDelta * 0.272707695624114
#define ENCODER_DELTA_TO_RAD_PER_SEC_DRIVE (0.272707695624114f)
#define RAD_PER_SEC_TO_ENCODER_DELTA_DRIVE (3.666929888837269f)

// Conversions for speed estimate from current measurement and setpoint voltage
#define MODEL_SPEED_TO_RAD_PER_SEC_DRIBBLER (1024.0f/1000.0f)
#define RAD_PER_SEC_TO_MODEL_SPEED_DRIBBLER (1000.0f/1024.0f)

#define GAIN_REAL_TO_MOTOR_CTRL_DQ (10800.0f/(4096.0f*4.0f))
#define GAIN_MOTOR_TO_REAL_CTRL_DQ (1.0f/GAIN_REAL_TO_MOTOR_CTRL_DQ)
#define GAIN_REAL_TO_DRIBBLER_CTRL_DQ (10800.0f/(4096.0f*8.0f))
#define GAIN_DRIBBLER_TO_REAL_CTRL_DQ (1.0f/GAIN_REAL_TO_DRIBBLER_CTRL_DQ)
#define GAIN_REAL_TO_MOTOR_CTRL_VEL (4096.0f/(10800.0f*2.0f))
#define GAIN_MOTOR_TO_REAL_CTRL_VEL (1.0f/GAIN_REAL_TO_MOTOR_CTRL_VEL)
#define GAIN_REAL_TO_DRIBBLER_CTRL_VEL (4096.0f/10800.0f)
#define GAIN_DRIBBLER_TO_REAL_CTRL_VEL (1.0f/GAIN_REAL_TO_DRIBBLER_CTRL_VEL)

#define MAX_CURRENT 10.8f

// data holders
Motors motors = {
	.focConfig = {
		.driveDKp = 1.642f,
		.driveDKi = 0.1876f,
		.driveQKp = 0.597f,
		.driveQKi = 0.0682f,
		.dribblerDKp = 0.0f, // not used by dribbler control
		.dribblerDKi = 0.0f, // not used by dribbler control
		.dribblerQKp = 0.143f,
		.dribblerQKi = 0.05f,
	},
	.velConfig = {
		.driveKp = 300.0f,
		.driveKi = 0.0f,
		.driveAntiJitter = 1.5f,
		.dribblerKp = 25.0f,
		.dribblerKi = 0.0f,
	},
};

static const ConfigFileDesc configFileDescMotorsFoc =
	{ SID_CFG_MOTORS_FOC, 1, "motors/foc", 8, (ElementDesc[]) {
		{ FLOAT,  "drv_d_kp", "", "drive/D/Kp" },
		{ FLOAT,  "drv_d_ki", "", "drive/D/Ki" },
		{ FLOAT,  "drv_q_kp", "", "drive/Q/Kp" },
		{ FLOAT,  "drv_q_ki", "", "drive/Q/Ki" },
		{ FLOAT, "drib_d_kp", "", "dribbler/D/Kp" },
		{ FLOAT, "drib_d_ki", "", "dribbler/D/Ki" },
		{ FLOAT, "drib_q_kp", "", "dribbler/Q/Kp" },
		{ FLOAT, "drib_q_ki", "", "dribbler/Q/Ki" },
	} };

static const ConfigFileDesc configFileDescMotors =
	{ SID_CFG_MOTORS_VEL, 7, "motors/vel", 5, (ElementDesc[]) {
		{ FLOAT,  "drv_kp", "", "drive/Kp" },
		{ FLOAT,  "drv_ki", "", "drive/Ki" },
		{ FLOAT,  "drv_antiJitter", "rad/s", "drive/antiJitter" },
		{ FLOAT,  "drib_kp", "", "dribbler/Kp" },
		{ FLOAT,  "drib_ki", "", "dribbler/Ki" },
	} };

#define RECORD_SIZE 10000
static int16_t recordCurrent[RECORD_SIZE*2]  __attribute__((aligned(1024), section(".sram0_dma")));;
static uint16_t recordCounter;

// forward declerations
static void startBootloader(uint8_t target);
static void receiveMotorData(uint32_t motorId);
static void sendMotorData(uint32_t motorId);
static void flashTask(void* params);
static void parallelUpdate(uint32_t force);

// ####################
// IRQs
// ####################
void USART1_IRQHandler() // Motor 1
{
	CH_IRQ_PROLOGUE();
	UARTFifoIRQ(&motors.motor[0].uart);
	CH_IRQ_EPILOGUE();
}

void UART7_IRQHandler() // Motor 2
{
	CH_IRQ_PROLOGUE();
	UARTFifoIRQ(&motors.motor[1].uart);
	CH_IRQ_EPILOGUE();
}

void UART4_IRQHandler() // Motor 3
{
	CH_IRQ_PROLOGUE();
	UARTFifoIRQ(&motors.motor[2].uart);
	CH_IRQ_EPILOGUE();
}

void USART3_IRQHandler() // Motor 4
{
	CH_IRQ_PROLOGUE();
	UARTFifoIRQ(&motors.motor[3].uart);
	CH_IRQ_EPILOGUE();
}

void UART5_IRQHandler() // Motor 5
{
	CH_IRQ_PROLOGUE();
	UARTFifoIRQ(&motors.motor[4].uart);
	CH_IRQ_EPILOGUE();
}

// ####################
// Public functions
// ####################
void MotorsInit()
{
	motors.recordMotor = 0xFF;
	motors.commKillMotor = 0xFF;

	motors.motor[4].hallOnly = 1; // Interpolation not working well on dribbler

	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;
	gpioInit.pupd = GPIO_PUPD_UP;

	// Motor 1 (FR)
	gpioInit.alternate = 7;
	GPIOInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10, &gpioInit);
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

	// Motor 2 (FL)
	gpioInit.alternate = 7;
	GPIOInit(GPIOF, GPIO_PIN_6 | GPIO_PIN_7, &gpioInit);
	RCC->APB1LENR |= RCC_APB1LENR_UART7EN;

	// Motor 3 (RL)
	gpioInit.alternate = 8;
	GPIOInit(GPIOA, GPIO_PIN_0 | GPIO_PIN_1, &gpioInit);
	RCC->APB1LENR |= RCC_APB1LENR_UART4EN;

	// Motor 4 (RR)
	gpioInit.alternate = 7;
	GPIOInit(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, &gpioInit);
	RCC->APB1LENR |= RCC_APB1LENR_USART3EN;

	// Motor 5 (Dribbler)
	gpioInit.alternate = 14;
	GPIOInit(GPIOB, GPIO_PIN_5 | GPIO_PIN_6, &gpioInit);
	RCC->APB1LENR |= RCC_APB1LENR_UART5EN;
	__DSB();

	UARTFifoInit(&motors.motor[0].uart, USART1, 2000000, systemClockInfo.APB2PeriphClk, 1);
	UARTFifoInit(&motors.motor[1].uart,  UART7, 2000000, systemClockInfo.APB1PeriphClk, 1);
	UARTFifoInit(&motors.motor[2].uart,  UART4, 2000000, systemClockInfo.APB1PeriphClk, 1);
	UARTFifoInit(&motors.motor[3].uart, USART3, 2000000, systemClockInfo.APB1PeriphClk, 1);
	UARTFifoInit(&motors.motor[4].uart,  UART5, 2000000, systemClockInfo.APB1PeriphClk, 1);

	NVICEnableIRQ(USART1_IRQn, IRQL_MOTOR_COMMS);
	NVICEnableIRQ(UART7_IRQn, IRQL_MOTOR_COMMS);
	NVICEnableIRQ(UART4_IRQn, IRQL_MOTOR_COMMS);
	NVICEnableIRQ(USART3_IRQn, IRQL_MOTOR_COMMS);
	NVICEnableIRQ(UART5_IRQn, IRQL_MOTOR_COMMS);

	motors.motor[0].portExTarget = PORT_EX_TARGET_MOTOR1;
	motors.motor[1].portExTarget = PORT_EX_TARGET_MOTOR2;
	motors.motor[2].portExTarget = PORT_EX_TARGET_MOTOR3;
	motors.motor[3].portExTarget = PORT_EX_TARGET_MOTOR4;
	motors.motor[4].portExTarget = PORT_EX_TARGET_DRIBBLER;

	motors.pFocConfigFile = ConfigOpenOrCreate(&configFileDescMotorsFoc, &motors.focConfig, sizeof(MotorsFocConfig), 0, 1);
	motors.pVelConfigFile = ConfigOpenOrCreate(&configFileDescMotors, &motors.velConfig, sizeof(MotorsVelConfig), 0, 0);

	ConfigNotifyUpdate(motors.pVelConfigFile);
}

void MotorsTask(void* params)
{
	(void)params;

	chRegSetThreadName("Motors");

	systime_t prev = chVTGetSystemTimeX();
	systime_t next = prev+US2ST(10000);

	parallelUpdate(0);

	while(1)
	{
		// get new data from motors
		for(uint8_t motor = 0; motor < 5; motor++)
		{
			receiveMotorData(motor);
		}

		// send new data to motors
		for(uint8_t motor = 0; motor < 5; motor++)
		{
			sendMotorData(motor);
		}

		// wait for all motor transmissions to finish
		for(uint8_t motor = 0; motor < 5; motor++)
		{
			// need to finish communication in reverse order to make mutices happy
			MotorsSingle* pMotor = &motors.motor[4-motor];

			// TODO: report failures to some statistics structure
			if(UARTFifoWriteWait(&pMotor->uart, MS2ST(10)))
				LogWarnC("Motor TX timeout", motor);
		}

		chThdSleepUntilWindowed(prev, next);
		prev = next;
		next += US2ST(1000);
	}
}

void MotorsSetOff(uint8_t motor)
{
	motors.motor[motor].mode = MOTOR_MODE_OFF;
}

void MotorsSetVoltageAB(uint8_t motor, float voltageA, float voltageB)
{
	motors.motor[motor].mode = MOTOR_MODE_VOLT_AB;
	motors.motor[motor].setVoltage[0] = voltageA;
	motors.motor[motor].setVoltage[1] = voltageB;
}

void MotorsSetElectricalAngle(uint8_t motor, float angle, float voltage)
{
	motors.motor[motor].mode = MOTOR_MODE_VOLT_AB;

	float sin;
	float cos;
	arm_sin_cos_f32(AngleNormalize(angle) * 180.0f/(float)M_PI, &sin, &cos);

	motors.motor[motor].setVoltage[0] = cos * voltage;
	motors.motor[motor].setVoltage[1] = sin * voltage;
}

void MotorsSetVoltageDQ(uint8_t motor, float voltageD, float voltageQ)
{
	motors.motor[motor].mode = MOTOR_MODE_VOLT_DQ;
	motors.motor[motor].setVoltage[0] = voltageD;
	motors.motor[motor].setVoltage[1] = voltageQ;
}

void MotorsSetCurrentDVoltageQ(uint8_t motor, float currentD, float voltageQ)
{
	currentD = fminf(fmaxf(currentD, -MAX_CURRENT), MAX_CURRENT);

	motors.motor[motor].mode = MOTOR_MODE_CUR_D_VOLT_Q;
	motors.motor[motor].setCurrent[0] = currentD;
	motors.motor[motor].setVoltage[1] = voltageQ;
}

void MotorsSetCurrentDQ(uint8_t motor, float currentD, float currentQ)
{
	currentD = fminf(fmaxf(currentD, -MAX_CURRENT), MAX_CURRENT);
	currentQ = fminf(fmaxf(currentQ, -MAX_CURRENT), MAX_CURRENT);

	motors.motor[motor].mode = MOTOR_MODE_CUR_DQ;
	motors.motor[motor].setCurrent[0] = currentD;
	motors.motor[motor].setCurrent[1] = currentQ;
}

void MotorsSetVelocity(uint8_t motor, float velocity, float currentD, float currentQ)
{
	currentD = fminf(fmaxf(currentD, -MAX_CURRENT), MAX_CURRENT);
	currentQ = fminf(fmaxf(currentQ, -MAX_CURRENT), MAX_CURRENT);

	motors.motor[motor].mode = MOTOR_MODE_SPEED;
	motors.motor[motor].setVelocity = velocity;
	motors.motor[motor].setCurrent[0] = currentD;
	motors.motor[motor].setCurrent[1] = currentQ;
}

void MotorsSetPIGainsCurrentDDrive(float kp, float ki)
{
	motors.focConfig.driveDKp = FixedPointUnsignedSaturate(kp*GAIN_REAL_TO_MOTOR_CTRL_DQ, 7, 10)*GAIN_MOTOR_TO_REAL_CTRL_DQ;
	motors.focConfig.driveDKi = FixedPointUnsignedSaturate(ki*GAIN_REAL_TO_MOTOR_CTRL_DQ, 5, 13)*GAIN_MOTOR_TO_REAL_CTRL_DQ;

	ConfigNotifyUpdate(motors.pFocConfigFile);
}

void MotorsSetPIGainsCurrentDDribbler(float kp, float ki)
{
	motors.focConfig.dribblerDKp = FixedPointUnsignedSaturate(kp*GAIN_REAL_TO_DRIBBLER_CTRL_DQ, 7, 10)*GAIN_DRIBBLER_TO_REAL_CTRL_DQ;
	motors.focConfig.dribblerDKi = FixedPointUnsignedSaturate(ki*GAIN_REAL_TO_DRIBBLER_CTRL_DQ, 5, 13)*GAIN_DRIBBLER_TO_REAL_CTRL_DQ;

	ConfigNotifyUpdate(motors.pFocConfigFile);
}

void MotorsSetPIGainsCurrentQDrive(float kp, float ki)
{
	motors.focConfig.driveQKp = FixedPointUnsignedSaturate(kp*GAIN_REAL_TO_MOTOR_CTRL_DQ, 7, 10)*GAIN_MOTOR_TO_REAL_CTRL_DQ;
	motors.focConfig.driveQKi = FixedPointUnsignedSaturate(ki*GAIN_REAL_TO_MOTOR_CTRL_DQ, 5, 13)*GAIN_MOTOR_TO_REAL_CTRL_DQ;

	ConfigNotifyUpdate(motors.pFocConfigFile);
}

void MotorsSetPIGainsCurrentQDribbler(float kp, float ki)
{
	motors.focConfig.dribblerQKp = FixedPointUnsignedSaturate(kp*GAIN_REAL_TO_DRIBBLER_CTRL_DQ, 7, 10)*GAIN_DRIBBLER_TO_REAL_CTRL_DQ;
	motors.focConfig.dribblerQKi = FixedPointUnsignedSaturate(ki*GAIN_REAL_TO_DRIBBLER_CTRL_DQ, 5, 13)*GAIN_DRIBBLER_TO_REAL_CTRL_DQ;

	ConfigNotifyUpdate(motors.pFocConfigFile);
}

void MotorsSetPIGainsVelocityDrive(float kp, float ki)
{
	motors.velConfig.driveKp = FixedPointUnsignedSaturate(kp*GAIN_REAL_TO_MOTOR_CTRL_VEL, 7, 10)*GAIN_MOTOR_TO_REAL_CTRL_VEL;
	motors.velConfig.driveKi = FixedPointUnsignedSaturate(ki*GAIN_REAL_TO_MOTOR_CTRL_VEL, 5, 13)*GAIN_MOTOR_TO_REAL_CTRL_VEL;

	ConfigNotifyUpdate(motors.pVelConfigFile);
}

void MotorsSetPIGainsVelocityDribbler(float kp, float ki)
{
	motors.velConfig.dribblerKp = FixedPointUnsignedSaturate(kp*GAIN_REAL_TO_DRIBBLER_CTRL_VEL, 7, 10)*GAIN_DRIBBLER_TO_REAL_CTRL_VEL;
	motors.velConfig.dribblerKi = FixedPointUnsignedSaturate(ki*GAIN_REAL_TO_DRIBBLER_CTRL_VEL, 5, 13)*GAIN_DRIBBLER_TO_REAL_CTRL_VEL;

	ConfigNotifyUpdate(motors.pVelConfigFile);
}

void MotorsDebugRecord(uint8_t id, uint8_t recordMode)
{
	recordCounter = 0;
	motors.recordMode = recordMode;
	motors.recordMotor = id;
}

void MotorsDebugPrintRecording()
{
	for(uint16_t i = 0; i < RECORD_SIZE; i++)
	{
		ConsolePrint("% 6hd  % 6hd\r\n", recordCurrent[i*2], recordCurrent[i*2+1]);
		if(i % 20 == 0)
			chThdSleepMilliseconds(20);
	}
}

void MotorsDebugGetRecord(arm_matrix_instance_q15* pMat)
{
	pMat->numRows = RECORD_SIZE;
	pMat->numCols = 2;
	pMat->pData = recordCurrent;
}

// ####################
// Private functions
// ####################
static void startBootloader(uint8_t target)
{
	PortExBootSet(target, 1);
	PortExResetPulse(target);
	PortExBootSet(target, 0);
}

static void flashTask(void* params)
{
	chRegSetThreadName("FlashMotor");

	uint32_t allParams = *((uint32_t*)params);
	uint16_t motorId = allParams & 0xFFFF;
	uint16_t forceUpdate = allParams >> 16;

	if(motorId > 4)
		return;

	MotorsSingle* pMotor = &motors.motor[motorId];

	STBootloaderInit(&pMotor->bootloader, &pMotor->uart, &startBootloader, pMotor->portExTarget);

	uint32_t originalBaudrate = pMotor->uart.baudrate;
	UARTFifoSetBaudrate(&pMotor->uart, 115200);

	pMotor->flashFuncResult = STBootloaderFlash(&pMotor->bootloader, (uint8_t*)MOTORS_SOURCE_ADDRESS,
			MOTORS_PROGRAM_SIZE, MOTORS_TARGET_ADDRESS, forceUpdate, motorId, &pMotor->flashResult);

	UARTFifoSetBaudrate(&pMotor->uart, originalBaudrate);

	chThdExit(0);
}

static void parallelUpdate(uint32_t force)
{
	for(uint32_t i = 0; i < 5; i++)
	{
		MotorsSingle* pMotor = &motors.motor[i];

		pMotor->flashParam = force << 16 | i;
		pMotor->pFlashTask = chThdCreateStatic(pMotor->waFlashTask, sizeof(pMotor->waFlashTask), LOWPRIO,
				&flashTask, &pMotor->flashParam);
	}

	for(uint32_t i = 0; i < 5; i++)
	{
		MotorsSingle* pMotor = &motors.motor[i];

		chThdWait(pMotor->pFlashTask);
	}

	ConsolePrint("    Req  Upd  Res     Size    Blocks  Time\r\n");
	for(uint16_t i = 0; i < 5; i++)
	{
		MotorsSingle* pMotor = &motors.motor[i];
		ConsolePrint("M%hu  %hu    %hu    0x%04hX  %-6u%4u     %ums\r\n", i,
				(uint16_t)pMotor->flashResult.updateRequired, (uint16_t)pMotor->flashResult.updated,
				pMotor->flashFuncResult, pMotor->flashResult.programSize, pMotor->flashResult.numBlocks,
				pMotor->flashResult.timeUs/1000);
	}
}

static void receiveMotorData(uint32_t motorId)
{
	MotorsSingle* pMotor = &motors.motor[motorId];

	int16_t result;
	uint16_t fifoStatus = 0;
	uint32_t rxTime;
	uint16_t datasize;

	while(1)
	{
		// Read UART data
		datasize = sizeof(MotorExchangeMISO);
		result = UARTFifoRead(&pMotor->uart, (uint8_t*)&pMotor->miso, &datasize, &fifoStatus, &rxTime);
		if(result == UART_FIFO_ERROR_NO_DATA)
			break;
		if(result)
		{
			LogWarnC("read error", result | motorId << 8 | (uint32_t)fifoStatus << 16);
			continue;
		}
		if(datasize < (sizeof(MotorExchangeMISO) - sizeof(pMotor->miso.currentMeas)))
		{
			LogWarnC("Short read", datasize | motorId << 8);
			continue;
		}

		// process CPU load
		pMotor->rxTime = rxTime;
		pMotor->cpuLoad = ((float)(pMotor->miso.idleTicksNoLoad - pMotor->miso.idleTicksActive))/((float)pMotor->miso.idleTicksNoLoad)*100.0f;

		// process voltage, current, and temperature measurement
		pMotor->avgSupplyVoltage1ms = pMotor->miso.avgSupplyVoltage * 0.001f;
		pMotor->avgTemperature1ms = pMotor->miso.avgTemperature * 0.1f;

		pMotor->avgCurrentUVW1ms[0] = pMotor->miso.avgCurrentUVW[0] * 0.001f;
		pMotor->avgCurrentUVW1ms[1] = pMotor->miso.avgCurrentUVW[1] * 0.001f;
		pMotor->avgCurrentUVW1ms[2] = pMotor->miso.avgCurrentUVW[2] * 0.001f;
		pMotor->avgCurrentDQ1ms[0] = pMotor->miso.avgCurrentDQ[0] * 0.001f;
		pMotor->avgCurrentDQ1ms[1] = pMotor->miso.avgCurrentDQ[1] * 0.001f;
		pMotor->avgVoltageDQ1ms[0] = pMotor->miso.avgVoltageDQ[0] * 0.001f;
		pMotor->avgVoltageDQ1ms[1] = pMotor->miso.avgVoltageDQ[1] * 0.001f;
		pMotor->currentOffset = pMotor->miso.currentOffset * 0.001f;

		// process speed measurements
		if(pMotor->miso.hallComTime == INT32_MAX || pMotor->miso.hallComTime == 0)
		{
			pMotor->hallVelocity = 0.0f;
		}
		else
		{
			if(motorId > 3)
				pMotor->hallVelocity = HALL_COM_TIME_TO_RAD_PER_SEC_DRIBBLER_NUM/pMotor->miso.hallComTime;
			else
				pMotor->hallVelocity = HALL_COM_TIME_TO_RAD_PER_SEC_DRIVE_NUM/pMotor->miso.hallComTime;
		}

		if(motorId > 3)
			pMotor->encoderVelocity = pMotor->miso.encDelta * MODEL_SPEED_TO_RAD_PER_SEC_DRIBBLER;
		else
			pMotor->encoderVelocity = pMotor->miso.encDelta * ENCODER_DELTA_TO_RAD_PER_SEC_DRIVE;

		if((pMotor->miso.flags & MOTOR_EXCHANGE_MISO_FLAG_BREAK_ACTIVE) && motorId < 4)
		{
			BuzzerPlay(&buzzSeqUp20);
			LogWarnC("Active break on motor", motorId);
		}

		// record raw data if requested
		if(motors.recordMotor == motorId && pMotor->miso.hasDebugData && datasize >= sizeof(MotorExchangeMISO))
		{
			if(recordCounter+MOTOR_EXCHANGE_MISO_MAX_MEAS <= RECORD_SIZE)
			{
				memcpy(&recordCurrent[recordCounter*2], pMotor->miso.currentMeas, sizeof(int16_t)*MOTOR_EXCHANGE_MISO_MAX_MEAS*2);

				recordCounter += MOTOR_EXCHANGE_MISO_MAX_MEAS;
			}
			else
			{
				motors.recordMotor = 0xFF;
				motors.recordMode = MOTOR_EXCHANGE_MOSI_RECORD_NONE;
			}
		}
	}
}

static void sendMotorData(uint32_t motorId)
{
	if(motors.commKillMotor == motorId)
		return;

	MotorsSingle* pMotor = &motors.motor[motorId];
	MotorExchangeMOSI* pMosi = &pMotor->mosi;

	// Motor controller has some fixed built-in proportional gains to adjust fixed-point ranges.
	// Need to adjust gains here to reverse these fixed gains.
	if(motorId > 3)
	{
		pMosi->curCtrlDKp = FixedPointSignedFromFloat(motors.focConfig.dribblerDKp*GAIN_REAL_TO_DRIBBLER_CTRL_DQ, 10);
		pMosi->curCtrlDKi = FixedPointSignedFromFloat(motors.focConfig.dribblerDKi*GAIN_REAL_TO_DRIBBLER_CTRL_DQ, 13);
		pMosi->curCtrlQKp = FixedPointSignedFromFloat(motors.focConfig.dribblerQKp*GAIN_REAL_TO_DRIBBLER_CTRL_DQ, 10);
		pMosi->curCtrlQKi = FixedPointSignedFromFloat(motors.focConfig.dribblerQKi*GAIN_REAL_TO_DRIBBLER_CTRL_DQ, 13);
		pMosi->velCtrlKp = FixedPointSignedFromFloat(motors.velConfig.dribblerKp*GAIN_REAL_TO_DRIBBLER_CTRL_VEL, 10);
		pMosi->velCtrlKi = FixedPointSignedFromFloat(motors.velConfig.dribblerKi*GAIN_REAL_TO_DRIBBLER_CTRL_VEL, 13);
		pMosi->resistance = FixedPointUnsignedFromFloatSaturate(botParams.dribbler.motor.R, 4, 6);
		pMosi->backEmfConstantInv = FixedPointUnsignedFromFloatSaturate(1.0f/botParams.dribbler.motor.Ke, 10, 2);
	}
	else
	{
		pMosi->curCtrlDKp = FixedPointSignedFromFloat(motors.focConfig.driveDKp*GAIN_REAL_TO_MOTOR_CTRL_DQ, 10);
		pMosi->curCtrlDKi = FixedPointSignedFromFloat(motors.focConfig.driveDKi*GAIN_REAL_TO_MOTOR_CTRL_DQ, 13);
		pMosi->curCtrlQKp = FixedPointSignedFromFloat(motors.focConfig.driveQKp*GAIN_REAL_TO_MOTOR_CTRL_DQ, 10);
		pMosi->curCtrlQKi = FixedPointSignedFromFloat(motors.focConfig.driveQKi*GAIN_REAL_TO_MOTOR_CTRL_DQ, 13);
		pMosi->velCtrlKp = FixedPointSignedFromFloat(motors.velConfig.driveKp*GAIN_REAL_TO_MOTOR_CTRL_VEL, 10);
		pMosi->velCtrlKi = FixedPointSignedFromFloat(motors.velConfig.driveKi*GAIN_REAL_TO_MOTOR_CTRL_VEL, 13);
		pMosi->resistance = FixedPointUnsignedFromFloatSaturate(botParams.driveTrain.motor.R, 4, 6);
		pMosi->backEmfConstantInv = FixedPointUnsignedFromFloatSaturate(1.0f/botParams.driveTrain.motor.Ke, 10, 2);
		pMosi->velCtrlAntiJitter = (int32_t)(RAD_PER_SEC_TO_ENCODER_DELTA_DRIVE * motors.velConfig.driveAntiJitter);
	}

	if(motors.recordMotor == motorId)
		pMosi->recordMode = motors.recordMode;
	else
		pMosi->recordMode = 0;

	pMosi->flags = 0;

	if(pMotor->hallOnly)
		pMosi->flags |= MOTOR_EXCHANGE_MOSI_FLAG_HALL_ONLY;

	pMosi->motorMode = pMotor->mode;

	switch(pMotor->mode)
	{
		case MOTOR_MODE_VOLT_AB:
		case MOTOR_MODE_VOLT_DQ:
		{
			pMosi->input[0] = (int32_t)(pMotor->setVoltage[0]*1000.0f);;
			pMosi->input[1] = (int32_t)(pMotor->setVoltage[1]*1000.0f);
		}
		break;
		case MOTOR_MODE_CUR_D_VOLT_Q:
		{
			pMosi->input[0] = (int32_t)(pMotor->setCurrent[0]*1000.0f);;
			pMosi->input[1] = (int32_t)(pMotor->setVoltage[1]*1000.0f);
		}
		break;
		case MOTOR_MODE_CUR_DQ:
		{
			pMosi->input[0] = (int16_t)(pMotor->setCurrent[0]*1000.0f);
			pMosi->input[1] = (int16_t)(pMotor->setCurrent[1]*1000.0f);
		}
		break;
		case MOTOR_MODE_SPEED:
		{
			pMosi->input[0] = (int16_t)(pMotor->setCurrent[0]*1000.0f);
			pMosi->input[1] = (int16_t)(pMotor->setCurrent[1]*1000.0f);

			if(motorId > 3)
			{
				pMosi->encDeltaSetpoint = (int16_t)(RAD_PER_SEC_TO_MODEL_SPEED_DRIBBLER * pMotor->setVelocity);
			}
			else
			{
				pMosi->encDeltaSetpoint = (int16_t)(RAD_PER_SEC_TO_ENCODER_DELTA_DRIVE * pMotor->setVelocity);
			}
		}
		break;
		default:
		{
			pMosi->motorMode = MOTOR_MODE_OFF;
		}
		break;
	}

	pMotor->txTime = SysTimeUSec();

	UARTFifoWriteStart(&pMotor->uart, (uint8_t*)&pMotor->mosi, sizeof(MotorExchangeMOSI));
}
