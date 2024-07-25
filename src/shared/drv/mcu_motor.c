#include "mcu_motor.h"
#include "util/log.h"
#include "hal/sys_time.h"
#include "math/fixed_point.h"
#include "math/clamp.h"
#include "math/angle_math.h"
#include "arm_math.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static const ElementDesc mcuMotorFocConfigElements[] =
{
	{ FLOAT,  "cur_d_kp", "", "cur/D/Kp" },
	{ FLOAT,  "cur_d_ki", "", "cur/D/Ki" },
	{ FLOAT,  "cur_q_kp", "", "cur/Q/Kp" },
	{ FLOAT,  "cur_q_ki", "", "cur/Q/Ki" },
};

static const ElementDesc mcuMotorVelConfigElements[] =
{
	{ FLOAT,  "vel_kp", "", "vel/Kp" },
	{ FLOAT,  "vel_ki", "", "vel/Ki" },
	{ FLOAT,  "vel_antiJitter", "rad/s", "vel/antiJitter" },
};

static void motTask(void* pParam);
static void registerShellCommands(ShellCmdHandler* pHandler);

void McuMotorInit(McuMotor* pMot, McuMotorData* pInit, tprio_t prio)
{
	chMtxObjectInit(&pMot->measMutex);
	chMtxObjectInit(&pMot->outMutex);
	chEvtObjectInit(&pMot->eventSource);

	ShellCmdHandlerInit(&pMot->cmdHandler, pMot);
	registerShellCommands(&pMot->cmdHandler);

	STBootloaderInit(&pMot->bootloader, pInit->pUart, pInit->pRstPin, pInit->pBootPin);

	pMot->pUart = pInit->pUart;
	pMot->pFwProgram = pInit->pFwProgram;
	pMot->fwSize = pInit->fwSize;
	pMot->doFwUpdate = 1;
	pMot->motorId = pInit->motorId;
	pMot->pMotorParams = pInit->pMotorParams;
	pMot->pCtrlParams = pInit->pCtrlParams;
	pMot->pFocConfig = pInit->pFocConfig;
	pMot->pVelConfig = pInit->pVelConfig;
	pMot->pRecordData = pInit->pDebugRecordData;
	pMot->recordsMax = pInit->debugRecordSize;
	pMot->recordMode = MOTOR_EXCHANGE_MOSI_RECORD_NONE;

	DeviceProfilerInit(&pMot->profiler, 1.0f);

	snprintf(pMot->taskName, sizeof(pMot->taskName), "MCU_MOT%hu", (uint16_t)pMot->motorId);

	pMot->pTask = chThdCreateStatic(pMot->waTask, sizeof(pMot->waTask), prio, &motTask, pMot);
}

void McuMotorGet(McuMotor* pMot, McuMotorMeasurement* pMeas)
{
	chMtxLock(&pMot->measMutex);
	memcpy(pMeas, &pMot->meas, sizeof(pMot->meas));
	chMtxUnlock(&pMot->measMutex);
}

void McuMotorTriggerFwUpdate(McuMotor* pMot, uint8_t force)
{
	pMot->doFwUpdate = 1 + force;
}

void McuMotorSetOff(McuMotor* pMot)
{
	chMtxLock(&pMot->outMutex);
	pMot->mode = MOTOR_MODE_OFF;
	chMtxUnlock(&pMot->outMutex);
}

void McuMotorSetVoltageAB(McuMotor* pMot, float voltageA_V, float voltageB_V)
{
	chMtxLock(&pMot->outMutex);
	pMot->mode = MOTOR_MODE_VOLT_AB;
	pMot->setVoltage_V[0] = voltageA_V;
	pMot->setVoltage_V[1] = voltageB_V;
	chMtxUnlock(&pMot->outMutex);
}

void McuMotorSetElectricalAngle(McuMotor* pMot, float angle_rad, float voltage_V)
{
	chMtxLock(&pMot->outMutex);
	pMot->mode = MOTOR_MODE_VOLT_AB;

	float sin;
	float cos;
	arm_sin_cos_f32(AngleNormalize(angle_rad) * 180.0f/(float)M_PI, &sin, &cos);

	pMot->setVoltage_V[0] = cos * voltage_V;
	pMot->setVoltage_V[1] = sin * voltage_V;
	chMtxUnlock(&pMot->outMutex);
}

void McuMotorSetVoltageDQ(McuMotor* pMot, float voltageD_V, float voltageQ_V)
{
	chMtxLock(&pMot->outMutex);
	pMot->mode = MOTOR_MODE_VOLT_DQ;
	pMot->setVoltage_V[0] = voltageD_V;
	pMot->setVoltage_V[1] = voltageQ_V;
	chMtxUnlock(&pMot->outMutex);
}

void McuMotorSetCurrentDVoltageQ(McuMotor* pMot, float currentD_A, float voltageQ_V)
{
	chMtxLock(&pMot->outMutex);
	currentD_A = Clampf(currentD_A, pMot->pCtrlParams->maxCurrent_A);

	pMot->mode = MOTOR_MODE_CUR_D_VOLT_Q;
	pMot->setCurrent_A[0] = currentD_A;
	pMot->setVoltage_V[1] = voltageQ_V;
	chMtxUnlock(&pMot->outMutex);
}

void McuMotorSetCurrentDQ(McuMotor* pMot, float currentD_A, float currentQ_A)
{
	chMtxLock(&pMot->outMutex);
	currentD_A = Clampf(currentD_A, pMot->pCtrlParams->maxCurrent_A);
	currentQ_A = Clampf(currentQ_A, pMot->pCtrlParams->maxCurrent_A);

	pMot->mode = MOTOR_MODE_CUR_DQ;
	pMot->setCurrent_A[0] = currentD_A;
	pMot->setCurrent_A[1] = currentQ_A;
	chMtxUnlock(&pMot->outMutex);
}

void McuMotorSetVelocity(McuMotor* pMot, float velocity_radDs, float currentD_A, float currentQ_A)
{
	chMtxLock(&pMot->outMutex);
	currentD_A = Clampf(currentD_A, pMot->pCtrlParams->maxCurrent_A);
	currentQ_A = Clampf(currentQ_A, pMot->pCtrlParams->maxCurrent_A);

	pMot->mode = MOTOR_MODE_SPEED;
	pMot->setVelocity_radDs = velocity_radDs;
	pMot->setCurrent_A[0] = currentD_A;
	pMot->setCurrent_A[1] = currentQ_A;
	chMtxUnlock(&pMot->outMutex);
}

void McuMotorFocConfigDescInit(ConfigFileDesc* pDesc)
{
	pDesc->numElements = 4;
	pDesc->elements = mcuMotorFocConfigElements;
}

void McuMotorVelConfigDescInit(ConfigFileDesc* pDesc)
{
	pDesc->numElements = 3;
	pDesc->elements = mcuMotorVelConfigElements;
}

// Returns 1 if any value has been modified by clamping
uint8_t McuMotorClampFocConfig(McuMotorFocConfig* pCfg, const McuMotorCtrlParams* pParams)
{
	McuMotorFocConfig orig = *pCfg;

	const float curGainMotorToReal = 1.0f / pParams->curGainRealToMotor;

	pCfg->curDKp = FixedPointUnsignedSaturate(pCfg->curDKp * pParams->curGainRealToMotor, 7, 10) * curGainMotorToReal;
	pCfg->curDKi = FixedPointUnsignedSaturate(pCfg->curDKi * pParams->curGainRealToMotor, 5, 13) * curGainMotorToReal;

	pCfg->curQKp = FixedPointUnsignedSaturate(pCfg->curQKp * pParams->curGainRealToMotor, 7, 10) * curGainMotorToReal;
	pCfg->curQKi = FixedPointUnsignedSaturate(pCfg->curQKi * pParams->curGainRealToMotor, 5, 13) * curGainMotorToReal;

	uint8_t allValuesEqual = FixedPointIsEqual(pCfg->curDKp, orig.curDKp, 10) && FixedPointIsEqual(pCfg->curDKi, orig.curDKi, 13) &&
			FixedPointIsEqual(pCfg->curQKp, orig.curQKp, 10) && FixedPointIsEqual(pCfg->curQKi, orig.curQKi, 13);

	return !allValuesEqual;
}

uint8_t McuMotorClampVelConfig(McuMotorVelConfig* pCfg, const McuMotorCtrlParams* pParams)
{
	McuMotorVelConfig orig = *pCfg;

	const float velGainMotorToReal = 1.0f / pParams->velGainRealToMotor;
	const float radPerSecToSpeedRaw = 1.0f / pParams->speedRawToRadPerSec;

	pCfg->velKp = FixedPointUnsignedSaturate(pCfg->velKp * pParams->velGainRealToMotor, 7, 10) * velGainMotorToReal;
	pCfg->velKi = FixedPointUnsignedSaturate(pCfg->velKi * pParams->velGainRealToMotor, 5, 13) * velGainMotorToReal;

	uint8_t allValuesEqual = FixedPointIsEqual(pCfg->velKp, orig.velKp, 10) && FixedPointIsEqual(pCfg->velKi, orig.velKi, 13);

	int32_t aj = (int32_t)(radPerSecToSpeedRaw * pCfg->velAntiJitter);
	if(aj < 0)
	{
		aj = 0;
		pCfg->velAntiJitter = 0.0f;
		allValuesEqual = 0;
	}
	else if(aj > 2047)
	{
		aj = 2047;
		pCfg->velAntiJitter = aj * pParams->speedRawToRadPerSec;
		allValuesEqual = 0;
	}

	return !allValuesEqual;
}

void McuMotorDebugRecord(McuMotor* pMot, uint8_t recordMode, uint8_t isOneShot)
{
	if(!pMot->pRecordData)
		return;

	chMtxLock(&pMot->measMutex);

	if(isOneShot)
	{
		pMot->recordMode = recordMode;
		pMot->recordCounter = 0;
	}
	else
	{
		if(pMot->recordMode != recordMode)
		{
			pMot->recordMode = recordMode;
			pMot->recordCounter = 0;
		}
	}

	pMot->recordOneShot = isOneShot;

	chMtxUnlock(&pMot->measMutex);
}

void McuMotorDebugRecordStop(McuMotor* pMot)
{
	McuMotorDebugRecord(pMot, MOTOR_EXCHANGE_MOSI_RECORD_NONE, 0);
}

static void flashProgram(McuMotor* pMot, uint8_t force)
{
	uint32_t originalBaudrate = pMot->pUart->baudrate;

	pMot->pUart->sendBreakAfterTx = 0;
	UARTFifoSetBaudrate(pMot->pUart, 115200);
	pMot->flashFuncResult = STBootloaderFlash(&pMot->bootloader, pMot->pFwProgram, pMot->fwSize, 0x08000000, 32*1024, force, pMot->motorId, &pMot->flashResult);
	UARTFifoSetBaudrate(pMot->pUart, originalBaudrate);
	pMot->pUart->sendBreakAfterTx = 1;

	pMot->doFwUpdate = 0;
}

static void receiveData(McuMotor* pMot)
{
	MotorExchangeMISO miso;
	uint16_t rxDatasize;
	uint16_t fifoStatus = 0;
	uint32_t rxTimestamp_us;

	msg_t waitResult = UARTFifoReadWait(pMot->pUart, TIME_MS2I(2));
	if(waitResult != MSG_OK)
		return;

	int16_t result;
	while(1)
	{
		DeviceProfilerBegin(&pMot->profiler);

		rxDatasize = sizeof(MotorExchangeMISO);
		result = UARTFifoRead(pMot->pUart, (uint8_t*)&miso, &rxDatasize, &fifoStatus, &rxTimestamp_us);
		if(result == UART_FIFO_ERROR_NO_DATA)
			break;
		if(result)
		{
			LogWarnC("read error", result | pMot->motorId << 8 | (uint32_t)fifoStatus << 16);
			continue;
		}
		if(rxDatasize < (sizeof(MotorExchangeMISO) - sizeof(miso.currentMeas)))
		{
			LogWarnC("Short read", rxDatasize | pMot->motorId << 8);
			continue;
		}

		// Update measurement
		chMtxLock(&pMot->measMutex);

		eventflags_t eventFlags = MCU_MOTOR_EVENT_DATA_RECEIVED;

		// process CPU load
		pMot->meas.timestamp_us = rxTimestamp_us;
		pMot->meas.cpuLoad = ((float)(miso.idleTicksNoLoad - miso.idleTicksActive))/((float)miso.idleTicksNoLoad)*100.0f;

		// process voltage, current, and temperature measurement
		pMot->meas.avgSupplyVoltage1ms_V = miso.avgSupplyVoltage * 0.001f;
		pMot->meas.avgTemperature1ms_degC = miso.avgTemperature * 0.1f;

		pMot->meas.avgCurrentUVW1ms_A[0] = miso.avgCurrentUVW[0] * 0.001f;
		pMot->meas.avgCurrentUVW1ms_A[1] = miso.avgCurrentUVW[1] * 0.001f;
		pMot->meas.avgCurrentUVW1ms_A[2] = miso.avgCurrentUVW[2] * 0.001f;
		pMot->meas.avgCurrentDQ1ms_A[0] = miso.avgCurrentDQ[0] * 0.001f;
		pMot->meas.avgCurrentDQ1ms_A[1] = miso.avgCurrentDQ[1] * 0.001f;
		pMot->meas.avgVoltageDQ1ms_V[0] = miso.avgVoltageDQ[0] * 0.001f;
		pMot->meas.avgVoltageDQ1ms_V[1] = miso.avgVoltageDQ[1] * 0.001f;
		pMot->meas.currentOffset_A = miso.currentOffset * 0.001f;

		pMot->meas.hallPos = miso.hallPos;
		pMot->meas.hallComTime = miso.hallComTime;
		pMot->meas.hallInvalidTransitions = miso.hallInvalidTransitions;
		pMot->meas.encPos = miso.encPos;
		pMot->meas.encDelta = miso.encDelta;
		pMot->meas.encCorrection = miso.encCorrection;
		pMot->meas.encTicks = miso.encTicks;
		pMot->meas.encPrescaler = miso.encPrescaler;
		pMot->meas.flags = miso.flags;

		if(pMot->meas.flags & MOTOR_EXCHANGE_MISO_FLAG_BREAK_ACTIVE)
		{
			if(!pMot->breakTriggered)
				pMot->totalBreaks++;

			pMot->recordMode = MOTOR_EXCHANGE_MOSI_RECORD_NONE;
			pMot->breakTriggered = 1;
			eventFlags |= MCU_MOTOR_EVENT_BREAK_DETECTED;
		}

		// process speed measurements
		if(miso.hallComTime == INT32_MAX || miso.hallComTime == 0)
		{
			pMot->meas.hallVelocity_radDs = 0.0f;
		}
		else
		{
			// convert hall commutation time to rad/s speed
			// tCom = 48e6/(rads * 1/(2*float(%pi))*pp*6);
			float numerator = 48e6f / (1.0f/(2.0f*(float)M_PI) * pMot->pMotorParams->pp * 6);
			pMot->meas.hallVelocity_radDs = numerator/miso.hallComTime;
		}

		pMot->meas.encoderVelocity_radDs = miso.encDelta * pMot->pCtrlParams->speedRawToRadPerSec;

		// record raw data if requested
		if(pMot->recordMode != MOTOR_EXCHANGE_MOSI_RECORD_NONE && pMot->pRecordData && miso.hasDebugData && rxDatasize >= sizeof(MotorExchangeMISO))
		{
			if(pMot->recordCounter + MOTOR_EXCHANGE_MISO_MAX_MEAS <= pMot->recordsMax)
			{
				memcpy(&pMot->pRecordData[pMot->recordCounter*2], miso.currentMeas, sizeof(int16_t)*MOTOR_EXCHANGE_MISO_MAX_MEAS*2);
				pMot->recordCounter += MOTOR_EXCHANGE_MISO_MAX_MEAS;
			}
			else
			{
				if(pMot->recordOneShot)
					pMot->recordMode = MOTOR_EXCHANGE_MOSI_RECORD_NONE;
				else
					pMot->recordCounter = 0;
			}
		}

		chMtxUnlock(&pMot->measMutex);

		chEvtBroadcastFlags(&pMot->eventSource, eventFlags);
	}
}

static void sendData(McuMotor* pMot)
{
	if(pMot->debugInhibitTx)
		return;

	const float radPerSecToSpeedRaw = 1.0f / pMot->pCtrlParams->speedRawToRadPerSec;

	MotorExchangeMOSI mosi;
	mosi._reserved = 0;

	chMtxLock(&pMot->outMutex);

	// Motor controller has some fixed built-in proportional gains to adjust fixed-point ranges.
	// Need to adjust gains here to reverse these fixed gains.
	mosi.curCtrlDKp = FixedPointSignedFromFloat(pMot->pFocConfig->curDKp * pMot->pCtrlParams->curGainRealToMotor, 10);
	mosi.curCtrlDKi = FixedPointSignedFromFloat(pMot->pFocConfig->curDKi * pMot->pCtrlParams->curGainRealToMotor, 13);
	mosi.curCtrlQKp = FixedPointSignedFromFloat(pMot->pFocConfig->curQKp * pMot->pCtrlParams->curGainRealToMotor, 10);
	mosi.curCtrlQKi = FixedPointSignedFromFloat(pMot->pFocConfig->curQKi * pMot->pCtrlParams->curGainRealToMotor, 13);
	mosi.velCtrlKp = FixedPointSignedFromFloat(pMot->pVelConfig->velKp * pMot->pCtrlParams->velGainRealToMotor, 10);
	mosi.velCtrlKi = FixedPointSignedFromFloat(pMot->pVelConfig->velKi * pMot->pCtrlParams->velGainRealToMotor, 13);
	mosi.resistance = FixedPointUnsignedFromFloatSaturate(pMot->pMotorParams->R, 4, 6);
	mosi.backEmfConstantInv = FixedPointUnsignedFromFloatSaturate(1.0f/pMot->pMotorParams->Ke, 10, 2);
	mosi.velCtrlAntiJitter = (int32_t)(radPerSecToSpeedRaw * pMot->pVelConfig->velAntiJitter);

	mosi.recordMode = pMot->recordMode;
	mosi.flags = 0;

	if(pMot->hallOnly)
		mosi.flags |= MOTOR_EXCHANGE_MOSI_FLAG_HALL_ONLY;

	mosi.motorMode = pMot->mode;

	switch(pMot->mode)
	{
		case MOTOR_MODE_VOLT_AB:
		case MOTOR_MODE_VOLT_DQ:
		{
			mosi.input[0] = (int32_t)(pMot->setVoltage_V[0]*1000.0f);;
			mosi.input[1] = (int32_t)(pMot->setVoltage_V[1]*1000.0f);
		}
		break;
		case MOTOR_MODE_CUR_D_VOLT_Q:
		{
			mosi.input[0] = (int32_t)(pMot->setCurrent_A[0]*1000.0f);;
			mosi.input[1] = (int32_t)(pMot->setVoltage_V[1]*1000.0f);
		}
		break;
		case MOTOR_MODE_CUR_DQ:
		{
			mosi.input[0] = (int16_t)(pMot->setCurrent_A[0]*1000.0f);
			mosi.input[1] = (int16_t)(pMot->setCurrent_A[1]*1000.0f);
		}
		break;
		case MOTOR_MODE_SPEED:
		{
			mosi.input[0] = (int16_t)(pMot->setCurrent_A[0]*1000.0f);
			mosi.input[1] = (int16_t)(pMot->setCurrent_A[1]*1000.0f);

			mosi.encDeltaSetpoint = (int16_t)(radPerSecToSpeedRaw * pMot->setVelocity_radDs);
		}
		break;
		default:
		{
			mosi.motorMode = MOTOR_MODE_OFF;
		}
		break;
	}

	pMot->txTimestamp_us = SysTimeUSec();

	chMtxUnlock(&pMot->outMutex);

	UARTFifoWrite(pMot->pUart, (uint8_t*)&mosi, sizeof(MotorExchangeMOSI), TIME_MS2I(20));

	DeviceProfilerEnd(&pMot->profiler);
}

static void motTask(void* pParam)
{
	McuMotor* pMot = (McuMotor*)pParam;

	chRegSetThreadName(pMot->taskName);

	while(1)
	{
		if(pMot->doFwUpdate == 1)
			flashProgram(pMot, 0);

		if(pMot->doFwUpdate == 2)
			flashProgram(pMot, 1);

		receiveData(pMot);
		sendData(pMot);
	}
}

SHELL_CMD(data, "Summary of current motor state");

SHELL_CMD_IMPL(data)
{
	(void)argc; (void)argv;
	McuMotor* pMot = (McuMotor*)pUser;
	McuMotorMeasurement* pMotor = &pMot->meas;

	printf("CPU: %.3f%%, Temp: %.1fC, vSupply: %.3fV\r\n", pMotor->cpuLoad, pMotor->avgTemperature1ms_degC, pMotor->avgSupplyVoltage1ms_V);
	printf("vDQ: %.3fV / %.3fV\r\n", pMotor->avgVoltageDQ1ms_V[0], pMotor->avgVoltageDQ1ms_V[1]);
	printf("iUVW: %.3fA / %.3fA / %.3fA\r\n", pMotor->avgCurrentUVW1ms_A[0], pMotor->avgCurrentUVW1ms_A[1], pMotor->avgCurrentUVW1ms_A[2]);
	printf("iOffset: %.3fA\r\n", pMotor->currentOffset_A);
	printf("iDQ: %.3fA / %.3fA\r\n", pMotor->avgCurrentDQ1ms_A[0], pMotor->avgCurrentDQ1ms_A[1]);
	printf("Hall Pos: %hu, comTime: %d, invTran: %hu\r\n", (uint16_t) pMotor->hallPos,
			pMotor->hallComTime, pMotor->hallInvalidTransitions);
	printf("EncPos: %hu, delta: %hd, cor: %hd, ticks: %hu, psc: %hu\r\n", pMotor->encPos,
			pMotor->encDelta, pMotor->encCorrection, pMotor->encTicks, pMotor->encPrescaler);
	printf("Speed: enc: %.4f, hall: %.4f\r\n", pMotor->encoderVelocity_radDs, pMotor->hallVelocity_radDs);
	printf("Flags: 0x%hX, breaks: %u\r\n", pMotor->flags, pMot->totalBreaks);
}

SHELL_CMD(update, "Force update of motor controller");

SHELL_CMD_IMPL(update)
{
	(void)argc; (void)argv;
	McuMotor* pMot = (McuMotor*)pUser;

	printf("Forcing motor %hu program update ", (uint16_t)pMot->motorId);

	McuMotorTriggerFwUpdate(pMot, 1);

	while(pMot->doFwUpdate)
	{
		printf(".");
		fflush(stdout);
		chThdSleepMilliseconds(100);
	}

	printf("\r\n    Req  Upd  Res     Size    Blocks  Time\r\n");
	printf("M%hu  %hu    %hu    0x%04hX  %-6u%4u     %ums\r\n", (uint16_t)pMot->motorId,
			(uint16_t)pMot->flashResult.updateRequired, (uint16_t)pMot->flashResult.updated,
			pMot->flashFuncResult, pMot->flashResult.programSize, pMot->flashResult.numBlocks,
			pMot->flashResult.timeUs/1000);
}

SHELL_CMD(hall, "Set motor controller to use hall sensors only",
	SHELL_ARG(enable, "0=hall and encoder, 1=hall only")
);

SHELL_CMD_IMPL(hall)
{
	(void)argc;
	McuMotor* pMot = (McuMotor*)pUser;
	int hallOnly = atoi(argv[1]);

	printf("Setting hall only on motor %hu to %d\r\n", (uint16_t)pMot->motorId, hallOnly);

	pMot->hallOnly = hallOnly;
}

SHELL_CMD(record, "Record 0.5s of raw motor data at full frequency",
	SHELL_ARG(mode, "1=CUR_AB, 2=CUR_DQ, 3=VOLT_AB_CUR_DQ, 4=SET_Q_ENC_DETLA, 5=CUR_Q_SET_MEAS")
);

SHELL_CMD_IMPL(record)
{
	(void)argc;
	McuMotor* pMot = (McuMotor*)pUser;
	int mode = atoi(argv[1]);

	printf("Recording 0.5s of data on motor %hu, mode: %d\r\n", (uint16_t)pMot->motorId, mode);
	McuMotorDebugRecord(pMot, mode, 1);
}

SHELL_CMD(record_step, "Perform step and record data",
	SHELL_ARG(mode, "Recording mode, see record command for details"),
	SHELL_ARG(axis, "D=cur_D [A], Q=cur_Q [A], A=angle [deg] + voltage [V], V=velocity [rad/s]"),
	SHELL_ARG(step_value, "Step target value")
);

SHELL_CMD_IMPL(record_step)
{
	McuMotor* pMot = (McuMotor*)pUser;

	int mode = atoi(argv[1]);
	char axis = *argv[2];
	float target = atof(argv[3]);
	float voltage = 0.0f;
	if(argc >= 5)
		voltage = atof(argv[4]);

	printf("Performing step on motor %hu, mode: %d, axis: %c, target: %.2f, voltage (opt): %.2f\r\n", (uint16_t)pMot->motorId, mode, axis, target, voltage);

	uint8_t invalidAxis = 0;

	McuMotorDebugRecord(pMot, mode, 1);

	chThdSleepMilliseconds(50);

	switch(axis)
	{
		case 'D': McuMotorSetCurrentDQ(pMot, target, 0.0f); break;
		case 'Q': McuMotorSetCurrentDQ(pMot, 0.0f, target); break;
		case 'A': McuMotorSetElectricalAngle(pMot, target* M_PI / 180.0f, voltage); break;
		case 'V': McuMotorSetVelocity(pMot, target, 0.0f, 0.0f); break;
		default: invalidAxis = 1; break;
	}

	chThdSleepMilliseconds(1000);

	McuMotorSetOff(pMot);

	if(invalidAxis)
	{
		fprintf(stderr, "Invalid axis: %c\r\n", axis);
	}
	else
	{
		printf("Step complete\r\n");
	}
}

SHELL_CMD(record_print, "Print content of debug recording buffer");

SHELL_CMD_IMPL(record_print)
{
	(void)argc; (void)argv;
	McuMotor* pMot = (McuMotor*)pUser;

	for(uint16_t i = 0; i < pMot->recordsMax; i++)
	{
		printf("% 6hd  % 6hd\r\n", pMot->pRecordData[i*2], pMot->pRecordData[i*2+1]);
		if(i % 20 == 0)
			chThdSleepMilliseconds(20);
	}
}

SHELL_CMD(kill, "Kill communication to motor controller (TX only)",
	SHELL_ARG(inhibit, "0=TX on, 1=TX off")
);

SHELL_CMD_IMPL(kill)
{
	(void)argc;
	McuMotor* pMot = (McuMotor*)pUser;
	int inhibit = atoi(argv[1]);

	printf("Set TX inhibit on motor %hu to %d\r\n", (uint16_t)pMot->motorId, inhibit);

	pMot->debugInhibitTx = inhibit;
}

SHELL_CMD(angle, "Set motor electrical angle",
	SHELL_ARG(angle, "Angle [deg]"),
	SHELL_ARG(voltage, "Voltage [V]")
);

SHELL_CMD_IMPL(angle)
{
	(void)argc;
	McuMotor* pMot = (McuMotor*)pUser;
	float angle = atof(argv[1]);
	float voltage = atof(argv[2]);

	printf("Motor %hu angle: %.3fdeg, %.3fV\r\n", (uint16_t)pMot->motorId, angle, voltage);
	McuMotorSetElectricalAngle(pMot, angle * M_PI / 180.0f, voltage);
}

SHELL_CMD(voldq, "Set motor voltage for D and Q axis",
	SHELL_ARG(voltageD, "Voltage D axis [V]"),
	SHELL_ARG(voltageQ, "Voltage Q axis [V]")
);

SHELL_CMD_IMPL(voldq)
{
	(void)argc;
	McuMotor* pMot = (McuMotor*)pUser;
	float volD = atof(argv[1]);
	float volQ = atof(argv[2]);

	printf("Motor %hu voltages: %.3fV (D), %.3fV (Q)\r\n", (uint16_t)pMot->motorId, volD, volQ);
	McuMotorSetVoltageDQ(pMot, volD, volQ);
}

SHELL_CMD(vel, "Set motor velocity",
	SHELL_ARG(velocity, "Velocity [rad/s]")
);

SHELL_CMD_IMPL(vel)
{
	(void)argc;
	McuMotor* pMot = (McuMotor*)pUser;
	float vel = atof(argv[1]);

	printf("Motor %hu velocity: %.3frad/s\r\n", (uint16_t)pMot->motorId, vel);
	McuMotorSetVelocity(pMot, vel, 0.0f, 0.0f);
}

SHELL_CMD(curdq, "Set motor current for D and Q axis",
	SHELL_ARG(voltageD, "Current D axis [A]"),
	SHELL_ARG(voltageQ, "Current Q axis [A]")
);

SHELL_CMD_IMPL(curdq)
{
	(void)argc;
	McuMotor* pMot = (McuMotor*)pUser;
	float curD = atof(argv[1]);
	float curQ = atof(argv[2]);

	printf("Motor %hu current: %.3fA (D), %.3fA (Q)\r\n", (uint16_t)pMot->motorId, curD, curQ);
	McuMotorSetCurrentDQ(pMot, curD, curQ);
}

SHELL_CMD(off, "Set motor off");

SHELL_CMD_IMPL(off)
{
	(void)argc; (void)argv;
	McuMotor* pMot = (McuMotor*)pUser;

	printf("Motor %hu switched off\r\n", (uint16_t)pMot->motorId);

	McuMotorSetOff(pMot);
}

SHELL_CMD(turn, "Turn motor and report hall sensor changes",
	SHELL_ARG(voltage, "Voltage used for turning [V]")
);

SHELL_CMD_IMPL(turn)
{
	(void)argc;
	McuMotor* pMot = (McuMotor*)pUser;
	float vol = atof(argv[1]);

	printf("Turning motor %hu at %.3fV...\r\n", (uint16_t)pMot->motorId, vol);

	uint8_t lastHall = pMot->meas.hallPos;
	uint16_t lastEnc = pMot->meas.encPos;
	float lastAngle = 0;

	for(uint8_t turn = 0; turn < 8; turn++)
	{
		for(float angle = 0.0f; angle < 360.0f; angle += 0.1f)
		{
			McuMotorSetElectricalAngle(pMot, angle * M_PI / 180.0f, vol);

			chThdSleep(1);

			if(pMot->meas.hallPos != lastHall)
			{
				printf("Hall %hu => %hu @ %.2f (%d, %.2f)\r\n", (uint16_t) lastHall, (uint16_t) pMot->meas.hallPos, angle,
						(int32_t) pMot->meas.encPos - (int32_t) lastEnc, angle - lastAngle);
				lastHall = pMot->meas.hallPos;
				lastEnc = pMot->meas.encPos;
				lastAngle = angle;
			}
		}
	}

	printf("reverse\r\n");

	for(uint8_t turn = 0; turn < 8; turn++)
	{
		for(float angle = 360.0f; angle > 0.0f; angle += -0.1f)
		{
			McuMotorSetElectricalAngle(pMot, angle * M_PI / 180.0f, vol);

			chThdSleep(1);

			if(pMot->meas.hallPos != lastHall)
			{
				printf("Hall %hu => %hu @ %.2f (%d, %.2f)\r\n", (uint16_t) lastHall, (uint16_t) pMot->meas.hallPos, angle,
						(int32_t) pMot->meas.encPos - (int32_t) lastEnc, angle - lastAngle);
				lastHall = pMot->meas.hallPos;
				lastEnc = pMot->meas.encPos;
				lastAngle = angle;
			}
		}
	}

	printf("done\r\n");

	McuMotorSetOff(pMot);
}

static void registerShellCommands(ShellCmdHandler* pHandler)
{
	ShellCmdAdd(pHandler, data_command);
	ShellCmdAdd(pHandler, off_command);
	ShellCmdAdd(pHandler, voldq_command);
	ShellCmdAdd(pHandler, angle_command);
	ShellCmdAdd(pHandler, curdq_command);
	ShellCmdAdd(pHandler, vel_command);
	ShellCmdAdd(pHandler, turn_command);
	ShellCmdAdd(pHandler, hall_command);
	ShellCmdAdd(pHandler, kill_command);
	ShellCmdAdd(pHandler, update_command);
	ShellCmdAdd(pHandler, record_command);
	ShellCmdAdd(pHandler, record_step_command);
	ShellCmdAdd(pHandler, record_print_command);
}
