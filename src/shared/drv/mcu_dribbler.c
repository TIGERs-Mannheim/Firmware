#include "mcu_dribbler.h"
#include "math/arm_mat_util_f32.h"
#include "util/log.h"
#include "log_msgs.h"
#include "hal/sys_time.h"

#define IR_RAW_TO_VOLTAGE (1.0f/7.0f * 8.056640625e-4f)

static void dribTask(void* pParam);
static void calcBallPosition(McuDribbler* pDrib, float* pColSums, float* pPosition, uint8_t* pBallDetected);
static void registerShellCommands(ShellCmdHandler* pHandler);

static const uint8_t lateralReorderTable[] = { 4, 3, 0, 1, 2 };

void McuDribblerInit(McuDribbler* pDrib, UARTFifo* pUart, GPIOPinInterface* pRstPin, GPIOPinInterface* pBootPin, const uint8_t* pFwProgram, uint32_t fwSize, tprio_t prio)
{
	chMtxObjectInit(&pDrib->measMutex);
	chEvtObjectInit(&pDrib->eventSource);

	ShellCmdHandlerInit(&pDrib->cmdHandler, pDrib);
	registerShellCommands(&pDrib->cmdHandler);

	STBootloaderInit(&pDrib->bootloader, pUart, pRstPin, pBootPin);

	pDrib->pUart = pUart;
	pDrib->pFwProgram = pFwProgram;
	pDrib->fwSize = fwSize;
	pDrib->doFwUpdate = 1;

	float receiverXPositions[] = {-25.2f, -12.6f, 0.0f, 12.6f, 25.2f};

	// Setup Matrix A for fitting polynom to IR-Array data
	arm_mat_init_f32(&pDrib->matIrPolyfitA, 5, 3, pDrib->pIrPolyfitA);
	for(uint8_t row = 0; row < 5; row++)
	{
		MAT_ELEMENT(pDrib->matIrPolyfitA, row, 0) = receiverXPositions[row]*receiverXPositions[row];
		MAT_ELEMENT(pDrib->matIrPolyfitA, row, 1) = receiverXPositions[row];
		MAT_ELEMENT(pDrib->matIrPolyfitA, row, 2) = 1;
	}

	arm_mat_init_f32(&pDrib->matIrPolyfitAinv, 3, 5, pDrib->pIrPolyfitAinv);
	arm_mat_pinv(&pDrib->matIrPolyfitA, &pDrib->matIrPolyfitAinv);

	DeviceProfilerInit(&pDrib->profiler, 1.0f);

	pDrib->pTask = chThdCreateStatic(pDrib->waTask, sizeof(pDrib->waTask), prio, &dribTask, pDrib);
}

void McuDribblerGet(McuDribbler* pDrib, McuDribblerMeasurement* pMeas)
{
	chMtxLock(&pDrib->measMutex);
	memcpy(pMeas, &pDrib->meas, sizeof(pDrib->meas));
	chMtxUnlock(&pDrib->measMutex);
}

void McuDribblerTriggerFwUpdate(McuDribbler* pDrib, uint8_t force)
{
	pDrib->doFwUpdate = 1 + force;
}

static void flashProgram(McuDribbler* pDrib, uint8_t force)
{
	uint32_t originalBaudrate = pDrib->pUart->baudrate;

	pDrib->pUart->sendBreakAfterTx = 0;
	UARTFifoSetBaudrate(pDrib->pUart, 115200);
	pDrib->flashFuncResult = STBootloaderFlash(&pDrib->bootloader, pDrib->pFwProgram, pDrib->fwSize, 0x08000000, 32*1024, force, 0, &pDrib->flashResult);
	UARTFifoSetBaudrate(pDrib->pUart, originalBaudrate);
	pDrib->pUart->sendBreakAfterTx = 1;

	pDrib->doFwUpdate = 0;
}

static void update(McuDribbler* pDrib)
{
	IrExchangeMISO miso;
	uint16_t rxDatasize;

	msg_t waitResult = UARTFifoReadWait(pDrib->pUart, TIME_MS2I(20));
	if(waitResult != MSG_OK)
	{
		LogWarn("rx timeout");
		return;
	}

	int16_t result;
	while(1)
	{
		DeviceProfilerBegin(&pDrib->profiler);

		rxDatasize = sizeof(IrExchangeMISO);
		result = UARTFifoRead(pDrib->pUart, (uint8_t*)&miso, &rxDatasize, 0, 0);
		if(result == UART_FIFO_ERROR_NO_DATA)
		{
			break;
		}
		else if(result)
		{
			LogWarnC("read error", result);
			continue;
		}
		else if(rxDatasize < sizeof(IrExchangeMISO))
		{
			LogWarnC("Short read", rxDatasize);
			continue;
		}

		uint8_t* pIrData = (uint8_t*)miso.channels;
		uint8_t xor = 0;
		for(uint16_t i = 0; i < sizeof(miso.channels)+sizeof(miso.config)+sizeof(miso.xorChecksum); i++)
			xor ^= pIrData[i];

		if(xor != 0)
		{
			LogWarnC("Checksum error", xor);
			continue;
		}

		uint8_t channel = miso.config;
		if(channel > 4)
		{
			LogWarnC("Invalid IR channel", channel);
			continue;
		}

		DeviceProfilerEnd(&pDrib->profiler);

		if(miso.config == 0)
		{
			for(uint8_t i = 0; i < 6; i++)
				pDrib->vIrOff[i] = miso.channels[i+1] * IR_RAW_TO_VOLTAGE;
		}
		else
		{
			for(uint8_t i = 0; i < 5; i++)
			{
				pDrib->vLateral[miso.config-1][lateralReorderTable[i]] = miso.channels[i+2] * IR_RAW_TO_VOLTAGE - pDrib->vIrOff[i+1];
			}
		}

		// Estimate ball position
		float irColSums[5] = { 0 };
		for(uint8_t row = 0; row < 4; row++)
		{
			for(uint8_t col = 0; col < 5; col++)
			{
				irColSums[col] += pDrib->vLateral[row][col];
			}
		}

		float estimatedBallPos_cm[2] = { 0, 0 };
		uint8_t isEstimatedBallPosValid;

		calcBallPosition(pDrib, irColSums, estimatedBallPos_cm, &isEstimatedBallPosValid);

		// Update measurement
		chMtxLock(&pDrib->measMutex);

		pDrib->meas.timestamp_us = SysTimeUSec();

		pDrib->meas.barrierOn_V = miso.channels[8] * IR_RAW_TO_VOLTAGE;
		pDrib->meas.barrierOff_V = pDrib->vIrOff[0];

		pDrib->meas.dribblerTempRaw_V = (miso.channels[0] + miso.channels[7]) * IR_RAW_TO_VOLTAGE * 0.5f;
		pDrib->meas.dribblerTemp_degC = (pDrib->meas.dribblerTempRaw_V - 0.4f) * 51.28205128205128f - 5.0f; // MCP9701A sensor, poor calibration with offset of -5 deg C

		pDrib->meas.estimatedBallPos_cm[0] = estimatedBallPos_cm[0];
		pDrib->meas.estimatedBallPos_cm[1] = estimatedBallPos_cm[1];
		pDrib->meas.isEstimatedBallPosValid = isEstimatedBallPosValid;

		pDrib->meas.cpuLoad_perc = ((float)(miso.idleTicksNoLoad - miso.idleTicksActive))/((float)miso.idleTicksNoLoad)*100.0f;

		chMtxUnlock(&pDrib->measMutex);

		chEvtBroadcast(&pDrib->eventSource);
	}
}

static float evalPoly2(float x, const float* pCoeffs)
{
	return pCoeffs[0]*x*x + pCoeffs[1]*x + pCoeffs[2];
}

static void calcBallPosition(McuDribbler* pDrib, float* pColSums, float* pPosition, uint8_t* pBallDetected)
{
	const float xLimit = 25.2f;

	const arm_matrix_instance_f32 vecColSums = {5, 1, pColSums };

	// fit a function of type y = a*x^2 + b*x + c using least squares method
	arm_matrix_instance_f32 vecPolyCoeffs = {3, 1, (float[3]){} };
	arm_mat_mult_f32(&pDrib->matIrPolyfitAinv, &vecColSums, &vecPolyCoeffs);

	// coefficients for faster access
	const float a = MAT_ELEMENT(vecPolyCoeffs, 0, 0);
	const float b = MAT_ELEMENT(vecPolyCoeffs, 1, 0);

	// compute left/right limit values and extreme
	float leftVal = evalPoly2(-xLimit, vecPolyCoeffs.pData);
	float rightVal = evalPoly2(xLimit, vecPolyCoeffs.pData);

	float maxValX = -b/(2.0f*a);
	float maxValXLimited = fminf(xLimit, fmaxf(-xLimit, maxValX));

	float maxValYLimited = evalPoly2(maxValXLimited, vecPolyCoeffs.pData);

	float minVal = fminf(fminf(leftVal, rightVal), maxValYLimited);
	float maxVal = fmaxf(fmaxf(leftVal, rightVal), maxValYLimited);

	float minMaxDiff = maxVal - minVal;

	// Is the ball visible at all? Check differences among IR values.
	if(minMaxDiff < 0.2f)
	{
		*pBallDetected = 0;
		return;
	}

	*pBallDetected = 1;

	// compute X position with very obscure function
	float xPos = arm_atan2_f32(maxValXLimited, maxValYLimited*20.0f)*180.0f/(float)M_PI*1.0f/15.0f;

	// Y position is even more strange
	// start with normalizing intensity over X range
	const float aNorm[] = { 1.175f, 0.4623f };
	const float bNorm[] = { 0.2305f, 1.458f };

	float normalizationDenom = aNorm[0]*cosf(bNorm[0]*xPos) - aNorm[1]*cosf(bNorm[1]*xPos);

	float normalizedIntensity = minMaxDiff/normalizationDenom;

	// now map intensity to distance
	const float distanceMapPoly1[] = { -1.799f, 3.305f };

	float yPos = distanceMapPoly1[0]*normalizedIntensity + distanceMapPoly1[1];

	pPosition[0] = xPos;
	pPosition[1] = yPos;
}

static void dribTask(void* pParam)
{
	McuDribbler* pDrib = (McuDribbler*)pParam;

	chRegSetThreadName("MCU_DRIBBLER");

	while(1)
	{
		if(pDrib->doFwUpdate == 1)
			flashProgram(pDrib, 0);

		if(pDrib->doFwUpdate == 2)
			flashProgram(pDrib, 1);

		update(pDrib);
	}
}

SHELL_CMD(data, "Summary of current IR controller state");

SHELL_CMD_IMPL(data)
{
	(void)argc; (void)argv;
	McuDribbler* pDrib = (McuDribbler*)pUser;

	McuDribblerMeasurement ir;
	McuDribblerGet(pDrib, &ir);

	printf("Update Rate: %.2fHz\r\n", pDrib->profiler.updateRate_Hz);
	printf("CPU Load: %.2f%%\r\n", ir.cpuLoad_perc);
	printf("vBarrier: %f\r\n", ir.barrierOn_V);
	printf("vDribblerTemp: %f\r\n", ir.dribblerTempRaw_V);
	printf("tDribbler: %f\r\n", ir.dribblerTemp_degC);

	float vSum[5] = { 0 };

	printf("-: %.3f   %.3f   %.3f   %.3f   %.3f\r\n", pDrib->vIrOff[1],
			pDrib->vIrOff[2], pDrib->vIrOff[3], pDrib->vIrOff[4], pDrib->vIrOff[5]);

	for(uint16_t i = 0; i < 4; i++)
	{
		printf("%hu: %.3f %c %.3f %c %.3f %c %.3f %c %.3f\r\n", i,
				pDrib->vLateral[i][0], i == 0 ? '*' : ' ',
				pDrib->vLateral[i][1], i == 1 ? '*' : ' ',
				pDrib->vLateral[i][2], i == 2 ? '*' : ' ',
				pDrib->vLateral[i][3], i == 3 ? '*' : ' ',
				pDrib->vLateral[i][4]);

		for(uint8_t j = 0; j < 5; j++)
			vSum[j] += pDrib->vLateral[i][j];
	}
	printf("S: %.3f * %.3f * %.3f * %.3f * %.3f\r\n", vSum[0],
			vSum[1], vSum[2], vSum[3], vSum[4]);
	printf("Estimated Ball Position: [ %07.3f / %07.3f ]\r\n", ir.estimatedBallPos_cm[0], ir.estimatedBallPos_cm[1]);
}

SHELL_CMD(update, "Force update of IR controller");

SHELL_CMD_IMPL(update)
{
	(void)argc; (void)argv;
	McuDribbler* pDrib = (McuDribbler*)pUser;

	printf("Forcing IR Program Update ");

	McuDribblerTriggerFwUpdate(pDrib, 1);

	while(pDrib->doFwUpdate)
	{
		printf(".");
		fflush(stdout);
		chThdSleepMilliseconds(100);
	}

	printf("\r\n    Req  Upd  Res     Size    Blocks  Time\r\n");
	printf("IR  %hu    %hu    0x%04hX  %-6u%4u     %ums\r\n",
			(uint16_t)pDrib->flashResult.updateRequired, (uint16_t)pDrib->flashResult.updated,
			pDrib->flashFuncResult, pDrib->flashResult.programSize, pDrib->flashResult.numBlocks,
			pDrib->flashResult.timeUs/1000);
}

static void registerShellCommands(ShellCmdHandler* pHandler)
{
	ShellCmdAdd(pHandler, data_command);
	ShellCmdAdd(pHandler, update_command);
}
