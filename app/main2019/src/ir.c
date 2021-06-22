/*
 * ir.c
 *
 *  Created on: 12.01.2019
 *      Author: AndreR
 */

#include "ir.h"

#include "hal/port_ex.h"
#include "util/init_hal.h"
#include "util/log.h"
#include "util/console.h"
#include "util/log_file.h"
#include "util/sys_time.h"
#include "util/arm_mat_util_f32.h"
#include "kicker.h"
#include "log_msgs.h"
#include "struct_ids.h"
#include "constants.h"
#include <string.h>
#include "generated/ir_distance_prediction.h"

#define IR_RAW_TO_VOLTAGE (1.0f/7.0f * 8.056640625e-4f)

IR ir;

static const uint8_t lateralReorderTable[] = { 4, 3, 0, 1, 2 };

void UART8_IRQHandler()
{
	CH_IRQ_PROLOGUE();

	UARTFifoIRQ(&ir.uart);

	CH_IRQ_EPILOGUE();
}

static void startBootloader(uint8_t target)
{
	PortExBootSet(target, 1);
	PortExResetPulse(target);
	PortExBootSet(target, 0);
}

void IrInit()
{
	chMtxObjectInit(&ir.mtxUart);

	ir.irData.header.type = SID_IR_DATA;
	ir.irData.header.length = sizeof( IRData);

	// init port
	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;
	gpioInit.alternate = 8;
	gpioInit.pupd = GPIO_PUPD_UP;
	GPIOInit(GPIOE, GPIO_PIN_0 | GPIO_PIN_1, &gpioInit);

	// UART8
	RCC->APB1LENR |= RCC_APB1LENR_UART8EN;
	__DSB();

	UARTFifoInit(&ir.uart, UART8, 2000000, systemClockInfo.APB1PeriphClk, 1);

	NVICEnableIRQ(UART8_IRQn, IRQL_UART8);

	STBootloaderInit(&ir.bootloader, &ir.uart, &startBootloader, PORT_EX_TARGET_IR);

	float receiverXPositions[] = {-25.2f, -12.6f, 0.0f, 12.6f, 25.2f};

	// Setup Matrix A for fitting polynom to IR-Array data
	arm_mat_init_f32(&ir.matIrPolyfitA, 5, 3, ir.pIrPolyfitA);
	for(uint8_t row = 0; row < 5; row++)
	{
		MAT_ELEMENT(ir.matIrPolyfitA, row, 0) = receiverXPositions[row]*receiverXPositions[row];
		MAT_ELEMENT(ir.matIrPolyfitA, row, 1) = receiverXPositions[row];
		MAT_ELEMENT(ir.matIrPolyfitA, row, 2) = 1;
	}

	arm_mat_init_f32(&ir.matIrPolyfitAinv, 3, 5, ir.pIrPolyfitAinv);
	arm_mat_pinv(&ir.matIrPolyfitA, &ir.matIrPolyfitAinv);
}

void IrFlashProgram(uint8_t force)
{
	chMtxLock(&ir.mtxUart);

	uint32_t originalBaudrate = ir.uart.baudrate;
	UARTFifoSetBaudrate(&ir.uart, 115200);
	ir.flashFuncResult = STBootloaderFlash(&ir.bootloader ,(uint8_t*)0x081A0000, 32*1024, 0x08000000, force, 0, &ir.flashResult);
	UARTFifoSetBaudrate(&ir.uart, originalBaudrate);

	chMtxUnlock(&ir.mtxUart);

	ConsolePrint("IR  %hu    %hu    0x%04hX  %-6u%4u     %ums\r\n",
			(uint16_t)ir.flashResult.updateRequired, (uint16_t)ir.flashResult.updated,
			ir.flashFuncResult, ir.flashResult.programSize, ir.flashResult.numBlocks,
			ir.flashResult.timeUs/1000);
}

static float evalPoly2(float x, const float* pCoeffs)
{
	return pCoeffs[0]*x*x + pCoeffs[1]*x + pCoeffs[2];
}

void IrCalcBallPosition(float* pColSums, float* pPosition, uint8_t* pBallDetected)
{
	const float xLimit = 25.2f;

	const arm_matrix_instance_f32 vecColSums = {5, 1, pColSums };

	// fit a function of type y = a*x^2 + b*x + c using least squares method
	arm_matrix_instance_f32 vecPolyCoeffs = {3, 1, (float[3]){} };
	arm_mat_mult_f32(&ir.matIrPolyfitAinv, &vecColSums, &vecPolyCoeffs);

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

void IrTask(void* params)
{
	(void)params;

	chRegSetThreadName("IR");

	chThdSleepMilliseconds(200);

	IrFlashProgram(0);

	IrExchangeMISO miso;
	uint16_t rxDatasize;
	uint16_t fifoStatus;

	while(1)
	{
		chMtxLock(&ir.mtxUart);
		msg_t waitResult = UARTFifoReadWait(&ir.uart, MS2ST(20));
		chMtxUnlock(&ir.mtxUart);
		if(waitResult != MSG_OK)
		{
			LogWarn("rx timeout");
			continue;
		}

		int16_t result;
		while(1)
		{
			rxDatasize = sizeof(IrExchangeMISO);
			chMtxLock(&ir.mtxUart);
			result = UARTFifoRead(&ir.uart, (uint8_t*)&miso, &rxDatasize, &fifoStatus, 0);
			chMtxUnlock(&ir.mtxUart);
			if(result == UART_FIFO_ERROR_NO_DATA)
				break;
			if(result)
			{
				LogWarnC("read error", result);
				continue;
			}
			if(rxDatasize < sizeof(IrExchangeMISO))
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

			if(miso.config == 0)
			{
				for(uint8_t i = 0; i < 6; i++)
					ir.vIrOff[i] = miso.channels[i+1] * IR_RAW_TO_VOLTAGE;
			}
			else
			{
				for(uint8_t i = 0; i < 5; i++)
				{
					ir.irData.vLateral[miso.config-1][lateralReorderTable[i]] = miso.channels[i+2] * IR_RAW_TO_VOLTAGE - ir.vIrOff[i+1];
				}
			}

			ir.vBarrier = miso.channels[8] * IR_RAW_TO_VOLTAGE;

			ir.vDribblerTemp = (miso.channels[0] + miso.channels[7]) * IR_RAW_TO_VOLTAGE * 0.5f;
			ir.dribblerTemperature = (ir.vDribblerTemp - 0.4f) * 51.28205128205128f - 5.0f; // MCP9701A sensor, poor calibration with offset of -5 deg C

			if(ir.dribblerTemperature < 70.0f)
				ir.dribblerOverheated = 0;

			if(ir.dribblerTemperature > 80.0f)
				ir.dribblerOverheated = 1;

			KickerIrUpdate(ir.vBarrier, ir.vIrOff[0]);

			// Estimate ball position
			float irColSums[5] = { 0 };
			for(uint8_t row = 0; row < 4; row++)
			{
				for(uint8_t col = 0; col < 5; col++)
				{
					irColSums[col] += ir.irData.vLateral[row][col];
				}
			}

			IrCalcBallPosition(irColSums, ir.irData.estimatedBallPosition, &ir.irData.ballDetected);
			ir.irData.header.timestamp = SysTimeUSec();

			ir.cpuLoad = ((float)(miso.idleTicksNoLoad - miso.idleTicksActive))/((float)miso.idleTicksNoLoad)*100.0f;
		}
	}
}

void IrLogData()
{
	LogFileWrite(&ir.irData);
}
