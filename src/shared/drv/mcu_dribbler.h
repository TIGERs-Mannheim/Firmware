#pragma once

#include "hal/uart_fifo.h"
#include "util/st_bootloader.h"
#include "util/device_profiler.h"
#include "util/shell_cmd.h"
#include "arm_math.h"

typedef struct _McuDribblerMeasurement
{
	uint32_t timestamp_us;

	float dribblerTempRaw_V;
	float dribblerTemp_degC;

	float barrierOn_V;
	float barrierOff_V;

	float estimatedBallPos_cm[2];
	uint8_t isEstimatedBallPosValid;

	float cpuLoad_perc;
} McuDribblerMeasurement;

typedef struct _McuDribbler
{
	UARTFifo* pUart;

	STBootloader bootloader;
	STBootloaderFlashResult flashResult;
	int16_t flashFuncResult;
	const uint8_t* pFwProgram;
	uint32_t fwSize;

	uint8_t doFwUpdate;

	float vIrOff[6]; // [V]
	float vLateral[4][5]; // Output of all scans of the IR-Sensor array [V]

	McuDribblerMeasurement meas;

	DeviceProfiler profiler;

	ShellCmdHandler cmdHandler;

	THD_WORKING_AREA(waTask, 512);
	thread_t* pTask;

	mutex_t measMutex;

	event_source_t eventSource;

	// Constant Values for polynomial fitting
	arm_matrix_instance_f32 matIrPolyfitA;
	arm_matrix_instance_f32 matIrPolyfitAinv;
	float pIrPolyfitA[5*3];
	float pIrPolyfitAinv[3*5];
} McuDribbler;

void McuDribblerInit(McuDribbler* pDrib, UARTFifo* pUart, GPIOPinInterface* pRstPin, GPIOPinInterface* pBootPin, const uint8_t* pFwProgram, uint32_t fwSize, tprio_t prio);
void McuDribblerTriggerFwUpdate(McuDribbler* pDrib, uint8_t force);
void McuDribblerGet(McuDribbler* pDrib, McuDribblerMeasurement* pMeas);
