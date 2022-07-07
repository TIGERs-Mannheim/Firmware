/*
 * ir.h
 *
 *  Created on: 12.01.2019
 *      Author: AndreR
 */

#pragma once

#include "util/uart_fifo.h"
#include "util/st_bootloader.h"
#include "log_msgs.h"
#include "arm_math.h"

typedef struct _IR
{
	UARTFifo uart;
	STBootloader bootloader;
	STBootloaderFlashResult flashResult;
	int16_t flashFuncResult;

	float vBarrier; // [V]
	IRData irData;
	mutex_t mtxUart;
	float cpuLoad; // [%]

	float vIrOff[6]; // [V]

	float vDribblerTemp; // [V]
	float dribblerTemperature;
	uint8_t dribblerOverheated;

	// Constant Values for polynomial fitting
	arm_matrix_instance_f32 matIrPolyfitA;
	arm_matrix_instance_f32 matIrPolyfitAinv;
	float pIrPolyfitA[5*3];
	float pIrPolyfitAinv[3*5];
} IR;

extern IR ir;

void IrInit();
void IrFlashProgram(uint8_t force);
void IrTask(void* params);
void IrLogData();
