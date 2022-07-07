/*
 * adc.h
 *
 *  Created on: 18.02.2019
 *      Author: AndreR
 */

#pragma once

#include <stdint.h>

#define ADC_NUM_SAMPLES_PER_MS 20

typedef void(*ADCControlCallback)();

typedef struct _ADCSample
{
	int16_t iMotor;	// [mA]
	int16_t iMotor2; // [mA]
} ADCSample;

typedef struct _ADCData
{
	uint16_t dmaData[7];

	ADCSample sampleBuf[2][ADC_NUM_SAMPLES_PER_MS];

	ADCSample* pSampleCurrent;
	ADCSample* pSampleBufEnd;

	ADCSample* volatile pFinishedSampleBuf;
	volatile uint8_t trigger1ms;

	uint32_t voltageSum;
	int32_t tempSum;
	int32_t currentUVWSum[3];
	int32_t currentDQSum[2];

	int32_t temperatureFactor;
	int32_t temperatureOffset;

	volatile uint32_t avgVoltage_Q16_0; // [mV]
	volatile int32_t avgTemperature_S15_0; // [d°C]
	volatile int32_t avgCurrentUVW_S14_0[3]; // [mA]
	volatile int32_t avgCurrentDQ_S14_0[2]; // [mA]

	volatile int32_t tempAdc_S12_0;
	volatile uint32_t volAdc_U12_0;

	void(*pNextAdcFunc)();

	ADCControlCallback controlCallback;

	uint32_t logPrescaler;
	uint32_t logCounter;
} ADCData;

extern ADCData adc;

void ADCInit(ADCControlCallback controlFunc, uint32_t numCurSamples, uint32_t logPrescaler);
void ADCWaitFor1MsTrigger();
void ADCUpdate();
