/*
 * main.h
 *
 *  Created on: 20.08.2020
 *      Author: AndreR
 */

#pragma once

#include "log_msgs.h"
#include "pi_ctrl_s12.h"

#define TOGGLE_PRIMARY_LED() GPIOB->ODR ^= GPIO_ODR_7

extern void (*irqTableCore[])();
extern void (*irqTable[])();

typedef struct _Data
{
	uint32_t timeMs;

	struct _performance
	{
		uint32_t idleIncrements;
		uint32_t idleCounter;
	} performance;

	struct _sensors
	{
		struct _encoder
		{
			uint8_t installed;
			uint16_t position;
			int16_t delta; // within 1ms
			int32_t correction; // full circle -32768 to 32767
		} encoder;

		struct _hall
		{
			uint8_t position;
			int32_t commutationTime; // in timer clock cycles, negative for reverse dir
			uint16_t invalidTransitions;
		} hall;

		struct _adc
		{
			int32_t vSupply_Q16_0; // [mV]
			int32_t vSupplyReciprocal_Q0_31;

			int32_t currentUVW_S15_15[3];
			int32_t currentDQ_S16_0[2];
		} adc;
	} sensors;

	MotorExchangeMOSI command;

	struct _comm
	{
		uint32_t lastCommandTimeMs;
		uint8_t timeout;
	} comm;

	struct _motor
	{
		uint8_t mode;

		int32_t Uab[2];
		int32_t Udq[2];

		volatile uint8_t overcurrentDetected;
	} motor;

	struct _ctrl
	{
		uint8_t hallOnly;

		PICtrlS12 currentD;
		PICtrlS12 currentQ;

		PICtrlS12 speed;

		int32_t avgVoltageDQ[2]; // final voltage set by controllers
	} ctrl;
} Data;

extern Data data;
