/*
 * test_kicker.c
 *
 *  Created on: 21.05.2022
 *      Author: AndreR
 */

#include "test_kicker.h"
#include "test.h"
#include "gui/test_kicker.h"
#include "kicker.h"
#include "hal/led.h"

#define TEST_KICKER_TIMEOUT 10000 // in ms

static uint8_t dischargeKicker()
{
	TestKickerProgress("Discharging");
	KickerAutoDischarge();

	uint32_t time = 0;
	while(kicker.autoDischarge && ++time < TEST_KICKER_TIMEOUT)
		chThdSleepMilliseconds(1);

	if(time == TEST_KICKER_TIMEOUT)
	{
		TestKickerProgress("Discharging failed");
		TestModeExit();
		return 0;
	}

	return 1;
}

static void doReasoningOnKicker(TestKickerResult* pResult)
{
	if(pResult->chargingSpeed < 15.0f)
		pResult->reasoning.charge = TEST_REASONING_RESULT_BAD;
	else if(pResult->chargingSpeed < 30.0f || pResult->chargingSpeed > 70.0f)
		pResult->reasoning.charge = TEST_REASONING_RESULT_WARNING;
	else
		pResult->reasoning.charge = TEST_REASONING_RESULT_OK;

	if(pResult->straightVoltageDrop < 5.0f)
		pResult->reasoning.straight = TEST_REASONING_RESULT_BAD;
	else if(pResult->straightVoltageDrop < 10.0f || pResult->straightVoltageDrop > 30.0f)
		pResult->reasoning.straight = TEST_REASONING_RESULT_WARNING;
	else
		pResult->reasoning.straight = TEST_REASONING_RESULT_OK;

	if(pResult->chipVoltageDrop < 5.0f)
		pResult->reasoning.chip = TEST_REASONING_RESULT_BAD;
	else if(pResult->chipVoltageDrop < 10.0f || pResult->chipVoltageDrop > 30.0f)
		pResult->reasoning.chip = TEST_REASONING_RESULT_WARNING;
	else
		pResult->reasoning.chip = TEST_REASONING_RESULT_OK;
}

void TestKicker(TestKickerResult* pResult)
{
	TestModeStartup();
	pResult->chargingSpeed = 0.0f;
	pResult->straightVoltageDrop = 0.0f;
	pResult->chipVoltageDrop = 0.0f;

	LEDLeftSet(0.0f, 0.0f, 0.01f, 0.0f);
	LEDRightSet(0.0f, 0.0f, 0.01f, 0.0f);

	if(!dischargeKicker())
		return;

	LEDLeftSet(0.01f, 0.0f, 0.0f, 0.0f);
	LEDRightSet(0.01f, 0.0f, 0.0f, 0.0f);

	float maxVoltageBackup = kicker.config.maxVoltage;
	kicker.config.maxVoltage = 220.0f;
	float startVoltage = kicker.vCap;
	TestKickerProgress("Test: Charging");
	KickerEnableCharge(1);

	uint32_t time = 0;
	while(!KickerIsCharged() && ++time < TEST_KICKER_TIMEOUT)
		chThdSleepMilliseconds(1);

	if(time == TEST_KICKER_TIMEOUT)
	{
		dischargeKicker();
		TestKickerProgress("Charging failed");
		return;
	}

	pResult->chargingSpeed = (kicker.vCap - startVoltage) / (time * 1e-3f);

	TestKickerProgress("Test: Straight Kick");
	KickerFire(0.001f, KICKER_DEVICE_STRAIGHT);
	chThdSleepMilliseconds(100);
	pResult->straightVoltageDrop = kicker.vCapBeforeKick - kicker.vCap;

	TestKickerProgress("Recharging");
	KickerEnableCharge(1);
	time = 0;
	while(!KickerIsCharged() && ++time < TEST_KICKER_TIMEOUT)
		chThdSleepMilliseconds(1);

	if(time == TEST_KICKER_TIMEOUT)
	{
		dischargeKicker();
		TestKickerProgress("Recharging failed");
		return;
	}

	TestKickerProgress("Test: Chip Kick");
	KickerFire(0.001f, KICKER_DEVICE_CHIP);
	chThdSleepMilliseconds(100);
	pResult->chipVoltageDrop = kicker.vCapBeforeKick - kicker.vCap;

	LEDLeftSet(0.0f, 0.0f, 0.01f, 0.0f);
	LEDRightSet(0.0f, 0.0f, 0.01f, 0.0f);

	kicker.config.maxVoltage = maxVoltageBackup;
	if(!dischargeKicker())
		return;

	TestKickerProgress("Test complete");

	TestModeExit();

	doReasoningOnKicker(pResult);

	TestKickerResults(pResult);
}
