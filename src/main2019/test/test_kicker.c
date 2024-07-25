#include "test_kicker.h"
#include "test_common.h"
#include "test_data.h"
#include "util/test.h"
#include "dev/leds_front.h"
#include "tiger_bot.h"
#include <stdio.h>

#define TEST_KICKER_TIMEOUT_MS 10000

static uint8_t dischargeKicker()
{
	printf("Discharging\r\n");
	KickerSetChargeMode(&tigerBot.kicker, KICKER_CHG_MODE_AUTO_DISCHARGE);

	uint32_t time = 0;
	while(tigerBot.kicker.charger.mode == KICKER_CHG_MODE_AUTO_DISCHARGE && ++time < TEST_KICKER_TIMEOUT_MS)
		chThdSleepMilliseconds(1);

	if(time == TEST_KICKER_TIMEOUT_MS)
	{
		printf("Discharging failed\r\n");
		TestModeExit();
		return 0;
	}

	return 1;
}

static void doReasoningOnKicker(TestResultKicker* pResult)
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

static void testKicker(Test* pTest)
{
	TestResultKicker* pResult = (TestResultKicker*)TestGetResult(pTest);

	Kicker* pKicker = &tigerBot.kicker;

	TestModeStartup();
	pResult->chargingSpeed = 0.0f;
	pResult->straightVoltageDrop = 0.0f;
	pResult->chipVoltageDrop = 0.0f;

	LEDRGBWSet(&devLedFrontLeft, 0.0f, 0.0f, 0.01f, 0.0f);
	LEDRGBWSet(&devLedFrontRight, 0.0f, 0.0f, 0.01f, 0.0f);

	if(!dischargeKicker())
		return;

	LEDRGBWSet(&devLedFrontLeft, 0.01f, 0.0f, 0.0f, 0.0f);
	LEDRGBWSet(&devLedFrontRight, 0.01f, 0.0f, 0.0f, 0.0f);

	float maxVoltageBackup = pKicker->pConfig->maxVoltage_V;
	pKicker->pConfig->maxVoltage_V = 220.0f;
	float startVoltage = pKicker->charger.capAvgFast.value;
	printf("Test: Charging\r\n");
	KickerSetChargeMode(pKicker, KICKER_CHG_MODE_AUTO_CHARGE);

	uint32_t time = 0;
	while(!KickerIsCharged(pKicker) && ++time < TEST_KICKER_TIMEOUT_MS)
		chThdSleepMilliseconds(1);

	if(time == TEST_KICKER_TIMEOUT_MS)
	{
		dischargeKicker();
		printf("Charging failed\r\n");
		return;
	}

	pResult->chargingSpeed = (pKicker->charger.capAvgFast.value - startVoltage) / (time * 1e-3f);

	printf("Test: Straight Kick\r\n");
	KickerArmDuration(pKicker, 0.001f, KICKER_DEVICE_STRAIGHT, 1);
	chThdSleepMilliseconds(100);
	pResult->straightVoltageDrop = pKicker->kick.lastCapVoltageUsed_V;

	printf("Recharging\r\n");
	time = 0;
	while(!KickerIsCharged(pKicker) && ++time < TEST_KICKER_TIMEOUT_MS)
		chThdSleepMilliseconds(1);

	if(time == TEST_KICKER_TIMEOUT_MS)
	{
		dischargeKicker();
		printf("Recharging failed\r\n");
		return;
	}

	printf("Test: Chip Kick\r\n");
	KickerArmDuration(pKicker, 0.001f, KICKER_DEVICE_CHIP, 1);
	chThdSleepMilliseconds(100);
	pResult->chipVoltageDrop = pKicker->kick.lastCapVoltageUsed_V;

	LEDRGBWSet(&devLedFrontLeft, 0.0f, 0.0f, 0.01f, 0.0f);
	LEDRGBWSet(&devLedFrontRight, 0.0f, 0.0f, 0.01f, 0.0f);

	pKicker->pConfig->maxVoltage_V = maxVoltageBackup;
	if(!dischargeKicker())
		return;

	printf("Test complete\r\n");

	TestModeExit();

	doReasoningOnKicker(pResult);
}

SHELL_CMD(kicker, "Test kicker (charge, straight kick, chip kick)");

SHELL_CMD_IMPL(kicker)
{
	(void)pUser; (void)argc; (void)argv;

	TestSchedule(TEST_ID_KICKER, 0, 1);
}

void TestKickerInit(ShellCmdHandler* pCmdHandler)
{
	TestRegister(TEST_ID_KICKER, &testKicker, 0, sizeof(TestResultKicker));

	if(pCmdHandler)
	{
		ShellCmdAdd(pCmdHandler, kicker_command);
	}
}
