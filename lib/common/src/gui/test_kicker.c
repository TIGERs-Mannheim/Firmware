/*
 * test_kicker.c
 *
 *  Created on: 20.01.2020
 *      Author: lixfel
 */


#include "test_kicker.h"
#include "gui/styles.h"
#include <stdio.h>

static struct _handles
{
	GHandle hOverviewCont;
	GHandle hBtnTest;

	GLabelObject* hChargeSpeed;
	char speed[22];
	GLabelObject* hStraightVoltage;
	char straightVoltage[22];
	GLabelObject* hChipVoltage;
	char chipVoltage[22];
	GLabelObject* hProgress;

	TestKickerScheduleTestCallback cbScheduleTest;
} handles;

static int16_t eventHandler(GEvent* pEvent)
{
	if(pEvent->type == GEVENT_GWIN_BUTTON)
	{
		GEventGWinButton* pBtn = (GEventGWinButton*)pEvent;

		if(pBtn->gwin == handles.hBtnTest)
		{
			(*handles.cbScheduleTest)();

			gwinSetText(&handles.hChargeSpeed->w.g, "Charge: - V/s", FALSE);
			gwinSetText(&handles.hStraightVoltage->w.g, "Straight: - V", FALSE);
			gwinSetText(&handles.hChipVoltage->w.g, "Chip: - V", FALSE);

			handles.hChargeSpeed->w.pstyle = &BlackWidgetStyle;
			handles.hStraightVoltage->w.pstyle = &BlackWidgetStyle;
			handles.hChipVoltage->w.pstyle = &BlackWidgetStyle;

			gwinSetEnabled(handles.hBtnTest, FALSE);
		}
	}

	return -1;
}

static void reasoningColor(GLabelObject* pLabel, uint8_t reasoning)
{
	if(reasoning == TEST_REASONING_RESULT_BAD)
		pLabel->w.pstyle = &RedTextStyle;
	else if(reasoning == TEST_REASONING_RESULT_WARNING)
		pLabel->w.pstyle = &YellowTextStyle;
	else
		pLabel->w.pstyle = &GreenTextStyle;
}

void TestKickerResults(TestKickerResult* pResult)
{
	snprintf(handles.speed, 22, "Charge: %.1f V/s", pResult->chargingSpeed);
	snprintf(handles.straightVoltage, 22, "Straight: %.1f V", pResult->straightVoltageDrop);
	snprintf(handles.chipVoltage, 22, "Chip: %.1f V", pResult->chipVoltageDrop);

	reasoningColor(handles.hChargeSpeed, pResult->reasoning.charge);
	reasoningColor(handles.hStraightVoltage, pResult->reasoning.straight);
	reasoningColor(handles.hChipVoltage, pResult->reasoning.chip);

	gwinSetText(&handles.hChargeSpeed->w.g, handles.speed, FALSE);
	gwinSetText(&handles.hStraightVoltage->w.g, handles.straightVoltage, FALSE);
	gwinSetText(&handles.hChipVoltage->w.g, handles.chipVoltage, FALSE);

	gwinSetEnabled(handles.hBtnTest, TRUE);
}

void TestKickerProgress(const char *pProgress)
{
	gwinSetText(&handles.hProgress->w.g, pProgress, FALSE);
}

GHandle TestKickerCreate(TestKickerScheduleTestCallback scheduleTestCallback)
{
	handles.cbScheduleTest = scheduleTestCallback;

	GWidgetInit wi = { { 0, 60, 240, 260, FALSE, 0 }, "Kicker", 0, &eventHandler, 0 };
	GHandle hTop = gwinContainerCreate(0, &wi, 0);

	// Container for current settings
	GWidgetInit overviewContInit = { { 0, 0, 240, 260, TRUE, hTop }, 0, 0, 0, 0 };
	handles.hOverviewCont = gwinContainerCreate(0, &overviewContInit, 0);

	GWidgetInit speedInit = { { 5, 10, 150, 40, TRUE, handles.hOverviewCont }, "Charge: - V/s", 0, 0, 0 };
	handles.hChargeSpeed = (GLabelObject*)gwinLabelCreate(0, &speedInit);

	GWidgetInit volSInit = { { 5, 55, 150, 40, TRUE, handles.hOverviewCont }, "Straight: - V", 0, 0, 0 };
	handles.hStraightVoltage = (GLabelObject*)gwinLabelCreate(0, &volSInit);

	GWidgetInit volCInit = { { 5, 100, 150, 40, TRUE, handles.hOverviewCont }, "Chip: - V", 0, 0, 0 };
	handles.hChipVoltage = (GLabelObject*)gwinLabelCreate(0, &volCInit);

	GWidgetInit progressInit = { { 5, 145, 150, 40, TRUE, handles.hOverviewCont }, "", 0, 0, 0 };
	handles.hProgress = (GLabelObject*)gwinLabelCreate(0, &progressInit);
	handles.hProgress->w.pstyle = &BlueTextStyle;

	GWidgetInit testAllBtnInit = { { 5, 220, 230, 40, TRUE, handles.hOverviewCont }, "Test Kicker", 0, 0, 0 };
	handles.hBtnTest = gwinButtonCreate(0, &testAllBtnInit);

	return hTop;
}
