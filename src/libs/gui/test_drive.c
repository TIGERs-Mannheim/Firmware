/*
 * test_drive.c
 *
 *  Created on: 23.03.2019
 *      Author: AndreR
 */

#include "test_drive.h"
#include "gui/styles.h"
#include <stdio.h>
#include <math.h>

static const char* pMotors[] = {"FR", "FL", "RL", "RR", "DB"};

static struct _handles
{
	GHandle hOverviewCont;
	GHandle hBtnTestAll;

	struct
	{
		GHandle hBtnMotor;
		GHandle hMotCont;
		GLabelObject* hDamping;
		char damp[8];
		GLabelObject* hInertia;
		char inertia[8];
		GLabelObject* hCur;
		char cur[8];
		GLabelObject* hVel;
		char vel[8];
	} mot[5];

	TestDriveScheduleTestCallback cbScheduleTest;
} handles;

static int16_t eventHandler(GEvent* pEvent)
{
	if(pEvent->type == GEVENT_GWIN_BUTTON)
	{
		GEventGWinButton* pBtn = (GEventGWinButton*)pEvent;

		if(pBtn->gwin == handles.hBtnTestAll)
		{
			for(uint8_t i = 0; i < 5; i++)
			{
				handles.mot[i].hDamping->w.pstyle = &BlackWidgetStyle;
				gwinSetText(&handles.mot[i].hDamping->w.g, "-", FALSE);

				handles.mot[i].hInertia->w.pstyle = &BlackWidgetStyle;
				gwinSetText(&handles.mot[i].hInertia->w.g, "-", FALSE);

				handles.mot[i].hCur->w.pstyle = &BlackWidgetStyle;
				gwinSetText(&handles.mot[i].hCur->w.g, "-", FALSE);

				handles.mot[i].hVel->w.pstyle = &BlackWidgetStyle;
				gwinSetText(&handles.mot[i].hVel->w.g, "-", FALSE);

				gwinSetEnabled(handles.mot[i].hBtnMotor, FALSE);

				(*handles.cbScheduleTest)(IC_TEST_MOT_DYN, i);
			}

			gwinSetEnabled(handles.hBtnTestAll, FALSE);
		}

		for(uint8_t i = 0; i < 5; i++)
		{
			if(pBtn->gwin == handles.mot[i].hBtnMotor)
			{
				(*handles.cbScheduleTest)(IC_TEST_MOT_DYN, i);

				gwinSetEnabled(handles.mot[i].hBtnMotor, FALSE);
			}
		}
	}

	return -1;
}

void TestDriveProgress(const TestProgress* pProgress)
{
	static char buf[8];

	snprintf(buf, 8, "%3hu%%", (uint16_t)pProgress->percentComplete);

	switch(pProgress->testId)
	{
		case IC_TEST_MOT_IDENT_MECH:
		case IC_TEST_MOT_DYN:
		{
			gwinSetText(&handles.mot[pProgress->motorId].hDamping->w.g, buf, FALSE);
		}
		break;
	}
}

void TestDriveDynamicResult(TestMotorDynamicsResult* pResult)
{
	snprintf(handles.mot[pResult->motorId].damp, 8, "%.1f", pResult->damping*1e6);
	snprintf(handles.mot[pResult->motorId].inertia, 8, "%.1f", pResult->inertia*1e6);

	snprintf(handles.mot[pResult->motorId].cur, 8, "%.2f", pResult->cur);
	snprintf(handles.mot[pResult->motorId].vel, 8, "%.0f", pResult->speed);

	gwinSetText(&handles.mot[pResult->motorId].hDamping->w.g, handles.mot[pResult->motorId].damp, FALSE);
	gwinSetText(&handles.mot[pResult->motorId].hInertia->w.g, handles.mot[pResult->motorId].inertia, FALSE);
	gwinSetText(&handles.mot[pResult->motorId].hCur->w.g, handles.mot[pResult->motorId].cur, FALSE);
	gwinSetText(&handles.mot[pResult->motorId].hVel->w.g, handles.mot[pResult->motorId].vel, FALSE);

	handles.mot[pResult->motorId].hDamping->w.pstyle = &GreenTextStyle;
	handles.mot[pResult->motorId].hInertia->w.pstyle = &GreenTextStyle;
	handles.mot[pResult->motorId].hCur->w.pstyle = &GreenTextStyle;
	handles.mot[pResult->motorId].hVel->w.pstyle = &GreenTextStyle;

	if(pResult->reasoning.damping == TEST_REASONING_RESULT_WARNING)
	{
		handles.mot[pResult->motorId].hDamping->w.pstyle = &YellowTextStyle;
	}

	if(pResult->reasoning.damping == TEST_REASONING_RESULT_BAD)
	{
		handles.mot[pResult->motorId].hDamping->w.pstyle = &RedTextStyle;
	}

	if(pResult->reasoning.inertia)
	{
		handles.mot[pResult->motorId].hInertia->w.pstyle = &RedTextStyle;
	}

	if(pResult->reasoning.cur == TEST_REASONING_RESULT_WARNING)
	{
		handles.mot[pResult->motorId].hCur->w.pstyle = &YellowTextStyle;
	}

	if(pResult->reasoning.cur == TEST_REASONING_RESULT_BAD)
	{
		handles.mot[pResult->motorId].hCur->w.pstyle = &RedTextStyle;
	}

	if(pResult->reasoning.vel == TEST_REASONING_RESULT_WARNING)
	{
		handles.mot[pResult->motorId].hVel->w.pstyle = &YellowTextStyle;
	}

	if(pResult->reasoning.vel == TEST_REASONING_RESULT_BAD)
	{
		handles.mot[pResult->motorId].hVel->w.pstyle = &RedTextStyle;
	}

	gwinSetEnabled(handles.mot[pResult->motorId].hBtnMotor, TRUE);

	if(pResult->motorId > 3)
		gwinSetEnabled(handles.hBtnTestAll, TRUE);
}

GHandle TestDriveCreate(TestDriveScheduleTestCallback scheduleTestCallback)
{
	handles.cbScheduleTest = scheduleTestCallback;

    GWidgetInit wi = { { 0, 60, 240, 260, FALSE, 0 }, "Drives", 0, &eventHandler, 0 };
    GHandle hTop = gwinContainerCreate(0, &wi, 0);

    // Container for current settings
    GWidgetInit overviewContInit = { { 0, 0, 240, 260, TRUE, hTop }, 0, 0, 0, 0 };
    handles.hOverviewCont = gwinContainerCreate(0, &overviewContInit, 0);

    for(uint8_t i = 0; i < 5; i++)
    {
		GWidgetInit motBtnInit = { { 30+i*40, 5, 40, 40, TRUE, handles.hOverviewCont }, pMotors[i], 0, 0, 0 };
		handles.mot[i].hBtnMotor = gwinButtonCreate(0, &motBtnInit);

	    GWidgetInit resInit = { { 35+i*40, 55, 35, 35, TRUE, handles.hOverviewCont }, " -", 0, 0, 0 };
	    handles.mot[i].hDamping = (GLabelObject*)gwinLabelCreate(0, &resInit);

	    GWidgetInit emfInit = { { 35+i*40, 95, 35, 35, TRUE, handles.hOverviewCont }, " -", 0, 0, 0 };
	    handles.mot[i].hInertia = (GLabelObject*)gwinLabelCreate(0, &emfInit);

	    GWidgetInit curInit = { { 35+i*40, 135, 35, 35, TRUE, handles.hOverviewCont }, " -", 0, 0, 0 };
	    handles.mot[i].hCur = (GLabelObject*)gwinLabelCreate(0, &curInit);

	    GWidgetInit velInit = { { 35+i*40, 175, 35, 35, TRUE, handles.hOverviewCont }, " -", 0, 0, 0 };
	    handles.mot[i].hVel = (GLabelObject*)gwinLabelCreate(0, &velInit);
    }

    GWidgetInit resLabelInit = { { 5, 55, 55, 16, TRUE, handles.hOverviewCont }, "b", 0, 0, 0 };
    gwinLabelCreate(0, &resLabelInit);

    GWidgetInit emfLabelInit = { { 5, 95, 55, 16, TRUE, handles.hOverviewCont }, "J", 0, 0, 0 };
    gwinLabelCreate(0, &emfLabelInit);

    GWidgetInit curLabelInit = { { 5, 135, 55, 16, TRUE, handles.hOverviewCont }, "i", 0, 0, 0 };
    gwinLabelCreate(0, &curLabelInit);

    GWidgetInit velLabelInit = { { 5, 175, 55, 16, TRUE, handles.hOverviewCont }, "w", 0, 0, 0 };
    gwinLabelCreate(0, &velLabelInit);

	GWidgetInit testAllBtnInit = { { 5, 220, 230, 40, TRUE, handles.hOverviewCont }, "Test All", 0, 0, 0 };
	handles.hBtnTestAll = gwinButtonCreate(0, &testAllBtnInit);

    return hTop;
}

