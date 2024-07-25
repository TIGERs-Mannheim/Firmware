#include "dribbler.h"
#include "math/ema_filter.h"
#include "robot/robot.h"
#include "test_data.h"
#include "util/test.h"
#include "gui/styles.h"
#include <stdio.h>

static void testCalibUpdate(Test* pTest, void* pUser);

static struct _handles
{
	GHandle hBtnSpeed[2];
	GHandle hBtnCurrent[2];
	GHandle hBtnOnOff;
	GHandle hBtnPresets[4];
	GHandle hBtnCalib;

	GHandle hSetSpeed;
	GHandle hSetForce;

	GHandle hTemp;
	GHandle hVoltage;
	GHandle hSpeed;
	GHandle hCurrent;
	GHandle hFrictionEst;
	GHandle hForce;
	GLabelObject* hCalibText;

	uint8_t isOn;
	int32_t velocity_mmDs;
	int32_t force_mN;
} handles;

typedef struct _DribblerPreset
{
	float force;
	float velocity;
} DribblerPreset;

static const DribblerPreset presets[4] = {
	{ 2.0f, 2.0f },
	{ 3.0f, 3.5f },
	{ 4.0f, 3.5f },
	{ 5.0f, 5.0f },
};

static int16_t eventHandler(GEvent* pEvent)
{
	if(pEvent->type == GEVENT_GWIN_BUTTON)
	{
		GEventGWinButton* pBtn = (GEventGWinButton*)pEvent;

		for(uint8_t i = 0; i < 4; i++)
		{
			if(pBtn->gwin == handles.hBtnPresets[i])
			{
				handles.velocity_mmDs = presets[i].velocity * 1e3f;
				handles.force_mN = presets[i].force * 1e3f;
			}
		}

		if(pBtn->gwin == handles.hBtnOnOff)
		{
			if(handles.isOn)
			{
				handles.isOn = 0;

				robot.skillOutput.dribbler.mode = 0;
				robot.skillOutput.dribbler.velocity = 0;
				robot.skillOutput.dribbler.voltage = 0;
				robot.skillOutput.dribbler.maxForce = 0;

				RobotSetIdleMode();

				gwinSetEnabled(handles.hBtnCalib, TRUE);
			}
			else
			{
				RobotTestModeEnable(1);
				RobotTestModeStopAll();
				RobotTestModeSetSystems(ROBOT_SYSTEM_DRIBBLER | ROBOT_SYSTEM_CONTROL | ROBOT_SYSTEM_STATE_EST);

				handles.isOn = 1;

				gwinSetEnabled(handles.hBtnCalib, FALSE);
			}
		}

		if(pBtn->gwin == handles.hBtnCurrent[0] && handles.force_mN > 500)
			handles.force_mN -= 500;

		if(pBtn->gwin == handles.hBtnCurrent[1] && handles.force_mN < 9500)
			handles.force_mN += 500;

		if(pBtn->gwin == handles.hBtnSpeed[0] && handles.velocity_mmDs > 500)
			handles.velocity_mmDs -= 500;

		if(pBtn->gwin == handles.hBtnSpeed[1] && handles.velocity_mmDs < 20000)
			handles.velocity_mmDs += 500;

		if(pBtn->gwin == handles.hBtnCalib)
		{
			TestScheduleWithUpdates(TEST_ID_DRIBBLE_IDLE, 0, 1, &testCalibUpdate, 0);
		}
	}

	return -1;
}

static GHandle createLabel(coord_t x, coord_t y, coord_t w, coord_t h, GHandle parent, const char* pText)
{
	GWidgetInit labelInit = { { x, y, w, h, TRUE, parent }, pText, 0, 0, 0 };
	return gwinLabelCreate(0, &labelInit);
}

static GHandle createButton(coord_t x, coord_t y, coord_t w, coord_t h, GHandle parent, const char* pText)
{
	GWidgetInit buttonInit = { { x, y, w, h, TRUE, parent }, pText, 0, 0, 0 };
	return gwinButtonCreate(0, &buttonInit);
}

GHandle DribblerCreate()
{
	handles.velocity_mmDs = 3000;
	handles.force_mN = 3500;

	GWidgetInit topContInit = { { 0, 60, 240, 260, FALSE, 0 }, "Dribbler", 0, &eventHandler, 0 };
	GHandle hTop = gwinContainerCreate(0, &topContInit, 0);

	handles.hBtnCurrent[0] = createButton(  5, 10, 45, 45, hTop, "-");
	handles.hSetForce    = createLabel(  70, 10, 45, 45, hTop, "-");
	handles.hBtnCurrent[1] = createButton(120, 10, 45, 45, hTop, "+");

	handles.hBtnSpeed[0] = createButton(  5, 60, 45, 45, hTop, "-");
	handles.hSetSpeed    = createLabel(  55, 60, 60, 45, hTop, "-");
	handles.hBtnSpeed[1] = createButton(120, 60, 45, 45, hTop, "+");

	handles.hBtnOnOff = createButton(175, 10, 60, 95, hTop, "On");

	static const char* presetNames[4] = {"Low", "Mid", "High", "Ultra"};

	for(uint8_t i = 0; i < 4; i++)
	{
		handles.hBtnPresets[i] = createButton(2+i*60, 115, 55, 45, hTop, presetNames[i]);
	}

	handles.hBtnCalib = createButton(182, 165, 55, 45, hTop, "Calib");

	handles.hCalibText = (GLabelObject*)createLabel(180, 215, 55, 15, hTop, "CAL BAD");
	handles.hCalibText->w.pstyle = &RedTextStyle;

	createLabel(5, 165, 100, 15, hTop, "Temperature:");
	createLabel(5, 180, 100, 15, hTop, "Voltage:");
	createLabel(5, 195, 100, 15, hTop, "Speed:");
	createLabel(5, 210, 100, 15, hTop, "Current:");
	createLabel(5, 225, 100, 15, hTop, "Friction:");
	createLabel(5, 240, 100, 15, hTop, "Force:");

	handles.hTemp    = createLabel(105, 165, 70, 15, hTop, "-");
	handles.hVoltage = createLabel(105, 180, 70, 15, hTop, "-");
	handles.hSpeed   = createLabel(105, 195, 70, 15, hTop, "-");
	handles.hCurrent = createLabel(105, 210, 70, 15, hTop, "-");
	handles.hFrictionEst = createLabel(105, 225, 70, 15, hTop, "-");
	handles.hForce  = createLabel(105, 240, 130, 15, hTop, "-");

	return hTop;
}

//void DribblerUpdate(float temp, float volt, float speed, float cur, float friction, float force)
void DribblerUpdate(const McuMotor* pMot, const McuDribbler* pDrib, const RobotCtrlState* pState, const RobotCtrlReference* pRef)
{
	static char sTemp[16];
	static char sVoltage[16];
	static char sSpeed[16];
	static char sCurrent[16];
	static char sFriction[16];
	static char sForce[32];
	static char sSetSpeed[16];
	static char sSetForce[16];

	static EMAFilter emaTemp = { 0.99f, 25.0f };
	static EMAFilter emaVolt = { 0.90f, 0.0f };
	static EMAFilter emaSpeed = { 0.90f, 0.0f };
	static EMAFilter emaCur = { 0.90f, 0.0f };
	static EMAFilter emaFric = { 0.80f, 0.0f };
	static EMAFilter emaForce = { 0.80f, 0.0f };

	EMAFilterUpdate(&emaTemp, pDrib->meas.dribblerTemp_degC);
	EMAFilterUpdate(&emaVolt, pMot->meas.avgVoltageDQ1ms_V[1]);
	EMAFilterUpdate(&emaSpeed, pMot->meas.hallVelocity_radDs * 60.0f/(2.0f*M_PI));
	EMAFilterUpdate(&emaCur, pMot->meas.avgCurrentDQ1ms_A[1]);
	EMAFilterUpdate(&emaFric, pState->dribblerKf);
	EMAFilterUpdate(&emaForce, pState->dribblerForce);

	snprintf(sTemp, 16, "%.2f", emaTemp.value);
	gwinSetText(handles.hTemp, sTemp, FALSE);

	snprintf(sVoltage, 16, "%.3fV", emaVolt.value);
	gwinSetText(handles.hVoltage, sVoltage, FALSE);

	snprintf(sSpeed, 16, "%.1frpm", emaSpeed.value);
	gwinSetText(handles.hSpeed, sSpeed, FALSE);

	snprintf(sCurrent, 16, "%.2fA", emaCur.value);
	gwinSetText(handles.hCurrent, sCurrent, FALSE);

	snprintf(sFriction, 16, "%.3fmNms", emaFric.value * 1e3f);
	gwinSetText(handles.hFrictionEst, sFriction, FALSE);

	snprintf(sForce, 32, "% .3fN (%.3fN)", emaForce.value, pRef->dribblerForce);
	gwinSetText(handles.hForce, sForce, FALSE);

	snprintf(sSetForce, 16, "%.1fN", handles.force_mN*1e-3f);
	gwinSetText(handles.hSetForce, sSetForce, FALSE);

	snprintf(sSetSpeed, 16, "%.1fm/s", handles.velocity_mmDs*1e-3f);
	gwinSetText(handles.hSetSpeed, sSetSpeed, FALSE);

	if(handles.isOn)
	{
		robot.skillOutput.dribbler.mode = DRIBBLER_MODE_SPEED;
		robot.skillOutput.dribbler.velocity = handles.velocity_mmDs * 1e-3f;
		robot.skillOutput.dribbler.voltage = 0;
		robot.skillOutput.dribbler.maxForce = handles.force_mN * 1e-3f;

		gwinSetText(handles.hBtnOnOff, "Off", FALSE);
	}
	else
	{
		gwinSetText(handles.hBtnOnOff, "On", FALSE);
	}

	if(fusionEKF.dribbler.idleCurrentTable.usedPoints)
	{
		gwinSetText(&handles.hCalibText->w.g, "CAL OK", FALSE);
		handles.hCalibText->w.pstyle = &GreenTextStyle;
	}
	else
	{
		gwinSetText(&handles.hCalibText->w.g, "CAL BAD", FALSE);
		handles.hCalibText->w.pstyle = &RedTextStyle;
	}
}

static void testCalibUpdate(Test* pTest, void* pUser)
{
	(void)pUser;

	if(pTest->state == TEST_STATE_DONE)
	{
		gwinSetText(handles.hBtnCalib, "Calib", FALSE);
		gwinSetEnabled(handles.hBtnCalib, TRUE);
	}
	else
	{
		static char buf[8];
		snprintf(buf, 8, "%3hu%%", (uint16_t)TestGetProgress(pTest));
		gwinSetText(handles.hBtnCalib, buf, FALSE);
		gwinSetEnabled(handles.hBtnCalib, FALSE);
	}
}
