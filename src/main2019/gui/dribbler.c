/*
 * dribbler.c
 *
 *  Created on: 17.08.2020
 *      Author: AndreR
 */

#include "dribbler.h"
#include "util/ema_filter.h"
#include "../motors.h"
#include "robot/robot.h"
#include <stdio.h>

static struct _handles
{
	GHandle hBtnPresets[4];

	GHandle hTemp;
	GHandle hVoltage;
	GHandle hSpeed;
	GHandle hCurrent;
} handles;

typedef struct _DribblerPreset
{
	uint8_t mode;
	float currentOrVoltage;
	float speed;
} DribblerPreset;

static const DribblerPreset presets[4] = {
	{ 0, 0, 0 },
	{ DRIBBLER_MODE_SPEED,   2.5f, 5000.0f },
	{ DRIBBLER_MODE_SPEED,   4.0f, 12000.0f },
	{ DRIBBLER_MODE_SPEED,   5.0f, 20000.0f }
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
				RobotTestModeEnable(1);
				RobotTestModeStopAll();
				RobotTestModeSetSystems(ROBOT_SYSTEM_DRIBBLER | ROBOT_SYSTEM_STATE_EST);

				robot.skillOutput.dribbler.mode = presets[i].mode;
				robot.skillOutput.dribbler.speed = presets[i].speed * 2.0f*M_PI/60.0f;
				robot.skillOutput.dribbler.voltage = presets[i].currentOrVoltage;
				robot.skillOutput.dribbler.maxCurrent = presets[i].currentOrVoltage;
			}
		}
	}

	return -1;
}

static GHandle createLabel(coord_t x, coord_t y, coord_t w, coord_t h, GHandle parent, const char* pText)
{
	GWidgetInit deviceLabelInit = { { x, y, w, h, TRUE, parent }, pText, 0, 0, 0 };
	return gwinLabelCreate(0, &deviceLabelInit);
}

GHandle DribblerCreate()
{
	GWidgetInit topContInit = { { 0, 60, 240, 260, FALSE, 0 }, "Dribbler", 0, &eventHandler, 0 };
	GHandle hTop = gwinContainerCreate(0, &topContInit, 0);

	createLabel(2, 5, 120, 20, hTop, "Presets:");

	static const char* presetNames[4] = {"Stop", "Low", "Mid", "High"};

	for(uint8_t i = 0; i < 4; i++)
	{
		GWidgetInit btnPresetInit = { { 2+i*60, 30, 55, 55, TRUE, hTop }, presetNames[i], 0, 0, 0 };
		handles.hBtnPresets[i] = gwinButtonCreate(0, &btnPresetInit);
	}

	createLabel(5, 100, 100, 20, hTop, "Temperature:");
	createLabel(5, 120, 100, 20, hTop, "Voltage:");
	createLabel(5, 140, 100, 20, hTop, "Speed:");
	createLabel(5, 160, 100, 20, hTop, "Current:");

	handles.hTemp = createLabel(105, 100, 100, 20, hTop, "-");
	handles.hVoltage = createLabel(105, 120, 100, 20, hTop, "-");
	handles.hSpeed = createLabel(105, 140, 100, 20, hTop, "-");
	handles.hCurrent = createLabel(105, 160, 100, 20, hTop, "-");

	return hTop;
}

void DribblerUpdate(float temp, float volt, float speed, float cur)
{
	static char sTemp[16];
	static char sVoltage[16];
	static char sSpeed[16];
	static char sCurrent[16];

	static EMAFilter emaTemp = { 0.99f, 25.0f };
	static EMAFilter emaVolt = { 0.90f, 0.0f };
	static EMAFilter emaSpeed = { 0.90f, 0.0f };
	static EMAFilter emaCur = { 0.90f, 0.0f };

	temp = EMAFilterUpdate(&emaTemp, temp);
	volt = EMAFilterUpdate(&emaVolt, volt);
	speed = EMAFilterUpdate(&emaSpeed, speed * 60.0f/(2.0f*M_PI));
	cur = EMAFilterUpdate(&emaCur, cur);

	snprintf(sTemp, 16, "%.2f", temp);
	gwinSetText(handles.hTemp, sTemp, FALSE);

	snprintf(sVoltage, 16, "%.3fV", volt);
	gwinSetText(handles.hVoltage, sVoltage, FALSE);

	snprintf(sSpeed, 16, "%.1frpm", speed);
	gwinSetText(handles.hSpeed, sSpeed, FALSE);

	snprintf(sCurrent, 16, "%.2fA", cur);
	gwinSetText(handles.hCurrent, sCurrent, FALSE);
}
