/*
 * ir_gui.c
 *
 *  Created on: 26.01.2020
 *      Author: MariusM
 */

#include "ir.h"
#include "../ir.h"
#include "util/boot.h"
#include "gui/styles.h"

#include <stdio.h>

#define VIS_WIDTH  150
#define VIS_HEIGHT 120

static struct _handles
{
	GHandle sensors[5];
	GHandle posX;
	GHandle posY;
	GHandle logButton;
	GHandle ballVisualization;
	uint16_t nWrites;
} handles;

static GHandle createLabel(coord_t x, coord_t y, coord_t w, coord_t h, GHandle parent, const char* pText)
{
	GWidgetInit deviceLabelInit = { { x, y, w, h, TRUE, parent }, pText, 0, 0, 0 };
	return gwinLabelCreate(0, &deviceLabelInit);
}

static int16_t eventHandler(GEvent* pEvent)
{
	if(pEvent->type == GEVENT_GWIN_BUTTON)
	{
		GEventGWinButton* pBtn = (GEventGWinButton*)pEvent;

		if(pBtn->gwin == handles.logButton)
		{
			handles.nWrites++;
			IrLogData();
		}
	}
	return -1;
}

GHandle IrGuiCreate()
{
	handles.nWrites = 0;
	GWidgetInit wi = {{ 0, 60, 240, 260, FALSE, 0 }, "IR Array", 0, &eventHandler, 0 };
	GHandle hTop = gwinContainerCreate(0, &wi, 0);

	font_t curFont = gwinGetDefaultFont();
	gwinSetDefaultFont(gdispOpenFont("DejaVuSans10"));

	createLabel(5, 5, 70, 15, hTop, "Sensor Data:");

	GWidgetInit visInit = {{ 0, 140, VIS_WIDTH, VIS_HEIGHT, TRUE, hTop }, NULL, 0, 0, &RedTextStyle };
	handles.ballVisualization = gwinLabelCreate(0, &visInit);

	font_t deja16 = gdispOpenFont("DejaVuSans16");
	gwinSetFont(handles.ballVisualization, deja16);
	for(uint8_t x = 0; x < 5; x++)
	{
		GWidgetInit sensorInit = {{ 0, 20 * x + 20, 220, 15, TRUE, hTop }, 0, 0, 0, &GrayProgressBarStyle };
		handles.sensors[x] = gwinProgressbarCreate(0, &sensorInit);
		gwinProgressbarSetRange(handles.sensors[x], 0.0f, 100.0f * 3.33f);
	}

	handles.posX = createLabel(VIS_WIDTH + 2, 165, 200, 15, hTop, "-");
	handles.posY = createLabel(VIS_WIDTH + 2, 180, 200, 15, hTop, "-");

	GWidgetInit btnLog = {{ 150, 210, 100, 50, TRUE, hTop }, "Log Data", 0, 0, 0 };
	handles.logButton = gwinButtonCreate(0, &btnLog);

	gwinSetDefaultFont(curFont);

	return hTop;
}

static void drawBall(float x, float y, float radius, color_t color)
{
	// Limit and scale the X/Y Coordinates
	x *= 1.0f/4.0f;
	if(x < -1.0f)
	{
		x = -1.0f;
	}
	else if(x > 1.0f)
	{
		x = 1.0f;
	}
	x = x*VIS_WIDTH/2.0f + VIS_WIDTH/2.0f;

	y *= 1.0f/4.0f;
	if(y > 1.0f)
	{
		y = 1.0f;
	}
	else if(y < 0.0f)
	{
		y = 0.0f;
	}

	y = VIS_HEIGHT - VIS_HEIGHT*y;

	gwinSetColor(handles.ballVisualization, color);
	gwinFillCircle(handles.ballVisualization, x, y, radius);
}

void IrGuiUpdate(const IRData* pData, const RobotCtrlState* pState)
{
	float colSums[5] = {0};
	for(uint8_t x = 0; x < 4; ++x)
	{
		for(uint8_t y = 0; y < 5; ++y)
		{
			colSums[y] += pData->vLateral[x][y];
		}
	}

	gwinSetColor(handles.ballVisualization, HTML2COLOR(0x050505));
	gwinFillArea(handles.ballVisualization, 0, 0, 150, 120);

	uint8_t ballDetected = pData->ballDetected;

	// Update Visualization
	if(ballDetected)
	{
		gwinSetColor(handles.ballVisualization, DarkGray);
		gwinDrawLine(handles.ballVisualization, VIS_WIDTH/2, 0, VIS_WIDTH/2, VIS_HEIGHT);

		drawBall(pData->estimatedBallPosition[0], pData->estimatedBallPosition[1], 8.0f, Orange);
	}
	else
	{
		gwinSetText(handles.ballVisualization, " No Ball Detected", FALSE);
	}

	if(pState->ballIrState > 0)
	{
		drawBall(pState->ballIrPos[0]*0.1f, pState->ballIrPos[1]*0.1f, 6.0f, Red);
	}

	static char textBufferColSum[5][10];
	for(uint8_t i = 0; i < 5; ++i)
	{
		snprintf(textBufferColSum[i], 10, "%.03f", colSums[i]);
		gwinSetText(handles.sensors[i], textBufferColSum[i], FALSE);

		gwinProgressbarSetPosition(handles.sensors[i], 100.0f * colSums[i]);
	}

	static char posXBuffer[20];
	static char posYBuffer[20];
	snprintf(posXBuffer, 20, "X=%-5.03fcm", ir.irData.estimatedBallPosition[0]);
	snprintf(posYBuffer, 20, "Y=%-5.03fcm", ir.irData.estimatedBallPosition[1]);
	gwinSetText(handles.posX, posXBuffer, FALSE);
	gwinSetText(handles.posY, posYBuffer, FALSE);

	static char buttonBuffer[30];
	snprintf(buttonBuffer, 30, "Log Data! (%hu)", handles.nWrites);
	gwinSetText(handles.logButton, buttonBuffer, FALSE);
}

