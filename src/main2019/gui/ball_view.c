/*
 * ball_view.c
 *
 *  Created on: 12.11.2019
 *      Author: SabolcJ, MichaelR
 */

#include "ball_view.h"
#include "robot/util.h"
#include "../robot_pi.h"
#include <stdio.h>
#include <string.h>

static struct _handles
{
	GHandle hContLive;
	GHandle hContMap;
	GHandle hContMapDrawArea;

	GHandle hBtnLive;
	GHandle hBtnMap;
	GHandle hBtnNear;
	GHandle hBtnFar;
	GHandle hBtnResInc;
	GHandle hBtnResDec;
	GHandle hBtnRec;
	GHandle hBtnTrig;

	GHandle hResolution;
	GHandle hRate;
	GHandle hNumBalls;
	GHandle hCPU;
	GHandle hScale;
	GHandle hFileImg;
	GHandle hFileVid;
} handles;

static float scalingXY = 1000.0f; // 1m = 1000pixel

static int16_t eventHandler(GEvent* pEvent)
{
	if(pEvent->type == GEVENT_GWIN_BUTTON)
	{
		GEventGWinButton* pBtn = (GEventGWinButton*)pEvent;

		if(pBtn->gwin == handles.hBtnMap)
		{
			gwinSetVisible(handles.hContLive, FALSE);
			gwinSetVisible(handles.hContMap, TRUE);

			robotPi.enablePreview = 0;
		}

		if(pBtn->gwin == handles.hBtnLive)
		{
			gwinSetVisible(handles.hContMap, FALSE);
			gwinSetVisible(handles.hContLive, TRUE);

			robotPi.enablePreview = 1;
		}

		if(pBtn->gwin == handles.hBtnFar)
		{
			scalingXY = 100.0f;
		}

		if(pBtn->gwin == handles.hBtnNear)
		{
			scalingXY = 1000.0f;
		}

		if(pBtn->gwin == handles.hBtnResInc)
		{
			if(robotPi.cameraControl.resolution < 2)
				robotPi.cameraControl.resolution++;
		}

		if(pBtn->gwin == handles.hBtnResDec)
		{
			if(robotPi.cameraControl.resolution > 0)
				robotPi.cameraControl.resolution--;
		}

		if(pBtn->gwin == handles.hBtnRec)
		{
			gwinSetVisible(handles.hFileImg, FALSE);
			gwinSetVisible(handles.hFileVid, TRUE);

			if(robotPi.cameraControl.recording)
				robotPi.cameraControl.recording = 0;
			else
				robotPi.cameraControl.recording = 1;
		}

		if(pBtn->gwin == handles.hBtnTrig)
		{
			gwinSetVisible(handles.hFileVid, FALSE);
			gwinSetVisible(handles.hFileImg, TRUE);

			robotPi.cameraControl.recording = 0;
			RobotPiTriggerImageCapture();
		}
	}

	return -1;
}

GHandle BallViewCreate()
{
	GWidgetInit topContInit = { { 0, 60, 240, 260, FALSE, 0 }, "Camera", 0, &eventHandler, 0 };
	GHandle hTop = gwinContainerCreate(0, &topContInit, 0);

	// info fields
	GWidgetInit resLabelInit = { { 5, 5, 50, 15, TRUE, hTop }, "Res:", 0, 0, 0 };
	gwinLabelCreate(0, &resLabelInit);
	GWidgetInit resInit = { { 55, 5, 110, 15, TRUE, hTop }, "2560x1920", 0, 0, 0 };
	handles.hResolution = gwinLabelCreate(0, &resInit);

	GWidgetInit rateLabelInit = { { 5, 20, 50, 15, TRUE, hTop }, "Rate:", 0, 0, 0 };
	gwinLabelCreate(0, &rateLabelInit);
	GWidgetInit rateInit = { { 55, 20, 110, 15, TRUE, hTop }, "15.00fps", 0, 0, 0 };
	handles.hRate = gwinLabelCreate(0, &rateInit);

	GWidgetInit numBallsLabelInit = { { 5, 35, 50, 15, TRUE, hTop }, "Balls:", 0, 0, 0 };
	gwinLabelCreate(0, &numBallsLabelInit);
	GWidgetInit numBallsInit = { { 55, 35, 110, 15, TRUE, hTop }, "42", 0, 0, 0 };
	handles.hNumBalls = gwinLabelCreate(0, &numBallsInit);

	GWidgetInit cpuLabelInit = { { 5, 50, 50, 15, TRUE, hTop }, "CPU:", 0, 0, 0 };
	gwinLabelCreate(0, &cpuLabelInit);
	GWidgetInit cpuInit = { { 55, 50, 110, 15, TRUE, hTop }, "42.0%", 0, 0, 0 };
	handles.hCPU = gwinLabelCreate(0, &cpuInit);


	// resolution buttons
	GWidgetInit btnResIncInit = { { 170, 5, 65, 30, TRUE, hTop }, "Res+", 0, 0, 0 };
	handles.hBtnResInc = gwinButtonCreate(0, &btnResIncInit);

	GWidgetInit btnResDecInit = { { 170, 45, 65, 30, TRUE, hTop }, "Res-", 0, 0, 0 };
	handles.hBtnResDec = gwinButtonCreate(0, &btnResDecInit);


	// live camera preview panel
    GWidgetInit liveContInit = { { 0, 80, 240, 180, FALSE, hTop }, 0, 0, 0, 0 };
    handles.hContLive = gwinContainerCreate(0, &liveContInit, 0);

	GWidgetInit btnMapInit = { { 5, 5, 60, 30, TRUE, handles.hContLive }, "Map", 0, 0, 0 };
	handles.hBtnMap = gwinButtonCreate(0, &btnMapInit);

	GWidgetInit btnRecInit = { { 170, 105, 65, 30, TRUE, handles.hContLive }, "Record", 0, 0, 0 };
	handles.hBtnRec = gwinButtonCreate(0, &btnRecInit);

	GWidgetInit btnTrigInit = { { 170, 145, 65, 30, TRUE, handles.hContLive }, "Trigger", 0, 0, 0 };
	handles.hBtnTrig = gwinButtonCreate(0, &btnTrigInit);

	GWidgetInit fileLabelInit = { { 5, 40, 40, 15, TRUE, handles.hContLive }, "File:", 0, 0, 0 };
	gwinLabelCreate(0, &fileLabelInit);
	GWidgetInit fileInit = { { 50, 40, 190, 15, FALSE, handles.hContLive }, "-", 0, 0, 0 };
	handles.hFileImg = gwinLabelCreate(0, &fileInit);
	handles.hFileVid = gwinLabelCreate(0, &fileInit);


    // ball map panel
    GWidgetInit mapContInit = { { 0, 80, 240, 180, TRUE, hTop }, 0, 0, 0, 0 };
    handles.hContMap = gwinContainerCreate(0, &mapContInit, 0);

	GWidgetInit btnLiveInit = { { 5, 5, 60, 30, TRUE, handles.hContMap }, "Live", 0, 0, 0 };
	handles.hBtnLive = gwinButtonCreate(0, &btnLiveInit);

	GWidgetInit btnNearInit = { { 100, 5, 65, 30, TRUE, handles.hContMap }, "Near", 0, 0, 0 };
	handles.hBtnNear = gwinButtonCreate(0, &btnNearInit);

	GWidgetInit btnFarInit = { { 170, 5, 65, 30, TRUE, handles.hContMap }, "Far", 0, 0, 0 };
	handles.hBtnFar = gwinButtonCreate(0, &btnFarInit);

	GWidgetInit scaleLabelInit = { { 5, 40, 50, 15, TRUE, handles.hContMap }, "Scale:", 0, 0, 0 };
	gwinLabelCreate(0, &scaleLabelInit);
	GWidgetInit scaleInit = { { 55, 40, 110, 15, TRUE, handles.hContMap }, "20x10cm", 0, 0, 0 };
	handles.hScale = gwinLabelCreate(0, &scaleInit);

    GWidgetInit drawContInit = { { 0, 60, 240, 120, TRUE, handles.hContMap }, 0, 0, 0, 0 };
    handles.hContMapDrawArea = gwinContainerCreate(0, &drawContInit, 0);

	return hTop;
}

void BallViewUpdateCamStats(const ExtCameraStats* pStats)
{
	static char sRes[10];
	static char sRate[10];
	static char sCPU[10];
	static char sImg[32];
	static char sVid[32];

	snprintf(sRes, 10, "%hux%hu", pStats->width, pStats->height);
	gwinSetText(handles.hResolution, sRes, FALSE);

	snprintf(sRate, 10, "%.2ffps", 1.0f/pStats->dtAvg);
	gwinSetText(handles.hRate, sRate, FALSE);

	snprintf(sCPU, 10, "%.1f%%", pStats->rtAvg / pStats->dtAvg * 100.0f);
	gwinSetText(handles.hCPU, sCPU, FALSE);

	strncpy(sImg, pStats->imageFilename, 32);
	gwinSetText(handles.hFileImg, sImg, FALSE);

	if(pStats->recording)
	{
		strncpy(sVid, pStats->recordFilename, 32);
		gwinSetText(handles.hFileVid, sVid, FALSE);
	}
	else
	{
		gwinSetText(handles.hFileVid, "-", FALSE);
	}
}

void BallViewUpdateDetections(const ExtBallDetections* pDetections)
{
	static char sBalls[10];
	static char sScale[10];

	snprintf(sBalls, 10, "%hu", (uint16_t)pDetections->numBalls);
	gwinSetText(handles.hNumBalls, sBalls, FALSE);

	snprintf(sScale, 10, "%hux%hucm", (uint16_t)(20000.0f/scalingXY), (uint16_t)(10000.0f/scalingXY));
	gwinSetText(handles.hScale, sScale, FALSE);

	// viewport box with robot
	gwinSetColor(handles.hContMapDrawArea, Black);
	gwinFillArea(handles.hContMapDrawArea, 0, 0, 240, 120);

	gwinSetColor(handles.hContMapDrawArea, Gray);
	gwinDrawBox(handles.hContMapDrawArea, 20, 0, 220, 120);

	gwinSetColor(handles.hContMapDrawArea, White);
	gwinDrawBox(handles.hContMapDrawArea, 60, 10, 170, 100);
	gwinDrawLine(handles.hContMapDrawArea, 20, 50, 60, 10);
	gwinDrawLine(handles.hContMapDrawArea, 20, 70, 60, 110);

	gwinSetColor(handles.hContMapDrawArea, Black);
	gwinDrawLine(handles.hContMapDrawArea, 60, 10, 60, 110);

	gwinSetColor(handles.hContMapDrawArea, Gray);
	gwinDrawLine(handles.hContMapDrawArea, 20, 60, 230, 60);
	gwinDrawLine(handles.hContMapDrawArea, 125, 10, 125, 110);

	// balls
	const float scalingZ = 200.0f;
	const float maxPixX = 210;
	const float maxPixY = 100;

	gwinSetColor(handles.hContMapDrawArea, DarkGray);
	gwinFillArc(handles.hContMapDrawArea, -15, 10+maxPixY/2, 40, 30, 330);

	gwinSetColor(handles.hContMapDrawArea, Orange);
	float robotPos[3];
	memcpy(robotPos, pDetections->robotPos, sizeof(float)*3);

	for(uint8_t ball = 0; ball < pDetections->numBalls; ball++)
	{
		float posGlobal[3];
		memcpy(posGlobal, pDetections->balls[ball].pos, sizeof(float)*3);

		float posLocal[3] = { 0, 0, posGlobal[2] };
		posLocal[0] = posGlobal[0] - robotPos[0];
		posLocal[1] = posGlobal[1] - robotPos[1];
		CtrlUtilTurnGlobal2Local(robotPos[2], posLocal[0], posLocal[1], posLocal, posLocal+1);

		posLocal[1] -= 0.1f; // adjust zero to be at dribbler, not at robot center

		posLocal[0] *= scalingXY;
		posLocal[1] *= scalingXY;
		posLocal[2] *= scalingZ;

		if(posLocal[0] > maxPixY/2+5)
			posLocal[0] = maxPixY/2+5;
		if(posLocal[0] < -maxPixY/2-5)
			posLocal[0] = -maxPixY/2-5;
		if(posLocal[1] > maxPixX+5)
			posLocal[1] = maxPixX+5;
		if(posLocal[1] < 0)
			posLocal[1] = 0;
		if(posLocal[2] < 3)
			posLocal[2] = 3;
		if(posLocal[2] > 15)
			posLocal[2] = 15;

		gwinFillCircle(handles.hContMapDrawArea, 20+posLocal[1], 10+maxPixY/2+posLocal[0], posLocal[2]);
	}
}
