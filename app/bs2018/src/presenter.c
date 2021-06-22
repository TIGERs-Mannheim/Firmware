/*
 * presenter.c
 *
 *  Created on: 24.10.2017
 *      Author: AndreR
 */

#include "presenter.h"
#include "hal/tft.h"
#include "hal/touch.h"
#include "hal/rtc.h"
#include "hal/eth.h"
#include "util/console.h"
#include "gfx.h"
#include "wifi.h"
#include "network.h"
#include "hub.h"

#include "gui/ids.h"
#include "gui/top_bar.h"
#include "gui/menu.h"
#include "gui/robot_status.h"
#include "gui/setup.h"

#define GUI_HEAP_SIZE (40*1024)
memory_heap_t guiHeap;
uint8_t guiHeapBuffer[GUI_HEAP_SIZE] __attribute__((aligned(sizeof(stkalign_t))));

typedef int16_t(*EventHandlerFunc)(GEvent*);

static struct _handles
{
	GHandle hTopBar;
	GHandle hMenu;

	GHandle hWindows[9];
} handles;

static void setupChangedCallback(const SetupData* pSetup)
{
	WifiSetChannel(pSetup->wifi.channel);
	WifiSetFixedRuntime(pSetup->wifi.fixedRuntime);
	WifiSetMaxBot(pSetup->wifi.channel);
	WifiSaveConfig();

	NetworkSetMyIP(pSetup->eth.ip);
	NetworkSetMyPort(pSetup->eth.port);
	NetworkSetMyMAC(pSetup->eth.mac);

	NetworkSetVisionIP(pSetup->vision.ip);
	NetworkSetVisionPort(pSetup->vision.port);

	NetworkSaveConfig();
}

void PresenterInit()
{
	chHeapObjectInit(&guiHeap, guiHeapBuffer, GUI_HEAP_SIZE);
}

void PresenterTask(void* params)
{
	(void)params;

	chRegSetThreadName("Pres");

	gfxInit();
	gdispSetOrientation(GDISP_ROTATE_180);

	// Set the widget defaults
	gwinSetDefaultFont(gdispOpenFont("DejaVuSans16"));
	gwinSetDefaultStyle(&BlackWidgetStyle, FALSE);
	gwinSetDefaultBgColor(Black);
	gwinSetDefaultColor(White);
	gdispClear(Green);

	handles.hTopBar = TopBarCreate();
	handles.hWindows[GUI_ROBOT_STATUS] = RobotStatusCreate();
	handles.hWindows[GUI_SETUP] = SetupCreate();

	handles.hMenu = MenuCreate(handles.hWindows, 9);

	SetupSetCallback(&setupChangedCallback);

	GListener gl;

	geventListenerInit(&gl);
	gwinAttachListener(&gl);
	geventAttachSource(&gl, ginputGetMouse(GMOUSE_ALL_INSTANCES), GLISTEN_TOUCHDOWNMOVES | GLISTEN_TOUCHMETA);

	int16_t currentWindow = GUI_ROBOT_STATUS;

	TopBarSetTitle(((GWidgetObject*)handles.hWindows[currentWindow])->text);
	gwinSetVisible(handles.hWindows[currentWindow], TRUE);

	uint32_t lastActionUTC = RTCGetUnixTimestamp();

	systime_t last1sCall = chVTGetSystemTimeX();
	systime_t last50msCall = chVTGetSystemTimeX();

	static PresenterRobotInfo robotInfo[WIFI_MAX_BOTS];
	for(uint8_t i = 0; i < WIFI_MAX_BOTS; i++)
	{
		robotInfo[i].pFeedback = &hub.robot[i].lastMatchFeedback;
		robotInfo[i].pRobot = &wifi.bots[i];
		robotInfo[i].pHub = &hub.robot[i];
	}

	while(1)
	{
		GEvent* pEvent;

		for(uint8_t i = 0; i < WIFI_MAX_BOTS; i++)
		{
			if(wifi.bots[i].online)
				lastActionUTC = RTCGetUnixTimestamp();
		}

		if(touch.sensor.pressed)
			lastActionUTC = RTCGetUnixTimestamp();

		if(RTCGetUnixTimestamp() - lastActionUTC > 30)
		{
			TFTEnable(0);
		}
		else
		{
			TFTEnable(1);
		}

		pEvent = geventEventWait(&gl, 5);

		if(chVTTimeElapsedSinceX(last50msCall) > MS2ST(50))
		{
			last50msCall = chVTGetSystemTimeX();

			RobotStatusPositionFrameUpdate();
		}

		if(chVTTimeElapsedSinceX(last1sCall) > S2ST(1))
		{
			last1sCall = chVTGetSystemTimeX();

			TopBarSetEthStats(EthGetStats()->rxBytesProcessed, EthGetStats()->txBytesProcessed);
			TopBarSetEthLinkStatus(EthGetLinkStatus()->up, EthGetLinkStatus()->speed100M);
			TopBarSetEthIP(network.cfg.my.ip.u8, network.cfg.my.port);
			TopBarSetWifiStatus(wifi.cfg.channel, WifiGetNumBotsOnline(), WifiGetLinkQuality());
			TopBarSetRemotes(network.visionAvailable, network.sumatraAvailable);
			TopBarSetTime(RTCGetUnixTimestamp());

			RobotStatusUpdate(robotInfo, network.visionAvailable);

			SetupUpdate(&wifi.cfg, &network.cfg);
		}


		if(pEvent == 0)
		{
//			MainStatusWifiUpdate(&network.config);
//
//			KdFeedback kd;
//			kd.barrier.level[0] = kicker.vIrOn*1000.0f;
//			kd.barrier.level[1] = kicker.vIrOff*1000.0f;
//			kd.barrier.threshold = kicker.irAutoThreshold*1000.0f;
////			kd.barrier.threshold = kicker.config.irThreshold*1000.0f;
//			kd.power.bat = power.vBat;
//			kd.power.cur = power.iCur;
//			kd.kicker.cap = kicker.vCap;
//			kd.kicker.chg = kicker.iChg;
//			kd.kicker.temp = kicker.tEnv;
//			MainStatusKdUpdate(&kd);
//			MainStatusHwIdUpdate(robot.misc.hardwareId);
//
//			// Wifi
//			WifiUpdateConfig(&network.config);
//			WifiUpdateStatus(&robot.wifiStat);
//
//			// Optical
//			OpticalUpdate(&spi4.mouse[0].motion, 0);
//			OpticalUpdate(&spi4.mouse[1].motion, 1);
//			OpticalSpeedUpdate(spi4.opticalSpeed);

			continue;
		}

		if(TopBarIsMenuClicked(pEvent))
		{
			gwinSetVisible(handles.hTopBar, FALSE);
			gwinSetVisible(handles.hWindows[currentWindow], FALSE);

			gwinSetVisible(handles.hMenu, TRUE);

			continue;
		}

		int16_t menuSelection = -1;
		if(((GWidgetObject*)handles.hWindows[currentWindow])->fnParam)
		{
			// please close your eyes for the next line, looks messy ;)
			menuSelection = ((EventHandlerFunc)(((GWidgetObject*)handles.hWindows[currentWindow])->fnParam))(pEvent);
		}

		if(menuSelection < 0)
			menuSelection = MenuGetSelection(pEvent);

		if(menuSelection >= 0)
		{
			gwinSetVisible(handles.hWindows[currentWindow], FALSE);

			currentWindow = menuSelection;

			TopBarSetTitle(((GWidgetObject*)handles.hWindows[currentWindow])->text);

			gwinSetVisible(handles.hMenu, FALSE);
			gwinSetVisible(handles.hTopBar, TRUE);
			gwinSetVisible(handles.hWindows[currentWindow], TRUE);
		}

	}
}
