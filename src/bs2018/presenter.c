#include "presenter.h"
#include "dev/tft.h"
#include "dev/touch.h"
#include "hal/sys_time.h"
#include "sys/rtc.h"
#include "base_station.h"
#include "constants.h"
#include "sys/eth0.h"
#include "gfx.h"

#include "gui/ids.h"
#include "gui/top_bar.h"
#include "gui/menu.h"
#include "gui/robot_status.h"
#include "gui/setup.h"

#define GUI_HEAP_SIZE (40*1024)

static void presenterTask(void*);

Presenter presenter;

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
	RadioBaseSetChannel(&baseStation.radioBase, pSetup->wifi.channel);
	RadioBaseSetFixedRuntime(&baseStation.radioBase, pSetup->wifi.fixedRuntime);
	RadioBaseSetMaxBots(&baseStation.radioBase, pSetup->wifi.maxBots);
	RadioBaseSaveConfig(&baseStation.radioBase);

	BaseStationSetIp(pSetup->eth.ip);
	BaseStationSetPort(pSetup->eth.port);
	BaseStationSetVisionIp(pSetup->vision.ip);
	BaseStationSetVisionPort(pSetup->vision.port);
}

void PresenterInit()
{
	chHeapObjectInit(&guiHeap, guiHeapBuffer, GUI_HEAP_SIZE);

	chThdCreateStatic(presenter.waTask, sizeof(presenter.waTask), TASK_PRIO_PRESENTER, presenterTask, 0);
}

void PresenterPrintHeapStatus()
{
	size_t free;
	size_t fragments = chHeapStatus(&guiHeap, &free, 0);

	printf("Fragments: %u, Free: %u, Core: %u\r\n", fragments, free, chCoreGetStatusX());
}

static void presenterTask(void*)
{
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

	uint32_t lastAction_us = SysTimeUSec();

	systime_t last50msCall = chVTGetSystemTimeX();

	for(uint8_t i = 0; i < RADIO_NUM_ROBOT_CLIENTS; i++)
	{
		presenter.robotInfo[i].pRouterClient = &router.bots[i];
		presenter.robotInfo[i].pVisionObj = &baseStation.vision.robots[i];
		presenter.robotInfo[i].pRadioClient = &baseStation.radioBase.baseIn[i];
	}

	while(1)
	{
		GEvent* pEvent;

		for(uint8_t i = 0; i < RADIO_NUM_ROBOT_CLIENTS; i++)
		{
			if(baseStation.radioBase.baseIn[i].isOnline)
				lastAction_us = SysTimeUSec();
		}

		if(touch.sensor.meas.pressed || baseStation.sumatra.isOnline || baseStation.vision.isOnline)
			lastAction_us = SysTimeUSec();

		// Dimming logic
		uint32_t timeSinceLastAction_us = SysTimeUSec() - lastAction_us;
		if(timeSinceLastAction_us < 15000000)
		{
			TFTSetBrightness(255);
		}
		else if(timeSinceLastAction_us < 20000000)
		{
			TFTSetBrightness(255 - (timeSinceLastAction_us - 15000000) / 22300);
		}
		else
		{
			TFTSetBrightness(32);
			lastAction_us = SysTimeUSec() - 30000000;
		}

		pEvent = geventEventWait(&gl, 5);

		if(chVTTimeElapsedSinceX(last50msCall) > TIME_MS2I(50))
		{
			last50msCall = chVTGetSystemTimeX();

//			RobotStatusPositionFrameUpdate(); // TODO: re-enable or delete?

			TopBarSetEthStats(eth0.rates.rxBytesProcessed, eth0.rates.txBytesProcessed);
			TopBarSetEthLinkStatus(eth0.linkStatus.up, eth0.linkStatus.speed100M);
			TopBarSetEthIP(baseStation.config.base.ip.u8, baseStation.config.base.port);
			TopBarSetWifiStatus(baseStation.radioBase.config.channel, RadioBaseGetNumBotsOnline(&baseStation.radioBase), RadioBaseGetLinkQuality(&baseStation.radioBase));
			TopBarSetRemotes(baseStation.vision.isOnline, baseStation.sumatra.isOnline);
			TopBarSetTime(RTCGetUnixTimestamp());

			RobotStatusUpdate(presenter.robotInfo, baseStation.vision.isOnline);

			SetupUpdate(&baseStation.config, &baseStation.radioBase.config);
		}

		if(pEvent == 0)
			continue;

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
