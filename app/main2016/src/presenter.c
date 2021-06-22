/*
 * presenter.c
 *
 *  Created on: 31.10.2015
 *      Author: AndreR
 */

#include "presenter.h"
#include "hal/tft.h"
#include "gfx.h"
#include "util/console.h"
#include "main/network.h"
#include "spi6.h"
#include "hal/kicker.h"
#include "power.h"
#include "util/log_file.h"
#include "hal/led.h"
#include "util/network_print.h"
#include "spi4.h"
#include "test.h"
#include "main/ctrl.h"
#include "main/robot.h"
#include "usb/usb_hcd.h"
#include "usb/usb_msc.h"
#include "util/fw_updater.h"
#include "sdcard.h"

#include "gui/bot_cover.h"
#include "gui/main_status.h"
#include "gui/top_bar.h"
#include "gui/menu.h"
#include "gui/wifi.h"
#include "gui/select_botid.h"
#include "gui/ids.h"
#include "gui/logging.h"
#include "gui/test_drive.h"
#include "gui/fw_update.h"

#define GUI_HEAP_SIZE (20*1024)
memory_heap_t guiHeap;
uint8_t guiHeapBuffer[GUI_HEAP_SIZE] __attribute__((aligned(sizeof(stkalign_t))));

typedef int16_t(*EventHandlerFunc)(GEvent*);

static struct _handles
{
	GHandle hTopBar;
	GHandle hMenu;

	GHandle hWindows[9];
} handles;

static int16_t currentWindow = GUI_WINDOW_STATUS;

static void usbHcdCallback(USBHCDConnectionEvent* pConEvent)
{
	LoggingHCDUpdate(pConEvent);
}

static void usbMscCallback(USBMSCEvent* pMscEvent)
{
	LoggingMSCUpdate(pMscEvent);

	if(pMscEvent->connect)
	{
		if(f_stat("autorun_bootloader", 0) == FR_OK)
		{
			PresenterShowWindow(GUI_WINDOW_UPDATE);
			FwUpdaterLoad(FW_UPDATER_SOURCE_USB);
		}
	}
}

void PresenterInit()
{
	chHeapObjectInit(&guiHeap, guiHeapBuffer, GUI_HEAP_SIZE);

	usbHcd.pConCb = &usbHcdCallback;
	usbMsc.eventCb = &usbMscCallback;
}

void PresenterShowWindow(int16_t newWindow)
{
	if(newWindow < 0 || newWindow > 9)
		return;

	gwinSetVisible(handles.hWindows[currentWindow], FALSE);

	currentWindow = newWindow;

	TopBarSetTitle(((GWidgetObject*)handles.hWindows[currentWindow])->text);

	gwinSetVisible(handles.hMenu, FALSE);
	gwinSetVisible(handles.hTopBar, TRUE);
	gwinSetVisible(handles.hWindows[currentWindow], TRUE);
}


void PresenterTask(void* params)
{
	(void)params;

	chRegSetThreadName("Pres");

	gfxInit();
	gdispSetOrientation(GDISP_ROTATE_180);

	// Set the widget defaults
	gwinSetDefaultFont(gdispOpenFont("fixed_7x14"));
	gwinSetDefaultStyle(&BlackWidgetStyle, FALSE);
	gwinSetDefaultBgColor(Black);
	gwinSetDefaultColor(White);
	gdispClear(Black);

	handles.hTopBar = TopBarCreate(0);
	handles.hWindows[GUI_WINDOW_UPDATE] = FwUpdateCreate(0);
	handles.hWindows[GUI_WINDOW_STATUS] = MainStatusCreate(&KickerAutoDischarge, GUI_WINDOW_BOT_ID, kicker.v2017);
	handles.hWindows[GUI_WINDOW_WIFI_SETTINGS] = WifiCreate();
	handles.hWindows[GUI_WINDOW_BOT_ID] = SelectBotIDCreate(GUI_WINDOW_STATUS);
	handles.hWindows[GUI_WINDOW_LOGGING] = LoggingCreate();
	handles.hWindows[GUI_WINDOW_TEST_DRIVE] = TestDriveCreate(&TestScheduleMotorTest);

	handles.hMenu = MenuCreate(handles.hWindows, 9);

	GListener gl;

	geventListenerInit(&gl);
	gwinAttachListener(&gl);

	TopBarSetTitle(((GWidgetObject*)handles.hWindows[currentWindow])->text);
	gwinSetVisible(handles.hWindows[currentWindow], TRUE);

	while(1)
	{
		GEvent* pEvent;

		pEvent = geventEventWait(&gl, 40);

		if(pEvent == 0)
		{
			MainStatusConfigNetworkUpdate(&network.config);

			PresenterMainStatus stat;
			stat.barrier.level[0] = kicker.vIrOn*1000.0f;
			stat.barrier.level[1] = kicker.vIrOff*1000.0f;
			stat.barrier.threshold = kicker.irAutoThreshold*1000.0f;
			stat.barrier.txDamaged = kicker.barrierDamaged;
			stat.barrier.rxDamaged = kicker.barrierDamaged;
			stat.power.bat = power.vBat;
			stat.power.min = power.batCells*3.4f;
			stat.power.max = power.batCells*4.2f;
			stat.power.cur = power.iCur;
			stat.power.usbPowered = 0;
			stat.kicker.cap = kicker.vCap;
			stat.kicker.max = kicker.config.maxVoltage;
			stat.kicker.chg = kicker.iChg;
			stat.kicker.temp = kicker.tEnv;
			MainStatusKdUpdate(&stat);
			MainStatusHwIdUpdate(RobotImplGetHardwareId());
			MainStatusWifiUpdate(&robot.wifiStat);

			// Wifi
			WifiUpdateConfig(&network.config);
			WifiUpdateStatus(&robot.wifiStat);

			FwUpdateAvailableSources(robot.wifiStat.sumatraOnline, usbMsc.fatFs.fs_type, sdCard.fatFs.fs_type);
			FwUpdateStatus(fwUpdater.progresses, fwUpdater.usedPrograms);

			// Logging
			LoggingLogUpdate(&logFile.event);

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
			PresenterShowWindow(menuSelection);
		}
	}
}
