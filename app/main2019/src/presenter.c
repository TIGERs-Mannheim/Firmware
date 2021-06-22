/*
 * presenter.c
 *
 *  Created on: 09.01.2019
 *      Author: AndreR
 */

#include "presenter.h"
#include "hal/tft.h"
#include "gfx.h"
#include "util/console.h"
#include "main/network.h"
#include "power.h"
#include "util/log_file.h"
#include "util/network_print.h"
#include "main/robot.h"
#include "kicker.h"
#include "util/fw_updater.h"
#include "motors.h"
#include "ir.h"
#include "ext.h"
#include "test.h"
#include "usb/usb_hcd.h"
#include "usb/usb_msc.h"
#include "sdcard.h"
#include "util/bmp.h"
#include "util/sys_time.h"
#include "spi4.h"
#include "robot_pi.h"

#include "gui/bot_cover.h"
#include "gui/main_status.h"
#include "gui/top_bar.h"
#include "gui/menu.h"
#include "gui/wifi.h"
#include "gui/select_botid.h"
#include "gui/ids.h"
#include "gui/logging.h"
#include "gui/test_kicker.h"
#include "gui/test_drive.h"
#include "gui/ball_view.h"
#include "gui/versions.h"
#include "gui/fw_update.h"
#include "gui/ir_gui.h"
#include "gui/imu.h"
#include "gui/dribbler.h"

#include <stdio.h>

#define GUI_HEAP_SIZE (19*1024)
memory_heap_t guiHeap;
uint8_t guiHeapBuffer[GUI_HEAP_SIZE] __attribute__((aligned(sizeof(stkalign_t))));

typedef int16_t(*EventHandlerFunc)(GEvent*);

static struct _handles
{
	GHandle hTopBar;
	GHandle hMenu;

	GHandle hWindows[12];

	uint8_t takeScreenshot;
} handles;

static int16_t currentWindow = GUI_WINDOW_STATUS;

static void takeScreenshot();

#ifndef ENV_BOOT
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
#endif

void PresenterInit()
{
	chHeapObjectInit(&guiHeap, guiHeapBuffer, GUI_HEAP_SIZE);

#ifndef ENV_BOOT
	usbHcd.pConCb = &usbHcdCallback;
	usbMsc.eventCb = &usbMscCallback;
#endif
}

void PresenterShowWindow(int16_t newWindow)
{
	if(newWindow < 0 || newWindow > 11)
		return;

	gwinSetVisible(handles.hWindows[currentWindow], FALSE);

	currentWindow = newWindow;

	TopBarSetTitle(((GWidgetObject*)handles.hWindows[currentWindow])->text);

	gwinSetVisible(handles.hMenu, FALSE);
	gwinSetVisible(handles.hTopBar, TRUE);
	gwinSetVisible(handles.hWindows[currentWindow], TRUE);
}

void PresenterTakeScreenshot()
{
	handles.takeScreenshot = 1;
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

#ifndef ENV_BOOT
	handles.hTopBar = TopBarCreate(0);
	handles.hWindows[GUI_WINDOW_UPDATE] = FwUpdateCreate(0);
	handles.hWindows[GUI_WINDOW_STATUS] = MainStatusCreate(&KickerAutoDischarge, GUI_WINDOW_BOT_ID, 1);
	handles.hWindows[GUI_WINDOW_WIFI_SETTINGS] = WifiCreate();
	handles.hWindows[GUI_WINDOW_BOT_ID] = SelectBotIDCreate(GUI_WINDOW_STATUS);  // switch back to status window on selection
	handles.hWindows[GUI_WINDOW_LOGGING] = LoggingCreate();
	handles.hWindows[GUI_WINDOW_TEST_KICKER] = TestKickerCreate(&TestScheduleKickerTest);
	handles.hWindows[GUI_WINDOW_TEST_DRIVE] = TestDriveCreate(&TestScheduleMotorTest);
	handles.hWindows[GUI_WINDOW_BALL_VIEW] = BallViewCreate();
	handles.hWindows[GUI_WINDOW_VERSIONS] = VersionsCreate();
	handles.hWindows[GUI_WINDOW_IR] = IrGuiCreate();
	handles.hWindows[GUI_WINDOW_IMU] = ImuCreate();
	handles.hWindows[GUI_WINDOW_DRIBBLER] = DribblerCreate();
#else
	handles.hTopBar = TopBarCreate(1);
	handles.hWindows[GUI_WINDOW_UPDATE] = FwUpdateCreate(1);
	currentWindow = GUI_WINDOW_UPDATE;
#endif

	handles.hMenu = MenuCreate(handles.hWindows, 12);

	GListener gl;

	geventListenerInit(&gl);
	gwinAttachListener(&gl);

	TopBarSetTitle(((GWidgetObject*)handles.hWindows[currentWindow])->text);
	gwinSetVisible(handles.hWindows[currentWindow], TRUE);

	while(1)
	{
		if(handles.takeScreenshot)
		{
			handles.takeScreenshot = 0;

			takeScreenshot();
		}

		GEvent* pEvent = geventEventWait(&gl, 40);

		if(pEvent == 0)
		{
#ifndef ENV_BOOT
			MainStatusConfigNetworkUpdate(&network.config);

			PresenterMainStatus stat;
			stat.barrier.level[0] = kicker.vIrOn*1000.0f;
			stat.barrier.level[1] = kicker.vIrOff*1000.0f;
			stat.barrier.threshold = kicker.irAutoThreshold*1000.0f;
			stat.barrier.txDamaged = kicker.barrierTxDamaged;
			stat.barrier.rxDamaged = kicker.barrierRxDamaged;
			stat.power.bat = power.vBat;
			stat.power.min = power.batCells*power.cellMin;
			stat.power.max = power.batCells*power.cellMax;
			stat.power.cur = power.iCur;
			stat.power.usbPowered = power.usbPowered;
			stat.kicker.cap = kicker.vCap;
			stat.kicker.max = kicker.config.maxVoltage;
			stat.kicker.chg = 0.0f;
			stat.kicker.temp = kicker.tEnv;
			MainStatusKdUpdate(&stat);
			MainStatusHwIdUpdate(RobotImplGetHardwareId());
			MainStatusWifiUpdate(&robot.wifiStat);

			// Wifi
			WifiUpdateConfig(&network.config);
			WifiUpdateStatus(&robot.wifiStat);

			//Ball Detector
			BallViewUpdateCamStats(&robotPi.cameraStats);
			BallViewUpdateDetections(&robotPi.ballDetections);

			//Versions
			VersionsUpdateMotor(0, &motors.motor[0].flashResult);
			VersionsUpdateIr(&ir.flashResult);
			VersionsUpdateExt(&robotPi.robotPiVersion, ext.installed);

			if(power.usbPowered)
			{
				MenuSetEnabled(GUI_WINDOW_TEST_DRIVE, FALSE);
				MenuSetEnabled(GUI_WINDOW_TEST_KICKER, FALSE);
			}
			else
			{
				MenuSetEnabled(GUI_WINDOW_TEST_DRIVE, TRUE);
				MenuSetEnabled(GUI_WINDOW_TEST_KICKER, TRUE);
			}

			FwUpdateAvailableSources(robot.wifiStat.sumatraOnline, usbMsc.fatFs.fs_type, sdCard.fatFs.fs_type);
#else
			FwUpdateAvailableSources(0, 0, 0);
#endif

			FwUpdateStatus(fwUpdater.progresses, fwUpdater.usedPrograms);

			// Logging
			LoggingLogUpdate(&logFile.event);

			// IR Array
			IrGuiUpdate(&ir.irData, &robot.state);

			// IMU
			ImuUpdate(spi4.imu.acc, spi4.imu.gyr, spi4.mag.strength, spi4.imu.temp, spi4.mag.temp);

			// Dribbler
			DribblerUpdate(ir.dribblerTemperature, motors.motor[4].avgVoltageDQ1ms[1], motors.motor[4].hallVelocity, motors.motor[4].avgCurrentDQ1ms[1]);

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

void PresenterPrintHeapStatus()
{
	size_t free;
	size_t fragments = chHeapStatus(&guiHeap, &free);

	ConsolePrint("Fragments: %u, Free: %u, Core: %u\r\n", fragments, free, chCoreGetStatusX());
}

static void takeScreenshot()
{
	FIL file;
	UINT bytesWritten;
	FRESULT fresult = FR_EXIST;

	uint32_t shotNr = 0;
	char filename[32];

	while(fresult == FR_EXIST)
	{
		snprintf(filename, 32, "USB:/screenshot%04u.bmp", shotNr);

		fresult = f_open(&file, filename, FA_CREATE_NEW | FA_WRITE);
		if(fresult == FR_OK)
			break;
		if(fresult != FR_EXIST)
		{
			ConsolePrint("Cannot open file: %s   Error: %u\r\n", filename, fresult);
			return;
		}

		++shotNr;
	}

	uint32_t tStart = SysTimeUSec();

	BMPFileHeader fileHeader;
	fileHeader.bfType = BMP_BF_TYPE;
	fileHeader.bfSize = sizeof(BMPFileHeader) + sizeof(BMPInfoHeader) + 320*240*2;
	fileHeader.bfReserved = 0;
	fileHeader.bfOffBits = sizeof(BMPFileHeader) + sizeof(BMPInfoHeader);

	BMPInfoHeader infoHeader;
	infoHeader.biSize = sizeof(BMPInfoHeader);
	infoHeader.biWidth = gdispGetWidth();
	infoHeader.biHeight = -((int32_t)gdispGetHeight());
	infoHeader.biPlanes = 1;
	infoHeader.biBitCount = 16;
	infoHeader.biCompression = BMP_BI_RGB;
	infoHeader.biSizeImage = 0;
	infoHeader.biXPelsPerMeter = 0;
	infoHeader.biYPelsPerMeter = 0;
	infoHeader.biClrUsed = 0;
	infoHeader.biClrImportant = 0;

	BMPWriteHeaders(&file, &fileHeader, &infoHeader);

	static uint16_t rows[240*2];

	for(uint32_t y = 0; y < 320; y += 2)
	{
		*TFT_CMD_REG = 0x2A;
		*TFT_DATA_REG = 0;
		*TFT_DATA_REG = 0;
		*TFT_DATA_REG = 0;
		*TFT_DATA_REG = 239;

		*TFT_CMD_REG = 0x2B;
		*TFT_DATA_REG = y >> 8;
		*TFT_DATA_REG = y & 0xFF;
		*TFT_DATA_REG = (y+2) >> 8;
		*TFT_DATA_REG = (y+2) & 0xFF;

		*TFT_CMD_REG = 0x2E;
		rows[0] = *TFT_DATA_REG; // dummy read (required)

		for(uint32_t i = 0; i < 240*2; i += 2)
		{
			uint16_t blockA = *TFT_DATA_REG;
			uint16_t blockB = *TFT_DATA_REG;
			uint16_t blockC = *TFT_DATA_REG;

			uint16_t bgr[2][3];
			bgr[0][0] = (blockB &0xF800) >> 8; // blue 1
			bgr[0][1] = blockA & 0xF8; // green 1
			bgr[0][2] = (blockA & 0xF800) >> 8; // red 1
			bgr[1][0] = blockC & 0xF8; // blue 2
			bgr[1][1] = (blockC & 0xF800) >> 8; // green 2
			bgr[1][2] = blockB & 0xF8; // red 2

			rows[i + 0] = (bgr[0][0] >> 3) | ((bgr[0][1] >> 3) << 5) | ((bgr[0][2] >> 3) << 10);
			rows[i + 1] = (bgr[1][0] >> 3) | ((bgr[1][1] >> 3) << 5) | ((bgr[1][2] >> 3) << 10);
		}

		f_write(&file, rows, sizeof(rows), &bytesWritten);
	}

	f_close(&file);

	float timeTaken = (SysTimeUSec() - tStart) * 1e-6f;

	ConsolePrint("Screenshot taken: %s (took %.3fs)\r\n", filename, timeTaken);
}
