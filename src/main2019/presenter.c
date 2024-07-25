#include "presenter.h"
#include "dev/tft.h"
#include "robot/network.h"
#include "util/log_file.h"
#include "util/network_print.h"
#include "robot/robot.h"
#include "util/fw_updater.h"
#include "hal/usb/usb_hcd.h"
#include "hal/usb/usb_msc.h"
#include "sys/sdcard.h"
#include "util/bmp.h"
#include "hal/sys_time.h"
#include "tiger_bot.h"
#include "dev/imu.h"
#include "dev/mag.h"
#include "dev/drib_ir.h"
#include "dev/motors.h"
#include "dev/raspberry_pi.h"
#include "dev/touch.h"

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

#include "gfx.h"
#include "src/ginput/ginput_driver_mouse.h"
#include "src/gdisp/gdisp_driver.h"

#include <string.h>
#include <stdio.h>

#define GUI_HEAP_SIZE (24*1024)
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
			FwUpdaterLoad(&tigerBot.fwUpdater, &tigerBot.fwLoaderUsb.source);
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

static void kickerDischarge()
{
	KickerSetChargeMode(&tigerBot.kicker, KICKER_CHG_MODE_AUTO_DISCHARGE);
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
	handles.hWindows[GUI_WINDOW_UPDATE] = FwUpdateCreate();
	handles.hWindows[GUI_WINDOW_STATUS] = MainStatusCreate(&kickerDischarge, GUI_WINDOW_BOT_ID);
	handles.hWindows[GUI_WINDOW_WIFI_SETTINGS] = WifiCreate();
	handles.hWindows[GUI_WINDOW_BOT_ID] = SelectBotIDCreate(GUI_WINDOW_STATUS);  // switch back to status window on selection
	handles.hWindows[GUI_WINDOW_LOGGING] = LoggingCreate();
	handles.hWindows[GUI_WINDOW_TEST_KICKER] = TestKickerCreate();
	handles.hWindows[GUI_WINDOW_TEST_DRIVE] = TestDriveCreate();
	handles.hWindows[GUI_WINDOW_BALL_VIEW] = BallViewCreate();
	handles.hWindows[GUI_WINDOW_VERSIONS] = VersionsCreate();
	handles.hWindows[GUI_WINDOW_IR] = IrGuiCreate();
	handles.hWindows[GUI_WINDOW_IMU] = ImuCreate();
	handles.hWindows[GUI_WINDOW_DRIBBLER] = DribblerCreate();

	handles.hMenu = MenuCreate(handles.hWindows, 12);

	GListener gl;

	geventListenerInit(&gl);
	gwinAttachListener(&gl);

	TopBarSetTitle(((GWidgetObject*)handles.hWindows[currentWindow])->text);
	gwinSetVisible(handles.hWindows[currentWindow], TRUE);

	while(1)
	{
#ifndef ENV_BOOT
		if(handles.takeScreenshot)
		{
			handles.takeScreenshot = 0;

			takeScreenshot();
		}
#endif

		GEvent* pEvent = geventEventWait(&gl, 40);

		if(pEvent == 0)
		{
			MainStatusConfigNetworkUpdate(&network.config);

			PresenterWifiStat wifiStat;
			wifiStat.linkDisabled = network.linkDisabled;
			wifiStat.bsOnline = robot.bsOnline;
			wifiStat.sumatraOnline = robot.sumatraOnline;
			wifiStat.updateFreq = 1e6f/tigerBot.radioBot.stats.baseCycleTime_us;
			wifiStat.visionDelay = robot.sensors.vision.delay;
			wifiStat.rssi = tigerBot.radioBot.fromBase[tigerBot.radioBot.data.clientId].avgRxRssi_mdBm * 0.001f;

			const Kicker* pKicker = &tigerBot.kicker;
			const McuDribblerMeasurement* pIr = &devDribIr.meas;

			PresenterMainStatus stat;
			stat.barrier.level[0] = pIr->barrierOn_V*1000.0f;
			stat.barrier.level[1] = pIr->barrierOff_V*1000.0f;
			stat.barrier.threshold = pKicker->barrier.irAutoThreshold*1000.0f;
			stat.barrier.txDamaged = pKicker->errorFlags & KICKER_ERROR_IR_TX_DMG;
			stat.barrier.rxDamaged = pKicker->errorFlags & KICKER_ERROR_IR_RX_DMG;
			stat.power.bat = tigerBot.powerControl.vBat;
			stat.power.min = tigerBot.powerControl.batCells*tigerBot.powerControl.cellMin;
			stat.power.max = tigerBot.powerControl.batCells*tigerBot.powerControl.cellMax;
			stat.power.cur = tigerBot.powerControl.iCur;
			stat.power.usbPowered = tigerBot.powerControl.state == POWER_CONTROL_STATE_USB_POWERED;
			stat.kicker.cap = pKicker->charger.capAvgFast.value;
			stat.kicker.max = pKicker->pConfig->maxVoltage_V;
			stat.kicker.chg = 0.0f;
			stat.kicker.temp = pKicker->charger.boardTemp_degC;
			stat.pExtUpdateProgress = &tigerBot.robotPi.updateProgress;
			MainStatusKdUpdate(&stat);
			MainStatusHwIdUpdate(RobotImplGetHardwareId());
			MainStatusWifiUpdate(&wifiStat);

			// Wifi
			WifiUpdateConfig(&network.config);
			WifiUpdateStatus(&wifiStat);

			//Ball Detector
			BallViewUpdateCamStats(&tigerBot.robotPi.cameraStats);
			BallViewUpdateDetections(&tigerBot.robotPi.ballDetections);

			//Versions
			VersionsUpdateMotor(0, &devMotors.mcu[0].flashResult);
			VersionsUpdateIr(&devDribIr.flashResult);
			VersionsUpdateExt(&tigerBot.robotPi.robotPiVersion, devRaspberryPi.isInstalled);

			if(tigerBot.powerControl.state == POWER_CONTROL_STATE_USB_POWERED)
			{
				MenuSetEnabled(GUI_WINDOW_TEST_DRIVE, FALSE);
				MenuSetEnabled(GUI_WINDOW_TEST_KICKER, FALSE);
			}
			else
			{
				MenuSetEnabled(GUI_WINDOW_TEST_DRIVE, TRUE);
				MenuSetEnabled(GUI_WINDOW_TEST_KICKER, TRUE);
			}

			FwUpdateAvailableSources(usbMsc.fatFs.fs_type, sdCard.fatFs.fs_type);

			// Logging
			LoggingLogUpdate(&logFile.event);

			// IR Array
			IrGuiData irData;
			irData.filteredBallPosValid = robot.state.ballState > 0;
			irData.filteredBallPos_m[0] = robot.state.ballPos[0];
			irData.filteredBallPos_m[1] = robot.state.ballPos[1];
			irData.irBallDetected = robot.sensors.ir.ballDetected;
			irData.irEstimatedBallPosition_mm[0] = robot.sensors.ir.estimatedBallPosition[0];
			irData.irEstimatedBallPosition_mm[1] = robot.sensors.ir.estimatedBallPosition[1];
			memcpy(irData.vLateral, devDribIr.vLateral, sizeof(irData.vLateral));

			IrGuiUpdate(&irData);

			// IMU
			ImuUpdate(devImu.meas.acc_mDs2, devImu.meas.gyr_radDs, devMag.meas.strength_uT, devImu.meas.temp_degC, devMag.meas.temp_degC);

			// Dribbler
			DribblerUpdate(&devMotors.mcu[4], &devDribIr, &robot.state, &robot.ctrlRef);

			FwUpdateStatus(&tigerBot.fwUpdater.progress);

			continue;
		}

		if(TopBarIsMenuClicked(pEvent))
		{
			gwinSetVisible(handles.hTopBar, FALSE);
			gwinSetVisible(handles.hWindows[currentWindow], FALSE);

			gwinSetVisible(handles.hMenu, TRUE);

			continue;
		}

		int16_t selectedFwSource = FwUpdateGetBtnClick(pEvent);
		if(selectedFwSource >= 0)
		{
			switch(selectedFwSource)
			{
				case GUI_FW_UPDATE_BTN_USB:
					FwUpdaterLoad(&tigerBot.fwUpdater, &tigerBot.fwLoaderUsb.source);
					break;
				case GUI_FW_UPDATE_BTN_SDCARD:
					FwUpdaterLoad(&tigerBot.fwUpdater, &tigerBot.fwLoaderSdCard.source);
					break;
				default:
					break;
			}

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
	size_t fragments = chHeapStatus(&guiHeap, &free, 0);

	printf("Fragments: %u, Free: %u, Core: %u\r\n", fragments, free, chCoreGetStatusX());
}

void PresenterSendPreviewLineToDisplay(ExtCameraPreviewLine160* pLine)
{
	// stream directly to display
	gfxMutexEnter(&GDISP->mutex);

	*TFT_CMD_REG = 0x2A;
	*TFT_DATA_REG = 0;
	*TFT_DATA_REG = 0;
	*TFT_DATA_REG = 0;
	*TFT_DATA_REG = 239;

	uint16_t y = pLine->row + 200;

	*TFT_CMD_REG = 0x2B;
	*TFT_DATA_REG = y >> 8;
	*TFT_DATA_REG = y & 0xFF;
	*TFT_DATA_REG = (y+1) >> 8;
	*TFT_DATA_REG = (y+1) & 0xFF;

	*TFT_CMD_REG = 0x2C;

	for(uint16_t x = 0; x < 160; x++)
		*TFT_DATA_REG = pLine->data[x];

	gfxMutexExit(&GDISP->mutex);
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
			printf("Cannot open file: %s   Error: %u\r\n", filename, fresult);
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

	printf("Screenshot taken: %s (took %.3fs)\r\n", filename, timeTaken);
}

static bool_t mouseGetXYZ(GMouse* m, GMouseReading* pdr)
{
	(void)m;

	// No buttons
	pdr->buttons = 0;
	pdr->z = 0;

	TouchAD7843Measurement touch;
	TouchAD7843Get(&devTouch, &touch);

	if(touch.pressed)
	{
		pdr->z = 1;
		pdr->x = touch.x;
		pdr->y = touch.y;
	}

	return TRUE;
}

static void calSave(GMouse *m, const void *buf, size_t sz)
{
	(void)m;
	(void)sz;

	float* pCalib = (float*)buf;

	for(uint8_t i = 0; i < 6; i++)
		printf("%.8f\r\n", pCalib[i]);
}

static const float calib[] = {
	0.00114166f, 0.06780271f, -19.17998314f,
	0.09104055f, 0.00010331f, -15.59686184f,
};

static bool_t calLoad(GMouse *m, void *buf, size_t sz)
{
	(void)m;

	if(sz != sizeof(calib))
		return FALSE;

	memcpy(buf, calib, sz);

	return TRUE;
}

static bool_t mouseInit(GMouse* m, unsigned driverinstance)
{
	(void)m;
	(void)driverinstance;

	return TRUE;
}

// Resolution and Accuracy Settings
#define GMOUSE_ADS7843_PEN_CALIBRATE_ERROR		8
#define GMOUSE_ADS7843_PEN_CLICK_ERROR			6
#define GMOUSE_ADS7843_PEN_MOVE_ERROR			4
#define GMOUSE_ADS7843_FINGER_CALIBRATE_ERROR	14
#define GMOUSE_ADS7843_FINGER_CLICK_ERROR		18
#define GMOUSE_ADS7843_FINGER_MOVE_ERROR		14

const GMouseVMT const GMOUSE_DRIVER_VMT[1] = {{
	{
		GDRIVER_TYPE_TOUCH,
		GMOUSE_VFLG_TOUCH | GMOUSE_VFLG_CALIBRATE | /*GMOUSE_VFLG_CAL_TEST |*/ GMOUSE_VFLG_ONLY_DOWN | /*GMOUSE_VFLG_POORUPDOWN |*/ GMOUSE_VFLG_CAL_EXTREMES,
		sizeof(GMouse),
		_gmouseInitDriver,
		_gmousePostInitDriver,
		_gmouseDeInitDriver
	},
	1,				// z_max - (currently?) not supported
	0,				// z_min - (currently?) not supported
	1,				// z_touchon
	0,				// z_touchoff
	{				// pen_jitter
		GMOUSE_ADS7843_PEN_CALIBRATE_ERROR,			// calibrate
		GMOUSE_ADS7843_PEN_CLICK_ERROR,				// click
		GMOUSE_ADS7843_PEN_MOVE_ERROR				// move
	},
	{				// finger_jitter
		GMOUSE_ADS7843_FINGER_CALIBRATE_ERROR,		// calibrate
		GMOUSE_ADS7843_FINGER_CLICK_ERROR,			// click
		GMOUSE_ADS7843_FINGER_MOVE_ERROR			// move
	},
	mouseInit,	 	// init
	0,				// deinit
	mouseGetXYZ,	// get
	calSave,		// calsave
	calLoad			// calload
}};
