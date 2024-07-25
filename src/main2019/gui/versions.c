#include "versions.h"
#include "version.h"
#include "util/test.h"
#include "test_data.h"
#include <stdio.h>
#include <string.h>

static struct _handles
{
	GHandle hMainVersion;
	GHandle hMotorVersion;
	GHandle hIrVersion;
	GHandle hExtVersion;
	GHandle hExtBuildDate;
	GHandle hBtnBatStorage;
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

		if(pBtn->gwin == handles.hBtnBatStorage)
		{
			TestSchedule(TEST_ID_BAT_DISCHARGE, 0, 1);
		}
	}
	return -1;
}

GHandle VersionsCreate()
{
	GWidgetInit wi = { { 0, 60, 240, 260, FALSE, 0 }, "Versions", 0, &eventHandler, 0 };
	GHandle hTop = gwinContainerCreate(0, &wi, 0);

	// Main
	createLabel(5, 5, 70, 20, hTop, "Main:");
	handles.hMainVersion = createLabel(60, 5, 180, 20, hTop, VersionGetString());

	// Sub-Processors
	createLabel(5, 25, 80, 20, hTop, "Motor:");
	handles.hMotorVersion = createLabel(60, 25, 180, 20, hTop, "?");

	createLabel(5, 45, 80, 20, hTop, "IR:");
	handles.hIrVersion = createLabel(60, 45, 180, 20, hTop, "?");

	// Ext
	createLabel(5, 100, 80, 15, hTop, "Ext:");
	handles.hExtVersion = createLabel(60, 100, 180, 15, hTop, "?");
	handles.hExtBuildDate = createLabel(60, 120, 180, 15, hTop, "?");

	GWidgetInit btnStorage = { { 150, 210, 100, 50, TRUE, hTop }, "Bat Storage", 0, 0, 0 };
	handles.hBtnBatStorage = gwinButtonCreate(0, &btnStorage);

	return hTop;
}

void VersionsUpdateExt(ExtRobotPiVersion* pRobotPiVersion, uint8_t installed)
{
	static char versionBuf[32];
	static char dateBuf[32];

	if(installed)
	{
		uint32_t major = pRobotPiVersion->version >> 24;
		uint32_t minor = (pRobotPiVersion->version >> 16) & 0xFF;
		uint32_t patch = (pRobotPiVersion->version >> 8) & 0xFF;
		uint32_t dirty = pRobotPiVersion->version & 0xFF;

		snprintf(versionBuf, 32, "v%u.%u.%u%s", major, minor, patch, dirty ? "-dirty" : "");
		strncpy(dateBuf, pRobotPiVersion->date, sizeof(dateBuf));
		gwinSetText(handles.hExtVersion, versionBuf, FALSE);
		gwinSetText(handles.hExtBuildDate, dateBuf, FALSE);
	}
	else
	{
		gwinSetText(handles.hExtVersion, "not installed", FALSE);
		gwinSetText(handles.hExtBuildDate, "-", FALSE);
	}
	
}

void VersionsUpdateMotor(uint8_t motorId, STBootloaderFlashResult* pFlashResult)
{
	(void)motorId; // not used yet

	static char buf[24];

	snprintf(buf, 24, "%5hu.%-5hu (%08X)", pFlashResult->versionMajor, pFlashResult->versionMinor, pFlashResult->programCrc);
	gwinSetText(handles.hMotorVersion, buf, FALSE);
}

void VersionsUpdateIr(STBootloaderFlashResult* pFlashResult)
{
	static char buf[24];

	snprintf(buf, 24, "%5hu.%-5hu (%08X)", pFlashResult->versionMajor, pFlashResult->versionMinor, pFlashResult->programCrc);
	gwinSetText(handles.hIrVersion, buf, FALSE);
}
