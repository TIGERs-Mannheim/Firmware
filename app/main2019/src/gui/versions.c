/*
 * versions.c
 *
 *  Created on: 01.07.2019
 *      Author: AndreR
 */

#include "versions.h"
#include "main/version.h"
#include "util/boot.h"
#include <stdio.h>

static struct _handles
{
	GHandle hMainVersion;
	GHandle hMotorVersion;
	GHandle hIrVersion;
	GHandle hExtVersion;
	GHandle hExtBuildDate;
} handles;

static GHandle createLabel(coord_t x, coord_t y, coord_t w, coord_t h, GHandle parent, const char* pText)
{
	GWidgetInit deviceLabelInit = { { x, y, w, h, TRUE, parent }, pText, 0, 0, 0 };
	return gwinLabelCreate(0, &deviceLabelInit);
}

GHandle VersionsCreate()
{
    GWidgetInit wi = { { 0, 60, 240, 260, FALSE, 0 }, "Versions", 0, 0, 0 };
    GHandle hTop = gwinContainerCreate(0, &wi, 0);

    // Main
	uint32_t version = VersionGet();
	uint32_t crc = BootGetApplicationCRC32();

	static char mainBuf[24];
	snprintf(mainBuf, 24, "%5u.%-5u (%08X)", version >> 16, version & 0xFFFF, crc);

    createLabel(5, 5, 70, 20, hTop, "Main:");
    handles.hMainVersion = createLabel(60, 5, 180, 20, hTop, mainBuf);

    // Sub-Processors
    createLabel(5, 25, 80, 20, hTop, "Motor:");
    handles.hMotorVersion = createLabel(60, 25, 180, 20, hTop, "?");

    createLabel(5, 45, 80, 20, hTop, "IR:");
    handles.hIrVersion = createLabel(60, 45, 180, 20, hTop, "?");

    // Ext
    createLabel(5, 100, 80, 15, hTop, "Ext:");
    handles.hExtVersion = createLabel(60, 100, 180, 15, hTop, "?");
    handles.hExtBuildDate = createLabel(60, 120, 180, 15, hTop, "?");

    return hTop;
}

void VersionsUpdateExt(ExtRobotPiVersion* pRobotPiVersion, uint8_t installed)
{
	static char versionBuf[12];

	if(installed)
	{
		snprintf(versionBuf, 12, "%5hu.%-5hu", (uint16_t)pRobotPiVersion->major, (uint16_t)pRobotPiVersion->minor);
		gwinSetText(handles.hExtVersion, versionBuf, FALSE);
		gwinSetText(handles.hExtBuildDate, pRobotPiVersion->date, FALSE);
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
