/*
 * logging.c
 *
 *  Created on: 26.04.2019
 *      Author: AndreR
 */

#include "logging.h"
#include <stdio.h>

static struct _handles
{
	GHandle hClass;
	GHandle hClassCode;
	GHandle hManufacturer;
	GHandle hProduct;
	GHandle hSerial;
	GHandle hFatSize;
	GHandle hFreeSize;
	GHandle hClusterSize;
	GHandle hFilename;
	GHandle hSize;
	GHandle hBytesIn;
	GHandle hBytesWritten;
	GHandle hBytesLost;
	GHandle hWriteSpeed;
	GHandle hBtnCloseLog;
} handles;

static int16_t eventHandler(GEvent* pEvent)
{
	if(pEvent->type == GEVENT_GWIN_BUTTON)
	{
		GEventGWinButton* pBtn = (GEventGWinButton*)pEvent;

		if(pBtn->gwin == handles.hBtnCloseLog)
		{
			LogFileClose();
		}
	}

	return -1;
}

static GHandle createLabel(coord_t x, coord_t y, coord_t w, coord_t h, GHandle parent, const char* pText)
{
	GWidgetInit deviceLabelInit = { { x, y, w, h, TRUE, parent }, pText, 0, 0, 0 };
	return gwinLabelCreate(0, &deviceLabelInit);
}

GHandle LoggingCreate()
{
    GWidgetInit wi = { { 0, 60, 240, 260, FALSE, 0 }, "Logging", 0, &eventHandler, 0 };
    GHandle hTop = gwinContainerCreate(0, &wi, 0);

    font_t curFont = gwinGetDefaultFont();
    gwinSetDefaultFont(gdispOpenFont("DejaVuSans10"));

    // HCD
    createLabel(5, 5, 70, 15, hTop, "USB Device:");
    handles.hClass = createLabel(80, 5, 160, 15, hTop, "dev");

    createLabel(180, 5, 50, 15, hTop, "Class:");
    handles.hClassCode = createLabel(210, 5, 60, 15, hTop, "42");

    createLabel(5, 20, 70, 15, hTop, "Manufacturer:");
    handles.hManufacturer = createLabel(80, 20, 160, 15, hTop, "?");

    createLabel(5, 35, 70, 15, hTop, "Product:");
    handles.hProduct = createLabel(80, 35, 160, 15, hTop, "prod");

    createLabel(5, 50, 70, 15, hTop, "Serial:");
    handles.hSerial = createLabel(80, 50, 160, 15, hTop, "012345");

    // MSC
    createLabel(5, 65, 70, 15, hTop, "FAT Size:");
    handles.hFatSize = createLabel(80, 65, 160, 15, hTop, "42 MB");

    createLabel(5, 80, 70, 15, hTop, "Free:");
    handles.hFreeSize = createLabel(80, 80, 160, 15, hTop, "43 MB");

    createLabel(5, 95, 70, 15, hTop, "Cluster Size:");
    handles.hClusterSize = createLabel(80, 95, 160, 15, hTop, "16k");

    // LOG
    createLabel(5, 125, 70, 15, hTop, "Logfile:");
    handles.hFilename = createLabel(80, 125, 160, 15, hTop, "N/A");

    createLabel(5, 140, 70, 15, hTop, "Filesize:");
    handles.hSize = createLabel(80, 140, 160, 15, hTop, "0");

    createLabel(5, 155, 70, 15, hTop, "Data In:");
    handles.hBytesIn = createLabel(80, 155, 50, 15, hTop, "0");

    createLabel(140, 155, 50, 15, hTop, "Data Out:");
    handles.hBytesWritten = createLabel(195, 155, 50, 15, hTop, "0");

    createLabel(5, 170, 70, 15, hTop, "Write Speed:");
    handles.hWriteSpeed = createLabel(80, 170, 50, 15, hTop, "0");

    createLabel(140, 170, 50, 15, hTop, "Data Loss:");
    handles.hBytesLost = createLabel(195, 170, 50, 15, hTop, "0");

    GWidgetInit closeInit = { { 5, 190, 230, 30, TRUE, hTop }, "Close Logfile", 0, 0, 0 };
    handles.hBtnCloseLog = gwinButtonCreate(0, &closeInit);

    gwinSetDefaultFont(curFont);

    return hTop;
}

void LoggingHCDUpdate(USBHCDConnectionEvent* pConEvent)
{
	static char buf[16];

	snprintf(buf, 16, "%hu", (uint16_t)pConEvent->classCode);
	gwinSetText(handles.hClassCode, buf, FALSE);

	gwinSetText(handles.hClass, pConEvent->pName, FALSE);
	gwinSetText(handles.hManufacturer, pConEvent->pManufacturer, FALSE);
	gwinSetText(handles.hProduct, pConEvent->pProduct, FALSE);
	gwinSetText(handles.hSerial, pConEvent->pSerialNumber, FALSE);
}

void LoggingMSCUpdate(USBMSCEvent* pEvent)
{
	static char fat[16];
	static char free[16];
	static char cluster[16];

	snprintf(fat, 16, "%u MB", pEvent->fatSize);
	gwinSetText(handles.hFatSize, fat, FALSE);

	snprintf(free, 16, "%u MB", pEvent->freeSize);
	gwinSetText(handles.hFreeSize, free, FALSE);

	snprintf(cluster, 16, "%u kB", pEvent->clusterSize);
	gwinSetText(handles.hClusterSize, cluster, FALSE);
}

void LoggingLogUpdate(LogFileEvent* pLogEvent)
{
	static char size[16];
	static char in[16];
	static char out[16];
	static char loss[16];
	static char writeSpeed[16];

	gwinSetText(handles.hFilename, pLogEvent->pFilename, FALSE);

	if(pLogEvent->fileSize < 1024*1024*10)
		snprintf(size, 16, "%u kB", pLogEvent->fileSize/1024);
	else
		snprintf(size, 16, "%u MB", pLogEvent->fileSize/(1024*1024));

	gwinSetText(handles.hSize, size, FALSE);

	snprintf(in, 16, "%u kB/s", pLogEvent->bytesPerSecIn/1024);
	gwinSetText(handles.hBytesIn, in, FALSE);

	snprintf(out, 16, "%u kB/s", pLogEvent->bytesPerSecWritten/1024);
	gwinSetText(handles.hBytesWritten, out, FALSE);

	snprintf(writeSpeed, 16, "%u kB/s", pLogEvent->writeSpeed/1024);
	gwinSetText(handles.hWriteSpeed, writeSpeed, FALSE);

	snprintf(loss, 16, "%u kB/s", pLogEvent->bytesPerSecLost/1024);
	gwinSetText(handles.hBytesLost, loss, FALSE);
}
