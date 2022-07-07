/*
 * fw_update.c
 *
 *  Created on: 10.04.2020
 *      Author: Andre
 */

#include "fw_update.h"
#include <stdio.h>

#define MAX_PROGS 4

static struct _handles
{
	struct _progs
	{
		GHandle hName;
		GHandle hStatusText;
		GHandle hProg;
	} progs[MAX_PROGS];

	GHandle hBtnWifi;
	GHandle hBtnUSB;
	GHandle hBtnSDCard;
} handles;

static int16_t eventHandler(GEvent* pEvent)
{
	if(pEvent->type == GEVENT_GWIN_BUTTON)
	{
		GEventGWinButton* pBtn = (GEventGWinButton*)pEvent;

		if(pBtn->gwin == handles.hBtnWifi)
		{
			FwUpdaterLoad(FW_UPDATER_SOURCE_WIFI);
		}
		else if(pBtn->gwin == handles.hBtnUSB)
		{
			FwUpdaterLoad(FW_UPDATER_SOURCE_USB);
		}
		else if(pBtn->gwin == handles.hBtnSDCard)
		{
			FwUpdaterLoad(FW_UPDATER_SOURCE_SDCARD);
		}
	}

	return -1;
}

GHandle FwUpdateCreate(uint8_t bootloader)
{
    GWidgetInit wi = { { 0, 60, 240, 260, FALSE, 0 }, "Update", 0, &eventHandler, 0 };

    if(bootloader)
    	wi.customParam = 0;

    GHandle hTop = gwinContainerCreate(0, &wi, 0);

    for(uint8_t i = 0; i < MAX_PROGS; i++)
    {
        GWidgetInit progContInit = { { 0, i*35, 240, 35, TRUE, hTop }, 0, 0, 0, 0 };
        GHandle hProgCont = gwinContainerCreate(0, &progContInit, 0);

        GWidgetInit nameLabelInit = { { 5, 5, 70, 15, FALSE, hProgCont }, "", 0, 0, 0 };
        handles.progs[i].hName = gwinLabelCreate(0, &nameLabelInit);

        GWidgetInit statusLabelInit = { { 75, 5, 160, 15, FALSE, hProgCont }, "", 0, 0, 0 };
        handles.progs[i].hStatusText = gwinLabelCreate(0, &statusLabelInit);

        GWidgetInit progInit = { { 5, 20, 240, 15, FALSE, hProgCont }, 0, 0, 0, 0 };
        handles.progs[i].hProg = gwinProgressbarCreate(0, &progInit);
        gwinProgressbarSetPosition(handles.progs[i].hProg, 50);
        gwinSetText(handles.progs[i].hProg, "42k / 50k", FALSE);
    }

    GWidgetInit sourceLabelInit = { { 5, 175, 230, 15, TRUE, hTop }, "Start update via:", 0, 0, 0 };
    gwinLabelCreate(0, &sourceLabelInit);

	GWidgetInit wifiInit = { { 5, 195, 70, 60, TRUE, hTop }, "Wifi", 0, 0, 0 };
	handles.hBtnWifi = gwinButtonCreate(0, &wifiInit);
	gwinSetFont(handles.hBtnWifi, gdispOpenFont("DejaVuSans16"));

	GWidgetInit usbInit = { { 85, 195, 70, 60, TRUE, hTop }, "USB", 0, 0, 0 };
	handles.hBtnUSB = gwinButtonCreate(0, &usbInit);
	gwinSetFont(handles.hBtnUSB, gdispOpenFont("DejaVuSans16"));

	GWidgetInit sdInit = { { 165, 195, 70, 60, TRUE, hTop }, "SD Card", 0, 0, 0 };
	handles.hBtnSDCard = gwinButtonCreate(0, &sdInit);
	gwinSetFont(handles.hBtnSDCard, gdispOpenFont("DejaVuSans16"));

    return hTop;
}

void FwUpdateStatus(FwUpdaterProgress* pProgressList, uint8_t numPrograms)
{
	static char progBuf[MAX_PROGS][24];

	if(numPrograms > MAX_PROGS)
		numPrograms = MAX_PROGS;

	for(uint8_t prog = 0; prog < MAX_PROGS; prog++)
	{
		if(prog >= numPrograms)
		{
			gwinSetVisible(handles.progs[prog].hName, FALSE);
			gwinSetVisible(handles.progs[prog].hStatusText, FALSE);
			gwinSetVisible(handles.progs[prog].hProg, FALSE);
			continue;
		}

		FwUpdaterProgress* pProgress = &pProgressList[prog];

		if(pProgress->pStepDescription)
		{
			gwinSetText(handles.progs[prog].hStatusText, pProgress->pStepDescription, FALSE);
			gwinSetVisible(handles.progs[prog].hStatusText, TRUE);

			gwinSetText(handles.progs[prog].hName, FwUpdaterGetName(pProgress->procId), FALSE);
			gwinSetVisible(handles.progs[prog].hName, TRUE);
		}
		else
		{
			gwinSetVisible(handles.progs[prog].hStatusText, FALSE);
			gwinSetVisible(handles.progs[prog].hName, FALSE);
		}

		if(pProgress->totalBytes == 0)
		{
			gwinSetVisible(handles.progs[prog].hProg, FALSE);
		}
		else
		{
			if(pProgress->totalBytes < 16*1024)
				snprintf(progBuf[prog], sizeof(progBuf[prog]), "%u / %u", pProgress->doneBytes, pProgress->totalBytes);
			else
				snprintf(progBuf[prog], sizeof(progBuf[prog]), "%uk / %uk", pProgress->doneBytes/1024, pProgress->totalBytes/1024);

			gwinProgressbarSetRange(handles.progs[prog].hProg, 0, pProgress->totalBytes);
			gwinProgressbarSetPosition(handles.progs[prog].hProg, pProgress->doneBytes);
			gwinSetText(handles.progs[prog].hProg, progBuf[prog], FALSE);
			gwinSetVisible(handles.progs[prog].hProg, TRUE);
		}
	}
}

void FwUpdateAvailableSources(uint8_t wifi, uint8_t usb, uint8_t sdCard)
{
	gwinSetEnabled(handles.hBtnWifi, wifi);
	gwinSetEnabled(handles.hBtnUSB, usb);
	gwinSetEnabled(handles.hBtnSDCard, sdCard);
}
