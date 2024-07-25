#include "fw_update.h"
#include <stdio.h>

static struct _handles
{
	GHandle hName;
	GHandle hStatusText;
	GHandle hProg;
	GHandle hBtnUSB;
	GHandle hBtnSDCard;
} handles;

int16_t FwUpdateGetBtnClick(GEvent* pEvent)
{
	if(pEvent->type == GEVENT_GWIN_BUTTON)
	{
		GEventGWinButton* pBtn = (GEventGWinButton*)pEvent;

		if(pBtn->gwin == handles.hBtnUSB)
		{
			return GUI_FW_UPDATE_BTN_USB;
		}
		else if(pBtn->gwin == handles.hBtnSDCard)
		{
			return GUI_FW_UPDATE_BTN_SDCARD;
		}
	}

	return GUI_FW_UPDATE_BTN_NONE;
}

GHandle FwUpdateCreate()
{
    GWidgetInit wi = { { 0, 60, 240, 260, FALSE, 0 }, "Update", 0, 0, 0 };

    GHandle hTop = gwinContainerCreate(0, &wi, 0);

	GWidgetInit progContInit = { { 0, 0, 240, 35, TRUE, hTop }, 0, 0, 0, 0 };
	GHandle hProgCont = gwinContainerCreate(0, &progContInit, 0);

	GWidgetInit nameLabelInit = { { 5, 5, 70, 15, FALSE, hProgCont }, "", 0, 0, 0 };
	handles.hName = gwinLabelCreate(0, &nameLabelInit);

	GWidgetInit statusLabelInit = { { 75, 5, 160, 15, FALSE, hProgCont }, "", 0, 0, 0 };
	handles.hStatusText = gwinLabelCreate(0, &statusLabelInit);

	GWidgetInit progInit = { { 5, 20, 240, 15, FALSE, hProgCont }, 0, 0, 0, 0 };
	handles.hProg = gwinProgressbarCreate(0, &progInit);
	gwinProgressbarSetPosition(handles.hProg, 50);
	gwinSetText(handles.hProg, "42k / 50k", FALSE);

    GWidgetInit sourceLabelInit = { { 5, 175, 230, 15, TRUE, hTop }, "Start update via:", 0, 0, 0 };
    gwinLabelCreate(0, &sourceLabelInit);

	GWidgetInit usbInit = { { 5, 195, 110, 60, TRUE, hTop }, "USB", 0, 0, 0 };
	handles.hBtnUSB = gwinButtonCreate(0, &usbInit);
	gwinSetFont(handles.hBtnUSB, gdispOpenFont("DejaVuSans16"));

	GWidgetInit sdInit = { { 125, 195, 110, 60, TRUE, hTop }, "SD Card", 0, 0, 0 };
	handles.hBtnSDCard = gwinButtonCreate(0, &sdInit);
	gwinSetFont(handles.hBtnSDCard, gdispOpenFont("DejaVuSans16"));

    return hTop;
}

void FwUpdateStatus(const FwUpdaterProgress* pProgress)
{
	static char progBuf[24];

	if(pProgress == 0)
	{
		gwinSetVisible(handles.hName, FALSE);
		gwinSetVisible(handles.hStatusText, FALSE);
		gwinSetVisible(handles.hProg, FALSE);
		return;
	}

	if(pProgress->pStepDescription)
	{
		gwinSetText(handles.hStatusText, pProgress->pStepDescription, FALSE);
		gwinSetVisible(handles.hStatusText, TRUE);

		gwinSetText(handles.hName, "MCU", FALSE);
		gwinSetVisible(handles.hName, TRUE);
	}
	else
	{
		gwinSetVisible(handles.hStatusText, FALSE);
		gwinSetVisible(handles.hName, FALSE);
	}

	if(pProgress->totalBytes == 0)
	{
		gwinSetVisible(handles.hProg, FALSE);
	}
	else
	{
		if(pProgress->totalBytes < 16*1024)
			snprintf(progBuf, sizeof(progBuf), "%u / %u", pProgress->doneBytes, pProgress->totalBytes);
		else
			snprintf(progBuf, sizeof(progBuf), "%uk / %uk", pProgress->doneBytes/1024, pProgress->totalBytes/1024);

		gwinProgressbarSetRange(handles.hProg, 0, pProgress->totalBytes);
		gwinProgressbarSetPosition(handles.hProg, pProgress->doneBytes);
		gwinSetText(handles.hProg, progBuf, FALSE);
		gwinSetVisible(handles.hProg, TRUE);
	}
}

void FwUpdateAvailableSources(uint8_t usb, uint8_t sdCard)
{
	gwinSetEnabled(handles.hBtnUSB, usb);
	gwinSetEnabled(handles.hBtnSDCard, sdCard);
}
