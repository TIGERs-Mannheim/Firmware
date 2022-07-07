/*
 * wifi_settings.c
 *
 *  Created on: 09.01.2019
 *      Author: AndreR
 */

#include "wifi.h"
#include "robot/network.h"
#include "robot/robot.h"
#include <stdio.h>

static struct _handles
{
	GHandle hShowCont;
	GHandle hChannel;
	GHandle hBtnChannel;
	GHandle hBSStatus;
	GHandle hSumatraStatus;
	GHandle hBSRate;
	GHandle hVisionDelay;
	GHandle hRSSI;

	GHandle hEnableCont;
	GHandle hBtnEnable;

	GHandle hChannelCont;
	GHandle hNewChannel;
	GHandle hBtnOK;
	GHandle hBtnDel;
	GHandle hNumpad[10];

	char chEditBuf[5];
	uint8_t chEditPos;
} handles;

static const char* pNumbers[] = {"1", "2", "3", "4", "5", "6", "7", "8", "9"};

static uint8_t getNewChannel()
{
	uint16_t ch = 0;

	sscanf(handles.chEditBuf, "%hu", &ch);

	if(ch > 255)
		ch = 255;

	return ch;
}

static int16_t eventHandler(GEvent* pEvent)
{
	if(pEvent->type == GEVENT_GWIN_BUTTON)
	{
		GEventGWinButton* pBtn = (GEventGWinButton*)pEvent;

		if(pBtn->gwin == handles.hBtnEnable)
		{
			RobotEnableNetwork();
		}

		if(pBtn->gwin == handles.hBtnChannel)
		{
			gwinSetVisible(handles.hShowCont, FALSE);
			gwinSetVisible(handles.hChannelCont, TRUE);
		}

		if(pBtn->gwin == handles.hBtnOK)
		{
			gwinSetVisible(handles.hChannelCont, FALSE);
			gwinSetVisible(handles.hShowCont, TRUE);
		}

		if(pBtn->gwin == handles.hBtnOK)
		{
			network.config.channel = getNewChannel();
			NetworkSaveConfig();
		}

		if(pBtn->gwin == handles.hBtnDel)
		{
			if(handles.chEditPos > 0)
			{
				--handles.chEditPos;
				handles.chEditBuf[handles.chEditPos] = ' ';
				gwinRedraw(handles.hNewChannel);
			}
		}

		for(uint8_t i = 0; i < 10; i++)
		{
			if(pBtn->gwin == handles.hNumpad[i])
			{
				if(handles.chEditPos < 3)
				{
					handles.chEditBuf[handles.chEditPos] = 48+i;
					++handles.chEditPos;
					gwinRedraw(handles.hNewChannel);
				}

				break;
			}
		}
	}

	return -1;
}

GHandle WifiCreate()
{
	font_t deja16 = gdispOpenFont("DejaVuSans16");

    GWidgetInit wi = { { 0, 60, 240, 260, FALSE, 0 }, "Wifi", 0, &eventHandler, 0 };
    GHandle hTop = gwinContainerCreate(0, &wi, 0);

    // Container for current settings
    GWidgetInit showContInit = { { 0, 0, 240, 260, FALSE, hTop }, 0, 0, 0, 0 };
    handles.hShowCont = gwinContainerCreate(0, &showContInit, 0);

    GWidgetInit channelLabelInit = { { 5, 70, 70, 50, TRUE, handles.hShowCont }, "Channel:", 0, 0, 0 };
    gwinLabelCreate(0, &channelLabelInit);
    GWidgetInit channelInit = { { 80, 70, 60, 50, TRUE, handles.hShowCont }, handles.chEditBuf, 0, 0, 0 };
    handles.hChannel = gwinLabelCreate(0, &channelInit);
    GWidgetInit btnChannelInit = { { 150, 70, 80, 50, TRUE, handles.hShowCont }, "Change", 0, 0, 0 };
    handles.hBtnChannel = gwinButtonCreate(0, &btnChannelInit);

    GWidgetInit bsLabelInit = { { 5, 140, 100, 15, TRUE, handles.hShowCont }, "Base Station:", 0, 0, 0 };
    gwinLabelCreate(0, &bsLabelInit);
    GWidgetInit bsInit = { { 110, 140, 100, 15, TRUE, handles.hShowCont }, "Offline", 0, 0, 0 };
    handles.hBSStatus = gwinLabelCreate(0, &bsInit);

    GWidgetInit sumatraLabelInit = { { 5, 160, 100, 15, TRUE, handles.hShowCont }, "Sumatra:", 0, 0, 0 };
    gwinLabelCreate(0, &sumatraLabelInit);
    GWidgetInit sumatraInit = { { 110, 160, 100, 15, TRUE, handles.hShowCont }, "Offline", 0, 0, 0 };
    handles.hSumatraStatus = gwinLabelCreate(0, &sumatraInit);

    GWidgetInit bsRateLabelInit = { { 5, 180, 100, 15, TRUE, handles.hShowCont }, "BS Rate:", 0, 0, 0 };
    gwinLabelCreate(0, &bsRateLabelInit);
    GWidgetInit bsRateInit = { { 110, 180, 100, 15, TRUE, handles.hShowCont }, "0", 0, 0, 0 };
    handles.hBSRate = gwinLabelCreate(0, &bsRateInit);

    GWidgetInit visionDelayLabelInit = { { 5, 200, 100, 15, TRUE, handles.hShowCont }, "Vision Delay:", 0, 0, 0 };
    gwinLabelCreate(0, &visionDelayLabelInit);
    GWidgetInit visionDelayInit = { { 110, 200, 100, 15, TRUE, handles.hShowCont }, "0", 0, 0, 0 };
    handles.hVisionDelay = gwinLabelCreate(0, &visionDelayInit);

    GWidgetInit rssiLabelInit = { { 5, 220, 100, 15, TRUE, handles.hShowCont }, "RSSI:", 0, 0, 0 };
    gwinLabelCreate(0, &rssiLabelInit);
    GWidgetInit rssiInit = { { 110, 220, 100, 15, TRUE, handles.hShowCont }, "-100", 0, 0, 0 };
    handles.hRSSI = gwinLabelCreate(0, &rssiInit);

    font_t oldFont = gwinGetDefaultFont();
    gwinSetDefaultFont(deja16);

    // Container to display enable button
    GWidgetInit enableContInit = { { 0, 0, 240, 260, TRUE, hTop }, 0, 0, 0, 0 };
    handles.hEnableCont = gwinContainerCreate(0, &enableContInit, 0);

    GWidgetInit disabledLabelInit = { { 70, 10, 140, 50, TRUE, handles.hEnableCont }, "Wifi disabled", 0, 0, 0 };
    gwinLabelCreate(0, &disabledLabelInit);

    GWidgetInit btnEnabledInit = { { 10, 60, 220, 50, TRUE, handles.hEnableCont }, "Enable", 0, 0, 0 };
    handles.hBtnEnable = gwinButtonCreate(0, &btnEnabledInit);

    GWidgetInit hint1LabelInit = { { 35, 200, 180, 15, TRUE, handles.hEnableCont }, "Wifi is auto-enabled if", 0, 0, 0 };
    gwinLabelCreate(0, &hint1LabelInit);
    GWidgetInit hint2LabelInit = { { 45, 220, 180, 15, TRUE, handles.hEnableCont }, "a cover is detected", 0, 0, 0 };
    gwinLabelCreate(0, &hint2LabelInit);

    // Container to change channel (via numpad)
    GWidgetInit channelContInit = { { 0, 0, 240, 260, FALSE, hTop }, 0, 0, 0, 0 };
    handles.hChannelCont = gwinContainerCreate(0, &channelContInit, 0);

    GWidgetInit newChLabelInit = { { 70, 10, 140, 15, TRUE, handles.hChannelCont }, "New Channel", 0, 0, 0 };
    gwinLabelCreate(0, &newChLabelInit);
    GWidgetInit newChInit = { { 100, 40, 70, 15, TRUE, handles.hChannelCont }, handles.chEditBuf, 0, 0, 0 };
    handles.hNewChannel = gwinLabelCreate(0, &newChInit);

    GWidgetInit btnZeroInit = { { 185, 145, 50, 50, TRUE, handles.hChannelCont }, "0", 0, 0, 0 };
    handles.hNumpad[0] = gwinButtonCreate(0, &btnZeroInit);

    for(uint8_t i = 0; i < 9; i++)
    {
    	uint8_t row = i/3;
    	uint8_t col = i%3;

    	coord_t x = 5 + col*60;
    	coord_t y = 85 + row*60;

    	GWidgetInit btnInit = { { x, y, 50, 50, TRUE, handles.hChannelCont }, pNumbers[i], 0, 0, 0 };
    	handles.hNumpad[i+1] = gwinButtonCreate(0, &btnInit);
    }

    GWidgetInit btnOKInit = { { 185, 205, 50, 50, TRUE, handles.hChannelCont }, "OK", 0, 0, 0 };
    handles.hBtnOK = gwinButtonCreate(0, &btnOKInit);

    GWidgetInit btnDelInit = { { 185, 85, 50, 50, TRUE, handles.hChannelCont }, "<=", 0, 0, 0 };
    handles.hBtnDel = gwinButtonCreate(0, &btnDelInit);

    gwinSetDefaultFont(oldFont);

    return hTop;
}

void WifiUpdateConfig(ConfigNetwork* pConfig)
{
	if(gwinGetVisible(handles.hChannelCont) == FALSE)
	{
		snprintf(handles.chEditBuf, 5, "%3hu", (uint16_t)pConfig->channel);
		gwinRedraw(handles.hChannel);
		gwinRedraw(handles.hNewChannel);
		handles.chEditPos = 3;
	}
}

void WifiUpdateStatus(PresenterWifiStat* pStat)
{
	static char vis[10];
	static char bs[10];
	static char rssi[10];

	if(pStat->bsOnline)
		gwinSetText(handles.hBSStatus, "Online", FALSE);
	else
		gwinSetText(handles.hBSStatus, "Offline", FALSE);

	if(pStat->sumatraOnline)
		gwinSetText(handles.hSumatraStatus, "Online", FALSE);
	else
		gwinSetText(handles.hSumatraStatus, "Offline", FALSE);

	snprintf(vis, 10, "%hu", (uint16_t)pStat->visionDelay);
	gwinSetText(handles.hVisionDelay, vis, FALSE);

	snprintf(bs, 10, "%hu", (uint16_t)pStat->updateFreq);
	gwinSetText(handles.hBSRate, bs, FALSE);

	snprintf(rssi, 10, "%3.2fdBm", pStat->rssi);
	gwinSetText(handles.hRSSI, rssi, FALSE);

	if(gwinGetVisible(handles.hEnableCont) == TRUE && pStat->linkDisabled == 0)
	{
		gwinSetVisible(handles.hEnableCont, FALSE);
		gwinSetVisible(handles.hShowCont, TRUE);
	}

	if(gwinGetVisible(handles.hEnableCont) == FALSE && pStat->linkDisabled)
	{
		gwinSetVisible(handles.hShowCont, FALSE);
		gwinSetVisible(handles.hEnableCont, TRUE);
	}
}
