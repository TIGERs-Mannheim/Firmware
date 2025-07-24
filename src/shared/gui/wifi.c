#include "wifi.h"
#include "robot/network.h"
#include "robot/robot.h"
#include <stdio.h>

static struct
{
	GHandle hShowCont;
	GHandle hChannel;
	GHandle hScanChannel;
	GHandle hNetMode;
	GHandle hBSStatus;
	GHandle hSumatraStatus;
	GHandle hBSRate;
	GHandle hVisionDelay;
	GHandle hRSSI;

	GHandle hEnableCont;
	GHandle hBtnEnable;
} handles;

static int16_t eventHandler(GEvent* pEvent)
{
	if(pEvent->type == GEVENT_GWIN_BUTTON)
	{
		GEventGWinButton* pBtn = (GEventGWinButton*)pEvent;

		if(pBtn->gwin == handles.hBtnEnable)
		{
			RobotEnableNetwork();
		}

		if(pBtn->gwin == handles.hScanChannel)
		{
			NetworkStartScan();
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

    GWidgetInit channelLabelInit = { { 5, 50, 70, 50, TRUE, handles.hShowCont }, "Channel:", 0, 0, 0 };
    gwinLabelCreate(0, &channelLabelInit);
    GWidgetInit channelInit = { { 80, 50, 60, 50, TRUE, handles.hShowCont }, "???", 0, 0, 0 };
    handles.hChannel = gwinLabelCreate(0, &channelInit);
    GWidgetInit btnScanInit = { { 150, 50, 80, 50, TRUE, handles.hShowCont }, "Scan", 0, 0, 0 };
    handles.hScanChannel = gwinButtonCreate(0, &btnScanInit);

	GWidgetInit modeLabelInit = { { 5, 120, 100, 15, TRUE, handles.hShowCont }, "Mode:", 0, 0, 0 };
	gwinLabelCreate(0, &modeLabelInit);
	GWidgetInit modeInit = { { 110, 120, 100, 15, TRUE, handles.hShowCont }, "Disabled", 0, 0, 0 };
	handles.hNetMode = gwinLabelCreate(0, &modeInit);

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

    gwinSetDefaultFont(oldFont);

    return hTop;
}

void WifiUpdateStatus(PresenterWifiStat* pStat)
{
	static char vis[10];
	static char bs[10];
	static char rssi[10];
	static char channel[4];

	gwinSetText(handles.hNetMode, NetworkGetModeString(pStat->networkMode), FALSE);

	if(pStat->bsOnline)
		gwinSetText(handles.hBSStatus, "Online", FALSE);
	else
		gwinSetText(handles.hBSStatus, "Offline", FALSE);

	if(pStat->sumatraOnline)
		gwinSetText(handles.hSumatraStatus, "Online", FALSE);
	else
		gwinSetText(handles.hSumatraStatus, "Offline", FALSE);

	snprintf(vis, sizeof(vis), "%hu", pStat->visionDelay);
	gwinSetText(handles.hVisionDelay, vis, FALSE);

	snprintf(bs, sizeof(bs), "%hu", pStat->updateFreq);
	gwinSetText(handles.hBSRate, bs, FALSE);

	snprintf(rssi, sizeof(rssi), "%3.2fdBm", pStat->rssi);
	gwinSetText(handles.hRSSI, rssi, FALSE);

	snprintf(channel, sizeof(channel), "%hu", pStat->channel);
	gwinSetText(handles.hChannel, channel, FALSE);

	if(gwinGetVisible(handles.hEnableCont) == TRUE && pStat->networkMode != NETWORK_MODE_DISABLED)
	{
		gwinSetVisible(handles.hEnableCont, FALSE);
		gwinSetVisible(handles.hShowCont, TRUE);
	}

	if(gwinGetVisible(handles.hEnableCont) == FALSE && pStat->networkMode == NETWORK_MODE_DISABLED)
	{
		gwinSetVisible(handles.hShowCont, FALSE);
		gwinSetVisible(handles.hEnableCont, TRUE);
	}
}
