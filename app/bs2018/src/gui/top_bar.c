/*
 * top_bar.c
 *
 *  Created on: 02.11.2017
 *      Author: AndreR
 */

#include "top_bar.h"
#include "gui/icons.h"
#include <time.h>
#include <stdio.h>

static struct _handles
{
	GHandle hMenuButton;
	GHandle hTitle;
	GHandle hTime;
	GHandle hDate;
	GHandle hEthRx;
	GHandle hEthTx;
	GHandle hEthIp;
	GHandle hIconNetwork;
	GHandle hIconWifi;
	GHandle hIconVision;
	GHandle hIconWhistle;
	GHandle hIconLaptop;
	GHandle hChan;
	GHandle hNumBots;
	GHandle hWifiQual;
} handles;

GHandle TopBarCreate()
{
	font_t deja16 = gdispOpenFont("DejaVuSans16");

	GWidgetInit contInit = { { 0, 0, 800, 60, TRUE, 0 }, 0, 0, 0, 0 };
	GHandle hTop = gwinContainerCreate(0, &contInit, 0);

	GWidgetInit menuBtnInit = { { 0, 0, 60, 60, TRUE, hTop }, "Menu", 0, 0, 0 };
	handles.hMenuButton = gwinButtonCreate(0, &menuBtnInit);
	gwinSetFont(handles.hMenuButton, deja16);

	GWidgetInit titleInit = { { 80, 0, 460, 60, TRUE, hTop }, "Overview", 0, 0, 0 };
	handles.hTitle = gwinLabelCreate(0, &titleInit);
	gwinSetFont(handles.hTitle, gdispOpenFont("DejaVuSans32"));

	GWidgetInit timeInit = { { 700, 0, 100, 20, TRUE, hTop }, "12:00:00", 0, 0, 0 };
	handles.hTime = gwinLabelCreate(0, &timeInit);
	gwinSetFont(handles.hTime, gdispOpenFont("DejaVuSans20"));

	GWidgetInit dateInit = { { 700, 20, 100, 20, TRUE, hTop }, "28.11.17", 0, 0, 0 };
	handles.hDate = gwinLabelCreate(0, &dateInit);
	gwinSetFont(handles.hDate, gdispOpenFont("DejaVuSans20"));

	GWidgetInit ethRxInit = { { 600, 0, 100, 20, TRUE, hTop }, "RX: 0.123MB/s", 0, 0, 0 };
	handles.hEthRx = gwinLabelCreate(0, &ethRxInit);
	gwinSetFont(handles.hEthRx, gdispOpenFont("fixed_7x14"));

	GWidgetInit ethTxInit = { { 600, 20, 100, 20, TRUE, hTop }, "TX: 2.456MB/s", 0, 0, 0 };
	handles.hEthTx = gwinLabelCreate(0, &ethTxInit);
	gwinSetFont(handles.hEthTx, gdispOpenFont("fixed_7x14"));

	GWidgetInit ethIpInit = { { 600, 40, 200, 20, TRUE, hTop }, "IP: 127.0.0.1", 0, 0, 0 };
	handles.hEthIp = gwinLabelCreate(0, &ethIpInit);
	gwinSetFont(handles.hEthIp, gdispOpenFont("fixed_7x14"));

	handles.hIconNetwork = IconsCreate(545, 5, hTop, ICON_NETWORK, Gray);

	GWidgetInit qualInit = { { 465, 0, 80, 20, TRUE, hTop }, "Link: 98.0%", 0, 0, 0 };
	handles.hWifiQual = gwinLabelCreate(0, &qualInit);
	gwinSetFont(handles.hWifiQual, gdispOpenFont("fixed_7x14"));

	GWidgetInit botsInit = { { 465, 20, 80, 20, TRUE, hTop }, "Bots: 4", 0, 0, 0 };
	handles.hNumBots = gwinLabelCreate(0, &botsInit);
	gwinSetFont(handles.hNumBots, gdispOpenFont("fixed_7x14"));

	GWidgetInit chanInit = { { 465, 40, 80, 20, TRUE, hTop }, "Chan: 150", 0, 0, 0 };
	handles.hChan = gwinLabelCreate(0, &chanInit);
	gwinSetFont(handles.hChan, gdispOpenFont("fixed_7x14"));

	handles.hIconWifi = IconsCreate(410, 5, hTop, ICON_WIFI_LARGE, Gray);

	handles.hIconVision = IconsCreate(370, 2, hTop, ICON_EYE, Gray);

	handles.hIconWhistle = IconsCreate(329, 30, hTop, ICON_WHISTLE, Red);
	gwinHide(handles.hIconWhistle); // RefBox support not implemented yet

	handles.hIconLaptop = IconsCreate(370, 30, hTop, ICON_LAPTOP, Gray);

	return hTop;
}

uint8_t TopBarIsMenuClicked(GEvent* pEvent)
{
	if(pEvent->type == GEVENT_GWIN_BUTTON)
	{
		GEventGWinButton* pBtn = (GEventGWinButton*)pEvent;
		if(pBtn->gwin == handles.hMenuButton)
		{
			return 1;
		}
	}

	return 0;
}

void TopBarSetTitle(const char* pTitle)
{
	gwinSetText(handles.hTitle, pTitle, FALSE);
}

void TopBarSetTime(uint32_t unixTimestamp)
{
	struct tm* nowtm;
	time_t now = unixTimestamp;
	nowtm = localtime(&now);

	static char timeBuf[10];
	static char dateBuf[10];

	strftime(timeBuf, sizeof(timeBuf), "%H:%M:%S", nowtm);
	strftime(dateBuf, sizeof(dateBuf), "%d.%m.%y", nowtm);

	gwinSetText(handles.hTime, timeBuf, FALSE);
	gwinSetText(handles.hDate, dateBuf, FALSE);
}

void TopBarSetEthStats(uint32_t rx, uint32_t tx)
{
	static uint32_t lastRx = 0;
	static uint32_t lastTx = 0;

	uint32_t rxBytes = rx-lastRx;
	uint32_t txBytes = tx-lastTx;

	lastRx = rx;
	lastTx = tx;

	static char rxBuf[16];
	static char txBuf[16];

	snprintf(rxBuf, 16, "RX: %5ukB/s", rxBytes/1024);
	snprintf(txBuf, 16, "TX: %5ukB/s", txBytes/1024);

	gwinSetText(handles.hEthRx, rxBuf, FALSE);
	gwinSetText(handles.hEthTx, txBuf, FALSE);
}

void TopBarSetEthIP(uint8_t* ip, uint16_t port)
{
	static char ipBuf[32];
	snprintf(ipBuf, 32, "IP: %d.%d.%d.%d:%d", ip[0], ip[1], ip[2], ip[3], port);
	gwinSetText(handles.hEthIp, ipBuf, FALSE);
}

void TopBarSetEthLinkStatus(uint8_t up, uint8_t speed100M)
{
	if(up == 0)
	{
		IconsSetColor(handles.hIconNetwork, Gray);
	}
	else
	{
		IconsSetColor(handles.hIconNetwork, speed100M ? Lime : Yellow);
	}
}

void TopBarSetWifiStatus(uint16_t channel, uint16_t botsOnline, float link)
{
	static char chanBuf[16];
	static char botBuf[16];
	static char linkBuf[16];

	snprintf(linkBuf, 16, "Link: %4.1f%%", link);
	snprintf(botBuf, 16, "Bots: %hu", botsOnline);
	snprintf(chanBuf, 16, "Chan: %hu", channel);

	gwinSetText(handles.hWifiQual, linkBuf, FALSE);
	gwinSetText(handles.hNumBots, botBuf, FALSE);
	gwinSetText(handles.hChan, chanBuf, FALSE);

	IconsSetColor(handles.hIconWifi, botsOnline ? Lime : Gray);
}

void TopBarSetRemotes(uint8_t vision, uint8_t sumatra)
{
	IconsSetColor(handles.hIconVision, vision ? Lime : Gray);
	IconsSetColor(handles.hIconLaptop, sumatra ? Lime : Gray);
}
