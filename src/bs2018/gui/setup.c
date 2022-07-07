/*
 * setup.c
 *
 *  Created on: 18.11.2017
 *      Author: AndreR
 */

#include <stdio.h>
#include <string.h>

#include "src/gwin/gwin_keyboard_layout.h"
#include "util/console.h"
#include "util/boot.h"
#include "setup.h"

static const GVSpecialKey numpadSKeys[] = {
		{ "\010", "\b", 0, 0 },							// \001 (1)	= Backspace
		{ "\015", "\r", 0, 0 },							// \002 (2)	= Enter 1
};

static const char *numpadSet[] = { "789", "456", "123", "0.\001", 0 };
static const GVKeySet numpadKeySet[] = { numpadSet, 0 };
const GVKeyTable VirtualKeyboard_Numpad = { numpadSKeys, numpadKeySet };

static struct _handles {
	GHandle hChannelEdit;
	GHandle hMaxBotsEdit;
	GHandle hRuntime;

	GHandle hEthIp;
	GHandle hEthPort;
	GHandle hEthMac;

	GHandle hVisionIp;
	GHandle hVisionPort;

	GHandle hKeyboard;

	GHandle hBtnEditSave;
	GHandle hBtnRecalib;

	uint8_t editModeOn;
} handles;

static SetupData data;
static SetupChangedCallback pCallback = 0;

static int16_t eventHandler(GEvent* pEvent)
{
	if(pEvent->type == GEVENT_GWIN_BUTTON)
	{
		GEventGWinButton* pBtn = (GEventGWinButton*)pEvent;

		if(pBtn->gwin == handles.hBtnEditSave)
		{
			if(strncmp(gwinGetText(handles.hBtnEditSave), "Edit", 4) == 0)
			{
				gwinSetText(handles.hBtnEditSave, "Save", FALSE);
				gwinSetVisible(handles.hKeyboard, TRUE);
				gwinEnable(handles.hRuntime);
				handles.editModeOn = 1;
			}
			else
			{
				uint16_t u16;
				uint16_t ip[4];

				if(sscanf(gwinGetText(handles.hChannelEdit), "%hu", &u16) == 1)
					data.wifi.channel = u16;

				if(sscanf(gwinGetText(handles.hMaxBotsEdit), "%hu", &u16) == 1)
					data.wifi.maxBots = u16;

				data.wifi.fixedRuntime = (uint8_t)gwinCheckboxIsChecked(handles.hRuntime);

				if(sscanf(gwinGetText(handles.hEthIp), "%hu.%hu.%hu.%hu", &ip[0], &ip[1], &ip[2], &ip[3]) == 4)
				{
					for(uint8_t i = 0; i < 4; i++)
						data.eth.ip.u8[i] = ip[i];
				}

				if(sscanf(gwinGetText(handles.hEthPort), "%hu", &u16) == 1)
					data.eth.port = u16;

				if(sscanf(gwinGetText(handles.hEthMac), "%hu", &u16) == 1)
					data.eth.mac = u16;

				if(sscanf(gwinGetText(handles.hVisionIp), "%hu.%hu.%hu.%hu", &ip[0], &ip[1], &ip[2], &ip[3]) == 4)
				{
					for(uint8_t i = 0; i < 4; i++)
						data.vision.ip.u8[i] = ip[i];
				}

				if(sscanf(gwinGetText(handles.hVisionPort), "%hu", &u16) == 1)
					data.vision.port = u16;

				if(pCallback)
					(*pCallback)(&data);

				gwinSetText(handles.hBtnEditSave, "Edit", FALSE);
				gwinSetVisible(handles.hKeyboard, FALSE);
				gwinDisable(handles.hRuntime);
				handles.editModeOn = 0;
			}
		}
		else if(pBtn->gwin == handles.hBtnRecalib)
		{
			FlashFSDelete("touch/calib");
			BootReset();
		}
	}

	return -1;
}

void SetupSetCallback(SetupChangedCallback cb)
{
	pCallback = cb;
}

GHandle SetupCreate()
{
	handles.editModeOn = 0;

	GWidgetInit wi = { { 0, 60, 800, 420, FALSE, 0 }, "Setup", 0, &eventHandler, 0 };
	GHandle hTop = gwinContainerCreate(0, &wi, 0);

	GWidgetInit wifiConfig = {{ 5, 5, 225, 155, TRUE, hTop}, 0, 0, 0, 0};
	GHandle hWifiConfig = gwinContainerCreate(0, &wifiConfig, GWIN_CONTAINER_BORDER);

	GWidgetInit wifiTitlelLabelInit = {{5, 5, 200, 20, TRUE, hWifiConfig}, "Wifi Configuration", 0, 0, 0};
	gwinLabelCreate(0, &wifiTitlelLabelInit);

	GWidgetInit wifiChannelLabelInit = {{5, 35, 100, 30, TRUE, hWifiConfig}, "Channel:", 0, 0, 0};
	gwinLabelCreate(0, &wifiChannelLabelInit);

	GWidgetInit wifiChannelInit = {{120, 35, 95, 30, TRUE, hWifiConfig}, 0, 0, 0, 0};
	handles.hChannelEdit = gwinTexteditCreate(0, &wifiChannelInit, 4);

	GWidgetInit wifiMaxBotsLabelInit = {{5, 75, 100, 30, TRUE, hWifiConfig}, "Max. Bots:", 0, 0, 0};
	gwinLabelCreate(0, &wifiMaxBotsLabelInit);

	GWidgetInit wifiMaxBotsInit = {{120, 75, 95, 30, TRUE, hWifiConfig}, "6", 0, 0, 0};
	handles.hMaxBotsEdit = gwinTexteditCreate(0, &wifiMaxBotsInit, 2);

	GWidgetInit wifiRuntimeLabelInit = {{5, 115, 100, 30, TRUE, hWifiConfig}, "Runtime:", 0, 0, 0};
	gwinLabelCreate(0, &wifiRuntimeLabelInit);

	GWidgetInit wifiRuntimeInit = {{120, 115, 95, 30, TRUE, hWifiConfig}, "fixed", 0, 0, 0};
	handles.hRuntime = gwinCheckboxCreate(0, &wifiRuntimeInit);
	gwinDisable(handles.hRuntime);


	GWidgetInit ethConfig = {{ 5, 170, 225, 155, TRUE, hTop}, 0, 0, 0, 0};
	GHandle hEthConfig = gwinContainerCreate(0, &ethConfig, GWIN_CONTAINER_BORDER);

	GWidgetInit ethTitlelLabelInit = {{5, 5, 200, 20, TRUE, hEthConfig}, "Network Configuration", 0, 0, 0};
	gwinLabelCreate(0, &ethTitlelLabelInit);

	GWidgetInit ethIpLabelInit = {{5, 35, 40, 30, TRUE, hEthConfig}, "IP:", 0, 0, 0};
	gwinLabelCreate(0, &ethIpLabelInit);

	GWidgetInit ethIpInit = {{55, 35, 160, 30, TRUE, hEthConfig}, "192.168.100.210", 0, 0, 0};
	handles.hEthIp = gwinTexteditCreate(0, &ethIpInit, 15);

	GWidgetInit ethPortLabelInit = {{5, 75, 40, 30, TRUE, hEthConfig}, "Port:", 0, 0, 0};
	gwinLabelCreate(0, &ethPortLabelInit);

	GWidgetInit ethPortInit = {{55, 75, 160, 30, TRUE, hEthConfig}, "10201", 0, 0, 0};
	handles.hEthPort = gwinTexteditCreate(0, &ethPortInit, 5);

	GWidgetInit ethMacLabelInit = {{5, 115, 40, 30, TRUE, hEthConfig}, "Mac:", 0, 0, 0};
	gwinLabelCreate(0, &ethMacLabelInit);

	GWidgetInit ethMacInit = {{55, 115, 160, 30, TRUE, hEthConfig}, "48", 0, 0, 0};
	handles.hEthMac = gwinTexteditCreate(0, &ethMacInit, 3);


	GWidgetInit visionConfig = {{ 240, 5, 225, 155, TRUE, hTop}, 0, 0, 0, 0};
	GHandle hVisionConfig = gwinContainerCreate(0, &visionConfig, GWIN_CONTAINER_BORDER);

	GWidgetInit visionTitlelLabelInit = {{5, 5, 200, 20, TRUE, hVisionConfig}, "Vision Configuration", 0, 0, 0};
	gwinLabelCreate(0, &visionTitlelLabelInit);

	GWidgetInit visionIpLabelInit = {{5, 35, 40, 30, TRUE, hVisionConfig}, "IP:", 0, 0, 0};
	gwinLabelCreate(0, &visionIpLabelInit);

	GWidgetInit visionIpInit = {{55, 35, 160, 30, TRUE, hVisionConfig}, "224.5.23.2", 0, 0, 0};
	handles.hVisionIp = gwinTexteditCreate(0, &visionIpInit, 15);

	GWidgetInit visionPortLabelInit = {{5, 75, 40, 30, TRUE, hVisionConfig}, "Port:", 0, 0, 0};
	gwinLabelCreate(0, &visionPortLabelInit);

	GWidgetInit visionPortInit = {{55, 75, 160, 30, TRUE, hVisionConfig}, "10002", 0, 0, 0};
	handles.hVisionPort = gwinTexteditCreate(0, &visionPortInit, 5);


	GWidgetInit btnEditSaveInit = {{400, 360, 100, 50, TRUE, hTop}, "Edit", 0, 0, 0};
	handles.hBtnEditSave = gwinButtonCreate(0, &btnEditSaveInit);

	GWidgetInit btnRecalib = {{510, 360, 150, 50, TRUE, hTop}, "Calib Screen", 0, 0, 0};
	handles.hBtnRecalib = gwinButtonCreate(0, &btnRecalib);

	GWidgetInit keyboardInit = {{510, 10, 280, 400, FALSE, hTop}, 0, 0, 0, 0};
	handles.hKeyboard = gwinKeyboardCreate(0, &keyboardInit);
	gwinKeyboardSetLayout(handles.hKeyboard, &VirtualKeyboard_Numpad);

	return hTop;
}

void SetupUpdate(WifiConfig *pWifiConfig, NetworkConfig* pNetworkConfig)
{
	if(handles.editModeOn)
		return;

	data.wifi.channel = pWifiConfig->channel;
	data.wifi.maxBots = pWifiConfig->maxBots;
	data.wifi.fixedRuntime = pWifiConfig->fixedRuntime;
	data.eth.ip = pNetworkConfig->my.ip;
	data.eth.port = pNetworkConfig->my.port;
	data.eth.mac = pNetworkConfig->my.mac.u8[5];
	data.vision.ip = pNetworkConfig->vision.ip;
	data.vision.port = pNetworkConfig->vision.port;

	static char chBuf[4];
	snprintf(chBuf, 4, "%3hu", (uint16_t)pWifiConfig->channel);
	gwinSetText(handles.hChannelEdit, chBuf, FALSE);

	static char maxBotsBuf[4];
	snprintf(maxBotsBuf, 4, "%hu", (uint16_t)pWifiConfig->maxBots);
	gwinSetText(handles.hMaxBotsEdit, maxBotsBuf, FALSE);

	gwinCheckboxCheck(handles.hRuntime, pWifiConfig->fixedRuntime ? TRUE : FALSE);

	static char ethIpBuf[16];
	snprintf(ethIpBuf, 16, "%hu.%hu.%hu.%hu", (uint16_t)pNetworkConfig->my.ip.u8[0], (uint16_t)pNetworkConfig->my.ip.u8[1],
			(uint16_t)pNetworkConfig->my.ip.u8[2], (uint16_t)pNetworkConfig->my.ip.u8[3]);
	gwinSetText(handles.hEthIp, ethIpBuf, FALSE);

	static char ethPortBuf[6];
	snprintf(ethPortBuf, 6, "%hu", pNetworkConfig->my.port);
	gwinSetText(handles.hEthPort, ethPortBuf, FALSE);

	static char ethMacBuf[3];
	snprintf(ethMacBuf, 3, "%hu", (uint16_t)pNetworkConfig->my.mac.u8[5]);
	gwinSetText(handles.hEthMac, ethMacBuf, FALSE);

	static char visionIpBuf[16];
	snprintf(visionIpBuf, 16, "%hu.%hu.%hu.%hu", (uint16_t)pNetworkConfig->vision.ip.u8[0], (uint16_t)pNetworkConfig->vision.ip.u8[1],
			(uint16_t)pNetworkConfig->vision.ip.u8[2], (uint16_t)pNetworkConfig->vision.ip.u8[3]);
	gwinSetText(handles.hVisionIp, visionIpBuf, FALSE);

	static char visionPortBuf[6];
	snprintf(visionPortBuf, 6, "%hu", pNetworkConfig->vision.port);
	gwinSetText(handles.hVisionPort, visionPortBuf, FALSE);
}
