/*
 * main_status.c
 *
 *  Created on: 09.01.2019
 *      Author: AndreR
 */

#include "main_status.h"
#include "gui/bot_cover.h"
#include "gui/styles.h"
#include "version.h"
#include "robot/robot.h"
#include "util/boot.h"
#include <stdio.h>

static struct _handles
{
	GHandle hBotCover;
	GHandle hDischarge;
	GHandle hBatProg;
	GHandle hBatUsb;
	GHandle hCur;
	GHandle hChannel;
	GHandle hRSSI;
	GHandle hBarrier;
	GHandle hBarrier2;
	GHandle hHwId;
	GHandle hCapProg;
	GHandle hCapDisabled;
	GHandle hChg;
	GHandle hCharged;
	GHandle hVersion;
	GHandle hSumatraStatus;

	GTimer flashTimer;

	MainStatusKickerDischargeCallback dischageCallback;
	int16_t idSelectWindowId;
} handles;

static void flashFunc(void* param)
{
	(void)param;

	gwinSetVisible(handles.hCharged, !gwinGetVisible(handles.hCharged));
}

static int16_t eventHandler(GEvent* pEvent)
{
	if(pEvent->type == GEVENT_GWIN_BUTTON)
	{
		GEventGWinButton* pBtn = (GEventGWinButton*)pEvent;

		if(pBtn->gwin == handles.hBotCover)
		{
			return handles.idSelectWindowId;
		}

		if(pBtn->gwin == handles.hDischarge)
		{
			RobotSetIdleMode();
			(*handles.dischageCallback)();
		}
	}

	return -1;
}

GHandle MainStatusCreate(MainStatusKickerDischargeCallback dischageCallback, int16_t idSelectWindowId, uint8_t kickerV2017)
{
	handles.dischageCallback = dischageCallback;
	handles.idSelectWindowId = idSelectWindowId;

	gtimerInit(&handles.flashTimer);

    GWidgetInit wi = { { 0, 60, 240, 260, FALSE, 0 }, "Status", 0, &eventHandler, 0 };
    GHandle hTop = gwinContainerCreate(0, &wi, 0);

    // Power & AI
    GWidgetInit powerContInit = { { 0, 0, 240, 45, TRUE, hTop }, 0, 0, 0, 0 };
    GHandle hPowerCont = gwinContainerCreate(0, &powerContInit, 0);

    GWidgetInit batLabelInit = { { 5, 5, 70, 15, TRUE, hPowerCont }, "Battery:", 0, 0, 0 };
    gwinLabelCreate(0, &batLabelInit);

    GWidgetInit batInit = { { 5, 25, 110, 15, TRUE, hPowerCont }, 0, 0, 0, 0 };
    handles.hBatProg = gwinProgressbarCreate(0, &batInit);
    gwinProgressbarSetPosition(handles.hBatProg, 50);
    gwinSetText(handles.hBatProg, "11.1V", FALSE);

    GWidgetInit batUsbInit = { { 5, 25, 110, 15, FALSE, hPowerCont }, "USB powered", 0, 0, 0 };
    handles.hBatUsb = gwinLabelCreate(0, &batUsbInit);

    GWidgetInit curLabelInit = { { 125, 5, 60, 15, TRUE, hPowerCont }, "Usage:", 0, 0, 0 };
    gwinLabelCreate(0, &curLabelInit);

    GWidgetInit curInit = { { 125, 25, 60, 15, TRUE, hPowerCont }, "4.2W", 0, 0, 0 };
    handles.hCur = gwinLabelCreate(0, &curInit);

    GWidgetInit sumatraStatusInit = { { 200, 15, 30, 15, FALSE, hPowerCont }, "AI", 0, 0, &GreenTextStyle };
    handles.hSumatraStatus = gwinLabelCreate(0, &sumatraStatusInit);

    // Wifi: Channel, ID, RSSI
    GWidgetInit miscContInit = { { 0, 50, 240, 100, TRUE, hTop }, 0, 0, 0, 0 };
    GHandle hMiscCont = gwinContainerCreate(0, &miscContInit, 0);

	GWidgetInit channelLabelInit = { { 5, 5, 70, 15, TRUE, hMiscCont }, "Channel:", 0, 0, 0 };
	gwinLabelCreate(0, &channelLabelInit);
	GWidgetInit channelInit = { { 5, 25, 70, 15, TRUE, hMiscCont }, "0", 0, 0, 0 };
	handles.hChannel = gwinLabelCreate(0, &channelInit);

	GWidgetInit rssiLabelInit = { { 85, 5, 70, 15, TRUE, hMiscCont }, "Wifi:", 0, 0, 0 };
	gwinLabelCreate(0, &rssiLabelInit);
	GWidgetInit rssiInit = { { 85, 25, 70, 15, TRUE, hMiscCont }, "-", 0, 0, 0 };
	handles.hRSSI = gwinLabelCreate(0, &rssiInit);

	GWidgetInit idLabelInit = { { 165, 5, 70, 15, TRUE, hMiscCont }, "ID:", 0, 0, 0 };
	gwinLabelCreate(0, &idLabelInit);
	handles.hBotCover = BotCoverCreate(165, 25, hMiscCont, 11, 0);

	GWidgetInit barrierLabelInit = { { 5, 45, 70, 15, TRUE, hMiscCont }, "Barrier:", 0, 0, 0 };
	gwinLabelCreate(0, &barrierLabelInit);
	GWidgetInit barrierInit = { { 5, 65, 70, 15, TRUE, hMiscCont }, "0/0", 0, 0, 0 };
	handles.hBarrier = gwinLabelCreate(0, &barrierInit);
	GWidgetInit barrier2Init = { { 5, 80, 70, 15, TRUE, hMiscCont }, "-", 0, 0, 0 };
	handles.hBarrier2 = gwinLabelCreate(0, &barrier2Init);

	GWidgetInit hwIdLabelInit = { { 85, 45, 70, 15, TRUE, hMiscCont }, "HW ID:", 0, 0, 0 };
	gwinLabelCreate(0, &hwIdLabelInit);
	GWidgetInit hwIdInit = { { 85, 65, 70, 15, TRUE, hMiscCont }, "255", 0, 0, 0 };
	handles.hHwId = gwinLabelCreate(0, &hwIdInit);

    // Kicker level
	GWidgetInit kickerContInit = { { 0, 140, 240, 120, TRUE, hTop }, 0, 0, 0, 0 };
	GHandle hKickerCont = gwinContainerCreate(0, &kickerContInit, 0);

	GWidgetInit capLabelInit = { { 5, 5, 110, 15, TRUE, hKickerCont }, "Kicker Level:", 0, 0, 0 };
	gwinLabelCreate(0, &capLabelInit);
	GWidgetInit capInit = { { 5, 25, 145, 15, TRUE, hKickerCont }, 0, 0, 0, 0 };
	handles.hCapProg = gwinProgressbarCreate(0, &capInit);
	gwinProgressbarSetRange(handles.hCapProg, 0, 240);
	gwinProgressbarSetPosition(handles.hCapProg, 0);
	gwinSetText(handles.hCapProg, "0.0V", FALSE);

	GWidgetInit capDisabledInit = { { 5, 25, 145, 15, FALSE, hKickerCont }, "disabled", 0, 0, 0 };
	handles.hCapDisabled = gwinLabelCreate(0, &capDisabledInit);

	if(kickerV2017)
	{
		GWidgetInit chgLabelInit = { { 160, 5, 75, 15, TRUE, hKickerCont }, "Temp:", 0, 0, 0 };
		gwinLabelCreate(0, &chgLabelInit);
	}
	else
	{
		GWidgetInit chgLabelInit = { { 160, 5, 75, 15, TRUE, hKickerCont }, "Chg Amps:", 0, 0, 0 };
		gwinLabelCreate(0, &chgLabelInit);
	}
	GWidgetInit chgInit = { { 160, 25, 75, 15, TRUE, hKickerCont }, "25.0C", 0, 0, 0 };
	handles.hChg = gwinLabelCreate(0, &chgInit);

	GWidgetInit chargedInit = { { 20, 50, 200, 20, FALSE, hKickerCont }, ">>> Kicker charged! <<<", 0, 0, 0 };
	handles.hCharged = gwinLabelCreate(0, &chargedInit);
	((GWidgetObject*)handles.hCharged)->pstyle = &RedTextStyle;
	gwinSetFont(handles.hCharged, gdispOpenFont("DejaVuSans16"));

	static char versionBuf[48];
	snprintf(versionBuf, 48, "Firmware: %s", VersionGetString());
	GWidgetInit versionInit = { { 20, 50, 200, 20, TRUE, hKickerCont }, versionBuf, 0, 0, 0 };
	handles.hVersion = gwinLabelCreate(0, &versionInit);

	GWidgetInit disInit = { { 5, 80, 230, 40, TRUE, hKickerCont }, "Discharge", 0, 0, 0 };
	handles.hDischarge = gwinButtonCreate(0, &disInit);
	gwinSetFont(handles.hDischarge, gdispOpenFont("DejaVuSans16"));

	gtimerStart(&handles.flashTimer, &flashFunc, 0, TRUE, 150);

	return hTop;
}

void MainStatusKdUpdate(PresenterMainStatus* pFb)
{
	static char bat[10];
	static char cur[10];
	static char cap[10];
	static char chg[10];
	static char ir[10];
	static char ir2[10];

	if(pFb->power.usbPowered)
	{
		gwinSetText(handles.hCur, "-", FALSE);
		gwinSetText(handles.hChg, "-", FALSE);
		gwinSetVisible(handles.hBatProg, FALSE);
		gwinSetVisible(handles.hBatUsb, TRUE);
		gwinSetVisible(handles.hCapProg, FALSE);
		gwinSetVisible(handles.hDischarge, FALSE);
		gwinSetVisible(handles.hCapDisabled, TRUE);
	}
	else
	{
		snprintf(bat, 10, "%5.2fV", pFb->power.bat);
		gwinProgressbarSetRange(handles.hBatProg, pFb->power.min*1000, pFb->power.max*1000);
		gwinProgressbarSetPosition(handles.hBatProg, pFb->power.bat*1000);
		gwinSetText(handles.hBatProg, bat, FALSE);

		snprintf(cur, 10, "%6.1fW", pFb->power.cur*pFb->power.bat);
		gwinSetText(handles.hCur, cur, FALSE);

		snprintf(cap, 10, "%5.1fV", pFb->kicker.cap);
		gwinProgressbarSetRange(handles.hCapProg, 0, (int)pFb->kicker.max);
		gwinProgressbarSetPosition(handles.hCapProg, (int)pFb->kicker.cap);
		gwinSetText(handles.hCapProg, cap, FALSE);

		if(pFb->kicker.temp > 1.0f)
			snprintf(chg, 10, "%5.1fC", pFb->kicker.temp);
		else
			snprintf(chg, 10, "%5.2fA", pFb->kicker.chg);

		gwinSetText(handles.hChg, chg, FALSE);

		gwinSetVisible(handles.hBatUsb, FALSE);
		gwinSetVisible(handles.hCapDisabled, FALSE);
		gwinSetVisible(handles.hBatProg, TRUE);
		gwinSetVisible(handles.hCapProg, TRUE);
		gwinSetVisible(handles.hDischarge, TRUE);
	}

	if(pFb->barrier.rxDamaged)
	{
		gwinSetText(handles.hBarrier, "RX Fault", FALSE);
		gwinSetText(handles.hBarrier2, "", FALSE);
		gwinSetStyle(handles.hBarrier, &RedTextStyle);
	}
	else if(pFb->barrier.txDamaged)
	{
		gwinSetText(handles.hBarrier, "TX Fault", FALSE);
		gwinSetText(handles.hBarrier2, "", FALSE);
		gwinSetStyle(handles.hBarrier, &RedTextStyle);
	}
	else
	{
		gwinSetStyle(handles.hBarrier, 0);

		snprintf(ir, 10, "%4hu/%4hu", pFb->barrier.level[0], pFb->barrier.level[1]);
		gwinSetText(handles.hBarrier, ir, FALSE);

		snprintf(ir2, 10, "%4hu", pFb->barrier.threshold);
		gwinSetText(handles.hBarrier2, ir2, FALSE);
	}

	if(pFb->kicker.cap > 10.0f && !pFb->power.usbPowered)
	{
		if(gtimerIsActive(&handles.flashTimer) == FALSE)
		{
			gtimerStart(&handles.flashTimer, &flashFunc, 0, TRUE, 150);
			gwinSetVisible(handles.hVersion, FALSE);
		}
	}
	else
	{
		gtimerStop(&handles.flashTimer);
		gwinSetVisible(handles.hCharged, FALSE);
		gwinSetVisible(handles.hVersion, TRUE);
	}
}

void MainStatusConfigNetworkUpdate(ConfigNetwork* pCfg)
{
	static char buf[10];

	BotCoverSetId(handles.hBotCover, pCfg->botId);

	snprintf(buf, 10, "%3hu", (uint16_t)pCfg->channel);
	gwinSetText(handles.hChannel, buf, FALSE);
}

void MainStatusWifiUpdate(PresenterWifiStat* pStat)
{
	static char rssi[10];

	if(pStat->linkDisabled)
	{
		gwinSetText(handles.hRSSI, "disabled", FALSE);
	}
	else
	{
		if(pStat->bsOnline)
		{
			snprintf(rssi, 10, "%3.2fdBm", pStat->rssi);
			gwinSetText(handles.hRSSI, rssi, FALSE);
		}
		else
		{
			gwinSetText(handles.hRSSI, "Link lost", FALSE);
		}
	}

	if(pStat->sumatraOnline)
		gwinSetVisible(handles.hSumatraStatus, TRUE);
	else
		gwinSetVisible(handles.hSumatraStatus, FALSE);
}

void MainStatusHwIdUpdate(uint8_t id)
{
	static char buf[10];

	if(id == 0xFF)
	{
		gwinSetText(handles.hHwId, "N/A", FALSE);
	}
	else
	{
		snprintf(buf, 10, "%3hu", (uint16_t)id);
		gwinSetText(handles.hHwId, buf, FALSE);
	}
}
