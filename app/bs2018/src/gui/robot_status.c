/*
 * robot_status.c
 *
 *  Created on: 02.11.2017
 *      Author: AndreR
 */

#include "robot_status.h"
#include "gui/bot_cover.h"
#include "gui/styles.h"
#include "gui/icons.h"
#include "../hub.h"
#include <stdio.h>

RobotStatus robotStatus;

typedef struct _ParamIncDec
{
	GHandle hBtnInc;
	GHandle hBtnDec;
	GHandle hValue;
} ParamIncDec;

static struct _handles
{
	GHandle hOverview;
	GHandle hDetails;

	GHandle hBtnBack;

	// status overview
	struct _robots
	{
		GHandle hBot;
		GHandle hId;
		GHandle hState;
		GHandle hBatProg;
		GHandle hBatIcon;
		GHandle hKickerIcon;
		GHandle hKickerProg;
		GHandle hVision;
		GHandle hWifi;
		GHandle hFeatures;
		GHandle hBarrier;
		GHandle hButton;

		char batText[8];
		char kickerText[8];
	} robots[32];

	// detailed robot status
	GHandle hBotId;
	GHandle hState;
	GHandle hHwId;
	GHandle hBatProg;
	GHandle hWifiIcon[2];
	GHandle hWifiRssi[2];
	GHandle hKickerProg;
	GHandle hDribbler;
	GHandle hBarrier;
	GHandle hFeatStraight;
	GHandle hFeatCharge;
	GHandle hFeatChip;
	GHandle hFeatMove;
	GHandle hFeatDribble;
	GHandle hFeatBarrier;
	GHandle hFeatMBv2016;
	GHandle hFeatKDv2017;
	GHandle hFeatExt;
	GHandle hFeatCover;
	GHandle hVisionPos;
	GHandle hBotPos;
	GHandle hBotVel;

	// robot control
	GHandle hBtnManualControl;
	GHandle hControl;

	GHandle hBtnPos;
	GHandle hBtnVelYW;
	GHandle hBtnParams;
	GHandle hBtnKickDribble;

	GHandle hPosCtrl;
	GHandle hVelCtrl;
	GHandle hUltraVel;
	GHandle hParamsCtrl;
	GHandle hKdCtrl;

	GHandle* pCurrentControl;

	// velocity control
	GHandle hMaxVelXY;
	GHandle hMaxVelW;
	GHandle hMaxAccXY;
	GHandle hMaxAccW;
	GHandle hVelModeXY;
	GHandle hVelModeYW;
	GHandle hBtnMotorsOff;

	ParamIncDec paramMaxVelXY;
	ParamIncDec paramMaxVelW;
	ParamIncDec paramMaxAccXY;
	ParamIncDec paramMaxAccW;

	// Kicker & Dribbler control
	GHandle hBtnKickStraight;
	GHandle hBtnKickChip;
	GHandle hBtnDribble;
	GHandle hChargeKicker;

	ParamIncDec paramKickSpeed;
	ParamIncDec paramDribbleSpeed;
} handles;

static uint8_t lastKickCount = 0xFF;

void RobotStatusPositionFrameUpdate()
{
	if(!gwinGetVisible(handles.hPosCtrl))
		return;

	gwinRedraw(handles.hPosCtrl);
}

static void gwinButtonDraw_Clean(GWidgetObject *gw, void *param)
{
	(void)gw;
	(void)param;
}

static void refreshPositionFrame(GWidgetObject* gw, void* param)
{
	(void)param;
	GContainerObject* pCont = (GContainerObject*) gw;
	GWindowObject* pObj = &pCont->g;
	coord_t x = pObj->x;
	coord_t y = pObj->y;

	gdispFillArea(x, y, pObj->width, pObj->height, Green);

	coord_t halfx = pObj->x + pObj->width / 2;
	coord_t halfy = pObj->y + pObj->height / 2;
	gdispDrawLine(halfx, y, halfx, y + pObj->height, White);

	if(!hub.field.fieldLength)
		return;

	float xMultiplier = (float)(pObj->width) / (float) (- hub.field.fieldLength + hub.field.boundaryWidth * -2);
	float yMultiplier = (float)(pObj->height) / (float) (hub.field.fieldWidth + hub.field.boundaryWidth * 2);

	gdispDrawBox(
			x - hub.field.boundaryWidth * xMultiplier,
			y + hub.field.boundaryWidth * yMultiplier,
			pObj->width + 2 * hub.field.boundaryWidth * xMultiplier,
			pObj->height - 2 * hub.field.boundaryWidth * yMultiplier,
			White);

	gdispDrawBox(
			x - (hub.field.boundaryWidth - hub.field.goalDepth) * xMultiplier,
			y + (hub.field.boundaryWidth + hub.field.fieldWidth/2 - hub.field.goalWidth/2) * yMultiplier,
			- hub.field.goalDepth * xMultiplier,
			hub.field.goalWidth * yMultiplier,
			White);
	gdispDrawBox(
			x - (hub.field.boundaryWidth + hub.field.fieldLength) * xMultiplier,
			y + (hub.field.boundaryWidth + hub.field.fieldWidth/2 - hub.field.goalWidth/2) * yMultiplier,
			- hub.field.goalDepth * xMultiplier,
			hub.field.goalWidth * yMultiplier,
			White);

	for(uint8_t i = 0; i < WIFI_MAX_BOTS; i++)
	{
		if(!hub.robot[i].isOnVision)
			continue;

		coord_t x = halfx + xMultiplier * hub.robot[i].lastVisionPosFloat[0];
		coord_t y = halfy + yMultiplier * hub.robot[i].lastVisionPosFloat[1];

		gdispFillCircle(x, y, 15, i <= 11 ? Yellow : Blue);

		char str[3];
		sprintf(str, "%d", i%16);
		gdispDrawString(x - 12, y - 8, str, gdispOpenFont("DejaVuSans20"), Black);
	}
}

static int16_t eventHandler(GEvent* pEvent)
{
	if(pEvent->type == GEVENT_TOUCH)
	{
		GEventMouse* pMouse = (GEventMouse*)pEvent;

		if((pMouse->buttons & GMETA_MOUSE_UP) && gwinGetVisible(handles.hUltraVel))
		{
			// finger moved up
			robotStatus.ultraState.x = 0;
			robotStatus.ultraState.y = 0;
			robotStatus.ultraState.w = 0;

			return -1;
		}

		if(gwinGetVisible(handles.hUltraVel) && (pMouse->buttons & GINPUT_TOUCH_PRESSED))
		{
			coord_t x = pMouse->x - gwinGetScreenX(handles.hUltraVel);
			coord_t y = pMouse->y - gwinGetScreenY(handles.hUltraVel);

			coord_t w = gwinGetWidth(handles.hUltraVel);
			coord_t h = gwinGetHeight(handles.hUltraVel);

			if(x < w && y < h && x > 0 && y > 0)
			{
				float xVel = ((float)x/(float)w-0.5f)*2.0f;
				float yVel = -((float)y/(float)h-0.5f)*2.0f;

				float absVel = sqrtf(xVel*xVel + yVel*yVel);
				if(absVel > 1.0f)
				{
					xVel /= absVel;
					yVel /= absVel;
				}

				if(gwinRadioIsPressed(handles.hVelModeXY))
				{
					robotStatus.ultraState.mode = MOVE_MODE_VEL_XY;
					robotStatus.ultraState.x = xVel*robotStatus.limits.maxVelXY;
					robotStatus.ultraState.y = yVel*robotStatus.limits.maxVelXY;
					robotStatus.ultraState.w = 0;
				}
				else
				{
					robotStatus.ultraState.mode = MOVE_MODE_VEL_YW;
					robotStatus.ultraState.x = 0;
					robotStatus.ultraState.y = yVel*robotStatus.limits.maxVelXY;
					robotStatus.ultraState.w = -xVel*robotStatus.limits.maxVelW;
				}
			}
		}
		else if(gwinGetVisible(handles.hPosCtrl) && (pMouse->buttons & GINPUT_TOUCH_PRESSED))
		{
			coord_t x = pMouse->x - gwinGetScreenX(handles.hPosCtrl);
			coord_t y = pMouse->y - gwinGetScreenY(handles.hPosCtrl);
			coord_t w = gwinGetWidth(handles.hPosCtrl);
			coord_t h = gwinGetHeight(handles.hPosCtrl);

			if(x < w && y < h && x > 0 && y > 0 && hub.field.fieldLength)
			{
				float xMultiplier = (float) (hub.field.fieldLength + hub.field.boundaryWidth * 2) / (float) w;
				float yMultiplier = (float) (hub.field.fieldWidth + hub.field.boundaryWidth * 2) / (float) h;
				robotStatus.ultraState.mode = MOVE_MODE_POS;
				robotStatus.ultraState.x = (w / 2 - x) * xMultiplier;
				robotStatus.ultraState.y = (y - h / 2) * yMultiplier;
				robotStatus.ultraState.w = 0;
				gwinRedraw(handles.hPosCtrl);
			}
		}
	}

	if(pEvent->type == GEVENT_GWIN_BUTTON)
	{
		GEventGWinButton* pBtn = (GEventGWinButton*)pEvent;

		if(pBtn->gwin == handles.hBtnBack)
		{
			gwinHide(handles.hDetails);

			robotStatus.selectedBotId = 0xFF;

			robotStatus.ultraState.mode = MOVE_MODE_OFF;
			robotStatus.ultraState.x = 0;
			robotStatus.ultraState.y = 0;
			robotStatus.ultraState.w = 0;

			robotStatus.kickState.mode = KICK_MODE_DISARM;
			robotStatus.kickState.speed = 0;
			gwinSetText(handles.hBtnKickChip, "Chip", FALSE);
			gwinSetText(handles.hBtnKickStraight, "Straight", FALSE);

			robotStatus.dribbleState = 0;
			gwinSetText(handles.hBtnDribble, "Dribble", FALSE);

			gwinHide(handles.hControl);
			gwinSetText(handles.hBtnManualControl, "Take Control", FALSE);
			robotStatus.manualControlOn = 0;

			lastKickCount = 0xFF;

			gwinShow(handles.hOverview);
		}

		for(uint16_t id = 0; id < CMD_BOT_COUNT_TOTAL; id++)
		{
			if(pBtn->gwin == handles.robots[id].hButton)
			{
				robotStatus.selectedBotId = id;
				gwinHide(handles.hOverview);
				BotCoverSetId(handles.hBotId, id);

				gwinShow(handles.hDetails);
			}
		}

		if(pBtn->gwin == handles.hBtnManualControl)
		{
			if(robotStatus.manualControlOn)
			{
				gwinHide(handles.hControl);
				gwinSetText(handles.hBtnManualControl, "Take Control", FALSE);
				robotStatus.manualControlOn = 0;
				robotStatus.ultraState.mode = MOVE_MODE_OFF;
			}
			else
			{
				gwinShow(handles.hControl);
				gwinSetText(handles.hBtnManualControl, "Stop Control", FALSE);
				robotStatus.manualControlOn = 1;
			}
		}

		if(pBtn->gwin == handles.hBtnPos)
		{
			gwinHide(*handles.pCurrentControl);
			gwinShow(handles.hPosCtrl);
			handles.pCurrentControl = &handles.hPosCtrl;
		}

		if(pBtn->gwin == handles.hBtnVelYW)
		{
			gwinHide(*handles.pCurrentControl);
			gwinShow(handles.hVelCtrl);
			handles.pCurrentControl = &handles.hVelCtrl;
		}

		if(pBtn->gwin == handles.hBtnParams)
		{
			gwinHide(*handles.pCurrentControl);
			gwinShow(handles.hParamsCtrl);
			handles.pCurrentControl = &handles.hParamsCtrl;
		}

		if(pBtn->gwin == handles.hBtnKickDribble)
		{
			gwinHide(*handles.pCurrentControl);
			gwinShow(handles.hKdCtrl);
			handles.pCurrentControl = &handles.hKdCtrl;
		}

		if(pBtn->gwin == handles.hBtnMotorsOff)
		{
			robotStatus.ultraState.mode = MOVE_MODE_OFF;

			robotStatus.kickState.mode = KICK_MODE_DISARM;
			robotStatus.kickState.speed = 0;
			gwinSetText(handles.hBtnKickStraight, "Straight", FALSE);
			gwinSetText(handles.hBtnKickChip, "Chip", FALSE);
		}

		if(pBtn->gwin == handles.hBtnKickStraight)
		{
			if(robotStatus.kickState.mode == KICK_MODE_ARM_STRAIGHT)
			{
				robotStatus.kickState.mode = KICK_MODE_DISARM;
				robotStatus.kickState.speed = 0;
				robotStatus.ultraState.mode = MOVE_MODE_OFF;
				gwinSetText(handles.hBtnKickStraight, "Straight", FALSE);
			}
			else
			{
				robotStatus.kickState.mode = KICK_MODE_ARM_STRAIGHT;
				robotStatus.kickState.speed = robotStatus.limits.kickSpeed;
				robotStatus.ultraState.mode = MOVE_MODE_VEL_XY;
				robotStatus.ultraState.x = 0;
				robotStatus.ultraState.y = 0;
				robotStatus.ultraState.w = 0;
				gwinSetText(handles.hBtnKickStraight, "Straight (armed)", FALSE);
			}

			gwinSetText(handles.hBtnKickChip, "Chip", FALSE);
		}

		if(pBtn->gwin == handles.hBtnKickChip)
		{
			if(robotStatus.kickState.mode == KICK_MODE_ARM_CHIP)
			{
				robotStatus.kickState.mode = KICK_MODE_DISARM;
				robotStatus.kickState.speed = 0;
				robotStatus.ultraState.mode = MOVE_MODE_OFF;
				gwinSetText(handles.hBtnKickChip, "Chip", FALSE);
			}
			else
			{
				robotStatus.kickState.mode = KICK_MODE_ARM_CHIP;
				robotStatus.kickState.speed = robotStatus.limits.kickSpeed;
				robotStatus.ultraState.mode = MOVE_MODE_VEL_XY;
				robotStatus.ultraState.x = 0;
				robotStatus.ultraState.y = 0;
				robotStatus.ultraState.w = 0;
				gwinSetText(handles.hBtnKickChip, "Chip (armed)", FALSE);
			}

			gwinSetText(handles.hBtnKickStraight, "Straight", FALSE);
		}

		if(pBtn->gwin == handles.hBtnDribble)
		{
			if(robotStatus.dribbleState == 0)
			{
				robotStatus.dribbleState = (int16_t)robotStatus.limits.dribbleSpeed;
				robotStatus.ultraState.mode = MOVE_MODE_VEL_XY;
				robotStatus.ultraState.x = 0;
				robotStatus.ultraState.y = 0;
				robotStatus.ultraState.w = 0;

				gwinSetText(handles.hBtnDribble, "Stop dribbling", FALSE);
			}
			else
			{
				robotStatus.dribbleState = 0;
				robotStatus.ultraState.mode = MOVE_MODE_OFF;

				gwinSetText(handles.hBtnDribble, "Dribble", FALSE);
			}
		}

		// Adjust Limits
		if(pBtn->gwin == handles.paramMaxVelXY.hBtnDec && robotStatus.limits.maxVelXY > 0.5f)
			robotStatus.limits.maxVelXY -= 0.5f;

		if(pBtn->gwin == handles.paramMaxVelXY.hBtnInc && robotStatus.limits.maxVelXY < 5.0f)
			robotStatus.limits.maxVelXY += 0.5f;

		static char maxVelXYBuf[16];
		snprintf(maxVelXYBuf, 16, "%.1fm/s", robotStatus.limits.maxVelXY);
		gwinSetText(handles.paramMaxVelXY.hValue, maxVelXYBuf, FALSE);
		gwinSetText(handles.hMaxVelXY, maxVelXYBuf, FALSE);

		if(pBtn->gwin == handles.paramMaxVelW.hBtnDec && robotStatus.limits.maxVelW > 3.0f)
			robotStatus.limits.maxVelW -= 3.0f;

		if(pBtn->gwin == handles.paramMaxVelW.hBtnInc && robotStatus.limits.maxVelW < 30.0f)
			robotStatus.limits.maxVelW += 3.0f;

		static char maxVelWBuf[16];
		snprintf(maxVelWBuf, 16, "%.1frad/s", robotStatus.limits.maxVelW);
		gwinSetText(handles.paramMaxVelW.hValue, maxVelWBuf, FALSE);
		gwinSetText(handles.hMaxVelW, maxVelWBuf, FALSE);

		if(pBtn->gwin == handles.paramMaxAccXY.hBtnDec && robotStatus.limits.maxAccXY > 0.5f)
			robotStatus.limits.maxAccXY -= 0.5f;

		if(pBtn->gwin == handles.paramMaxAccXY.hBtnInc && robotStatus.limits.maxAccXY < 10.0f)
			robotStatus.limits.maxAccXY += 0.5f;

		static char maxAccXYBuf[16];
		snprintf(maxAccXYBuf, 16, "%.1fm/s2", robotStatus.limits.maxAccXY);
		gwinSetText(handles.paramMaxAccXY.hValue, maxAccXYBuf, FALSE);
		gwinSetText(handles.hMaxAccXY, maxAccXYBuf, FALSE);

		if(pBtn->gwin == handles.paramMaxAccW.hBtnDec && robotStatus.limits.maxAccW > 10.0f)
			robotStatus.limits.maxAccW -= 3.0f;

		if(pBtn->gwin == handles.paramMaxAccW.hBtnInc && robotStatus.limits.maxAccW < 100.0f)
			robotStatus.limits.maxAccW += 3.0f;

		static char maxAccWBuf[16];
		snprintf(maxAccWBuf, 16, "%.1frad/s2", robotStatus.limits.maxAccW);
		gwinSetText(handles.paramMaxAccW.hValue, maxAccWBuf, FALSE);
		gwinSetText(handles.hMaxAccW, maxAccWBuf, FALSE);

		// Configure kicker & dribbler
		if(pBtn->gwin == handles.paramKickSpeed.hBtnDec && robotStatus.limits.kickSpeed > 0.5f)
			robotStatus.limits.kickSpeed -= 0.5f;

		if(pBtn->gwin == handles.paramKickSpeed.hBtnInc && robotStatus.limits.kickSpeed < 8.0f)
			robotStatus.limits.kickSpeed += 0.5f;

		static char kickSpeedBuf[16];
		snprintf(kickSpeedBuf, 16, "%.1fm/s", robotStatus.limits.kickSpeed);
		gwinSetText(handles.paramKickSpeed.hValue, kickSpeedBuf, FALSE);

		if(pBtn->gwin == handles.paramDribbleSpeed.hBtnDec && robotStatus.limits.dribbleSpeed > 1000.0f)
			robotStatus.limits.dribbleSpeed -= 1000.0f;

		if(pBtn->gwin == handles.paramDribbleSpeed.hBtnInc && robotStatus.limits.dribbleSpeed < 20000.0f)
			robotStatus.limits.dribbleSpeed += 1000.0f;

		static char dribblerSpeedBuf[16];
		snprintf(dribblerSpeedBuf, 16, "%.0fRPM", robotStatus.limits.dribbleSpeed);
		gwinSetText(handles.paramDribbleSpeed.hValue, dribblerSpeedBuf, FALSE);
	}

	return -1;
}

static void paramIncDecCreate(ParamIncDec* pParam, GHandle hParent, coord_t x, coord_t y, const char* text)
{
	GWidgetInit contInit = { { x, y, 270, 75, TRUE, hParent }, 0, 0, 0, 0 };
	GHandle hCont = gwinContainerCreate(0, &contInit, 0);

	GWidgetInit maxVelLabelInit = { { 5, 0, 260, 20, TRUE, hCont }, text, &gwinLabelDrawJustifiedCenter, 0, 0 };
	gwinLabelCreate(0, &maxVelLabelInit);

	GWidgetInit btnDecInit = {{5, 25, 80, 50, TRUE, hCont}, "-", 0, 0, 0};
	pParam->hBtnDec = gwinButtonCreate(0, &btnDecInit);

	GWidgetInit maxVelInit = { { 95, 25, 80, 50, TRUE, hCont }, "val", &gwinLabelDrawJustifiedCenter, 0, 0 };
	pParam->hValue = gwinLabelCreate(0, &maxVelInit);

	GWidgetInit btnIncInit = {{185, 25, 80, 50, TRUE, hCont}, "+", 0, 0, 0};
	pParam->hBtnInc = gwinButtonCreate(0, &btnIncInit);
}


static void createOverview(GHandle hTop)
{
	GWidgetInit overviewInit = { { 0, 0, 800, 420, TRUE, hTop }, 0, 0, 0, 0 };
	handles.hOverview = gwinContainerCreate(0, &overviewInit, 0);

	for(uint16_t botId = 0; botId < WIFI_MAX_BOTS; botId++)
	{
		uint16_t row = (botId%16)/4;
		uint16_t column = botId%4 + botId/16*4;
		uint16_t x = column*100+3; //135
		uint16_t y = row*105+2;

		GWidgetInit botInit = { { x, y, 95, 100, TRUE, handles.hOverview}, 0, 0, 0, 0 };
		GHandle hBot = gwinContainerCreate(0, &botInit, 0);
		handles.robots[botId].hBot = hBot;

		const GWidgetStyle* pStyle;
		if(botId >= 16)
			pStyle = &BlueTextStyle;
		else
			pStyle = &YellowTextStyle;

		char* idStr = (char*)gfxAlloc(4);
		snprintf(idStr, 4, "%hu", botId%16);
		GWidgetInit botIdInit = { { 0, 0, 40, 30, TRUE, hBot }, idStr, &gwinLabelDrawJustifiedCenter, 0, pStyle };
		handles.robots[botId].hId = gwinLabelCreate(0, &botIdInit);
		gwinSetFont(handles.robots[botId].hId, gdispOpenFont("DejaVuSans32"));

		GWidgetInit stateInit = { { 40, 5, 85, 20, TRUE, hBot }, "Offline", &gwinLabelDrawJustifiedLeft, 0, 0 };
		handles.robots[botId].hState = gwinLabelCreate(0, &stateInit);

		handles.robots[botId].hBatIcon = IconsCreate(0, 33, hBot, ICON_BATTERY, Lime);

	    GWidgetInit batInit = { { 20, 33, 105, 15, TRUE, hBot }, 0, 0, 0, &GrayProgressBarStyle };
	    handles.robots[botId].hBatProg = gwinProgressbarCreate(0, &batInit);
	    gwinProgressbarSetRange(handles.robots[botId].hBatProg, 0, 255);
	    gwinProgressbarSetPosition(handles.robots[botId].hBatProg, 127);
	    gwinSetText(handles.robots[botId].hBatProg, "14.8V", FALSE);

	    handles.robots[botId].hKickerIcon = IconsCreate(0, 51, hBot, ICON_LIGHTNING, Lime);

	    GWidgetInit kickerInit = { { 20, 51, 105, 15, TRUE, hBot }, 0, 0, 0, &GrayProgressBarStyle };
	    handles.robots[botId].hKickerProg = gwinProgressbarCreate(0, &kickerInit);
	    gwinProgressbarSetRange(handles.robots[botId].hKickerProg, 0, 220);
	    gwinProgressbarSetPosition(handles.robots[botId].hKickerProg, 0);
	    gwinSetText(handles.robots[botId].hKickerProg, "0V", FALSE);

	    handles.robots[botId].hVision = IconsCreate(0, 69, hBot, ICON_GPS, Red);

	    handles.robots[botId].hWifi = IconsCreate(25, 69, hBot, ICON_WIFI_SMALL, Lime);

	    handles.robots[botId].hFeatures = IconsCreate(50, 69, hBot, ICON_HEARTBEAT, Lime);

	    handles.robots[botId].hBarrier = IconsCreate(75, 69, hBot, ICON_LASER, Lime);

		GWidgetInit btnInit = { { x, y, 95, 100, TRUE, hTop}, 0, &gwinButtonDraw_Clean, (void*)((uint32_t)botId), 0 };
		handles.robots[botId].hButton = gwinButtonCreate(0, &btnInit);
	}
}

static void createStatus(GHandle hDetails)
{
	// Detailed status sub-panel
	GWidgetInit statusInit = { { 5, 5, 215, 350, TRUE, hDetails }, 0, 0, 0, 0 };
	GHandle hStatus = gwinContainerCreate(0, &statusInit, GWIN_CONTAINER_BORDER);

	handles.hBotId = BotCoverCreate(0, 0, hStatus, 0, 0);

	GWidgetInit stateInit = { { 65, 5, 150, 20, TRUE, hStatus }, "Offline", &gwinLabelDrawJustifiedCenter, 0, 0 };
	handles.hState = gwinLabelCreate(0, &stateInit);

	GWidgetInit hwidInit = { { 65, 30, 150, 20, TRUE, hStatus }, "HW ID: 42", &gwinLabelDrawJustifiedCenter, 0, 0 };
	handles.hHwId = gwinLabelCreate(0, &hwidInit);

    handles.hWifiIcon[0] = IconsCreate(0, 55, hStatus, ICON_WIFI_SMALL, Lime);
	GWidgetInit wifi0RssiInit = { { 30, 55, 80, 20, TRUE, hStatus }, "-15.0dBm", 0, 0, 0 };
	handles.hWifiRssi[0] = gwinLabelCreate(0, &wifi0RssiInit);

	handles.hWifiIcon[1] = IconsCreate(110, 55, hStatus, ICON_WIFI_SMALL, Red);
	GWidgetInit wifi1RssiInit = { { 140, 55, 80, 20, TRUE, hStatus }, "-65.3dBm", 0, 0, 0 };
	handles.hWifiRssi[1] = gwinLabelCreate(0, &wifi1RssiInit);

	GWidgetInit batLabelInit = { { 0, 85, 65, 20, TRUE, hStatus }, "Battery :", 0, 0, 0 };
	gwinLabelCreate(0, &batLabelInit);

    GWidgetInit batInit = { { 70, 85, 140, 20, TRUE, hStatus }, 0, 0, 0, &GreenProgressBarStyle };
    handles.hBatProg = gwinProgressbarCreate(0, &batInit);
    gwinProgressbarSetRange(handles.hBatProg, 0, 255);
    gwinProgressbarSetPosition(handles.hBatProg, 127);
    gwinSetText(handles.hBatProg, "15.8V", FALSE);

	GWidgetInit kickerLabelInit = { { 0, 110, 65, 20, TRUE, hStatus }, "Kicker:", 0, 0, 0 };
	gwinLabelCreate(0, &kickerLabelInit);

    GWidgetInit kickerInit = { { 70, 110, 140, 20, TRUE, hStatus }, 0, 0, 0, &RedProgressBarStyle };
    handles.hKickerProg = gwinProgressbarCreate(0, &kickerInit);
    gwinProgressbarSetRange(handles.hKickerProg, 0, 220);
    gwinProgressbarSetPosition(handles.hKickerProg, 0);
    gwinSetText(handles.hKickerProg, "160.0V", FALSE);

	GWidgetInit dribblerLabelInit = { { 0, 135, 65, 20, TRUE, hStatus }, "Dribbler:", 0, 0, 0 };
	gwinLabelCreate(0, &dribblerLabelInit);

	GWidgetInit dribblerInit = { { 70, 135, 140, 20, TRUE, hStatus }, "12342RPM / 32.2C", 0, 0, 0 };
	handles.hDribbler = gwinLabelCreate(0, &dribblerInit);

	GWidgetInit barrierLabelInit = { { 0, 160, 65, 20, TRUE, hStatus }, "Barrier:", 0, 0, 0 };
	gwinLabelCreate(0, &barrierLabelInit);

	GWidgetInit barrierInit = { { 70, 160, 140, 20, TRUE, hStatus }, "clear", 0, 0, 0 };
	handles.hBarrier = gwinLabelCreate(0, &barrierInit);

	GWidgetInit featuresLabelInit = { { 0, 185, 65, 20, TRUE, hStatus }, "Features:", 0, 0, 0 };
	gwinLabelCreate(0, &featuresLabelInit);

	GWidgetInit featStraightInit = { { 10, 205, 56, 15, TRUE, hStatus }, "Straight", 0, 0, 0 };
	handles.hFeatStraight = gwinLabelCreate(0, &featStraightInit);

	GWidgetInit featChipInit = { { 80, 205, 35, 15, TRUE, hStatus }, "Chip", 0, 0, 0 };
	handles.hFeatChip = gwinLabelCreate(0, &featChipInit);

	GWidgetInit featChargeInit = { { 150, 205, 56, 15, TRUE, hStatus }, "Charge", 0, 0, 0 };
	handles.hFeatCharge = gwinLabelCreate(0, &featChargeInit);

	GWidgetInit featMoveInit = { { 10, 225, 35, 15, TRUE, hStatus }, "Move", 0, 0, 0 };
	handles.hFeatMove = gwinLabelCreate(0, &featMoveInit);

	GWidgetInit featDribbleInit = { { 80, 225, 56, 15, TRUE, hStatus }, "Dribble", 0, 0, 0 };
	handles.hFeatDribble = gwinLabelCreate(0, &featDribbleInit);

	GWidgetInit featBarrierInit = { { 150, 225, 56, 15, TRUE, hStatus }, "Barrier", 0, 0, 0 };
	handles.hFeatBarrier = gwinLabelCreate(0, &featBarrierInit);

	GWidgetInit featMBv2016Init = { { 10, 245, 56, 15, TRUE, hStatus }, "MBv2016", 0, 0, 0 };
	handles.hFeatMBv2016 = gwinLabelCreate(0, &featMBv2016Init);

	GWidgetInit featKDv2017Init = { { 80, 245, 56, 15, TRUE, hStatus }, "KDv2017", 0, 0, 0 };
	handles.hFeatKDv2017 = gwinLabelCreate(0, &featKDv2017Init);

	GWidgetInit featExtInit = { { 150, 245, 35, 15, TRUE, hStatus }, "Ext.", 0, 0, 0 };
	handles.hFeatExt = gwinLabelCreate(0, &featExtInit);

	GWidgetInit featCoverInit = { { 10, 265, 56, 15, TRUE, hStatus }, "Cover", 0, 0, 0 };
	handles.hFeatCover = gwinLabelCreate(0, &featCoverInit);

	GWidgetInit visionPosLabelInit = { { 0, 285, 65, 15, TRUE, hStatus }, "Vis.Pos.:", 0, 0, 0 };
	gwinLabelCreate(0, &visionPosLabelInit);

	GWidgetInit visionPosInit = { { 70, 285, 140, 15, TRUE, hStatus }, "10.53 11.43 123.0", 0, 0, 0 };
	handles.hVisionPos = gwinLabelCreate(0, &visionPosInit);

	GWidgetInit botPosLabelInit = { { 0, 305, 65, 15, TRUE, hStatus }, "Bot.Pos.:", 0, 0, 0 };
	gwinLabelCreate(0, &botPosLabelInit);

	GWidgetInit botPosInit = { { 70, 305, 140, 15, TRUE, hStatus }, "10.53 11.43 123.0", 0, 0, 0 };
	handles.hBotPos = gwinLabelCreate(0, &botPosInit);

	GWidgetInit botVelLabelInit = { { 0, 325, 65, 15, TRUE, hStatus }, "Bot.Vel.:", 0, 0, 0 };
	gwinLabelCreate(0, &botVelLabelInit);

	GWidgetInit botVelInit = { { 70, 325, 140, 15, TRUE, hStatus }, "10.53 11.43 123.0", 0, 0, 0 };
	handles.hBotVel = gwinLabelCreate(0, &botVelInit);
}

GHandle RobotStatusCreate()
{
	/* hTop
	 * - hOverview
	 * - hDetails
	 *   - hBtnBack
	 *   - hStatus
	 *   - hControl
	 */

	robotStatus.limits.maxVelXY = 1.0f;
	robotStatus.limits.maxVelW = 9.0f;
	robotStatus.limits.maxAccXY = 2.0f;
	robotStatus.limits.maxAccW = 30.0f;
	robotStatus.limits.dribbleSpeed = 10000.0f;
	robotStatus.limits.kickSpeed = 4.0f;

    font_t curFont = gwinGetDefaultFont();
    gwinSetDefaultFont(gdispOpenFont("fixed_7x14"));

	GWidgetInit wi = { { 0, 60, 800, 420, FALSE, 0 }, "Robot Status", 0, &eventHandler, &GrayBackgroundStyle };
	GHandle hTop = gwinContainerCreate(0, &wi, 0);

	createOverview(hTop);

	GWidgetInit detailsInit = { { 0, 0, 800, 420, FALSE, hTop }, 0, 0, 0, 0 };
	handles.hDetails = gwinContainerCreate(0, &detailsInit, 0);

	createStatus(handles.hDetails);

	// Back to overview button
	GWidgetInit btnBackInit = {{5, 365, 100, 50, TRUE, handles.hDetails}, "<= Back", 0, 0, 0};
	handles.hBtnBack = gwinButtonCreate(0, &btnBackInit);

	// Manual control
	GWidgetInit controlInit = { { 225, 0, 575, 420, FALSE, handles.hDetails }, 0, 0, 0, 0 };
	handles.hControl = gwinContainerCreate(0, &controlInit, 0);

	GWidgetInit btnManualControlInit = {{230, 365, 100, 50, TRUE, handles.hDetails}, "Take Control", 0, 0, 0};
	handles.hBtnManualControl = gwinButtonCreate(0, &btnManualControlInit);

	// Control buttons
	GWidgetInit btnPosInit = {{115, 365, 100, 50, TRUE, handles.hControl}, "Position", 0, 0, 0};
	handles.hBtnPos = gwinButtonCreate(0, &btnPosInit);

	GWidgetInit btnVelYWInit = {{225, 365, 100, 50, TRUE, handles.hControl}, "Velocity", 0, 0, 0};
	handles.hBtnVelYW = gwinButtonCreate(0, &btnVelYWInit);

	GWidgetInit btnParamsInit = {{335, 365, 100, 50, TRUE, handles.hControl}, "Params", 0, 0, 0};
	handles.hBtnParams = gwinButtonCreate(0, &btnParamsInit);

	GWidgetInit btnKickDribbleInit = {{445, 365, 125, 50, TRUE, handles.hControl}, "Kick & Dribble", 0, 0, 0};
	handles.hBtnKickDribble = gwinButtonCreate(0, &btnKickDribbleInit);

	handles.pCurrentControl = &handles.hVelCtrl;

	//Control Pages
	//Position Control
	GWidgetInit posCtrlInit = { { 0, 5, 575, 350, FALSE, handles.hControl }, 0, &refreshPositionFrame, 0, 0};
	handles.hPosCtrl = gwinContainerCreate(0, &posCtrlInit, 0);

	//Velocity Control
	GWidgetInit velYWCtrlInit = { { 0, 5, 575, 350, TRUE, handles.hControl }, 0, 0, 0, 0 };
	handles.hVelCtrl = gwinContainerCreate(0, &velYWCtrlInit, GWIN_CONTAINER_BORDER);

	handles.hUltraVel = IconsCreate(225, 0, handles.hVelCtrl, ICON_CROSSHAIR, White);

	GWidgetInit maxVelXYLabelInit = { { 10, 10, 220, 15, TRUE, handles.hVelCtrl }, "Maximum Linear Velocity", 0, 0, 0 };
	gwinLabelCreate(0, &maxVelXYLabelInit);

	GWidgetInit maxVelXYInit = { { 10, 30, 220, 15, TRUE, handles.hVelCtrl }, "1.0m/s", 0, 0, 0 };
	handles.hMaxVelXY = gwinLabelCreate(0, &maxVelXYInit);

	GWidgetInit maxVelWLabelInit = { { 10, 60, 220, 15, TRUE, handles.hVelCtrl }, "Maximum Rotational Velocity", 0, 0, 0 };
	gwinLabelCreate(0, &maxVelWLabelInit);

	GWidgetInit maxVelWInit = { { 10, 80, 220, 15, TRUE, handles.hVelCtrl }, "9.0rad/s", 0, 0, 0 };
	handles.hMaxVelW = gwinLabelCreate(0, &maxVelWInit);

	GWidgetInit maxAccXYLabelInit = { { 10, 110, 220, 15, TRUE, handles.hVelCtrl }, "Maximum Linear Acceleration", 0, 0, 0 };
	gwinLabelCreate(0, &maxAccXYLabelInit);

	GWidgetInit maxAccXYInit = { { 10, 130, 220, 15, TRUE, handles.hVelCtrl }, "2.0m/s2", 0, 0, 0 };
	handles.hMaxAccXY = gwinLabelCreate(0, &maxAccXYInit);

	GWidgetInit maxAccWLabelInit = { { 10, 160, 220, 15, TRUE, handles.hVelCtrl }, "Maximum Rotational Acceleration", 0, 0, 0 };
	gwinLabelCreate(0, &maxAccWLabelInit);

	GWidgetInit maxAccWInit = { { 10, 180, 220, 15, TRUE, handles.hVelCtrl }, "30.0rad/s2", 0, 0, 0 };
	handles.hMaxAccW = gwinLabelCreate(0, &maxAccWInit);


	GWidgetInit ctrlModeLabelInit = { { 10, 220, 150, 15, TRUE, handles.hVelCtrl }, "Control Mode", 0, 0, 0 };
	gwinLabelCreate(0, &ctrlModeLabelInit);

	GWidgetInit xyInit = { { 10, 250, 100, 40, TRUE, handles.hVelCtrl }, "  XY", 0, 0, 0 };
	handles.hVelModeXY = gwinRadioCreate(0, &xyInit, 0);
	gwinRadioPress(handles.hVelModeXY);
	gwinSetFont(handles.hVelModeXY, gdispOpenFont("DejaVuSans20"));

	GWidgetInit ywInit = { { 10, 300, 100, 40, TRUE, handles.hVelCtrl }, "  YW", 0, 0, 0 };
	handles.hVelModeYW = gwinRadioCreate(0, &ywInit, 0);
	gwinSetFont(handles.hVelModeYW, gdispOpenFont("DejaVuSans20"));

	GWidgetInit btnMotorsOffInit = {{130, 260, 80, 80, TRUE, handles.hVelCtrl}, "Motors Off", 0, 0, 0};
	handles.hBtnMotorsOff = gwinButtonCreate(0, &btnMotorsOffInit);


	//Params Control
	GWidgetInit paramsCtrlInit = { { 0, 5, 575, 350, FALSE, handles.hControl }, 0, 0, 0, 0 };
	handles.hParamsCtrl = gwinContainerCreate(0, &paramsCtrlInit, GWIN_CONTAINER_BORDER);

	paramIncDecCreate(&handles.paramMaxVelXY, handles.hParamsCtrl, 5, 5, "Maximum Linear Velocity");
	gwinSetText(handles.paramMaxVelXY.hValue, "1.0m/s", FALSE);
	paramIncDecCreate(&handles.paramMaxVelW, handles.hParamsCtrl, 280, 5, "Maximum Rotational Velocity");
	gwinSetText(handles.paramMaxVelW.hValue, "9.0rad/s", FALSE);
	paramIncDecCreate(&handles.paramMaxAccXY, handles.hParamsCtrl, 5, 100, "Maximum Linear Acceleration");
	gwinSetText(handles.paramMaxAccXY.hValue, "2.0m/s2", FALSE);
	paramIncDecCreate(&handles.paramMaxAccW, handles.hParamsCtrl, 280, 100, "Maximum Rotational Acceleration");
	gwinSetText(handles.paramMaxAccW.hValue, "30.0rad/s2", FALSE);


	//Kick Control
	GWidgetInit kdCtrlInit = { { 0, 5, 575, 350, FALSE, handles.hControl }, 0, 0, 0, 0 };
	handles.hKdCtrl = gwinContainerCreate(0, &kdCtrlInit, GWIN_CONTAINER_BORDER);

	paramIncDecCreate(&handles.paramKickSpeed, handles.hKdCtrl, 5, 5, "Kick Speed");

	GWidgetInit btnKickStraightInit = {{10, 100, 260, 50, TRUE, handles.hKdCtrl}, "Straight", 0, 0, 0};
	handles.hBtnKickStraight = gwinButtonCreate(0, &btnKickStraightInit);

	GWidgetInit btnKickChipInit = {{10, 180, 260, 50, TRUE, handles.hKdCtrl}, "Chip", 0, 0, 0};
	handles.hBtnKickChip = gwinButtonCreate(0, &btnKickChipInit);

	paramIncDecCreate(&handles.paramDribbleSpeed, handles.hKdCtrl, 280, 5, "Dribble Speed");

	GWidgetInit btnDribbleInit = {{285, 100, 260, 50, TRUE, handles.hKdCtrl}, "Dribble", 0, 0, 0};
	handles.hBtnDribble = gwinButtonCreate(0, &btnDribbleInit);

	GWidgetInit chargeKickerInit = {{10, 310, 260, 30, TRUE, handles.hKdCtrl}, "  Charge Kicker", 0, 0, 0};
	handles.hChargeKicker = gwinCheckboxCreate(0, &chargeKickerInit);

	gwinSetDefaultFont(curFont);

	return hTop;
}

uint8_t RobotStatusIsChargeKicker()
{
	return gwinCheckboxIsChecked(handles.hChargeKicker);
}

static void setStateText(GHandle hLabel, uint16_t features)
{
	uint16_t mode = features >> 12;
	switch(mode)
	{
		case 0: // Idle
			gwinSetText(hLabel, "Idle", FALSE);
			gwinSetStyle(hLabel, &YellowTextStyle);
			break;
		case 1: // Ready
			gwinSetText(hLabel, "Ready", FALSE);
			gwinSetStyle(hLabel, &GreenTextStyle);
			break;
		case 2: // Low-power
			gwinSetText(hLabel, "Empty", FALSE);
			gwinSetStyle(hLabel, &RedTextStyle);
			break;
		case 3: // Test
			gwinSetText(hLabel, "Test", FALSE);
			gwinSetStyle(hLabel, &BlueTextStyle);
			break;
		default:
			break;
	}
}

void RobotStatusUpdate(PresenterRobotInfo* pRobots, uint8_t visionAvailable)
{
	const uint16_t requiredFeatures = SYSTEM_MATCH_FEEDBACK_FEATURE_STRAIGHT |
			SYSTEM_MATCH_FEEDBACK_FEATURE_CHIP | SYSTEM_MATCH_FEEDBACK_FEATURE_MOVE |
			SYSTEM_MATCH_FEEDBACK_FEATURE_DRIBBLE | SYSTEM_MATCH_FEEDBACK_FEATURE_BARRIER |
			SYSTEM_MATCH_FEEDBACK_FEATURE_CHARGE;

	for(uint8_t id = 0; id < WIFI_MAX_BOTS; id++)
	{
		struct _robots *handle = &(handles.robots[id]);

		if(!(pRobots[id].pHub->isOnVision || pRobots[id].pRobot->online))
		{
			gwinSetVisible(handle->hBot, false);
			continue;
		}

		gwinSetVisible(handle->hBot, true);

		if(pRobots[id].pHub->isOnVision)
			IconsSetColor(handle->hVision, Lime);
		else if(pRobots[id].pRobot->online && visionAvailable)
			IconsSetColor(handle->hVision, Red);
		else
			IconsSetColor(handle->hVision, DarkGray);

		if(!pRobots[id].pRobot->online)
		{
			gwinSetText(handle->hState, "Offline", FALSE);
			gwinSetStyle(handle->hState, &DarkenedWidgetStyle);
			gwinSetStyle(handle->hBatProg, &DarkenedWidgetStyle);
			gwinSetStyle(handle->hKickerProg, &DarkenedWidgetStyle);
			IconsSetColor(handle->hBatIcon, DarkGray);
			IconsSetColor(handle->hKickerIcon, DarkGray);
			IconsSetColor(handle->hWifi, DarkGray);
			IconsSetColor(handle->hFeatures, DarkGray);
			IconsSetColor(handle->hBarrier, DarkGray);

			continue;
		}
		else
		{
			gwinSetStyle(handle->hState, &BlackWidgetStyle);
			gwinSetStyle(handle->hBatProg, &BlackWidgetStyle);
			gwinSetStyle(handle->hKickerProg, &BlackWidgetStyle);
		}

		IconsSetColor(handle->hBatIcon, Lime);
		IconsSetColor(handle->hKickerIcon, Lime);
		IconsSetColor(handle->hWifi, Lime);

		if(pRobots[id].pFeedback->features >> 12 == 1) // ready?
		{
			if((pRobots[id].pFeedback->features & requiredFeatures) == requiredFeatures)
				IconsSetColor(handle->hFeatures, Lime);
			else
				IconsSetColor(handle->hFeatures, Red);
		}
		else
		{
			IconsSetColor(handle->hFeatures, Gray);
		}

		if(pRobots[id].pFeedback->flags & SYSTEM_MATCH_FEEDBACK_FLAGS_BARRIER_MASK)
			IconsSetColor(handle->hBarrier, Red);
		else
			IconsSetColor(handle->hBarrier, Lime);

		setStateText(handle->hState, pRobots[id].pFeedback->features);

		float batVoltage = pRobots[id].pFeedback->batteryLevel*0.1f;
		snprintf(handle->batText, 8, "%4.1fV", batVoltage);
		gwinProgressbarSetPosition(handle->hBatProg, pRobots[id].pFeedback->batteryPercent);
		gwinSetText(handle->hBatProg, handle->batText, FALSE);

		snprintf(handle->kickerText, 8, "%huV", (uint16_t)pRobots[id].pFeedback->kickerLevel);
		gwinProgressbarSetRange(handle->hKickerProg, 0, (int)pRobots[id].pFeedback->kickerMax);
		gwinProgressbarSetPosition(handle->hKickerProg, (int)pRobots[id].pFeedback->kickerLevel);
		gwinSetText(handle->hKickerProg, handle->kickerText, FALSE);
	}

	if(robotStatus.selectedBotId > WIFI_MAX_BOTS)
		return;

	PresenterRobotInfo* pSelected = &pRobots[robotStatus.selectedBotId];

	if(!pSelected->pRobot->online)
	{
		gwinSetText(handles.hState, "Offline", FALSE);
		gwinSetStyle(handles.hState, &BlackWidgetStyle);

		IconsSetColor(handles.hWifiIcon[0], Gray);
		IconsSetColor(handles.hWifiIcon[1], Gray);

		gwinSetStyle(handles.hFeatStraight, &BlackWidgetStyle);
		gwinSetStyle(handles.hFeatChip, &BlackWidgetStyle);
		gwinSetStyle(handles.hFeatCharge, &BlackWidgetStyle);
		gwinSetStyle(handles.hFeatMove,  &BlackWidgetStyle);
		gwinSetStyle(handles.hFeatDribble, &BlackWidgetStyle);
		gwinSetStyle(handles.hFeatBarrier, &BlackWidgetStyle);
		gwinSetStyle(handles.hFeatMBv2016, &BlackWidgetStyle);
		gwinSetStyle(handles.hFeatKDv2017, &BlackWidgetStyle);
		gwinSetStyle(handles.hFeatExt, &BlackWidgetStyle);
		gwinSetStyle(handles.hFeatCover, &BlackWidgetStyle);

		return;
	}

	uint8_t kickCount = pSelected->pFeedback->flags & SYSTEM_MATCH_FEEDBACK_FLAGS_KICK_MASK;

	if(kickCount != lastKickCount)
	{
		lastKickCount = kickCount;

		robotStatus.kickState.mode = KICK_MODE_DISARM;
		robotStatus.kickState.speed = 0;
		gwinSetText(handles.hBtnKickChip, "Chip", FALSE);
		gwinSetText(handles.hBtnKickStraight, "Straight", FALSE);
	}

	setStateText(handles.hState, pSelected->pFeedback->features);

	static char hwIdBuf[12];
	snprintf(hwIdBuf, 12, "HW ID: %hu", (uint16_t)pSelected->pFeedback->hardwareId);
	gwinSetText(handles.hHwId, hwIdBuf, FALSE);

	static char rssiBuf0[12];

	float bsRssi = pSelected->pRobot->emaRssi[0].value;
	if(pSelected->pRobot->emaRssi[1].value > bsRssi)
		bsRssi = pSelected->pRobot->emaRssi[1].value;

	snprintf(rssiBuf0, 12, "%.1fdBm", bsRssi);
	gwinSetText(handles.hWifiRssi[0], rssiBuf0, FALSE);

	static char rssiBuf1[12];
	snprintf(rssiBuf1, 12, "%.1fdBm", pSelected->pRobot->botRssi.value);
	gwinSetText(handles.hWifiRssi[1], rssiBuf1, FALSE);

	IconsSetColor(handles.hWifiIcon[0], Lime);
	IconsSetColor(handles.hWifiIcon[1], Lime);

	float batVoltage = pSelected->pFeedback->batteryLevel*0.1f;
	static char batBuf[8];
	snprintf(batBuf, 8, "%4.1fV", batVoltage);
	gwinProgressbarSetPosition(handles.hBatProg, pSelected->pFeedback->batteryPercent);
	gwinSetText(handles.hBatProg, batBuf, FALSE);

	static char kickerBuf[8];
	snprintf(kickerBuf, 8, "%huV", pSelected->pFeedback->kickerLevel);
	gwinProgressbarSetRange(handles.hKickerProg, 0, (int)pSelected->pFeedback->kickerMax);
	gwinProgressbarSetPosition(handles.hKickerProg, (int)pSelected->pFeedback->kickerLevel);
	gwinSetText(handles.hKickerProg, kickerBuf, FALSE);

	static const char* dribTempText[] = {"Cold", "Warm", "Hot", "Over"};
	uint8_t dribTempId = (pSelected->pFeedback->flags & SYSTEM_MATCH_FEEDBACK_FLAGS_DRIB_TEMP_MASK) >> 5;

	static char dribblerBuf[32];
	snprintf(dribblerBuf, 32, "%5hdRPM / %s", pSelected->pFeedback->dribblerSpeed*100, dribTempText[dribTempId]);
	gwinSetText(handles.hDribbler, dribblerBuf, FALSE);

	if(pSelected->pFeedback->flags & SYSTEM_MATCH_FEEDBACK_FLAGS_BARRIER_MASK)
		gwinSetText(handles.hBarrier, "interrupted", FALSE);
	else
		gwinSetText(handles.hBarrier, "clear", FALSE);

	if(pSelected->pHub->isOnVision)
	{
		static char visionPosBuf[24];
		snprintf(visionPosBuf, 24, "%5.2f %5.2f %5.1f", pSelected->pHub->lastVisionPosFloat[0]*0.001f,
				pSelected->pHub->lastVisionPosFloat[1]*0.001f, pSelected->pHub->lastVisionPosFloat[2]);
		gwinSetText(handles.hVisionPos, visionPosBuf, FALSE);
	}
	else
	{
		gwinSetText(handles.hVisionPos, "not on Vision", FALSE);
	}

	static char botPosBuf[24];
	snprintf(botPosBuf, 24, "%5.2f %5.2f %5.1f", pSelected->pFeedback->curPosition[0]*0.001f,
			pSelected->pFeedback->curPosition[1]*0.001f, pSelected->pFeedback->curPosition[2]*0.001f);
	gwinSetText(handles.hBotPos, botPosBuf, FALSE);

	static char botVelBuf[24];
	snprintf(botVelBuf, 24, "%5.2f %5.2f %5.1f", pSelected->pFeedback->curVelocity[0]*0.001f,
			pSelected->pFeedback->curVelocity[1]*0.001f, pSelected->pFeedback->curVelocity[2]*0.001f);
	gwinSetText(handles.hBotVel, botVelBuf, FALSE);

	gwinSetStyle(handles.hFeatStraight, (pSelected->pFeedback->features & SYSTEM_MATCH_FEEDBACK_FEATURE_STRAIGHT) ? &GreenTextStyle : &RedTextStyle);
	gwinSetStyle(handles.hFeatChip, (pSelected->pFeedback->features & SYSTEM_MATCH_FEEDBACK_FEATURE_CHIP) ? &GreenTextStyle : &RedTextStyle);
	gwinSetStyle(handles.hFeatCharge, (pSelected->pFeedback->features & SYSTEM_MATCH_FEEDBACK_FEATURE_CHARGE) ? &GreenTextStyle : &RedTextStyle);
	gwinSetStyle(handles.hFeatMove, (pSelected->pFeedback->features & SYSTEM_MATCH_FEEDBACK_FEATURE_MOVE) ? &GreenTextStyle : &RedTextStyle);
	gwinSetStyle(handles.hFeatDribble, (pSelected->pFeedback->features & SYSTEM_MATCH_FEEDBACK_FEATURE_DRIBBLE) ? &GreenTextStyle : &RedTextStyle);
	gwinSetStyle(handles.hFeatBarrier, (pSelected->pFeedback->features & SYSTEM_MATCH_FEEDBACK_FEATURE_BARRIER) ? &GreenTextStyle : &RedTextStyle);
	gwinSetStyle(handles.hFeatMBv2016, (pSelected->pFeedback->features & SYSTEM_MATCH_FEEDBACK_FEATURE_V2016) ? &GreenTextStyle : &RedTextStyle);
	gwinSetStyle(handles.hFeatKDv2017, (pSelected->pFeedback->features & SYSTEM_MATCH_FEEDBACK_FEATURE_KICKER_V2017) ? &GreenTextStyle : &RedTextStyle);
	gwinSetStyle(handles.hFeatExt, (pSelected->pFeedback->features & SYSTEM_MATCH_FEEDBACK_FEATURE_EXT_BOARD) ? &GreenTextStyle : &RedTextStyle);
	gwinSetStyle(handles.hFeatCover, (pSelected->pFeedback->features & SYSTEM_MATCH_FEEDBACK_FEATURE_COVER) ? &GreenTextStyle : &RedTextStyle);
}
