/*
 * select_botid.c
 *
 *  Created on: 09.01.2019
 *      Author: AndreR
 */

#include "select_botid.h"
#include "gui/bot_cover.h"
#include "robot/network.h"

static struct _handles
{
	GHandle hBlueIDs[CMD_BOT_COUNT_HALF];
	GHandle hYellowIDs[CMD_BOT_COUNT_HALF];
	GHandle hBtnYellow;
	GHandle hBtnBlue;
	GHandle hYCont;
	GHandle hBCont;
} handles;

static int16_t selectReturnWindowId;

static int16_t eventHandler(GEvent* pEvent)
{
	if(pEvent->type == GEVENT_GWIN_BUTTON)
	{
		GEventGWinButton* pBtn = (GEventGWinButton*)pEvent;

		if(pBtn->gwin == handles.hBtnBlue)
		{
			gwinSetVisible(handles.hYCont, FALSE);
			gwinSetVisible(handles.hBCont, TRUE);
		}

		if(pBtn->gwin == handles.hBtnYellow)
		{
			gwinSetVisible(handles.hBCont, FALSE);
			gwinSetVisible(handles.hYCont, TRUE);
		}

		int16_t newId = -1;
		for(uint8_t i = 0; i < CMD_BOT_COUNT_HALF; i++)
		{
			if(pBtn->gwin == handles.hYellowIDs[i])
			{
				newId = i;
				break;
			}

			if(pBtn->gwin == handles.hBlueIDs[i])
			{
				newId = i+CMD_BOT_COUNT_HALF;
				break;
			}
		}

		if(newId >= 0)
		{
			network.config.botId = (uint8_t)newId;
			NetworkSaveConfig();

			return selectReturnWindowId;
		}
	}

	return -1;
}

GHandle SelectBotIDCreate(int16_t returnWindowId)
{
	selectReturnWindowId = returnWindowId;

    GWidgetInit topContInit = { { 0, 60, 240, 260, FALSE, 0 }, "Bot ID", 0, &eventHandler, 0 };
    GHandle hTop = gwinContainerCreate(0, &topContInit, 0);

    // Yellow/Blue Buttons
    GWidgetInit btnContInit = { { 0, 0, 240, 40, TRUE, hTop }, 0, 0, 0, 0 };
    GHandle hButtons = gwinContainerCreate(0, &btnContInit, 0);

    GWidgetInit btnYellowInit = { { 5, 5, 110, 30, TRUE, hButtons }, "Yellow", 0, 0, 0 };
    handles.hBtnYellow = gwinButtonCreate(0, &btnYellowInit);

    GWidgetInit btnBlueInit = { { 125, 5, 110, 30, TRUE, hButtons }, "Blue", 0, 0, 0 };
    handles.hBtnBlue = gwinButtonCreate(0, &btnBlueInit);

    // Bot covers
    GWidgetInit yContInit = { { 0, 40, 240, 220, TRUE, hTop }, 0, 0, 0, 0 };
    handles.hYCont = gwinContainerCreate(0, &yContInit, 0);

    GWidgetInit bContInit = { { 0, 40, 240, 220, FALSE, hTop }, 0, 0, 0, 0 };
    handles.hBCont = gwinContainerCreate(0, &bContInit, 0);

    for(uint16_t i = 0; i < CMD_BOT_COUNT_HALF; i++)
    {
    	uint16_t row = i/4;
    	uint16_t column = i%4;

    	uint16_t x = column*60;
    	uint16_t y = 2+row*55;

    	handles.hYellowIDs[i] = BotCoverCreate(x, y, handles.hYCont, i, 0);
    	handles.hBlueIDs[i] = BotCoverCreate(x, y, handles.hBCont, CMD_BOT_COUNT_HALF+i, 0);
    }

    return hTop;
}
