/*
 * bot_cover.c
 *
 *  Created on: 05.10.2015
 *      Author: AndreR
 */

#include "bot_cover.h"
#include "commands.h"
#include <stdio.h>

static const uint16_t botCovers[16][4] = {
	{Magenta, Magenta, Lime, Magenta},
	{Lime, Magenta, Lime, Magenta},
	{Lime, Lime, Lime, Magenta},
	{Magenta, Lime, Lime, Magenta},
	{Magenta, Magenta, Magenta, Lime},
	{Lime, Magenta, Magenta, Lime},
	{Lime, Lime, Magenta, Lime},
	{Magenta, Lime, Magenta, Lime},
	{Lime, Lime, Lime, Lime},
	{Magenta, Magenta, Magenta, Magenta},
	{Magenta, Magenta, Lime, Lime},
	{Lime, Lime, Magenta, Magenta},
	{Lime, Magenta, Lime, Lime},
	{Lime, Magenta, Magenta, Magenta},
	{Magenta, Lime, Lime, Lime},
	{Magenta, Lime, Magenta, Magenta}
};

static void gwinDrawBotCover(GWidgetObject* gw, void* param)
{
	GButtonObject* pBtn = (GButtonObject*)gw;
	GWindowObject* pObj = &pBtn->w.g;
	coord_t x = pObj->x;
	coord_t y = pObj->y;
	uint32_t id = (uint32_t)param;
	uint8_t blue = 0;

	// size assumed 60x50

	if(pObj->flags & 0x01)
		gdispFillArea(x, y, pObj->width, pObj->height, Maroon);
	else
		gdispFillArea(x, y, pObj->width, pObj->height, Black);

	gdispFillCircle(x+30, y+20, 28, Black);
	gdispDrawCircle(x+30, y+20, 29, Gray);
	gdispDrawLine(x+8, y, x+52, y, Gray);

	if(id > CMD_BOT_LAST_ID)
		return;

	if(id >= CMD_BOT_COUNT_HALF)
	{
		id -= CMD_BOT_COUNT_HALF;
		blue = 1;
	}

	// center color
	gdispFillCircle(x+30, y+21, 10, blue ? Blue : Yellow);

	// lower right
	gdispFillCircle(x+12, y+9, 7, botCovers[id][0]);

	// lower left
	gdispFillCircle(x+48, y+9, 7, botCovers[id][1]);

	// upper right
	gdispFillCircle(x+18, y+39, 7, botCovers[id][2]);

	// upper left
	gdispFillCircle(x+42, y+39, 7, botCovers[id][3]);

	static char buf[4];
	snprintf(buf, 4, "%u", (unsigned int)id);

	gdispDrawStringBox(x+20, y+13, 20, 20, buf, gdispOpenFont("DejaVuSans16"), blue ? White : Black, justifyCenter);
}

GHandle BotCoverCreate(coord_t x, coord_t y, GHandle hParent, uint32_t id, GButtonObject* pInstance)
{
	GWidgetInit idInit = { { x, y, 60, 50, TRUE, hParent }, 0, &gwinDrawBotCover, (void*)id, 0 };
	GHandle hId = gwinButtonCreate(pInstance, &idInit);

	return hId;
}

void BotCoverSetId(GHandle hCover, uint32_t id)
{
	gwinSetCustomDraw(hCover, &gwinDrawBotCover, (void*)id);
}
