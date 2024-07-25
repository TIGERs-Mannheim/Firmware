/*
 * top_bar.c
 *
 *  Created on: 09.01.2019
 *      Author: AndreR
 */

#include "top_bar.h"

static struct _handles
{
	GHandle hMenuButton;
	GHandle hTitle;
} handles;

GHandle TopBarCreate(uint8_t bootloader)
{
	font_t deja16 = gdispOpenFont("DejaVuSans16");

    GWidgetInit contInit = { { 0, 0, 240, 60, TRUE, 0 }, 0, 0, 0, 0 };
    GHandle hTop = gwinContainerCreate(0, &contInit, 0);

    GWidgetInit menuBtnInit = { { 0, 0, 60, 60, TRUE, hTop }, "Menu", 0, 0, 0 };
    handles.hMenuButton = gwinButtonCreate(0, &menuBtnInit);
    gwinSetFont(handles.hMenuButton, deja16);

    if(bootloader)
    	gwinSetEnabled(handles.hMenuButton, FALSE);

    GWidgetInit titleInit = { { 80, 0, 160, 60, TRUE, hTop }, "Overview", 0, 0, 0 };
	handles.hTitle = gwinLabelCreate(0, &titleInit);
	gwinSetFont(handles.hTitle, deja16);

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
