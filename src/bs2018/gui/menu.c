#include "menu.h"

static struct _handles
{
	GHandle* pWindowList;
	GHandle* pButtonList;

	uint16_t numWindows;
} handles;

GHandle MenuCreate(GHandle* pWindowList, uint16_t numWindows)
{
    GWidgetInit contInit = { { 0, 0, 800, 480, FALSE, 0 }, 0, 0, 0, 0 };
    GHandle hTop = gwinContainerCreate(0, &contInit, 0);

    GWidgetInit winInit = { { 0, 0, 120, 120, TRUE, hTop }, 0, 0, 0, 0 };

    handles.pButtonList = gfxAlloc(sizeof(GHandle)*numWindows);
    if(handles.pButtonList == 0)
    	return hTop;

    for(uint16_t i = 0; i < numWindows; i++)
    {
    	if(pWindowList[i] == 0)
    		continue;

    	uint16_t x = 5 + (i % 3)*130;
    	uint16_t y = 5 + (i/3)*130;
    	winInit.g.x = x;
    	winInit.g.y = y;
    	winInit.text = ((GWidgetObject*)pWindowList[i])->text;

    	handles.pButtonList[i] = gwinButtonCreate(0, &winInit);
    	gwinSetFont(handles.pButtonList[i], gdispOpenFont("DejaVuSans16"));
    }

    handles.pWindowList = pWindowList;
    handles.numWindows = numWindows;

	return hTop;
}

int16_t MenuGetSelection(GEvent* pEvent)
{
	if(pEvent->type == GEVENT_GWIN_BUTTON)
	{
		GEventGWinButton* pBtn = (GEventGWinButton*)pEvent;

		for(uint16_t i = 0; i < handles.numWindows; i++)
		{
			if(pBtn->gwin == handles.pButtonList[i])
			{
				return i;
			}
		}
	}

	return -1;
}
