/*
 * icons.c
 *
 *  Created on: 29.11.2017
 *      Author: AndreR
 */

#include "icons.h"

static void gwinDrawIconLaptop(GWidgetObject* gw, void* param)
{
	GContainerObject* pCont = (GContainerObject*) gw;
	GWindowObject* pObj = &pCont->g;
	coord_t x = pObj->x;
	coord_t y = pObj->y;
	color_t col = (uint32_t) param;

	gdispFillArea(x, y, pObj->width, pObj->height, Black);

	gdispFillRoundedBox(x + 5, y, 30, 19, 3, col);
	gdispFillArea(x + 8, y + 3, 24, 13, Black);

	point lower[4] = {
		{ 5, 21 },
		{ 35, 21 },
		{ 40, 29 },
		{ 0, 29 }, };

	gdispFillConvexPoly(x, y, lower, 4, col);
}

static void gwinDrawIconEye(GWidgetObject* gw, void* param)
{
	GContainerObject* pCont = (GContainerObject*) gw;
	GWindowObject* pObj = &pCont->g;
	coord_t x = pObj->x;
	coord_t y = pObj->y;
	color_t col = (uint32_t) param;

	gdispFillArea(x, y, pObj->width, pObj->height, Black);

	// 40x22
	gdispDrawArc(x + 20, y + 21, 21, 30, 150, col);
	gdispDrawArc(x + 20, y + 21, 20, 30, 150, col);

	gdispDrawArc(x + 20, y, 21, 210, 330, col);
	gdispDrawArc(x + 20, y, 20, 210, 330, col);

	gdispFillCircle(x + 20, y + 11, 5, col);
	gdispFillCircle(x + 20, y + 11, 3, Black);
}

static void gwinDrawIconWifiLarge(GWidgetObject* gw, void* param)
{
	GContainerObject* pCont = (GContainerObject*) gw;
	GWindowObject* pObj = &pCont->g;
	coord_t x = pObj->x;
	coord_t y = pObj->y;
	color_t col = (uint32_t) param;

	gdispFillArea(x, y, pObj->width, pObj->height, Black);

	gdispFillArc(x + 24, y + 44, 43, 60, 120, col);
	gdispFillArc(x + 24, y + 44, 39, 55, 125, Black);

	gdispFillArc(x + 24, y + 44, 31, 60, 120, col);
	gdispFillArc(x + 24, y + 44, 27, 55, 125, Black);

	gdispFillArc(x + 24, y + 44, 19, 60, 120, col);
	gdispFillArc(x + 24, y + 44, 15, 55, 125, Black);

	gdispFillCircle(x + 24, y + 44, 5, col);
}

static void gwinDrawIconWifiSmall(GWidgetObject* gw, void* param)
{
	GContainerObject* pCont = (GContainerObject*) gw;
	GWindowObject* pObj = &pCont->g;
	coord_t x = pObj->x;
	coord_t y = pObj->y;
	color_t col = (uint32_t) param;

	gdispFillArea(x, y, pObj->width, pObj->height, Black);

	gdispFillArc(x + 10, y + 21, 21, 60, 120, col);
	gdispFillArc(x + 10, y + 21, 19, 55, 125, Black);

	gdispFillArc(x + 10, y + 21, 15, 60, 120, col);
	gdispFillArc(x + 10, y + 21, 13, 55, 125, Black);

	gdispFillArc(x + 10, y + 21, 9, 60, 120, col);
	gdispFillArc(x + 10, y + 21, 7, 55, 125, Black);

	gdispFillCircle(x + 10, y + 21, 3, col);
}

static void gwinDrawIconNetwork(GWidgetObject* gw, void* param)
{
	GContainerObject* pCont = (GContainerObject*) gw;
	GWindowObject* pObj = &pCont->g;
	coord_t x = pObj->x;
	coord_t y = pObj->y;
	color_t col = (uint32_t) param;

	gdispFillArea(x, y, pObj->width, pObj->height, Black);

	// horizontal bus line
	gdispDrawThickLine(x, y + 25, x + 50, y + 25, col, 5, FALSE);

	// upper node connect
	gdispDrawThickLine(x + 25, y + 12, x + 25, y + 25, col, 5, FALSE);

	// lower node connects
	gdispDrawThickLine(x + 9, y + 25, x + 9, y + 37, col, 5, FALSE);
	gdispDrawThickLine(x + 41, y + 25, x + 41, y + 37, col, 5, FALSE);

	// lower left node
	gdispFillArea(x, y + 37, 17, 12, col);
	gdispFillArea(x + 2, y + 39, 13, 8, Black);

	// lower right node
	gdispFillArea(x + 32, y + 37, 17, 12, col);
	gdispFillArea(x + 34, y + 39, 13, 8, Black);

	// upper node
	gdispFillArea(x + 16, y, 17, 12, col);
	gdispFillArea(x + 18, y + 2, 13, 8, Black);
}

static void gwinDrawIconCamera(GWidgetObject* gw, void* param)
{
	GContainerObject* pCont = (GContainerObject*) gw;
	GWindowObject* pObj = &pCont->g;
	coord_t x = pObj->x;
	coord_t y = pObj->y;
	color_t col = (uint32_t) param;

	gdispFillArea(x, y, pObj->width, pObj->height, Black);

	// camera body
	gdispFillRoundedBox(x, y + 20, 35, 25, 3, col);
	gdispFillRoundedBox(x + 4, y + 24, 27, 17, 3, Black);

	// camera lens
	gdispDrawThickLine(x + 35, y + 30, x + 48, y + 20, col, 4, TRUE);
	gdispDrawThickLine(x + 35, y + 35, x + 48, y + 45, col, 4, TRUE);
	gdispDrawThickLine(x + 48, y + 20, x + 48, y + 45, col, 4, TRUE);

	// rear film roll
	gdispFillCircle(x + 8, y + 12, 8, col);
	gdispFillCircle(x + 8, y + 12, 5, Black);

	// front film roll
	gdispFillCircle(x + 26, y + 10, 10, col);
	gdispFillCircle(x + 26, y + 10, 7, Black);
}

static void gwinDrawIconWhistle(GWidgetObject* gw, void* param)
{
	GContainerObject* pCont = (GContainerObject*) gw;
	GWindowObject* pObj = &pCont->g;
	coord_t x = pObj->x;
	coord_t y = pObj->y;
	color_t col = (uint32_t) param;

	gdispFillArea(x, y, pObj->width, pObj->height, Black);

	// front blow-in
	gdispFillArea(x + 10, y + 6, 22, 7, col);

	// little cut-out
	gdispFillArea(x + 20, y + 6, 6, 2, Black);

	// shrieking lines from cut-out
	gdispDrawLine(x + 24, y + 4, x + 20, y, col);
	gdispDrawLine(x + 20, y + 4, x + 16, y, col);

	// body
	gdispFillCircle(x + 10, y + 16, 10, col);
	gdispFillCircle(x + 10, y + 16, 4, Black);
}

static void gwinDrawIconLaser(GWidgetObject* gw, void* param)
{
	GContainerObject* pCont = (GContainerObject*) gw;
	GWindowObject* pObj = &pCont->g;
	coord_t x = pObj->x;
	coord_t y = pObj->y;
	color_t col = (uint32_t) param;

	gdispFillArea(x, y, pObj->width, pObj->height, Black);

	// primary ray
	gdispDrawThickLine(x+1, y + 12, x + 10, y + 12, col, 2, TRUE);

	// splash rays
	gdispDrawLine(x + 10, y + 3, x + 10, y + 21, col); //x=15 ist max
	gdispDrawLine(x + 7, y + 7, x + 13, y + 17, col);
	gdispDrawLine(x + 13, y + 7, x + 7, y + 17, col);
	gdispDrawLine(x + 10, y + 12, x + 13, y + 12, col);
}

static void gwinDrawIconHeartbeat(GWidgetObject* gw, void* param)
{
	GContainerObject* pCont = (GContainerObject*) gw;
	GWindowObject* pObj = &pCont->g;
	coord_t x = pObj->x;
	coord_t y = pObj->y;
	color_t col = (uint32_t) param;

	gdispFillArea(x, y, pObj->width, pObj->height, Black);

	// monitor frame
	gdispFillRoundedBox(x, y+3, 20, 20, 3, col);
	gdispFillRoundedBox(x + 2, y + 5, 16, 16, 3, Black);

	// graph
	gdispDrawLine(x+2, y + 12, x + 5, y + 12, col);
	gdispDrawLine(x+5, y + 12, x + 7, y + 9, col);
	gdispDrawLine(x+7, y + 9, x + 10, y + 18, col);
	gdispDrawLine(x+10, y + 18, x + 13, y + 7, col);
	gdispDrawLine(x+13, y + 7, x + 15, y + 12, col);
	gdispDrawLine(x+15, y + 12, x + 19, y + 12, col);
}

static void gwinDrawIconBattery(GWidgetObject* gw, void* param)
{
	GContainerObject* pCont = (GContainerObject*) gw;
	GWindowObject* pObj = &pCont->g;
	coord_t x = pObj->x;
	coord_t y = pObj->y;
	color_t col = (uint32_t) param;

	gdispFillArea(x, y, pObj->width, pObj->height, Black);

	// main body
	gdispDrawBox(x+3, y+2, 9, 13, col);

	// top connection
	gdispFillArea(x+5, y, 5, 2, col);

	// bars
	gdispFillArea(x+5, y+6, 5, 3, col);
	gdispFillArea(x+5, y+10, 5, 3, col);
}

static void gwinDrawIconLightning(GWidgetObject* gw, void* param)
{
	GContainerObject* pCont = (GContainerObject*) gw;
	GWindowObject* pObj = &pCont->g;
	coord_t x = pObj->x;
	coord_t y = pObj->y;
	color_t col = (uint32_t) param;

	gdispFillArea(x, y, pObj->width, pObj->height, Black);

	gdispDrawLine(x+7, y, x+4, y+7, col);
	gdispDrawLine(x+4, y+7, x+9, y+5, col);
	gdispDrawLine(x+9, y+5, x+6, y+14, col);
	gdispDrawLine(x+6, y+14, x+5, y+11, col);
	gdispDrawLine(x+6, y+14, x+9, y+11, col);
}

static void gwinDrawIconCrosshair(GWidgetObject* gw, void* param)
{
	GContainerObject* pCont = (GContainerObject*) gw;
	GWindowObject* pObj = &pCont->g;
	coord_t x = pObj->x;
	coord_t y = pObj->y;
	color_t col = (uint32_t) param;

	gdispFillArea(x, y, pObj->width, pObj->height, Black);

	gdispDrawCircle(x+175, y+175, 150, Red);
	gdispDrawCircle(x+175, y+175, 120, Orange);
	gdispDrawCircle(x+175, y+175, 90, Yellow);
	gdispDrawCircle(x+175, y+175, 60, Lime);
	gdispDrawCircle(x+175, y+175, 30, Green);

	gdispDrawLine(x+5, y+175, x+345, y+175, col);
	gdispDrawLine(x+175, y+5, x+175, y+345, col);
}

static void gwinDrawIconGps(GWidgetObject* gw, void* param)
{
	GContainerObject* pCont = (GContainerObject*) gw;
	GWindowObject* pObj = &pCont->g;
	coord_t x = pObj->x;
	coord_t y = pObj->y;
	color_t col = (uint32_t) param;

	gdispFillArea(x, y, pObj->width, pObj->height, Black);

	gdispDrawEllipse(x+10, y+21, 5, 3, col);

	point tri[3] = {
		{ 0, 8 },
		{ 20, 8 },
		{ 10, 23 }, };

	gdispFillConvexPoly(x, y, tri, 3, Black);

	point tri2[3] = {
		{ 4, 8 },
		{ 16, 8 },
		{ 10, 22 }, };

	gdispFillConvexPoly(x, y, tri2, 3, col);

	gdispFillCircle(x+10, y+8, 8, col);
	gdispFillCircle(x+10, y+8, 4, Black);
}

GHandle IconsCreate(coord_t x, coord_t y, GHandle hParent, uint8_t type, color_t color)
{
	GHandle hIcon = 0;
	uint32_t col = color;

	switch(type)
	{
		case ICON_NETWORK:
		{
			GWidgetInit iconInit = {{ x, y, 50, 50, TRUE, hParent }, 0, &gwinDrawIconNetwork, (void*) col, 0 };
			hIcon = gwinContainerCreate(0, &iconInit, 0);
		}
		break;
		case ICON_LAPTOP:
		{
			GWidgetInit iconInit = {{ x, y, 40, 30, TRUE, hParent }, 0, &gwinDrawIconLaptop, (void*) col, 0 };
			hIcon = gwinContainerCreate(0, &iconInit, 0);
		}
		break;
		case ICON_WIFI_LARGE:
		{
			GWidgetInit iconInit = {{ x, y, 50, 50, TRUE, hParent }, 0, &gwinDrawIconWifiLarge, (void*) col, 0 };
			hIcon = gwinContainerCreate(0, &iconInit, 0);
		}
		break;
		case ICON_WIFI_SMALL:
		{
			GWidgetInit iconInit = {{ x, y, 20, 25, TRUE, hParent }, 0, &gwinDrawIconWifiSmall, (void*) col, 0 };
			hIcon = gwinContainerCreate(0, &iconInit, 0);
		}
		break;
		case ICON_EYE:
		{
			GWidgetInit iconInit = {{ x, y, 40, 22, TRUE, hParent }, 0, &gwinDrawIconEye, (void*) col, 0 };
			hIcon = gwinContainerCreate(0, &iconInit, 0);
		}
		break;
		case ICON_WHISTLE:
		{
			GWidgetInit iconInit = {{ x, y, 32, 26, TRUE, hParent }, 0, &gwinDrawIconWhistle, (void*) col, 0 };
			hIcon = gwinContainerCreate(0, &iconInit, 0);
		}
		break;
		case ICON_CAMERA:
		{
			GWidgetInit iconInit = {{ x, y, 50, 50, TRUE, hParent }, 0, &gwinDrawIconCamera, (void*) col, 0 };
			hIcon = gwinContainerCreate(0, &iconInit, 0);
		}
		break;
		case ICON_LASER:
		{
			GWidgetInit iconInit = {{ x, y, 20, 25, TRUE, hParent }, 0, &gwinDrawIconLaser, (void*) col, 0 };
			hIcon = gwinContainerCreate(0, &iconInit, 0);
		}
		break;
		case ICON_HEARTBEAT:
		{
			GWidgetInit iconInit = {{ x, y, 20, 25, TRUE, hParent }, 0, &gwinDrawIconHeartbeat, (void*) col, 0 };
			hIcon = gwinContainerCreate(0, &iconInit, 0);
		}
		break;
		case ICON_BATTERY:
		{
			GWidgetInit iconInit = {{ x, y, 15, 15, TRUE, hParent }, 0, &gwinDrawIconBattery, (void*) col, 0 };
			hIcon = gwinContainerCreate(0, &iconInit, 0);
		}
		break;
		case ICON_LIGHTNING:
		{
			GWidgetInit iconInit = {{ x, y, 15, 15, TRUE, hParent }, 0, &gwinDrawIconLightning, (void*) col, 0 };
			hIcon = gwinContainerCreate(0, &iconInit, 0);
		}
		break;
		case ICON_GPS:
		{
			GWidgetInit iconInit = {{ x, y, 20, 25, TRUE, hParent }, 0, &gwinDrawIconGps, (void*) col, 0 };
			hIcon = gwinContainerCreate(0, &iconInit, 0);
		}
		break;
		case ICON_CROSSHAIR:
		{
			GWidgetInit iconInit = {{ x, y, 350, 350, TRUE, hParent }, 0, &gwinDrawIconCrosshair, (void*) col, 0 };
			hIcon = gwinContainerCreate(0, &iconInit, 0);
		}
		break;
		default:
			break;
	}

	return hIcon;
}

void IconsSetColor(GHandle hIcon, color_t color)
{
	uint32_t val = color;
	((GWidgetObject *) hIcon)->fnParam = (void*) val;
	gwinRedraw(hIcon);
}
