/*
 * icons.h
 *
 *  Created on: 29.11.2017
 *      Author: AndreR
 */

#pragma once

#include "gfx.h"

#define ICON_NETWORK 0
#define ICON_WIFI_LARGE 1
#define ICON_WIFI_SMALL 2
#define ICON_LAPTOP 3
#define ICON_EYE 4
#define ICON_WHISTLE 5
#define ICON_CAMERA 6
#define ICON_LASER 7
#define ICON_HEARTBEAT 8
#define ICON_BATTERY 9
#define ICON_LIGHTNING 10
#define ICON_GPS 11
#define ICON_CROSSHAIR 12

GHandle IconsCreate(coord_t x, coord_t y, GHandle hParent, uint8_t type, color_t color);
void IconsSetColor(GHandle hIcon, color_t color);
