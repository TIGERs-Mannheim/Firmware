/*
 * menu.h
 *
 *  Created on: 02.11.2017
 *      Author: AndreR
 */

#pragma once

#include "gfx.h"

GHandle MenuCreate(GHandle* pWindowList, uint16_t numWindows);
int16_t MenuGetSelection(GEvent* pEvent);
