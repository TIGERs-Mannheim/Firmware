/*
 * menu.h
 *
 *  Created on: 09.01.2019
 *      Author: AndreR
 */

#pragma once

#include "gfx.h"

GHandle MenuCreate(GHandle* pWindowList, uint16_t numWindows);
int16_t MenuGetSelection(GEvent* pEvent);
void MenuSetEnabled(uint16_t windowId, bool_t enabled);
