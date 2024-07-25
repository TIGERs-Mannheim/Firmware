/*
 * top_bar.h
 *
 *  Created on: 09.01.2019
 *      Author: AndreR
 */

#pragma once

#include "gfx.h"

GHandle TopBarCreate(uint8_t bootloader);
uint8_t TopBarIsMenuClicked(GEvent* pEvent);
void TopBarSetTitle(const char* pTitle);
