/*
 * top_bar.h
 *
 *  Created on: 02.11.2017
 *      Author: AndreR
 */

#pragma once

#include "gfx.h"

GHandle TopBarCreate();
uint8_t TopBarIsMenuClicked(GEvent* pEvent);

void TopBarSetTitle(const char* pTitle);
void TopBarSetTime(uint32_t unixTimestamp);
void TopBarSetEthStats(uint32_t rx, uint32_t tx);
void TopBarSetEthIP(uint8_t* ip, uint16_t port);
void TopBarSetEthLinkStatus(uint8_t up, uint8_t speed100M);
void TopBarSetWifiStatus(uint16_t channel, uint16_t botsOnline, float link);
void TopBarSetRemotes(uint8_t vision, uint8_t sumatra);
