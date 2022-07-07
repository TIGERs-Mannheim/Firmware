/*
 * main_status.h
 *
 *  Created on: 09.01.2019
 *      Author: AndreR
 */

#pragma once

#include "gfx.h"
#include "intercom_constants.h"

typedef void(*MainStatusKickerDischargeCallback)();

GHandle MainStatusCreate(MainStatusKickerDischargeCallback dischageCallback, int16_t idSelectWindowId, uint8_t kickerV2017);
void	MainStatusKdUpdate(PresenterMainStatus* pFb);
void	MainStatusConfigNetworkUpdate(ConfigNetwork* pCfg);
void	MainStatusHwIdUpdate(uint8_t id);
void	MainStatusWifiUpdate(PresenterWifiStat* pStat);
