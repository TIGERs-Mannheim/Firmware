/*
 * wifi_settings.h
 *
 *  Created on: 09.01.2019
 *      Author: AndreR
 */

#pragma once

#include "gfx.h"
#include "intercom_constants.h"

GHandle WifiCreate();
void	WifiUpdateConfig(ConfigNetwork* pConfig);
void	WifiUpdateStatus(PresenterWifiStat* pStat);
