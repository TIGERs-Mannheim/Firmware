/*
 * presenter.h
 *
 *  Created on: 24.10.2017
 *      Author: AndreR
 */

#pragma once

#include "ch.h"
#include "commands.h"
#include "wifi.h"
#include "hub.h"

typedef struct _PresenterRobotInfo
{
	SystemMatchFeedback* pFeedback;
	WifiRobot* pRobot;
	HubStorage* pHub;
} PresenterRobotInfo;

void PresenterInit();
void PresenterTask(void* params);
