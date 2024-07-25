#pragma once

#include "ch.h"
#include "commands.h"
#include "router.h"
#include "util/ssl_vision.h"
#include "module/radio/radio_base.h"
#include "module/radio/radio_settings.h"

typedef struct _PresenterRobotInfo
{
	const RouterClient* pRouterClient;
	const SslVisionObject* pVisionObj;
	const RadioClientRx* pRadioClient;
} PresenterRobotInfo;

typedef struct _Presenter
{
	PresenterRobotInfo robotInfo[RADIO_NUM_ROBOT_CLIENTS];

	THD_WORKING_AREA(waTask, 2048);
	thread_t* pTask;
} Presenter;

extern Presenter presenter;

void PresenterInit();
void PresenterPrintHeapStatus();
