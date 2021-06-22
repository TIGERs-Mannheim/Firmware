/*
 * presenter.h
 *
 *  Created on: 09.01.2019
 *      Author: AndreR
 */

#pragma once

#include "ch.h"

void PresenterInit();
void PresenterShowWindow(int16_t newWindow);
void PresenterTask(void* params);
void PresenterPrintHeapStatus();
void PresenterTakeScreenshot();
