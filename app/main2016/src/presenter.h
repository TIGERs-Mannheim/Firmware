/*
 * presenter.h
 *
 *  Created on: 31.10.2015
 *      Author: AndreR
 */

#pragma once

#include "ch.h"

void PresenterInit();
void PresenterShowWindow(int16_t newWindow);
void PresenterTask(void* params);
