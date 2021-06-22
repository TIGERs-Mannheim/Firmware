/*
 * dribbler.h
 *
 *  Created on: 17.08.2020
 *      Author: AndreR
 */

#pragma once

#include "gfx.h"

GHandle DribblerCreate();
void DribblerUpdate(float temp, float volt, float speed, float cur);
