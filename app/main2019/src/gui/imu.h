/*
 * imu.h
 *
 *  Created on: 23.07.2020
 *      Author: AndreR
 */

#pragma once

#include "gfx.h"

GHandle ImuCreate();
void ImuUpdate(const float* pAcc, const float* pGyr, const float* pMag, float imuTemp, float magTemp);
