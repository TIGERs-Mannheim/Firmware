#pragma once

#include "drv/touch_ad7843.h"

extern TouchAD7843 devTouch;

void DevTouchInit(SPI* pSPI, tprio_t taskPrio);
