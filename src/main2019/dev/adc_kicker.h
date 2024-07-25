#pragma once

#include "drv/adc_kicker.h"

extern ADCKicker devAdcKicker;

void DevAdcKickerInit(SPI* pSPI, tprio_t taskPrio);
