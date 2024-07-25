#pragma once

#include "drv/io_tca9539.h"

typedef struct _DevPortExp
{
	IoTCA9539 tca;
	GPIOPinTCA9539 pins[16];
} DevPortExp;

extern DevPortExp devPortExp;

void DevPortExpInit(I2C* pI2C, tprio_t taskPrio);
