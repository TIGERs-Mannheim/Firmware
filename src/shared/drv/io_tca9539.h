#pragma once

#include "hal/i2c.h"
#include "hal/gpio_pin.h"
#include "hal/init_hal.h"

typedef struct _IoTCA9539
{
	I2C* pBus;
	GPIOPin rstPin;

	mutex_t writeMutex;

	int16_t comStatus;

	union
	{
		uint16_t u16;
		uint8_t u8[2];
	} outReg;

	union
	{
		uint16_t u16;
		uint8_t u8[2];
	} cfgReg;

	THD_WORKING_AREA(waTask, 256);
	thread_t* pTask;
} IoTCA9539;

typedef struct _GPIOPinTCA9539
{
	GPIOPinInterface pin;

	IoTCA9539* pTca;
	uint16_t pinMask;
} GPIOPinTCA9539;

void IoTCA9539Init(IoTCA9539* pIo, I2C* pBus, GPIOPin rstPin, tprio_t prio);
void IoTCA9539SetAsInput(IoTCA9539* pIo, uint16_t set, uint16_t reset);
void IoTCA9539SetOutLevel(IoTCA9539* pIo, uint16_t set, uint16_t reset);
void IoTCA9539GPIOPinInit(GPIOPinTCA9539* pPin, IoTCA9539* pIo, uint16_t pinMask);
