/*
 * i2c2.c
 *
 *  Created on: 01.01.2019
 *      Author: AndreR
 */

#include "i2c2.h"

#include "ch.h"
#include "util/init_hal.h"
#include "../constants.h"

I2C i2c2;

void I2C2_EV_IRQHandler()
{
	I2CEventIRQ(&i2c2);
}

void I2C2_ER_IRQHandler()
{
	I2CErrorIRQ(&i2c2);
}

void I2C2Init()
{
	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.alternate = 4;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;
	gpioInit.otype = GPIO_OTYPE_OPEN_DRAIN;
	gpioInit.pupd = GPIO_PUPD_NONE; // externally pulled up by 2k2

	GPIOInit(GPIOF, GPIO_PIN_0 | GPIO_PIN_1, &gpioInit);

	RCC->APB1LENR |= RCC_APB1LENR_I2C2EN;
	__DSB();

	I2CInit(&i2c2, I2C2);

	NVICEnableIRQ(I2C2_EV_IRQn, IRQL_I2C2);
	NVICEnableIRQ(I2C2_ER_IRQn, IRQL_I2C2);
}
