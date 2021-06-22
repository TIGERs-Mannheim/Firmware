/*
 * port_ex.c
 *
 *  Created on: 06.01.2019
 *      Author: AndreR
 */

#include "port_ex.h"

#include "util/init_hal.h"

/**
 * TC9539 pinout:
 *  P0: SRST5 (IR)
 *  P1: BOOT5
 *  P2: SRST4 (Dribbler)
 *  P3: BOOT4
 *  P4: SRST1 (Motor 2)
 *  P5: BOOT1
 *  P6:
 *  P7:
 *  P8: SRST2 (Motor 3)
 *  P9: BOOT2
 * P10: SRST3 (Motor 4)
 * P11: BOOT3
 * P12: SRST0 (Motor 1)
 * P13: BOOT0
 * P14:
 * P15:
 */

static struct _PortEx
{
	I2C* pI2C;

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
} portEx;

static const uint8_t target2Pin[6] = { 12, 4, 8, 10, 2, 0 };

static void updateOutputRegister()
{
	uint8_t reg[3] = { 0x02, portEx.outReg.u8[0], portEx.outReg.u8[1] };
	portEx.comStatus = I2CWrite(portEx.pI2C, I2C_ADDR_TCA9539, 3, reg);
	if(portEx.comStatus)
	{
		// comm failed, at least keep it in reset (if possible)
		GPIOReset(GPIOG, GPIO_PIN_13);
	}
}

static void updateConfigRegister()
{
	uint8_t reg[3] = { 0x06, portEx.cfgReg.u8[0], portEx.cfgReg.u8[1] };
	portEx.comStatus = I2CWrite(portEx.pI2C, I2C_ADDR_TCA9539, 3, reg);
	if(portEx.comStatus)
	{
		// comm failed, at least keep it in reset (if possible)
		GPIOReset(GPIOG, GPIO_PIN_13);
	}
}

static void resetConfig()
{
	// Pull reset low
	GPIOReset(GPIOG, GPIO_PIN_13);

	chThdSleepMilliseconds(10);

	// Get TCA9539 out of reset
	GPIOSet(GPIOG, GPIO_PIN_13);

	chThdSleepMilliseconds(10);

	// configure BOOT pins as output
	portEx.cfgReg.u16 = 0b1101010111010101;
	updateConfigRegister();

	portEx.outReg.u16 = 0;
	updateOutputRegister();
}

void PortExInit(I2C* pI2C)
{
	portEx.pI2C = pI2C;

	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_OUTPUT;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;
	gpioInit.pupd = GPIO_PUPD_NONE;
	GPIOInit(GPIOG, GPIO_PIN_13, &gpioInit); // SMB-RST pin for TCA9539

	resetConfig();
}

int16_t PortExResetPulse(uint8_t target)
{
	if(target > 5)
		return -2;

	uint16_t srstPin = (1 << target2Pin[target]);

	portEx.cfgReg.u16 &= ~srstPin;
	updateConfigRegister();

	chThdSleepMilliseconds(50);

	portEx.cfgReg.u16 |= srstPin;
	updateConfigRegister();

	chThdSleepMilliseconds(50);

	return portEx.comStatus;
}

int16_t PortExBootSet(uint8_t target, uint8_t enable)
{
	if(target > 5)
		return -2;

	uint16_t bootPin = (1 << (target2Pin[target]+1));

	if(enable)
		portEx.outReg.u16 |= bootPin;
	else
		portEx.outReg.u16 &= ~bootPin;

	updateOutputRegister();

	return portEx.comStatus;
}
