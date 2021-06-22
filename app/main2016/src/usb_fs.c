/*
 * usb_fs.c
 *
 *  Created on: 06.04.2020
 *      Author: AndreR
 *
 */

#include "usb_fs.h"
#include "util/init_hal.h"
#include "usb/usb.h"
#include "usb/usb_hcd.h"
#include "usb/usb_msc.h"
#include "constants.h"

static void enableVBus(uint8_t enable)
{
	if(enable)
		GPIOSet(GPIOG, GPIO_PIN_10);
	else
		GPIOReset(GPIOG, GPIO_PIN_10);
}

void USBFsInit()
{
	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.alternate = 10;	// OTG_FS
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_50MHZ;
	gpioInit.pupd = GPIO_PUPD_NONE;
	GPIOInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12, &gpioInit);	// USB-DM, USB-DP

	// USB-EN pin
	enableVBus(0);

	gpioInit.mode = GPIO_MODE_OUTPUT;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;
	GPIOInit(GPIOG, GPIO_PIN_10, &gpioInit);

	// Peripheral clock enable
	RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;

	USBInit(&enableVBus, IRQL_USB);

	USBHCDInit();
	USBMSCInit();
}
