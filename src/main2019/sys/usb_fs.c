/*
 * usb_fs.c
 *
 *  Created on: 06.04.2020
 *      Author: AndreR
 *
 * Full-Speed USB host (OTG_HS2).
 */

#include "usb_fs.h"
#include "hal/init_hal.h"
#include "hal/usb/usb.h"
#include "hal/usb/usb_hcd.h"
#include "hal/usb/usb_msc.h"
#include "constants.h"

static void enableVBus(uint8_t enable)
{
	if(enable)
		GPIOSet(GPIOF, GPIO_PIN_14);
	else
		GPIOReset(GPIOF, GPIO_PIN_14);
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
	GPIOInit(GPIOF, GPIO_PIN_14, &gpioInit);

	// OTG_HS2 used on OTG_FS_DM/DP
	// Peripheral clock enable
	RCC->AHB1ENR |= RCC_AHB1ENR_USB2OTGHSEN;
	__DSB();

	USBInit(&enableVBus, IRQL_USB);

	USBHCDInit();
	USBMSCInit();
}
