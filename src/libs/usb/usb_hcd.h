/*
 * usb_hcd.h
 *
 *  Created on: 05.08.2014
 *      Author: AndreR
 */

#pragma once

#include "usb.h"
#include "util/usb_events.h"

#define USB_HCD_EVENT_QUEUE_SIZE 10
#define USB_HCD_MAX_CLASSES 5
#define USB_HCD_MAX_CONFIG_SIZE 128

typedef struct _USBHCDClass
{
	uint8_t isDeviceClass;
	uint8_t classCode;
	const char* pName;
	void (*process)(msg_t event);
	void (*Init)(USBDeviceDesc* pDeviceDesc, uint8_t* pConfigDesc, uint16_t cfgDescSize);
	void (*Stop)();
} USBHCDClass;

typedef void(*USBHCDConnCb)(USBHCDConnectionEvent*);

typedef struct _USBHCDGlobal
{
	USBEndpoint* pCtrlOut;
	USBEndpoint* pCtrlIn;

	msg_t eventQueueData[USB_HCD_EVENT_QUEUE_SIZE];
	mailbox_t eventQueue;

	USBDeviceDesc deviceDesc;
	uint8_t configDesc[USB_HCD_MAX_CONFIG_SIZE];

	uint16_t langCode;
	char manufacturer[USB_MAX_STRING_SIZE];
	char product[USB_MAX_STRING_SIZE];
	char serialNumber[USB_MAX_STRING_SIZE];

	USBHCDClass classes[USB_HCD_MAX_CLASSES];
	uint8_t classesUsed;

	USBHCDClass* pActiveClass;

	USBHCDConnCb pConCb;
	USBHCDConnectionEvent conEv;
} USBHCDGlobal;

extern USBHCDGlobal usbHcd;

void	USBHCDInit();
void	USBHCDTask(void* params);

int16_t	USBCtrlWrite(const USBSetupPkt* pSetup, uint8_t* pTxData);
int16_t	USBCtrlRead(const USBSetupPkt* pSetup, uint8_t* pRxData, uint16_t* pBytesRead);

void	USBHCDPostEvent(msg_t event);
int16_t USBHCDAddClass(USBHCDClass* pClass);
