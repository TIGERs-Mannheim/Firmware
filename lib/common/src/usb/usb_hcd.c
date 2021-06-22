/*
 * usb_hcd.c
 *
 *  Created on: 05.08.2014
 *      Author: AndreR
 */

#include "usb_hcd.h"
#include "util/console.h"
#include <string.h>
#include <stdio.h>

USBHCDGlobal usbHcd;

#define RX_BUF_SIZE 256

#define EVENT_CONNECT		0x40000000
#define EVENT_DISCONNECT	0x80000000

static const USBSetupPkt setupDeviceDescInitial = {
		.bmRequestType = USB_D2H | USB_REQ_TYPE_STANDARD | USB_REQ_RECIPIENT_DEVICE,
		.bRequest = USB_REQ_GET_DESCRIPTOR,
		.wValue = (USB_DESC_TYPE_DEVICE << 8),
		.wIndex = 0,
		.wLength = 8,
};

static const USBSetupPkt setupDeviceDescFull = {
		.bmRequestType = USB_D2H | USB_REQ_TYPE_STANDARD | USB_REQ_RECIPIENT_DEVICE,
		.bRequest = USB_REQ_GET_DESCRIPTOR,
		.wValue = (USB_DESC_TYPE_DEVICE << 8),
		.wIndex = 0,
		.wLength = sizeof(USBDeviceDesc),
};

static const USBSetupPkt setupGetFirstLanguageCode = {
		.bmRequestType = USB_D2H | USB_REQ_TYPE_STANDARD | USB_REQ_RECIPIENT_DEVICE,
		.bRequest = USB_REQ_GET_DESCRIPTOR,
		.wValue = (USB_DESC_TYPE_STRING << 8),
		.wIndex = 0,
		.wLength = 4,
};

static const USBSetupPkt setupSetAddress = {
		.bmRequestType = USB_H2D | USB_REQ_TYPE_STANDARD | USB_REQ_RECIPIENT_DEVICE,
		.bRequest = USB_REQ_SET_ADDRESS,
		.wValue = 1,
		.wIndex = 0,
		.wLength = 0,
};

static USBSetupPkt setupSetConfiguration = {
		.bmRequestType = USB_H2D | USB_REQ_TYPE_STANDARD | USB_REQ_RECIPIENT_DEVICE,
		.bRequest = USB_REQ_SET_CONFIGURATION,
		.wValue = 1,
		.wIndex = 0,
		.wLength = 0,
};

static USBSetupPkt setupGetDeviceString = {
		.bmRequestType = USB_D2H | USB_REQ_TYPE_STANDARD | USB_REQ_RECIPIENT_DEVICE,
		.bRequest = USB_REQ_GET_DESCRIPTOR,
		.wValue = (USB_DESC_TYPE_STRING << 8),
		.wIndex = 0,	// set before executing
		.wLength = USB_MAX_STRING_SIZE*2+2,
};

static USBSetupPkt setupGetConfigDesc = {
		.bmRequestType = USB_D2H | USB_REQ_TYPE_STANDARD | USB_REQ_RECIPIENT_DEVICE,
		.bRequest = USB_REQ_GET_DESCRIPTOR,
		.wValue = (USB_DESC_TYPE_CONFIGURATION << 8),
		.wIndex = 0,	// configuration index
		.wLength = USB_HCD_MAX_CONFIG_SIZE,
};

static uint8_t rxBuf[RX_BUF_SIZE];

static void connectCallback(uint8_t connect)
{
	if(connect)
		chMBPost(&usbHcd.eventQueue, EVENT_CONNECT, MS2ST(200));
	else
		chMBPost(&usbHcd.eventQueue, EVENT_DISCONNECT, MS2ST(200));
}

void USBHCDInit()
{
	chMBObjectInit(&usbHcd.eventQueue, usbHcd.eventQueueData, USB_HCD_EVENT_QUEUE_SIZE);

	usb.conCallback = &connectCallback;
}


int16_t USBCtrlWrite(const USBSetupPkt* pSetup, uint8_t* pTxData)
{
	USBIRP setupIRP;
	setupIRP.pData = (uint8_t*)pSetup;
	setupIRP.dataLength = sizeof(USBSetupPkt);
	setupIRP.pid = USB_PID_MDATA_SETUP;

	USBIRP txIRP;
	txIRP.pData = pTxData;
	txIRP.dataLength = pSetup->wLength;
	txIRP.pid = USB_PID_DATA1;

	USBIRP statusIRP;
	statusIRP.pData = 0;
	statusIRP.dataLength = 0;
	statusIRP.pid = USB_PID_DATA1;

	int16_t result = USBIRPTransfer(usbHcd.pCtrlOut, &setupIRP);
	if(result)
		return result;

	if(pSetup->wLength)
	{
		result = USBIRPTransfer(usbHcd.pCtrlOut, &txIRP);
		if(result)
			return result;
	}

	result = USBIRPTransfer(usbHcd.pCtrlIn, &statusIRP);
	if(result)
		return result;

	return 0;
}

int16_t USBCtrlRead(const USBSetupPkt* pSetup, uint8_t* pRxData, uint16_t* pBytesRead)
{
	USBIRP setupIRP;
	setupIRP.pData = (uint8_t*)pSetup;
	setupIRP.dataLength = sizeof(USBSetupPkt);
	setupIRP.pid = USB_PID_MDATA_SETUP;

	USBIRP rxIRP;
	rxIRP.pData = pRxData;
	rxIRP.dataLength = pSetup->wLength;
	rxIRP.pid = USB_PID_DATA1;

	USBIRP statusIRP;
	statusIRP.pData = 0;
	statusIRP.dataLength = 0;
	statusIRP.pid = USB_PID_DATA1;

	if(pBytesRead)
		*pBytesRead = 0;

	int16_t result = USBIRPTransfer(usbHcd.pCtrlOut, &setupIRP);
	if(result)
		return result;

	result = USBIRPTransfer(usbHcd.pCtrlIn, &rxIRP);
	if(result)
		return result;

	result = USBIRPTransfer(usbHcd.pCtrlOut, &statusIRP);
	if(result)
		return result;

	if(pBytesRead)
		*pBytesRead = rxIRP.bytesDone;

	return 0;
}

int16_t USBGetString(uint8_t index, char* pDest, uint16_t dstLen)
{
	if(index == 0)
	{
		snprintf(pDest, dstLen, "N/A");
		return 0;
	}

	setupGetDeviceString.wValue = (USB_DESC_TYPE_STRING << 8) | index;
	setupGetDeviceString.wIndex = usbHcd.langCode;
	int16_t result = USBCtrlRead(&setupGetDeviceString, rxBuf, 0);
	if(result)
	{
		ConsolePrint("Reading string failed: 0x%04X\r\n", result);
		return result;
	}


	USBDescHeader* pDescHeader = (USBDescHeader*)rxBuf;
	if(pDescHeader->bLength/2 > dstLen)
		pDescHeader->bLength = dstLen*2;

	for(uint8_t i = 0; i < pDescHeader->bLength-2; i += 2)
	{
		pDest[i/2] = rxBuf[i+2];
	}

	pDest[pDescHeader->bLength-1] = 0;

	return 0;
}

int16_t USBHCDAddClass(USBHCDClass* pClass)
{
	if(usbHcd.classesUsed == USB_HCD_MAX_CLASSES)
		return 1;

	usbHcd.classes[usbHcd.classesUsed++] = *pClass;

	return 0;
}

void USBHCDPostEvent(msg_t event)
{
	chMBPost(&usbHcd.eventQueue, event, TIME_INFINITE);
}

void USBHCDTask(void* params)
{
	(void)params;
	msg_t event;

	chRegSetThreadName("USB-HCD");

	while(1)
	{
		if(chMBFetch(&usbHcd.eventQueue, &event, MS2ST(10)) != MSG_OK)
		{
			if(usbHcd.pActiveClass)
				(*usbHcd.pActiveClass->process)(0);

			continue;
		}

		if((event & (EVENT_DISCONNECT | EVENT_CONNECT)) == 0)
		{
			if(usbHcd.pActiveClass)
				(*usbHcd.pActiveClass->process)(event);
		}

		if(event & EVENT_DISCONNECT)
		{
			if(usbHcd.pActiveClass)
				(*usbHcd.pActiveClass->Stop)();

			memset(usbHcd.manufacturer, 0, USB_MAX_STRING_SIZE);
			memset(usbHcd.product, 0, USB_MAX_STRING_SIZE);
			memset(usbHcd.serialNumber, 0, USB_MAX_STRING_SIZE);
			usbHcd.langCode = 0;
			usbHcd.pCtrlIn = 0;
			usbHcd.pCtrlOut = 0;
			usbHcd.pActiveClass = 0;

			usbHcd.conEv.classCode = 0;
			usbHcd.conEv.connect = 0;
			usbHcd.conEv.pManufacturer = "";
			usbHcd.conEv.pName = "N/A";
			usbHcd.conEv.pProduct = "";
			usbHcd.conEv.pSerialNumber = "";

			if(usbHcd.pConCb)
				(*usbHcd.pConCb)(&usbHcd.conEv);
		}

		if(event & EVENT_CONNECT)
		{
			chThdSleepMilliseconds(200);

			// configure control EPs
			USBEndpointCfg ctrlOut;
			ctrlOut.epIn = 0;
			ctrlOut.epNumber = 0;
			ctrlOut.maxPktSize = 8;
			ctrlOut.type = USB_EPTYPE_CONTROL;
			ctrlOut.timeout = 0;

			USBEndpointCfg ctrlIn;
			ctrlIn.epIn = 1;
			ctrlIn.epNumber = 0;
			ctrlIn.maxPktSize = 8;
			ctrlIn.type = USB_EPTYPE_CONTROL;
			ctrlIn.timeout = 0;

			usbHcd.pCtrlOut = USBEndpointCreate(&ctrlOut);
			usbHcd.pCtrlIn = USBEndpointCreate(&ctrlIn);

			int16_t result = USBCtrlRead(&setupDeviceDescInitial, rxBuf, 0);
			if(result)
			{
				ConsolePrint("Reading initial device desc failed: 0x%04X\r\n", result);
				continue;
			}

			USBDeviceDesc* pDevDesc = (USBDeviceDesc*)rxBuf;
			if(pDevDesc->bMaxPacketSize != ctrlOut.maxPktSize)
			{
				ConsolePrint("Max packet size: %hu, reconfiguring endpoints...\r\n", (uint16_t)pDevDesc->bMaxPacketSize);

				chThdSleepMilliseconds(10);

				USBEndpointDelete(usbHcd.pCtrlOut);
				USBEndpointDelete(usbHcd.pCtrlIn);

				ctrlOut.maxPktSize = pDevDesc->bMaxPacketSize;
				ctrlIn.maxPktSize = pDevDesc->bMaxPacketSize;

				usbHcd.pCtrlOut = USBEndpointCreate(&ctrlOut);
				usbHcd.pCtrlIn = USBEndpointCreate(&ctrlIn);
			}

			// --- control pipe open and properly configured

			// set address
			result = USBCtrlWrite(&setupSetAddress, 0);
			if(result)
			{
				ConsolePrint("Set address failed: 0x%04X\r\n", result);
				continue;
			}

			chThdSleepMilliseconds(10);
			USBChangeDeviceAddress(1);

			// read full device descriptor
			result = USBCtrlRead(&setupDeviceDescFull, rxBuf, 0);
			if(result)
			{
				ConsolePrint("Reading full device desc failed: 0x%04X\r\n", result);
				continue;
			}

			memcpy(&usbHcd.deviceDesc, pDevDesc, sizeof(USBDeviceDesc));

			// read first language ID
			result = USBCtrlRead(&setupGetFirstLanguageCode, rxBuf, 0);
			if(result)
			{
				ConsolePrint("Reading first language code failed: 0x%04X\r\n", result);
				continue;
			}

			usbHcd.langCode = *((uint16_t*)(rxBuf+2));
			ConsolePrint("First language code: %04hX\r\n", usbHcd.langCode);

			// get some strings (if available)
			USBGetString(usbHcd.deviceDesc.iManufacturer, usbHcd.manufacturer, USB_MAX_STRING_SIZE);
			USBGetString(usbHcd.deviceDesc.iProduct, usbHcd.product, USB_MAX_STRING_SIZE);
			USBGetString(usbHcd.deviceDesc.iSerialNumber, usbHcd.serialNumber, USB_MAX_STRING_SIZE);

			ConsolePrint("--- Device Descriptor ---\r\n");
			ConsolePrint("VID/PID:        %04hX/%04hX\r\n", usbHcd.deviceDesc.idVendor, usbHcd.deviceDesc.idProduct);
			ConsolePrint("Class/Sub/Prot: %02hX/%02hX/%02hX\r\n", (uint16_t)usbHcd.deviceDesc.bDeviceClass,
					(uint16_t)usbHcd.deviceDesc.bDeviceSubClass, (uint16_t)usbHcd.deviceDesc.bDeviceProtocol);
			ConsolePrint("numConfigs:     %hu\r\n", (uint16_t)usbHcd.deviceDesc.bNumConfigurations);
			ConsolePrint("Manufacturer:   %s\r\n", usbHcd.manufacturer);
			ConsolePrint("Product:        %s\r\n", usbHcd.product);
			ConsolePrint("Serial Number:  %s\r\n", usbHcd.serialNumber);

			// Process configuration
			uint16_t configLen;
			result = USBCtrlRead(&setupGetConfigDesc, usbHcd.configDesc, &configLen);
			if(result)
			{
				ConsolePrint("Reading configuration descriptor failed: 0x%04X\r\n", result);
				continue;
			}

			ConsolePrint("Config descriptor length: %hu", configLen);

			uint8_t firstInterfaceClassCode = 0;

			// Parse configuration
			USBDescHeader* pRead = (USBDescHeader*)usbHcd.configDesc;
			uint8_t* pEnd = usbHcd.configDesc+configLen;

			for(uint8_t config = 0; config < usbHcd.deviceDesc.bNumConfigurations; config++)
			{
				pRead = USBFindDescriptor(USB_DESC_TYPE_CONFIGURATION, pRead, pEnd);
				USBConfigurationDesc* pConfigDesc = (USBConfigurationDesc*)pRead;
				if(pConfigDesc == 0)
					break;

				USB_SKIP_DESCRIPTOR(pRead);

				for(uint8_t i = 0; i < pConfigDesc->wTotalLength; i++)
				{
					if((i % 16) == 0)
						ConsolePrint("\r\n");

					ConsolePrint("%02X ", usbHcd.configDesc[i]);
				}
				ConsolePrint("\r\n");

				ConsolePrint("+-Configuration: %hu\r\n", (uint16_t)pConfigDesc->bConfigurationValue);
				ConsolePrint("  Interfaces:    %hu\r\n", (uint16_t)pConfigDesc->bNumInterfaces);
				ConsolePrint("  Total Length:  %hu\r\n", pConfigDesc->wTotalLength);

				for(uint8_t interface = 0; interface < pConfigDesc->bNumInterfaces; interface++)
				{
					pRead = USBFindDescriptor(USB_DESC_TYPE_INTERFACE, pRead, pEnd);
					USBInterfaceDesc* pInterfaceDesc = (USBInterfaceDesc*)pRead;
					if(pInterfaceDesc == 0)
						break;

					USB_SKIP_DESCRIPTOR(pRead);

					if(firstInterfaceClassCode == 0)
						firstInterfaceClassCode = pInterfaceDesc->bInterfaceClass;

					ConsolePrint("  +-Interface:      %hu\r\n", pInterfaceDesc->bInterfaceNumber);
					ConsolePrint("    Class/Sub/Prot: %02hX/%02hX/%02hX\r\n",
							(uint16_t)pInterfaceDesc->bInterfaceClass, (uint16_t)pInterfaceDesc->bInterfaceSubClass,
							(uint16_t)pInterfaceDesc->bInterfaceProtocol);
					ConsolePrint("    Endpoints:      %hu\r\n", (uint16_t)pInterfaceDesc->bNumEndpoints);

					for(uint8_t endpoint = 0; endpoint < pInterfaceDesc->bNumEndpoints; endpoint++)
					{
						pRead = USBFindDescriptor(USB_DESC_TYPE_ENDPOINT, pRead, pEnd);
						USBEndpointDesc* pEPDesc = (USBEndpointDesc*)pRead;
						if(pEPDesc == 0)
							break;

						USB_SKIP_DESCRIPTOR(pRead);

						ConsolePrint("    +-Endpoint:   %hu\r\n", (uint16_t)(pEPDesc->bEndpointAddress & 0x0F));
						if(pEPDesc->bEndpointAddress & 0x80)
							ConsolePrint("      Direction:  IN\r\n");
						else
							ConsolePrint("      Direction:  OUT\r\n");
						switch(pEPDesc->bmAttributes & 0x03)
						{
							case 0: ConsolePrint("      Type:       Control\r\n"); break;
							case 1: ConsolePrint("      Type:       Isochronous\r\n"); break;
							case 2: ConsolePrint("      Type:       Bulk\r\n"); break;
							default: ConsolePrint("      Type:       Interrupt\r\n"); break;
						}
						ConsolePrint("      maxPktSize: %hu\r\n", pEPDesc->wMaxPacketSize);
						ConsolePrint("      Interval:   %hu\r\n", (uint16_t)pEPDesc->bInterval);
					}
				}
			}

			// set configuration
			USBConfigurationDesc* pConfigDesc = (USBConfigurationDesc*)usbHcd.configDesc;
			setupSetConfiguration.wValue = pConfigDesc->bConfigurationValue;
			result = USBCtrlWrite(&setupSetConfiguration, 0);
			if(result)
			{
				ConsolePrint("Set address failed: 0x%04X\r\n", result);
				continue;
			}

			// call in appropriate class driver
			if(usbHcd.deviceDesc.bDeviceClass == 0)
			{
				// defined in each interface
				for(uint8_t i = 0; i < usbHcd.classesUsed; i++)
				{
					if(usbHcd.classes[i].isDeviceClass == 0 && usbHcd.classes[i].classCode == firstInterfaceClassCode)
					{
						usbHcd.pActiveClass = &usbHcd.classes[i];
						break;
					}
				}
			}
			else
			{
				for(uint8_t i = 0; i < usbHcd.classesUsed; i++)
				{
					if(usbHcd.classes[i].isDeviceClass != 0 && usbHcd.classes[i].classCode == usbHcd.deviceDesc.bDeviceClass)
					{
						usbHcd.pActiveClass = &usbHcd.classes[i];
						break;
					}
				}
			}

			if(usbHcd.deviceDesc.bDeviceClass == 0)
				usbHcd.conEv.classCode = firstInterfaceClassCode;
			else
				usbHcd.conEv.classCode = usbHcd.deviceDesc.bDeviceClass;

			usbHcd.conEv.connect = 1;
			usbHcd.conEv.pManufacturer = usbHcd.manufacturer;
			usbHcd.conEv.pProduct = usbHcd.product;
			usbHcd.conEv.pSerialNumber = usbHcd.serialNumber;

			if(usbHcd.pActiveClass)
				usbHcd.conEv.pName = usbHcd.pActiveClass->pName;
			else
				usbHcd.conEv.pName = "";

			if(usbHcd.pConCb)
				(*usbHcd.pConCb)(&usbHcd.conEv);

			if(usbHcd.pActiveClass == 0)
			{
				ConsolePrint("No proper class found for this device\r\n");
				continue;
			}

			ConsolePrint("Loading class driver: %s\r\n", usbHcd.pActiveClass->pName);

			(*usbHcd.pActiveClass->Init)(&usbHcd.deviceDesc, usbHcd.configDesc, configLen);

			// Interface classes:
			// 0x03: HID
			// 0x08: MSC
		}
	}
}

/*
 * MOUSE
 *
First language code: 0409
--- Device Descriptor ---
VID/PID:        046D/C05A
Class/Sub/Prot: 00/00/00
numConfigs:     1
Manufacturer:   Logitech
Product:        USB Optical Mouse
Serial Number:  N/A

09 02 22 00 01 01 00 A0 32 09 04 00 00 01 03 01
02 00 09 21 11 01 00 01 22 34 00 07 05 81 03 04
00 0A
+-Configuration: 1
  Interfaces:    1
  Total Length:  34
  +-Interface:      0
    Class/Sub/Prot: 03/01/02
    Endpoints:      1
    +-Endpoint:   1
      Direction:  OUT
      Type:       Isochronous
      maxPktSize: 256
      Interval:   34

 * USB STICK
 First language code: 0409
--- Device Descriptor ---
VID/PID:        0930/6545
Class/Sub/Prot: 00/00/00
numConfigs:     1
Manufacturer:   N/A
Product:        USB Flash Memorye
Serial Number:  01C174715340CEFD

09 02 20 00 01 01 00 80 64 09 04 00 00 02 08 06
50 00 07 05 81 02 40 00 00 07 05 02 02 40 00 00
+-Configuration: 1
  Interfaces:    1
  Total Length:  32
  +-Interface:      0
    Class/Sub/Prot: 08/06/50
    Endpoints:      2
    +-Endpoint:   1
      Direction:  IN
      Type:       Bulk
      maxPktSize: 64
      Interval:   0
    +-Endpoint:   2
      Direction:  OUT
      Type:       Bulk
      maxPktSize: 64
      Interval:   0

*/
