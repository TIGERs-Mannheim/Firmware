/*
 * usb.h
 *
 *  Created on: 01.08.2014
 *      Author: AndreR
 */

#pragma once

#include "ch.h"
#include "usb_def.h"

#ifdef STM32H7XX
#include "stm32h743xx.h"
#else
#include "stm32f746xx.h"
#endif

#define USB_EVENT_QUEUE_SIZE 24
#define USB_EP_EVENT_QUEUE_SIZE 16

#define ERROR_USB_IRP_TOO_BIG			1
#define ERROR_USB_IRP_INVALID_IN_SIZE	2

typedef enum _USBIRPPID
{
	USB_PID_DATA0 = 0,
	USB_PID_DATA2 = 1,
	USB_PID_DATA1 = 2,
	USB_PID_MDATA_SETUP = 3,
	USB_PID_HOST_CONTROLLED = 4,
} USBIRPPID;

typedef enum _USBIRPState
{
	USB_IRP_STATE_DONE = 0,
	USB_IRP_STATE_ABORTED = 1,
	USB_IRP_STATE_TIMEOUT = 2,
	USB_IRP_STATE_TXERR = 3,
	USB_IRP_STATE_STALL,
	USB_IRP_STATE_NEW = 100,
	USB_IRP_STATE_ENQUEUED = 101,
	USB_IRP_STATE_ACTIVE = 102,
} USBIRPState;

typedef enum _USBEPState
{
	USB_EP_STATE_NONE = 0,
	USB_EP_STATE_ACK,
	USB_EP_STATE_NAK,
	USB_EP_STATE_STALL,
	USB_EP_STATE_TXERR,
	USB_EP_STATE_BBERR,
	USB_EP_STATE_DTERR,
	USB_EP_STATE_XFRC,
	USB_EP_STATE_SOF_RESTART,
	USB_EP_STATE_SOF_CONT,
} USBEPState;

typedef struct _USBIRP
{
	uint8_t* pData;
	uint32_t dataLength;
	USBIRPPID pid;

	uint32_t numPackets;

	// status
	volatile USBIRPState state;
	uint16_t packetsDone;
	uint32_t bytesDone;

	binary_semaphore_t semDone;

	struct _USBIRP* pNext;
} USBIRP;

typedef enum _USBEndpointType
{
	USB_EPTYPE_CONTROL = 0,
	USB_EPTYPE_ISOCHRONOUS = 1,
	USB_EPTYPE_BULK = 2,
	USB_EPTYPE_INTERRUPT = 3,
} USBEndpointType;

typedef struct _USBEndpointCfg
{
	uint8_t epNumber;
	uint8_t epIn;
	uint16_t maxPktSize;
	USBEndpointType type;
	uint32_t timeout;
} USBEndpointCfg;

typedef struct _USBEndpoint
{
	uint8_t used;
	USBEndpointCfg cfg;
	USB_OTG_HostChannelTypeDef* pChannel;
	uint8_t chNum;
	volatile USBEPState state;

	USBIRP* pIRPList;
	mutex_t irpListMutex;

	msg_t eventQueueData[USB_EP_EVENT_QUEUE_SIZE];
	mailbox_t eventQueue;

	uint8_t txErrCnt;
	uint32_t nakCnt;
} USBEndpoint;

typedef void(*USBConnectCallback)(uint8_t);
typedef void(*USBEnableVBusFunc)(uint8_t);

typedef struct _USBGlobal
{
	USBEndpoint endpoints[USB_MAX_ENDPOINTS];

	uint32_t lastActivePCh;
	uint32_t lastActiveNPCh;

	msg_t eventQueueData[USB_EVENT_QUEUE_SIZE];
	mailbox_t eventQueue;

	USBEnableVBusFunc enableVBusFunc;
	USBConnectCallback conCallback;

	uint32_t deviceAddress;
} USBGlobal;

extern USBGlobal usb;

void			USBInit(USBEnableVBusFunc enableVBus, uint32_t usbIRQLevel);
void			USBTask(void* params);
void			USBChangeDeviceAddress(uint32_t newAddress);

USBDescHeader*	USBFindDescriptor(uint8_t type, USBDescHeader* pRead, uint8_t* pEnd);

USBEndpoint*	USBEndpointCreate(USBEndpointCfg* pCfg);
void			USBEndpointDelete(USBEndpoint* pEP);

int16_t			USBIRPSubmit(USBEndpoint* pEP, USBIRP* pIRP);
void			USBIRPWait(USBIRP* pIRP);
int16_t			USBIRPTransfer(USBEndpoint* pEP, USBIRP* pIRP);
