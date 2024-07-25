/*
 * usb_events.h
 *
 *  Created on: 03.04.2020
 *      Author: AndreR
 */

#pragma once

#include <stdint.h>

typedef struct _USBHCDConnectionEvent
{
	uint8_t connect;

	uint8_t classCode;
	const char* pName;
	const char* pManufacturer;
	const char* pProduct;
	const char* pSerialNumber;
} USBHCDConnectionEvent;


typedef struct _USBMSCEvent
{
	uint8_t connect;
	uint32_t fatSize;	// MB
	uint32_t freeSize;	// MB
	uint32_t clusterSize; 	// kB
} USBMSCEvent;
