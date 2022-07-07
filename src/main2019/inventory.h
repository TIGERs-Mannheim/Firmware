/*
 * inventory.h
 *
 *  Created on: 26.07.2020
 *      Author: AndreR
 */

#pragma once

#include <stdint.h>

typedef struct _InventoryImuCalibration
{
	// for all biases:
	// calibrated value = raw - bias

	float gyrBias[3];
	float accBias[3];
	float accTiltBias[2];
	float imuCalibTemp;
	float magBias[3];
	float magCalibTemp;
	uint32_t calibrated;
} InventoryImuCalibration;

typedef struct _InventoryEntry
{
	uint32_t cpuId[3];
	uint32_t hwId;

	InventoryImuCalibration imuCalib;
} InventoryEntry;

void InventoryInit();
const InventoryEntry* InventoryGetEntry();
const InventoryImuCalibration* InventoryGetImuCalibration();
