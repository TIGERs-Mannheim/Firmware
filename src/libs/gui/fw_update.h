/*
 * fw_update.h
 *
 *  Created on: 10.04.2020
 *      Author: AndreR
 */

#pragma once

#include "gfx.h"
#include "util/fw_updater.h"

GHandle FwUpdateCreate(uint8_t bootloader);
void FwUpdateStatus(FwUpdaterProgress* pProgressList, uint8_t numPrograms);
void FwUpdateAvailableSources(uint8_t wifi, uint8_t usb, uint8_t sdCard);
