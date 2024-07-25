#pragma once

#include "gfx.h"
#include "util/fw_updater.h"

#define GUI_FW_UPDATE_BTN_NONE		-1
#define GUI_FW_UPDATE_BTN_USB		0
#define GUI_FW_UPDATE_BTN_SDCARD	1

GHandle FwUpdateCreate();
int16_t FwUpdateGetBtnClick(GEvent* pEvent);
void FwUpdateStatus(const FwUpdaterProgress* pProgress);
void FwUpdateAvailableSources(uint8_t usb, uint8_t sdCard);
