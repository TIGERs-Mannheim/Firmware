#pragma once

#include "ext_commands.h"

void PresenterInit();
void PresenterShowWindow(int16_t newWindow);
void PresenterTask(void* params);
void PresenterPrintHeapStatus();
void PresenterTakeScreenshot();
void PresenterSendPreviewLineToDisplay(ExtCameraPreviewLine160* pLine);
