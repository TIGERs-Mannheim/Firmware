/*
 * logging.h
 *
 *  Created on: 26.04.2019
 *      Author: AndreR
 */

#pragma once

#include "gfx.h"
#include "util/usb_events.h"
#include "util/log_file.h"

GHandle LoggingCreate();
void	LoggingHCDUpdate(USBHCDConnectionEvent* pConEvent);
void	LoggingMSCUpdate(USBMSCEvent* pEvent);
void	LoggingLogUpdate(LogFileEvent* pLogEvent);
