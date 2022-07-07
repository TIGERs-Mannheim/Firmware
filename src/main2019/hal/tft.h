/*
 * tft.h
 *
 *  Created on: 08.01.2019
 *      Author: AndreR
 */

#pragma once

#include "ch.h"

#define TFT_CMD_REG ((volatile uint16_t*)0x60000000)
#define TFT_DATA_REG ((volatile uint16_t*)0x60000100)

void TFTInit();
void TFTEnable(uint8_t enable);
void TFTReset(uint8_t reset);
