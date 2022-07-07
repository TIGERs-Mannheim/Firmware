/*
 * tft.h
 *
 *  Created on: 31.10.2015
 *      Author: AndreR
 */

#ifndef TFT_H_
#define TFT_H_

#include "ch.h"

#define TFT_CMD_REG ((volatile uint16_t*)0x60000000)
#define TFT_DATA_REG ((volatile uint16_t*)0x60000100)

void TFTInit();
void TFTEnable(uint8_t enable);
void TFTReset(uint8_t reset);

#endif /* TFT_H_ */
