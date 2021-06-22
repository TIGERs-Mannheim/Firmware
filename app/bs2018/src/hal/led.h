/*
 * led.h
 *
 *  Created on: 23.10.2017
 *      Author: AndreR
 */

#pragma once

#include <stdint.h>

#define LED_GREEN	0x01
#define LED_RED		0x02

void LEDInit();
void LEDSet(uint8_t state);
void LEDMustOn(uint8_t on);
void LEDMustOff(uint8_t off);
void LEDToggle(uint8_t toggle);
uint8_t LEDGetState();
