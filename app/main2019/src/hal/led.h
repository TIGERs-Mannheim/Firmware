/*
 * led.h
 *
 *  Created on: 30.12.2018
 *      Author: AndreR
 */

#pragma once

#include <stdint.h>

#define LED_MAIN_RED			0x0001
#define LED_MAIN_GREEN			0x0002
#define LED_FRONT_LEFT_RED		0x0100
#define LED_FRONT_LEFT_GREEN	0x0200
#define LED_FRONT_LEFT_BLUE		0x0400
#define LED_FRONT_LEFT_WHITE	0x0800
#define LED_FRONT_LEFT_MASK		0x0F00
#define LED_FRONT_RIGHT_RED		0x1000
#define LED_FRONT_RIGHT_GREEN	0x2000
#define LED_FRONT_RIGHT_BLUE	0x4000
#define LED_FRONT_RIGHT_WHITE	0x8000
#define LED_FRONT_RIGHT_MASK	0xF000

void LEDInit();
void LEDSet(uint16_t state);
void LEDFrontSet(uint16_t state);
void LEDToggle(uint16_t toggle);
void LEDDemo();

void LEDLeftSet(float red, float green, float blue, float white);
void LEDRightSet(float red, float green, float blue, float white);
