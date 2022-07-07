/*
 * led.h
 *
 *  Created on: 24.10.2015
 *      Author: AndreR
 */

#ifndef LED_H_
#define LED_H_

#include <stdint.h>

#define LED_LEFT_RED	0x01
#define LED_LEFT_GREEN	0x02
#define LED_RIGHT_RED	0x04
#define LED_RIGHT_GREEN	0x08

void LEDInit();
void LEDSet(uint8_t state);
void LEDMustOn(uint8_t on);
void LEDMustOff(uint8_t off);
void LEDToggle(uint8_t toggle);
uint8_t LEDGetState();

#endif /* LED_H_ */
