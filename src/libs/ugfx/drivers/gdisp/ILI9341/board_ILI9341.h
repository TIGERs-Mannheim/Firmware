/*
 * This file is subject to the terms of the GFX License. If a copy of
 * the license was not distributed with this file, you can obtain one at:
 *
 *              http://ugfx.org/license.html
 */

#ifndef _GDISP_LLD_BOARD_H
#define _GDISP_LLD_BOARD_H

#include "hal/init_hal.h"

#define TFT_CMD_REG ((volatile uint16_t*)0x60000000)
#define TFT_DATA_REG ((volatile uint16_t*)0x60000100)

static inline void init_board(GDisplay *g) {
	(void) g;
}

static inline void post_init_board(GDisplay *g) {
	(void) g;
}

static inline void setpin_reset(GDisplay *g, bool_t state) {
	(void) g;
	(void) state;

	if(state == TRUE)
		GPIOReset(GPIOG, GPIO_PIN_3);
	else
		GPIOSet(GPIOG, GPIO_PIN_3);
}

static inline void set_backlight(GDisplay *g, uint8_t percent) {
	(void) g;
	(void) percent;

	if(percent == 0)
		GPIOReset(GPIOG, GPIO_PIN_2);
	else
		GPIOSet(GPIOG, GPIO_PIN_2);
}

static inline void acquire_bus(GDisplay *g) {
	(void) g;
}

static inline void release_bus(GDisplay *g) {
	(void) g;
}

static inline void write_index(GDisplay *g, uint16_t index) {
	(void) g;

	*TFT_CMD_REG = index;
}

static inline void write_data(GDisplay *g, uint16_t data) {
	(void) g;

	*TFT_DATA_REG = data;
}

static inline void setreadmode(GDisplay *g) {
	(void) g;
}

static inline void setwritemode(GDisplay *g) {
	(void) g;
}

static inline uint16_t read_data(GDisplay *g) {
	(void) g;

	return *TFT_DATA_REG;
}

#endif /* _GDISP_LLD_BOARD_H */
