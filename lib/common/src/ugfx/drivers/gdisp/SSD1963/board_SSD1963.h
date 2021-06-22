/*
 * This file is subject to the terms of the GFX License. If a copy of
 * the license was not distributed with this file, you can obtain one at:
 *
 *              http://ugfx.org/license.html
 */

#ifndef _GDISP_LLD_BOARD_H
#define _GDISP_LLD_BOARD_H

#define GDISP_SSD1963_NO_INIT     TRUE

#define TFT_CMD_REG ((volatile uint16_t*)0x60000000)
#define TFT_DATA_REG ((volatile uint16_t*)0x60020000)

static const LCD_Parameters	DisplayTimings[] = {
	// You need one of these array elements per display
	{
		800, 480,								// Panel width and height
		2, 2, 41,								// Horizontal Timings (back porch, front porch, pulse)
		CALC_PERIOD(800,2,2,41),				// Total Horizontal Period (calculated from above line)
		2, 2, 10,								// Vertical Timings (back porch, front porch, pulse)
		CALC_PERIOD(480,2,2,10),				// Total Vertical Period (calculated from above line)
		CALC_FPR(800,480,2,2,41,2,2,10,60ULL),	// FPR - the 60ULL is the frames per second. Note the ULL!
		LCD_PANEL_LSHIFT_FALLING_EDGE,
		FALSE,									// Flip horizontally
		FALSE									// Flip vertically
	},
};

static inline void init_board(GDisplay *g) {
	(void) g;
}

static inline void post_init_board(GDisplay *g) {
	(void) g;
}

static inline void setpin_reset(GDisplay *g, bool_t state) {
	(void) g;
	(void) state;
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
