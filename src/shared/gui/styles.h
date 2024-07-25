/*
 * styles.h
 *
 *  Created on: 07.10.2015
 *      Author: AndreR
 */

#ifndef STYLES_H_
#define STYLES_H_

#include "gfx.h"

static const GWidgetStyle RedTextStyle = {
	HTML2COLOR(0x000000),			// window background
	HTML2COLOR(0x000000),			// TODO: focus

	// enabled color set
	{
		HTML2COLOR(0xFF0000),		// text
		HTML2COLOR(0xFF0000),		// edge
		HTML2COLOR(0x606060),		// fill
		HTML2COLOR(0x404040),		// progress - inactive area
	},

	// disabled color set
	{
		HTML2COLOR(0x880000),		// text
		HTML2COLOR(0x880000),		// edge
		HTML2COLOR(0x404040),		// fill
		HTML2COLOR(0x004000),		// progress - active area
	},

	// pressed color set
	{
		HTML2COLOR(0xFFFFFF),		// text
		HTML2COLOR(0xC0C0C0),		// edge
		HTML2COLOR(0xE0E0E0),		// fill
		HTML2COLOR(0x008000),		// progress - active area
	},
};

static const GWidgetStyle YellowTextStyle = {
	HTML2COLOR(0x000000),			// window background
	HTML2COLOR(0x000000),			// TODO: focus

	// enabled color set
	{
		HTML2COLOR(0xFFFF00),		// text
		HTML2COLOR(0xFFFF00),		// edge
		HTML2COLOR(0x606060),		// fill
		HTML2COLOR(0x404040),		// progress - inactive area
	},

	// disabled color set
	{
		HTML2COLOR(0x888800),		// text
		HTML2COLOR(0x888800),		// edge
		HTML2COLOR(0x404040),		// fill
		HTML2COLOR(0x004000),		// progress - active area
	},

	// pressed color set
	{
		HTML2COLOR(0xFFFFFF),		// text
		HTML2COLOR(0xC0C0C0),		// edge
		HTML2COLOR(0xE0E0E0),		// fill
		HTML2COLOR(0x008000),		// progress - active area
	},
};

static const GWidgetStyle BlueTextStyle = {
	HTML2COLOR(0x000000),			// window background
	HTML2COLOR(0x000000),			// TODO: focus

	// enabled color set
	{
		HTML2COLOR(0x0000FF),		// text
		HTML2COLOR(0x0000FF),		// edge
		HTML2COLOR(0x606060),		// fill
		HTML2COLOR(0x404040),		// progress - inactive area
	},

	// disabled color set
	{
		HTML2COLOR(0x000088),		// text
		HTML2COLOR(0x000088),		// edge
		HTML2COLOR(0x404040),		// fill
		HTML2COLOR(0x004000),		// progress - active area
	},

	// pressed color set
	{
		HTML2COLOR(0xFFFFFF),		// text
		HTML2COLOR(0xC0C0C0),		// edge
		HTML2COLOR(0xE0E0E0),		// fill
		HTML2COLOR(0x008000),		// progress - active area
	},
};

static const GWidgetStyle GreenTextStyle = {
	HTML2COLOR(0x000000),			// window background
	HTML2COLOR(0x000000),			// TODO: focus

	// enabled color set
	{
		HTML2COLOR(0x00FF00),		// text
		HTML2COLOR(0x00FF00),		// edge
		HTML2COLOR(0x606060),		// fill
		HTML2COLOR(0x404040),		// progress - inactive area
	},

	// disabled color set
	{
		HTML2COLOR(0x008800),		// text
		HTML2COLOR(0x008800),		// edge
		HTML2COLOR(0x404040),		// fill
		HTML2COLOR(0x004000),		// progress - active area
	},

	// pressed color set
	{
		HTML2COLOR(0xFFFFFF),		// text
		HTML2COLOR(0xC0C0C0),		// edge
		HTML2COLOR(0xE0E0E0),		// fill
		HTML2COLOR(0x008000),		// progress - active area
	},
};

static const GWidgetStyle GreenProgressBarStyle = {
	HTML2COLOR(0x000000),			// window background
	HTML2COLOR(0x000000),			// TODO: focus

	// enabled color set
	{
		HTML2COLOR(0xC0C0C0),		// text
		HTML2COLOR(0xC0C0C0),		// edge
		HTML2COLOR(0x000000),		// fill
		HTML2COLOR(0x008000),		// progress - inactive area
	},

	// disabled color set
	{
		HTML2COLOR(0x008800),		// text
		HTML2COLOR(0x008800),		// edge
		HTML2COLOR(0x404040),		// fill
		HTML2COLOR(0x004000),		// progress - active area
	},

	// pressed color set
	{
		HTML2COLOR(0xFFFFFF),		// text
		HTML2COLOR(0xC0C0C0),		// edge
		HTML2COLOR(0xE0E0E0),		// fill
		HTML2COLOR(0x008000),		// progress - active area
	},
};

static const GWidgetStyle RedProgressBarStyle = {
	HTML2COLOR(0x000000),			// window background
	HTML2COLOR(0x000000),			// TODO: focus

	// enabled color set
	{
		HTML2COLOR(0xC0C0C0),		// text
		HTML2COLOR(0xC0C0C0),		// edge
		HTML2COLOR(0x000000),		// fill
		HTML2COLOR(0x800000),		// progress - inactive area
	},

	// disabled color set
	{
		HTML2COLOR(0x008800),		// text
		HTML2COLOR(0x008800),		// edge
		HTML2COLOR(0x404040),		// fill
		HTML2COLOR(0x004000),		// progress - active area
	},

	// pressed color set
	{
		HTML2COLOR(0xFFFFFF),		// text
		HTML2COLOR(0xC0C0C0),		// edge
		HTML2COLOR(0xE0E0E0),		// fill
		HTML2COLOR(0x008000),		// progress - active area
	},
};

static const GWidgetStyle GrayProgressBarStyle = {
	HTML2COLOR(0x000000),			// window background
	HTML2COLOR(0x000000),			// TODO: focus

	// enabled color set
	{
		HTML2COLOR(0xC0C0C0),		// text
		HTML2COLOR(0xC0C0C0),		// edge
		HTML2COLOR(0x000000),		// fill
		HTML2COLOR(0x404040),		// progress - inactive area
	},

	// disabled color set
	{
		HTML2COLOR(0x008800),		// text
		HTML2COLOR(0x008800),		// edge
		HTML2COLOR(0x404040),		// fill
		HTML2COLOR(0x004000),		// progress - active area
	},

	// pressed color set
	{
		HTML2COLOR(0xFFFFFF),		// text
		HTML2COLOR(0xC0C0C0),		// edge
		HTML2COLOR(0xE0E0E0),		// fill
		HTML2COLOR(0x008000),		// progress - active area
	},
};

static const GWidgetStyle GrayBackgroundStyle = {
	HTML2COLOR(0x444444),			// window background
	HTML2COLOR(0x000000),			// TODO: focus

	// enabled color set
	{
		HTML2COLOR(0xC0C0C0),		// text
		HTML2COLOR(0xC0C0C0),		// edge
		HTML2COLOR(0x000000),		// fill
		HTML2COLOR(0x800000),		// progress - inactive area
	},

	// disabled color set
	{
		HTML2COLOR(0x008800),		// text
		HTML2COLOR(0x008800),		// edge
		HTML2COLOR(0x404040),		// fill
		HTML2COLOR(0x004000),		// progress - active area
	},

	// pressed color set
	{
		HTML2COLOR(0xFFFFFF),		// text
		HTML2COLOR(0xC0C0C0),		// edge
		HTML2COLOR(0xE0E0E0),		// fill
		HTML2COLOR(0x008000),		// progress - active area
	},
};

static const GWidgetStyle YellowButtonStyle = {
	HTML2COLOR(0x000000),			// window background
	HTML2COLOR(0x000000),			// TODO: focus

	// enabled color set
	{
		HTML2COLOR(0xFFFF00),		// text
		HTML2COLOR(0x000000),		// edge
		HTML2COLOR(0x000000),		// fill
		HTML2COLOR(0x404040),		// progress - inactive area
	},

	// disabled color set
	{
		HTML2COLOR(0x888800),		// text
		HTML2COLOR(0x000000),		// edge
		HTML2COLOR(0x000000),		// fill
		HTML2COLOR(0x004000),		// progress - active area
	},

	// pressed color set
	{
		HTML2COLOR(0xFFFFFF),		// text
		HTML2COLOR(0x000000),		// edge
		HTML2COLOR(0xE0E0E0),		// fill
		HTML2COLOR(0x008000),		// progress - active area
	},
};

static const GWidgetStyle BlueButtonStyle = {
	HTML2COLOR(0x000000),			// window background
	HTML2COLOR(0x000000),			// TODO: focus

	// enabled color set
	{
		HTML2COLOR(0x0000FF),		// text
		HTML2COLOR(0x000000),		// edge
		HTML2COLOR(0x000000),		// fill
		HTML2COLOR(0x404040),		// progress - inactive area
	},

	// disabled color set
	{
		HTML2COLOR(0x000088),		// text
		HTML2COLOR(0x000000),		// edge
		HTML2COLOR(0x000000),		// fill
		HTML2COLOR(0x004000),		// progress - active area
	},

	// pressed color set
	{
		HTML2COLOR(0xFFFFFF),		// text
		HTML2COLOR(0x000000),		// edge
		HTML2COLOR(0xE0E0E0),		// fill
		HTML2COLOR(0x008000),		// progress - active area
	},
};


#endif /* STYLES_H_ */
