/*
 * linux/include/video/hub12fb.c -- Frame buffer driver for "HUB12" based
 * monochromatic LED panels.
 *
 * based on skeletonfb.c by James Simmons and Geert Uytterhoeven
 *
 * Copyright (C) 2013 Darren Garnier <dgarnier@reinrag.net>
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

#ifndef __VIDEO_HUB12FB_H__
#define __VIDEO_HUB12FB_H__

#include <linux/types.h>
#define HUB12FB_MODALIAS	"hub12fb"

/*
 * this is what could be put into the board driver file to load this driver
 *

 static const struct hub12fb_platform_data hub12fb_pdata __devinitdata = {
	 .gpio = {
		 .enable = 18,
		 .latch  = 17,
		 .a      = 22,
		 .b      = 27,
	 },
	 .width      = 32,
	 .height     = 32,
	 .framerate  = 60,
	 .brightness = 30,
 };


 static const struct spi_board_info spi_board_info[] __devinitdata = {
	 [0] = {
		 .modalias	= HUB12FB_MODALIAS,
		 .platform_data	= &hub12fb_pdata,
		 .mode		= SPI_MODE_0,
		 .max_speed_hz	= 8000000,
		 .bus_num	= 0,
		 .chip_select	= 0,
	 },
 };

 *
 */


struct hub12fb_platform_data {
	struct hub12_gpios {
		unsigned enable;
		unsigned latch;
		unsigned a;
		unsigned b;
	} gpio;
	unsigned width;
	unsigned height;
	unsigned refresh;
	unsigned brightness;
};

#endif
