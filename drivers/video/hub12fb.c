/*
 * linux/drivers/video/hub12fb.c -- Frame buffer driver for "HUB12" based
 * monochromatic LED panels.
 *
 * based on skeletonfb.c by James Simmons and Geert Uytterhoeven
 * inspired by and borrowing from st7735fb.c by Matt Porter
 *
 * Copyright (C) 2013 Darren Garnier <dgarnier@reinrag.net>
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>

#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/dma-mapping.h>
#include <linux/hrtimer.h>
#include <linux/backlight.h>

#include <video/hub12fb.h>

/*
 * Driver data
 */

#define DRIVER_NAME HUB12FB_MODALIAS

struct hub12_par {
	struct hub12fb_platform_data	pdata;
	struct spi_device *		spi;
	struct fb_info *		info;
	void *				fb_buffer;
	void *				hsync_buf[4];
	dma_addr_t			hsync_dma[4];
	unsigned			hsync_length;
	struct spi_message		message;
	struct spi_transfer		transfer;
	wait_queue_head_t		vsync_wait;
	struct hrtimer			hsync_timer;
	struct hrtimer			ledon_timer;
	ktime_t				hsync_period;
	ktime_t				ledon_period;
	int				vsync_timeout;
	int				blank;
	unsigned			running       :1;
	unsigned			hsync_running :1;
	unsigned			vsync_flag    :1;
	int				i_scan;
	u32				pseudo_palette[16];
};

static const struct hub12fb_platform_data default_platform_data = {
	.gpio = {
		.enable = 18,
		.latch  = 17,
		.a      = 22,
		.b      = 27,
	},
	.width      = 32,
	.height     = 16,
	.refresh    = 60,
	.brightness = 128,
};

static struct spi_board_info board_info = {
	.modalias	= HUB12FB_MODALIAS,
	.platform_data	= &default_platform_data,
	.mode		= SPI_MODE_0,
	.max_speed_hz	= 8000000,
	.bus_num	= 0,
	.chip_select	= 0,
};


/* test pattern .. lsb format
 *
 * 0***1***2***3***4***5***6***7***  = FF FF FF FF
 * **              **            **  = 30 00 30 0C
 * * *       **    * *          * *  = 50 C0 50 0A
 * *  *     *  *   *  *        *  *  = 90 21 90 09
 * 4   *     **    *   *      *   *  = 11 C0 11 88
 * *    *          *    *    *    *  = 12 00 12 48
 * *     *         *     *  *     *  = 14 00 14 28
 * *      *        *      **      *  = 18 00 18 18
 * 8       *       *      **      *  = 10 10 18 18
 * *    *   *      *     *  *     *  = 12 20 14 28
 * *   * *   *     *    *  * *    *  = 15 40 12 58
 * *  *   *   *    *   *  **  *   *  = 98 80 19 98
 * C   * *     *   *  *    *   *  *  = 15 01 90 19
 * *    *       *  * *     *    * *  = 12 02 50 1A
 * *             * **     ***    **  = 10 01 38 3C
 * F*** ***  **   *******     *****  = FE C8 F3 1F
 *
 */

static const char test_pattern_32_16[64] = {
	0xFF, 0xFF, 0xFF, 0xFF,
	0x30, 0x00, 0x30, 0x0C,
	0x50, 0xC0, 0x50, 0x0A,
	0x90, 0x21, 0x90, 0x09,
	0x11, 0xC0, 0x11, 0x88,
	0x12, 0x00, 0x12, 0x48,
	0x14, 0x00, 0x14, 0x28,
	0x18, 0x00, 0x18, 0x18,
	0x10, 0x10, 0x18, 0x18,
	0x12, 0x20, 0x14, 0x28,
	0x15, 0x40, 0x12, 0x58,
	0x98, 0x80, 0x19, 0x98,
	0x15, 0x01, 0x90, 0x19,
	0x12, 0x02, 0x50, 0x1A,
	0x10, 0x01, 0x38, 0x3C,
	0xFE, 0xC8, 0xF3, 0x1F,
};

/*
 * Here ,we de,fine the default structs fb_fix_screeninfo and fb_var_screeninfo
 * if we don't use modedb. If we do use modedb see hub12fb_init how to use it
 * to get a fb_var_screeninfo. Otherwise define a default var as well.
 */
static const struct fb_fix_screeninfo hub12fb_fix __devinitconst = {
	.id 		= DRIVER_NAME,
	.type		= FB_TYPE_PACKED_PIXELS,
	.visual		= FB_VISUAL_MONO10,
	.xpanstep	= 0,
	.ypanstep	= 0,
	.ywrapstep	= 0,
	.accel =	FB_ACCEL_NONE,
};

static char	*mode	__devinitdata = NULL;
static int	spi[3]  __devinitdata = {-1,0,8000000}; /* no default bus */
static int	gpio[4]	__devinitdata = {-1,-1,-1,-1}; /* overrides if not -1 */

static inline void hub12fb_set_brightness(struct hub12_par *par)
{
	u64 ns;

	ns = par->pdata.brightness * ktime_to_ns(par->hsync_period) / 256;

	// should have some reasonable min and max here
	// and the change the run mode if you are outside those ranges...

	par->ledon_period = ns_to_ktime(ns);
}

#if CONFIG_FB_HUB12_BACKLIGHT
static int hub12fb_blank(int blank_mode, struct fb_info *info);

static int hub12bl_update_status(struct backlight_device *bd)
{
	struct hub12_par *par = bl_get_data(bd);
	int retval = 0;
	int blank = FB_BLANK_UNBLANK;

	par->pdata.brightness = bd->props.brightness;
	hub12fb_set_brightness(par);

	if (blank != bd->props.power)
		blank = bd->props.power;

	if (blank != bd->props.fb_blank)
		blank = bd->props.fb_blank;

	if (blank != par->blank)
		retval = hub12fb_blank(blank, par->info);

	return retval;
}

static int hub12bl_get_brightness(struct backlight_device *bl)
{
	struct hub12_par *par = bl_get_data(bl);
	return par->pdata.brightness;
}

static const struct backlight_ops hub12bl_ops = {
	.get_brightness = hub12bl_get_brightness,
	.update_status = hub12bl_update_status,
};

static void __devinit init_hub12bl(struct fb_info *info)
{
	struct hub12_par *par = info->par;
	struct backlight_device	*	bl;
	struct backlight_properties	props;
	char name[12];

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = 255;
	props.power = FB_BLANK_UNBLANK;

	snprintf(name, sizeof(name), DRIVER_NAME "-bl%d", info->node);

	bl = backlight_device_register(name, &par->spi->dev,
					 par, &hub12bl_ops, &props);
	if (IS_ERR(bl)) {
		dev_err( &par->spi->dev, "error %ld on backlight register\n",
			PTR_ERR(bl));
	}
	info->bl_dev = bl;
}

static void __devexit exit_hub12bl(struct fb_info *info)
{
	if (info->bl_dev) {
		backlight_device_unregister(info->bl_dev);
		info->bl_dev = NULL;
	}
}

#else
static void init_hub12bl(struct fb_info *info)
{
	struct hub12_par *par = info->par;
	dev_warn(  &par->spi->dev, "backlight control is not available\n");
}

static void exit_hub12bl(struct fb_info *info)
{
}

#endif

static int wait_for_vsync(struct hub12_par *par)
{
	int ret;

	/*
	 * slight race condition might mean have to wait for second vsync
	 */

	par->vsync_flag = 0;
	ret = wait_event_interruptible_timeout(par->vsync_wait,
					       par->vsync_flag != 0,
					       par->vsync_timeout);
	if (ret < 0)
		return ret;

	if (ret == 0) {
		printk(KERN_ALERT DRIVER_NAME " wait for vsync timed out.\n");
		return -ETIMEDOUT;
	}

	return 0;
}

/*
 * do_vsync breaks up the framebuffer into the
 * to data that will go over the wire to the shift registers.
 * individual displays are 32 w x 16 h with 4 scan lines, in 4 colums.
 *
 * many things to iterate over...
 *  i, 4 "scan lines"
 *  j, height / 16 "module rows"
 *  k, width / 8 "columes per row"
 *  l, 4 lines per colume (per scanline)
 */

static void do_vsync(struct hub12_par *par)
{
	u16 i,j,k,l;
	u16 rowbytes = par->pdata.width/8;

	u8 *fb = par->fb_buffer;
	u8 *lb;

	/* copy the data, byte by byte */
	/* might need bitswap for SPI */

	for(i=0;i<4;i++) {
		lb = par->hsync_buf[i];
		for (j=0;j<par->pdata.height/16;j++)
			for(k=0;k<rowbytes;k++)
				for(l=0;l<4;l++)
					*lb++ = fb[(j*16+l*4+i)*rowbytes + k];
	}
	par->i_scan = 0;
	par->vsync_flag = 1;
	if (waitqueue_active(&par->vsync_wait))
		wake_up_interruptible(&par->vsync_wait);
}

static void shift_scanline_completion(void *context)
{
	/* run after the spi transfer is complete */

	static const int avals[4] = {0,0,1,1};
	static const int bvals[4] = {0,1,0,1};

	struct hub12_par *par = (struct hub12_par *) context;

	/* bits have been shifted.. now lets enable the proper output */
	/* move them to the output */
	gpio_set_value(par->pdata.gpio.latch, 1);
	/* select the proper output lines */
	gpio_set_value(par->pdata.gpio.a, avals[par->i_scan]);
	gpio_set_value(par->pdata.gpio.b, bvals[par->i_scan]);

	if (par->blank == FB_BLANK_UNBLANK && par->running) {
		/* turn on the light */
		gpio_set_value(par->pdata.gpio.enable, 1);
		hrtimer_start(&par->ledon_timer, par->ledon_period,
				HRTIMER_MODE_REL);
	}

	par->i_scan++;

	if (par->i_scan == 4) 	/* do the vsync now */
		do_vsync(par);

	par->hsync_running = 0;
}


static void setup_hsync(struct hub12_par *par)
{
	/* setup the spi transfer
	 * we don't have to do this every time
	 * just after we change settings
	 */

	struct spi_message *m = &par->message;
	struct spi_transfer *t = &par->transfer;

	par->hsync_running = 0;

	t->rx_buf = NULL;
	t->rx_dma = 0;
	t->len    = par->hsync_length;

	t->cs_change		= 1;
	t->bits_per_word	= 8;
	t->delay_usecs		= 0;
	t->speed_hz		= 0; /* default */

	spi_message_init(m);
	spi_message_add_tail(t,m);

	m->context = par;
	m->complete = shift_scanline_completion;
}

static void shift_scanline_start(struct hub12_par *par)
{
	struct spi_message *m = &par->message;
	struct spi_transfer *t = &par->transfer;

	/* sometimes the last transfer can still be running!
	 * this happens on the RPi because the SPI master can take
	 * up to 10msec to respond sometimes
	 *
	 * we just stop here and it gets tried again on
	 * the next go around.
	 */

	if (par->hsync_running)
		return;

	par->hsync_running = 1;

	t->tx_buf = par->hsync_buf[par->i_scan];
	t->tx_dma = par->hsync_dma[par->i_scan];

	/* only our driver until we can latch */
	spi_async_locked(par->spi, m);
}


static enum hrtimer_restart ledon_expired(struct hrtimer *t)
{
	struct hub12_par *par = container_of(t, struct hub12_par, ledon_timer);

	/* turn out the light. */
	gpio_set_value(par->pdata.gpio.enable, 0);

	/* timer is restarted by end of hsync spi transfer */
	return HRTIMER_NORESTART;
}


static enum hrtimer_restart do_hsync(struct hrtimer *timer)
{
	struct hub12_par *par = container_of(timer,
					     struct hub12_par, hsync_timer);
	/* turn off the latch */
	gpio_set_value(par->pdata.gpio.latch, 0);

	/* no more hsync */
	if (!par->running) {
		par->vsync_flag = 1;
		if (waitqueue_active(&par->vsync_wait))
			wake_up_interruptible(&par->vsync_wait);
		/* signal vsync in case of waiters... */
		return HRTIMER_NORESTART;
	}

	shift_scanline_start(par);

	/* having trouble with this... */
	/* really shift it forward */
	hrtimer_add_expires(timer, par->hsync_period);
	/* in case it fell to far behind */
	hrtimer_forward_now(timer, par->hsync_period);
	return HRTIMER_RESTART;
}

static void hub12fb_stop_running(struct hub12_par *par)
{
	if (par->running) {
		par->running = 0;
		/* will sleep until hsync is stopped */
		hrtimer_cancel(&par->hsync_timer);
		hrtimer_cancel(&par->ledon_timer);
		/* this might be all we need..
		 * but I seem to be leaving dross in the
		 * process list
		 */
		wait_for_vsync(par);
	}
}

static void hub12fb_start_running(struct hub12_par *par)
{
	if (par->running)
		return;

	/* begin with vsync */
	do_vsync(par);

	setup_hsync(par);

	par->running = 1;
	hrtimer_start(&par->hsync_timer, par->hsync_period, HRTIMER_MODE_REL);
}

static inline u32 refresh_to_pixclock(int refresh, struct fb_var_screeninfo *var)
{
	u32 pixclock;
	pixclock = 1000 * KHZ2PICOS(refresh * var->xres * var->yres);
	return pixclock;
}

static inline u32 hub12fb_refresh_rate(struct fb_var_screeninfo *var)
{
	u32 refresh;
	refresh = PICOS2KHZ(var->pixclock/1000) / (var->xres * var->yres);
	return refresh;
}

static int hub12fb_modestr_to_var(const char *mode_option,
				  struct fb_var_screeninfo *var)
{
	/* parseing section borrowed form modedb.c */
	int i;
	const char *name;
	unsigned int namelen;
	int res_specified = 0, refresh_specified = 0;
	int yres_specified = 0;
	unsigned int xres = 32, yres = 16, refresh = 60;

	if (!mode_option)
		mode_option = "32x32@60";

	name = mode_option;
	namelen = strlen(name);

	for (i = namelen-1; i >= 0; i--) {
		switch (name[i]) {
		case '@':
			namelen = i;
			if (!refresh_specified && !yres_specified) {
				refresh = simple_strtol(&name[i+1], NULL,
							10);
				refresh_specified = 1;
			} else
				goto done;
			break;
		case 'x':
			if (!yres_specified) {
				yres = simple_strtol(&name[i+1], NULL,
						     10);
				yres_specified = 1;
			} else
				goto done;
			break;
		case '0' ... '9':
			break;
		default:
			goto done;
		}
	}
	if (i < 0 && yres_specified) {
		xres = simple_strtol(name, NULL, 10);
		res_specified = 1;
	}
done:
	if (res_specified)
		var->xres = 32*((xres - 1)/32 + 1);

	if (yres_specified)
		var->yres = 16*((yres - 1)/16 + 1);

	if (refresh_specified) {
		var->pixclock = refresh_to_pixclock(refresh, var);
	}

	printk(KERN_ALERT DRIVER_NAME "Setting mode %d x %d @ %d (%d) Hz \n",
	       var->xres, var->yres, refresh, hub12fb_refresh_rate(var));

	return 0;
}

/**
 *      hub12fb_check_var - Validates a var passed in.
 *      @var: frame buffer variable screen structure
 *      @info: frame buffer structure that represents a single frame buffer
 *
 */
static int hub12fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	if (!var->xres)
		var->xres = 1;
	if (!var->yres)
		var->yres = 1;

	/* fixed array of panels. Round up to next unit size. */
	var->xres = ((var->xres + 31) / 32) * 32;
	var->yres = ((var->yres + 15) / 16) * 16;

	/* limit refresh to 1000 hz */
	if (var->pixclock < refresh_to_pixclock(1000,var))
		var->pixclock = refresh_to_pixclock(1000, var);

	/* don't try "virtual" */
	var->xres_virtual = var->xres;
	var->yres_virtual = var->yres;
	var->xoffset = 0;
	var->yoffset = 0;

	var->height = var->yres * 10; /* for P10 arrays */
	var->width  = var->xres * 10;

	var->upper_margin = 0;
	var->lower_margin = 0;
	var->left_margin  = 0;
	var->right_margin = 0;

	var->bits_per_pixel = 1;  /* future, allow 8 bit gray */

	var->vmode = FB_VMODE_NONINTERLACED;

	switch (var->bits_per_pixel) {
		case 1:
			var->red.offset = 0;
			var->red.length = 1;
			var->green.offset = 0;
			var->green.length = 1;
			var->blue.offset = 0;
			var->blue.length = 1;
			var->transp.offset = 0;
			var->transp.length = 0;
			break;
		case 8:
			var->red.offset = 0;
			var->red.length = 8;
			var->green.offset = 0;
			var->green.length = 8;
			var->blue.offset = 0;
			var->blue.length = 8;
			var->transp.offset = 0;
			var->transp.length = 0;
			break;
	}
	var->red.msb_right = 0;
	var->green.msb_right = 0;
	var->blue.msb_right = 0;
	var->transp.msb_right = 0;


	return 0;
}

static void hub12fb_free_buffers(struct hub12_par *par)
{
	int i;
	struct device *device = &par->spi->dev;

	DEFINE_DMA_ATTRS(attrs);
	dma_set_attr(DMA_ATTR_WRITE_COMBINE, &attrs);

	/* stop running the kernel refresh thread before this! */
	hub12fb_stop_running(par);

	for (i=0; i<4; i++) {
		if (par->hsync_dma[i])
			dma_free_attrs(device, par->hsync_length, par->hsync_buf[i],
				       par->hsync_dma[i], &attrs);
		else if (par->hsync_buf[i])
			kfree(par->hsync_buf[i]);

		par->hsync_buf[i] = NULL;
		par->hsync_dma[i] = 0;
	}

	if (par->fb_buffer) {
		kfree(par->fb_buffer);
		par->fb_buffer = NULL;
	}
}

static int hub12fb_allocate_buffers(struct hub12_par *par)
{
	int i;
	size_t bsize;
	struct device *device = &par->spi->dev;
	DEFINE_DMA_ATTRS(attrs);
	dma_addr_t dma_handle = 0;
	void *p;
	int use_dma = 1;

	bsize = (par->pdata.width/8) * par->pdata.height;

	par->fb_buffer = kzalloc(bsize, GFP_KERNEL);

	if (!par->fb_buffer)
		return -ENOMEM;

	par->hsync_length = bsize/4;

	if (!device->coherent_dma_mask) {
		printk(KERN_ALERT DRIVER_NAME
		       " can't allocate DMA buffer.\n");
		printk(KERN_ALERT
		       "spi driver (master) coherent_dma_masks %llx (%llx)\n",
		       device->coherent_dma_mask,
		       par->spi->master->dev.coherent_dma_mask);
		use_dma = 0;
	} else
		/* get dma memory if available */
		dma_set_attr(DMA_ATTR_WRITE_COMBINE, &attrs);

	for (i=0; i<4; i++) {
		if (use_dma)
			p = dma_alloc_attrs(device, par->hsync_length,
					    &dma_handle, GFP_KERNEL, &attrs);
		else
			p = kmalloc(par->hsync_length, GFP_KERNEL);

		if (!p)
			goto cleanup_buffers;

		par->hsync_buf[i] = p;
		par->hsync_dma[i] = dma_handle;
	}

	return 0;

cleanup_buffers:
	hub12fb_free_buffers(par);
	return -ENOMEM;
}


/**
 *      hub12fb_set_par - Alters the hardware state.
 *      @info: frame buffer structure that represents a single frame buffer
 *
 *	Using the fb_var_screeninfo in fb_info we set the resolution of the
 *	this particular framebuffer. This function alters the par AND the
 *	fb_fix_screeninfo stored in fb_info. It doesn't not alter var in
 *	fb_info since we are using that data. This means we depend on the
 *	data in var inside fb_info to be supported by the hardware.
 *
 *      This function is also used to recover/restore the hardware to a
 *      known working state.
 *
 *	hub12fb_check_var is always called before hub12fb_set_par to ensure that
 *      the contents of var is always valid.
 *
 *	Returns negative errno on error, or zero on success.
 */
static int hub12fb_set_par(struct fb_info *info)
{
	int retval;
	struct hub12_par *par = info->par;
	struct fb_fix_screeninfo *fix = &info->fix;

	/* before we do anything... lets stop the current framebuffer */

	hub12fb_stop_running(info->par);

	par->pdata.width  = info->var.xres;
	par->pdata.height = info->var.yres;

	/* clear old buffers */
	if (par->fb_buffer)
		hub12fb_free_buffers(par);

	retval = hub12fb_allocate_buffers(par);

	/* test pattern */
	if (par->pdata.width == 32 && par->pdata.height == 16)
		memcpy(par->fb_buffer, test_pattern_32_16, 64);

	if (retval) {
		dev_err(&par->spi->dev, "Unable to allocate buffers.");
		return retval;
	}

	/* set buffers now to fb device */
	info->screen_base = par->fb_buffer;
	fix->smem_start = (unsigned long) par->fb_buffer;
	fix->smem_len = par->pdata.width * par->pdata.height / 8;

	/* set fix based on var */
	fix->line_length = par->pdata.width / 8;

	/* set timings */
	par->hsync_period = ns_to_ktime((NSEC_PER_SEC /
					 hub12fb_refresh_rate(&info->var))/4);
	par->vsync_timeout = (2 * HZ) / hub12fb_refresh_rate(&info->var);
	hub12fb_set_brightness(par);

	printk(KERN_ALERT DRIVER_NAME
	       " timings (usec) hsync: %lld, led: %lld, vsync timeout: %ld\n",
	       ktime_to_us(par->hsync_period),
	       ktime_to_us(par->ledon_period),
	       (long) jiffies_to_usecs(par->vsync_timeout));

	info->flags = FBINFO_DEFAULT;

	if (par->blank == FB_BLANK_UNBLANK)
		hub12fb_start_running(par);

	return 0;
}

/**
 *      hub12fb_blank - NOT a required function. Blanks the display.
 *      @blank_mode: the blank mode we want.
 *      @info: frame buffer structure that represents a single frame buffer
 *
 *      If implementing this function, at least support FB_BLANK_UNBLANK.
 *      Return !0 for any modes that are unimplemented.
 *
 */
static int hub12fb_blank(int blank_mode, struct fb_info *info)
{
	struct hub12_par *par = info->par;

	par->blank = blank_mode;

	switch(blank_mode) {

	case FB_BLANK_NORMAL:         /* Normal blanking */
	case FB_BLANK_VSYNC_SUSPEND:  /* VESA blank (vsync off) */
	case FB_BLANK_HSYNC_SUSPEND:  /* VESA blank (hsync off) */
	case FB_BLANK_POWERDOWN:      /* Poweroff */
		hub12fb_stop_running(par);
		break;

	case FB_BLANK_UNBLANK: /* Unblanking */
		hub12fb_start_running(par);
		break;
	}
	return 0;
}

    /*
     *  Frame buffer operations
     */

static struct fb_ops hub12fb_ops = {
	.owner		= THIS_MODULE,
	.fb_check_var	= hub12fb_check_var,
	.fb_set_par	= hub12fb_set_par,
	.fb_blank	= hub12fb_blank,
	/* use this if no reversed pixels
	.fb_fillrect	= sys_fillrect,
	.fb_copyarea	= sys_copyarea,
	.fb_imageblit	= sys_imageblit,
	 */
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
};

/* ------------------------------------------------------------------------- */

    /*
     *  Initialization
     */

static int __devinit hub12fb_probe (struct spi_device *spidev)
{
	struct fb_info   *info;
	struct hub12_par *par;
	struct device    *device = &spidev->dev;
	struct fb_var_screeninfo *var;
	struct fb_fix_screeninfo *fix;
	struct hub12fb_platform_data *pdata;
	int retval = -ENOMEM;

	/*
	 * Dynamically allocate info and par
	 */

	info = framebuffer_alloc(sizeof(struct hub12_par), device);

	if (!info) {
		dev_err(device, "Unable to allocate framebuffer.");
		goto probe_fail_final;
	}
	par = info->par;
	memset(par,0,sizeof(struct hub12_par));
	par->spi = spidev;
	par->info = info;

	info->fbops = &hub12fb_ops;

	var = &info->var;
	fix = &info->fix;

	/* keep a copy in the parameters */
	memcpy(&par->pdata, spidev->dev.platform_data, sizeof(struct hub12fb_platform_data));
	pdata = &par->pdata;

	/* lets fill in fb_info */
	memcpy(&info->fix, &hub12fb_fix, sizeof(struct fb_fix_screeninfo));

	/* module command line overrides of pdata */
	if (gpio[0] != -1)
		pdata->gpio.enable = gpio[0];
	if (gpio[1] != -1)
		pdata->gpio.latch  = gpio[1];
	if (gpio[2] != -1)
		pdata->gpio.a      = gpio[2];
	if (gpio[3] != -1)
		pdata->gpio.b      = gpio[3];

	/* request the gpio pins, released when device is */
	retval = devm_gpio_request_one(device, pdata->gpio.enable,
					GPIOF_OUT_INIT_LOW, DRIVER_NAME "_oe");
	if (retval) goto probe_fail_free_fb;

	retval = devm_gpio_request_one(device, pdata->gpio.latch,
					GPIOF_OUT_INIT_LOW, DRIVER_NAME "_la");
	if (retval) goto probe_fail_free_fb;

	retval = devm_gpio_request_one(device, pdata->gpio.a,
					GPIOF_OUT_INIT_LOW, DRIVER_NAME "_a");
	if (retval) goto probe_fail_free_fb;

	retval = devm_gpio_request_one(device, pdata->gpio.b,
					GPIOF_OUT_INIT_LOW, DRIVER_NAME "_b");
	if (retval) goto probe_fail_free_fb;

	/* register the backlight before we register the framebuffer. */
	init_hub12bl(info);

	/*
	if (!info->bl_dev) {
		dev_err(device, "Unable to register backlight.\n");
		goto probe_fail_free_fb;
	}
	*/

	/* initialize kernel thread structures in hub12_par */

	init_waitqueue_head(&par->vsync_wait);

	hrtimer_init(&par->hsync_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	par->hsync_timer.function = do_hsync;

	hrtimer_init(&par->ledon_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	par->ledon_timer.function = ledon_expired;


	/* some filling in of fb_var_screeninfo */
	var->xres 	= par->pdata.width;
	var->yres 	= par->pdata.height;
	var->pixclock	= refresh_to_pixclock(par->pdata.refresh, var);

	/* lets take care of initial mode override now */
	if (mode)
		hub12fb_modestr_to_var(mode, var);

	/* possibly need this? */
	/*
	 *  var->nonstd		= FB_NONSTD_REV_PIX_IN_B;
	 */

	info->fbops = &hub12fb_ops;
	info->flags = FBINFO_FLAG_DEFAULT;
	info->pseudo_palette = par->pseudo_palette; /* not used */

	hub12fb_check_var(&info->var, info);

	/* does allocation of buffers, and sets thread in motion */
	retval = hub12fb_set_par(info);
	if (retval)
		goto probe_fail_free_bl;

	if (fb_alloc_cmap(&info->cmap, 256, 0)) {
		retval = -ENOMEM;
		goto probe_fail_free_buffers;
	}

	if (register_framebuffer(info) < 0) {
		printk(KERN_ERR DRIVER_NAME ": unable to register framebuffer.\n");
		retval = -EINVAL;
		goto probe_fail_free_cmap;
	}

	spi_set_drvdata(spidev, info);

	return retval;

probe_fail_free_cmap:
	fb_dealloc_cmap(&info->cmap);

probe_fail_free_buffers:
	hub12fb_free_buffers(par);

probe_fail_free_bl:
	exit_hub12bl(info);

probe_fail_free_fb:
	framebuffer_release(info);

probe_fail_final:
	return retval;
}

    /*
     *  Cleanup
     */
static int __devexit hub12fb_remove(struct spi_device *spidev)
{
	struct fb_info *info = spi_get_drvdata(spidev);

	if (info) {
		hub12fb_stop_running(info->par);
		unregister_framebuffer(info);
		spi_set_drvdata(spidev, NULL);
		fb_dealloc_cmap(&info->cmap);
		exit_hub12bl(info);
		hub12fb_free_buffers(info->par);
		/* free gpio ? */
		framebuffer_release(info);
	}
	return 0;
}

#ifdef CONFIG_PM
/**
 *	hub12fb_suspend - Suspend the device.
 *	@dev: spi device
 *	@msg: the suspend event code.
 */
static int hub12fb_suspend(struct spi_device *dev, pm_message_t msg)
{
	struct fb_info *info = spi_get_drvdata(dev);

	if (!info)
		return -EINVAL;

	/* suspend is easy... just stop the line writing */
	hub12fb_stop_running(info->par);

	return 0;
}

/**
 *	hub12fb_resume - Resume the device.
 *	@dev: spi device
 */
static int hub12fb_resume(struct spi_device *dev)
{
	struct fb_info *info = spi_get_drvdata(dev);

	if (!info)
		return -EINVAL;

	/* resume here */
	hub12fb_start_running(info->par);

	return 0;
}
#else
#define hub12fb_suspend NULL
#define hub12fb_resume NULL
#endif /* CONFIG_PM */


#ifndef MODULE
/*
 * Only necessary if your driver takes special options,
 * otherwise we fall back on the generic fb_setup().
 */
static int __init hub12fb_setup(char *options)
{
	char *this_opt;
	DBG(__func__)

	if (!options || !*options)
		return 0;

	while ((this_opt = strsep(&options, ",")) != NULL) {
		if (!*this_opt) continue;
	}

	return 0;
}
#endif /* MODULE */


static const struct spi_device_id hub12fb_ids[] = {
	{ HUB12FB_MODALIAS,	0 },
	{ },
};

MODULE_DEVICE_TABLE(spi, hub12fb_ids);

static struct spi_driver hub12fb_driver = {
	.driver		= {
		.name  = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.id_table	= hub12fb_ids,
	.probe		= hub12fb_probe,
	.remove		= __devexit_p(hub12fb_remove),
	.suspend	= hub12fb_suspend,
	.resume		= hub12fb_resume,
};

/*
 * dynamic loading of device not setup with board_info from mach driver
 *
 * expanding on Scott Ellis's idea https://gist.github.com/scottellis/716613
 *
 * probably should go away, but great for Raspberry Pi or other
 * hackable boards
 *
 */
static struct spi_device * dynamic_loaded_device = NULL;

#ifdef MODULE
static int check_params_for_dynamic_loading(void)
{
	if (spi[0] != -1) {
		board_info.bus_num = spi[0];
		if (spi[1] != -1)
			board_info.chip_select = spi[1];
		if (spi[2] != -1)
			board_info.max_speed_hz = spi[2];
		return 1;
	}
	return 0;
}

static int __init add_hub12fb_device_to_bus(void)
{
	struct spi_master *spi_master;
	struct spi_device *spi_device;
	char buff[64];
	int retval = 0;
	u16 spi_bus = board_info.bus_num;
	u8  spi_cs  = board_info.chip_select;

	spi_master = spi_busnum_to_master(spi_bus);
	if (!spi_master) {
		printk(KERN_ALERT "spi_busnum_to_master(%d) returned NULL\n",
		       spi_bus);
		printk(KERN_ALERT "Missing modprobe of spi master?\n");
		return -1;
	}

	/* Check whether this SPI bus.cs is already claimed */
	snprintf(buff, sizeof(buff), "%s.%u",
		 dev_name(&spi_master->dev), spi_cs);

	spi_device = to_spi_device(bus_find_device_by_name(&spi_bus_type,
							   NULL, buff));

	if (spi_device) {
		/* There is already a device configured for this bus.cs
		 * combination. If there is no driver loaded, we remove the
		 * driverless device (clear away spidev for example).
                 * If a driver is loaded and it is not us, we complain and fail.
		 */
		spi_dev_put(spi_device); /* we won't keep a reference */
		printk(KERN_ALERT DRIVER_NAME
		       " found existing device [%s], modalias = [%s]\n",
		       dev_name(&spi_device->dev), spi_device->modalias);
		if (!spi_device->dev.driver) {
			printk(KERN_ALERT DRIVER_NAME
			       " Removing driverless device at %s\n", buff);
			spi_unregister_device(spi_device);
			spi_device = NULL; /* lets add ours now! */
		} else if (strcmp(DRIVER_NAME, spi_device->dev.driver->name)) {
			/* not us.. :-( */
			printk(KERN_ALERT DRIVER_NAME
				": Driver [%s] already registered for %s\n",
				spi_device->dev.driver->name, buff);
			printk(KERN_ALERT DRIVER_NAME ": Try \"rmmod %s\" ?\n",
			       spi_device->dev.driver->name);
			retval = -1;
		}
	}
	if (!spi_device) {
		/* board info already modified and ready */

		dynamic_loaded_device = spi_new_device(spi_master, &board_info);

		if (dynamic_loaded_device) {
			printk(KERN_ALERT DRIVER_NAME
			       " loaded dynamically on spi%d.%d\n",
			       spi_bus, spi_cs);
		} else {
			printk(KERN_ERR DRIVER_NAME
			       ": spi_new_device() failed.\n");
			retval = -1;
		}
	}

	spi_master_put(spi_master);

	return retval;
}
#endif

static int __init hub12fb_init(void)
{
	int retval;
#ifndef MODULE
	/*
	 *  For kernel boot options (in 'video=hub12fb:<options>' format)
	 */
	char *option = NULL;

	if (fb_get_options("hub12fb", &option))
		return -ENODEV;

	if (option) mode = option; /* save option for probe */
#else
	/* let's first try to add the device in case it doesn't exist yet */
	/* which would be the case if we didn't add it to the board driver */

	if (check_params_for_dynamic_loading()) {
		retval = add_hub12fb_device_to_bus();
		if (retval)
			return retval;
	}
#endif
	return spi_register_driver(&hub12fb_driver);
}

static void __exit hub12fb_exit(void)
{
	spi_unregister_driver(&hub12fb_driver);
	if (dynamic_loaded_device) {
		spi_unregister_device(dynamic_loaded_device);
		spi_dev_put(dynamic_loaded_device);
		dynamic_loaded_device = NULL;
	}
}

/* ------------------------------------------------------------------------- */


    /*
     *  Modularization
     */

module_init(hub12fb_init);
module_exit(hub12fb_exit);


module_param(mode, charp, 0);
MODULE_PARM_DESC(mode,
		 "Specify video mode as \"<xres>x<yres>[@<refresh>]\" ");

module_param_array(spi, int, NULL, 0);
MODULE_PARM_DESC(spi_option,
		 "Dynamically bind to SPI with spi=<bus>,<cs>,<maxspeed>");

module_param_array(gpio, int, NULL, 0);
MODULE_PARM_DESC(spi_option,
		 "Specifiy GPIO connections as gpio=<oe>,<la>,<a>,<b>");

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Darren Garnier");
MODULE_ALIAS("spi:" HUB12FB_MODALIAS);
