/*
 * Copyright 2000 ATI Technologies Inc., Markham, Ontario, and
 *                VA Linux Systems Inc., Fremont, California.
 * Copyright 2008 Red Hat Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Original Authors:
 *   Kevin E. Martin, Rickard E. Faith, Alan Hourihane
 *
 * Kernel port Author: Dave Airlie
 */

#ifndef GSGPU_MODE_H
#define GSGPU_MODE_H

#include <drm/display/drm_dp_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_edid.h>
#include <drm/drm_encoder.h>
#include <drm/drm_fixed.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_probe_helper.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <linux/hrtimer.h>
#include "gsgpu_irq.h"

#include <drm/display/drm_dp_mst_helper.h>
#include "gsgpu_dc_irq.h"

struct gsgpu_bo;
struct gsgpu_device;
struct gsgpu_encoder;
struct gsgpu_router;
struct gsgpu_hpd;

#define to_gsgpu_crtc(x) container_of(x, struct gsgpu_crtc, base)
#define to_gsgpu_connector(x) container_of(x, struct gsgpu_connector, base)
#define to_gsgpu_encoder(x) container_of(x, struct gsgpu_encoder, base)
#define to_gsgpu_framebuffer(x) container_of(x, struct gsgpu_framebuffer, base)
#define to_gsgpu_plane(x)	container_of(x, struct gsgpu_plane, base)

#define GSGPU_MAX_HPD_PINS 3
#define GSGPU_MAX_CRTCS 2
#define GSGPU_MAX_PLANES 4

enum gsgpu_rmx_type {
	RMX_OFF,
	RMX_FULL,
	RMX_CENTER,
	RMX_ASPECT
};

enum gsgpu_underscan_type {
	UNDERSCAN_OFF,
	UNDERSCAN_ON,
	UNDERSCAN_AUTO,
};

enum gsgpu_hpd_id {
	GSGPU_HPD_1 = 0,
	GSGPU_HPD_2,
	GSGPU_HPD_3,
	GSGPU_HPD_4,
	GSGPU_HPD_5,
	GSGPU_HPD_6,
	GSGPU_HPD_NONE = 0xff,
};

enum gsgpu_crtc_irq {
	GSGPU_CRTC_IRQ_VBLANK1 = 0,
	GSGPU_CRTC_IRQ_VBLANK2,
	GSGPU_CRTC_IRQ_VBLANK3,
	GSGPU_CRTC_IRQ_VBLANK4,
	GSGPU_CRTC_IRQ_VBLANK5,
	GSGPU_CRTC_IRQ_VBLANK6,
	GSGPU_CRTC_IRQ_VLINE1,
	GSGPU_CRTC_IRQ_VLINE2,
	GSGPU_CRTC_IRQ_VLINE3,
	GSGPU_CRTC_IRQ_VLINE4,
	GSGPU_CRTC_IRQ_VLINE5,
	GSGPU_CRTC_IRQ_VLINE6,
	GSGPU_CRTC_IRQ_NONE = 0xff
};

enum gsgpu_pageflip_irq {
	GSGPU_PAGEFLIP_IRQ_D1 = 0,
	GSGPU_PAGEFLIP_IRQ_D2,
	GSGPU_PAGEFLIP_IRQ_D3,
	GSGPU_PAGEFLIP_IRQ_D4,
	GSGPU_PAGEFLIP_IRQ_D5,
	GSGPU_PAGEFLIP_IRQ_D6,
	GSGPU_PAGEFLIP_IRQ_NONE = 0xff
};

enum gsgpu_flip_status {
	GSGPU_FLIP_NONE,
	GSGPU_FLIP_PENDING,
	GSGPU_FLIP_SUBMITTED
};


/* gsgpu gpio-based i2c
 * 1. "mask" reg and bits
 *    grabs the gpio pins for software use
 *    0=not held  1=held
 * 2. "a" reg and bits
 *    output pin value
 *    0=low 1=high
 * 3. "en" reg and bits
 *    sets the pin direction
 *    0=input 1=output
 * 4. "y" reg and bits
 *    input pin value
 *    0=low 1=high
 */
struct gsgpu_i2c_bus_rec {
	bool valid;
	/* id used by atom */
	uint8_t i2c_id;
	/* id used by atom */
	enum gsgpu_hpd_id hpd;
	/* can be used with hw i2c engine */
	bool hw_capable;
	/* uses multi-media i2c engine */
	bool mm_i2c;
	/* regs and bits */
	uint32_t mask_clk_reg;
	uint32_t mask_data_reg;
	uint32_t a_clk_reg;
	uint32_t a_data_reg;
	uint32_t en_clk_reg;
	uint32_t en_data_reg;
	uint32_t y_clk_reg;
	uint32_t y_data_reg;
	uint32_t mask_clk_mask;
	uint32_t mask_data_mask;
	uint32_t a_clk_mask;
	uint32_t a_data_mask;
	uint32_t en_clk_mask;
	uint32_t en_data_mask;
	uint32_t y_clk_mask;
	uint32_t y_data_mask;
};

struct gsgpu_pll {
	/* reference frequency */
	uint32_t reference_freq;

	/* fixed dividers */
	uint32_t reference_div;
	uint32_t post_div;

	/* pll in/out limits */
	uint32_t pll_in_min;
	uint32_t pll_in_max;
	uint32_t pll_out_min;
	uint32_t pll_out_max;
	uint32_t lcd_pll_out_min;
	uint32_t lcd_pll_out_max;
	uint32_t best_vco;

	/* divider limits */
	uint32_t min_ref_div;
	uint32_t max_ref_div;
	uint32_t min_post_div;
	uint32_t max_post_div;
	uint32_t min_feedback_div;
	uint32_t max_feedback_div;
	uint32_t min_frac_feedback_div;
	uint32_t max_frac_feedback_div;

	/* flags for the current clock */
	uint32_t flags;

	/* pll id */
	uint32_t id;
};

struct gsgpu_i2c_chan {
	struct i2c_adapter adapter;
	struct drm_device *dev;
	struct i2c_algo_bit_data bit;
	struct gsgpu_i2c_bus_rec rec;
	struct drm_dp_aux aux;
	bool has_aux;
	struct mutex mutex;
};

struct gsgpu_afmt {
	bool enabled;
	int offset;
	bool last_buffer_filled_status;
	int id;
	struct gsgpu_audio_pin *pin;
};

struct gsgpu_display_funcs {
	/* get frame count */
	u32 (*vblank_get_counter)(struct gsgpu_device *adev, int crtc);
	/* set backlight level */
	void (*backlight_set_level)(struct gsgpu_encoder *gsgpu_encoder,
				    u8 level);
	/* get backlight level */
	u8 (*backlight_get_level)(struct gsgpu_encoder *gsgpu_encoder);
	/* hotplug detect */
	bool (*hpd_sense)(struct gsgpu_device *adev, enum gsgpu_hpd_id hpd);
	u32 (*hpd_get_gpio_reg)(struct gsgpu_device *adev);
	/* pageflipping */
	void (*page_flip)(struct gsgpu_device *adev,
			  int crtc_id, u64 crtc_base, bool async);
	int (*page_flip_get_scanoutpos)(struct gsgpu_device *adev, int crtc,
					u32 *vbl, u32 *position);
};

struct gsgpu_framebuffer {
	struct drm_framebuffer base;

	uint64_t tiling_flags;
	bool tmz_surface;

	/* caching for later use */
	uint64_t address;
};

struct gsgpu_mode_info {
	bool mode_config_initialized;
	struct gsgpu_crtc *crtcs[GSGPU_MAX_CRTCS];
	struct drm_plane *planes[GSGPU_MAX_PLANES];
	
	struct gsgpu_connector *connectors[2];
	struct gsgpu_encoder *encoders[2];

	/* underscan */
	struct drm_property *underscan_property;
	struct drm_property *underscan_hborder_property;
	struct drm_property *underscan_vborder_property;
	/* audio */
	struct drm_property *audio_property;
	
	/* pointer to backlight encoder */
	struct gsgpu_encoder *bl_encoder;
	u8 bl_level; /* saved backlight level */

	int			num_crtc; /* number of crtcs */
	int			num_hpd; /* number of hpd pins */
	int			num_i2c;
	bool			gpu_vm_support; /* supports display from GTT */
	int			disp_priority;
	const struct gsgpu_display_funcs *funcs;
	const enum drm_plane_type *plane_type;
};


struct gsgpu_backlight_privdata {
	struct gsgpu_encoder *encoder;
	uint8_t negative;
};

struct gsgpu_atom_ss {
	uint16_t percentage;
	uint16_t percentage_divider;
	uint8_t type;
	uint16_t step;
	uint8_t delay;
	uint8_t range;
	uint8_t refdiv;
	/* asic_ss */
	uint16_t rate;
	uint16_t amount;
};

struct gsgpu_crtc {
	struct drm_crtc base;
	int crtc_id;
	bool enabled;
	bool can_tile;
	uint32_t crtc_offset;
	enum dc_irq_source irq_source_vsync;
	struct drm_gem_object *cursor_bo;
	uint64_t cursor_addr;
	int cursor_x;
	int cursor_y;
	int cursor_hot_x;
	int cursor_hot_y;
	int cursor_width;
	int cursor_height;
	int max_cursor_width;
	int max_cursor_height;
	struct mutex cursor_lock;

	/* page flipping */
	struct gsgpu_flip_work *pflip_works;
	enum gsgpu_flip_status pflip_status;

	u32 lb_vblank_lead_lines;
	struct drm_display_mode hw_mode;

	struct drm_pending_vblank_event *event;
};

struct gsgpu_plane {
	struct drm_plane base;
	enum drm_plane_type plane_type;
};

struct gsgpu_encoder {
	struct drm_encoder base;
	uint32_t encoder_enum;
	uint32_t encoder_id;
	struct gsgpu_bridge_phy *bridge;
	uint32_t devices;
	uint32_t active_device;
	uint32_t flags;
	uint32_t pixel_clock;
	enum gsgpu_rmx_type rmx_type;
	enum gsgpu_underscan_type underscan_type;
	uint32_t underscan_hborder;
	uint32_t underscan_vborder;
	struct drm_display_mode native_mode;
	void *enc_priv;
	int audio_polling_active;
	bool is_ext_encoder;
	u16 caps;
};

struct gsgpu_connector_atom_dig {
	/* displayport */
	u8 dpcd[DP_RECEIVER_CAP_SIZE];
	u8 downstream_ports[DP_MAX_DOWNSTREAM_PORTS];
	u8 dp_sink_type;
	int dp_clock;
	int dp_lane_count;
	bool edp_on;
};

struct gsgpu_gpio_rec {
	bool valid;
	u8 id;
	u32 reg;
	u32 mask;
	u32 shift;
};

struct gsgpu_hpd {
	enum gsgpu_hpd_id hpd;
	u8 plugged_state;
	struct gsgpu_gpio_rec gpio;
};

struct gsgpu_router {
	u32 router_id;
	struct gsgpu_i2c_bus_rec i2c_info;
	u8 i2c_addr;
	/* i2c mux */
	bool ddc_valid;
	u8 ddc_mux_type;
	u8 ddc_mux_control_pin;
	u8 ddc_mux_state;
	/* clock/data mux */
	bool cd_valid;
	u8 cd_mux_type;
	u8 cd_mux_control_pin;
	u8 cd_mux_state;
};

enum gsgpu_connector_audio {
	GSGPU_AUDIO_DISABLE = 0,
	GSGPU_AUDIO_ENABLE = 1,
	GSGPU_AUDIO_AUTO = 2
};

struct gsgpu_connector {
	struct drm_connector base;
	uint32_t connector_id;
	uint32_t devices;

	enum gsgpu_connector_audio audio;
	
	int num_modes;
	int pixel_clock_mhz;
	struct mutex hpd_lock;
	enum dc_irq_source irq_source_i2c;
	enum dc_irq_source irq_source_hpd;
	enum dc_irq_source irq_source_vga_hpd;
};

/* Driver internal use only flags of gsgpu_display_get_crtc_scanoutpos() */
#define DRM_SCANOUTPOS_VALID        (1 << 0)
#define DRM_SCANOUTPOS_IN_VBLANK    (1 << 1)
#define DRM_SCANOUTPOS_ACCURATE     (1 << 2)
#define USE_REAL_VBLANKSTART		(1 << 30)
#define GET_DISTANCE_TO_VBLANKSTART	(1 << 31)

struct drm_connector *
gsgpu_get_connector_for_encoder(struct drm_encoder *encoder);

int gsgpu_display_get_crtc_scanoutpos(struct drm_device *dev,
			unsigned int pipe, unsigned int flags, int *vpos,
			int *hpos, ktime_t *stime, ktime_t *etime,
			const struct drm_display_mode *mode);

/* gsgpu_display.c */
int gsgpu_display_modeset_create_props(struct gsgpu_device *adev);

#endif
