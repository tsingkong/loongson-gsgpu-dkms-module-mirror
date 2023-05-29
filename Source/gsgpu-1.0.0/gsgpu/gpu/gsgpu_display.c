/*
 * Copyright 2007-8 Advanced Micro Devices, Inc.
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
 * Authors: Dave Airlie
 *          Alex Deucher
 */

#include <drm/gsgpu_drm.h>
#include "gsgpu.h"
#include "gsgpu_display.h"

#include <linux/pci.h>
#include <linux/pm_runtime.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_modeset_helper.h>
#include <drm/drm_vblank.h>

static int gsgpu_display_framebuffer_init(struct drm_device *dev,
					   struct gsgpu_framebuffer *rfb,
					   const struct drm_mode_fb_cmd2 *mode_cmd,
					   struct drm_gem_object *obj);

static void gsgpu_display_flip_callback(struct dma_fence *f,
					 struct dma_fence_cb *cb)
{
	struct gsgpu_flip_work *work =
		container_of(cb, struct gsgpu_flip_work, cb);

	dma_fence_put(f);
	schedule_work(&work->flip_work.work);
}

static bool gsgpu_display_flip_handle_fence(struct gsgpu_flip_work *work,
					     struct dma_fence **f)
{
	struct dma_fence *fence= *f;

	if (fence == NULL)
		return false;

	*f = NULL;

	if (!dma_fence_add_callback(fence, &work->cb,
				    gsgpu_display_flip_callback))
		return true;

	dma_fence_put(fence);
	return false;
}

static void gsgpu_display_flip_work_func(struct work_struct *__work)
{
	struct delayed_work *delayed_work =
		container_of(__work, struct delayed_work, work);
	struct gsgpu_flip_work *work =
		container_of(delayed_work, struct gsgpu_flip_work, flip_work);
	struct gsgpu_device *adev = work->adev;
	struct gsgpu_crtc *gsgpu_crtc = adev->mode_info.crtcs[work->crtc_id];

	struct drm_crtc *crtc = &gsgpu_crtc->base;
	unsigned long flags;
	unsigned i;
	int vpos, hpos;

	for (i = 0; i < work->shared_count; ++i)
		if (gsgpu_display_flip_handle_fence(work, &work->shared[i]))
			return;

	/* Wait until we're out of the vertical blank period before the one
	 * targeted by the flip
	 */
	if (gsgpu_crtc->enabled &&
	    (gsgpu_display_get_crtc_scanoutpos(adev_to_drm(adev), work->crtc_id, 0,
						&vpos, &hpos, NULL, NULL,
						&crtc->hwmode)
	     & (DRM_SCANOUTPOS_VALID | DRM_SCANOUTPOS_IN_VBLANK)) ==
	    (DRM_SCANOUTPOS_VALID | DRM_SCANOUTPOS_IN_VBLANK) &&
	    (int)(work->target_vblank -
		  gsgpu_get_vblank_counter_kms(crtc)) > 0) {
		schedule_delayed_work(&work->flip_work, usecs_to_jiffies(1000));
		return;
	}

	/* We borrow the event spin lock for protecting flip_status */
	spin_lock_irqsave(&crtc->dev->event_lock, flags);

	/* Do the flip (mmio) */
	adev->mode_info.funcs->page_flip(adev, work->crtc_id, work->base, work->async);

	/* Set the flip status */
	gsgpu_crtc->pflip_status = GSGPU_FLIP_SUBMITTED;
	spin_unlock_irqrestore(&crtc->dev->event_lock, flags);


	drm_dbg_vbl(adev_to_drm(adev),
		    "crtc:%d[%p], pflip_stat:GSGPU_FLIP_SUBMITTED, work: %p,\n",
		    gsgpu_crtc->crtc_id, gsgpu_crtc, work);

}

/*
 * Handle unpin events outside the interrupt handler proper.
 */
static void gsgpu_display_unpin_work_func(struct work_struct *__work)
{
	struct gsgpu_flip_work *work =
		container_of(__work, struct gsgpu_flip_work, unpin_work);
	int r;

	/* unpin of the old buffer */
	r = gsgpu_bo_reserve(work->old_abo, true);
	if (likely(r == 0)) {
		gsgpu_bo_unpin(work->old_abo);
		gsgpu_bo_unreserve(work->old_abo);
	} else
		DRM_ERROR("failed to reserve buffer after flip\n");

	gsgpu_bo_unref(&work->old_abo);
	kfree(work->shared);
	kfree(work);
}

int gsgpu_display_crtc_page_flip_target(struct drm_crtc *crtc,
				struct drm_framebuffer *fb,
				struct drm_pending_vblank_event *event,
				uint32_t page_flip_flags, uint32_t target,
				struct drm_modeset_acquire_ctx *ctx)
{
	struct drm_device *dev = crtc->dev;
	struct gsgpu_device *adev = drm_to_adev(dev);
	struct gsgpu_crtc *gsgpu_crtc = to_gsgpu_crtc(crtc);
	struct drm_gem_object *obj;
	struct gsgpu_flip_work *work;
	struct gsgpu_bo *new_abo;
	unsigned long flags;
	u64 tiling_flags;
	int i, r;
	void *fb_vaddr = NULL;

	work = kzalloc(sizeof *work, GFP_KERNEL);
	if (work == NULL)
		return -ENOMEM;

	INIT_DELAYED_WORK(&work->flip_work, gsgpu_display_flip_work_func);
	INIT_WORK(&work->unpin_work, gsgpu_display_unpin_work_func);

	work->event = event;
	work->adev = adev;
	work->crtc_id = gsgpu_crtc->crtc_id;
	work->async = (page_flip_flags & DRM_MODE_PAGE_FLIP_ASYNC) != 0;

	/* schedule unpin of the old buffer */
	obj = crtc->primary->fb->obj[0];

	/* take a reference to the old object */
	work->old_abo = gem_to_gsgpu_bo(obj);
	gsgpu_bo_ref(work->old_abo);

	obj = fb->obj[0];
	new_abo = gem_to_gsgpu_bo(obj);

	/* pin the new buffer */
	r = gsgpu_bo_reserve(new_abo, false);
	if (unlikely(r != 0)) {
		DRM_ERROR("failed to reserve new abo buffer before flip\n");
		goto cleanup;
	}

	if (!adev->enable_virtual_display) {
		r = gsgpu_bo_pin(new_abo,
				  gsgpu_display_supported_domains(adev, new_abo->flags));
		if (unlikely(r != 0)) {
			DRM_ERROR("failed to pin new abo buffer before flip\n");
			goto unreserve;
		}
	}

	r = gsgpu_ttm_alloc_gart(&new_abo->tbo);
	if (unlikely(r != 0)) {
		DRM_ERROR("%p bind failed\n", new_abo);
		goto unpin;
	}

	r = dma_resv_get_fences(new_abo->tbo.base.resv, DMA_RESV_USAGE_WRITE,
				&work->shared_count,
				&work->shared);
	if (unlikely(r != 0)) {
		DRM_ERROR("failed to get fences for buffer\n");
		goto unpin;
	}

	gsgpu_bo_get_tiling_flags(new_abo, &tiling_flags);

	gsgpu_bo_kmap(new_abo, &fb_vaddr);

	if (gsgpu_using_ram) {
		work->base = virt_to_phys(fb_vaddr);
		/* 0x460000000 - 0x46fffffff to 0x20000000 - 0x2fffffff */
		work->base = work->base & 0x3fffffff;
	} else {
		work->base = gsgpu_bo_gpu_offset(new_abo);
	}

	gsgpu_bo_unreserve(new_abo);

	work->target_vblank = target - (uint32_t)drm_crtc_vblank_count(crtc) +
		gsgpu_get_vblank_counter_kms(crtc);

	/* we borrow the event spin lock for protecting flip_wrok */
	spin_lock_irqsave(&crtc->dev->event_lock, flags);
	if (gsgpu_crtc->pflip_status != GSGPU_FLIP_NONE) {
		DRM_DEBUG_DRIVER("flip queue: crtc already busy\n");
		spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
		r = -EBUSY;
		goto pflip_cleanup;
	}

	gsgpu_crtc->pflip_status = GSGPU_FLIP_PENDING;
	gsgpu_crtc->pflip_works = work;


	DRM_DEBUG_DRIVER("crtc:%d[%p], pflip_stat:GSGPU_FLIP_PENDING, work: %p,\n",
					 gsgpu_crtc->crtc_id, gsgpu_crtc, work);
	/* update crtc fb */
	crtc->primary->fb = fb;
	spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
	gsgpu_display_flip_work_func(&work->flip_work.work);
	return 0;

pflip_cleanup:
	if (unlikely(gsgpu_bo_reserve(new_abo, false) != 0)) {
		DRM_ERROR("failed to reserve new abo in error path\n");
		goto cleanup;
	}
unpin:
	if (!adev->enable_virtual_display)
		gsgpu_bo_unpin(new_abo);

unreserve:
	gsgpu_bo_unreserve(new_abo);

cleanup:
	gsgpu_bo_unref(&work->old_abo);
	for (i = 0; i < work->shared_count; ++i)
		dma_fence_put(work->shared[i]);
	kfree(work->shared);
	kfree(work);

	return r;
}

static const struct drm_framebuffer_funcs gsgpu_fb_funcs = {
	.destroy = drm_gem_fb_destroy,
	.create_handle = drm_gem_fb_create_handle,
};

uint32_t gsgpu_display_supported_domains(struct gsgpu_device *adev,
					  uint64_t bo_flags)
{
	uint32_t domain = GSGPU_GEM_DOMAIN_VRAM;
	return domain;
}

static const struct drm_format_info dcc_formats[] = {
	{ .format = DRM_FORMAT_XRGB8888, .depth = 24, .num_planes = 2,
	  .cpp = { 4, 0, }, .block_w = {1, 1, 1}, .block_h = {1, 1, 1}, .hsub = 1, .vsub = 1, },
	 { .format = DRM_FORMAT_XBGR8888, .depth = 24, .num_planes = 2,
	  .cpp = { 4, 0, }, .block_w = {1, 1, 1}, .block_h = {1, 1, 1}, .hsub = 1, .vsub = 1, },
	{ .format = DRM_FORMAT_ARGB8888, .depth = 32, .num_planes = 2,
	  .cpp = { 4, 0, }, .block_w = {1, 1, 1}, .block_h = {1, 1, 1}, .hsub = 1, .vsub = 1,
	   .has_alpha = true, },
	{ .format = DRM_FORMAT_ABGR8888, .depth = 32, .num_planes = 2,
	  .cpp = { 4, 0, }, .block_w = {1, 1, 1}, .block_h = {1, 1, 1}, .hsub = 1, .vsub = 1,
	  .has_alpha = true, },
	{ .format = DRM_FORMAT_BGRA8888, .depth = 32, .num_planes = 2,
	  .cpp = { 4, 0, }, .block_w = {1, 1, 1}, .block_h = {1, 1, 1}, .hsub = 1, .vsub = 1,
	  .has_alpha = true, },
	{ .format = DRM_FORMAT_XRGB2101010, .depth = 30, .num_planes = 2,
	  .cpp = { 4, 0, }, .block_w = {1, 1, 1}, .block_h = {1, 1, 1}, .hsub = 1, .vsub = 1, },
	{ .format = DRM_FORMAT_XBGR2101010, .depth = 30, .num_planes = 2,
	  .cpp = { 4, 0, }, .block_w = {1, 1, 1}, .block_h = {1, 1, 1}, .hsub = 1, .vsub = 1, },
	{ .format = DRM_FORMAT_ARGB2101010, .depth = 30, .num_planes = 2,
	  .cpp = { 4, 0, }, .block_w = {1, 1, 1}, .block_h = {1, 1, 1}, .hsub = 1, .vsub = 1,
	  .has_alpha = true, },
	{ .format = DRM_FORMAT_ABGR2101010, .depth = 30, .num_planes = 2,
	  .cpp = { 4, 0, }, .block_w = {1, 1, 1}, .block_h = {1, 1, 1}, .hsub = 1, .vsub = 1,
	  .has_alpha = true, },
	{ .format = DRM_FORMAT_RGB565, .depth = 16, .num_planes = 2,
	  .cpp = { 2, 0, }, .block_w = {1, 1, 1}, .block_h = {1, 1, 1}, .hsub = 1, .vsub = 1, },
};

static const struct drm_format_info dcc_retile_formats[] = {
	{ .format = DRM_FORMAT_XRGB8888, .depth = 24, .num_planes = 3,
	  .cpp = { 4, 0, 0 }, .block_w = {1, 1, 1}, .block_h = {1, 1, 1}, .hsub = 1, .vsub = 1, },
	 { .format = DRM_FORMAT_XBGR8888, .depth = 24, .num_planes = 3,
	  .cpp = { 4, 0, 0 }, .block_w = {1, 1, 1}, .block_h = {1, 1, 1}, .hsub = 1, .vsub = 1, },
	{ .format = DRM_FORMAT_ARGB8888, .depth = 32, .num_planes = 3,
	  .cpp = { 4, 0, 0 }, .block_w = {1, 1, 1}, .block_h = {1, 1, 1}, .hsub = 1, .vsub = 1,
	   .has_alpha = true, },
	{ .format = DRM_FORMAT_ABGR8888, .depth = 32, .num_planes = 3,
	  .cpp = { 4, 0, 0 }, .block_w = {1, 1, 1}, .block_h = {1, 1, 1}, .hsub = 1, .vsub = 1,
	  .has_alpha = true, },
	{ .format = DRM_FORMAT_BGRA8888, .depth = 32, .num_planes = 3,
	  .cpp = { 4, 0, 0 }, .block_w = {1, 1, 1}, .block_h = {1, 1, 1}, .hsub = 1, .vsub = 1,
	  .has_alpha = true, },
	{ .format = DRM_FORMAT_XRGB2101010, .depth = 30, .num_planes = 3,
	  .cpp = { 4, 0, 0 }, .block_w = {1, 1, 1}, .block_h = {1, 1, 1}, .hsub = 1, .vsub = 1, },
	{ .format = DRM_FORMAT_XBGR2101010, .depth = 30, .num_planes = 3,
	  .cpp = { 4, 0, 0 }, .block_w = {1, 1, 1}, .block_h = {1, 1, 1}, .hsub = 1, .vsub = 1, },
	{ .format = DRM_FORMAT_ARGB2101010, .depth = 30, .num_planes = 3,
	  .cpp = { 4, 0, 0 }, .block_w = {1, 1, 1}, .block_h = {1, 1, 1}, .hsub = 1, .vsub = 1,
	  .has_alpha = true, },
	{ .format = DRM_FORMAT_ABGR2101010, .depth = 30, .num_planes = 3,
	  .cpp = { 4, 0, 0 }, .block_w = {1, 1, 1}, .block_h = {1, 1, 1}, .hsub = 1, .vsub = 1,
	  .has_alpha = true, },
	{ .format = DRM_FORMAT_RGB565, .depth = 16, .num_planes = 3,
	  .cpp = { 2, 0, 0 }, .block_w = {1, 1, 1}, .block_h = {1, 1, 1}, .hsub = 1, .vsub = 1, },
};

// static const struct drm_format_info *
// lookup_format_info(const struct drm_format_info formats[],
// 		  int num_formats, u32 format)
// {
// 	int i;

// 	for (i = 0; i < num_formats; i++) {
// 		if (formats[i].format == format)
// 			return &formats[i];
// 	}

// 	return NULL;
// }

/*
 * Tries to extract the renderable DCC offset from the opaque metadata attached
 * to the buffer.
 */
// static int
// extract_render_dcc_offset(struct gsgpu_device *adev,
// 			  struct drm_gem_object *obj,
// 			  uint64_t *offset)
// {
// 	struct gsgpu_bo *rbo;
// 	int r = 0;
// 	uint32_t metadata[10]; /* Something that fits a descriptor + header. */
// 	uint32_t size;

// 	rbo = gem_to_gsgpu_bo(obj);
// 	r = gsgpu_bo_reserve(rbo, false);

// 	if (unlikely(r)) {
// 		/* Don't show error message when returning -ERESTARTSYS */
// 		if (r != -ERESTARTSYS)
// 			DRM_ERROR("Unable to reserve buffer: %d\n", r);
// 		return r;
// 	}

// 	r = gsgpu_bo_get_metadata(rbo, metadata, sizeof(metadata), &size, NULL);
// 	gsgpu_bo_unreserve(rbo);

// 	if (r)
// 		return r;

// 	/*
// 	 * The first word is the metadata version, and we need space for at least
// 	 * the version + pci vendor+device id + 8 words for a descriptor.
// 	 */
// 	if (size < 40  || metadata[0] != 1)
// 		return -EINVAL;

// 	if (adev->family >= GSGPU_FAMILY_NV) {
// 		/* resource word 6/7 META_DATA_ADDRESS{_LO} */
// 		*offset = ((u64)metadata[9] << 16u) |
// 			  ((metadata[8] & 0xFF000000u) >> 16);
// 	} else {
// 		/* resource word 5/7 META_DATA_ADDRESS */
// 		*offset = ((u64)metadata[9] << 8u) |
// 			  ((u64)(metadata[7] & 0x1FE0000u) << 23);
// 	}

// 	return 0;
// }

// static void get_block_dimensions(unsigned int block_log2, unsigned int cpp,
// 				 unsigned int *width, unsigned int *height)
// {
// 	unsigned int cpp_log2 = ilog2(cpp);
// 	unsigned int pixel_log2 = block_log2 - cpp_log2;
// 	unsigned int width_log2 = (pixel_log2 + 1) / 2;
// 	unsigned int height_log2 = pixel_log2 - width_log2;

// 	*width = 1 << width_log2;
// 	*height = 1 << height_log2;
// }

// static int gsgpu_display_verify_plane(struct gsgpu_framebuffer *rfb, int plane,
// 				       const struct drm_format_info *format,
// 				       unsigned int block_width, unsigned int block_height,
// 				       unsigned int block_size_log2)
// {
// 	unsigned int width = rfb->base.width /
// 		((plane && plane < format->num_planes) ? format->hsub : 1);
// 	unsigned int height = rfb->base.height /
// 		((plane && plane < format->num_planes) ? format->vsub : 1);
// 	unsigned int cpp = plane < format->num_planes ? format->cpp[plane] : 1;
// 	unsigned int block_pitch = block_width * cpp;
// 	unsigned int min_pitch = ALIGN(width * cpp, block_pitch);
// 	unsigned int block_size = 1 << block_size_log2;
// 	uint64_t size;

// 	if (rfb->base.pitches[plane] % block_pitch) {
// 		drm_dbg_kms(rfb->base.dev,
// 			    "pitch %d for plane %d is not a multiple of block pitch %d\n",
// 			    rfb->base.pitches[plane], plane, block_pitch);
// 		return -EINVAL;
// 	}
// 	if (rfb->base.pitches[plane] < min_pitch) {
// 		drm_dbg_kms(rfb->base.dev,
// 			    "pitch %d for plane %d is less than minimum pitch %d\n",
// 			    rfb->base.pitches[plane], plane, min_pitch);
// 		return -EINVAL;
// 	}

// 	/* Force at least natural alignment. */
// 	if (rfb->base.offsets[plane] % block_size) {
// 		drm_dbg_kms(rfb->base.dev,
// 			    "offset 0x%x for plane %d is not a multiple of block pitch 0x%x\n",
// 			    rfb->base.offsets[plane], plane, block_size);
// 		return -EINVAL;
// 	}

// 	size = rfb->base.offsets[plane] +
// 		(uint64_t)rfb->base.pitches[plane] / block_pitch *
// 		block_size * DIV_ROUND_UP(height, block_height);

// 	if (rfb->base.obj[0]->size < size) {
// 		drm_dbg_kms(rfb->base.dev,
// 			    "BO size 0x%zx is less than 0x%llx required for plane %d\n",
// 			    rfb->base.obj[0]->size, size, plane);
// 		return -EINVAL;
// 	}

// 	return 0;
// }

static int gsgpu_display_get_fb_info(const struct gsgpu_framebuffer *gsgpu_fb,
				      uint64_t *tiling_flags, bool *tmz_surface)
{
	struct gsgpu_bo *rbo;
	int r;

	if (!gsgpu_fb) {
		*tiling_flags = 0;
		*tmz_surface = false;
		return 0;
	}

	rbo = gem_to_gsgpu_bo(gsgpu_fb->base.obj[0]);
	r = gsgpu_bo_reserve(rbo, false);

	if (unlikely(r)) {
		/* Don't show error message when returning -ERESTARTSYS */
		if (r != -ERESTARTSYS)
			DRM_ERROR("Unable to reserve buffer: %d\n", r);
		return r;
	}

	if (tiling_flags)
		gsgpu_bo_get_tiling_flags(rbo, tiling_flags);

	if (tmz_surface)
		*tmz_surface = gsgpu_bo_encrypted(rbo);

	gsgpu_bo_unreserve(rbo);

	return r;
}

static int gsgpu_display_gem_fb_verify_and_init(struct drm_device *dev,
						 struct gsgpu_framebuffer *rfb,
						 struct drm_file *file_priv,
						 const struct drm_mode_fb_cmd2 *mode_cmd,
						 struct drm_gem_object *obj)
{
	int ret;

	rfb->base.obj[0] = obj;
	drm_helper_mode_fill_fb_struct(dev, &rfb->base, mode_cmd);
	/* Verify that the modifier is supported. */
	if (!drm_any_plane_has_format(dev, mode_cmd->pixel_format,
				      mode_cmd->modifier[0])) {
		drm_dbg_kms(dev,
			    "unsupported pixel format %p4cc / modifier 0x%llx\n",
			    &mode_cmd->pixel_format, mode_cmd->modifier[0]);

		ret = -EINVAL;
		goto err;
	}

	ret = gsgpu_display_framebuffer_init(dev, rfb, mode_cmd, obj);
	if (ret)
		goto err;

	ret = drm_framebuffer_init(dev, &rfb->base, &gsgpu_fb_funcs);

	if (ret)
		goto err;

	return 0;
err:
	drm_dbg_kms(dev, "Failed to verify and init gem fb: %d\n", ret);
	rfb->base.obj[0] = NULL;
	return ret;
}

static int gsgpu_display_framebuffer_init(struct drm_device *dev,
					   struct gsgpu_framebuffer *rfb,
					   const struct drm_mode_fb_cmd2 *mode_cmd,
					   struct drm_gem_object *obj)
{
	// struct gsgpu_device *adev = drm_to_adev(dev);
	int ret, i;

	/*
	 * This needs to happen before modifier conversion as that might change
	 * the number of planes.
	 */
	for (i = 1; i < rfb->base.format->num_planes; ++i) {
		if (mode_cmd->handles[i] != mode_cmd->handles[0]) {
			drm_dbg_kms(dev, "Plane 0 and %d have different BOs: %u vs. %u\n",
				    i, mode_cmd->handles[0], mode_cmd->handles[i]);
			ret = -EINVAL;
			return ret;
		}
	}

	ret = gsgpu_display_get_fb_info(rfb, &rfb->tiling_flags, &rfb->tmz_surface);
	if (ret)
		return ret;

	for (i = 0; i < rfb->base.format->num_planes; ++i) {
		drm_gem_object_get(rfb->base.obj[0]);
		rfb->base.obj[i] = rfb->base.obj[0];
	}

	return 0;
}

struct drm_framebuffer *
gsgpu_display_user_framebuffer_create(struct drm_device *dev,
				       struct drm_file *file_priv,
				       const struct drm_mode_fb_cmd2 *mode_cmd)
{
	struct gsgpu_framebuffer *gsgpu_fb;
	struct drm_gem_object *obj;
	struct gsgpu_bo *bo;
	uint32_t domains;
	int ret;

	obj = drm_gem_object_lookup(file_priv, mode_cmd->handles[0]);
	if (obj ==  NULL) {
		drm_dbg_kms(dev, "No GEM object associated to handle 0x%08X, "
			    "can't create framebuffer\n", mode_cmd->handles[0]);
		return ERR_PTR(-ENOENT);
	}

	/* Handle is imported dma-buf, so cannot be migrated to VRAM for scanout */
	bo = gem_to_gsgpu_bo(obj);
	domains = gsgpu_display_supported_domains(drm_to_adev(dev), bo->flags);
	if (obj->import_attach && !(domains & GSGPU_GEM_DOMAIN_GTT)) {
		drm_dbg_kms(dev, "Cannot create framebuffer from imported dma_buf\n");
		drm_gem_object_put(obj);
		return ERR_PTR(-EINVAL);
	}

	gsgpu_fb = kzalloc(sizeof(*gsgpu_fb), GFP_KERNEL);
	if (gsgpu_fb == NULL) {
		drm_gem_object_put(obj);
		return ERR_PTR(-ENOMEM);
	}

	ret = gsgpu_display_gem_fb_verify_and_init(dev, gsgpu_fb, file_priv,
						    mode_cmd, obj);
	if (ret) {
		kfree(gsgpu_fb);
		drm_gem_object_put(obj);
		return ERR_PTR(ret);
	}

	drm_gem_object_put(obj);
	return &gsgpu_fb->base;
}

static const struct drm_prop_enum_list gsgpu_audio_enum_list[] =
{	{ GSGPU_AUDIO_DISABLE, "off" },
	{ GSGPU_AUDIO_ENABLE, "on" },
	{ GSGPU_AUDIO_AUTO, "auto" },
};

int gsgpu_display_modeset_create_props(struct gsgpu_device *adev)
{
	int sz;

	drm_mode_create_scaling_mode_property(adev_to_drm(adev));

	sz = ARRAY_SIZE(gsgpu_audio_enum_list);
	adev->mode_info.audio_property =
		drm_property_create_enum(adev_to_drm(adev), 0,
					 "audio",
					 gsgpu_audio_enum_list, sz);

	return 0;
}

/*
 * Retrieve current video scanout position of crtc on a given gpu, and
 * an optional accurate timestamp of when query happened.
 *
 * \param dev Device to query.
 * \param pipe Crtc to query.
 * \param flags Flags from caller (DRM_CALLED_FROM_VBLIRQ or 0).
 *              For driver internal use only also supports these flags:
 *
 *              USE_REAL_VBLANKSTART to use the real start of vblank instead
 *              of a fudged earlier start of vblank.
 *
 *              GET_DISTANCE_TO_VBLANKSTART to return distance to the
 *              fudged earlier start of vblank in *vpos and the distance
 *              to true start of vblank in *hpos.
 *
 * \param *vpos Location where vertical scanout position should be stored.
 * \param *hpos Location where horizontal scanout position should go.
 * \param *stime Target location for timestamp taken immediately before
 *               scanout position query. Can be NULL to skip timestamp.
 * \param *etime Target location for timestamp taken immediately after
 *               scanout position query. Can be NULL to skip timestamp.
 *
 * Returns vpos as a positive number while in active scanout area.
 * Returns vpos as a negative number inside vblank, counting the number
 * of scanlines to go until end of vblank, e.g., -1 means "one scanline
 * until start of active scanout / end of vblank."
 *
 * \return Flags, or'ed together as follows:
 *
 * DRM_SCANOUTPOS_VALID = Query successful.
 * DRM_SCANOUTPOS_INVBL = Inside vblank.
 * DRM_SCANOUTPOS_ACCURATE = Returned position is accurate. A lack of
 * this flag means that returned position may be offset by a constant but
 * unknown small number of scanlines wrt. real scanout position.
 *
 */
int gsgpu_display_get_crtc_scanoutpos(struct drm_device *dev,
			unsigned int pipe, unsigned int flags, int *vpos,
			int *hpos, ktime_t *stime, ktime_t *etime,
			const struct drm_display_mode *mode)
{
	u32 vbl = 0, position = 0;
	int vbl_start, vbl_end, vtotal, ret = 0;
	bool in_vbl = true;

	struct gsgpu_device *adev = drm_to_adev(dev);

	/* preempt_disable_rt() should go right here in PREEMPT_RT patchset. */

	/* Get optional system timestamp before query. */
	if (stime)
		*stime = ktime_get();

	if (gsgpu_display_page_flip_get_scanoutpos(adev, pipe, &vbl, &position) == 0)
		ret |= DRM_SCANOUTPOS_VALID;

	/* Get optional system timestamp after query. */
	if (etime)
		*etime = ktime_get();

	/* preempt_enable_rt() should go right here in PREEMPT_RT patchset. */

	/* Decode into vertical and horizontal scanout position. */
	*vpos = position & 0x1fff;
	*hpos = (position >> 16) & 0x1fff;

	/* Valid vblank area boundaries from gpu retrieved? */
	if (vbl > 0) {
		/* Yes: Decode. */
		ret |= DRM_SCANOUTPOS_ACCURATE;
		vbl_start = vbl & 0x1fff;
		vbl_end = (vbl >> 16) & 0x1fff;
	}
	else {
		/* No: Fake something reasonable which gives at least ok results. */
		vbl_start = mode->crtc_vdisplay;
		vbl_end = 0;
	}

	/* Called from driver internal vblank counter query code? */
	if (flags & GET_DISTANCE_TO_VBLANKSTART) {
	    /* Caller wants distance from real vbl_start in *hpos */
	    *hpos = *vpos - vbl_start;
	}

	/* Fudge vblank to start a few scanlines earlier to handle the
	 * problem that vblank irqs fire a few scanlines before start
	 * of vblank. Some driver internal callers need the true vblank
	 * start to be used and signal this via the USE_REAL_VBLANKSTART flag.
	 *
	 * The cause of the "early" vblank irq is that the irq is triggered
	 * by the line buffer logic when the line buffer read position enters
	 * the vblank, whereas our crtc scanout position naturally lags the
	 * line buffer read position.
	 */
	if (!(flags & USE_REAL_VBLANKSTART))
		vbl_start -= adev->mode_info.crtcs[pipe]->lb_vblank_lead_lines;

	/* Test scanout position against vblank region. */
	if ((*vpos < vbl_start) && (*vpos >= vbl_end))
		in_vbl = false;

	/* In vblank? */
	if (in_vbl)
	    ret |= DRM_SCANOUTPOS_IN_VBLANK;

	/* Called from driver internal vblank counter query code? */
	if (flags & GET_DISTANCE_TO_VBLANKSTART) {
		/* Caller wants distance from fudged earlier vbl_start */
		*vpos -= vbl_start;
		return ret;
	}

	/* Check if inside vblank area and apply corrective offsets:
	 * vpos will then be >=0 in video scanout area, but negative
	 * within vblank area, counting down the number of lines until
	 * start of scanout.
	 */

	/* Inside "upper part" of vblank area? Apply corrective offset if so: */
	if (in_vbl && (*vpos >= vbl_start)) {
		vtotal = mode->crtc_vtotal;

		/* With variable refresh rate displays the vpos can exceed
		 * the vtotal value. Clamp to 0 to return -vbl_end instead
		 * of guessing the remaining number of lines until scanout.
		 */
		*vpos = (*vpos < vtotal) ? (*vpos - vtotal) : 0;
	}

	/* Correct for shifted end of vbl at vbl_end. */
	*vpos = *vpos - vbl_end;

	return ret;
}

bool gsgpu_crtc_get_scanout_position(struct drm_crtc *crtc,
			bool in_vblank_irq, int *vpos,
			int *hpos, ktime_t *stime, ktime_t *etime,
			const struct drm_display_mode *mode)
{
	struct drm_device *dev = crtc->dev;
	unsigned int pipe = crtc->index;

	return gsgpu_display_get_crtc_scanoutpos(dev, pipe, 0, vpos, hpos,
						  stime, etime, mode);
}

// static bool
// gsgpu_display_robj_is_fb(struct gsgpu_device *adev, struct gsgpu_bo *robj)
// {
// 	struct drm_device *dev = adev_to_drm(adev);
// 	struct drm_fb_helper *fb_helper = dev->fb_helper;

// 	if (!fb_helper || !fb_helper->buffer)
// 		return false;

// 	if (gem_to_gsgpu_bo(fb_helper->buffer->gem) != robj)
// 		return false;

// 	return true;
// }
