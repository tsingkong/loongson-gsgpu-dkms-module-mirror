/*
 * Copyright 2018 Advanced Micro Devices, Inc.
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
 */
#ifndef __GSGPU_GEM_H__
#define __GSGPU_GEM_H__

#include <drm/gsgpu_drm.h>
#include <drm/drm_gem.h>

/*
 * GEM.
 */

#define GSGPU_GEM_DOMAIN_MAX		0x3
#define gem_to_gsgpu_bo(gobj) container_of((gobj), struct gsgpu_bo, tbo.base)

unsigned long gsgpu_gem_timeout(uint64_t timeout_ns);

/*
 * GEM objects.
 */
void gsgpu_gem_force_release(struct gsgpu_device *adev);
int gsgpu_gem_object_create(struct gsgpu_device *adev, unsigned long size,
			     int alignment, u32 initial_domain,
			     u64 flags, enum ttm_bo_type type,
			     struct dma_resv *resv,
			     struct drm_gem_object **obj);

int gsgpu_mode_dumb_create(struct drm_file *file_priv,
			    struct drm_device *dev,
			    struct drm_mode_create_dumb *args);
int gsgpu_mode_dumb_mmap(struct drm_file *filp,
			  struct drm_device *dev,
			  uint32_t handle, uint64_t *offset_p);

int gsgpu_gem_create_ioctl(struct drm_device *dev, void *data,
			    struct drm_file *filp);
int gsgpu_gem_info_ioctl(struct drm_device *dev, void *data,
			  struct drm_file *filp);
int gsgpu_gem_userptr_ioctl(struct drm_device *dev, void *data,
			struct drm_file *filp);
int gsgpu_gem_mmap_ioctl(struct drm_device *dev, void *data,
			  struct drm_file *filp);
int gsgpu_gem_wait_idle_ioctl(struct drm_device *dev, void *data,
			      struct drm_file *filp);
uint64_t gsgpu_gem_va_map_flags(struct gsgpu_device *adev, uint32_t flags);
int gsgpu_gem_va_ioctl(struct drm_device *dev, void *data,
			  struct drm_file *filp);
int gsgpu_gem_op_ioctl(struct drm_device *dev, void *data,
			struct drm_file *filp);

int gsgpu_gem_metadata_ioctl(struct drm_device *dev, void *data,
				struct drm_file *filp);

#endif
