/*
 * Copyright 2016 Advanced Micro Devices, Inc.
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

#ifndef __GSGPU_TTM_H__
#define __GSGPU_TTM_H__

#include <linux/dma-direction.h>
#include <drm/gpu_scheduler.h>
#include "gsgpu_vram_mgr.h"
#include "gsgpu.h"

#define GSGPU_PL_GDS		(TTM_PL_PRIV + 0)
#define GSGPU_PL_GWS		(TTM_PL_PRIV + 1)
#define GSGPU_PL_OA		(TTM_PL_PRIV + 2)
#define GSGPU_PL_PREEMPT	(TTM_PL_PRIV + 3)

#define GSGPU_GTT_MAX_TRANSFER_SIZE	512
#define GSGPU_GTT_NUM_TRANSFER_WINDOWS	2

#define GSGPU_POISON	0xd0bed0be

struct hmm_range;

struct gsgpu_gtt_mgr {
	struct ttm_resource_manager manager;
	struct drm_mm mm;
	spinlock_t lock;
};

struct gsgpu_mman {
	struct ttm_device		bdev;
	bool				initialized;
	void __iomem			*aper_base_kaddr;

	/* buffer handling */
	const struct gsgpu_buffer_funcs	*buffer_funcs;
	struct gsgpu_ring			*buffer_funcs_ring;
	bool					buffer_funcs_enabled;

	struct mutex				gtt_window_lock;
	/* Scheduler entity for buffer moves */
	struct drm_sched_entity			entity;

	struct gsgpu_vram_mgr vram_mgr;
	struct gsgpu_gtt_mgr gtt_mgr;
	struct ttm_resource_manager preempt_mgr;

	uint64_t		stolen_vga_size;
	struct gsgpu_bo	*stolen_vga_memory;
	uint64_t		stolen_extended_size;
	struct gsgpu_bo	*stolen_extended_memory;
	bool			keep_stolen_vga_memory;

	struct gsgpu_bo	*stolen_reserved_memory;
	uint64_t		stolen_reserved_offset;
	uint64_t		stolen_reserved_size;

	/* firmware VRAM reservation */
	u64		fw_vram_usage_start_offset;
	u64		fw_vram_usage_size;
	struct gsgpu_bo	*fw_vram_usage_reserved_bo;
	void		*fw_vram_usage_va;

	/* driver VRAM reservation */
	u64		drv_vram_usage_start_offset;
	u64		drv_vram_usage_size;
	struct gsgpu_bo	*drv_vram_usage_reserved_bo;
	void		*drv_vram_usage_va;

	/* PAGE_SIZE'd BO for process memory r/w over SDMA. */
	struct gsgpu_bo	*sdma_access_bo;
	void			*sdma_access_ptr;
};

struct gsgpu_copy_mem {
	struct ttm_buffer_object	*bo;
	struct ttm_resource		*mem;
	unsigned long			offset;
};

int gsgpu_gtt_mgr_init(struct gsgpu_device *adev, uint64_t gtt_size);
void gsgpu_gtt_mgr_fini(struct gsgpu_device *adev);
int gsgpu_preempt_mgr_init(struct gsgpu_device *adev);
void gsgpu_preempt_mgr_fini(struct gsgpu_device *adev);
int gsgpu_vram_mgr_init(struct gsgpu_device *adev);
void gsgpu_vram_mgr_fini(struct gsgpu_device *adev);

bool gsgpu_gtt_mgr_has_gart_addr(struct ttm_resource *mem);
void gsgpu_gtt_mgr_recover(struct gsgpu_gtt_mgr *mgr);

uint64_t gsgpu_preempt_mgr_usage(struct ttm_resource_manager *man);

u64 gsgpu_vram_mgr_bo_visible_size(struct gsgpu_bo *bo);
int gsgpu_vram_mgr_alloc_sgt(struct gsgpu_device *adev,
			      struct ttm_resource *mem,
			      u64 offset, u64 size,
			      struct device *dev,
			      enum dma_data_direction dir,
			      struct sg_table **sgt);
void gsgpu_vram_mgr_free_sgt(struct device *dev,
			      enum dma_data_direction dir,
			      struct sg_table *sgt);
uint64_t gsgpu_vram_mgr_vis_usage(struct gsgpu_vram_mgr *mgr);
int gsgpu_vram_mgr_reserve_range(struct gsgpu_vram_mgr *mgr,
				  uint64_t start, uint64_t size);
int gsgpu_vram_mgr_query_page_status(struct gsgpu_vram_mgr *mgr,
				      uint64_t start);

int gsgpu_ttm_init(struct gsgpu_device *adev);
void gsgpu_ttm_fini(struct gsgpu_device *adev);
void gsgpu_ttm_set_buffer_funcs_status(struct gsgpu_device *adev,
					bool enable);

int gsgpu_copy_buffer(struct gsgpu_ring *ring, uint64_t src_offset,
		       uint64_t dst_offset, uint32_t byte_count,
		       struct dma_resv *resv,
		       struct dma_fence **fence, bool direct_submit,
		       bool vm_needs_flush, bool tmz);
int gsgpu_ttm_copy_mem_to_mem(struct gsgpu_device *adev,
			       const struct gsgpu_copy_mem *src,
			       const struct gsgpu_copy_mem *dst,
			       uint64_t size, bool tmz,
			       struct dma_resv *resv,
			       struct dma_fence **f);
int gsgpu_fill_buffer(struct gsgpu_bo *bo,
			uint32_t src_data,
			struct dma_resv *resv,
			struct dma_fence **fence);

int gsgpu_ttm_alloc_gart(struct ttm_buffer_object *bo);
void gsgpu_ttm_recover_gart(struct ttm_buffer_object *tbo);
uint64_t gsgpu_ttm_domain_start(struct gsgpu_device *adev, uint32_t type);

#if IS_ENABLED(CONFIG_DRM_GSGPU_USERPTR)
int gsgpu_ttm_tt_get_user_pages(struct gsgpu_bo *bo, struct page **pages,
				 struct hmm_range **range);
void gsgpu_ttm_tt_discard_user_pages(struct ttm_tt *ttm,
				      struct hmm_range *range);
bool gsgpu_ttm_tt_get_user_pages_done(struct ttm_tt *ttm,
				       struct hmm_range *range);
#else
static inline int gsgpu_ttm_tt_get_user_pages(struct gsgpu_bo *bo,
					       struct page **pages,
					       struct hmm_range **range)
{
	return -EPERM;
}
static inline void gsgpu_ttm_tt_discard_user_pages(struct ttm_tt *ttm,
						    struct hmm_range *range)
{
}
static inline bool gsgpu_ttm_tt_get_user_pages_done(struct ttm_tt *ttm,
						     struct hmm_range *range)
{
	return false;
}
#endif

void gsgpu_ttm_tt_set_user_pages(struct ttm_tt *ttm, struct page **pages);
int gsgpu_ttm_tt_get_userptr(const struct ttm_buffer_object *tbo,
			      uint64_t *user_addr);
int gsgpu_ttm_tt_set_userptr(struct ttm_buffer_object *bo,
			      uint64_t addr, uint32_t flags);
bool gsgpu_ttm_tt_has_userptr(struct ttm_tt *ttm);
struct mm_struct *gsgpu_ttm_tt_get_usermm(struct ttm_tt *ttm);
bool gsgpu_ttm_tt_affect_userptr(struct ttm_tt *ttm, unsigned long start,
				  unsigned long end, unsigned long *userptr);
bool gsgpu_ttm_tt_userptr_invalidated(struct ttm_tt *ttm,
				       int *last_invalidated);
bool gsgpu_ttm_tt_is_userptr(struct ttm_tt *ttm);
bool gsgpu_ttm_tt_is_readonly(struct ttm_tt *ttm);
uint64_t gsgpu_ttm_tt_pde_flags(struct ttm_tt *ttm, struct ttm_resource *mem);
uint64_t gsgpu_ttm_tt_pte_flags(struct gsgpu_device *adev, struct ttm_tt *ttm,
				 struct ttm_resource *mem);
int gsgpu_ttm_evict_resources(struct gsgpu_device *adev, int mem_type);

void gsgpu_ttm_debugfs_init(struct gsgpu_device *adev);

#endif
