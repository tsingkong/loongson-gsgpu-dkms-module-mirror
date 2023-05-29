/*
 * Copyright 2008 Advanced Micro Devices, Inc.
 * Copyright 2008 Red Hat Inc.
 * Copyright 2009 Jerome Glisse.
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
 *          Jerome Glisse
 */
#ifndef __GSGPU_OBJECT_H__
#define __GSGPU_OBJECT_H__

#include <drm/gsgpu_drm.h>
#include "gsgpu.h"
#include "gsgpu_res_cursor.h"

#ifdef CONFIG_MMU_NOTIFIER
#include <linux/mmu_notifier.h>
#endif

#define GSGPU_BO_INVALID_OFFSET	LONG_MAX
#define GSGPU_BO_MAX_PLACEMENTS	3

/* BO flag to indicate a KFD userptr BO */
#define GSGPU_AMDKFD_CREATE_USERPTR_BO	(1ULL << 63)

#define to_gsgpu_bo_user(abo) container_of((abo), struct gsgpu_bo_user, bo)
#define to_gsgpu_bo_vm(abo) container_of((abo), struct gsgpu_bo_vm, bo)

struct gsgpu_bo_param {
	unsigned long			size;
	int				byte_align;
	u32				bo_ptr_size;
	u32				domain;
	u32				preferred_domain;
	u64				flags;
	enum ttm_bo_type		type;
	bool				no_wait_gpu;
	struct dma_resv			*resv;
	void				(*destroy)(struct ttm_buffer_object *bo);
};

/* User space allocated BO in a VM */
struct gsgpu_bo_va {
	struct gsgpu_vm_bo_base	base;

	/* protected by bo being reserved */
	unsigned			ref_count;

	/* all other members protected by the VM PD being reserved */
	struct dma_fence	        *last_pt_update;

	/* mappings for this bo_va */
	struct list_head		invalids;
	struct list_head		valids;

	/* If the mappings are cleared or filled */
	bool				cleared;
};

struct gsgpu_bo {
	/* Protected by tbo.reserved */
	u32				preferred_domains;
	u32				allowed_domains;
	struct ttm_place		placements[GSGPU_BO_MAX_PLACEMENTS];
	struct ttm_placement		placement;
	struct ttm_buffer_object	tbo;
	struct ttm_bo_kmap_obj		kmap;
	u64				flags;
	u64				node_offset;
	/* per VM structure for page tables and with virtual addresses */
	struct gsgpu_vm_bo_base	*vm_bo;
	/* Constant after initialization */
	struct gsgpu_bo		*parent;

#ifdef CONFIG_MMU_NOTIFIER
	struct mmu_interval_notifier	notifier;
#endif
};

struct gsgpu_bo_user {
	struct gsgpu_bo		bo;
	u64				tiling_flags;
	u64				metadata_flags;
	void				*metadata;
	u32				metadata_size;

};

struct gsgpu_bo_vm {
	struct gsgpu_bo		bo;
	struct gsgpu_bo		*shadow;
	struct list_head		shadow_list;
	struct gsgpu_vm_bo_base        entries[];
};

static inline struct gsgpu_bo *ttm_to_gsgpu_bo(struct ttm_buffer_object *tbo)
{
	return container_of(tbo, struct gsgpu_bo, tbo);
}

/**
 * gsgpu_mem_type_to_domain - return domain corresponding to mem_type
 * @mem_type:	ttm memory type
 *
 * Returns corresponding domain of the ttm mem_type
 */
static inline unsigned gsgpu_mem_type_to_domain(u32 mem_type)
{
	switch (mem_type) {
	case TTM_PL_VRAM:
		return GSGPU_GEM_DOMAIN_VRAM;
	case TTM_PL_TT:
		return GSGPU_GEM_DOMAIN_GTT;
	case TTM_PL_SYSTEM:
		return GSGPU_GEM_DOMAIN_CPU;
	default:
		break;
	}
	return 0;
}

/**
 * gsgpu_bo_reserve - reserve bo
 * @bo:		bo structure
 * @no_intr:	don't return -ERESTARTSYS on pending signal
 *
 * Returns:
 * -ERESTARTSYS: A wait for the buffer to become unreserved was interrupted by
 * a signal. Release all buffer reservations and return to user-space.
 */
static inline int gsgpu_bo_reserve(struct gsgpu_bo *bo, bool no_intr)
{
	struct gsgpu_device *adev = gsgpu_ttm_adev(bo->tbo.bdev);
	int r;

	r = ttm_bo_reserve(&bo->tbo, !no_intr, false, NULL);
	if (unlikely(r != 0)) {
		if (r != -ERESTARTSYS)
			dev_err(adev->dev, "%p reserve failed\n", bo);
		return r;
	}
	return 0;
}

static inline void gsgpu_bo_unreserve(struct gsgpu_bo *bo)
{
	ttm_bo_unreserve(&bo->tbo);
}

static inline unsigned long gsgpu_bo_size(struct gsgpu_bo *bo)
{
	return bo->tbo.base.size;
}

static inline unsigned gsgpu_bo_ngpu_pages(struct gsgpu_bo *bo)
{
	return bo->tbo.base.size / GSGPU_GPU_PAGE_SIZE;
}

static inline unsigned gsgpu_bo_gpu_page_alignment(struct gsgpu_bo *bo)
{
	return (bo->tbo.page_alignment << PAGE_SHIFT) / GSGPU_GPU_PAGE_SIZE;
}

/**
 * gsgpu_bo_mmap_offset - return mmap offset of bo
 * @bo:	gsgpu object for which we query the offset
 *
 * Returns mmap offset of the object.
 */
static inline u64 gsgpu_bo_mmap_offset(struct gsgpu_bo *bo)
{
	return drm_vma_node_offset_addr(&bo->tbo.base.vma_node);
}

/**
 * gsgpu_bo_in_cpu_visible_vram - check if BO is (partly) in visible VRAM
 */
static inline bool gsgpu_bo_in_cpu_visible_vram(struct gsgpu_bo *bo)
{
	struct gsgpu_device *adev = gsgpu_ttm_adev(bo->tbo.bdev);
	struct gsgpu_res_cursor cursor;

	if (bo->tbo.resource->mem_type != TTM_PL_VRAM)
		return false;

	gsgpu_res_first(bo->tbo.resource, 0, gsgpu_bo_size(bo), &cursor);
	while (cursor.remaining) {
		if (cursor.start < adev->gmc.visible_vram_size)
			return true;

		gsgpu_res_next(&cursor, cursor.size);
	}

	return false;
}

/**
 * gsgpu_bo_explicit_sync - return whether the bo is explicitly synced
 */
static inline bool gsgpu_bo_explicit_sync(struct gsgpu_bo *bo)
{
	return bo->flags & GSGPU_GEM_CREATE_EXPLICIT_SYNC;
}

/**
 * gsgpu_bo_encrypted - test if the BO is encrypted
 * @bo: pointer to a buffer object
 *
 * Return true if the buffer object is encrypted, false otherwise.
 */
static inline bool gsgpu_bo_encrypted(struct gsgpu_bo *bo)
{
	return bo->flags & GSGPU_GEM_CREATE_ENCRYPTED;
}

/**
 * gsgpu_bo_shadowed - check if the BO is shadowed
 *
 * @bo: BO to be tested.
 *
 * Returns:
 * NULL if not shadowed or else return a BO pointer.
 */
static inline struct gsgpu_bo *gsgpu_bo_shadowed(struct gsgpu_bo *bo)
{
	if (bo->tbo.type == ttm_bo_type_kernel)
		return to_gsgpu_bo_vm(bo)->shadow;

	return NULL;
}

bool gsgpu_bo_is_gsgpu_bo(struct ttm_buffer_object *bo);
void gsgpu_bo_placement_from_domain(struct gsgpu_bo *abo, u32 domain);

int gsgpu_bo_create(struct gsgpu_device *adev,
		     struct gsgpu_bo_param *bp,
		     struct gsgpu_bo **bo_ptr);
int gsgpu_bo_create_reserved(struct gsgpu_device *adev,
			      unsigned long size, int align,
			      u32 domain, struct gsgpu_bo **bo_ptr,
			      u64 *gpu_addr, void **cpu_addr);
int gsgpu_bo_create_kernel(struct gsgpu_device *adev,
			    unsigned long size, int align,
			    u32 domain, struct gsgpu_bo **bo_ptr,
			    u64 *gpu_addr, void **cpu_addr);
int gsgpu_bo_create_kernel_at(struct gsgpu_device *adev,
			       uint64_t offset, uint64_t size,
			       struct gsgpu_bo **bo_ptr, void **cpu_addr);
int gsgpu_bo_create_user(struct gsgpu_device *adev,
			  struct gsgpu_bo_param *bp,
			  struct gsgpu_bo_user **ubo_ptr);
int gsgpu_bo_create_vm(struct gsgpu_device *adev,
			struct gsgpu_bo_param *bp,
			struct gsgpu_bo_vm **ubo_ptr);
void gsgpu_bo_free_kernel(struct gsgpu_bo **bo, u64 *gpu_addr,
			   void **cpu_addr);
int gsgpu_bo_kmap(struct gsgpu_bo *bo, void **ptr);
void *gsgpu_bo_kptr(struct gsgpu_bo *bo);
void gsgpu_bo_kunmap(struct gsgpu_bo *bo);
struct gsgpu_bo *gsgpu_bo_ref(struct gsgpu_bo *bo);
void gsgpu_bo_unref(struct gsgpu_bo **bo);
int gsgpu_bo_pin(struct gsgpu_bo *bo, u32 domain);
int gsgpu_bo_pin_restricted(struct gsgpu_bo *bo, u32 domain,
			     u64 min_offset, u64 max_offset);
void gsgpu_bo_unpin(struct gsgpu_bo *bo);
int gsgpu_bo_init(struct gsgpu_device *adev);
void gsgpu_bo_fini(struct gsgpu_device *adev);
int gsgpu_bo_set_tiling_flags(struct gsgpu_bo *bo, u64 tiling_flags);
void gsgpu_bo_get_tiling_flags(struct gsgpu_bo *bo, u64 *tiling_flags);
int gsgpu_bo_set_metadata (struct gsgpu_bo *bo, void *metadata,
			    uint32_t metadata_size, uint64_t flags);
int gsgpu_bo_get_metadata(struct gsgpu_bo *bo, void *buffer,
			   size_t buffer_size, uint32_t *metadata_size,
			   uint64_t *flags);
void gsgpu_bo_move_notify(struct ttm_buffer_object *bo,
			   bool evict,
			   struct ttm_resource *new_mem);
void gsgpu_bo_release_notify(struct ttm_buffer_object *bo);
vm_fault_t gsgpu_bo_fault_reserve_notify(struct ttm_buffer_object *bo);
void gsgpu_bo_fence(struct gsgpu_bo *bo, struct dma_fence *fence,
		     bool shared);
int gsgpu_bo_sync_wait_resv(struct gsgpu_device *adev, struct dma_resv *resv,
			     enum gsgpu_sync_mode sync_mode, void *owner,
			     bool intr);
int gsgpu_bo_sync_wait(struct gsgpu_bo *bo, void *owner, bool intr);
u64 gsgpu_bo_gpu_offset(struct gsgpu_bo *bo);
u64 gsgpu_bo_gpu_offset_no_check(struct gsgpu_bo *bo);
void gsgpu_bo_get_memory(struct gsgpu_bo *bo, uint64_t *vram_mem,
				uint64_t *gtt_mem, uint64_t *cpu_mem);
void gsgpu_bo_add_to_shadow_list(struct gsgpu_bo_vm *vmbo);
int gsgpu_bo_restore_shadow(struct gsgpu_bo *shadow,
			     struct dma_fence **fence);
uint32_t gsgpu_bo_get_preferred_domain(struct gsgpu_device *adev,
					    uint32_t domain);

/*
 * sub allocation
 */

static inline uint64_t gsgpu_sa_bo_gpu_addr(struct gsgpu_sa_bo *sa_bo)
{
	return sa_bo->manager->gpu_addr + sa_bo->soffset;
}

static inline void * gsgpu_sa_bo_cpu_addr(struct gsgpu_sa_bo *sa_bo)
{
	return sa_bo->manager->cpu_ptr + sa_bo->soffset;
}

int gsgpu_sa_bo_manager_init(struct gsgpu_device *adev,
				     struct gsgpu_sa_manager *sa_manager,
				     unsigned size, u32 align, u32 domain);
void gsgpu_sa_bo_manager_fini(struct gsgpu_device *adev,
				      struct gsgpu_sa_manager *sa_manager);
int gsgpu_sa_bo_manager_start(struct gsgpu_device *adev,
				      struct gsgpu_sa_manager *sa_manager);
int gsgpu_sa_bo_new(struct gsgpu_sa_manager *sa_manager,
		     struct gsgpu_sa_bo **sa_bo,
		     unsigned size, unsigned align);
void gsgpu_sa_bo_free(struct gsgpu_device *adev,
			      struct gsgpu_sa_bo **sa_bo,
			      struct dma_fence *fence);
#if defined(CONFIG_DEBUG_FS)
void gsgpu_sa_bo_dump_debug_info(struct gsgpu_sa_manager *sa_manager,
					 struct seq_file *m);
u64 gsgpu_bo_print_info(int id, struct gsgpu_bo *bo, struct seq_file *m);
#endif
void gsgpu_debugfs_sa_init(struct gsgpu_device *adev);

bool gsgpu_bo_support_uswc(u64 bo_flags);


#endif
