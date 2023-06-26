/*
 * Copyright 2009 Jerome Glisse.
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sub license, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS, AUTHORS AND/OR ITS SUPPLIERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
 * USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 */
/*
 * Authors:
 *    Jerome Glisse <glisse@freedesktop.org>
 *    Thomas Hellstrom <thomas-at-tungstengraphics-dot-com>
 *    Dave Airlie
 */
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/dma-buf.h>

#include <drm/drm_drv.h>
#include <drm/gsgpu_drm.h>
#include <drm/drm_cache.h>
#include "gsgpu.h"
#include "gsgpu_trace.h"

/**
 * DOC: gsgpu_object
 *
 * This defines the interfaces to operate on an &gsgpu_bo buffer object which
 * represents memory used by driver (VRAM, system memory, etc.). The driver
 * provides DRM/GEM APIs to userspace. DRM/GEM APIs then use these interfaces
 * to create/destroy/set buffer object which are then managed by the kernel TTM
 * memory manager.
 * The interfaces are also used internally by kernel clients, including gfx,
 * uvd, etc. for kernel managed allocations used by the GPU.
 *
 */

static void gsgpu_bo_destroy(struct ttm_buffer_object *tbo)
{
	struct gsgpu_bo *bo = ttm_to_gsgpu_bo(tbo);

	gsgpu_bo_kunmap(bo);

	if (bo->tbo.base.import_attach)
		drm_prime_gem_destroy(&bo->tbo.base, bo->tbo.sg);
	drm_gem_object_release(&bo->tbo.base);
	gsgpu_bo_unref(&bo->parent);
	kvfree(bo);
}

static void gsgpu_bo_user_destroy(struct ttm_buffer_object *tbo)
{
	struct gsgpu_bo *bo = ttm_to_gsgpu_bo(tbo);
	struct gsgpu_bo_user *ubo;

	ubo = to_gsgpu_bo_user(bo);
	kfree(ubo->metadata);
	gsgpu_bo_destroy(tbo);
}

static void gsgpu_bo_vm_destroy(struct ttm_buffer_object *tbo)
{
	struct gsgpu_device *adev = gsgpu_ttm_adev(tbo->bdev);
	struct gsgpu_bo *bo = ttm_to_gsgpu_bo(tbo);
	struct gsgpu_bo_vm *vmbo;

	vmbo = to_gsgpu_bo_vm(bo);
	/* in case gsgpu_device_recover_vram got NULL of bo->parent */
	if (!list_empty(&vmbo->shadow_list)) {
		mutex_lock(&adev->shadow_list_lock);
		list_del_init(&vmbo->shadow_list);
		mutex_unlock(&adev->shadow_list_lock);
	}

	gsgpu_bo_destroy(tbo);
}

/**
 * gsgpu_bo_is_gsgpu_bo - check if the buffer object is an &gsgpu_bo
 * @bo: buffer object to be checked
 *
 * Uses destroy function associated with the object to determine if this is
 * an &gsgpu_bo.
 *
 * Returns:
 * true if the object belongs to &gsgpu_bo, false if not.
 */
bool gsgpu_bo_is_gsgpu_bo(struct ttm_buffer_object *bo)
{
	if (bo->destroy == &gsgpu_bo_destroy ||
	    bo->destroy == &gsgpu_bo_user_destroy ||
	    bo->destroy == &gsgpu_bo_vm_destroy)
		return true;

	return false;
}

/**
 * gsgpu_bo_placement_from_domain - set buffer's placement
 * @abo: &gsgpu_bo buffer object whose placement is to be set
 * @domain: requested domain
 *
 * Sets buffer's placement according to requested domain and the buffer's
 * flags.
 */
void gsgpu_bo_placement_from_domain(struct gsgpu_bo *abo, u32 domain)
{
	struct gsgpu_device *adev = gsgpu_ttm_adev(abo->tbo.bdev);
	struct ttm_placement *placement = &abo->placement;
	struct ttm_place *places = abo->placements;
	u64 flags = abo->flags;
	u32 c = 0;

	if (domain & GSGPU_GEM_DOMAIN_VRAM) {
		unsigned visible_pfn = adev->gmc.visible_vram_size >> PAGE_SHIFT;

		places[c].fpfn = 0;
		places[c].lpfn = 0;
		places[c].mem_type = TTM_PL_VRAM;
		places[c].flags = 0;

		if (flags & GSGPU_GEM_CREATE_CPU_ACCESS_REQUIRED)
			places[c].lpfn = visible_pfn;
		else if (adev->gmc.real_vram_size != adev->gmc.visible_vram_size)
			places[c].flags |= TTM_PL_FLAG_TOPDOWN;

		if (flags & GSGPU_GEM_CREATE_VRAM_CONTIGUOUS)
			places[c].flags |= TTM_PL_FLAG_CONTIGUOUS;
		c++;
	}

	if (domain & GSGPU_GEM_DOMAIN_GTT) {
		places[c].fpfn = 0;
		places[c].lpfn = 0;
		places[c].mem_type =
			abo->flags & GSGPU_GEM_CREATE_PREEMPTIBLE ?
			GSGPU_PL_PREEMPT : TTM_PL_TT;
		places[c].flags = 0;
		c++;
	}

	if (domain & GSGPU_GEM_DOMAIN_CPU) {
		places[c].fpfn = 0;
		places[c].lpfn = 0;
		places[c].mem_type = TTM_PL_SYSTEM;
		places[c].flags = 0;
		c++;
	}

	if (domain & GSGPU_GEM_DOMAIN_GDS) {
		places[c].fpfn = 0;
		places[c].lpfn = 0;
		places[c].mem_type = GSGPU_PL_GDS;
		places[c].flags = 0;
		c++;
	}

	if (domain & GSGPU_GEM_DOMAIN_GWS) {
		places[c].fpfn = 0;
		places[c].lpfn = 0;
		places[c].mem_type = GSGPU_PL_GWS;
		places[c].flags = 0;
		c++;
	}

	if (domain & GSGPU_GEM_DOMAIN_OA) {
		places[c].fpfn = 0;
		places[c].lpfn = 0;
		places[c].mem_type = GSGPU_PL_OA;
		places[c].flags = 0;
		c++;
	}

	if (!c) {
		places[c].fpfn = 0;
		places[c].lpfn = 0;
		places[c].mem_type = TTM_PL_SYSTEM;
		places[c].flags = 0;
		c++;
	}

	BUG_ON(c > GSGPU_BO_MAX_PLACEMENTS);

	placement->num_placement = c;
	placement->placement = places;

	placement->num_busy_placement = c;
	placement->busy_placement = places;
}

/**
 * gsgpu_bo_create_reserved - create reserved BO for kernel use
 *
 * @adev: gsgpu device object
 * @size: size for the new BO
 * @align: alignment for the new BO
 * @domain: where to place it
 * @bo_ptr: used to initialize BOs in structures
 * @gpu_addr: GPU addr of the pinned BO
 * @cpu_addr: optional CPU address mapping
 *
 * Allocates and pins a BO for kernel internal use, and returns it still
 * reserved.
 *
 * Note: For bo_ptr new BO is only created if bo_ptr points to NULL.
 *
 * Returns:
 * 0 on success, negative error code otherwise.
 */
int gsgpu_bo_create_reserved(struct gsgpu_device *adev,
			      unsigned long size, int align,
			      u32 domain, struct gsgpu_bo **bo_ptr,
			      u64 *gpu_addr, void **cpu_addr)
{
	struct gsgpu_bo_param bp;
	bool free = false;
	int r;

	if (!size) {
		gsgpu_bo_unref(bo_ptr);
		return 0;
	}

	memset(&bp, 0, sizeof(bp));
	bp.size = size;
	bp.byte_align = align;
	bp.domain = domain;
	bp.flags = cpu_addr ? GSGPU_GEM_CREATE_CPU_ACCESS_REQUIRED
		: GSGPU_GEM_CREATE_NO_CPU_ACCESS;
	bp.flags |= GSGPU_GEM_CREATE_VRAM_CONTIGUOUS;
	bp.type = ttm_bo_type_kernel;
	bp.resv = NULL;
	bp.bo_ptr_size = sizeof(struct gsgpu_bo);

	if (!*bo_ptr) {
		r = gsgpu_bo_create(adev, &bp, bo_ptr);
		if (r) {
			dev_err(adev->dev, "(%d) failed to allocate kernel bo\n",
				r);
			return r;
		}
		free = true;
	}

	r = gsgpu_bo_reserve(*bo_ptr, false);
	if (r) {
		dev_err(adev->dev, "(%d) failed to reserve kernel bo\n", r);
		goto error_free;
	}

	r = gsgpu_bo_pin(*bo_ptr, domain);
	if (r) {
		dev_err(adev->dev, "(%d) kernel bo pin failed\n", r);
		goto error_unreserve;
	}

	r = gsgpu_ttm_alloc_gart(&(*bo_ptr)->tbo);
	if (r) {
		dev_err(adev->dev, "%p bind failed\n", *bo_ptr);
		goto error_unpin;
	}

	if (gpu_addr)
		*gpu_addr = gsgpu_bo_gpu_offset(*bo_ptr);

	if (cpu_addr) {
		r = gsgpu_bo_kmap(*bo_ptr, cpu_addr);
		if (r) {
			dev_err(adev->dev, "(%d) kernel bo map failed\n", r);
			goto error_unpin;
		}
	}

	return 0;

error_unpin:
	gsgpu_bo_unpin(*bo_ptr);
error_unreserve:
	gsgpu_bo_unreserve(*bo_ptr);

error_free:
	if (free)
		gsgpu_bo_unref(bo_ptr);

	return r;
}

/**
 * gsgpu_bo_create_kernel - create BO for kernel use
 *
 * @adev: gsgpu device object
 * @size: size for the new BO
 * @align: alignment for the new BO
 * @domain: where to place it
 * @bo_ptr:  used to initialize BOs in structures
 * @gpu_addr: GPU addr of the pinned BO
 * @cpu_addr: optional CPU address mapping
 *
 * Allocates and pins a BO for kernel internal use.
 *
 * Note: For bo_ptr new BO is only created if bo_ptr points to NULL.
 *
 * Returns:
 * 0 on success, negative error code otherwise.
 */
int gsgpu_bo_create_kernel(struct gsgpu_device *adev,
			    unsigned long size, int align,
			    u32 domain, struct gsgpu_bo **bo_ptr,
			    u64 *gpu_addr, void **cpu_addr)
{
	int r;

	r = gsgpu_bo_create_reserved(adev, size, align, domain, bo_ptr,
				      gpu_addr, cpu_addr);

	if (r)
		return r;

	if (*bo_ptr)
		gsgpu_bo_unreserve(*bo_ptr);

	return 0;
}

/**
 * gsgpu_bo_create_kernel_at - create BO for kernel use at specific location
 *
 * @adev: gsgpu device object
 * @offset: offset of the BO
 * @size: size of the BO
 * @bo_ptr:  used to initialize BOs in structures
 * @cpu_addr: optional CPU address mapping
 *
 * Creates a kernel BO at a specific offset in VRAM.
 *
 * Returns:
 * 0 on success, negative error code otherwise.
 */
int gsgpu_bo_create_kernel_at(struct gsgpu_device *adev,
			       uint64_t offset, uint64_t size,
			       struct gsgpu_bo **bo_ptr, void **cpu_addr)
{
	struct ttm_operation_ctx ctx = { false, false };
	unsigned int i;
	int r;

	offset &= PAGE_MASK;
	size = ALIGN(size, PAGE_SIZE);

	r = gsgpu_bo_create_reserved(adev, size, PAGE_SIZE,
				      GSGPU_GEM_DOMAIN_VRAM, bo_ptr, NULL,
				      cpu_addr);
	if (r)
		return r;

	if ((*bo_ptr) == NULL)
		return 0;

	/*
	 * Remove the original mem node and create a new one at the request
	 * position.
	 */
	if (cpu_addr)
		gsgpu_bo_kunmap(*bo_ptr);

	ttm_resource_free(&(*bo_ptr)->tbo, &(*bo_ptr)->tbo.resource);

	for (i = 0; i < (*bo_ptr)->placement.num_placement; ++i) {
		(*bo_ptr)->placements[i].fpfn = offset >> PAGE_SHIFT;
		(*bo_ptr)->placements[i].lpfn = (offset + size) >> PAGE_SHIFT;
	}
	r = ttm_bo_mem_space(&(*bo_ptr)->tbo, &(*bo_ptr)->placement,
			     &(*bo_ptr)->tbo.resource, &ctx);
	if (r)
		goto error;

	if (cpu_addr) {
		r = gsgpu_bo_kmap(*bo_ptr, cpu_addr);
		if (r)
			goto error;
	}

	gsgpu_bo_unreserve(*bo_ptr);
	return 0;

error:
	gsgpu_bo_unreserve(*bo_ptr);
	gsgpu_bo_unref(bo_ptr);
	return r;
}

/**
 * gsgpu_bo_free_kernel - free BO for kernel use
 *
 * @bo: gsgpu BO to free
 * @gpu_addr: pointer to where the BO's GPU memory space address was stored
 * @cpu_addr: pointer to where the BO's CPU memory space address was stored
 *
 * unmaps and unpin a BO for kernel internal use.
 */
void gsgpu_bo_free_kernel(struct gsgpu_bo **bo, u64 *gpu_addr,
			   void **cpu_addr)
{
	if (*bo == NULL)
		return;

	WARN_ON(gsgpu_ttm_adev((*bo)->tbo.bdev)->in_suspend);

	if (likely(gsgpu_bo_reserve(*bo, true) == 0)) {
		if (cpu_addr)
			gsgpu_bo_kunmap(*bo);

		gsgpu_bo_unpin(*bo);
		gsgpu_bo_unreserve(*bo);
	}
	gsgpu_bo_unref(bo);

	if (gpu_addr)
		*gpu_addr = 0;

	if (cpu_addr)
		*cpu_addr = NULL;
}

/* Validate bo size is bit bigger then the request domain */
static bool gsgpu_bo_validate_size(struct gsgpu_device *adev,
					  unsigned long size, u32 domain)
{
	struct ttm_resource_manager *man = NULL;

	/*
	 * If GTT is part of requested domains the check must succeed to
	 * allow fall back to GTT.
	 */
	if (domain & GSGPU_GEM_DOMAIN_GTT) {
		man = ttm_manager_type(&adev->mman.bdev, TTM_PL_TT);

		if (man && size < man->size)
			return true;
		else if (!man)
			WARN_ON_ONCE("GTT domain requested but GTT mem manager uninitialized");
		goto fail;
	} else if (domain & GSGPU_GEM_DOMAIN_VRAM) {
		man = ttm_manager_type(&adev->mman.bdev, TTM_PL_VRAM);

		if (man && size < man->size)
			return true;
		goto fail;
	}

	/* TODO add more domains checks, such as GSGPU_GEM_DOMAIN_CPU */
	return true;

fail:
	if (man)
		DRM_DEBUG("BO size %lu > total memory in domain: %llu\n", size,
			  man->size);
	return false;
}

bool gsgpu_bo_support_uswc(u64 bo_flags)
{
	if (!drm_arch_can_wc_memory())
		return false;

	return true;
}

/**
 * gsgpu_bo_create - create an &gsgpu_bo buffer object
 * @adev: gsgpu device object
 * @bp: parameters to be used for the buffer object
 * @bo_ptr: pointer to the buffer object pointer
 *
 * Creates an &gsgpu_bo buffer object.
 *
 * Returns:
 * 0 for success or a negative error code on failure.
 */
int gsgpu_bo_create(struct gsgpu_device *adev,
			       struct gsgpu_bo_param *bp,
			       struct gsgpu_bo **bo_ptr)
{
	struct ttm_operation_ctx ctx = {
		.interruptible = (bp->type != ttm_bo_type_kernel),
		.no_wait_gpu = bp->no_wait_gpu,
		/* We opt to avoid OOM on system pages allocations */
		.gfp_retry_mayfail = true,
		.allow_res_evict = bp->type != ttm_bo_type_kernel,
		.resv = bp->resv
	};
	struct gsgpu_bo *bo;
	unsigned long page_align, size = bp->size;
	int r;

	/* Note that GDS/GWS/OA allocates 1 page per byte/resource. */
	if (bp->domain & (GSGPU_GEM_DOMAIN_GWS | GSGPU_GEM_DOMAIN_OA)) {
		/* GWS and OA don't need any alignment. */
		page_align = bp->byte_align;
		size <<= PAGE_SHIFT;

	} else if (bp->domain & GSGPU_GEM_DOMAIN_GDS) {
		/* Both size and alignment must be a multiple of 4. */
		page_align = ALIGN(bp->byte_align, 4);
		size = ALIGN(size, 4) << PAGE_SHIFT;
	} else {
		/* Memory should be aligned at least to a page size. */
		page_align = ALIGN(bp->byte_align, PAGE_SIZE) >> PAGE_SHIFT;
		size = ALIGN(size, PAGE_SIZE);
	}

	if (!gsgpu_bo_validate_size(adev, size, bp->domain))
		return -ENOMEM;

	BUG_ON(bp->bo_ptr_size < sizeof(struct gsgpu_bo));

	*bo_ptr = NULL;
	bo = kvzalloc(bp->bo_ptr_size, GFP_KERNEL);
	if (bo == NULL)
		return -ENOMEM;
	drm_gem_private_object_init(adev_to_drm(adev), &bo->tbo.base, size);
	bo->vm_bo = NULL;
	bo->preferred_domains = bp->preferred_domain ? bp->preferred_domain :
		bp->domain;
	bo->allowed_domains = bo->preferred_domains;
	if (bp->type != ttm_bo_type_kernel &&
	    !(bp->flags & GSGPU_GEM_CREATE_DISCARDABLE) &&
	    bo->allowed_domains == GSGPU_GEM_DOMAIN_VRAM)
		bo->allowed_domains |= GSGPU_GEM_DOMAIN_GTT;

	bo->flags = bp->flags;

	if (!gsgpu_bo_support_uswc(bo->flags))
		bo->flags &= ~GSGPU_GEM_CREATE_CPU_GTT_USWC;

	bo->tbo.bdev = &adev->mman.bdev;
	if (bp->domain & (GSGPU_GEM_DOMAIN_GWS | GSGPU_GEM_DOMAIN_OA |
			  GSGPU_GEM_DOMAIN_GDS))
		gsgpu_bo_placement_from_domain(bo, GSGPU_GEM_DOMAIN_CPU);
	else
		gsgpu_bo_placement_from_domain(bo, bp->domain);
	if (bp->type == ttm_bo_type_kernel)
		bo->tbo.priority = 1;

	if (!bp->destroy)
		bp->destroy = &gsgpu_bo_destroy;

	r = ttm_bo_init_reserved(&adev->mman.bdev, &bo->tbo, bp->type,
				 &bo->placement, page_align, &ctx,  NULL,
				 bp->resv, bp->destroy);
	if (unlikely(r != 0))
		return r;

	if (!gsgpu_gmc_vram_full_visible(&adev->gmc) &&
	    bo->tbo.resource->mem_type == TTM_PL_VRAM &&
	    bo->tbo.resource->start < adev->gmc.visible_vram_size >> PAGE_SHIFT)
		gsgpu_cs_report_moved_bytes(adev, ctx.bytes_moved,
					     ctx.bytes_moved);
	else
		gsgpu_cs_report_moved_bytes(adev, ctx.bytes_moved, 0);

	if (bp->flags & GSGPU_GEM_CREATE_VRAM_CLEARED &&
	    bo->tbo.resource->mem_type == TTM_PL_VRAM) {
		struct dma_fence *fence;

		r = gsgpu_fill_buffer(bo, 0, bo->tbo.base.resv, &fence);
		if (unlikely(r))
			goto fail_unreserve;

		dma_resv_add_fence(bo->tbo.base.resv, fence,
				   DMA_RESV_USAGE_KERNEL);
		dma_fence_put(fence);
	}
	if (!bp->resv)
		gsgpu_bo_unreserve(bo);
	*bo_ptr = bo;

	trace_gsgpu_bo_create(bo);

	/* Treat CPU_ACCESS_REQUIRED only as a hint if given by UMD */
	if (bp->type == ttm_bo_type_device)
		bo->flags &= ~GSGPU_GEM_CREATE_CPU_ACCESS_REQUIRED;

	return 0;

fail_unreserve:
	if (!bp->resv)
		dma_resv_unlock(bo->tbo.base.resv);
	gsgpu_bo_unref(&bo);
	return r;
}

/**
 * gsgpu_bo_create_user - create an &gsgpu_bo_user buffer object
 * @adev: gsgpu device object
 * @bp: parameters to be used for the buffer object
 * @ubo_ptr: pointer to the buffer object pointer
 *
 * Create a BO to be used by user application;
 *
 * Returns:
 * 0 for success or a negative error code on failure.
 */

int gsgpu_bo_create_user(struct gsgpu_device *adev,
			  struct gsgpu_bo_param *bp,
			  struct gsgpu_bo_user **ubo_ptr)
{
	struct gsgpu_bo *bo_ptr;
	int r;

	bp->bo_ptr_size = sizeof(struct gsgpu_bo_user);
	bp->destroy = &gsgpu_bo_user_destroy;
	r = gsgpu_bo_create(adev, bp, &bo_ptr);
	if (r)
		return r;

	*ubo_ptr = to_gsgpu_bo_user(bo_ptr);
	return r;
}

/**
 * gsgpu_bo_create_vm - create an &gsgpu_bo_vm buffer object
 * @adev: gsgpu device object
 * @bp: parameters to be used for the buffer object
 * @vmbo_ptr: pointer to the buffer object pointer
 *
 * Create a BO to be for GPUVM.
 *
 * Returns:
 * 0 for success or a negative error code on failure.
 */

int gsgpu_bo_create_vm(struct gsgpu_device *adev,
			struct gsgpu_bo_param *bp,
			struct gsgpu_bo_vm **vmbo_ptr)
{
	struct gsgpu_bo *bo_ptr;
	int r;

	/* bo_ptr_size will be determined by the caller and it depends on
	 * num of gsgpu_vm_pt entries.
	 */
	BUG_ON(bp->bo_ptr_size < sizeof(struct gsgpu_bo_vm));
	r = gsgpu_bo_create(adev, bp, &bo_ptr);
	if (r)
		return r;

	*vmbo_ptr = to_gsgpu_bo_vm(bo_ptr);
	INIT_LIST_HEAD(&(*vmbo_ptr)->shadow_list);
	/* Set destroy callback to gsgpu_bo_vm_destroy after vmbo->shadow_list
	 * is initialized.
	 */
	bo_ptr->tbo.destroy = &gsgpu_bo_vm_destroy;
	return r;
}

/**
 * gsgpu_bo_add_to_shadow_list - add a BO to the shadow list
 *
 * @vmbo: BO that will be inserted into the shadow list
 *
 * Insert a BO to the shadow list.
 */
void gsgpu_bo_add_to_shadow_list(struct gsgpu_bo_vm *vmbo)
{
	struct gsgpu_device *adev = gsgpu_ttm_adev(vmbo->bo.tbo.bdev);

	mutex_lock(&adev->shadow_list_lock);
	list_add_tail(&vmbo->shadow_list, &adev->shadow_list);
	mutex_unlock(&adev->shadow_list_lock);
}

/**
 * gsgpu_bo_restore_shadow - restore an &gsgpu_bo shadow
 *
 * @shadow: &gsgpu_bo shadow to be restored
 * @fence: dma_fence associated with the operation
 *
 * Copies a buffer object's shadow content back to the object.
 * This is used for recovering a buffer from its shadow in case of a gpu
 * reset where vram context may be lost.
 *
 * Returns:
 * 0 for success or a negative error code on failure.
 */
int gsgpu_bo_restore_shadow(struct gsgpu_bo *shadow, struct dma_fence **fence)

{
	struct gsgpu_device *adev = gsgpu_ttm_adev(shadow->tbo.bdev);
	struct gsgpu_ring *ring = adev->mman.buffer_funcs_ring;
	uint64_t shadow_addr, parent_addr;

	shadow_addr = gsgpu_bo_gpu_offset(shadow);
	parent_addr = gsgpu_bo_gpu_offset(shadow->parent);

	return gsgpu_copy_buffer(ring, shadow_addr, parent_addr,
				  gsgpu_bo_size(shadow), NULL, fence,
				  true, false, false);
}

/**
 * gsgpu_bo_kmap - map an &gsgpu_bo buffer object
 * @bo: &gsgpu_bo buffer object to be mapped
 * @ptr: kernel virtual address to be returned
 *
 * Calls ttm_bo_kmap() to set up the kernel virtual mapping; calls
 * gsgpu_bo_kptr() to get the kernel virtual address.
 *
 * Returns:
 * 0 for success or a negative error code on failure.
 */
int gsgpu_bo_kmap(struct gsgpu_bo *bo, void **ptr)
{
	void *kptr;
	long r;

	if (bo->flags & GSGPU_GEM_CREATE_NO_CPU_ACCESS)
		return -EPERM;

	r = dma_resv_wait_timeout(bo->tbo.base.resv, DMA_RESV_USAGE_KERNEL,
				  false, MAX_SCHEDULE_TIMEOUT);
	if (r < 0)
		return r;

	kptr = gsgpu_bo_kptr(bo);
	if (kptr) {
		if (ptr)
			*ptr = kptr;
		return 0;
	}

	r = ttm_bo_kmap(&bo->tbo, 0, PFN_UP(bo->tbo.base.size), &bo->kmap);
	if (r)
		return r;

	if (ptr)
		*ptr = gsgpu_bo_kptr(bo);

	return 0;
}

/**
 * gsgpu_bo_kptr - returns a kernel virtual address of the buffer object
 * @bo: &gsgpu_bo buffer object
 *
 * Calls ttm_kmap_obj_virtual() to get the kernel virtual address
 *
 * Returns:
 * the virtual address of a buffer object area.
 */
void *gsgpu_bo_kptr(struct gsgpu_bo *bo)
{
	bool is_iomem;

	return ttm_kmap_obj_virtual(&bo->kmap, &is_iomem);
}

/**
 * gsgpu_bo_kunmap - unmap an &gsgpu_bo buffer object
 * @bo: &gsgpu_bo buffer object to be unmapped
 *
 * Unmaps a kernel map set up by gsgpu_bo_kmap().
 */
void gsgpu_bo_kunmap(struct gsgpu_bo *bo)
{
	if (bo->kmap.bo)
		ttm_bo_kunmap(&bo->kmap);
}

/**
 * gsgpu_bo_ref - reference an &gsgpu_bo buffer object
 * @bo: &gsgpu_bo buffer object
 *
 * References the contained &ttm_buffer_object.
 *
 * Returns:
 * a refcounted pointer to the &gsgpu_bo buffer object.
 */
struct gsgpu_bo *gsgpu_bo_ref(struct gsgpu_bo *bo)
{
	if (bo == NULL)
		return NULL;

	ttm_bo_get(&bo->tbo);
	return bo;
}

/**
 * gsgpu_bo_unref - unreference an &gsgpu_bo buffer object
 * @bo: &gsgpu_bo buffer object
 *
 * Unreferences the contained &ttm_buffer_object and clear the pointer
 */
void gsgpu_bo_unref(struct gsgpu_bo **bo)
{
	struct ttm_buffer_object *tbo;

	if ((*bo) == NULL)
		return;

	tbo = &((*bo)->tbo);
	ttm_bo_put(tbo);
	*bo = NULL;
}

/**
 * gsgpu_bo_pin_restricted - pin an &gsgpu_bo buffer object
 * @bo: &gsgpu_bo buffer object to be pinned
 * @domain: domain to be pinned to
 * @min_offset: the start of requested address range
 * @max_offset: the end of requested address range
 *
 * Pins the buffer object according to requested domain and address range. If
 * the memory is unbound gart memory, binds the pages into gart table. Adjusts
 * pin_count and pin_size accordingly.
 *
 * Pinning means to lock pages in memory along with keeping them at a fixed
 * offset. It is required when a buffer can not be moved, for example, when
 * a display buffer is being scanned out.
 *
 * Compared with gsgpu_bo_pin(), this function gives more flexibility on
 * where to pin a buffer if there are specific restrictions on where a buffer
 * must be located.
 *
 * Returns:
 * 0 for success or a negative error code on failure.
 */
int gsgpu_bo_pin_restricted(struct gsgpu_bo *bo, u32 domain,
			     u64 min_offset, u64 max_offset)
{
	struct gsgpu_device *adev = gsgpu_ttm_adev(bo->tbo.bdev);
	struct ttm_operation_ctx ctx = { false, false };
	int r, i;

	if (gsgpu_ttm_tt_get_usermm(bo->tbo.ttm))
		return -EPERM;

	if (WARN_ON_ONCE(min_offset > max_offset))
		return -EINVAL;

	/* Check domain to be pinned to against preferred domains */
	if (bo->preferred_domains & domain)
		domain = bo->preferred_domains & domain;

	/* A shared bo cannot be migrated to VRAM */
	if (bo->tbo.base.import_attach) {
		if (domain & GSGPU_GEM_DOMAIN_GTT)
			domain = GSGPU_GEM_DOMAIN_GTT;
		else
			return -EINVAL;
	}

	if (bo->tbo.pin_count) {
		uint32_t mem_type = bo->tbo.resource->mem_type;
		uint32_t mem_flags = bo->tbo.resource->placement;

		if (!(domain & gsgpu_mem_type_to_domain(mem_type)))
			return -EINVAL;

		if ((mem_type == TTM_PL_VRAM) &&
		    (bo->flags & GSGPU_GEM_CREATE_VRAM_CONTIGUOUS) &&
		    !(mem_flags & TTM_PL_FLAG_CONTIGUOUS))
			return -EINVAL;

		ttm_bo_pin(&bo->tbo);

		if (max_offset != 0) {
			u64 domain_start = gsgpu_ttm_domain_start(adev,
								   mem_type);
			WARN_ON_ONCE(max_offset <
				     (gsgpu_bo_gpu_offset(bo) - domain_start));
		}

		return 0;
	}

	/* This assumes only APU display buffers are pinned with (VRAM|GTT).
	 * See function gsgpu_display_supported_domains()
	 */
	domain = gsgpu_bo_get_preferred_domain(adev, domain);

	if (bo->tbo.base.import_attach)
		dma_buf_pin(bo->tbo.base.import_attach);

	/* force to pin into visible video ram */
	if (!(bo->flags & GSGPU_GEM_CREATE_NO_CPU_ACCESS))
		bo->flags |= GSGPU_GEM_CREATE_CPU_ACCESS_REQUIRED;
	gsgpu_bo_placement_from_domain(bo, domain);
	for (i = 0; i < bo->placement.num_placement; i++) {
		unsigned fpfn, lpfn;

		fpfn = min_offset >> PAGE_SHIFT;
		lpfn = max_offset >> PAGE_SHIFT;

		if (fpfn > bo->placements[i].fpfn)
			bo->placements[i].fpfn = fpfn;
		if (!bo->placements[i].lpfn ||
		    (lpfn && lpfn < bo->placements[i].lpfn))
			bo->placements[i].lpfn = lpfn;
	}

	r = ttm_bo_validate(&bo->tbo, &bo->placement, &ctx);
	if (unlikely(r)) {
		dev_err(adev->dev, "%p pin failed\n", bo);
		goto error;
	}

	ttm_bo_pin(&bo->tbo);

	domain = gsgpu_mem_type_to_domain(bo->tbo.resource->mem_type);
	if (domain == GSGPU_GEM_DOMAIN_VRAM) {
		atomic64_add(gsgpu_bo_size(bo), &adev->vram_pin_size);
		atomic64_add(gsgpu_vram_mgr_bo_visible_size(bo),
			     &adev->visible_pin_size);
	} else if (domain == GSGPU_GEM_DOMAIN_GTT) {
		atomic64_add(gsgpu_bo_size(bo), &adev->gart_pin_size);
	}

error:
	return r;
}

/**
 * gsgpu_bo_pin - pin an &gsgpu_bo buffer object
 * @bo: &gsgpu_bo buffer object to be pinned
 * @domain: domain to be pinned to
 *
 * A simple wrapper to gsgpu_bo_pin_restricted().
 * Provides a simpler API for buffers that do not have any strict restrictions
 * on where a buffer must be located.
 *
 * Returns:
 * 0 for success or a negative error code on failure.
 */
int gsgpu_bo_pin(struct gsgpu_bo *bo, u32 domain)
{
	bo->flags |= GSGPU_GEM_CREATE_VRAM_CONTIGUOUS;
	return gsgpu_bo_pin_restricted(bo, domain, 0, 0);
}

/**
 * gsgpu_bo_unpin - unpin an &gsgpu_bo buffer object
 * @bo: &gsgpu_bo buffer object to be unpinned
 *
 * Decreases the pin_count, and clears the flags if pin_count reaches 0.
 * Changes placement and pin size accordingly.
 *
 * Returns:
 * 0 for success or a negative error code on failure.
 */
void gsgpu_bo_unpin(struct gsgpu_bo *bo)
{
	struct gsgpu_device *adev = gsgpu_ttm_adev(bo->tbo.bdev);

	ttm_bo_unpin(&bo->tbo);
	if (bo->tbo.pin_count)
		return;

	if (bo->tbo.base.import_attach)
		dma_buf_unpin(bo->tbo.base.import_attach);

	if (bo->tbo.resource->mem_type == TTM_PL_VRAM) {
		atomic64_sub(gsgpu_bo_size(bo), &adev->vram_pin_size);
		atomic64_sub(gsgpu_vram_mgr_bo_visible_size(bo),
			     &adev->visible_pin_size);
	} else if (bo->tbo.resource->mem_type == TTM_PL_TT) {
		atomic64_sub(gsgpu_bo_size(bo), &adev->gart_pin_size);
	}
}

static const char *gsgpu_vram_names[] = {
	"UNKNOWN",
	"GDDR1",
	"DDR2",
	"GDDR3",
	"GDDR4",
	"GDDR5",
	"HBM",
	"DDR3",
	"DDR4",
	"GDDR6",
	"DDR5",
	"LPDDR4",
	"LPDDR5"
};

/**
 * gsgpu_bo_init - initialize memory manager
 * @adev: gsgpu device object
 *
 * Calls gsgpu_ttm_init() to initialize gsgpu memory manager.
 *
 * Returns:
 * 0 for success or a negative error code on failure.
 */
int gsgpu_bo_init(struct gsgpu_device *adev)
{
	/* On A+A platform, VRAM can be mapped as WB */
	{
		/* reserve PAT memory space to WC for VRAM */
		int r = arch_io_reserve_memtype_wc(adev->gmc.aper_base,
				adev->gmc.aper_size);

		if (r) {
			DRM_ERROR("Unable to set WC memtype for the aperture base\n");
			return r;
		}

		/* Add an MTRR for the VRAM */
		adev->gmc.vram_mtrr = arch_phys_wc_add(adev->gmc.aper_base,
				adev->gmc.aper_size);
	}

	DRM_INFO("Detected VRAM RAM=%lluM, BAR=%lluM\n",
		 adev->gmc.mc_vram_size >> 20,
		 (unsigned long long)adev->gmc.aper_size >> 20);
	DRM_INFO("RAM width %dbits %s\n",
		 adev->gmc.vram_width, gsgpu_vram_names[adev->gmc.vram_type]);
	return gsgpu_ttm_init(adev);
}

/**
 * gsgpu_bo_fini - tear down memory manager
 * @adev: gsgpu device object
 *
 * Reverses gsgpu_bo_init() to tear down memory manager.
 */
void gsgpu_bo_fini(struct gsgpu_device *adev)
{
	int idx;

	gsgpu_ttm_fini(adev);

	if (drm_dev_enter(adev_to_drm(adev), &idx)) {

		{
			arch_phys_wc_del(adev->gmc.vram_mtrr);
			arch_io_free_memtype_wc(adev->gmc.aper_base, adev->gmc.aper_size);
		}
		drm_dev_exit(idx);
	}
}

/**
 * gsgpu_bo_set_tiling_flags - set tiling flags
 * @bo: &gsgpu_bo buffer object
 * @tiling_flags: new flags
 *
 * Sets buffer object's tiling flags with the new one. Used by GEM ioctl or
 * kernel driver to set the tiling flags on a buffer.
 *
 * Returns:
 * 0 for success or a negative error code on failure.
 */
int gsgpu_bo_set_tiling_flags(struct gsgpu_bo *bo, u64 tiling_flags)
{
	struct gsgpu_device *adev = gsgpu_ttm_adev(bo->tbo.bdev);
	struct gsgpu_bo_user *ubo;

	BUG_ON(bo->tbo.type == ttm_bo_type_kernel);
	if (adev->family <= GSGPU_FAMILY_CZ &&
	    GSGPU_TILING_GET(tiling_flags, TILE_SPLIT) > 6)
		return -EINVAL;

	ubo = to_gsgpu_bo_user(bo);
	ubo->tiling_flags = tiling_flags;
	return 0;
}

/**
 * gsgpu_bo_get_tiling_flags - get tiling flags
 * @bo: &gsgpu_bo buffer object
 * @tiling_flags: returned flags
 *
 * Gets buffer object's tiling flags. Used by GEM ioctl or kernel driver to
 * set the tiling flags on a buffer.
 */
void gsgpu_bo_get_tiling_flags(struct gsgpu_bo *bo, u64 *tiling_flags)
{
	struct gsgpu_bo_user *ubo;

	BUG_ON(bo->tbo.type == ttm_bo_type_kernel);
	dma_resv_assert_held(bo->tbo.base.resv);
	ubo = to_gsgpu_bo_user(bo);

	if (tiling_flags)
		*tiling_flags = ubo->tiling_flags;
}

/**
 * gsgpu_bo_set_metadata - set metadata
 * @bo: &gsgpu_bo buffer object
 * @metadata: new metadata
 * @metadata_size: size of the new metadata
 * @flags: flags of the new metadata
 *
 * Sets buffer object's metadata, its size and flags.
 * Used via GEM ioctl.
 *
 * Returns:
 * 0 for success or a negative error code on failure.
 */
int gsgpu_bo_set_metadata (struct gsgpu_bo *bo, void *metadata,
			    uint32_t metadata_size, uint64_t flags)
{
	struct gsgpu_bo_user *ubo;
	void *buffer;

	BUG_ON(bo->tbo.type == ttm_bo_type_kernel);
	ubo = to_gsgpu_bo_user(bo);
	if (!metadata_size) {
		if (ubo->metadata_size) {
			kfree(ubo->metadata);
			ubo->metadata = NULL;
			ubo->metadata_size = 0;
		}
		return 0;
	}

	if (metadata == NULL)
		return -EINVAL;

	buffer = kmemdup(metadata, metadata_size, GFP_KERNEL);
	if (buffer == NULL)
		return -ENOMEM;

	kfree(ubo->metadata);
	ubo->metadata_flags = flags;
	ubo->metadata = buffer;
	ubo->metadata_size = metadata_size;

	return 0;
}

/**
 * gsgpu_bo_get_metadata - get metadata
 * @bo: &gsgpu_bo buffer object
 * @buffer: returned metadata
 * @buffer_size: size of the buffer
 * @metadata_size: size of the returned metadata
 * @flags: flags of the returned metadata
 *
 * Gets buffer object's metadata, its size and flags. buffer_size shall not be
 * less than metadata_size.
 * Used via GEM ioctl.
 *
 * Returns:
 * 0 for success or a negative error code on failure.
 */
int gsgpu_bo_get_metadata(struct gsgpu_bo *bo, void *buffer,
			   size_t buffer_size, uint32_t *metadata_size,
			   uint64_t *flags)
{
	struct gsgpu_bo_user *ubo;

	if (!buffer && !metadata_size)
		return -EINVAL;

	BUG_ON(bo->tbo.type == ttm_bo_type_kernel);
	ubo = to_gsgpu_bo_user(bo);
	if (metadata_size)
		*metadata_size = ubo->metadata_size;

	if (buffer) {
		if (buffer_size < ubo->metadata_size)
			return -EINVAL;

		if (ubo->metadata_size)
			memcpy(buffer, ubo->metadata, ubo->metadata_size);
	}

	if (flags)
		*flags = ubo->metadata_flags;

	return 0;
}

/**
 * gsgpu_bo_move_notify - notification about a memory move
 * @bo: pointer to a buffer object
 * @evict: if this move is evicting the buffer from the graphics address space
 * @new_mem: new information of the bufer object
 *
 * Marks the corresponding &gsgpu_bo buffer object as invalid, also performs
 * bookkeeping.
 * TTM driver callback which is called when ttm moves a buffer.
 */
void gsgpu_bo_move_notify(struct ttm_buffer_object *bo,
			   bool evict,
			   struct ttm_resource *new_mem)
{
	struct gsgpu_device *adev = gsgpu_ttm_adev(bo->bdev);
	struct gsgpu_bo *abo;
	struct ttm_resource *old_mem = bo->resource;

	if (!gsgpu_bo_is_gsgpu_bo(bo))
		return;

	abo = ttm_to_gsgpu_bo(bo);
	gsgpu_vm_bo_invalidate(adev, abo, evict);

	gsgpu_bo_kunmap(abo);

	if (abo->tbo.base.dma_buf && !abo->tbo.base.import_attach &&
	    bo->resource->mem_type != TTM_PL_SYSTEM)
		dma_buf_move_notify(abo->tbo.base.dma_buf);

	/* remember the eviction */
	if (evict)
		atomic64_inc(&adev->num_evictions);

	/* update statistics */
	if (!new_mem)
		return;

	/* move_notify is called before move happens */
	trace_gsgpu_bo_move(abo, new_mem->mem_type, old_mem->mem_type);
}

void gsgpu_bo_get_memory(struct gsgpu_bo *bo, uint64_t *vram_mem,
				uint64_t *gtt_mem, uint64_t *cpu_mem)
{
	unsigned int domain;

	domain = gsgpu_mem_type_to_domain(bo->tbo.resource->mem_type);
	switch (domain) {
	case GSGPU_GEM_DOMAIN_VRAM:
		*vram_mem += gsgpu_bo_size(bo);
		break;
	case GSGPU_GEM_DOMAIN_GTT:
		*gtt_mem += gsgpu_bo_size(bo);
		break;
	case GSGPU_GEM_DOMAIN_CPU:
	default:
		*cpu_mem += gsgpu_bo_size(bo);
		break;
	}
}

/**
 * gsgpu_bo_release_notify - notification about a BO being released
 * @bo: pointer to a buffer object
 *
 * Wipes VRAM buffers whose contents should not be leaked before the
 * memory is released.
 */
void gsgpu_bo_release_notify(struct ttm_buffer_object *bo)
{
	struct gsgpu_device *adev = gsgpu_ttm_adev(bo->bdev);
	struct dma_fence *fence = NULL;
	struct gsgpu_bo *abo;
	int r;

	if (!gsgpu_bo_is_gsgpu_bo(bo))
		return;

	abo = ttm_to_gsgpu_bo(bo);

	/* We only remove the fence if the resv has individualized. */
	WARN_ON_ONCE(bo->type == ttm_bo_type_kernel
			&& bo->base.resv != &bo->base._resv);

	if (!bo->resource || bo->resource->mem_type != TTM_PL_VRAM ||
	    !(abo->flags & GSGPU_GEM_CREATE_VRAM_WIPE_ON_RELEASE) ||
	    adev->in_suspend || drm_dev_is_unplugged(adev_to_drm(adev)))
		return;

	if (WARN_ON_ONCE(!dma_resv_trylock(bo->base.resv)))
		return;

	r = gsgpu_fill_buffer(abo, GSGPU_POISON, bo->base.resv, &fence);
	if (!WARN_ON(r)) {
		gsgpu_bo_fence(abo, fence, false);
		dma_fence_put(fence);
	}

	dma_resv_unlock(bo->base.resv);
}

/**
 * gsgpu_bo_fault_reserve_notify - notification about a memory fault
 * @bo: pointer to a buffer object
 *
 * Notifies the driver we are taking a fault on this BO and have reserved it,
 * also performs bookkeeping.
 * TTM driver callback for dealing with vm faults.
 *
 * Returns:
 * 0 for success or a negative error code on failure.
 */
vm_fault_t gsgpu_bo_fault_reserve_notify(struct ttm_buffer_object *bo)
{
	struct gsgpu_device *adev = gsgpu_ttm_adev(bo->bdev);
	struct ttm_operation_ctx ctx = { false, false };
	struct gsgpu_bo *abo = ttm_to_gsgpu_bo(bo);
	unsigned long offset;
	int r;

	/* Remember that this BO was accessed by the CPU */
	abo->flags |= GSGPU_GEM_CREATE_CPU_ACCESS_REQUIRED;

	if (bo->resource->mem_type != TTM_PL_VRAM)
		return 0;

	offset = bo->resource->start << PAGE_SHIFT;
	if ((offset + bo->base.size) <= adev->gmc.visible_vram_size)
		return 0;

	/* Can't move a pinned BO to visible VRAM */
	if (abo->tbo.pin_count > 0)
		return VM_FAULT_SIGBUS;

	/* hurrah the memory is not visible ! */
	atomic64_inc(&adev->num_vram_cpu_page_faults);
	gsgpu_bo_placement_from_domain(abo, GSGPU_GEM_DOMAIN_VRAM |
					GSGPU_GEM_DOMAIN_GTT);

	/* Avoid costly evictions; only set GTT as a busy placement */
	abo->placement.num_busy_placement = 1;
	abo->placement.busy_placement = &abo->placements[1];

	r = ttm_bo_validate(bo, &abo->placement, &ctx);
	if (unlikely(r == -EBUSY || r == -ERESTARTSYS))
		return VM_FAULT_NOPAGE;
	else if (unlikely(r))
		return VM_FAULT_SIGBUS;

	offset = bo->resource->start << PAGE_SHIFT;
	/* this should never happen */
	if (bo->resource->mem_type == TTM_PL_VRAM &&
	    (offset + bo->base.size) > adev->gmc.visible_vram_size)
		return VM_FAULT_SIGBUS;

	ttm_bo_move_to_lru_tail_unlocked(bo);
	return 0;
}

/**
 * gsgpu_bo_fence - add fence to buffer object
 *
 * @bo: buffer object in question
 * @fence: fence to add
 * @shared: true if fence should be added shared
 *
 */
void gsgpu_bo_fence(struct gsgpu_bo *bo, struct dma_fence *fence,
		     bool shared)
{
	struct dma_resv *resv = bo->tbo.base.resv;
	int r;

	r = dma_resv_reserve_fences(resv, 1);
	if (r) {
		/* As last resort on OOM we block for the fence */
		dma_fence_wait(fence, false);
		return;
	}

	dma_resv_add_fence(resv, fence, shared ? DMA_RESV_USAGE_READ :
			   DMA_RESV_USAGE_WRITE);
}

/**
 * gsgpu_bo_sync_wait_resv - Wait for BO reservation fences
 *
 * @adev: gsgpu device pointer
 * @resv: reservation object to sync to
 * @sync_mode: synchronization mode
 * @owner: fence owner
 * @intr: Whether the wait is interruptible
 *
 * Extract the fences from the reservation object and waits for them to finish.
 *
 * Returns:
 * 0 on success, errno otherwise.
 */
int gsgpu_bo_sync_wait_resv(struct gsgpu_device *adev, struct dma_resv *resv,
			     enum gsgpu_sync_mode sync_mode, void *owner,
			     bool intr)
{
	struct gsgpu_sync sync;
	int r;

	gsgpu_sync_create(&sync);
	gsgpu_sync_resv(adev, &sync, resv, sync_mode, owner);
	r = gsgpu_sync_wait(&sync, intr);
	gsgpu_sync_free(&sync);
	return r;
}

/**
 * gsgpu_bo_sync_wait - Wrapper for gsgpu_bo_sync_wait_resv
 * @bo: buffer object to wait for
 * @owner: fence owner
 * @intr: Whether the wait is interruptible
 *
 * Wrapper to wait for fences in a BO.
 * Returns:
 * 0 on success, errno otherwise.
 */
int gsgpu_bo_sync_wait(struct gsgpu_bo *bo, void *owner, bool intr)
{
	struct gsgpu_device *adev = gsgpu_ttm_adev(bo->tbo.bdev);

	return gsgpu_bo_sync_wait_resv(adev, bo->tbo.base.resv,
					GSGPU_SYNC_NE_OWNER, owner, intr);
}

/**
 * gsgpu_bo_gpu_offset - return GPU offset of bo
 * @bo:	gsgpu object for which we query the offset
 *
 * Note: object should either be pinned or reserved when calling this
 * function, it might be useful to add check for this for debugging.
 *
 * Returns:
 * current GPU offset of the object.
 */
u64 gsgpu_bo_gpu_offset(struct gsgpu_bo *bo)
{
	WARN_ON_ONCE(bo->tbo.resource->mem_type == TTM_PL_SYSTEM);
	WARN_ON_ONCE(!dma_resv_is_locked(bo->tbo.base.resv) &&
		     !bo->tbo.pin_count && bo->tbo.type != ttm_bo_type_kernel);
	WARN_ON_ONCE(bo->tbo.resource->start == GSGPU_BO_INVALID_OFFSET);
	WARN_ON_ONCE(bo->tbo.resource->mem_type == TTM_PL_VRAM &&
		     !(bo->flags & GSGPU_GEM_CREATE_VRAM_CONTIGUOUS));

	return gsgpu_bo_gpu_offset_no_check(bo);
}

/**
 * gsgpu_bo_gpu_offset_no_check - return GPU offset of bo
 * @bo:	gsgpu object for which we query the offset
 *
 * Returns:
 * current GPU offset of the object without raising warnings.
 */
u64 gsgpu_bo_gpu_offset_no_check(struct gsgpu_bo *bo)
{
	struct gsgpu_device *adev = gsgpu_ttm_adev(bo->tbo.bdev);
	uint64_t offset;

	offset = (bo->tbo.resource->start << PAGE_SHIFT) +
		 gsgpu_ttm_domain_start(adev, bo->tbo.resource->mem_type);

	return gsgpu_gmc_sign_extend(offset);
}

/**
 * gsgpu_bo_get_preferred_domain - get preferred domain
 * @adev: gsgpu device object
 * @domain: allowed :ref:`memory domains <gsgpu_memory_domains>`
 *
 * Returns:
 * Which of the allowed domains is preferred for allocating the BO.
 */
uint32_t gsgpu_bo_get_preferred_domain(struct gsgpu_device *adev,
					    uint32_t domain)
{
	if ((domain == (GSGPU_GEM_DOMAIN_VRAM | GSGPU_GEM_DOMAIN_GTT))) {
		domain = GSGPU_GEM_DOMAIN_VRAM;
		if (adev->gmc.real_vram_size <= GSGPU_SG_THRESHOLD)
			domain = GSGPU_GEM_DOMAIN_GTT;
	}
	return domain;
}

#if defined(CONFIG_DEBUG_FS)
#define gsgpu_bo_print_flag(m, bo, flag)		        \
	do {							\
		if (bo->flags & (GSGPU_GEM_CREATE_ ## flag)) {	\
			seq_printf((m), " " #flag);		\
		}						\
	} while (0)

/**
 * gsgpu_bo_print_info - print BO info in debugfs file
 *
 * @id: Index or Id of the BO
 * @bo: Requested BO for printing info
 * @m: debugfs file
 *
 * Print BO information in debugfs file
 *
 * Returns:
 * Size of the BO in bytes.
 */
u64 gsgpu_bo_print_info(int id, struct gsgpu_bo *bo, struct seq_file *m)
{
	struct dma_buf_attachment *attachment;
	struct dma_buf *dma_buf;
	unsigned int domain;
	const char *placement;
	unsigned int pin_count;
	u64 size;

	domain = gsgpu_mem_type_to_domain(bo->tbo.resource->mem_type);
	switch (domain) {
	case GSGPU_GEM_DOMAIN_VRAM:
		placement = "VRAM";
		break;
	case GSGPU_GEM_DOMAIN_GTT:
		placement = " GTT";
		break;
	case GSGPU_GEM_DOMAIN_CPU:
	default:
		placement = " CPU";
		break;
	}

	size = gsgpu_bo_size(bo);
	seq_printf(m, "\t\t0x%08x: %12lld byte %s",
			id, size, placement);

	pin_count = READ_ONCE(bo->tbo.pin_count);
	if (pin_count)
		seq_printf(m, " pin count %d", pin_count);

	dma_buf = READ_ONCE(bo->tbo.base.dma_buf);
	attachment = READ_ONCE(bo->tbo.base.import_attach);

	if (attachment)
		seq_printf(m, " imported from ino:%lu", file_inode(dma_buf->file)->i_ino);
	else if (dma_buf)
		seq_printf(m, " exported as ino:%lu", file_inode(dma_buf->file)->i_ino);

	gsgpu_bo_print_flag(m, bo, CPU_ACCESS_REQUIRED);
	gsgpu_bo_print_flag(m, bo, NO_CPU_ACCESS);
	gsgpu_bo_print_flag(m, bo, CPU_GTT_USWC);
	gsgpu_bo_print_flag(m, bo, VRAM_CLEARED);
	gsgpu_bo_print_flag(m, bo, VRAM_CONTIGUOUS);
	gsgpu_bo_print_flag(m, bo, VM_ALWAYS_VALID);
	gsgpu_bo_print_flag(m, bo, EXPLICIT_SYNC);

	seq_puts(m, "\n");

	return size;
}
#endif
