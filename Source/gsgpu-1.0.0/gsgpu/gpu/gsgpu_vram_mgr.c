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
 * Authors: Christian König
 */

#include <linux/dma-mapping.h>
#include <drm/ttm/ttm_range_manager.h>

#include "gsgpu.h"
#include "gsgpu_vm.h"
#include "gsgpu_res_cursor.h"
#include "atombios.h"

struct gsgpu_vram_reservation {
	u64 start;
	u64 size;
	struct list_head allocated;
	struct list_head blocks;
};

static inline struct gsgpu_vram_mgr *
to_vram_mgr(struct ttm_resource_manager *man)
{
	return container_of(man, struct gsgpu_vram_mgr, manager);
}

static inline struct gsgpu_device *
to_gsgpu_device(struct gsgpu_vram_mgr *mgr)
{
	return container_of(mgr, struct gsgpu_device, mman.vram_mgr);
}

static inline struct drm_buddy_block *
gsgpu_vram_mgr_first_block(struct list_head *list)
{
	return list_first_entry_or_null(list, struct drm_buddy_block, link);
}

static inline bool gsgpu_is_vram_mgr_blocks_contiguous(struct list_head *head)
{
	struct drm_buddy_block *block;
	u64 start, size;

	block = gsgpu_vram_mgr_first_block(head);
	if (!block)
		return false;

	while (head != block->link.next) {
		start = gsgpu_vram_mgr_block_start(block);
		size = gsgpu_vram_mgr_block_size(block);

		block = list_entry(block->link.next, struct drm_buddy_block, link);
		if (start + size != gsgpu_vram_mgr_block_start(block))
			return false;
	}

	return true;
}



/**
 * DOC: mem_info_vram_total
 *
 * The gsgpu driver provides a sysfs API for reporting current total VRAM
 * available on the device
 * The file mem_info_vram_total is used for this and returns the total
 * amount of VRAM in bytes
 */
static ssize_t gsgpu_mem_info_vram_total_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_device *ddev = dev_get_drvdata(dev);
	struct gsgpu_device *adev = drm_to_adev(ddev);

	return sysfs_emit(buf, "%llu\n", adev->gmc.real_vram_size);
}

/**
 * DOC: mem_info_vis_vram_total
 *
 * The gsgpu driver provides a sysfs API for reporting current total
 * visible VRAM available on the device
 * The file mem_info_vis_vram_total is used for this and returns the total
 * amount of visible VRAM in bytes
 */
static ssize_t gsgpu_mem_info_vis_vram_total_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_device *ddev = dev_get_drvdata(dev);
	struct gsgpu_device *adev = drm_to_adev(ddev);

	return sysfs_emit(buf, "%llu\n", adev->gmc.visible_vram_size);
}

/**
 * DOC: mem_info_vram_used
 *
 * The gsgpu driver provides a sysfs API for reporting current total VRAM
 * available on the device
 * The file mem_info_vram_used is used for this and returns the total
 * amount of currently used VRAM in bytes
 */
static ssize_t gsgpu_mem_info_vram_used_show(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	struct drm_device *ddev = dev_get_drvdata(dev);
	struct gsgpu_device *adev = drm_to_adev(ddev);
	struct ttm_resource_manager *man = &adev->mman.vram_mgr.manager;

	return sysfs_emit(buf, "%llu\n", ttm_resource_manager_usage(man));
}

/**
 * DOC: mem_info_vis_vram_used
 *
 * The gsgpu driver provides a sysfs API for reporting current total of
 * used visible VRAM
 * The file mem_info_vis_vram_used is used for this and returns the total
 * amount of currently used visible VRAM in bytes
 */
static ssize_t gsgpu_mem_info_vis_vram_used_show(struct device *dev,
						  struct device_attribute *attr,
						  char *buf)
{
	struct drm_device *ddev = dev_get_drvdata(dev);
	struct gsgpu_device *adev = drm_to_adev(ddev);

	return sysfs_emit(buf, "%llu\n",
			  gsgpu_vram_mgr_vis_usage(&adev->mman.vram_mgr));
}

/**
 * DOC: mem_info_vram_vendor
 *
 * The gsgpu driver provides a sysfs API for reporting the vendor of the
 * installed VRAM
 * The file mem_info_vram_vendor is used for this and returns the name of the
 * vendor.
 */
static ssize_t gsgpu_mem_info_vram_vendor(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct drm_device *ddev = dev_get_drvdata(dev);
	struct gsgpu_device *adev = drm_to_adev(ddev);

	switch (adev->gmc.vram_vendor) {
	case SAMSUNG:
		return sysfs_emit(buf, "samsung\n");
	case INFINEON:
		return sysfs_emit(buf, "infineon\n");
	case ELPIDA:
		return sysfs_emit(buf, "elpida\n");
	case ETRON:
		return sysfs_emit(buf, "etron\n");
	case NANYA:
		return sysfs_emit(buf, "nanya\n");
	case HYNIX:
		return sysfs_emit(buf, "hynix\n");
	case MOSEL:
		return sysfs_emit(buf, "mosel\n");
	case WINBOND:
		return sysfs_emit(buf, "winbond\n");
	case ESMT:
		return sysfs_emit(buf, "esmt\n");
	case MICRON:
		return sysfs_emit(buf, "micron\n");
	default:
		return sysfs_emit(buf, "unknown\n");
	}
}

static DEVICE_ATTR(mem_info_vram_total, S_IRUGO,
		   gsgpu_mem_info_vram_total_show, NULL);
static DEVICE_ATTR(mem_info_vis_vram_total, S_IRUGO,
		   gsgpu_mem_info_vis_vram_total_show,NULL);
static DEVICE_ATTR(mem_info_vram_used, S_IRUGO,
		   gsgpu_mem_info_vram_used_show, NULL);
static DEVICE_ATTR(mem_info_vis_vram_used, S_IRUGO,
		   gsgpu_mem_info_vis_vram_used_show, NULL);
static DEVICE_ATTR(mem_info_vram_vendor, S_IRUGO,
		   gsgpu_mem_info_vram_vendor, NULL);

static struct attribute *gsgpu_vram_mgr_attributes[] = {
	&dev_attr_mem_info_vram_total.attr,
	&dev_attr_mem_info_vis_vram_total.attr,
	&dev_attr_mem_info_vram_used.attr,
	&dev_attr_mem_info_vis_vram_used.attr,
	&dev_attr_mem_info_vram_vendor.attr,
	NULL
};

const struct attribute_group gsgpu_vram_mgr_attr_group = {
	.attrs = gsgpu_vram_mgr_attributes
};

/**
 * gsgpu_vram_mgr_vis_size - Calculate visible block size
 *
 * @adev: gsgpu_device pointer
 * @block: DRM BUDDY block structure
 *
 * Calculate how many bytes of the DRM BUDDY block are inside visible VRAM
 */
static u64 gsgpu_vram_mgr_vis_size(struct gsgpu_device *adev,
				    struct drm_buddy_block *block)
{
	u64 start = gsgpu_vram_mgr_block_start(block);
	u64 end = start + gsgpu_vram_mgr_block_size(block);

	if (start >= adev->gmc.visible_vram_size)
		return 0;

	return (end > adev->gmc.visible_vram_size ?
		adev->gmc.visible_vram_size : end) - start;
}

/**
 * gsgpu_vram_mgr_bo_visible_size - CPU visible BO size
 *
 * @bo: &gsgpu_bo buffer object (must be in VRAM)
 *
 * Returns:
 * How much of the given &gsgpu_bo buffer object lies in CPU visible VRAM.
 */
u64 gsgpu_vram_mgr_bo_visible_size(struct gsgpu_bo *bo)
{
	struct gsgpu_device *adev = gsgpu_ttm_adev(bo->tbo.bdev);
	struct ttm_resource *res = bo->tbo.resource;
	struct gsgpu_vram_mgr_resource *vres = to_gsgpu_vram_mgr_resource(res);
	struct drm_buddy_block *block;
	u64 usage = 0;

	if (gsgpu_gmc_vram_full_visible(&adev->gmc))
		return gsgpu_bo_size(bo);

	if (res->start >= adev->gmc.visible_vram_size >> PAGE_SHIFT)
		return 0;

	list_for_each_entry(block, &vres->blocks, link)
		usage += gsgpu_vram_mgr_vis_size(adev, block);

	return usage;
}

/* Commit the reservation of VRAM pages */
static void gsgpu_vram_mgr_do_reserve(struct ttm_resource_manager *man)
{
	struct gsgpu_vram_mgr *mgr = to_vram_mgr(man);
	struct gsgpu_device *adev = to_gsgpu_device(mgr);
	struct drm_buddy *mm = &mgr->mm;
	struct gsgpu_vram_reservation *rsv, *temp;
	struct drm_buddy_block *block;
	uint64_t vis_usage;

	list_for_each_entry_safe(rsv, temp, &mgr->reservations_pending, blocks) {
		if (drm_buddy_alloc_blocks(mm, rsv->start, rsv->start + rsv->size,
					   rsv->size, mm->chunk_size, &rsv->allocated,
					   DRM_BUDDY_RANGE_ALLOCATION))
			continue;

		block = gsgpu_vram_mgr_first_block(&rsv->allocated);
		if (!block)
			continue;

		dev_dbg(adev->dev, "Reservation 0x%llx - %lld, Succeeded\n",
			rsv->start, rsv->size);

		vis_usage = gsgpu_vram_mgr_vis_size(adev, block);
		atomic64_add(vis_usage, &mgr->vis_usage);
		spin_lock(&man->bdev->lru_lock);
		man->usage += rsv->size;
		spin_unlock(&man->bdev->lru_lock);
		list_move(&rsv->blocks, &mgr->reserved_pages);
	}
}

/**
 * gsgpu_vram_mgr_reserve_range - Reserve a range from VRAM
 *
 * @mgr: gsgpu_vram_mgr pointer
 * @start: start address of the range in VRAM
 * @size: size of the range
 *
 * Reserve memory from start address with the specified size in VRAM
 */
int gsgpu_vram_mgr_reserve_range(struct gsgpu_vram_mgr *mgr,
				  uint64_t start, uint64_t size)
{
	struct gsgpu_vram_reservation *rsv;

	rsv = kzalloc(sizeof(*rsv), GFP_KERNEL);
	if (!rsv)
		return -ENOMEM;

	INIT_LIST_HEAD(&rsv->allocated);
	INIT_LIST_HEAD(&rsv->blocks);

	rsv->start = start;
	rsv->size = size;

	mutex_lock(&mgr->lock);
	list_add_tail(&rsv->blocks, &mgr->reservations_pending);
	gsgpu_vram_mgr_do_reserve(&mgr->manager);
	mutex_unlock(&mgr->lock);

	return 0;
}

/**
 * gsgpu_vram_mgr_query_page_status - query the reservation status
 *
 * @mgr: gsgpu_vram_mgr pointer
 * @start: start address of a page in VRAM
 *
 * Returns:
 *	-EBUSY: the page is still hold and in pending list
 *	0: the page has been reserved
 *	-ENOENT: the input page is not a reservation
 */
int gsgpu_vram_mgr_query_page_status(struct gsgpu_vram_mgr *mgr,
				      uint64_t start)
{
	struct gsgpu_vram_reservation *rsv;
	int ret;

	mutex_lock(&mgr->lock);

	list_for_each_entry(rsv, &mgr->reservations_pending, blocks) {
		if (rsv->start <= start &&
		    (start < (rsv->start + rsv->size))) {
			ret = -EBUSY;
			goto out;
		}
	}

	list_for_each_entry(rsv, &mgr->reserved_pages, blocks) {
		if (rsv->start <= start &&
		    (start < (rsv->start + rsv->size))) {
			ret = 0;
			goto out;
		}
	}

	ret = -ENOENT;
out:
	mutex_unlock(&mgr->lock);
	return ret;
}

/**
 * gsgpu_vram_mgr_new - allocate new ranges
 *
 * @man: TTM memory type manager
 * @tbo: TTM BO we need this range for
 * @place: placement flags and restrictions
 * @res: the resulting mem object
 *
 * Allocate VRAM for the given BO.
 */
static int gsgpu_vram_mgr_new(struct ttm_resource_manager *man,
			       struct ttm_buffer_object *tbo,
			       const struct ttm_place *place,
			       struct ttm_resource **res)
{
	u64 vis_usage = 0, max_bytes, cur_size, min_block_size;
	struct gsgpu_vram_mgr *mgr = to_vram_mgr(man);
	struct gsgpu_device *adev = to_gsgpu_device(mgr);
	struct gsgpu_vram_mgr_resource *vres;
	u64 size, remaining_size, lpfn, fpfn;
	struct drm_buddy *mm = &mgr->mm;
	struct drm_buddy_block *block;
	unsigned long pages_per_block;
	int r;

	lpfn = (u64)place->lpfn << PAGE_SHIFT;
	if (!lpfn)
		lpfn = man->size;

	fpfn = (u64)place->fpfn << PAGE_SHIFT;

	max_bytes = adev->gmc.mc_vram_size;
	if (tbo->type != ttm_bo_type_kernel)
		max_bytes -= GSGPU_VM_RESERVED_VRAM;

	if (place->flags & TTM_PL_FLAG_CONTIGUOUS) {
		pages_per_block = ~0ul;
	} else {
#ifdef CONFIG_TRANSPARENT_HUGEPAGE
		pages_per_block = HPAGE_PMD_NR;
#else
		/* default to 2MB */
		pages_per_block = 2UL << (20UL - PAGE_SHIFT);
#endif
		pages_per_block = max_t(uint32_t, pages_per_block,
					tbo->page_alignment);
	}

	vres = kzalloc(sizeof(*vres), GFP_KERNEL);
	if (!vres)
		return -ENOMEM;

	ttm_resource_init(tbo, place, &vres->base);

	/* bail out quickly if there's likely not enough VRAM for this BO */
	if (ttm_resource_manager_usage(man) > max_bytes) {
		r = -ENOSPC;
		goto error_fini;
	}

	INIT_LIST_HEAD(&vres->blocks);

	if (place->flags & TTM_PL_FLAG_TOPDOWN)
		vres->flags |= DRM_BUDDY_TOPDOWN_ALLOCATION;

	if (fpfn || lpfn != mgr->mm.size)
		/* Allocate blocks in desired range */
		vres->flags |= DRM_BUDDY_RANGE_ALLOCATION;

	remaining_size = (u64)vres->base.size;

	mutex_lock(&mgr->lock);
	while (remaining_size) {
		if (tbo->page_alignment)
			min_block_size = (u64)tbo->page_alignment << PAGE_SHIFT;
		else
			min_block_size = mgr->default_page_size;

		BUG_ON(min_block_size < mm->chunk_size);

		/* Limit maximum size to 2GiB due to SG table limitations */
		size = min(remaining_size, 2ULL << 30);

		if ((size >= (u64)pages_per_block << PAGE_SHIFT) &&
				!(size & (((u64)pages_per_block << PAGE_SHIFT) - 1)))
			min_block_size = (u64)pages_per_block << PAGE_SHIFT;

		cur_size = size;

		if (fpfn + size != (u64)place->lpfn << PAGE_SHIFT) {
			/*
			 * Except for actual range allocation, modify the size and
			 * min_block_size conforming to continuous flag enablement
			 */
			if (place->flags & TTM_PL_FLAG_CONTIGUOUS) {
				size = roundup_pow_of_two(size);
				min_block_size = size;
			/*
			 * Modify the size value if size is not
			 * aligned with min_block_size
			 */
			} else if (!IS_ALIGNED(size, min_block_size)) {
				size = round_up(size, min_block_size);
			}
		}

		r = drm_buddy_alloc_blocks(mm, fpfn,
					   lpfn,
					   size,
					   min_block_size,
					   &vres->blocks,
					   vres->flags);
		if (unlikely(r))
			goto error_free_blocks;

		if (size > remaining_size)
			remaining_size = 0;
		else
			remaining_size -= size;
	}
	mutex_unlock(&mgr->lock);

	if (cur_size != size) {
		struct drm_buddy_block *block;
		struct list_head *trim_list;
		u64 original_size;
		LIST_HEAD(temp);

		trim_list = &vres->blocks;
		original_size = (u64)vres->base.size;

		/*
		 * If size value is rounded up to min_block_size, trim the last
		 * block to the required size
		 */
		if (!list_is_singular(&vres->blocks)) {
			block = list_last_entry(&vres->blocks, typeof(*block), link);
			list_move_tail(&block->link, &temp);
			trim_list = &temp;
			/*
			 * Compute the original_size value by subtracting the
			 * last block size with (aligned size - original size)
			 */
			original_size = gsgpu_vram_mgr_block_size(block) - (size - cur_size);
		}

		mutex_lock(&mgr->lock);
		drm_buddy_block_trim(mm,
				     original_size,
				     trim_list);
		mutex_unlock(&mgr->lock);

		if (!list_empty(&temp))
			list_splice_tail(trim_list, &vres->blocks);
	}

	vres->base.start = 0;
	list_for_each_entry(block, &vres->blocks, link) {
		unsigned long start;

		start = gsgpu_vram_mgr_block_start(block) +
			gsgpu_vram_mgr_block_size(block);
		start >>= PAGE_SHIFT;

		if (start > PFN_UP(vres->base.size))
			start -= PFN_UP(vres->base.size);
		else
			start = 0;
		vres->base.start = max(vres->base.start, start);

		vis_usage += gsgpu_vram_mgr_vis_size(adev, block);
	}

	if (gsgpu_is_vram_mgr_blocks_contiguous(&vres->blocks)) {
		vres->base.placement |= TTM_PL_FLAG_CONTIGUOUS;
	}

		vres->base.bus.caching = ttm_write_combined;

	atomic64_add(vis_usage, &mgr->vis_usage);
	*res = &vres->base;
	return 0;

error_free_blocks:
	drm_buddy_free_list(mm, &vres->blocks);
	mutex_unlock(&mgr->lock);
error_fini:
	ttm_resource_fini(man, &vres->base);
	kfree(vres);

	return r;
}

/**
 * gsgpu_vram_mgr_del - free ranges
 *
 * @man: TTM memory type manager
 * @res: TTM memory object
 *
 * Free the allocated VRAM again.
 */
static void gsgpu_vram_mgr_del(struct ttm_resource_manager *man,
				struct ttm_resource *res)
{
	struct gsgpu_vram_mgr_resource *vres = to_gsgpu_vram_mgr_resource(res);
	struct gsgpu_vram_mgr *mgr = to_vram_mgr(man);
	struct gsgpu_device *adev = to_gsgpu_device(mgr);
	struct drm_buddy *mm = &mgr->mm;
	struct drm_buddy_block *block;
	uint64_t vis_usage = 0;

	mutex_lock(&mgr->lock);
	list_for_each_entry(block, &vres->blocks, link)
		vis_usage += gsgpu_vram_mgr_vis_size(adev, block);

	gsgpu_vram_mgr_do_reserve(man);

	drm_buddy_free_list(mm, &vres->blocks);
	mutex_unlock(&mgr->lock);

	atomic64_sub(vis_usage, &mgr->vis_usage);

	ttm_resource_fini(man, res);
	kfree(vres);
}

/**
 * gsgpu_vram_mgr_alloc_sgt - allocate and fill a sg table
 *
 * @adev: gsgpu device pointer
 * @res: TTM memory object
 * @offset: byte offset from the base of VRAM BO
 * @length: number of bytes to export in sg_table
 * @dev: the other device
 * @dir: dma direction
 * @sgt: resulting sg table
 *
 * Allocate and fill a sg table from a VRAM allocation.
 */
int gsgpu_vram_mgr_alloc_sgt(struct gsgpu_device *adev,
			      struct ttm_resource *res,
			      u64 offset, u64 length,
			      struct device *dev,
			      enum dma_data_direction dir,
			      struct sg_table **sgt)
{
	struct gsgpu_res_cursor cursor;
	struct scatterlist *sg;
	int num_entries = 0;
	int i, r;

	*sgt = kmalloc(sizeof(**sgt), GFP_KERNEL);
	if (!*sgt)
		return -ENOMEM;

	/* Determine the number of DRM_BUDDY blocks to export */
	gsgpu_res_first(res, offset, length, &cursor);
	while (cursor.remaining) {
		num_entries++;
		gsgpu_res_next(&cursor, cursor.size);
	}

	r = sg_alloc_table(*sgt, num_entries, GFP_KERNEL);
	if (r)
		goto error_free;

	/* Initialize scatterlist nodes of sg_table */
	for_each_sgtable_sg((*sgt), sg, i)
		sg->length = 0;

	/*
	 * Walk down DRM_BUDDY blocks to populate scatterlist nodes
	 * @note: Use iterator api to get first the DRM_BUDDY block
	 * and the number of bytes from it. Access the following
	 * DRM_BUDDY block(s) if more buffer needs to exported
	 */
	gsgpu_res_first(res, offset, length, &cursor);
	for_each_sgtable_sg((*sgt), sg, i) {
		phys_addr_t phys = cursor.start + adev->gmc.aper_base;
		size_t size = cursor.size;
		dma_addr_t addr;

		addr = dma_map_resource(dev, phys, size, dir,
					DMA_ATTR_SKIP_CPU_SYNC);
		r = dma_mapping_error(dev, addr);
		if (r)
			goto error_unmap;

		sg_set_page(sg, NULL, size, 0);
		sg_dma_address(sg) = addr;
		sg_dma_len(sg) = size;

		gsgpu_res_next(&cursor, cursor.size);
	}

	return 0;

error_unmap:
	for_each_sgtable_sg((*sgt), sg, i) {
		if (!sg->length)
			continue;

		dma_unmap_resource(dev, sg->dma_address,
				   sg->length, dir,
				   DMA_ATTR_SKIP_CPU_SYNC);
	}
	sg_free_table(*sgt);

error_free:
	kfree(*sgt);
	return r;
}

/**
 * gsgpu_vram_mgr_free_sgt - allocate and fill a sg table
 *
 * @dev: device pointer
 * @dir: data direction of resource to unmap
 * @sgt: sg table to free
 *
 * Free a previously allocate sg table.
 */
void gsgpu_vram_mgr_free_sgt(struct device *dev,
			      enum dma_data_direction dir,
			      struct sg_table *sgt)
{
	struct scatterlist *sg;
	int i;

	for_each_sgtable_sg(sgt, sg, i)
		dma_unmap_resource(dev, sg->dma_address,
				   sg->length, dir,
				   DMA_ATTR_SKIP_CPU_SYNC);
	sg_free_table(sgt);
	kfree(sgt);
}

/**
 * gsgpu_vram_mgr_vis_usage - how many bytes are used in the visible part
 *
 * @mgr: gsgpu_vram_mgr pointer
 *
 * Returns how many bytes are used in the visible part of VRAM
 */
uint64_t gsgpu_vram_mgr_vis_usage(struct gsgpu_vram_mgr *mgr)
{
	return atomic64_read(&mgr->vis_usage);
}

/**
 * gsgpu_vram_mgr_intersects - test each drm buddy block for intersection
 *
 * @man: TTM memory type manager
 * @res: The resource to test
 * @place: The place to test against
 * @size: Size of the new allocation
 *
 * Test each drm buddy block for intersection for eviction decision.
 */
static bool gsgpu_vram_mgr_intersects(struct ttm_resource_manager *man,
				       struct ttm_resource *res,
				       const struct ttm_place *place,
				       size_t size)
{
	struct gsgpu_vram_mgr_resource *mgr = to_gsgpu_vram_mgr_resource(res);
	struct drm_buddy_block *block;

	/* Check each drm buddy block individually */
	list_for_each_entry(block, &mgr->blocks, link) {
		unsigned long fpfn =
			gsgpu_vram_mgr_block_start(block) >> PAGE_SHIFT;
		unsigned long lpfn = fpfn +
			(gsgpu_vram_mgr_block_size(block) >> PAGE_SHIFT);

		if (place->fpfn < lpfn &&
		    (!place->lpfn || place->lpfn > fpfn))
			return true;
	}

	return false;
}

/**
 * gsgpu_vram_mgr_compatible - test each drm buddy block for compatibility
 *
 * @man: TTM memory type manager
 * @res: The resource to test
 * @place: The place to test against
 * @size: Size of the new allocation
 *
 * Test each drm buddy block for placement compatibility.
 */
static bool gsgpu_vram_mgr_compatible(struct ttm_resource_manager *man,
				       struct ttm_resource *res,
				       const struct ttm_place *place,
				       size_t size)
{
	struct gsgpu_vram_mgr_resource *mgr = to_gsgpu_vram_mgr_resource(res);
	struct drm_buddy_block *block;

	/* Check each drm buddy block individually */
	list_for_each_entry(block, &mgr->blocks, link) {
		unsigned long fpfn =
			gsgpu_vram_mgr_block_start(block) >> PAGE_SHIFT;
		unsigned long lpfn = fpfn +
			(gsgpu_vram_mgr_block_size(block) >> PAGE_SHIFT);

		if (fpfn < place->fpfn ||
		    (place->lpfn && lpfn > place->lpfn))
			return false;
	}

	return true;
}

/**
 * gsgpu_vram_mgr_debug - dump VRAM table
 *
 * @man: TTM memory type manager
 * @printer: DRM printer to use
 *
 * Dump the table content using printk.
 */
static void gsgpu_vram_mgr_debug(struct ttm_resource_manager *man,
				  struct drm_printer *printer)
{
	struct gsgpu_vram_mgr *mgr = to_vram_mgr(man);
	struct drm_buddy *mm = &mgr->mm;
	struct drm_buddy_block *block;

	drm_printf(printer, "  vis usage:%llu\n",
		   gsgpu_vram_mgr_vis_usage(mgr));

	mutex_lock(&mgr->lock);
	drm_printf(printer, "default_page_size: %lluKiB\n",
		   mgr->default_page_size >> 10);

	drm_buddy_print(mm, printer);

	drm_printf(printer, "reserved:\n");
	list_for_each_entry(block, &mgr->reserved_pages, link)
		drm_buddy_block_print(mm, block, printer);
	mutex_unlock(&mgr->lock);
}

static const struct ttm_resource_manager_func gsgpu_vram_mgr_func = {
	.alloc	= gsgpu_vram_mgr_new,
	.free	= gsgpu_vram_mgr_del,
	.intersects = gsgpu_vram_mgr_intersects,
	.compatible = gsgpu_vram_mgr_compatible,
	.debug	= gsgpu_vram_mgr_debug
};

/**
 * gsgpu_vram_mgr_init - init VRAM manager and DRM MM
 *
 * @adev: gsgpu_device pointer
 *
 * Allocate and initialize the VRAM manager.
 */
int gsgpu_vram_mgr_init(struct gsgpu_device *adev)
{
	struct gsgpu_vram_mgr *mgr = &adev->mman.vram_mgr;
	struct ttm_resource_manager *man = &mgr->manager;
	int err;

	ttm_resource_manager_init(man, &adev->mman.bdev,
				  adev->gmc.real_vram_size);

	man->func = &gsgpu_vram_mgr_func;

	err = drm_buddy_init(&mgr->mm, man->size, PAGE_SIZE);
	if (err)
		return err;

	mutex_init(&mgr->lock);
	INIT_LIST_HEAD(&mgr->reservations_pending);
	INIT_LIST_HEAD(&mgr->reserved_pages);
	mgr->default_page_size = PAGE_SIZE;

	ttm_set_driver_manager(&adev->mman.bdev, TTM_PL_VRAM, &mgr->manager);
	ttm_resource_manager_set_used(man, true);
	return 0;
}

/**
 * gsgpu_vram_mgr_fini - free and destroy VRAM manager
 *
 * @adev: gsgpu_device pointer
 *
 * Destroy and free the VRAM manager, returns -EBUSY if ranges are still
 * allocated inside it.
 */
void gsgpu_vram_mgr_fini(struct gsgpu_device *adev)
{
	struct gsgpu_vram_mgr *mgr = &adev->mman.vram_mgr;
	struct ttm_resource_manager *man = &mgr->manager;
	int ret;
	struct gsgpu_vram_reservation *rsv, *temp;

	ttm_resource_manager_set_used(man, false);

	ret = ttm_resource_manager_evict_all(&adev->mman.bdev, man);
	if (ret)
		return;

	mutex_lock(&mgr->lock);
	list_for_each_entry_safe(rsv, temp, &mgr->reservations_pending, blocks)
		kfree(rsv);

	list_for_each_entry_safe(rsv, temp, &mgr->reserved_pages, blocks) {
		drm_buddy_free_list(&mgr->mm, &rsv->allocated);
		kfree(rsv);
	}
	drm_buddy_fini(&mgr->mm);
	mutex_unlock(&mgr->lock);

	ttm_resource_manager_cleanup(man);
	ttm_set_driver_manager(&adev->mman.bdev, TTM_PL_VRAM, NULL);
}
