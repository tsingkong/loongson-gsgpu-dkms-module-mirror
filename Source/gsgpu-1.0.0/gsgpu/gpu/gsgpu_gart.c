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

#include <linux/pci.h>
#include <linux/vmalloc.h>

#include <drm/gsgpu_drm.h>
#include "gsgpu.h"
#include <drm/drm_drv.h>

/*
 * GART
 * The GART (Graphics Aperture Remapping Table) is an aperture
 * in the GPU's address space.  System pages can be mapped into
 * the aperture and look like contiguous pages from the GPU's
 * perspective.  A page table maps the pages in the aperture
 * to the actual backing pages in system memory.
 *
 * GSGPU GPUs support both an internal GART, as described above,
 * and AGP.  AGP works similarly, but the GART table is configured
 * and maintained by the northbridge rather than the driver.
 * GSGPU hw has a separate AGP aperture that is programmed to
 * point to the AGP aperture provided by the northbridge and the
 * requests are passed through to the northbridge aperture.
 * Both AGP and internal GART can be used at the same time, however
 * that is not currently supported by the driver.
 *
 * This file handles the common internal GART management.
 */

/*
 * Common GART table functions.
 */

/**
 * gsgpu_gart_dummy_page_init - init dummy page used by the driver
 *
 * @adev: gsgpu_device pointer
 *
 * Allocate the dummy page used by the driver (all asics).
 * This dummy page is used by the driver as a filler for gart entries
 * when pages are taken out of the GART
 * Returns 0 on sucess, -ENOMEM on failure.
 */
static int gsgpu_gart_dummy_page_init(struct gsgpu_device *adev)
{
	struct page *dummy_page = ttm_glob.dummy_read_page;
	void *dummy_addr;

	if (adev->dummy_page_addr)
		return 0;

	dummy_addr = page_address(dummy_page);
	memset(dummy_addr, 0xdd, PAGE_SIZE);
	
	adev->dummy_page_addr = dma_map_page(&adev->pdev->dev, dummy_page, 0,
					     PAGE_SIZE, DMA_BIDIRECTIONAL);
	if (dma_mapping_error(&adev->pdev->dev, adev->dummy_page_addr)) {
		dev_err(&adev->pdev->dev, "Failed to DMA MAP the dummy page\n");
		adev->dummy_page_addr = 0;
		return -ENOMEM;
	}
	return 0;
}

/**
 * gsgpu_gart_dummy_page_fini - free dummy page used by the driver
 *
 * @adev: gsgpu_device pointer
 *
 * Frees the dummy page used by the driver (all asics).
 */
void gsgpu_gart_dummy_page_fini(struct gsgpu_device *adev)
{
	if (!adev->dummy_page_addr)
		return;
	dma_unmap_page(&adev->pdev->dev, adev->dummy_page_addr, PAGE_SIZE,
		       DMA_BIDIRECTIONAL);
	adev->dummy_page_addr = 0;
}

/**
 * gsgpu_gart_table_vram_alloc - allocate vram for gart page table
 *
 * @adev: gsgpu_device pointer
 *
 * Allocate video memory for GART page table
 * (pcie r4xx, r5xx+).  These asics require the
 * gart table to be in video memory.
 * Returns 0 for success, error for failure.
 */
int gsgpu_gart_table_vram_alloc(struct gsgpu_device *adev)
{
	if (adev->gart.bo != NULL)
		return 0;

	return gsgpu_bo_create_kernel(adev,  adev->gart.table_size, PAGE_SIZE,
				       GSGPU_GEM_DOMAIN_VRAM, &adev->gart.bo,
				       NULL, (void *)&adev->gart.ptr);
}

/**
 * gsgpu_gart_table_vram_pin - pin gart page table in vram
 *
 * @adev: gsgpu_device pointer
 *
 * Pin the GART page table in vram so it will not be moved
 * by the memory manager (pcie r4xx, r5xx+).  These asics require the
 * gart table to be in video memory.
 * Returns 0 for success, error for failure.
 */
int gsgpu_gart_table_vram_pin(struct gsgpu_device *adev)
{
	int r;

	r = gsgpu_bo_reserve(adev->gart.bo, false);
	if (unlikely(r != 0))
		return r;
	r = gsgpu_bo_pin(adev->gart.bo, GSGPU_GEM_DOMAIN_VRAM);
	if (r) {
		gsgpu_bo_unreserve(adev->gart.bo);
		return r;
	}
	r = gsgpu_bo_kmap(adev->gart.bo, &adev->gart.ptr);
	if (r)
		gsgpu_bo_unpin(adev->gart.bo);
	gsgpu_bo_unreserve(adev->gart.bo);
	// adev->gart.table_addr = gsgpu_bo_gpu_offset(adev->gart.bo);
	return r;
}

/**
 * gsgpu_gart_table_vram_unpin - unpin gart page table in vram
 *
 * @adev: gsgpu_device pointer
 *
 * Unpin the GART page table in vram (pcie r4xx, r5xx+).
 * These asics require the gart table to be in video memory.
 */
void gsgpu_gart_table_vram_unpin(struct gsgpu_device *adev)
{
	int r;

	if (adev->gart.bo == NULL) {
		return;
	}
	r = gsgpu_bo_reserve(adev->gart.bo, true);
	if (likely(r == 0)) {
		gsgpu_bo_kunmap(adev->gart.bo);
		gsgpu_bo_unpin(adev->gart.bo);
		gsgpu_bo_unreserve(adev->gart.bo);
		adev->gart.ptr = NULL;
	}
}

/**
 * gsgpu_gart_table_vram_free - free gart page table vram
 *
 * @adev: gsgpu_device pointer
 *
 * Free the video memory used for the GART page table
 * (pcie r4xx, r5xx+).  These asics require the gart table to
 * be in video memory.
 */
void gsgpu_gart_table_vram_free(struct gsgpu_device *adev)
{
	gsgpu_bo_free_kernel(&adev->gart.bo, NULL, (void *)&adev->gart.ptr);
}

/*
 * Common gart functions.
 */
/**
 * gsgpu_gart_unbind - unbind pages from the gart page table
 *
 * @adev: gsgpu_device pointer
 * @offset: offset into the GPU's gart aperture
 * @pages: number of pages to unbind
 *
 * Unbinds the requested pages from the gart page table and
 * replaces them with the dummy page (all asics).
 * Returns 0 for success, -EINVAL for failure.
 */
void gsgpu_gart_unbind(struct gsgpu_device *adev, uint64_t offset,
			int pages)
{
	unsigned t;
	unsigned p;
	int i, j;
	u64 page_base;
	/* Starting from VEGA10, system bit must be 0 to mean invalid. */
	uint64_t flags = 0;
	int idx;

	if (!adev->gart.ptr)
		return;

	if (!drm_dev_enter(adev_to_drm(adev), &idx))
		return;

	t = offset / GSGPU_GPU_PAGE_SIZE;
	p = t / GSGPU_GPU_PAGES_IN_CPU_PAGE;
	for (i = 0; i < pages; i++, p++) {
		page_base = adev->dummy_page_addr;
		if (!adev->gart.ptr)
			continue;

		for (j = 0; j < GSGPU_GPU_PAGES_IN_CPU_PAGE; j++, t++) {
			gsgpu_gmc_set_pte_pde(adev, adev->gart.ptr,
					       t, page_base, flags);
			page_base += GSGPU_GPU_PAGE_SIZE;
		}
	}
	mb();
	gsgpu_device_flush_hdp(adev, NULL);
	for (i = 0; i < adev->num_vmhubs; i++)
		gsgpu_gmc_flush_gpu_tlb(adev, 0, i, 0);

	drm_dev_exit(idx);
}

/**
 * gsgpu_gart_map - map dma_addresses into GART entries
 *
 * @adev: gsgpu_device pointer
 * @offset: offset into the GPU's gart aperture
 * @pages: number of pages to bind
 * @dma_addr: DMA addresses of pages
 * @flags: page table entry flags
 * @dst: CPU address of the gart table
 *
 * Map the dma_addresses into GART entries (all asics).
 * Returns 0 for success, -EINVAL for failure.
 */
void gsgpu_gart_map(struct gsgpu_device *adev, uint64_t offset,
		    int pages, dma_addr_t *dma_addr, uint64_t flags,
		    void *dst)
{
	uint64_t page_base;
	unsigned i, j, t;
	int idx;

	if (!drm_dev_enter(adev_to_drm(adev), &idx))
		return;

	t = offset / GSGPU_GPU_PAGE_SIZE;

	for (i = 0; i < pages; i++) {
		page_base = dma_addr[i];
		for (j = 0; j < GSGPU_GPU_PAGES_IN_CPU_PAGE; j++, t++) {
			gsgpu_gmc_set_pte_pde(adev, dst, t, page_base, flags);
			page_base += GSGPU_GPU_PAGE_SIZE;
		}
	}
	drm_dev_exit(idx);
}

/**
 * gsgpu_gart_bind - bind pages into the gart page table
 *
 * @adev: gsgpu_device pointer
 * @offset: offset into the GPU's gart aperture
 * @pages: number of pages to bind
 * @dma_addr: DMA addresses of pages
 * @flags: page table entry flags
 *
 * Binds the requested pages to the gart page table
 * (all asics).
 * Returns 0 for success, -EINVAL for failure.
 */
void gsgpu_gart_bind(struct gsgpu_device *adev, uint64_t offset,
		     int pages, dma_addr_t *dma_addr,
		     uint64_t flags)
{
	if (!adev->gart.ptr)
		return;

	gsgpu_gart_map(adev, offset, pages, dma_addr, flags, adev->gart.ptr);
}

/**
 * gsgpu_gart_invalidate_tlb - invalidate gart TLB
 *
 * @adev: gsgpu device driver pointer
 *
 * Invalidate gart TLB which can be use as a way to flush gart changes
 *
 */
void gsgpu_gart_invalidate_tlb(struct gsgpu_device *adev)
{
	int i;

	if (!adev->gart.ptr)
		return;

	mb();
	gsgpu_device_flush_hdp(adev, NULL);
	for (i = 0; i < adev->num_vmhubs; i++)
		gsgpu_gmc_flush_gpu_tlb(adev, 0, i, 0);
}

/**
 * gsgpu_gart_init - init the driver info for managing the gart
 *
 * @adev: gsgpu_device pointer
 *
 * Allocate the dummy page and init the gart driver info (all asics).
 * Returns 0 for success, error for failure.
 */
int gsgpu_gart_init(struct gsgpu_device *adev)
{
	int r;

	if (adev->dummy_page_addr)
		return 0;

	/* We need PAGE_SIZE >= GSGPU_GPU_PAGE_SIZE */
	if (PAGE_SIZE < GSGPU_GPU_PAGE_SIZE) {
		DRM_ERROR("Page size is smaller than GPU page size!\n");
		return -EINVAL;
	}
	r = gsgpu_gart_dummy_page_init(adev);
	if (r)
		return r;
	/* Compute table size */
	adev->gart.num_cpu_pages = adev->gmc.gart_size / PAGE_SIZE;
	adev->gart.num_gpu_pages = adev->gmc.gart_size / GSGPU_GPU_PAGE_SIZE;
	DRM_INFO("GART: num cpu pages %u, num gpu pages %u\n",
		 adev->gart.num_cpu_pages, adev->gart.num_gpu_pages);

	return 0;
}
