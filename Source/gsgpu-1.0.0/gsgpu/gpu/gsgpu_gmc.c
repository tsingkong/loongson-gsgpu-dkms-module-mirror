/*
 * Copyright 2018 Advanced Micro Devices, Inc.
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

#include <linux/io-64-nonatomic-lo-hi.h>
#ifdef CONFIG_X86
#include <asm/hypervisor.h>
#endif

#include "gsgpu.h"
#include "gsgpu_gmc.h"

#include <drm/drm_drv.h>
#include <drm/ttm/ttm_tt.h>

/**
 * gsgpu_gmc_get_pde_for_bo - get the PDE for a BO
 *
 * @bo: the BO to get the PDE for
 * @level: the level in the PD hirarchy
 * @addr: resulting addr
 * @flags: resulting flags
 *
 * Get the address and flags to be used for a PDE (Page Directory Entry).
 */
void gsgpu_gmc_get_pde_for_bo(struct gsgpu_bo *bo, int level,
			       uint64_t *addr, uint64_t *flags)
{
	struct gsgpu_device *adev = gsgpu_ttm_adev(bo->tbo.bdev);

	switch (bo->tbo.resource->mem_type) {
	case TTM_PL_TT:
		*addr = bo->tbo.ttm->dma_address[0];
		break;
	case TTM_PL_VRAM:
		*addr = gsgpu_bo_gpu_offset(bo);
		break;
	default:
		*addr = 0;
		break;
	}
	*flags = gsgpu_ttm_tt_pde_flags(bo->tbo.ttm, bo->tbo.resource);
	gsgpu_gmc_get_vm_pde(adev, level, addr, flags);
}

/*
 * gsgpu_gmc_pd_addr - return the address of the root directory
 */
uint64_t gsgpu_gmc_pd_addr(struct gsgpu_bo *bo)
{
	// struct gsgpu_device *adev = gsgpu_ttm_adev(bo->tbo.bdev);
	uint64_t pd_addr;

	{
		pd_addr = gsgpu_bo_gpu_offset(bo);
	}
	return pd_addr;
}

/**
 * gsgpu_gmc_set_pte_pde - update the page tables using CPU
 *
 * @adev: gsgpu_device pointer
 * @cpu_pt_addr: cpu address of the page table
 * @gpu_page_idx: entry in the page table to update
 * @addr: dst addr to write into pte/pde
 * @flags: access flags
 *
 * Update the page tables using CPU.
 */
int gsgpu_gmc_set_pte_pde(struct gsgpu_device *adev, void *cpu_pt_addr,
				uint32_t gpu_page_idx, uint64_t addr,
				uint64_t flags)
{
	void __iomem *ptr = (void *)cpu_pt_addr;
	uint64_t value;

	/*
	 * The following is for PTE only. GART does not have PDEs.
	*/
	value = addr & 0x0000FFFFFFFFF000ULL;
	value |= flags;
	writeq(value, ptr + (gpu_page_idx * 8));

	return 0;
}

/**
 * gsgpu_gmc_agp_addr - return the address in the AGP address space
 *
 * @bo: TTM BO which needs the address, must be in GTT domain
 *
 * Tries to figure out how to access the BO through the AGP aperture. Returns
 * GSGPU_BO_INVALID_OFFSET if that is not possible.
 */
uint64_t gsgpu_gmc_agp_addr(struct ttm_buffer_object *bo)
{
	struct gsgpu_device *adev = gsgpu_ttm_adev(bo->bdev);

	if (bo->ttm->num_pages != 1 || bo->ttm->caching == ttm_cached)
		return GSGPU_BO_INVALID_OFFSET;

	if (bo->ttm->dma_address[0] + PAGE_SIZE >= adev->gmc.agp_size)
		return GSGPU_BO_INVALID_OFFSET;

	return adev->gmc.agp_start + bo->ttm->dma_address[0];
}

/**
 * gsgpu_gmc_vram_location - try to find VRAM location
 *
 * @adev: gsgpu device structure holding all necessary information
 * @mc: memory controller structure holding memory information
 * @base: base address at which to put VRAM
 *
 * Function will try to place VRAM at base address provided
 * as parameter.
 */
void gsgpu_gmc_vram_location(struct gsgpu_device *adev, struct gsgpu_gmc *mc,
			      u64 base)
{
	uint64_t vis_limit = (uint64_t)gsgpu_vis_vram_limit << 20;
	uint64_t limit = (uint64_t)gsgpu_vram_limit << 20;

	mc->vram_start = base;
	mc->vram_end = mc->vram_start + mc->mc_vram_size - 1;
	if (limit < mc->real_vram_size)
		mc->real_vram_size = limit;

	if (vis_limit && vis_limit < mc->visible_vram_size)
		mc->visible_vram_size = vis_limit;

	if (mc->real_vram_size < mc->visible_vram_size)
		mc->visible_vram_size = mc->real_vram_size;

	{
		mc->fb_start = mc->vram_start;
		mc->fb_end = mc->vram_end;
	}
	dev_info(adev->dev, "VRAM: %lluM 0x%016llX - 0x%016llX (%lluM used)\n",
			mc->mc_vram_size >> 20, mc->vram_start,
			mc->vram_end, mc->real_vram_size >> 20);
}

/**
 * gsgpu_gmc_gart_location - try to find GART location
 *
 * @adev: gsgpu device structure holding all necessary information
 * @mc: memory controller structure holding memory information
 *
 * Function will place try to place GART before or after VRAM.
 * If GART size is bigger than space left then we ajust GART size.
 * Thus function will never fails.
 */
void gsgpu_gmc_gart_location(struct gsgpu_device *adev, struct gsgpu_gmc *mc)
{
	const uint64_t four_gb = 0x100000000ULL;
	u64 size_af, size_bf;
	/*To avoid the hole, limit the max mc address to GSGPU_GMC_HOLE_START*/
	u64 max_mc_address = min(adev->gmc.mc_mask, GSGPU_GMC_HOLE_START - 1);

	/* VCE doesn't like it when BOs cross a 4GB segment, so align
	 * the GART base on a 4GB boundary as well.
	 */
	size_bf = mc->fb_start;
	size_af = max_mc_address + 1 - ALIGN(mc->fb_end + 1, four_gb);

	if (mc->gart_size > max(size_bf, size_af)) {
		dev_warn(adev->dev, "limiting GART\n");
		mc->gart_size = max(size_bf, size_af);
	}

	if ((size_bf >= mc->gart_size && size_bf < size_af) ||
	    (size_af < mc->gart_size))
		mc->gart_start = 0;
	else
		mc->gart_start = max_mc_address - mc->gart_size + 1;

	mc->gart_start &= ~(four_gb - 1);
	mc->gart_end = mc->gart_start + mc->gart_size - 1;
	dev_info(adev->dev, "GART: %lluM 0x%016llX - 0x%016llX\n",
			mc->gart_size >> 20, mc->gart_start, mc->gart_end);
}

/**
 * gsgpu_gmc_agp_location - try to find AGP location
 * @adev: gsgpu device structure holding all necessary information
 * @mc: memory controller structure holding memory information
 *
 * Function will place try to find a place for the AGP BAR in the MC address
 * space.
 *
 * AGP BAR will be assigned the largest available hole in the address space.
 * Should be called after VRAM and GART locations are setup.
 */
void gsgpu_gmc_agp_location(struct gsgpu_device *adev, struct gsgpu_gmc *mc)
{
	const uint64_t sixteen_gb = 1ULL << 34;
	const uint64_t sixteen_gb_mask = ~(sixteen_gb - 1);
	u64 size_af, size_bf;

	if (mc->fb_start > mc->gart_start) {
		size_bf = (mc->fb_start & sixteen_gb_mask) -
			ALIGN(mc->gart_end + 1, sixteen_gb);
		size_af = mc->mc_mask + 1 - ALIGN(mc->fb_end + 1, sixteen_gb);
	} else {
		size_bf = mc->fb_start & sixteen_gb_mask;
		size_af = (mc->gart_start & sixteen_gb_mask) -
			ALIGN(mc->fb_end + 1, sixteen_gb);
	}

	if (size_bf > size_af) {
		mc->agp_start = (mc->fb_start - size_bf) & sixteen_gb_mask;
		mc->agp_size = size_bf;
	} else {
		mc->agp_start = ALIGN(mc->fb_end + 1, sixteen_gb);
		mc->agp_size = size_af;
	}

	mc->agp_end = mc->agp_start + mc->agp_size - 1;
	dev_info(adev->dev, "AGP: %lluM 0x%016llX - 0x%016llX\n",
			mc->agp_size >> 20, mc->agp_start, mc->agp_end);
}

/**
 * gsgpu_gmc_fault_key - get hask key from vm fault address and pasid
 *
 * @addr: 48 bit physical address, page aligned (36 significant bits)
 * @pasid: 16 bit process address space identifier
 */
static inline uint64_t gsgpu_gmc_fault_key(uint64_t addr, uint16_t pasid)
{
	return addr << 4 | pasid;
}

/**
 * gsgpu_gmc_filter_faults - filter VM faults
 *
 * @adev: gsgpu device structure
 * @ih: interrupt ring that the fault received from
 * @addr: address of the VM fault
 * @pasid: PASID of the process causing the fault
 * @timestamp: timestamp of the fault
 *
 * Returns:
 * True if the fault was filtered and should not be processed further.
 * False if the fault is a new one and needs to be handled.
 */
bool gsgpu_gmc_filter_faults(struct gsgpu_device *adev,
			      struct gsgpu_ih_ring *ih, uint64_t addr,
			      uint16_t pasid, uint64_t timestamp)
{
	struct gsgpu_gmc *gmc = &adev->gmc;
	uint64_t stamp, key = gsgpu_gmc_fault_key(addr, pasid);
	struct gsgpu_gmc_fault *fault;
	uint32_t hash;

	/* Stale retry fault if timestamp goes backward */
	if (gsgpu_ih_ts_after(timestamp, ih->processed_timestamp))
		return true;

	/* If we don't have space left in the ring buffer return immediately */
	stamp = max(timestamp, GSGPU_GMC_FAULT_TIMEOUT + 1) -
		GSGPU_GMC_FAULT_TIMEOUT;
	if (gmc->fault_ring[gmc->last_fault].timestamp >= stamp)
		return true;

	/* Try to find the fault in the hash */
	hash = hash_64(key, GSGPU_GMC_FAULT_HASH_ORDER);
	fault = &gmc->fault_ring[gmc->fault_hash[hash].idx];
	while (fault->timestamp >= stamp) {
		uint64_t tmp;

		if (atomic64_read(&fault->key) == key)
			return true;

		tmp = fault->timestamp;
		fault = &gmc->fault_ring[fault->next];

		/* Check if the entry was reused */
		if (fault->timestamp >= tmp)
			break;
	}

	/* Add the fault to the ring */
	fault = &gmc->fault_ring[gmc->last_fault];
	atomic64_set(&fault->key, key);
	fault->timestamp = timestamp;

	/* And update the hash */
	fault->next = gmc->fault_hash[hash].idx;
	gmc->fault_hash[hash].idx = gmc->last_fault++;
	return false;
}

/**
 * gsgpu_gmc_filter_faults_remove - remove address from VM faults filter
 *
 * @adev: gsgpu device structure
 * @addr: address of the VM fault
 * @pasid: PASID of the process causing the fault
 *
 * Remove the address from fault filter, then future vm fault on this address
 * will pass to retry fault handler to recover.
 */
void gsgpu_gmc_filter_faults_remove(struct gsgpu_device *adev, uint64_t addr,
				     uint16_t pasid)
{
	struct gsgpu_gmc *gmc = &adev->gmc;
	uint64_t key = gsgpu_gmc_fault_key(addr, pasid);
	struct gsgpu_gmc_fault *fault;
	uint32_t hash;
	uint64_t tmp;

	hash = hash_64(key, GSGPU_GMC_FAULT_HASH_ORDER);
	fault = &gmc->fault_ring[gmc->fault_hash[hash].idx];
	do {
		if (atomic64_cmpxchg(&fault->key, key, 0) == key)
			break;

		tmp = fault->timestamp;
		fault = &gmc->fault_ring[fault->next];
	} while (fault->timestamp < tmp);
}

	/*
	 * The latest engine allocation on gfx9/10 is:
	 * Engine 2, 3: firmware
	 * Engine 0, 1, 4~16: gsgpu ring,
	 *                    subject to change when ring number changes
	 * Engine 17: Gart flushes
	 */
#define GFXHUB_FREE_VM_INV_ENGS_BITMAP		0x1FFF3
#define MMHUB_FREE_VM_INV_ENGS_BITMAP		0x1FFF3

/**
 * gsgpu_gmc_noretry_set -- set per asic noretry defaults
 * @adev: gsgpu_device pointer
 *
 * Set a per asic default for the no-retry parameter.
 *
 */
void gsgpu_gmc_noretry_set(struct gsgpu_device *adev)
{
	struct gsgpu_gmc *gmc = &adev->gmc;
	bool noretry_default = false;

	gmc->noretry = (gsgpu_noretry == -1) ? noretry_default : gsgpu_noretry;
}

int gsgpu_gmc_vram_checking(struct gsgpu_device *adev)
{
	struct gsgpu_bo *vram_bo = NULL;
	uint64_t vram_gpu = 0;
	void *vram_ptr = NULL;

	int ret, size = 0x100000;
	uint8_t cptr[10];

	ret = gsgpu_bo_create_kernel(adev, size, PAGE_SIZE,
				GSGPU_GEM_DOMAIN_VRAM,
				&vram_bo,
				&vram_gpu,
				&vram_ptr);
	if (ret)
		return ret;

	memset(vram_ptr, 0x86, size);
	memset(cptr, 0x86, 10);

	/**
	 * Check the start, the mid, and the end of the memory if the content of
	 * each byte is the pattern "0x86". If yes, we suppose the vram bo is
	 * workable.
	 *
	 * Note: If check the each byte of whole 1M bo, it will cost too many
	 * seconds, so here, we just pick up three parts for emulation.
	 */
	ret = memcmp(vram_ptr, cptr, 10);
	if (ret)
		return ret;

	ret = memcmp(vram_ptr + (size / 2), cptr, 10);
	if (ret)
		return ret;

	ret = memcmp(vram_ptr + size - 10, cptr, 10);
	if (ret)
		return ret;

	gsgpu_bo_free_kernel(&vram_bo, &vram_gpu,
			&vram_ptr);

	return 0;
}
