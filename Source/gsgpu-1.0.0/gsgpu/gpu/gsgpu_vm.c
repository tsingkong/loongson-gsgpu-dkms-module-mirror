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

#include <linux/dma-fence-array.h>
#include <linux/interval_tree_generic.h>
#include <linux/idr.h>
#include <linux/dma-buf.h>

#include <drm/gsgpu_drm.h>
#include <drm/drm_drv.h>
#include <drm/ttm/ttm_tt.h>
#include "gsgpu.h"
#include "gsgpu_trace.h"
#include "gsgpu_gmc.h"
#include "gsgpu_dma_buf.h"
#include "gsgpu_res_cursor.h"
#include "gsgpu_vm_it.h"

/**
 * DOC: GPUVM
 *
 * GPUVM is the MMU functionality provided on the GPU.
 * GPUVM is similar to the legacy GART on older asics, however
 * rather than there being a single global GART table
 * for the entire GPU, there can be multiple GPUVM page tables active
 * at any given time.  The GPUVM page tables can contain a mix
 * VRAM pages and system pages (both memory and MMIO) and system pages
 * can be mapped as snooped (cached system pages) or unsnooped
 * (uncached system pages).
 *
 * Each active GPUVM has an ID associated with it and there is a page table
 * linked with each VMID.  When executing a command buffer,
 * the kernel tells the engine what VMID to use for that command
 * buffer.  VMIDs are allocated dynamically as commands are submitted.
 * The userspace drivers maintain their own address space and the kernel
 * sets up their pages tables accordingly when they submit their
 * command buffers and a VMID is assigned.
 * The hardware supports up to 16 active GPUVMs at any given time.
 *
 * Each GPUVM is represented by a 1-2 or 1-5 level page table, depending
 * on the ASIC family.  GPUVM supports RWX attributes on each page as well
 * as other features such as encryption and caching attributes.
 *
 * VMID 0 is special.  It is the GPUVM used for the kernel driver.  In
 * addition to an aperture managed by a page table, VMID 0 also has
 * several other apertures.  There is an aperture for direct access to VRAM
 * and there is a legacy AGP aperture which just forwards accesses directly
 * to the matching system physical addresses (or IOVAs when an IOMMU is
 * present).  These apertures provide direct access to these memories without
 * incurring the overhead of a page table.  VMID 0 is used by the kernel
 * driver for tasks like memory management.
 *
 * GPU clients (i.e., engines on the GPU) use GPUVM VMIDs to access memory.
 * For user applications, each application can have their own unique GPUVM
 * address space.  The application manages the address space and the kernel
 * driver manages the GPUVM page tables for each process.  If an GPU client
 * accesses an invalid page, it will generate a GPU page fault, similar to
 * accessing an invalid page on a CPU.
 */

/**
 * struct gsgpu_prt_cb - Helper to disable partial resident texture feature from a fence callback
 */
struct gsgpu_prt_cb {

	/**
	 * @adev: gsgpu device
	 */
	struct gsgpu_device *adev;

	/**
	 * @cb: callback
	 */
	struct dma_fence_cb cb;
};

/**
 * struct gsgpu_vm_tlb_seq_cb - Helper to increment the TLB flush sequence
 */
struct gsgpu_vm_tlb_seq_cb {
	/**
	 * @vm: pointer to the gsgpu_vm structure to set the fence sequence on
	 */
	struct gsgpu_vm *vm;

	/**
	 * @cb: callback
	 */
	struct dma_fence_cb cb;
};

/**
 * gsgpu_vm_set_pasid - manage pasid and vm ptr mapping
 *
 * @adev: gsgpu_device pointer
 * @vm: gsgpu_vm pointer
 * @pasid: the pasid the VM is using on this GPU
 *
 * Set the pasid this VM is using on this GPU, can also be used to remove the
 * pasid by passing in zero.
 *
 */
int gsgpu_vm_set_pasid(struct gsgpu_device *adev, struct gsgpu_vm *vm,
			u32 pasid)
{
	int r;

	if (vm->pasid == pasid)
		return 0;

	if (vm->pasid) {
		r = xa_err(xa_erase_irq(&adev->vm_manager.pasids, vm->pasid));
		if (r < 0)
			return r;

		vm->pasid = 0;
	}

	if (pasid) {
		r = xa_err(xa_store_irq(&adev->vm_manager.pasids, pasid, vm,
					GFP_KERNEL));
		if (r < 0)
			return r;

		vm->pasid = pasid;
	}


	return 0;
}

/**
 * gsgpu_vm_bo_evicted - vm_bo is evicted
 *
 * @vm_bo: vm_bo which is evicted
 *
 * State for PDs/PTs and per VM BOs which are not at the location they should
 * be.
 */
static void gsgpu_vm_bo_evicted(struct gsgpu_vm_bo_base *vm_bo)
{
	struct gsgpu_vm *vm = vm_bo->vm;
	struct gsgpu_bo *bo = vm_bo->bo;

	vm_bo->moved = true;
	spin_lock(&vm_bo->vm->status_lock);
	if (bo->tbo.type == ttm_bo_type_kernel)
		list_move(&vm_bo->vm_status, &vm->evicted);
	else
		list_move_tail(&vm_bo->vm_status, &vm->evicted);
	spin_unlock(&vm_bo->vm->status_lock);
}
/**
 * gsgpu_vm_bo_moved - vm_bo is moved
 *
 * @vm_bo: vm_bo which is moved
 *
 * State for per VM BOs which are moved, but that change is not yet reflected
 * in the page tables.
 */
static void gsgpu_vm_bo_moved(struct gsgpu_vm_bo_base *vm_bo)
{
	spin_lock(&vm_bo->vm->status_lock);
	list_move(&vm_bo->vm_status, &vm_bo->vm->moved);
	spin_unlock(&vm_bo->vm->status_lock);
}

/**
 * gsgpu_vm_bo_idle - vm_bo is idle
 *
 * @vm_bo: vm_bo which is now idle
 *
 * State for PDs/PTs and per VM BOs which have gone through the state machine
 * and are now idle.
 */
static void gsgpu_vm_bo_idle(struct gsgpu_vm_bo_base *vm_bo)
{
	spin_lock(&vm_bo->vm->status_lock);
	list_move(&vm_bo->vm_status, &vm_bo->vm->idle);
	spin_unlock(&vm_bo->vm->status_lock);
	vm_bo->moved = false;
}

/**
 * gsgpu_vm_bo_invalidated - vm_bo is invalidated
 *
 * @vm_bo: vm_bo which is now invalidated
 *
 * State for normal BOs which are invalidated and that change not yet reflected
 * in the PTs.
 */
static void gsgpu_vm_bo_invalidated(struct gsgpu_vm_bo_base *vm_bo)
{
	spin_lock(&vm_bo->vm->status_lock);
	list_move(&vm_bo->vm_status, &vm_bo->vm->invalidated);
	spin_unlock(&vm_bo->vm->status_lock);
}

/**
 * gsgpu_vm_bo_relocated - vm_bo is reloacted
 *
 * @vm_bo: vm_bo which is relocated
 *
 * State for PDs/PTs which needs to update their parent PD.
 * For the root PD, just move to idle state.
 */
static void gsgpu_vm_bo_relocated(struct gsgpu_vm_bo_base *vm_bo)
{
	if (vm_bo->bo->parent) {
		spin_lock(&vm_bo->vm->status_lock);
		list_move(&vm_bo->vm_status, &vm_bo->vm->relocated);
		spin_unlock(&vm_bo->vm->status_lock);
	} else {
		gsgpu_vm_bo_idle(vm_bo);
	}
}

/**
 * gsgpu_vm_bo_done - vm_bo is done
 *
 * @vm_bo: vm_bo which is now done
 *
 * State for normal BOs which are invalidated and that change has been updated
 * in the PTs.
 */
static void gsgpu_vm_bo_done(struct gsgpu_vm_bo_base *vm_bo)
{
	spin_lock(&vm_bo->vm->status_lock);
	list_move(&vm_bo->vm_status, &vm_bo->vm->done);
	spin_unlock(&vm_bo->vm->status_lock);
}

/**
 * gsgpu_vm_bo_base_init - Adds bo to the list of bos associated with the vm
 *
 * @base: base structure for tracking BO usage in a VM
 * @vm: vm to which bo is to be added
 * @bo: gsgpu buffer object
 *
 * Initialize a bo_va_base structure and add it to the appropriate lists
 *
 */
void gsgpu_vm_bo_base_init(struct gsgpu_vm_bo_base *base,
			    struct gsgpu_vm *vm, struct gsgpu_bo *bo)
{
	base->vm = vm;
	base->bo = bo;
	base->next = NULL;
	INIT_LIST_HEAD(&base->vm_status);

	if (!bo)
		return;
	base->next = bo->vm_bo;
	bo->vm_bo = base;

	if (bo->tbo.base.resv != vm->root.bo->tbo.base.resv)
		return;

	dma_resv_assert_held(vm->root.bo->tbo.base.resv);

	ttm_bo_set_bulk_move(&bo->tbo, &vm->lru_bulk_move);
	if (bo->tbo.type == ttm_bo_type_kernel && bo->parent)
		gsgpu_vm_bo_relocated(base);
	else
		gsgpu_vm_bo_idle(base);

	if (bo->preferred_domains &
	    gsgpu_mem_type_to_domain(bo->tbo.resource->mem_type))
		return;

	/*
	 * we checked all the prerequisites, but it looks like this per vm bo
	 * is currently evicted. add the bo to the evicted list to make sure it
	 * is validated on next vm use to avoid fault.
	 * */
	gsgpu_vm_bo_evicted(base);
}


// static u32 gsgpu_get_pde_pte_size (struct gsgpu_device *ldev)
// {
// 	return ldev->vm_manager.pde_pte_bytes;
// }


/**
 * gsgpu_vm_get_pd_bo - add the VM PD to a validation list
 *
 * @vm: vm providing the BOs
 * @validated: head of validation list
 * @entry: entry to add
 *
 * Add the page directory to the list of BOs to
 * validate for command submission.
 */
void gsgpu_vm_get_pd_bo(struct gsgpu_vm *vm,
			 struct list_head *validated,
			 struct gsgpu_bo_list_entry *entry)
{
	entry->priority = 0;
	entry->tv.bo = &vm->root.bo->tbo;
	/* Two for VM updates, one for TTM and one for the CS job */
	entry->tv.num_shared = 4;
	entry->user_pages = NULL;
	list_add(&entry->tv.head, validated);
}

/**
 * gsgpu_vm_move_to_lru_tail - move all BOs to the end of LRU
 *
 * @adev: gsgpu device pointer
 * @vm: vm providing the BOs
 *
 * Move all BOs to the end of LRU and remember their positions to put them
 * together.
 */
void gsgpu_vm_move_to_lru_tail(struct gsgpu_device *adev,
				struct gsgpu_vm *vm)
{
	spin_lock(&adev->mman.bdev.lru_lock);
	ttm_lru_bulk_move_tail(&vm->lru_bulk_move);
	spin_unlock(&adev->mman.bdev.lru_lock);
}

/**
 * gsgpu_vm_validate_pt_bos - validate the page table BOs
 *
 * @adev: gsgpu device pointer
 * @vm: vm providing the BOs
 * @validate: callback to do the validation
 * @param: parameter for the validation callback
 *
 * Validate the page table BOs on command submission if neccessary.
 *
 * Returns:
 * Validation result.
 */
int gsgpu_vm_validate_pt_bos(struct gsgpu_device *adev, struct gsgpu_vm *vm,
			      int (*validate)(void *p, struct gsgpu_bo *bo),
			      void *param)
{
	struct gsgpu_vm_bo_base *bo_base;
	struct gsgpu_bo *shadow;
	struct gsgpu_bo *bo;
	int r;

	spin_lock(&vm->status_lock);
	while (!list_empty(&vm->evicted)) {
		bo_base = list_first_entry(&vm->evicted,
					   struct gsgpu_vm_bo_base,
					   vm_status);
		spin_unlock(&vm->status_lock);

		bo = bo_base->bo;
		shadow = gsgpu_bo_shadowed(bo);

		r = validate(param, bo);
		if (r)
			return r;
		if (shadow) {
			r = validate(param, shadow);
			if (r)
				return r;
		}

		if (bo->tbo.type != ttm_bo_type_kernel) {
			gsgpu_vm_bo_moved(bo_base);
		} else {
			vm->update_funcs->map_table(to_gsgpu_bo_vm(bo));
			gsgpu_vm_bo_relocated(bo_base);
		}
		spin_lock(&vm->status_lock);
	}
	spin_unlock(&vm->status_lock);

	gsgpu_vm_eviction_lock(vm);
	vm->evicting = false;
	gsgpu_vm_eviction_unlock(vm);

	return 0;
}

/**
 * gsgpu_vm_ready - check VM is ready for updates
 *
 * @vm: VM to check
 *
 * Check if all VM PDs/PTs are ready for updates
 *
 * Returns:
 * True if VM is not evicting.
 */
bool gsgpu_vm_ready(struct gsgpu_vm *vm)
{
	bool empty;
	bool ret;

	gsgpu_vm_eviction_lock(vm);
	ret = !vm->evicting;
	gsgpu_vm_eviction_unlock(vm);

	spin_lock(&vm->status_lock);
	empty = list_empty(&vm->evicted);
	spin_unlock(&vm->status_lock);

	return ret && empty;
}

/**
 * gsgpu_vm_need_pipeline_sync - Check if pipe sync is needed for job.
 *
 * @ring: ring on which the job will be submitted
 * @job: job to submit
 *
 * Returns:
 * True if sync is needed.
 */
bool gsgpu_vm_need_pipeline_sync(struct gsgpu_ring *ring,
				  struct gsgpu_job *job)
{
	struct gsgpu_device *adev = ring->adev;
	struct gsgpu_vmid_mgr *id_mgr = &adev->vm_manager.id_mgr;

	if (job->vmid == 0)
		return false;

	if (gsgpu_vmid_had_gpu_reset(adev, &id_mgr->ids[job->vmid]))
		return true;

	return false;
}

/**
 * gsgpu_vm_flush - hardware flush the vm
 *
 * @ring: ring to use for flush
 * @job:  related job
 * @need_pipe_sync: is pipe sync needed
 *
 * Emit a VM flush when it is necessary.
 *
 * Returns:
 * 0 on success, errno otherwise.
 */
int gsgpu_vm_flush(struct gsgpu_ring *ring, struct gsgpu_job *job,
		    bool need_pipe_sync)
{
	struct gsgpu_device *adev = ring->adev;
	struct gsgpu_vmid_mgr *id_mgr = &adev->vm_manager.id_mgr;
	struct gsgpu_vmid *id = &id_mgr->ids[job->vmid];
	bool spm_update_needed = job->spm_update_needed;
	bool vm_flush_needed = job->vm_needs_flush;
	struct dma_fence *fence = NULL;
	bool pasid_mapping_needed = false;
	int r;

	if (gsgpu_vmid_had_gpu_reset(adev, id)) {
		vm_flush_needed = true;
		pasid_mapping_needed = true;
		spm_update_needed = true;
	}

	mutex_lock(&id_mgr->lock);
	if (id->pasid != job->pasid || !id->pasid_mapping ||
	    !dma_fence_is_signaled(id->pasid_mapping))
		pasid_mapping_needed = true;
	mutex_unlock(&id_mgr->lock);

	vm_flush_needed &= !!ring->funcs->emit_vm_flush  &&
			job->vm_pd_addr != GSGPU_BO_INVALID_OFFSET;
	pasid_mapping_needed &= adev->gmc.gmc_funcs->emit_pasid_mapping &&
		ring->funcs->emit_wreg;

	if (!vm_flush_needed && !need_pipe_sync)
		return 0;

	gsgpu_ring_ib_begin(ring);

	if (vm_flush_needed || pasid_mapping_needed) {
		trace_gsgpu_vm_flush(ring, job->vmid, job->vm_pd_addr);
		gsgpu_ring_emit_vm_flush(ring, job->vmid, job->vm_pd_addr);
	}

	if (vm_flush_needed) {
		r = gsgpu_fence_emit(ring, &fence, NULL, 0);
		if (r)
			return r;
	}

	if (vm_flush_needed) {
		mutex_lock(&id_mgr->lock);
		dma_fence_put(id->last_flush);
		id->last_flush = dma_fence_get(fence);
		id->current_gpu_reset_count =
			atomic_read(&adev->gpu_reset_counter);
		mutex_unlock(&id_mgr->lock);
	}

	if (pasid_mapping_needed) {
		mutex_lock(&id_mgr->lock);
		id->pasid = job->pasid;
		dma_fence_put(id->pasid_mapping);
		id->pasid_mapping = dma_fence_get(fence);
		mutex_unlock(&id_mgr->lock);
	}
	dma_fence_put(fence);

	gsgpu_ring_ib_end(ring);
	return 0;
}

/**
 * gsgpu_vm_bo_find - find the bo_va for a specific vm & bo
 *
 * @vm: requested vm
 * @bo: requested buffer object
 *
 * Find @bo inside the requested vm.
 * Search inside the @bos vm list for the requested vm
 * Returns the found bo_va or NULL if none is found
 *
 * Object has to be reserved!
 *
 * Returns:
 * Found bo_va or NULL.
 */
struct gsgpu_bo_va *gsgpu_vm_bo_find(struct gsgpu_vm *vm,
				       struct gsgpu_bo *bo)
{
	struct gsgpu_vm_bo_base *base;

	for (base = bo->vm_bo; base; base = base->next) {
		if (base->vm != vm)
			continue;

		return container_of(base, struct gsgpu_bo_va, base);
	}
	return NULL;
}

/**
 * gsgpu_vm_map_gart - Resolve gart mapping of addr
 *
 * @pages_addr: optional DMA address to use for lookup
 * @addr: the unmapped addr
 *
 * Look up the physical address of the page that the pte resolves
 * to.
 *
 * Returns:
 * The pointer for the page table entry.
 */
uint64_t gsgpu_vm_map_gart(const dma_addr_t *pages_addr, uint64_t addr)
{
	uint64_t result;

	/* page table offset */
	result = pages_addr[addr >> PAGE_SHIFT];

	/* in case cpu page size != gpu page size*/
	result |= addr & (~PAGE_MASK);

	result &= ~((1ULL << GSGPU_GPU_PAGE_SHIFT) - 1);

	return result;
}

/**
 * gsgpu_vm_update_pdes - make sure that all directories are valid
 *
 * @adev: gsgpu_device pointer
 * @vm: requested vm
 * @immediate: submit immediately to the paging queue
 *
 * Makes sure all directories are up to date.
 *
 * Returns:
 * 0 for success, error for failure.
 */
int gsgpu_vm_update_pdes(struct gsgpu_device *adev,
			  struct gsgpu_vm *vm, bool immediate)
{
	struct gsgpu_vm_update_params params;
	struct gsgpu_vm_bo_base *entry;
	bool flush_tlb_needed = false;
	LIST_HEAD(relocated);
	int r, idx;

	spin_lock(&vm->status_lock);
	list_splice_init(&vm->relocated, &relocated);
	spin_unlock(&vm->status_lock);

	if (list_empty(&relocated))
		return 0;

	if (!drm_dev_enter(adev_to_drm(adev), &idx))
		return -ENODEV;

	memset(&params, 0, sizeof(params));
	params.adev = adev;
	params.vm = vm;
	params.immediate = immediate;

	r = vm->update_funcs->prepare(&params, NULL, GSGPU_SYNC_EXPLICIT);
	if (r)
		goto error;

	list_for_each_entry(entry, &relocated, vm_status) {
		/* vm_flush_needed after updating moved PDEs */
		flush_tlb_needed |= entry->moved;

		r = gsgpu_vm_pde_update(&params, entry);
		if (r)
			goto error;
	}

	r = vm->update_funcs->commit(&params, &vm->last_update);
	if (r)
		goto error;

	if (flush_tlb_needed)
		atomic64_inc(&vm->tlb_seq);

	while (!list_empty(&relocated)) {
		entry = list_first_entry(&relocated, struct gsgpu_vm_bo_base,
					 vm_status);
		gsgpu_vm_bo_idle(entry);
	}

error:
	drm_dev_exit(idx);
	return r;
}

/**
 * gsgpu_vm_tlb_seq_cb - make sure to increment tlb sequence
 * @fence: unused
 * @cb: the callback structure
 *
 * Increments the tlb sequence to make sure that future CS execute a VM flush.
 */
static void gsgpu_vm_tlb_seq_cb(struct dma_fence *fence,
				 struct dma_fence_cb *cb)
{
	struct gsgpu_vm_tlb_seq_cb *tlb_cb;

	tlb_cb = container_of(cb, typeof(*tlb_cb), cb);
	atomic64_inc(&tlb_cb->vm->tlb_seq);
	kfree(tlb_cb);
}

/**
 * gsgpu_vm_update_range - update a range in the vm page table
 *
 * @adev: gsgpu_device pointer to use for commands
 * @vm: the VM to update the range
 * @immediate: immediate submission in a page fault
 * @unlocked: unlocked invalidation during MM callback
 * @flush_tlb: trigger tlb invalidation after update completed
 * @resv: fences we need to sync to
 * @start: start of mapped range
 * @last: last mapped entry
 * @flags: flags for the entries
 * @offset: offset into nodes and pages_addr
 * @vram_base: base for vram mappings
 * @res: ttm_resource to map
 * @pages_addr: DMA addresses to use for mapping
 * @fence: optional resulting fence
 *
 * Fill in the page table entries between @start and @last.
 *
 * Returns:
 * 0 for success, negative erro code for failure.
 */
int gsgpu_vm_update_range(struct gsgpu_device *adev, struct gsgpu_vm *vm,
			   bool immediate, bool unlocked, bool flush_tlb,
			   struct dma_resv *resv, uint64_t start, uint64_t last,
			   uint64_t flags, uint64_t offset, uint64_t vram_base,
			   struct ttm_resource *res, dma_addr_t *pages_addr,
			   struct dma_fence **fence)
{
	struct gsgpu_vm_update_params params;
	struct gsgpu_vm_tlb_seq_cb *tlb_cb;
	struct gsgpu_res_cursor cursor;
	enum gsgpu_sync_mode sync_mode;
	int r, idx;

	if (!drm_dev_enter(adev_to_drm(adev), &idx))
		return -ENODEV;

	tlb_cb = kmalloc(sizeof(*tlb_cb), GFP_KERNEL);
	if (!tlb_cb) {
		r = -ENOMEM;
		goto error_unlock;
	}

	memset(&params, 0, sizeof(params));
	params.adev = adev;
	params.vm = vm;
	params.immediate = immediate;
	params.pages_addr = pages_addr;
	params.unlocked = unlocked;

	/* Implicitly sync to command submissions in the same VM before
	 * unmapping. Sync to moving fences before mapping.
	 */
	if (!(flags & GSGPU_PTE_VALID))
		sync_mode = GSGPU_SYNC_EQ_OWNER;
	else
		sync_mode = GSGPU_SYNC_EXPLICIT;

	gsgpu_vm_eviction_lock(vm);
	if (vm->evicting) {
		r = -EBUSY;
		goto error_free;
	}

	if (!unlocked && !dma_fence_is_signaled(vm->last_unlocked)) {
		struct dma_fence *tmp = dma_fence_get_stub();

		gsgpu_bo_fence(vm->root.bo, vm->last_unlocked, true);
		swap(vm->last_unlocked, tmp);
		dma_fence_put(tmp);
	}

	r = vm->update_funcs->prepare(&params, resv, sync_mode);
	if (r)
		goto error_free;

	gsgpu_res_first(pages_addr ? NULL : res, offset,
			 (last - start + 1) * GSGPU_GPU_PAGE_SIZE, &cursor);
	while (cursor.remaining) {
		uint64_t tmp, num_entries, addr;

		num_entries = cursor.size >> GSGPU_GPU_PAGE_SHIFT;
		if (pages_addr) {
			bool contiguous = true;

			if (num_entries > GSGPU_GPU_PAGES_IN_CPU_PAGE) {
				uint64_t pfn = cursor.start >> PAGE_SHIFT;
				uint64_t count;

				contiguous = pages_addr[pfn + 1] ==
					pages_addr[pfn] + PAGE_SIZE;

				tmp = num_entries /
					GSGPU_GPU_PAGES_IN_CPU_PAGE;
				for (count = 2; count < tmp; ++count) {
					uint64_t idx = pfn + count;

					if (contiguous != (pages_addr[idx] ==
					    pages_addr[idx - 1] + PAGE_SIZE))
						break;
				}
				num_entries = count *
					GSGPU_GPU_PAGES_IN_CPU_PAGE;
			}

			if (!contiguous) {
				addr = cursor.start;
				params.pages_addr = pages_addr;
			} else {
				addr = pages_addr[cursor.start >> PAGE_SHIFT];
				params.pages_addr = NULL;
			}

		} else if (flags & (GSGPU_PTE_VALID | GSGPU_PTE_PRT)) {
			addr = vram_base + cursor.start;
		} else {
			addr = 0;
		}

		tmp = start + num_entries;
		r = gsgpu_vm_ptes_update(&params, start, tmp, addr, flags);
		if (r)
			goto error_free;

		gsgpu_res_next(&cursor, num_entries * GSGPU_GPU_PAGE_SIZE);
		start = tmp;
	}

	r = vm->update_funcs->commit(&params, fence);

	if (flush_tlb || params.table_freed) {
		tlb_cb->vm = vm;
		if (fence && *fence &&
		    !dma_fence_add_callback(*fence, &tlb_cb->cb,
					   gsgpu_vm_tlb_seq_cb)) {
			dma_fence_put(vm->last_tlb_flush);
			vm->last_tlb_flush = dma_fence_get(*fence);
		} else {
			gsgpu_vm_tlb_seq_cb(NULL, &tlb_cb->cb);
		}
		tlb_cb = NULL;
	}

error_free:
	kfree(tlb_cb);

error_unlock:
	gsgpu_vm_eviction_unlock(vm);
	drm_dev_exit(idx);
	return r;
}

void gsgpu_vm_get_memory(struct gsgpu_vm *vm, uint64_t *vram_mem,
				uint64_t *gtt_mem, uint64_t *cpu_mem)
{
	struct gsgpu_bo_va *bo_va, *tmp;

	spin_lock(&vm->status_lock);
	list_for_each_entry_safe(bo_va, tmp, &vm->idle, base.vm_status) {
		if (!bo_va->base.bo)
			continue;
		gsgpu_bo_get_memory(bo_va->base.bo, vram_mem,
				gtt_mem, cpu_mem);
	}
	list_for_each_entry_safe(bo_va, tmp, &vm->evicted, base.vm_status) {
		if (!bo_va->base.bo)
			continue;
		gsgpu_bo_get_memory(bo_va->base.bo, vram_mem,
				gtt_mem, cpu_mem);
	}
	list_for_each_entry_safe(bo_va, tmp, &vm->relocated, base.vm_status) {
		if (!bo_va->base.bo)
			continue;
		gsgpu_bo_get_memory(bo_va->base.bo, vram_mem,
				gtt_mem, cpu_mem);
	}
	list_for_each_entry_safe(bo_va, tmp, &vm->moved, base.vm_status) {
		if (!bo_va->base.bo)
			continue;
		gsgpu_bo_get_memory(bo_va->base.bo, vram_mem,
				gtt_mem, cpu_mem);
	}
	list_for_each_entry_safe(bo_va, tmp, &vm->invalidated, base.vm_status) {
		if (!bo_va->base.bo)
			continue;
		gsgpu_bo_get_memory(bo_va->base.bo, vram_mem,
				gtt_mem, cpu_mem);
	}
	list_for_each_entry_safe(bo_va, tmp, &vm->done, base.vm_status) {
		if (!bo_va->base.bo)
			continue;
		gsgpu_bo_get_memory(bo_va->base.bo, vram_mem,
				gtt_mem, cpu_mem);
	}
	spin_unlock(&vm->status_lock);
}
/**
 * gsgpu_vm_bo_update - update all BO mappings in the vm page table
 *
 * @adev: gsgpu_device pointer
 * @bo_va: requested BO and VM object
 * @clear: if true clear the entries
 *
 * Fill in the page table entries for @bo_va.
 *
 * Returns:
 * 0 for success, -EINVAL for failure.
 */
int gsgpu_vm_bo_update(struct gsgpu_device *adev, struct gsgpu_bo_va *bo_va,
			bool clear)
{
	struct gsgpu_bo *bo = bo_va->base.bo;
	struct gsgpu_vm *vm = bo_va->base.vm;
	struct gsgpu_bo_va_mapping *mapping;
	dma_addr_t *pages_addr = NULL;
	struct ttm_resource *mem;
	struct dma_fence **last_update;
	bool flush_tlb = clear;
	struct dma_resv *resv;
	uint64_t vram_base;
	uint64_t flags;
	int r;

	if (clear || !bo) {
		mem = NULL;
		resv = vm->root.bo->tbo.base.resv;
	} else {
		// struct drm_gem_object *obj = &bo->tbo.base;

		resv = bo->tbo.base.resv;
		
		mem = bo->tbo.resource;
		if (mem->mem_type == TTM_PL_TT ||
		    mem->mem_type == GSGPU_PL_PREEMPT)
			pages_addr = bo->tbo.ttm->dma_address;
	}

	if (bo) {
		struct gsgpu_device *bo_adev;

		flags = gsgpu_ttm_tt_pte_flags(adev, bo->tbo.ttm, mem);

		if (gsgpu_bo_encrypted(bo))
			flags |= GSGPU_PTE_TMZ;

		bo_adev = gsgpu_ttm_adev(bo->tbo.bdev);
		vram_base = bo_adev->vm_manager.vram_base_offset;
	} else {
		flags = 0x0;
		vram_base = 0;
	}

	if(bo && bo->flags & GSGPU_GEM_CREATE_COMPRESSED_MASK)
		flags |= (bo->flags & GSGPU_GEM_CREATE_COMPRESSED_MASK)
				  >> GSGPU_PTE_COMPRESSED_SHIFT;

	if (clear || (bo && bo->tbo.base.resv ==
		      vm->root.bo->tbo.base.resv))
		last_update = &vm->last_update;
	else
		last_update = &bo_va->last_pt_update;

	if (!clear && bo_va->base.moved) {
		flush_tlb = true;
		list_splice_init(&bo_va->valids, &bo_va->invalids);

	} else if (bo_va->cleared != clear) {
		list_splice_init(&bo_va->valids, &bo_va->invalids);
	}

	list_for_each_entry(mapping, &bo_va->invalids, list) {
		uint64_t update_flags = flags;

		/* normally,bo_va->flags only contians READABLE and WIRTEABLE bit go here
		 * but in case of something, we filter the flags in first place
		 */
		if (!(mapping->flags & GSGPU_PTE_READABLE))
			update_flags &= ~GSGPU_PTE_READABLE;
		if (!(mapping->flags & GSGPU_PTE_WRITEABLE))
			update_flags &= ~GSGPU_PTE_WRITEABLE;

		/* Apply ASIC specific mapping flags */
		gsgpu_gmc_get_vm_pte(adev, mapping, &update_flags);

		trace_gsgpu_vm_bo_update(mapping);

		r = gsgpu_vm_update_range(adev, vm, false, false, flush_tlb,
					   resv, mapping->start, mapping->last,
					   update_flags, mapping->offset,
					   vram_base, mem, pages_addr,
					   last_update);
		if (r)
			return r;
	}

	/* If the BO is not in its preferred location add it back to
	 * the evicted list so that it gets validated again on the
	 * next command submission.
	 */
	if (bo && bo->tbo.base.resv == vm->root.bo->tbo.base.resv) {
		uint32_t mem_type = bo->tbo.resource->mem_type;

		if (!(bo->preferred_domains &
		      gsgpu_mem_type_to_domain(mem_type)))
			gsgpu_vm_bo_evicted(&bo_va->base);
		else
			gsgpu_vm_bo_idle(&bo_va->base);
	} else {
		gsgpu_vm_bo_done(&bo_va->base);
	}

	list_splice_init(&bo_va->invalids, &bo_va->valids);
	bo_va->cleared = clear;
	bo_va->base.moved = false;

	if (trace_gsgpu_vm_bo_mapping_enabled()) {
		list_for_each_entry(mapping, &bo_va->valids, list)
			trace_gsgpu_vm_bo_mapping(mapping);
	}

	return 0;
}

/**
 * gsgpu_vm_free_mapping - free a mapping
 *
 * @adev: gsgpu_device pointer
 * @vm: requested vm
 * @mapping: mapping to be freed
 * @fence: fence of the unmap operation
 *
 * Free a mapping and make sure we decrease the PRT usage count if applicable.
 */
static void gsgpu_vm_free_mapping(struct gsgpu_device *adev,
				   struct gsgpu_vm *vm,
				   struct gsgpu_bo_va_mapping *mapping,
				   struct dma_fence *fence)
{
	kfree(mapping);
}

/**
 * gsgpu_vm_clear_freed - clear freed BOs in the PT
 *
 * @adev: gsgpu_device pointer
 * @vm: requested vm
 * @fence: optional resulting fence (unchanged if no work needed to be done
 * or if an error occurred)
 *
 * Make sure all freed BOs are cleared in the PT.
 * PTs have to be reserved and mutex must be locked!
 *
 * Returns:
 * 0 for success.
 *
 */
int gsgpu_vm_clear_freed(struct gsgpu_device *adev,
			  struct gsgpu_vm *vm,
			  struct dma_fence **fence)
{
	struct dma_resv *resv = vm->root.bo->tbo.base.resv;
	struct gsgpu_bo_va_mapping *mapping;
	uint64_t init_pte_value = 0;
	struct dma_fence *f = NULL;
	int r;

	while (!list_empty(&vm->freed)) {
		mapping = list_first_entry(&vm->freed,
			struct gsgpu_bo_va_mapping, list);
		list_del(&mapping->list);

		r = gsgpu_vm_update_range(adev, vm, false, false, true, resv,
					   mapping->start, mapping->last,
					   init_pte_value, 0, 0, NULL, NULL,
					   &f);
		gsgpu_vm_free_mapping(adev, vm, mapping, f);
		if (r) {
			dma_fence_put(f);
			return r;
		}
	}

	if (fence && f) {
		dma_fence_put(*fence);
		*fence = f;
	} else {
		dma_fence_put(f);
	}

	return 0;

}

/**
 * gsgpu_vm_handle_moved - handle moved BOs in the PT
 *
 * @adev: gsgpu_device pointer
 * @vm: requested vm
 *
 * Make sure all BOs which are moved are updated in the PTs.
 *
 * Returns:
 * 0 for success.
 *
 * PTs have to be reserved!
 */
int gsgpu_vm_handle_moved(struct gsgpu_device *adev,
			   struct gsgpu_vm *vm)
{
	struct gsgpu_bo_va *bo_va;
	struct dma_resv *resv;
	bool clear;
	int r;

	spin_lock(&vm->status_lock);
	while (!list_empty(&vm->moved)) {
		bo_va = list_first_entry(&vm->moved, struct gsgpu_bo_va,
					 base.vm_status);
		spin_unlock(&vm->status_lock);

		/* Per VM BOs never need to bo cleared in the page tables */
		r = gsgpu_vm_bo_update(adev, bo_va, false);
		if (r)
			return r;
		spin_lock(&vm->status_lock);
	}

	while (!list_empty(&vm->invalidated)) {
		bo_va = list_first_entry(&vm->invalidated, struct gsgpu_bo_va,
					 base.vm_status);
		resv = bo_va->base.bo->tbo.base.resv;
		spin_unlock(&vm->status_lock);

		/* Try to reserve the BO to avoid clearing its ptes */
		if (!gsgpu_vm_debug && dma_resv_trylock(resv))
			clear = false;
		/* Somebody else is using the BO right now */
		else
			clear = true;

		r = gsgpu_vm_bo_update(adev, bo_va, clear);
		if (r)
			return r;

		if (!clear)
			dma_resv_unlock(resv);
		spin_lock(&vm->status_lock);
	}
	spin_unlock(&vm->status_lock);

	return 0;
}

/**
 * gsgpu_vm_bo_add - add a bo to a specific vm
 *
 * @adev: gsgpu_device pointer
 * @vm: requested vm
 * @bo: gsgpu buffer object
 *
 * Add @bo into the requested vm.
 * Add @bo to the list of bos associated with the vm
 *
 * Returns:
 * Newly added bo_va or NULL for failure
 *
 * Object has to be reserved!
 */
struct gsgpu_bo_va *gsgpu_vm_bo_add(struct gsgpu_device *adev,
				      struct gsgpu_vm *vm,
				      struct gsgpu_bo *bo)
{
	struct gsgpu_bo_va *bo_va;

	bo_va = kzalloc(sizeof(struct gsgpu_bo_va), GFP_KERNEL);
	if (bo_va == NULL) {
		return NULL;
	}
	gsgpu_vm_bo_base_init(&bo_va->base, vm, bo);

	bo_va->ref_count = 1;
	INIT_LIST_HEAD(&bo_va->valids);
	INIT_LIST_HEAD(&bo_va->invalids);

	if (!bo)
		return bo_va;

	dma_resv_assert_held(bo->tbo.base.resv);

	return bo_va;
}


/**
 * gsgpu_vm_bo_insert_map - insert a new mapping
 *
 * @adev: gsgpu_device pointer
 * @bo_va: bo_va to store the address
 * @mapping: the mapping to insert
 *
 * Insert a new mapping into all structures.
 */
static void gsgpu_vm_bo_insert_map(struct gsgpu_device *adev,
				    struct gsgpu_bo_va *bo_va,
				    struct gsgpu_bo_va_mapping *mapping)
{
	struct gsgpu_vm *vm = bo_va->base.vm;
	struct gsgpu_bo *bo = bo_va->base.bo;

	mapping->bo_va = bo_va;
	list_add(&mapping->list, &bo_va->invalids);
	gsgpu_vm_it_insert(mapping, &vm->va);

	if (bo && bo->tbo.base.resv == vm->root.bo->tbo.base.resv &&
	    !bo_va->base.moved) {
		gsgpu_vm_bo_moved(&bo_va->base);
	}
	trace_gsgpu_vm_bo_map(bo_va, mapping);
}

/**
 * gsgpu_vm_bo_map - map bo inside a vm
 *
 * @adev: gsgpu_device pointer
 * @bo_va: bo_va to store the address
 * @saddr: where to map the BO
 * @offset: requested offset in the BO
 * @size: BO size in bytes
 * @flags: attributes of pages (read/write/valid/etc.)
 *
 * Add a mapping of the BO at the specefied addr into the VM.
 *
 * Returns:
 * 0 for success, error for failure.
 *
 * Object has to be reserved and unreserved outside!
 */
int gsgpu_vm_bo_map(struct gsgpu_device *adev,
		     struct gsgpu_bo_va *bo_va,
		     uint64_t saddr, uint64_t offset,
		     uint64_t size, uint64_t flags)
{
	struct gsgpu_bo_va_mapping *mapping, *tmp;
	struct gsgpu_bo *bo = bo_va->base.bo;
	struct gsgpu_vm *vm = bo_va->base.vm;
	uint64_t eaddr;

	/* validate the parameters */
	if (saddr & ~PAGE_MASK || offset & ~PAGE_MASK ||
	    size == 0 || size & ~PAGE_MASK)
		return -EINVAL;

	/* make sure object fit at this offset */
	eaddr = saddr + size - 1;
	if (saddr >= eaddr ||
	    (bo && offset + size > gsgpu_bo_size(bo)) ||
	    (eaddr >= adev->vm_manager.max_pfn << GSGPU_GPU_PAGE_SHIFT))
		return -EINVAL;

	saddr /= GSGPU_GPU_PAGE_SIZE;
	eaddr /= GSGPU_GPU_PAGE_SIZE;

	tmp = gsgpu_vm_it_iter_first(&vm->va, saddr, eaddr);
	if (tmp) {
		/* bo and tmp overlap, invalid addr */
		dev_err(adev->dev, "bo %p va 0x%010Lx-0x%010Lx conflict with "
			"0x%010Lx-0x%010Lx\n", bo, saddr, eaddr,
			tmp->start, tmp->last + 1);
		return -EINVAL;
	}

	mapping = kmalloc(sizeof(*mapping), GFP_KERNEL);
	if (!mapping)
		return -ENOMEM;

	mapping->start = saddr;
	mapping->last = eaddr;
	mapping->offset = offset;
	mapping->flags = flags;

	gsgpu_vm_bo_insert_map(adev, bo_va, mapping);

	return 0;
}

/**
 * gsgpu_vm_bo_replace_map - map bo inside a vm, replacing existing mappings
 *
 * @adev: gsgpu_device pointer
 * @bo_va: bo_va to store the address
 * @saddr: where to map the BO
 * @offset: requested offset in the BO
 * @size: BO size in bytes
 * @flags: attributes of pages (read/write/valid/etc.)
 *
 * Add a mapping of the BO at the specefied addr into the VM. Replace existing
 * mappings as we do so.
 *
 * Returns:
 * 0 for success, error for failure.
 *
 * Object has to be reserved and unreserved outside!
 */
int gsgpu_vm_bo_replace_map(struct gsgpu_device *adev,
			     struct gsgpu_bo_va *bo_va,
			     uint64_t saddr, uint64_t offset,
			     uint64_t size, uint64_t flags)
{
	struct gsgpu_bo_va_mapping *mapping;
	struct gsgpu_bo *bo = bo_va->base.bo;
	uint64_t eaddr;
	int r;

	/* validate the parameters */
	if (saddr & ~PAGE_MASK || offset & ~PAGE_MASK ||
	    size == 0 || size & ~PAGE_MASK)
		return -EINVAL;

	/* make sure object fit at this offset */
	eaddr = saddr + size - 1;
	if (saddr >= eaddr ||
	    (bo && offset + size > gsgpu_bo_size(bo)) ||
	    (eaddr >= adev->vm_manager.max_pfn << GSGPU_GPU_PAGE_SHIFT))
		return -EINVAL;

	/* Allocate all the needed memory */
	mapping = kmalloc(sizeof(*mapping), GFP_KERNEL);
	if (!mapping)
		return -ENOMEM;

	r = gsgpu_vm_bo_clear_mappings(adev, bo_va->base.vm, saddr, size);
	if (r) {
		kfree(mapping);
		return r;
	}

	saddr /= GSGPU_GPU_PAGE_SIZE;
	eaddr /= GSGPU_GPU_PAGE_SIZE;

	mapping->start = saddr;
	mapping->last = eaddr;
	mapping->offset = offset;
	mapping->flags = flags;

	gsgpu_vm_bo_insert_map(adev, bo_va, mapping);

	return 0;
}

/**
 * gsgpu_vm_bo_unmap - remove bo mapping from vm
 *
 * @adev: gsgpu_device pointer
 * @bo_va: bo_va to remove the address from
 * @saddr: where to the BO is mapped
 *
 * Remove a mapping of the BO at the specefied addr from the VM.
 *
 * Returns:
 * 0 for success, error for failure.
 *
 * Object has to be reserved and unreserved outside!
 */
int gsgpu_vm_bo_unmap(struct gsgpu_device *adev,
		       struct gsgpu_bo_va *bo_va,
		       uint64_t saddr)
{
	struct gsgpu_bo_va_mapping *mapping;
	struct gsgpu_vm *vm = bo_va->base.vm;
	bool valid = true;

	saddr /= GSGPU_GPU_PAGE_SIZE;

	list_for_each_entry(mapping, &bo_va->valids, list) {
		if (mapping->start == saddr)
			break;
	}

	if (&mapping->list == &bo_va->valids) {
		valid = false;

		list_for_each_entry(mapping, &bo_va->invalids, list) {
			if (mapping->start == saddr)
				break;
		}

		if (&mapping->list == &bo_va->invalids)
			return -ENOENT;
	}

	list_del(&mapping->list);
	gsgpu_vm_it_remove(mapping, &vm->va);
	mapping->bo_va = NULL;
	trace_gsgpu_vm_bo_unmap(bo_va, mapping);

	if (valid)
		list_add(&mapping->list, &vm->freed);
	else
		gsgpu_vm_free_mapping(adev, vm, mapping,
				       bo_va->last_pt_update);

	return 0;
}

/**
 * gsgpu_vm_bo_clear_mappings - remove all mappings in a specific range
 *
 * @adev: gsgpu_device pointer
 * @vm: VM structure to use
 * @saddr: start of the range
 * @size: size of the range
 *
 * Remove all mappings in a range, split them as appropriate.
 *
 * Returns:
 * 0 for success, error for failure.
 */
int gsgpu_vm_bo_clear_mappings(struct gsgpu_device *adev,
				struct gsgpu_vm *vm,
				uint64_t saddr, uint64_t size)
{
	struct gsgpu_bo_va_mapping *before, *after, *tmp, *next;
	LIST_HEAD(removed);
	uint64_t eaddr;

	eaddr = saddr + size - 1;
	saddr /= GSGPU_GPU_PAGE_SIZE;
	eaddr /= GSGPU_GPU_PAGE_SIZE;

	/* Allocate all the needed memory */
	before = kzalloc(sizeof(*before), GFP_KERNEL);
	if (!before)
		return -ENOMEM;
	INIT_LIST_HEAD(&before->list);

	after = kzalloc(sizeof(*after), GFP_KERNEL);
	if (!after) {
		kfree(before);
		return -ENOMEM;
	}
	INIT_LIST_HEAD(&after->list);

	/* Now gather all removed mappings */
	tmp = gsgpu_vm_it_iter_first(&vm->va, saddr, eaddr);
	while (tmp) {
		/* Remember mapping split at the start */
		if (tmp->start < saddr) {
			before->start = tmp->start;
			before->last = saddr - 1;
			before->offset = tmp->offset;
			before->flags = tmp->flags;
			before->bo_va = tmp->bo_va;
			list_add(&before->list, &tmp->bo_va->invalids);
		}

		/* Remember mapping split at the end */
		if (tmp->last > eaddr) {
			after->start = eaddr + 1;
			after->last = tmp->last;
			after->offset = tmp->offset;
			after->offset += (after->start - tmp->start) << PAGE_SHIFT;
			after->flags = tmp->flags;
			after->bo_va = tmp->bo_va;
			list_add(&after->list, &tmp->bo_va->invalids);
		}

		list_del(&tmp->list);
		list_add(&tmp->list, &removed);

		tmp = gsgpu_vm_it_iter_next(tmp, saddr, eaddr);
	}

	/* And free them up */
	list_for_each_entry_safe(tmp, next, &removed, list) {
		gsgpu_vm_it_remove(tmp, &vm->va);
		list_del(&tmp->list);

		if (tmp->start < saddr)
		    tmp->start = saddr;
		if (tmp->last > eaddr)
		    tmp->last = eaddr;

		tmp->bo_va = NULL;
		list_add(&tmp->list, &vm->freed);
		trace_gsgpu_vm_bo_unmap(NULL, tmp);
	}

	/* Insert partial mapping before the range */
	if (!list_empty(&before->list)) {
		gsgpu_vm_it_insert(before, &vm->va);
	} else {
		kfree(before);
	}

	/* Insert partial mapping after the range */
	if (!list_empty(&after->list)) {
		gsgpu_vm_it_insert(after, &vm->va);
	} else {
		kfree(after);
	}

	return 0;
}

/**
 * gsgpu_vm_bo_lookup_mapping - find mapping by address
 *
 * @vm: the requested VM
 * @addr: the address
 *
 * Find a mapping by it's address.
 *
 * Returns:
 * The gsgpu_bo_va_mapping matching for addr or NULL
 *
 */
struct gsgpu_bo_va_mapping *gsgpu_vm_bo_lookup_mapping(struct gsgpu_vm *vm,
							 uint64_t addr)
{
	return gsgpu_vm_it_iter_first(&vm->va, addr, addr);
}

/**
 * gsgpu_vm_bo_trace_cs - trace all reserved mappings
 *
 * @vm: the requested vm
 * @ticket: CS ticket
 *
 * Trace all mappings of BOs reserved during a command submission.
 */
void gsgpu_vm_bo_trace_cs(struct gsgpu_vm *vm, struct ww_acquire_ctx *ticket)
{
	struct gsgpu_bo_va_mapping *mapping;

	if (!trace_gsgpu_vm_bo_cs_enabled())
		return;

	for (mapping = gsgpu_vm_it_iter_first(&vm->va, 0, U64_MAX); mapping;
	     mapping = gsgpu_vm_it_iter_next(mapping, 0, U64_MAX)) {
		if (mapping->bo_va && mapping->bo_va->base.bo) {
			struct gsgpu_bo *bo;

			bo = mapping->bo_va->base.bo;
			if (dma_resv_locking_ctx(bo->tbo.base.resv) !=
			    ticket)
				continue;
		}

		trace_gsgpu_vm_bo_cs(mapping);
	}
}

/**
 * gsgpu_vm_bo_del - remove a bo from a specific vm
 *
 * @adev: gsgpu_device pointer
 * @bo_va: requested bo_va
 *
 * Remove @bo_va->bo from the requested vm.
 *
 * Object have to be reserved!
 */
void gsgpu_vm_bo_del(struct gsgpu_device *adev,
		      struct gsgpu_bo_va *bo_va)
{
	struct gsgpu_bo_va_mapping *mapping, *next;
	struct gsgpu_bo *bo = bo_va->base.bo;
	struct gsgpu_vm *vm = bo_va->base.vm;
	struct gsgpu_vm_bo_base **base;

	dma_resv_assert_held(vm->root.bo->tbo.base.resv);

	if (bo) {
		dma_resv_assert_held(bo->tbo.base.resv);
		if (bo->tbo.base.resv == vm->root.bo->tbo.base.resv)
			ttm_bo_set_bulk_move(&bo->tbo, NULL);

		for (base = &bo_va->base.bo->vm_bo; *base;
		     base = &(*base)->next) {
			if (*base != &bo_va->base)
				continue;

			*base = bo_va->base.next;
			break;
		}
	}

	spin_lock(&vm->status_lock);
	list_del(&bo_va->base.vm_status);
	spin_unlock(&vm->status_lock);

	list_for_each_entry_safe(mapping, next, &bo_va->valids, list) {
		list_del(&mapping->list);
		gsgpu_vm_it_remove(mapping, &vm->va);
		mapping->bo_va = NULL;
		trace_gsgpu_vm_bo_unmap(bo_va, mapping);
		list_add(&mapping->list, &vm->freed);
	}
	list_for_each_entry_safe(mapping, next, &bo_va->invalids, list) {
		list_del(&mapping->list);
		gsgpu_vm_it_remove(mapping, &vm->va);
		gsgpu_vm_free_mapping(adev, vm, mapping,
				       bo_va->last_pt_update);
	}

	dma_fence_put(bo_va->last_pt_update);

	kfree(bo_va);
}

/**
 * gsgpu_vm_evictable - check if we can evict a VM
 *
 * @bo: A page table of the VM.
 *
 * Check if it is possible to evict a VM.
 */
bool gsgpu_vm_evictable(struct gsgpu_bo *bo)
{
	struct gsgpu_vm_bo_base *bo_base = bo->vm_bo;

	/* Page tables of a destroyed VM can go away immediately */
	if (!bo_base || !bo_base->vm)
		return true;

	/* Don't evict VM page tables while they are busy */
	if (!dma_resv_test_signaled(bo->tbo.base.resv, DMA_RESV_USAGE_BOOKKEEP))
		return false;

	/* Try to block ongoing updates */
	if (!gsgpu_vm_eviction_trylock(bo_base->vm))
		return false;

	/* Don't evict VM page tables while they are updated */
	if (!dma_fence_is_signaled(bo_base->vm->last_unlocked)) {
		gsgpu_vm_eviction_unlock(bo_base->vm);
		return false;
	}

	bo_base->vm->evicting = true;
	gsgpu_vm_eviction_unlock(bo_base->vm);
	return true;
}

/**
 * gsgpu_vm_bo_invalidate - mark the bo as invalid
 *
 * @adev: gsgpu_device pointer
 * @bo: gsgpu buffer object
 * @evicted: is the BO evicted
 *
 * Mark @bo as invalid.
 */
void gsgpu_vm_bo_invalidate(struct gsgpu_device *adev,
			     struct gsgpu_bo *bo, bool evicted)
{
	struct gsgpu_vm_bo_base *bo_base;

	/* shadow bo doesn't have bo base, its validation needs its parent */
	if (bo->parent && (gsgpu_bo_shadowed(bo->parent) == bo))
		bo = bo->parent;

	for (bo_base = bo->vm_bo; bo_base; bo_base = bo_base->next) {
		struct gsgpu_vm *vm = bo_base->vm;

		if (evicted && bo->tbo.base.resv == vm->root.bo->tbo.base.resv) {
			gsgpu_vm_bo_evicted(bo_base);
			continue;
		}

		if (bo_base->moved)
			continue;
		bo_base->moved = true;

		if (bo->tbo.type == ttm_bo_type_kernel)
			gsgpu_vm_bo_relocated(bo_base);
		else if (bo->tbo.base.resv == vm->root.bo->tbo.base.resv)
			gsgpu_vm_bo_moved(bo_base);
		else
			gsgpu_vm_bo_invalidated(bo_base);
	}
}

/**
 * gsgpu_vm_get_block_size - calculate the shift of PTEs a block contains
 *
 * @vm_size: VM size in GB
 *
 * Returns:
 * The shift of PTEs a block contains
 */
static uint32_t gsgpu_vm_get_block_size(uint64_t vm_size)
{
	(void)(vm_size);
	return (GSGPU_PAGE_PTE_SHIFT);
}

/**
 * gsgpu_vm_adjust_size - adjust vm size, block size and fragment size
 *
 * @adev: gsgpu_device pointer
 * @min_vm_size: the minimum vm size in GB if it's set auto
 * @fragment_size_default: Default PTE fragment size
 * @max_level: max VMPT level
 * @max_bits: max address space size in bits
 *
 */
void gsgpu_vm_adjust_size(struct gsgpu_device *adev, uint32_t min_vm_size,
			   uint32_t fragment_size_default, unsigned max_level,
			   unsigned max_bits)
{
	// unsigned int max_size = 1 << (max_bits - 30);
	unsigned int vm_size;
	uint64_t tmp;

#if 0
	/* adjust vm size first */
	if (gsgpu_vm_size != -1) {
		vm_size = gsgpu_vm_size;
		if (vm_size > max_size) {
			dev_warn(adev->dev, "VM size (%d) too large, max is %u GB\n",
				 gsgpu_vm_size, max_size);
			vm_size = max_size;
		}
	} else {
		struct sysinfo si;
		unsigned int phys_ram_gb;

		/* Optimal VM size depends on the amount of physical
		 * RAM available. Underlying requirements and
		 * assumptions:
		 *
		 *  - Need to map system memory and VRAM from all GPUs
		 *     - VRAM from other GPUs not known here
		 *     - Assume VRAM <= system memory
		 *  - On GFX8 and older, VM space can be segmented for
		 *    different MTYPEs
		 *  - Need to allow room for fragmentation, guard pages etc.
		 *
		 * This adds up to a rough guess of system memory x3.
		 * Round up to power of two to maximize the available
		 * VM size with the given page table size.
		 */
		si_meminfo(&si);
		phys_ram_gb = ((uint64_t)si.totalram * si.mem_unit +
			       (1 << 30) - 1) >> 30;
		vm_size = roundup_pow_of_two(
			min(max(phys_ram_gb * 3, min_vm_size), max_size));
	}
#else
	vm_size = 1 << (max_bits - GSGPU_GB_SHIFT_BITS);
#endif
	adev->vm_manager.pde_pte_bytes = GSGPU_VM_PDE_PTE_BYTES;
	adev->vm_manager.max_pfn = (u64)vm_size <<
				(GSGPU_GB_SHIFT_BITS - GSGPU_GPU_PAGE_SHIFT);

	tmp = roundup_pow_of_two(adev->vm_manager.max_pfn);
	if (gsgpu_vm_block_size != -1)
		tmp >>= gsgpu_vm_block_size - GSGPU_PAGE_PTE_SHIFT;

	tmp = DIV_ROUND_UP(fls64(tmp), GSGPU_PAGE_PTE_SHIFT);
	adev->vm_manager.num_level = min(max_level, (unsigned)tmp);
	adev->vm_manager.root_level = GSGPU_VM_DIR0;
	adev->vm_manager.dir2_width = GSGPU_PAGE_PTE_SHIFT;
	adev->vm_manager.dir2_shift = GSGPU_GPU_PAGE_SHIFT;
	adev->vm_manager.dir1_shift = adev->vm_manager.dir2_shift + adev->vm_manager.dir2_width;
	adev->vm_manager.dir1_width = GSGPU_PAGE_PTE_SHIFT;
	adev->vm_manager.dir0_shift = adev->vm_manager.dir1_shift + adev->vm_manager.dir1_width;
	adev->vm_manager.dir0_width = max_bits - adev->vm_manager.dir0_shift;

	/* block size depends on vm size and hw setup*/
	if (gsgpu_vm_block_size != -1)
		adev->vm_manager.block_size = min((unsigned)gsgpu_vm_block_size, (unsigned)GSGPU_PAGE_PTE_SHIFT);
	else if (adev->vm_manager.num_level > 1)
		adev->vm_manager.block_size = GSGPU_PAGE_PTE_SHIFT;
	else
		adev->vm_manager.block_size = gsgpu_vm_get_block_size(vm_size);

	adev->vm_manager.fragment_size = fragment_size_default;

	DRM_INFO("vm size is %u GB, %u levels, block size is %u-bit, fragment size is %u-bit\n",
		 vm_size, adev->vm_manager.num_level + 1,
		 adev->vm_manager.block_size,
		 adev->vm_manager.fragment_size);
}

/**
 * gsgpu_vm_wait_idle - wait for the VM to become idle
 *
 * @vm: VM object to wait for
 * @timeout: timeout to wait for VM to become idle
 */
long gsgpu_vm_wait_idle(struct gsgpu_vm *vm, long timeout)
{
	timeout = dma_resv_wait_timeout(vm->root.bo->tbo.base.resv,
					DMA_RESV_USAGE_BOOKKEEP,
					true, timeout);
	if (timeout <= 0)
		return timeout;

	return dma_fence_wait_timeout(vm->last_unlocked, true, timeout);
}

/**
 * gsgpu_vm_init - initialize a vm instance
 *
 * @adev: gsgpu_device pointer
 * @vm: requested vm
 *
 * Init @vm fields.
 *
 * Returns:
 * 0 for success, error for failure.
 */
int gsgpu_vm_init(struct gsgpu_device *adev, struct gsgpu_vm *vm)
{
	struct gsgpu_bo *root_bo;
	struct gsgpu_bo_vm *root;
	int r; // , i;

	vm->va = RB_ROOT_CACHED;
	vm->reserved_vmid = NULL;
	INIT_LIST_HEAD(&vm->evicted);
	INIT_LIST_HEAD(&vm->relocated);
	INIT_LIST_HEAD(&vm->moved);
	INIT_LIST_HEAD(&vm->idle);
	INIT_LIST_HEAD(&vm->invalidated);
	spin_lock_init(&vm->status_lock);
	INIT_LIST_HEAD(&vm->freed);
	INIT_LIST_HEAD(&vm->done);
	INIT_LIST_HEAD(&vm->pt_freed);
	INIT_WORK(&vm->pt_free_work, gsgpu_vm_pt_free_work);

	/* create scheduler entities for page table updates */
	r = drm_sched_entity_init(&vm->immediate, DRM_SCHED_PRIORITY_NORMAL,
				  adev->vm_manager.vm_pte_scheds,
				  adev->vm_manager.vm_pte_num_scheds, NULL);
	if (r)
		return r;

	r = drm_sched_entity_init(&vm->delayed, DRM_SCHED_PRIORITY_NORMAL,
				  adev->vm_manager.vm_pte_scheds,
				  adev->vm_manager.vm_pte_num_scheds, NULL);
	if (r)
		goto error_free_immediate;

	vm->pte_support_ats = false;
	vm->is_compute_context = false;

	vm->use_cpu_for_update = adev->vm_manager.vm_update_mode;

	DRM_DEBUG_DRIVER("VM update mode is %s\n",
			 vm->use_cpu_for_update ? "CPU" : "XDMA");
	WARN_ONCE((vm->use_cpu_for_update &&
		   !gsgpu_gmc_vram_full_visible(&adev->gmc)),
		  "CPU update of VM recommended only for large BAR system\n");

	if (vm->use_cpu_for_update)
		vm->update_funcs = &gsgpu_vm_cpu_funcs;
	else
		vm->update_funcs = &gsgpu_vm_cpu_funcs; // &gsgpu_vm_sdma_funcs;
	vm->last_update = NULL;
	vm->last_unlocked = dma_fence_get_stub();
	vm->last_tlb_flush = dma_fence_get_stub();

	mutex_init(&vm->eviction_lock);
	vm->evicting = false;

	r = gsgpu_vm_pt_create(adev, vm, adev->vm_manager.root_level,
				false, &root);
	if (r)
		goto error_free_delayed;
	root_bo = &root->bo;
	r = gsgpu_bo_reserve(root_bo, true);
	if (r)
		goto error_free_root;

	r = dma_resv_reserve_fences(root_bo->tbo.base.resv, 1);
	if (r)
		goto error_unreserve;

	gsgpu_vm_bo_base_init(&vm->root, vm, root_bo);

	r = gsgpu_vm_pt_clear(adev, vm, root, false);
	if (r)
		goto error_unreserve;

	gsgpu_bo_unreserve(vm->root.bo);

	INIT_KFIFO(vm->faults);

	return 0;

error_unreserve:
	gsgpu_bo_unreserve(vm->root.bo);

error_free_root:
	gsgpu_bo_unref(&root->shadow);
	gsgpu_bo_unref(&root_bo);
	vm->root.bo = NULL;

error_free_delayed:
	dma_fence_put(vm->last_tlb_flush);
	dma_fence_put(vm->last_unlocked);
	drm_sched_entity_destroy(&vm->delayed);

error_free_immediate:
	drm_sched_entity_destroy(&vm->immediate);

	return r;
}

/**
 * gsgpu_vm_release_compute - release a compute vm
 * @adev: gsgpu_device pointer
 * @vm: a vm turned into compute vm by calling gsgpu_vm_make_compute
 *
 * This is a correspondant of gsgpu_vm_make_compute. It decouples compute
 * pasid from vm. Compute should stop use of vm after this call.
 */
void gsgpu_vm_release_compute(struct gsgpu_device *adev, struct gsgpu_vm *vm)
{
	gsgpu_vm_set_pasid(adev, vm, 0);
	vm->is_compute_context = false;
}

/**
 * gsgpu_vm_fini - tear down a vm instance
 *
 * @adev: gsgpu_device pointer
 * @vm: requested vm
 *
 * Tear down @vm.
 * Unbind the VM and remove all bos from the vm bo list
 */
void gsgpu_vm_fini(struct gsgpu_device *adev, struct gsgpu_vm *vm)
{
	struct gsgpu_bo_va_mapping *mapping, *tmp;
	struct gsgpu_bo *root;
	unsigned long flags;
	// int i;

	flush_work(&vm->pt_free_work);

	root = gsgpu_bo_ref(vm->root.bo);
	gsgpu_bo_reserve(root, true);
	gsgpu_vm_set_pasid(adev, vm, 0);
	dma_fence_wait(vm->last_unlocked, false);
	dma_fence_put(vm->last_unlocked);
	dma_fence_wait(vm->last_tlb_flush, false);
	/* Make sure that all fence callbacks have completed */
	spin_lock_irqsave(vm->last_tlb_flush->lock, flags);
	spin_unlock_irqrestore(vm->last_tlb_flush->lock, flags);
	dma_fence_put(vm->last_tlb_flush);

	list_for_each_entry_safe(mapping, tmp, &vm->freed, list) {
		list_del(&mapping->list);
		gsgpu_vm_free_mapping(adev, vm, mapping, NULL);
	}

	gsgpu_vm_pt_free_root(adev, vm);
	gsgpu_bo_unreserve(root);
	gsgpu_bo_unref(&root);
	WARN_ON(vm->root.bo);

	drm_sched_entity_destroy(&vm->immediate);
	drm_sched_entity_destroy(&vm->delayed);

	if (!RB_EMPTY_ROOT(&vm->va.rb_root)) {
		dev_err(adev->dev, "still active bo inside vm\n");
	}
	rbtree_postorder_for_each_entry_safe(mapping, tmp,
					     &vm->va.rb_root, rb) {
		/* Don't remove the mapping here, we don't want to trigger a
		 * rebalance and the tree is about to be destroyed anyway.
		 */
		list_del(&mapping->list);
		kfree(mapping);
	}

	dma_fence_put(vm->last_update);
	gsgpu_vmid_free_reserved(adev, vm);

	gsgpu_sema_free(adev, vm);
}

/**
 * gsgpu_vm_manager_init - init the VM manager
 *
 * @adev: gsgpu_device pointer
 *
 * Initialize the VM manager structures
 */
void gsgpu_vm_manager_init(struct gsgpu_device *adev)
{
	unsigned i;

	/* Concurrent flushes are only possible starting with Vega10 and
	 * are broken on Navi10 and Navi14.
	 */
	adev->vm_manager.concurrent_flush = false;
	gsgpu_vmid_mgr_init(adev);

	adev->vm_manager.fence_context =
		dma_fence_context_alloc(GSGPU_MAX_RINGS);
	for (i = 0; i < GSGPU_MAX_RINGS; ++i)
		adev->vm_manager.seqno[i] = 0;

	spin_lock_init(&adev->vm_manager.prt_lock);
	atomic_set(&adev->vm_manager.num_prt_users, 0);

	if (gsgpu_vm_update_mode == -1) {
		/* For asic with VF MMIO access protection
		 * avoid using CPU for VM table updates
		 */
		if (gsgpu_gmc_vram_full_visible(&adev->gmc))
			adev->vm_manager.vm_update_mode = 1;
		else
			adev->vm_manager.vm_update_mode = 0;
	} else
		adev->vm_manager.vm_update_mode = gsgpu_vm_update_mode;

	xa_init_flags(&adev->vm_manager.pasids, XA_FLAGS_LOCK_IRQ);
}

/**
 * gsgpu_vm_manager_fini - cleanup VM manager
 *
 * @adev: gsgpu_device pointer
 *
 * Cleanup the VM manager and free resources.
 */
void gsgpu_vm_manager_fini(struct gsgpu_device *adev)
{
	WARN_ON(!xa_empty(&adev->vm_manager.pasids));
	xa_destroy(&adev->vm_manager.pasids);

	gsgpu_vmid_mgr_fini(adev);
}

/**
 * gsgpu_vm_ioctl - Manages VMID reservation for vm hubs.
 *
 * @dev: drm device pointer
 * @data: drm_gsgpu_vm
 * @filp: drm file pointer
 *
 * Returns:
 * 0 for success, -errno for errors.
 */
int gsgpu_vm_ioctl(struct drm_device *dev, void *data, struct drm_file *filp)
{
	union drm_gsgpu_vm *args = data;
	struct gsgpu_device *adev = drm_to_adev(dev);
	struct gsgpu_fpriv *fpriv = filp->driver_priv;
	int r;

	switch (args->in.op) {
	case GSGPU_VM_OP_RESERVE_VMID:
		/* current, we only have requirement to reserve vmid */
		r = gsgpu_vmid_alloc_reserved(adev, &fpriv->vm);
		if (r)
			return r;
		break;
	case GSGPU_VM_OP_UNRESERVE_VMID:
		gsgpu_vmid_free_reserved(adev, &fpriv->vm);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/**
 * gsgpu_vm_get_task_info - Extracts task info for a PASID.
 *
 * @adev: drm device pointer
 * @pasid: PASID identifier for VM
 * @task_info: task_info to fill.
 */
void gsgpu_vm_get_task_info(struct gsgpu_device *adev, u32 pasid,
			 struct gsgpu_task_info *task_info)
{
	struct gsgpu_vm *vm;
	unsigned long flags;

	xa_lock_irqsave(&adev->vm_manager.pasids, flags);

	vm = xa_load(&adev->vm_manager.pasids, pasid);
	if (vm)
		*task_info = vm->task_info;

	xa_unlock_irqrestore(&adev->vm_manager.pasids, flags);
}

/**
 * gsgpu_vm_set_task_info - Sets VMs task info.
 *
 * @vm: vm for which to set the info
 */
void gsgpu_vm_set_task_info(struct gsgpu_vm *vm)
{
	if (vm->task_info.pid)
		return;

	vm->task_info.pid = current->pid;
	get_task_comm(vm->task_info.task_name, current);

	if (current->group_leader->mm != current->mm)
		return;

	vm->task_info.tgid = current->group_leader->pid;
	get_task_comm(vm->task_info.process_name, current->group_leader);
}

#if defined(CONFIG_DEBUG_FS)
/**
 * gsgpu_debugfs_vm_bo_info  - print BO info for the VM
 *
 * @vm: Requested VM for printing BO info
 * @m: debugfs file
 *
 * Print BO information in debugfs file for the VM
 */
void gsgpu_debugfs_vm_bo_info(struct gsgpu_vm *vm, struct seq_file *m)
{
	struct gsgpu_bo_va *bo_va, *tmp;
	u64 total_idle = 0;
	u64 total_evicted = 0;
	u64 total_relocated = 0;
	u64 total_moved = 0;
	u64 total_invalidated = 0;
	u64 total_done = 0;
	unsigned int total_idle_objs = 0;
	unsigned int total_evicted_objs = 0;
	unsigned int total_relocated_objs = 0;
	unsigned int total_moved_objs = 0;
	unsigned int total_invalidated_objs = 0;
	unsigned int total_done_objs = 0;
	unsigned int id = 0;

	spin_lock(&vm->status_lock);
	seq_puts(m, "\tIdle BOs:\n");
	list_for_each_entry_safe(bo_va, tmp, &vm->idle, base.vm_status) {
		if (!bo_va->base.bo)
			continue;
		total_idle += gsgpu_bo_print_info(id++, bo_va->base.bo, m);
	}
	total_idle_objs = id;
	id = 0;

	seq_puts(m, "\tEvicted BOs:\n");
	list_for_each_entry_safe(bo_va, tmp, &vm->evicted, base.vm_status) {
		if (!bo_va->base.bo)
			continue;
		total_evicted += gsgpu_bo_print_info(id++, bo_va->base.bo, m);
	}
	total_evicted_objs = id;
	id = 0;

	seq_puts(m, "\tRelocated BOs:\n");
	list_for_each_entry_safe(bo_va, tmp, &vm->relocated, base.vm_status) {
		if (!bo_va->base.bo)
			continue;
		total_relocated += gsgpu_bo_print_info(id++, bo_va->base.bo, m);
	}
	total_relocated_objs = id;
	id = 0;

	seq_puts(m, "\tMoved BOs:\n");
	list_for_each_entry_safe(bo_va, tmp, &vm->moved, base.vm_status) {
		if (!bo_va->base.bo)
			continue;
		total_moved += gsgpu_bo_print_info(id++, bo_va->base.bo, m);
	}
	total_moved_objs = id;
	id = 0;

	seq_puts(m, "\tInvalidated BOs:\n");
	list_for_each_entry_safe(bo_va, tmp, &vm->invalidated, base.vm_status) {
		if (!bo_va->base.bo)
			continue;
		total_invalidated += gsgpu_bo_print_info(id++,	bo_va->base.bo, m);
	}
	total_invalidated_objs = id;
	id = 0;

	seq_puts(m, "\tDone BOs:\n");
	list_for_each_entry_safe(bo_va, tmp, &vm->done, base.vm_status) {
		if (!bo_va->base.bo)
			continue;
		total_done += gsgpu_bo_print_info(id++, bo_va->base.bo, m);
	}
	spin_unlock(&vm->status_lock);
	total_done_objs = id;

	seq_printf(m, "\tTotal idle size:        %12lld\tobjs:\t%d\n", total_idle,
		   total_idle_objs);
	seq_printf(m, "\tTotal evicted size:     %12lld\tobjs:\t%d\n", total_evicted,
		   total_evicted_objs);
	seq_printf(m, "\tTotal relocated size:   %12lld\tobjs:\t%d\n", total_relocated,
		   total_relocated_objs);
	seq_printf(m, "\tTotal moved size:       %12lld\tobjs:\t%d\n", total_moved,
		   total_moved_objs);
	seq_printf(m, "\tTotal invalidated size: %12lld\tobjs:\t%d\n", total_invalidated,
		   total_invalidated_objs);
	seq_printf(m, "\tTotal done size:        %12lld\tobjs:\t%d\n", total_done,
		   total_done_objs);
}
#endif
