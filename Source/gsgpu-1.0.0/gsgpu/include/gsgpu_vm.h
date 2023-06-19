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
#ifndef __GSGPU_VM_H__
#define __GSGPU_VM_H__

#include <linux/idr.h>
#include <linux/kfifo.h>
#include <linux/rbtree.h>
#include <drm/gpu_scheduler.h>
#include <drm/drm_file.h>
#include <drm/ttm/ttm_bo.h>
#include <linux/sched/mm.h>

#include "gsgpu_sync.h"
#include "gsgpu_ring.h"
#include "gsgpu_ids.h"

struct gsgpu_bo_va;
struct gsgpu_job;
struct gsgpu_bo_list_entry;
struct gsgpu_bo_vm;

#define GSGPU_VM_PDE_PTE_BYTES      8

/*
 * GPUVM handling
 */

/* Maximum number of PTEs the hardware can write with one command */
#define GSGPU_VM_MAX_UPDATE_SIZE	(1024ull)

/* number of entries in page table */
#define GSGPU_VM_PTE_COUNT(adev) (1 << (adev)->vm_manager.block_size)

#define GSGPU_PTE_VALID	(1ULL << 0)
#define GSGPU_PTE_SYSTEM	(1ULL << 1)
#define GSGPU_PTE_SNOOPED	(1ULL << 2)

/* comprssed pte flags use bit 4 to 6 */
#define GSGPU_PTE_COMPRESSED_SHIFT (5)

/* RV+ */
#define GSGPU_PTE_TMZ		(1ULL << 3)

/* VI only */
#define GSGPU_PTE_EXECUTABLE	(1ULL << 4)

#define GSGPU_PTE_READABLE	(1ULL << 5)
#define GSGPU_PTE_WRITEABLE	(1ULL << 6)

#define GSGPU_PTE_FRAG(x)	((x & 0x1fULL) << 7)

/* TILED for VEGA10, reserved for older ASICs  */
#define GSGPU_PTE_PRT		(1ULL << 51)

/* PDE is handled as PTE for VEGA10 */
#define GSGPU_PDE_PTE		(1ULL << 54)

#define GSGPU_PTE_LOG          (1ULL << 55)

/* PTE is handled as PDE for VEGA10 (Translate Further) */
#define GSGPU_PTE_TF		(1ULL << 56)

/* MALL noalloc for sienna_cichlid, reserved for older ASICs  */
#define GSGPU_PTE_NOALLOC	(1ULL << 58)

/* PDE Block Fragment Size for VEGA10 */
#define GSGPU_PDE_BFS(a)	((uint64_t)a << 59)


/* For GFX9 */
#define GSGPU_PTE_MTYPE_VG10(a)	((uint64_t)(a) << 57)
#define GSGPU_PTE_MTYPE_VG10_MASK	GSGPU_PTE_MTYPE_VG10(3ULL)

#define GSGPU_MTYPE_NC 0
#define GSGPU_MTYPE_CC 2

#define GSGPU_PTE_DEFAULT_ATC  (GSGPU_PTE_SYSTEM      \
                                | GSGPU_PTE_SNOOPED    \
                                | GSGPU_PTE_EXECUTABLE \
                                | GSGPU_PTE_READABLE   \
                                | GSGPU_PTE_WRITEABLE  \
                                | GSGPU_PTE_MTYPE_VG10(GSGPU_MTYPE_CC))

/* gfx10 */
#define GSGPU_PTE_MTYPE_NV10(a)       ((uint64_t)(a) << 48)
#define GSGPU_PTE_MTYPE_NV10_MASK     GSGPU_PTE_MTYPE_NV10(7ULL)

/* How to program VM fault handling */
#define GSGPU_VM_FAULT_STOP_NEVER	0
#define GSGPU_VM_FAULT_STOP_FIRST	1
#define GSGPU_VM_FAULT_STOP_ALWAYS	2

/* Reserve 4MB VRAM for page tables */
#define GSGPU_VM_RESERVED_VRAM		(8ULL << 20)

/* Reserve 2MB at top/bottom of address space for kernel use */
#define GSGPU_VA_RESERVED_SIZE			(2ULL << 20)

/* VMPT level enumerate, and the hiberachy is:
 * DIR0->DIR1->DIR2
 */
enum gsgpu_vm_level {
	GSGPU_VM_DIR0,
	GSGPU_VM_DIR1,
	GSGPU_VM_DIR2
};

/* base structure for tracking BO usage in a VM */
struct gsgpu_vm_bo_base {
	/* constant after initialization */
	struct gsgpu_vm		*vm;
	struct gsgpu_bo		*bo;

	/* protected by bo being reserved */
	struct gsgpu_vm_bo_base	*next;

	/* protected by spinlock */
	struct list_head		vm_status;

	/* protected by the BO being reserved */
	bool				moved;
};

/* provided by hw blocks that can write ptes, e.g., sdma */
struct gsgpu_vm_pte_funcs {
	/* number of dw to reserve per operation */
	unsigned	copy_pte_num_dw;

	/* number of dw to reserve per operation */
	unsigned	set_pte_pde_num_dw;

	/* copy pte entries from GART */
	void (*copy_pte)(struct gsgpu_ib *ib,
			 uint64_t pe, uint64_t src,
			 unsigned count);

	/* write pte one entry at a time with addr mapping */
	void (*write_pte)(struct gsgpu_ib *ib, uint64_t pe,
			  uint64_t value, unsigned count,
			  uint32_t incr);
	/* for linear pte/pde updates without addr mapping */
	void (*set_pte_pde)(struct gsgpu_ib *ib,
			    uint64_t pe,
			    uint64_t addr, unsigned count,
			    uint32_t incr, uint64_t flags);
};

struct gsgpu_task_info {
	char	process_name[TASK_COMM_LEN];
	char	task_name[TASK_COMM_LEN];
	pid_t	pid;
	pid_t	tgid;
};

/**
 * struct gsgpu_vm_update_params
 *
 * Encapsulate some VM table update parameters to reduce
 * the number of function parameters
 *
 */
struct gsgpu_vm_update_params {

	/**
	 * @adev: gsgpu device we do this update for
	 */
	struct gsgpu_device *adev;

	/**
	 * @vm: optional gsgpu_vm we do this update for
	 */
	struct gsgpu_vm *vm;

	/**
	 * @immediate: if changes should be made immediately
	 */
	bool immediate;

	/**
	 * @unlocked: true if the root BO is not locked
	 */
	bool unlocked;

	/**
	 * @pages_addr:
	 *
	 * DMA addresses to use for mapping
	 */
	dma_addr_t *pages_addr;

	/**
	 * @job: job to used for hw submission
	 */
	struct gsgpu_job *job;

	/**
	 * @num_dw_left: number of dw left for the IB
	 */
	unsigned int num_dw_left;

	/**
	 * @table_freed: return true if page table is freed when updating
	 */
	bool table_freed;
};

struct gsgpu_vm_update_funcs {
	int (*map_table)(struct gsgpu_bo_vm *bo);
	int (*prepare)(struct gsgpu_vm_update_params *p, struct dma_resv *resv,
		       enum gsgpu_sync_mode sync_mode);
	int (*update)(struct gsgpu_vm_update_params *p,
		      struct gsgpu_bo_vm *bo, uint64_t pe, uint64_t addr,
		      unsigned count, uint32_t incr, uint64_t flags);
	int (*commit)(struct gsgpu_vm_update_params *p,
		      struct dma_fence **fence);
};

struct gsgpu_vm {
	/* tree of virtual addresses mapped */
	struct rb_root_cached	va;

	/* Lock to prevent eviction while we are updating page tables
	 * use vm_eviction_lock/unlock(vm)
	 */
	struct mutex		eviction_lock;
	bool			evicting;
	unsigned int		saved_flags;

	/* Lock to protect vm_bo add/del/move on all lists of vm */
	spinlock_t		status_lock;

	/* BOs who needs a validation */
	struct list_head	evicted;

	/* PT BOs which relocated and their parent need an update */
	struct list_head	relocated;

	/* per VM BOs moved, but not yet updated in the PT */
	struct list_head	moved;

	/* All BOs of this VM not currently in the state machine */
	struct list_head	idle;

	/* regular invalidated BOs, but not yet updated in the PT */
	struct list_head	invalidated;

	/* BO mappings freed, but not yet updated in the PT */
	struct list_head	freed;

	/* BOs which are invalidated, has been updated in the PTs */
	struct list_head        done;

	/* PT BOs scheduled to free and fill with zero if vm_resv is not hold */
	struct list_head	pt_freed;
	struct work_struct	pt_free_work;

	/* contains the page directory */
	struct gsgpu_vm_bo_base     root;
	struct dma_fence	*last_update;

	/* Scheduler entities for page table updates */
	struct drm_sched_entity	immediate;
	struct drm_sched_entity	delayed;

	/* Last finished delayed update */
	atomic64_t		tlb_seq;
	struct dma_fence	*last_tlb_flush;

	/* Last unlocked submission to the scheduler entities */
	struct dma_fence	*last_unlocked;

	unsigned int		pasid;
	struct gsgpu_vmid	*reserved_vmid;

	/* Flag to indicate if VM tables are updated by CPU or GPU (XDMA) */
	bool					use_cpu_for_update;

	/* Functions to use for VM table updates */
	const struct gsgpu_vm_update_funcs	*update_funcs;

	/* Flag to indicate ATS support from PTE for GFX9 */
	bool			pte_support_ats;

	/* Up to 128 pending retry page faults */
	DECLARE_KFIFO(faults, u64, 128);

	/* Valid while the PD is reserved or fenced */
	uint64_t		pd_phys_addr;

	/* Some basic info about the task */
	struct gsgpu_task_info task_info;

	/* Store positions of group of BOs */
	struct ttm_lru_bulk_move lru_bulk_move;
	/* Flag to indicate if VM is used for compute */
	bool			is_compute_context;
};

struct gsgpu_vm_manager {
	/* Handling of VMIDs */
	struct gsgpu_vmid_mgr			id_mgr;
	unsigned int				first_kfd_vmid;
	bool					concurrent_flush;

	/* Handling of VM fences */
	u64					fence_context;
	unsigned				seqno[GSGPU_MAX_RINGS];

	uint32_t                pde_pte_bytes;

	uint64_t				max_pfn;
	uint32_t				num_level;
	uint32_t				block_size;
	uint32_t				fragment_size;
	enum gsgpu_vm_level			root_level;

	uint32_t dir0_shift, dir0_width;
	uint32_t dir1_shift, dir1_width;
	uint32_t dir2_shift, dir2_width;

	/* vram base address for page table entry  */
	u64					vram_base_offset;
	/* vm pte handling */
	const struct gsgpu_vm_pte_funcs	*vm_pte_funcs;
	struct drm_gpu_scheduler		*vm_pte_scheds[GSGPU_MAX_RINGS];
	unsigned				vm_pte_num_scheds;
	struct gsgpu_ring			*page_fault;

	/* partial resident texture handling */
	spinlock_t				prt_lock;
	atomic_t				num_prt_users;

	/* controls how VM page tables are updated for Graphics and Compute.
	 * BIT0[= 0] Graphics updated by XDMA [= 1] by CPU
	 * BIT1[= 0] Compute updated by XDMA [= 1] by CPU
	 */
	int					vm_update_mode;

	/* PASID to VM mapping, will be used in interrupt context to
	 * look up VM of a page fault
	 */
	struct xarray				pasids;
};

struct gsgpu_bo_va_mapping;

#define gsgpu_vm_copy_pte(adev, ib, pe, src, count) ((adev)->vm_manager.vm_pte_funcs->copy_pte((ib), (pe), (src), (count)))
#define gsgpu_vm_write_pte(adev, ib, pe, value, count, incr) ((adev)->vm_manager.vm_pte_funcs->write_pte((ib), (pe), (value), (count), (incr)))
#define gsgpu_vm_set_pte_pde(adev, ib, pe, addr, count, incr, flags) ((adev)->vm_manager.vm_pte_funcs->set_pte_pde((ib), (pe), (addr), (count), (incr), (flags)))

extern const struct gsgpu_vm_update_funcs gsgpu_vm_cpu_funcs;
// extern const struct gsgpu_vm_update_funcs gsgpu_vm_sdma_funcs;

void gsgpu_vm_manager_init(struct gsgpu_device *adev);
void gsgpu_vm_manager_fini(struct gsgpu_device *adev);

int gsgpu_vm_set_pasid(struct gsgpu_device *adev, struct gsgpu_vm *vm,
			u32 pasid);

long gsgpu_vm_wait_idle(struct gsgpu_vm *vm, long timeout);
int gsgpu_vm_init(struct gsgpu_device *adev, struct gsgpu_vm *vm);
void gsgpu_vm_release_compute(struct gsgpu_device *adev, struct gsgpu_vm *vm);
void gsgpu_vm_fini(struct gsgpu_device *adev, struct gsgpu_vm *vm);
void gsgpu_vm_get_pd_bo(struct gsgpu_vm *vm,
			 struct list_head *validated,
			 struct gsgpu_bo_list_entry *entry);
bool gsgpu_vm_ready(struct gsgpu_vm *vm);
int gsgpu_vm_validate_pt_bos(struct gsgpu_device *adev, struct gsgpu_vm *vm,
			      int (*callback)(void *p, struct gsgpu_bo *bo),
			      void *param);
int gsgpu_vm_flush(struct gsgpu_ring *ring, struct gsgpu_job *job, bool need_pipe_sync);
int gsgpu_vm_update_pdes(struct gsgpu_device *adev,
			  struct gsgpu_vm *vm, bool immediate);
int gsgpu_vm_clear_freed(struct gsgpu_device *adev,
			  struct gsgpu_vm *vm,
			  struct dma_fence **fence);
int gsgpu_vm_handle_moved(struct gsgpu_device *adev,
			   struct gsgpu_vm *vm);
void gsgpu_vm_bo_base_init(struct gsgpu_vm_bo_base *base,
			    struct gsgpu_vm *vm, struct gsgpu_bo *bo);
int gsgpu_vm_update_range(struct gsgpu_device *adev, struct gsgpu_vm *vm,
			   bool immediate, bool unlocked, bool flush_tlb,
			   struct dma_resv *resv, uint64_t start, uint64_t last,
			   uint64_t flags, uint64_t offset, uint64_t vram_base,
			   struct ttm_resource *res, dma_addr_t *pages_addr,
			   struct dma_fence **fence);
int gsgpu_vm_bo_update(struct gsgpu_device *adev,
			struct gsgpu_bo_va *bo_va,
			bool clear);
bool gsgpu_vm_evictable(struct gsgpu_bo *bo);
void gsgpu_vm_bo_invalidate(struct gsgpu_device *adev,
			     struct gsgpu_bo *bo, bool evicted);
uint64_t gsgpu_vm_map_gart(const dma_addr_t *pages_addr, uint64_t addr);
struct gsgpu_bo_va *gsgpu_vm_bo_find(struct gsgpu_vm *vm,
				       struct gsgpu_bo *bo);
struct gsgpu_bo_va *gsgpu_vm_bo_add(struct gsgpu_device *adev,
				      struct gsgpu_vm *vm,
				      struct gsgpu_bo *bo);
int gsgpu_vm_bo_map(struct gsgpu_device *adev,
		     struct gsgpu_bo_va *bo_va,
		     uint64_t addr, uint64_t offset,
		     uint64_t size, uint64_t flags);
int gsgpu_vm_bo_replace_map(struct gsgpu_device *adev,
			     struct gsgpu_bo_va *bo_va,
			     uint64_t addr, uint64_t offset,
			     uint64_t size, uint64_t flags);
int gsgpu_vm_bo_unmap(struct gsgpu_device *adev,
		       struct gsgpu_bo_va *bo_va,
		       uint64_t addr);
int gsgpu_vm_bo_clear_mappings(struct gsgpu_device *adev,
				struct gsgpu_vm *vm,
				uint64_t saddr, uint64_t size);
struct gsgpu_bo_va_mapping *gsgpu_vm_bo_lookup_mapping(struct gsgpu_vm *vm,
							 uint64_t addr);
void gsgpu_vm_bo_trace_cs(struct gsgpu_vm *vm, struct ww_acquire_ctx *ticket);
void gsgpu_vm_bo_del(struct gsgpu_device *adev,
		      struct gsgpu_bo_va *bo_va);
void gsgpu_vm_adjust_size(struct gsgpu_device *adev, uint32_t min_vm_size,
			   uint32_t fragment_size_default, unsigned max_level,
			   unsigned max_bits);
int gsgpu_vm_ioctl(struct drm_device *dev, void *data, struct drm_file *filp);
bool gsgpu_vm_need_pipeline_sync(struct gsgpu_ring *ring,
				  struct gsgpu_job *job);

void gsgpu_vm_get_task_info(struct gsgpu_device *adev, u32 pasid,
			     struct gsgpu_task_info *task_info);

void gsgpu_vm_set_task_info(struct gsgpu_vm *vm);

void gsgpu_vm_move_to_lru_tail(struct gsgpu_device *adev,
				struct gsgpu_vm *vm);
void gsgpu_vm_get_memory(struct gsgpu_vm *vm, uint64_t *vram_mem,
				uint64_t *gtt_mem, uint64_t *cpu_mem);

int gsgpu_vm_pt_clear(struct gsgpu_device *adev, struct gsgpu_vm *vm,
		       struct gsgpu_bo_vm *vmbo, bool immediate);
int gsgpu_vm_pt_create(struct gsgpu_device *adev, struct gsgpu_vm *vm,
			int level, bool immediate, struct gsgpu_bo_vm **vmbo);
void gsgpu_vm_pt_free_root(struct gsgpu_device *adev, struct gsgpu_vm *vm);
bool gsgpu_vm_pt_is_root_clean(struct gsgpu_device *adev,
				struct gsgpu_vm *vm);

int gsgpu_vm_pde_update(struct gsgpu_vm_update_params *params,
			 struct gsgpu_vm_bo_base *entry);
int gsgpu_vm_ptes_update(struct gsgpu_vm_update_params *params,
			  uint64_t start, uint64_t end,
			  uint64_t dst, uint64_t flags);
void gsgpu_vm_pt_free_work(struct work_struct *work);

#if defined(CONFIG_DEBUG_FS)
void gsgpu_debugfs_vm_bo_info(struct gsgpu_vm *vm, struct seq_file *m);
#endif

/**
 * gsgpu_vm_tlb_seq - return tlb flush sequence number
 * @vm: the gsgpu_vm structure to query
 *
 * Returns the tlb flush sequence number which indicates that the VM TLBs needs
 * to be invalidated whenever the sequence number change.
 */
static inline uint64_t gsgpu_vm_tlb_seq(struct gsgpu_vm *vm)
{
	unsigned long flags;
	spinlock_t *lock;

	/*
	 * Workaround to stop racing between the fence signaling and handling
	 * the cb. The lock is static after initially setting it up, just make
	 * sure that the dma_fence structure isn't freed up.
	 */
	rcu_read_lock();
	lock = vm->last_tlb_flush->lock;
	rcu_read_unlock();

	spin_lock_irqsave(lock, flags);
	spin_unlock_irqrestore(lock, flags);

	return atomic64_read(&vm->tlb_seq);
}

/*
 * vm eviction_lock can be taken in MMU notifiers. Make sure no reclaim-FS
 * happens while holding this lock anywhere to prevent deadlocks when
 * an MMU notifier runs in reclaim-FS context.
 */
static inline void gsgpu_vm_eviction_lock(struct gsgpu_vm *vm)
{
	mutex_lock(&vm->eviction_lock);
	vm->saved_flags = memalloc_noreclaim_save();
}

static inline bool gsgpu_vm_eviction_trylock(struct gsgpu_vm *vm)
{
	if (mutex_trylock(&vm->eviction_lock)) {
		vm->saved_flags = memalloc_noreclaim_save();
		return true;
	}
	return false;
}

static inline void gsgpu_vm_eviction_unlock(struct gsgpu_vm *vm)
{
	memalloc_noreclaim_restore(vm->saved_flags);
	mutex_unlock(&vm->eviction_lock);
}

#endif
