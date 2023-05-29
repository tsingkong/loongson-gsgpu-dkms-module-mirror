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
#ifndef __GSGPU_RING_H__
#define __GSGPU_RING_H__

#include <drm/gsgpu_drm.h>
#include <drm/gpu_scheduler.h>
#include <drm/drm_print.h>

struct gsgpu_device;
struct gsgpu_ring;
struct gsgpu_ib;
struct gsgpu_cs_parser;
struct gsgpu_job;
struct gsgpu_vm;

/* max number of rings */
#define GSGPU_MAX_RINGS		28
#define GSGPU_MAX_HWIP_RINGS		8
#define GSGPU_MAX_GFX_RINGS		2

enum gsgpu_ring_priority_level {
	GSGPU_RING_PRIO_0,
	GSGPU_RING_PRIO_1,
	GSGPU_RING_PRIO_DEFAULT = 1,
	GSGPU_RING_PRIO_2,
	GSGPU_RING_PRIO_MAX
};

/* some special values for the owner field */
#define GSGPU_FENCE_OWNER_UNDEFINED	((void *)0ul)
#define GSGPU_FENCE_OWNER_VM		((void *)1ul)
#define GSGPU_FENCE_OWNER_KFD		((void *)2ul)

#define GSGPU_FENCE_FLAG_64BIT         (1 << 0)
#define GSGPU_FENCE_FLAG_INT           (1 << 1)
#define GSGPU_FENCE_FLAG_TC_WB_ONLY    (1 << 2)
#define GSGPU_FENCE_FLAG_EXEC          (1 << 3)

#define to_gsgpu_ring(s) container_of((s), struct gsgpu_ring, sched)

#define GSGPU_IB_POOL_SIZE	(1024 * 1024)

enum gsgpu_ring_type {
	GSGPU_RING_TYPE_GFX		= GSGPU_HW_IP_GFX,
	GSGPU_RING_TYPE_XDMA
};

enum gsgpu_ib_pool_type {
	/* Normal submissions to the top of the pipeline. */
	GSGPU_IB_POOL_DELAYED,
	/* Immediate submissions to the bottom of the pipeline. */
	GSGPU_IB_POOL_IMMEDIATE,
	/* Direct submission to the ring buffer during init and reset. */
	GSGPU_IB_POOL_DIRECT,

	GSGPU_IB_POOL_MAX
};

struct gsgpu_ib {
	struct gsgpu_sa_bo		*sa_bo;
	uint32_t			length_dw;
	uint64_t			gpu_addr;
	uint32_t			*ptr;
	uint32_t			flags;
};

struct gsgpu_sched {
	u32				num_scheds;
	struct drm_gpu_scheduler	*sched[GSGPU_MAX_HWIP_RINGS];
};

/*
 * Fences.
 */
struct gsgpu_fence_driver {
	uint64_t			gpu_addr;
	volatile uint32_t		*cpu_addr;
	/* sync_seq is protected by ring emission lock */
	uint32_t			sync_seq;
	atomic_t			last_seq;
	bool				initialized;
	struct gsgpu_irq_src		*irq_src;
	unsigned			irq_type;
	struct timer_list		fallback_timer;
	unsigned			num_fences_mask;
	spinlock_t			lock;
	struct dma_fence		**fences;
};

extern const struct drm_sched_backend_ops gsgpu_sched_ops;

void gsgpu_fence_driver_clear_job_fences(struct gsgpu_ring *ring);
void gsgpu_fence_driver_force_completion(struct gsgpu_ring *ring);

int gsgpu_fence_driver_init_ring(struct gsgpu_ring *ring);
int gsgpu_fence_driver_start_ring(struct gsgpu_ring *ring,
				   struct gsgpu_irq_src *irq_src,
				   unsigned irq_type);
void gsgpu_fence_driver_hw_init(struct gsgpu_device *adev);
void gsgpu_fence_driver_hw_fini(struct gsgpu_device *adev);
int gsgpu_fence_driver_sw_init(struct gsgpu_device *adev);
void gsgpu_fence_driver_sw_fini(struct gsgpu_device *adev);
int gsgpu_fence_emit(struct gsgpu_ring *ring, struct dma_fence **fence, struct gsgpu_job *job,
		      unsigned flags);
int gsgpu_fence_emit_polling(struct gsgpu_ring *ring, uint32_t *s,
			      uint32_t timeout);
bool gsgpu_fence_process(struct gsgpu_ring *ring);
int gsgpu_fence_wait_empty(struct gsgpu_ring *ring);
signed long gsgpu_fence_wait_polling(struct gsgpu_ring *ring,
				      uint32_t wait_seq,
				      signed long timeout);
unsigned gsgpu_fence_count_emitted(struct gsgpu_ring *ring);

void gsgpu_fence_driver_isr_toggle(struct gsgpu_device *adev, bool stop);

u64 gsgpu_fence_last_unsignaled_time_us(struct gsgpu_ring *ring);
void gsgpu_fence_update_start_timestamp(struct gsgpu_ring *ring, uint32_t seq,
					 ktime_t timestamp);

/*
 * Rings.
 */

/* provided by hw blocks that expose a ring buffer for commands */
struct gsgpu_ring_funcs {
	enum gsgpu_ring_type	type;
	uint32_t		align_mask;
	u32			nop;
	bool			support_64bit_ptrs;
	bool			no_user_fence;
	bool			secure_submission_supported;
	unsigned		vmhub;
	unsigned		extra_dw;

	/* ring read/write ptr handling */
	u64 (*get_rptr)(struct gsgpu_ring *ring);
	u64 (*get_wptr)(struct gsgpu_ring *ring);
	void (*set_wptr)(struct gsgpu_ring *ring);
	/* validating and patching of IBs */
	int (*parse_cs)(struct gsgpu_cs_parser *p,
			struct gsgpu_job *job,
			struct gsgpu_ib *ib);
	int (*patch_cs_in_place)(struct gsgpu_cs_parser *p,
				 struct gsgpu_job *job,
				 struct gsgpu_ib *ib);
	/* constants to calculate how many DW are needed for an emit */
	unsigned emit_frame_size;
	unsigned emit_ib_size;
	/* command emit functions */
	void (*emit_ib)(struct gsgpu_ring *ring,
			struct gsgpu_job *job,
			struct gsgpu_ib *ib,
			uint32_t flags);
	void (*emit_fence)(struct gsgpu_ring *ring, uint64_t addr,
			   uint64_t seq, unsigned flags);
	void (*emit_pipeline_sync)(struct gsgpu_ring *ring);
	void (*emit_vm_flush)(struct gsgpu_ring *ring, unsigned vmid,
			      uint64_t pd_addr);
	/* testing functions */
	int (*test_ring)(struct gsgpu_ring *ring);
	int (*test_ib)(struct gsgpu_ring *ring, long timeout);
	int (*test_xdma)(struct gsgpu_ring *ring, long timeout);
	/* insert NOP packets */
	void (*insert_nop)(struct gsgpu_ring *ring, uint32_t count);
	void (*insert_start)(struct gsgpu_ring *ring);
	void (*insert_end)(struct gsgpu_ring *ring);
	/* pad the indirect buffer to the necessary number of dw */
	void (*pad_ib)(struct gsgpu_ring *ring, struct gsgpu_ib *ib);
	unsigned (*init_cond_exec)(struct gsgpu_ring *ring);
	void (*patch_cond_exec)(struct gsgpu_ring *ring, unsigned offset);
	/* note usage for clock and power gating */
	void (*begin_use)(struct gsgpu_ring *ring);
	void (*end_use)(struct gsgpu_ring *ring);
	void (*emit_switch_buffer) (struct gsgpu_ring *ring);
	void (*emit_cntxcntl) (struct gsgpu_ring *ring, uint32_t flags);
	void (*emit_rreg)(struct gsgpu_ring *ring, uint32_t reg,
			  uint32_t reg_val_offs);
	void (*emit_wreg)(struct gsgpu_ring *ring, uint32_t reg, uint32_t val);
	void (*emit_reg_wait)(struct gsgpu_ring *ring, uint32_t reg,
			      uint32_t val, uint32_t mask);
	void (*emit_reg_write_reg_wait)(struct gsgpu_ring *ring,
					uint32_t reg0, uint32_t reg1,
					uint32_t ref, uint32_t mask);
	void (*emit_frame_cntl)(struct gsgpu_ring *ring, bool start,
				bool secure);
	/* Try to soft recover the ring to make the fence signal */
	void (*soft_recovery)(struct gsgpu_ring *ring, unsigned vmid);
	int (*preempt_ib)(struct gsgpu_ring *ring);
	void (*emit_mem_sync)(struct gsgpu_ring *ring);
	void (*emit_wave_limit)(struct gsgpu_ring *ring, bool enable);
};

struct gsgpu_ring {
	struct gsgpu_device		*adev;
	const struct gsgpu_ring_funcs	*funcs;
	struct gsgpu_fence_driver	fence_drv;
	struct drm_gpu_scheduler	sched;

	struct gsgpu_bo	*ring_obj;
	volatile uint32_t	*ring;
	unsigned		rptr_offs;
	u64			rptr_gpu_addr;
	volatile u32		*rptr_cpu_addr;
	u64			wptr;
	u64			wptr_old;
	unsigned		ring_size;
	unsigned		max_dw;
	int			count_dw;
	uint64_t		gpu_addr;
	uint64_t		ptr_mask;
	uint32_t		buf_mask;
	u32			idx;
	u32			me;
	u32			pipe;
	u32			queue;
	bool			use_pollmem;
	unsigned		wptr_offs;
	u64			wptr_gpu_addr;
	volatile u32		*wptr_cpu_addr;
	unsigned		fence_offs;
	u64			fence_gpu_addr;
	volatile u32		*fence_cpu_addr;
	uint64_t		current_ctx;
	char			name[16];
	u32                     trail_seq;
	unsigned		trail_fence_offs;
	u64			trail_fence_gpu_addr;
	volatile u32		*trail_fence_cpu_addr;
	unsigned		cond_exe_offs;
	u64			cond_exe_gpu_addr;
	volatile u32		*cond_exe_cpu_addr;
	unsigned		vm_inv_eng;
	struct dma_fence	*vmid_wait;
	bool			no_scheduler;
	int			hw_prio;
	unsigned 		num_hw_submission;
	atomic_t		*sched_score;

	bool            is_sw_ring;
	unsigned int    entry_index;

};

#define gsgpu_ring_parse_cs(r, p, job, ib) ((r)->funcs->parse_cs((p), (job), (ib)))
#define gsgpu_ring_patch_cs_in_place(r, p, job, ib) ((r)->funcs->patch_cs_in_place((p), (job), (ib)))
#define gsgpu_ring_test_ring(r) (r)->funcs->test_ring((r))
#define gsgpu_ring_test_ib(r, t) ((r)->funcs->test_ib ? (r)->funcs->test_ib((r), (t)) : 0)
#define gsgpu_ring_get_rptr(r) (r)->funcs->get_rptr((r))
#define gsgpu_ring_get_wptr(r) (r)->funcs->get_wptr((r))
#define gsgpu_ring_set_wptr(r) (r)->funcs->set_wptr((r))
#define gsgpu_ring_emit_ib(r, job, ib, flags) ((r)->funcs->emit_ib((r), (job), (ib), (flags)))
#define gsgpu_ring_emit_pipeline_sync(r) (r)->funcs->emit_pipeline_sync((r))
#define gsgpu_ring_emit_vm_flush(r, vmid, addr) (r)->funcs->emit_vm_flush((r), (vmid), (addr))
#define gsgpu_ring_emit_fence(r, addr, seq, flags) (r)->funcs->emit_fence((r), (addr), (seq), (flags))
#define gsgpu_ring_emit_gds_switch(r, v, db, ds, wb, ws, ab, as) (r)->funcs->emit_gds_switch((r), (v), (db), (ds), (wb), (ws), (ab), (as))
#define gsgpu_ring_emit_switch_buffer(r) (r)->funcs->emit_switch_buffer((r))
#define gsgpu_ring_emit_cntxcntl(r, d) (r)->funcs->emit_cntxcntl((r), (d))
#define gsgpu_ring_emit_rreg(r, d, o) (r)->funcs->emit_rreg((r), (d), (o))
#define gsgpu_ring_emit_wreg(r, d, v) (r)->funcs->emit_wreg((r), (d), (v))
#define gsgpu_ring_emit_reg_wait(r, d, v, m) (r)->funcs->emit_reg_wait((r), (d), (v), (m))
#define gsgpu_ring_emit_reg_write_reg_wait(r, d0, d1, v, m) (r)->funcs->emit_reg_write_reg_wait((r), (d0), (d1), (v), (m))
#define gsgpu_ring_emit_frame_cntl(r, b, s) (r)->funcs->emit_frame_cntl((r), (b), (s))
#define gsgpu_ring_pad_ib(r, ib) ((r)->funcs->pad_ib((r), (ib)))
#define gsgpu_ring_init_cond_exec(r) (r)->funcs->init_cond_exec((r))
#define gsgpu_ring_patch_cond_exec(r,o) (r)->funcs->patch_cond_exec((r),(o))
#define gsgpu_ring_preempt_ib(r) (r)->funcs->preempt_ib(r)

int gsgpu_ring_alloc(struct gsgpu_ring *ring, unsigned ndw);
void gsgpu_ring_ib_begin(struct gsgpu_ring *ring);
void gsgpu_ring_ib_end(struct gsgpu_ring *ring);

void gsgpu_ring_insert_nop(struct gsgpu_ring *ring, uint32_t count);
void gsgpu_ring_generic_pad_ib(struct gsgpu_ring *ring, struct gsgpu_ib *ib);
void gsgpu_ring_commit(struct gsgpu_ring *ring);
void gsgpu_ring_undo(struct gsgpu_ring *ring);
int gsgpu_ring_init(struct gsgpu_device *adev, struct gsgpu_ring *ring,
		     unsigned int max_dw, struct gsgpu_irq_src *irq_src,
		     unsigned int irq_type, unsigned int hw_prio,
		     atomic_t *sched_score);
void gsgpu_ring_fini(struct gsgpu_ring *ring);
void gsgpu_ring_emit_reg_write_reg_wait_helper(struct gsgpu_ring *ring,
						uint32_t reg0, uint32_t val0,
						uint32_t reg1, uint32_t val1);
bool gsgpu_ring_soft_recovery(struct gsgpu_ring *ring, unsigned int vmid,
			       struct dma_fence *fence);

static inline void gsgpu_ring_set_preempt_cond_exec(struct gsgpu_ring *ring,
							bool cond_exec)
{
	*ring->cond_exe_cpu_addr = cond_exec;
}

static inline void gsgpu_ring_clear_ring(struct gsgpu_ring *ring)
{
	int i = 0;
	while (i <= ring->buf_mask)
		ring->ring[i++] = ring->funcs->nop;

}

static inline void gsgpu_ring_write(struct gsgpu_ring *ring, uint32_t v)
{
	if (ring->count_dw <= 0)
		DRM_ERROR("gsgpu: writing more dwords to the ring than expected!\n");
	ring->ring[ring->wptr++ & ring->buf_mask] = v;
	ring->wptr &= ring->ptr_mask;
	ring->count_dw--;
}

static inline void gsgpu_ring_write_multiple(struct gsgpu_ring *ring,
					      void *src, int count_dw)
{
	unsigned occupied, chunk1, chunk2;
	void *dst;

	if (unlikely(ring->count_dw < count_dw))
		DRM_ERROR("gsgpu: writing more dwords to the ring than expected!\n");

	occupied = ring->wptr & ring->buf_mask;
	dst = (void *)&ring->ring[occupied];
	chunk1 = ring->buf_mask + 1 - occupied;
	chunk1 = (chunk1 >= count_dw) ? count_dw: chunk1;
	chunk2 = count_dw - chunk1;
	chunk1 <<= 2;
	chunk2 <<= 2;

	if (chunk1)
		memcpy(dst, src, chunk1);

	if (chunk2) {
		src += chunk1;
		dst = (void *)ring->ring;
		memcpy(dst, src, chunk2);
	}

	ring->wptr += count_dw;
	ring->wptr &= ring->ptr_mask;
	ring->count_dw -= count_dw;
}

int gsgpu_ring_test_helper(struct gsgpu_ring *ring);

void gsgpu_debugfs_ring_init(struct gsgpu_device *adev,
			      struct gsgpu_ring *ring);

static inline u32 gsgpu_ib_get_value(struct gsgpu_ib *ib, int idx)
{
	return ib->ptr[idx];
}

static inline void gsgpu_ib_set_value(struct gsgpu_ib *ib, int idx,
				       uint32_t value)
{
	ib->ptr[idx] = value;
}

int gsgpu_ib_get(struct gsgpu_device *adev, struct gsgpu_vm *vm,
		  unsigned size,
		  enum gsgpu_ib_pool_type pool,
		  struct gsgpu_ib *ib);
void gsgpu_ib_free(struct gsgpu_device *adev, struct gsgpu_ib *ib,
		    struct dma_fence *f);
int gsgpu_ib_schedule(struct gsgpu_ring *ring, unsigned num_ibs,
		       struct gsgpu_ib *ibs, struct gsgpu_job *job,
		       struct dma_fence **f);
int gsgpu_ib_pool_init(struct gsgpu_device *adev);
void gsgpu_ib_pool_fini(struct gsgpu_device *adev);
int gsgpu_ib_ring_tests(struct gsgpu_device *adev);

#endif
