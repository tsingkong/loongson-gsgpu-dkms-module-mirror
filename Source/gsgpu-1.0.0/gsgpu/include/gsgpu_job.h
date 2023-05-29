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
#ifndef __GSGPU_JOB_H__
#define __GSGPU_JOB_H__

#include <drm/gpu_scheduler.h>
#include "gsgpu_sync.h"
#include "gsgpu_ring.h"

/* bit set means command submit involves a preamble IB */
#define GSGPU_PREAMBLE_IB_PRESENT          (1 << 0)
/* bit set means preamble IB is first presented in belonging context */
#define GSGPU_PREAMBLE_IB_PRESENT_FIRST    (1 << 1)
/* bit set means context switch occured */
#define GSGPU_HAVE_CTX_SWITCH              (1 << 2)
/* bit set means IB is preempted */
#define GSGPU_IB_PREEMPTED                 (1 << 3)

#define to_gsgpu_job(sched_job)		\
		container_of((sched_job), struct gsgpu_job, base)

#define GSGPU_JOB_GET_VMID(job) ((job) ? (job)->vmid : 0)

struct gsgpu_fence;
enum gsgpu_ib_pool_type;

struct gsgpu_job {
	struct drm_sched_job    base;
	struct gsgpu_vm	*vm;
	struct gsgpu_sync	explicit_sync;
	struct dma_fence	hw_fence;
	struct dma_fence	*gang_submit;
	uint32_t		preamble_status;
	uint32_t                preemption_status;
	bool                    vm_needs_flush;
	bool			gds_switch_needed;
	bool			spm_update_needed;
	uint64_t		vm_pd_addr;
	unsigned		vmid;
	unsigned		pasid;
	uint32_t		vram_lost_counter;

	/* user fence handling */
	uint64_t		uf_addr;
	uint64_t		uf_sequence;

	/* job_run_counter >= 1 means a resubmit job */
	uint32_t		job_run_counter;

	uint32_t		num_ibs;
	struct gsgpu_ib	ibs[];
};

static inline struct gsgpu_ring *gsgpu_job_ring(struct gsgpu_job *job)
{
	return to_gsgpu_ring(job->base.entity->rq->sched);
}

int gsgpu_job_alloc(struct gsgpu_device *adev, struct gsgpu_vm *vm,
		     struct drm_sched_entity *entity, void *owner,
		     unsigned int num_ibs, struct gsgpu_job **job);
int gsgpu_job_alloc_with_ib(struct gsgpu_device *adev,
			     struct drm_sched_entity *entity, void *owner,
			     size_t size, enum gsgpu_ib_pool_type pool_type,
			     struct gsgpu_job **job);

void gsgpu_job_free_resources(struct gsgpu_job *job);
void gsgpu_job_set_gang_leader(struct gsgpu_job *job,
				struct gsgpu_job *leader);
void gsgpu_job_free(struct gsgpu_job *job);
struct dma_fence *gsgpu_job_submit(struct gsgpu_job *job);
int gsgpu_job_submit_direct(struct gsgpu_job *job, struct gsgpu_ring *ring,
			     struct dma_fence **fence);

void gsgpu_job_stop_all_jobs_on_sched(struct drm_gpu_scheduler *sched);

#endif
