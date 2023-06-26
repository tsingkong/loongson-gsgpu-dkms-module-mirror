/*
 * Copyright 2015 Advanced Micro Devices, Inc.
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
 *
 */
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/sched.h>

#include <drm/drm_drv.h>

#include "gsgpu.h"
#include "gsgpu_trace.h"
#include "gsgpu_reset.h"

static enum drm_gpu_sched_stat gsgpu_job_timedout(struct drm_sched_job *s_job)
{
	struct gsgpu_ring *ring = to_gsgpu_ring(s_job->sched);
	struct gsgpu_job *job = to_gsgpu_job(s_job);
	struct gsgpu_task_info ti;
	struct gsgpu_device *adev = ring->adev;
	int idx;
	// int r;

	if (!drm_dev_enter(adev_to_drm(adev), &idx)) {
		DRM_INFO("%s - device unplugged skipping recovery on scheduler:%s",
			 __func__, s_job->sched->name);

		/* Effectively the job is aborted as the device is gone */
		return DRM_GPU_SCHED_STAT_ENODEV;
	}

	memset(&ti, 0, sizeof(struct gsgpu_task_info));
	adev->job_hang = true;

	gsgpu_vm_get_task_info(ring->adev, job->pasid, &ti);
	DRM_ERROR("ring %s timeout, signaled seq=%u, emitted seq=%u\n",
		  job->base.sched->name, atomic_read(&ring->fence_drv.last_seq),
		  ring->fence_drv.sync_seq);
	DRM_ERROR("Process information: process %s pid %d thread %s pid %d\n",
		  ti.process_name, ti.tgid, ti.task_name, ti.pid);

		drm_sched_suspend_timeout(&ring->sched);

// exit:
	adev->job_hang = false;
	drm_dev_exit(idx);
	return DRM_GPU_SCHED_STAT_NOMINAL;
}

int gsgpu_job_alloc(struct gsgpu_device *adev, struct gsgpu_vm *vm,
		     struct drm_sched_entity *entity, void *owner,
		     unsigned int num_ibs, struct gsgpu_job **job)
{
	if (num_ibs == 0)
		return -EINVAL;

	*job = kzalloc(struct_size(*job, ibs, num_ibs), GFP_KERNEL);
	if (!*job)
		return -ENOMEM;

	/*
	 * Initialize the scheduler to at least some ring so that we always
	 * have a pointer to adev.
	 */
	(*job)->base.sched = &adev->rings[0]->sched;
	(*job)->vm = vm;

	gsgpu_sync_create(&(*job)->explicit_sync);
	(*job)->vram_lost_counter = atomic_read(&adev->vram_lost_counter);
	(*job)->vm_pd_addr = GSGPU_BO_INVALID_OFFSET;

	if (!entity)
		return 0;

	return drm_sched_job_init(&(*job)->base, entity, owner);
}

int gsgpu_job_alloc_with_ib(struct gsgpu_device *adev,
			     struct drm_sched_entity *entity, void *owner,
			     size_t size, enum gsgpu_ib_pool_type pool_type,
			     struct gsgpu_job **job)
{
	int r;

	r = gsgpu_job_alloc(adev, NULL, entity, owner, 1, job);
	if (r)
		return r;

	(*job)->num_ibs = 1;
	r = gsgpu_ib_get(adev, NULL, size, pool_type, &(*job)->ibs[0]);
	if (r) {
		if (entity)
			drm_sched_job_cleanup(&(*job)->base);
		kfree(*job);
	}

	return r;
}

void gsgpu_job_free_resources(struct gsgpu_job *job)
{
	struct gsgpu_ring *ring = to_gsgpu_ring(job->base.sched);
	struct dma_fence *f;
	unsigned i;

	/* Check if any fences where initialized */
	if (job->base.s_fence && job->base.s_fence->finished.ops)
		f = &job->base.s_fence->finished;
	else if (job->hw_fence.ops)
		f = &job->hw_fence;
	else
		f = NULL;

	for (i = 0; i < job->num_ibs; ++i)
		gsgpu_ib_free(ring->adev, &job->ibs[i], f);
}

static void gsgpu_job_free_cb(struct drm_sched_job *s_job)
{
	struct gsgpu_job *job = to_gsgpu_job(s_job);

	drm_sched_job_cleanup(s_job);

	gsgpu_sync_free(&job->explicit_sync);

	/* only put the hw fence if has embedded fence */
	if (!job->hw_fence.ops)
		kfree(job);
	else
		dma_fence_put(&job->hw_fence);
}

void gsgpu_job_set_gang_leader(struct gsgpu_job *job,
				struct gsgpu_job *leader)
{
	struct dma_fence *fence = &leader->base.s_fence->scheduled;

	WARN_ON(job->gang_submit);

	/*
	 * Don't add a reference when we are the gang leader to avoid circle
	 * dependency.
	 */
	if (job != leader)
		dma_fence_get(fence);
	job->gang_submit = fence;
}

void gsgpu_job_free(struct gsgpu_job *job)
{
	if (job->base.entity)
		drm_sched_job_cleanup(&job->base);

	gsgpu_job_free_resources(job);
	gsgpu_sync_free(&job->explicit_sync);
	if (job->gang_submit != &job->base.s_fence->scheduled)
		dma_fence_put(job->gang_submit);

	if (!job->hw_fence.ops)
		kfree(job);
	else
		dma_fence_put(&job->hw_fence);
}

struct dma_fence *gsgpu_job_submit(struct gsgpu_job *job)
{
	struct dma_fence *f;

	drm_sched_job_arm(&job->base);
	f = dma_fence_get(&job->base.s_fence->finished);
	gsgpu_job_free_resources(job);
	drm_sched_entity_push_job(&job->base);

	return f;
}

int gsgpu_job_submit_direct(struct gsgpu_job *job, struct gsgpu_ring *ring,
			     struct dma_fence **fence)
{
	int r;

	job->base.sched = &ring->sched;
	r = gsgpu_ib_schedule(ring, job->num_ibs, job->ibs, job, fence);

	if (r)
		return r;

	gsgpu_job_free(job);
	return 0;
}

static struct dma_fence *
gsgpu_job_prepare_job(struct drm_sched_job *sched_job,
		      struct drm_sched_entity *s_entity)
{
	struct gsgpu_ring *ring = to_gsgpu_ring(s_entity->rq->sched);
	struct gsgpu_job *job = to_gsgpu_job(sched_job);
	struct dma_fence *fence = NULL;
	int r;

	if (!fence && job->gang_submit)
		fence = gsgpu_device_switch_gang(ring->adev, job->gang_submit);

	while (!fence && job->vm && !job->vmid) {
		r = gsgpu_vmid_grab(job->vm, ring, job, &fence);
		if (r)
			DRM_ERROR("Error getting VM ID (%d)\n", r);
	}

	return fence;
}

static struct dma_fence *gsgpu_job_run(struct drm_sched_job *sched_job)
{
	struct gsgpu_ring *ring = to_gsgpu_ring(sched_job->sched);
	struct gsgpu_device *adev = ring->adev;
	struct dma_fence *fence = NULL, *finished;
	struct gsgpu_job *job;
	int r = 0;

	job = to_gsgpu_job(sched_job);
	finished = &job->base.s_fence->finished;

	trace_gsgpu_sched_run_job(job);

	/* Skip job if VRAM is lost and never resubmit gangs */
	if (job->vram_lost_counter != atomic_read(&adev->vram_lost_counter) ||
	    (job->job_run_counter && job->gang_submit))
		dma_fence_set_error(finished, -ECANCELED);

	if (finished->error < 0) {
		DRM_INFO("Skip scheduling IBs!\n");
	} else {
		r = gsgpu_ib_schedule(ring, job->num_ibs, job->ibs, job,
				       &fence);
		if (r)
			DRM_ERROR("Error scheduling IBs (%d)\n", r);
		else
			DRM_INFO("Success scheduling IBs (drm_sched_job:%p)\n", sched_job);
	}

	job->job_run_counter++;
	gsgpu_job_free_resources(job);

	fence = r ? ERR_PTR(r) : fence;
	return fence;
}

#define to_drm_sched_job(sched_job)		\
		container_of((sched_job), struct drm_sched_job, queue_node)

void gsgpu_job_stop_all_jobs_on_sched(struct drm_gpu_scheduler *sched)
{
	struct drm_sched_job *s_job;
	struct drm_sched_entity *s_entity = NULL;
	int i;

	/* Signal all jobs not yet scheduled */
	for (i = DRM_SCHED_PRIORITY_COUNT - 1; i >= DRM_SCHED_PRIORITY_MIN; i--) {
		struct drm_sched_rq *rq = &sched->sched_rq[i];
		spin_lock(&rq->lock);
		list_for_each_entry(s_entity, &rq->entities, list) {
			while ((s_job = to_drm_sched_job(spsc_queue_pop(&s_entity->job_queue)))) {
				struct drm_sched_fence *s_fence = s_job->s_fence;

				dma_fence_signal(&s_fence->scheduled);
				dma_fence_set_error(&s_fence->finished, -EHWPOISON);
				dma_fence_signal(&s_fence->finished);
			}
		}
		spin_unlock(&rq->lock);
	}

	/* Signal all jobs already scheduled to HW */
	list_for_each_entry(s_job, &sched->pending_list, list) {
		struct drm_sched_fence *s_fence = s_job->s_fence;

		dma_fence_set_error(&s_fence->finished, -EHWPOISON);
		dma_fence_signal(&s_fence->finished);
	}
}

const struct drm_sched_backend_ops gsgpu_sched_ops = {
	.prepare_job = gsgpu_job_prepare_job,
	.run_job = gsgpu_job_run,
	.timedout_job = gsgpu_job_timedout,
	.free_job = gsgpu_job_free_cb
};
