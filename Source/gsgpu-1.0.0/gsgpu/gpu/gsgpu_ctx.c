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
 * Authors: monk liu <monk.liu@amd.com>
 */

#include <drm/drm_auth.h>
#include <drm/drm_drv.h>
#include "gsgpu.h"
#include "gsgpu_sched.h"
#include <linux/nospec.h>

#define to_gsgpu_ctx_entity(e)	\
	container_of((e), struct gsgpu_ctx_entity, entity)

const unsigned int gsgpu_ctx_num_entities[GSGPU_HW_IP_NUM] = {
	[GSGPU_HW_IP_GFX]	=	1,
	[GSGPU_HW_IP_DMA]	=	2,
};

bool gsgpu_ctx_priority_is_valid(int32_t ctx_prio)
{
	switch (ctx_prio) {
	case GSGPU_CTX_PRIORITY_UNSET:
	case GSGPU_CTX_PRIORITY_VERY_LOW:
	case GSGPU_CTX_PRIORITY_LOW:
	case GSGPU_CTX_PRIORITY_NORMAL:
	case GSGPU_CTX_PRIORITY_HIGH:
	case GSGPU_CTX_PRIORITY_VERY_HIGH:
		return true;
	default:
		return false;
	}
}

static enum drm_sched_priority
gsgpu_ctx_to_drm_sched_prio(int32_t ctx_prio)
{
	switch (ctx_prio) {
	case GSGPU_CTX_PRIORITY_UNSET:
		return DRM_SCHED_PRIORITY_UNSET;

	case GSGPU_CTX_PRIORITY_VERY_LOW:
		return DRM_SCHED_PRIORITY_MIN;

	case GSGPU_CTX_PRIORITY_LOW:
		return DRM_SCHED_PRIORITY_MIN;

	case GSGPU_CTX_PRIORITY_NORMAL:
		return DRM_SCHED_PRIORITY_NORMAL;

	case GSGPU_CTX_PRIORITY_HIGH:
		return DRM_SCHED_PRIORITY_HIGH;

	case GSGPU_CTX_PRIORITY_VERY_HIGH:
		return DRM_SCHED_PRIORITY_HIGH;

	/* This should not happen as we sanitized userspace provided priority
	 * already, WARN if this happens.
	 */
	default:
		WARN(1, "Invalid context priority %d\n", ctx_prio);
		return DRM_SCHED_PRIORITY_NORMAL;
	}

}

static int gsgpu_ctx_priority_permit(struct drm_file *filp,
				      int32_t priority)
{
	if (!gsgpu_ctx_priority_is_valid(priority))
		return -EINVAL;

	/* NORMAL and below are accessible by everyone */
	if (priority <= GSGPU_CTX_PRIORITY_NORMAL)
		return 0;

	if (capable(CAP_SYS_NICE))
		return 0;

	if (drm_is_current_master(filp))
		return 0;

	return -EACCES;
}

static enum gsgpu_gfx_pipe_priority gsgpu_ctx_prio_to_gfx_pipe_prio(int32_t prio)
{
	switch (prio) {
	case GSGPU_CTX_PRIORITY_HIGH:
	case GSGPU_CTX_PRIORITY_VERY_HIGH:
		return GSGPU_GFX_PIPE_PRIO_HIGH;
	default:
		return GSGPU_GFX_PIPE_PRIO_NORMAL;
	}
}

// static enum gsgpu_ring_priority_level gsgpu_ctx_sched_prio_to_ring_prio(int32_t prio)
// {
// 	switch (prio) {
// 	case GSGPU_CTX_PRIORITY_HIGH:
// 		return GSGPU_RING_PRIO_1;
// 	case GSGPU_CTX_PRIORITY_VERY_HIGH:
// 		return GSGPU_RING_PRIO_2;
// 	default:
// 		return GSGPU_RING_PRIO_0;
// 	}
// }

static unsigned int gsgpu_ctx_get_hw_prio(struct gsgpu_ctx *ctx, u32 hw_ip)
{
	struct gsgpu_device *adev = ctx->mgr->adev;
	unsigned int hw_prio;
	int32_t ctx_prio;

	ctx_prio = (ctx->override_priority == GSGPU_CTX_PRIORITY_UNSET) ?
			ctx->init_priority : ctx->override_priority;

	switch (hw_ip) {
	case GSGPU_HW_IP_GFX:
		hw_prio = gsgpu_ctx_prio_to_gfx_pipe_prio(ctx_prio);
		break;
	default:
		hw_prio = GSGPU_RING_PRIO_DEFAULT;
		break;
	}

	hw_ip = array_index_nospec(hw_ip, GSGPU_HW_IP_NUM);
	if (adev->gpu_sched[hw_ip][hw_prio].num_scheds == 0)
		hw_prio = GSGPU_RING_PRIO_DEFAULT;

	return hw_prio;
}

/* Calculate the time spend on the hw */
static ktime_t gsgpu_ctx_fence_time(struct dma_fence *fence)
{
	struct drm_sched_fence *s_fence;

	if (!fence)
		return ns_to_ktime(0);

	/* When the fence is not even scheduled it can't have spend time */
	s_fence = to_drm_sched_fence(fence);
	if (!test_bit(DMA_FENCE_FLAG_TIMESTAMP_BIT, &s_fence->scheduled.flags))
		return ns_to_ktime(0);

	/* When it is still running account how much already spend */
	if (!test_bit(DMA_FENCE_FLAG_TIMESTAMP_BIT, &s_fence->finished.flags))
		return ktime_sub(ktime_get(), s_fence->scheduled.timestamp);

	return ktime_sub(s_fence->finished.timestamp,
			 s_fence->scheduled.timestamp);
}

static ktime_t gsgpu_ctx_entity_time(struct gsgpu_ctx *ctx,
				      struct gsgpu_ctx_entity *centity)
{
	ktime_t res = ns_to_ktime(0);
	uint32_t i;

	spin_lock(&ctx->ring_lock);
	for (i = 0; i < gsgpu_sched_jobs; i++) {
		res = ktime_add(res, gsgpu_ctx_fence_time(centity->fences[i]));
	}
	spin_unlock(&ctx->ring_lock);
	return res;
}

static int gsgpu_ctx_init_entity(struct gsgpu_ctx *ctx, u32 hw_ip,
				  const u32 ring)
{
	struct drm_gpu_scheduler **scheds = NULL; // , *sched = NULL;
	struct gsgpu_device *adev = ctx->mgr->adev;
	struct gsgpu_ctx_entity *entity;
	enum drm_sched_priority drm_prio;
	unsigned int hw_prio, num_scheds;
	int32_t ctx_prio;
	int r;

	entity = kzalloc(struct_size(entity, fences, gsgpu_sched_jobs),
			 GFP_KERNEL);
	if (!entity)
		return  -ENOMEM;

	ctx_prio = (ctx->override_priority == GSGPU_CTX_PRIORITY_UNSET) ?
			ctx->init_priority : ctx->override_priority;
	entity->hw_ip = hw_ip;
	entity->sequence = 1;
	hw_prio = gsgpu_ctx_get_hw_prio(ctx, hw_ip);
	drm_prio = gsgpu_ctx_to_drm_sched_prio(ctx_prio);

	hw_ip = array_index_nospec(hw_ip, GSGPU_HW_IP_NUM);
	scheds = adev->gpu_sched[hw_ip][hw_prio].sched;
	num_scheds = adev->gpu_sched[hw_ip][hw_prio].num_scheds;

	r = drm_sched_entity_init(&entity->entity, drm_prio, scheds, num_scheds,
				  &ctx->guilty);
	if (r)
		goto error_free_entity;

	/* It's not an error if we fail to install the new entity */
	if (cmpxchg(&ctx->entities[hw_ip][ring], NULL, entity))
		goto cleanup_entity;

	return 0;

cleanup_entity:
	drm_sched_entity_fini(&entity->entity);

error_free_entity:
	kfree(entity);

	return r;
}

static ktime_t gsgpu_ctx_fini_entity(struct gsgpu_ctx_entity *entity)
{
	ktime_t res = ns_to_ktime(0);
	int i;

	if (!entity)
		return res;

	for (i = 0; i < gsgpu_sched_jobs; ++i) {
		res = ktime_add(res, gsgpu_ctx_fence_time(entity->fences[i]));
		dma_fence_put(entity->fences[i]);
	}

	kfree(entity);
	return res;
}

static int gsgpu_ctx_get_stable_pstate(struct gsgpu_ctx *ctx,
					u32 *stable_pstate)
{
	// struct gsgpu_device *adev = ctx->mgr->adev;
	enum gsgpu_dpm_forced_level current_level;

	current_level = GSGPU_DPM_FORCED_LEVEL_PROFILE_STANDARD; // gsgpu_dpm_get_performance_level(adev);

	switch (current_level) {
	case GSGPU_DPM_FORCED_LEVEL_PROFILE_STANDARD:
		*stable_pstate = GSGPU_CTX_STABLE_PSTATE_STANDARD;
		break;
	case GSGPU_DPM_FORCED_LEVEL_PROFILE_MIN_SCLK:
		*stable_pstate = GSGPU_CTX_STABLE_PSTATE_MIN_SCLK;
		break;
	case GSGPU_DPM_FORCED_LEVEL_PROFILE_MIN_MCLK:
		*stable_pstate = GSGPU_CTX_STABLE_PSTATE_MIN_MCLK;
		break;
	case GSGPU_DPM_FORCED_LEVEL_PROFILE_PEAK:
		*stable_pstate = GSGPU_CTX_STABLE_PSTATE_PEAK;
		break;
	default:
		*stable_pstate = GSGPU_CTX_STABLE_PSTATE_NONE;
		break;
	}
	return 0;
}

static int gsgpu_ctx_init(struct gsgpu_ctx_mgr *mgr, int32_t priority,
			   struct drm_file *filp, struct gsgpu_ctx *ctx)
{
	u32 current_stable_pstate;
	int r;

	r = gsgpu_ctx_priority_permit(filp, priority);
	if (r)
		return r;

	memset(ctx, 0, sizeof(*ctx));

	kref_init(&ctx->refcount);
	ctx->mgr = mgr;
	spin_lock_init(&ctx->ring_lock);

	ctx->reset_counter = atomic_read(&mgr->adev->gpu_reset_counter);
	ctx->reset_counter_query = ctx->reset_counter;
	ctx->vram_lost_counter = atomic_read(&mgr->adev->vram_lost_counter);
	ctx->init_priority = priority;
	ctx->override_priority = GSGPU_CTX_PRIORITY_UNSET;

	r = gsgpu_ctx_get_stable_pstate(ctx, &current_stable_pstate);
	if (r)
		return r;

	ctx->stable_pstate = current_stable_pstate;

	return 0;
}

static int gsgpu_ctx_set_stable_pstate(struct gsgpu_ctx *ctx,
					u32 stable_pstate)
{
	// struct gsgpu_device *adev = ctx->mgr->adev;
	enum gsgpu_dpm_forced_level level;
	u32 current_stable_pstate;
	int r;

	r = gsgpu_ctx_get_stable_pstate(ctx, &current_stable_pstate);
	if (r || (stable_pstate == current_stable_pstate))
		goto done;

	switch (stable_pstate) {
	case GSGPU_CTX_STABLE_PSTATE_NONE:
		level = GSGPU_DPM_FORCED_LEVEL_AUTO;
		break;
	case GSGPU_CTX_STABLE_PSTATE_STANDARD:
		level = GSGPU_DPM_FORCED_LEVEL_PROFILE_STANDARD;
		break;
	case GSGPU_CTX_STABLE_PSTATE_MIN_SCLK:
		level = GSGPU_DPM_FORCED_LEVEL_PROFILE_MIN_SCLK;
		break;
	case GSGPU_CTX_STABLE_PSTATE_MIN_MCLK:
		level = GSGPU_DPM_FORCED_LEVEL_PROFILE_MIN_MCLK;
		break;
	case GSGPU_CTX_STABLE_PSTATE_PEAK:
		level = GSGPU_DPM_FORCED_LEVEL_PROFILE_PEAK;
		break;
	default:
		r = -EINVAL;
		goto done;
	}

done:

	return r;
}

static void gsgpu_ctx_fini(struct kref *ref)
{
	struct gsgpu_ctx *ctx = container_of(ref, struct gsgpu_ctx, refcount);
	struct gsgpu_ctx_mgr *mgr = ctx->mgr;
	struct gsgpu_device *adev = mgr->adev;
	unsigned i, j, idx;

	if (!adev)
		return;

	for (i = 0; i < GSGPU_HW_IP_NUM; ++i) {
		for (j = 0; j < GSGPU_MAX_ENTITY_NUM; ++j) {
			ktime_t spend;

			spend = gsgpu_ctx_fini_entity(ctx->entities[i][j]);
			atomic64_add(ktime_to_ns(spend), &mgr->time_spend[i]);
		}
	}

	if (drm_dev_enter(adev_to_drm(adev), &idx)) {
		gsgpu_ctx_set_stable_pstate(ctx, ctx->stable_pstate);
		drm_dev_exit(idx);
	}

	kfree(ctx);
}

int gsgpu_ctx_get_entity(struct gsgpu_ctx *ctx, u32 hw_ip, u32 instance,
			  u32 ring, struct drm_sched_entity **entity)
{
	int r;

	if (hw_ip >= GSGPU_HW_IP_NUM) {
		DRM_ERROR("unknown HW IP type: %d\n", hw_ip);
		return -EINVAL;
	}

	/* Right now all IPs have only one instance - multiple rings. */
	if (instance != 0) {
		DRM_DEBUG("invalid ip instance: %d\n", instance);
		return -EINVAL;
	}

	if (ring >= gsgpu_ctx_num_entities[hw_ip]) {
		DRM_DEBUG("invalid ring: %d %d\n", hw_ip, ring);
		return -EINVAL;
	}

	if (ctx->entities[hw_ip][ring] == NULL) {
		r = gsgpu_ctx_init_entity(ctx, hw_ip, ring);
		if (r)
			return r;
	}

	*entity = &ctx->entities[hw_ip][ring]->entity;
	return 0;
}

static int gsgpu_ctx_alloc(struct gsgpu_device *adev,
			    struct gsgpu_fpriv *fpriv,
			    struct drm_file *filp,
			    int32_t priority,
			    uint32_t *id)
{
	struct gsgpu_ctx_mgr *mgr = &fpriv->ctx_mgr;
	struct gsgpu_ctx *ctx;
	int r;

	ctx = kmalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mutex_lock(&mgr->lock);
	r = idr_alloc(&mgr->ctx_handles, ctx, 1, GSGPU_VM_MAX_NUM_CTX, GFP_KERNEL);
	if (r < 0) {
		mutex_unlock(&mgr->lock);
		kfree(ctx);
		return r;
	}

	*id = (uint32_t)r;
	r = gsgpu_ctx_init(mgr, priority, filp, ctx);
	if (r) {
		idr_remove(&mgr->ctx_handles, *id);
		*id = 0;
		kfree(ctx);
	}
	mutex_unlock(&mgr->lock);
	return r;
}

static void gsgpu_ctx_do_release(struct kref *ref)
{
	struct gsgpu_ctx *ctx;
	u32 i, j;

	ctx = container_of(ref, struct gsgpu_ctx, refcount);
	for (i = 0; i < GSGPU_HW_IP_NUM; ++i) {
		for (j = 0; j < gsgpu_ctx_num_entities[i]; ++j) {
			if (!ctx->entities[i][j])
				continue;

			drm_sched_entity_destroy(&ctx->entities[i][j]->entity);
		}
	}

	gsgpu_ctx_fini(ref);
}

static int gsgpu_ctx_free(struct gsgpu_fpriv *fpriv, uint32_t id)
{
	struct gsgpu_ctx_mgr *mgr = &fpriv->ctx_mgr;
	struct gsgpu_ctx *ctx;

	mutex_lock(&mgr->lock);
	ctx = idr_remove(&mgr->ctx_handles, id);
	if (ctx)
		kref_put(&ctx->refcount, gsgpu_ctx_do_release);
	mutex_unlock(&mgr->lock);
	return ctx ? 0 : -EINVAL;
}

static int gsgpu_ctx_query(struct gsgpu_device *adev,
			    struct gsgpu_fpriv *fpriv, uint32_t id,
			    union drm_gsgpu_ctx_out *out)
{
	struct gsgpu_ctx *ctx;
	struct gsgpu_ctx_mgr *mgr;
	unsigned reset_counter;

	if (!fpriv)
		return -EINVAL;

	mgr = &fpriv->ctx_mgr;
	mutex_lock(&mgr->lock);
	ctx = idr_find(&mgr->ctx_handles, id);
	if (!ctx) {
		mutex_unlock(&mgr->lock);
		return -EINVAL;
	}

	/* TODO: these two are always zero */
	out->state.flags = 0x0;
	out->state.hangs = 0x0;

	/* determine if a GPU reset has occured since the last call */
	reset_counter = atomic_read(&adev->gpu_reset_counter);
	/* TODO: this should ideally return NO, GUILTY, or INNOCENT. */
	if (ctx->reset_counter_query == reset_counter)
		out->state.reset_status = GSGPU_CTX_NO_RESET;
	else
		out->state.reset_status = GSGPU_CTX_UNKNOWN_RESET;
	ctx->reset_counter_query = reset_counter;

	mutex_unlock(&mgr->lock);
	return 0;
}

#define GSGPU_RAS_COUNTE_DELAY_MS 3000

static int gsgpu_ctx_query2(struct gsgpu_device *adev,
			     struct gsgpu_fpriv *fpriv, uint32_t id,
			     union drm_gsgpu_ctx_out *out)
{
	struct gsgpu_ctx *ctx;
	struct gsgpu_ctx_mgr *mgr;

	if (!fpriv)
		return -EINVAL;

	mgr = &fpriv->ctx_mgr;
	mutex_lock(&mgr->lock);
	ctx = idr_find(&mgr->ctx_handles, id);
	if (!ctx) {
		mutex_unlock(&mgr->lock);
		return -EINVAL;
	}

	out->state.flags = 0x0;
	out->state.hangs = 0x0;

	if (ctx->reset_counter != atomic_read(&adev->gpu_reset_counter))
		out->state.flags |= GSGPU_CTX_QUERY2_FLAGS_RESET;

	if (ctx->vram_lost_counter != atomic_read(&adev->vram_lost_counter))
		out->state.flags |= GSGPU_CTX_QUERY2_FLAGS_VRAMLOST;

	if (atomic_read(&ctx->guilty))
		out->state.flags |= GSGPU_CTX_QUERY2_FLAGS_GUILTY;

	mutex_unlock(&mgr->lock);
	return 0;
}



static int gsgpu_ctx_stable_pstate(struct gsgpu_device *adev,
				    struct gsgpu_fpriv *fpriv, uint32_t id,
				    bool set, u32 *stable_pstate)
{
	struct gsgpu_ctx *ctx;
	struct gsgpu_ctx_mgr *mgr;
	int r;

	if (!fpriv)
		return -EINVAL;

	mgr = &fpriv->ctx_mgr;
	mutex_lock(&mgr->lock);
	ctx = idr_find(&mgr->ctx_handles, id);
	if (!ctx) {
		mutex_unlock(&mgr->lock);
		return -EINVAL;
	}

	if (set)
		r = gsgpu_ctx_set_stable_pstate(ctx, *stable_pstate);
	else
		r = gsgpu_ctx_get_stable_pstate(ctx, stable_pstate);

	mutex_unlock(&mgr->lock);
	return r;
}

int gsgpu_ctx_ioctl(struct drm_device *dev, void *data,
		     struct drm_file *filp)
{
	int r;
	uint32_t id, stable_pstate;
	int32_t priority;

	union drm_gsgpu_ctx *args = data;
	struct gsgpu_device *adev = drm_to_adev(dev);
	struct gsgpu_fpriv *fpriv = filp->driver_priv;

	id = args->in.ctx_id;
	priority = args->in.priority;

	/* For backwards compatibility reasons, we need to accept
	 * ioctls with garbage in the priority field */
	if (!gsgpu_ctx_priority_is_valid(priority))
		priority = GSGPU_CTX_PRIORITY_NORMAL;

	switch (args->in.op) {
	case GSGPU_CTX_OP_ALLOC_CTX:
		r = gsgpu_ctx_alloc(adev, fpriv, filp, priority, &id);
		args->out.alloc.ctx_id = id;
		break;
	case GSGPU_CTX_OP_FREE_CTX:
		r = gsgpu_ctx_free(fpriv, id);
		break;
	case GSGPU_CTX_OP_QUERY_STATE:
		r = gsgpu_ctx_query(adev, fpriv, id, &args->out);
		break;
	case GSGPU_CTX_OP_QUERY_STATE2:
		r = gsgpu_ctx_query2(adev, fpriv, id, &args->out);
		break;
	case GSGPU_CTX_OP_GET_STABLE_PSTATE:
		if (args->in.flags)
			return -EINVAL;
		r = gsgpu_ctx_stable_pstate(adev, fpriv, id, false, &stable_pstate);
		if (!r)
			args->out.pstate.flags = stable_pstate;
		break;
	case GSGPU_CTX_OP_SET_STABLE_PSTATE:
		if (args->in.flags & ~GSGPU_CTX_STABLE_PSTATE_FLAGS_MASK)
			return -EINVAL;
		stable_pstate = args->in.flags & GSGPU_CTX_STABLE_PSTATE_FLAGS_MASK;
		if (stable_pstate > GSGPU_CTX_STABLE_PSTATE_PEAK)
			return -EINVAL;
		r = gsgpu_ctx_stable_pstate(adev, fpriv, id, true, &stable_pstate);
		break;
	default:
		return -EINVAL;
	}

	return r;
}

struct gsgpu_ctx *gsgpu_ctx_get(struct gsgpu_fpriv *fpriv, uint32_t id)
{
	struct gsgpu_ctx *ctx;
	struct gsgpu_ctx_mgr *mgr;

	if (!fpriv)
		return NULL;

	mgr = &fpriv->ctx_mgr;

	mutex_lock(&mgr->lock);
	ctx = idr_find(&mgr->ctx_handles, id);
	if (ctx)
		kref_get(&ctx->refcount);
	mutex_unlock(&mgr->lock);
	return ctx;
}

int gsgpu_ctx_put(struct gsgpu_ctx *ctx)
{
	if (ctx == NULL)
		return -EINVAL;

	kref_put(&ctx->refcount, gsgpu_ctx_do_release);
	return 0;
}

uint64_t gsgpu_ctx_add_fence(struct gsgpu_ctx *ctx,
			      struct drm_sched_entity *entity,
			      struct dma_fence *fence)
{
	struct gsgpu_ctx_entity *centity = to_gsgpu_ctx_entity(entity);
	uint64_t seq = centity->sequence;
	struct dma_fence *other = NULL;
	unsigned idx = 0;

	idx = seq & (gsgpu_sched_jobs - 1);
	other = centity->fences[idx];
	WARN_ON(other && !dma_fence_is_signaled(other));

	dma_fence_get(fence);

	spin_lock(&ctx->ring_lock);
	centity->fences[idx] = fence;
	centity->sequence++;
	spin_unlock(&ctx->ring_lock);

	atomic64_add(ktime_to_ns(gsgpu_ctx_fence_time(other)),
		     &ctx->mgr->time_spend[centity->hw_ip]);

	dma_fence_put(other);
	return seq;
}

struct dma_fence *gsgpu_ctx_get_fence(struct gsgpu_ctx *ctx,
				       struct drm_sched_entity *entity,
				       uint64_t seq)
{
	struct gsgpu_ctx_entity *centity = to_gsgpu_ctx_entity(entity);
	struct dma_fence *fence;

	spin_lock(&ctx->ring_lock);

	if (seq == ~0ull)
		seq = centity->sequence - 1;

	if (seq >= centity->sequence) {
		spin_unlock(&ctx->ring_lock);
		return ERR_PTR(-EINVAL);
	}


	if (seq + gsgpu_sched_jobs < centity->sequence) {
		spin_unlock(&ctx->ring_lock);
		return NULL;
	}

	fence = dma_fence_get(centity->fences[seq & (gsgpu_sched_jobs - 1)]);
	spin_unlock(&ctx->ring_lock);

	return fence;
}

static void gsgpu_ctx_set_entity_priority(struct gsgpu_ctx *ctx,
					   struct gsgpu_ctx_entity *aentity,
					   int hw_ip,
					   int32_t priority)
{
	struct gsgpu_device *adev = ctx->mgr->adev;
	unsigned int hw_prio;
	struct drm_gpu_scheduler **scheds = NULL;
	unsigned num_scheds;

	/* set sw priority */
	drm_sched_entity_set_priority(&aentity->entity,
				      gsgpu_ctx_to_drm_sched_prio(priority));

	/* set hw priority */
	if (hw_ip == GSGPU_HW_IP_GFX) {
		hw_prio = gsgpu_ctx_get_hw_prio(ctx, hw_ip);
		hw_prio = array_index_nospec(hw_prio, GSGPU_RING_PRIO_MAX);
		scheds = adev->gpu_sched[hw_ip][hw_prio].sched;
		num_scheds = adev->gpu_sched[hw_ip][hw_prio].num_scheds;
		drm_sched_entity_modify_sched(&aentity->entity, scheds,
					      num_scheds);
	}
}

void gsgpu_ctx_priority_override(struct gsgpu_ctx *ctx,
				  int32_t priority)
{
	int32_t ctx_prio;
	unsigned i, j;

	ctx->override_priority = priority;

	ctx_prio = (ctx->override_priority == GSGPU_CTX_PRIORITY_UNSET) ?
			ctx->init_priority : ctx->override_priority;
	for (i = 0; i < GSGPU_HW_IP_NUM; ++i) {
		for (j = 0; j < gsgpu_ctx_num_entities[i]; ++j) {
			if (!ctx->entities[i][j])
				continue;

			gsgpu_ctx_set_entity_priority(ctx, ctx->entities[i][j],
						       i, ctx_prio);
		}
	}
}

int gsgpu_ctx_wait_prev_fence(struct gsgpu_ctx *ctx,
			       struct drm_sched_entity *entity)
{
	struct gsgpu_ctx_entity *centity = to_gsgpu_ctx_entity(entity);
	struct dma_fence *other;
	unsigned idx;
	long r;

	spin_lock(&ctx->ring_lock);
	idx = centity->sequence & (gsgpu_sched_jobs - 1);
	other = dma_fence_get(centity->fences[idx]);
	spin_unlock(&ctx->ring_lock);

	if (!other)
		return 0;

	r = dma_fence_wait(other, true);
	if (r < 0 && r != -ERESTARTSYS)
		DRM_ERROR("Error (%ld) waiting for fence!\n", r);

	dma_fence_put(other);
	return r;
}

void gsgpu_ctx_mgr_init(struct gsgpu_ctx_mgr *mgr,
			 struct gsgpu_device *adev)
{
	unsigned int i;

	mgr->adev = adev;
	mutex_init(&mgr->lock);
	idr_init_base(&mgr->ctx_handles, 1);

	for (i = 0; i < GSGPU_HW_IP_NUM; ++i)
		atomic64_set(&mgr->time_spend[i], 0);
}

long gsgpu_ctx_mgr_entity_flush(struct gsgpu_ctx_mgr *mgr, long timeout)
{
	struct gsgpu_ctx *ctx;
	struct idr *idp;
	uint32_t id, i, j;

	idp = &mgr->ctx_handles;

	mutex_lock(&mgr->lock);
	idr_for_each_entry(idp, ctx, id) {
		for (i = 0; i < GSGPU_HW_IP_NUM; ++i) {
			for (j = 0; j < gsgpu_ctx_num_entities[i]; ++j) {
				struct drm_sched_entity *entity;

				if (!ctx->entities[i][j])
					continue;

				entity = &ctx->entities[i][j]->entity;
				timeout = drm_sched_entity_flush(entity, timeout);
			}
		}
	}
	mutex_unlock(&mgr->lock);
	return timeout;
}

void gsgpu_ctx_mgr_entity_fini(struct gsgpu_ctx_mgr *mgr)
{
	struct gsgpu_ctx *ctx;
	struct idr *idp;
	uint32_t id, i, j;

	idp = &mgr->ctx_handles;

	idr_for_each_entry(idp, ctx, id) {
		if (kref_read(&ctx->refcount) != 1) {
			DRM_ERROR("ctx %p is still alive\n", ctx);
			continue;
		}

		for (i = 0; i < GSGPU_HW_IP_NUM; ++i) {
			for (j = 0; j < gsgpu_ctx_num_entities[i]; ++j) {
				struct drm_sched_entity *entity;

				if (!ctx->entities[i][j])
					continue;

				entity = &ctx->entities[i][j]->entity;
				drm_sched_entity_fini(entity);
			}
		}
	}
}

void gsgpu_ctx_mgr_fini(struct gsgpu_ctx_mgr *mgr)
{
	struct gsgpu_ctx *ctx;
	struct idr *idp;
	uint32_t id;

	gsgpu_ctx_mgr_entity_fini(mgr);

	idp = &mgr->ctx_handles;

	idr_for_each_entry(idp, ctx, id) {
		if (kref_put(&ctx->refcount, gsgpu_ctx_fini) != 1)
			DRM_ERROR("ctx %p is still alive\n", ctx);
	}

	idr_destroy(&mgr->ctx_handles);
	mutex_destroy(&mgr->lock);
}

void gsgpu_ctx_mgr_usage(struct gsgpu_ctx_mgr *mgr,
			  ktime_t usage[GSGPU_HW_IP_NUM])
{
	struct gsgpu_ctx *ctx;
	unsigned int hw_ip, i;
	uint32_t id;

	/*
	 * This is a little bit racy because it can be that a ctx or a fence are
	 * destroyed just in the moment we try to account them. But that is ok
	 * since exactly that case is explicitely allowed by the interface.
	 */
	mutex_lock(&mgr->lock);
	for (hw_ip = 0; hw_ip < GSGPU_HW_IP_NUM; ++hw_ip) {
		uint64_t ns = atomic64_read(&mgr->time_spend[hw_ip]);

		usage[hw_ip] = ns_to_ktime(ns);
	}

	idr_for_each_entry(&mgr->ctx_handles, ctx, id) {
		for (hw_ip = 0; hw_ip < GSGPU_HW_IP_NUM; ++hw_ip) {
			for (i = 0; i < gsgpu_ctx_num_entities[hw_ip]; ++i) {
				struct gsgpu_ctx_entity *centity;
				ktime_t spend;

				centity = ctx->entities[hw_ip][i];
				if (!centity)
					continue;
				spend = gsgpu_ctx_entity_time(ctx, centity);
				usage[hw_ip] = ktime_add(usage[hw_ip], spend);
			}
		}
	}
	mutex_unlock(&mgr->lock);
}
