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
#ifndef __GSGPU_CTX_H__
#define __GSGPU_CTX_H__

#include <linux/ktime.h>
#include <linux/types.h>

#include "gsgpu_ring.h"

struct drm_device;
struct drm_file;
struct gsgpu_fpriv;
struct gsgpu_ctx_mgr;

#define GSGPU_MAX_ENTITY_NUM 4


enum gsgpu_dpm_forced_level {
	GSGPU_DPM_FORCED_LEVEL_AUTO = 0x1,
	GSGPU_DPM_FORCED_LEVEL_MANUAL = 0x2,
	GSGPU_DPM_FORCED_LEVEL_LOW = 0x4,
	GSGPU_DPM_FORCED_LEVEL_HIGH = 0x8,
	GSGPU_DPM_FORCED_LEVEL_PROFILE_STANDARD = 0x10,
	GSGPU_DPM_FORCED_LEVEL_PROFILE_MIN_SCLK = 0x20,
	GSGPU_DPM_FORCED_LEVEL_PROFILE_MIN_MCLK = 0x40,
	GSGPU_DPM_FORCED_LEVEL_PROFILE_PEAK = 0x80,
	GSGPU_DPM_FORCED_LEVEL_PROFILE_EXIT = 0x100,
	GSGPU_DPM_FORCED_LEVEL_PERF_DETERMINISM = 0x200,
};

struct gsgpu_ctx_entity {
	uint32_t		hw_ip;
	uint64_t		sequence;
	struct drm_sched_entity	entity;
	struct dma_fence	*fences[];
};

struct gsgpu_ctx {
	struct kref			refcount;
	struct gsgpu_ctx_mgr		*mgr;
	unsigned			reset_counter;
	unsigned			reset_counter_query;
	uint32_t			vram_lost_counter;
	spinlock_t			ring_lock;
	struct gsgpu_ctx_entity	*entities[GSGPU_HW_IP_NUM][GSGPU_MAX_ENTITY_NUM];
	bool				preamble_presented;
	int32_t				init_priority;
	int32_t				override_priority;
	atomic_t			guilty;
	unsigned long			ras_counter_ce;
	unsigned long			ras_counter_ue;
	uint32_t			stable_pstate;
};

struct gsgpu_ctx_mgr {
	struct gsgpu_device	*adev;
	struct mutex		lock;
	/* protected by lock */
	struct idr		ctx_handles;
	atomic64_t		time_spend[GSGPU_HW_IP_NUM];
};

extern const unsigned int gsgpu_ctx_num_entities[GSGPU_HW_IP_NUM];

struct gsgpu_ctx *gsgpu_ctx_get(struct gsgpu_fpriv *fpriv, uint32_t id);
int gsgpu_ctx_put(struct gsgpu_ctx *ctx);

int gsgpu_ctx_get_entity(struct gsgpu_ctx *ctx, u32 hw_ip, u32 instance,
			  u32 ring, struct drm_sched_entity **entity);
uint64_t gsgpu_ctx_add_fence(struct gsgpu_ctx *ctx,
			      struct drm_sched_entity *entity,
			      struct dma_fence *fence);
struct dma_fence *gsgpu_ctx_get_fence(struct gsgpu_ctx *ctx,
				       struct drm_sched_entity *entity,
				       uint64_t seq);
bool gsgpu_ctx_priority_is_valid(int32_t ctx_prio);
void gsgpu_ctx_priority_override(struct gsgpu_ctx *ctx, int32_t ctx_prio);

int gsgpu_ctx_ioctl(struct drm_device *dev, void *data,
		     struct drm_file *filp);

int gsgpu_ctx_wait_prev_fence(struct gsgpu_ctx *ctx,
			       struct drm_sched_entity *entity);

void gsgpu_ctx_mgr_init(struct gsgpu_ctx_mgr *mgr,
			 struct gsgpu_device *adev);
void gsgpu_ctx_mgr_entity_fini(struct gsgpu_ctx_mgr *mgr);
long gsgpu_ctx_mgr_entity_flush(struct gsgpu_ctx_mgr *mgr, long timeout);
void gsgpu_ctx_mgr_fini(struct gsgpu_ctx_mgr *mgr);
void gsgpu_ctx_mgr_usage(struct gsgpu_ctx_mgr *mgr,
			  ktime_t usage[GSGPU_HW_IP_NUM]);

#endif
