/*
 * Copyright 2022 Advanced Micro Devices, Inc.
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
#ifndef __GSGPU_CS_H__
#define __GSGPU_CS_H__

#include <linux/ww_mutex.h>

#include "gsgpu_job.h"
#include "gsgpu_bo_list.h"
#include "gsgpu_ring.h"

#define GSGPU_CS_GANG_SIZE	4

struct gsgpu_bo_va_mapping;

struct gsgpu_cs_chunk {
	uint32_t		chunk_id;
	uint32_t		length_dw;
	void			*kdata;
};

struct gsgpu_cs_post_dep {
	struct drm_syncobj *syncobj;
	struct dma_fence_chain *chain;
	u64 point;
};

struct gsgpu_cs_parser {
	struct gsgpu_device	*adev;
	struct drm_file		*filp;
	struct gsgpu_ctx	*ctx;

	/* chunks */
	unsigned		nchunks;
	struct gsgpu_cs_chunk	*chunks;

	/* scheduler job objects */
	unsigned int		gang_size;
	unsigned int		gang_leader_idx;
	struct drm_sched_entity	*entities[GSGPU_CS_GANG_SIZE];
	struct gsgpu_job	*jobs[GSGPU_CS_GANG_SIZE];
	struct gsgpu_job	*gang_leader;

	/* buffer objects */
	struct ww_acquire_ctx		ticket;
	struct gsgpu_bo_list		*bo_list;
	struct gsgpu_mn		*mn;
	struct gsgpu_bo_list_entry	vm_pd;
	struct list_head		validated;
	struct dma_fence		*fence;
	uint64_t			bytes_moved_threshold;
	uint64_t			bytes_moved_vis_threshold;
	uint64_t			bytes_moved;
	uint64_t			bytes_moved_vis;

	/* user fence */
	struct gsgpu_bo_list_entry	uf_entry;

	unsigned			num_post_deps;
	struct gsgpu_cs_post_dep	*post_deps;

	struct gsgpu_sync		sync;
};

int gsgpu_cs_find_mapping(struct gsgpu_cs_parser *parser,
			   uint64_t addr, struct gsgpu_bo **bo,
			   struct gsgpu_bo_va_mapping **mapping);

#endif
