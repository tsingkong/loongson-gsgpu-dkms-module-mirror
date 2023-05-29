/*
 * Copyright 2017 Valve Corporation
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
 * Authors: Andres Rodriguez <andresx7@gmail.com>
 */

#include <linux/fdtable.h>
#include <linux/file.h>
#include <linux/pid.h>

#include <drm/gsgpu_drm.h>

#include "gsgpu.h"
#include "gsgpu_sched.h"
#include "gsgpu_vm.h"

static int gsgpu_sched_process_priority_override(struct gsgpu_device *adev,
						  int fd,
						  int32_t priority)
{
	struct fd f = fdget(fd);
	struct gsgpu_fpriv *fpriv;
	struct gsgpu_ctx *ctx;
	uint32_t id;
	int r;

	if (!f.file)
		return -EINVAL;

	r = gsgpu_file_to_fpriv(f.file, &fpriv);
	if (r) {
		fdput(f);
		return r;
	}

	idr_for_each_entry(&fpriv->ctx_mgr.ctx_handles, ctx, id)
		gsgpu_ctx_priority_override(ctx, priority);

	fdput(f);
	return 0;
}

static int gsgpu_sched_context_priority_override(struct gsgpu_device *adev,
						  int fd,
						  unsigned ctx_id,
						  int32_t priority)
{
	struct fd f = fdget(fd);
	struct gsgpu_fpriv *fpriv;
	struct gsgpu_ctx *ctx;
	int r;

	if (!f.file)
		return -EINVAL;

	r = gsgpu_file_to_fpriv(f.file, &fpriv);
	if (r) {
		fdput(f);
		return r;
	}

	ctx = gsgpu_ctx_get(fpriv, ctx_id);

	if (!ctx) {
		fdput(f);
		return -EINVAL;
	}

	gsgpu_ctx_priority_override(ctx, priority);
	gsgpu_ctx_put(ctx);
	fdput(f);

	return 0;
}

int gsgpu_sched_ioctl(struct drm_device *dev, void *data,
		       struct drm_file *filp)
{
	union drm_gsgpu_sched *args = data;
	struct gsgpu_device *adev = drm_to_adev(dev);
	int r;

	/* First check the op, then the op's argument.
	 */
	switch (args->in.op) {
	case GSGPU_SCHED_OP_PROCESS_PRIORITY_OVERRIDE:
	case GSGPU_SCHED_OP_CONTEXT_PRIORITY_OVERRIDE:
		break;
	default:
		DRM_ERROR("Invalid sched op specified: %d\n", args->in.op);
		return -EINVAL;
	}

	if (!gsgpu_ctx_priority_is_valid(args->in.priority)) {
		WARN(1, "Invalid context priority %d\n", args->in.priority);
		return -EINVAL;
	}

	switch (args->in.op) {
	case GSGPU_SCHED_OP_PROCESS_PRIORITY_OVERRIDE:
		r = gsgpu_sched_process_priority_override(adev,
							   args->in.fd,
							   args->in.priority);
		break;
	case GSGPU_SCHED_OP_CONTEXT_PRIORITY_OVERRIDE:
		r = gsgpu_sched_context_priority_override(adev,
							   args->in.fd,
							   args->in.ctx_id,
							   args->in.priority);
		break;
	default:
		/* Impossible.
		 */
		r = -EINVAL;
		break;
	}

	return r;
}
