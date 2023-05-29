/*
 * Copyright 2021 Advanced Micro Devices, Inc.
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

#include "gsgpu_reset.h"

int gsgpu_reset_add_handler(struct gsgpu_reset_control *reset_ctl,
			     struct gsgpu_reset_handler *handler)
{
	/* TODO: Check if handler exists? */
	list_add_tail(&handler->handler_list, &reset_ctl->reset_handlers);
	return 0;
}

int gsgpu_reset_init(struct gsgpu_device *adev)
{
	int ret = 0;

	return ret;
}

int gsgpu_reset_fini(struct gsgpu_device *adev)
{
	int ret = 0;

	return ret;
}

int gsgpu_reset_prepare_hwcontext(struct gsgpu_device *adev,
				   struct gsgpu_reset_context *reset_context)
{
	struct gsgpu_reset_handler *reset_handler = NULL;

	if (adev->reset_cntl && adev->reset_cntl->get_reset_handler)
		reset_handler = adev->reset_cntl->get_reset_handler(
			adev->reset_cntl, reset_context);
	if (!reset_handler)
		return -ENOSYS;

	return reset_handler->prepare_hwcontext(adev->reset_cntl,
						reset_context);
}

int gsgpu_reset_perform_reset(struct gsgpu_device *adev,
			       struct gsgpu_reset_context *reset_context)
{
	int ret;
	struct gsgpu_reset_handler *reset_handler = NULL;

	if (adev->reset_cntl)
		reset_handler = adev->reset_cntl->get_reset_handler(
			adev->reset_cntl, reset_context);
	if (!reset_handler)
		return -ENOSYS;

	ret = reset_handler->perform_reset(adev->reset_cntl, reset_context);
	if (ret)
		return ret;

	return reset_handler->restore_hwcontext(adev->reset_cntl,
						reset_context);
}


void gsgpu_reset_destroy_reset_domain(struct kref *ref)
{
	struct gsgpu_reset_domain *reset_domain = container_of(ref,
								struct gsgpu_reset_domain,
								refcount);
	if (reset_domain->wq)
		destroy_workqueue(reset_domain->wq);

	kvfree(reset_domain);
}

struct gsgpu_reset_domain *gsgpu_reset_create_reset_domain(enum gsgpu_reset_domain_type type,
							     char *wq_name)
{
	struct gsgpu_reset_domain *reset_domain;

	reset_domain = kvzalloc(sizeof(struct gsgpu_reset_domain), GFP_KERNEL);
	if (!reset_domain) {
		DRM_ERROR("Failed to allocate gsgpu_reset_domain!");
		return NULL;
	}

	reset_domain->type = type;
	kref_init(&reset_domain->refcount);

	reset_domain->wq = create_singlethread_workqueue(wq_name);
	if (!reset_domain->wq) {
		DRM_ERROR("Failed to allocate wq for gsgpu_reset_domain!");
		gsgpu_reset_put_reset_domain(reset_domain);
		return NULL;

	}

	atomic_set(&reset_domain->in_gpu_reset, 0);
	atomic_set(&reset_domain->reset_res, 0);
	init_rwsem(&reset_domain->sem);

	return reset_domain;
}

void gsgpu_device_lock_reset_domain(struct gsgpu_reset_domain *reset_domain)
{
	atomic_set(&reset_domain->in_gpu_reset, 1);
	down_write(&reset_domain->sem);
}


void gsgpu_device_unlock_reset_domain(struct gsgpu_reset_domain *reset_domain)
{
	atomic_set(&reset_domain->in_gpu_reset, 0);
	up_write(&reset_domain->sem);
}



