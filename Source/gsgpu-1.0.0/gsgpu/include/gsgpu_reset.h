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

#ifndef __GSGPU_RESET_H__
#define __GSGPU_RESET_H__

#include "gsgpu.h"

enum GSGPU_RESET_FLAGS {

	GSGPU_NEED_FULL_RESET = 0,
	GSGPU_SKIP_HW_RESET = 1,
	GSGPU_RESET_FOR_DEVICE_REMOVE = 2,
};

struct gsgpu_reset_context {
	enum gsgpu_reset_method method;
	struct gsgpu_device *reset_req_dev;
	struct gsgpu_job *job;
	struct list_head *reset_device_list;
	unsigned long flags;
};

struct gsgpu_reset_handler {
	enum gsgpu_reset_method reset_method;
	struct list_head handler_list;
	int (*prepare_env)(struct gsgpu_reset_control *reset_ctl,
			   struct gsgpu_reset_context *context);
	int (*prepare_hwcontext)(struct gsgpu_reset_control *reset_ctl,
				 struct gsgpu_reset_context *context);
	int (*perform_reset)(struct gsgpu_reset_control *reset_ctl,
			     struct gsgpu_reset_context *context);
	int (*restore_hwcontext)(struct gsgpu_reset_control *reset_ctl,
				 struct gsgpu_reset_context *context);
	int (*restore_env)(struct gsgpu_reset_control *reset_ctl,
			   struct gsgpu_reset_context *context);

	int (*do_reset)(struct gsgpu_device *adev);
};

struct gsgpu_reset_control {
	void *handle;
	struct work_struct reset_work;
	struct mutex reset_lock;
	struct list_head reset_handlers;
	atomic_t in_reset;
	enum gsgpu_reset_method active_reset;
	struct gsgpu_reset_handler *(*get_reset_handler)(
		struct gsgpu_reset_control *reset_ctl,
		struct gsgpu_reset_context *context);
	void (*async_reset)(struct work_struct *work);
};


enum gsgpu_reset_domain_type {
	SINGLE_DEVICE,
	XGMI_HIVE
};

struct gsgpu_reset_domain {
	struct kref refcount;
	struct workqueue_struct *wq;
	enum gsgpu_reset_domain_type type;
	struct rw_semaphore sem;
	atomic_t in_gpu_reset;
	atomic_t reset_res;
};


int gsgpu_reset_init(struct gsgpu_device *adev);
int gsgpu_reset_fini(struct gsgpu_device *adev);

int gsgpu_reset_prepare_hwcontext(struct gsgpu_device *adev,
				   struct gsgpu_reset_context *reset_context);

int gsgpu_reset_perform_reset(struct gsgpu_device *adev,
			       struct gsgpu_reset_context *reset_context);

int gsgpu_reset_add_handler(struct gsgpu_reset_control *reset_ctl,
			     struct gsgpu_reset_handler *handler);

struct gsgpu_reset_domain *gsgpu_reset_create_reset_domain(enum gsgpu_reset_domain_type type,
							     char *wq_name);

void gsgpu_reset_destroy_reset_domain(struct kref *ref);

static inline bool gsgpu_reset_get_reset_domain(struct gsgpu_reset_domain *domain)
{
	return kref_get_unless_zero(&domain->refcount) != 0;
}

static inline void gsgpu_reset_put_reset_domain(struct gsgpu_reset_domain *domain)
{
	if (domain)
		kref_put(&domain->refcount, gsgpu_reset_destroy_reset_domain);
}

static inline bool gsgpu_reset_domain_schedule(struct gsgpu_reset_domain *domain,
						struct work_struct *work)
{
	return queue_work(domain->wq, work);
}

void gsgpu_device_lock_reset_domain(struct gsgpu_reset_domain *reset_domain);

void gsgpu_device_unlock_reset_domain(struct gsgpu_reset_domain *reset_domain);

#endif
