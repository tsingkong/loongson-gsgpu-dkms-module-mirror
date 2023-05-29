/* gsgpu_drv.h -- Private header for gsgpu driver -*- linux-c -*-
 *
 * Copyright 1999 Precision Insight, Inc., Cedar Park, Texas.
 * Copyright 2000 VA Linux Systems, Inc., Fremont, California.
 * All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * PRECISION INSIGHT AND/OR ITS SUPPLIERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 */

#ifndef __GSGPU_DRV_H__
#define __GSGPU_DRV_H__

#include <linux/firmware.h>
#include <linux/platform_device.h>

#include "gsgpu_shared.h"

/* General customization:
 */

#define DRIVER_AUTHOR		"Loongson graphics driver team"

#define DRIVER_NAME		"gsgpu"
#define DRIVER_DESC		"GS GPU Driver"
#define DRIVER_DATE		"20200501"

long gsgpu_drm_ioctl(struct file *filp,
		      unsigned int cmd, unsigned long arg);

long gsgpu_kms_compat_ioctl(struct file *filp,
			     unsigned int cmd, unsigned long arg);

#endif
