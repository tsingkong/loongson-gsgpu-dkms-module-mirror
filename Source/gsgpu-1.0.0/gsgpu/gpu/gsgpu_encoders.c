/*
 * Copyright 2007-8 Advanced Micro Devices, Inc.
 * Copyright 2008 Red Hat Inc.
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
 * Authors: Dave Airlie
 *          Alex Deucher
 */

#include <drm/gsgpu_drm.h>
#include "gsgpu.h"
// #include "gsgpu_connectors.h"
#include "gsgpu_display.h"

struct drm_connector *
gsgpu_get_connector_for_encoder(struct drm_encoder *encoder)
{
	struct drm_device *dev = encoder->dev;
	struct gsgpu_encoder *gsgpu_encoder = to_gsgpu_encoder(encoder);
	struct drm_connector *connector, *found = NULL;
	struct drm_connector_list_iter iter;
	struct gsgpu_connector *gsgpu_connector;

	drm_connector_list_iter_begin(dev, &iter);
	drm_for_each_connector_iter(connector, &iter) {
		gsgpu_connector = to_gsgpu_connector(connector);
		if (gsgpu_encoder->active_device & gsgpu_connector->devices) {
			found = connector;
			break;
		}
	}
	drm_connector_list_iter_end(&iter);
	return found;
}
