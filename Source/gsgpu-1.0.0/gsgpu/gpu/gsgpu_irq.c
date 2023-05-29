/*
 * Copyright 2008 Advanced Micro Devices, Inc.
 * Copyright 2008 Red Hat Inc.
 * Copyright 2009 Jerome Glisse.
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
 *          Jerome Glisse
 */

/**
 * DOC: Interrupt Handling
 *
 * Interrupts generated within GPU hardware raise interrupt requests that are
 * passed to gsgpu IRQ handler which is responsible for detecting source and
 * type of the interrupt and dispatching matching handlers. If handling an
 * interrupt requires calling kernel functions that may sleep processing is
 * dispatched to work handlers.
 *
 * If MSI functionality is not disabled by module parameter then MSI
 * support will be enabled.
 *
 * For GPU interrupt sources that may be driven by another driver, IRQ domain
 * support is used (with mapping between virtual and hardware IRQs).
 */

#include <linux/irq.h>
#include <linux/pci.h>

#include <drm/drm_vblank.h>
#include <drm/gsgpu_drm.h>
#include <drm/drm_drv.h>
#include "gsgpu.h"
#include "gsgpu_ih.h"
#include "gsgpu_trace.h"
#include "gsgpu_dc_irq.h"
#include "gsgpu_dc_reg.h"
#include "ivsrcid/ivsrcid_vislands30.h"

#include <linux/pm_runtime.h>

#ifdef CONFIG_DRM_GSGPU_DC
#include "gsgpu_dm_irq.h"
#endif

#define GSGPU_WAIT_IDLE_TIMEOUT 200

const char *soc15_ih_clientid_name[] = {
	"IH",
	"SDMA2 or ACP",
	"ATHUB",
	"BIF",
	"SDMA3 or DCE",
	"SDMA4 or ISP",
	"VMC1 or PCIE0",
	"RLC",
	"SDMA0",
	"SDMA1",
	"SE0SH",
	"SE1SH",
	"SE2SH",
	"SE3SH",
	"VCN1 or UVD1",
	"THM",
	"VCN or UVD",
	"SDMA5 or VCE0",
	"VMC",
	"SDMA6 or XDMA",
	"GRBM_CP",
	"ATS",
	"ROM_SMUIO",
	"DF",
	"SDMA7 or VCE1",
	"PWR",
	"reserved",
	"UTCL2",
	"EA",
	"UTCL2LOG",
	"MP0",
	"MP1"
};

/**
 * gsgpu_irq_disable_all - disable *all* interrupts
 *
 * @adev: gsgpu device pointer
 *
 * Disable all types of interrupts from all sources.
 */
void gsgpu_irq_disable_all(struct gsgpu_device *adev)
{
	unsigned long irqflags;
	unsigned i, j, k;
	int r;

	spin_lock_irqsave(&adev->irq.lock, irqflags);
	for (i = 0; i < GSGPU_IRQ_CLIENTID_MAX; ++i) {
		if (!adev->irq.client[i].sources)
			continue;

		for (j = 0; j < GSGPU_MAX_IRQ_SRC_ID; ++j) {
			struct gsgpu_irq_src *src = adev->irq.client[i].sources[j];

			if (!src || !src->funcs->set || !src->num_types)
				continue;

			for (k = 0; k < src->num_types; ++k) {
				atomic_set(&src->enabled_types[k], 0);
				r = src->funcs->set(adev, src, k,
						    GSGPU_IRQ_STATE_DISABLE);
				if (r)
					DRM_ERROR("error disabling interrupt (%d)\n",
						  r);
			}
		}
	}
	spin_unlock_irqrestore(&adev->irq.lock, irqflags);
}

/**
 * gsgpu_irq_handler - IRQ handler
 *
 * @irq: IRQ number (unused)
 * @arg: pointer to DRM device
 *
 * IRQ handler for gsgpu driver (all ASICs).
 *
 * Returns:
 * result of handling the IRQ, as defined by &irqreturn_t
 */
static irqreturn_t gsgpu_irq_handler(int irq, void *arg)
{
	struct drm_device *dev = (struct drm_device *) arg;
	struct gsgpu_device *adev = drm_to_adev(dev);
	irqreturn_t ret;

	ret = gsgpu_ih_process(adev, &adev->irq.ih);
	if (ret == IRQ_HANDLED)
		pm_runtime_mark_last_busy(dev->dev);

	return ret;
}

/**
 * gsgpu_irq_handle_ih1 - kick of processing for IH1
 *
 * @work: work structure in struct gsgpu_irq
 *
 * Kick of processing IH ring 1.
 */
static void gsgpu_irq_handle_ih1(struct work_struct *work)
{
	struct gsgpu_device *adev = container_of(work, struct gsgpu_device,
						  irq.ih1_work);

	gsgpu_ih_process(adev, &adev->irq.ih1);
}

/**
 * gsgpu_irq_handle_ih2 - kick of processing for IH2
 *
 * @work: work structure in struct gsgpu_irq
 *
 * Kick of processing IH ring 2.
 */
static void gsgpu_irq_handle_ih2(struct work_struct *work)
{
	struct gsgpu_device *adev = container_of(work, struct gsgpu_device,
						  irq.ih2_work);

	gsgpu_ih_process(adev, &adev->irq.ih2);
}

/**
 * gsgpu_irq_handle_ih_soft - kick of processing for ih_soft
 *
 * @work: work structure in struct gsgpu_irq
 *
 * Kick of processing IH soft ring.
 */
static void gsgpu_irq_handle_ih_soft(struct work_struct *work)
{
	struct gsgpu_device *adev = container_of(work, struct gsgpu_device,
						  irq.ih_soft_work);

	gsgpu_ih_process(adev, &adev->irq.ih_soft);
}

static irqreturn_t gsgpu_dc_irq_handler(int irq, void *arg)
{
	u32 int_reg;
	unsigned long base;
	struct gsgpu_iv_entry entry;
	struct gsgpu_device *adev = (struct gsgpu_device *)arg;
	int i = 1;

	base = (unsigned long)(adev->loongson_dc_rmmio_base);

	int_reg = dc_readl(adev, DC_INT_REG);
	entry.client_id = SOC15_IH_CLIENTID_DCE;

	int_reg &= 0xffff;
	while (int_reg && (i < DC_INT_ID_MAX)) {
		if (!(int_reg & 0x1)) {
			int_reg = int_reg >> 1;
			i++;
			continue;
		}

		entry.src_id = i;
		// gsgpu_irq_dispatch(adev, &entry);
		gsgpu_irq_delegate(adev, &entry, 1);

		int_reg = int_reg >> 1;
		i++;
	}

	return IRQ_HANDLED;
}

/**
 * gsgpu_msi_ok - check whether MSI functionality is enabled
 *
 * @adev: gsgpu device pointer (unused)
 *
 * Checks whether MSI functionality has been disabled via module parameter
 * (all ASICs).
 *
 * Returns:
 * *true* if MSIs are allowed to be enabled or *false* otherwise
 */
static bool gsgpu_msi_ok(struct gsgpu_device *adev)
{
	if (gsgpu_msi == 1)
		return true;
	else if (gsgpu_msi == 0)
		return false;

	return true;
}

// static void gsgpu_restore_msix(struct gsgpu_device *adev)
// {
// 	u16 ctrl;

// 	pci_read_config_word(adev->pdev, adev->pdev->msix_cap + PCI_MSIX_FLAGS, &ctrl);
// 	if (!(ctrl & PCI_MSIX_FLAGS_ENABLE))
// 		return;

// 	/* VF FLR */
// 	ctrl &= ~PCI_MSIX_FLAGS_ENABLE;
// 	pci_write_config_word(adev->pdev, adev->pdev->msix_cap + PCI_MSIX_FLAGS, ctrl);
// 	ctrl |= PCI_MSIX_FLAGS_ENABLE;
// 	pci_write_config_word(adev->pdev, adev->pdev->msix_cap + PCI_MSIX_FLAGS, ctrl);
// }

/**
 * gsgpu_irq_init - initialize interrupt handling
 *
 * @adev: gsgpu device pointer
 *
 * Sets up work functions for hotplug and reset interrupts, enables MSI
 * functionality, initializes vblank, hotplug and reset interrupt handling.
 *
 * Returns:
 * 0 on success or error code on failure
 */
int gsgpu_irq_init(struct gsgpu_device *adev)
{
	int r = 0;
	unsigned int irq;
	struct pci_dev *loongson_dc;

	spin_lock_init(&adev->irq.lock);

	/* Enable MSI if not disabled by module parameter */
	adev->irq.msi_enabled = false;

	if (gsgpu_msi_ok(adev)) {
		int nvec = pci_msix_vec_count(adev->pdev);
		unsigned int flags;

		if (nvec <= 0) {
			flags = PCI_IRQ_MSI;
		} else {
			flags = PCI_IRQ_MSI | PCI_IRQ_MSIX;
		}
		/* we only need one vector */
		nvec = pci_alloc_irq_vectors(adev->pdev, 1, 1, flags);
		if (nvec > 0) {
			adev->irq.msi_enabled = true;
			dev_dbg(adev->dev, "using MSI/MSI-X.\n");
		}
	}

	INIT_WORK(&adev->irq.ih1_work, gsgpu_irq_handle_ih1);
	INIT_WORK(&adev->irq.ih2_work, gsgpu_irq_handle_ih2);
	INIT_WORK(&adev->irq.ih_soft_work, gsgpu_irq_handle_ih_soft);

	/* Use vector 0 for MSI-X. */
	r = pci_irq_vector(adev->pdev, 0);
	if (r < 0)
		return r;
	irq = r;

	/* PCI devices require shared interrupts. */
	r = request_irq(irq, gsgpu_irq_handler, IRQF_SHARED, adev_to_drm(adev)->driver->name,
			adev_to_drm(adev));
	if (r)
		return r;
	adev->irq.installed = true;
	adev->irq.irq = irq;
	adev_to_drm(adev)->max_vblank_count = 0x00ffffff;

	loongson_dc = adev->loongson_dc;
	if (loongson_dc) {
		u32 dc_irq = loongson_dc->irq;
		r = request_irq(dc_irq, gsgpu_dc_irq_handler, 0,
				loongson_dc->driver->name, adev);
		if (r) {
			DRM_ERROR("gsgpu register dc irq failed\n");
			return r;
		}
	} else {
		DRM_ERROR("gsgpu dc device is null\n");
		return ENODEV;
	}

	DRM_DEBUG("gsgpu: irq initialized.\n");
	return 0;
}


void gsgpu_irq_fini_hw(struct gsgpu_device *adev)
{
	if (adev->irq.installed) {
		free_irq(adev->irq.irq, adev_to_drm(adev));
		adev->irq.installed = false;
		if (adev->irq.msi_enabled)
			pci_free_irq_vectors(adev->pdev);
	}

	gsgpu_ih_ring_fini(adev, &adev->irq.ih_soft);
	gsgpu_ih_ring_fini(adev, &adev->irq.ih);
	gsgpu_ih_ring_fini(adev, &adev->irq.ih1);
	gsgpu_ih_ring_fini(adev, &adev->irq.ih2);
}

/**
 * gsgpu_irq_fini_sw - shut down interrupt handling
 *
 * @adev: gsgpu device pointer
 *
 * Tears down work functions for hotplug and reset interrupts, disables MSI
 * functionality, shuts down vblank, hotplug and reset interrupt handling,
 * turns off interrupts from all sources (all ASICs).
 */
void gsgpu_irq_fini_sw(struct gsgpu_device *adev)
{
	unsigned i, j;

	for (i = 0; i < GSGPU_IRQ_CLIENTID_MAX; ++i) {
		if (!adev->irq.client[i].sources)
			continue;

		for (j = 0; j < GSGPU_MAX_IRQ_SRC_ID; ++j) {
			struct gsgpu_irq_src *src = adev->irq.client[i].sources[j];

			if (!src)
				continue;

			kfree(src->enabled_types);
			src->enabled_types = NULL;
		}
		kfree(adev->irq.client[i].sources);
		adev->irq.client[i].sources = NULL;
	}
}

/**
 * gsgpu_irq_add_id - register IRQ source
 *
 * @adev: gsgpu device pointer
 * @client_id: client id
 * @src_id: source id
 * @source: IRQ source pointer
 *
 * Registers IRQ source on a client.
 *
 * Returns:
 * 0 on success or error code otherwise
 */
int gsgpu_irq_add_id(struct gsgpu_device *adev,
		      unsigned client_id, unsigned src_id,
		      struct gsgpu_irq_src *source)
{
	if (client_id >= GSGPU_IRQ_CLIENTID_MAX)
		return -EINVAL;

	if (src_id >= GSGPU_MAX_IRQ_SRC_ID)
		return -EINVAL;

	if (!source->funcs)
		return -EINVAL;

	if (!adev->irq.client[client_id].sources) {
		adev->irq.client[client_id].sources =
			kcalloc(GSGPU_MAX_IRQ_SRC_ID,
				sizeof(struct gsgpu_irq_src *),
				GFP_KERNEL);
		if (!adev->irq.client[client_id].sources)
			return -ENOMEM;
	}

	if (adev->irq.client[client_id].sources[src_id] != NULL)
		return -EINVAL;

	if (source->num_types && !source->enabled_types) {
		atomic_t *types;

		types = kcalloc(source->num_types, sizeof(atomic_t),
				GFP_KERNEL);
		if (!types)
			return -ENOMEM;

		source->enabled_types = types;
	}

	adev->irq.client[client_id].sources[src_id] = source;
	return 0;
}

/**
 * gsgpu_irq_dispatch - dispatch IRQ to IP blocks
 *
 * @adev: gsgpu device pointer
 * @ih: interrupt ring instance
 *
 * Dispatches IRQ to IP blocks.
 */
void gsgpu_irq_dispatch(struct gsgpu_device *adev,
			 struct gsgpu_ih_ring *ih)
{
	u32 ring_index = ih->rptr;
	struct gsgpu_iv_entry entry;
	unsigned client_id, src_id;
	struct gsgpu_irq_src *src;
	bool handled = false;
	int r;

	entry.ih = ih;
	entry.iv_entry = (const uint32_t *)&ih->ring[ring_index];
	gsgpu_ih_decode_iv(adev, &entry);

	trace_gsgpu_iv(ih - &adev->irq.ih, &entry);

	client_id = entry.client_id;
	src_id = entry.src_id;

	if (client_id >= GSGPU_IRQ_CLIENTID_MAX) {
		DRM_DEBUG("Invalid client_id in IV: %d\n", client_id);

	} else	if (src_id >= GSGPU_MAX_IRQ_SRC_ID) {
		DRM_DEBUG("Invalid src_id in IV: %d\n", src_id);

	} else if (!adev->irq.client[client_id].sources) {
		DRM_DEBUG("Unregistered interrupt client_id: %d src_id: %d\n",
			  client_id, src_id);

	} else if ((src = adev->irq.client[client_id].sources[src_id])) {
		r = src->funcs->process(adev, src, &entry);
		if (r < 0)
			DRM_ERROR("error processing interrupt (%d)\n", r);
		else if (r)
			handled = true;

	} else {
		DRM_DEBUG("Unhandled interrupt src_id: %d\n", src_id);
	}

	if (gsgpu_ih_ts_after(ih->processed_timestamp, entry.timestamp))
		ih->processed_timestamp = entry.timestamp;
}

/**
 * gsgpu_irq_delegate - delegate IV to soft IH ring
 *
 * @adev: gsgpu device pointer
 * @entry: IV entry
 * @num_dw: size of IV
 *
 * Delegate the IV to the soft IH ring and schedule processing of it. Used
 * if the hardware delegation to IH1 or IH2 doesn't work for some reason.
 */
void gsgpu_irq_delegate(struct gsgpu_device *adev,
			 struct gsgpu_iv_entry *entry,
			 unsigned int num_dw)
{
	gsgpu_ih_ring_write(&adev->irq.ih_soft, entry->iv_entry, num_dw);
	schedule_work(&adev->irq.ih_soft_work);
}

/**
 * gsgpu_irq_update - update hardware interrupt state
 *
 * @adev: gsgpu device pointer
 * @src: interrupt source pointer
 * @type: type of interrupt
 *
 * Updates interrupt state for the specific source (all ASICs).
 */
int gsgpu_irq_update(struct gsgpu_device *adev,
			     struct gsgpu_irq_src *src, unsigned type)
{
	unsigned long irqflags;
	enum gsgpu_interrupt_state state;
	int r;

	spin_lock_irqsave(&adev->irq.lock, irqflags);

	/* We need to determine after taking the lock, otherwise
	   we might disable just enabled interrupts again */
	if (gsgpu_irq_enabled(adev, src, type))
		state = GSGPU_IRQ_STATE_ENABLE;
	else
		state = GSGPU_IRQ_STATE_DISABLE;

	r = src->funcs->set(adev, src, type, state);
	spin_unlock_irqrestore(&adev->irq.lock, irqflags);
	return r;
}

/**
 * gsgpu_irq_gpu_reset_resume_helper - update interrupt states on all sources
 *
 * @adev: gsgpu device pointer
 *
 * Updates state of all types of interrupts on all sources on resume after
 * reset.
 */
void gsgpu_irq_gpu_reset_resume_helper(struct gsgpu_device *adev)
{
	int i, j, k;

	for (i = 0; i < GSGPU_IRQ_CLIENTID_MAX; ++i) {
		if (!adev->irq.client[i].sources)
			continue;

		for (j = 0; j < GSGPU_MAX_IRQ_SRC_ID; ++j) {
			struct gsgpu_irq_src *src = adev->irq.client[i].sources[j];

			if (!src || !src->funcs || !src->funcs->set)
				continue;
			for (k = 0; k < src->num_types; k++)
				gsgpu_irq_update(adev, src, k);
		}
	}
}

/**
 * gsgpu_irq_get - enable interrupt
 *
 * @adev: gsgpu device pointer
 * @src: interrupt source pointer
 * @type: type of interrupt
 *
 * Enables specified type of interrupt on the specified source (all ASICs).
 *
 * Returns:
 * 0 on success or error code otherwise
 */
int gsgpu_irq_get(struct gsgpu_device *adev, struct gsgpu_irq_src *src,
		   unsigned type)
{
	if (!adev->irq.installed)
		return -ENOENT;

	if (type >= src->num_types)
		return -EINVAL;

	if (!src->enabled_types || !src->funcs->set)
		return -EINVAL;

	if (atomic_inc_return(&src->enabled_types[type]) == 1)
		return gsgpu_irq_update(adev, src, type);

	return 0;
}

/**
 * gsgpu_irq_put - disable interrupt
 *
 * @adev: gsgpu device pointer
 * @src: interrupt source pointer
 * @type: type of interrupt
 *
 * Enables specified type of interrupt on the specified source (all ASICs).
 *
 * Returns:
 * 0 on success or error code otherwise
 */
int gsgpu_irq_put(struct gsgpu_device *adev, struct gsgpu_irq_src *src,
		   unsigned type)
{
	if (!adev->irq.installed)
		return -ENOENT;

	if (type >= src->num_types)
		return -EINVAL;

	if (!src->enabled_types || !src->funcs->set)
		return -EINVAL;

	if (WARN_ON(!gsgpu_irq_enabled(adev, src, type)))
		return -EINVAL;

	if (atomic_dec_and_test(&src->enabled_types[type]))
		return gsgpu_irq_update(adev, src, type);

	return 0;
}

/**
 * gsgpu_irq_enabled - check whether interrupt is enabled or not
 *
 * @adev: gsgpu device pointer
 * @src: interrupt source pointer
 * @type: type of interrupt
 *
 * Checks whether the given type of interrupt is enabled on the given source.
 *
 * Returns:
 * *true* if interrupt is enabled, *false* if interrupt is disabled or on
 * invalid parameters
 */
bool gsgpu_irq_enabled(struct gsgpu_device *adev, struct gsgpu_irq_src *src,
			unsigned type)
{
	if (!adev->irq.installed)
		return false;

	if (type >= src->num_types)
		return false;

	if (!src->enabled_types || !src->funcs->set)
		return false;

	return !!atomic_read(&src->enabled_types[type]);
}
