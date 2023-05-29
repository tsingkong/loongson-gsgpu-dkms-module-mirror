/*
 * Copyright 2014 Advanced Micro Devices, Inc.
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

#include <linux/dma-mapping.h>

#include "gsgpu.h"
#include "gsgpu_ih.h"
#include "ivsrcid/ivsrcid_vislands30.h"

/*
 * Interrupts
 * Starting with r6xx, interrupts are handled via a ring buffer.
 * Ring buffers are areas of GPU accessible memory that the GPU
 * writes interrupt vectors into and the host reads vectors out of.
 * There is a rptr (read pointer) that determines where the
 * host is currently reading, and a wptr (write pointer)
 * which determines where the GPU has written.  When the
 * pointers are equal, the ring is idle.  When the GPU
 * writes vectors to the ring buffer, it increments the
 * wptr.  When there is an interrupt, the host then starts
 * fetching commands and processing them until the pointers are
 * equal again at which point it updates the rptr.
 */

static void gsgpu_ih_set_interrupt_funcs(struct gsgpu_device *adev);

/**
 * gsgpu_ih_enable_interrupts - Enable the interrupt ring buffer
 *
 * @adev: gsgpu_device pointer
 *
 * Enable the interrupt ring buffer .
 */
static void gsgpu_ih_enable_interrupts(struct gsgpu_device *adev)
{
	adev->irq.ih.enabled = true;
}

/**
 * gsgpu_ih_disable_interrupts - Disable the interrupt ring buffer
 *
 * @adev: gsgpu_device pointer
 *
 * Disable the interrupt ring buffer .
 */
static void gsgpu_ih_disable_interrupts(struct gsgpu_device *adev)
{
	adev->irq.ih.enabled = false;
	adev->irq.ih.rptr = 0;
}

/**
 * gsgpu_ih_irq_init - init and enable the interrupt ring
 *
 * @adev: gsgpu_device pointer
 *
 * Allocate a ring buffer for the interrupt controller,
 * enable the RLC, disable interrupts, enable the IH
 * ring buffer and enable it .
 * Called at device load and reume.
 * Returns 0 for success, errors for failure.
 */
static int gsgpu_ih_irq_init(struct gsgpu_device *adev)
{
	// struct gsgpu_ih_ring *ih = &adev->irq.ih;
	int rb_bufsz;
	u64 wptr_off;

	/* disable irqs */
	gsgpu_ih_disable_interrupts(adev);

	rb_bufsz = adev->irq.ih.ring_size / 4;
	WREG32(GSGPU_INT_CB_SIZE_OFFSET, rb_bufsz);

	/* set the writeback address whether it's enabled or not */
	wptr_off = adev->wb.gpu_addr;
	WREG32(GSGPU_INT_CB_BASE_LO_OFFSET, lower_32_bits(wptr_off));
	WREG32(GSGPU_INT_CB_BASE_HI_OFFSET, upper_32_bits(wptr_off));

	/* set rptr, wptr to 0 */
	WREG32(GSGPU_INT_CB_WPTR_OFFSET, 0);
	WREG32(GSGPU_INT_CB_RPTR_OFFSET, 0);

	pci_set_master(adev->pdev);

	/* enable interrupts */
	gsgpu_ih_enable_interrupts(adev);

	return 0;
}

/**
 * gsgpu_ih_irq_disable - disable interrupts
 *
 * @adev: gsgpu_device pointer
 *
 * Disable interrupts on the hw .
 */
static void gsgpu_ih_irq_disable(struct gsgpu_device *adev)
{
	gsgpu_ih_disable_interrupts(adev);

	/* Wait and acknowledge irq */
	mdelay(1);
}

/**
 * ih_func_get_wptr - get the IH ring buffer wptr
 *
 * @adev: gsgpu_device pointer
 * @ih: IH ring buffer to fetch wptr
 *
 * Get the IH ring buffer wptr from either the register
 * or the writeback memory buffer .  Also check for
 * ring buffer overflow and deal with it.
 * Used by gsgpu_irq_process.
 * Returns the value of the wptr.
 */
static u32 ih_func_get_wptr(struct gsgpu_device *adev,
			  struct gsgpu_ih_ring *ih)
{
	u32 wptr;
	wptr = le32_to_cpu(RREG32(GSGPU_INT_CB_WPTR_OFFSET));
	
	return (wptr & ih->ptr_mask);
}

/**
 * ih_func_decode_iv - decode an interrupt vector
 *
 * @adev: gsgpu_device pointer
 * @ih: IH ring buffer to decode
 * @entry: IV entry to place decoded information into
 *
 * Decodes the interrupt vector at the current rptr
 * position and also advance the position.
 */
static void ih_func_decode_iv(struct gsgpu_device *adev,
			    struct gsgpu_ih_ring *ih,
			    struct gsgpu_iv_entry *entry)
{
	/* wptr/rptr are in bytes! */
	u32 ring_index = adev->irq.ih.rptr;
	uint32_t dw[4];

	dw[0] = le32_to_cpu(ih->ring[ring_index + 0]);
	dw[1] = le32_to_cpu(ih->ring[ring_index + 1]);
	dw[2] = le32_to_cpu(ih->ring[ring_index + 2]);
	dw[3] = le32_to_cpu(ih->ring[ring_index + 3]);

	DRM_DEBUG("ih_func_decode_iv dw0 %x dw1 %x dw2 %x dw3 %x \n", dw[0], dw[1], dw[2], dw[3]);

	entry->client_id = GSGPU_IRQ_CLIENTID_LEGACY;
	entry->src_id = dw[0] & 0xff;
	entry->src_data[0] = dw[1] & 0xffffffff;
	entry->ring_id = dw[2] & 0xff;
	entry->vmid = (dw[2] >> 8) & 0xff;
	entry->pasid = (dw[2] >> 16) & 0xffff;
	entry->src_data[1] = dw[3];

	/* wptr/rptr are in bytes! */
	adev->irq.ih.rptr += 4;
}

/**
 * ih_func_set_rptr - set the IH ring buffer rptr
 *
 * @adev: gsgpu_device pointer
 * @ih: IH ring buffer to set rptr
 *
 * Set the IH ring buffer rptr.
 */
static void ih_func_set_rptr(struct gsgpu_device *adev,
			   struct gsgpu_ih_ring *ih)
{
	WREG32(GSGPU_INT_CB_RPTR_OFFSET, adev->irq.ih.rptr);
}

static int gsgpu_ih_early_init(void *handle)
{
	struct gsgpu_device *adev = (struct gsgpu_device *)handle;

	gsgpu_ih_set_interrupt_funcs(adev);

	return 0;
}

static int gsgpu_ih_sw_init(void *handle)
{
	int r;
	struct gsgpu_device *adev = (struct gsgpu_device *)handle;

	r = gsgpu_ih_ring_init(adev, &adev->irq.ih, 4 * 1024, true);
	if (r)
		return r;

	r = gsgpu_irq_init(adev);
	if (r)
		return r;

	r = gsgpu_hw_sema_mgr_init(adev);

	return r;
}

static int gsgpu_ih_sw_fini(void *handle)
{
	struct gsgpu_device *adev = (struct gsgpu_device *)handle;

	gsgpu_irq_fini_sw(adev);
	gsgpu_hw_sema_mgr_fini(adev);

	return 0;
}

static int gsgpu_ih_hw_init(void *handle)
{
	int r;
	struct gsgpu_device *adev = (struct gsgpu_device *)handle;

	r = gsgpu_ih_irq_init(adev);
	if (r)
		return r;

	return 0;
}

static int gsgpu_ih_hw_fini(void *handle)
{
	struct gsgpu_device *adev = (struct gsgpu_device *)handle;

	gsgpu_ih_irq_disable(adev);

	return 0;
}

static int gsgpu_ih_suspend(void *handle)
{
	struct gsgpu_device *adev = (struct gsgpu_device *)handle;

	return gsgpu_ih_hw_fini(adev);
}

static int gsgpu_ih_resume(void *handle)
{
	struct gsgpu_device *adev = (struct gsgpu_device *)handle;

	return gsgpu_ih_hw_init(adev);
}

static bool gsgpu_ih_is_idle(void *handle)
{
	return true;
}

static int gsgpu_ih_wait_for_idle(void *handle)
{
	return 0;
}

static const struct gsgpu_ip_funcs gsgpu_ih_ip_funcs = {
	.name = "gsgpu_ih",
	.early_init = gsgpu_ih_early_init,
	.late_init = NULL,
	.sw_init = gsgpu_ih_sw_init,
	.sw_fini = gsgpu_ih_sw_fini,
	.hw_init = gsgpu_ih_hw_init,
	.hw_fini = gsgpu_ih_hw_fini,
	.suspend = gsgpu_ih_suspend,
	.resume = gsgpu_ih_resume,
	.is_idle = gsgpu_ih_is_idle,
	.wait_for_idle = gsgpu_ih_wait_for_idle,
};

static const struct gsgpu_ih_funcs gsgpu_ih_funcs = {
	.get_wptr = ih_func_get_wptr,
	.decode_iv = ih_func_decode_iv,
	.decode_iv_ts = NULL,
	.set_rptr = ih_func_set_rptr,
};

static void gsgpu_ih_set_interrupt_funcs(struct gsgpu_device *adev)
{
	adev->irq.ih_funcs = &gsgpu_ih_funcs;
}

const struct gsgpu_ip_block_version gsgpu_ih_ip_block =
{
	.type = GSGPU_IP_BLOCK_TYPE_IH,
	.major = 3,
	.minor = 0,
	.rev = 0,
	.funcs = &gsgpu_ih_ip_funcs,
};

// ================================================================

/**
 * gsgpu_ih_ring_init - initialize the IH state
 *
 * @adev: gsgpu_device pointer
 * @ih: ih ring to initialize
 * @ring_size: ring size to allocate
 * @use_bus_addr: true when we can use dma_alloc_coherent
 *
 * Initializes the IH state and allocates a buffer
 * for the IH ring buffer.
 * Returns 0 for success, errors for failure.
 */
int gsgpu_ih_ring_init(struct gsgpu_device *adev, struct gsgpu_ih_ring *ih,
			unsigned ring_size, bool use_bus_addr)
{
	u32 rb_bufsz;
	int r;

	/* Align ring size */
	rb_bufsz = order_base_2(ring_size / 4);
	ring_size = (1 << rb_bufsz) * 4;
	ih->ring_size = ring_size;
	ih->ptr_mask = ih->ring_size / 4 - 1;
	ih->rptr = 0;
	ih->use_bus_addr = use_bus_addr;

	if (use_bus_addr) {
		dma_addr_t dma_addr;

		if (ih->ring)
			return 0;

		/* add 8 bytes for the rptr/wptr shadows and
		 * add them to the end of the ring allocation.
		 */
		ih->ring = dma_alloc_coherent(adev->dev, ih->ring_size + 8,
					      &dma_addr, GFP_KERNEL);
		if (ih->ring == NULL)
			return -ENOMEM;

		ih->gpu_addr = dma_addr;
		ih->wptr_addr = dma_addr + ih->ring_size;
		ih->wptr_cpu = &ih->ring[ih->ring_size / 4];
		ih->rptr_addr = dma_addr + ih->ring_size + 4;
		ih->rptr_cpu = &ih->ring[(ih->ring_size / 4) + 1];
	} else {
		unsigned wptr_offs, rptr_offs;

		r = gsgpu_device_wb_get(adev, &wptr_offs);
		if (r)
			return r;

		r = gsgpu_device_wb_get(adev, &rptr_offs);
		if (r) {
			gsgpu_device_wb_free(adev, wptr_offs);
			return r;
		}

		r = gsgpu_bo_create_kernel(adev, ih->ring_size, PAGE_SIZE,
					    GSGPU_GEM_DOMAIN_GTT,
					    &ih->ring_obj, &ih->gpu_addr,
					    (void **)&ih->ring);
		if (r) {
			gsgpu_device_wb_free(adev, rptr_offs);
			gsgpu_device_wb_free(adev, wptr_offs);
			return r;
		}

		ih->wptr_addr = adev->wb.gpu_addr + wptr_offs * 4;
		ih->wptr_cpu = &adev->wb.wb[wptr_offs];
		ih->rptr_addr = adev->wb.gpu_addr + rptr_offs * 4;
		ih->rptr_cpu = &adev->wb.wb[rptr_offs];
	}

	init_waitqueue_head(&ih->wait_process);
	return 0;
}

/**
 * gsgpu_ih_ring_fini - tear down the IH state
 *
 * @adev: gsgpu_device pointer
 * @ih: ih ring to tear down
 *
 * Tears down the IH state and frees buffer
 * used for the IH ring buffer.
 */
void gsgpu_ih_ring_fini(struct gsgpu_device *adev, struct gsgpu_ih_ring *ih)
{

	if (!ih->ring)
		return;

	if (ih->use_bus_addr) {

		/* add 8 bytes for the rptr/wptr shadows and
		 * add them to the end of the ring allocation.
		 */
		dma_free_coherent(adev->dev, ih->ring_size + 8,
				  (void *)ih->ring, ih->gpu_addr);
		ih->ring = NULL;
	} else {
		gsgpu_bo_free_kernel(&ih->ring_obj, &ih->gpu_addr,
				      (void **)&ih->ring);
		gsgpu_device_wb_free(adev, (ih->wptr_addr - ih->gpu_addr) / 4);
		gsgpu_device_wb_free(adev, (ih->rptr_addr - ih->gpu_addr) / 4);
	}
}

/**
 * gsgpu_ih_ring_write - write IV to the ring buffer
 *
 * @ih: ih ring to write to
 * @iv: the iv to write
 * @num_dw: size of the iv in dw
 *
 * Writes an IV to the ring buffer using the CPU and increment the wptr.
 * Used for testing and delegating IVs to a software ring.
 */
void gsgpu_ih_ring_write(struct gsgpu_ih_ring *ih, const uint32_t *iv,
			  unsigned int num_dw)
{
	uint32_t wptr = le32_to_cpu(*ih->wptr_cpu) >> 2;
	unsigned int i;

	for (i = 0; i < num_dw; ++i)
	        ih->ring[wptr++] = cpu_to_le32(iv[i]);

	wptr <<= 2;
	wptr &= ih->ptr_mask;

	/* Only commit the new wptr if we don't overflow */
	if (wptr != READ_ONCE(ih->rptr)) {
		wmb();
		WRITE_ONCE(*ih->wptr_cpu, cpu_to_le32(wptr));
	}
}

/**
 * gsgpu_ih_wait_on_checkpoint_process_ts - wait to process IVs up to checkpoint
 *
 * @adev: gsgpu_device pointer
 * @ih: ih ring to process
 *
 * Used to ensure ring has processed IVs up to the checkpoint write pointer.
 */
int gsgpu_ih_wait_on_checkpoint_process_ts(struct gsgpu_device *adev,
					struct gsgpu_ih_ring *ih)
{
	uint32_t checkpoint_wptr;
	uint64_t checkpoint_ts;
	long timeout = HZ;

	if (!ih->enabled || adev->shutdown)
		return -ENODEV;

	checkpoint_wptr = gsgpu_ih_get_wptr(adev, ih);
	/* Order wptr with ring data. */
	rmb();
	checkpoint_ts = gsgpu_ih_decode_iv_ts(adev, ih, checkpoint_wptr, -1);

	return wait_event_interruptible_timeout(ih->wait_process,
		    gsgpu_ih_ts_after(checkpoint_ts, ih->processed_timestamp) ||
		    ih->rptr == gsgpu_ih_get_wptr(adev, ih), timeout);
}

/**
 * gsgpu_ih_process - interrupt handler
 *
 * @adev: gsgpu_device pointer
 * @ih: ih ring to process
 *
 * Interrupt hander , walk the IH ring.
 * Returns irq process return code.
 */
int gsgpu_ih_process(struct gsgpu_device *adev, struct gsgpu_ih_ring *ih)
{
	unsigned int count;
	u32 wptr;

	if (!ih->enabled || adev->shutdown)
		return IRQ_NONE;

	if (!adev->irq.msi_enabled)
		WREG32(GSGPU_HOST_INT, 0);

	wptr = gsgpu_ih_get_wptr(adev, ih);

restart_ih:
	count  = GSGPU_IH_MAX_NUM_IVS;
	DRM_DEBUG("%s: rptr %d, wptr %d\n", __func__, ih->rptr, wptr);

	/* Order reading of wptr vs. reading of IH ring data */
	rmb();

	while (ih->rptr != wptr && --count) {
		gsgpu_irq_dispatch(adev, ih);
		ih->rptr &= ih->ptr_mask;
	}

	gsgpu_ih_set_rptr(adev, ih);
	wake_up_all(&ih->wait_process);

	/* make sure wptr hasn't changed while processing */
	wptr = gsgpu_ih_get_wptr(adev, ih);
	if (wptr != ih->rptr)
		goto restart_ih;

	return IRQ_HANDLED;
}
