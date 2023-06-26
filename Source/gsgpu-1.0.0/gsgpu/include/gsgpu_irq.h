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

#ifndef __GSGPU_IRQ_H__
#define __GSGPU_IRQ_H__

#include <linux/irqdomain.h>
#include "gsgpu_dc_i2c.h"
#include "gsgpu_ih.h"

#define GSGPU_MAX_IRQ_SRC_ID		0x100
#define GSGPU_MAX_IRQ_CLIENT_ID	0x100

/* TODO irq srcid need rewrite*/
#define GSGPU_SRCID_GFX_PAGE_INV_FAULT                 0x00000092  /* 146 */
#define GSGPU_SRCID_GFX_MEM_PROT_FAULT                 0x00000093  /* 147 */
#define GSGPU_SRCID_CP_END_OF_PIPE                     0x000000b5  /* 181 */
#define GSGPU_SRCID_CP_PRIV_REG_FAULT                  0x000000b8  /* 184 */
#define GSGPU_SRCID_CP_PRIV_INSTR_FAULT                0x000000b9  /* 185 */
#define GSGPU_SRCID_XDMA_TRAP          	               0x000000e0  /* 224 */
#define GSGPU_SRCID_XDMA_SRBM_WRITE                    0x000000f7  /* 247 */

#define GSGPU_IRQ_CLIENTID_LEGACY	0
#define GSGPU_IRQ_CLIENTID_MAX		SOC15_IH_CLIENTID_MAX

#define GSGPU_IRQ_SRC_DATA_MAX_SIZE_DW	4

struct gsgpu_device;

enum gsgpu_interrupt_state {
	GSGPU_IRQ_STATE_DISABLE,
	GSGPU_IRQ_STATE_ENABLE,
};

struct gsgpu_iv_entry {
	struct gsgpu_ih_ring *ih;
	unsigned client_id;
	unsigned src_id;
	unsigned ring_id;
	unsigned vmid;
	unsigned vmid_src;
	uint64_t timestamp;
	unsigned timestamp_src;
	unsigned pasid;
	unsigned pasid_src;
	unsigned src_data[GSGPU_IRQ_SRC_DATA_MAX_SIZE_DW];
	const uint32_t *iv_entry;
};

struct gsgpu_irq_src {
	unsigned				num_types;
	atomic_t				*enabled_types;
	const struct gsgpu_irq_src_funcs	*funcs;
};

struct gsgpu_irq_client {
	struct gsgpu_irq_src **sources;
};

/* provided by interrupt generating IP blocks */
struct gsgpu_irq_src_funcs {
	int (*set)(struct gsgpu_device *adev, struct gsgpu_irq_src *source,
		   unsigned type, enum gsgpu_interrupt_state state);

	int (*process)(struct gsgpu_device *adev,
		       struct gsgpu_irq_src *source,
		       struct gsgpu_iv_entry *entry);
};

struct gsgpu_irq {
	bool				installed;
	unsigned int			irq;
	spinlock_t			lock;
	/* interrupt sources */
	struct gsgpu_irq_client	client[GSGPU_IRQ_CLIENTID_MAX];

	/* status, etc. */
	bool				msi_enabled; /* msi enabled */

	/* interrupt rings */
	struct gsgpu_ih_ring		ih, ih1, ih2, ih_soft;
	const struct gsgpu_ih_funcs    *ih_funcs;
	struct work_struct		ih1_work, ih2_work, ih_soft_work;
	struct gsgpu_irq_src		self_irq;
};

void gsgpu_irq_disable_all(struct gsgpu_device *adev);

int gsgpu_irq_init(struct gsgpu_device *adev);
void gsgpu_irq_fini_sw(struct gsgpu_device *adev);
void gsgpu_irq_fini_hw(struct gsgpu_device *adev);
int gsgpu_irq_add_id(struct gsgpu_device *adev,
		      unsigned client_id, unsigned src_id,
		      struct gsgpu_irq_src *source);
void gsgpu_irq_dispatch(struct gsgpu_device *adev,
			 struct gsgpu_ih_ring *ih);
void gsgpu_irq_delegate(struct gsgpu_device *adev,
			 struct gsgpu_iv_entry *entry,
			 unsigned int num_dw);
int gsgpu_irq_update(struct gsgpu_device *adev, struct gsgpu_irq_src *src,
		      unsigned type);
int gsgpu_irq_get(struct gsgpu_device *adev, struct gsgpu_irq_src *src,
		   unsigned type);
int gsgpu_irq_put(struct gsgpu_device *adev, struct gsgpu_irq_src *src,
		   unsigned type);
bool gsgpu_irq_enabled(struct gsgpu_device *adev, struct gsgpu_irq_src *src,
			unsigned type);
void gsgpu_irq_gpu_reset_resume_helper(struct gsgpu_device *adev);

#endif
