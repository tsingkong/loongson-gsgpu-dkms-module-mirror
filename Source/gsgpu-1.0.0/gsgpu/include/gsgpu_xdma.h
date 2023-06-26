/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020-2022 Loongson Technology Corporation Limited
 */

#ifndef __GSGPU_XDMA_H__
#define __GSGPU_XDMA_H__

/* max number of IP instances */
#define GSGPU_MAX_XDMA_INSTANCES		2
#define GSGPU_XDMA_FLAG_UMAP 0x20000

enum gsgpu_xdma_irq {
	GSGPU_XDMA_IRQ_TRAP0 = 0,
	GSGPU_XDMA_IRQ_TRAP1,
	GSGPU_XDMA_IRQ_LAST
};

/*
 * SDMA
 */
struct gsgpu_xdma_instance {
	/* SDMA firmware */
	const struct firmware	*fw;
	uint32_t		fw_version;
	uint32_t		feature_version;

	struct gsgpu_ring	ring;
	bool			burst_nop;
};

struct gsgpu_xdma {
	struct gsgpu_xdma_instance instance[GSGPU_MAX_XDMA_INSTANCES];
	struct gsgpu_irq_src	trap_irq;
	struct gsgpu_irq_src	illegal_inst_irq;
	int			num_instances;
};

/*
 * Provided by hw blocks that can move/clear data.  e.g., gfx or sdma
 * But currently, we use sdma to move data.
 */
struct gsgpu_buffer_funcs {
	/* maximum bytes in a single operation */
	uint32_t	copy_max_bytes;

	/* number of dw to reserve per operation */
	unsigned	copy_num_dw;

	/* used for buffer migration */
	void (*emit_copy_buffer)(struct gsgpu_ib *ib,
				 /* src addr in bytes */
				 uint64_t src_offset,
				 /* dst addr in bytes */
				 uint64_t dst_offset,
				 /* number of byte to transfer */
				 uint32_t byte_count,
				 bool tmz);

	/* maximum bytes in a single operation */
	uint32_t	fill_max_bytes;

	/* number of dw to reserve per operation */
	unsigned	fill_num_dw;

	/* used for buffer clearing */
	void (*emit_fill_buffer)(struct gsgpu_ib *ib,
				 /* value to write to memory */
				 uint32_t src_data,
				 /* dst addr in bytes */
				 uint64_t dst_offset,
				 /* number of byte to fill */
				 uint32_t byte_count);
};

#define gsgpu_emit_copy_buffer(adev, ib, s, d, b, t) (adev)->mman.buffer_funcs->emit_copy_buffer((ib),  (s), (d), (b), (t))
#define gsgpu_emit_fill_buffer(adev, ib, s, d, b) (adev)->mman.buffer_funcs->emit_fill_buffer((ib), (s), (d), (b))

struct gsgpu_xdma_instance *
gsgpu_get_xdma_instance(struct gsgpu_ring *ring);
void xdma_ring_test_xdma_loop(struct gsgpu_ring *ring, long timeout);
extern const struct gsgpu_ip_block_version xdma_ip_block;

#endif /*__GSGPU_XDMA_H__*/
