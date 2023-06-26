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

#ifndef __GSGPU_IH_H__
#define __GSGPU_IH_H__

/* Maximum number of IVs processed at once */
#define GSGPU_IH_MAX_NUM_IVS	32

struct gsgpu_device;
struct gsgpu_iv_entry;

extern const struct gsgpu_ip_block_version gsgpu_ih_ip_block;

struct gsgpu_ih_regs {
	uint32_t ih_rb_base;
	uint32_t ih_rb_base_hi;
	uint32_t ih_rb_cntl;
	uint32_t ih_rb_wptr;
	uint32_t ih_rb_rptr;
	uint32_t ih_doorbell_rptr;
	uint32_t ih_rb_wptr_addr_lo;
	uint32_t ih_rb_wptr_addr_hi;
	uint32_t psp_reg_id;
};

/*
 * Vega10+ IH clients
 * Whenever this structure is updated, which should not happen, make sure
 * soc15_ih_clientid_name in the below is also updated accordingly.
 */
enum soc15_ih_clientid {
	SOC15_IH_CLIENTID_IH		= 0x00,
	SOC15_IH_CLIENTID_ACP		= 0x01,
	SOC15_IH_CLIENTID_ATHUB		= 0x02,
	SOC15_IH_CLIENTID_BIF		= 0x03,
	SOC15_IH_CLIENTID_DCE		= 0x04,
	SOC15_IH_CLIENTID_ISP		= 0x05,
	SOC15_IH_CLIENTID_PCIE0		= 0x06,
	SOC15_IH_CLIENTID_RLC		= 0x07,
	SOC15_IH_CLIENTID_SDMA0		= 0x08,
	SOC15_IH_CLIENTID_SDMA1		= 0x09,
	SOC15_IH_CLIENTID_SE0SH		= 0x0a,
	SOC15_IH_CLIENTID_SE1SH		= 0x0b,
	SOC15_IH_CLIENTID_SE2SH		= 0x0c,
	SOC15_IH_CLIENTID_SE3SH		= 0x0d,
	SOC15_IH_CLIENTID_UVD1		= 0x0e,
	SOC15_IH_CLIENTID_THM		= 0x0f,
	SOC15_IH_CLIENTID_UVD		= 0x10,
	SOC15_IH_CLIENTID_VCE0		= 0x11,
	SOC15_IH_CLIENTID_VMC		= 0x12,
	SOC15_IH_CLIENTID_XDMA		= 0x13,
	SOC15_IH_CLIENTID_GRBM_CP	= 0x14,
	SOC15_IH_CLIENTID_ATS		= 0x15,
	SOC15_IH_CLIENTID_ROM_SMUIO	= 0x16,
	SOC15_IH_CLIENTID_DF		= 0x17,
	SOC15_IH_CLIENTID_VCE1		= 0x18,
	SOC15_IH_CLIENTID_PWR		= 0x19,
	SOC15_IH_CLIENTID_RESERVED	= 0x1a,
	SOC15_IH_CLIENTID_UTCL2		= 0x1b,
	SOC15_IH_CLIENTID_EA		= 0x1c,
	SOC15_IH_CLIENTID_UTCL2LOG	= 0x1d,
	SOC15_IH_CLIENTID_MP0		= 0x1e,
	SOC15_IH_CLIENTID_MP1		= 0x1f,

	SOC15_IH_CLIENTID_MAX,

	SOC15_IH_CLIENTID_VCN		= SOC15_IH_CLIENTID_UVD,
	SOC15_IH_CLIENTID_VCN1		= SOC15_IH_CLIENTID_UVD1,
	SOC15_IH_CLIENTID_SDMA2		= SOC15_IH_CLIENTID_ACP,
	SOC15_IH_CLIENTID_SDMA3		= SOC15_IH_CLIENTID_DCE,
	SOC15_IH_CLIENTID_SDMA3_Sienna_Cichlid    = SOC15_IH_CLIENTID_ISP,
	SOC15_IH_CLIENTID_SDMA4		= SOC15_IH_CLIENTID_ISP,
	SOC15_IH_CLIENTID_SDMA5		= SOC15_IH_CLIENTID_VCE0,
	SOC15_IH_CLIENTID_SDMA6		= SOC15_IH_CLIENTID_XDMA,
	SOC15_IH_CLIENTID_SDMA7		= SOC15_IH_CLIENTID_VCE1,
	SOC15_IH_CLIENTID_VMC1		= SOC15_IH_CLIENTID_PCIE0,
};


/*
 * R6xx+ IH ring
 */
struct gsgpu_ih_ring {
	unsigned		ring_size;
	uint32_t		ptr_mask;

	bool			use_bus_addr;

	struct gsgpu_bo	*ring_obj;
	volatile uint32_t	*ring;
	uint64_t		gpu_addr;

	uint64_t		wptr_addr;
	volatile uint32_t	*wptr_cpu;

	uint64_t		rptr_addr;
	volatile uint32_t	*rptr_cpu;

	bool                    enabled;
	unsigned		rptr;
	struct gsgpu_ih_regs	ih_regs;

	/* For waiting on IH processing at checkpoint. */
	wait_queue_head_t wait_process;
	uint64_t		processed_timestamp;
};

/* return true if time stamp t2 is after t1 with 48bit wrap around */
#define gsgpu_ih_ts_after(t1, t2) \
		(((int64_t)((t2) << 16) - (int64_t)((t1) << 16)) > 0LL)

/* provided by the ih block */
struct gsgpu_ih_funcs {
	/* ring read/write ptr handling, called from interrupt context */
	u32 (*get_wptr)(struct gsgpu_device *adev, struct gsgpu_ih_ring *ih);
	void (*decode_iv)(struct gsgpu_device *adev, struct gsgpu_ih_ring *ih,
			  struct gsgpu_iv_entry *entry);
	uint64_t (*decode_iv_ts)(struct gsgpu_ih_ring *ih, u32 rptr,
				 signed int offset);
	void (*set_rptr)(struct gsgpu_device *adev, struct gsgpu_ih_ring *ih);
};

#define gsgpu_ih_get_wptr(adev, ih) (adev)->irq.ih_funcs->get_wptr((adev), (ih))
#define gsgpu_ih_decode_iv(adev, ih, iv) \
	(adev)->irq.ih_funcs->decode_iv((adev), (ih), (iv))
#define gsgpu_ih_decode_iv_ts(adev, ih, rptr, offset) \
	(WARN_ON_ONCE(!(adev)->irq.ih_funcs->decode_iv_ts) ? 0 : \
	(adev)->irq.ih_funcs->decode_iv_ts((ih), (rptr), (offset)))
#define gsgpu_ih_set_rptr(adev, ih) (adev)->irq.ih_funcs->set_rptr((adev), (ih))

int gsgpu_ih_ring_init(struct gsgpu_device *adev, struct gsgpu_ih_ring *ih,
			unsigned ring_size, bool use_bus_addr);
void gsgpu_ih_ring_fini(struct gsgpu_device *adev, struct gsgpu_ih_ring *ih);
void gsgpu_ih_ring_write(struct gsgpu_ih_ring *ih, const uint32_t *iv,
			  unsigned int num_dw);
int gsgpu_ih_wait_on_checkpoint_process_ts(struct gsgpu_device *adev,
					    struct gsgpu_ih_ring *ih);
int gsgpu_ih_process(struct gsgpu_device *adev, struct gsgpu_ih_ring *ih);

#endif
