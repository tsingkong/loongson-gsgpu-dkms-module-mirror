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
#ifndef __GSGPU_H__
#define __GSGPU_H__

#ifdef pr_fmt
#undef pr_fmt
#endif

#define pr_fmt(fmt) "gsgpu: " fmt

#ifdef dev_fmt
#undef dev_fmt
#endif

#define dev_fmt(fmt) "gsgpu: " fmt

#include "gsgpu_ctx.h"

#include <linux/atomic.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/kref.h>
#include <linux/rbtree.h>
#include <linux/hashtable.h>
#include <linux/dma-fence.h>
#include <linux/pci.h>
#include <linux/aer.h>

#include <drm/ttm/ttm_bo.h>
#include <drm/ttm/ttm_placement.h>
#include <drm/ttm/ttm_execbuf_util.h>

#include <drm/gsgpu_drm.h>
#include <drm/drm_gem.h>
#include <drm/drm_ioctl.h>

#include "gsgpu_shared.h"
#include "gsgpu_mode.h"
#include "gsgpu_ih.h"
#include "gsgpu_irq.h"
#include "gsgpu_ttm.h"
#include "gsgpu_sync.h"
#include "gsgpu_ring.h"
#include "gsgpu_vm.h"
#include "gsgpu_gmc.h"
#include "gsgpu_gem.h"
#include "gsgpu_gfx.h"
#include "gsgpu_xdma.h"
#include "gsgpu_dc.h"
#include "gsgpu_gart.h"
#include "gsgpu_debugfs.h"
#include "gsgpu_job.h"
#include "gsgpu_bo_list.h"
#include "gsgpu_zip_meta.h"
#include "gsgpu_hw_sema.h"

#define MAX_GPU_INSTANCE		16

struct gsgpu_gpu_instance
{
	struct gsgpu_device		*adev;
	int				mgpu_fan_enabled;
};

struct gsgpu_mgpu_info
{
	struct gsgpu_gpu_instance	gpu_ins[MAX_GPU_INSTANCE];
	struct mutex			mutex;
	uint32_t			num_gpu;
	uint32_t			num_dgpu;
	uint32_t			num_apu;

	/* delayed reset_func for XGMI configuration if necessary */
	struct delayed_work		delayed_reset_work;
	bool				pending_reset;
};

enum gsgpu_ss {
	GSGPU_SS_DRV_LOAD,
	GSGPU_SS_DEV_D0,
	GSGPU_SS_DEV_D3,
	GSGPU_SS_DRV_UNLOAD
};

struct gsgpu_watchdog_timer
{
	bool timeout_fatal_disable;
	uint32_t period; /* maxCycles = (1 << period), the number of cycles before a timeout */
};

#define GSGPU_MAX_TIMEOUT_PARAM_LENGTH	256

/*
 * Modules parameters.
 */
extern int gsgpu_modeset;
extern unsigned int gsgpu_vram_limit;
extern int gsgpu_vis_vram_limit;
extern int gsgpu_gart_size;
extern int gsgpu_gtt_size;
extern int gsgpu_moverate;
extern int gsgpu_disp_priority;
extern int gsgpu_msi;
extern char gsgpu_lockup_timeout[GSGPU_MAX_TIMEOUT_PARAM_LENGTH];
extern int gsgpu_vm_size;
extern int gsgpu_vm_block_size;
extern int gsgpu_vm_fault_stop;
extern int gsgpu_vm_debug;
extern int gsgpu_vm_update_mode;
extern int gsgpu_exp_hw_support;
extern int gsgpu_sched_jobs;
extern int gsgpu_sched_hw_submission;
extern uint gsgpu_force_long_training;
extern int gsgpu_job_hang_limit;

extern uint gsgpu_dm_abm_level;
extern int gsgpu_backlight;
extern struct gsgpu_mgpu_info mgpu_info;

extern int gsgpu_bad_page_threshold;
extern bool gsgpu_ignore_bad_page_threshold;
extern struct gsgpu_watchdog_timer gsgpu_watchdog_timer;

extern int gsgpu_noretry;

static const bool __maybe_unused debug_evictions; /* = false */
static const bool __maybe_unused no_system_mem_limit;
static const int __maybe_unused halt_if_hws_hang;

extern int gsgpu_using_ram;

#define GSGPU_BYTES_PER_DW           4

#define GSGPU_KB_SHIFT_BITS          10
#define GSGPU_MB_SHIFT_BITS          20
#define GSGPU_GB_SHIFT_BITS          30

#define GSGPU_VM_MAX_NUM_CTX			4096
#define GSGPU_SG_THRESHOLD			(256*1024*1024)
#define GSGPU_DEFAULT_GTT_SIZE_MB		3072ULL /* 3GB by default */
#define GSGPU_WAIT_IDLE_TIMEOUT_IN_MS	        3000
#define GSGPU_MAX_USEC_TIMEOUT			100000	/* 100 ms */
#define GSGPU_FENCE_JIFFIES_TIMEOUT		(HZ / 2)
#define GSGPU_DEBUGFS_MAX_COMPONENTS		32
#define GSGPUFB_CONN_LIMIT			4
#define GSGPU_BIOS_NUM_SCRATCH			16

#define GSGPU_VBIOS_VGA_ALLOCATION		(9 * 1024 * 1024) /* reserve 8MB for vga emulator and 1 MB for FB */

/* hard reset data */
#define GSGPU_ASIC_RESET_DATA                  0x39d5e86b

/* smart shift bias level limits */
#define GSGPU_SMARTSHIFT_MAX_BIAS (100)
#define GSGPU_SMARTSHIFT_MIN_BIAS (-100)

struct gsgpu_device;
struct gsgpu_irq_src;
struct gsgpu_fpriv;
struct gsgpu_bo_va_mapping;
struct kfd_vm_fault_info;
struct gsgpu_reset_context;
struct gsgpu_reset_control;

enum gsgpu_cp_irq {
	GSGPU_CP_IRQ_GFX_ME0_PIPE0_EOP = 0,
	GSGPU_CP_IRQ_GFX_ME0_PIPE1_EOP,
	GSGPU_CP_IRQ_LAST
};

#define SRIOV_USEC_TIMEOUT  1200000 /* wait 12 * 100ms for SRIOV */
#define MAX_KIQ_REG_WAIT       5000 /* in usecs, 5ms */
#define MAX_KIQ_REG_BAILOUT_INTERVAL   5 /* in msecs, 5ms */
#define MAX_KIQ_REG_TRY 1000

int gsgpu_device_ip_wait_for_idle(struct gsgpu_device *adev,
				   enum gsgpu_ip_block_type block_type);
bool gsgpu_device_ip_is_idle(struct gsgpu_device *adev,
			      enum gsgpu_ip_block_type block_type);

#define GSGPU_MAX_IP_NUM 16

struct gsgpu_ip_block_status {
	bool valid;
	bool sw;
	bool hw;
	bool late_initialized;
	bool hang;
};

struct gsgpu_ip_block_version {
	const enum gsgpu_ip_block_type type;
	const u32 major;
	const u32 minor;
	const u32 rev;
	const struct gsgpu_ip_funcs *funcs;
};

#define HW_REV(_Major, _Minor, _Rev) \
	((((uint32_t) (_Major)) << 16) | ((uint32_t) (_Minor) << 8) | ((uint32_t) (_Rev)))

struct gsgpu_ip_block {
	struct gsgpu_ip_block_status status;
	const struct gsgpu_ip_block_version *version;
};

int gsgpu_device_ip_block_version_cmp(struct gsgpu_device *adev,
				       enum gsgpu_ip_block_type type,
				       u32 major, u32 minor);

struct gsgpu_ip_block *
gsgpu_device_ip_get_ip_block(struct gsgpu_device *adev,
			      enum gsgpu_ip_block_type type);

int gsgpu_device_ip_block_add(struct gsgpu_device *adev,
			       const struct gsgpu_ip_block_version *ip_block_version);

/*
 * BIOS.
 */
bool gsgpu_get_bios(struct gsgpu_device *adev);
bool gsgpu_read_bios(struct gsgpu_device *adev);
bool gsgpu_soc15_read_bios_from_rom(struct gsgpu_device *adev,
				     u8 *bios, u32 length_bytes);
/*
 * Clocks
 */

#define GSGPU_MAX_PPLL 3

struct gsgpu_clock {
	/* 10 Khz units */
	uint32_t default_mclk;
	uint32_t default_sclk;
	uint32_t default_dispclk;
	uint32_t current_dispclk;
	uint32_t dp_extclk;
	uint32_t max_pixel_clock;
};

/* sub-allocation manager, it has to be protected by another lock.
 * By conception this is an helper for other part of the driver
 * like the indirect buffer or semaphore, which both have their
 * locking.
 *
 * Principe is simple, we keep a list of sub allocation in offset
 * order (first entry has offset == 0, last entry has the highest
 * offset).
 *
 * When allocating new object we first check if there is room at
 * the end total_size - (last_object_offset + last_object_size) >=
 * alloc_size. If so we allocate new object there.
 *
 * When there is not enough room at the end, we start waiting for
 * each sub object until we reach object_offset+object_size >=
 * alloc_size, this object then become the sub object we return.
 *
 * Alignment can't be bigger than page size.
 *
 * Hole are not considered for allocation to keep things simple.
 * Assumption is that there won't be hole (all object on same
 * alignment).
 */

#define GSGPU_SA_NUM_FENCE_LISTS	32

struct gsgpu_sa_manager {
	wait_queue_head_t	wq;
	struct gsgpu_bo	*bo;
	struct list_head	*hole;
	struct list_head	flist[GSGPU_SA_NUM_FENCE_LISTS];
	struct list_head	olist;
	unsigned		size;
	uint64_t		gpu_addr;
	void			*cpu_ptr;
	uint32_t		domain;
	uint32_t		align;
};

/* sub-allocation buffer */
struct gsgpu_sa_bo {
	struct list_head		olist;
	struct list_head		flist;
	struct gsgpu_sa_manager	*manager;
	unsigned			soffset;
	unsigned			eoffset;
	struct dma_fence	        *fence;
};

int gsgpu_fence_slab_init(void);
void gsgpu_fence_slab_fini(void);

//GS Registers
#define GSGPU_COMMAND				0x0
#define GSGPU_STATUS				0x4
#define GSGPU_ARGUMENT0				0x8
#define GSGPU_ARGUMENT1				0xc
#define GSGPU_RETURN0				0x10
#define GSGPU_RETURN1				0x14
#define GSGPU_GFX_CB_BASE_LO_OFFSET		0x18
#define GSGPU_GFX_CB_BASE_HI_OFFSET		0x1c
#define GSGPU_GFX_CB_SIZE_OFFSET		0x20
#define GSGPU_GFX_CB_WPTR_OFFSET		0x24
#define GSGPU_GFX_CB_RPTR_OFFSET		0x28
#define GSGPU_XDMA_CB_BASE_LO_OFFSET		0x2c
#define GSGPU_XDMA_CB_BASE_HI_OFFSET		0x30
#define GSGPU_XDMA_CB_SIZE_OFFSET		0x34
#define GSGPU_XDMA_CB_WPTR_OFFSET		0x38
#define GSGPU_XDMA_CB_RPTR_OFFSET		0x3c
#define GSGPU_INT_CB_BASE_LO_OFFSET		0x40
#define GSGPU_INT_CB_BASE_HI_OFFSET		0x44
#define GSGPU_INT_CB_SIZE_OFFSET		0x48
#define GSGPU_INT_CB_WPTR_OFFSET		0x4c
#define GSGPU_INT_CB_RPTR_OFFSET		0x50
/* reserved 0x54 ~ 0x74 */
#define GSGPU_RESERVE_START_OFFSET		0x54
#define GSGPU_RESERVE_END_OFFSET		0x74
#define GSGPU_FW_VERSION_OFFSET			0x78
#define GSGPU_HW_FEATURE_OFFSET			0x7c

#define GSGPU_EC_CTRL				0x80
#define GSGPU_EC_INT				0x84
#define GSGPU_HOST_INT				0x88
#define GSGPU_HWINF				0x8c
#define GSGPU_FREQ_SCALE			0x9c

#define GSGPU_FW_WPORT				0xf0
#define GSGPU_FW_WPTR				0xf4
#define GSGPU_FW_CKSUM				0xf8

//GS Commands
#define GSCMD(cmd, subcmd)	(((cmd) & 0xFF)  | ((subcmd) & 0xFF) << 8)
#define GSCMDi(cmd, subcmd, i)	(((cmd) & 0xFF)  | ((subcmd) & 0xFF) << 8 | ((i) & 0xF) << 16)

#define GSCMD_HALT		0x00000000 // stop jobs in GPU, return
#define GSCMD_PING_5A		0x00000001 // return 5a5a5a5a in status
#define GSCMD_PING_A5		0x00000002 // return a5a5a5a5 in status
#define GSCMD_LOOP_DRAM		0x00000003 // loop through DRAM
#define GSCMD_LOOP_SSRV		0x00000004 // loop through SSRV
#define GSCMD_START		0x00000005 // start processing command buffer
#define GSCMD_STOP		0x00000006 // stop processing command buffer
#define GSCMD_SYNC		0x00000007 // wait pipeline empty
#define GSCMD_MMU		0x00000008 // mmu related op
#define GSCMD_SETREG		0x00000009 // internal reg op
#define GSCMD_PIPE		0x0000000A // op pipeline
#define GSCMD_ZIP		0x0000000B // op zip
#define GSCMD_PIPE_FLUSH	1 // op pipeline
#define GSCMD_FREQ		0x0000000C

#define GSCMD_STS_NULL		0x00000000
#define GSCMD_STS_BOOT		0xB007B007 // BOOT
#define GSCMD_STS_DONE		0xD02ED02E // DONE
#define GSCMD_STS_RUN		0xFFFF0000 // RUNING, lower 16bit can store total command count

#define EC_CTRL_RUN		0x01
#define EC_CTRL_STOP		0x00

//GS Packets
#define GSPKT(op, n)	(((op) & 0xFF) | ((n) & 0xFFFF) << 16)

#define	GSPKT_NOP			0x80
#define	GSPKT_WRITE			0x81
#define GSPKT_INDIRECT			0x82
#define GSPKT_FENCE			0x83
#define GSPKT_TRAP			0x84
#define GSPKT_POLL			0x85
#define 	POLL_CONDITION(x)		((x) << 8)
		/* 0 - true
		 * 1 - <
		 * 2 - <=
		 * 3 - ==
		 * 4 - !=
		 * 5 - >=
		 * 6 - >
		 */
#define 	POLL_REG_MEM(x)			((x) << 12)
		/* 0 - reg
		 * 1 - mem
		 */
#define		POLL_TIMES_INTERVAL(t, i)	((t) << 16 | (i))
#define GSPKT_WPOLL			0x86
#define	GSPKT_READ			0x87

//DRAW 0x89
#define GSPKT_VM_BIND			0x8A

#define GSPKT_XDMA_COPY			0xc0

/* 0 - register
 * 1 - memory
 */
#define	READ_SRC_SEL(x)			((x) << 9)
#define	WRITE_DST_SEL(x)		((x) << 8)
#define	WRITE_WAIT			(1 << 15)

/*
 * IRQS.
 */

struct gsgpu_flip_work {
	struct delayed_work		flip_work;
	struct work_struct		unpin_work;
	struct gsgpu_device		*adev;
	int				crtc_id;
	u32				target_vblank;
	uint64_t			base;
	struct drm_pending_vblank_event *event;
	struct gsgpu_bo		*old_abo;
	unsigned			shared_count;
	struct dma_fence		**shared;
	struct dma_fence_cb		cb;
	bool				async;
};


/*
 * file private structure
 */

struct gsgpu_fpriv {
	struct gsgpu_vm	vm;
	struct gsgpu_bo_va	*prt_va;
	struct gsgpu_bo_va	*csa_va;
	struct mutex		bo_list_lock;
	struct idr		bo_list_handles;
	struct gsgpu_ctx_mgr	ctx_mgr;
};

int gsgpu_file_to_fpriv(struct file *filp, struct gsgpu_fpriv **fpriv);

/*
 * Writeback
 */
#define GSGPU_MAX_WB 256	/* Reserve at most 256 WB slots for gsgpu-owned rings. */

struct gsgpu_wb {
	struct gsgpu_bo	*wb_obj;
	volatile uint32_t	*wb;
	uint64_t		gpu_addr;
	u32			num_wb;	/* Number of wb slots actually reserved for gsgpu. */
	unsigned long		used[DIV_ROUND_UP(GSGPU_MAX_WB, BITS_PER_LONG)];
};

int gsgpu_device_wb_get(struct gsgpu_device *adev, u32 *wb);
void gsgpu_device_wb_free(struct gsgpu_device *adev, u32 wb);

/*
 * Benchmarking
 */
int gsgpu_benchmark(struct gsgpu_device *adev, int test_number);

/*
 * ASIC specific register table accessible by UMD
 */
struct gsgpu_allowed_register_entry {
	uint32_t reg_offset;
	bool grbm_indexed;
};

enum gsgpu_reset_method {
	GSGPU_RESET_METHOD_NONE = -1,
	GSGPU_RESET_METHOD_LEGACY = 0,
	GSGPU_RESET_METHOD_MODE0,
	GSGPU_RESET_METHOD_MODE1,
	GSGPU_RESET_METHOD_MODE2,
	GSGPU_RESET_METHOD_BACO,
	GSGPU_RESET_METHOD_PCI,
};

struct gsgpu_video_codec_info {
	u32 codec_type;
	u32 max_width;
	u32 max_height;
	u32 max_pixels_per_frame;
	u32 max_level;
};

#define codec_info_build(type, width, height, level) \
			 .codec_type = type,\
			 .max_width = width,\
			 .max_height = height,\
			 .max_pixels_per_frame = height * width,\
			 .max_level = level,

struct gsgpu_video_codecs {
	const u32 codec_count;
	const struct gsgpu_video_codec_info *codec_array;
};

/*
 * ASIC specific functions.
 */
struct gsgpu_asic_funcs {
	bool (*read_bios_from_rom)(struct gsgpu_device *adev,
				   u8 *bios, u32 length_bytes);
	int (*read_register)(struct gsgpu_device *adev, u32 se_num,
			     u32 sh_num, u32 reg_offset, u32 *value);
	void (*set_vga_state)(struct gsgpu_device *adev, bool state);
	int (*reset)(struct gsgpu_device *adev);
	/* get the reference clock */
	u32 (*get_clk)(struct gsgpu_device *adev);
	/* static power management */
	int (*get_pcie_lanes)(struct gsgpu_device *adev);
	void (*set_pcie_lanes)(struct gsgpu_device *adev, int lanes);
	/* check if the asic needs a full reset of if soft reset will work */
	bool (*need_full_reset)(struct gsgpu_device *adev);
};

/*
 * IOCTL.
 */
int gsgpu_bo_list_ioctl(struct drm_device *dev, void *data,
				struct drm_file *filp);

int gsgpu_cs_ioctl(struct drm_device *dev, void *data, struct drm_file *filp);
int gsgpu_cs_fence_to_handle_ioctl(struct drm_device *dev, void *data,
				    struct drm_file *filp);
int gsgpu_cs_wait_ioctl(struct drm_device *dev, void *data, struct drm_file *filp);
int gsgpu_cs_wait_fences_ioctl(struct drm_device *dev, void *data,
				struct drm_file *filp);

int gsgpu_hw_sema_op_ioctl(struct drm_device *dev, void *data, struct drm_file *filp);

/* VRAM scratch page for HDP bug, default vram page */
struct gsgpu_mem_scratch {
	struct gsgpu_bo		*robj;
	volatile uint32_t		*ptr;
	u64				gpu_addr;
};

/*
 * Core structure, functions and helpers.
 */
typedef uint32_t (*gsgpu_rreg_t)(struct gsgpu_device*, uint32_t);
typedef void (*gsgpu_wreg_t)(struct gsgpu_device*, uint32_t, uint32_t);

typedef uint64_t (*gsgpu_rreg64_t)(struct gsgpu_device*, uint32_t);
typedef void (*gsgpu_wreg64_t)(struct gsgpu_device*, uint32_t, uint64_t);

typedef uint32_t (*gsgpu_block_rreg_t)(struct gsgpu_device*, uint32_t, uint32_t);
typedef void (*gsgpu_block_wreg_t)(struct gsgpu_device*, uint32_t, uint32_t, uint32_t);

struct gsgpu_mmio_remap {
	u32 reg_offset;
	resource_size_t bus_addr;
};

/* Define the HW IP blocks will be used in driver , add more if necessary */
enum gsgpu_hw_ip_block_type {
	GC_HWIP = 1,
	HDP_HWIP,
	SDMA0_HWIP,
	SDMA1_HWIP,
	SDMA2_HWIP,
	SDMA3_HWIP,
	SDMA4_HWIP,
	SDMA5_HWIP,
	SDMA6_HWIP,
	SDMA7_HWIP,
	LSDMA_HWIP,
	MMHUB_HWIP,
	ATHUB_HWIP,
	NBIO_HWIP,
	MP0_HWIP,
	MP1_HWIP,
	UVD_HWIP,
	VCN_HWIP = UVD_HWIP,
	JPEG_HWIP = VCN_HWIP,
	VCN1_HWIP,
	VCE_HWIP,
	DF_HWIP,
	DCE_HWIP,
	OSSSYS_HWIP,
	SMUIO_HWIP,
	PWR_HWIP,
	NBIF_HWIP,
	THM_HWIP,
	CLK_HWIP,
	UMC_HWIP,
	RSMU_HWIP,
	XGMI_HWIP,
	DCI_HWIP,
	PCIE_HWIP,
	MAX_HWIP
};

#define HWIP_MAX_INSTANCE	28

#define HW_ID_MAX		300
#define IP_VERSION(mj, mn, rv) (((mj) << 16) | ((mn) << 8) | (rv))
#define IP_VERSION_MAJ(ver) ((ver) >> 16)
#define IP_VERSION_MIN(ver) (((ver) >> 8) & 0xFF)
#define IP_VERSION_REV(ver) ((ver) & 0xFF)

struct ip_discovery_top;

/* polaris10 kickers */
#define ASICID_IS_P20(did, rid)		(((did == 0x67DF) && \
					 ((rid == 0xE3) || \
					  (rid == 0xE4) || \
					  (rid == 0xE5) || \
					  (rid == 0xE7) || \
					  (rid == 0xEF))) || \
					 ((did == 0x6FDF) && \
					 ((rid == 0xE7) || \
					  (rid == 0xEF) || \
					  (rid == 0xFF))))

#define ASICID_IS_P30(did, rid)		((did == 0x67DF) && \
					((rid == 0xE1) || \
					 (rid == 0xF7)))

/* polaris11 kickers */
#define ASICID_IS_P21(did, rid)		(((did == 0x67EF) && \
					 ((rid == 0xE0) || \
					  (rid == 0xE5))) || \
					 ((did == 0x67FF) && \
					 ((rid == 0xCF) || \
					  (rid == 0xEF) || \
					  (rid == 0xFF))))

#define ASICID_IS_P31(did, rid)		((did == 0x67EF) && \
					((rid == 0xE2)))

/* polaris12 kickers */
#define ASICID_IS_P23(did, rid)		(((did == 0x6987) && \
					 ((rid == 0xC0) || \
					  (rid == 0xC1) || \
					  (rid == 0xC3) || \
					  (rid == 0xC7))) || \
					 ((did == 0x6981) && \
					 ((rid == 0x00) || \
					  (rid == 0x01) || \
					  (rid == 0x10))))

#define GSGPU_RESET_MAGIC_NUM 64
#define GSGPU_MAX_DF_PERFMONS 4
#define GSGPU_PRODUCT_NAME_LEN 64
struct gsgpu_reset_domain;

enum pp_mp1_state {
	PP_MP1_STATE_NONE,
	PP_MP1_STATE_SHUTDOWN,
	PP_MP1_STATE_UNLOAD,
	PP_MP1_STATE_RESET,
};

/*
 * Non-zero (true) if the GPU has VRAM. Zero (false) otherwise.
 */
#define GSGPU_HAS_VRAM(_adev) ((_adev)->gmc.real_vram_size)

struct gsgpu_device {
	struct device			*dev;
	struct pci_dev			*pdev;
	struct drm_device		ddev;

	struct pci_dev			*loongson_dc;
	u8 chip_revision;

	/* ASIC */
	enum gsgpu_family_type		family_type;
	uint32_t			family;
	unsigned long			flags;
	unsigned long			apu_flags;
	int				usec_timeout;
	const struct gsgpu_asic_funcs	*asic_funcs;
	bool				shutdown;
	bool				need_swiotlb;
	bool				accel_working;
	struct notifier_block		acpi_nb;
	struct debugfs_blob_wrapper     debugfs_vbios_blob;

	struct mutex			srbm_mutex;
	/* GRBM index mutex. Protects concurrent access to GRBM index */
	struct mutex                    grbm_idx_mutex;
	struct dev_pm_domain		vga_pm_domain;
	bool				have_disp_power_ref;
	bool                            have_atomics_support;

	/* BIOS */
	bool				is_atom_fw;
	uint8_t				*bios;
	uint32_t			bios_size;
	uint32_t			bios_scratch_reg_offset;
	uint32_t			bios_scratch[GSGPU_BIOS_NUM_SCRATCH];

	/* Register/doorbell mmio */
	resource_size_t			rmmio_base;
	resource_size_t			rmmio_size;
	void __iomem			*rmmio;

	/* loongson dc mmio */
	resource_size_t			loongson_dc_rmmio_base;
	resource_size_t			loongson_dc_rmmio_size;
	void __iomem			*loongson_dc_rmmio;

	/* protects concurrent MM_INDEX/DATA based register access */
	spinlock_t mmio_idx_lock;
	spinlock_t dc_mmio_lock;

	struct gsgpu_mmio_remap        rmmio_remap;

	/* protects concurrent PCIE register access */
	spinlock_t pcie_idx_lock;

	/* protects concurrent se_cac register access */
	spinlock_t se_cac_idx_lock;
	gsgpu_rreg_t			se_cac_rreg;
	gsgpu_wreg_t			se_cac_wreg;

	/* clock/pll info */
	struct gsgpu_clock            clock;

	/* MC */
	struct gsgpu_gmc		gmc;
	struct gsgpu_gart		gart;
	struct gsgpu_zip_meta	zip_meta;

	dma_addr_t			dummy_page_addr;
	struct gsgpu_vm_manager	vm_manager;
	unsigned			num_vmhubs;

	/* memory management */
	struct gsgpu_mman		mman;
	struct gsgpu_mem_scratch	mem_scratch;
	struct gsgpu_wb		wb;
	atomic64_t			num_bytes_moved;
	atomic64_t			num_evictions;
	atomic64_t			num_vram_cpu_page_faults;
	atomic_t			gpu_reset_counter;
	atomic_t			vram_lost_counter;

	struct gsgpu_hw_sema_mgr hw_sema_mgr;

	/* data for buffer migration throttling */
	struct {
		spinlock_t		lock;
		s64			last_update_us;
		s64			accum_us; /* accumulated microseconds */
		s64			accum_us_vis; /* for visible VRAM */
		u32			log2_max_MBps;
	} mm_stats;

	/* display */
	bool				enable_virtual_display;
	struct gsgpu_dc			*dc;

	struct gsgpu_mode_info		mode_info;
	struct gsgpu_dc_i2c		*i2c[2];
	struct delayed_work         hotplug_work;
	struct gsgpu_irq_src		vsync_irq;
	struct gsgpu_irq_src		i2c_irq;
	struct gsgpu_irq_src		hpd_irq;
	struct gsgpu_irq_src		dmub_trace_irq;
	struct gsgpu_irq_src		dmub_outbox_irq;

	/* rings */
	u64				fence_context;
	unsigned			num_rings;
	struct gsgpu_ring		*rings[GSGPU_MAX_RINGS];
	struct dma_fence __rcu		*gang_submit;
	bool				ib_pool_ready;
	struct gsgpu_sa_manager	ib_pools[GSGPU_IB_POOL_MAX];
	struct gsgpu_sched		gpu_sched[GSGPU_HW_IP_NUM][GSGPU_RING_PRIO_MAX];

	/* interrupts */
	struct gsgpu_irq		irq;

	/* HPD */
	int				vga_hpd_status;
	u64				cg_flags;
	u32				pg_flags;

	/* gfx */
	struct gsgpu_gfx		gfx;

	/* xdma */
	struct gsgpu_xdma		xdma;

	struct gsgpu_ip_block          ip_blocks[GSGPU_MAX_IP_NUM];
	uint32_t		        harvest_ip_mask;
	int				num_ip_blocks;
	struct mutex	mn_lock;
	DECLARE_HASHTABLE(mn_hash, 7);

	/* tracking pinned memory */
	atomic64_t vram_pin_size;
	atomic64_t visible_pin_size;
	atomic64_t gart_pin_size;

	/* delayed work_func for deferring clockgating during resume */
	struct delayed_work     delayed_init_work;

	//zl prior virt
	uint32_t			reg_val_offs;

	/* link all shadow bo */
	struct list_head                shadow_list;
	struct mutex                    shadow_list_lock;

	/* record hw reset is performed */
	bool has_hw_reset;
	u8				reset_magic[GSGPU_RESET_MAGIC_NUM];

	/* s3/s4 mask */
	bool                            in_suspend;
	bool				in_s3;
	bool				in_s4;
	bool				in_s0ix;

	enum pp_mp1_state               mp1_state;

	struct mutex			notifier_lock;

	int asic_reset_res;

	struct list_head		reset_list;

	long				gfx_timeout;
	long				xdma_timeout;
	long				video_timeout;

	uint64_t			unique_id;
	uint64_t	df_perfmon_config_assign_mask[GSGPU_MAX_DF_PERFMONS];

	/* enable runtime pm on the device */
	bool                            in_runpm;
	bool                            has_pr3;

	bool                            pm_sysfs_en;

	/* Chip product information */
	char				product_number[20];
	char				product_name[GSGPU_PRODUCT_NAME_LEN];
	char				serial[20];

	atomic_t			throttling_logging_enabled;
	struct ratelimit_state		throttling_logging_rs;
	uint32_t                        ras_hw_enabled;

	bool                            no_hw_access;
	struct pci_saved_state          *pci_state;
	pci_channel_state_t		pci_channel_state;

	struct gsgpu_reset_control     *reset_cntl;
	uint32_t                        ip_versions[MAX_HWIP][HWIP_MAX_INSTANCE];

	bool				ram_is_direct_mapped;

	struct list_head                ras_list;

	struct ip_discovery_top         *ip_top;

	struct gsgpu_reset_domain	*reset_domain;

	struct mutex			benchmark_mutex;

	/* reset dump register */
	uint32_t                        *reset_dump_reg_list;
	uint32_t			*reset_dump_reg_value;
	int                             num_regs;
#ifdef CONFIG_DEV_COREDUMP
	struct gsgpu_task_info         reset_task_info;
	bool                            reset_vram_lost;
	struct timespec64               reset_time;
#endif

	bool                            scpm_enabled;
	uint32_t                        scpm_status;

	struct work_struct		reset_work;

	bool                            job_hang;
	bool                            dc_enabled;

	struct loongson_vbios *vbios;
	bool cursor_showed;
	bool clone_mode;
	int cursor_crtc_id;
	bool inited;
};

static inline struct gsgpu_device *drm_to_adev(struct drm_device *ddev)
{
	return container_of(ddev, struct gsgpu_device, ddev);
}

static inline struct drm_device *adev_to_drm(struct gsgpu_device *adev)
{
	return &adev->ddev;
}

static inline struct gsgpu_device *gsgpu_ttm_adev(struct ttm_device *bdev)
{
	return container_of(bdev, struct gsgpu_device, mman.bdev);
}

int gsgpu_device_init(struct gsgpu_device *adev,
		       uint32_t flags);
void gsgpu_device_fini_hw(struct gsgpu_device *adev);
void gsgpu_device_fini_sw(struct gsgpu_device *adev);

int gsgpu_gpu_wait_for_idle(struct gsgpu_device *adev);

uint64_t gsgpu_cmd_exec(struct gsgpu_device *adev, uint32_t cmd, 
			uint32_t arg0, uint32_t arg1);

void gsgpu_device_mm_access(struct gsgpu_device *adev, loff_t pos,
			     void *buf, size_t size, bool write);
size_t gsgpu_device_aper_access(struct gsgpu_device *adev, loff_t pos,
				 void *buf, size_t size, bool write);

void gsgpu_device_vram_access(struct gsgpu_device *adev, loff_t pos,
			       void *buf, size_t size, bool write);
uint32_t gsgpu_device_rreg(struct gsgpu_device *adev,
			    uint32_t reg, uint32_t acc_flags);
void gsgpu_device_wreg(struct gsgpu_device *adev,
			uint32_t reg, uint32_t v,
			uint32_t acc_flags);
void gsgpu_mm_wreg8(struct gsgpu_device *adev, uint32_t offset, uint8_t value);
uint8_t gsgpu_mm_rreg8(struct gsgpu_device *adev, uint32_t offset);

u32 gsgpu_device_indirect_rreg(struct gsgpu_device *adev,
				u32 pcie_index, u32 pcie_data,
				u32 reg_addr);
u64 gsgpu_device_indirect_rreg64(struct gsgpu_device *adev,
				  u32 pcie_index, u32 pcie_data,
				  u32 reg_addr);
void gsgpu_device_indirect_wreg(struct gsgpu_device *adev,
				 u32 pcie_index, u32 pcie_data,
				 u32 reg_addr, u32 reg_data);
void gsgpu_device_indirect_wreg64(struct gsgpu_device *adev,
				   u32 pcie_index, u32 pcie_data,
				   u32 reg_addr, u64 reg_data);

int gsgpu_device_pre_asic_reset(struct gsgpu_device *adev,
				 struct gsgpu_reset_context *reset_context);

int gsgpu_do_asic_reset(struct list_head *device_list_handle,
			 struct gsgpu_reset_context *reset_context);

/*
 * Registers read & write functions.
 */
#define GSGPU_REGS_IDX       (1<<0)
#define GSGPU_REGS_NO_KIQ    (1<<1)
#define GSGPU_REGS_RLC	(1<<2)

#define RREG32_IDX(reg) gsgpu_device_rreg(adev, (reg), GSGPU_REGS_IDX)
#define RREG32_NO_KIQ(reg) gsgpu_device_rreg(adev, (reg), GSGPU_REGS_NO_KIQ)
#define WREG32_NO_KIQ(reg, v) gsgpu_device_wreg(adev, (reg), (v), GSGPU_REGS_NO_KIQ)

#define RREG8(reg) gsgpu_mm_rreg8(adev, (reg))
#define WREG8(reg, v) gsgpu_mm_wreg8(adev, (reg), (v))

#define RREG32(reg) gsgpu_device_rreg(adev, (reg), 0)
#define DREG32(reg) printk(KERN_INFO "REGISTER: " #reg " : 0x%08X\n", gsgpu_device_rreg(adev, (reg), 0))
#define WREG32(reg, v) gsgpu_device_wreg(adev, (reg), (v), 0)
#define REG_SET(FIELD, v) (((v) << FIELD##_SHIFT) & FIELD##_MASK)
#define REG_GET(FIELD, v) (((v) << FIELD##_SHIFT) & FIELD##_MASK)
#define RREG32_SE_CAC(reg) adev->se_cac_rreg(adev, (reg))
#define WREG32_SE_CAC(reg, v) adev->se_cac_wreg(adev, (reg), (v))
#define WREG32_P(reg, val, mask)				\
	do {							\
		uint32_t tmp_ = RREG32(reg);			\
		tmp_ &= (mask);					\
		tmp_ |= ((val) & ~(mask));			\
		WREG32(reg, tmp_);				\
	} while (0)
#define WREG32_AND(reg, and) WREG32_P(reg, 0, and)
#define WREG32_OR(reg, or) WREG32_P(reg, or, ~(or))
#define WREG32_PLL_P(reg, val, mask)				\
	do {							\
		uint32_t tmp_ = RREG32_PLL(reg);		\
		tmp_ &= (mask);					\
		tmp_ |= ((val) & ~(mask));			\
		WREG32_PLL(reg, tmp_);				\
	} while (0)

#define WREG32_SMC_P(_Reg, _Val, _Mask)                         \
	do {                                                    \
		u32 tmp = RREG32_SMC(_Reg);                     \
		tmp &= (_Mask);                                 \
		tmp |= ((_Val) & ~(_Mask));                     \
		WREG32_SMC(_Reg, tmp);                          \
	} while (0)

#define DREG32_SYS(sqf, adev, reg) seq_printf((sqf), #reg " : 0x%08X\n", gsgpu_device_rreg((adev), (reg), false))

#define REG_FIELD_SHIFT(reg, field) reg##__##field##__SHIFT
#define REG_FIELD_MASK(reg, field) reg##__##field##_MASK

#define REG_SET_FIELD(orig_val, reg, field, field_val)			\
	(((orig_val) & ~REG_FIELD_MASK(reg, field)) |			\
	 (REG_FIELD_MASK(reg, field) & ((field_val) << REG_FIELD_SHIFT(reg, field))))

#define REG_GET_FIELD(value, reg, field)				\
	(((value) & REG_FIELD_MASK(reg, field)) >> REG_FIELD_SHIFT(reg, field))

#define WREG32_FIELD(reg, field, val)	\
	WREG32(mm##reg, (RREG32(mm##reg) & ~REG_FIELD_MASK(reg, field)) | (val) << REG_FIELD_SHIFT(reg, field))

#define WREG32_FIELD_OFFSET(reg, offset, field, val)	\
	WREG32(mm##reg + offset, (RREG32(mm##reg + offset) & ~REG_FIELD_MASK(reg, field)) | (val) << REG_FIELD_SHIFT(reg, field))

/*
 * BIOS helpers.
 */
#define RBIOS8(i) (adev->bios[i])
#define RBIOS16(i) (RBIOS8(i) | (RBIOS8((i)+1) << 8))
#define RBIOS32(i) ((RBIOS16(i)) | (RBIOS16((i)+2) << 16))

/*
 * ASICs macro.
 */
#define gsgpu_asic_set_vga_state(adev, state) (adev)->asic_funcs->set_vga_state((adev), (state))
#define gsgpu_asic_reset(adev) (adev)->asic_funcs->reset((adev))
#define gsgpu_asic_get_clk(adev) (adev)->asic_funcs->get_clk((adev))
#define gsgpu_get_pcie_lanes(adev) (adev)->asic_funcs->get_pcie_lanes((adev))
#define gsgpu_set_pcie_lanes(adev, l) (adev)->asic_funcs->set_pcie_lanes((adev), (l))
#define gsgpu_asic_read_bios_from_rom(adev, b, l) (adev)->asic_funcs->read_bios_from_rom((adev), (b), (l))
#define gsgpu_asic_read_register(adev, se, sh, offset, v)((adev)->asic_funcs->read_register((adev), (se), (sh), (offset), (v)))
#define gsgpu_asic_need_full_reset(adev) (adev)->asic_funcs->need_full_reset((adev))
#define gsgpu_ring_test_xdma(r, t) (r)->funcs->test_xdma((r), (t))

#define gsgpu_inc_vram_lost(adev) atomic_inc(&((adev)->vram_lost_counter));

#define MIN(X, Y) ((X) < (Y) ? (X) : (Y))

/* Common functions */
bool gsgpu_device_has_job_running(struct gsgpu_device *adev);

int gsgpu_device_gpu_recover(struct gsgpu_device *adev,
			      struct gsgpu_job *job,
			      struct gsgpu_reset_context *reset_context);
void gsgpu_device_pci_config_reset(struct gsgpu_device *adev);
int gsgpu_device_pci_reset(struct gsgpu_device *adev);
bool gsgpu_device_need_post(struct gsgpu_device *adev);

void gsgpu_cs_report_moved_bytes(struct gsgpu_device *adev, u64 num_bytes,
				  u64 num_vis_bytes);
int gsgpu_device_resize_fb_bar(struct gsgpu_device *adev);
void gsgpu_device_program_register_sequence(struct gsgpu_device *adev,
					     const u32 *registers,
					     const u32 array_size);

bool gsgpu_device_is_peer_accessible(struct gsgpu_device *adev,
				      struct gsgpu_device *peer_adev);

void gsgpu_device_flush_hdp(struct gsgpu_device *adev,
		struct gsgpu_ring *ring);
void gsgpu_device_invalidate_hdp(struct gsgpu_device *adev,
		struct gsgpu_ring *ring);

void gsgpu_device_halt(struct gsgpu_device *adev);
struct dma_fence *gsgpu_device_switch_gang(struct gsgpu_device *adev,
					    struct dma_fence *gang);

/*
 * KMS
 */
extern const struct drm_ioctl_desc gsgpu_ioctls_kms[];
extern const int gsgpu_max_kms_ioctl;

int gsgpu_driver_load_kms(struct gsgpu_device *adev, unsigned long flags);
void gsgpu_driver_unload_kms(struct drm_device *dev);
void gsgpu_driver_lastclose_kms(struct drm_device *dev);
int gsgpu_driver_open_kms(struct drm_device *dev, struct drm_file *file_priv);
void gsgpu_driver_postclose_kms(struct drm_device *dev,
				 struct drm_file *file_priv);
void gsgpu_driver_release_kms(struct drm_device *dev);

int gsgpu_device_ip_suspend(struct gsgpu_device *adev);
int gsgpu_device_suspend(struct drm_device *dev, bool fbcon);
int gsgpu_device_resume(struct drm_device *dev, bool fbcon);
u32 gsgpu_get_vblank_counter_kms(struct drm_crtc *crtc);
int gsgpu_info_ioctl(struct drm_device *dev, void *data,
		      struct drm_file *filp);

/*
 * functions used by gsgpu_encoder.c
 */
struct gsgpu_afmt_acr {
	u32 clock;

	int n_32khz;
	int cts_32khz;

	int n_44_1khz;
	int cts_44_1khz;

	int n_48khz;
	int cts_48khz;

};

struct gsgpu_afmt_acr gsgpu_afmt_acr(uint32_t clock);

/* gsgpu_acpi.c */

/* ATCS Device/Driver State */
#define GSGPU_ATCS_PSC_DEV_STATE_D0		0
#define GSGPU_ATCS_PSC_DEV_STATE_D3_HOT	3
#define GSGPU_ATCS_PSC_DRV_STATE_OPR		0
#define GSGPU_ATCS_PSC_DRV_STATE_NOT_OPR	1
/*
#if defined(CONFIG_ACPI)
int gsgpu_acpi_init(struct gsgpu_device *adev);
void gsgpu_acpi_fini(struct gsgpu_device *adev);
bool gsgpu_acpi_is_pcie_performance_request_supported(struct gsgpu_device *adev);
bool gsgpu_acpi_is_power_shift_control_supported(void);
int gsgpu_acpi_pcie_performance_request(struct gsgpu_device *adev,
						u8 perf_req, bool advertise);
int gsgpu_acpi_power_shift_control(struct gsgpu_device *adev,
				    u8 dev_state, bool drv_state);
int gsgpu_acpi_smart_shift_update(struct drm_device *dev, enum gsgpu_ss ss_state);
int gsgpu_acpi_pcie_notify_device_ready(struct gsgpu_device *adev);

void gsgpu_acpi_get_backlight_caps(struct gsgpu_dm_backlight_caps *caps);
bool gsgpu_acpi_should_gpu_reset(struct gsgpu_device *adev);
void gsgpu_acpi_detect(void);
#else
*/
static inline int gsgpu_acpi_init(struct gsgpu_device *adev) { return 0; }
static inline void gsgpu_acpi_fini(struct gsgpu_device *adev) { }
static inline bool gsgpu_acpi_should_gpu_reset(struct gsgpu_device *adev) { return false; }
static inline void gsgpu_acpi_detect(void) { }
static inline bool gsgpu_acpi_is_power_shift_control_supported(void) { return false; }
static inline int gsgpu_acpi_power_shift_control(struct gsgpu_device *adev,
						  u8 dev_state, bool drv_state) { return 0; }
static inline int gsgpu_acpi_smart_shift_update(struct drm_device *dev,
						 enum gsgpu_ss ss_state) { return 0; }
// #endif

/*
#if defined(CONFIG_ACPI) && defined(CONFIG_SUSPEND)
bool gsgpu_acpi_is_s3_active(struct gsgpu_device *adev);
bool gsgpu_acpi_is_s0ix_active(struct gsgpu_device *adev);
#else
*/

static inline bool gsgpu_acpi_is_s0ix_active(struct gsgpu_device *adev) { return false; }
static inline bool gsgpu_acpi_is_s3_active(struct gsgpu_device *adev) { return false; }
// #endif

void gsgpu_register_gpu_instance(struct gsgpu_device *adev);
void gsgpu_unregister_gpu_instance(struct gsgpu_device *adev);

pci_ers_result_t gsgpu_pci_error_detected(struct pci_dev *pdev,
					   pci_channel_state_t state);
pci_ers_result_t gsgpu_pci_mmio_enabled(struct pci_dev *pdev);
pci_ers_result_t gsgpu_pci_slot_reset(struct pci_dev *pdev);
void gsgpu_pci_resume(struct pci_dev *pdev);

bool gsgpu_device_cache_pci_state(struct pci_dev *pdev);
bool gsgpu_device_load_pci_state(struct pci_dev *pdev);

bool gsgpu_device_skip_hw_access(struct gsgpu_device *adev);

#include "gsgpu_object.h"

int gsgpu_in_reset(struct gsgpu_device *adev);

#endif
