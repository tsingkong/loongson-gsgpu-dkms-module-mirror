/* gsgpu_drm.h -- Public header for the gsgpu driver -*- linux-c -*-
 *
 * Copyright 2000 Precision Insight, Inc., Cedar Park, Texas.
 * Copyright 2000 VA Linux Systems, Inc., Fremont, California.
 * Copyright 2002 Tungsten Graphics, Inc., Cedar Park, Texas.
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
 * Authors:
 *    Kevin E. Martin <martin@valinux.com>
 *    Gareth Hughes <gareth@valinux.com>
 *    Keith Whitwell <keith@tungstengraphics.com>
 */

#ifndef __GSGPU_DRM_H__
#define __GSGPU_DRM_H__

#include <drm/drm.h>

#if defined(__cplusplus)
extern "C" {
#endif

#define DRM_GSGPU_GEM_CREATE		0x00
#define DRM_GSGPU_GEM_MMAP		0x01
#define DRM_GSGPU_CTX			0x02
#define DRM_GSGPU_BO_LIST		0x03
#define DRM_GSGPU_CS			0x04
#define DRM_GSGPU_INFO			0x05
#define DRM_GSGPU_GEM_METADATA		0x06
#define DRM_GSGPU_GEM_WAIT_IDLE	0x07
#define DRM_GSGPU_GEM_VA		0x08
#define DRM_GSGPU_WAIT_CS		0x09
#define DRM_GSGPU_GEM_OP		0x10
#define DRM_GSGPU_GEM_USERPTR		0x11
#define DRM_GSGPU_WAIT_FENCES		0x12
#define DRM_GSGPU_VM			0x13
#define DRM_GSGPU_FENCE_TO_HANDLE	0x14
#define DRM_GSGPU_SCHED		0x15
#define DRM_GSGPU_HWSEMA_OP		0x16

#define DRM_IOCTL_GSGPU_GEM_CREATE	DRM_IOWR(DRM_COMMAND_BASE + DRM_GSGPU_GEM_CREATE, union drm_gsgpu_gem_create)
#define DRM_IOCTL_GSGPU_GEM_MMAP	DRM_IOWR(DRM_COMMAND_BASE + DRM_GSGPU_GEM_MMAP, union drm_gsgpu_gem_mmap)
#define DRM_IOCTL_GSGPU_CTX		DRM_IOWR(DRM_COMMAND_BASE + DRM_GSGPU_CTX, union drm_gsgpu_ctx)
#define DRM_IOCTL_GSGPU_BO_LIST	DRM_IOWR(DRM_COMMAND_BASE + DRM_GSGPU_BO_LIST, union drm_gsgpu_bo_list)
#define DRM_IOCTL_GSGPU_CS		DRM_IOWR(DRM_COMMAND_BASE + DRM_GSGPU_CS, union drm_gsgpu_cs)
#define DRM_IOCTL_GSGPU_INFO		DRM_IOW(DRM_COMMAND_BASE + DRM_GSGPU_INFO, struct drm_gsgpu_info)
#define DRM_IOCTL_GSGPU_GEM_METADATA	DRM_IOWR(DRM_COMMAND_BASE + DRM_GSGPU_GEM_METADATA, struct drm_gsgpu_gem_metadata)
#define DRM_IOCTL_GSGPU_GEM_WAIT_IDLE	DRM_IOWR(DRM_COMMAND_BASE + DRM_GSGPU_GEM_WAIT_IDLE, union drm_gsgpu_gem_wait_idle)
#define DRM_IOCTL_GSGPU_GEM_VA		DRM_IOW(DRM_COMMAND_BASE + DRM_GSGPU_GEM_VA, struct drm_gsgpu_gem_va)
#define DRM_IOCTL_GSGPU_WAIT_CS	DRM_IOWR(DRM_COMMAND_BASE + DRM_GSGPU_WAIT_CS, union drm_gsgpu_wait_cs)
#define DRM_IOCTL_GSGPU_GEM_OP		DRM_IOWR(DRM_COMMAND_BASE + DRM_GSGPU_GEM_OP, struct drm_gsgpu_gem_op)
#define DRM_IOCTL_GSGPU_GEM_USERPTR	DRM_IOWR(DRM_COMMAND_BASE + DRM_GSGPU_GEM_USERPTR, struct drm_gsgpu_gem_userptr)
#define DRM_IOCTL_GSGPU_WAIT_FENCES	DRM_IOWR(DRM_COMMAND_BASE + DRM_GSGPU_WAIT_FENCES, union drm_gsgpu_wait_fences)
#define DRM_IOCTL_GSGPU_VM		DRM_IOWR(DRM_COMMAND_BASE + DRM_GSGPU_VM, union drm_gsgpu_vm)
#define DRM_IOCTL_GSGPU_FENCE_TO_HANDLE DRM_IOWR(DRM_COMMAND_BASE + DRM_GSGPU_FENCE_TO_HANDLE, union drm_gsgpu_fence_to_handle)
#define DRM_IOCTL_GSGPU_SCHED		DRM_IOW(DRM_COMMAND_BASE + DRM_GSGPU_SCHED, union drm_gsgpu_sched)
#define DRM_IOCTL_GSGPU_HWSEMA_OP	DRM_IOWR(DRM_COMMAND_BASE + DRM_GSGPU_HWSEMA_OP, struct drm_gsgpu_hw_sema)

/**
 * DOC: memory domains
 *
 * %GSGPU_GEM_DOMAIN_CPU	System memory that is not GPU accessible.
 * Memory in this pool could be swapped out to disk if there is pressure.
 *
 * %GSGPU_GEM_DOMAIN_GTT	GPU accessible system memory, mapped into the
 * GPU's virtual address space via gart. Gart memory linearizes non-contiguous
 * pages of system memory, allows GPU access system memory in a linearized
 * fashion.
 *
 * %GSGPU_GEM_DOMAIN_VRAM	Local video memory. For APUs, it is memory
 * carved out by the BIOS.
 *
 * %GSGPU_GEM_DOMAIN_GDS	Global on-chip data storage used to share data
 * across shader threads.
 *
 * %GSGPU_GEM_DOMAIN_GWS	Global wave sync, used to synchronize the
 * execution of all the waves on a device.
 *
 * %GSGPU_GEM_DOMAIN_OA	Ordered append, used by 3D or Compute engines
 * for appending data.
 */
#define GSGPU_GEM_DOMAIN_CPU		0x1
#define GSGPU_GEM_DOMAIN_GTT		0x2
#define GSGPU_GEM_DOMAIN_VRAM		0x4
#define GSGPU_GEM_DOMAIN_GDS		0x8
#define GSGPU_GEM_DOMAIN_GWS		0x10
#define GSGPU_GEM_DOMAIN_OA		0x20
#define GSGPU_GEM_DOMAIN_MASK		(GSGPU_GEM_DOMAIN_CPU | \
					 GSGPU_GEM_DOMAIN_GTT | \
					 GSGPU_GEM_DOMAIN_VRAM | \
					 GSGPU_GEM_DOMAIN_GDS | \
					 GSGPU_GEM_DOMAIN_GWS | \
					 GSGPU_GEM_DOMAIN_OA)

/* Flag that CPU access will be required for the case of VRAM domain */
#define GSGPU_GEM_CREATE_CPU_ACCESS_REQUIRED	(1 << 0)
/* Flag that CPU access will not work, this VRAM domain is invisible */
#define GSGPU_GEM_CREATE_NO_CPU_ACCESS		(1 << 1)
/* Flag that USWC attributes should be used for GTT */
#define GSGPU_GEM_CREATE_CPU_GTT_USWC		(1 << 2)
/* Flag that the memory should be in VRAM and cleared */
#define GSGPU_GEM_CREATE_VRAM_CLEARED		(1 << 3)
/* Flag that allocating the BO should use linear VRAM */
#define GSGPU_GEM_CREATE_VRAM_CONTIGUOUS	(1 << 5)
/* Flag that BO is always valid in this VM */
#define GSGPU_GEM_CREATE_VM_ALWAYS_VALID	(1 << 6)
/* Flag that BO sharing will be explicitly synchronized */
#define GSGPU_GEM_CREATE_EXPLICIT_SYNC		(1 << 7)
/* Flag that indicates allocating MQD gart on GFX9, where the mtype
 * for the second page onward should be set to NC. It should never
 * be used by user space applications.
 */
#define GSGPU_GEM_CREATE_CP_MQD_GFX9		(1 << 8)

/* Flag that BO may contain sensitive data that must be wiped before
 * releasing the memory
 */
/* Flag that BO is compressed bit 9-11 */
#define GSGPU_GEM_CREATE_COMPRESSED_MASK	(0x7ull << 9)

#define GSGPU_GEM_CREATE_VRAM_WIPE_ON_RELEASE	(1 << 9)
/* Flag that BO will be encrypted and that the TMZ bit should be
 * set in the PTEs when mapping this buffer via GPUVM or
 * accessing it with various hw blocks
 */
#define GSGPU_GEM_CREATE_ENCRYPTED		(1 << 10)
/* Flag that BO will be used only in preemptible context, which does
 * not require GTT memory accounting
 */
#define GSGPU_GEM_CREATE_PREEMPTIBLE		(1 << 11)
/* Flag that BO can be discarded under memory pressure without keeping the
 * content.
 */
#define GSGPU_GEM_CREATE_DISCARDABLE		(1 << 12)
/* Flag that BO is shared coherently between multiple devices or CPU threads.
 * May depend on GPU instructions to flush caches explicitly
 *
 * This influences the choice of MTYPE in the PTEs on GFXv9 and later GPUs and
 * may override the MTYPE selected in GSGPU_VA_OP_MAP.
 */
#define GSGPU_GEM_CREATE_COHERENT		(1 << 13)
/* Flag that BO should not be cached by GPU. Coherent without having to flush
 * GPU caches explicitly
 *
 * This influences the choice of MTYPE in the PTEs on GFXv9 and later GPUs and
 * may override the MTYPE selected in GSGPU_VA_OP_MAP.
 */
#define GSGPU_GEM_CREATE_UNCACHED		(1 << 14)

struct drm_gsgpu_gem_create_in  {
	/** the requested memory size */
	__u64 bo_size;
	/** physical start_addr alignment in bytes for some HW requirements */
	__u64 alignment;
	/** the requested memory domains */
	__u64 domains;
	/** allocation flags */
	__u64 domain_flags;
};

struct drm_gsgpu_gem_create_out  {
	/** returned GEM object handle */
	__u32 handle;
	__u32 _pad;
};

union drm_gsgpu_gem_create {
	struct drm_gsgpu_gem_create_in		in;
	struct drm_gsgpu_gem_create_out	out;
};

/** Opcode to create new residency list.  */
#define GSGPU_BO_LIST_OP_CREATE	0
/** Opcode to destroy previously created residency list */
#define GSGPU_BO_LIST_OP_DESTROY	1
/** Opcode to update resource information in the list */
#define GSGPU_BO_LIST_OP_UPDATE	2

struct drm_gsgpu_bo_list_in {
	/** Type of operation */
	__u32 operation;
	/** Handle of list or 0 if we want to create one */
	__u32 list_handle;
	/** Number of BOs in list  */
	__u32 bo_number;
	/** Size of each element describing BO */
	__u32 bo_info_size;
	/** Pointer to array describing BOs */
	__u64 bo_info_ptr;
};

struct drm_gsgpu_bo_list_entry {
	/** Handle of BO */
	__u32 bo_handle;
	/** New (if specified) BO priority to be used during migration */
	__u32 bo_priority;
};

struct drm_gsgpu_bo_list_out {
	/** Handle of resource list  */
	__u32 list_handle;
	__u32 _pad;
};

union drm_gsgpu_bo_list {
	struct drm_gsgpu_bo_list_in in;
	struct drm_gsgpu_bo_list_out out;
};

/* context related */
#define GSGPU_CTX_OP_ALLOC_CTX	1
#define GSGPU_CTX_OP_FREE_CTX	2
#define GSGPU_CTX_OP_QUERY_STATE	3
#define GSGPU_CTX_OP_QUERY_STATE2	4
#define GSGPU_CTX_OP_GET_STABLE_PSTATE	5
#define GSGPU_CTX_OP_SET_STABLE_PSTATE	6

/* GPU reset status */
#define GSGPU_CTX_NO_RESET		0
/* this the context caused it */
#define GSGPU_CTX_GUILTY_RESET		1
/* some other context caused it */
#define GSGPU_CTX_INNOCENT_RESET	2
/* unknown cause */
#define GSGPU_CTX_UNKNOWN_RESET	3

/* indicate gpu reset occured after ctx created */
#define GSGPU_CTX_QUERY2_FLAGS_RESET    (1<<0)
/* indicate vram lost occured after ctx created */
#define GSGPU_CTX_QUERY2_FLAGS_VRAMLOST (1<<1)
/* indicate some job from this context once cause gpu hang */
#define GSGPU_CTX_QUERY2_FLAGS_GUILTY   (1<<2)
/* indicate some errors are detected by RAS */
#define GSGPU_CTX_QUERY2_FLAGS_RAS_CE   (1<<3)
#define GSGPU_CTX_QUERY2_FLAGS_RAS_UE   (1<<4)

/* Context priority level */
#define GSGPU_CTX_PRIORITY_UNSET       -2048
#define GSGPU_CTX_PRIORITY_VERY_LOW    -1023
#define GSGPU_CTX_PRIORITY_LOW         -512
#define GSGPU_CTX_PRIORITY_NORMAL      0
/*
 * When used in struct drm_gsgpu_ctx_in, a priority above NORMAL requires
 * CAP_SYS_NICE or DRM_MASTER
*/
#define GSGPU_CTX_PRIORITY_HIGH        512
#define GSGPU_CTX_PRIORITY_VERY_HIGH   1023

/* select a stable profiling pstate for perfmon tools */
#define GSGPU_CTX_STABLE_PSTATE_FLAGS_MASK  0xf
#define GSGPU_CTX_STABLE_PSTATE_NONE  0
#define GSGPU_CTX_STABLE_PSTATE_STANDARD  1
#define GSGPU_CTX_STABLE_PSTATE_MIN_SCLK  2
#define GSGPU_CTX_STABLE_PSTATE_MIN_MCLK  3
#define GSGPU_CTX_STABLE_PSTATE_PEAK  4

struct drm_gsgpu_ctx_in {
	/** GSGPU_CTX_OP_* */
	__u32	op;
	/** Flags */
	__u32	flags;
	__u32	ctx_id;
	/** GSGPU_CTX_PRIORITY_* */
	__s32	priority;
};

union drm_gsgpu_ctx_out {
		struct {
			__u32	ctx_id;
			__u32	_pad;
		} alloc;

		struct {
			/** For future use, no flags defined so far */
			__u64	flags;
			/** Number of resets caused by this context so far. */
			__u32	hangs;
			/** Reset status since the last call of the ioctl. */
			__u32	reset_status;
		} state;

		struct {
			__u32	flags;
			__u32	_pad;
		} pstate;
};

union drm_gsgpu_ctx {
	struct drm_gsgpu_ctx_in in;
	union drm_gsgpu_ctx_out out;
};

/* vm ioctl */
#define GSGPU_VM_OP_RESERVE_VMID	1
#define GSGPU_VM_OP_UNRESERVE_VMID	2

struct drm_gsgpu_vm_in {
	/** GSGPU_VM_OP_* */
	__u32	op;
	__u32	flags;
};

struct drm_gsgpu_vm_out {
	/** For future use, no flags defined so far */
	__u64	flags;
};

union drm_gsgpu_vm {
	struct drm_gsgpu_vm_in in;
	struct drm_gsgpu_vm_out out;
};

/* sched ioctl */
#define GSGPU_SCHED_OP_PROCESS_PRIORITY_OVERRIDE	1
#define GSGPU_SCHED_OP_CONTEXT_PRIORITY_OVERRIDE	2

struct drm_gsgpu_sched_in {
	/* GSGPU_SCHED_OP_* */
	__u32	op;
	__u32	fd;
	/** GSGPU_CTX_PRIORITY_* */
	__s32	priority;
	__u32   ctx_id;
};

union drm_gsgpu_sched {
	struct drm_gsgpu_sched_in in;
};

/*
 * This is not a reliable API and you should expect it to fail for any
 * number of reasons and have fallback path that do not use userptr to
 * perform any operation.
 */
#define GSGPU_GEM_USERPTR_READONLY	(1 << 0)
#define GSGPU_GEM_USERPTR_ANONONLY	(1 << 1)
#define GSGPU_GEM_USERPTR_VALIDATE	(1 << 2)
#define GSGPU_GEM_USERPTR_REGISTER	(1 << 3)

struct drm_gsgpu_gem_userptr {
	__u64		addr;
	__u64		size;
	/* GSGPU_GEM_USERPTR_* */
	__u32		flags;
	/* Resulting GEM handle */
	__u32		handle;
};

/* SI-CI-VI: */
/* same meaning as the GB_TILE_MODE and GL_MACRO_TILE_MODE fields */
#define GSGPU_TILING_ARRAY_MODE_SHIFT			0
#define GSGPU_TILING_ARRAY_MODE_MASK			0xf
#define GSGPU_TILING_PIPE_CONFIG_SHIFT			4
#define GSGPU_TILING_PIPE_CONFIG_MASK			0x1f
#define GSGPU_TILING_TILE_SPLIT_SHIFT			9
#define GSGPU_TILING_TILE_SPLIT_MASK			0x7
#define GSGPU_TILING_MICRO_TILE_MODE_SHIFT		12
#define GSGPU_TILING_MICRO_TILE_MODE_MASK		0x7
#define GSGPU_TILING_BANK_WIDTH_SHIFT			15
#define GSGPU_TILING_BANK_WIDTH_MASK			0x3
#define GSGPU_TILING_BANK_HEIGHT_SHIFT			17
#define GSGPU_TILING_BANK_HEIGHT_MASK			0x3
#define GSGPU_TILING_MACRO_TILE_ASPECT_SHIFT		19
#define GSGPU_TILING_MACRO_TILE_ASPECT_MASK		0x3
#define GSGPU_TILING_NUM_BANKS_SHIFT			21
#define GSGPU_TILING_NUM_BANKS_MASK			0x3

/* GFX9 and later: */
#define GSGPU_TILING_SWIZZLE_MODE_SHIFT		0
#define GSGPU_TILING_SWIZZLE_MODE_MASK			0x1f
#define GSGPU_TILING_DCC_OFFSET_256B_SHIFT		5
#define GSGPU_TILING_DCC_OFFSET_256B_MASK		0xFFFFFF
#define GSGPU_TILING_DCC_PITCH_MAX_SHIFT		29
#define GSGPU_TILING_DCC_PITCH_MAX_MASK		0x3FFF
#define GSGPU_TILING_DCC_INDEPENDENT_64B_SHIFT		43
#define GSGPU_TILING_DCC_INDEPENDENT_64B_MASK		0x1
#define GSGPU_TILING_DCC_INDEPENDENT_128B_SHIFT	44
#define GSGPU_TILING_DCC_INDEPENDENT_128B_MASK		0x1
#define GSGPU_TILING_SCANOUT_SHIFT			63
#define GSGPU_TILING_SCANOUT_MASK			0x1

/* Set/Get helpers for tiling flags. */
#define GSGPU_TILING_SET(field, value) \
	(((__u64)(value) & GSGPU_TILING_##field##_MASK) << GSGPU_TILING_##field##_SHIFT)
#define GSGPU_TILING_GET(value, field) \
	(((__u64)(value) >> GSGPU_TILING_##field##_SHIFT) & GSGPU_TILING_##field##_MASK)

#define GSGPU_GEM_METADATA_OP_SET_METADATA                  1
#define GSGPU_GEM_METADATA_OP_GET_METADATA                  2

/** The same structure is shared for input/output */
struct drm_gsgpu_gem_metadata {
	/** GEM Object handle */
	__u32	handle;
	/** Do we want get or set metadata */
	__u32	op;
	struct {
		/** For future use, no flags defined so far */
		__u64	flags;
		/** family specific tiling info */
		__u64	tiling_info;
		__u32	data_size_bytes;
		__u32	data[64];
	} data;
};

struct drm_gsgpu_gem_mmap_in {
	/** the GEM object handle */
	__u32 handle;
	__u32 _pad;
};

struct drm_gsgpu_gem_mmap_out {
	/** mmap offset from the vma offset manager */
	__u64 addr_ptr;
};

union drm_gsgpu_gem_mmap {
	struct drm_gsgpu_gem_mmap_in   in;
	struct drm_gsgpu_gem_mmap_out out;
};

struct drm_gsgpu_gem_wait_idle_in {
	/** GEM object handle */
	__u32 handle;
	/** For future use, no flags defined so far */
	__u32 flags;
	/** Absolute timeout to wait */
	__u64 timeout;
};

struct drm_gsgpu_gem_wait_idle_out {
	/** BO status:  0 - BO is idle, 1 - BO is busy */
	__u32 status;
	/** Returned current memory domain */
	__u32 domain;
};

union drm_gsgpu_gem_wait_idle {
	struct drm_gsgpu_gem_wait_idle_in  in;
	struct drm_gsgpu_gem_wait_idle_out out;
};

struct drm_gsgpu_wait_cs_in {
	/* Command submission handle
         * handle equals 0 means none to wait for
         * handle equals ~0ull means wait for the latest sequence number
         */
	__u64 handle;
	/** Absolute timeout to wait */
	__u64 timeout;
	__u32 ip_type;
	__u32 ip_instance;
	__u32 ring;
	__u32 ctx_id;
};

struct drm_gsgpu_wait_cs_out {
	/** CS status:  0 - CS completed, 1 - CS still busy */
	__u64 status;
};

union drm_gsgpu_wait_cs {
	struct drm_gsgpu_wait_cs_in in;
	struct drm_gsgpu_wait_cs_out out;
};

struct drm_gsgpu_fence {
	__u32 ctx_id;
	__u32 ip_type;
	__u32 ip_instance;
	__u32 ring;
	__u64 seq_no;
};

struct drm_gsgpu_wait_fences_in {
	/** This points to uint64_t * which points to fences */
	__u64 fences;
	__u32 fence_count;
	__u32 wait_all;
	__u64 timeout_ns;
};

struct drm_gsgpu_wait_fences_out {
	__u32 status;
	__u32 first_signaled;
};

union drm_gsgpu_wait_fences {
	struct drm_gsgpu_wait_fences_in in;
	struct drm_gsgpu_wait_fences_out out;
};

#define GSGPU_GEM_OP_GET_GEM_CREATE_INFO	0
#define GSGPU_GEM_OP_SET_PLACEMENT		1

/* Sets or returns a value associated with a buffer. */
struct drm_gsgpu_gem_op {
	/** GEM object handle */
	__u32	handle;
	/** GSGPU_GEM_OP_* */
	__u32	op;
	/** Input or return value */
	__u64	value;
};

#define GSGPU_VA_OP_MAP			1
#define GSGPU_VA_OP_UNMAP			2
#define GSGPU_VA_OP_CLEAR			3
#define GSGPU_VA_OP_REPLACE			4

/* Delay the page table update till the next CS */
#define GSGPU_VM_DELAY_UPDATE		(1 << 0)

/* Mapping flags */
/* readable mapping */
#define GSGPU_VM_PAGE_READABLE		(1 << 1)
/* writable mapping */
#define GSGPU_VM_PAGE_WRITEABLE	(1 << 2)
/* executable mapping, new for VI */
#define GSGPU_VM_PAGE_EXECUTABLE	(1 << 3)
/* partially resident texture */
#define GSGPU_VM_PAGE_PRT		(1 << 4)
/* MTYPE flags use bit 5 to 8 */
#define GSGPU_VM_MTYPE_MASK		(0xf << 5)
/* Default MTYPE. Pre-AI must use this.  Recommended for newer ASICs. */
#define GSGPU_VM_MTYPE_DEFAULT		(0 << 5)
/* Use Non Coherent MTYPE instead of default MTYPE */
#define GSGPU_VM_MTYPE_NC		(1 << 5)
/* Use Write Combine MTYPE instead of default MTYPE */
#define GSGPU_VM_MTYPE_WC		(2 << 5)
/* Use Cache Coherent MTYPE instead of default MTYPE */
#define GSGPU_VM_MTYPE_CC		(3 << 5)
/* Use UnCached MTYPE instead of default MTYPE */
#define GSGPU_VM_MTYPE_UC		(4 << 5)
/* Use Read Write MTYPE instead of default MTYPE */
#define GSGPU_VM_MTYPE_RW		(5 << 5)
/* don't allocate MALL */
#define GSGPU_VM_PAGE_NOALLOC		(1 << 9)

struct drm_gsgpu_gem_va {
	/** GEM object handle */
	__u32 handle;
	__u32 _pad;
	/** GSGPU_VA_OP_* */
	__u32 operation;
	/** GSGPU_VM_PAGE_* */
	__u32 flags;
	/** va address to assign . Must be correctly aligned.*/
	__u64 va_address;
	/** Specify offset inside of BO to assign. Must be correctly aligned.*/
	__u64 offset_in_bo;
	/** Specify mapping size. Must be correctly aligned. */
	__u64 map_size;
};

#define GSGPU_HW_IP_GFX          0
#define GSGPU_HW_IP_DMA          2
#define GSGPU_HW_IP_NUM          9

#define GSGPU_HW_IP_INSTANCE_MAX_COUNT 1

#define GSGPU_CHUNK_ID_IB		0x01
#define GSGPU_CHUNK_ID_FENCE		0x02
#define GSGPU_CHUNK_ID_DEPENDENCIES	0x03
#define GSGPU_CHUNK_ID_SYNCOBJ_IN      0x04
#define GSGPU_CHUNK_ID_SYNCOBJ_OUT     0x05
#define GSGPU_CHUNK_ID_BO_HANDLES      0x06
#define GSGPU_CHUNK_ID_SCHEDULED_DEPENDENCIES	0x07
#define GSGPU_CHUNK_ID_SYNCOBJ_TIMELINE_WAIT    0x08
#define GSGPU_CHUNK_ID_SYNCOBJ_TIMELINE_SIGNAL  0x09

struct drm_gsgpu_cs_chunk {
	__u32		chunk_id;
	__u32		length_dw;
	__u64		chunk_data;
};

struct drm_gsgpu_cs_in {
	/** Rendering context id */
	__u32		ctx_id;
	/**  Handle of resource list associated with CS */
	__u32		bo_list_handle;
	__u32		num_chunks;
	__u32		flags;
	/** this points to __u64 * which point to cs chunks */
	__u64		chunks;
};

struct drm_gsgpu_cs_out {
	__u64 handle;
};

union drm_gsgpu_cs {
	struct drm_gsgpu_cs_in in;
	struct drm_gsgpu_cs_out out;
};

/* Specify flags to be used for IB */

/* This IB should be submitted to CE */
#define GSGPU_IB_FLAG_CE	(1<<0)

/* Preamble flag, which means the IB could be dropped if no context switch */
#define GSGPU_IB_FLAG_PREAMBLE (1<<1)

/* Preempt flag, IB should set Pre_enb bit if PREEMPT flag detected */
#define GSGPU_IB_FLAG_PREEMPT (1<<2)

/* The IB fence should do the L2 writeback but not invalidate any shader
 * caches (L2/vL1/sL1/I$). */
#define GSGPU_IB_FLAG_TC_WB_NOT_INVALIDATE (1 << 3)

/* Set GDS_COMPUTE_MAX_WAVE_ID = DEFAULT before PACKET3_INDIRECT_BUFFER.
 * This will reset wave ID counters for the IB.
 */
#define GSGPU_IB_FLAG_RESET_GDS_MAX_WAVE_ID (1 << 4)

/* Flag the IB as secure (TMZ)
 */
#define GSGPU_IB_FLAGS_SECURE  (1 << 5)

/* Tell KMD to flush and invalidate caches
 */
#define GSGPU_IB_FLAG_EMIT_MEM_SYNC  (1 << 6)

struct drm_gsgpu_cs_chunk_ib {
	__u32 _pad;
	/** GSGPU_IB_FLAG_* */
	__u32 flags;
	/** Virtual address to begin IB execution */
	__u64 va_start;
	/** Size of submission */
	__u32 ib_bytes;
	/** HW IP to submit to */
	__u32 ip_type;
	/** HW IP index of the same type to submit to  */
	__u32 ip_instance;
	/** Ring index to submit to */
	__u32 ring;
};

struct drm_gsgpu_cs_chunk_dep {
	__u32 ip_type;
	__u32 ip_instance;
	__u32 ring;
	__u32 ctx_id;
	__u64 handle;
};

struct drm_gsgpu_cs_chunk_fence {
	__u32 handle;
	__u32 offset;
};

struct drm_gsgpu_cs_chunk_sem {
	__u32 handle;
};

struct drm_gsgpu_cs_chunk_syncobj {
       __u32 handle;
       __u32 flags;
       __u64 point;
};

#define GSGPU_FENCE_TO_HANDLE_GET_SYNCOBJ	0
#define GSGPU_FENCE_TO_HANDLE_GET_SYNCOBJ_FD	1
#define GSGPU_FENCE_TO_HANDLE_GET_SYNC_FILE_FD	2

union drm_gsgpu_fence_to_handle {
	struct {
		struct drm_gsgpu_fence fence;
		__u32 what;
		__u32 pad;
	} in;
	struct {
		__u32 handle;
	} out;
};

struct drm_gsgpu_cs_chunk_data {
	union {
		struct drm_gsgpu_cs_chunk_ib		ib_data;
		struct drm_gsgpu_cs_chunk_fence	fence_data;
	};
};

/*
 *  Query h/w info: Flag that this is integrated (a.h.a. fusion) GPU
 *
 */
#define GSGPU_IDS_FLAGS_FUSION         0x1
#define GSGPU_IDS_FLAGS_PREEMPTION     0x2
#define GSGPU_IDS_FLAGS_TMZ            0x4
#define GSGPU_IDS_FLAGS_CONFORMANT_TRUNC_COORD 0x8

/* indicate if acceleration can be working */
#define GSGPU_INFO_ACCEL_WORKING		0x00
/* get the crtc_id from the mode object id? */
#define GSGPU_INFO_CRTC_FROM_ID		0x01
/* query hw IP info */
#define GSGPU_INFO_HW_IP_INFO			0x02
/* query hw IP instance count for the specified type */
#define GSGPU_INFO_HW_IP_COUNT			0x03
/* timestamp for GL_ARB_timer_query */
#define GSGPU_INFO_TIMESTAMP			0x05
/* Query the firmware version */
#define GSGPU_INFO_FW_VERSION			0x0e
	/* Subquery id: Query VCE firmware version */
	#define GSGPU_INFO_FW_VCE		0x1
	/* Subquery id: Query UVD firmware version */
	#define GSGPU_INFO_FW_UVD		0x2
	/* Subquery id: Query GMC firmware version */
	#define GSGPU_INFO_FW_GMC		0x03
	/* Subquery id: Query GFX ME firmware version */
	#define GSGPU_INFO_FW_GFX_ME		0x04
	/* Subquery id: Query GFX PFP firmware version */
	#define GSGPU_INFO_FW_GFX_PFP		0x05
	/* Subquery id: Query GFX CE firmware version */
	#define GSGPU_INFO_FW_GFX_CE		0x06
	/* Subquery id: Query GFX RLC firmware version */
	#define GSGPU_INFO_FW_GFX_RLC		0x07
	/* Subquery id: Query GFX MEC firmware version */
	#define GSGPU_INFO_FW_GFX_MEC		0x08
	/* Subquery id: Query SMC firmware version */
	#define GSGPU_INFO_FW_SMC		0x0a
	/* Subquery id: Query SDMA firmware version */
	#define GSGPU_INFO_FW_XDMA		0x0b
	/* Subquery id: Query PSP SOS firmware version */
	#define GSGPU_INFO_FW_SOS		0x0c
	/* Subquery id: Query PSP ASD firmware version */
	#define GSGPU_INFO_FW_ASD		0x0d
	/* Subquery id: Query VCN firmware version */
	#define GSGPU_INFO_FW_VCN		0x0e
	/* Subquery id: Query GFX RLC SRLC firmware version */
	#define GSGPU_INFO_FW_GFX_RLC_RESTORE_LIST_CNTL 0x0f
	/* Subquery id: Query GFX RLC SRLG firmware version */
	#define GSGPU_INFO_FW_GFX_RLC_RESTORE_LIST_GPM_MEM 0x10
	/* Subquery id: Query GFX RLC SRLS firmware version */
	#define GSGPU_INFO_FW_GFX_RLC_RESTORE_LIST_SRM_MEM 0x11
	/* Subquery id: Query DMCU firmware version */
	#define GSGPU_INFO_FW_DMCU		0x12
	#define GSGPU_INFO_FW_TA		0x13
	/* Subquery id: Query DMCUB firmware version */
	#define GSGPU_INFO_FW_DMCUB		0x14
	/* Subquery id: Query TOC firmware version */
	#define GSGPU_INFO_FW_TOC		0x15
	/* Subquery id: Query CAP firmware version */
	#define GSGPU_INFO_FW_CAP		0x16
	/* Subquery id: Query GFX RLCP firmware version */
	#define GSGPU_INFO_FW_GFX_RLCP		0x17
	/* Subquery id: Query GFX RLCV firmware version */
	#define GSGPU_INFO_FW_GFX_RLCV		0x18
	/* Subquery id: Query MES_KIQ firmware version */
	#define GSGPU_INFO_FW_MES_KIQ		0x19
	/* Subquery id: Query MES firmware version */
	#define GSGPU_INFO_FW_MES		0x1a
	/* Subquery id: Query IMU firmware version */
	#define GSGPU_INFO_FW_IMU		0x1b

/* number of bytes moved for TTM migration */
#define GSGPU_INFO_NUM_BYTES_MOVED		0x0f
/* the used VRAM size */
#define GSGPU_INFO_VRAM_USAGE			0x10
/* the used GTT size */
#define GSGPU_INFO_GTT_USAGE			0x11
/* Information about GDS, etc. resource configuration */
#define GSGPU_INFO_GDS_CONFIG			0x13
/* Query information about VRAM and GTT domains */
#define GSGPU_INFO_VRAM_GTT			0x14
/* Query information about register in MMR address space*/
#define GSGPU_INFO_READ_MMR_REG		0x15
/* Query information about device: rev id, family, etc. */
#define GSGPU_INFO_DEV_INFO			0x16
/* visible vram usage */
#define GSGPU_INFO_VIS_VRAM_USAGE		0x17
/* number of TTM buffer evictions */
#define GSGPU_INFO_NUM_EVICTIONS		0x18
/* Query memory about VRAM and GTT domains */
#define GSGPU_INFO_MEMORY			0x19
/* Query vce clock table */
#define GSGPU_INFO_VCE_CLOCK_TABLE		0x1A
/* Query vbios related information */
#define GSGPU_INFO_VBIOS			0x1B
	/* Subquery id: Query vbios size */
	#define GSGPU_INFO_VBIOS_SIZE		0x1
	/* Subquery id: Query vbios image */
	#define GSGPU_INFO_VBIOS_IMAGE		0x2
	/* Subquery id: Query vbios info */
	#define GSGPU_INFO_VBIOS_INFO		0x3
/* Query UVD handles */
#define GSGPU_INFO_NUM_HANDLES			0x1C
/* Query sensor related information */
#define GSGPU_INFO_SENSOR			0x1D
	/* Subquery id: Query GPU shader clock */
	#define GSGPU_INFO_SENSOR_GFX_SCLK		0x1
	/* Subquery id: Query GPU memory clock */
	#define GSGPU_INFO_SENSOR_GFX_MCLK		0x2
	/* Subquery id: Query GPU temperature */
	#define GSGPU_INFO_SENSOR_GPU_TEMP		0x3
	/* Subquery id: Query GPU load */
	#define GSGPU_INFO_SENSOR_GPU_LOAD		0x4
	/* Subquery id: Query average GPU power	*/
	#define GSGPU_INFO_SENSOR_GPU_AVG_POWER	0x5
	/* Subquery id: Query northbridge voltage */
	#define GSGPU_INFO_SENSOR_VDDNB		0x6
	/* Subquery id: Query graphics voltage */
	#define GSGPU_INFO_SENSOR_VDDGFX		0x7
	/* Subquery id: Query GPU stable pstate shader clock */
	#define GSGPU_INFO_SENSOR_STABLE_PSTATE_GFX_SCLK		0x8
	/* Subquery id: Query GPU stable pstate memory clock */
	#define GSGPU_INFO_SENSOR_STABLE_PSTATE_GFX_MCLK		0x9
	/* Subquery id: Query GPU peak pstate shader clock */
	#define GSGPU_INFO_SENSOR_PEAK_PSTATE_GFX_SCLK			0xa
	/* Subquery id: Query GPU peak pstate memory clock */
	#define GSGPU_INFO_SENSOR_PEAK_PSTATE_GFX_MCLK			0xb
/* Number of VRAM page faults on CPU access. */
#define GSGPU_INFO_NUM_VRAM_CPU_PAGE_FAULTS	0x1E
#define GSGPU_INFO_VRAM_LOST_COUNTER		0x1F
/* query ras mask of enabled features*/
#define GSGPU_INFO_RAS_ENABLED_FEATURES	0x20
/* RAS MASK: UMC (VRAM) */
#define GSGPU_INFO_RAS_ENABLED_UMC			(1 << 0)
/* RAS MASK: SDMA */
#define GSGPU_INFO_RAS_ENABLED_SDMA			(1 << 1)
/* RAS MASK: GFX */
#define GSGPU_INFO_RAS_ENABLED_GFX			(1 << 2)
/* RAS MASK: MMHUB */
#define GSGPU_INFO_RAS_ENABLED_MMHUB			(1 << 3)
/* RAS MASK: ATHUB */
#define GSGPU_INFO_RAS_ENABLED_ATHUB			(1 << 4)
/* RAS MASK: PCIE */
#define GSGPU_INFO_RAS_ENABLED_PCIE			(1 << 5)
/* RAS MASK: HDP */
#define GSGPU_INFO_RAS_ENABLED_HDP			(1 << 6)
/* RAS MASK: XGMI */
#define GSGPU_INFO_RAS_ENABLED_XGMI			(1 << 7)
/* RAS MASK: DF */
#define GSGPU_INFO_RAS_ENABLED_DF			(1 << 8)
/* RAS MASK: SMN */
#define GSGPU_INFO_RAS_ENABLED_SMN			(1 << 9)
/* RAS MASK: SEM */
#define GSGPU_INFO_RAS_ENABLED_SEM			(1 << 10)
/* RAS MASK: MP0 */
#define GSGPU_INFO_RAS_ENABLED_MP0			(1 << 11)
/* RAS MASK: MP1 */
#define GSGPU_INFO_RAS_ENABLED_MP1			(1 << 12)
/* RAS MASK: FUSE */
#define GSGPU_INFO_RAS_ENABLED_FUSE			(1 << 13)
/* query video encode/decode caps */
#define GSGPU_INFO_VIDEO_CAPS			0x21
	/* Subquery id: Decode */
	#define GSGPU_INFO_VIDEO_CAPS_DECODE		0
	/* Subquery id: Encode */
	#define GSGPU_INFO_VIDEO_CAPS_ENCODE		1

#define GSGPU_INFO_MMR_SE_INDEX_SHIFT	0
#define GSGPU_INFO_MMR_SE_INDEX_MASK	0xff
#define GSGPU_INFO_MMR_SH_INDEX_SHIFT	8
#define GSGPU_INFO_MMR_SH_INDEX_MASK	0xff

struct drm_gsgpu_query_fw {
	/** GSGPU_INFO_FW_* */
	__u32 fw_type;
	/**
	 * Index of the IP if there are more IPs of
	 * the same type.
	 */
	__u32 ip_instance;
	/**
	 * Index of the engine. Whether this is used depends
	 * on the firmware type. (e.g. MEC, SDMA)
	 */
	__u32 index;
	__u32 _pad;
};

/* Input structure for the INFO ioctl */
struct drm_gsgpu_info {
	/* Where the return value will be stored */
	__u64 return_pointer;
	/* The size of the return value. Just like "size" in "snprintf",
	 * it limits how many bytes the kernel can write. */
	__u32 return_size;
	/* The query request id. */
	__u32 query;

	union {
		struct {
			__u32 id;
			__u32 _pad;
		} mode_crtc;

		struct {
			/** GSGPU_HW_IP_* */
			__u32 type;
			/**
			 * Index of the IP if there are more IPs of the same
			 * type. Ignored by GSGPU_INFO_HW_IP_COUNT.
			 */
			__u32 ip_instance;
		} query_hw_ip;

		struct {
			__u32 dword_offset;
			/** number of registers to read */
			__u32 count;
			__u32 instance;
			/** For future use, no flags defined so far */
			__u32 flags;
		} read_mmr_reg;

		struct drm_gsgpu_query_fw query_fw;

		struct {
			__u32 type;
			__u32 offset;
		} vbios_info;

		struct {
			__u32 type;
		} sensor_info;

		struct {
			__u32 type;
		} video_cap;
	};
};

struct drm_gsgpu_info_gds {
	/** GDS GFX partition size */
	__u32 gds_gfx_partition_size;
	/** GDS compute partition size */
	__u32 compute_partition_size;
	/** total GDS memory size */
	__u32 gds_total_size;
	/** GWS size per GFX partition */
	__u32 gws_per_gfx_partition;
	/** GSW size per compute partition */
	__u32 gws_per_compute_partition;
	/** OA size per GFX partition */
	__u32 oa_per_gfx_partition;
	/** OA size per compute partition */
	__u32 oa_per_compute_partition;
	__u32 _pad;
};

struct drm_gsgpu_info_vram_gtt {
	__u64 vram_size;
	__u64 vram_cpu_accessible_size;
	__u64 gtt_size;
};

struct drm_gsgpu_heap_info {
	/** max. physical memory */
	__u64 total_heap_size;

	/** Theoretical max. available memory in the given heap */
	__u64 usable_heap_size;

	/**
	 * Number of bytes allocated in the heap. This includes all processes
	 * and private allocations in the kernel. It changes when new buffers
	 * are allocated, freed, and moved. It cannot be larger than
	 * heap_size.
	 */
	__u64 heap_usage;

	/**
	 * Theoretical possible max. size of buffer which
	 * could be allocated in the given heap
	 */
	__u64 max_allocation;
};

struct drm_gsgpu_memory_info {
	struct drm_gsgpu_heap_info vram;
	struct drm_gsgpu_heap_info cpu_accessible_vram;
	struct drm_gsgpu_heap_info gtt;
};

struct drm_gsgpu_info_firmware {
	__u32 ver;
	__u32 feature;
};

struct drm_gsgpu_info_vbios {
	__u8 name[64];
	__u8 vbios_pn[64];
	__u32 version;
	__u32 pad;
	__u8 vbios_ver_str[32];
	__u8 date[32];
};

#define GSGPU_VRAM_TYPE_UNKNOWN 0
#define GSGPU_VRAM_TYPE_GDDR1 1
#define GSGPU_VRAM_TYPE_DDR2  2
#define GSGPU_VRAM_TYPE_GDDR3 3
#define GSGPU_VRAM_TYPE_GDDR4 4
#define GSGPU_VRAM_TYPE_GDDR5 5
#define GSGPU_VRAM_TYPE_HBM   6
#define GSGPU_VRAM_TYPE_DDR3  7
#define GSGPU_VRAM_TYPE_DDR4  8
#define GSGPU_VRAM_TYPE_GDDR6 9
#define GSGPU_VRAM_TYPE_DDR5  10
#define GSGPU_VRAM_TYPE_LPDDR4 11
#define GSGPU_VRAM_TYPE_LPDDR5 12

struct drm_gsgpu_info_device {
	/** PCI Device ID */
	__u32 device_id;
	/** Internal chip revision: A0, A1, etc.) */
	__u32 chip_rev;
	__u32 external_rev;
	/** Revision id in PCI Config space */
	__u32 pci_rev;
	__u32 family;
	__u32 num_shader_engines;
	__u32 num_shader_arrays_per_engine;
	/* in KHz */
	__u32 gpu_counter_freq;
	__u64 max_engine_clock;
	__u64 max_memory_clock;
	/* cu information */
	__u32 cu_active_number;
	/* NOTE: cu_ao_mask is INVALID, DON'T use it */
	__u32 cu_ao_mask;
	__u32 cu_bitmap[4][4];
	/** Render backend pipe mask. One render backend is CB+DB. */
	__u32 enabled_rb_pipes_mask;
	__u32 num_rb_pipes;
	__u32 num_hw_gfx_contexts;
	/* PCIe version (the smaller of the GPU and the CPU/motherboard) */
	__u32 pcie_gen;
	__u64 ids_flags;
	/** Starting virtual address for UMDs. */
	__u64 virtual_address_offset;
	/** The maximum virtual address */
	__u64 virtual_address_max;
	/** Required alignment of virtual addresses. */
	__u32 virtual_address_alignment;
	/** Page table entry - fragment size */
	__u32 pte_fragment_size;
	__u32 gart_page_size;
	/** constant engine ram size*/
	__u32 ce_ram_size;
	/** video memory type info*/
	__u32 vram_type;
	/** video memory bit width*/
	__u32 vram_bit_width;
	/* vce harvesting instance */
	__u32 vce_harvest_config;
	/* gfx double offchip LDS buffers */
	__u32 gc_double_offchip_lds_buf;
	/* NGG Primitive Buffer */
	__u64 prim_buf_gpu_addr;
	/* NGG Position Buffer */
	__u64 pos_buf_gpu_addr;
	/* NGG Control Sideband */
	__u64 cntl_sb_buf_gpu_addr;
	/* NGG Parameter Cache */
	__u64 param_buf_gpu_addr;
	__u32 prim_buf_size;
	__u32 pos_buf_size;
	__u32 cntl_sb_buf_size;
	__u32 param_buf_size;
	/* wavefront size*/
	__u32 wave_front_size;
	/* shader visible vgprs*/
	__u32 num_shader_visible_vgprs;
	/* CU per shader array*/
	__u32 num_cu_per_sh;
	/* number of tcc blocks*/
	__u32 num_tcc_blocks;
	/* gs vgt table depth*/
	__u32 gs_vgt_table_depth;
	/* gs primitive buffer depth*/
	__u32 gs_prim_buffer_depth;
	/* max gs wavefront per vgt*/
	__u32 max_gs_waves_per_vgt;
	/* PCIe number of lanes (the smaller of the GPU and the CPU/motherboard) */
	__u32 pcie_num_lanes;
	/* always on cu bitmap */
	__u32 cu_ao_bitmap[4][4];
	/** Starting high virtual address for UMDs. */
	__u64 high_va_offset;
	/** The maximum high virtual address */
	__u64 high_va_max;
	/* gfx10 pa_sc_tile_steering_override */
	__u32 pa_sc_tile_steering_override;
	/* disabled TCCs */
	__u64 tcc_disabled_mask;
	__u64 min_engine_clock;
	__u64 min_memory_clock;
	/* The following fields are only set on gfx11+, older chips set 0. */
	__u32 tcp_cache_size;       /* AKA GL0, VMEM cache */
	__u32 num_sqc_per_wgp;
	__u32 sqc_data_cache_size;  /* AKA SMEM cache */
	__u32 sqc_inst_cache_size;
	__u32 gl1c_cache_size;
	__u32 gl2c_cache_size;
	__u64 mall_size;            /* AKA infinity cache */
	/* high 32 bits of the rb pipes mask */
	__u32 enabled_rb_pipes_mask_hi;
};

struct drm_gsgpu_info_hw_ip {
	/** Version of h/w IP */
	__u32  hw_ip_version_major;
	__u32  hw_ip_version_minor;
	/** Capabilities */
	__u64  capabilities_flags;
	/** command buffer address start alignment*/
	__u32  ib_start_alignment;
	/** command buffer size alignment*/
	__u32  ib_size_alignment;
	/** Bitmask of available rings. Bit 0 means ring 0, etc. */
	__u32  available_rings;
	/** version info: bits 23:16 major, 15:8 minor, 7:0 revision */
	__u32  ip_discovery_version;
};

struct drm_gsgpu_info_num_handles {
	/** Max handles as supported by firmware for UVD */
	__u32  uvd_max_handles;
	/** Handles currently in use for UVD */
	__u32  uvd_used_handles;
};

#define       GSGPU_HW_SEMA_GET         1
#define       GSGPU_HW_SEMA_PUT         2

struct drm_gsgpu_hw_sema {
    /*get or set sema*/
    __u64 id;
    /*resv for next feature*/
    __u32 ctx_id;
     /*ops*/
    __u32 ops;
};

/* query video encode/decode caps */
#define GSGPU_INFO_VIDEO_CAPS_CODEC_IDX_MPEG2			0
#define GSGPU_INFO_VIDEO_CAPS_CODEC_IDX_MPEG4			1
#define GSGPU_INFO_VIDEO_CAPS_CODEC_IDX_VC1			2
#define GSGPU_INFO_VIDEO_CAPS_CODEC_IDX_MPEG4_AVC		3
#define GSGPU_INFO_VIDEO_CAPS_CODEC_IDX_HEVC			4
#define GSGPU_INFO_VIDEO_CAPS_CODEC_IDX_JPEG			5
#define GSGPU_INFO_VIDEO_CAPS_CODEC_IDX_VP9			6
#define GSGPU_INFO_VIDEO_CAPS_CODEC_IDX_AV1			7
#define GSGPU_INFO_VIDEO_CAPS_CODEC_IDX_COUNT			8

struct drm_gsgpu_info_video_codec_info {
	__u32 valid;
	__u32 max_width;
	__u32 max_height;
	__u32 max_pixels_per_frame;
	__u32 max_level;
	__u32 pad;
};

struct drm_gsgpu_info_video_caps {
	struct drm_gsgpu_info_video_codec_info codec_info[GSGPU_INFO_VIDEO_CAPS_CODEC_IDX_COUNT];
};

/*
 * Supported GPU families
 */
#define GSGPU_FAMILY_UNKNOWN			0
#define GSGPU_FAMILY_SI			110 /* Hainan, Oland, Verde, Pitcairn, Tahiti */
#define GSGPU_FAMILY_CI			120 /* Bonaire, Hawaii */
#define GSGPU_FAMILY_KV			125 /* Kaveri, Kabini, Mullins */
#define GSGPU_FAMILY_VI			130 /* Iceland, Tonga */
#define GSGPU_FAMILY_CZ			135 /* Carrizo, Stoney */
#define GSGPU_FAMILY_AI			141 /* Vega10 */
#define GSGPU_FAMILY_RV			142 /* Raven */
#define GSGPU_FAMILY_NV			143 /* Navi10 */
#define GSGPU_FAMILY_VGH			144 /* Van Gogh */
#define GSGPU_FAMILY_GC_11_0_0			145 /* GC 11.0.0 */
#define GSGPU_FAMILY_YC			146 /* Yellow Carp */
#define GSGPU_FAMILY_GC_11_0_1			148 /* GC 11.0.1 */
#define GSGPU_FAMILY_GC_10_3_6			149 /* GC 10.3.6 */
#define GSGPU_FAMILY_GC_10_3_7			151 /* GC 10.3.7 */

#if defined(__cplusplus)
}
#endif

#endif
