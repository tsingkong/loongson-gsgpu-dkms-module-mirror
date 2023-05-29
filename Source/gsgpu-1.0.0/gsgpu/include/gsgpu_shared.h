/*
 * Copyright 2015 Advanced Micro Devices, Inc.
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
 */

#ifndef __GSGPU_SHARED_H__
#define __GSGPU_SHARED_H__

#include <drm/gsgpu_family_type.h>

/*
 * Chip flags
 */
enum gsgpu_chip_flags {
	GSGPU_ASIC_MASK = 0x0000ffffUL,
	GSGPU_FLAGS_MASK  = 0xffff0000UL,
	GSGPU_IS_MOBILITY = 0x00010000UL,
	GSGPU_IS_APU      = 0x00020000UL,
	GSGPU_IS_PX       = 0x00040000UL,
	GSGPU_EXP_HW_SUPPORT = 0x00080000UL,
};

enum gsgpu_apu_flags {
	GSGPU_APU_IS_RAVEN = 0x00000001UL,
	GSGPU_APU_IS_RAVEN2 = 0x00000002UL,
	GSGPU_APU_IS_PICASSO = 0x00000004UL,
	GSGPU_APU_IS_RENOIR = 0x00000008UL,
	GSGPU_APU_IS_GREEN_SARDINE = 0x00000010UL,
	GSGPU_APU_IS_VANGOGH = 0x00000020UL,
	GSGPU_APU_IS_CYAN_SKILLFISH2 = 0x00000040UL,
};

/**
* DOC: IP Blocks
*
* GPUs are composed of IP (intellectual property) blocks. These
* IP blocks provide various functionalities: display, graphics,
* video decode, etc. The IP blocks that comprise a particular GPU
* are listed in the GPU's respective SoC file. gsgpu_device.c
* acquires the list of IP blocks for the GPU in use on initialization.
* It can then operate on this list to perform standard driver operations
* such as: init, fini, suspend, resume, etc.
* 
*
* IP block implementations are named using the following convention:
* <functionality>_v<version> (E.g.: gfx_v6_0).
*/

/**
* enum gsgpu_ip_block_type - Used to classify IP blocks by functionality.
*
* @GSGPU_IP_BLOCK_TYPE_COMMON: GPU Family
* @GSGPU_IP_BLOCK_TYPE_GMC: Graphics Memory Controller
* @GSGPU_IP_BLOCK_TYPE_IH: Interrupt Handler
* @GSGPU_IP_BLOCK_TYPE_SMC: System Management Controller
* @GSGPU_IP_BLOCK_TYPE_PSP: Platform Security Processor
* @GSGPU_IP_BLOCK_TYPE_DCE: Display and Compositing Engine
* @GSGPU_IP_BLOCK_TYPE_GFX: Graphics and Compute Engine
* @GSGPU_IP_BLOCK_TYPE_SDMA: System DMA Engine
* @GSGPU_IP_BLOCK_TYPE_UVD: Unified Video Decoder
* @GSGPU_IP_BLOCK_TYPE_VCE: Video Compression Engine
* @GSGPU_IP_BLOCK_TYPE_ACP: Audio Co-Processor
* @GSGPU_IP_BLOCK_TYPE_VCN: Video Core/Codec Next
* @GSGPU_IP_BLOCK_TYPE_MES: Micro-Engine Scheduler
* @GSGPU_IP_BLOCK_TYPE_JPEG: JPEG Engine
* @GSGPU_IP_BLOCK_TYPE_NUM: Total number of IP block types
*/
enum gsgpu_ip_block_type {
	GSGPU_IP_BLOCK_TYPE_COMMON,
	GSGPU_IP_BLOCK_TYPE_GMC,
	GSGPU_IP_BLOCK_TYPE_ZIP,
	GSGPU_IP_BLOCK_TYPE_IH,
	GSGPU_IP_BLOCK_TYPE_DCE,
	GSGPU_IP_BLOCK_TYPE_GFX,
	GSGPU_IP_BLOCK_TYPE_XDMA
};

enum gsgpu_harvest_ip_mask {
    GSGPU_HARVEST_IP_VCN_MASK = 0x1,
    GSGPU_HARVEST_IP_JPEG_MASK = 0x2,
    GSGPU_HARVEST_IP_DMU_MASK = 0x4,
};

/**
 * struct gsgpu_ip_funcs - general hooks for managing gsgpu IP Blocks
 * @name: Name of IP block
 * @early_init: sets up early driver state (pre sw_init),
 *              does not configure hw - Optional
 * @late_init: sets up late driver/hw state (post hw_init) - Optional
 * @sw_init: sets up driver state, does not configure hw
 * @sw_fini: tears down driver state, does not configure hw
 * @early_fini: tears down stuff before dev detached from driver
 * @hw_init: sets up the hw state
 * @hw_fini: tears down the hw state
 * @late_fini: final cleanup
 * @suspend: handles IP specific hw/sw changes for suspend
 * @resume: handles IP specific hw/sw changes for resume
 * @is_idle: returns current IP block idle status
 * @wait_for_idle: poll for idle
 * @check_soft_reset: check soft reset the IP block
 * @pre_soft_reset: pre soft reset the IP block
 * @soft_reset: soft reset the IP block
 * @post_soft_reset: post soft reset the IP block
 *
 * These hooks provide an interface for controlling the operational state
 * of IP blocks. After acquiring a list of IP blocks for the GPU in use,
 * the driver can make chip-wide state changes by walking this list and
 * making calls to hooks from each IP block. This list is ordered to ensure
 * that the driver initializes the IP blocks in a safe sequence.
 */
struct gsgpu_ip_funcs {
	char *name;
	int (*early_init)(void *handle);
	int (*late_init)(void *handle);
	int (*sw_init)(void *handle);
	int (*sw_fini)(void *handle);
	int (*early_fini)(void *handle);
	int (*hw_init)(void *handle);
	int (*hw_fini)(void *handle);
	void (*late_fini)(void *handle);
	int (*suspend)(void *handle);
	int (*resume)(void *handle);
	bool (*is_idle)(void *handle);
	int (*wait_for_idle)(void *handle);
	bool (*check_soft_reset)(void *handle);
	int (*pre_soft_reset)(void *handle);
	int (*soft_reset)(void *handle);
	int (*post_soft_reset)(void *handle);
};


#endif /* __GSGPU_SHARED_H__ */
