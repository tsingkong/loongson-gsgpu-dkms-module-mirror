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
#include <linux/power_supply.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/console.h>
#include <linux/slab.h>
#include <linux/iommu.h>
#include <linux/pci.h>
#include <linux/devcoredump.h>
#include <generated/utsrelease.h>
#include <linux/pci-p2pdma.h>

#include <drm/drm_aperture.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/gsgpu_drm.h>
#include <linux/vgaarb.h>
#include <linux/vga_switcheroo.h>
#include <linux/efi.h>
#include "gsgpu.h"
#include "gsgpu_trace.h"
#include "gsgpu_cp.h"
#include "gsgpu_common.h"
#include <linux/firmware.h>
#include "gsgpu_pm.h"
#include "gsgpu_reset.h"
#include "gsgpu_gem.h"
#include <linux/suspend.h>
#include <drm/task_barrier.h>
#include <linux/pm_runtime.h>

#include <drm/drm_drv.h>
#include <drm/ttm/ttm_tt.h>

#if IS_ENABLED(CONFIG_X86)
#include <asm/intel-family.h>
#endif

#define GSGPU_RESUME_MS		2000
#define GSGPU_MAX_RETRY_LIMIT		2
#define GSGPU_RETRY_SRIOV_RESET(r) ((r) == -EBUSY || (r) == -ETIMEDOUT || (r) == -EINVAL)

static const struct drm_driver gsgpu_kms_driver;

static const char *gsgpu_family_name[] = {
	"LG100",
};

/**
 * DOC: pcie_replay_count
 *
 * The gsgpu driver provides a sysfs API for reporting the total number
 * of PCIe replays (NAKs)
 * The file pcie_replay_count is used for this and returns the total
 * number of replays as a sum of the NAKs generated and NAKs received
 */

static ssize_t gsgpu_device_get_pcie_replay_count(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	// struct drm_device *ddev = dev_get_drvdata(dev);
	// struct gsgpu_device *adev = drm_to_adev(ddev);
	uint64_t cnt = 0; // gsgpu_asic_get_pcie_replay_count(adev);

	return sysfs_emit(buf, "%llu\n", cnt);
}

static DEVICE_ATTR(pcie_replay_count, S_IRUGO,
		gsgpu_device_get_pcie_replay_count, NULL);

static void gsgpu_device_get_pcie_info(struct gsgpu_device *adev);

/**
 * DOC: product_name
 *
 * The gsgpu driver provides a sysfs API for reporting the product name
 * for the device
 * The file product_name is used for this and returns the product name
 * as returned from the FRU.
 * NOTE: This is only available for certain server cards
 */

static ssize_t gsgpu_device_get_product_name(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_device *ddev = dev_get_drvdata(dev);
	struct gsgpu_device *adev = drm_to_adev(ddev);

	return sysfs_emit(buf, "%s\n", adev->product_name);
}

static DEVICE_ATTR(product_name, S_IRUGO,
		gsgpu_device_get_product_name, NULL);

/**
 * DOC: product_number
 *
 * The gsgpu driver provides a sysfs API for reporting the part number
 * for the device
 * The file product_number is used for this and returns the part number
 * as returned from the FRU.
 * NOTE: This is only available for certain server cards
 */

static ssize_t gsgpu_device_get_product_number(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_device *ddev = dev_get_drvdata(dev);
	struct gsgpu_device *adev = drm_to_adev(ddev);

	return sysfs_emit(buf, "%s\n", adev->product_number);
}

static DEVICE_ATTR(product_number, S_IRUGO,
		gsgpu_device_get_product_number, NULL);

/**
 * DOC: serial_number
 *
 * The gsgpu driver provides a sysfs API for reporting the serial number
 * for the device
 * The file serial_number is used for this and returns the serial number
 * as returned from the FRU.
 * NOTE: This is only available for certain server cards
 */

static ssize_t gsgpu_device_get_serial_number(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_device *ddev = dev_get_drvdata(dev);
	struct gsgpu_device *adev = drm_to_adev(ddev);

	return sysfs_emit(buf, "%s\n", adev->serial);
}

static DEVICE_ATTR(serial_number, S_IRUGO,
		gsgpu_device_get_serial_number, NULL);

/**
 * gsgpu_cmd_exec
 * XXX while block will taking kernel hang!
 * @adev
 * @cmd
 * @arg0
 * @arg1
 *
 * Return:

 */
uint64_t gsgpu_cmd_exec(struct gsgpu_device *adev, uint32_t cmd, uint32_t arg0, uint32_t arg1)
{
	uint64_t ret;
	// int sts;
	// unsigned i;

	if(gsgpu_cp_wait_done(adev) == false)
		return  ~0ULL;

	writel(GSCMD_STS_NULL, ((void __iomem *)adev->rmmio) + GSGPU_STATUS);

	writel(cmd, ((void __iomem *)adev->rmmio) + GSGPU_COMMAND);
	writel(arg0, ((void __iomem *)adev->rmmio) + GSGPU_ARGUMENT0);
	writel(arg1, ((void __iomem *)adev->rmmio) + GSGPU_ARGUMENT1);

	writel(1, ((void __iomem *)adev->rmmio) + GSGPU_EC_INT);


	if(gsgpu_cp_wait_done(adev) == false)
		return  ~0ULL;

	ret = readl(((void __iomem *)adev->rmmio) + GSGPU_RETURN0);
	ret |= (uint64_t)readl(((void __iomem *)adev->rmmio) + GSGPU_RETURN1)<<32;

	return ret;
}

/*
 * VRAM access helper functions
 */
#define mmMM_INDEX         0x0
#define mmMM_INDEX_HI      0x6
#define mmMM_DATA          0x1

/**
 * gsgpu_device_mm_access - access vram by MM_INDEX/MM_DATA
 *
 * @adev: gsgpu_device pointer
 * @pos: offset of the buffer in vram
 * @buf: virtual address of the buffer in system memory
 * @size: read/write size, sizeof(@buf) must > @size
 * @write: true - write to vram, otherwise - read from vram
 */
void gsgpu_device_mm_access(struct gsgpu_device *adev, loff_t pos,
			     void *buf, size_t size, bool write)
{
	unsigned long flags;
	uint32_t hi = ~0, tmp = 0;
	uint32_t *data = buf;
	uint64_t last;
	int idx;

	if (!drm_dev_enter(adev_to_drm(adev), &idx))
		return;

	BUG_ON(!IS_ALIGNED(pos, 4) || !IS_ALIGNED(size, 4));

	spin_lock_irqsave(&adev->mmio_idx_lock, flags);
	for (last = pos + size; pos < last; pos += 4) {
		tmp = pos >> 31;

		WREG32_NO_KIQ(mmMM_INDEX, ((uint32_t)pos) | 0x80000000);
		if (tmp != hi) {
			WREG32_NO_KIQ(mmMM_INDEX_HI, tmp);
			hi = tmp;
		}
		if (write)
			WREG32_NO_KIQ(mmMM_DATA, *data++);
		else
			*data++ = RREG32_NO_KIQ(mmMM_DATA);
	}

	spin_unlock_irqrestore(&adev->mmio_idx_lock, flags);
	drm_dev_exit(idx);
}

/**
 * gsgpu_device_aper_access - access vram by vram aperature
 *
 * @adev: gsgpu_device pointer
 * @pos: offset of the buffer in vram
 * @buf: virtual address of the buffer in system memory
 * @size: read/write size, sizeof(@buf) must > @size
 * @write: true - write to vram, otherwise - read from vram
 *
 * The return value means how many bytes have been transferred.
 */
size_t gsgpu_device_aper_access(struct gsgpu_device *adev, loff_t pos,
				 void *buf, size_t size, bool write)
{
#ifdef CONFIG_64BIT
	void __iomem *addr;
	size_t count = 0;
	uint64_t last;

	if (!adev->mman.aper_base_kaddr)
		return 0;

	last = min(pos + size, adev->gmc.visible_vram_size);
	if (last > pos) {
		addr = adev->mman.aper_base_kaddr + pos;
		count = last - pos;

		if (write) {
			memcpy_toio(addr, buf, count);
			mb();
			gsgpu_device_flush_hdp(adev, NULL);
		} else {
			gsgpu_device_invalidate_hdp(adev, NULL);
			mb();
			memcpy_fromio(buf, addr, count);
		}

	}

	return count;
#else
	return 0;
#endif
}

/**
 * gsgpu_device_vram_access - read/write a buffer in vram
 *
 * @adev: gsgpu_device pointer
 * @pos: offset of the buffer in vram
 * @buf: virtual address of the buffer in system memory
 * @size: read/write size, sizeof(@buf) must > @size
 * @write: true - write to vram, otherwise - read from vram
 */
void gsgpu_device_vram_access(struct gsgpu_device *adev, loff_t pos,
			       void *buf, size_t size, bool write)
{
	size_t count;

	/* try to using vram apreature to access vram first */
	count = gsgpu_device_aper_access(adev, pos, buf, size, write);
	size -= count;
	if (size) {
		/* using MM to access rest vram */
		pos += count;
		buf += count;
		gsgpu_device_mm_access(adev, pos, buf, size, write);
	}
}

/*
 * register access helper functions.
 */

/* Check if hw access should be skipped because of hotplug or device error */
bool gsgpu_device_skip_hw_access(struct gsgpu_device *adev)
{
	if (adev->no_hw_access)
		return true;

#ifdef CONFIG_LOCKDEP
	/*
	 * This is a bit complicated to understand, so worth a comment. What we assert
	 * here is that the GPU reset is not running on another thread in parallel.
	 *
	 * For this we trylock the read side of the reset semaphore, if that succeeds
	 * we know that the reset is not running in paralell.
	 *
	 * If the trylock fails we assert that we are either already holding the read
	 * side of the lock or are the reset thread itself and hold the write side of
	 * the lock.
	 */
	if (in_task()) {
		if (down_read_trylock(&adev->reset_domain->sem))
			up_read(&adev->reset_domain->sem);
		else
			lockdep_assert_held(&adev->reset_domain->sem);
	}
#endif
	return false;
}

/**
 * gsgpu_device_rreg - read a memory mapped IO or indirect register
 *
 * @adev: gsgpu_device pointer
 * @reg: dword aligned register offset
 * @acc_flags: access flags which require special behavior
 *
 * Returns the 32 bit value from the offset specified.
 */
uint32_t gsgpu_device_rreg(struct gsgpu_device *adev,
			    uint32_t reg, uint32_t acc_flags)
{
	uint32_t ret;

	if (gsgpu_device_skip_hw_access(adev))
		return 0;

	if (reg < adev->rmmio_size && !(acc_flags & GSGPU_REGS_IDX))
		ret = readl(((void __iomem *)adev->rmmio) + reg);
	else {
		ret = 0;
		DRM_DEBUG_DRIVER("%s Not implemented\n", __func__);
	}

	trace_gsgpu_device_rreg(adev->pdev->device, reg, ret);

	return ret;
}

/*
 * MMIO register read with bytes helper functions
 * @offset:bytes offset from MMIO start
 *
*/

/**
 * gsgpu_mm_rreg8 - read a memory mapped IO register
 *
 * @adev: gsgpu_device pointer
 * @offset: byte aligned register offset
 *
 * Returns the 8 bit value from the offset specified.
 */
uint8_t gsgpu_mm_rreg8(struct gsgpu_device *adev, uint32_t offset)
{
	if (gsgpu_device_skip_hw_access(adev))
		return 0;

	if (offset < adev->rmmio_size)
		return (readb(adev->rmmio + offset));
	BUG();
}

/*
 * MMIO register write with bytes helper functions
 * @offset:bytes offset from MMIO start
 * @value: the value want to be written to the register
 *
*/
/**
 * gsgpu_mm_wreg8 - read a memory mapped IO register
 *
 * @adev: gsgpu_device pointer
 * @offset: byte aligned register offset
 * @value: 8 bit value to write
 *
 * Writes the value specified to the offset specified.
 */
void gsgpu_mm_wreg8(struct gsgpu_device *adev, uint32_t offset, uint8_t value)
{
	if (gsgpu_device_skip_hw_access(adev))
		return;

	if (offset < adev->rmmio_size)
		writeb(value, adev->rmmio + offset);
	else
		BUG();
}

/**
 * gsgpu_device_wreg - write to a memory mapped IO or indirect register
 *
 * @adev: gsgpu_device pointer
 * @reg: dword aligned register offset
 * @v: 32 bit value to write to the register
 * @acc_flags: access flags which require special behavior
 *
 * Writes the value specified to the offset specified.
 */
void gsgpu_device_wreg(struct gsgpu_device *adev,
			uint32_t reg, uint32_t v,
			uint32_t acc_flags)
{
	if (gsgpu_device_skip_hw_access(adev))
		return;

	if ((reg) < adev->rmmio_size) {
		writel(v, ((void __iomem *)adev->rmmio) + reg);
	} else
		BUG();

	trace_gsgpu_device_wreg(adev->pdev->device, reg, v);
}

/**
 * gsgpu_device_indirect_rreg - read an indirect register
 *
 * @adev: gsgpu_device pointer
 * @pcie_index: mmio register offset
 * @pcie_data: mmio register offset
 * @reg_addr: indirect register address to read from
 *
 * Returns the value of indirect register @reg_addr
 */
u32 gsgpu_device_indirect_rreg(struct gsgpu_device *adev,
				u32 pcie_index, u32 pcie_data,
				u32 reg_addr)
{
	unsigned long flags;
	u32 r;
	void __iomem *pcie_index_offset;
	void __iomem *pcie_data_offset;

	spin_lock_irqsave(&adev->pcie_idx_lock, flags);
	pcie_index_offset = (void __iomem *)adev->rmmio + pcie_index * 4;
	pcie_data_offset = (void __iomem *)adev->rmmio + pcie_data * 4;

	writel(reg_addr, pcie_index_offset);
	readl(pcie_index_offset);
	r = readl(pcie_data_offset);
	spin_unlock_irqrestore(&adev->pcie_idx_lock, flags);

	return r;
}

/**
 * gsgpu_device_indirect_rreg64 - read a 64bits indirect register
 *
 * @adev: gsgpu_device pointer
 * @pcie_index: mmio register offset
 * @pcie_data: mmio register offset
 * @reg_addr: indirect register address to read from
 *
 * Returns the value of indirect register @reg_addr
 */
u64 gsgpu_device_indirect_rreg64(struct gsgpu_device *adev,
				  u32 pcie_index, u32 pcie_data,
				  u32 reg_addr)
{
	unsigned long flags;
	u64 r;
	void __iomem *pcie_index_offset;
	void __iomem *pcie_data_offset;

	spin_lock_irqsave(&adev->pcie_idx_lock, flags);
	pcie_index_offset = (void __iomem *)adev->rmmio + pcie_index * 4;
	pcie_data_offset = (void __iomem *)adev->rmmio + pcie_data * 4;

	/* read low 32 bits */
	writel(reg_addr, pcie_index_offset);
	readl(pcie_index_offset);
	r = readl(pcie_data_offset);
	/* read high 32 bits */
	writel(reg_addr + 4, pcie_index_offset);
	readl(pcie_index_offset);
	r |= ((u64)readl(pcie_data_offset) << 32);
	spin_unlock_irqrestore(&adev->pcie_idx_lock, flags);

	return r;
}

/**
 * gsgpu_device_indirect_wreg - write an indirect register address
 *
 * @adev: gsgpu_device pointer
 * @pcie_index: mmio register offset
 * @pcie_data: mmio register offset
 * @reg_addr: indirect register offset
 * @reg_data: indirect register data
 *
 */
void gsgpu_device_indirect_wreg(struct gsgpu_device *adev,
				 u32 pcie_index, u32 pcie_data,
				 u32 reg_addr, u32 reg_data)
{
	unsigned long flags;
	void __iomem *pcie_index_offset;
	void __iomem *pcie_data_offset;

	spin_lock_irqsave(&adev->pcie_idx_lock, flags);
	pcie_index_offset = (void __iomem *)adev->rmmio + pcie_index * 4;
	pcie_data_offset = (void __iomem *)adev->rmmio + pcie_data * 4;

	writel(reg_addr, pcie_index_offset);
	readl(pcie_index_offset);
	writel(reg_data, pcie_data_offset);
	readl(pcie_data_offset);
	spin_unlock_irqrestore(&adev->pcie_idx_lock, flags);
}

/**
 * gsgpu_device_indirect_wreg64 - write a 64bits indirect register address
 *
 * @adev: gsgpu_device pointer
 * @pcie_index: mmio register offset
 * @pcie_data: mmio register offset
 * @reg_addr: indirect register offset
 * @reg_data: indirect register data
 *
 */
void gsgpu_device_indirect_wreg64(struct gsgpu_device *adev,
				   u32 pcie_index, u32 pcie_data,
				   u32 reg_addr, u64 reg_data)
{
	unsigned long flags;
	void __iomem *pcie_index_offset;
	void __iomem *pcie_data_offset;

	spin_lock_irqsave(&adev->pcie_idx_lock, flags);
	pcie_index_offset = (void __iomem *)adev->rmmio + pcie_index * 4;
	pcie_data_offset = (void __iomem *)adev->rmmio + pcie_data * 4;

	/* write low 32 bits */
	writel(reg_addr, pcie_index_offset);
	readl(pcie_index_offset);
	writel((u32)(reg_data & 0xffffffffULL), pcie_data_offset);
	readl(pcie_data_offset);
	/* write high 32 bits */
	writel(reg_addr + 4, pcie_index_offset);
	readl(pcie_index_offset);
	writel((u32)(reg_data >> 32), pcie_data_offset);
	readl(pcie_data_offset);
	spin_unlock_irqrestore(&adev->pcie_idx_lock, flags);
}

/**
 * gsgpu_invalid_rreg - dummy reg read function
 *
 * @adev: gsgpu_device pointer
 * @reg: offset of register
 *
 * Dummy register read function.  Used for register blocks
 * that certain asics don't have (all asics).
 * Returns the value in the register.
 */
// static uint32_t gsgpu_invalid_rreg(struct gsgpu_device *adev, uint32_t reg)
// {
// 	DRM_ERROR("Invalid callback to read register 0x%04X\n", reg);
// 	BUG();
// 	return 0;
// }

/**
 * gsgpu_invalid_wreg - dummy reg write function
 *
 * @adev: gsgpu_device pointer
 * @reg: offset of register
 * @v: value to write to the register
 *
 * Dummy register read function.  Used for register blocks
 * that certain asics don't have (all asics).
 */
// static void gsgpu_invalid_wreg(struct gsgpu_device *adev, uint32_t reg, uint32_t v)
// {
// 	DRM_ERROR("Invalid callback to write register 0x%04X with 0x%08X\n",
// 		  reg, v);
// 	BUG();
// }

/**
 * gsgpu_invalid_rreg64 - dummy 64 bit reg read function
 *
 * @adev: gsgpu_device pointer
 * @reg: offset of register
 *
 * Dummy register read function.  Used for register blocks
 * that certain asics don't have (all asics).
 * Returns the value in the register.
 */
// static uint64_t gsgpu_invalid_rreg64(struct gsgpu_device *adev, uint32_t reg)
// {
// 	DRM_ERROR("Invalid callback to read 64 bit register 0x%04X\n", reg);
// 	BUG();
// 	return 0;
// }

/**
 * gsgpu_invalid_wreg64 - dummy reg write function
 *
 * @adev: gsgpu_device pointer
 * @reg: offset of register
 * @v: value to write to the register
 *
 * Dummy register read function.  Used for register blocks
 * that certain asics don't have (all asics).
 */
// static void gsgpu_invalid_wreg64(struct gsgpu_device *adev, uint32_t reg, uint64_t v)
// {
// 	DRM_ERROR("Invalid callback to write 64 bit register 0x%04X with 0x%08llX\n",
// 		  reg, v);
// 	BUG();
// }

/**
 * gsgpu_block_invalid_rreg - dummy reg read function
 *
 * @adev: gsgpu_device pointer
 * @block: offset of instance
 * @reg: offset of register
 *
 * Dummy register read function.  Used for register blocks
 * that certain asics don't have (all asics).
 * Returns the value in the register.
 */
// static uint32_t gsgpu_block_invalid_rreg(struct gsgpu_device *adev,
// 					  uint32_t block, uint32_t reg)
// {
// 	DRM_ERROR("Invalid callback to read register 0x%04X in block 0x%04X\n",
// 		  reg, block);
// 	BUG();
// 	return 0;
// }

/**
 * gsgpu_block_invalid_wreg - dummy reg write function
 *
 * @adev: gsgpu_device pointer
 * @block: offset of instance
 * @reg: offset of register
 * @v: value to write to the register
 *
 * Dummy register read function.  Used for register blocks
 * that certain asics don't have (all asics).
 */
// static void gsgpu_block_invalid_wreg(struct gsgpu_device *adev,
// 				      uint32_t block,
// 				      uint32_t reg, uint32_t v)
// {
// 	DRM_ERROR("Invalid block callback to write register 0x%04X in block 0x%04X with 0x%08X\n",
// 		  reg, block, v);
// 	BUG();
// }

/**
 * gsgpu_device_asic_init - Wrapper for atom asic_init
 *
 * @adev: gsgpu_device pointer
 *
 * Does any asic specific work and then calls atom asic init.
 */
static int gsgpu_device_asic_init(struct gsgpu_device *adev)
{
	return 0; // gsgpu_atom_asic_init(adev->mode_info.atom_context);
}

/**
 * gsgpu_device_mem_scratch_init - allocate the VRAM scratch page
 *
 * @adev: gsgpu_device pointer
 *
 * Allocates a scratch page of VRAM for use by various things in the
 * driver.
 */
static int gsgpu_device_mem_scratch_init(struct gsgpu_device *adev)
{
	return gsgpu_bo_create_kernel(adev, GSGPU_GPU_PAGE_SIZE, PAGE_SIZE,
				       GSGPU_GEM_DOMAIN_VRAM |
				       GSGPU_GEM_DOMAIN_GTT,
				       &adev->mem_scratch.robj,
				       &adev->mem_scratch.gpu_addr,
				       (void **)&adev->mem_scratch.ptr);
}

/**
 * gsgpu_device_mem_scratch_fini - Free the VRAM scratch page
 *
 * @adev: gsgpu_device pointer
 *
 * Frees the VRAM scratch page.
 */
// static void gsgpu_device_mem_scratch_fini(struct gsgpu_device *adev)
// {
// 	gsgpu_bo_free_kernel(&adev->mem_scratch.robj, NULL, NULL);
// }

/**
 * gsgpu_device_program_register_sequence - program an array of registers.
 *
 * @adev: gsgpu_device pointer
 * @registers: pointer to the register array
 * @array_size: size of the register array
 *
 * Programs an array or registers with and and or masks.
 * This is a helper for setting golden registers.
 */
void gsgpu_device_program_register_sequence(struct gsgpu_device *adev,
					     const u32 *registers,
					     const u32 array_size)
{
	u32 tmp, reg, and_mask, or_mask;
	int i;

	if (array_size % 3)
		return;

	for (i = 0; i < array_size; i +=3) {
		reg = registers[i + 0];
		and_mask = registers[i + 1];
		or_mask = registers[i + 2];

		if (and_mask == 0xffffffff) {
			tmp = or_mask;
		} else {
			tmp = RREG32(reg);
			tmp &= ~and_mask;
			if (adev->family >= GSGPU_FAMILY_AI)
				tmp |= (or_mask & and_mask);
			else
				tmp |= or_mask;
		}
		WREG32(reg, tmp);
	}
}

/**
 * gsgpu_device_pci_config_reset - reset the GPU
 *
 * @adev: gsgpu_device pointer
 *
 * Resets the GPU using the pci config reset sequence.
 * Only applicable to asics prior to vega10.
 */
void gsgpu_device_pci_config_reset(struct gsgpu_device *adev)
{
	pci_write_config_dword(adev->pdev, 0x7c, GSGPU_ASIC_RESET_DATA);
}

/**
 * gsgpu_device_pci_reset - reset the GPU using generic PCI means
 *
 * @adev: gsgpu_device pointer
 *
 * Resets the GPU using generic pci reset interfaces (FLR, SBR, etc.).
 */
int gsgpu_device_pci_reset(struct gsgpu_device *adev)
{
	return pci_reset_function(adev->pdev);
}

/*
 * gsgpu_device_wb_*()
 * Writeback is the method by which the GPU updates special pages in memory
 * with the status of certain GPU events (fences, ring pointers,etc.).
 */

/**
 * gsgpu_device_wb_fini - Disable Writeback and free memory
 *
 * @adev: gsgpu_device pointer
 *
 * Disables Writeback and frees the Writeback memory (all asics).
 * Used at driver shutdown.
 */
// static void gsgpu_device_wb_fini(struct gsgpu_device *adev)
// {
// 	if (adev->wb.wb_obj) {
// 		gsgpu_bo_free_kernel(&adev->wb.wb_obj,
// 				      &adev->wb.gpu_addr,
// 				      (void **)&adev->wb.wb);
// 		adev->wb.wb_obj = NULL;
// 	}
// }

/**
 * gsgpu_device_wb_init - Init Writeback driver info and allocate memory
 *
 * @adev: gsgpu_device pointer
 *
 * Initializes writeback and allocates writeback memory (all asics).
 * Used at driver startup.
 * Returns 0 on success or an -error on failure.
 */
static int gsgpu_device_wb_init(struct gsgpu_device *adev)
{
	int r;

	if (adev->wb.wb_obj == NULL) {
		/* GSGPU_MAX_WB * sizeof(uint32_t) * 8 = GSGPU_MAX_WB 256bit slots */
		r = gsgpu_bo_create_kernel(adev, GSGPU_MAX_WB * sizeof(uint32_t) * 8,
					    PAGE_SIZE, GSGPU_GEM_DOMAIN_GTT,
					    &adev->wb.wb_obj, &adev->wb.gpu_addr,
					    (void **)&adev->wb.wb);
		if (r) {
			dev_warn(adev->dev, "(%d) create WB bo failed\n", r);
			return r;
		}

		adev->wb.num_wb = GSGPU_MAX_WB;
		memset(&adev->wb.used, 0, sizeof(adev->wb.used));

		/* clear wb memory */
		memset((char *)adev->wb.wb, 0, GSGPU_MAX_WB * sizeof(uint32_t) * 8);
	}

	return 0;
}

/**
 * gsgpu_device_wb_get - Allocate a wb entry
 *
 * @adev: gsgpu_device pointer
 * @wb: wb index
 *
 * Allocate a wb slot for use by the driver (all asics).
 * Returns 0 on success or -EINVAL on failure.
 */
int gsgpu_device_wb_get(struct gsgpu_device *adev, u32 *wb)
{
	unsigned long offset = find_first_zero_bit(adev->wb.used, adev->wb.num_wb);

	if (offset < adev->wb.num_wb) {
		__set_bit(offset, adev->wb.used);
		*wb = offset << 3; /* convert to dw offset */
		return 0;
	} else {
		return -EINVAL;
	}
}

/**
 * gsgpu_device_wb_free - Free a wb entry
 *
 * @adev: gsgpu_device pointer
 * @wb: wb index
 *
 * Free a wb slot allocated for use by the driver (all asics)
 */
void gsgpu_device_wb_free(struct gsgpu_device *adev, u32 wb)
{
	wb >>= 3;
	if (wb < adev->wb.num_wb)
		__clear_bit(wb, adev->wb.used);
}

/**
 * gsgpu_device_resize_fb_bar - try to resize FB BAR
 *
 * @adev: gsgpu_device pointer
 *
 * Try to resize FB BAR to make all VRAM CPU accessible. We try very hard not
 * to fail, but if any of the BARs is not accessible after the size we abort
 * driver loading by returning -ENODEV.
 */
int gsgpu_device_resize_fb_bar(struct gsgpu_device *adev)
{
	int rbar_size = pci_rebar_bytes_to_size(adev->gmc.real_vram_size);
	struct pci_bus *root;
	struct resource *res;
	unsigned i;
	u16 cmd;
	int r;

	/* skip if the bios has already enabled large BAR */
	if (adev->gmc.real_vram_size &&
	    (pci_resource_len(adev->pdev, 0) >= adev->gmc.real_vram_size))
		return 0;

	/* Check if the root BUS has 64bit memory resources */
	root = adev->pdev->bus;
	while (root->parent)
		root = root->parent;

	pci_bus_for_each_resource(root, res, i) {
		if (res && res->flags & (IORESOURCE_MEM | IORESOURCE_MEM_64) &&
		    res->start > 0x100000000ull)
			break;
	}

	/* Trying to resize is pointless without a root hub window above 4GB */
	if (!res)
		return 0;

	/* Limit the BAR size to what is available */
	rbar_size = min(fls(pci_rebar_get_possible_sizes(adev->pdev, 0)) - 1,
			rbar_size);

	/* Disable memory decoding while we change the BAR addresses and size */
	pci_read_config_word(adev->pdev, PCI_COMMAND, &cmd);
	pci_write_config_word(adev->pdev, PCI_COMMAND,
			      cmd & ~PCI_COMMAND_MEMORY);

	pci_release_resource(adev->pdev, 3);

	pci_release_resource(adev->pdev, 0);

	r = pci_resize_resource(adev->pdev, 0, rbar_size);
	if (r == -ENOSPC)
		DRM_INFO("Not enough PCI address space for a large BAR.");
	else if (r && r != -ENOTSUPP)
		DRM_ERROR("Problem resizing BAR0 (%d).", r);

	pci_assign_unassigned_bus_resources(adev->pdev->bus);

	/* When the fb BAR isn't available we have no chance of
	 * using the device.
	 */
	if ((pci_resource_flags(adev->pdev, 0) & IORESOURCE_UNSET))
		return -ENODEV;

	pci_write_config_word(adev->pdev, PCI_COMMAND, cmd);

	return 0;
}

/*
 * GPU helpers function.
 */
/**
 * gsgpu_device_need_post - check if the hw need post or not
 *
 * @adev: gsgpu_device pointer
 *
 * Check if the asic has been initialized (all asics) at driver startup
 * or post is needed if  hw reset is performed.
 * Returns true if need or false if not.
 */
bool gsgpu_device_need_post(struct gsgpu_device *adev)
{
	if (adev->has_hw_reset) {
		adev->has_hw_reset = false;
	}

	return true;
}

/* if we get transitioned to only one device, take VGA back */
/**
 * gsgpu_device_vga_set_decode - enable/disable vga decode
 *
 * @pdev: PCI device pointer
 * @state: enable/disable vga decode
 *
 * Enable/disable vga decode (all asics).
 * Returns VGA resource flags.
 */
static unsigned int gsgpu_device_vga_set_decode(struct pci_dev *pdev,
		bool state)
{
	struct gsgpu_device *adev = drm_to_adev(pci_get_drvdata(pdev));
	gsgpu_asic_set_vga_state(adev, state);
	if (state)
		return VGA_RSRC_LEGACY_IO | VGA_RSRC_LEGACY_MEM |
		       VGA_RSRC_NORMAL_IO | VGA_RSRC_NORMAL_MEM;
	else
		return VGA_RSRC_NORMAL_IO | VGA_RSRC_NORMAL_MEM;
}

/**
 * gsgpu_device_check_block_size - validate the vm block size
 *
 * @adev: gsgpu_device pointer
 *
 * Validates the vm block size specified via module parameter.
 * The vm block size defines number of bits in page table versus page directory,
 * a page is 4KB so we have 12 bits offset, minimum 9 bits in the
 * page table and the remaining bits are in the page directory.
 */
static void gsgpu_device_check_block_size(struct gsgpu_device *adev)
{
	/* defines number of bits in page table versus page directory,
	 * a page is 4KB so we have 12 bits offset, minimum 9 bits in the
	 * page table and the remaining bits are in the page directory */
	if (gsgpu_vm_block_size == -1)
		return;

	if (gsgpu_vm_block_size < GSGPU_PAGE_PTE_SHIFT) {
		dev_warn(adev->dev, "VM page table size (%d) too small\n",
			 gsgpu_vm_block_size);
		gsgpu_vm_block_size = -1;
	}
}

/**
 * gsgpu_device_check_vm_size - validate the vm size
 *
 * @adev: gsgpu_device pointer
 *
 * Validates the vm size in GB specified via module parameter.
 * The VM size is the size of the GPU virtual memory space in GB.
 */
static void gsgpu_device_check_vm_size(struct gsgpu_device *adev)
{
	/* no need to check the default value */
	if (gsgpu_vm_size == -1)
		return;

	if (gsgpu_vm_size < 1) {
		dev_warn(adev->dev, "VM size (%d) too small, min is 1GB\n",
			 gsgpu_vm_size);
		gsgpu_vm_size = -1;
	}
}

/**
 * gsgpu_device_check_arguments - validate module params
 *
 * @adev: gsgpu_device pointer
 *
 * Validates certain module parameters and updates
 * the associated values used by the driver (all asics).
 */
static int gsgpu_device_check_arguments(struct gsgpu_device *adev)
{
	if (gsgpu_sched_jobs < 4) {
		dev_warn(adev->dev, "sched jobs (%d) must be at least 4\n",
			 gsgpu_sched_jobs);
		gsgpu_sched_jobs = 4;
	} else if (!is_power_of_2(gsgpu_sched_jobs)){
		dev_warn(adev->dev, "sched jobs (%d) must be a power of 2\n",
			 gsgpu_sched_jobs);
		gsgpu_sched_jobs = roundup_pow_of_two(gsgpu_sched_jobs);
	}

	if (gsgpu_gart_size != -1 && gsgpu_gart_size < 32) {
		/* gart size must be greater or equal to 32M */
		dev_warn(adev->dev, "gart size (%d) too small\n",
			 gsgpu_gart_size);
		gsgpu_gart_size = -1;
	}

	if (gsgpu_gtt_size != -1 && gsgpu_gtt_size < 32) {
		/* gtt size must be greater or equal to 32M */
		dev_warn(adev->dev, "gtt size (%d) too small\n",
				 gsgpu_gtt_size);
		gsgpu_gtt_size = -1;
	}

	gsgpu_device_check_vm_size(adev);

	gsgpu_device_check_block_size(adev);

	return 0;
}

/**
 * gsgpu_switcheroo_set_state - set switcheroo state
 *
 * @pdev: pci dev pointer
 * @state: vga_switcheroo state
 *
 * Callback for the switcheroo driver.  Suspends or resumes
 * the asics before or after it is powered up using ACPI methods.
 */
static void gsgpu_switcheroo_set_state(struct pci_dev *pdev,
					enum vga_switcheroo_state state)
{
	struct drm_device *dev = pci_get_drvdata(pdev);
	int r;

	if (state == VGA_SWITCHEROO_OFF)
		return;

	if (state == VGA_SWITCHEROO_ON) {
		pr_info("switched on\n");
		/* don't suspend or resume card normally */
		dev->switch_power_state = DRM_SWITCH_POWER_CHANGING;

		pci_set_power_state(pdev, PCI_D0);
		gsgpu_device_load_pci_state(pdev);
		r = pci_enable_device(pdev);
		if (r)
			DRM_WARN("pci_enable_device failed (%d)\n", r);
		gsgpu_device_resume(dev, true);

		dev->switch_power_state = DRM_SWITCH_POWER_ON;
	} else {
		pr_info("switched off\n");
		dev->switch_power_state = DRM_SWITCH_POWER_CHANGING;
		gsgpu_device_suspend(dev, true);
		gsgpu_device_cache_pci_state(pdev);
		/* Shut down the device */
		pci_disable_device(pdev);
		pci_set_power_state(pdev, PCI_D3cold);
		dev->switch_power_state = DRM_SWITCH_POWER_OFF;
	}
}

/**
 * gsgpu_switcheroo_can_switch - see if switcheroo state can change
 *
 * @pdev: pci dev pointer
 *
 * Callback for the switcheroo driver.  Check of the switcheroo
 * state can be changed.
 * Returns true if the state can be changed, false if not.
 */
static bool gsgpu_switcheroo_can_switch(struct pci_dev *pdev)
{
	struct drm_device *dev = pci_get_drvdata(pdev);

	/*
	* FIXME: open_count is protected by drm_global_mutex but that would lead to
	* locking inversion with the driver load path. And the access here is
	* completely racy anyway. So don't bother with locking for now.
	*/
	return atomic_read(&dev->open_count) == 0;
}

static const struct vga_switcheroo_client_ops gsgpu_switcheroo_ops = {
	.set_gpu_state = gsgpu_switcheroo_set_state,
	.reprobe = NULL,
	.can_switch = gsgpu_switcheroo_can_switch,
};

/**
 * gsgpu_device_ip_wait_for_idle - wait for idle
 *
 * @adev: gsgpu_device pointer
 * @block_type: Type of hardware IP (SMU, GFX, UVD, etc.)
 *
 * Waits for the request hardware IP to be idle.
 * Returns 0 for success or a negative error code on failure.
 */
int gsgpu_device_ip_wait_for_idle(struct gsgpu_device *adev,
				   enum gsgpu_ip_block_type block_type)
{
	int i, r;

	for (i = 0; i < adev->num_ip_blocks; i++) {
		if (!adev->ip_blocks[i].status.valid)
			continue;
		if (adev->ip_blocks[i].version->type == block_type) {
			r = adev->ip_blocks[i].version->funcs->wait_for_idle((void *)adev);
			if (r)
				return r;
			break;
		}
	}
	return 0;

}

/**
 * gsgpu_device_ip_is_idle - is the hardware IP idle
 *
 * @adev: gsgpu_device pointer
 * @block_type: Type of hardware IP (SMU, GFX, UVD, etc.)
 *
 * Check if the hardware IP is idle or not.
 * Returns true if it the IP is idle, false if not.
 */
bool gsgpu_device_ip_is_idle(struct gsgpu_device *adev,
			      enum gsgpu_ip_block_type block_type)
{
	int i;

	for (i = 0; i < adev->num_ip_blocks; i++) {
		if (!adev->ip_blocks[i].status.valid)
			continue;
		if (adev->ip_blocks[i].version->type == block_type)
			return adev->ip_blocks[i].version->funcs->is_idle((void *)adev);
	}
	return true;

}

/**
 * gsgpu_device_ip_get_ip_block - get a hw IP pointer
 *
 * @adev: gsgpu_device pointer
 * @type: Type of hardware IP (SMU, GFX, UVD, etc.)
 *
 * Returns a pointer to the hardware IP block structure
 * if it exists for the asic, otherwise NULL.
 */
struct gsgpu_ip_block *
gsgpu_device_ip_get_ip_block(struct gsgpu_device *adev,
			      enum gsgpu_ip_block_type type)
{
	int i;

	for (i = 0; i < adev->num_ip_blocks; i++)
		if (adev->ip_blocks[i].version->type == type)
			return &adev->ip_blocks[i];

	return NULL;
}

/**
 * gsgpu_device_ip_block_version_cmp
 *
 * @adev: gsgpu_device pointer
 * @type: enum gsgpu_ip_block_type
 * @major: major version
 * @minor: minor version
 *
 * return 0 if equal or greater
 * return 1 if smaller or the ip_block doesn't exist
 */
int gsgpu_device_ip_block_version_cmp(struct gsgpu_device *adev,
				       enum gsgpu_ip_block_type type,
				       u32 major, u32 minor)
{
	struct gsgpu_ip_block *ip_block = gsgpu_device_ip_get_ip_block(adev, type);

	if (ip_block && ((ip_block->version->major > major) ||
			((ip_block->version->major == major) &&
			(ip_block->version->minor >= minor))))
		return 0;

	return 1;
}

/**
 * gsgpu_device_ip_block_add
 *
 * @adev: gsgpu_device pointer
 * @ip_block_version: pointer to the IP to add
 *
 * Adds the IP block driver information to the collection of IPs
 * on the asic.
 */
int gsgpu_device_ip_block_add(struct gsgpu_device *adev,
			       const struct gsgpu_ip_block_version *ip_block_version)
{
	if (!ip_block_version)
		return -EINVAL;

	DRM_DEBUG_DRIVER("add ip block number %d <%s>\n", adev->num_ip_blocks,
		  ip_block_version->funcs->name);

	adev->ip_blocks[adev->num_ip_blocks++].version = ip_block_version;

	return 0;
}

/**
 * gsgpu_device_ip_early_init - run early init for hardware IPs
 *
 * @adev: gsgpu_device pointer
 *
 * Early initialization pass for hardware IPs.  The hardware IPs that make
 * up each asic are discovered each IP's early_init callback is run.  This
 * is the first stage in initializing the asic.
 * Returns 0 on success, negative error code on failure.
 */
static int gsgpu_device_ip_early_init(struct gsgpu_device *adev)
{
	// struct drm_device *dev = adev_to_drm(adev);
	// struct pci_dev *parent;
	int i, r;
	// bool total;
	
	adev->family = GSGPU_FAMILY_VI;
	r = gsgpu_set_ip_blocks(adev);
	if (r)
		return r;

	for (i = 0; i < adev->num_ip_blocks; i++) {
		if (adev->ip_blocks[i].version->funcs->early_init) {
			r = adev->ip_blocks[i].version->funcs->early_init((void *)adev);
			if (r == -ENOENT) {
				adev->ip_blocks[i].status.valid = false;
			} else if (r) {
				DRM_ERROR("early_init of IP block <%s> failed %d\n",
					  adev->ip_blocks[i].version->funcs->name, r);
				return r;
			} else {
				adev->ip_blocks[i].status.valid = true;
			}
		} else {
			adev->ip_blocks[i].status.valid = true;
		}
	}

	return 0;
}

static int gsgpu_device_ip_hw_init_phase1(struct gsgpu_device *adev)
{
	int i, r;

	for (i = 0; i < adev->num_ip_blocks; i++) {
		if (!adev->ip_blocks[i].status.sw)
			continue;
		if (adev->ip_blocks[i].status.hw)
			continue;
		if (adev->ip_blocks[i].version->type == GSGPU_IP_BLOCK_TYPE_COMMON ||
		    adev->ip_blocks[i].version->type == GSGPU_IP_BLOCK_TYPE_IH) {
			r = adev->ip_blocks[i].version->funcs->hw_init(adev);
			if (r) {
				DRM_ERROR("hw_init of IP block <%s> failed %d\n",
					  adev->ip_blocks[i].version->funcs->name, r);
				return r;
			}
			adev->ip_blocks[i].status.hw = true;
		}
	}

	return 0;
}

static int gsgpu_device_ip_hw_init_phase2(struct gsgpu_device *adev)
{
	int i, r;

	for (i = 0; i < adev->num_ip_blocks; i++) {
		if (!adev->ip_blocks[i].status.sw)
			continue;
		if (adev->ip_blocks[i].status.hw)
			continue;
		r = adev->ip_blocks[i].version->funcs->hw_init(adev);
		if (r) {
			DRM_ERROR("hw_init of IP block <%s> failed %d\n",
				  adev->ip_blocks[i].version->funcs->name, r);
			return r;
		}
		adev->ip_blocks[i].status.hw = true;
	}

	return 0;
}

static int gsgpu_device_init_schedulers(struct gsgpu_device *adev)
{
	long timeout;
	int r, i;

	for (i = 0; i < GSGPU_MAX_RINGS; ++i) {
		struct gsgpu_ring *ring = adev->rings[i];

		/* No need to setup the GPU scheduler for rings that don't need it */
		if (!ring || ring->no_scheduler)
			continue;

		switch (ring->funcs->type) {
		case GSGPU_RING_TYPE_GFX:
			timeout = adev->gfx_timeout;
			break;
		case GSGPU_RING_TYPE_XDMA:
			timeout = adev->xdma_timeout;
			break;
		default:
			timeout = adev->video_timeout;
			break;
		}

		r = drm_sched_init(&ring->sched, &gsgpu_sched_ops,
				   ring->num_hw_submission, gsgpu_job_hang_limit,
				   timeout, adev->reset_domain->wq,
				   ring->sched_score, ring->name,
				   adev->dev);
		if (r) {
			DRM_ERROR("Failed to create scheduler on ring %s.\n",
				  ring->name);
			return r;
		}
	}

	return 0;
}


/**
 * gsgpu_device_ip_init - run init for hardware IPs
 *
 * @adev: gsgpu_device pointer
 *
 * Main initialization pass for hardware IPs.  The list of all the hardware
 * IPs that make up the asic is walked and the sw_init and hw_init callbacks
 * are run.  sw_init initializes the software state associated with each IP
 * and hw_init initializes the hardware associated with each IP.
 * Returns 0 on success, negative error code on failure.
 */
static int gsgpu_device_ip_init(struct gsgpu_device *adev)
{
	int i, r;

	for (i = 0; i < adev->num_ip_blocks; i++) {
		if (!adev->ip_blocks[i].status.valid)
			continue;
		r = adev->ip_blocks[i].version->funcs->sw_init((void *)adev);
		if (r) {
			DRM_ERROR("sw_init of IP block <%s> failed %d\n",
				  adev->ip_blocks[i].version->funcs->name, r);
			goto init_failed;
		}
		adev->ip_blocks[i].status.sw = true;

		if (adev->ip_blocks[i].version->type == GSGPU_IP_BLOCK_TYPE_COMMON) {
			/* need to do common hw init early so everything is set up for gmc */
			r = adev->ip_blocks[i].version->funcs->hw_init((void *)adev);
			if (r) {
				DRM_ERROR("hw_init %d failed %d\n", i, r);
				goto init_failed;
			}
			adev->ip_blocks[i].status.hw = true;
		} else if (adev->ip_blocks[i].version->type == GSGPU_IP_BLOCK_TYPE_GMC) {
			
			r = gsgpu_device_mem_scratch_init(adev);
			if (r) {
				DRM_ERROR("gsgpu_mem_scratch_init failed %d\n", r);
				goto init_failed;
			}
			r = adev->ip_blocks[i].version->funcs->hw_init((void *)adev);
			if (r) {
				DRM_ERROR("hw_init %d failed %d\n", i, r);
				goto init_failed;
			}
			r = gsgpu_device_wb_init(adev);
			if (r) {
				DRM_ERROR("gsgpu_device_wb_init failed %d\n", r);
				goto init_failed;
			}
			adev->ip_blocks[i].status.hw = true;
		}
	}

	r = gsgpu_ib_pool_init(adev);
	if (r) {
		dev_err(adev->dev, "IB initialization failed (%d).\n", r);
		goto init_failed;
	}

	r = gsgpu_device_ip_hw_init_phase1(adev);
	if (r)
		goto init_failed;

	r = gsgpu_device_ip_hw_init_phase2(adev);
	if (r)
		goto init_failed;

	r = gsgpu_device_init_schedulers(adev);
	if (r)
		goto init_failed;

init_failed:

	return r;
}

/**
 * gsgpu_device_fill_reset_magic - writes reset magic to gart pointer
 *
 * @adev: gsgpu_device pointer
 *
 * Writes a reset magic value to the gart pointer in VRAM.  The driver calls
 * this function before a GPU reset.  If the value is retained after a
 * GPU reset, VRAM has not been lost.  Some GPU resets may destry VRAM contents.
 */
static void gsgpu_device_fill_reset_magic(struct gsgpu_device *adev)
{
	memcpy(adev->reset_magic, adev->gart.ptr, GSGPU_RESET_MAGIC_NUM);
}

/**
 * gsgpu_device_check_vram_lost - check if vram is valid
 *
 * @adev: gsgpu_device pointer
 *
 * Checks the reset magic value written to the gart pointer in VRAM.
 * The driver calls this after a GPU reset to see if the contents of
 * VRAM is lost or now.
 * returns true if vram is lost, false if not.
 */
static bool gsgpu_device_check_vram_lost(struct gsgpu_device *adev)
{
	if (memcmp(adev->gart.ptr, adev->reset_magic,
			GSGPU_RESET_MAGIC_NUM))
		return true;

	if (!gsgpu_in_reset(adev))
		return false;
	return false;
}

/**
 * gsgpu_device_ip_late_init - run late init for hardware IPs
 *
 * @adev: gsgpu_device pointer
 *
 * Late initialization pass for hardware IPs.  The list of all the hardware
 * IPs that make up the asic is walked and the late_init callbacks are run.
 * late_init covers any special initialization that an IP requires
 * after all of the have been initialized or something that needs to happen
 * late in the init process.
 * Returns 0 on success, negative error code on failure.
 */
static int gsgpu_device_ip_late_init(struct gsgpu_device *adev)
{
	// struct gsgpu_gpu_instance *gpu_instance;
	int i = 0, r;

	for (i = 0; i < adev->num_ip_blocks; i++) {
		if (!adev->ip_blocks[i].status.hw)
			continue;
		if (adev->ip_blocks[i].version->funcs->late_init) {
			r = adev->ip_blocks[i].version->funcs->late_init((void *)adev);
			if (r) {
				DRM_ERROR("late_init of IP block <%s> failed %d\n",
					  adev->ip_blocks[i].version->funcs->name, r);
				return r;
			}
		}
		adev->ip_blocks[i].status.late_initialized = true;
	}

	gsgpu_device_fill_reset_magic(adev);

	return 0;
}

static int gsgpu_device_ip_fini_early(struct gsgpu_device *adev)
{
	int i, r;

	for (i = 0; i < adev->num_ip_blocks; i++) {
		if (!adev->ip_blocks[i].version->funcs->early_fini)
			continue;

		r = adev->ip_blocks[i].version->funcs->early_fini((void *)adev);
		if (r) {
			DRM_DEBUG("early_fini of IP block <%s> failed %d\n",
				  adev->ip_blocks[i].version->funcs->name, r);
		}
	}

	for (i = adev->num_ip_blocks - 1; i >= 0; i--) {
		if (!adev->ip_blocks[i].status.hw)
			continue;

		r = adev->ip_blocks[i].version->funcs->hw_fini((void *)adev);
		/* XXX handle errors */
		if (r) {
			DRM_DEBUG("hw_fini of IP block <%s> failed %d\n",
				  adev->ip_blocks[i].version->funcs->name, r);
		}

		adev->ip_blocks[i].status.hw = false;
	}

	return 0;
}

/**
 * gsgpu_device_ip_fini - run fini for hardware IPs
 *
 * @adev: gsgpu_device pointer
 *
 * Main teardown pass for hardware IPs.  The list of all the hardware
 * IPs that make up the asic is walked and the hw_fini and sw_fini callbacks
 * are run.  hw_fini tears down the hardware associated with each IP
 * and sw_fini tears down any software state associated with each IP.
 * Returns 0 on success, negative error code on failure.
 */
static int gsgpu_device_ip_fini(struct gsgpu_device *adev)
{
	int i, r;

	for (i = adev->num_ip_blocks - 1; i >= 0; i--) {
		if (!adev->ip_blocks[i].status.sw)
			continue;

		r = adev->ip_blocks[i].version->funcs->sw_fini((void *)adev);
		/* XXX handle errors */
		if (r) {
			DRM_DEBUG("sw_fini of IP block <%s> failed %d\n",
				  adev->ip_blocks[i].version->funcs->name, r);
		}
		adev->ip_blocks[i].status.sw = false;
		adev->ip_blocks[i].status.valid = false;
	}

	for (i = adev->num_ip_blocks - 1; i >= 0; i--) {
		if (!adev->ip_blocks[i].status.late_initialized)
			continue;
		if (adev->ip_blocks[i].version->funcs->late_fini)
			adev->ip_blocks[i].version->funcs->late_fini((void *)adev);
		adev->ip_blocks[i].status.late_initialized = false;
	}

	return 0;
}

/**
 * gsgpu_device_delayed_init_work_handler - work handler for IB tests
 *
 * @work: work_struct.
 */
static void gsgpu_device_delayed_init_work_handler(struct work_struct *work)
{
	struct gsgpu_device *adev =
		container_of(work, struct gsgpu_device, delayed_init_work.work);
	int r;

	r = gsgpu_ib_ring_tests(adev);
	if (r)
		DRM_ERROR("ib ring test failed (%d).\n", r);
}

static void gsgpu_device_delay_enable_gfx_off(struct work_struct *work)
{
	struct gsgpu_device *adev =
		container_of(work, struct gsgpu_device, gfx.gfx_off_delay_work.work);

	WARN_ON_ONCE(adev->gfx.gfx_off_state);
	WARN_ON_ONCE(adev->gfx.gfx_off_req_count);
}

/**
 * gsgpu_device_ip_suspend_phase1 - run suspend for hardware IPs (phase 1)
 *
 * @adev: gsgpu_device pointer
 *
 * Main suspend function for hardware IPs.  The list of all the hardware
 * IPs that make up the asic is walked, clockgating is disabled and the
 * suspend callbacks are run.  suspend puts the hardware and software state
 * in each IP into a state suitable for suspend.
 * Returns 0 on success, negative error code on failure.
 */
static int gsgpu_device_ip_suspend_phase1(struct gsgpu_device *adev)
{
	int i, r;

	for (i = adev->num_ip_blocks - 1; i >= 0; i--) {
		if (!adev->ip_blocks[i].status.valid)
			continue;

		/* displays are handled separately */
		if (adev->ip_blocks[i].version->type != GSGPU_IP_BLOCK_TYPE_DCE)
			continue;

		/* XXX handle errors */
		r = adev->ip_blocks[i].version->funcs->suspend(adev);
		/* XXX handle errors */
		if (r) {
			DRM_ERROR("suspend of IP block <%s> failed %d\n",
				  adev->ip_blocks[i].version->funcs->name, r);
			return r;
		}

		adev->ip_blocks[i].status.hw = false;
	}

	return 0;
}

/**
 * gsgpu_device_ip_suspend_phase2 - run suspend for hardware IPs (phase 2)
 *
 * @adev: gsgpu_device pointer
 *
 * Main suspend function for hardware IPs.  The list of all the hardware
 * IPs that make up the asic is walked, clockgating is disabled and the
 * suspend callbacks are run.  suspend puts the hardware and software state
 * in each IP into a state suitable for suspend.
 * Returns 0 on success, negative error code on failure.
 */
static int gsgpu_device_ip_suspend_phase2(struct gsgpu_device *adev)
{
	int i, r;

	for (i = adev->num_ip_blocks - 1; i >= 0; i--) {
		if (!adev->ip_blocks[i].status.valid)
			continue;
		/* displays are handled in phase1 */
		if (adev->ip_blocks[i].version->type == GSGPU_IP_BLOCK_TYPE_DCE)
			continue;

		/* XXX handle errors */
		r = adev->ip_blocks[i].version->funcs->suspend(adev);
		/* XXX handle errors */
		if (r) {
			DRM_ERROR("suspend of IP block <%s> failed %d\n",
				  adev->ip_blocks[i].version->funcs->name, r);
		}
		adev->ip_blocks[i].status.hw = false;
	}

	return 0;
}

/**
 * gsgpu_device_ip_suspend - run suspend for hardware IPs
 *
 * @adev: gsgpu_device pointer
 *
 * Main suspend function for hardware IPs.  The list of all the hardware
 * IPs that make up the asic is walked, clockgating is disabled and the
 * suspend callbacks are run.  suspend puts the hardware and software state
 * in each IP into a state suitable for suspend.
 * Returns 0 on success, negative error code on failure.
 */
int gsgpu_device_ip_suspend(struct gsgpu_device *adev)
{
	int r;

	r = gsgpu_device_ip_suspend_phase1(adev);
	if (r)
		return r;
	r = gsgpu_device_ip_suspend_phase2(adev);

	return r;
}

/**
 * gsgpu_device_ip_resume_phase1 - run resume for hardware IPs
 *
 * @adev: gsgpu_device pointer
 *
 * First resume function for hardware IPs.  The list of all the hardware
 * IPs that make up the asic is walked and the resume callbacks are run for
 * COMMON, GMC, and IH.  resume puts the hardware into a functional state
 * after a suspend and updates the software state as necessary.  This
 * function is also used for restoring the GPU after a GPU reset.
 * Returns 0 on success, negative error code on failure.
 */
static int gsgpu_device_ip_resume_phase1(struct gsgpu_device *adev)
{
	int i, r;

	for (i = 0; i < adev->num_ip_blocks; i++) {
		if (!adev->ip_blocks[i].status.valid || adev->ip_blocks[i].status.hw)
			continue;
		if (adev->ip_blocks[i].version->type == GSGPU_IP_BLOCK_TYPE_COMMON ||
		    adev->ip_blocks[i].version->type == GSGPU_IP_BLOCK_TYPE_GMC ||
		    adev->ip_blocks[i].version->type == GSGPU_IP_BLOCK_TYPE_IH) {

			r = adev->ip_blocks[i].version->funcs->resume(adev);
			if (r) {
				DRM_ERROR("resume of IP block <%s> failed %d\n",
					  adev->ip_blocks[i].version->funcs->name, r);
				return r;
			}
			adev->ip_blocks[i].status.hw = true;
		}
	}

	return 0;
}

/**
 * gsgpu_device_ip_resume_phase2 - run resume for hardware IPs
 *
 * @adev: gsgpu_device pointer
 *
 * First resume function for hardware IPs.  The list of all the hardware
 * IPs that make up the asic is walked and the resume callbacks are run for
 * all blocks except COMMON, GMC, and IH.  resume puts the hardware into a
 * functional state after a suspend and updates the software state as
 * necessary.  This function is also used for restoring the GPU after a GPU
 * reset.
 * Returns 0 on success, negative error code on failure.
 */
static int gsgpu_device_ip_resume_phase2(struct gsgpu_device *adev)
{
	int i, r;

	for (i = 0; i < adev->num_ip_blocks; i++) {
		if (!adev->ip_blocks[i].status.valid || adev->ip_blocks[i].status.hw)
			continue;
		if (adev->ip_blocks[i].version->type == GSGPU_IP_BLOCK_TYPE_COMMON ||
		    adev->ip_blocks[i].version->type == GSGPU_IP_BLOCK_TYPE_GMC ||
		    adev->ip_blocks[i].version->type == GSGPU_IP_BLOCK_TYPE_IH)
			continue;
		r = adev->ip_blocks[i].version->funcs->resume(adev);
		if (r) {
			DRM_ERROR("resume of IP block <%s> failed %d\n",
				  adev->ip_blocks[i].version->funcs->name, r);
			return r;
		}
		adev->ip_blocks[i].status.hw = true;
	}

	return 0;
}

/**
 * gsgpu_device_ip_resume - run resume for hardware IPs
 *
 * @adev: gsgpu_device pointer
 *
 * Main resume function for hardware IPs.  The hardware IPs
 * are split into two resume functions because they are
 * are also used in in recovering from a GPU reset and some additional
 * steps need to be take between them.  In this case (S3/S4) they are
 * run sequentially.
 * Returns 0 on success, negative error code on failure.
 */
static int gsgpu_device_ip_resume(struct gsgpu_device *adev)
{
	int r;

	r = gsgpu_device_ip_resume_phase1(adev);
	if (r)
		return r;

	r = gsgpu_device_ip_resume_phase2(adev);

	return r;
}

static int gsgpu_device_get_job_timeout_settings(struct gsgpu_device *adev)
{
	char *input = gsgpu_lockup_timeout;
	char *timeout_setting = NULL;
	int index = 0;
	long timeout;
	int ret = 0;

	/*
	 * By default timeout for non compute jobs is 10000
	 * and 60000 for compute jobs.
	 * In SR-IOV or passthrough mode, timeout for compute
	 * jobs are 60000 by default.
	 */
	adev->gfx_timeout = msecs_to_jiffies(10000);
	adev->xdma_timeout = adev->video_timeout = adev->gfx_timeout;

	if (strnlen(input, GSGPU_MAX_TIMEOUT_PARAM_LENGTH)) {
		while ((timeout_setting = strsep(&input, ",")) &&
				strnlen(timeout_setting, GSGPU_MAX_TIMEOUT_PARAM_LENGTH)) {
			ret = kstrtol(timeout_setting, 0, &timeout);
			if (ret)
				return ret;

			if (timeout == 0) {
				index++;
				continue;
			} else if (timeout < 0) {
				timeout = MAX_SCHEDULE_TIMEOUT;
				dev_warn(adev->dev, "lockup timeout disabled");
				add_taint(TAINT_SOFTLOCKUP, LOCKDEP_STILL_OK);
			} else {
				timeout = msecs_to_jiffies(timeout);
			}

			switch (index++) {
			case GSGPU_RING_TYPE_GFX:
				adev->gfx_timeout = timeout;
				break;
			case GSGPU_RING_TYPE_XDMA:
				adev->xdma_timeout = timeout;
				break;
			case 2:
				adev->video_timeout = timeout;
				break;
			default:
				break;
			}
		}
		/*
		 * There is only one value specified and
		 * it should apply to all non-compute jobs.
		 */
		if (index == 1) {
			adev->xdma_timeout = adev->video_timeout = adev->gfx_timeout;
		}
	}

	return ret;
}

/**
 * gsgpu_device_check_iommu_direct_map - check if RAM direct mapped to GPU
 *
 * @adev: gsgpu_device pointer
 *
 * RAM direct mapped to GPU if IOMMU is not enabled or is pass through mode
 */
static void gsgpu_device_check_iommu_direct_map(struct gsgpu_device *adev)
{
	struct iommu_domain *domain;

	domain = iommu_get_domain_for_dev(adev->dev);
	if (!domain || domain->type == IOMMU_DOMAIN_IDENTITY)
		adev->ram_is_direct_mapped = true;
}

static const struct attribute *gsgpu_dev_attributes[] = {
	&dev_attr_product_name.attr,
	&dev_attr_product_number.attr,
	&dev_attr_serial_number.attr,
	&dev_attr_pcie_replay_count.attr,
	NULL
};

extern struct pci_dev *loongson_dc_pdev;
extern void xdma_ring_test_xdma_loop(struct gsgpu_ring *ring, long timeout);

/**
 * gsgpu_device_init - initialize the driver
 *
 * @adev: gsgpu_device pointer
 * @flags: driver flags
 *
 * Initializes the driver info and hw (all asics).
 * Returns 0 for success or an error on failure.
 * Called at driver startup.
 */
int gsgpu_device_init(struct gsgpu_device *adev,
		       uint32_t flags)
{
	// struct drm_device *ddev = adev_to_drm(adev);
	struct pci_dev *pdev = adev->pdev;
	int r; // , i;
	// bool px = false;
	u32 max_MBps;

	adev->shutdown = false;
	adev->flags = flags;
	adev->loongson_dc = loongson_dc_pdev;

	adev->family_type = flags & GSGPU_ASIC_MASK;

	adev->usec_timeout = GSGPU_MAX_USEC_TIMEOUT;

	adev->gmc.gart_size = 512 * 1024 * 1024;
	adev->accel_working = false;
	adev->num_rings = 0;
	RCU_INIT_POINTER(adev->gang_submit, dma_fence_get_stub());
	adev->mman.buffer_funcs = NULL;
	adev->mman.buffer_funcs_ring = NULL;
	adev->vm_manager.vm_pte_funcs = NULL;
	adev->vm_manager.vm_pte_num_scheds = 0;
	adev->gmc.gmc_funcs = NULL;
	adev->harvest_ip_mask = 0x0;
	adev->fence_context = dma_fence_context_alloc(GSGPU_MAX_RINGS);

	DRM_INFO("initializing kernel modesetting (%s 0x%04X:0x%04X 0x%04X:0x%04X 0x%02X).\n",
		 gsgpu_family_name[adev->family_type], pdev->vendor, pdev->device,
		 pdev->subsystem_vendor, pdev->subsystem_device, pdev->revision);

	/* mutex initialization are all done here so we
	 * can recall function without having locking issues */
	mutex_init(&adev->gfx.gpu_clock_mutex);
	mutex_init(&adev->srbm_mutex);
	mutex_init(&adev->gfx.gfx_off_mutex);
	mutex_init(&adev->grbm_idx_mutex);
	mutex_init(&adev->mn_lock);
	hash_init(adev->mn_hash);
	mutex_init(&adev->notifier_lock);
	mutex_init(&adev->benchmark_mutex);

	r = gsgpu_device_check_arguments(adev);
	if (r)
		return r;

	spin_lock_init(&adev->dc_mmio_lock);
	spin_lock_init(&adev->mmio_idx_lock);
	spin_lock_init(&adev->pcie_idx_lock);
	spin_lock_init(&adev->se_cac_idx_lock);
	spin_lock_init(&adev->mm_stats.lock);

	INIT_LIST_HEAD(&adev->shadow_list);
	mutex_init(&adev->shadow_list_lock);

	INIT_LIST_HEAD(&adev->reset_list);

	INIT_LIST_HEAD(&adev->ras_list);

	INIT_DELAYED_WORK(&adev->delayed_init_work,
			  gsgpu_device_delayed_init_work_handler);
	INIT_DELAYED_WORK(&adev->gfx.gfx_off_delay_work,
			  gsgpu_device_delay_enable_gfx_off);

	adev->gfx.gfx_off_req_count = 1;
	adev->gfx.gfx_off_residency = 0;
	adev->gfx.gfx_off_entrycount = 0;
	
	/* pci get dc revision */
	pci_read_config_byte(adev->loongson_dc, 0x8, &adev->chip_revision);

	atomic_set(&adev->throttling_logging_enabled, 1);
	/*
	 * If throttling continues, logging will be performed every minute
	 * to avoid log flooding. "-1" is subtracted since the thermal
	 * throttling interrupt comes every second. Thus, the total logging
	 * interval is 59 seconds(retelimited printk interval) + 1(waiting
	 * for throttling interrupt) = 60 seconds.
	 */
	ratelimit_state_init(&adev->throttling_logging_rs, (60 - 1) * HZ, 1);
	ratelimit_set_flags(&adev->throttling_logging_rs, RATELIMIT_MSG_ON_RELEASE);

	adev->rmmio_base = pci_resource_start(adev->pdev, 0);
	adev->rmmio_size = pci_resource_len(adev->pdev, 0);

	adev->rmmio = ioremap(adev->rmmio_base, adev->rmmio_size);
	if (adev->rmmio == NULL) {
		return -ENOMEM;
	}
	DRM_INFO("register mmio base: 0x%08X\n", (uint32_t)adev->rmmio_base);
	DRM_INFO("register mmio size: %u\n", (unsigned)adev->rmmio_size);

	/* loongson dc */
	adev->loongson_dc_rmmio_base = pci_resource_start(adev->loongson_dc, 0);
	adev->loongson_dc_rmmio_size = pci_resource_len(adev->loongson_dc, 0);

	adev->loongson_dc_rmmio = pci_iomap(adev->loongson_dc, 0, adev->loongson_dc_rmmio_size);
	if (adev->loongson_dc_rmmio == NULL) {
		return -ENOMEM;
	}

	DRM_INFO("loongson dc register mmio base: 0x%08X\n", (uint32_t)adev->loongson_dc_rmmio_base);
	DRM_INFO("loongson dc register mmio size: %u\n", (unsigned)adev->loongson_dc_rmmio_size);

	gsgpu_device_get_pcie_info(adev);

	/*
	 * Reset domain needs to be present early, before XGMI hive discovered
	 * (if any) and intitialized to use reset sem and in_gpu reset flag
	 * early on during init and before calling to RREG32.
	 */
	adev->reset_domain = gsgpu_reset_create_reset_domain(SINGLE_DEVICE, "gsgpu-reset-dev");
	if (!adev->reset_domain)
		return -ENOMEM;

	r = gsgpu_device_get_job_timeout_settings(adev);
	if (r) {
		dev_err(adev->dev, "invalid lockup_timeout parameter syntax\n");
		return r;
	}

	/* early init functions */
	r = gsgpu_device_ip_early_init(adev);
	if (r)
		return r;

	/* Get rid of things like offb */
	r = drm_aperture_remove_conflicting_pci_framebuffers(adev->pdev, &gsgpu_kms_driver);
	if (r)
		return r;

	gsgpu_gmc_noretry_set(adev);

		adev->have_atomics_support =
			!pci_enable_atomic_ops_to_root(adev->pdev,
					  PCI_EXP_DEVCAP2_ATOMIC_COMP32 |
					  PCI_EXP_DEVCAP2_ATOMIC_COMP64);

	if (!adev->have_atomics_support)
		dev_info(adev->dev, "PCIE atomic ops is not supported\n");

	gsgpu_reset_init(adev);

	pci_enable_pcie_error_reporting(adev->pdev);

// fence_driver_init:
	/* Fence driver */
	r = gsgpu_fence_driver_sw_init(adev);
	if (r) {
		dev_err(adev->dev, "gsgpu_fence_driver_sw_init failed\n");
		goto failed;
	}

	/* init the mode config */
	drm_mode_config_init(adev_to_drm(adev));

	r = gsgpu_cp_init(adev);
	if (r) {
		/* failed in exclusive mode due to timeout */
		dev_err(adev->dev, "gsgpu_cp_init failed\n");
		goto failed;
	}

	r = gsgpu_device_ip_init(adev);
	if (r) {
		dev_err(adev->dev, "gsgpu_device_ip_init failed\n");
		goto release_ras_con;
	}

	gsgpu_fence_driver_hw_init(adev);

	dev_info(adev->dev,
		"SE %d, SH per SE %d, CU per SH %d, active_cu_number %d\n",
			adev->gfx.config.max_shader_engines,
			adev->gfx.config.max_sh_per_se,
			adev->gfx.config.max_cu_per_sh,
			adev->gfx.cu_info.number);

	adev->accel_working = true;

	/* Initialize the buffer migration limit. */
	if (gsgpu_moverate >= 0)
		max_MBps = gsgpu_moverate;
	else
		max_MBps = 8; /* Allow 8 MB/s. */
	/* Get a log2 for easy divisions. */
	adev->mm_stats.log2_max_MBps = ilog2(max(1u, max_MBps));

	r = gsgpu_pm_sysfs_init(adev);
	if (r) {
		adev->pm_sysfs_en = false;
		DRM_ERROR("registering pm debugfs failed (%d).\n", r);
	} else
		adev->pm_sysfs_en = true;

	/*
	 * Register gpu instance before gsgpu_device_enable_mgpu_fan_boost.
	 * Otherwise the mgpu fan boost feature will be skipped due to the
	 * gpu instance is counted less.
	 */
	gsgpu_register_gpu_instance(adev);

	/* enable clockgating, etc. after ib tests, etc. since some blocks require
	 * explicit gating rather than handling it automatically.
	 */
	{
		r = gsgpu_device_ip_late_init(adev);
		if (r) {
			dev_err(adev->dev, "gsgpu_device_ip_late_init failed\n");
			goto release_ras_con;
		}
		/* must succeed. */
		queue_delayed_work(system_wq, &adev->delayed_init_work,
				   msecs_to_jiffies(GSGPU_RESUME_MS));
	}

	r = sysfs_create_files(&adev->dev->kobj, gsgpu_dev_attributes);
	if (r)
		dev_err(adev->dev, "Could not create gsgpu device attr\n");

	/* Have stored pci confspace at hand for restore in sudden PCI error */
	if (gsgpu_device_cache_pci_state(adev->pdev))
		pci_restore_state(pdev);

	/* if we have > 1 VGA cards, then disable the gsgpu VGA resources */
	/* this will fail for cards that aren't VGA class devices, just
	 * ignore it */
	if ((adev->pdev->class >> 8) == PCI_CLASS_DISPLAY_VGA)
		vga_client_register(adev->pdev, gsgpu_device_vga_set_decode);

	gsgpu_device_check_iommu_direct_map(adev);

	xdma_ring_test_xdma_loop(&adev->xdma.instance[0].ring, msecs_to_jiffies(5000));

	return 0;

release_ras_con:

failed:

	return r;
}

static void gsgpu_device_unmap_mmio(struct gsgpu_device *adev)
{
	/* Clear all CPU mappings pointing to this device */
	unmap_mapping_range(adev->ddev.anon_inode->i_mapping, 0, 0, 1);

	iounmap(adev->rmmio);
	adev->rmmio = NULL;
	if (adev->mman.aper_base_kaddr)
		iounmap(adev->mman.aper_base_kaddr);
	adev->mman.aper_base_kaddr = NULL;

	/* Memory manager related */
	{
		arch_phys_wc_del(adev->gmc.vram_mtrr);
		arch_io_free_memtype_wc(adev->gmc.aper_base, adev->gmc.aper_size);
	}
}

/**
 * gsgpu_device_fini_hw - tear down the driver
 *
 * @adev: gsgpu_device pointer
 *
 * Tear down the driver info (all asics).
 * Called at driver shutdown.
 */
void gsgpu_device_fini_hw(struct gsgpu_device *adev)
{
	dev_info(adev->dev, "gsgpu: finishing device.\n");
	flush_delayed_work(&adev->delayed_init_work);
	adev->shutdown = true;

	/* disable all interrupts */
	gsgpu_irq_disable_all(adev);
	if (adev->mode_info.mode_config_initialized){
		drm_atomic_helper_shutdown(adev_to_drm(adev));
	}
	gsgpu_fence_driver_hw_fini(adev);

	if (adev->mman.initialized)
		drain_workqueue(adev->mman.bdev.wq);

	sysfs_remove_files(&adev->dev->kobj, gsgpu_dev_attributes);

	gsgpu_device_ip_fini_early(adev);

	gsgpu_irq_fini_hw(adev);

	if (adev->mman.initialized)
		ttm_device_clear_dma_mappings(&adev->mman.bdev);

	gsgpu_gart_dummy_page_fini(adev);

	if (drm_dev_is_unplugged(adev_to_drm(adev)))
		gsgpu_device_unmap_mmio(adev);

}

void gsgpu_device_fini_sw(struct gsgpu_device *adev)
{
	int idx;

	gsgpu_fence_driver_sw_fini(adev);
	gsgpu_device_ip_fini(adev);
	adev->accel_working = false;
	dma_fence_put(rcu_dereference_protected(adev->gang_submit, true));

	gsgpu_reset_fini(adev);

	gsgpu_cp_fini(adev);

	kfree(adev->bios);
	adev->bios = NULL;
	
	if ((adev->pdev->class >> 8) == PCI_CLASS_DISPLAY_VGA)
		vga_client_unregister(adev->pdev);

	if (drm_dev_enter(adev_to_drm(adev), &idx)) {

		iounmap(adev->rmmio);
		adev->rmmio = NULL;
		drm_dev_exit(idx);
	}

	gsgpu_reset_put_reset_domain(adev->reset_domain);
	adev->reset_domain = NULL;

	kfree(adev->pci_state);

}

/**
 * gsgpu_device_evict_resources - evict device resources
 * @adev: gsgpu device object
 *
 * Evicts all ttm device resources(vram BOs, gart table) from the lru list
 * of the vram memory type. Mainly used for evicting device resources
 * at suspend time.
 *
 */
static int gsgpu_device_evict_resources(struct gsgpu_device *adev)
{
	int ret;

	/* No need to evict vram on APUs for suspend to ram or s2idle */
	if ((adev->in_s3 || adev->in_s0ix) && (adev->flags & GSGPU_IS_APU))
		return 0;

	ret = gsgpu_ttm_evict_resources(adev, TTM_PL_VRAM);
	if (ret)
		DRM_WARN("evicting device resources failed\n");
	return ret;
}

static int gsgpu_zip_gem_bo_validate(int id, void *ptr, void *data)
{
	struct drm_gem_object *gobj = ptr;
	struct gsgpu_bo *bo = gem_to_gsgpu_bo(gobj);
	int r, i;
	struct ttm_operation_ctx ctx = { false, false };
	struct drm_mm_node *node = NULL;
	unsigned domain;

	domain = gsgpu_mem_type_to_domain(bo->tbo.resource->mem_type);

	if (bo->flags & GSGPU_GEM_CREATE_COMPRESSED_MASK) {

		gsgpu_bo_reserve(bo, false);

		domain = bo->preferred_domains;

		node = &to_ttm_range_mgr_node(bo->tbo.resource)->mm_nodes[0];

		bo->flags |= GSGPU_GEM_CREATE_VRAM_CONTIGUOUS;
		/* force to pin into visible video ram */
		if (!(bo->flags & GSGPU_GEM_CREATE_NO_CPU_ACCESS))
			bo->flags |= GSGPU_GEM_CREATE_CPU_ACCESS_REQUIRED;
		gsgpu_bo_placement_from_domain(bo, domain);
		for (i = 0; i < bo->placement.num_placement; i++) {
			unsigned fpfn, lpfn;

			fpfn = bo->node_offset;
			lpfn = bo->node_offset + bo->tbo.ttm->num_pages;

			if (fpfn > bo->placements[i].fpfn)
				bo->placements[i].fpfn = fpfn;
			if (!bo->placements[i].lpfn ||
				(lpfn && lpfn < bo->placements[i].lpfn))
				bo->placements[i].lpfn = lpfn;
			// bo->placements[i].flags |= TTM_PL_FLAG_NO_EVICT;
		}

		r = ttm_bo_validate(&bo->tbo, &bo->placement, &ctx);
		if (unlikely(r)) {
			DRM_ERROR("gsgpu_zip_gem_bo_valid failed  tbo=0x%x r=0x%x\n", &bo->tbo, r);
		}

		gsgpu_bo_unreserve(bo);

	}
	
	return 0;
}

static int gsgpu_zip_gem_bo_evict(int id, void *ptr, void *data)
{
	struct drm_gem_object *gobj = ptr;
	struct gsgpu_bo *bo = gem_to_gsgpu_bo(gobj);
	int r, i;
	struct ttm_operation_ctx ctx = { false, false };
	struct drm_mm_node *node = NULL;
	unsigned domain;

	domain = gsgpu_mem_type_to_domain(bo->tbo.resource->mem_type);

	if (bo->flags & GSGPU_GEM_CREATE_COMPRESSED_MASK && domain == GSGPU_GEM_DOMAIN_VRAM) {

		gsgpu_bo_reserve(bo, false);

		node = &to_ttm_range_mgr_node(bo->tbo.resource)->mm_nodes[0];

		bo->node_offset = node->start;

		for (i = 0; i < bo->placement.num_placement; i++) {
			bo->placements[i].lpfn = 0;
			// bo->placements[i].flags &= ~TTM_PL_FLAG_NO_EVICT;
		}

		r = ttm_bo_validate(&bo->tbo, &bo->placement, &ctx);
		if (unlikely(r))
			DRM_ERROR("gsgpu_zip_gem_bo_evict failed  tbo=0x%x r=0x%x\n", &bo->tbo, r);

		gsgpu_bo_unreserve(bo);
	}
	
	return 0;
}

/*
 * Suspend & resume.
 */
/**
 * gsgpu_device_suspend - initiate device suspend
 *
 * @dev: drm dev pointer
 * @fbcon : notify the fbdev of suspend
 *
 * Puts the hw in the suspend state (all asics).
 * Returns 0 for success or an error on failure.
 * Called at driver suspend.
 */
int gsgpu_device_suspend(struct drm_device *dev, bool fbcon)
{
	struct gsgpu_device *adev = drm_to_adev(dev);
	int r = 0;
	struct drm_file *file;

	if (dev->switch_power_state == DRM_SWITCH_POWER_OFF)
		return 0;

	adev->in_suspend = true;

	/* Evict the majority of BOs before grabbing the full access */
	r = gsgpu_device_evict_resources(adev);
	if (r)
		return r;

	if (gsgpu_acpi_smart_shift_update(dev, GSGPU_SS_DEV_D3))
		DRM_WARN("smart shift update failed\n");

	if (fbcon)
		drm_fb_helper_set_suspend_unlocked(adev_to_drm(adev)->fb_helper, true);

	cancel_delayed_work_sync(&adev->delayed_init_work);

	gsgpu_device_ip_suspend_phase1(adev);

	r = mutex_lock_interruptible(&dev->filelist_mutex);
	if (r)
		return r;

	list_for_each_entry(file, &dev->filelist, lhead) {
		spin_lock(&file->table_lock);
		idr_for_each(&file->object_idr, gsgpu_zip_gem_bo_evict, NULL);
		spin_unlock(&file->table_lock);
	}

	mutex_unlock(&dev->filelist_mutex);

	r = gsgpu_device_evict_resources(adev);
	if (r)
		return r;

	gsgpu_fence_driver_hw_fini(adev);

	gsgpu_device_ip_suspend_phase2(adev);

	return 0;
}

/**
 * gsgpu_device_resume - initiate device resume
 *
 * @dev: drm dev pointer
 * @fbcon : notify the fbdev of resume
 *
 * Bring the hw back to operating state (all asics).
 * Returns 0 for success or an error on failure.
 * Called at driver resume.
 */
int gsgpu_device_resume(struct drm_device *dev, bool fbcon)
{
	struct gsgpu_device *adev = drm_to_adev(dev);
	int r = 0;
	struct drm_file *file;

	if (dev->switch_power_state == DRM_SWITCH_POWER_OFF)
		return 0;

	*(int *)(0x80000e0010010444) |= 0x10;

	r = gsgpu_cp_gfx_load_microcode(adev);
	if (r) {
		DRM_ERROR(" gsgpu_cp_gfx_load_microcode fail\n");
		return r;
	}
		
	r = gsgpu_cp_enable(adev, true);
	if (r) {
		DRM_ERROR(" gsgpu_cp_enable fail\n");
		return r;
	}

	r = mutex_lock_interruptible(&dev->filelist_mutex);
	if (r)
		return r;

	list_for_each_entry(file, &dev->filelist, lhead) {
		spin_lock(&file->table_lock);
		idr_for_each(&file->object_idr, gsgpu_zip_gem_bo_validate, NULL);
		spin_unlock(&file->table_lock);
	}

	mutex_unlock(&dev->filelist_mutex);

	r = gsgpu_device_ip_resume(adev);

	if (r) {
		dev_err(adev->dev, "gsgpu_device_ip_resume failed (%d).\n", r);
		goto exit;
	}
	gsgpu_fence_driver_hw_init(adev);

	r = gsgpu_device_ip_late_init(adev);
	if (r)
		goto exit;

	queue_delayed_work(system_wq, &adev->delayed_init_work,
			   msecs_to_jiffies(GSGPU_RESUME_MS));

exit:
	if (r)
		return r;

	/* Make sure IB tests flushed */
	flush_delayed_work(&adev->delayed_init_work);

	if (fbcon)
		drm_fb_helper_set_suspend_unlocked(adev_to_drm(adev)->fb_helper, false);

	if (adev->mode_info.num_crtc) {
		/*
		 * Most of the connector probing functions try to acquire runtime pm
		 * refs to ensure that the GPU is powered on when connector polling is
		 * performed. Since we're calling this from a runtime PM callback,
		 * trying to acquire rpm refs will cause us to deadlock.
		 *
		 * Since we're guaranteed to be holding the rpm lock, it's safe to
		 * temporarily disable the rpm helpers so this doesn't deadlock us.
		 */
#ifdef CONFIG_PM
		dev->dev->power.disable_depth++;
#endif
		drm_kms_helper_hotplug_event(dev);
#ifdef CONFIG_PM
		dev->dev->power.disable_depth--;
#endif
	}
	adev->in_suspend = false;

	if (gsgpu_acpi_smart_shift_update(dev, GSGPU_SS_DEV_D0))
		DRM_WARN("smart shift update failed\n");

	return 0;
}

/**
 * gsgpu_device_ip_check_soft_reset - did soft reset succeed
 *
 * @adev: gsgpu_device pointer
 *
 * The list of all the hardware IPs that make up the asic is walked and
 * the check_soft_reset callbacks are run.  check_soft_reset determines
 * if the asic is still hung or not.
 * Returns true if any of the IPs are still in a hung state, false if not.
 */
// static bool gsgpu_device_ip_check_soft_reset(struct gsgpu_device *adev)
// {
// 	int i;
// 	bool asic_hang = false;

// 	if (gsgpu_asic_need_full_reset(adev))
// 		return true;

// 	for (i = 0; i < adev->num_ip_blocks; i++) {
// 		if (!adev->ip_blocks[i].status.valid)
// 			continue;
// 		if (adev->ip_blocks[i].version->funcs->check_soft_reset)
// 			adev->ip_blocks[i].status.hang =
// 				adev->ip_blocks[i].version->funcs->check_soft_reset(adev);
// 		if (adev->ip_blocks[i].status.hang) {
// 			dev_info(adev->dev, "IP block:%s is hung!\n", adev->ip_blocks[i].version->funcs->name);
// 			asic_hang = true;
// 		}
// 	}
// 	return asic_hang;
// }

/**
 * gsgpu_device_ip_pre_soft_reset - prepare for soft reset
 *
 * @adev: gsgpu_device pointer
 *
 * The list of all the hardware IPs that make up the asic is walked and the
 * pre_soft_reset callbacks are run if the block is hung.  pre_soft_reset
 * handles any IP specific hardware or software state changes that are
 * necessary for a soft reset to succeed.
 * Returns 0 on success, negative error code on failure.
 */
// static int gsgpu_device_ip_pre_soft_reset(struct gsgpu_device *adev)
// {
// 	int i, r = 0;

// 	for (i = 0; i < adev->num_ip_blocks; i++) {
// 		if (!adev->ip_blocks[i].status.valid)
// 			continue;
// 		if (adev->ip_blocks[i].status.hang &&
// 		    adev->ip_blocks[i].version->funcs->pre_soft_reset) {
// 			r = adev->ip_blocks[i].version->funcs->pre_soft_reset(adev);
// 			if (r)
// 				return r;
// 		}
// 	}

// 	return 0;
// }

/**
 * gsgpu_device_ip_need_full_reset - check if a full asic reset is needed
 *
 * @adev: gsgpu_device pointer
 *
 * Some hardware IPs cannot be soft reset.  If they are hung, a full gpu
 * reset is necessary to recover.
 * Returns true if a full asic reset is required, false if not.
 */
static bool gsgpu_device_ip_need_full_reset(struct gsgpu_device *adev)
{
	int i;

	if (gsgpu_asic_need_full_reset(adev))
		return true;

	for (i = 0; i < adev->num_ip_blocks; i++) {
		if (!adev->ip_blocks[i].status.valid)
			continue;
		if ((adev->ip_blocks[i].version->type == GSGPU_IP_BLOCK_TYPE_GMC) ||
		    (adev->ip_blocks[i].version->type == GSGPU_IP_BLOCK_TYPE_DCE)) {
			if (adev->ip_blocks[i].status.hang) {
				dev_info(adev->dev, "Some block need full reset!\n");
				return true;
			}
		}
	}
	return false;
}

/**
 * gsgpu_device_ip_soft_reset - do a soft reset
 *
 * @adev: gsgpu_device pointer
 *
 * The list of all the hardware IPs that make up the asic is walked and the
 * soft_reset callbacks are run if the block is hung.  soft_reset handles any
 * IP specific hardware or software state changes that are necessary to soft
 * reset the IP.
 * Returns 0 on success, negative error code on failure.
 */
// static int gsgpu_device_ip_soft_reset(struct gsgpu_device *adev)
// {
// 	int i, r = 0;

// 	for (i = 0; i < adev->num_ip_blocks; i++) {
// 		if (!adev->ip_blocks[i].status.valid)
// 			continue;
// 		if (adev->ip_blocks[i].status.hang &&
// 		    adev->ip_blocks[i].version->funcs->soft_reset) {
// 			r = adev->ip_blocks[i].version->funcs->soft_reset(adev);
// 			if (r)
// 				return r;
// 		}
// 	}

// 	return 0;
// }

/**
 * gsgpu_device_ip_post_soft_reset - clean up from soft reset
 *
 * @adev: gsgpu_device pointer
 *
 * The list of all the hardware IPs that make up the asic is walked and the
 * post_soft_reset callbacks are run if the asic was hung.  post_soft_reset
 * handles any IP specific hardware or software state changes that are
 * necessary after the IP has been soft reset.
 * Returns 0 on success, negative error code on failure.
 */
// static int gsgpu_device_ip_post_soft_reset(struct gsgpu_device *adev)
// {
// 	int i, r = 0;

// 	for (i = 0; i < adev->num_ip_blocks; i++) {
// 		if (!adev->ip_blocks[i].status.valid)
// 			continue;
// 		if (adev->ip_blocks[i].status.hang &&
// 		    adev->ip_blocks[i].version->funcs->post_soft_reset)
// 			r = adev->ip_blocks[i].version->funcs->post_soft_reset(adev);
// 		if (r)
// 			return r;
// 	}

// 	return 0;
// }

/**
 * gsgpu_device_recover_vram - Recover some VRAM contents
 *
 * @adev: gsgpu_device pointer
 *
 * Restores the contents of VRAM buffers from the shadows in GTT.  Used to
 * restore things like GPUVM page tables after a GPU reset where
 * the contents of VRAM might be lost.
 *
 * Returns:
 * 0 on success, negative error code on failure.
 */
static int gsgpu_device_recover_vram(struct gsgpu_device *adev)
{
	struct dma_fence *fence = NULL, *next = NULL;
	struct gsgpu_bo *shadow;
	struct gsgpu_bo_vm *vmbo;
	long r = 1, tmo;

	tmo = msecs_to_jiffies(100);

	dev_info(adev->dev, "recover vram bo from shadow start\n");
	mutex_lock(&adev->shadow_list_lock);
	list_for_each_entry(vmbo, &adev->shadow_list, shadow_list) {
		shadow = &vmbo->bo;
		/* No need to recover an evicted BO */
		if (shadow->tbo.resource->mem_type != TTM_PL_TT ||
		    shadow->tbo.resource->start == GSGPU_BO_INVALID_OFFSET ||
		    shadow->parent->tbo.resource->mem_type != TTM_PL_VRAM)
			continue;

		r = gsgpu_bo_restore_shadow(shadow, &next);
		if (r)
			break;

		if (fence) {
			tmo = dma_fence_wait_timeout(fence, false, tmo);
			dma_fence_put(fence);
			fence = next;
			if (tmo == 0) {
				r = -ETIMEDOUT;
				break;
			} else if (tmo < 0) {
				r = tmo;
				break;
			}
		} else {
			fence = next;
		}
	}
	mutex_unlock(&adev->shadow_list_lock);

	if (fence)
		tmo = dma_fence_wait_timeout(fence, false, tmo);
	dma_fence_put(fence);

	if (r < 0 || tmo <= 0) {
		dev_err(adev->dev, "recover vram bo from shadow failed, r is %ld, tmo is %ld\n", r, tmo);
		return -EIO;
	}

	dev_info(adev->dev, "recover vram bo from shadow done\n");
	return 0;
}

/**
 * gsgpu_device_has_job_running - check if there is any job in mirror list
 *
 * @adev: gsgpu_device pointer
 *
 * check if there is any job in mirror list
 */
bool gsgpu_device_has_job_running(struct gsgpu_device *adev)
{
	int i;
	struct drm_sched_job *job;

	for (i = 0; i < GSGPU_MAX_RINGS; ++i) {
		struct gsgpu_ring *ring = adev->rings[i];

		if (!ring || !ring->sched.thread)
			continue;

		spin_lock(&ring->sched.job_list_lock);
		job = list_first_entry_or_null(&ring->sched.pending_list,
					       struct drm_sched_job, list);
		spin_unlock(&ring->sched.job_list_lock);
		if (job)
			return true;
	}
	return false;
}

int gsgpu_device_pre_asic_reset(struct gsgpu_device *adev,
				 struct gsgpu_reset_context *reset_context)
{
	int i, r = 0;
	struct gsgpu_job *job = NULL;
	bool need_full_reset =
		test_bit(GSGPU_NEED_FULL_RESET, &reset_context->flags);

	if (reset_context->reset_req_dev == adev)
		job = reset_context->job;

	gsgpu_fence_driver_isr_toggle(adev, true);

	/* block all schedulers and reset given job's ring */
	for (i = 0; i < GSGPU_MAX_RINGS; ++i) {
		struct gsgpu_ring *ring = adev->rings[i];

		if (!ring || !ring->sched.thread)
			continue;

		/*clear job fence from fence drv to avoid force_completion
		 *leave NULL and vm flush fence in fence drv */
		gsgpu_fence_driver_clear_job_fences(ring);

		/* after all hw jobs are reset, hw fence is meaningless, so force_completion */
		gsgpu_fence_driver_force_completion(ring);
	}

	gsgpu_fence_driver_isr_toggle(adev, false);

	if (job && job->vm)
		drm_sched_increase_karma(&job->base);

	r = gsgpu_reset_prepare_hwcontext(adev, reset_context);
	/* If reset handler not implemented, continue; otherwise return */
	if (r == -ENOSYS) {
		r = 0;
	}
	else {
		return r;
	}

		if (!need_full_reset)
			need_full_reset = gsgpu_device_ip_need_full_reset(adev);

		if (need_full_reset)
			r = gsgpu_device_ip_suspend(adev);
		if (need_full_reset)
			set_bit(GSGPU_NEED_FULL_RESET, &reset_context->flags);
		else
			clear_bit(GSGPU_NEED_FULL_RESET,
				  &reset_context->flags);

	return r;
}

static int gsgpu_reset_reg_dumps(struct gsgpu_device *adev)
{
	int i;

	lockdep_assert_held(&adev->reset_domain->sem);

	for (i = 0; i < adev->num_regs; i++) {
		adev->reset_dump_reg_value[i] = RREG32(adev->reset_dump_reg_list[i]);
		trace_gsgpu_reset_reg_dumps(adev->reset_dump_reg_list[i],
					     adev->reset_dump_reg_value[i]);
	}

	return 0;
}

#ifdef CONFIG_DEV_COREDUMP
static ssize_t gsgpu_devcoredump_read(char *buffer, loff_t offset,
		size_t count, void *data, size_t datalen)
{
	struct drm_printer p;
	struct gsgpu_device *adev = data;
	struct drm_print_iterator iter;
	int i;

	iter.data = buffer;
	iter.offset = 0;
	iter.start = offset;
	iter.remain = count;

	p = drm_coredump_printer(&iter);

	drm_printf(&p, "**** GSGPU Device Coredump ****\n");
	drm_printf(&p, "kernel: " UTS_RELEASE "\n");
	drm_printf(&p, "module: " KBUILD_MODNAME "\n");
	drm_printf(&p, "time: %lld.%09ld\n", adev->reset_time.tv_sec, adev->reset_time.tv_nsec);
	if (adev->reset_task_info.pid)
		drm_printf(&p, "process_name: %s PID: %d\n",
			   adev->reset_task_info.process_name,
			   adev->reset_task_info.pid);

	if (adev->reset_vram_lost)
		drm_printf(&p, "VRAM is lost due to GPU reset!\n");
	if (adev->num_regs) {
		drm_printf(&p, "GSGPU register dumps:\nOffset:     Value:\n");

		for (i = 0; i < adev->num_regs; i++)
			drm_printf(&p, "0x%08x: 0x%08x\n",
				   adev->reset_dump_reg_list[i],
				   adev->reset_dump_reg_value[i]);
	}

	return count - iter.remain;
}

static void gsgpu_devcoredump_free(void *data)
{
}

static void gsgpu_reset_capture_coredumpm(struct gsgpu_device *adev)
{
	struct drm_device *dev = adev_to_drm(adev);

	ktime_get_ts64(&adev->reset_time);
	dev_coredumpm(dev->dev, THIS_MODULE, adev, 0, GFP_KERNEL,
		      gsgpu_devcoredump_read, gsgpu_devcoredump_free);
}
#endif

int gsgpu_do_asic_reset(struct list_head *device_list_handle,
			 struct gsgpu_reset_context *reset_context)
{
	struct gsgpu_device *tmp_adev = NULL;
	bool need_full_reset, skip_hw_reset, vram_lost = false;
	int r = 0;
	bool gpu_reset_for_dev_remove = 0;

	/* Try reset handler method first */
	tmp_adev = list_first_entry(device_list_handle, struct gsgpu_device,
				    reset_list);
	gsgpu_reset_reg_dumps(tmp_adev);

	reset_context->reset_device_list = device_list_handle;
	r = gsgpu_reset_perform_reset(tmp_adev, reset_context);
	/* If reset handler not implemented, continue; otherwise return */
	if (r == -ENOSYS)
		r = 0;
	else
		return r;

	/* Reset handler not implemented, use the default method */
	need_full_reset =
		test_bit(GSGPU_NEED_FULL_RESET, &reset_context->flags);
	skip_hw_reset = test_bit(GSGPU_SKIP_HW_RESET, &reset_context->flags);

	gpu_reset_for_dev_remove =
		test_bit(GSGPU_RESET_FOR_DEVICE_REMOVE, &reset_context->flags) &&
			test_bit(GSGPU_NEED_FULL_RESET, &reset_context->flags);

	/*
	 * ASIC reset has to be done on all XGMI hive nodes ASAP
	 * to allow proper links negotiation in FW (within 1 sec)
	 */
	if (!skip_hw_reset && need_full_reset) {
		list_for_each_entry(tmp_adev, device_list_handle, reset_list) {
			/* For XGMI run all resets in parallel to speed up the process */
				r = gsgpu_asic_reset(tmp_adev);

			if (r) {
				dev_err(tmp_adev->dev, "ASIC reset failed with error, %d for drm dev, %s",
					 r, adev_to_drm(tmp_adev)->unique);
				break;
			}
		}
	}

	/* Since the mode1 reset affects base ip blocks, the
	 * phase1 ip blocks need to be resumed. Otherwise there
	 * will be a BIOS signature error and the psp bootloader
	 * can't load kdb on the next gsgpu install.
	 */
	if (gpu_reset_for_dev_remove) {
		list_for_each_entry(tmp_adev, device_list_handle, reset_list)
			gsgpu_device_ip_resume_phase1(tmp_adev);

		goto end;
	}

	list_for_each_entry(tmp_adev, device_list_handle, reset_list) {
		if (need_full_reset) {
			/* post card */
			r = gsgpu_device_asic_init(tmp_adev);
			if (r) {
				dev_warn(tmp_adev->dev, "asic atom init failed!");
			} else {
				dev_info(tmp_adev->dev, "GPU reset succeeded, trying to resume\n");
				r = gsgpu_device_ip_resume_phase1(tmp_adev);
				if (r)
					goto out;

				vram_lost = gsgpu_device_check_vram_lost(tmp_adev);
#ifdef CONFIG_DEV_COREDUMP
				tmp_adev->reset_vram_lost = vram_lost;
				memset(&tmp_adev->reset_task_info, 0,
						sizeof(tmp_adev->reset_task_info));
				if (reset_context->job && reset_context->job->vm)
					tmp_adev->reset_task_info =
						reset_context->job->vm->task_info;
				gsgpu_reset_capture_coredumpm(tmp_adev);
#endif
				if (vram_lost) {
					DRM_INFO("VRAM is lost due to GPU reset!\n");
					gsgpu_inc_vram_lost(tmp_adev);
				}

				r = gsgpu_device_ip_resume_phase2(tmp_adev);
				if (r)
					goto out;

				if (vram_lost)
					gsgpu_device_fill_reset_magic(tmp_adev);

				/*
				 * Add this ASIC as tracked as reset was already
				 * complete successfully.
				 */
				gsgpu_register_gpu_instance(tmp_adev);

				r = gsgpu_device_ip_late_init(tmp_adev);
				if (r)
					goto out;

				drm_fb_helper_set_suspend_unlocked(adev_to_drm(tmp_adev)->fb_helper, false);
			}
		}

out:
		if (!r) {
			gsgpu_irq_gpu_reset_resume_helper(tmp_adev);
			r = gsgpu_ib_ring_tests(tmp_adev);
			if (r) {
				dev_err(tmp_adev->dev, "ib ring test failed (%d).\n", r);
				need_full_reset = true;
				r = -EAGAIN;
				goto end;
			}
		}

		if (!r)
			r = gsgpu_device_recover_vram(tmp_adev);
		else
			tmp_adev->asic_reset_res = r;
	}

end:
	if (need_full_reset)
		set_bit(GSGPU_NEED_FULL_RESET, &reset_context->flags);
	else
		clear_bit(GSGPU_NEED_FULL_RESET, &reset_context->flags);
	return r;
}

static void gsgpu_device_set_mp1_state(struct gsgpu_device *adev)
{

}

static void gsgpu_device_unset_mp1_state(struct gsgpu_device *adev)
{
	adev->mp1_state = PP_MP1_STATE_NONE;
}

static void gsgpu_device_resume_display_audio(struct gsgpu_device *adev)
{
	struct pci_dev *p = NULL;

	p = pci_get_domain_bus_and_slot(pci_domain_nr(adev->pdev->bus),
			adev->pdev->bus->number, 1);
	if (p) {
		pm_runtime_enable(&(p->dev));
		pm_runtime_resume(&(p->dev));
	}

	pci_dev_put(p);
}

static inline void gsgpu_device_stop_pending_resets(struct gsgpu_device *adev)
{
#if defined(CONFIG_DEBUG_FS)
		cancel_work(&adev->reset_work);
#endif
}

/**
 * gsgpu_device_gpu_recover - reset the asic and recover scheduler
 *
 * @adev: gsgpu_device pointer
 * @job: which job trigger hang
 *
 * Attempt to reset the GPU if it has hung (all asics).
 * Attempt to do soft-reset or full-reset and reinitialize Asic
 * Returns 0 for success or an error on failure.
 */

int gsgpu_device_gpu_recover(struct gsgpu_device *adev,
			      struct gsgpu_job *job,
			      struct gsgpu_reset_context *reset_context)
{
	struct list_head device_list, *device_list_handle =  NULL;
	bool job_signaled = false;
	struct gsgpu_device *tmp_adev = NULL;
	int i, r = 0;
	bool need_emergency_restart = false;
	bool audio_suspended = false;
	bool gpu_reset_for_dev_remove = false;

	gpu_reset_for_dev_remove =
			test_bit(GSGPU_RESET_FOR_DEVICE_REMOVE, &reset_context->flags) &&
				test_bit(GSGPU_NEED_FULL_RESET, &reset_context->flags);

	dev_info(adev->dev, "GPU %s begin!\n",
		need_emergency_restart ? "jobs stop":"reset");

	reset_context->job = job;
	/*
	 * Build list of devices to reset.
	 * In case we are in XGMI hive mode, resort the device list
	 * to put adev in the 1st position.
	 */
	INIT_LIST_HEAD(&device_list);

		list_add_tail(&adev->reset_list, &device_list);
		device_list_handle = &device_list;

	/* We need to lock reset domain only once both for XGMI and single device */
	tmp_adev = list_first_entry(device_list_handle, struct gsgpu_device,
				    reset_list);
	gsgpu_device_lock_reset_domain(tmp_adev->reset_domain);

	/* block all schedulers and reset given job's ring */
	list_for_each_entry(tmp_adev, device_list_handle, reset_list) {

		gsgpu_device_set_mp1_state(tmp_adev);

		cancel_delayed_work_sync(&tmp_adev->delayed_init_work);

		/*
		 * Mark these ASICs to be reseted as untracked first
		 * And add them back after reset completed
		 */
		gsgpu_unregister_gpu_instance(tmp_adev);

		drm_fb_helper_set_suspend_unlocked(adev_to_drm(tmp_adev)->fb_helper, true);

		for (i = 0; i < GSGPU_MAX_RINGS; ++i) {
			struct gsgpu_ring *ring = tmp_adev->rings[i];

			if (!ring || !ring->sched.thread)
				continue;

			drm_sched_stop(&ring->sched, job ? &job->base : NULL);

			if (need_emergency_restart)
				gsgpu_job_stop_all_jobs_on_sched(&ring->sched);
		}
		atomic_inc(&tmp_adev->gpu_reset_counter);
	}

	if (need_emergency_restart)
		goto skip_sched_resume;

	/*
	 * Must check guilty signal here since after this point all old
	 * HW fences are force signaled.
	 *
	 * job->base holds a reference to parent fence
	 */
	if (job && dma_fence_is_signaled(&job->hw_fence)) {
		job_signaled = true;
		dev_info(adev->dev, "Guilty job already signaled, skipping HW reset");
		goto skip_hw_reset;
	}

retry:	/* Rest of adevs pre asic reset from XGMI hive. */
	list_for_each_entry(tmp_adev, device_list_handle, reset_list) {
		r = gsgpu_device_pre_asic_reset(tmp_adev, reset_context);
		/*TODO Should we stop ?*/
		if (r) {
			dev_err(tmp_adev->dev, "GPU pre asic reset failed with err, %d for drm dev, %s ",
				  r, adev_to_drm(tmp_adev)->unique);
			tmp_adev->asic_reset_res = r;
		}

		/*
		 * Drop all pending non scheduler resets. Scheduler resets
		 * were already dropped during drm_sched_stop
		 */
		gsgpu_device_stop_pending_resets(tmp_adev);
	}
	
	{
		r = gsgpu_do_asic_reset(device_list_handle, reset_context);
		if (r && r == -EAGAIN)
			goto retry;

		if (!r && gpu_reset_for_dev_remove)
			goto recover_end;
	}

skip_hw_reset:

	/* Post ASIC reset for all devs .*/
	list_for_each_entry(tmp_adev, device_list_handle, reset_list) {

		for (i = 0; i < GSGPU_MAX_RINGS; ++i) {
			struct gsgpu_ring *ring = tmp_adev->rings[i];

			if (!ring || !ring->sched.thread)
				continue;

			drm_sched_start(&ring->sched, true);
		}

		if (!drm_drv_uses_atomic_modeset(adev_to_drm(tmp_adev)) && !job_signaled) {
			drm_helper_resume_force_mode(adev_to_drm(tmp_adev));
		}

		if (tmp_adev->asic_reset_res)
			r = tmp_adev->asic_reset_res;

		tmp_adev->asic_reset_res = 0;

		if (r) {
			/* bad news, how to tell it to userspace ? */
			dev_info(tmp_adev->dev, "GPU reset(%d) failed\n", atomic_read(&tmp_adev->gpu_reset_counter));
		} else {
			dev_info(tmp_adev->dev, "GPU reset(%d) succeeded!\n", atomic_read(&tmp_adev->gpu_reset_counter));
			if (gsgpu_acpi_smart_shift_update(adev_to_drm(tmp_adev), GSGPU_SS_DEV_D0))
				DRM_WARN("smart shift update failed\n");
		}
	}

skip_sched_resume:
	list_for_each_entry(tmp_adev, device_list_handle, reset_list) {

		if (audio_suspended)
			gsgpu_device_resume_display_audio(tmp_adev);

		gsgpu_device_unset_mp1_state(tmp_adev);
	}

recover_end:
	tmp_adev = list_first_entry(device_list_handle, struct gsgpu_device,
					    reset_list);
	gsgpu_device_unlock_reset_domain(tmp_adev->reset_domain);

	if (r)
		dev_info(adev->dev, "GPU reset end with ret = %d\n", r);

	atomic_set(&adev->reset_domain->reset_res, r);
	return r;
}

/**
 * gsgpu_device_get_pcie_info - fence pcie info about the PCIE slot
 *
 * @adev: gsgpu_device pointer
 *
 * Fetchs and stores in the driver the PCIE capabilities (gen speed
 * and lanes) of the slot the device is in. Handles APUs and
 * virtualized environments where PCIE config space may not be available.
 */
static void gsgpu_device_get_pcie_info(struct gsgpu_device *adev)
{
}

/**
 * gsgpu_device_is_peer_accessible - Check peer access through PCIe BAR
 *
 * @adev: gsgpu_device pointer
 * @peer_adev: gsgpu_device pointer for peer device trying to access @adev
 *
 * Return true if @peer_adev can access (DMA) @adev through the PCIe
 * BAR, i.e. @adev is "large BAR" and the BAR matches the DMA mask of
 * @peer_adev.
 */
bool gsgpu_device_is_peer_accessible(struct gsgpu_device *adev,
				      struct gsgpu_device *peer_adev)
{
	return false;
}

/**
 * gsgpu_pci_error_detected - Called when a PCI error is detected.
 * @pdev: PCI device struct
 * @state: PCI channel state
 *
 * Description: Called when a PCI error is detected.
 *
 * Return: PCI_ERS_RESULT_NEED_RESET or PCI_ERS_RESULT_DISCONNECT.
 */
pci_ers_result_t gsgpu_pci_error_detected(struct pci_dev *pdev, pci_channel_state_t state)
{
	struct drm_device *dev = pci_get_drvdata(pdev);
	struct gsgpu_device *adev = drm_to_adev(dev);
	int i;

	DRM_INFO("PCI error: detected callback, state(%d)!!\n", state);

	adev->pci_channel_state = state;

	switch (state) {
	case pci_channel_io_normal:
		return PCI_ERS_RESULT_CAN_RECOVER;
	/* Fatal error, prepare for slot reset */
	case pci_channel_io_frozen:
		/*
		 * Locking adev->reset_domain->sem will prevent any external access
		 * to GPU during PCI error recovery
		 */
		gsgpu_device_lock_reset_domain(adev->reset_domain);
		gsgpu_device_set_mp1_state(adev);

		/*
		 * Block any work scheduling as we do for regular GPU reset
		 * for the duration of the recovery
		 */
		for (i = 0; i < GSGPU_MAX_RINGS; ++i) {
			struct gsgpu_ring *ring = adev->rings[i];

			if (!ring || !ring->sched.thread)
				continue;

			drm_sched_stop(&ring->sched, NULL);
		}
		atomic_inc(&adev->gpu_reset_counter);
		return PCI_ERS_RESULT_NEED_RESET;
	case pci_channel_io_perm_failure:
		/* Permanent error, prepare for device removal */
		return PCI_ERS_RESULT_DISCONNECT;
	}

	return PCI_ERS_RESULT_NEED_RESET;
}

/**
 * gsgpu_pci_mmio_enabled - Enable MMIO and dump debug registers
 * @pdev: pointer to PCI device
 */
pci_ers_result_t gsgpu_pci_mmio_enabled(struct pci_dev *pdev)
{

	DRM_INFO("PCI error: mmio enabled callback!!\n");

	/* TODO - dump whatever for debugging purposes */

	/* This called only if gsgpu_pci_error_detected returns
	 * PCI_ERS_RESULT_CAN_RECOVER. Read/write to the device still
	 * works, no need to reset slot.
	 */

	return PCI_ERS_RESULT_RECOVERED;
}

/**
 * gsgpu_pci_slot_reset - Called when PCI slot has been reset.
 * @pdev: PCI device struct
 *
 * Description: This routine is called by the pci error recovery
 * code after the PCI slot has been reset, just before we
 * should resume normal operations.
 */
pci_ers_result_t gsgpu_pci_slot_reset(struct pci_dev *pdev)
{
	struct drm_device *dev = pci_get_drvdata(pdev);
	struct gsgpu_device *adev = drm_to_adev(dev);
	int r; // , i;
	struct gsgpu_reset_context reset_context;
	// u32 memsize;
	struct list_head device_list;

	DRM_INFO("PCI error: slot reset callback!!\n");

	memset(&reset_context, 0, sizeof(reset_context));

	INIT_LIST_HEAD(&device_list);
	list_add_tail(&adev->reset_list, &device_list);

	/* wait for asic to come out of reset */
	msleep(500);

	/* Restore PCI confspace */
	gsgpu_device_load_pci_state(pdev);

	reset_context.method = GSGPU_RESET_METHOD_NONE;
	reset_context.reset_req_dev = adev;
	set_bit(GSGPU_NEED_FULL_RESET, &reset_context.flags);
	set_bit(GSGPU_SKIP_HW_RESET, &reset_context.flags);

	adev->no_hw_access = true;
	r = gsgpu_device_pre_asic_reset(adev, &reset_context);
	adev->no_hw_access = false;
	if (r)
		goto out;

	r = gsgpu_do_asic_reset(&device_list, &reset_context);

out:
	if (!r) {
		if (gsgpu_device_cache_pci_state(adev->pdev))
			pci_restore_state(adev->pdev);

		DRM_INFO("PCIe error recovery succeeded\n");
	} else {
		DRM_ERROR("PCIe error recovery failed, err:%d", r);
		gsgpu_device_unset_mp1_state(adev);
		gsgpu_device_unlock_reset_domain(adev->reset_domain);
	}

	return r ? PCI_ERS_RESULT_DISCONNECT : PCI_ERS_RESULT_RECOVERED;
}

/**
 * gsgpu_pci_resume() - resume normal ops after PCI reset
 * @pdev: pointer to PCI device
 *
 * Called when the error recovery driver tells us that its
 * OK to resume normal operation.
 */
void gsgpu_pci_resume(struct pci_dev *pdev)
{
	struct drm_device *dev = pci_get_drvdata(pdev);
	struct gsgpu_device *adev = drm_to_adev(dev);
	int i;


	DRM_INFO("PCI error: resume callback!!\n");

	/* Only continue execution for the case of pci_channel_io_frozen */
	if (adev->pci_channel_state != pci_channel_io_frozen)
		return;

	for (i = 0; i < GSGPU_MAX_RINGS; ++i) {
		struct gsgpu_ring *ring = adev->rings[i];

		if (!ring || !ring->sched.thread)
			continue;

		drm_sched_start(&ring->sched, true);
	}

	gsgpu_device_unset_mp1_state(adev);
	gsgpu_device_unlock_reset_domain(adev->reset_domain);
}

bool gsgpu_device_cache_pci_state(struct pci_dev *pdev)
{
	struct drm_device *dev = pci_get_drvdata(pdev);
	struct gsgpu_device *adev = drm_to_adev(dev);
	int r;

	r = pci_save_state(pdev);
	if (!r) {
		kfree(adev->pci_state);

		adev->pci_state = pci_store_saved_state(pdev);

		if (!adev->pci_state) {
			DRM_ERROR("Failed to store PCI saved state");
			return false;
		}
	} else {
		DRM_WARN("Failed to save PCI state, err:%d\n", r);
		return false;
	}

	return true;
}

bool gsgpu_device_load_pci_state(struct pci_dev *pdev)
{
	struct drm_device *dev = pci_get_drvdata(pdev);
	struct gsgpu_device *adev = drm_to_adev(dev);
	int r;

	if (!adev->pci_state)
		return false;

	r = pci_load_saved_state(pdev, adev->pci_state);

	if (!r) {
		pci_restore_state(pdev);
	} else {
		DRM_WARN("Failed to load PCI state, err:%d\n", r);
		return false;
	}

	return true;
}

void gsgpu_device_flush_hdp(struct gsgpu_device *adev,
		struct gsgpu_ring *ring)
{	
}

void gsgpu_device_invalidate_hdp(struct gsgpu_device *adev,
		struct gsgpu_ring *ring)
{
}

int gsgpu_in_reset(struct gsgpu_device *adev)
{
	return atomic_read(&adev->reset_domain->in_gpu_reset);
}

/**
 * gsgpu_device_halt() - bring hardware to some kind of halt state
 *
 * @adev: gsgpu_device pointer
 *
 * Bring hardware to some kind of halt state so that no one can touch it
 * any more. It will help to maintain error context when error occurred.
 * Compare to a simple hang, the system will keep stable at least for SSH
 * access. Then it should be trivial to inspect the hardware state and
 * see what's going on. Implemented as following:
 *
 * 1. drm_dev_unplug() makes device inaccessible to user space(IOCTLs, etc),
 *    clears all CPU mappings to device, disallows remappings through page faults
 * 2. gsgpu_irq_disable_all() disables all interrupts
 * 3. gsgpu_fence_driver_hw_fini() signals all HW fences
 * 4. set adev->no_hw_access to avoid potential crashes after setp 5
 * 5. gsgpu_device_unmap_mmio() clears all MMIO mappings
 * 6. pci_disable_device() and pci_wait_for_pending_transaction()
 *    flush any in flight DMA operations
 */
void gsgpu_device_halt(struct gsgpu_device *adev)
{
	struct pci_dev *pdev = adev->pdev;
	struct drm_device *ddev = adev_to_drm(adev);

	drm_dev_unplug(ddev);

	gsgpu_irq_disable_all(adev);

	gsgpu_fence_driver_hw_fini(adev);

	adev->no_hw_access = true;

	gsgpu_device_unmap_mmio(adev);

	pci_disable_device(pdev);
	pci_wait_for_pending_transaction(pdev);
}

/**
 * gsgpu_device_switch_gang - switch to a new gang
 * @adev: gsgpu_device pointer
 * @gang: the gang to switch to
 *
 * Try to switch to a new gang.
 * Returns: NULL if we switched to the new gang or a reference to the current
 * gang leader.
 */
struct dma_fence *gsgpu_device_switch_gang(struct gsgpu_device *adev,
					    struct dma_fence *gang)
{
	struct dma_fence *old = NULL;

	do {
		dma_fence_put(old);
		rcu_read_lock();
		old = dma_fence_get_rcu_safe(&adev->gang_submit);
		rcu_read_unlock();

		if (old == gang)
			break;

		if (!dma_fence_is_signaled(old))
			return old;

	} while (cmpxchg((struct dma_fence __force **)&adev->gang_submit,
			 old, gang) != old);

	dma_fence_put(old);
	return NULL;
}
