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

#include "gsgpu.h"
#include <drm/gsgpu_drm.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_helper.h>

#include <linux/vga_switcheroo.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/pci.h>
#include <linux/pm_runtime.h>
#include "gsgpu_display.h"

void gsgpu_unregister_gpu_instance(struct gsgpu_device *adev)
{
	struct gsgpu_gpu_instance *gpu_instance;
	int i;

	mutex_lock(&mgpu_info.mutex);

	for (i = 0; i < mgpu_info.num_gpu; i++) {
		gpu_instance = &(mgpu_info.gpu_ins[i]);
		if (gpu_instance->adev == adev) {
			mgpu_info.gpu_ins[i] =
				mgpu_info.gpu_ins[mgpu_info.num_gpu - 1];
			mgpu_info.num_gpu--;
			if (adev->flags & GSGPU_IS_APU)
				mgpu_info.num_apu--;
			else
				mgpu_info.num_dgpu--;
			break;
		}
	}

	mutex_unlock(&mgpu_info.mutex);
}

/**
 * gsgpu_driver_unload_kms - Main unload function for KMS.
 *
 * @dev: drm dev pointer
 *
 * This is the main unload function for KMS (all asics).
 * Returns 0 on success.
 */
void gsgpu_driver_unload_kms(struct drm_device *dev)
{
	struct gsgpu_device *adev = drm_to_adev(dev);

	if (adev == NULL)
		return;

	gsgpu_unregister_gpu_instance(adev);

	if (adev->rmmio == NULL)
		return;

	if (gsgpu_acpi_smart_shift_update(dev, GSGPU_SS_DRV_UNLOAD))
		DRM_WARN("smart shift update failed\n");

	gsgpu_acpi_fini(adev);
	gsgpu_device_fini_hw(adev);
}

void gsgpu_register_gpu_instance(struct gsgpu_device *adev)
{
	struct gsgpu_gpu_instance *gpu_instance;

	mutex_lock(&mgpu_info.mutex);

	if (mgpu_info.num_gpu >= MAX_GPU_INSTANCE) {
		DRM_ERROR("Cannot register more gpu instance\n");
		mutex_unlock(&mgpu_info.mutex);
		return;
	}

	gpu_instance = &(mgpu_info.gpu_ins[mgpu_info.num_gpu]);
	gpu_instance->adev = adev;
	gpu_instance->mgpu_fan_enabled = 0;

	mgpu_info.num_gpu++;
	if (adev->flags & GSGPU_IS_APU)
		mgpu_info.num_apu++;
	else
		mgpu_info.num_dgpu++;

	mutex_unlock(&mgpu_info.mutex);
}

/**
 * gsgpu_driver_load_kms - Main load function for KMS.
 *
 * @adev: pointer to struct gsgpu_device
 * @flags: device flags
 *
 * This is the main load function for KMS (all asics).
 * Returns 0 on success, error on failure.
 */
int gsgpu_driver_load_kms(struct gsgpu_device *adev, unsigned long flags)
{
	struct drm_device *dev;
	int r; // , acpi_status;

	*(int *)(0x80000e0010010444) |= 0x10;

	dev = adev_to_drm(adev);

	/* gsgpu_device_init should report only fatal error
	 * like memory allocation failure or iomapping failure,
	 * or memory manager initialization failure, it must
	 * properly initialize the GPU MC controller and permit
	 * VRAM allocation
	 */
	r = gsgpu_device_init(adev, flags);
	if (r) {
		dev_err(dev->dev, "Fatal error during GPU init\n");
		goto out;
	}

out:
	if (r)
		gsgpu_driver_unload_kms(dev);

	return r;
}

static int gsgpu_firmware_info(struct drm_gsgpu_info_firmware *fw_info,
				struct drm_gsgpu_query_fw *query_fw,
				struct gsgpu_device *adev)
{
	switch (query_fw->fw_type) {
	case GSGPU_INFO_FW_VCE:
		fw_info->ver = 0;
		fw_info->feature = 0;
		break;
	case GSGPU_INFO_FW_UVD:
		fw_info->ver = 0;
		fw_info->feature = 0;
		break;
	case GSGPU_INFO_FW_VCN:
		fw_info->ver = 0;
		fw_info->feature = 0;
		break;
	case GSGPU_INFO_FW_GMC:
		fw_info->ver = 0;
		fw_info->feature = 0;
		break;
	case GSGPU_INFO_FW_GFX_ME:
		fw_info->ver = 0;
		fw_info->feature = 0;
		break;
	case GSGPU_INFO_FW_GFX_PFP:
		fw_info->ver = 0;
		fw_info->feature = 0;
		break;
	case GSGPU_INFO_FW_GFX_CE:
		fw_info->ver = 0;
		fw_info->feature = 0;
		break;
	case GSGPU_INFO_FW_GFX_RLC:
		fw_info->ver = 0;
		fw_info->feature = 0;
		break;
	case GSGPU_INFO_FW_GFX_RLC_RESTORE_LIST_CNTL:
		fw_info->ver = 0;
		fw_info->feature = 0;
		break;
	case GSGPU_INFO_FW_GFX_RLC_RESTORE_LIST_GPM_MEM:
		fw_info->ver = 0;
		fw_info->feature = 0;
		break;
	case GSGPU_INFO_FW_GFX_RLC_RESTORE_LIST_SRM_MEM:
		fw_info->ver = 0;
		fw_info->feature = 0;
		break;
	case GSGPU_INFO_FW_GFX_RLCP:
		fw_info->ver = 0;
		fw_info->feature = 0;
		break;
	case GSGPU_INFO_FW_GFX_RLCV:
		fw_info->ver = 0;
		fw_info->feature = 0;
		break;
	case GSGPU_INFO_FW_GFX_MEC:
		fw_info->ver = 0;
		fw_info->feature = 0;
		break;
	case GSGPU_INFO_FW_SMC:
		fw_info->ver = 0;
		fw_info->feature = 0;
		break;
	case GSGPU_INFO_FW_TA:
		fw_info->ver = 0;
		fw_info->feature = 0;
		break;
	case GSGPU_INFO_FW_XDMA:
		if (query_fw->index >= adev->xdma.num_instances)
			return -EINVAL;
		fw_info->ver = adev->xdma.instance[query_fw->index].fw_version;
		fw_info->feature = adev->xdma.instance[query_fw->index].feature_version;
		break;
	case GSGPU_INFO_FW_SOS:
		fw_info->ver = 0;
		fw_info->feature = 0;
		break;
	case GSGPU_INFO_FW_ASD:
		fw_info->ver = 0;
		fw_info->feature = 0;
		break;
	case GSGPU_INFO_FW_DMCU:
		fw_info->ver = 0;
		fw_info->feature = 0;
		break;
	case GSGPU_INFO_FW_DMCUB:
		fw_info->ver = 0;
		fw_info->feature = 0;
		break;
	case GSGPU_INFO_FW_TOC:
		fw_info->ver = 0;
		fw_info->feature = 0;
		break;
	case GSGPU_INFO_FW_CAP:
		fw_info->ver = 0;
		fw_info->feature = 0;
		break;
	case GSGPU_INFO_FW_MES_KIQ:
		fw_info->ver = 0;
		fw_info->feature = 0;
		break;
	case GSGPU_INFO_FW_MES:
		fw_info->ver = 0;
		fw_info->feature = 0;
		break;
	case GSGPU_INFO_FW_IMU:
		fw_info->ver = 0;
		fw_info->feature = 0;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int gsgpu_hw_ip_info(struct gsgpu_device *adev,
			     struct drm_gsgpu_info *info,
			     struct drm_gsgpu_info_hw_ip *result)
{
	uint32_t ib_start_alignment = 0;
	uint32_t ib_size_alignment = 0;
	enum gsgpu_ip_block_type type;
	unsigned int num_rings = 0;
	unsigned int i; // , j;

	if (info->query_hw_ip.ip_instance >= GSGPU_HW_IP_INSTANCE_MAX_COUNT)
		return -EINVAL;

	switch (info->query_hw_ip.type) {
	case GSGPU_HW_IP_GFX:
		type = GSGPU_IP_BLOCK_TYPE_GFX;
		for (i = 0; i < adev->gfx.num_gfx_rings; i++)
			if (adev->gfx.gfx_ring[i].sched.ready)
				++num_rings;
		ib_start_alignment = 32;
		ib_size_alignment = 8;
		break;
	case GSGPU_HW_IP_DMA:
		type = GSGPU_IP_BLOCK_TYPE_XDMA;
		for (i = 0; i < adev->xdma.num_instances; i++)
			if (adev->xdma.instance[i].ring.sched.ready)
				++num_rings;
		ib_start_alignment = 256;
		ib_size_alignment = 8;
		break;
	default:
		return -EINVAL;
	}

	for (i = 0; i < adev->num_ip_blocks; i++)
		if (adev->ip_blocks[i].version->type == type &&
		    adev->ip_blocks[i].status.valid)
			break;

	if (i == adev->num_ip_blocks)
		return 0;

	num_rings = min(gsgpu_ctx_num_entities[info->query_hw_ip.type],
			num_rings);

	result->hw_ip_version_major = adev->ip_blocks[i].version->major;
	result->hw_ip_version_minor = adev->ip_blocks[i].version->minor;

	result->ip_discovery_version = 0;

	result->capabilities_flags = 0;
	result->available_rings = (1 << num_rings) - 1;
	result->ib_start_alignment = ib_start_alignment;
	result->ib_size_alignment = ib_size_alignment;
	return 0;
}

/*
 * Userspace get information ioctl
 */
/**
 * gsgpu_info_ioctl - answer a device specific request.
 *
 * @dev: drm device pointer
 * @data: request object
 * @filp: drm filp
 *
 * This function is used to pass device specific parameters to the userspace
 * drivers.  Examples include: pci device id, pipeline parms, tiling params,
 * etc. (all asics).
 * Returns 0 on success, -EINVAL on failure.
 */
int gsgpu_info_ioctl(struct drm_device *dev, void *data, struct drm_file *filp)
{
	struct gsgpu_device *adev = drm_to_adev(dev);
	struct drm_gsgpu_info *info = data;
	struct gsgpu_mode_info *minfo = &adev->mode_info;
	void __user *out = (void __user *)(uintptr_t)info->return_pointer;
	uint32_t size = info->return_size;
	struct drm_crtc *crtc;
	uint32_t ui32 = 0;
	uint64_t ui64 = 0;
	int i, found;
	// int ui32_size = sizeof(ui32);

	if (!info->return_size || !info->return_pointer)
		return -EINVAL;

	switch (info->query) {
	case GSGPU_INFO_ACCEL_WORKING:
		ui32 = adev->accel_working;
		return copy_to_user(out, &ui32, min(size, 4u)) ? -EFAULT : 0;
	case GSGPU_INFO_CRTC_FROM_ID:
		for (i = 0, found = 0; i < adev->mode_info.num_crtc; i++) {
			crtc = (struct drm_crtc *)minfo->crtcs[i];
			if (crtc && crtc->base.id == info->mode_crtc.id) {
				struct gsgpu_crtc *gsgpu_crtc = to_gsgpu_crtc(crtc);
				ui32 = gsgpu_crtc->crtc_id;
				found = 1;
				break;
			}
		}
		if (!found) {
			DRM_DEBUG_KMS("unknown crtc id %d\n", info->mode_crtc.id);
			return -EINVAL;
		}
		return copy_to_user(out, &ui32, min(size, 4u)) ? -EFAULT : 0;
	case GSGPU_INFO_HW_IP_INFO: {
		struct drm_gsgpu_info_hw_ip ip = {};
		int ret;

		ret = gsgpu_hw_ip_info(adev, info, &ip);
		if (ret)
			return ret;

		ret = copy_to_user(out, &ip, min((size_t)size, sizeof(ip)));
		return ret ? -EFAULT : 0;
	}
	case GSGPU_INFO_HW_IP_COUNT: {
		enum gsgpu_ip_block_type type;
		uint32_t count = 0;

		switch (info->query_hw_ip.type) {
		case GSGPU_HW_IP_GFX:
			type = GSGPU_IP_BLOCK_TYPE_GFX;
			break;
		case GSGPU_HW_IP_DMA:
			type = GSGPU_IP_BLOCK_TYPE_XDMA;
			break;
		default:
			return -EINVAL;
		}

		for (i = 0; i < adev->num_ip_blocks; i++)
			if (adev->ip_blocks[i].version->type == type &&
			    adev->ip_blocks[i].status.valid &&
			    count < GSGPU_HW_IP_INSTANCE_MAX_COUNT)
				count++;

		return copy_to_user(out, &count, min(size, 4u)) ? -EFAULT : 0;
	}
	case GSGPU_INFO_TIMESTAMP:
		ui64 = gsgpu_gfx_get_gpu_clock_counter(adev);
		return copy_to_user(out, &ui64, min(size, 8u)) ? -EFAULT : 0;
	case GSGPU_INFO_FW_VERSION: {
		struct drm_gsgpu_info_firmware fw_info;
		int ret;

		/* We only support one instance of each IP block right now. */
		if (info->query_fw.ip_instance != 0)
			return -EINVAL;

		ret = gsgpu_firmware_info(&fw_info, &info->query_fw, adev);
		if (ret)
			return ret;

		return copy_to_user(out, &fw_info,
				    min((size_t)size, sizeof(fw_info))) ? -EFAULT : 0;
	}
	case GSGPU_INFO_NUM_BYTES_MOVED:
		ui64 = atomic64_read(&adev->num_bytes_moved);
		return copy_to_user(out, &ui64, min(size, 8u)) ? -EFAULT : 0;
	case GSGPU_INFO_NUM_EVICTIONS:
		ui64 = atomic64_read(&adev->num_evictions);
		return copy_to_user(out, &ui64, min(size, 8u)) ? -EFAULT : 0;
	case GSGPU_INFO_NUM_VRAM_CPU_PAGE_FAULTS:
		ui64 = atomic64_read(&adev->num_vram_cpu_page_faults);
		return copy_to_user(out, &ui64, min(size, 8u)) ? -EFAULT : 0;
	case GSGPU_INFO_VRAM_USAGE:
		ui64 = ttm_resource_manager_usage(&adev->mman.vram_mgr.manager);
		return copy_to_user(out, &ui64, min(size, 8u)) ? -EFAULT : 0;
	case GSGPU_INFO_VIS_VRAM_USAGE:
		ui64 = gsgpu_vram_mgr_vis_usage(&adev->mman.vram_mgr);
		return copy_to_user(out, &ui64, min(size, 8u)) ? -EFAULT : 0;
	case GSGPU_INFO_GTT_USAGE:
		ui64 = ttm_resource_manager_usage(&adev->mman.gtt_mgr.manager);
		return copy_to_user(out, &ui64, min(size, 8u)) ? -EFAULT : 0;
	case GSGPU_INFO_GDS_CONFIG:
		return -ENODATA;
	case GSGPU_INFO_VRAM_GTT: {
		struct drm_gsgpu_info_vram_gtt vram_gtt;

		vram_gtt.vram_size = adev->gmc.real_vram_size -
			atomic64_read(&adev->vram_pin_size) -
			GSGPU_VM_RESERVED_VRAM;
		vram_gtt.vram_cpu_accessible_size =
			min(adev->gmc.visible_vram_size -
			    atomic64_read(&adev->visible_pin_size),
			    vram_gtt.vram_size);
		vram_gtt.gtt_size = ttm_manager_type(&adev->mman.bdev, TTM_PL_TT)->size;
		vram_gtt.gtt_size -= atomic64_read(&adev->gart_pin_size);
		return copy_to_user(out, &vram_gtt,
				    min((size_t)size, sizeof(vram_gtt))) ? -EFAULT : 0;
	}
	case GSGPU_INFO_MEMORY: {
		struct drm_gsgpu_memory_info mem;
		struct ttm_resource_manager *gtt_man =
			&adev->mman.gtt_mgr.manager;
		struct ttm_resource_manager *vram_man =
			&adev->mman.vram_mgr.manager;

		memset(&mem, 0, sizeof(mem));
		mem.vram.total_heap_size = adev->gmc.real_vram_size;
		mem.vram.usable_heap_size = adev->gmc.real_vram_size -
			atomic64_read(&adev->vram_pin_size) -
			GSGPU_VM_RESERVED_VRAM;
		mem.vram.heap_usage =
			ttm_resource_manager_usage(vram_man);
		mem.vram.max_allocation = mem.vram.usable_heap_size * 3 / 4;

		mem.cpu_accessible_vram.total_heap_size =
			adev->gmc.visible_vram_size;
		mem.cpu_accessible_vram.usable_heap_size =
			min(adev->gmc.visible_vram_size -
			    atomic64_read(&adev->visible_pin_size),
			    mem.vram.usable_heap_size);
		mem.cpu_accessible_vram.heap_usage =
			gsgpu_vram_mgr_vis_usage(&adev->mman.vram_mgr);
		mem.cpu_accessible_vram.max_allocation =
			mem.cpu_accessible_vram.usable_heap_size * 3 / 4;

		mem.gtt.total_heap_size = gtt_man->size;
		mem.gtt.usable_heap_size = mem.gtt.total_heap_size -
			atomic64_read(&adev->gart_pin_size);
		mem.gtt.heap_usage = ttm_resource_manager_usage(gtt_man);
		mem.gtt.max_allocation = mem.gtt.usable_heap_size * 3 / 4;

		return copy_to_user(out, &mem,
				    min((size_t)size, sizeof(mem)))
				    ? -EFAULT : 0;
	}
	case GSGPU_INFO_READ_MMR_REG: {
		unsigned n, alloc_size;
		uint32_t *regs;
		unsigned se_num = (info->read_mmr_reg.instance >>
				   GSGPU_INFO_MMR_SE_INDEX_SHIFT) &
				  GSGPU_INFO_MMR_SE_INDEX_MASK;
		unsigned sh_num = (info->read_mmr_reg.instance >>
				   GSGPU_INFO_MMR_SH_INDEX_SHIFT) &
				  GSGPU_INFO_MMR_SH_INDEX_MASK;

		/* set full masks if the userspace set all bits
		 * in the bitfields */
		if (se_num == GSGPU_INFO_MMR_SE_INDEX_MASK)
			se_num = 0xffffffff;
		else if (se_num >= GSGPU_GFX_MAX_SE)
			return -EINVAL;
		if (sh_num == GSGPU_INFO_MMR_SH_INDEX_MASK)
			sh_num = 0xffffffff;
		else if (sh_num >= GSGPU_GFX_MAX_SH_PER_SE)
			return -EINVAL;

		if (info->read_mmr_reg.count > 128)
			return -EINVAL;

		regs = kmalloc_array(info->read_mmr_reg.count, sizeof(*regs), GFP_KERNEL);
		if (!regs)
			return -ENOMEM;
		alloc_size = info->read_mmr_reg.count * sizeof(*regs);

		gsgpu_gfx_off_ctrl(adev, false);
		for (i = 0; i < info->read_mmr_reg.count; i++) {
			if (gsgpu_asic_read_register(adev, se_num, sh_num,
						      info->read_mmr_reg.dword_offset + i,
						      &regs[i])) {
				DRM_DEBUG_KMS("unallowed offset %#x\n",
					      info->read_mmr_reg.dword_offset + i);
				kfree(regs);
				gsgpu_gfx_off_ctrl(adev, true);
				return -EFAULT;
			}
		}
		gsgpu_gfx_off_ctrl(adev, true);
		n = copy_to_user(out, regs, min(size, alloc_size));
		kfree(regs);
		return n ? -EFAULT : 0;
	}
	case GSGPU_INFO_DEV_INFO: {
		struct drm_gsgpu_info_device *dev_info;
		uint64_t vm_size;
		// uint32_t pcie_gen_mask;
		int ret;

		dev_info = kzalloc(sizeof(*dev_info), GFP_KERNEL);
		if (!dev_info)
			return -ENOMEM;

		dev_info->device_id = adev->pdev->device;
		dev_info->pci_rev = adev->pdev->revision;
		dev_info->family = adev->family;
		dev_info->num_shader_engines = adev->gfx.config.max_shader_engines;
		dev_info->num_shader_arrays_per_engine = adev->gfx.config.max_sh_per_se;
		/* return all clocks in KHz */
		dev_info->gpu_counter_freq = gsgpu_asic_get_clk(adev) * 10;

			dev_info->max_engine_clock =
				dev_info->min_engine_clock =
					adev->clock.default_sclk * 10;
			dev_info->max_memory_clock =
				dev_info->min_memory_clock =
					adev->clock.default_mclk * 10;

		dev_info->enabled_rb_pipes_mask = adev->gfx.config.backend_enable_mask;
		dev_info->num_rb_pipes = adev->gfx.config.max_backends_per_se *
			adev->gfx.config.max_shader_engines;
		dev_info->num_hw_gfx_contexts = adev->gfx.config.max_hw_contexts;
		dev_info->ids_flags = 0;
		if (adev->flags & GSGPU_IS_APU)
			dev_info->ids_flags |= GSGPU_IDS_FLAGS_FUSION;
		if (adev->gfx.config.ta_cntl2_truncate_coord_mode)
			dev_info->ids_flags |= GSGPU_IDS_FLAGS_CONFORMANT_TRUNC_COORD;

		vm_size = adev->vm_manager.max_pfn * GSGPU_GPU_PAGE_SIZE;
		vm_size -= GSGPU_VA_RESERVED_SIZE;

		dev_info->virtual_address_offset = GSGPU_VA_RESERVED_SIZE;
		dev_info->virtual_address_max =
			min(vm_size, GSGPU_GMC_HOLE_START);

		if (vm_size > GSGPU_GMC_HOLE_START) {
			dev_info->high_va_offset = GSGPU_GMC_HOLE_END;
			dev_info->high_va_max = GSGPU_GMC_HOLE_END | vm_size;
		}
		dev_info->virtual_address_alignment = max_t(u32, PAGE_SIZE, GSGPU_GPU_PAGE_SIZE);
		dev_info->pte_fragment_size = (1 << adev->vm_manager.fragment_size) * GSGPU_GPU_PAGE_SIZE;
		dev_info->gart_page_size = max_t(u32, PAGE_SIZE, GSGPU_GPU_PAGE_SIZE);
		dev_info->cu_active_number = adev->gfx.cu_info.number;
		dev_info->cu_ao_mask = adev->gfx.cu_info.ao_cu_mask;
		dev_info->ce_ram_size = adev->gfx.ce_ram_size;
		memcpy(&dev_info->cu_ao_bitmap[0], &adev->gfx.cu_info.ao_cu_bitmap[0],
		       sizeof(adev->gfx.cu_info.ao_cu_bitmap));
		memcpy(&dev_info->cu_bitmap[0], &adev->gfx.cu_info.bitmap[0],
		       sizeof(adev->gfx.cu_info.bitmap));
		dev_info->vram_type = adev->gmc.vram_type;
		dev_info->vram_bit_width = adev->gmc.vram_width;
		// dev_info->vce_harvest_config = adev->vce.harvest_config;
		dev_info->gc_double_offchip_lds_buf =
			adev->gfx.config.double_offchip_lds_buf;
		dev_info->wave_front_size = adev->gfx.cu_info.wave_front_size;
		dev_info->num_shader_visible_vgprs = adev->gfx.config.max_gprs;
		dev_info->num_cu_per_sh = adev->gfx.config.max_cu_per_sh;
		dev_info->num_tcc_blocks = adev->gfx.config.max_texture_channel_caches;
		dev_info->gs_vgt_table_depth = adev->gfx.config.gs_vgt_table_depth;
		dev_info->gs_prim_buffer_depth = adev->gfx.config.gs_prim_buffer_depth;
		dev_info->max_gs_waves_per_vgt = adev->gfx.config.max_gs_threads;

		if (adev->family >= GSGPU_FAMILY_NV)
			dev_info->pa_sc_tile_steering_override =
				adev->gfx.config.pa_sc_tile_steering_override;

		dev_info->tcc_disabled_mask = adev->gfx.config.tcc_disabled_mask;

		/* Combine the chip gen mask with the platform (CPU/mobo) mask. */
		dev_info->pcie_gen = 0;
		dev_info->pcie_num_lanes = 0;

		dev_info->tcp_cache_size = adev->gfx.config.gc_tcp_l1_size;
		dev_info->num_sqc_per_wgp = adev->gfx.config.gc_num_sqc_per_wgp;
		dev_info->sqc_data_cache_size = adev->gfx.config.gc_l1_data_cache_size_per_sqc;
		dev_info->sqc_inst_cache_size = adev->gfx.config.gc_l1_instruction_cache_size_per_sqc;
		dev_info->gl1c_cache_size = adev->gfx.config.gc_gl1c_size_per_instance *
					    adev->gfx.config.gc_gl1c_per_sa;
		dev_info->gl2c_cache_size = adev->gfx.config.gc_gl2c_per_gpu;
		dev_info->mall_size = adev->gmc.mall_size;

		ret = copy_to_user(out, dev_info,
				   min((size_t)size, sizeof(*dev_info))) ? -EFAULT : 0;
		kfree(dev_info);
		return ret;
	}
	case GSGPU_INFO_VCE_CLOCK_TABLE: {
		return -ENODATA;
	}
	case GSGPU_INFO_VBIOS: {
		uint32_t bios_size = adev->bios_size;

		switch (info->vbios_info.type) {
		case GSGPU_INFO_VBIOS_SIZE:
			return copy_to_user(out, &bios_size,
					min((size_t)size, sizeof(bios_size)))
					? -EFAULT : 0;
		case GSGPU_INFO_VBIOS_IMAGE: {
			uint8_t *bios;
			uint32_t bios_offset = info->vbios_info.offset;

			if (bios_offset >= bios_size)
				return -EINVAL;

			bios = adev->bios + bios_offset;
			return copy_to_user(out, bios,
					    min((size_t)size, (size_t)(bios_size - bios_offset)))
					? -EFAULT : 0;
		}
		case GSGPU_INFO_VBIOS_INFO: {
			struct drm_gsgpu_info_vbios vbios_info = {};
			return copy_to_user(out, &vbios_info,
						min((size_t)size, sizeof(vbios_info))) ? -EFAULT : 0;
		}
		default:
			DRM_DEBUG_KMS("Invalid request %d\n",
					info->vbios_info.type);
			return -EINVAL;
		}
	}
	case GSGPU_INFO_NUM_HANDLES: {
		return -EINVAL;
	}
	case GSGPU_INFO_SENSOR: {
		return -ENOENT;
	}
	case GSGPU_INFO_VRAM_LOST_COUNTER:
		ui32 = atomic_read(&adev->vram_lost_counter);
		return copy_to_user(out, &ui32, min(size, 4u)) ? -EFAULT : 0;
	case GSGPU_INFO_VIDEO_CAPS: {
		const struct gsgpu_video_codecs *codecs;
		struct drm_gsgpu_info_video_caps *caps;
		int r;

		switch (info->video_cap.type) {
		case GSGPU_INFO_VIDEO_CAPS_DECODE:
		case GSGPU_INFO_VIDEO_CAPS_ENCODE:
		default:
			DRM_DEBUG_KMS("Invalid request %d\n",
				      info->video_cap.type);
			return -EINVAL;
		}

		caps = kzalloc(sizeof(*caps), GFP_KERNEL);
		if (!caps)
			return -ENOMEM;

		for (i = 0; i < codecs->codec_count; i++) {
			int idx = codecs->codec_array[i].codec_type;

			switch (idx) {
			case GSGPU_INFO_VIDEO_CAPS_CODEC_IDX_MPEG2:
			case GSGPU_INFO_VIDEO_CAPS_CODEC_IDX_MPEG4:
			case GSGPU_INFO_VIDEO_CAPS_CODEC_IDX_VC1:
			case GSGPU_INFO_VIDEO_CAPS_CODEC_IDX_MPEG4_AVC:
			case GSGPU_INFO_VIDEO_CAPS_CODEC_IDX_HEVC:
			case GSGPU_INFO_VIDEO_CAPS_CODEC_IDX_JPEG:
			case GSGPU_INFO_VIDEO_CAPS_CODEC_IDX_VP9:
			case GSGPU_INFO_VIDEO_CAPS_CODEC_IDX_AV1:
				caps->codec_info[idx].valid = 1;
				caps->codec_info[idx].max_width =
					codecs->codec_array[i].max_width;
				caps->codec_info[idx].max_height =
					codecs->codec_array[i].max_height;
				caps->codec_info[idx].max_pixels_per_frame =
					codecs->codec_array[i].max_pixels_per_frame;
				caps->codec_info[idx].max_level =
					codecs->codec_array[i].max_level;
				break;
			default:
				break;
			}
		}
		r = copy_to_user(out, caps,
				 min((size_t)size, sizeof(*caps))) ? -EFAULT : 0;
		kfree(caps);
		return r;
	}
	default:
		DRM_DEBUG_KMS("Invalid request %d\n", info->query);
		return -EINVAL;
	}
	return 0;
}


/*
 * Outdated mess for old drm with Xorg being in charge (void function now).
 */
/**
 * gsgpu_driver_lastclose_kms - drm callback for last close
 *
 * @dev: drm dev pointer
 *
 * Switch vga_switcheroo state after last close (all asics).
 */
void gsgpu_driver_lastclose_kms(struct drm_device *dev)
{
	drm_fb_helper_lastclose(dev);
	vga_switcheroo_process_delayed_switch();
}

/**
 * gsgpu_driver_open_kms - drm callback for open
 *
 * @dev: drm dev pointer
 * @file_priv: drm file
 *
 * On device open, init vm on cayman+ (all asics).
 * Returns 0 on success, error on failure.
 */
int gsgpu_driver_open_kms(struct drm_device *dev, struct drm_file *file_priv)
{
	struct gsgpu_device *adev = drm_to_adev(dev);
	struct gsgpu_fpriv *fpriv;
	int r, pasid;

	/* Ensure IB tests are run on ring */
	flush_delayed_work(&adev->delayed_init_work);

	file_priv->driver_priv = NULL;

	r = pm_runtime_get_sync(dev->dev);
	if (r < 0)
		goto pm_put;

	fpriv = kzalloc(sizeof(*fpriv), GFP_KERNEL);
	if (unlikely(!fpriv)) {
		r = -ENOMEM;
		goto out_suspend;
	}

	pasid = gsgpu_pasid_alloc(16);
	if (pasid < 0) {
		dev_warn(adev->dev, "No more PASIDs available!");
		pasid = 0;
	}

	r = gsgpu_vm_init(adev, &fpriv->vm);
	if (r)
		goto error_pasid;

	r = gsgpu_vm_set_pasid(adev, &fpriv->vm, pasid);
	if (r)
		goto error_vm;

	fpriv->prt_va = gsgpu_vm_bo_add(adev, &fpriv->vm, NULL);
	if (!fpriv->prt_va) {
		r = -ENOMEM;
		goto error_vm;
	}

	mutex_init(&fpriv->bo_list_lock);
	idr_init_base(&fpriv->bo_list_handles, 1);

	gsgpu_ctx_mgr_init(&fpriv->ctx_mgr, adev);

	file_priv->driver_priv = fpriv;
	goto out_suspend;

error_vm:
	gsgpu_vm_fini(adev, &fpriv->vm);

error_pasid:
	if (pasid) {
		gsgpu_pasid_free(pasid);
		gsgpu_vm_set_pasid(adev, &fpriv->vm, 0);
	}

	kfree(fpriv);

out_suspend:
	pm_runtime_mark_last_busy(dev->dev);
pm_put:
	pm_runtime_put_autosuspend(dev->dev);

	return r;
}

/**
 * gsgpu_driver_postclose_kms - drm callback for post close
 *
 * @dev: drm dev pointer
 * @file_priv: drm file
 *
 * On device post close, tear down vm on cayman+ (all asics).
 */
void gsgpu_driver_postclose_kms(struct drm_device *dev,
				 struct drm_file *file_priv)
{
	struct gsgpu_device *adev = drm_to_adev(dev);
	struct gsgpu_fpriv *fpriv = file_priv->driver_priv;
	struct gsgpu_bo_list *list;
	struct gsgpu_bo *pd;
	u32 pasid;
	int handle;

	if (!fpriv)
		return;

	pm_runtime_get_sync(dev->dev);

	pasid = fpriv->vm.pasid;
	pd = gsgpu_bo_ref(fpriv->vm.root.bo);
	if (!WARN_ON(gsgpu_bo_reserve(pd, true))) {
		gsgpu_vm_bo_del(adev, fpriv->prt_va);
		gsgpu_bo_unreserve(pd);
	}

	gsgpu_ctx_mgr_fini(&fpriv->ctx_mgr);
	gsgpu_vm_fini(adev, &fpriv->vm);

	if (pasid)
		gsgpu_pasid_free_delayed(pd->tbo.base.resv, pasid);
	gsgpu_bo_unref(&pd);

	idr_for_each_entry(&fpriv->bo_list_handles, list, handle)
		gsgpu_bo_list_put(list);

	idr_destroy(&fpriv->bo_list_handles);
	mutex_destroy(&fpriv->bo_list_lock);

	kfree(fpriv);
	file_priv->driver_priv = NULL;

	pm_runtime_mark_last_busy(dev->dev);
	pm_runtime_put_autosuspend(dev->dev);
}


void gsgpu_driver_release_kms(struct drm_device *dev)
{
	struct gsgpu_device *adev = drm_to_adev(dev);

	gsgpu_device_fini_sw(adev);
	pci_set_drvdata(adev->pdev, NULL);
}

/*
 * VBlank related functions.
 */
/**
 * gsgpu_get_vblank_counter_kms - get frame count
 *
 * @crtc: crtc to get the frame count from
 *
 * Gets the frame count on the requested crtc (all asics).
 * Returns frame count on success, -EINVAL on failure.
 */
u32 gsgpu_get_vblank_counter_kms(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	unsigned int pipe = crtc->index;
	struct gsgpu_device *adev = drm_to_adev(dev);
	int vpos, hpos, stat;
	u32 count;

	if (pipe >= adev->mode_info.num_crtc) {
		DRM_ERROR("Invalid crtc %u\n", pipe);
		return -EINVAL;
	}

	/* The hw increments its frame counter at start of vsync, not at start
	 * of vblank, as is required by DRM core vblank counter handling.
	 * Cook the hw count here to make it appear to the caller as if it
	 * incremented at start of vblank. We measure distance to start of
	 * vblank in vpos. vpos therefore will be >= 0 between start of vblank
	 * and start of vsync, so vpos >= 0 means to bump the hw frame counter
	 * result by 1 to give the proper appearance to caller.
	 */
	if (adev->mode_info.crtcs[pipe]) {
		/* Repeat readout if needed to provide stable result if
		 * we cross start of vsync during the queries.
		 */
		do {
			count = gsgpu_display_vblank_get_counter(adev, pipe);
			/* Ask gsgpu_display_get_crtc_scanoutpos to return
			 * vpos as distance to start of vblank, instead of
			 * regular vertical scanout pos.
			 */
			stat = gsgpu_display_get_crtc_scanoutpos(
				dev, pipe, GET_DISTANCE_TO_VBLANKSTART,
				&vpos, &hpos, NULL, NULL,
				&adev->mode_info.crtcs[pipe]->base.hwmode);
		} while (count != gsgpu_display_vblank_get_counter(adev, pipe));

		if (((stat & (DRM_SCANOUTPOS_VALID | DRM_SCANOUTPOS_ACCURATE)) !=
		    (DRM_SCANOUTPOS_VALID | DRM_SCANOUTPOS_ACCURATE))) {
			DRM_DEBUG_VBL("Query failed! stat %d\n", stat);
		} else {
			DRM_DEBUG_VBL("crtc %d: dist from vblank start %d\n",
				      pipe, vpos);

			/* Bump counter if we are at >= leading edge of vblank,
			 * but before vsync where vpos would turn negative and
			 * the hw counter really increments.
			 */
			if (vpos >= 0)
				count++;
		}
	} else {
		/* Fallback to use value as is. */
		count = gsgpu_display_vblank_get_counter(adev, pipe);
		DRM_DEBUG_VBL("NULL mode info! Returned count may be wrong.\n");
	}

	return count;
}
