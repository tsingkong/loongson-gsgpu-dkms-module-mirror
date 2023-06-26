/*
 * Copyright 2000 VA Linux Systems, Inc., Sunnyvale, California.
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * VA LINUX SYSTEMS AND/OR ITS SUPPLIERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include <drm/gsgpu_drm.h>
#include <drm/drm_drv.h>
#include <drm/drm_fbdev_generic.h>
#include <drm/drm_gem.h>
#include <drm/drm_vblank.h>
#include <drm/drm_managed.h>
#include "gsgpu_drv.h"

#include <drm/drm_pciids.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/vga_switcheroo.h>
#include <drm/drm_probe_helper.h>
#include <linux/mmu_notifier.h>
#include <linux/suspend.h>
#include <linux/cc_platform.h>
#include <linux/dynamic_debug.h>

#include "gsgpu.h"
#include "gsgpu_irq.h"
#include "gsgpu_dc_vbios.h"
#include "gsgpu_dma_buf.h"
#include "gsgpu_sched.h"
#include "gsgpu_gem.h"
#include "gsgpu_reset.h"

/*
 * KMS wrapper.
 * - 3.0.0 - initial driver
 * - 3.1.0 - allow reading more status registers (GRBM, SRBM, XDMA, CP)
 * - 3.2.0 - GFX8: Uses EOP_TC_WB_ACTION_EN, so UMDs don't have to do the same
 *           at the end of IBs.
 * - 3.3.0 - Add VM support for UVD on supported hardware.
 * - 3.4.0 - Add GSGPU_INFO_NUM_EVICTIONS.
 * - 3.5.0 - Add support for new UVD_NO_OP register.
 * - 3.6.0 - kmd involves use CONTEXT_CONTROL in ring buffer.
 * - 3.7.0 - Add support for VCE clock list packet
 * - 3.8.0 - Add support raster config init in the kernel
 * - 3.9.0 - Add support for memory query info about VRAM and GTT.
 * - 3.10.0 - Add support for new fences ioctl, new gem ioctl flags
 * - 3.11.0 - Add support for sensor query info (clocks, temp, etc).
 * - 3.12.0 - Add query for double offchip LDS buffers
 * - 3.13.0 - Add PRT support
 * - 3.14.0 - Fix race in gsgpu_ctx_get_fence() and note new functionality
 * - 3.15.0 - Export more gpu info for gfx9
 * - 3.16.0 - Add reserved vmid support
 * - 3.17.0 - Add GSGPU_NUM_VRAM_CPU_PAGE_FAULTS.
 * - 3.18.0 - Export gpu always on cu bitmap
 * - 3.19.0 - Add support for UVD MJPEG decode
 * - 3.20.0 - Add support for local BOs
 * - 3.21.0 - Add DRM_GSGPU_FENCE_TO_HANDLE ioctl
 * - 3.22.0 - Add DRM_GSGPU_SCHED ioctl
 * - 3.23.0 - Add query for VRAM lost counter
 * - 3.24.0 - Add high priority compute support for gfx9
 * - 3.25.0 - Add support for sensor query info (stable pstate sclk/mclk).
 * - 3.26.0 - GFX9: Process GSGPU_IB_FLAG_TC_WB_NOT_INVALIDATE.
 * - 3.27.0 - Add new chunk to GSGPU_CS to enable BO_LIST creation.
 * - 3.28.0 - Add GSGPU_CHUNK_ID_SCHEDULED_DEPENDENCIES
 * - 3.29.0 - Add GSGPU_IB_FLAG_RESET_GDS_MAX_WAVE_ID
 * - 3.30.0 - Add GSGPU_SCHED_OP_CONTEXT_PRIORITY_OVERRIDE.
 * - 3.31.0 - Add support for per-flip tiling attribute changes with DC
 * - 3.32.0 - Add syncobj timeline support to GSGPU_CS.
 * - 3.33.0 - Fixes for GDS ENOMEM failures in GSGPU_CS.
 * - 3.34.0 - Non-DC can flip correctly between buffers with different pitches
 * - 3.35.0 - Add drm_gsgpu_info_device::tcc_disabled_mask
 * - 3.36.0 - Allow reading more status registers on si/cik
 * - 3.37.0 - L2 is invalidated before SDMA IBs, needed for correctness
 * - 3.38.0 - Add GSGPU_IB_FLAG_EMIT_MEM_SYNC
 * - 3.39.0 - DMABUF implicit sync does a full pipeline sync
 * - 3.40.0 - Add GSGPU_IDS_FLAGS_TMZ
 * - 3.41.0 - Add video codec query
 * - 3.42.0 - Add 16bpc fixed point display support
 * - 3.43.0 - Add device hot plug/unplug support
 * - 3.44.0 - DCN3 supports DCC independent block settings: !64B && 128B, 64B && 128B
 * - 3.45.0 - Add context ioctl stable pstate interface
 * - 3.46.0 - To enable hot plug gsgpu tests in libdrm
 * - 3.47.0 - Add GSGPU_GEM_CREATE_DISCARDABLE and GSGPU_VM_NOALLOC flags
 * - 3.48.0 - Add IP discovery version info to HW INFO
 * - 3.49.0 - Add gang submit into CS IOCTL
 * - 3.50.0 - Update GSGPU_INFO_DEV_INFO IOCTL for minimum engine and memory clock
 *            Update GSGPU_INFO_SENSOR IOCTL for PEAK_PSTATE engine and memory clock
 *   3.51.0 - Return the PCIe gen and lanes from the INFO ioctl
 *   3.52.0 - Add GSGPU_IDS_FLAGS_CONFORMANT_TRUNC_COORD, add device_info fields:
 *            tcp_cache_size, num_sqc_per_wgp, sqc_data_cache_size, sqc_inst_cache_size,
 *            gl1c_cache_size, gl2c_cache_size, mall_size, enabled_rb_pipes_mask_hi
 */
#define KMS_DRIVER_MAJOR	0
#define KMS_DRIVER_MINOR	1
#define KMS_DRIVER_PATCHLEVEL	0

unsigned int gsgpu_vram_limit = UINT_MAX;
int gsgpu_vis_vram_limit = 0;
int gsgpu_gart_size = -1; /* auto */
int gsgpu_gtt_size = -1; /* auto */
int gsgpu_moverate = -1; /* auto */
int gsgpu_disp_priority;
int gsgpu_msi = -1;
char gsgpu_lockup_timeout[GSGPU_MAX_TIMEOUT_PARAM_LENGTH];
int gsgpu_runtime_pm = -1;
int gsgpu_vm_size = -1;
int gsgpu_vm_block_size = -1;
int gsgpu_vm_fault_stop;
int gsgpu_vm_debug;
int gsgpu_vm_update_mode = -1;
int gsgpu_exp_hw_support;
int gsgpu_sched_jobs = 32;
int gsgpu_sched_hw_submission = 2;
uint gsgpu_force_long_training;
int gsgpu_job_hang_limit;
int gsgpu_using_ram = 0; /* using system  memory for gpu*/
int gsgpu_noretry = -1;

static void gsgpu_drv_delayed_reset_work_handler(struct work_struct *work);

DECLARE_DYNDBG_CLASSMAP(drm_debug_classes, DD_CLASS_TYPE_DISJOINT_BITS, 0,
			"DRM_UT_CORE",
			"DRM_UT_DRIVER",
			"DRM_UT_KMS",
			"DRM_UT_PRIME",
			"DRM_UT_ATOMIC",
			"DRM_UT_VBL",
			"DRM_UT_STATE",
			"DRM_UT_LEASE",
			"DRM_UT_DP",
			"DRM_UT_DRMRES");

struct gsgpu_mgpu_info mgpu_info = {
	.mutex = __MUTEX_INITIALIZER(mgpu_info.mutex),
	.delayed_reset_work = __DELAYED_WORK_INITIALIZER(
			mgpu_info.delayed_reset_work,
			gsgpu_drv_delayed_reset_work_handler, 0),
};

int gsgpu_bad_page_threshold = -1;
struct gsgpu_watchdog_timer gsgpu_watchdog_timer = {
	.timeout_fatal_disable = false,
	.period = 0x0, /* default to 0x0 (timeout disable) */
};

int gsgpu_lg100_support = 1;
MODULE_PARM_DESC(LG100_support, "LG100 support (1 = enabled (default), 0 = disabled");
module_param_named(LG100_support, gsgpu_lg100_support, int, 0444);

/**
 * DOC: vramlimit (int)
 * Restrict the total amount of VRAM in MiB for testing.  The default is 0 (Use full VRAM).
 */
MODULE_PARM_DESC(vramlimit, "Restrict VRAM for testing, in megabytes");
module_param_named(vramlimit, gsgpu_vram_limit, int, 0600);

/**
 * DOC: vis_vramlimit (int)
 * Restrict the amount of CPU visible VRAM in MiB for testing.  The default is 0 (Use full CPU visible VRAM).
 */
MODULE_PARM_DESC(vis_vramlimit, "Restrict visible VRAM for testing, in megabytes");
module_param_named(vis_vramlimit, gsgpu_vis_vram_limit, int, 0444);

/**
 * DOC: gartsize (uint)
 * Restrict the size of GART (for kernel use) in Mib (32, 64, etc.) for testing.
 * The default is -1 (The size depends on asic).
 */
MODULE_PARM_DESC(gartsize, "Size of kernel GART to setup in megabytes (32, 64, etc., -1=auto)");
module_param_named(gartsize, gsgpu_gart_size, uint, 0600);

/**
 * DOC: gttsize (int)
 * Restrict the size of GTT domain (for userspace use) in MiB for testing.
 * The default is -1 (Use 1/2 RAM, minimum value is 3GB).
 */
MODULE_PARM_DESC(gttsize, "Size of the GTT userspace domain in megabytes (-1 = auto)");
module_param_named(gttsize, gsgpu_gtt_size, int, 0600);

/**
 * DOC: moverate (int)
 * Set maximum buffer migration rate in MB/s. The default is -1 (8 MB/s).
 */
MODULE_PARM_DESC(moverate, "Maximum buffer migration rate in MB/s. (32, 64, etc., -1=auto, 0=1=disabled)");
module_param_named(moverate, gsgpu_moverate, int, 0600);

/**
 * DOC: disp_priority (int)
 * Set display Priority (1 = normal, 2 = high). Only affects non-DC display handling. The default is 0 (auto).
 */
MODULE_PARM_DESC(disp_priority, "Display Priority (0 = auto, 1 = normal, 2 = high)");
module_param_named(disp_priority, gsgpu_disp_priority, int, 0444);

/**
 * DOC: msi (int)
 * To disable Message Signaled Interrupts (MSI) functionality (1 = enable, 0 = disable). The default is -1 (auto, enabled).
 */
MODULE_PARM_DESC(msi, "MSI support (1 = enable, 0 = disable, -1 = auto)");
module_param_named(msi, gsgpu_msi, int, 0444);

/**
 * DOC: lockup_timeout (string)
 * Set GPU scheduler timeout value in ms.
 *
 * The format can be [Non-Compute] or [GFX,Compute,SDMA,Video]. That is there can be one or
 * multiple values specified. 0 and negative values are invalidated. They will be adjusted
 * to the default timeout.
 *
 * - With one value specified, the setting will apply to all non-compute jobs.
 * - With multiple values specified, the first one will be for GFX.
 *   The second one is for Compute. The third and fourth ones are
 *   for SDMA and Video.
 *
 * By default(with no lockup_timeout settings), the timeout for all non-compute(GFX, SDMA and Video)
 * jobs is 10000. The timeout for compute is 60000.
 */
MODULE_PARM_DESC(lockup_timeout, "GPU lockup timeout in ms (default: for bare metal 10000 for non-compute jobs and 60000 for compute jobs; "
		"for passthrough or sriov, 10000 for all jobs."
		" 0: keep default value. negative: infinity timeout), "
		"format: for bare metal [Non-Compute] or [GFX,Compute,SDMA,Video]; "
		"for passthrough or sriov [all jobs] or [GFX,Compute,SDMA,Video].");
module_param_string(lockup_timeout, gsgpu_lockup_timeout, sizeof(gsgpu_lockup_timeout), 0444);

/**
 * DOC: runpm (int)
 * Override for runtime power management control for dGPUs in PX/HG laptops. The gsgpu driver can dynamically power down
 * the dGPU on PX/HG laptops when it is idle. The default is -1 (auto enable). Setting the value to 0 disables this functionality.
 */
MODULE_PARM_DESC(runpm, "PX runtime pm (1 = force enable, 0 = disable, -1 = PX only default)");
module_param_named(runpm, gsgpu_runtime_pm, int, 0444);

/**
 * DOC: vm_size (int)
 * Override the size of the GPU's per client virtual address space in GiB.  The default is -1 (automatic for each asic).
 */
MODULE_PARM_DESC(vm_size, "VM address space size in gigabytes (default 64GB)");
module_param_named(vm_size, gsgpu_vm_size, int, 0444);

/**
 * DOC: vm_block_size (int)
 * Override VM page table size in bits (default depending on vm_size and hw setup). The default is -1 (automatic for each asic).
 */
MODULE_PARM_DESC(vm_block_size, "VM page table size in bits (default depending on vm_size)");
module_param_named(vm_block_size, gsgpu_vm_block_size, int, 0444);

/**
 * DOC: vm_fault_stop (int)
 * Stop on VM fault for debugging (0 = never, 1 = print first, 2 = always). The default is 0 (No stop).
 */
MODULE_PARM_DESC(vm_fault_stop, "Stop on VM fault (0 = never (default), 1 = print first, 2 = always)");
module_param_named(vm_fault_stop, gsgpu_vm_fault_stop, int, 0444);

/**
 * DOC: vm_debug (int)
 * Debug VM handling (0 = disabled, 1 = enabled). The default is 0 (Disabled).
 */
MODULE_PARM_DESC(vm_debug, "Debug VM handling (0 = disabled (default), 1 = enabled)");
module_param_named(vm_debug, gsgpu_vm_debug, int, 0644);

/**
 * DOC: vm_update_mode (int)
 * Override VM update mode. VM updated by using CPU (0 = never, 1 = Graphics only, 2 = Compute only, 3 = Both). The default
 * is -1 (Only in large BAR(LB) systems Compute VM tables will be updated by CPU, otherwise 0, never).
 */
MODULE_PARM_DESC(vm_update_mode, "VM update using CPU (0 = never (default except for large BAR(LB)), 1 = Graphics only, 2 = Compute only (default for LB), 3 = Both");
module_param_named(vm_update_mode, gsgpu_vm_update_mode, int, 0444);

/**
 * DOC: exp_hw_support (int)
 * Enable experimental hw support (1 = enable). The default is 0 (disabled).
 */
MODULE_PARM_DESC(exp_hw_support, "experimental hw support (1 = enable, 0 = disable (default))");
module_param_named(exp_hw_support, gsgpu_exp_hw_support, int, 0444);

/**
 * DOC: sched_jobs (int)
 * Override the max number of jobs supported in the sw queue. The default is 32.
 */
MODULE_PARM_DESC(sched_jobs, "the max number of jobs supported in the sw queue (default 32)");
module_param_named(sched_jobs, gsgpu_sched_jobs, int, 0444);

/**
 * DOC: sched_hw_submission (int)
 * Override the max number of HW submissions. The default is 2.
 */
MODULE_PARM_DESC(sched_hw_submission, "the max number of HW submissions (default 2)");
module_param_named(sched_hw_submission, gsgpu_sched_hw_submission, int, 0444);

/**
 * DOC: job_hang_limit (int)
 * Set how much time allow a job hang and not drop it. The default is 0.
 */
MODULE_PARM_DESC(job_hang_limit, "how much time allow a job hang and not drop it (default 0)");
module_param_named(job_hang_limit, gsgpu_job_hang_limit, int ,0444);

MODULE_PARM_DESC(gsgpu_using_ram,"Gpu uses memory instead vram"
		 "0: using vram for gpu 1:use system for gpu");
module_param_named(gsgpu_using_ram, gsgpu_using_ram, uint, 0444);

/**
 * DOC: noretry (int)
 * Disable XNACK retry in the SQ by default on GFXv9 hardware. On ASICs that
 * do not support per-process XNACK this also disables retry page faults.
 * (0 = retry enabled, 1 = retry disabled, -1 auto (default))
 */
MODULE_PARM_DESC(noretry,
	"Disable retry faults (0 = retry enabled, 1 = retry disabled, -1 auto (default))");
module_param_named(noretry, gsgpu_noretry, int, 0644);


static const struct pci_device_id pciidlist[] = {
	{PCI_VENDOR_ID_LOONGSON, 0x7A25, PCI_ANY_ID, PCI_ANY_ID, 0, 0, CHIP_LG100}, //GSGPU
	{0, 0, 0}
};

MODULE_DEVICE_TABLE(pci, pciidlist);

static const struct drm_driver gsgpu_kms_driver;

static int gsgpu_pci_probe(struct pci_dev *pdev,
			    const struct pci_device_id *ent)
{
	struct drm_device *ddev;
	struct gsgpu_device *adev;
	unsigned long flags = ent->driver_data;
	int ret, retry = 0;

	if ((flags & GSGPU_EXP_HW_SUPPORT) && !gsgpu_exp_hw_support) {
		DRM_INFO("This hardware requires experimental hardware support.\n"
			 "See modparam exp_hw_support\n");
		return -ENODEV;
	}

	adev = devm_drm_dev_alloc(&pdev->dev, &gsgpu_kms_driver, typeof(*adev), ddev);
	if (IS_ERR(adev))
		return PTR_ERR(adev);

	adev->dev  = &pdev->dev;
	adev->pdev = pdev;
	ddev = adev_to_drm(adev);

	ret = pci_enable_device(pdev);
	if (ret)
		return ret;

	pci_set_drvdata(pdev, ddev);

	ret = gsgpu_driver_load_kms(adev, flags);
	if (ret)
		goto err_pci;

retry_init:
	ret = drm_dev_register(ddev, flags);
	if (ret == -EAGAIN && ++retry <= 3) {
		DRM_INFO("retry init %d\n", retry);
		/* Don't request EX mode too frequently which is attacking */
		msleep(5000);
		goto retry_init;
	} else if (ret) {
		goto err_pci;
	}

	/*
	 * 1. don't init fbdev on hw without DCE
	 * 2. don't init fbdev if there are no connectors
	 */
	if (adev->mode_info.mode_config_initialized &&
	    !list_empty(&adev_to_drm(adev)->mode_config.connector_list)) {
		/* select 8 bpp console on low vram cards */
		if (adev->gmc.real_vram_size <= (32*1024*1024))
			drm_fbdev_generic_setup(adev_to_drm(adev), 8);
		else
			drm_fbdev_generic_setup(adev_to_drm(adev), 32);
	}

	ret = gsgpu_debugfs_init(adev);
	if (ret)
		DRM_ERROR("Creating debugfs files failed (%d).\n", ret);

	return 0;

err_pci:
	pci_disable_device(pdev);
	return ret;
}

static void
gsgpu_pci_remove(struct pci_dev *pdev)
{
	struct drm_device *dev = pci_get_drvdata(pdev);
	// struct gsgpu_device *adev = drm_to_adev(dev);

	drm_dev_unplug(dev);

	gsgpu_driver_unload_kms(dev);

	/*
	 * Flush any in flight DMA operations from device.
	 * Clear the Bus Master Enable bit and then wait on the PCIe Device
	 * StatusTransactions Pending bit.
	 */
	pci_disable_device(pdev);
	pci_wait_for_pending_transaction(pdev);
}

static void
gsgpu_pci_shutdown(struct pci_dev *pdev)
{
	struct drm_device *dev = pci_get_drvdata(pdev);
	struct gsgpu_device *adev = drm_to_adev(dev);

	/* if we are running in a VM, make sure the device
	 * torn down properly on reboot/shutdown.
	 * unfortunately we can't detect certain
	 * hypervisors so just do this all the time.
	 */
	adev->mp1_state = PP_MP1_STATE_UNLOAD;
	gsgpu_device_ip_suspend(adev);
	adev->mp1_state = PP_MP1_STATE_NONE;
}

/**
 * gsgpu_drv_delayed_reset_work_handler - work handler for reset
 *
 * @work: work_struct.
 */
static void gsgpu_drv_delayed_reset_work_handler(struct work_struct *work)
{
	struct list_head device_list;
	struct gsgpu_device *adev;
	int i, r;
	struct gsgpu_reset_context reset_context;

	memset(&reset_context, 0, sizeof(reset_context));

	mutex_lock(&mgpu_info.mutex);
	if (mgpu_info.pending_reset == true) {
		mutex_unlock(&mgpu_info.mutex);
		return;
	}
	mgpu_info.pending_reset = true;
	mutex_unlock(&mgpu_info.mutex);

	/* Use a common context, just need to make sure full reset is done */
	reset_context.method = GSGPU_RESET_METHOD_NONE;
	set_bit(GSGPU_NEED_FULL_RESET, &reset_context.flags);

	for (i = 0; i < mgpu_info.num_dgpu; i++) {
		adev = mgpu_info.gpu_ins[i].adev;
		reset_context.reset_req_dev = adev;
		r = gsgpu_device_pre_asic_reset(adev, &reset_context);
		if (r) {
			dev_err(adev->dev, "GPU pre asic reset failed with err, %d for drm dev, %s ",
				r, adev_to_drm(adev)->unique);
		}
	}

	INIT_LIST_HEAD(&device_list);

	for (i = 0; i < mgpu_info.num_dgpu; i++)
		list_add_tail(&mgpu_info.gpu_ins[i].adev->reset_list, &device_list);

	/* unregister the GPU first, reset function will add them back */
	list_for_each_entry(adev, &device_list, reset_list)
		gsgpu_unregister_gpu_instance(adev);

	/* Use a common context, just need to make sure full reset is done */
	set_bit(GSGPU_SKIP_HW_RESET, &reset_context.flags);
	r = gsgpu_do_asic_reset(&device_list, &reset_context);

	if (r) {
		DRM_ERROR("reinit gpus failure");
		return;
	}
	for (i = 0; i < mgpu_info.num_dgpu; i++) {
		adev = mgpu_info.gpu_ins[i].adev;
		gsgpu_ttm_set_buffer_funcs_status(adev, true);
	}
	return;
}

static int gsgpu_pmops_prepare(struct device *dev)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);
	struct gsgpu_device *adev = drm_to_adev(drm_dev);

	/* if we will not support s3 or s2i for the device
	 *  then skip suspend
	 */
	if (!gsgpu_acpi_is_s0ix_active(adev) &&
	    !gsgpu_acpi_is_s3_active(adev))
		return 1;

	return 0;
}

static void gsgpu_pmops_complete(struct device *dev)
{
	/* nothing to do */
}

static int gsgpu_pmops_suspend(struct device *dev)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);
	struct gsgpu_device *adev = drm_to_adev(drm_dev);

	if (gsgpu_acpi_is_s0ix_active(adev))
		adev->in_s0ix = true;
	else if (gsgpu_acpi_is_s3_active(adev))
		adev->in_s3 = true;
	if (!adev->in_s0ix && !adev->in_s3)
		return 0;
	return gsgpu_device_suspend(drm_dev, true);
}

static int gsgpu_pmops_suspend_noirq(struct device *dev)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);
	struct gsgpu_device *adev = drm_to_adev(drm_dev);

	if (gsgpu_acpi_should_gpu_reset(adev))
		return gsgpu_asic_reset(adev);

	return 0;
}

static int gsgpu_pmops_resume(struct device *dev)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);
	struct gsgpu_device *adev = drm_to_adev(drm_dev);
	int r;

	if (!adev->in_s0ix && !adev->in_s3)
		return 0;

	/* Avoids registers access if device is physically gone */
	if (!pci_device_is_present(adev->pdev))
		adev->no_hw_access = true;

	r = gsgpu_device_resume(drm_dev, true);
	if (gsgpu_acpi_is_s0ix_active(adev))
		adev->in_s0ix = false;
	else
		adev->in_s3 = false;
	return r;
}

static int gsgpu_pmops_freeze(struct device *dev)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);
	struct gsgpu_device *adev = drm_to_adev(drm_dev);
	int r;

	adev->in_s4 = true;
	r = gsgpu_device_suspend(drm_dev, true);
	adev->in_s4 = false;
	if (r)
		return r;

	if (gsgpu_acpi_should_gpu_reset(adev))
		return gsgpu_asic_reset(adev);
	return 0;
}

static int gsgpu_pmops_thaw(struct device *dev)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);

	return gsgpu_device_resume(drm_dev, true);
}

static int gsgpu_pmops_poweroff(struct device *dev)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);

	return gsgpu_device_suspend(drm_dev, true);
}

static int gsgpu_pmops_restore(struct device *dev)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);

	return gsgpu_device_resume(drm_dev, true);
}

// static int gsgpu_runtime_idle_check_display(struct device *dev)
// {
// 	struct pci_dev *pdev = to_pci_dev(dev);
// 	struct drm_device *drm_dev = pci_get_drvdata(pdev);
// 	struct gsgpu_device *adev = drm_to_adev(drm_dev);

// 	if (adev->mode_info.num_crtc) {
// 		struct drm_connector *list_connector;
// 		struct drm_connector_list_iter iter;
// 		int ret = 0;

// 		/* XXX: Return busy if any displays are connected to avoid
// 		 * possible display wakeups after runtime resume due to
// 		 * hotplug events in case any displays were connected while
// 		 * the GPU was in suspend.  Remove this once that is fixed.
// 		 */
// 		mutex_lock(&drm_dev->mode_config.mutex);
// 		drm_connector_list_iter_begin(drm_dev, &iter);
// 		drm_for_each_connector_iter(list_connector, &iter) {
// 			if (list_connector->status == connector_status_connected) {
// 				ret = -EBUSY;
// 				break;
// 			}
// 		}
// 		drm_connector_list_iter_end(&iter);
// 		mutex_unlock(&drm_dev->mode_config.mutex);

// 		if (ret)
// 			return ret;

// 		if (adev->dc_enabled) {
// 			struct drm_crtc *crtc;

// 			drm_for_each_crtc(crtc, drm_dev) {
// 				drm_modeset_lock(&crtc->mutex, NULL);
// 				if (crtc->state->active)
// 					ret = -EBUSY;
// 				drm_modeset_unlock(&crtc->mutex);
// 				if (ret < 0)
// 					break;
// 			}
// 		} else {
// 			mutex_lock(&drm_dev->mode_config.mutex);
// 			drm_modeset_lock(&drm_dev->mode_config.connection_mutex, NULL);

// 			drm_connector_list_iter_begin(drm_dev, &iter);
// 			drm_for_each_connector_iter(list_connector, &iter) {
// 				if (list_connector->dpms ==  DRM_MODE_DPMS_ON) {
// 					ret = -EBUSY;
// 					break;
// 				}
// 			}

// 			drm_connector_list_iter_end(&iter);

// 			drm_modeset_unlock(&drm_dev->mode_config.connection_mutex);
// 			mutex_unlock(&drm_dev->mode_config.mutex);
// 		}
// 		if (ret)
// 			return ret;
// 	}

// 	return 0;
// }

static int gsgpu_pmops_runtime_suspend(struct device *dev)
{
	pm_runtime_forbid(dev);
	return -EBUSY;
}

static int gsgpu_pmops_runtime_resume(struct device *dev)
{
	return -EINVAL;
}

static int gsgpu_pmops_runtime_idle(struct device *dev)
{
	pm_runtime_forbid(dev);
	return -EBUSY;
}

long gsgpu_drm_ioctl(struct file *filp,
		      unsigned int cmd, unsigned long arg)
{
	struct drm_file *file_priv = filp->private_data;
	struct drm_device *dev;
	long ret;
	dev = file_priv->minor->dev;
	ret = pm_runtime_get_sync(dev->dev);
	if (ret < 0)
		goto out;

	ret = drm_ioctl(filp, cmd, arg);

	pm_runtime_mark_last_busy(dev->dev);
out:
	pm_runtime_put_autosuspend(dev->dev);
	return ret;
}

/**
 * loongson_vga_pci_devices  -- pci device id info
 *
 * __u32 vendor, device            Vendor and device ID or PCI_ANY_ID
 * __u32 subvendor, subdevice     Subsystem ID's or PCI_ANY_ID
 * __u32 class, class_mask        (class,subclass,prog-if) triplet
 * kernel_ulong_t driver_data     Data private to the driver
 */
static struct pci_device_id loongson_vga_pci_devices[] = {
	{PCI_DEVICE(PCI_VENDOR_ID_LOONGSON, 0x7a36)},
	{0, 0, 0, 0, 0, 0, 0},
};

struct pci_dev *loongson_dc_pdev;
EXPORT_SYMBOL(loongson_dc_pdev);

/**
 * loongson_vga_pci_register -- add pci device
 *
 * @pdev PCI device
 * @ent pci device id
 */
static int loongson_vga_pci_register(struct pci_dev *pdev,
				 const struct pci_device_id *ent)

{
	int ret;

	ret = pci_enable_device(pdev);
	loongson_dc_pdev = pdev;

	return ret;
}

/**
 * loongson_vga_pci_unregister -- release drm device
 *
 * @pdev PCI device
 */
static void loongson_vga_pci_unregister(struct pci_dev *pdev)
{
	pci_disable_device(pdev);
}

static const struct dev_pm_ops gsgpu_pm_ops = {
	.prepare = gsgpu_pmops_prepare,
	.complete = gsgpu_pmops_complete,
	.suspend = gsgpu_pmops_suspend,
	.suspend_noirq = gsgpu_pmops_suspend_noirq,
	.resume = gsgpu_pmops_resume,
	.freeze = gsgpu_pmops_freeze,
	.thaw = gsgpu_pmops_thaw,
	.poweroff = gsgpu_pmops_poweroff,
	.restore = gsgpu_pmops_restore,
	.runtime_suspend = gsgpu_pmops_runtime_suspend,
	.runtime_resume = gsgpu_pmops_runtime_resume,
	.runtime_idle = gsgpu_pmops_runtime_idle,
};

static int gsgpu_flush(struct file *f, fl_owner_t id)
{
	struct drm_file *file_priv = f->private_data;
	struct gsgpu_fpriv *fpriv = file_priv->driver_priv;
	long timeout = MAX_WAIT_SCHED_ENTITY_Q_EMPTY;

	timeout = gsgpu_ctx_mgr_entity_flush(&fpriv->ctx_mgr, timeout);
	timeout = gsgpu_vm_wait_idle(&fpriv->vm, timeout);

	return timeout >= 0 ? 0 : timeout;
}

static const struct file_operations gsgpu_driver_kms_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.flush = gsgpu_flush,
	.release = drm_release,
	.unlocked_ioctl = gsgpu_drm_ioctl,
	.mmap = drm_gem_mmap,
	.poll = drm_poll,
	.read = drm_read,
#ifdef CONFIG_COMPAT
	.compat_ioctl = gsgpu_kms_compat_ioctl,
#endif
};

int gsgpu_file_to_fpriv(struct file *filp, struct gsgpu_fpriv **fpriv)
{
	struct drm_file *file;

	if (!filp)
		return -EINVAL;

	if (filp->f_op != &gsgpu_driver_kms_fops) {
		return -EINVAL;
	}

	file = filp->private_data;
	*fpriv = file->driver_priv;
	return 0;
}

const struct drm_ioctl_desc gsgpu_ioctls_kms[] = {
	DRM_IOCTL_DEF_DRV(GSGPU_GEM_CREATE, gsgpu_gem_create_ioctl, DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(GSGPU_CTX, gsgpu_ctx_ioctl, DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(GSGPU_VM, gsgpu_vm_ioctl, DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(GSGPU_SCHED, gsgpu_sched_ioctl, DRM_MASTER),
	DRM_IOCTL_DEF_DRV(GSGPU_BO_LIST, gsgpu_bo_list_ioctl, DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(GSGPU_FENCE_TO_HANDLE, gsgpu_cs_fence_to_handle_ioctl, DRM_AUTH|DRM_RENDER_ALLOW),
	/* KMS */
	DRM_IOCTL_DEF_DRV(GSGPU_GEM_MMAP, gsgpu_gem_mmap_ioctl, DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(GSGPU_GEM_WAIT_IDLE, gsgpu_gem_wait_idle_ioctl, DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(GSGPU_CS, gsgpu_cs_ioctl, DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(GSGPU_INFO, gsgpu_info_ioctl, DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(GSGPU_WAIT_CS, gsgpu_cs_wait_ioctl, DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(GSGPU_WAIT_FENCES, gsgpu_cs_wait_fences_ioctl, DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(GSGPU_GEM_METADATA, gsgpu_gem_metadata_ioctl, DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(GSGPU_GEM_VA, gsgpu_gem_va_ioctl, DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(GSGPU_GEM_OP, gsgpu_gem_op_ioctl, DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(GSGPU_GEM_USERPTR, gsgpu_gem_userptr_ioctl, DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(GSGPU_HWSEMA_OP, gsgpu_hw_sema_op_ioctl, DRM_AUTH|DRM_RENDER_ALLOW)
};

static const struct drm_driver gsgpu_kms_driver = {
	.driver_features =
	    DRIVER_ATOMIC |
	    DRIVER_GEM |
	    DRIVER_RENDER | DRIVER_MODESET | DRIVER_SYNCOBJ,
	.open = gsgpu_driver_open_kms,
	.postclose = gsgpu_driver_postclose_kms,
	.lastclose = gsgpu_driver_lastclose_kms,
	.ioctls = gsgpu_ioctls_kms,
	.num_ioctls = ARRAY_SIZE(gsgpu_ioctls_kms),
	.dumb_create = gsgpu_mode_dumb_create,
	.dumb_map_offset = gsgpu_mode_dumb_mmap,
	.fops = &gsgpu_driver_kms_fops,
	.release = &gsgpu_driver_release_kms,

	.prime_handle_to_fd = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle = drm_gem_prime_fd_to_handle,
	.gem_prime_import = gsgpu_gem_prime_import,
	.gem_prime_mmap = drm_gem_prime_mmap,

	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.date = DRIVER_DATE,
	.major = KMS_DRIVER_MAJOR,
	.minor = KMS_DRIVER_MINOR,
	.patchlevel = KMS_DRIVER_PATCHLEVEL,
};

static struct pci_error_handlers gsgpu_pci_err_handler = {
	.error_detected	= gsgpu_pci_error_detected,
	.mmio_enabled	= gsgpu_pci_mmio_enabled,
	.slot_reset	= gsgpu_pci_slot_reset,
	.resume		= gsgpu_pci_resume,
};

extern const struct attribute_group gsgpu_vram_mgr_attr_group;
extern const struct attribute_group gsgpu_gtt_mgr_attr_group;
// extern const struct attribute_group gsgpu_vbios_version_attr_group;

static const struct attribute_group *gsgpu_sysfs_groups[] = {
	&gsgpu_vram_mgr_attr_group,
	&gsgpu_gtt_mgr_attr_group,
	NULL, // &gsgpu_vbios_version_attr_group,
	NULL,
};


static struct pci_driver gsgpu_kms_pci_driver = {
	.name = DRIVER_NAME,
	.id_table = pciidlist,
	.probe = gsgpu_pci_probe,
	.remove = gsgpu_pci_remove,
	.shutdown = gsgpu_pci_shutdown,
	.driver.pm = &gsgpu_pm_ops,
	.err_handler = &gsgpu_pci_err_handler,
	.dev_groups = gsgpu_sysfs_groups,
};

/**
 * loongson_vga_pci_driver -- pci driver structure
 *
 * .id_table : must be non-NULL for probe to be called
 * .probe: New device inserted
 * .remove: Device removed
 * .resume: Device suspended
 * .suspend: Device woken up
 */
static struct pci_driver loongson_vga_pci_driver = {
	.name = "gsgpu-dc",
	.id_table = loongson_vga_pci_devices,
	.probe = loongson_vga_pci_register,
	.remove = loongson_vga_pci_unregister,
};

static struct pci_driver *loongson_dc_pdriver;

static int __init gsgpu_init(void)
{
	struct pci_dev *pdev = NULL;
	struct file *fw_file = NULL;
	int r;

	if (drm_firmware_drivers_only())
		return -EINVAL;

	/* Prefer discrete card if present */
	while ((pdev = pci_get_class(PCI_CLASS_DISPLAY_VGA << 8, pdev))) {
		if (pdev->vendor != PCI_VENDOR_ID_LOONGSON)
			return 0;

		if (!gsgpu_lg100_support || (pdev->device != 0x7a36))
			return -EINVAL;

		fw_file = filp_open("/usr/lib/firmware/loongson/lg100_cp.bin",
				    O_RDONLY, 0600);
		if (IS_ERR(fw_file))
			return -EINVAL;

		filp_close(fw_file, NULL);
	}

	if (!check_vbios_info()) {
		DRM_INFO("gsgpu can not support this board!!!\n");
		return -EINVAL;
	}

	r = gsgpu_sync_init();
	if (r)
		goto error_sync;

	r = gsgpu_fence_slab_init();
	if (r)
		goto error_fence;

	DRM_INFO("gsgpu kernel modesetting enabled.\n");

	gsgpu_acpi_detect();

	loongson_dc_pdriver = &loongson_vga_pci_driver;
	r = pci_register_driver(loongson_dc_pdriver);
	if (r) {
		goto error_sync;
	}

	/* let modprobe override vga console setting */
	return pci_register_driver(&gsgpu_kms_pci_driver);

error_fence:
	gsgpu_sync_fini();

error_sync:
	return r;
}

static void __exit gsgpu_exit(void)
{
	pci_unregister_driver(&gsgpu_kms_pci_driver);
	pci_unregister_driver(loongson_dc_pdriver);
	gsgpu_sync_fini();
	gsgpu_fence_slab_fini();
	mmu_notifier_synchronize();
}

module_init(gsgpu_init);
module_exit(gsgpu_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL and additional rights");
