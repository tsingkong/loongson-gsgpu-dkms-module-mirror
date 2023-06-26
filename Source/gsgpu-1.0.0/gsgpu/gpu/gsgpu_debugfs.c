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
 */

#include <linux/kthread.h>
#include <linux/pci.h>
#include <linux/uaccess.h>
#include <linux/pm_runtime.h>
#include <drm/drm.h>

#include "gsgpu.h"
#include "gsgpu_umr.h"

#include "gsgpu_reset.h"

#if defined(CONFIG_DEBUG_FS)

/*
 * gsgpu_debugfs_regs_read - Callback for reading MMIO registers
 */
static ssize_t gsgpu_debugfs_regs_read(struct file *f, char __user *buf,
					size_t size, loff_t *pos)
{
	struct gsgpu_debugfs_regs2_data *rd = f->private_data;
	struct gsgpu_device *adev = rd->adev;
	dev_err(adev->dev, "%s Not Implement\n", __func__);
	return -EINVAL;
}

/**
 * gsgpu_debugfs_regs_write - Callback for writing MMIO registers
 */
static ssize_t gsgpu_debugfs_regs_write(struct file *f, const char __user *buf,
					 size_t size, loff_t *pos)
{
	struct gsgpu_debugfs_regs2_data *rd = f->private_data;
	struct gsgpu_device *adev = rd->adev;
	dev_err(adev->dev, "%s Not Implement\n", __func__);
	return -EINVAL;
}

static int gsgpu_debugfs_regs2_open(struct inode *inode, struct file *file)
{
	struct gsgpu_debugfs_regs2_data *rd;

	rd = kzalloc(sizeof *rd, GFP_KERNEL);
	if (!rd)
		return -ENOMEM;
	rd->adev = file_inode(file)->i_private;
	file->private_data = rd;
	mutex_init(&rd->lock);

	return 0;
}

static int gsgpu_debugfs_regs2_release(struct inode *inode, struct file *file)
{
	struct gsgpu_debugfs_regs2_data *rd = file->private_data;
	mutex_destroy(&rd->lock);
	kfree(file->private_data);
	return 0;
}

static long gsgpu_debugfs_regs2_ioctl(struct file *f, unsigned int cmd, unsigned long data)
{
	struct gsgpu_debugfs_regs2_data *rd = f->private_data;
	int r;

	switch (cmd) {
	case GSGPU_DEBUGFS_REGS2_IOC_SET_STATE:
		mutex_lock(&rd->lock);
		r = copy_from_user(&rd->id, (struct gsgpu_debugfs_regs2_iocdata *)data, sizeof rd->id);
		mutex_unlock(&rd->lock);
		return r ? -EINVAL : 0;
	default:
		return -EINVAL;
	}
	return 0;
}

static ssize_t gsgpu_debugfs_regs2_read(struct file *f, char __user *buf, size_t size, loff_t *pos)
{
	struct gsgpu_debugfs_regs2_data *rd = f->private_data;
	struct gsgpu_device *adev = rd->adev;
	dev_err(adev->dev, "%s Not Implement\n", __func__);
	return -EINVAL;
}

static ssize_t gsgpu_debugfs_regs2_write(struct file *f, const char __user *buf, size_t size, loff_t *pos)
{
	struct gsgpu_debugfs_regs2_data *rd = f->private_data;
	struct gsgpu_device *adev = rd->adev;
	dev_err(adev->dev, "%s Not Implement\n", __func__);
	return -EINVAL;
}

/**
 * gsgpu_debugfs_gca_config_read - Read from gfx config data
 *
 * @f: open file handle
 * @buf: User buffer to store read data in
 * @size: Number of bytes to read
 * @pos:  Offset to seek to
 *
 * This file is used to access configuration data in a somewhat
 * stable fashion.  The format is a series of DWORDs with the first
 * indicating which revision it is.  New content is appended to the
 * end so that older software can still read the data.
 */

static ssize_t gsgpu_debugfs_gca_config_read(struct file *f, char __user *buf,
					size_t size, loff_t *pos)
{
	struct gsgpu_device *adev = file_inode(f)->i_private;
	ssize_t result = 0;
	int r;
	uint32_t *config, no_regs = 0;

	if (size & 0x3 || *pos & 0x3)
		return -EINVAL;

	config = kmalloc_array(256, sizeof(*config), GFP_KERNEL);
	if (!config)
		return -ENOMEM;

	/* version, increment each time something is added */
	config[no_regs++] = 5;
	config[no_regs++] = adev->gfx.config.max_shader_engines;
	config[no_regs++] = adev->gfx.config.max_tile_pipes;
	config[no_regs++] = adev->gfx.config.max_cu_per_sh;
	config[no_regs++] = adev->gfx.config.max_sh_per_se;
	config[no_regs++] = adev->gfx.config.max_backends_per_se;
	config[no_regs++] = adev->gfx.config.max_texture_channel_caches;
	config[no_regs++] = adev->gfx.config.max_gprs;
	config[no_regs++] = adev->gfx.config.max_gs_threads;
	config[no_regs++] = adev->gfx.config.max_hw_contexts;
	config[no_regs++] = adev->gfx.config.sc_prim_fifo_size_frontend;
	config[no_regs++] = adev->gfx.config.sc_prim_fifo_size_backend;
	config[no_regs++] = adev->gfx.config.sc_hiz_tile_fifo_size;
	config[no_regs++] = adev->gfx.config.sc_earlyz_tile_fifo_size;
	config[no_regs++] = adev->gfx.config.num_tile_pipes;
	config[no_regs++] = adev->gfx.config.backend_enable_mask;
	config[no_regs++] = adev->gfx.config.mem_max_burst_length_bytes;
	config[no_regs++] = adev->gfx.config.mem_row_size_in_kb;
	config[no_regs++] = adev->gfx.config.shader_engine_tile_size;
	config[no_regs++] = adev->gfx.config.num_gpus;
	config[no_regs++] = adev->gfx.config.multi_gpu_tile_size;
	config[no_regs++] = adev->gfx.config.mc_arb_ramcfg;
	config[no_regs++] = adev->gfx.config.gb_addr_config;
	config[no_regs++] = adev->gfx.config.num_rbs;

	/* rev==2 */
	config[no_regs++] = adev->family;

	/* rev==3 */
	config[no_regs++] = adev->pdev->device;
	config[no_regs++] = adev->pdev->revision;
	config[no_regs++] = adev->pdev->subsystem_device;
	config[no_regs++] = adev->pdev->subsystem_vendor;

	/* rev==4 APU flag */
	config[no_regs++] = adev->flags & GSGPU_IS_APU ? 1 : 0;

	/* rev==5 PG/CG flag upper 32bit */
	config[no_regs++] = upper_32_bits(adev->pg_flags);
	config[no_regs++] = upper_32_bits(adev->cg_flags);

	while (size && (*pos < no_regs * 4)) {
		uint32_t value;

		value = config[*pos >> 2];
		r = put_user(value, (uint32_t *)buf);
		if (r) {
			kfree(config);
			return r;
		}

		result += 4;
		buf += 4;
		*pos += 4;
		size -= 4;
	}

	kfree(config);
	return result;
}

/**
 * gsgpu_debugfs_sensor_read - Read from the powerplay sensors
 *
 * @f: open file handle
 * @buf: User buffer to store read data in
 * @size: Number of bytes to read
 * @pos:  Offset to seek to
 */
static ssize_t gsgpu_debugfs_sensor_read(struct file *f, char __user *buf,
					size_t size, loff_t *pos)
{
	return -EINVAL;
}

/** gsgpu_debugfs_wave_read - Read WAVE STATUS data
 *
 * @f: open file handle
 * @buf: User buffer to store read data in
 * @size: Number of bytes to read
 * @pos:  Offset to seek to
 *
 * The offset being sought changes which wave that the status data
 * will be returned for.  The bits are used as follows:
 *
 * Bits 0..6: 	Byte offset into data
 * Bits 7..14:	SE selector
 * Bits 15..22:	SH/SA selector
 * Bits 23..30: CU/{WGP+SIMD} selector
 * Bits 31..36: WAVE ID selector
 * Bits 37..44: SIMD ID selector
 *
 * The returned data begins with one DWORD of version information
 * Followed by WAVE STATUS registers relevant to the GFX IP version
 * being used.  See gfx_v8_0_read_wave_data() for an example output.
 */
static ssize_t gsgpu_debugfs_wave_read(struct file *f, char __user *buf,
					size_t size, loff_t *pos)
{
	struct gsgpu_device *adev = f->f_inode->i_private;
	int r, x;
	ssize_t result = 0;
	uint32_t offset, se, sh, cu, wave, simd, data[32];

	if (size & 3 || *pos & 3)
		return -EINVAL;

	/* decode offset */
	offset = (*pos & GENMASK_ULL(6, 0));
	se = (*pos & GENMASK_ULL(14, 7)) >> 7;
	sh = (*pos & GENMASK_ULL(22, 15)) >> 15;
	cu = (*pos & GENMASK_ULL(30, 23)) >> 23;
	wave = (*pos & GENMASK_ULL(36, 31)) >> 31;
	simd = (*pos & GENMASK_ULL(44, 37)) >> 37;

	r = pm_runtime_get_sync(adev_to_drm(adev)->dev);
	if (r < 0) {
		pm_runtime_put_autosuspend(adev_to_drm(adev)->dev);
		return r;
	}

	/* switch to the specific se/sh/cu */
	mutex_lock(&adev->grbm_idx_mutex);
	// gsgpu_gfx_select_se_sh(adev, se, sh, cu);

	x = 0;
	if (adev->gfx.funcs->read_wave_data)
		adev->gfx.funcs->read_wave_data(adev, simd, wave, data, &x);

	// gsgpu_gfx_select_se_sh(adev, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF);
	mutex_unlock(&adev->grbm_idx_mutex);

	pm_runtime_mark_last_busy(adev_to_drm(adev)->dev);
	pm_runtime_put_autosuspend(adev_to_drm(adev)->dev);

	if (!x) {
		return -EINVAL;
	}

	while (size && (offset < x * 4)) {
		uint32_t value;

		value = data[offset >> 2];
		r = put_user(value, (uint32_t *)buf);
		if (r) {
			return r;
		}

		result += 4;
		buf += 4;
		offset += 4;
		size -= 4;
	}

	return result;
}

/** gsgpu_debugfs_gpr_read - Read wave gprs
 *
 * @f: open file handle
 * @buf: User buffer to store read data in
 * @size: Number of bytes to read
 * @pos:  Offset to seek to
 *
 * The offset being sought changes which wave that the status data
 * will be returned for.  The bits are used as follows:
 *
 * Bits 0..11:	Byte offset into data
 * Bits 12..19:	SE selector
 * Bits 20..27:	SH/SA selector
 * Bits 28..35: CU/{WGP+SIMD} selector
 * Bits 36..43: WAVE ID selector
 * Bits 37..44: SIMD ID selector
 * Bits 52..59: Thread selector
 * Bits 60..61: Bank selector (VGPR=0,SGPR=1)
 *
 * The return data comes from the SGPR or VGPR register bank for
 * the selected operational unit.
 */
static ssize_t gsgpu_debugfs_gpr_read(struct file *f, char __user *buf,
					size_t size, loff_t *pos)
{
	struct gsgpu_device *adev = f->f_inode->i_private;
	int r;
	ssize_t result = 0;
	uint32_t offset, se, sh, cu, wave, simd, thread, bank, *data;

	if (size > 4096 || size & 3 || *pos & 3)
		return -EINVAL;

	/* decode offset */
	offset = (*pos & GENMASK_ULL(11, 0)) >> 2;
	se = (*pos & GENMASK_ULL(19, 12)) >> 12;
	sh = (*pos & GENMASK_ULL(27, 20)) >> 20;
	cu = (*pos & GENMASK_ULL(35, 28)) >> 28;
	wave = (*pos & GENMASK_ULL(43, 36)) >> 36;
	simd = (*pos & GENMASK_ULL(51, 44)) >> 44;
	thread = (*pos & GENMASK_ULL(59, 52)) >> 52;
	bank = (*pos & GENMASK_ULL(61, 60)) >> 60;

	data = kcalloc(1024, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	r = pm_runtime_get_sync(adev_to_drm(adev)->dev);
	if (r < 0)
		goto err;

	/* switch to the specific se/sh/cu */
	mutex_lock(&adev->grbm_idx_mutex);
	// gsgpu_gfx_select_se_sh(adev, se, sh, cu);

	if (bank == 0) {
		if (adev->gfx.funcs->read_wave_vgprs)
			adev->gfx.funcs->read_wave_vgprs(adev, simd, wave, thread, offset, size>>2, data);
	} else {
		if (adev->gfx.funcs->read_wave_sgprs)
			adev->gfx.funcs->read_wave_sgprs(adev, simd, wave, offset, size>>2, data);
	}

	// gsgpu_gfx_select_se_sh(adev, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF);
	mutex_unlock(&adev->grbm_idx_mutex);

	pm_runtime_mark_last_busy(adev_to_drm(adev)->dev);
	pm_runtime_put_autosuspend(adev_to_drm(adev)->dev);

	while (size) {
		uint32_t value;

		value = data[result >> 2];
		r = put_user(value, (uint32_t *)buf);
		if (r) {
			goto err;
		}

		result += 4;
		buf += 4;
		size -= 4;
	}

	kfree(data);
	return result;

err:
	pm_runtime_put_autosuspend(adev_to_drm(adev)->dev);
	kfree(data);
	return r;
}

static const struct file_operations gsgpu_debugfs_regs2_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = gsgpu_debugfs_regs2_ioctl,
	.read = gsgpu_debugfs_regs2_read,
	.write = gsgpu_debugfs_regs2_write,
	.open = gsgpu_debugfs_regs2_open,
	.release = gsgpu_debugfs_regs2_release,
	.llseek = default_llseek
};

static const struct file_operations gsgpu_debugfs_regs_fops = {
	.owner = THIS_MODULE,
	.read = gsgpu_debugfs_regs_read,
	.write = gsgpu_debugfs_regs_write,
	.llseek = default_llseek
};

static const struct file_operations gsgpu_debugfs_gca_config_fops = {
	.owner = THIS_MODULE,
	.read = gsgpu_debugfs_gca_config_read,
	.llseek = default_llseek
};

static const struct file_operations gsgpu_debugfs_sensors_fops = {
	.owner = THIS_MODULE,
	.read = gsgpu_debugfs_sensor_read,
	.llseek = default_llseek
};

static const struct file_operations gsgpu_debugfs_wave_fops = {
	.owner = THIS_MODULE,
	.read = gsgpu_debugfs_wave_read,
	.llseek = default_llseek
};
static const struct file_operations gsgpu_debugfs_gpr_fops = {
	.owner = THIS_MODULE,
	.read = gsgpu_debugfs_gpr_read,
	.llseek = default_llseek
};

static const struct file_operations *debugfs_regs[] = {
	&gsgpu_debugfs_regs_fops,
	&gsgpu_debugfs_regs2_fops,
	&gsgpu_debugfs_gca_config_fops,
	&gsgpu_debugfs_sensors_fops,
	&gsgpu_debugfs_wave_fops,
	&gsgpu_debugfs_gpr_fops,
};

static const char *debugfs_regs_names[] = {
	"gsgpu_regs",
	"gsgpu_regs2",
	"gsgpu_gca_config",
	"gsgpu_sensors",
	"gsgpu_wave",
	"gsgpu_gpr",
};

/**
 * gsgpu_debugfs_regs_init -	Initialize debugfs entries that provide
 * 				register access.
 *
 * @adev: The device to attach the debugfs entries to
 */
int gsgpu_debugfs_regs_init(struct gsgpu_device *adev)
{
	struct drm_minor *minor = adev_to_drm(adev)->primary;
	struct dentry *ent, *root = minor->debugfs_root;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(debugfs_regs); i++) {
		ent = debugfs_create_file(debugfs_regs_names[i],
					  S_IFREG | S_IRUGO, root,
					  adev, debugfs_regs[i]);
		if (!i && !IS_ERR_OR_NULL(ent))
			i_size_write(ent->d_inode, adev->rmmio_size);
	}

	return 0;
}

static int gsgpu_debugfs_test_ib_show(struct seq_file *m, void *unused)
{
	struct gsgpu_device *adev = (struct gsgpu_device *)m->private;
	struct drm_device *dev = adev_to_drm(adev);
	int r = 0, i;

	r = pm_runtime_get_sync(dev->dev);
	if (r < 0) {
		pm_runtime_put_autosuspend(dev->dev);
		return r;
	}

	/* Avoid accidently unparking the sched thread during GPU reset */
	r = down_write_killable(&adev->reset_domain->sem);
	if (r)
		return r;

	/* hold on the scheduler */
	for (i = 0; i < GSGPU_MAX_RINGS; i++) {
		struct gsgpu_ring *ring = adev->rings[i];

		if (!ring || !ring->sched.thread)
			continue;
		kthread_park(ring->sched.thread);
	}

	seq_printf(m, "run ib test:\n");
	r = gsgpu_ib_ring_tests(adev);
	if (r)
		seq_printf(m, "ib ring tests failed (%d).\n", r);
	else
		seq_printf(m, "ib ring tests passed.\n");

	/* go on the scheduler */
	for (i = 0; i < GSGPU_MAX_RINGS; i++) {
		struct gsgpu_ring *ring = adev->rings[i];

		if (!ring || !ring->sched.thread)
			continue;
		kthread_unpark(ring->sched.thread);
	}

	up_write(&adev->reset_domain->sem);

	pm_runtime_mark_last_busy(dev->dev);
	pm_runtime_put_autosuspend(dev->dev);

	return 0;
}

static int gsgpu_debugfs_test_ring_show(struct seq_file *m, void *data)
{
	struct gsgpu_device *adev = (struct gsgpu_device *)m->private;
	// struct drm_device *dev = adev_to_drm(adev);
	struct gsgpu_ring *ring;
	int r = 0, i;

	/* hold on the scheduler */
	for (i = 0; i < GSGPU_MAX_RINGS; i++) {
		ring = adev->rings[i];

		if (!ring || !ring->sched.thread)
			continue;
		kthread_park(ring->sched.thread);
	}

	seq_printf(m, "run ring test:\n");
	ring = &adev->gfx.gfx_ring[0];
	r = gsgpu_ring_test_ring(ring);
	if (r)
		seq_printf(m, "%s ring tests failed (%d).\n", ring->name, r);
	else
		seq_printf(m, "%s ring tests passed.\n", ring->name);


	for (i = 0; i < adev->xdma.num_instances; i++) {
		ring = &adev->xdma.instance[i].ring;
		r = gsgpu_ring_test_ring(ring);
		if (r)
			seq_printf(m, "%s ring tests failed (%d).\n", ring->name, r);
		else
			seq_printf(m, "%s ring tests passed.\n", ring->name);
	}

	/* go on the scheduler */
	for (i = 0; i < GSGPU_MAX_RINGS; i++) {
		struct gsgpu_ring *ring = adev->rings[i];

		if (!ring || !ring->sched.thread)
			continue;
		kthread_unpark(ring->sched.thread);
	}

	return 0;
}

static int gsgpu_debugfs_test_xdma_show(struct seq_file *m, void *data)
{
	struct gsgpu_device *adev = (struct gsgpu_device *)m->private;
	// struct drm_device *dev = adev_to_drm(adev);
	struct gsgpu_ring *ring;
	int r = 0, i;

	/* hold on the scheduler */
	for (i = 0; i < GSGPU_MAX_RINGS; i++) {
		ring = adev->rings[i];

		if (!ring || !ring->sched.thread)
			continue;
		kthread_park(ring->sched.thread);
	}

	seq_printf(m, "run xdma test:\n");
	for (i = 0; i < adev->xdma.num_instances; i++) {
		ring = &adev->xdma.instance[i].ring;
		r = gsgpu_ring_test_xdma(ring, msecs_to_jiffies(5000));
		if (r)
			seq_printf(m, "%s ring tests failed (%d).\n", ring->name, r);
		else
			seq_printf(m, "%s ring tests passed.\n", ring->name);
	}

	/* go on the scheduler */
	for (i = 0; i < GSGPU_MAX_RINGS; i++) {
		struct gsgpu_ring *ring = adev->rings[i];

		if (!ring || !ring->sched.thread)
			continue;
		kthread_unpark(ring->sched.thread);
	}

	return 0;
}

static int gsgpu_debugfs_evict_vram(void *data, u64 *val)
{
	struct gsgpu_device *adev = (struct gsgpu_device *)data;
	struct drm_device *dev = adev_to_drm(adev);
	int r;

	r = pm_runtime_get_sync(dev->dev);
	if (r < 0) {
		pm_runtime_put_autosuspend(dev->dev);
		return r;
	}

	*val = gsgpu_ttm_evict_resources(adev, TTM_PL_VRAM);

	pm_runtime_mark_last_busy(dev->dev);
	pm_runtime_put_autosuspend(dev->dev);

	return 0;
}


static int gsgpu_debugfs_evict_gtt(void *data, u64 *val)
{
	struct gsgpu_device *adev = (struct gsgpu_device *)data;
	struct drm_device *dev = adev_to_drm(adev);
	int r;

	r = pm_runtime_get_sync(dev->dev);
	if (r < 0) {
		pm_runtime_put_autosuspend(dev->dev);
		return r;
	}

	*val = gsgpu_ttm_evict_resources(adev, TTM_PL_TT);

	pm_runtime_mark_last_busy(dev->dev);
	pm_runtime_put_autosuspend(dev->dev);

	return 0;
}

static int gsgpu_debugfs_benchmark(void *data, u64 val)
{
	struct gsgpu_device *adev = (struct gsgpu_device *)data;
	struct drm_device *dev = adev_to_drm(adev);
	int r;

	r = pm_runtime_get_sync(dev->dev);
	if (r < 0) {
		pm_runtime_put_autosuspend(dev->dev);
		return r;
	}

	r = gsgpu_benchmark(adev, val);

	pm_runtime_mark_last_busy(dev->dev);
	pm_runtime_put_autosuspend(dev->dev);

	return r;
}

static int gsgpu_debugfs_vm_info_show(struct seq_file *m, void *unused)
{
	struct gsgpu_device *adev = (struct gsgpu_device *)m->private;
	struct drm_device *dev = adev_to_drm(adev);
	struct drm_file *file;
	int r;

	r = mutex_lock_interruptible(&dev->filelist_mutex);
	if (r)
		return r;

	list_for_each_entry(file, &dev->filelist, lhead) {
		struct gsgpu_fpriv *fpriv = file->driver_priv;
		struct gsgpu_vm *vm = &fpriv->vm;

		seq_printf(m, "pid:%d\tProcess:%s ----------\n",
				vm->task_info.pid, vm->task_info.process_name);
		r = gsgpu_bo_reserve(vm->root.bo, true);
		if (r)
			break;
		gsgpu_debugfs_vm_bo_info(vm, m);
		gsgpu_bo_unreserve(vm->root.bo);
	}

	mutex_unlock(&dev->filelist_mutex);

	return r;
}

DEFINE_SHOW_ATTRIBUTE(gsgpu_debugfs_test_ib);
DEFINE_SHOW_ATTRIBUTE(gsgpu_debugfs_vm_info);
DEFINE_SHOW_ATTRIBUTE(gsgpu_debugfs_test_ring);
DEFINE_SHOW_ATTRIBUTE(gsgpu_debugfs_test_xdma);
DEFINE_DEBUGFS_ATTRIBUTE(gsgpu_evict_vram_fops, gsgpu_debugfs_evict_vram,
			 NULL, "%lld\n");
DEFINE_DEBUGFS_ATTRIBUTE(gsgpu_evict_gtt_fops, gsgpu_debugfs_evict_gtt,
			 NULL, "%lld\n");
DEFINE_DEBUGFS_ATTRIBUTE(gsgpu_benchmark_fops, NULL, gsgpu_debugfs_benchmark,
			 "%lld\n");

static void gsgpu_ib_preempt_fences_swap(struct gsgpu_ring *ring,
					  struct dma_fence **fences)
{
	struct gsgpu_fence_driver *drv = &ring->fence_drv;
	uint32_t sync_seq, last_seq;

	last_seq = atomic_read(&ring->fence_drv.last_seq);
	sync_seq = ring->fence_drv.sync_seq;

	last_seq &= drv->num_fences_mask;
	sync_seq &= drv->num_fences_mask;

	do {
		struct dma_fence *fence, **ptr;

		++last_seq;
		last_seq &= drv->num_fences_mask;
		ptr = &drv->fences[last_seq];

		fence = rcu_dereference_protected(*ptr, 1);
		RCU_INIT_POINTER(*ptr, NULL);

		if (!fence)
			continue;

		fences[last_seq] = fence;

	} while (last_seq != sync_seq);
}

static void gsgpu_ib_preempt_signal_fences(struct dma_fence **fences,
					    int length)
{
	int i;
	struct dma_fence *fence;

	for (i = 0; i < length; i++) {
		fence = fences[i];
		if (!fence)
			continue;
		dma_fence_signal(fence);
		dma_fence_put(fence);
	}
}

static void gsgpu_ib_preempt_job_recovery(struct drm_gpu_scheduler *sched)
{
	struct drm_sched_job *s_job;
	struct dma_fence *fence;

	spin_lock(&sched->job_list_lock);
	list_for_each_entry(s_job, &sched->pending_list, list) {
		fence = sched->ops->run_job(s_job);
		dma_fence_put(fence);
	}
	spin_unlock(&sched->job_list_lock);
}

static void gsgpu_ib_preempt_mark_partial_job(struct gsgpu_ring *ring)
{
	struct gsgpu_job *job;
	struct drm_sched_job *s_job, *tmp;
	uint32_t preempt_seq;
	struct dma_fence *fence, **ptr;
	struct gsgpu_fence_driver *drv = &ring->fence_drv;
	struct drm_gpu_scheduler *sched = &ring->sched;
	bool preempted = true;

	if (ring->funcs->type != GSGPU_RING_TYPE_GFX)
		return;

	preempt_seq = le32_to_cpu(*(drv->cpu_addr + 2));
	if (preempt_seq <= atomic_read(&drv->last_seq)) {
		preempted = false;
		goto no_preempt;
	}

	preempt_seq &= drv->num_fences_mask;
	ptr = &drv->fences[preempt_seq];
	fence = rcu_dereference_protected(*ptr, 1);

no_preempt:
	spin_lock(&sched->job_list_lock);
	list_for_each_entry_safe(s_job, tmp, &sched->pending_list, list) {
		if (dma_fence_is_signaled(&s_job->s_fence->finished)) {
			/* remove job from ring_mirror_list */
			list_del_init(&s_job->list);
			sched->ops->free_job(s_job);
			continue;
		}
		job = to_gsgpu_job(s_job);
		if (preempted && (&job->hw_fence) == fence)
			/* mark the job as preempted */
			job->preemption_status |= GSGPU_IB_PREEMPTED;
	}
	spin_unlock(&sched->job_list_lock);
}

static int gsgpu_debugfs_ib_preempt(void *data, u64 val)
{
	int r, length;
	struct gsgpu_ring *ring;
	struct dma_fence **fences = NULL;
	struct gsgpu_device *adev = (struct gsgpu_device *)data;

	if (val >= GSGPU_MAX_RINGS)
		return -EINVAL;

	ring = adev->rings[val];

	if (!ring || !ring->funcs->preempt_ib || !ring->sched.thread)
		return -EINVAL;

	/* the last preemption failed */
	if (ring->trail_seq != le32_to_cpu(*ring->trail_fence_cpu_addr))
		return -EBUSY;

	length = ring->fence_drv.num_fences_mask + 1;
	fences = kcalloc(length, sizeof(void *), GFP_KERNEL);
	if (!fences)
		return -ENOMEM;

	/* Avoid accidently unparking the sched thread during GPU reset */
	r = down_read_killable(&adev->reset_domain->sem);
	if (r)
		goto pro_end;

	/* stop the scheduler */
	kthread_park(ring->sched.thread);

	/* preempt the IB */
	r = gsgpu_ring_preempt_ib(ring);
	if (r) {
		DRM_WARN("failed to preempt ring %d\n", ring->idx);
		goto failure;
	}

	gsgpu_fence_process(ring);

	if (atomic_read(&ring->fence_drv.last_seq) !=
	    ring->fence_drv.sync_seq) {
		DRM_INFO("ring %d was preempted\n", ring->idx);

		gsgpu_ib_preempt_mark_partial_job(ring);

		/* swap out the old fences */
		gsgpu_ib_preempt_fences_swap(ring, fences);

		gsgpu_fence_driver_force_completion(ring);

		/* resubmit unfinished jobs */
		gsgpu_ib_preempt_job_recovery(&ring->sched);

		/* wait for jobs finished */
		gsgpu_fence_wait_empty(ring);

		/* signal the old fences */
		gsgpu_ib_preempt_signal_fences(fences, length);
	}

failure:
	/* restart the scheduler */
	kthread_unpark(ring->sched.thread);

	up_read(&adev->reset_domain->sem);

pro_end:
	kfree(fences);

	return r;
}

static int gsgpu_debugfs_sclk_set(void *data, u64 val)
{
	int ret = 0;
	// uint32_t max_freq, min_freq;
	struct gsgpu_device *adev = (struct gsgpu_device *)data;

	ret = pm_runtime_get_sync(adev_to_drm(adev)->dev);
	if (ret < 0) {
		pm_runtime_put_autosuspend(adev_to_drm(adev)->dev);
		return ret;
	}

// out:
	pm_runtime_mark_last_busy(adev_to_drm(adev)->dev);
	pm_runtime_put_autosuspend(adev_to_drm(adev)->dev);

	return ret;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_ib_preempt, NULL,
			gsgpu_debugfs_ib_preempt, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(fops_sclk_set, NULL,
			gsgpu_debugfs_sclk_set, "%llu\n");

static ssize_t gsgpu_reset_dump_register_list_read(struct file *f,
				char __user *buf, size_t size, loff_t *pos)
{
	struct gsgpu_device *adev = (struct gsgpu_device *)file_inode(f)->i_private;
	char reg_offset[12];
	int i, ret, len = 0;

	if (*pos)
		return 0;

	memset(reg_offset, 0, 12);
	ret = down_read_killable(&adev->reset_domain->sem);
	if (ret)
		return ret;

	for (i = 0; i < adev->num_regs; i++) {
		sprintf(reg_offset, "0x%x\n", adev->reset_dump_reg_list[i]);
		up_read(&adev->reset_domain->sem);
		if (copy_to_user(buf + len, reg_offset, strlen(reg_offset)))
			return -EFAULT;

		len += strlen(reg_offset);
		ret = down_read_killable(&adev->reset_domain->sem);
		if (ret)
			return ret;
	}

	up_read(&adev->reset_domain->sem);
	*pos += len;

	return len;
}

static ssize_t gsgpu_reset_dump_register_list_write(struct file *f,
			const char __user *buf, size_t size, loff_t *pos)
{
	struct gsgpu_device *adev = (struct gsgpu_device *)file_inode(f)->i_private;
	char reg_offset[11];
	uint32_t *new = NULL, *tmp = NULL;
	int ret, i = 0, len = 0;

	do {
		memset(reg_offset, 0, 11);
		if (copy_from_user(reg_offset, buf + len,
					min(10, ((int)size-len)))) {
			ret = -EFAULT;
			goto error_free;
		}

		new = krealloc_array(tmp, i + 1, sizeof(uint32_t), GFP_KERNEL);
		if (!new) {
			ret = -ENOMEM;
			goto error_free;
		}
		tmp = new;
		if (sscanf(reg_offset, "%X %n", &tmp[i], &ret) != 1) {
			ret = -EINVAL;
			goto error_free;
		}

		len += ret;
		i++;
	} while (len < size);

	new = kmalloc_array(i, sizeof(uint32_t), GFP_KERNEL);
	if (!new) {
		ret = -ENOMEM;
		goto error_free;
	}
	ret = down_write_killable(&adev->reset_domain->sem);
	if (ret)
		goto error_free;

	swap(adev->reset_dump_reg_list, tmp);
	swap(adev->reset_dump_reg_value, new);
	adev->num_regs = i;
	up_write(&adev->reset_domain->sem);
	ret = size;

error_free:
	if (tmp != new)
		kfree(tmp);
	kfree(new);
	return ret;
}

static const struct file_operations gsgpu_reset_dump_register_list = {
	.owner = THIS_MODULE,
	.read = gsgpu_reset_dump_register_list_read,
	.write = gsgpu_reset_dump_register_list_write,
	.llseek = default_llseek
};

int gsgpu_debugfs_init(struct gsgpu_device *adev)
{
	struct dentry *root = adev_to_drm(adev)->primary->debugfs_root;
	struct dentry *ent;
	int r, i;

	if (!debugfs_initialized())
		return 0;

	ent = debugfs_create_file("gsgpu_preempt_ib", 0600, root, adev,
				  &fops_ib_preempt);
	if (IS_ERR(ent)) {
		DRM_ERROR("unable to create gsgpu_preempt_ib debugsfs file\n");
		return PTR_ERR(ent);
	}

	ent = debugfs_create_file("gsgpu_force_sclk", 0200, root, adev,
				  &fops_sclk_set);
	if (IS_ERR(ent)) {
		DRM_ERROR("unable to create gsgpu_set_sclk debugsfs file\n");
		return PTR_ERR(ent);
	}

	/* Register debugfs entries for gsgpu_ttm */
	gsgpu_ttm_debugfs_init(adev);
	gsgpu_debugfs_sa_init(adev);
	gsgpu_debugfs_fence_init(adev);
	gsgpu_debugfs_gem_init(adev);

	r = gsgpu_debugfs_sema_init(adev);
	if (r)
		DRM_ERROR("registering sema debugfs failed (%d).\n", r);

	r = gsgpu_debugfs_regs_init(adev);
	if (r)
		DRM_ERROR("registering register debugfs failed (%d).\n", r);

#if defined(CONFIG_DRM_GSGPU_DC)
	if (adev->dc_enabled)
		dtn_debugfs_init(adev);
#endif

	for (i = 0; i < GSGPU_MAX_RINGS; ++i) {
		struct gsgpu_ring *ring = adev->rings[i];

		if (!ring)
			continue;

		gsgpu_debugfs_ring_init(adev, ring);
	}

	debugfs_create_file("gsgpu_evict_vram", 0444, root, adev,
			    &gsgpu_evict_vram_fops);
	debugfs_create_file("gsgpu_evict_gtt", 0444, root, adev,
			    &gsgpu_evict_gtt_fops);
	debugfs_create_file("gsgpu_test_ib", 0444, root, adev,
			    &gsgpu_debugfs_test_ib_fops);
	debugfs_create_file("gsgpu_test_ring", 0444, root, adev,
			    &gsgpu_debugfs_test_ring_fops);
	debugfs_create_file("gsgpu_test_xdma", 0444, root, adev,
			    &gsgpu_debugfs_test_xdma_fops);
	debugfs_create_file("gsgpu_vm_info", 0444, root, adev,
			    &gsgpu_debugfs_vm_info_fops);
	debugfs_create_file("gsgpu_benchmark", 0200, root, adev,
			    &gsgpu_benchmark_fops);
	debugfs_create_file("gsgpu_reset_dump_register_list", 0644, root, adev,
			    &gsgpu_reset_dump_register_list);

	adev->debugfs_vbios_blob.data = adev->bios;
	adev->debugfs_vbios_blob.size = adev->bios_size;
	debugfs_create_blob("gsgpu_vbios", 0444, root,
			    &adev->debugfs_vbios_blob);

	return 0;
}

#else
int gsgpu_debugfs_init(struct gsgpu_device *adev)
{
	return 0;
}
int gsgpu_debugfs_regs_init(struct gsgpu_device *adev)
{
	return 0;
}
#endif
