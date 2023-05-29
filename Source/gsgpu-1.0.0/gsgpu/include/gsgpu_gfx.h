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

#ifndef __GFX_H__
#define __GFX_H__

extern const struct gsgpu_ip_block_version gfx_ip_block;

struct gsgpu_device;

/* GFX current status */
#define GSGPU_GFX_NORMAL_MODE			0x00000000L
#define GSGPU_GFX_SAFE_MODE			0x00000001L
#define GSGPU_GFX_PG_DISABLED_MODE		0x00000002L
#define GSGPU_GFX_CG_DISABLED_MODE		0x00000004L
#define GSGPU_GFX_LBPW_DISABLED_MODE		0x00000008L

#define GSGPU_MAX_GFX_QUEUES KGD_MAX_QUEUES
#define GSGPU_MAX_COMPUTE_QUEUES KGD_MAX_QUEUES

enum gsgpu_gfx_pipe_priority {
	GSGPU_GFX_PIPE_PRIO_NORMAL = GSGPU_RING_PRIO_1,
	GSGPU_GFX_PIPE_PRIO_HIGH = GSGPU_RING_PRIO_2
};

#define GSGPU_GFX_QUEUE_PRIORITY_MINIMUM  0
#define GSGPU_GFX_QUEUE_PRIORITY_MAXIMUM  15


/*
 * GFX configurations
 */
#define GSGPU_GFX_MAX_SE 4
#define GSGPU_GFX_MAX_SH_PER_SE 2

struct gsgpu_rb_config {
	uint32_t rb_backend_disable;
	uint32_t user_rb_backend_disable;
	uint32_t raster_config;
	uint32_t raster_config_1;
};

struct gb_addr_config {
	uint16_t pipe_interleave_size;
	uint8_t num_pipes;
	uint8_t max_compress_frags;
	uint8_t num_banks;
	uint8_t num_se;
	uint8_t num_rb_per_se;
	uint8_t num_pkrs;
};

struct gsgpu_gfx_config {
	unsigned max_shader_engines;
	unsigned max_tile_pipes;
	unsigned max_cu_per_sh;
	unsigned max_sh_per_se;
	unsigned max_backends_per_se;
	unsigned max_texture_channel_caches;
	unsigned max_gprs;
	unsigned max_gs_threads;
	unsigned max_hw_contexts;
	unsigned sc_prim_fifo_size_frontend;
	unsigned sc_prim_fifo_size_backend;
	unsigned sc_hiz_tile_fifo_size;
	unsigned sc_earlyz_tile_fifo_size;

	unsigned num_tile_pipes;
	unsigned backend_enable_mask;
	unsigned mem_max_burst_length_bytes;
	unsigned mem_row_size_in_kb;
	unsigned shader_engine_tile_size;
	unsigned num_gpus;
	unsigned multi_gpu_tile_size;
	unsigned mc_arb_ramcfg;
	unsigned num_banks;
	unsigned num_ranks;
	unsigned gb_addr_config;
	unsigned num_rbs;
	unsigned gs_vgt_table_depth;
	unsigned gs_prim_buffer_depth;

	uint32_t tile_mode_array[32];
	uint32_t macrotile_mode_array[16];

	struct gb_addr_config gb_addr_config_fields;
	struct gsgpu_rb_config rb_config[GSGPU_GFX_MAX_SE][GSGPU_GFX_MAX_SH_PER_SE];

	/* gfx configure feature */
	uint32_t double_offchip_lds_buf;
	/* cached value of DB_DEBUG2 */
	uint32_t db_debug2;
	/* gfx10 specific config */
	uint32_t num_sc_per_sh;
	uint32_t num_packer_per_sc;
	uint32_t pa_sc_tile_steering_override;
	/* Whether texture coordinate truncation is conformant. */
	bool ta_cntl2_truncate_coord_mode;
	uint64_t tcc_disabled_mask;
	uint32_t gc_num_tcp_per_sa;
	uint32_t gc_num_sdp_interface;
	uint32_t gc_num_tcps;
	uint32_t gc_num_tcp_per_wpg;
	uint32_t gc_tcp_l1_size;
	uint32_t gc_num_sqc_per_wgp;
	uint32_t gc_l1_instruction_cache_size_per_sqc;
	uint32_t gc_l1_data_cache_size_per_sqc;
	uint32_t gc_gl1c_per_sa;
	uint32_t gc_gl1c_size_per_instance;
	uint32_t gc_gl2c_per_gpu;
};

struct gsgpu_cu_info {
	uint32_t simd_per_cu;
	uint32_t max_waves_per_simd;
	uint32_t wave_front_size;
	uint32_t max_scratch_slots_per_cu;
	uint32_t lds_size;

	/* total active CU number */
	uint32_t number;
	uint32_t ao_cu_mask;
	uint32_t ao_cu_bitmap[4][4];
	uint32_t bitmap[4][4];
};


struct gsgpu_gfx_funcs {
	/* get the gpu clock counter */
	uint64_t (*get_gpu_clock_counter)(struct gsgpu_device *adev);
	void (*select_se_sh)(struct gsgpu_device *adev, u32 se_num,
			     u32 sh_num, u32 instance);
	void (*read_wave_data)(struct gsgpu_device *adev, uint32_t simd,
			       uint32_t wave, uint32_t *dst, int *no_fields);
	void (*read_wave_vgprs)(struct gsgpu_device *adev, uint32_t simd,
				uint32_t wave, uint32_t thread, uint32_t start,
				uint32_t size, uint32_t *dst);
	void (*read_wave_sgprs)(struct gsgpu_device *adev, uint32_t simd,
				uint32_t wave, uint32_t start, uint32_t size,
				uint32_t *dst);
	void (*select_me_pipe_q)(struct gsgpu_device *adev, u32 me, u32 pipe,
				 u32 queue, u32 vmid);
	void (*init_spm_golden)(struct gsgpu_device *adev);
	void (*update_perfmon_mgcg)(struct gsgpu_device *adev, bool enable);
};


struct sq_work {
	struct work_struct	work;
	unsigned ih_data;
};

struct gsgpu_gfx {
	struct mutex			gpu_clock_mutex;
	struct gsgpu_gfx_config	config;
	
    const struct firmware		*cp_fw;	/* CP firmware */
	uint32_t			cp_fw_version;
	uint32_t			cp_feature_version;

	struct gsgpu_ring		gfx_ring[GSGPU_MAX_GFX_RINGS];
	unsigned			num_gfx_rings;
	
	struct gsgpu_irq_src		eop_irq;
	struct gsgpu_irq_src		priv_reg_irq;
	struct gsgpu_irq_src		priv_inst_irq;
	struct gsgpu_irq_src		cp_ecc_error_irq;
	struct gsgpu_irq_src		rlc_gc_fed_irq;

	/* gfx status */
	uint32_t			gfx_current_status;
	/* ce ram size*/
	unsigned			ce_ram_size;
	struct gsgpu_cu_info		cu_info;
	const struct gsgpu_gfx_funcs	*funcs;

	/* reset mask */
	uint32_t                        grbm_soft_reset;
	uint32_t                        srbm_soft_reset;

	/* gfx off */
	bool                            gfx_off_state;      /* true: enabled, false: disabled */
	struct mutex                    gfx_off_mutex;      /* mutex to change gfxoff state */
	uint32_t                        gfx_off_req_count;  /* default 1, enable gfx off: dec 1, disable gfx off: add 1 */
	struct delayed_work             gfx_off_delay_work; /* async work to set gfx block off */
	uint32_t                        gfx_off_residency;  /* last logged residency */
	uint64_t                        gfx_off_entrycount; /* count of times GPU has get into GFXOFF state */

	/*ras */
	struct ras_common_if		*ras_if;
	struct gsgpu_gfx_ras		*ras;

	bool				is_poweron;
};

#define gsgpu_gfx_get_gpu_clock_counter(adev) (adev)->gfx.funcs->get_gpu_clock_counter((adev))
#define gsgpu_gfx_select_se_sh(adev, se, sh, instance) (adev)->gfx.funcs->select_se_sh((adev), (se), (sh), (instance))
#define gsgpu_gfx_select_me_pipe_q(adev, me, pipe, q, vmid) (adev)->gfx.funcs->select_me_pipe_q((adev), (me), (pipe), (q), (vmid))
#define gsgpu_gfx_init_spm_golden(adev) (adev)->gfx.funcs->init_spm_golden((adev))

void gsgpu_gfx_off_ctrl(struct gsgpu_device *adev, bool enable);

#endif /*__GFX_H__*/
