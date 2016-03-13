/*
 * platform_sst_audio.h:  sst audio platform data header file
 *
 * Copyright (C) 2012 Intel Corporation
 * Author: Jeeja KP <jeeja.kp@intel.com>
 *	Omair Mohammed Abdullah <omair.m.abdullah@intel.com>
 *	Vinod Koul ,vinod.koul@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _PLATFORM_SST_AUDIO_H_
#define _PLATFORM_SST_AUDIO_H_

#include <linux/sfi.h>

#define MAX_DESCRIPTOR_SIZE 172

#define MAX_NUM_STREAMS_MRFLD	25
#define MAX_NUM_STREAMS	MAX_NUM_STREAMS_MRFLD
/* The stream map status is used to dynamically assign
 * device-id to a device, for example probe device. If
 * a stream map entry is free for a device then the device-id
 * for that device will be popluated when the device is
 * opened and then the status set to IN_USE. When device
 * is closed, the strm map status is set to FREE again.
 */
enum sst_strm_map_status {
	SST_DEV_MAP_FREE = 0,
	SST_DEV_MAP_IN_USE,
};

/* Device IDs for CTP are same as stream IDs */
enum sst_audio_device_id_ctp {
	SST_PCM_OUT0 = 1,
	SST_PCM_OUT1 = 2,
	SST_COMPRESSED_OUT = 3,
	SST_CAPTURE_IN = 4,
	SST_PROBE_IN = 5,
};

enum sst_audio_task_id_mrfld {
	SST_TASK_ID_NONE = 0,
	SST_TASK_ID_SBA = 1,
	SST_TASK_ID_FBA_UL = 2,
	SST_TASK_ID_MEDIA = 3,
	SST_TASK_ID_AWARE = 4,
	SST_TASK_ID_FBA_DL = 5,
	SST_TASK_ID_MAX = SST_TASK_ID_FBA_DL,
};

/* Device IDs for Merrifield are Pipe IDs,
 * ref: LPE DSP command interface spec v0.75 */
enum sst_audio_device_id_mrfld {
	/* Output pipeline IDs */
	PIPE_ID_OUT_START = 0x0,
	PIPE_MODEM_OUT = 0x0,
	PIPE_BT_OUT = 0x1,
	PIPE_CODEC_OUT0 = 0x2,
	PIPE_CODEC_OUT1 = 0x3,
	PIPE_SPROT_LOOP_OUT = 0x4,
	PIPE_MEDIA_LOOP1_OUT = 0x5,
	PIPE_MEDIA_LOOP2_OUT = 0x6,
	PIPE_PROBE_OUT = 0x7,
	PIPE_HF_SNS_OUT = 0x8, /* VOCIE_UPLINK_REF2 */
	PIPE_HF_OUT = 0x9, /* VOICE_UPLINK_REF1 */
	PIPE_SPEECH_OUT = 0xA, /* VOICE UPLINK */
	PIPE_RxSPEECH_OUT = 0xB, /* VOICE_DOWNLINK */
	PIPE_VOIP_OUT = 0xC,
	PIPE_PCM0_OUT = 0xD,
	PIPE_PCM1_OUT = 0xE,
	PIPE_PCM2_OUT = 0xF,
	PIPE_AWARE_OUT = 0x10,
	PIPE_VAD_OUT = 0x11,
	PIPE_MEDIA0_OUT = 0x12,
	PIPE_MEDIA1_OUT = 0x13,
	PIPE_FM_OUT = 0x14,
	PIPE_PROBE1_OUT = 0x15,
	PIPE_PROBE2_OUT = 0x16,
	PIPE_PROBE3_OUT = 0x17,
	PIPE_PROBE4_OUT = 0x18,
	PIPE_PROBE5_OUT = 0x19,
	PIPE_PROBE6_OUT = 0x1A,
	PIPE_PROBE7_OUT = 0x1B,
	PIPE_PROBE8_OUT = 0x1C,
/* Input Pipeline IDs */
	PIPE_ID_IN_START = 0x80,
	PIPE_MODEM_IN = 0x80,
	PIPE_BT_IN = 0x81,
	PIPE_CODEC_IN0 = 0x82,
	PIPE_CODEC_IN1 = 0x83,
	PIPE_SPROT_LOOP_IN = 0x84,
	PIPE_MEDIA_LOOP1_IN = 0x85,
	PIPE_MEDIA_LOOP2_IN = 0x86,
	PIPE_PROBE_IN = 0x87,
	PIPE_SIDETONE_IN = 0x88,
	PIPE_TxSPEECH_IN = 0x89,
	PIPE_SPEECH_IN = 0x8A,
	PIPE_TONE_IN = 0x8B,
	PIPE_VOIP_IN = 0x8C,
	PIPE_PCM0_IN = 0x8D,
	PIPE_PCM1_IN = 0x8E,
	PIPE_MEDIA0_IN = 0x8F,
	PIPE_MEDIA1_IN = 0x90,
	PIPE_MEDIA2_IN = 0x91,
	PIPE_FM_IN = 0x92,
	PIPE_PROBE1_IN = 0x93,
	PIPE_PROBE2_IN = 0x94,
	PIPE_PROBE3_IN = 0x95,
	PIPE_PROBE4_IN = 0x96,
	PIPE_PROBE5_IN = 0x97,
	PIPE_PROBE6_IN = 0x98,
	PIPE_PROBE7_IN = 0x99,
	PIPE_PROBE8_IN = 0x9A,
	PIPE_MEDIA3_IN = 0x9C,
	PIPE_LOW_PCM0_IN = 0x9D,
	PIPE_RSVD = 0xFF,
};

/* The stream map for each platform consists of an array of the below
 * stream map structure. The array index is used as the static stream-id
 * associated with a device and (dev_num,subdev_num,direction) tuple match
 * gives the device_id for the device.
 */
struct sst_dev_stream_map {
	u8 dev_num;		/* device id */
	u8 subdev_num;		/* substream */
	u8 direction;
	u8 device_id;		/* fw id */
	u8 task_id;		/* fw task */
	u8 status;
};

struct sst_dev_effects_map {
	char	uuid[16];
	u16	algo_id;
	char	descriptor[MAX_DESCRIPTOR_SIZE];
};

struct sst_dev_effects_resource_map {
	char  uuid[16];
	unsigned int flags;
	u16 cpuLoad;
	u16 memoryUsage;
};

struct sst_dev_effects {
	struct sst_dev_effects_map *effs_map;
	struct sst_dev_effects_resource_map *effs_res_map;
	unsigned int effs_num_map;
};

struct sst_platform_data {
	/* Intel software platform id*/
	const struct soft_platform_id *spid;
	struct sst_dev_stream_map *pdev_strm_map;
	struct sst_dev_effects pdev_effs;
	unsigned int strm_map_size;
};

struct sst_info {
	u32 iram_start;
	u32 iram_end;
	bool iram_use;
	u32 dram_start;
	u32 dram_end;
	bool dram_use;
	u32 imr_start;
	u32 imr_end;
	bool imr_use;
	u32 mailbox_start;
	bool use_elf;
	bool lpe_viewpt_rqd;
	unsigned int max_streams;
	u32 dma_max_len;
	u8 num_probes;
};

struct sst_lib_dnld_info {
	unsigned int mod_base;
	unsigned int mod_end;
	unsigned int mod_table_offset;
	unsigned int mod_table_size;
	bool mod_ddr_dnld;
};

struct sst_res_info {
	unsigned int shim_offset;
	unsigned int shim_size;
	unsigned int shim_phy_addr;
	unsigned int ssp0_offset;
	unsigned int ssp0_size;
	unsigned int dma0_offset;
	unsigned int dma0_size;
	unsigned int dma1_offset;
	unsigned int dma1_size;
	unsigned int iram_offset;
	unsigned int iram_size;
	unsigned int dram_offset;
	unsigned int dram_size;
	unsigned int mbox_offset;
	unsigned int mbox_size;
	unsigned int acpi_lpe_res_index;
	unsigned int acpi_ddr_index;
	unsigned int acpi_ipc_irq_index;
};

struct sst_ipc_info {
	int ipc_offset;
	unsigned int mbox_recv_off;
};

struct sst_platform_info {
	const struct sst_info *probe_data;
	const struct sst_ipc_info *ipc_info;
	const struct sst_res_info *res_info;
	const struct sst_lib_dnld_info *lib_info;
	const char *platform;
};

int add_sst_platform_device(void);
#endif

