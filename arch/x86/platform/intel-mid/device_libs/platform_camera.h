/*
 * platform_camera.h: CAMERA platform library header file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _PLATFORM_CAMERA_H_
#define _PLATFORM_CAMERA_H_

#include <linux/atomisp_platform.h>
#include <asm/intel-mid.h>

extern const struct intel_v4l2_subdev_id v4l2_ids[] __attribute__((weak));

#define IS_BYT (INTEL_MID_BOARD(1, PHONE, BYT) || \
	INTEL_MID_BOARD(1, TABLET, BYT))

/* MFLD iCDK camera sensor GPIOs */

/* Obsolete pin, maybe used by old MFLD iCDK */
#define GP_CAMERA_0_POWER_DOWN          "cam0_vcm_2p8"
/* Camera VDD 1.8V Switch */
#define GP_CAMERA_1P8			"camera_on_n"
/* Camera0 Standby pin control */
#define GP_CAMERA_0_STANDBY		"camera_0_power"
#define GP_CAMERA_1_RESET               "CAM_1_RST_N"  //173
#define GP_CAMERA_1_PWR_EN              "VGA_LDO_EN"   //174
#define GP_CAMERA_1_POWER_DOWN          "CAM_POWER_EN"   //56

#define GP_CAMERA_0_RESET               "CAM_0_RST_N"
#define GP_CAMERA_PWDN            "CAM_PWDN"
#define GP_CAMERA_AF_EN			"CAM_0_AF_EN"
extern int camera_sensor_gpio(int gpio, char *name, int dir, int value);
extern int camera_sensor_csi(struct v4l2_subdev *sd, u32 port,
			u32 lanes, u32 format, u32 bayer_order, int flag);
extern void intel_register_i2c_camera_device(
				struct sfi_device_table_entry *pentry,
				struct devs_id *dev)
				__attribute__((weak));

/*
 * FIXME! This PMIC power access workaround for BTY
 * since currently no VRF for BTY
 */
#ifdef CONFIG_CRYSTAL_COVE
#define VPROG_2P8V 0x66
#define VPROG_1P8V 0x5D
#define VPROG_ENABLE 0x3
#define VPROG_DISABLE 0x2

enum camera_pmic_pin {
	CAMERA_1P8V,
	CAMERA_2P8V,
	CAMERA_POWER_NUM,
};

struct vprog_status {
	unsigned int user;
};

int camera_set_pmic_power(enum camera_pmic_pin pin, bool flag);
#endif
#endif
