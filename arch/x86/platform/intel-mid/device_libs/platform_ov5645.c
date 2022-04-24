/*
 * platform_ov5645.c: ov5645 platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/atomisp_platform.h>
#include <asm/intel_scu_ipcutil.h>
#include <asm/intel-mid.h>
#include <media/v4l2-subdev.h>
#include <linux/regulator/consumer.h>
#include <linux/sfi.h>
#include "platform_camera.h"
#include "platform_ov5645.h"


static int camera_reset;
static int camera_power_down;
static int camera_vprog1_on;
static int camera_af_en;


static struct regulator *vprog1_reg;
#define VPROG1_VAL 2800000
/*
 * CLV PR0 primary camera sensor - OV5645 platform data
 */

static int ov5645_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;

	if (camera_reset < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_0_RESET,
					GPIOF_DIR_OUT, 1);
		if (ret < 0)
			return ret;
		camera_reset = ret;
	}
printk(" %s camera_reset = %d flag = %d  0-1 \n",__func__,camera_reset,flag);
	if (flag) {
		gpio_set_value(camera_reset, 0);
		msleep(20);
		gpio_set_value(camera_reset, 1);
	} else {
		gpio_set_value(camera_reset, 0);
	}
printk(" %s  over \n",__func__,camera_reset);
	return 0;
}

static int ov5645_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM0, flag ? clock_khz : 0);
}

/*
 * Checking the SOC type is temporary workaround to enable OV5645
 * on Bodegabay (tangier) platform. Once standard regulator devices
 * (e.g. vprog1, vprog2) and control functions (pmic_avp) are added
 * for the platforms with tangier, then we can revert this change.
 * (dongwon.kim@intel.com)
 */
static int ov5645_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret = 0;

	/* The camera powering is different on RedHookBay and VictoriaBay
	 * On RHB, vprog1 is at 2.8V and supplies both cameras
	 * On VB, vprog1 supplies the 2nd camera and must not rise over 1.2V
	 * Check if the RHB SW has accidentally been flashed to VB
	 * If yes, don't turn on the regulator. The VB secondary camera will
	 * be permanently damaged by the too high voltage
	 */
	if (INTEL_MID_BOARD(2, PHONE, CLVTP, VB, PRO) ||
	    INTEL_MID_BOARD(2, PHONE, CLVTP, VB, ENG)) {
		printk(KERN_ALERT \
		"Aborted vprog1 enable to protect VictoriaBay 2nd camera HW\n");
		return -ENODEV;
	}
printk(" %s camera_vprog1_on = %d flag = %d \n",__func__,camera_vprog1_on,flag);
	if (flag) {
	    if (camera_af_en < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_AF_EN,
					GPIOF_DIR_OUT, 1);
		if (ret < 0)
			return ret;
		camera_af_en = ret;
		}
		gpio_set_value(camera_af_en, 1);
		printk("%s line = %d \n ",__func__,__LINE__);
	#if 0
	ret = gpio_direction_output(172,1);
	printk("%s line = %d  output pin 172 CAM_0_AF_EN\n ",__func__,__LINE__);
	
	if (ret) {
		pr_err("%s: failed to set gpio(pin 172) direction\n",
							__func__);
		gpio_free(172);
	}		
	#endif
	msleep(5);
	printk("%s line = %d pwdn 1\n ",__func__,__LINE__);	
	    if (camera_power_down < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_PWDN,
					GPIOF_DIR_OUT, 1);
		if (ret < 0)
			return ret;
		camera_power_down = ret;
		}
		gpio_set_value(camera_power_down, 1);
		
		printk("%s line = %d \n ",__func__,__LINE__);
		if (!camera_vprog1_on) {		
			if (intel_mid_identify_cpu() ==
			   INTEL_MID_CPU_CHIP_TANGIER)
				ret = intel_scu_ipc_msic_vprog1(1);
			else
				ret = regulator_enable(vprog1_reg);
			if (!ret)
				camera_vprog1_on = 1;
			else
				printk(KERN_ALERT "Failed to enable regulator vprog1\n");
			return ret;
		}
		printk("%s line = %d \n ",__func__,__LINE__);
	} else {
	
		gpio_set_value(camera_power_down, 0);
		
		gpio_set_value(camera_af_en, 0);
		
		if (camera_vprog1_on) {
			if (intel_mid_identify_cpu() ==
			   INTEL_MID_CPU_CHIP_TANGIER)
				ret = intel_scu_ipc_msic_vprog1(0);
			else
				ret = regulator_disable(vprog1_reg);

			if (!ret)
				camera_vprog1_on = 0;
			else
				printk(KERN_ALERT "Failed to disable regulator vprog1\n");
			return ret;
		}
	}
	printk("%s line = %d lane 2\n ",__func__,__LINE__);
	return ret;
}

static int ov5645_csi_configure(struct v4l2_subdev *sd, int flag)
{
	static const int LANES = 2;
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, LANES,
		ATOMISP_INPUT_FORMAT_YUV422_8, -1, flag);
}

/*
 * Checking the SOC type is temporary workaround to enable OV5645
 * on Bodegabay (tangier) platform. Once standard regulator devices
 * (e.g. vprog1, vprog2) and control functions (pmic_avp) are added
 * for the platforms with tangier, then we can revert this change.
 * (dongwon.kim@intel.com)
 */
static int ov5645_platform_init(struct i2c_client *client)
{
	int ret;
	if (intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_TANGIER)
		return 0;
printk("%s line = %d \n ",__func__,__LINE__);
	vprog1_reg = regulator_get(&client->dev, "vprog1");
	if (IS_ERR(vprog1_reg)) {
		dev_err(&client->dev, "regulator_get failed\n");
		return PTR_ERR(vprog1_reg);
	}
	ret = regulator_set_voltage(vprog1_reg, VPROG1_VAL, VPROG1_VAL);
	if (ret) {
		dev_err(&client->dev, "regulator voltage set failed\n");
		regulator_put(vprog1_reg);
	}
	return ret;
}

/*
 * Checking the SOC type is temporary workaround to enable OV5645 on Bodegabay
 * (tangier) platform once standard regulator devices (e.g. vprog1, vprog2) and
 * control functions (pmic_avp) are added for the platforms with tangier, then
 * we can revert this change.(dongwon.kim@intel.com
 */
static int ov5645_platform_deinit(void)
{
	if (intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_TANGIER)
		return 0;

	regulator_put(vprog1_reg);

	return 0;
}
static struct camera_sensor_platform_data ov5645_sensor_platform_data = {
	.gpio_ctrl      = ov5645_gpio_ctrl,
	.flisclk_ctrl   = ov5645_flisclk_ctrl,
	.power_ctrl     = ov5645_power_ctrl,
	.csi_cfg        = ov5645_csi_configure,
	.platform_init = ov5645_platform_init,
	.platform_deinit = ov5645_platform_deinit,
};

void *ov5645_platform_data(void *info)
{
	camera_reset = -1;
	camera_power_down = -1;
	camera_af_en = -1;

	return &ov5645_sensor_platform_data;
}
