/*
 * platform_ov7692.c: ov7692 platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/atomisp_platform.h>
#include <asm/intel-mid.h>
#include <asm/intel_scu_ipcutil.h>
#include <media/v4l2-subdev.h>
#include "platform_camera.h"
#include "platform_ov7692.h"
#include "platform_mt9e013.h"
#include "platform_ov5640.h"
static int camera_1_pwr_en;
static int camera_power_down;
static int camera_vprog1_on;
static int camera_ldo_power;
/*
 * MFLD PR2 secondary camera sensor - OV7692 platform data
 */
static int ov7692_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;
	if (camera_power_down < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_1_POWER_DOWN,
					GPIOF_DIR_OUT, 1);
		if (ret < 0){
			printk("ov7692_gpio_ctrl GP_CAMERA_1_POWER_DOWN not available.\n");	
			return ret;
		}
		camera_power_down = ret;
	}
	if (flag) {
		gpio_set_value(camera_power_down, 0);
		//msleep(3);		
		//gpio_set_value(camera_power_down, 0);
		}
	else
		{
		gpio_set_value(camera_power_down, 1);
	}
	return 0;
}

static int ov7692_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM1, flag ? clock_khz : 0);
}

static int ov7692_power_ctrl(struct v4l2_subdev *sd, int flag)
{
        int ret;
	/*
	if (camera_ldo_en < 0) {
	ret = camera_sensor_gpio(-1, GP_CAMERA_PWR_EN,
				GPIOF_DIR_OUT, 1);
	if (ret < 0)
		return ret;
	camera_ldo_en = ret;
	}
	if (flag) {
		   gpio_set_value(camera_ldo_en, 1);
		
	} else {
		gpio_set_value(camera_ldo_en, 0);
	}*/
	
	 if (camera_ldo_power < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_1_PWR_EN,
					GPIOF_DIR_OUT, 1);
		if (ret < 0)
		{
		   printk("ov7692_power_ctrl not available.\n");	    
			return ret;
		}
		camera_ldo_power = ret;
	}
    if (flag) {
		   gpio_set_value(camera_ldo_power, 1);
		
	} else {
		gpio_set_value(camera_ldo_power, 0);
	}
	return 0;
}
void ov7692_reset(struct v4l2_subdev *sd)
{
     ov7692_power_ctrl(sd,0);
}

static int ov7692_csi_configure(struct v4l2_subdev *sd, int flag)
{
	/* soc sensor, there is no raw bayer order (set to -1) */
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, 1,
		ATOMISP_INPUT_FORMAT_YUV422_8, -1,flag);
}

static struct camera_sensor_platform_data ov7692_sensor_platform_data = {
	.gpio_ctrl	= ov7692_gpio_ctrl,
	.flisclk_ctrl	= ov7692_flisclk_ctrl,
	.power_ctrl	= ov7692_power_ctrl,
	.csi_cfg	= ov7692_csi_configure,
};

void *ov7692_platform_data(void *info)
{
      printk("ov7692_platform_data\n");
	camera_1_pwr_en = -1;
	camera_power_down = -1;
        camera_ldo_power = -1;
	return &ov7692_sensor_platform_data;
}

