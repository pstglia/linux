/*
 * platform_s5k5cagx.c: s5k5cagx platform data initilization file
 *
 * (C) Copyright 2012 Intel Corporation
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
#include "platform_camera.h"
#include "platform_s5k5cagx.h"


static int camera_reset;
static int camera_power_en;
static int camera_power_down;

#ifdef CONFIG_BOARD_CTP
static int camera_vemmc1_on;
static struct regulator *vemmc1_reg;
#define VEMMC1_VAL 2850000
static int camera_vprog1_on;
static struct regulator *vprog1_reg;
#define VPROG1_VAL 1200000
#else
static int camera_vprog1_on;
#endif


/*
 * MRFLD VV primary camera sensor - IMX135 platform data
 */

static int s5k5cagx_platform_init(struct v4l2_subdev *sd, int flag)
{

	int ret = 0;
	if (camera_reset < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_1_RESET,
					GPIOF_DIR_OUT, 0);
		if (ret < 0)
			return ret;
		camera_reset = ret;
		gpio_set_value(camera_reset, 0);
	}

	if (camera_power_en < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_AF_EN,
					GPIOF_DIR_OUT, 0);
		if (ret < 0)
			return ret;
		camera_power_en = ret;
		gpio_set_value(camera_power_en, 0);
	}

	if (camera_power_down < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_PWDN,
					GPIOF_DIR_OUT, 0);
		if (ret < 0)
			return ret;
		camera_power_down = ret;
		gpio_set_value(camera_power_down, 1);
	}
	return 0;
}
static int s5k5cagx_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{

if (flag){
	#if 0
	 gpio_direction_output(172, 0);
	 gpio_direction_output(173, 0);
	 gpio_direction_output(87, 0);
	 msleep(50);
	 gpio_direction_output(172, 1);
	 msleep(100);
	gpio_direction_output(173, 1);
	 gpio_direction_output(173, 1);
	 #endif
	gpio_set_value(camera_power_down, 0);
	msleep(20);
	//usleep_range(15, 2 * 15);
	gpio_set_value(camera_reset, 1);
	msleep(100);
	//usleep_range(100,  150);
	gpio_set_value(camera_power_down, 0);
}
else {
	#if 0
	printk("%s  power down come in\n",__func__);
	 gpio_direction_output(172, 0);
	 gpio_direction_output(173, 0);
	 gpio_direction_output(87, 0);
	 #endif
	gpio_set_value(camera_power_down, 1);	 
	//gpio_set_value(camera_reset, 0);
}

	return 0;
}

static int s5k5cagx_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM0, flag ? clock_khz : 0);
}

static int s5k5cagx_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	if(flag){
		gpio_set_value(camera_power_en, 1);	
	}
	else {
		//gpio_set_value(camera_power_en, 0);	
	}
	return 0;
}


static int s5k5cagx_csi_configure(struct v4l2_subdev *sd, int flag)
{
	static const int LANES = 1;
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, LANES,
		ATOMISP_INPUT_FORMAT_YUV422_8, -1, flag);
}

static struct camera_sensor_platform_data s5k5cagx_sensor_platform_data = {
	.gpio_ctrl      = s5k5cagx_gpio_ctrl,
	.flisclk_ctrl   = s5k5cagx_flisclk_ctrl,
	.power_ctrl     = s5k5cagx_power_ctrl,
	.csi_cfg        = s5k5cagx_csi_configure,
	.platform_init	=	s5k5cagx_platform_init,
};

void *s5k5cagx_platform_data(void *info)
{
	camera_reset = -1;
	camera_power_en = -1;
	camera_power_down = -1;

	return &s5k5cagx_sensor_platform_data;
}

