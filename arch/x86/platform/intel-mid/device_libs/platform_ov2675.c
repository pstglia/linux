/*
 * platform_ov2675.c: ov2675 platform data initilization file
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
#include "platform_camera.h"
#include "platform_ov2675.h"


static int camera_reset;
static int camera_ldo_power;
static int camera_power_down;

/*
 * GRACELAND DV1 secondary camera sensor - ov2675 platform data
 */

static int ov2675_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;
      
	printk("---%s flag = %d -----\n",__func__,flag);
	if (camera_reset < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_1_RESET,
					GPIOF_DIR_OUT, 1);
		if (ret < 0){
			printk("ov2675_gpio_ctrl GP_CAMERA_1_RESET not available.\n");	
			return ret;
		}
		camera_reset = ret;
	}
	
	if (camera_power_down < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_1_POWER_DOWN,
					GPIOF_DIR_OUT, 1);
		if (ret < 0){
			printk("ov2675_gpio_ctrl GP_CAMERA_1_POWER_DOWN not available.\n");	
			return ret;
		}
		camera_power_down = ret;
	}
 
	//michal cong add for test code
	/*
	  camera_reset = 173;
	  if (camera_reset < 0)
			  return -EINVAL;

	  ret = gpio_request(camera_reset, "ov2675");
	   if (ret)
		  {
		   pr_debug("%s not available.", GP_CAMERA_1_RESET);  
				 return ret;
		  }

	   ret = gpio_direction_output(camera_reset,0);
	   if (ret) {
				 gpio_free(camera_reset);
				 return ret;
		 }	   


	 camera_power_down = 56;
        if (camera_power_down < 0)
                return -EINVAL;

        ret = gpio_request(camera_power_down, "ov2675");
         if (ret)
         	{
		     pr_debug("%s not available.", GP_CAMERA_1_RESET);	
                   return ret;
         	}

         ret = gpio_direction_output(camera_power_down,0);
         if (ret) {
                   gpio_free(camera_power_down);
                   return ret;
           }  

	*/
       //test code end	

	if (flag) {
		gpio_set_value(camera_power_down, 0);
		gpio_set_value(camera_reset, 0);
		msleep(20);
		gpio_set_value(camera_reset, 1);
	} else {
		gpio_set_value(camera_reset, 0);
		gpio_set_value(camera_power_down, 1);

	}

	return 0;
}

static int ov2675_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM1, flag ? clock_khz : 0);
}

static int ov2675_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;
       
	 printk("---%s flag = %d -----\n",__func__,flag);
       if (camera_ldo_power < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_1_PWR_EN,
					GPIOF_DIR_OUT, 1);
		if (ret < 0)
		{
		       printk("ov2675_power_ctrl not available.\n");	    
			return ret;
		}
		camera_ldo_power = ret;
	}

	//michal cong add for test code
/*
	 camera_ldo_power = 174;
        if (camera_ldo_power < 0)
                return -EINVAL;

        ret = gpio_request(camera_ldo_power, "ov2675");
         if (ret)
         	{
		     pr_debug("%s not available.", GP_CAMERA_1_PWR_EN);	
                   return ret;
         	}

         ret = gpio_direction_output(camera_ldo_power,0);
         if (ret) {
                   gpio_free(camera_ldo_power);
                   return ret;
           }  
       //test code end		 
	*/	 
	if (flag) {
		   gpio_set_value(camera_ldo_power, 1);
		
	} else {
		gpio_set_value(camera_ldo_power, 0);
	}
	return 0;
}

static int ov2675_csi_configure(struct v4l2_subdev *sd, int flag)
{
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, 1,
		ATOMISP_INPUT_FORMAT_YUV422_8, -1, flag);
}

static struct camera_sensor_platform_data ov2675_sensor_platform_data = {
	.gpio_ctrl      = ov2675_gpio_ctrl,
	.flisclk_ctrl   = ov2675_flisclk_ctrl,
	.power_ctrl     = ov2675_power_ctrl,
	.csi_cfg        = ov2675_csi_configure,
};

void *ov2675_platform_data(void *info)
{
	camera_reset = -1;
	camera_ldo_power = -1;
        camera_power_down = -1;

	return &ov2675_sensor_platform_data;
}
