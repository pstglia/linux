/*
 * platform_mxt224.c: mxt224 platform data initilization file
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
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include<linux/ft5506_touch.h>
#include "platform_ft5506.h"


#include <linux/i2c.h>
#include <linux/lnw_gpio.h>
#include <linux/atomisp_platform.h>
#include <asm/intel-mid.h>


#if 0//defined(CONFIG_TOUCHSCREEN_FT5X06)
//static int ft5x06_rst = FT5X06_RST_PIN;
static ssize_t virtual_key_show(struct kobject *kobj,
                                    struct kobj_attribute *attr, char *buf)
{
	   pr_info("ft5506_platform_data()\n");
        return sprintf(buf,
               __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":120:1000:50:50"
           ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME) ":280:1000:50:50"
           ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":390:1000:50:50"
           "\n");
}

static struct kobj_attribute ft5x06_virtual_keys_attr = {
        .attr = {
                .name = "virtualkeys.ft5x06",
                .mode = S_IRUGO,
        },
        .show = virtual_key_show,
};

static struct attribute *virtual_key_attributes[] = {
        &ft5x06_virtual_keys_attr.attr,
        NULL
};

static struct attribute_group virtual_key_group = {
        .attrs = virtual_key_attributes
};
static void  ft5306_init_virtual_key(void)
{
        struct kobject *properties_kobj;
        int ret = 0;
        properties_kobj = kobject_create_and_add("board_properties", NULL);
        if (properties_kobj)
                ret = sysfs_create_group(properties_kobj, &virtual_key_group);
        if (ret < 0)
                printk("Create virtual key properties failed!\n");
}


#if 0
static struct ft5x06_info comip_i2c_ft5x06_info = {
	.reset = ft5x06_reset,
	.power_ic = ft5x06_ic_power,
	.irq_gpio = mfp_to_gpio(FT5X06_INT_PIN),
	.power_ic_flag = 1,
};
#endif

void *ft5506_platform_data(void *info)
{
	static struct ft5x06_info ft5506_pdata;

	//ft5506_pdata.numtouch       = 10;
	//ft5506_pdata.maxx          = 540;
	//ft5506_pdata.maxy          = 960;
	//ft5506_pdata.orientation    = MXT_MSGB_T9_ORIENT_HORZ_FLIP;
	ft5506_pdata.power_ic_flag = 1,
	ft5506_pdata.rst_gpio = get_gpio_by_name("ts_rst");
	ft5506_pdata.irq_gpio= get_gpio_by_name("ts_int");
	ft5506_pdata.init_virtual_key=ft5306_init_virtual_key;
	pr_info("ft5506_platform_data()\n");

	return &ft5506_pdata;
}
#endif
#if 1
void *ft5506_platform_data(void *info)
{
	static struct ft5506_touch_platform_data ft5506_pdata;

	//ft5506_pdata.numtouch       = 10;
	ft5506_pdata.maxx          = 1024;
	ft5506_pdata.maxy          = 600;
	//ft5506_pdata.orientation    = MXT_MSGB_T9_ORIENT_HORZ_FLIP;
	ft5506_pdata.reset          = get_gpio_by_name("TOUCH_RST_N");
	ft5506_pdata.irq            = get_gpio_by_name("TOUCH_INT_N");
	pr_info("ft5506_platform_data()\n");

	return &ft5506_pdata;
}
#endif
