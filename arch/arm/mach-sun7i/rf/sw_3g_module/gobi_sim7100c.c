/*
 * SoftWinners huawei sim7100c modem module
 *
 * Copyright (C) 2012 SoftWinners Incorporated
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/kmemcheck.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/idr.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/signal.h>

#include <linux/time.h>
#include <linux/timer.h>

#include <mach/sys_config.h>
#include <mach/gpio.h>
#include <linux/clk.h>
#include <linux/gpio.h>

#include "sw_module.h"

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
#define DRIVER_DESC             SW_DRIVER_NAME
#define DRIVER_VERSION          "1.0"
#define DRIVER_AUTHOR			"Javen Xu"

#define MODEM_NAME             "sim7100c"

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
static struct sw_modem g_sim7100c;
static char g_sim7100c_name[] = MODEM_NAME;

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------

static void sim7100c_vbus_on_work(struct work_struct *work)
{
	struct sw_modem *modem = container_of(work, struct sw_modem, work.work);

	modem_dbg("############ %s modem vbus power on\n", modem->name);
	
	modem_reset(modem, 1); 
}


void sim7100c_reset(struct sw_modem *modem)
{
    modem_dbg("reset %s modem\n", modem->name);

	modem_reset(modem, 0);
    sw_module_mdelay(100);
	modem_reset(modem, 1);

    return;
}

/*
*******************************************************************************
*
* wakeup_in:
*   H: wakeup MU509.
*   L: set MU509 to sleep mode.
*
*******************************************************************************
*/
static void sim7100c_sleep(struct sw_modem *modem, u32 sleep)
{
    modem_dbg("%s modem %s\n", modem->name, (sleep ? "sleep" : "wakeup"));

    if(sleep){
        modem_sleep(modem, 0);
		modem_reset(modem, 0); 
    }else{
        modem_sleep(modem, 1);
		schedule_delayed_work(&modem->work,	msecs_to_jiffies(5 * 1000));
    }

    return;
}

static void sim7100c_rf_disable(struct sw_modem *modem, u32 disable)
{
    modem_dbg("set %s modem rf %s\n", modem->name, (disable ? "disable" : "enable"));

    modem_rf_disable(modem, disable);

    return;
}

/*
*******************************************************************************
* 模组内部默认:
* vbat  : 低
* power : 高
* reset : 高
* sleep : 高
*
* 开机过程:
* (1)、默认pin配置，power拉高、reset拉高、sleep拉高
* (1)、vbat拉高
* (2)、power, 拉低持续0.7s，后拉高
*
* 关机过程:
* (1)、power, 拉低持续2.5s，后拉高
* (2)、vbat拉低
*
*******************************************************************************
*/
void sim7100c_power(struct sw_modem *modem, u32 on)
{
    modem_dbg("set %s modem power %s\n", modem->name, (on ? "on" : "off"));

    if(on){
    	/* power on */
#if 0
		modem_reset(modem, 1);
		msleep(100);
		modem_reset(modem, 0);
		msleep(100);
		modem_vbat(modem, 0);
		msleep(100);

		modem_vbat(modem, 1);
		mdelay(5000);
        modem_power_on_off(modem, 1);
#else
		modem_vbat(modem, 1);           //bpi, v1.1 LTE-RESET
		msleep(100);					
		modem_power_on_off(modem, 1);	//bpi, v1.1 LTE-POWER
		mdelay(5000);
		modem_reset(modem, 1);   		//bpi, v1.1 LTE-VBUS-ON
#endif
    }else{
		/* power off */
		modem_reset(modem, 0);
        modem_power_on_off(modem, 0);
		modem_vbat(modem, 0);
    }

	modem_dbg("set %s modem power %s end\n", modem->name, (on ? "on" : "off"));

    return;
}

static int sim7100c_start(struct sw_modem *mdev)
{
    int ret = 0;

    ret = modem_irq_init(mdev, IRQF_TRIGGER_FALLING);
    if(ret != 0){
       modem_err("err: sw_module_irq_init failed\n");
       return -1;
    }

	modem_dbg("%s\n", __func__);

    sim7100c_power(mdev, 1);

	INIT_DELAYED_WORK(&mdev->work, sim7100c_vbus_on_work);

    return 0;
}

static int sim7100c_stop(struct sw_modem *mdev)
{
    sim7100c_power(mdev, 0);
    modem_irq_exit(mdev);

    return 0;
}

static int sim7100c_suspend(struct sw_modem *mdev)
{
    sim7100c_sleep(mdev, 1);

    return 0;
}

static int sim7100c_resume(struct sw_modem *mdev)
{
    sim7100c_sleep(mdev, 0);

    return 0;
}

static struct sw_modem_ops sim7100c_ops = {
	.power          = sim7100c_power,
	.reset          = sim7100c_reset,
	.sleep          = sim7100c_sleep,
	.rf_disable     = sim7100c_rf_disable,

	.start          = sim7100c_start,
	.stop           = sim7100c_stop,

	.early_suspend  = modem_early_suspend,
	.early_resume   = modem_early_resume,

	.suspend        = sim7100c_suspend,
	.resume         = sim7100c_resume,
};

static struct platform_device sim7100c_device = {
	.name				= SW_DRIVER_NAME,
	.id					= -1,

	.dev = {
		.platform_data  = &g_sim7100c,
	},
};

static int __init sim7100c_init(void)
{
    int ret = 0;

	modem_dbg("sim7100c modem init\n");

    memset(&g_sim7100c, 0, sizeof(struct sw_modem));

    /* gpio */
    ret = modem_get_config(&g_sim7100c);
    if(ret != 0){
        modem_err("err: sim7100c_get_config failed\n");
        goto get_config_failed;
    }

    if(g_sim7100c.used == 0){
        modem_err("sim7100c is not used\n");
        goto get_config_failed;
    }

    ret = modem_pin_init(&g_sim7100c);
    if(ret != 0){
       modem_err("err: sim7100c_pin_init failed\n");
       goto pin_init_failed;
    }

    strcpy(g_sim7100c.name, g_sim7100c_name);
    g_sim7100c.ops = &sim7100c_ops;

    platform_device_register(&sim7100c_device);

	modem_dbg("sim7100c modem init end\n");

	return 0;
pin_init_failed:

get_config_failed:

    modem_dbg("%s modem init failed\n", g_sim7100c.name);

	return -1;
}

#if 0
late_initcall(sim7100c_init);
#else
static void __exit sim7100c_exit(void)
{
    platform_device_unregister(&sim7100c_device);
}

module_init(sim7100c_init);
module_exit(sim7100c_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(MODEM_NAME);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");
#endif

