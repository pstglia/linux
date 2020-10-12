/*
 * SoftWinners 3G module core Linux support
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

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/pm.h>
#include <linux/earlysuspend.h>
#endif

#include <linux/time.h>
#include <linux/timer.h>
#include <linux/input.h>
#include <linux/ioport.h>
#include <linux/io.h>

#include <mach/platform.h>
#include <mach/sys_config.h>
#include <mach/gpio.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
//#include <linux/pinctrl/consumer.h>
//#include <linux/pinctrl/pinconf-sunxi.h>

#include "sw_module.h"

//#define  SW_MODEM_GPIO_WAKEUP_SYSTEM

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------

void sw_module_mdelay(u32 time)
{
	mdelay(time);
}
EXPORT_SYMBOL(sw_module_mdelay);

s32 modem_get_config(struct sw_modem *modem)
{
    script_item_value_type_e type = 0;
    script_item_u item_temp;

    /* modem_used */
    type = script_get_item("modem_para", "modem_used", &item_temp);
    if(type == SCIRPT_ITEM_VALUE_TYPE_INT){
        modem->used = item_temp.val;
    }else{
        modem_err("ERR: get modem_used failed\n");
        modem->used = 0;
    }

    /* modem_usbc_num */
    type = script_get_item("modem_para", "modem_usbc_num", &item_temp);
    if(type == SCIRPT_ITEM_VALUE_TYPE_INT){
        modem->usbc_no = item_temp.val;
    }else{
        modem_err("ERR: get modem_usbc_num failed\n");
        modem->usbc_no = 0;
    }

    /* modem_uart_num */
    type = script_get_item("modem_para", "modem_uart_num", &item_temp);
    if(type == SCIRPT_ITEM_VALUE_TYPE_INT){
        modem->uart_no = item_temp.val;
    }else{
        modem_err("ERR: get modem_uart_num failed\n");
        modem->uart_no = 0;
    }

    /* modem_name */
    type = script_get_item("modem_para", "modem_name", &item_temp);
    if(type == SCIRPT_ITEM_VALUE_TYPE_STR){
        strcpy(modem->name, item_temp.str);
        modem_dbg("%s modem support\n", modem->name);
    }else{
        modem_err("ERR: get modem_name failed\n");
        modem->name[0] = 0;
    }

    /* modem_vbat */
    type = script_get_item("modem_para", "modem_vbat", &modem->modem_vbat.pio);
    if(type == SCIRPT_ITEM_VALUE_TYPE_PIO){
        modem->modem_vbat.valid = 1;
    }else{
        modem_err("ERR: get modem_vbat failed\n");
        modem->modem_vbat.valid = 0;
    }

    /* modem_pwr_on */
    type = script_get_item("modem_para", "modem_pwr_on", &modem->modem_pwr_on.pio);
    if(type == SCIRPT_ITEM_VALUE_TYPE_PIO){
        modem->modem_pwr_on.valid = 1;
    }else{
        modem_err("ERR: get modem_pwr_on failed\n");
        modem->modem_pwr_on.valid  = 0;
    }

    /* modem_rst */
    type = script_get_item("modem_para", "modem_rst", &modem->modem_rst.pio);
    if(type == SCIRPT_ITEM_VALUE_TYPE_PIO){
        modem->modem_rst.valid = 1;
    }else{
        modem_err("ERR: get modem_rst failed\n");
        modem->modem_rst.valid  = 0;
    }

    /* modem_rf_dis */
    type = script_get_item("modem_para", "modem_rf_dis", &modem->modem_rf_dis.pio);
    if(type == SCIRPT_ITEM_VALUE_TYPE_PIO){
        modem->modem_rf_dis.valid = 1;
    }else{
        modem_err("ERR: get modem_rf_dis failed\n");
        modem->modem_rf_dis.valid  = 0;
    }

    /* modem_wake_ap */
    type = script_get_item("wakeup_src_para", "modem_wake_ap", &modem->modem_wake_ap.pio);
    if(type == SCIRPT_ITEM_VALUE_TYPE_PIO){
        modem->modem_wake_ap.valid = 1;
    }else{
        modem_err("ERR: get modem_wake_ap failed\n");
        modem->modem_wake_ap.valid  = 0;
    }

    /* modem_wake */
    type = script_get_item("modem_para", "modem_wake", &modem->modem_wake.pio);
    if(type == SCIRPT_ITEM_VALUE_TYPE_PIO){
        modem->modem_wake.valid = 1;
    }else{
        modem_err("ERR: get modem_wake failed\n");
        modem->modem_wake.valid  = 0;
    }

	/* modem_dldo */
    type = script_get_item("modem_para", "modem_dldo", &item_temp);
    if(type == SCIRPT_ITEM_VALUE_TYPE_STR){
        strcpy(modem->dldo_name, item_temp.str);
        modem_dbg("%s modem support\n", modem->dldo_name);
    }else{
        modem_err("ERR: get dldo_name failed\n");
    }

    /* modem_dldo_min_uV */
    type = script_get_item("modem_para", "modem_dldo_min_uV", &item_temp);
    if(type == SCIRPT_ITEM_VALUE_TYPE_INT){
        modem->dldo_min_uV = item_temp.val;
    }else{
        modem_err("ERR: get modem_dldo_min_uV failed\n");
        modem->dldo_min_uV = 0;
    }

    /* modem_dldo_max_uV */
    type = script_get_item("modem_para", "modem_dldo_max_uV", &item_temp);
    if(type == SCIRPT_ITEM_VALUE_TYPE_INT){
        modem->dldo_max_uV = item_temp.val;
    }else{
        modem_err("ERR: get modem_dldo_max_uV failed\n");
        modem->dldo_max_uV = 0;
    }

    return 0;
}
EXPORT_SYMBOL(modem_get_config);

s32 modem_pin_init(struct sw_modem *modem)
{
    int ret = 0;
	char pin_name[255];
	unsigned long config;

	modem_err("------------before modem init-------------------\n");
	modem_err("modem_vbat state = %d\n", __gpio_get_value(modem->modem_vbat.pio.gpio.gpio));
	modem_err("modem_pwr_on state = %d\n", __gpio_get_value(modem->modem_pwr_on.pio.gpio.gpio));
	modem_err("modem_rst state = %d\n", __gpio_get_value(modem->modem_rst.pio.gpio.gpio));
	modem_err("modem_rf_dis state = %d\n", __gpio_get_value(modem->modem_rf_dis.pio.gpio.gpio));
	

    //---------------------------------
    //  modem_vbat
    //---------------------------------
    if(modem->modem_vbat.valid){
        ret = gpio_request(modem->modem_vbat.pio.gpio.gpio, "modem_vbat");
        if(ret != 0){
            modem_err("gpio_request modem_vbat failed\n");
            modem->modem_vbat.valid = 0;
        }else{
            /* set config, ouput */
			gpio_direction_output(modem->modem_vbat.pio.gpio.gpio, modem->modem_vbat.pio.gpio.data);
            //sw_gpio_setcfg(modem->modem_vbat.pio.gpio.gpio, GPIO_CFG_OUTPUT);
        }
    }

	//delay 4sec for sim7100c lte module initial
	//msleep(4000);

    //---------------------------------
    //  modem_pwr_on
    //---------------------------------
    if(modem->modem_pwr_on.valid){
        ret = gpio_request(modem->modem_pwr_on.pio.gpio.gpio, "modem_pwr_on");
        if(ret != 0){
            modem_err("gpio_request modem_pwr_on failed\n");
            modem->modem_pwr_on.valid = 0;
        }else{
            /* set config, ouput */
			gpio_direction_output(modem->modem_pwr_on.pio.gpio.gpio, modem->modem_pwr_on.pio.gpio.data);
            //sw_gpio_setcfg(modem->modem_pwr_on.pio.gpio.gpio, GPIO_CFG_OUTPUT);
        }
    }

    //---------------------------------
    //  modem_rst
    //---------------------------------
    if(modem->modem_rst.valid){
        ret = gpio_request(modem->modem_rst.pio.gpio.gpio, "modem_rst");
        if(ret != 0){
            modem_err("gpio_request modem_rst failed\n");
            modem->modem_rst.valid = 0;
        }else{
            /* set config, ouput */
			gpio_direction_output(modem->modem_rst.pio.gpio.gpio, modem->modem_rst.pio.gpio.data);
            //sw_gpio_setcfg(modem->modem_rst.pio.gpio.gpio, GPIO_CFG_OUTPUT);
        }
    }

    //---------------------------------
    //  modem_wake
    //---------------------------------
    if(modem->modem_wake.valid){
        ret = gpio_request(modem->modem_wake.pio.gpio.gpio, "modem_wake");
        if(ret != 0){
            modem_err("gpio_request modem_wake failed\n");
            modem->modem_wake.valid = 0;
        }else{
            /* set config, ouput */
			gpio_direction_output(modem->modem_wake.pio.gpio.gpio, modem->modem_wake.pio.gpio.data);
            //sw_gpio_setcfg(modem->modem_wake.pio.gpio.gpio, GPIO_CFG_OUTPUT);
        }
    }

    //---------------------------------
    //  modem_rf_dis
    //---------------------------------
    if(modem->modem_rf_dis.valid){
        ret = gpio_request(modem->modem_rf_dis.pio.gpio.gpio, "modem_rf_dis");
        if(ret != 0){
            modem_err("gpio_request modem_rf_dis failed\n");
            modem->modem_rf_dis.valid = 0;
        }else{
            /* set config, ouput */
			gpio_direction_output(modem->modem_rf_dis.pio.gpio.gpio, modem->modem_rf_dis.pio.gpio.data);
            //sw_gpio_setcfg(modem->modem_rf_dis.pio.gpio.gpio, GPIO_CFG_OUTPUT);
        }
    }

	modem_err("------------after modem init-------------------\n");
	modem_err("modem_vbat state = %d\n", __gpio_get_value(modem->modem_vbat.pio.gpio.gpio));
	modem_err("modem_pwr_on state = %d\n", __gpio_get_value(modem->modem_pwr_on.pio.gpio.gpio));
	modem_err("modem_rst state = %d\n", __gpio_get_value(modem->modem_rst.pio.gpio.gpio));
	modem_err("modem_rf_dis state = %d\n", __gpio_get_value(modem->modem_rf_dis.pio.gpio.gpio));

    return 0;
}

EXPORT_SYMBOL(modem_pin_init);

s32 modem_pin_exit(struct sw_modem *modem)
{
    //---------------------------------
    //  modem_vbat
    //---------------------------------
    if(modem->modem_vbat.valid){
        gpio_free(modem->modem_vbat.pio.gpio.gpio);
        modem->modem_vbat.valid = 0;
    }

    //---------------------------------
    //  modem_pwr_on
    //---------------------------------
    if(modem->modem_pwr_on.valid){
        gpio_free(modem->modem_pwr_on.pio.gpio.gpio);
        modem->modem_pwr_on.valid = 0;
    }

    //---------------------------------
    //  modem_rst
    //---------------------------------
    if(modem->modem_rst.valid){
        gpio_free(modem->modem_rst.pio.gpio.gpio);
        modem->modem_rst.valid = 0;
    }

    //---------------------------------
    //  modem_wake
    //---------------------------------
    if(modem->modem_wake.valid){
        gpio_free(modem->modem_wake.pio.gpio.gpio);
        modem->modem_wake.valid = 0;
    }

    //---------------------------------
    //  modem_rf_dis
    //---------------------------------
    if(modem->modem_rf_dis.valid){
        gpio_free(modem->modem_rf_dis.pio.gpio.gpio);
        modem->modem_rf_dis.valid = 0;
    }

    return 0;
}
EXPORT_SYMBOL(modem_pin_exit);

void modem_vbat(struct sw_modem *modem, u32 value)
{
    if(!modem->modem_vbat.valid){
        return;
    }

    __gpio_set_value(modem->modem_vbat.pio.gpio.gpio, value);

    return;
}
EXPORT_SYMBOL(modem_vbat);

/* modem reset delay is 100 */
void modem_reset(struct sw_modem *modem, u32 value)
{
    if(!modem->modem_rst.valid){
        return;
    }

    __gpio_set_value(modem->modem_rst.pio.gpio.gpio, value);

    return;
}
EXPORT_SYMBOL(modem_reset);

void modem_sleep(struct sw_modem *modem, u32 value)
{
    if(!modem->modem_wake.valid){
        return;
    }

    __gpio_set_value(modem->modem_wake.pio.gpio.gpio, value);

    return;
}
EXPORT_SYMBOL(modem_sleep);

void modem_dldo_on_off(struct sw_modem *modem, u32 on)
{
	struct regulator *ldo = NULL;
	int ret = 0;

    if(modem->dldo_name[0] == 0 || modem->dldo_min_uV == 0 || modem->dldo_max_uV == 0){
        modem_err("err: dldo parameter is invalid. dldo_name=%s, dldo_min_uV=%d, dldo_max_uV=%d.\n",
                  modem->dldo_name, modem->dldo_min_uV, modem->dldo_max_uV);
        return;
    }

    /* set ldo */
	ldo = regulator_get(NULL, modem->dldo_name);
	if (!ldo) {
		modem_err("get power regulator failed.\n");
		return;
	}

	if(on){
		regulator_set_voltage(ldo, modem->dldo_min_uV, modem->dldo_max_uV);
		ret = regulator_enable(ldo);
		if(ret < 0){
			modem_err("regulator_enable failed.\n");
			regulator_put(ldo);
			return;
		}
	}else{
		ret = regulator_disable(ldo);
		if(ret < 0){
			modem_err("regulator_disable failed.\n");
			regulator_put(ldo);
			return;
		}
	}

	regulator_put(ldo);

	return;
}
EXPORT_SYMBOL(modem_dldo_on_off);

void modem_power_on_off(struct sw_modem *modem, u32 value)
{
    if(!modem->modem_pwr_on.valid){
        return;
    }

    __gpio_set_value(modem->modem_pwr_on.pio.gpio.gpio, value);

    return;
}
EXPORT_SYMBOL(modem_power_on_off);

void modem_rf_disable(struct sw_modem *modem, u32 value)
{
    if(!modem->modem_rf_dis.valid){
        return;
    }

    __gpio_set_value(modem->modem_rf_dis.pio.gpio.gpio, value);

    return;
}
EXPORT_SYMBOL(modem_rf_disable);

#ifdef  SW_3G_GPIO_WAKEUP_SYSTEM
static int modem_create_input_device(struct sw_modem *modem)
{
    int ret = 0;

    modem->key = input_allocate_device();
    if (!modem->key) {
        modem_err("err: not enough memory for input device\n");
        return -ENOMEM;
    }

    modem->key->name          = "sw_modem";
    modem->key->phys          = "modem/input0";
    modem->key->id.bustype    = BUS_HOST;
    modem->key->id.vendor     = 0xf001;
    modem->key->id.product    = 0xffff;
    modem->key->id.version    = 0x0100;

    modem->key->evbit[0] = BIT_MASK(EV_KEY);
    set_bit(KEY_POWER, modem->key->keybit);

    ret = input_register_device(modem->key);
    if (ret) {
        modem_err("err: input_register_device failed\n");
        input_free_device(modem->key);
        return -ENOMEM;
    }

    return 0;
}

static int modem_free_input_device(struct sw_modem *modem)
{
    if(modem->key){
        input_unregister_device(modem->key);
        input_free_device(modem->key);
    }

    return 0;
}

/* 通知android，唤醒系统 */
static void modem_wakeup_system(struct sw_modem *modem)
{
    modem_dbg("---------%s modem wakeup system----------\n", modem->name);

    input_report_key(modem->key, KEY_POWER, 1);
    input_sync(modem->key);
    msleep(100);
    input_report_key(modem->key, KEY_POWER, 0);
    input_sync(modem->key);

    return ;
}
#endif

static void modem_irq_enable(struct sw_modem *modem)
{
	
	enable_irq(modem->irq_hd);

    return;
}

static void modem_irq_disable(struct sw_modem *modem)
{
	
	disable_irq(modem->irq_hd);

    return;
}

#ifdef  SW_3G_GPIO_WAKEUP_SYSTEM
static void modem_irq_work(struct work_struct *data)
{
	struct sw_modem *modem = container_of(data, struct sw_modem, irq_work);

	modem_wakeup_system(modem);

	return;
}
#endif

static irqreturn_t modem_irq_interrupt(int irq, void *para)
{
	struct sw_modem *modem = (struct sw_modem *)para;
    int result = 0;

    //modem_irq_disable(modem);

#ifdef  SW_3G_GPIO_WAKEUP_SYSTEM
    if(result){
        schedule_work(&modem->irq_work);
    }
#endif

	return IRQ_HANDLED;
}

int modem_irq_init(struct sw_modem *modem, unsigned long trig_type)
{
#ifdef  SW_3G_GPIO_WAKEUP_SYSTEM
    int ret = 0;
	

    ret = modem_create_input_device(modem);
    if(ret != 0){
        modem_err("err: modem_create_input_device failed\n");
        return -1;
    }

	INIT_WORK(&modem->irq_work, modem_irq_work);
#endif

	int ret;

	if(!modem->modem_wake.valid){
		modem_err("%s: modem gpio wakeup ap not support\n", __func__);
		return 0;
	}

    modem->trig_type = trig_type;
	/*
    modem->irq_hd = sw_gpio_irq_request(modem->modem_wake_ap.pio.gpio.gpio,
                                        modem->trig_type,
                                        modem_irq_interrupt,
                                        modem);
	*/
	modem->irq_hd = gpio_to_irq(modem->modem_wake_ap.pio.gpio.gpio);
	ret = request_irq(modem->irq_hd, modem_irq_interrupt, modem->trig_type, "SoftWinner 3G Module", modem);
	
    if(IS_ERR_VALUE(ret)){
    	modem_err("err: request_irq failed\n");
		
#ifdef  SW_3G_GPIO_WAKEUP_SYSTEM
        modem_free_input_device(modem);
#endif
        return -1;
    }

    modem_irq_disable(modem);

    return 0;
}
EXPORT_SYMBOL(modem_irq_init);

int modem_irq_exit(struct sw_modem *modem)
{
    if(!modem->modem_wake_ap.valid){
        return -1;
    }

	modem_irq_disable(modem);
	free_irq(modem->irq_hd, modem);

    cancel_work_sync(&modem->irq_work);
#if 0
    modem_free_input_device(modem);
#endif
    return 0;
}
EXPORT_SYMBOL(modem_irq_exit);

void modem_early_suspend(struct sw_modem *modem)
{
    modem_irq_enable(modem);

    return;
}
EXPORT_SYMBOL(modem_early_suspend);

void modem_early_resume(struct sw_modem *modem)
{
    modem_irq_disable(modem);

    return;
}
EXPORT_SYMBOL(modem_early_resume);


