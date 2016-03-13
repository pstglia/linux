/*
 * platform_wm5102.c: wm51020 platform data initilization file
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
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/mfd/arizona/pdata.h>
#include <linux/mfd/arizona/registers.h>
#include <linux/regulator/machine.h>
#include <asm/intel_scu_ipcutil.h>
#include <asm/intel-mid.h>
#include "platform_wm5102.h"

#define CODEC_GPIO_BASE			300

static const struct arizona_micd_range micd_ctp_ranges[] = {
	{ .max =  11, .key = KEY_MEDIA },
	{ .max =  28, .key = KEY_MEDIA },
	{ .max =  54, .key = KEY_MEDIA },
	{ .max = 100, .key = KEY_MEDIA },
	{ .max = 186, .key = KEY_MEDIA },
	{ .max = 430, .key = KEY_MEDIA },
};

static struct arizona_micd_config wm5102_micd_config[]={
	{ 0, 2, 0 },
	{ ARIZONA_ACCDET_SRC, 1, 1},
};

static struct arizona_pdata wm5102_pdata  = {
    .ldoena = -1 /*TODO: _SB.I2C7.PMIC", 3 GPIO for LDOEN.Curently hardcoded*/,
    .clk32k_src = ARIZONA_32KZ_MCLK2,
    .irq_gpio = 146,
    .irq_flags = IRQF_TRIGGER_FALLING,
    .gpio_base = CODEC_GPIO_BASE,
    .micd_pol_gpio = CODEC_GPIO_BASE + 4,
    .micd_bias_start_time = 7,
    .micd_rate = 6,
    .micd_configs = wm5102_micd_config,
    .num_micd_configs = ARRAY_SIZE(wm5102_micd_config),
    .micd_ranges = micd_ctp_ranges,
    .num_micd_ranges = ARRAY_SIZE(micd_ctp_ranges),
    .micd_force_micbias = 1,
    .inmode = {
        [0]= ARIZONA_INMODE_DIFF, /*IN1L for Headset*/
        [1]= ARIZONA_INMODE_DIFF,
        [2]= ARIZONA_INMODE_DIFF,
     },
    .micbias = {
        [0] = { /*MICBIAS1*/
            .mV =2200 ,
            .discharge =1 ,
            .soft_start =0,
            .bypass =1,
        },
        [1] = { /*MICBIAS2*/
            .mV =2200 ,
            .discharge =1 ,
            .soft_start =0,
            .bypass =1,
        },
        [2] = { /*MICBIAS3*/
            .mV =2200 ,
            .discharge =1 ,
            .soft_start =0,
            .bypass =1,
        },
    },
};

void __init *wm5102_platform_data(void *info)
{
//	struct spi_board_info *spi_info = (struct spi_board_info *)info;

	//wm5102_pdata.reset = acpi_get_gpio("\\_SB.I2C7.PMIC", 3);
	wm5102_pdata.reset = 342;
	wm5102_pdata.ldoena = 405;

	return &wm5102_pdata;
}
