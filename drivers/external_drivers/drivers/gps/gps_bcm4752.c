/*
 * GPS BCM4752 interface.
 *
 * (C) Copyright 2009 BYD.
 * All Rights Reserved
 */
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/atomisp_platform.h>
#include <asm/intel_scu_ipcutil.h>
#include <media/v4l2-subdev.h>

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include "gps_bcm4752.h"
#include <asm/intel-mid.h>


#include <linux/serial_core.h>

#include <asm/intel_mid_hsu.h>
#include <linux/pm_runtime.h>

#define UART_PORT_NO 1
#define GPS_SDL_DEBUG_WRITE_STR printk("sdl %s value=%c p_name=\"%s\" pid=%d ppid=%d\n", __func__ ,messages[0],current->comm, current->pid,current->parent->pid)
static int GPIO_AGPS_PON=59;
static int GPIO_AGPS_CAL=92;
static int GPIO_AGPS_WAKES_HOST=89;

//static	struct device * gps_bcm4752_tty_dev;


static int gps_power_on(void)
{
    //pm_runtime_get(gps_bcm4752_tty_dev);
    gpio_set_value(GPIO_AGPS_PON,1);
    return 0;
}

static int gps_power_off(void)
{
   // pm_runtime_put(gps_bcm4752_tty_dev);
    gpio_set_value(GPIO_AGPS_PON,0);
    return 0;
}

static int gps_reset(int flag)
{
    
    mdelay(120);
    gpio_set_value(GPIO_AGPS_PON,flag);
    return 0;
}


static int gps_bcm4752_open(struct inode *inode, struct file *filp)
{
    return 0;
}

static int gps_bcm4752_close(struct inode *inode, struct file *filp)
{
    return 0;
}
static ssize_t bcm4752_write_standby_proc(struct file *filp,
		const char *buff, size_t len, loff_t *off)
{
	char messages[256]; 
	if (copy_from_user(messages, buff, 1))
		return -EFAULT;	
    GPS_SDL_DEBUG_WRITE_STR ;
	if(strncmp(messages, "1", 1) == 0){
        gps_power_on();
	}
	else if(strncmp(messages, "0", 1) == 0){
        gps_power_off();
	}
    mdelay(120);
	return len;
}
static ssize_t bcm4752_read_standby_proc(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	int len = gpio_get_value(GPIO_AGPS_PON);
	if (len != 0)
	{
		len = 1;
	}

	len = sprintf(page, "%d\n",len);
	return len;
}
 static const struct file_operations bcm4752_proc_standby_fops = {
     .read = bcm4752_read_standby_proc,
     .write = bcm4752_write_standby_proc,
 };

static void create_bcm4752_proc_standby_file(void)
{
	struct proc_dir_entry *bcm4752_proc_file = NULL;
    bcm4752_proc_file=proc_create("driver/bcms",0644,NULL,
    &bcm4752_proc_standby_fops);
}




static const struct file_operations gps_bcm4752_fops = {
	.owner = THIS_MODULE,
	.open = gps_bcm4752_open,	/* open */
	.release = gps_bcm4752_close,	/* release */
};

static struct miscdevice gps_bcm4752_miscdev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = GPS_DRIVER_NAME,
    .fops  = &gps_bcm4752_fops,
};

static int gps_bcm4752_probe(struct platform_device *pdev)
{
    int result = 0;
    int ret;
    printk("sdl --- gps_bcm4752_probe\n");
	if (GPIO_AGPS_PON == -1) {
		printk("gps_bcm4752_probe GPIO_AGPS_PON error\n");
		return -EINVAL;
	}
	ret = gpio_request(GPIO_AGPS_PON, "GPS_PON");
		if (ret) {
		printk("gps_bcm4752_probe failed to request GPS_PON \n");
		return -EINVAL;
	}

    gpio_direction_output(GPIO_AGPS_PON, 1);

    result = misc_register(&gps_bcm4752_miscdev);
    if (result) {
    	printk("gps_bcm4752: misc register fail, result=%d\r\n", result);
    	return result;
    }
    mdelay(120);
    gpio_set_value(GPIO_AGPS_PON,0);

    
	if (GPIO_AGPS_CAL == -1) {
	printk("gps_bcm4752_probe GPIO_AGPS_CAL error\n");
	return -EINVAL;
	}
	ret = gpio_request(GPIO_AGPS_CAL, "GPIO_AGPS_CAL");
	if (ret) {
	printk("gps_bcm4752_probe failed to request GPIO_AGPS_CAL \n");
	return -EINVAL;
	}

    create_bcm4752_proc_standby_file();

    //gps_bcm4752_tty_dev=intel_mid_hsu_set_wake_peer(1,NULL);
    return 0;
}

static int gps_bcm4752_remove(struct platform_device *pdev)
{
	misc_deregister(&gps_bcm4752_miscdev);

	return 0;
}

static struct platform_driver gps_bcm4752_driver = {
    .driver = {
        .name	= GPS_DRIVER_NAME,
    },
    .probe		= gps_bcm4752_probe,
    .remove		= gps_bcm4752_remove,
};
static struct platform_device gps_bcm4752_device = {
	.name = "agpsgpio",
	.id = -1,
};
void reg_bcm4752(void)
{
	platform_device_register(&gps_bcm4752_device);
	
}

static int __init gps_bcm4752_init(void)
{
	reg_bcm4752();
	return platform_driver_register(&gps_bcm4752_driver);
}

static void __exit gps_bcm4752_exit(void)
{
	platform_driver_unregister(&gps_bcm4752_driver);
}

module_init(gps_bcm4752_init);
module_exit(gps_bcm4752_exit);

MODULE_LICENSE("GPL");

