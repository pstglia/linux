/*
 *
 *  spsvc_module.c - secure platform services kernel driver interface
 *
 *  Copyright(c) 2013 Intel Corporation. All rights reserved.
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the Free
 *  Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 *  more details.
 *
 *  You should have received a copy of the GNU General Public License along with
 *  this program; if not, write to the Free Software Foundation, Inc., 59
 *  Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 *
 */

#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/printk.h>
#include <linux/debugfs.h>

#include <asm/paravirt.h>
#include <asm/xen/hypervisor.h>
#include <asm/xen/hypercall.h>

#include "intel_sps_kernel.h"

/* FIXME:  this should come from include/xen/interface/xen.h */
#define __HYPERVISOR_sps_hypercall    42

#define PKTAG   "SPSvc"

static struct dentry *g_xen_dir = NULL;
static struct dentry *g_xen_log_file = NULL;

static int sps_xen_log_open(struct inode *inode, struct file *file)
{
        file->private_data = NULL; /* FIXME: figure out some private data to use */
        return 0;
}

static ssize_t sps_xen_log_get(struct file *file, char __user *user_buf,
                                size_t count, loff_t *ppos)
{
	char	tmpbuf[512];
	int	bytesread = 0;
	int res;

	do {
		res = _hypercall3(int, xenlog_op, tmpbuf, sizeof(tmpbuf), (unsigned long *)ppos);
		if (res > 0) {
			if (copy_to_user(user_buf+bytesread, tmpbuf, res)) {
				pr_err(PKTAG ": %s() copy_to_user failed\n", __FUNCTION__) ;
				return -EFAULT;
			}
			bytesread += res;
		}
	} while (res > 0 && bytesread < count);
	if (bytesread >= 0) {
		return bytesread;
	} else
		return res;
}

static const struct file_operations sps_xen_log_ops = {
        .owner          = THIS_MODULE,
        .open           = sps_xen_log_open,
        .read           = sps_xen_log_get,
        .llseek         = NULL,
};

static long handle_ioctl(struct file* fp, unsigned int cmd, unsigned long arg) {
    void __user *argp = (void __user *)arg;
    u32 result = -EINVAL;
    sps_hypercall_params_t params ; 


    if ( !xen_start_info ) {
        pr_err(PKTAG ": ERROR: service not available - kernel not virtualized\n");
        return -ENOSYS;
    }

    if ( cmd != INTEL_SPS_HYPERCALL ) {
        pr_err(PKTAG ": unknown ioctl (%u)\n", cmd);
	return -EINVAL;
    }

    /* copy hypercall params structure */
    /* FIXME: we probably can avoid this copy_from_user call */
    if ( copy_from_user(&params, argp, sizeof(params)) ) {
        pr_err(PKTAG ": copy_from_user %p ==> %p failed\n", argp, &params);
	return -EFAULT;
    }

    result = _hypercall1(u32, sps_hypercall, &params);


    return result;
}
/* End of handle_ioctl() */

static int module_release(struct inode* ind, struct file* fp)
{
    return 0 ;
}
/* End of module_release() */

/* Kernel module registration info */

static const struct file_operations module_fops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = handle_ioctl,
    .open = nonseekable_open,
    .release = module_release,
    .llseek = no_llseek,
} ;

static struct miscdevice module_miscdev = {
    .minor = MISC_DYNAMIC_MINOR,    /* dynamic allocation */
    .name = INTEL_SPS_DEVICE_NAME,  /* /dev/secplatsvc */
    .fops = &module_fops
} ;

static int __init spsvcmodule_init(void)
{
    int err;


    err = misc_register(&module_miscdev) ;

    g_xen_dir = debugfs_create_dir("xen", NULL);
    if (g_xen_dir == ERR_PTR(-ENODEV)) {
	printk(KERN_ERR "%s: debugfs_file_create error %d\n", __FUNCTION__, ENODEV);
	return -ENODEV;
    } else if (g_xen_dir == NULL) {
	printk(KERN_ERR "%s: debugfs_file_create: unknown error\n", __FUNCTION__ );
	return -EFAULT;
    }
    g_xen_log_file = debugfs_create_file("log", S_IFREG | S_IRUSR, g_xen_dir, NULL, &sps_xen_log_ops);

    return err ;
}
/* End of spsvcmodule_init() */

static void __exit spsvcmodule_exit(void) {


    debugfs_remove(g_xen_log_file);
    debugfs_remove(g_xen_dir);
    misc_deregister(&module_miscdev);
}
/* End of spsvcmodule_exit() */

module_init(spsvcmodule_init) ;
module_exit(spsvcmodule_exit) ;


/* Real code ends at this point */

MODULE_AUTHOR("Eugene Epshteyn <eugene.epshteyn@intel.com>");
MODULE_DESCRIPTION("Kernel interface to IA Secure Platform Services") ;
MODULE_VERSION("0.1") ;
MODULE_LICENSE("GPL") ;
