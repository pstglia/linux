/*
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
*/

/* Kernel interface for SPS functionality */

#ifndef INTEL_SPS_KERNEL_INCLUDED
#define INTEL_SPS_KERNEL_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#include <linux/ioctl.h>
#include <linux/types.h>

#ifdef __KERNEL__
#include <asm/uaccess.h>
#endif

#define SPS_NUM_PARAM_WORDS   9

typedef struct tag_sps_hypercall_params {
    uint32_t params[SPS_NUM_PARAM_WORDS] ;
} __attribute__ ((packed)) sps_hypercall_params_t ;

#define INTEL_SPS_DEVICE_NAME   "secplatsvc"

#define INTEL_SPS_IOCTL_BASE    0xE1

#define INTEL_SPS_HYPERCALL_CMD    10
#define INTEL_SPS_HYPERCALL        _IOW(INTEL_SPS_IOCTL_BASE,    \
                                        INTEL_SPS_HYPERCALL_CMD, \
                                        sps_hypercall_params_t)

#ifdef __cplusplus
}
#endif

#endif /* INTEL_SPS_KERNEL_INCLUDED */



