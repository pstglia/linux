/*
 * platform_ov7692.h: ov7692 platform data header file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _PLATFORM_OV7692_H_
#define _PLATFORM_OV7692_H_
extern void ov7692_reset(struct v4l2_subdev *sd) __attribute__((weak));

extern void *ov7692_platform_data(void *info) __attribute__((weak));
#endif
