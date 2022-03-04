/*
 * Support for ov7692 Camera Sensor.
 *
 * Copyright (c) 2010 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#ifndef __A1040_H__
#define __A1040_H__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/spinlock.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
#include <linux/v4l2-mediabus.h>
#include <media/media-entity.h>
#include <linux/atomisp_platform.h>
#include <linux/atomisp.h>

#define V4L2_IDENT_OV7692 261

#define FULLINISUPPORT


#define REG_COM7	0x12	/* Control 7 */
#define   COM7_RESET	  0x80	  /* Register reset */

#define OV7692_MIDH		0x1C
#define OV7692_MIDL		0x1D
#define OV7692_PIDH		0x0A	/* Product ID REG*/
#define OV7692_PIDL		0x0B	/* Product ID */
#define REG_MVFP                  0x0c  /*  flip */
#define   MVFP_FLIP	        0x80	  /* Vertical flip */
#define   MVFP_MLIP	        0x40	  /* Mirr flip */

 #define OV7692_FLICKER_MODE_50HZ   	1
#define OV7692_FLICKER_MODE_60HZ	        2
struct regval_list {
	unsigned char reg_num;
	unsigned char value;
};







#define SENSOR_DETECTED		1
#define SENSOR_NOT_DETECTED	0

#define I2C_RETRY_COUNT		5
#define MSG_LEN_OFFSET		2
#define MAX_FMTS		1

#ifndef MIPI_CONTROL
#define MIPI_CONTROL		0x3400	/* MIPI_Control */
#endif

/* System control register for Aptina A-1040SOC*/
#define OV7692_PID		0x0

/* MT9P111_DEVICE_ID */
#define OV7692_MOD_ID		0x2481

/* ulBPat; */

/*
 * focal length bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define OV7692_FOCAL_LENGTH_DEFAULT 0xD00064

/*
 * current f-number bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define OV7692_F_NUMBER_DEFAULT 0x18000a
#define OV7692_FOCAL_LENGTH_NUM	208	/*2.08mm*/
#define OV7692_FOCAL_LENGTH_DEM	100
#define OV7692_F_NUMBER_DEFAULT_NUM	24
#define OV7692_F_NUMBER_DEM	10
#define OV7692_WAIT_STAT_TIMEOUT	100

/*
 * f-number range bits definition:
 * bits 31-24: max f-number numerator
 * bits 23-16: max f-number denominator
 * bits 15-8: min f-number numerator
 * bits 7-0: min f-number denominator
 */
#define OV7692_F_NUMBER_RANGE 0x180a180a

/* Supported resolutions */
enum {
	OV7692_RES_QCIF,
	OV7692_RES_QVGA,
	OV7692_RES_CIF,
	OV7692_RES_VGA,
};
#define OV7692_RES_VGA_SIZE_H		640
#define OV7692_RES_VGA_SIZE_V		480
#define OV7692_RES_CIF_SIZE_H		        352
#define OV7692_RES_CIF_SIZE_V		        288
#define OV7692_RES_QVGA_SIZE_H		320
#define OV7692_RES_QVGA_SIZE_V		240
#define OV7692_RES_QCIF_SIZE_H		176
#define OV7692_RES_QCIF_SIZE_V		144

/* completion status polling requirements, usage based on Aptina .INI Rev2 */
enum poll_reg {
	NO_POLLING,
	PRE_POLLING,
	POST_POLLING,
};
/* Supported resolutions */
enum {
	MT9M114_RES_QCIF,
	MT9M114_RES_CIF,
	MT9M114_RES_QVGA,
	MT9M114_RES_VGA,
};





struct ov7692_device {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;

	struct camera_sensor_platform_data *platform_data;
	int real_model_id;
	int nctx;
	int power;

	unsigned int bus_width;
	unsigned int mode;
	unsigned int field_inv;
	unsigned int field_sel;
	unsigned int ycseq;
	unsigned int conv422;
	unsigned int bpat;
	unsigned int hpol;
	unsigned int vpol;
	unsigned int edge;
	unsigned int bls;
	unsigned int gamma;
	unsigned int cconv;
	unsigned int res;
	unsigned int dwn_sz;
	unsigned int blc;
	unsigned int agc;
	unsigned int awb;
	unsigned int aec;
	/* extention SENSOR version 2 */
	unsigned int cie_profile;

	/* extention SENSOR version 3 */
	unsigned int flicker_freq;

	/* extension SENSOR version 4 */
	unsigned int smia_mode;
	unsigned int mipi_mode;

	/* Add name here to load shared library */
	unsigned int type;

	/*Number of MIPI lanes*/
	unsigned int mipi_lanes;
	char name[32];

	u8 lightfreq;
};

struct ov7692_format_struct {
	u8 *desc;
	u32 pixelformat;
	struct regval_list *regs;
};

struct ov7692_res_struct {
	u8 *desc;
	int res;
	int width;
	int height;
	int fps;
	int skip_frames;
	bool used;
	struct regval_list *regs;
};

struct ov7692_control {
	struct v4l2_queryctrl qc;
	int (*query)(struct v4l2_subdev *sd, s32 *value);
	int (*tweak)(struct v4l2_subdev *sd, int value);
};

#define OV7692_MAX_WRITE_BUF_SIZE	32
struct ov7692_write_buffer {
	u16 addr;
	u8 data[OV7692_MAX_WRITE_BUF_SIZE];
};

struct ov7692_write_ctrl {
	int index;
	struct ov7692_write_buffer buffer;
};

/*
 * Modes supported by the ov7692 driver.
 * Please, keep them in ascending order.
 */
static struct ov7692_res_struct ov7692_res[] = {
	{
	.desc	= "QCIF",
	.res	= OV7692_RES_QCIF,
	.width	= 176,
	.height	= 144,
	.fps	= 30,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 1,
	},
	{
	.desc	= "QVGA",
	.res	= OV7692_RES_QVGA,
	.width	= 320,
	.height	= 240,
	.fps	= 30,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 1,
	},
	{
	.desc	= "CIF",
	.res	= OV7692_RES_CIF,
	.width	= 352,
	.height	= 288,
	.fps	= 30,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 1,
	},
	{
	.desc	= "VGA",
	.res	= OV7692_RES_VGA,
	.width	= 640,
	.height	= 480,
	.fps	= 30,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 1,
	},
};
#define N_RES (ARRAY_SIZE(ov7692_res))

static const struct i2c_device_id ov7692_id[] = {
	{"ov7692", 0},
	{}
};

static struct regval_list ov7692_suspend[] = {
         {0xff, 0x01},
         {0xb4, 0x40},
         {0xb5, 0x30},
         {0xff, 0x00},
         {0xff, 0xff},

};

static struct regval_list ov7692_streaming[] = {
         {0xff, 0x01},
         {0xb4, 0xc0},
         {0xb5, 0x40},
         {0xff, 0x00},
		
         {0xff, 0xff},

};


static struct regval_list ov7692_qcif_init[] = {
        {0x16, 0x03},
         {0x17, 0xF9},
         {0x18, 0x5C},
         {0x19, 0x6C},
         {0x1A, 0x96},
         //{0x22, 0x10},
         {0xC8, 0x01},
         {0xC9, 0x60},
         {0xCA, 0x01},
         {0xCB, 0x20},
         {0xCC, 0x00},
         {0xCD, 0xB0},
         {0xCE, 0x00},
         {0xCF, 0x90},
         {0xff, 0xff},


};
static struct regval_list ov7692_cif_init[] = {
	{0x16, 0x03},
	{0x17, 0xF9},
	{0x18, 0x5C},
	{0x19, 0x6C},
	{0x1A, 0x96},
	{0x22, 0x00},
	{0xC8, 0x01},
	{0xC9, 0x60},
	{0xCA, 0x01},
	{0xCB, 0x20},
	{0xCC, 0x01},
	{0xCD, 0x60},
	{0xCE, 0x01},
	{0xCF, 0x20},

         {0xff, 0xff},

};

static struct regval_list ov7692_qvga_init[] = {

	{0x16, 0x03},
	{0x17, 0x69},
	{0x18, 0xa4},
	{0x19, 0x06},
	{0x1A, 0xf6},
	//{0x22, 0x10},
	{0xC8, 0x02},
	{0xC9, 0x80},
	{0xCA, 0x01}, //0x00
	{0xCB, 0xe0},
	{0xCC, 0x01},
	{0xCD, 0x40},
	{0xCE, 0x00},
	{0xCF, 0xf0},

	{0xff, 0xff},

};

static struct regval_list ov7692_vga_init[] = {

	{0x16, 0x03},
	{0x17, 0x69},
	{0x18, 0xa4},
	{0x19, 0x0c},
	{0x1A, 0xf6},
	//{0x22, 0x00},
	{0xC8, 0x02},
	{0xC9, 0x80},
	{0xCA, 0x01},
	{0xCB, 0xe0},
	{0xCC, 0x02},
	{0xCD, 0x80},
	{0xCE, 0x01},
	{0xCF, 0xe0},
        {0xff, 0xff},

};
#if 0
static struct regval_list  reg_expo_n2_seq[] = {

         {0xff, 0xff},
};
static struct regval_list  reg_expo_n1_seq[] = {

         {0xff, 0xff},
};
static struct regval_list  reg_expo_zero_seq[] = {

         {0xff, 0xff},
};
static struct regval_list  reg_expo_p1_seq[] = {

         {0xff, 0xff},
};
static struct regval_list  reg_expo_p2_seq[] = {

         {0xff, 0xff},
};
static struct regval_list  reg_color_none_seq[] = {

         {0xff, 0xff},
};
static struct regval_list  reg_color_mono_seq[] = {

         {0xff, 0xff},
};
static struct regval_list  reg_color_vivid_seq[] = {

         {0xff, 0xff},
};
static struct regval_list  reg_color_negative_seq[] = {

         {0xff, 0xff},
};
static struct regval_list  reg_color_sepia_seq[] = {

         {0xff, 0xff},
};
#endif
static struct regval_list  ov7692_common[] = {

         {0x12,0x80},                         
         {0x0e,0x08},                         
         {0x69,0x52},                         
         {0x1e,0xb3},                         
         {0x48,0x42},                         
         {0xff,0x01},                         
         {0xae,0xa0},                         
         {0xa8,0x26},                         
         {0xff,0x00},                         
         {0x0c,0xd0},   //0x10 lfy                      
         {0x62,0x10},                         
         {0x12,0x00},                         
         {0x17,0x65},                         
         {0x18,0xa4},                         
         {0x19,0x0a},                         
         {0x1a,0xf6},                         
         {0x3e,0x30},                         
         {0x64,0x0a},                         
         {0xff,0x01},                         
         {0x80,0x24},                         
         {0xb4,0x40},                         
         {0xb5,0x30},                         
         {0x86,0x48},                         
         {0xff,0x00},                         
         {0x67,0x20},                         
         {0x81,0x3f},                         
         {0xcc,0x02},                         
         {0xcd,0x80},                         
         {0xce,0x01},                         
         {0xcf,0xe0},                         
         {0xc8,0x02},                         
         {0xc9,0x80},                         
         {0xca,0x01},                         
         {0xcb,0xe0},                         
         {0xd0,0x48},                         
         {0x82,0x03},                         
         {0x70,0x00},                         
         {0x71,0x34},                         
         {0x74,0x28},                         
         {0x75,0x98},                         
         {0x76,0x00},                         
         {0x77,0x64},                         
         {0x78,0x01},                         
         {0x79,0xc2},                         
         {0x7a,0x4e},                         
         {0x7b,0x1f},                         
         {0x7c,0x00},                         
         {0x11,0x00},                         
         {0x20,0x00},                         
         {0x21,0x23},                         
         {0x50,0x9a},                         
         {0x51,0x80},                         
         {0x4c,0x7d},                         
         {0x41,0x43},                         
         {0x80,0x7f},                         
         {0x15,0x90},// lfy 90                         
         {0x85,0x10},//;;lens correction      
         {0x86,0x10},                         
         {0x87,0x10},                         
         {0x88,0x80},                         
         {0x89,0x2a},                         
         {0x8a,0x25},                         
         {0x8b,0x25},  
         
         {0xbb,0xac},                         
         {0xbc,0xae},                         
         {0xbd,0x02},                         
         {0xbe,0x1f},                         
         {0xbf,0x93},                         
         {0xc0,0xb1},                         
         {0xc1,0x1a},                         
         {0xb4,0x06},//;;edge/denise/exposure 
         {0xb5,0x05},                         
         {0xb6,0x00},                         
         {0xb7,0x00},                         
         {0xb8,0x06},                         
         {0xb9,0x02},                         
         {0xba,0x78},                         
         {0x81,0xff},//;;=====UV adjust======;
         {0x5A,0x10},                         
         {0x5B,0xA1},                         
         {0x5C,0x3A},                         
         {0x5d,0x20},                         
                                
         {0x00,0x40},                         
         {0x10,0x80},                         
         {0x13,0xf7},                         
         {0x24,0x88},                         
         {0x25,0x78},                         
         {0x26,0xb5},                         
         {0xa3,0x10},//;;====Gamma====;;      
         {0xa4,0x1c},                         
         {0xa5,0x30},                         
         {0xa6,0x58},                         
         {0xa7,0x68},                         
         {0xa8,0x76},                         
         {0xa9,0x81},                         
         {0xaa,0x8a},                         
         {0xab,0x92},                         
         {0xac,0x98},                         
         {0xad,0xa4},                         
         {0xae,0xb1},                         
         {0xaf,0xc5},                         
         {0xb0,0xd7},                         
         {0xb1,0xe8},                         
         {0xb2,0x20},                         
         {0x8c,0x52}, //;;==Advance==;;       
         {0x8d,0x11},                         
         {0x8e,0x12},                         
         {0x8f,0x19},                         
         {0x90,0x50},                         
         {0x91,0x20},                         
         {0x92,0xb1},                         
         {0x93,0x9a},                         
         {0x94,0x0c},                         
         {0x95,0x0c},                         
         {0x96,0xf0},                         
         {0x97,0x10},                         
         {0x98,0x61},                         
         {0x99,0x63},                         
         {0x9a,0x71},                         
         {0x9b,0x78},                         
         {0x9c,0xf0},                         
         {0x9d,0xf0},                         
         {0x9e,0xf0},                         
         {0x9f,0xff},                         
         {0xa0,0xa7},                         
         {0xa1,0xb0},                         
         {0xa2,0x0f},                         
         {0x50,0x9a},//;;banding              
         {0x51,0x80},                         
         {0x21,0x57},                         
         {0x20,0x00},                         
         {0x14,0x29},                         
         {0x68,0xb0},                         
         {0xd2,0x07},                         
         {0xd3,0x10},                         
         {0xdc,0x08},                         
         {0x0e,0x00},    
                      
         {0xff, 0xff},
};


#endif
