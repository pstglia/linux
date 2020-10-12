/*
 * A V4L2 driver for Novatek nt99141 cameras.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/clk.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-mediabus.h>//linux-3.0
#include <linux/io.h>
//#include <mach/gpio_v2.h>
#include <mach/sys_config.h>
#include <linux/regulator/consumer.h>
#include <mach/system.h>
//#include "../../../../power/axp_power/axp-gpio.h"
#if defined CONFIG_ARCH_SUN4I
#include "../include/sun4i_csi_core.h"
#include "../include/sun4i_dev_csi.h"
#elif defined CONFIG_ARCH_SUN5I
#include "../include/sun5i_csi_core.h"
#include "../include/sun5i_dev_csi.h"
#else 
#include "../include/sunxi_csi_core.h"
#include "../include/sunxi_dev_csi.h"
#endif

#include "../include/sunxi_csi_core.h"
#include "../include/sunxi_dev_csi.h"
#include <linux/gpio.h>

MODULE_AUTHOR("raymonxiu");
MODULE_DESCRIPTION("A low-level driver for NT99141 sensors");
MODULE_LICENSE("GPL");

//#define SYS_FS_RW

//for internel driver debug
#define DEV_DBG_EN   		1 
#if(DEV_DBG_EN == 1)		
#define csi_dev_dbg(x,arg...) pr_err(KERN_INFO"[CSI_DEBUG][nt99141]"x,##arg)
#else
#define csi_dev_dbg(x,arg...) 
#endif
#define csi_dev_err(x,arg...) pr_err(KERN_INFO"[CSI_ERR][nt99141]"x,##arg)
#define csi_dev_print(x,arg...) pr_err(KERN_INFO"[CSI][nt99141]"x,##arg)

#define MCLK (24*1000*1000)
#define VREF_POL	CSI_HIGH
#define HREF_POL	CSI_HIGH
#define CLK_POL		CSI_RISING
#define IO_CFG		0				//0:csi back 1:csi front
#define V4L2_IDENT_SENSOR 0x141		//NT99141

//define the voltage level of control signal
#define CSI_STBY_ON			1
#define CSI_STBY_OFF 		0
#define CSI_RST_ON			0
#define CSI_RST_OFF			1
#define CSI_PWR_ON			1
#define CSI_PWR_OFF			0
 

#define REG_TERM 0xff
#define VAL_TERM 0xff


#define REG_ADDR_STEP 2
#define REG_DATA_STEP 1
#define REG_STEP 			(REG_ADDR_STEP+REG_DATA_STEP)

/*
 * Basic window sizes.  These probably belong somewhere more globally
 * useful.
 */
#define UXGA_WIDTH		1280
#define UXGA_HEIGHT	    720
#define SXGA_WIDTH 		1280
#define SXGA_HEIGHT		960
#define XGA_WIDTH 		1024
#define XGA_HEIGHT		768
#define SVGA_WIDTH		800
#define SVGA_HEIGHT 	600
#define VGA_WIDTH		640
#define VGA_HEIGHT		480
#define QVGA_WIDTH		320
#define QVGA_HEIGHT		240
#define CIF_WIDTH		352
#define CIF_HEIGHT		288
#define QCIF_WIDTH		176
#define	QCIF_HEIGHT		144



/*
 * Our nominal (default) frame rate.
 */
#define SENSOR_FRAME_RATE 30

/*
 * The nt99141 sits on i2c with ID 0x54
 */
#define I2C_ADDR 0x54

/* Registers */




/*
 * Information we maintain about a known sensor.
 */
struct sensor_format_struct;  /* coming later */
struct snesor_colorfx_struct; /* coming later */
__csi_subdev_info_t ccm_info_con = {
	.mclk 	= MCLK,
	.vref 	= VREF_POL,
	.href 	= HREF_POL,
	.clock	= CLK_POL,
	.iocfg	= IO_CFG,
};

struct sensor_info {
	struct v4l2_subdev sd;
	struct sensor_format_struct *fmt;  /* Current format */
	__csi_subdev_info_t *ccm_info;
	int	width;
	int	height;
	int brightness;
	int	contrast;
	int saturation;
	int hue;
	int hflip;
	int vflip;
	int gain;
	int autogain;
	int exp;
	enum v4l2_exposure_auto_type autoexp;
	int autowb;
	enum v4l2_whiteblance wb;
	enum v4l2_colorfx clrfx;
	enum v4l2_flash_led_mode flash_mode;
	u8 clkrc;			/* Clock divider value */
	struct delayed_work work;
	struct workqueue_struct *wq;
	int night_mode;
	int streaming;
	enum v4l2_power_line_frequency band_filter;

};

static inline struct sensor_info *to_state(struct v4l2_subdev *sd) {
	return container_of(sd, struct sensor_info, sd);
}
/*
 * The default register settings
 *
 */

struct regval_list {
	unsigned char reg_num[REG_ADDR_STEP];
	unsigned char value[REG_DATA_STEP];
};

static struct regval_list sensor_default_regs[] = {
//**************************************
//common
//NT99141DVRSetting
//winson20130916
//**************************************
{{0x30, 0x69}, {0x03}},
{{0x30, 0x6A}, {0x03}},

{{0x32, 0x0A}, {0xB2}},//linebuffer
{{0x31, 0x09}, {0x84}},//0x84: internal power, 0x04: external power.
{{0x30, 0x40}, {0x04}},//CalibrationControl
{{0x30, 0x41}, {0x02}},//AutoCalibration
{{0x30, 0x55}, {0x40}},
{{0x30, 0x54}, {0x00}},
{{0x30, 0x42}, {0xFF}},// ABLC_Thr_Top
{{0x30, 0x43}, {0x08}},// ABLC_Thr_Bottom14->08
{{0x30, 0x52}, {0xE0}},// ABLC_GainW
{{0x30, 0x5F}, {0x11}},// Clamp_Control
{{0x31, 0x00}, {0x0f}},// BIAS,BIAS_Pixel
{{0x31, 0x06}, {0x03}},// abs_en=1}, abloom_en=1
{{0x31, 0x05}, {0x01}},
{{0x31, 0x08}, {0x05}},
{{0x31, 0x10}, {0x22}},
{{0x31, 0x11}, {0x57}},
{{0x31, 0x12}, {0x22}},
{{0x31, 0x13}, {0x55}},
{{0x31, 0x14}, {0x05}},
{{0x31, 0x35}, {0x00}},
{{0x32, 0xF0}, {0x01}},// format: YUYV 0x03
{{0x32, 0xF2}, {0x7C}},
{{0x32, 0xFc}, {0x00}},// Brightness
{{0x32, 0x90}, {0x01}},// WB_Gain_R
{{0x32, 0x91}, {0x88}},
{{0x32, 0x96}, {0x01}},// WB_Gain_B
{{0x32, 0x97}, {0x71}},
//Call = [AWB_Setting_Sunny3304]
{{0x32, 0x50}, {0x80}},
{{0x32, 0x51}, {0x01}},
{{0x32, 0x52}, {0x48}},
{{0x32, 0x53}, {0xA5}},
{{0x32, 0x54}, {0x00}},
{{0x32, 0x55}, {0xE8}},
{{0x32, 0x56}, {0x85}},
{{0x32, 0x57}, {0x3d}},
{{0x32, 0x9B}, {0x01}},
{{0x32, 0xA1}, {0x01}},
{{0x32, 0xA2}, {0x10}},
{{0x32, 0xA3}, {0x01}},
{{0x32, 0xA4}, {0xa0}},
{{0x32, 0xA5}, {0x01}},
{{0x32, 0xA6}, {0x22}},
{{0x32, 0xA7}, {0x02}},
{{0x32, 0xA8}, {0x00}},
{{0x32, 0xA9}, {0x10}},
{{0x32, 0xB0}, {0x28}},
{{0x32, 0xB1}, {0x14}},
{{0x32, 0xB2}, {0xFF}},
{{0x32, 0xB3}, {0xBE}},//0x7d,//0x69,

//Call = [LSC_Sunny3304_85%_LV_MACHINE]
{{0x32, 0x10}, {0x18}},  //Gain0 of R
{{0x32, 0x11}, {0x1B}},  //Gain1 of R
{{0x32, 0x12}, {0x1B}},  //Gain2 of R
{{0x32, 0x13}, {0x1A}},  //Gain3 of R
{{0x32, 0x14}, {0x17}},  //Gain0 of Gr
{{0x32, 0x15}, {0x18}},  //Gain1 of Gr
{{0x32, 0x16}, {0x19}},  //Gain2 of Gr
{{0x32, 0x17}, {0x18}},  //Gain3 of Gr
{{0x32, 0x18}, {0x17}},  //Gain0 of Gb
{{0x32, 0x19}, {0x18}},  //Gain1 of Gb
{{0x32, 0x1A}, {0x19}},  //Gain2 of Gb
{{0x32, 0x1B}, {0x18}},  //Gain3 of Gb
{{0x32, 0x1C}, {0x16}},  //Gain0 of B
{{0x32, 0x1D}, {0x16}},  //Gain1 of B
{{0x32, 0x1E}, {0x16}},  //Gain2 of B
{{0x32, 0x1F}, {0x15}},  //Gain3 of B
{{0x32, 0x31}, {0x50}},
{{0x32, 0x32}, {0xC5}},

//Gamma
{{0x32, 0x70}, {0x00}},
{{0x32, 0x71}, {0x0c}},
{{0x32, 0x72}, {0x18}},
{{0x32, 0x73}, {0x2d}},
{{0x32, 0x74}, {0x41}},
{{0x32, 0x75}, {0x52}},
{{0x32, 0x76}, {0x6d}},
{{0x32, 0x77}, {0x82}},
{{0x32, 0x78}, {0x93}},
{{0x32, 0x79}, {0xa3}},
{{0x32, 0x7A}, {0xbc}},
{{0x32, 0x7B}, {0xd3}},
{{0x32, 0x7C}, {0xE6}},
{{0x32, 0x7D}, {0xF5}},
{{0x32, 0x7E}, {0xFF}},

//CC_Tune
{{0x33, 0x02}, {0x00}},
{{0x33, 0x03}, {0x5F}},
{{0x33, 0x04}, {0x00}},
{{0x33, 0x05}, {0x67}},
{{0x33, 0x06}, {0x00}},
{{0x33, 0x07}, {0x39}},
{{0x33, 0x08}, {0x07}},
{{0x33, 0x09}, {0xBC}},
{{0x33, 0x0A}, {0x06}},
{{0x33, 0x0B}, {0xF4}},
{{0x33, 0x0C}, {0x01}},
{{0x33, 0x0D}, {0x51}},
{{0x33, 0x0E}, {0x01}},
{{0x33, 0x0F}, {0x28}},
{{0x33, 0x10}, {0x06}},
{{0x33, 0x11}, {0xD3}},
{{0x33, 0x12}, {0x00}},
{{0x33, 0x13}, {0x04}},

//Call = [Edge_Table_Default]
{{0x33, 0x26}, {0x03}},
{{0x33, 0x27}, {0x0A}},
{{0x33, 0x28}, {0x0A}},
{{0x33, 0x29}, {0x06}},
{{0x33, 0x2A}, {0x06}},
{{0x33, 0x2B}, {0x1C}},
{{0x33, 0x2C}, {0x1C}},
{{0x33, 0x2D}, {0x00}},
{{0x33, 0x2E}, {0x1D}},
{{0x33, 0x2F}, {0x1F}},
{{0x32, 0xF6}, {0xCF}},
{{0x32, 0xF9}, {0x62}},
{{0x32, 0xFA}, {0x26}},
{{0x33, 0x25}, {0x5F}},
{{0x33, 0x30}, {0x00}},
{{0x33, 0x31}, {0x04}},
{{0x33, 0x32}, {0xdc}},
{{0x33, 0x38}, {0x18}},
{{0x33, 0x39}, {0xa4}},
{{0x33, 0x3A}, {0x4a}},
{{0x33, 0x3F}, {0x07}},
//[Auto_Ctrl]
{{0x33, 0x60}, {0x0c}},//IQ_Param_Sel_0
{{0x33, 0x61}, {0x14}},//IQ_Param_Sel_1
{{0x33, 0x62}, {0x1F}},//IQ_Param_Sel_2
{{0x33, 0x63}, {0x37}},//IQ_Auto_Control
{{0x33, 0x64}, {0x98}},//CC_Ctrl_0
{{0x33, 0x65}, {0x88}},//CC_Ctrl_1
{{0x33, 0x66}, {0x70}},//CC_Ctrl_2
{{0x33, 0x67}, {0x60}},//CC_Ctrl_3
{{0x33, 0x68}, {0x8F}},//edge
{{0x33, 0x69}, {0x68}},
{{0x33, 0x6A}, {0x50}},
{{0x33, 0x6B}, {0x38}},
{{0x33, 0x6C}, {0x00}},
{{0x33, 0x6D}, {0x20}},//badpixel
{{0x33, 0x6E}, {0x1C}},
{{0x33, 0x6F}, {0x18}},
{{0x33, 0x70}, {0x10}},
{{0x33, 0x71}, {0x28}},//NR_Weight_0
{{0x33, 0x72}, {0x30}},//NR_Weight_1
{{0x33, 0x73}, {0x38}},
{{0x33, 0x74}, {0x3f}},
{{0x33, 0x75}, {0x06}},
{{0x33, 0x76}, {0x06}},
{{0x33, 0x77}, {0x06}},
{{0x33, 0x78}, {0x0a}},
{{0x33, 0x8A}, {0x34}},//GFilter_Ctrl_0
{{0x33, 0x8B}, {0x7F}},//GF_Dark_Level_0
{{0x33, 0x8C}, {0x10}},//GF_Comp_Max_0
{{0x33, 0x8D}, {0x23}},//GFilter_Ctrl_1
{{0x33, 0x8E}, {0x7F}},//GF_Dark_Level_1
{{0x33, 0x8F}, {0x14}},//GF_Comp_Max_1
//**********normal
{{0x30, 0x53}, {0x4f}},
{{0x32, 0xF2}, {0x7c}},//0x80
{{0x32, 0xFc}, {0x00}},
{{0x32, 0xB8}, {0x36}},//0x33
{{0x32, 0xB9}, {0x2a}},//0x27
{{0x32, 0xBC}, {0x30}},//0x2d
{{0x32, 0xBD}, {0x33}},//0x30
{{0x32, 0xBE}, {0x2d}},//0x2a

};

//25fps_64MPCLK
static struct regval_list sensor_hd720_regs[] = {
{{0x32, 0x0A}, {0x00}},//linebuffer off.
{{0x32, 0xBF}, {0x60}}, 
{{0x32, 0xC0}, {0x60}}, 
{{0x32, 0xC1}, {0x60}}, 
{{0x32, 0xC2}, {0x60}}, 
{{0x32, 0xC3}, {0x00}}, 
{{0x32, 0xC4}, {0x2B}}, 
{{0x32, 0xC5}, {0x20}}, 
{{0x32, 0xC6}, {0x20}}, 
{{0x32, 0xC7}, {0x00}}, 
{{0x32, 0xC8}, {0xC1}}, 
{{0x32, 0xC9}, {0x60}}, 
{{0x32, 0xCA}, {0x80}}, 
{{0x32, 0xCB}, {0x80}}, 
{{0x32, 0xCC}, {0x80}}, 
{{0x32, 0xCD}, {0x80}}, 
{{0x32, 0xDB}, {0x78}}, 
{{0x32, 0xE0}, {0x05}}, 
{{0x32, 0xE1}, {0x00}}, 
{{0x32, 0xE2}, {0x02}}, 
{{0x32, 0xE3}, {0xD0}}, 
{{0x32, 0xE4}, {0x00}}, 
{{0x32, 0xE5}, {0x00}}, 
{{0x32, 0xE6}, {0x00}}, 
{{0x32, 0xE7}, {0x00}}, 
{{0x32, 0x00}, {0x3E}}, 
{{0x32, 0x01}, {0x0F}}, 
{{0x30, 0x28}, {0x1F}}, 
{{0x30, 0x29}, {0x20}}, 
{{0x30, 0x2A}, {0x04}}, 
//{{0x30, 0x22}, {0x27}}, 
{{0x30, 0x23}, {0x24}}, 
{{0x30, 0x02}, {0x00}}, 
{{0x30, 0x03}, {0x04}}, 
{{0x30, 0x04}, {0x00}}, 
{{0x30, 0x05}, {0x04}}, 
{{0x30, 0x06}, {0x05}}, 
{{0x30, 0x07}, {0x03}}, 
{{0x30, 0x08}, {0x02}}, 
{{0x30, 0x09}, {0xD3}}, 
{{0x30, 0x0A}, {0x06}}, 
{{0x30, 0x0B}, {0x7C}}, 
{{0x30, 0x0C}, {0x03}}, 
{{0x30, 0x0D}, {0x03}}, 
{{0x30, 0x0E}, {0x05}},   /* 0x500 == 1280 */
{{0x30, 0x0F}, {0x00}}, 
{{0x30, 0x10}, {0x02}},   /* 0x2d0 == 720 */
{{0x30, 0x11}, {0xD0}},
{{0x32, 0xBB}, {0x87}},
{{0x32, 0x01}, {0x7F}}, 
{{0x32, 0xF1}, {0x00}},
{{0x30, 0x21}, {0x06}}, 
{{0x30, 0x60}, {0x01}}, 
};

static struct regval_list sensor_vga_regs[] = {
{{0x32, 0x0A}, {0xB2}},//linebuffer on.
{{0x32, 0xBF}, {0x60}}, 
{{0x32, 0xC0}, {0x60}}, 
{{0x32, 0xC1}, {0x60}}, 
{{0x32, 0xC2}, {0x60}}, 
{{0x32, 0xC3}, {0x00}}, 
{{0x32, 0xC4}, {0x2B}}, 
{{0x32, 0xC5}, {0x20}}, 
{{0x32, 0xC6}, {0x20}}, 
{{0x32, 0xC7}, {0x00}}, 
{{0x32, 0xC8}, {0xC1}}, 
{{0x32, 0xC9}, {0x60}}, 
{{0x32, 0xCA}, {0x80}}, 
{{0x32, 0xCB}, {0x80}}, 
{{0x32, 0xCC}, {0x80}}, 
{{0x32, 0xCD}, {0x80}}, 
{{0x32, 0xDB}, {0x78}}, 
{{0x32, 0xE0}, {0x02}}, 
{{0x32, 0xE1}, {0x80}}, 
{{0x32, 0xE2}, {0x01}}, 
{{0x32, 0xE3}, {0xE0}}, 
{{0x32, 0xE4}, {0x00}}, 
{{0x32, 0xE5}, {0x80}}, 
{{0x32, 0xE6}, {0x00}}, 
{{0x32, 0xE7}, {0x80}}, 
{{0x32, 0x00}, {0x3E}}, 
{{0x32, 0x01}, {0x0F}}, 
{{0x30, 0x28}, {0x1F}}, 
{{0x30, 0x29}, {0x20}}, 
{{0x30, 0x2A}, {0x04}}, 
{{0x30, 0x23}, {0x24}}, 
{{0x30, 0x02}, {0x00}}, 
{{0x30, 0x03}, {0xA4}}, 
{{0x30, 0x04}, {0x00}}, 
{{0x30, 0x05}, {0x04}}, 
{{0x30, 0x06}, {0x04}}, 
{{0x30, 0x07}, {0x63}}, 
{{0x30, 0x08}, {0x02}}, 
{{0x30, 0x09}, {0xD3}}, 
{{0x30, 0x0A}, {0x06}}, 
{{0x30, 0x0B}, {0x7C}}, 
{{0x30, 0x0C}, {0x03}}, 
{{0x30, 0x0D}, {0x03}}, 
{{0x30, 0x0E}, {0x03}},   /* 0x3c0 == 960 */
{{0x30, 0x0F}, {0xC0}}, 
{{0x30, 0x10}, {0x02}},   /* 0x2d0 == 720 */
{{0x30, 0x11}, {0xD0}}, 
{{0x32, 0xBB}, {0x87}},
{{0x32, 0x01}, {0x7F}}, 
{{0x32, 0xF1}, {0x00}},
{{0x30, 0x21}, {0x06}}, 
{{0x30, 0x60}, {0x01}},
};


/*
 * The white balance settings
 * Here only tune the R G B channel gain.
 * The white balance enalbe bit is modified in sensor_s_autowb and sensor_s_wb
 */
static struct regval_list sensor_wb_auto_regs[] = {
	{{0x32, 0x01}, {0x7F}},
	{{0x32, 0x90}, {0x01}},
	{{0x32, 0x91}, {0xA0}},
	{{0x32, 0x96}, {0x01}},
	{{0x32, 0x97}, {0x73}},
	{{0x30, 0x60}, {0x01}},
};

static struct regval_list sensor_wb_cloud_regs[] = {
	{{0x32, 0x01}, {0x6F}},
	{{0x32, 0x90}, {0x01}},
	{{0x32, 0x91}, {0x51}},
	{{0x32, 0x96}, {0x01}},
	{{0x32, 0x97}, {0x00}},
	{{0x30, 0x60}, {0x01}},
};

static struct regval_list sensor_wb_daylight_regs[] = {
	{{0x32, 0x01}, {0x6F}},
	{{0x32, 0x90}, {0x01}},
	{{0x32, 0x91}, {0x38}},
	{{0x32, 0x96}, {0x01}},
	{{0x32, 0x97}, {0x68}},
	{{0x30, 0x60}, {0x01}},
};

static struct regval_list sensor_wb_incandescence_regs[] = {
	//bai re guang
	{{0x32, 0x01}, {0x6F}},
	{{0x32, 0x90}, {0x01}},
	{{0x32, 0x91}, {0x30}},
	{{0x32, 0x96}, {0x01}},
	{{0x32, 0x97}, {0xcb}},
	{{0x30, 0x60} , {0x01}},
};

static struct regval_list sensor_wb_fluorescent_regs[] = {
	//ri guang deng
	{{0x32, 0x01}, {0x6F}},
	{{0x32, 0x90}, {0x01}},
	{{0x32, 0x91}, {0x70}},
	{{0x32, 0x96}, {0x01}},
	{{0x32, 0x97}, {0xff}},
	{{0x30, 0x60} , {0x01}},
};

static struct regval_list sensor_wb_tungsten_regs[] = {
	//wu si deng
	/* Office Colour Temperature : 3500K - 5000K  */
	{{0x32, 0x01}, {0x6F}},
	{{0x32, 0x90}, {0x01}},
	{{0x32, 0x91}, {0x00}},
	{{0x32, 0x96}, {0x02}},
	{{0x32, 0x97}, {0x30}},
	{{0x30, 0x60} , {0x01}},
};

/*
 * The color effect settings
 */
static struct regval_list sensor_colorfx_none_regs[] = {
	//sensor_Effect_Normal
	{{0x32, 0xf1} , {0x00}},
};

static struct regval_list sensor_colorfx_bw_regs[] = {
	//sensor_Effect_WandB
	{{0x32, 0xf1} , {0x01}},
};

static struct regval_list sensor_colorfx_sepia_regs[] = {
	//sensor_Effect_Sepia
	{{0x32, 0xf1} , {0x02}},
};

static struct regval_list sensor_colorfx_negative_regs[] = {
	//sensor_Effect_Negative
	{{0x32, 0xf1} , {0x03}},
};

static struct regval_list sensor_colorfx_emboss_regs[] = {
	//NULL
};

static struct regval_list sensor_colorfx_sketch_regs[] = {
	//NULL
};

static struct regval_list sensor_colorfx_sky_blue_regs[] = {
	//sensor_Effect_Bluish
	{{0x32, 0xf1} , {0x05}},
	{{0x32, 0xf4} , {0xf0}},
	{{0x32, 0xf5} , {0x80}},
};

static struct regval_list sensor_colorfx_grass_green_regs[] = {
	//sensor_Effect_Green
	{{0x32, 0xf1} , {0x05}},
	{{0x32, 0xf4} , {0x60}},
	{{0x32, 0xf5} , {0x20}},
};

static struct regval_list sensor_colorfx_skin_whiten_regs[] = {
	//NULL
};

static struct regval_list sensor_colorfx_vivid_regs[] = {
	//NULL
};

/*
 * The brightness setttings //John_gao
 */
static struct regval_list sensor_brightness_neg4_regs[] = {
	// Brightness -4
	{{0x32, 0xfc} , {0x80}},
	{{0x30, 0x60} , {0x01}},
};

static struct regval_list sensor_brightness_neg3_regs[] = {
	// Brightness -3
	{{0x32, 0xfc} , {0xa0}},
	{{0x30, 0x60} , {0x01}},
};

static struct regval_list sensor_brightness_neg2_regs[] = {
	// Brightness -2
	{{0x32, 0xfc} , {0xc0}},
	{{0x30, 0x60} , {0x01}},
};

static struct regval_list sensor_brightness_neg1_regs[] = {
	// Brightness -1
	{{0x32, 0xfc} , {0xe0}},
	{{0x30, 0x60} , {0x01}},
};

static struct regval_list sensor_brightness_zero_regs[] = {
	//  Brightness 0
	{{0x32, 0xfc} , {0x00}},
	{{0x30, 0x60} , {0x01}},
};

static struct regval_list sensor_brightness_pos1_regs[] = {
	// Brightness +1
	{{0x32, 0xfc} , {0x20}},
	{{0x30, 0x60} , {0x01}},
};

static struct regval_list sensor_brightness_pos2_regs[] = {
	//  Brightness +2
	{{0x32, 0xfc} , {0x40}},
	{{0x30, 0x60} , {0x01}},
};

static struct regval_list sensor_brightness_pos3_regs[] = {
	//  Brightness +3
	{{0x32, 0xfc} , {0x60}},
	{{0x30, 0x60} , {0x01}},
};

static struct regval_list sensor_brightness_pos4_regs[] = {
	//  Brightness +4
	{{0x32, 0xfc} , {0x7f}},
	{{0x30, 0x60} , {0x01}},
};

/*
 * The contrast setttings
 */
static struct regval_list sensor_contrast_neg4_regs[] = {
	//NULL
};

static struct regval_list sensor_contrast_neg3_regs[] = {
	//NULL
};

static struct regval_list sensor_contrast_neg2_regs[] = {
	//NULL
};

static struct regval_list sensor_contrast_neg1_regs[] = {
	//NULL
};

static struct regval_list sensor_contrast_zero_regs[] = {
	//NULL
};

static struct regval_list sensor_contrast_pos1_regs[] = {
	//NULL
};

static struct regval_list sensor_contrast_pos2_regs[] = {
	//NULL
};

static struct regval_list sensor_contrast_pos3_regs[] = {
	//NULL
};

static struct regval_list sensor_contrast_pos4_regs[] = {
	//NULL
};

/*
 * The saturation setttings  //John_gao
 */
static struct regval_list sensor_saturation_neg4_regs[] = {
	//[SATURATION : -4]
	{{0x32, 0xf1} , {0x05}},
	{{0x32, 0xf3} , {0x40}},
};

static struct regval_list sensor_saturation_neg3_regs[] = {
	//[SATURATION : -3]
	{{0x32, 0xf1} , {0x05}},
	{{0x32, 0xf3} , {0x50}},
};

static struct regval_list sensor_saturation_neg2_regs[] = {
	//[SATURATION : -2]
	{{0x32, 0xf1} , {0x05}},
	{{0x32, 0xf3} , {0x60}},
};

static struct regval_list sensor_saturation_neg1_regs[] = {
	//[SATURATION : -1]
	{{0x32, 0xf1} , {0x05}},
	{{0x32, 0xf3} , {0x70}},
};

static struct regval_list sensor_saturation_zero_regs[] = {
	//[SATURATION : 0]
	{{0x32, 0xf1} , {0x05}},
	{{0x32, 0xf3} , {0x80}},
};

static struct regval_list sensor_saturation_pos1_regs[] = {
	//[SATURATION : +1]
	{{0x32, 0xf1} , {0x05}},
	{{0x32, 0xf3} , {0x90}},
};

static struct regval_list sensor_saturation_pos2_regs[] = {
	//[SATURATION : +2]
	{{0x32, 0xf1} , {0x05}},
	{{0x32, 0xf3} , {0xA0}},
};

static struct regval_list sensor_saturation_pos3_regs[] = {
	//[SATURATION : +3]
	{{0x32, 0xf1} , {0x05}},
	{{0x32, 0xf3} , {0xB0}},
};

static struct regval_list sensor_saturation_pos4_regs[] = {
	//[SATURATION : +4]
	{{0x32, 0xf1} , {0x05}},
	{{0x32, 0xf3} , {0xC0}},
};

static struct regval_list sensor_flicker_50hz_reg_25Fps_64Mhz_MCLK24Mhz[] =
{
	//[50Hz]
	{{0x32, 0xBF}, {0x60}},
	{{0x32, 0xC0}, {0x60}},
	{{0x32, 0xC1}, {0x60}},
	{{0x32, 0xC2}, {0x60}},
	{{0x32, 0xC8}, {0xC1}},
	{{0x32, 0xC9}, {0x60}},
	{{0x32, 0xCA}, {0x80}},
	{{0x32, 0xCB}, {0x80}},
	{{0x32, 0xCC}, {0x80}},
	{{0x32, 0xCD}, {0x80}},
	{{0x32, 0xDB}, {0x78}},
};

static struct regval_list sensor_flicker_60hz_reg_25Fps_64Mhz_MCLK24Mhz[] =
{
	//[60Hz]
	{{0x32, 0xBF}, {0x60}},
	{{0x32, 0xC0}, {0x63}},
	{{0x32, 0xC1}, {0x63}},
	{{0x32, 0xC2}, {0x63}},
	{{0x32, 0xC8}, {0xA1}},
	{{0x32, 0xC9}, {0x63}},
	{{0x32, 0xCA}, {0x83}},
	{{0x32, 0xCB}, {0x83}},
	{{0x32, 0xCC}, {0x83}},
	{{0x32, 0xCD}, {0x83}},
	{{0x32, 0xDB}, {0x74}},
};
static struct regval_list sensor_flicker_50hz_reg_30Fps_64Mhz_MCLK24Mhz[] =
{
	//[50Hz]
	{{0x32, 0xBF}, {0x60}},
	{{0x32, 0xC0}, {0x5A}},
	{{0x32, 0xC1}, {0x5A}},
	{{0x32, 0xC2}, {0x5A}},
	{{0x32, 0xC8}, {0xDF}},
	{{0x32, 0xC9}, {0x5A}},
	{{0x32, 0xCA}, {0x7A}},
	{{0x32, 0xCB}, {0x7A}},
	{{0x32, 0xCC}, {0x7A}},
	{{0x32, 0xCD}, {0x7A}},
	{{0x32, 0xDB}, {0x7B}},
};

static struct regval_list sensor_flicker_60hz_reg_30Fps_64Mhz_MCLK24Mhz[] =
{
	//[60Hz]
	{{0x32, 0xBF}, {0x60}},
	{{0x32, 0xC0}, {0x60}},
	{{0x32, 0xC1}, {0x5F}},
	{{0x32, 0xC2}, {0x5F}},
	{{0x32, 0xC8}, {0xBA}},
	{{0x32, 0xC9}, {0x5F}},
	{{0x32, 0xCA}, {0x7F}},
	{{0x32, 0xCB}, {0x7F}},
	{{0x32, 0xCC}, {0x7F}},
	{{0x32, 0xCD}, {0x80}},
	{{0x32, 0xDB}, {0x77}},
};


/*
 * The exposure target setttings
 */
static struct regval_list sensor_ev_neg4_regs[] = {

	{{0x32, 0xf2} , {0x40}},

};

static struct regval_list sensor_ev_neg3_regs[] = {

	{{0x32, 0xf2} , {0x50}},

};

static struct regval_list sensor_ev_neg2_regs[] = {

	{{0x32, 0xf2} , {0x60}},

};

static struct regval_list sensor_ev_neg1_regs[] = {

	{{0x32, 0xf2} , {0x70}},

};

static struct regval_list sensor_ev_zero_regs[] = {
	//default
	{{0x32, 0xf2} , {0x80}},
};

static struct regval_list sensor_ev_pos1_regs[] = {

	{{0x32, 0xf2} , {0x90}},

};

static struct regval_list sensor_ev_pos2_regs[] = {

	{{0x32, 0xf2} , {0xa0}},

};

static struct regval_list sensor_ev_pos3_regs[] = {

	{{0x32, 0xf2} , {0xb0}},

};

static struct regval_list sensor_ev_pos4_regs[] = {

	{{0x32, 0xf2} , {0xc0}},

};


/*
 * Here we'll try to encapsulate the changes for just the output
 * video format.
 *
 */


static struct regval_list sensor_fmt_yuv422_yuyv[] = {

	{{0x32,	0xf0} , {0x01}}	//YCbYCr
};


static struct regval_list sensor_fmt_yuv422_yvyu[] = {

	{{0x32,	0xf0} , {0x03}}	//YCrYCb
};

static struct regval_list sensor_fmt_yuv422_vyuy[] = {

	{{0x32,	0xf0} , {0x02}}	//CrYCbY
};

static struct regval_list sensor_fmt_yuv422_uyvy[] = {

	{{0x32,	0xf0} , {0x00}}	//CbYCrY
};

//static struct regval_list sensor_fmt_raw[] = {
//	{{0x32,	0xf0} , {0x50}} //raw
//};


static struct i2c_board_info nt99141_i2c_board_info[] = {
        {
                .type = "nt99141",
                .addr = 0x54,
                //.platform_data = &axp_pdata,
                //.irq = pmu_irq_id,
        },
};




/*
 * Low-level register I/O.
 *
 */


/*
 * On most platforms, we'd rather do straight i2c I/O.
 */
static int sensor_read(struct v4l2_subdev *sd, unsigned char *reg,
					   unsigned char *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 data[REG_STEP];
	struct i2c_msg msg;
	int ret, i;

	for (i = 0; i < REG_ADDR_STEP; i++)
		data[i] = reg[i];

	for (i = REG_ADDR_STEP; i < REG_STEP; i++)
		data[i] = 0xff;
	/*
	 * Send out the register address...
	 */
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = REG_ADDR_STEP;
	msg.buf = data;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		csi_dev_err("Error %d on register write\n", ret);
		return ret;
	}
	/*
	 * ...then read back the result.
	 */

	msg.flags = I2C_M_RD;
	msg.len = REG_DATA_STEP;
	msg.buf = &data[REG_ADDR_STEP];

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret >= 0) {
		for (i = 0; i < REG_DATA_STEP; i++)
			value[i] = data[i + REG_ADDR_STEP];
		ret = 0;
	} else {
		csi_dev_err("Error %d on register read\n", ret);
	}
	return ret;
}


static int sensor_write(struct v4l2_subdev *sd, unsigned char *reg,
						unsigned char *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct i2c_msg msg;
	unsigned char data[REG_STEP];
	int ret, i;

	for (i = 0; i < REG_ADDR_STEP; i++)
		data[i] = reg[i];
	for (i = REG_ADDR_STEP; i < REG_STEP; i++)
		data[i] = value[i - REG_ADDR_STEP];

	//	for(i = 0; i < REG_STEP; i++)
	//		printk("data[%x]=%x\n",i,data[i]);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = REG_STEP;
	msg.buf = data;

	//	printk("msg.addr=%x\n",msg.addr);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret > 0) {
		ret = 0;
	} else if (ret < 0) {
		csi_dev_err("sensor_write error!\n");
	}
	return ret;
}



/*
 * Write a list of register settings;
 */
static int sensor_write_array(struct v4l2_subdev *sd, struct regval_list *vals , uint size)
{
	int i, ret;
	//	unsigned char rd;

	for (i = 0; i < size ; i++) {
		ret = sensor_write(sd, vals->reg_num, vals->value);
		if (ret < 0) {
			csi_dev_err("sensor_write_err!\n");
			return ret;
		}
		//		msleep(100);

		//		ret = sensor_read(sd, vals->reg_num, &rd);
		//
		//		if (ret < 0)
		//			{
		//				printk("sensor_read_err!\n");
		//				return ret;
		//			}
		//		if(rd != *vals->value)
		//			printk("read_val = %x\n",rd);

		vals++;
	}

	return 0;
}

static void csi_gpio_write(struct v4l2_subdev *sd, struct gpio_config *gpio, int level)
{
//	struct csi_dev *dev=(struct csi_dev *)dev_get_drvdata(sd->v4l2_dev->dev);
/**		
  if(gpio->port == 0xffff) {
//    axp_gpio_set_io(gpio->port_num, 1);
//    axp_gpio_set_value(gpio->port_num, status); 
  } else {
    gpio_write_one_pin_value(dev->csi_pin_hd,status,(char *)&gpio->gpio_name);
  }
  **/
   if(gpio->gpio==GPIO_INDEX_INVALID)
  {
    csi_dev_dbg("invalid gpio\n");
    return;
  }
  
	if(gpio->mul_sel==1)
	{
	  gpio_direction_output(gpio->gpio, level);
	  gpio->data=level;
	} else {
	  csi_dev_dbg("gpio is not in output function\n");
	}
  
}
/*
 * Stuff that knows about the sensor.
 */

static int sensor_power(struct v4l2_subdev *sd, int on)
{
	struct csi_dev *dev = (struct csi_dev *)dev_get_drvdata(sd->v4l2_dev->dev);
	struct sensor_info *info = to_state(sd);

	switch (on) {
		case CSI_SUBDEV_STBY_ON:
			csi_dev_dbg("CSI_SUBDEV_STBY_ON\n");
			
			info->streaming = 0;
			cancel_delayed_work_sync(&info->work);
			
			//reset off io
			csi_gpio_write(sd, &dev->reset_io, CSI_RST_OFF);
			//msleep(10);
			//active mclk before stadby in
			clk_enable(dev->csi_module_clk);
			//msleep(100);
			//standby on io
			csi_gpio_write(sd,&dev->standby_io,CSI_STBY_ON);
			//inactive mclk after stadby in
			clk_disable(dev->csi_module_clk);

			csi_gpio_write(sd, &dev->flash_io, dev->flash_pol ? 0 : 1);
			msleep(20);
			break;
		case CSI_SUBDEV_STBY_OFF:
			csi_dev_dbg("CSI_SUBDEV_STBY_OFF\n");
			//active mclk before stadby out
			clk_enable(dev->csi_module_clk);
			//msleep(10);
			//standby off io
			csi_gpio_write(sd,&dev->standby_io,CSI_STBY_OFF);
			//msleep(10);
			//reset off io
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_ON);
			msleep(1);
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_OFF);
			msleep(20);
			break;
		case CSI_SUBDEV_PWR_ON:
			csi_dev_dbg("CSI_SUBDEV_PWR_ON\n");
			//inactive mclk before power on
			clk_disable(dev->csi_module_clk);
			//power on reset
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_ON);
			//msleep(1);
			//active mclk before power on
			clk_enable(dev->csi_module_clk);
			//msleep(10);
			//power supply
			csi_gpio_write(sd,&dev->power_io,CSI_PWR_ON);
			//msleep(10);
			if (dev->dvdd) {
				regulator_enable(dev->dvdd);
				msleep(10);
			}
			if (dev->avdd) {
				regulator_enable(dev->avdd);
				msleep(10);
			}
			if (dev->iovdd) {
				regulator_enable(dev->iovdd);
				msleep(10);
			}
			//msleep(10);
			//reset after power on
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_OFF);
			msleep(1);
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_ON);
			msleep(1);
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_OFF);
			msleep(20);
			break;
		case CSI_SUBDEV_PWR_OFF:
#if 0
			csi_dev_dbg("CSI_SUBDEV_PWR_OFF\n");
			//standby and reset io
			gpio_write_one_pin_value(dev->csi_pin_hd, CSI_STBY_ON, csi_stby_str);
			//msleep(100);
			gpio_write_one_pin_value(dev->csi_pin_hd, CSI_RST_ON, csi_reset_str);
			//msleep(100);
			//power supply off
			if (dev->iovdd) {
				regulator_disable(dev->iovdd);
				msleep(10);
			}
			if (dev->avdd) {
				regulator_disable(dev->avdd);
				msleep(10);
			}
			if (dev->dvdd) {
				regulator_disable(dev->dvdd);
				msleep(10);
			}
			gpio_write_one_pin_value(dev->csi_pin_hd, CSI_PWR_OFF, csi_power_str);
			//msleep(10);

			//inactive mclk after power off
			clk_disable(dev->csi_module_clk);

			//set the io to hi-z
			gpio_set_one_pin_io_status(dev->csi_pin_hd, 0, csi_reset_str); //set the gpio to input
			gpio_set_one_pin_io_status(dev->csi_pin_hd, 0, csi_stby_str); //set the gpio to input
#else
			{
				int ret;

				struct regval_list regs;
				csi_dev_dbg("CSI_SUBDEV_PWR_OFF\n");

				info->streaming = 0;
				cancel_delayed_work_sync(&info->work);

				regs.reg_num[0] = 0x30;
				regs.reg_num[1] = 0x21;
				ret = sensor_read(sd, regs.reg_num, regs.value);
				if (ret < 0) {
					csi_dev_err("sensor_read err at sensor_s_vflip!\n");
				}

				regs.value[0] &= ~0x02;
				ret = sensor_write(sd, regs.reg_num, regs.value);
				if (ret < 0) {
					csi_dev_err("sensor_write err at sensor_s_vflip!\n");
				}
			}
#endif
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

static int sensor_reset(struct v4l2_subdev *sd, u32 val)
{
	struct csi_dev *dev = (struct csi_dev *)dev_get_drvdata(sd->v4l2_dev->dev);

	switch (val) {
		case CSI_SUBDEV_RST_OFF:
			csi_dev_dbg("CSI_SUBDEV_RST_OFF\n");
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_OFF);
			msleep(10);
			break;
		case CSI_SUBDEV_RST_ON:
			csi_dev_dbg("CSI_SUBDEV_RST_ON\n");
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_ON);
			msleep(10);
			break;
		case CSI_SUBDEV_RST_PUL:
			csi_dev_dbg("CSI_SUBDEV_RST_PUL\n");
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_OFF);
			msleep(10);
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_ON);
			msleep(100);
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_OFF);
			msleep(10);
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

static int sensor_detect(struct v4l2_subdev *sd)
{
	int ret;
	struct regval_list regs;
	csi_dev_dbg("CSI_Read_Sensor_ID\n");

	regs.reg_num[0] = 0x30;
	regs.reg_num[1] = 0x00;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_detect!\n");
		return ret;
	}
	csi_dev_dbg("Sensor ID =%x\n ", regs.value[0]);

	if (regs.value[0] != 0x14)		//NT99140(NT99141)
		return -ENODEV;

	return 0;
}



static int sensor_init(struct v4l2_subdev *sd, u32 val)
{
	int ret;
	csi_dev_dbg("sensor_init\n");
	/*Make sure it is a target sensor*/
	ret = sensor_detect(sd);
	if (ret) {
		csi_dev_err("chip found is not an target chip.\n");
		return ret;
	}

	ret = sensor_write_array(sd, sensor_default_regs , ARRAY_SIZE(sensor_default_regs));
	msleep(50);
	return ret;
}

static long sensor_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	int ret = 0;

	switch (cmd) {
		case CSI_SUBDEV_CMD_GET_INFO: {
			struct sensor_info *info = to_state(sd);
			__csi_subdev_info_t *ccm_info = arg;

			//			printk("CSI_SUBDEV_CMD_GET_INFO\n");

			ccm_info->mclk 	=	info->ccm_info->mclk ;
			ccm_info->vref 	=	info->ccm_info->vref ;
			ccm_info->href 	=	info->ccm_info->href ;
			ccm_info->clock	=	info->ccm_info->clock;
			ccm_info->iocfg	=	info->ccm_info->iocfg;

			//			printk("ccm_info.mclk=%x\n ",info->ccm_info->mclk);
			//			printk("ccm_info.vref=%x\n ",info->ccm_info->vref);
			//			printk("ccm_info.href=%x\n ",info->ccm_info->href);
			//			printk("ccm_info.clock=%x\n ",info->ccm_info->clock);
			//			printk("ccm_info.iocfg=%x\n ",info->ccm_info->iocfg);
			break;
		}
		case CSI_SUBDEV_CMD_SET_INFO: {
			struct sensor_info *info = to_state(sd);
			__csi_subdev_info_t *ccm_info = arg;

			//			printk("CSI_SUBDEV_CMD_SET_INFO\n");

			info->ccm_info->mclk 	=	ccm_info->mclk 	;
			info->ccm_info->vref 	=	ccm_info->vref 	;
			info->ccm_info->href 	=	ccm_info->href 	;
			info->ccm_info->clock	=	ccm_info->clock	;
			info->ccm_info->iocfg	=	ccm_info->iocfg	;

			//			printk("ccm_info.mclk=%x\n ",info->ccm_info->mclk);
			//			printk("ccm_info.vref=%x\n ",info->ccm_info->vref);
			//			printk("ccm_info.href=%x\n ",info->ccm_info->href);
			//			printk("ccm_info.clock=%x\n ",info->ccm_info->clock);
			//			printk("ccm_info.iocfg=%x\n ",info->ccm_info->iocfg);

			break;
		}
		default:
			break;
	}
	return ret;
}


/*
 * Store information about the video data format.
 */
static struct sensor_format_struct {
	__u8 *desc;
	//__u32 pixelformat;
	enum v4l2_mbus_pixelcode mbus_code;//linux-3.0
	struct regval_list *regs;
	int	regs_size;
	int bpp;   /* Bytes per pixel */
} sensor_formats[] = {
	{
		.desc		= "YUYV 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_YUYV8_2X8,//linux-3.0
		.regs 		= sensor_fmt_yuv422_yuyv,
		.regs_size = ARRAY_SIZE(sensor_fmt_yuv422_yuyv),
		.bpp		= 2,
	},
	{
		.desc		= "YVYU 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_YVYU8_2X8,//linux-3.0
		.regs 		= sensor_fmt_yuv422_yvyu,
		.regs_size = ARRAY_SIZE(sensor_fmt_yuv422_yvyu),
		.bpp		= 2,
	},
	{
		.desc		= "UYVY 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_UYVY8_2X8,//linux-3.0
		.regs 		= sensor_fmt_yuv422_uyvy,
		.regs_size = ARRAY_SIZE(sensor_fmt_yuv422_uyvy),
		.bpp		= 2,
	},
	{
		.desc		= "VYUY 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_VYUY8_2X8,//linux-3.0
		.regs 		= sensor_fmt_yuv422_vyuy,
		.regs_size = ARRAY_SIZE(sensor_fmt_yuv422_vyuy),
		.bpp		= 2,
	},
//	{
//		.desc		= "Raw RGB Bayer",
//		.mbus_code	= V4L2_MBUS_FMT_SBGGR8_1X8,//linux-3.0
//		.regs 		= sensor_fmt_raw,
//		.regs_size = ARRAY_SIZE(sensor_fmt_raw),
//		.bpp		= 1
//	},
};
#define N_FMTS ARRAY_SIZE(sensor_formats)



/*
 * Then there is the issue of window sizes.  Try to capture the info here.
 */


static struct sensor_win_size {
	int	width;
	int	height;
	int	hstart;		/* Start/stop values for the camera.  Note */
	int	hstop;		/* that they do not always make complete */
	int	vstart;		/* sense to humans, but evidently the sensor */
	int	vstop;		/* will do the right thing... */
	struct regval_list *regs; /* Regs to tweak */
	int regs_size;
	int (*set_size)(struct v4l2_subdev *sd);
	/* h/vref stuff */
} sensor_win_sizes[] = {
	/* UXGA */
	{
		.width			= UXGA_WIDTH,
		.height			= UXGA_HEIGHT,
		.regs 			= sensor_hd720_regs,
		.regs_size	= ARRAY_SIZE(sensor_hd720_regs),
		.set_size		= NULL,
	},
	/* VGA */
	{
		.width			= VGA_WIDTH,
		.height			= VGA_HEIGHT,
		.regs				= sensor_vga_regs,
		.regs_size	= ARRAY_SIZE(sensor_vga_regs),
		.set_size		= NULL,
	},
};

#define N_WIN_SIZES (ARRAY_SIZE(sensor_win_sizes))




static int sensor_enum_fmt(struct v4l2_subdev *sd, unsigned index,
						   enum v4l2_mbus_pixelcode *code)//linux-3.0
{
	//	struct sensor_format_struct *ofmt;

	if (index >= N_FMTS)//linux-3.0
		return -EINVAL;

	*code = sensor_formats[index].mbus_code;//linux-3.0

	//	ofmt = sensor_formats + fmt->index;
	//	fmt->flags = 0;
	//	strcpy(fmt->description, ofmt->desc);
	//	fmt->pixelformat = ofmt->pixelformat;
	return 0;
}


static int sensor_try_fmt_internal(struct v4l2_subdev *sd,
								   //struct v4l2_format *fmt,
								   struct v4l2_mbus_framefmt *fmt,//linux-3.0
								   struct sensor_format_struct **ret_fmt,
								   struct sensor_win_size **ret_wsize)
{
	int index;
	struct sensor_win_size *wsize;
	//	struct v4l2_pix_format *pix = &fmt->fmt.pix;//linux-3.0
	csi_dev_dbg("sensor_try_fmt_internal\n");
	for (index = 0; index < N_FMTS; index++)
		if (sensor_formats[index].mbus_code == fmt->code)//linux-3.0
			break;

	csi_dev_dbg("sensor_try_fmt_internal 001\n");
	if (index >= N_FMTS) {
		/* default to first format */
		index = 0;
		fmt->code = sensor_formats[0].mbus_code;//linux-3.0
	}

	csi_dev_dbg("sensor_try_fmt_internal 002\n");
	csi_dev_dbg("sensor_try_fmt_internal index = %d\n", index);
	
	if (ret_fmt != NULL)
		*ret_fmt = sensor_formats + index;

	/*
	 * Fields: the sensor devices claim to be progressive.
	 */
	fmt->field = V4L2_FIELD_NONE;//linux-3.0


	/*
	 * Round requested image size down to the nearest
	 * we support, but not below the smallest.
	 */
	for (wsize = sensor_win_sizes; wsize < sensor_win_sizes + N_WIN_SIZES;
		 wsize++)
		if (fmt->width >= wsize->width && fmt->height >= wsize->height)//linux-3.0
			break;
		
	csi_dev_dbg("sensor_try_fmt_internal 003\n");

	if (wsize >= sensor_win_sizes + N_WIN_SIZES)
		wsize--;   /* Take the smallest one */
	if (ret_wsize != NULL)
		*ret_wsize = wsize;
	/*
	 * Note the size we'll actually handle.
	 */
	fmt->width = wsize->width;//linux-3.0
	fmt->height = wsize->height;//linux-3.0
	//pix->bytesperline = pix->width*sensor_formats[index].bpp;//linux-3.0
	//pix->sizeimage = pix->height*pix->bytesperline;//linux-3.0

	csi_dev_dbg("sensor_try_fmt_internal h = %d w = %d\n", fmt->height, fmt->width );
	
	return 0;
}

static int sensor_try_fmt(struct v4l2_subdev *sd,
						  struct v4l2_mbus_framefmt *fmt)//linux-3.0
{
	return sensor_try_fmt_internal(sd, fmt, NULL, NULL);
}
#if 0
static unsigned char g_pv_shutter_reg3012 = 0x03;
static unsigned char g_pv_shutter_reg3013 = 0x00;

static unsigned char AGain_shutter_reg301d = 0x08;
static unsigned char Reg_updata_reg3060 = 0x01;

static int sensor_s_fmt_before(struct v4l2_subdev *sd)
{
	struct regval_list regs;
	int ret;
	//Stop AE AWB

	printk("sensor_s_fmt_beforei!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
	regs.reg_num[0] = 0x32;
	regs.reg_num[1] = 0x01;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	regs.value[0] &= 0xCF;
	ret = sensor_write(sd, regs.reg_num, regs.value);

	regs.reg_num[0] = 0x30;
	regs.reg_num[1] = 0x12;
	ret = sensor_read(sd, regs.reg_num, &g_pv_shutter_reg3012);

	regs.reg_num[0] = 0x30;
	regs.reg_num[1] = 0x13;
	ret = sensor_read(sd, regs.reg_num, &g_pv_shutter_reg3013);

	regs.reg_num[0] = 0x30;
	regs.reg_num[1] = 0x1d;
	ret = sensor_read(sd, regs.reg_num, &AGain_shutter_reg301d);

	return 0;
}
static int sensor_s_fmt_after(struct v4l2_subdev *sd)
{
	//For reshutter use
	struct regval_list regs;
	int ret;

	printk("sensor_s_fmt_after!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
	regs.reg_num[0] = 0x30;
	regs.reg_num[1] = 0x12;
	regs.value[0] = g_pv_shutter_reg3012;
	ret = sensor_write(sd, regs.reg_num, regs.value);

	regs.reg_num[0] = 0x30;
	regs.reg_num[1] = 0x13;
	regs.value[0] = g_pv_shutter_reg3013;
	ret = sensor_write(sd, regs.reg_num, regs.value);

	regs.reg_num[0] = 0x30;
	regs.reg_num[1] = 0x1d;
	regs.value[0] = AGain_shutter_reg301d;
	ret = sensor_write(sd, regs.reg_num, regs.value);

	regs.reg_num[0] = 0x30;
	regs.reg_num[1] = 0x60;
	regs.value[0] = Reg_updata_reg3060;
	ret = sensor_write(sd, regs.reg_num, regs.value);

	msleep(50);
	return 0;
}
#endif

#ifdef SYS_FS_RW
static int ninssTime = 0; //John_gao 0 == daylight_regs, 1==night_regs
struct device camera_device;
static ssize_t nt99141_show(struct device *dev,struct device_attribute *attr, char *buf){
	return sprintf(buf, "%d\n", ninssTime);;
}
static ssize_t nt99141_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	ninssTime = simple_strtol(buf, NULL, 10);

	return count;
}

static DEVICE_ATTR(ninssCam, S_IWUSR | S_IRUGO, nt99141_show, nt99141_store);
static int nt99141_sys(void)
{
	int ret;
	dev_set_name(&camera_device, "ninssCam");
	if (device_register(&camera_device))
                pr_err("error ninssCam device_register()\n");

        ret = device_create_file(&camera_device, &dev_attr_ninssCam);
        if (ret)
                pr_err("ninssCam device_create_file error\n");

        pr_err("ninssCam tvd device register ok and device_create_file ok\n");
        return 0;

}
#endif

/*
 * Set a format.
 */
static int sensor_s_fmt(struct v4l2_subdev *sd,
						struct v4l2_mbus_framefmt *fmt)//linux-3.0
{
//	struct regval_list regs;
	int ret;
	struct sensor_format_struct *sensor_fmt;
	struct sensor_win_size *wsize;
	struct sensor_info *info = to_state(sd);
	csi_dev_dbg("sensor_s_fmt\n");
	ret = sensor_try_fmt_internal(sd, fmt, &sensor_fmt, &wsize);
	if (ret)
		return ret;
	
	printk("chr wsize.width = [%d], wsize.height = [%d]\n", wsize->width, wsize->height);

	sensor_write_array(sd, sensor_fmt->regs , sensor_fmt->regs_size);
	/*
	   if (wsize->width == 1280 && wsize->height == 720)
	{
		sensor_s_fmt_before(sd);  // james added before capture
	}
	*/
	ret = 0;
	if (wsize->regs) {
		ret = sensor_write_array(sd, wsize->regs , wsize->regs_size);
		if (ret < 0)
			return ret;
	}
	if (wsize->width == 1280 && wsize->height == 720) {
		//sensor_s_fmt_after(sd); //james
		//msleep(125);
	}
	if ((wsize->width == 640 && wsize->height == 480) || (wsize->width == 800 && wsize->height == 600)) {
		/*
			regs.reg_num[0] = 0x32;
			regs.reg_num[1] = 0x01;
			ret = sensor_read(sd, regs.reg_num, regs.value);
			if (ret < 0) {
				csi_dev_err("sensor_read err @ sensor_s_fmt_before !\n");
				return ret;
			}

			regs.value[0] |= 0x30;
			ret = sensor_write(sd, regs.reg_num, regs.value);
		*/

		//msleep(20); //james 2012-2-15
	}

	//msleep(125);
	//msleep(125);
	if (wsize->set_size) {
		ret = wsize->set_size(sd);
		if (ret < 0)
			return ret;
	}

	info->fmt = sensor_fmt;
	info->width = wsize->width;
	info->height = wsize->height;
	info->streaming = 1;
	info->night_mode = 0;
	queue_delayed_work(info->wq, &info->work, msecs_to_jiffies(500));
/*
//John_gao  add 
	struct regval_list night_regs[] = {
		{{0x30, 0x53}, {0x4d}},
		{{0x32, 0xf2}, {0x6c}},
		{{0x32, 0xfc}, ninssTime},//{0x10}},
		{{0x32, 0xb8}, {0x11}},
		{{0x32, 0xb9}, {0x0b}},
		{{0x32, 0xbc}, {0x0e}},
		{{0x32, 0xbd}, {0x10}},
		{{0x32, 0xbe}, {0x0c}},
		{{0x30, 0x60}, {0x01}},
		{{0x33, 0x64}, {0x60}},
		{{0x33, 0x65}, {0x54}},
		{{0x33, 0x66}, {0x48}},
		{{0x33, 0x67}, {0x3C}},
	};
	struct regval_list daylight_regs[] = {
		{{0x30, 0x53}, {0x4f}},
		{{0x32, 0xf2}, {0x78}},
		{{0x32, 0xfc}, {0x02}},
		{{0x32, 0xb8}, {0x36}},
		{{0x32, 0xb9}, {0x2a}},
		{{0x32, 0xbc}, {0x30}},
		{{0x32, 0xbd}, {0x33}},
		{{0x32, 0xbe}, {0x2d}},
		{{0x30, 0x60}, {0x01}},
		{{0x33, 0x64}, {0x98}},
		{{0x33, 0x65}, {0x88}},
		{{0x33, 0x66}, {0x70}},
		{{0x33, 0x67}, {0x60}},
	};

//if(ninssTime == 1){
		pr_err("gaoliang set night_regs \n");
		ret = sensor_write_array(sd, night_regs, ARRAY_SIZE(night_regs));
		if (ret < 0) {
			pr_err("gaoliang set night_regs err, return %x!\n", ret);
			return ;
		}
}else if(ninssTime == 2){
		pr_err("gaoliang set daylight_regs \n");
		ret = sensor_write_array(sd, daylight_regs, ARRAY_SIZE(daylight_regs));
		if (ret < 0) {
			pr_err("gaoliang set daylight_regs err, return %x!\n", ret);
			return ;
		}
}
*/
//end John_gao 

	return 0;
}

/*
 * Implement G/S_PARM.  There is a "high quality" mode we could try
 * to do someday; for now, we just do the frame rate tweak.
 */
static int sensor_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct sensor_info *info = to_state(sd);

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(cp, 0, sizeof(struct v4l2_captureparm));
	cp->capability = V4L2_CAP_TIMEPERFRAME;
	cp->timeperframe.numerator = 1;

	if (info->width > VGA_WIDTH && info->height > VGA_HEIGHT) {
		cp->timeperframe.denominator = 25;
	} else {
		cp->timeperframe.denominator = 30;
	}

	return 0;
}

static int sensor_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	//	struct v4l2_captureparm *cp = &parms->parm.capture;
	//	struct v4l2_fract *tpf = &cp->timeperframe;
	//	struct sensor_info *info = to_state(sd);
	//	int div;

	//	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
	//		return -EINVAL;
	//	if (cp->extendedmode != 0)
	//		return -EINVAL;

	//	if (tpf->numerator == 0 || tpf->denominator == 0)
	//		div = 1;  /* Reset to full rate */
	//	else {
	//		if (info->width > SVGA_WIDTH && info->height > SVGA_HEIGHT) {
	//			div = (tpf->numerator*SENSOR_FRAME_RATE/2)/tpf->denominator;
	//		}
	//		else {
	//			div = (tpf->numerator*SENSOR_FRAME_RATE)/tpf->denominator;
	//		}
	//	}
	//
	//	if (div == 0)
	//		div = 1;
	//	else if (div > 8)
	//		div = 8;
	//
	//	switch()
	//
	//	info->clkrc = (info->clkrc & 0x80) | div;
	//	tpf->numerator = 1;
	//	tpf->denominator = sensor_FRAME_RATE/div;
	//
	//	sensor_write(sd, REG_CLKRC, info->clkrc);
	return 0;
}


/*
 * Code for dealing with controls.
 * fill with different sensor module
 * different sensor module has different settings here
 * if not support the follow function ,retrun -EINVAL
 */

/* *********************************************begin of ******************************************** */
static int sensor_queryctrl(struct v4l2_subdev *sd,
							struct v4l2_queryctrl *qc)
{
	/* Fill in min, max, step and default value for these controls. */
	/* see include/linux/videodev2.h for details */
	/* see sensor_s_parm and sensor_g_parm for the meaning of value */

	switch (qc->id) {
			//	case V4L2_CID_BRIGHTNESS:
			//		return v4l2_ctrl_query_fill(qc, -4, 4, 1, 1);
			//	case V4L2_CID_CONTRAST:
			//		return v4l2_ctrl_query_fill(qc, -4, 4, 1, 1);
			//	case V4L2_CID_SATURATION:
			//		return v4l2_ctrl_query_fill(qc, -4, 4, 1, 1);
			//	case V4L2_CID_HUE:
			//		return v4l2_ctrl_query_fill(qc, -180, 180, 5, 0);
		case V4L2_CID_VFLIP:
		case V4L2_CID_HFLIP:
			return v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
			//	case V4L2_CID_GAIN:
			//		return v4l2_ctrl_query_fill(qc, 0, 255, 1, 128);
			//	case V4L2_CID_AUTOGAIN:
			//		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 1);
		case V4L2_CID_EXPOSURE:
			return v4l2_ctrl_query_fill(qc, -4, 4, 1, 0);
		case V4L2_CID_EXPOSURE_AUTO:
			return v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
		case V4L2_CID_DO_WHITE_BALANCE:
			return v4l2_ctrl_query_fill(qc, 0, 5, 1, 0);
		case V4L2_CID_AUTO_WHITE_BALANCE:
			return v4l2_ctrl_query_fill(qc, 0, 1, 1, 1);
		case V4L2_CID_COLORFX:
			return v4l2_ctrl_query_fill(qc, 0, 9, 1, 0);
		case V4L2_CID_FLASH_LED_MODE:
			return v4l2_ctrl_query_fill(qc, 0, 4, 1, 0);
		case V4L2_CID_POWER_LINE_FREQUENCY:
			return v4l2_ctrl_query_fill(qc, 0, 2, 1, 0);
	}
	return -EINVAL;
}

static int sensor_g_hflip(struct v4l2_subdev *sd, __s32 *value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;

	regs.reg_num[0] = 0x30;
	regs.reg_num[1] = 0x22;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_g_hflip!\n");
		return ret;
	}

	regs.value[0] &= (1 << 1);
	regs.value[0] = regs.value[0] >> 1;		//0x0101 bit0 is mirror

	*value = regs.value[0];

	info->hflip = *value;
	return 0;
}

static int sensor_s_hflip(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;

	regs.reg_num[0] = 0x30;
	regs.reg_num[1] = 0x22;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_s_hflip!\n");
		return ret;
	}

	switch (value) {
		case 0:
			regs.value[0] &= ~0x02;
			break;
		case 1:
			regs.value[0] |= 0x02;
			break;
		default:
			return -EINVAL;
	}
	ret = sensor_write(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_write err at sensor_s_hflip!\n");
		return ret;
	}

	msleep(10);

	info->hflip = value;
	return 0;
}

static int sensor_g_vflip(struct v4l2_subdev *sd, __s32 *value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;

	regs.reg_num[0] = 0x30;
	regs.reg_num[1] = 0x22;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_g_vflip!\n");
		return ret;
	}

	regs.value[0] &= (1 << 0);
	regs.value[0] = regs.value[0] >> 0;		//0x0101 bit1 is upsidedown

	*value = regs.value[0];

	info->vflip = *value;
	return 0;
}

static int sensor_s_vflip(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;

	regs.reg_num[0] = 0x30;
	regs.reg_num[1] = 0x22;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_s_vflip!\n");
		return ret;
	}

	switch (value) {
		case 0:
			regs.value[0] &= ~0x01;
			break;
		case 1:
			regs.value[0] |= 0x01;
			break;
		default:
			return -EINVAL;
	}
	ret = sensor_write(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_write err at sensor_s_vflip!\n");
		return ret;
	}

	msleep(10);

	info->vflip = value;
	return 0;
}

static int sensor_g_autogain(struct v4l2_subdev *sd, __s32 *value)
{
	return -EINVAL;
}

static int sensor_s_autogain(struct v4l2_subdev *sd, int value)
{
	return -EINVAL;
}

static int sensor_g_autoexp(struct v4l2_subdev *sd, __s32 *value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;

	regs.reg_num[0] = 0x32;
	regs.reg_num[1] = 0x01;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_g_autoexp!\n");
		return ret;
	}

	regs.value[0] &= 0x20;
	if (regs.value[0] == 0x20) {
		*value = V4L2_EXPOSURE_AUTO;
	} else {
		*value = V4L2_EXPOSURE_MANUAL;
	}

	info->autoexp = *value;
	return 0;
}

static int sensor_s_autoexp(struct v4l2_subdev *sd,
							enum v4l2_exposure_auto_type value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;

	regs.reg_num[0] = 0x32;
	regs.reg_num[1] = 0x01;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_s_autoexp!\n");
		return ret;
	}

	switch (value) {
		case V4L2_EXPOSURE_AUTO:
			regs.value[0] |= 0x20;
			break;
		case V4L2_EXPOSURE_MANUAL:
			regs.value[0] &= 0xdf;
			break;
		case V4L2_EXPOSURE_SHUTTER_PRIORITY:
			return -EINVAL;
		case V4L2_EXPOSURE_APERTURE_PRIORITY:
			return -EINVAL;
		default:
			return -EINVAL;
	}

	ret = sensor_write(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_write err at sensor_s_autoexp!\n");
		return ret;
	}

	msleep(10);

	info->autoexp = value;
	return 0;
}

static int sensor_g_autowb(struct v4l2_subdev *sd, int *value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;

	regs.reg_num[0] = 0x32;
	regs.reg_num[1] = 0x01;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_g_autowb!\n");
		return ret;
	}

	regs.value[0] &= (1 << 4);
	regs.value[0] = regs.value[0] >> 4;		//0x031a bit7 is awb enable

	*value = regs.value[0];
	info->autowb = *value;

	return 0;
}

static int sensor_s_autowb(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;

pr_err("001\n");
	ret = sensor_write_array(sd, sensor_wb_auto_regs, ARRAY_SIZE(sensor_wb_auto_regs));
	if (ret < 0) {
		csi_dev_err("sensor_write_array err at sensor_s_autowb!\n");
		return ret;
	}

	regs.reg_num[0] = 0x32;
	regs.reg_num[1] = 0x01;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_s_autowb!\n");
		return ret;
	}

	switch (value) {
		case 0:
			regs.value[0] &= 0xef;
			break;
		case 1:
			regs.value[0] |= 0x10;
			break;
		default:
			break;
	}
	ret = sensor_write(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_write err at sensor_s_autowb!\n");
		return ret;
	}

	msleep(10);

	info->autowb = value;
	return 0;
}

static int sensor_g_hue(struct v4l2_subdev *sd, __s32 *value)
{
	return -EINVAL;
}

static int sensor_s_hue(struct v4l2_subdev *sd, int value)
{
	return -EINVAL;
}

static int sensor_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	return -EINVAL;
}

static int sensor_s_gain(struct v4l2_subdev *sd, int value)
{
	return -EINVAL;
}

static int sensor_g_band_filter(struct v4l2_subdev *sd, 
		__s32 *value)
{
	struct sensor_info *info = to_state(sd);
	
	*value = info->band_filter;
	return 0;
}

static int sensor_s_band_filter(struct v4l2_subdev *sd, 
		enum v4l2_power_line_frequency value)
{
	struct sensor_info *info = to_state(sd);

	int ret = 0;
	
	switch(value) {
		case V4L2_CID_POWER_LINE_FREQUENCY_AUTO:
		case V4L2_CID_POWER_LINE_FREQUENCY_DISABLED:
		case V4L2_CID_POWER_LINE_FREQUENCY_50HZ:
			if (info->width > VGA_WIDTH && info->height > VGA_HEIGHT) {
pr_err("002\n");
				ret = sensor_write_array(sd, sensor_flicker_50hz_reg_25Fps_64Mhz_MCLK24Mhz, ARRAY_SIZE(sensor_flicker_50hz_reg_25Fps_64Mhz_MCLK24Mhz));
			} else {
pr_err("003\n");
				ret = sensor_write_array(sd, sensor_flicker_50hz_reg_30Fps_64Mhz_MCLK24Mhz, ARRAY_SIZE(sensor_flicker_50hz_reg_30Fps_64Mhz_MCLK24Mhz));
			}
			if (ret < 0)
				csi_dev_err("sensor_write_array err at sensor_s_band_filter!\n");
			break;
		case V4L2_CID_POWER_LINE_FREQUENCY_60HZ:
			if (info->width > VGA_WIDTH && info->height > VGA_HEIGHT) {
pr_err("004\n");
				ret = sensor_write_array(sd, sensor_flicker_60hz_reg_25Fps_64Mhz_MCLK24Mhz, ARRAY_SIZE(sensor_flicker_60hz_reg_25Fps_64Mhz_MCLK24Mhz));
			} else {
pr_err("005\n");
				ret = sensor_write_array(sd, sensor_flicker_60hz_reg_30Fps_64Mhz_MCLK24Mhz, ARRAY_SIZE(sensor_flicker_60hz_reg_30Fps_64Mhz_MCLK24Mhz));
			}
			if (ret < 0)
				csi_dev_err("sensor_write_array err at sensor_s_band_filter!\n");
		  break;
	}
	//mdelay(10);
	info->band_filter = value;
	return ret;
}

/* *********************************************end of ******************************************** */

static int sensor_g_brightness(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);

	*value = info->brightness;
	return 0;
}

static int sensor_s_brightness(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);

	switch (value) {
		case -4:
pr_err("006\n");
			ret = sensor_write_array(sd, sensor_brightness_neg4_regs, ARRAY_SIZE(sensor_brightness_neg4_regs));
			break;
		case -3:
pr_err("007\n");
			ret = sensor_write_array(sd, sensor_brightness_neg3_regs, ARRAY_SIZE(sensor_brightness_neg3_regs));
			break;
		case -2:
pr_err("008\n");
			ret = sensor_write_array(sd, sensor_brightness_neg2_regs, ARRAY_SIZE(sensor_brightness_neg2_regs));
			break;
		case -1:
pr_err("009\n");
			ret = sensor_write_array(sd, sensor_brightness_neg1_regs, ARRAY_SIZE(sensor_brightness_neg1_regs));
			break;
		case 0:
pr_err("010\n");
			ret = sensor_write_array(sd, sensor_brightness_zero_regs, ARRAY_SIZE(sensor_brightness_zero_regs));
			break;
		case 1:
pr_err("011\n");
			ret = sensor_write_array(sd, sensor_brightness_pos1_regs, ARRAY_SIZE(sensor_brightness_pos1_regs));
			break;
		case 2:
pr_err("012\n");
			ret = sensor_write_array(sd, sensor_brightness_pos2_regs, ARRAY_SIZE(sensor_brightness_pos2_regs));
			break;
		case 3:
pr_err("013\n");
			ret = sensor_write_array(sd, sensor_brightness_pos3_regs, ARRAY_SIZE(sensor_brightness_pos3_regs));
			break;
		case 4:
pr_err("014\n");
			ret = sensor_write_array(sd, sensor_brightness_pos4_regs, ARRAY_SIZE(sensor_brightness_pos4_regs));
			break;
		default:
			return -EINVAL;
	}

	if (ret < 0) {
		csi_dev_err("sensor_write_array err at sensor_s_brightness!\n");
		return ret;
	}

	msleep(10);

	info->brightness = value;
	return 0;
}

static int sensor_g_contrast(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);

	*value = info->contrast;
	return 0;
}

static int sensor_s_contrast(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);

	switch (value) {
		case -4:
pr_err("015\n");
			ret = sensor_write_array(sd, sensor_contrast_neg4_regs, ARRAY_SIZE(sensor_contrast_neg4_regs));
			break;
		case -3:
pr_err("016\n");
			ret = sensor_write_array(sd, sensor_contrast_neg3_regs, ARRAY_SIZE(sensor_contrast_neg3_regs));
			break;
		case -2:
pr_err("017\n");
			ret = sensor_write_array(sd, sensor_contrast_neg2_regs, ARRAY_SIZE(sensor_contrast_neg2_regs));
			break;
		case -1:
pr_err("018\n");
			ret = sensor_write_array(sd, sensor_contrast_neg1_regs, ARRAY_SIZE(sensor_contrast_neg1_regs));
			break;
		case 0:
pr_err("019\n");
			ret = sensor_write_array(sd, sensor_contrast_zero_regs, ARRAY_SIZE(sensor_contrast_zero_regs));
			break;
		case 1:
pr_err("020\n");
			ret = sensor_write_array(sd, sensor_contrast_pos1_regs, ARRAY_SIZE(sensor_contrast_pos1_regs));
			break;
		case 2:
pr_err("021\n");
			ret = sensor_write_array(sd, sensor_contrast_pos2_regs, ARRAY_SIZE(sensor_contrast_pos2_regs));
			break;
		case 3:
pr_err("022\n");
			ret = sensor_write_array(sd, sensor_contrast_pos3_regs, ARRAY_SIZE(sensor_contrast_pos3_regs));
			break;
		case 4:
pr_err("023\n");
			ret = sensor_write_array(sd, sensor_contrast_pos4_regs, ARRAY_SIZE(sensor_contrast_pos4_regs));
			break;
		default:
			return -EINVAL;
	}

	if (ret < 0) {
		csi_dev_err("sensor_write_array err at sensor_s_contrast!\n");
		return ret;
	}

	msleep(10);

	info->contrast = value;
	return 0;
}

static int sensor_g_saturation(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);

	*value = info->saturation;
	return 0;
}

static int sensor_s_saturation(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);

	switch (value) {
		case -4:
pr_err("024\n");
			ret = sensor_write_array(sd, sensor_saturation_neg4_regs, ARRAY_SIZE(sensor_saturation_neg4_regs));
			break;
		case -3:
pr_err("025\n");
			ret = sensor_write_array(sd, sensor_saturation_neg3_regs, ARRAY_SIZE(sensor_saturation_neg3_regs));
			break;
		case -2:
pr_err("026\n");
			ret = sensor_write_array(sd, sensor_saturation_neg2_regs, ARRAY_SIZE(sensor_saturation_neg2_regs));
			break;
		case -1:
pr_err("027\n");
			ret = sensor_write_array(sd, sensor_saturation_neg1_regs, ARRAY_SIZE(sensor_saturation_neg1_regs));
			break;
		case 0:
pr_err("028\n");
			ret = sensor_write_array(sd, sensor_saturation_zero_regs, ARRAY_SIZE(sensor_saturation_zero_regs));
			break;
		case 1:
pr_err("029\n");
			ret = sensor_write_array(sd, sensor_saturation_pos1_regs, ARRAY_SIZE(sensor_saturation_pos1_regs));
			break;
		case 2:
pr_err("030\n");
			ret = sensor_write_array(sd, sensor_saturation_pos2_regs, ARRAY_SIZE(sensor_saturation_pos2_regs));
			break;
		case 3:
pr_err("031\n");
			ret = sensor_write_array(sd, sensor_saturation_pos3_regs, ARRAY_SIZE(sensor_saturation_pos3_regs));
			break;
		case 4:
pr_err("032\n");
			ret = sensor_write_array(sd, sensor_saturation_pos4_regs, ARRAY_SIZE(sensor_saturation_pos4_regs));
			break;
		default:
			return -EINVAL;
	}

	if (ret < 0) {
		csi_dev_err("sensor_write_array err at sensor_s_saturation!\n");
		return ret;
	}

	msleep(10);

	info->saturation = value;
	return 0;
}

static int sensor_g_exp(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);

	*value = info->exp;
	return 0;
}

static int sensor_s_exp(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);

pr_err("033  value = %d\n", value);
	switch (value) {
		case -4:
			ret = sensor_write_array(sd, sensor_ev_neg4_regs, ARRAY_SIZE(sensor_ev_neg4_regs));
			break;
		case -3:
			ret = sensor_write_array(sd, sensor_ev_neg3_regs, ARRAY_SIZE(sensor_ev_neg3_regs));
			break;
		case -2:
			ret = sensor_write_array(sd, sensor_ev_neg2_regs, ARRAY_SIZE(sensor_ev_neg2_regs));
			break;
		case -1:
			ret = sensor_write_array(sd, sensor_ev_neg1_regs, ARRAY_SIZE(sensor_ev_neg1_regs));
			break;
		case 0:
			ret = sensor_write_array(sd, sensor_ev_zero_regs, ARRAY_SIZE(sensor_ev_zero_regs));
			break;
		case 1:
			ret = sensor_write_array(sd, sensor_ev_pos1_regs, ARRAY_SIZE(sensor_ev_pos1_regs));
			break;
		case 2:
			ret = sensor_write_array(sd, sensor_ev_pos2_regs, ARRAY_SIZE(sensor_ev_pos2_regs));
			break;
		case 3:
			ret = sensor_write_array(sd, sensor_ev_pos3_regs, ARRAY_SIZE(sensor_ev_pos3_regs));
			break;
		case 4:
			ret = sensor_write_array(sd, sensor_ev_pos4_regs, ARRAY_SIZE(sensor_ev_pos4_regs));
			break;
		default:
			return -EINVAL;
	}

	if (ret < 0) {
		csi_dev_err("sensor_write_array err at sensor_s_exp!\n");
		return ret;
	}

	msleep(10);

	info->exp = value;
	return 0;
}

static int sensor_g_wb(struct v4l2_subdev *sd, int *value)
{
	struct sensor_info *info = to_state(sd);
	enum v4l2_whiteblance *wb_type = (enum v4l2_whiteblance *)value;

	*wb_type = info->wb;

	return 0;
}

static int sensor_s_wb(struct v4l2_subdev *sd,
					   enum v4l2_whiteblance value)
{
	int ret;
	struct sensor_info *info = to_state(sd);

	if (value == V4L2_WB_AUTO) {
		ret = sensor_s_autowb(sd, 1);
		return ret;
	} else {
		ret = sensor_s_autowb(sd, 0);
		if (ret < 0) {
			csi_dev_err("sensor_s_autowb error, return %x!\n", ret);
			return ret;
		}

pr_err("034\n");
		switch (value) {
			case V4L2_WB_CLOUD:
				ret = sensor_write_array(sd, sensor_wb_cloud_regs, ARRAY_SIZE(sensor_wb_cloud_regs));
				break;
			case V4L2_WB_DAYLIGHT:
				ret = sensor_write_array(sd, sensor_wb_daylight_regs, ARRAY_SIZE(sensor_wb_daylight_regs));
				break;
			case V4L2_WB_INCANDESCENCE:
				ret = sensor_write_array(sd, sensor_wb_incandescence_regs, ARRAY_SIZE(sensor_wb_incandescence_regs));
				break;
			case V4L2_WB_FLUORESCENT:
				ret = sensor_write_array(sd, sensor_wb_fluorescent_regs, ARRAY_SIZE(sensor_wb_fluorescent_regs));
				break;
			case V4L2_WB_TUNGSTEN:
				ret = sensor_write_array(sd, sensor_wb_tungsten_regs, ARRAY_SIZE(sensor_wb_tungsten_regs));
				break;
			default:
				return -EINVAL;
		}
	}

	if (ret < 0) {
		csi_dev_err("sensor_s_wb error, return %x!\n", ret);
		return ret;
	}

	msleep(10);

	info->wb = value;
	return 0;
}

static int sensor_g_colorfx(struct v4l2_subdev *sd,
							__s32 *value)
{
	struct sensor_info *info = to_state(sd);
	enum v4l2_colorfx *clrfx_type = (enum v4l2_colorfx *)value;

	*clrfx_type = info->clrfx;
	return 0;
}

static int sensor_s_colorfx(struct v4l2_subdev *sd,
							enum v4l2_colorfx value)
{
	int ret;
	struct sensor_info *info = to_state(sd);

pr_err("035 value = %d\n", value);
	switch (value) {
		case V4L2_COLORFX_NONE:
			ret = sensor_write_array(sd, sensor_colorfx_none_regs, ARRAY_SIZE(sensor_colorfx_none_regs));
			break;
		case V4L2_COLORFX_BW:
			ret = sensor_write_array(sd, sensor_colorfx_bw_regs, ARRAY_SIZE(sensor_colorfx_bw_regs));
			break;
		case V4L2_COLORFX_SEPIA:
			ret = sensor_write_array(sd, sensor_colorfx_sepia_regs, ARRAY_SIZE(sensor_colorfx_sepia_regs));
			break;
		case V4L2_COLORFX_NEGATIVE:
			ret = sensor_write_array(sd, sensor_colorfx_negative_regs, ARRAY_SIZE(sensor_colorfx_negative_regs));
			break;
		case V4L2_COLORFX_EMBOSS:
			ret = sensor_write_array(sd, sensor_colorfx_emboss_regs, ARRAY_SIZE(sensor_colorfx_emboss_regs));
			break;
		case V4L2_COLORFX_SKETCH:
			ret = sensor_write_array(sd, sensor_colorfx_sketch_regs, ARRAY_SIZE(sensor_colorfx_sketch_regs));
			break;
		case V4L2_COLORFX_SKY_BLUE:
			ret = sensor_write_array(sd, sensor_colorfx_sky_blue_regs, ARRAY_SIZE(sensor_colorfx_sky_blue_regs));
			break;
		case V4L2_COLORFX_GRASS_GREEN:
			ret = sensor_write_array(sd, sensor_colorfx_grass_green_regs, ARRAY_SIZE(sensor_colorfx_grass_green_regs));
			break;
		case V4L2_COLORFX_SKIN_WHITEN:
			ret = sensor_write_array(sd, sensor_colorfx_skin_whiten_regs, ARRAY_SIZE(sensor_colorfx_skin_whiten_regs));
			break;
		case V4L2_COLORFX_VIVID:
			ret = sensor_write_array(sd, sensor_colorfx_vivid_regs, ARRAY_SIZE(sensor_colorfx_vivid_regs));
			break;
		default:
			return -EINVAL;
	}

	if (ret < 0) {
		csi_dev_err("sensor_s_colorfx error, return %x!\n", ret);
		return ret;
	}

	msleep(10);

	info->clrfx = value;
	return 0;
}

static int sensor_g_flash_mode(struct v4l2_subdev *sd,
							   __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	enum v4l2_flash_led_mode *flash_mode = (enum v4l2_flash_led_mode *)value;

	*flash_mode = info->flash_mode;
	return 0;
}

static int sensor_s_flash_mode(struct v4l2_subdev *sd,
							   enum v4l2_flash_led_mode value)
{
	struct sensor_info *info = to_state(sd);
	struct csi_dev *dev = (struct csi_dev *)dev_get_drvdata(sd->v4l2_dev->dev);
	int flash_on, flash_off;

	flash_on = (dev->flash_pol != 0) ? 1 : 0;
	flash_off = (flash_on == 1) ? 0 : 1;

	switch (value) {
		case V4L2_FLASH_LED_MODE_NONE:
			csi_gpio_write(sd,&dev->flash_io,flash_off);
			break;
		case V4L2_FLASH_LED_MODE_TORCH:
			csi_gpio_write(sd,&dev->flash_io,flash_on);
			break;
//John_gao		case V4L2_FLASH_LED_MODE_AUTO:
//			break;
		default:
			return -EINVAL;
	}

	info->flash_mode = value;
	return 0;
}

static int sensor_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	switch (ctrl->id) {
		case V4L2_CID_BRIGHTNESS:
			return sensor_g_brightness(sd, &ctrl->value);
		case V4L2_CID_CONTRAST:
			return sensor_g_contrast(sd, &ctrl->value);
		case V4L2_CID_SATURATION:
			return sensor_g_saturation(sd, &ctrl->value);
		case V4L2_CID_HUE:
			return sensor_g_hue(sd, &ctrl->value);
		case V4L2_CID_VFLIP:
			return sensor_g_vflip(sd, &ctrl->value);
		case V4L2_CID_HFLIP:
			return sensor_g_hflip(sd, &ctrl->value);
		case V4L2_CID_GAIN:
			return sensor_g_gain(sd, &ctrl->value);
		case V4L2_CID_AUTOGAIN:
			return sensor_g_autogain(sd, &ctrl->value);
		case V4L2_CID_EXPOSURE:
			return sensor_g_exp(sd, &ctrl->value);
		case V4L2_CID_EXPOSURE_AUTO:
			return sensor_g_autoexp(sd, &ctrl->value);
		case V4L2_CID_DO_WHITE_BALANCE:
			return sensor_g_wb(sd, &ctrl->value);
		case V4L2_CID_AUTO_WHITE_BALANCE:
			return sensor_g_autowb(sd, &ctrl->value);
		case V4L2_CID_COLORFX:
			return sensor_g_colorfx(sd,	&ctrl->value);
		case V4L2_CID_FLASH_LED_MODE:
			return sensor_g_flash_mode(sd, &ctrl->value);
		case V4L2_CID_POWER_LINE_FREQUENCY:
			return sensor_g_band_filter(sd, &ctrl->value);
	}
	return -EINVAL;
}

static int sensor_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	switch (ctrl->id) {
		case V4L2_CID_BRIGHTNESS:
			return sensor_s_brightness(sd, ctrl->value);
		case V4L2_CID_CONTRAST:
			return sensor_s_contrast(sd, ctrl->value);
		case V4L2_CID_SATURATION:
			return sensor_s_saturation(sd, ctrl->value);
		case V4L2_CID_HUE:
			return sensor_s_hue(sd, ctrl->value);
		case V4L2_CID_VFLIP:
			return sensor_s_vflip(sd, ctrl->value);
		case V4L2_CID_HFLIP:
			return sensor_s_hflip(sd, ctrl->value);
		case V4L2_CID_GAIN:
			return sensor_s_gain(sd, ctrl->value);
		case V4L2_CID_AUTOGAIN:
			return sensor_s_autogain(sd, ctrl->value);
		case V4L2_CID_EXPOSURE:
			return sensor_s_exp(sd, ctrl->value);
		case V4L2_CID_EXPOSURE_AUTO:
			return sensor_s_autoexp(sd, (enum v4l2_exposure_auto_type) ctrl->value);
		case V4L2_CID_DO_WHITE_BALANCE:
			return sensor_s_wb(sd, (enum v4l2_whiteblance) ctrl->value);
		case V4L2_CID_AUTO_WHITE_BALANCE:
			return sensor_s_autowb(sd, ctrl->value);
		case V4L2_CID_COLORFX:
			return sensor_s_colorfx(sd, (enum v4l2_colorfx) ctrl->value);
		case V4L2_CID_FLASH_LED_MODE:
			return sensor_s_flash_mode(sd, (enum v4l2_flash_led_mode) ctrl->value);
		case V4L2_CID_POWER_LINE_FREQUENCY:
			return sensor_s_band_filter(sd, (enum v4l2_power_line_frequency)ctrl->value);
	}
	return -EINVAL;
}


static int sensor_g_chip_ident(struct v4l2_subdev *sd,
							   struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_SENSOR, 0);
}


/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops sensor_core_ops = {
	.g_chip_ident = sensor_g_chip_ident,
	.g_ctrl = sensor_g_ctrl,
	.s_ctrl = sensor_s_ctrl,
	.queryctrl = sensor_queryctrl,
	.reset = sensor_reset,
	.init = sensor_init,
	.s_power = sensor_power,
	.ioctl = sensor_ioctl,
};

static const struct v4l2_subdev_video_ops sensor_video_ops = {
	.enum_mbus_fmt = sensor_enum_fmt,//linux-3.0
	.try_mbus_fmt = sensor_try_fmt,//linux-3.0
	.s_mbus_fmt = sensor_s_fmt,//linux-3.0
	.s_parm = sensor_s_parm,//linux-3.0
	.g_parm = sensor_g_parm,//linux-3.0
};

static const struct v4l2_subdev_ops sensor_ops = {
	.core = &sensor_core_ops,
	.video = &sensor_video_ops,
};

/* ----------------------------------------------------------------------- */

static void auto_scene_worker(struct work_struct *work)
{
	struct sensor_info *info = container_of(work, struct sensor_info,work.work);
	struct v4l2_subdev *sd = &info->sd;
	struct csi_dev *dev = (struct csi_dev *)dev_get_drvdata(sd->v4l2_dev->dev);
	struct regval_list regs;
	int ret;
	struct regval_list night_regs[] = {
		{{0x30, 0x53}, {0x4d}},
		{{0x32, 0xf2}, {0x6c}},
		{{0x32, 0xfc}, {0x20}}, //John_gao ninssTime},//{0x10}},
		{{0x32, 0xb8}, {0x11}},
		{{0x32, 0xb9}, {0x0b}},
		{{0x32, 0xbc}, {0x0e}},
		{{0x32, 0xbd}, {0x10}},
		{{0x32, 0xbe}, {0x0c}},
		{{0x30, 0x60}, {0x01}},
		{{0x33, 0x64}, {0x60}},
		{{0x33, 0x65}, {0x54}},
		{{0x33, 0x66}, {0x48}},
		{{0x33, 0x67}, {0x3C}},
	};
	struct regval_list daylight_regs[] = {
		{{0x30, 0x53}, {0x4f}},
		{{0x32, 0xf2}, {0x78}},
		{{0x32, 0xfc}, {0x02}},
		{{0x32, 0xb8}, {0x36}},
		{{0x32, 0xb9}, {0x2a}},
		{{0x32, 0xbc}, {0x30}},
		{{0x32, 0xbd}, {0x33}},
		{{0x32, 0xbe}, {0x2d}},
		{{0x30, 0x60}, {0x01}},
		{{0x33, 0x64}, {0x98}},
		{{0x33, 0x65}, {0x88}},
		{{0x33, 0x66}, {0x70}},
		{{0x33, 0x67}, {0x60}},
	};

	if (!info->streaming) {
		return ;
	}

	regs.reg_num[0] = 0x32;
	regs.reg_num[1] = 0xD6;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("auto_scene_worker error.\n");
		return ;
	}
	if ((regs.value[0] > 0x9A) && (info->night_mode == 0)) {	//74,50//night mode
        pr_err("036\n");
		ret = sensor_write_array(sd, night_regs, ARRAY_SIZE(night_regs));
		if (ret < 0) {
			csi_dev_err("auto_scene error, return %x!\n", ret);
			return ;
		}
		info->night_mode = 1;
//John_gao		if (info->flash_mode == V4L2_FLASH_LED_MODE_AUTO) {
	//		csi_gpio_write(sd,&dev->flash_io,dev->flash_pol ? 1 : 0);
	//	}
		csi_dev_dbg("switch to night mode\n");
	} else  if ((regs.value[0] < 0x6D) && (info->night_mode == 1)) {  				//daylight mode
pr_err("037\n");
		ret = sensor_write_array(sd, daylight_regs, ARRAY_SIZE(daylight_regs));
		if (ret < 0) {
			csi_dev_err("auto_scene error, return %x!\n", ret);
			return ;
		}
		info->night_mode = 0;
//John_gao		if (info->flash_mode == V4L2_FLASH_LED_MODE_AUTO) {
	//		csi_gpio_write(sd,&dev->flash_io,!(dev->flash_pol ? 1 : 0));
	//	}
		csi_dev_dbg("switch to daylight mode\n");
	}
	queue_delayed_work(info->wq, &info->work, msecs_to_jiffies(500));
}
static int sensor_probe(struct i2c_client *client,
						const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct sensor_info *info;
	//	int ret;

	csi_dev_dbg("gaoliang  %s \n", __func__);
	
	info = kzalloc(sizeof(struct sensor_info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	
	sd = &info->sd;
	v4l2_i2c_subdev_init(sd, client, &sensor_ops);

	info->fmt = &sensor_formats[0];
	info->ccm_info = &ccm_info_con;
	info->brightness = 0;
	info->contrast = 0;
	info->saturation = 0;
	info->hue = 0;
	info->hflip = 0;
	info->vflip = 0;
	info->gain = 0;
	info->autogain = 1;
	info->exp = 0;
	info->autoexp = 0;
	info->autowb = 1;
	info->wb = 0;
	info->clrfx = 0;
	//	info->clkrc = 1;	/* 30fps */
    csi_dev_dbg("gaoliang  %s 002\n", __func__);

	info->wq = create_singlethread_workqueue("kworkqueue_nt99141");
	if (!info->wq) {
		dev_err(&client->dev, "Could not create workqueue\n");
	}
    csi_dev_dbg("gaoliang  %s 003\n", __func__);
	flush_workqueue(info->wq);
	INIT_DELAYED_WORK(&info->work, auto_scene_worker);
	
csi_dev_dbg("gaoliang  %s 004\n", __func__);
	info->night_mode = 0;
	info->streaming = 0;

	return 0;
}


static int sensor_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sensor_info *info = to_state(sd);

	cancel_delayed_work_sync(&info->work);
	destroy_workqueue(info->wq);

	v4l2_device_unregister_subdev(sd);
	kfree(to_state(sd));
	return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{ "nt99141", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sensor_id);

//linux-3.0
static struct i2c_driver sensor_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "nt99141",
	},
	.probe = sensor_probe,
	.remove = sensor_remove,
	.id_table = sensor_id,
};
static __init int init_sensor(void)
{
	csi_dev_dbg("gaoliang  %s \n", __func__);

#ifdef SYS_FS_RW
	nt99141_sys();
#endif 
	return i2c_add_driver(&sensor_driver);
}

static __exit void exit_sensor(void)
{
	i2c_del_driver(&sensor_driver);
}

module_init(init_sensor);
module_exit(exit_sensor);
