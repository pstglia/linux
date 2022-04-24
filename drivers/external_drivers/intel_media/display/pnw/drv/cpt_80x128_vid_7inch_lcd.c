/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * wang junxian: Himax LCD blink sometimes, maybe MIPI clock
 * need to adjust
 * ----------------------------------------------------------------------------
 */
/*
 * Copyright ? 2010 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Authors:
 * Xiaojun Wang <wang.xiaojun3@byd.com>
 */

#include "mdfld_dsi_dbi.h"
#include "mdfld_dsi_pkg_sender.h"
#include "mdfld_dsi_esd.h"
#include <asm/intel_scu_pmic.h>
#include <linux/gpio.h>
#include <psb_intel_reg.h>
//#include "displays/cpt_7_inch_vid.h"

/* Brightness related */
#define MINIMUM_BRIGHTNESS_LEVEL_20KHZ	15
/* MSIC PWM duty cycle goes up to 0x63 = 99% */
#define BACKLIGHT_DUTY_FACTOR	0x63
#define PWM0DUTYCYCLE		0x67

#define GPIOPWMCTRL	0x38F
#define PWM0CLKDIV0	0x62 /* low byte */
#define PWM0CLKDIV1	0x61 /* high byte */ 

#define MSICGPIO_H 		0x3B	//0x3D	//0x21	//0x3F
#define MSICGPIO_L 		0x3E	//0x3C	//0x20	// 0x3E

#define CPT_7_inch_PANEL_WIDTH 150
#define CPT_7_inch_PANEL_HEIGHT 94

#define LCD_PANEL_LDO_EN 		0x6E	 	//GPIO0HV2
#define LCD_PANEL_LDO_EN_2	 	0x73		//GPIO1HV1 control VDDI 3.3V
static int lcd_gpio_vcc_en = 175;		//gp_core_079  (GP_CAMERA_SB7) control VCCI 1.8V
static int lcd_gpio_stbyb = 177; 		//gp_core_081(GP_CAMERA_SB9)

static u8 cpt_7_inch_soft_reset[]      = {0x01, 0x00, 0x00, 0x00};
static u8 cpt_7_inch_set_RTERM[] = {0xAE, 0x0B, 0x00, 0x00};
static u8 cpt_7_inch_enter_bistb[] = {0xB1, 0xEF, 0x00, 0x00};

static
int cpt_7_inch_vid_ic_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);

	PSB_DEBUG_WARN("\n");
	
	if (!sender) {
		DRM_ERROR("Cannot get sender\n");
		return -EINVAL;
	}	

//	mdfld_dsi_send_mcs_short_hs(sender, cpt_7_inch_set_RTERM[0], cpt_7_inch_set_RTERM[1], 1, 0);

	mdfld_dsi_send_mcs_short_hs(sender, 0xEE, 0xEA, 1, 0);

	mdfld_dsi_send_mcs_short_hs(sender, 0xEF, 0x5F, 1, 0);

	mdfld_dsi_send_mcs_short_hs(sender, 0xF2, 0x68, 1, 0);

	mdfld_dsi_send_mcs_short_hs(sender, 0xB2, 0xFF, 1, 0);

	mdfld_dsi_send_mcs_short_hs(sender, 0xEE, 0x00, 1, 0);

	mdfld_dsi_send_mcs_short_hs(sender, 0xEF, 0x00, 1, 0);

	return 0;
}

static
int cpt_7_inch_vid_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err = 0;

	PSB_DEBUG_WARN("\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	/*send TURN_ON packet*/
	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender, 
		MDFLD_DSI_DPI_SPK_TURN_ON);
	if (err) {
		DRM_ERROR("Faild to send turn on packet\n");
//		return err;
	}

	msleep(5);
	err = mdfld_dsi_send_mcs_short_hs(sender, cpt_7_inch_soft_reset[0], cpt_7_inch_soft_reset[1], 0, 0);
	if (err) {
		DRM_ERROR("Faild to send SW reset packet\n");
//		return err;
	}
	msleep(15);

	//	cpt_7_inch_vid_ic_init(dsi_config);
	
	dsi_config->dsi_hw_context.panel_on = true;

	return 0;
}

static
int cpt_7_inch_vid_power_off(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err = 0;
	
	PSB_DEBUG_WARN("\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	mdfld_dsi_send_mcs_short_hs(sender, set_display_off, 0x00, 1, 0);
	msleep(2);

	mdfld_dsi_send_mcs_short_hs(sender, exit_sleep_mode, 0x00, 1, 0);
	msleep(5);	
	
	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,
			MDFLD_DSI_DPI_SPK_SHUT_DOWN);
	if (err) {
		DRM_ERROR("Faild to send turn on packet\n");
		return err;
		}

	msleep(20);
	err = intel_scu_ipc_iowrite8(LCD_PANEL_LDO_EN, MSICGPIO_L);
	err = intel_scu_ipc_iowrite8(LCD_PANEL_LDO_EN_2, MSICGPIO_L);
	if (err)
		pr_err("%s: LCD_PANEL_LDO_EN reset failed\n", __func__);

	gpio_direction_output(lcd_gpio_vcc_en, 0);
	msleep(400);

	dsi_config->dsi_hw_context.panel_on = false;
	return 0;
}

static
int cpt_7_inch_vid_detect(struct mdfld_dsi_config *dsi_config)
{
	int status;
	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	int pipe = dsi_config->pipe;
	u32 dpll_val, device_ready_val;

	PSB_DEBUG_ENTRY("\n");

	if (pipe == 0) {
		/*
		 * FIXME: WA to detect the panel connection status, and need to
		 * implement detection feature with get_power_mode DSI command.
		 */
		if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					OSPM_UHB_FORCE_POWER_ON)) {
			DRM_ERROR("hw begin failed\n");
			return -EAGAIN;
		}

		dpll_val = REG_READ(regs->dpll_reg);
		device_ready_val = REG_READ(regs->device_ready_reg);
		if ((device_ready_val & DSI_DEVICE_READY) &&
		    (dpll_val & DPLL_VCO_ENABLE)) {
			dsi_config->dsi_hw_context.panel_on = true;
			psb_enable_vblank(dev, pipe);
		} else {
			dsi_config->dsi_hw_context.panel_on = false;
			DRM_INFO("%s: panel is not detected!\n", __func__);
		}

		status = MDFLD_DSI_PANEL_CONNECTED;

		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	} else {
		DRM_INFO("%s: do NOT support dual panel\n", __func__);
		status = MDFLD_DSI_PANEL_DISCONNECTED;
	}

	return status;
}

static
int mdfld_dsi_cpt_7_inch_panel_reset(struct mdfld_dsi_config *dsi_config)
{
	int ret = 0;

	PSB_DEBUG_WARN("\n");

	ret = intel_scu_ipc_iowrite8(LCD_PANEL_LDO_EN, MSICGPIO_H);
	ret = intel_scu_ipc_iowrite8(LCD_PANEL_LDO_EN_2, MSICGPIO_H);
	if (ret)
		pr_err("%s: LCD_PANEL_LDO_EN set failed\n", __func__);

	mdelay(2);
	gpio_direction_output(lcd_gpio_vcc_en, 1);
	mdelay(10);

	return 0;
}

static
struct drm_display_mode *cpt_7_inch_vid_get_config_mode(void)
{
	struct drm_display_mode *mode;

	PSB_DEBUG_ENTRY("\n");

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;
	mode->hdisplay = 800;
	mode->vdisplay = 1280;

	mode->hsync_start = mode->hdisplay + 12;
	mode->hsync_end = mode->hsync_start + 4;
	mode->htotal = mode->hsync_end + 48;

	mode->vsync_start = mode->vdisplay + 10;
	mode->vsync_end = mode->vsync_start + 2;
	mode->vtotal = mode->vsync_end + 10;

	mode->vrefresh = 60;
	mode->clock = mode->vrefresh * mode->htotal * mode->vtotal / 1000;

	PSB_DEBUG_ENTRY("hdisplay is %d\n", mode->hdisplay);
	PSB_DEBUG_ENTRY("vdisplay is %d\n", mode->vdisplay);
	PSB_DEBUG_ENTRY("HSS is %d\n", mode->hsync_start);
	PSB_DEBUG_ENTRY("HSE is %d\n", mode->hsync_end);
	PSB_DEBUG_ENTRY("htotal is %d\n", mode->htotal);
	PSB_DEBUG_ENTRY("VSS is %d\n", mode->vsync_start);
	PSB_DEBUG_ENTRY("VSE is %d\n", mode->vsync_end);
	PSB_DEBUG_ENTRY("vtotal is %d\n", mode->vtotal);
	PSB_DEBUG_ENTRY("clock is %d\n", mode->clock);

	mode->type |= DRM_MODE_TYPE_PREFERRED;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	return mode;
}

static
void cpt_7_inch_vid_get_panel_info(int pipe, struct panel_info *pi)
{
	PSB_DEBUG_ENTRY("\n");

	if (pipe == 0) {
		pi->width_mm = CPT_7_inch_PANEL_WIDTH;
		pi->height_mm = CPT_7_inch_PANEL_HEIGHT;
	}
}

static
int cpt_7_inch_vid_set_brightness(struct mdfld_dsi_config *dsi_config, int level)
{
	int duty_val = 0;
	int ret = 0;

	#define MDFLD_DSI_BRIGHTNESS_MAX_LEVEL 125

	duty_val = level*BACKLIGHT_DUTY_FACTOR/MDFLD_DSI_BRIGHTNESS_MAX_LEVEL;
	
	PSB_DEBUG_ENTRY("level is %d and duty = %x\n", level, duty_val);

	ret = intel_scu_ipc_iowrite8(PWM0DUTYCYCLE, duty_val);
	if (ret) {
		printk(KERN_ERR "[DISPLAY] PWM0DUTYCYCLE: ipc write fail\n");
		return 0;
		}

	return 0;
}

static
void cpt_7_inch_vid_dsi_controller_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_context *hw_ctx = &dsi_config->dsi_hw_context;

	PSB_DEBUG_ENTRY("\n");

	dsi_config->lane_count = 4;
	dsi_config->bpp = 24;
	dsi_config->lane_config = MDFLD_DSI_DATA_LANE_4_0;
	dsi_config->video_mode = MDFLD_DSI_VIDEO_NON_BURST_MODE_SYNC_EVENTS;

	//702 parameters
	/* This is for 400 mhz.  Set it to 0 for 800mhz */
	hw_ctx->cck_div = 0;
	hw_ctx->pll_bypass_mode = 0;

	hw_ctx->mipi_control = 0x18;
	hw_ctx->intr_en = 0xffffffff;
	hw_ctx->hs_tx_timeout = 0xdcf50;
	hw_ctx->lp_rx_timeout = 0xffff;
	hw_ctx->turn_around_timeout = 0x14;
	hw_ctx->device_reset_timer = 0xffff;
	hw_ctx->high_low_switch_count = 0x18;
	hw_ctx->init_count = 0x7d0;
	hw_ctx->eot_disable = 0x03;//0x00;
	hw_ctx->lp_byteclk = 0x3;
	hw_ctx->clk_lane_switch_time_cnt = 0x18000b;
	hw_ctx->video_mode_format = 0x6;
	hw_ctx->dphy_param = 0x2A0D350C;	// 0x150D350C; // 0x1F0F350A;	// 0x15091F06; //0x160d3610;  //0x3F0d3510; 

	/*set up func_prg*/
	hw_ctx->dsi_func_prg = (0x200 | dsi_config->lane_count);
	hw_ctx->mipi = PASS_FROM_SPHY_TO_AFE | dsi_config->lane_config;
}

static void mdfld_dsi_cpt_7_inch_brightness_init(void)
{
	int ret;
	u8 pwmctrl;

	printk(KERN_INFO "[DISPLAY] %s: Enter\n", __func__);

	/* Make sure the PWM reference is the 19.2 MHz system clock. Read first
	 * instead of setting directly to catch potential conflicts between PWM
	 * users. */
	ret = intel_scu_ipc_ioread8(GPIOPWMCTRL, &pwmctrl);
	if (ret || pwmctrl != 0x01) {
		if (ret)
			pr_err("%s: GPIOPWMCTRL read failed\n", __func__);
		else
			PSB_DEBUG_ENTRY("GPIOPWMCTRL was not set to system"\
					"clock (pwmctrl = 0x%02x)\n", pwmctrl);

		ret = intel_scu_ipc_iowrite8(GPIOPWMCTRL, 0x01);
		if (ret)
			pr_err("%s: GPIOPWMCTRL set failed\n", __func__);
	}

	ret = intel_scu_ipc_iowrite8(PWM0CLKDIV1, 0x00);

	ret = intel_scu_ipc_iowrite8(PWM0CLKDIV0, 0x08);

	if (ret)
		pr_err("%s: PWM0CLKDIV set failed\n", __func__);
	else
		PSB_DEBUG_ENTRY("PWM0CLKDIV set to 0x%04x\n", 0x25);
}

static int cpt_7_inch_vid_gpio_init(void)
{
	int ret = 0;

	PSB_DEBUG_ENTRY("\n");

	ret = gpio_request(lcd_gpio_vcc_en, "LCD_VCC_ENABLE");	
	if (ret) {
		DRM_ERROR("Faild to request panel reset gpio\n");
//		return -EINVAL;
	}else{
		PSB_DEBUG_WARN("request gpio57 for panel reset succesful\n");
	}

	ret = gpio_request(lcd_gpio_stbyb, "LCD_STBYB_ENABLE");	
	if (ret) {
		DRM_ERROR("Faild to request panel stbyb gpio\n");
//		return -EINVAL;
	}else{
		PSB_DEBUG_WARN("request gpio57 for panel stbyb succesful\n");
		gpio_direction_output(lcd_gpio_stbyb, 1);
	}
#if	0
	ret = intel_scu_ipc_iowrite8(LCD_PANEL_LDO_EN, MSICGPIO_H);
	ret = intel_scu_ipc_iowrite8(LCD_PANEL_LDO_EN_2, MSICGPIO_H);
	if (ret)
		pr_err("%s: LCD_PANEL_LDO_EN set failed\n", __func__);

	mdelay(2);
	gpio_direction_output(lcd_gpio_vcc_en, 1);
#endif

	gpio_export(lcd_gpio_vcc_en, 0);
	gpio_export(lcd_gpio_stbyb, 0);
	
	return 0;
}

void cpt_7_inch_vid_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	PSB_DEBUG_ENTRY("\n");

	p_funcs->get_config_mode = cpt_7_inch_vid_get_config_mode;
	p_funcs->get_panel_info = cpt_7_inch_vid_get_panel_info;
	p_funcs->dsi_controller_init = cpt_7_inch_vid_dsi_controller_init;
	p_funcs->detect = cpt_7_inch_vid_detect;
	p_funcs->reset = mdfld_dsi_cpt_7_inch_panel_reset;
	p_funcs->power_on = cpt_7_inch_vid_power_on;
	p_funcs->power_off = cpt_7_inch_vid_power_off;
	p_funcs->set_brightness = cpt_7_inch_vid_set_brightness;
//	p_funcs->drv_ic_init = cpt_7_inch_vid_ic_init;

	cpt_7_inch_vid_gpio_init();
//	mdfld_dsi_cpt_7_inch_brightness_init();
}

static int cpt_7_inch_vid_probe(struct platform_device *pdev)
{
	DRM_INFO("%s: cpt 7inch panel detected\n", __func__);
	intel_mid_panel_register(cpt_7_inch_vid_init);

	return 0;
}

struct platform_driver cpt_7_inch_driver = {
	.probe	= cpt_7_inch_vid_probe,
	.driver	= {
		.name	= "CPT 7 inch VID",
		.owner	= THIS_MODULE,
	},
};

/*
static int __init cpt_7_inch_lcd_init(void)
{
	DRM_INFO("%s\n", __func__);
	
	return platform_driver_register(&cpt_7_inch_driver);
}

module_init(cpt_7_inch_lcd_init);
*/
