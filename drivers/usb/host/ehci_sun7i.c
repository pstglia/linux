/*
*************************************************************************************
*                         			      Linux
*					           USB Host Controller Driver
*
*				        (c) Copyright 2006-2012, SoftWinners Co,Ld.
*							       All Rights Reserved
*
* File Name 	: ehci_sun4i.c
*
* Author 		: javen
*
* Description 	: SoftWinner EHCI Driver
*
* Notes         :
*
* History 		:
*      <author>    		<time>       	<version >    		<desc>
*    yangnaitian      2011-5-24            1.0          create this file
*    javen            2011-6-26            1.1          add suspend and resume
*    javen            2011-7-18            1.2          时钟开关和供电开关从驱动移出来
*
*************************************************************************************
*/

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/timer.h>

#include <mach/sys_config.h>
#include <linux/clk.h>

#include  <mach/clock.h>
#include "sw_hci_sun7i.h"

/*.......................................................................................*/
//                               全局信息定义
/*.......................................................................................*/

//#define  SW_USB_EHCI_DEBUG

#define  SW_EHCI_NAME				"sw-ehci"
static const char ehci_name[] 		= SW_EHCI_NAME;

static struct sw_hci_hcd *g_sw_ehci[3];
static u32 ehci_first_probe[3] = {1, 1, 1};

/*.......................................................................................*/
//                                      函数区
/*.......................................................................................*/

extern int usb_disabled(void);
int sw_usb_disable_ehci(__u32 usbc_no);
int sw_usb_enable_ehci(__u32 usbc_no);
extern int sw_usb_suspend_disabled(int usbc_no);

/* test ehci usb eye */
static void  ehci_set_interrupt_enable(const struct ehci_hcd *ehci, __u32 regs, u32 enable)
{
	ehci_writel(ehci, enable & 0x3f, (__u32 __iomem *)(regs + EHCI_OPR_USBINTR));
}

static void ehci_disable_periodic_schedule(const struct ehci_hcd *ehci, __u32 regs)
{
	u32 reg_val = 0;
	reg_val =  ehci_readl(ehci, (__u32 __iomem *)(regs + EHCI_OPR_USBCMD));
	reg_val	&= ~(0x1<<4);
	ehci_writel(ehci, reg_val, (__u32 __iomem *)(regs + EHCI_OPR_USBCMD));
	//ehci_writel(pehci, EHCI_OPR_USBCMD, ehci_readl(pehci, EHCI_OPR_USBCMD) & ~(0x1<<4));
}

static void ehci_disable_async_schedule(const struct ehci_hcd *ehci, __u32 regs)
{
	unsigned int reg_val = 0;
	reg_val =  ehci_readl(ehci, (__u32 __iomem *)(regs + EHCI_OPR_USBCMD));
	reg_val &= ~(0x1<<5);
	ehci_writel(ehci, reg_val, (__u32 __iomem *)(regs + EHCI_OPR_USBCMD));
	//ehci_writel(pehci, EHCI_OPR_USBCMD, ehci_readl(pehci, EHCI_OPR_USBCMD) & ~(0x1<<5));
}

static void ehci_set_config_flag(const struct ehci_hcd *ehci, __u32 regs)
{
	ehci_writel(ehci, 0x1, (__u32 __iomem *)(regs + EHCI_OPR_CFGFLAG));
	//ehci_writel(pehci, EHCI_OPR_CFGFLAG, 0x1);
}

static void ehci_test_stop(const struct ehci_hcd *ehci, __u32 regs)
{
	unsigned int reg_val = 0;
	reg_val =  ehci_readl(ehci, (__u32 __iomem *)(regs + EHCI_OPR_USBCMD));
	reg_val &= (~0x1);
	ehci_writel(ehci, reg_val, (__u32 __iomem *)(regs + EHCI_OPR_USBCMD));
}

static void ehci_test_reset(const struct ehci_hcd *ehci, __u32 regs)
{
	u32 reg_val = 0;
	reg_val =  ehci_readl(ehci, (__u32 __iomem *)(regs + EHCI_OPR_USBCMD));
	reg_val	|= (0x1<<1);
	ehci_writel(ehci, reg_val, (__u32 __iomem *)(regs + EHCI_OPR_USBCMD));
}

static unsigned int ehci_test_reset_complete(const struct ehci_hcd *ehci, __u32  regs)
{

	unsigned int reg_val = 0;

	reg_val = ehci_readl(ehci, (__u32 __iomem *)(regs + EHCI_OPR_USBCMD));
	reg_val &= (0x1<<1);

	return !reg_val;
}

static void ehci_start(const struct ehci_hcd *ehci, __u32 regs)
{
	unsigned int reg_val = 0;
	reg_val =  ehci_readl(ehci, (__u32 __iomem *)(regs + EHCI_OPR_USBCMD));
	reg_val	|= 0x1;
	ehci_writel(ehci, reg_val, (__u32 __iomem *)(regs + EHCI_OPR_USBCMD));
}

static unsigned int ehci_is_halt(const struct ehci_hcd *ehci, __u32 regs)
{

	unsigned int reg_val = 0;
	reg_val = ehci_readl(ehci, (__u32 __iomem *)(regs + EHCI_OPR_USBSTS))  >> 12;
	reg_val &= 0x1;
	return reg_val;
}

static void ehci_port_control(const struct ehci_hcd *ehci, __u32 regs, u32 port_no, u32 control)
{
	ehci_writel(ehci, control, (__u32 __iomem *)(regs + EHCI_OPR_USBCMD + (port_no<<2)));
}

static void  ehci_put_port_suspend(const struct ehci_hcd *ehci, __u32 regs)
{
	unsigned int reg_val = 0;
	reg_val =  ehci_readl(ehci, (__u32 __iomem *)(regs + EHCI_OPR_PORTSC));
	reg_val	|= (0x01<<7);
	ehci_writel(ehci, reg_val, (__u32 __iomem *)(regs + EHCI_OPR_PORTSC));
}
static void ehci_test_mode(const struct ehci_hcd *ehci, __u32 regs, u32 test_mode)
{
	unsigned int reg_val = 0;
	reg_val =  ehci_readl(ehci, (__u32 __iomem *)(regs + EHCI_OPR_PORTSC));
	reg_val &= ~(0x0f<<16);
	reg_val |= test_mode;
	ehci_writel(ehci, reg_val, (__u32 __iomem *)(regs + EHCI_OPR_PORTSC));
}

void __ehci_ed_test(const struct ehci_hcd *ehci, __u32 regs, __u32 test_mode)
{
	ehci_set_interrupt_enable(ehci, regs, 0x00);
	ehci_disable_periodic_schedule(ehci, regs);
	ehci_disable_async_schedule(ehci, regs);

	ehci_set_config_flag(ehci, regs);

	ehci_test_stop(ehci, regs);
	ehci_test_reset(ehci, regs);
	while(!ehci_test_reset_complete(ehci, regs));  //Wait until EHCI reset complete

	if(!ehci_is_halt(ehci, regs))
	{
		pr_err("%s_%d\n", __func__, __LINE__);
	}

	ehci_start(ehci, regs);
	while(ehci_is_halt(ehci, regs));  //Wait until EHCI to be not halt

    /*Ehci Start, Config to Test*/
	ehci_set_config_flag(ehci, regs);
	ehci_port_control(ehci, regs, 0, EHCI_PORTSC_POWER);

	ehci_disable_periodic_schedule(ehci, regs);
	ehci_disable_async_schedule(ehci, regs);

    //Put Port Suspend

	ehci_put_port_suspend(ehci, regs);

   ehci_test_stop(ehci, regs);


   while((!ehci_is_halt(ehci, regs)));

   //Test Pack
   DMSG_INFO("Start Host Test,mode:0x%x!\n", test_mode);
   ehci_test_mode(ehci, regs, test_mode);
   DMSG_INFO("End Host Test,mode:0x%x!\n", test_mode);

}
#ifndef CONFIG_ARCH_SUN9IW1
static __u32 USBC_Phy_TpRead(__u32 usbc_no, __u32 addr, __u32 len)
{

	__u32 temp, ret;
	__u32 i = 0;
	__u32 j = 0;

	ret = 0;
	for(j = len; j > 0; j--)
	{
		USBC_Writeb((addr + j - 1), USBC_Phy_GetCsr(usbc_no) + 1);
		for(i = 0; i < 0x4; i++);
		temp = USBC_Readb(USBC_Phy_GetCsr(usbc_no) + 2);
		ret <<= 1;
		ret |= ((temp >> usbc_no) & 0x1);
	}
	return ret;
}

static int USBC_Width_Write(__u32 usbc_no, __u32 addr, __u32 data, __u32 len)
{
	__u32 temp = 0, dtmp = 0;
	__u32 j = 0;

	dtmp = data;
	for(j = 0; j < len; j++)
	{
		/* set the bit address to be write */
		temp = USBC_Readl(USBC_Phy_GetCsr(usbc_no));
		temp &= ~(0xff << 8);
		temp |= ((addr + j) << 8);
		USBC_Writel(temp, USBC_Phy_GetCsr(usbc_no));

		temp = USBC_Readb(USBC_Phy_GetCsr(usbc_no));
		temp &= ~(0x1 << 7);
		temp |= (dtmp & 0x1) << 7;
		temp &= ~(0x1 << (usbc_no << 1));
		USBC_Writeb(temp, USBC_Phy_GetCsr(usbc_no));

		temp = USBC_Readb(USBC_Phy_GetCsr(usbc_no));
		temp |= (0x1 << (usbc_no << 1));
		USBC_Writeb( temp, USBC_Phy_GetCsr(usbc_no));

		temp = USBC_Readb(USBC_Phy_GetCsr(usbc_no));
		temp &= ~(0x1 << (usbc_no <<1 ));
		USBC_Writeb(temp, USBC_Phy_GetCsr(usbc_no));
		dtmp >>= 1;
	}

	return data;
}

static int USBC_Edge_Write(__u32 usbc_no, __u32 addr, __u32 data, __u32 len)
{
	__u32 temp = 0, dtmp = 0;
	__u32 j = 2;

	dtmp = data;
	for(j = 2; j < (len + 2); j++)
	{
		/* set the bit address to be write */
		temp = USBC_Readl(USBC_Phy_GetCsr(usbc_no));
		temp &= ~(0xff << 8);
		temp |= ((addr + j) << 8);
		USBC_Writel(temp, USBC_Phy_GetCsr(usbc_no));

		temp = USBC_Readb(USBC_Phy_GetCsr(usbc_no));
		temp &= ~(0x1 << 7);
		temp |= (dtmp & 0x1) << 7;
		temp &= ~(0x1 << (usbc_no << 1));
		USBC_Writeb(temp, USBC_Phy_GetCsr(usbc_no));

		temp = USBC_Readb(USBC_Phy_GetCsr(usbc_no));
		temp |= (0x1 << (usbc_no << 1));
		USBC_Writeb( temp, USBC_Phy_GetCsr(usbc_no));

		temp = USBC_Readb(USBC_Phy_GetCsr(usbc_no));
		temp &= ~(0x1 << (usbc_no <<1 ));
		USBC_Writeb(temp, USBC_Phy_GetCsr(usbc_no));
		dtmp >>= 1;
	}

	return data;
}
#endif

static ssize_t show_ed_test(struct device *dev, struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t ehci_ed_test(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	__u32 test_mode = 0;
	struct usb_hcd *hcd = NULL;
	struct ehci_hcd *ehci = NULL;
	struct sw_hci_hcd *sw_ehci = NULL;
#ifndef CONFIG_ARCH_SUN9IW1
	int adjust_width = 0;
	int adjust_edge = 0;
	char *ptr2 = NULL;
#endif

	if(dev == NULL){
		DMSG_PANIC("ERR: Argment is invalid\n");
		return 0;
	}

	hcd = dev_get_drvdata(dev);
	if(hcd == NULL){
		DMSG_PANIC("ERR: hcd is null\n");
		return 0;
	}

	sw_ehci = dev->platform_data;
	if(sw_ehci == NULL){
		DMSG_PANIC("ERR: sw_ehci is null\n");
		return 0;
	}

	ehci = hcd_to_ehci(hcd);
	if(ehci == NULL){
		DMSG_PANIC("ERR: ehci is null\n");
		return 0;
	}

	mutex_lock(&dev->mutex);

	DMSG_INFO("ehci_ed_test:%s\n", buf);

	if(!strncmp(buf, "test_not_operating", 18)){
		test_mode = EHCI_PORTSC_PTC_DIS;
		DMSG_INFO("test_mode:0x%x\n", test_mode);
	}else if(!strncmp(buf, "test_j_state", 12)){
		test_mode = EHCI_PORTSC_PTC_J;
		DMSG_INFO("test_mode:0x%x\n", test_mode);
	}else if(!strncmp(buf, "test_k_state", 12)){
		test_mode = EHCI_PORTSC_PTC_K;
		DMSG_INFO("test_mode:0x%x\n", test_mode);
	}else if(!strncmp(buf, "test_se0_nak", 12)){
		test_mode = EHCI_PORTSC_PTC_SE0NAK;
		DMSG_INFO("test_mode:0x%x\n", test_mode);
	}else if(!strncmp(buf, "test_pack_adjust_amplitude", 26)){

		printk("test_mode:%s\n", buf);
#ifndef CONFIG_ARCH_SUN9IW1
		while((ptr2 = strchr(buf, ',')) != NULL) {

			buf = ptr2 + 1;

			sscanf(buf, "%x", &adjust_width);

			if(adjust_width > 3){
				printk("adjust width is %d, greater than Max value 3\n", adjust_width);
			}else{
				USBC_Width_Write(sw_ehci->usbc_no, 0x20, adjust_width, 2);
				printk("adjust_amplitude:%x\n", (USBC_Phy_TpRead(sw_ehci->usbc_no, 0x20, 5) & 0x3));
			}
		}
#endif
		test_mode = EHCI_PORTSC_PTC_PACKET;

	}else if(!strncmp(buf, "test_pack_adjust_slew_rate", 26)){
		printk("test_mode:%s\n", buf);

#ifndef CONFIG_ARCH_SUN9IW1
		while((ptr2 = strchr(buf, ',')) != NULL) {
			buf = ptr2 + 1;
			sscanf(buf, "%x", &adjust_edge);
			if(adjust_edge > 7){
				printk("adjust slew_rate is %d, greater than Max value 7\n", adjust_edge);
			}else{
				USBC_Edge_Write(sw_ehci->usbc_no, 0x20, adjust_edge, 3);
				printk("adjust_slew_rate:%x\n",((USBC_Phy_TpRead(sw_ehci->usbc_no, 0x20, 5) >> 2) & 0x7));
			}
		}
#endif
		test_mode = EHCI_PORTSC_PTC_PACKET;
	}else if(!strncmp(buf, "test_pack", 9)){
		test_mode = EHCI_PORTSC_PTC_PACKET;
		DMSG_INFO("test_mode:0x%x\n", test_mode);
	}else if(!strncmp(buf, "test_force_enable", 17)){
		test_mode = EHCI_PORTSC_PTC_FORCE;
		DMSG_INFO("test_mode:0x%x\n", test_mode);
	}else if(!strncmp(buf, "test_mask", 9)){
		test_mode = EHCI_PORTSC_PTC_MASK;
		DMSG_INFO("test_mode:0x%x\n", test_mode);
	}else {
		DMSG_PANIC("ERR: test_mode Argment is invalid\n");
		mutex_unlock(&dev->mutex);
		return count;
	}

	DMSG_INFO("regs: 0x%p\n",hcd->regs);
	__ehci_ed_test(ehci, (__u32)hcd->regs, test_mode);

	mutex_unlock(&dev->mutex);

	return count;
}
static DEVICE_ATTR(ed_test, 0644, show_ed_test, ehci_ed_test);
/*end usb eye*/

void print_ehci_info(struct sw_hci_hcd *sw_ehci)
{
    DMSG_INFO("----------print_ehci_info---------\n");
	DMSG_INFO("hci_name             = %s\n", sw_ehci->hci_name);
	DMSG_INFO("irq_no               = %d\n", sw_ehci->irq_no);
	DMSG_INFO("usbc_no              = %d\n", sw_ehci->usbc_no);

	DMSG_INFO("usb_vbase            = 0x%p\n", sw_ehci->usb_vbase);
	DMSG_INFO("sram_vbase           = 0x%p\n", sw_ehci->sram_vbase);
	DMSG_INFO("clock_vbase          = 0x%p\n", sw_ehci->clock_vbase);
	DMSG_INFO("sdram_vbase          = 0x%p\n", sw_ehci->sdram_vbase);

	DMSG_INFO("clock: AHB(0x%x), USB(0x%x)\n",
	          (u32)USBC_Readl(sw_ehci->clock_vbase + 0x60),
              (u32)USBC_Readl(sw_ehci->clock_vbase + 0xcc));

	DMSG_INFO("USB: 0x%x\n",(u32)USBC_Readl(sw_ehci->usb_vbase + SW_USB_PMU_IRQ_ENABLE));
	DMSG_INFO("DRAM: USB1(0x%x), USB2(0x%x)\n",
	          (u32)USBC_Readl(sw_ehci->sdram_vbase + SW_SDRAM_REG_HPCR_USB1),
	          (u32)USBC_Readl(sw_ehci->sdram_vbase + SW_SDRAM_REG_HPCR_USB2));

	DMSG_INFO("----------------------------------\n");
}

/*
*******************************************************************************
*                     sw_hcd_board_set_vbus
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
static void sw_hcd_board_set_vbus(struct sw_hci_hcd *sw_ehci, int is_on)
{
	sw_ehci->set_power(sw_ehci, is_on);

	return;
}

/*
*******************************************************************************
*                     open_ehci_clock
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
static int open_ehci_clock(struct sw_hci_hcd *sw_ehci)
{
	return sw_ehci->open_clock(sw_ehci, 0);
}

/*
*******************************************************************************
*                     close_ehci_clock
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
static int close_ehci_clock(struct sw_hci_hcd *sw_ehci)
{
	return sw_ehci->close_clock(sw_ehci, 0);
}

/*
*******************************************************************************
*                     sw_ehci_port_configure
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
static void sw_ehci_port_configure(struct sw_hci_hcd *sw_ehci, u32 enable)
{
	sw_ehci->port_configure(sw_ehci, enable);

	return;
}

/*
*******************************************************************************
*                     sw_get_io_resource
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
static int sw_get_io_resource(struct platform_device *pdev, struct sw_hci_hcd *sw_ehci)
{
	return 0;
}

/*
*******************************************************************************
*                     sw_release_io_resource
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
static int sw_release_io_resource(struct platform_device *pdev, struct sw_hci_hcd *sw_ehci)
{
	return 0;
}

/*
*******************************************************************************
*                     sw_start_ehci
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
static void sw_start_ehci(struct sw_hci_hcd *sw_ehci)
{
  	open_ehci_clock(sw_ehci); 
	sw_ehci->usb_passby(sw_ehci, 1);	
	sw_ehci_port_configure(sw_ehci, 1);
	sw_hcd_board_set_vbus(sw_ehci, 1);

	return;
}

/*
*******************************************************************************
*                     sw_stop_ehci
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
static void sw_stop_ehci(struct sw_hci_hcd *sw_ehci)
{
	sw_hcd_board_set_vbus(sw_ehci, 0);
	sw_ehci_port_configure(sw_ehci, 0);
	sw_ehci->usb_passby(sw_ehci, 0);
	close_ehci_clock(sw_ehci);

	return;
}

/*
*******************************************************************************
*                     sw_ehci_setup
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
static int sw_ehci_setup(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int ret = ehci_init(hcd);

	ehci->need_io_watchdog = 0;

	return ret;
}

static const struct hc_driver sw_ehci_hc_driver = {
	.description			= hcd_name,
	.product_desc			= "SW USB2.0 'Enhanced' Host Controller (EHCI) Driver",
	.hcd_priv_size			= sizeof(struct ehci_hcd),

	 /*
	 * generic hardware linkage
	 */
	 .irq					=  ehci_irq,
	 .flags					=  HCD_MEMORY | HCD_USB2,

	/*
	 * basic lifecycle operations
	 *
	 * FIXME -- ehci_init() doesn't do enough here.
	 * See ehci-ppc-soc for a complete implementation.
	 */
	.reset					= sw_ehci_setup,
	.start					= ehci_run,
	.stop					= ehci_stop,
	.shutdown				= ehci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue			= ehci_urb_enqueue,
	.urb_dequeue			= ehci_urb_dequeue,
	.endpoint_disable		= ehci_endpoint_disable,
	.endpoint_reset			= ehci_endpoint_reset,

	/*
	 * scheduling support
	 */
	.get_frame_number		= ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data		= ehci_hub_status_data,
	.hub_control			= ehci_hub_control,
	.bus_suspend			= ehci_bus_suspend,
	.bus_resume				= ehci_bus_resume,
	.relinquish_port		= ehci_relinquish_port,
	.port_handed_over		= ehci_port_handed_over,

	.clear_tt_buffer_complete	= ehci_clear_tt_buffer_complete,
};

/*
*******************************************************************************
*                     sw_ehci_hcd_probe
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
static int sw_ehci_hcd_probe(struct platform_device *pdev)
{
	struct usb_hcd 	*hcd 	= NULL;
	struct ehci_hcd *ehci	= NULL;
	struct sw_hci_hcd *sw_ehci = NULL;
	int ret = 0;

	if(pdev == NULL){
		DMSG_PANIC("ERR: Argment is invaild\n");
		return -1;
	}

	/* if usb is disabled, can not probe */
	if (usb_disabled()) {
		DMSG_PANIC("ERR: usb hcd is disabled\n");
		return -ENODEV;
	}

	sw_ehci = pdev->dev.platform_data;
	if(!sw_ehci){
		DMSG_PANIC("ERR: sw_ehci is null\n");
		ret = -ENOMEM;
		goto ERR1;
	}

	sw_ehci->pdev = pdev;
	g_sw_ehci[sw_ehci->usbc_no] = sw_ehci;

	DMSG_INFO("[%s%d]: probe, pdev->name: %s, pdev->id: %d, sw_ehci: 0x%p\n",
		      ehci_name, sw_ehci->usbc_no, pdev->name, pdev->id, sw_ehci);

	/* get io resource */
	sw_get_io_resource(pdev, sw_ehci);
	sw_ehci->ehci_base 			= sw_ehci->usb_vbase + SW_USB_EHCI_BASE_OFFSET;
	sw_ehci->ehci_reg_length 	= SW_USB_EHCI_LEN;

	/* creat a usb_hcd for the ehci controller */
	hcd = usb_create_hcd(&sw_ehci_hc_driver, &pdev->dev, ehci_name);
	if (!hcd){
		DMSG_PANIC("ERR: usb_create_hcd failed\n");
		ret = -ENOMEM;
		goto ERR2;
	}

  	hcd->rsrc_start = (u32)sw_ehci->ehci_base;
	hcd->rsrc_len 	= sw_ehci->ehci_reg_length;
	hcd->regs 		= sw_ehci->ehci_base;
	sw_ehci->hcd    = hcd;
    
	/* echi start to work */
	sw_start_ehci(sw_ehci);
    
	ehci = hcd_to_ehci(hcd);
	ehci->caps = hcd->regs;
	ehci->regs = hcd->regs + HC_LENGTH(ehci, readl(&ehci->caps->hc_capbase));

	/* cache this readonly data, minimize chip reads */
	ehci->hcs_params = readl(&ehci->caps->hcs_params);

	ret = usb_add_hcd(hcd, sw_ehci->irq_no, IRQF_DISABLED | IRQF_SHARED);
	if (ret != 0) {
		DMSG_PANIC("ERR: usb_add_hcd failed\n");
		ret = -ENOMEM;
		goto ERR3;
	}

	platform_set_drvdata(pdev, hcd);

	device_create_file(&pdev->dev, &dev_attr_ed_test);

#ifdef  SW_USB_EHCI_DEBUG
	DMSG_INFO("[%s]: probe, clock: 0x60(0x%x), 0xcc(0x%x); usb: 0x800(0x%x), dram:(0x%x, 0x%x)\n",
		      sw_ehci->hci_name,
		      (u32)USBC_Readl(sw_ehci->clock_vbase + 0x60),
		      (u32)USBC_Readl(sw_ehci->clock_vbase + 0xcc),
		      (u32)USBC_Readl(sw_ehci->usb_vbase + 0x800),
		      (u32)USBC_Readl(sw_ehci->sdram_vbase + SW_SDRAM_REG_HPCR_USB1),
		      (u32)USBC_Readl(sw_ehci->sdram_vbase + SW_SDRAM_REG_HPCR_USB2));
#endif

    device_enable_async_suspend(&pdev->dev);
    sw_ehci->probe = 1;

    /* Disable ehci, when driver probe */
    if(sw_ehci->host_init_state == 0){
        if(ehci_first_probe[sw_ehci->usbc_no]){
            sw_usb_disable_ehci(sw_ehci->usbc_no);
            ehci_first_probe[sw_ehci->usbc_no]--;
        }
    }

	return 0;

ERR3:
    usb_put_hcd(hcd);

ERR2:
	sw_ehci->hcd = NULL;
	g_sw_ehci[sw_ehci->usbc_no] = NULL;

ERR1:

	return ret;
}

/*
*******************************************************************************
*                     sw_ehci_hcd_remove
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
static int sw_ehci_hcd_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = NULL;
	struct sw_hci_hcd *sw_ehci = NULL;

	if(pdev == NULL){
		DMSG_PANIC("ERR: Argment is invalid\n");
		return -1;
	}

	hcd = platform_get_drvdata(pdev);
	if(hcd == NULL){
		DMSG_PANIC("ERR: hcd is null\n");
		return -1;
	}

	sw_ehci = pdev->dev.platform_data;
	if(sw_ehci == NULL){
		DMSG_PANIC("ERR: sw_ehci is null\n");
		return -1;
	}

	DMSG_INFO("[%s%d]: remove, pdev->name: %s, pdev->id: %d, sw_ehci: 0x%p\n",
		      ehci_name, sw_ehci->usbc_no, pdev->name, pdev->id, sw_ehci);

	device_remove_file(&pdev->dev, &dev_attr_ed_test);
	usb_remove_hcd(hcd);

	sw_release_io_resource(pdev, sw_ehci);

	usb_put_hcd(hcd);

	sw_stop_ehci(sw_ehci);
    sw_ehci->probe = 0;

	sw_ehci->hcd = NULL;

    if(sw_ehci->host_init_state){
    	g_sw_ehci[sw_ehci->usbc_no] = NULL;
    }

	platform_set_drvdata(pdev, NULL);

	return 0;
}

/*
*******************************************************************************
*                     sw_ehci_hcd_shutdown
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
void sw_ehci_hcd_shutdown(struct platform_device* pdev)
{
	struct sw_hci_hcd *sw_ehci = NULL;

	if(pdev == NULL){
		DMSG_PANIC("ERR: Argment is invalid\n");
		return;
	}

	sw_ehci = pdev->dev.platform_data;
	if(sw_ehci == NULL){
		DMSG_PANIC("ERR: sw_ehci is null\n");
		return;
	}

	if(sw_ehci->probe == 0){
		DMSG_PANIC("ERR: sw_ehci is disable, need not shutdown\n");
		return;
	}

 	DMSG_INFO("[%s]: ehci shutdown start\n", sw_ehci->hci_name);
    sw_ehci->probe = 0;
    usb_hcd_platform_shutdown(pdev);

    sw_stop_ehci(sw_ehci);

 	DMSG_INFO("[%s]: ehci shutdown end\n", sw_ehci->hci_name);

    return ;
}

#ifdef CONFIG_PM

/*
*******************************************************************************
*                     sw_ehci_hcd_suspend
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
static int sw_ehci_hcd_suspend(struct device *dev)
{
	struct sw_hci_hcd *sw_ehci = NULL;
	struct usb_hcd *hcd = NULL;
	struct ehci_hcd *ehci = NULL;
	unsigned long flags = 0;

	if(dev == NULL){
		DMSG_PANIC("ERR: Argment is invalid\n");
		return 0;
	}

	hcd = dev_get_drvdata(dev);
	if(hcd == NULL){
		DMSG_PANIC("ERR: hcd is null\n");
		return 0;
	}

	sw_ehci = dev->platform_data;
	if(sw_ehci == NULL){
		DMSG_PANIC("ERR: sw_ehci is null\n");
		return 0;
	}

	if(sw_ehci->probe == 0){
		DMSG_PANIC("ERR: sw_ehci is disable, can not suspend\n");
		return 0;
	}

	ehci = hcd_to_ehci(hcd);
	if(ehci == NULL){
		DMSG_PANIC("ERR: ehci is null\n");
		return 0;
	}
	
    if(sw_usb_suspend_disabled(sw_ehci->usbc_no))
        return 0;
        
 	DMSG_INFO("[%s]: sw_ehci_hcd_suspend\n", sw_ehci->hci_name);

	spin_lock_irqsave(&ehci->lock, flags);
	ehci_prepare_ports_for_controller_suspend(ehci, device_may_wakeup(dev));
	ehci_writel(ehci, 0, &ehci->regs->intr_enable);
	(void)ehci_readl(ehci, &ehci->regs->intr_enable);

	clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

	spin_unlock_irqrestore(&ehci->lock, flags);

	//sw_stop_ehci(sw_ehci);
    sw_ehci_port_configure(sw_ehci, 0);
	sw_ehci->usb_passby(sw_ehci, 0);
	close_ehci_clock(sw_ehci);
	
	return 0;
}

/*
*******************************************************************************
*                     sw_ehci_hcd_resume
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
static int sw_ehci_hcd_resume(struct device *dev)
{
	struct sw_hci_hcd *sw_ehci = NULL;
	struct usb_hcd *hcd = NULL;
	struct ehci_hcd *ehci = NULL;

	if(dev == NULL){
		DMSG_PANIC("ERR: Argment is invalid\n");
		return 0;
	}

	hcd = dev_get_drvdata(dev);
	if(hcd == NULL){
		DMSG_PANIC("ERR: hcd is null\n");
		return 0;
	}

	sw_ehci = dev->platform_data;
	if(sw_ehci == NULL){
		DMSG_PANIC("ERR: sw_ehci is null\n");
		return 0;
	}

	if(sw_ehci->probe == 0){
		DMSG_PANIC("ERR: sw_ehci is disable, can not resume\n");
		return 0;
	}

	ehci = hcd_to_ehci(hcd);
	if(ehci == NULL){
		DMSG_PANIC("ERR: ehci is null\n");
		return 0;
	}

    if(sw_usb_suspend_disabled(sw_ehci->usbc_no))
        return 0;
        
 	DMSG_INFO("[%s]: sw_ehci_hcd_resume\n", sw_ehci->hci_name);

	//sw_start_ehci(sw_ehci);
    open_ehci_clock(sw_ehci); 
	sw_ehci->usb_passby(sw_ehci, 1);	
	sw_ehci_port_configure(sw_ehci, 1);
	
	/* Mark hardware accessible again as we are out of D3 state by now */
	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

	if (ehci_readl(ehci, &ehci->regs->configured_flag) == FLAG_CF) {
		int	mask = INTR_MASK;

		ehci_prepare_ports_for_controller_resume(ehci);

		if (!hcd->self.root_hub->do_remote_wakeup){
			mask &= ~STS_PCD;
		}

		ehci_writel(ehci, mask, &ehci->regs->intr_enable);
		ehci_readl(ehci, &ehci->regs->intr_enable);

		return 0;
	}

 	DMSG_INFO("[%s]: lost power, restarting\n", sw_ehci->hci_name);

	usb_root_hub_lost_power(hcd->self.root_hub);

	/* Else reset, to cope with power loss or flush-to-storage
	 * style "resume" having let BIOS kick in during reboot.
	 */
	(void) ehci_halt(ehci);
	(void) ehci_reset(ehci);

	/* emptying the schedule aborts any urbs */
	spin_lock_irq(&ehci->lock);
	if (ehci->reclaim)
		end_unlink_async(ehci);
	ehci_work(ehci);
	spin_unlock_irq(&ehci->lock);

    ehci->command &= ~(CMD_ASE | CMD_IAAD | CMD_PSE);
    ehci->command |= CMD_RUN;
	ehci_writel(ehci, ehci->command, &ehci->regs->command);
	ehci_writel(ehci, FLAG_CF, &ehci->regs->configured_flag);
	ehci_readl(ehci, &ehci->regs->command);	/* unblock posted writes */

	/* here we "know" root ports should always stay powered */
	ehci_port_power(ehci, 1);

	hcd->state = HC_STATE_SUSPENDED;

	return 0;

}

static const struct dev_pm_ops  aw_ehci_pmops = {
	.suspend	= sw_ehci_hcd_suspend,
	.resume		= sw_ehci_hcd_resume,
};

#define SW_EHCI_PMOPS 	&aw_ehci_pmops

#else

#define SW_EHCI_PMOPS 	NULL

#endif

static struct platform_driver sw_ehci_hcd_driver ={
  .probe  	= sw_ehci_hcd_probe,
  .remove	= sw_ehci_hcd_remove,
  .shutdown = sw_ehci_hcd_shutdown,
  .driver = {
		.name	= ehci_name,
		.owner	= THIS_MODULE,
		.pm		= SW_EHCI_PMOPS,
  	}
};

/*
*******************************************************************************
*                     sw_usb_disable_ehci
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
int sw_usb_disable_ehci(__u32 usbc_no)
{
	struct sw_hci_hcd *sw_ehci = NULL;

	if(usbc_no != 1 && usbc_no != 2){
		DMSG_PANIC("ERR:Argmen invalid. usbc_no(%d)\n", usbc_no);
		return -1;
	}

	sw_ehci = g_sw_ehci[usbc_no];
	if(sw_ehci == NULL){
		DMSG_PANIC("ERR: sw_ehci is null\n");
		return -1;
	}

	if(sw_ehci->host_init_state){
		DMSG_PANIC("ERR: not support sw_usb_disable_ehci\n");
		return -1;
	}

	if(sw_ehci->probe == 0){
		DMSG_PANIC("ERR: sw_ehci is disable, can not disable again\n");
		return -1;
	}

	sw_ehci->probe = 0;

	DMSG_INFO("[%s]: sw_usb_disable_ehci\n", sw_ehci->hci_name);

    sw_ehci_hcd_remove(sw_ehci->pdev);

	return 0;
}
EXPORT_SYMBOL(sw_usb_disable_ehci);

/*
*******************************************************************************
*                     sw_usb_enable_ehci
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
int sw_usb_enable_ehci(__u32 usbc_no)
{
	struct sw_hci_hcd *sw_ehci = NULL;

	if(usbc_no != 1 && usbc_no != 2){
		DMSG_PANIC("ERR:Argmen invalid. usbc_no(%d)\n", usbc_no);
		return -1;
	}

	sw_ehci = g_sw_ehci[usbc_no];
	if(sw_ehci == NULL){
		DMSG_PANIC("ERR: sw_ehci is null\n");
		return -1;
	}

	if(sw_ehci->host_init_state){
		DMSG_PANIC("ERR: not support sw_usb_enable_ehci\n");
		return -1;
	}

	if(sw_ehci->probe == 1){
		DMSG_PANIC("ERR: sw_ehci is already enable, can not enable again\n");
		return -1;
	}

	//sw_ehci->probe = 1;

	DMSG_INFO("[%s]: sw_usb_enable_ehci\n", sw_ehci->hci_name);

    sw_ehci_hcd_probe(sw_ehci->pdev);

	return 0;
}
EXPORT_SYMBOL(sw_usb_enable_ehci);


