/*
 * B53 register access through MII registers
 *
 * Copyright (C) 2011-2013 Jonas Gorski <jogo@openwrt.org>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <linux/kernel.h>
#include <linux/phy.h>
#include <linux/module.h>

#include "b53_regs.h"
#include "b53_priv.h"

#define B53_PSEUDO_PHY	0x1e /* Register Access Pseudo PHY */
#define flow_log 0
/* MII registers */
#define REG_MII_PAGE    0x10    /* MII Page register */
#define REG_MII_ADDR    0x11    /* MII Address register */
#define REG_MII_DATA0   0x18    /* MII Data register 0 */
#define REG_MII_DATA1   0x19    /* MII Data register 1 */
#define REG_MII_DATA2   0x1a    /* MII Data register 2 */
#define REG_MII_DATA3   0x1b    /* MII Data register 3 */

#define REG_MII_PAGE_ENABLE     BIT(0)
#define REG_MII_ADDR_WRITE      BIT(0)
#define REG_MII_ADDR_READ       BIT(1)

static int b53_mdio_op(struct b53_device *dev, u8 page, u8 reg, u16 op)
{
	int i;
	u16 v;
	int ret;
	struct mii_bus *bus = dev->priv;
	if (dev->current_page != page) {
		/* set page number */
		v = (page << 8) | REG_MII_PAGE_ENABLE;
		ret = mdiobus_write(bus, B53_PSEUDO_PHY, REG_MII_PAGE, v);
		if (ret)
			return ret;
		dev->current_page = page;
	}

	/* set register address */
	v = (reg << 8) | op;
#if flow_log
printk("[arlen] b53_flow, %s\n", __func__);
#endif
	ret = mdiobus_write(bus, B53_PSEUDO_PHY, REG_MII_ADDR, v);
	if (ret)
		return ret;

	/* check if operation completed */
	for (i = 0; i < 5; ++i) {
		v = mdiobus_read(bus, B53_PSEUDO_PHY, REG_MII_ADDR);
		if (!(v & (REG_MII_ADDR_WRITE | REG_MII_ADDR_READ)))
			break;
		usleep_range(10, 100);
	}

	if (WARN_ON(i == 5))
		return -EIO;

	return 0;
}

static int b53_mdio_read8(struct b53_device *dev, u8 page, u8 reg, u8 *val)
{
	struct mii_bus *bus = dev->priv;
	int ret;
#if flow_log
printk("[arlen] b53_flow, %s\n", __func__);
#endif
	ret = b53_mdio_op(dev, page, reg, REG_MII_ADDR_READ);
	if (ret)
		return ret;

	*val = mdiobus_read(bus, B53_PSEUDO_PHY, REG_MII_DATA0) & 0xff;

	return 0;
}

static int b53_mdio_read16(struct b53_device *dev, u8 page, u8 reg, u16 *val)
{
	struct mii_bus *bus = dev->priv;
	int ret;
#if flow_log
printk("[arlen] b53_flow, %s\n", __func__);
#endif
	ret = b53_mdio_op(dev, page, reg, REG_MII_ADDR_READ);
	if (ret)
		return ret;

	*val = mdiobus_read(bus, B53_PSEUDO_PHY, REG_MII_DATA0);

	return 0;
}

static int b53_mdio_read32(struct b53_device *dev, u8 page, u8 reg, u32 *val)
{
	struct mii_bus *bus = dev->priv;
	int ret;
#if flow_log
printk("[arlen] b53_flow, %s\n", __func__);
#endif
	ret = b53_mdio_op(dev, page, reg, REG_MII_ADDR_READ);
	if (ret)
		return ret;

	*val = mdiobus_read(bus, B53_PSEUDO_PHY, REG_MII_DATA0);
	*val |= mdiobus_read(bus, B53_PSEUDO_PHY, REG_MII_DATA1) << 16;

	return 0;
}

static int b53_mdio_read48(struct b53_device *dev, u8 page, u8 reg, u64 *val)
{
	struct mii_bus *bus = dev->priv;
	u64 temp = 0;
	int i;
	int ret;
#if flow_log
printk("[arlen] b53_flow, %s\n", __func__);
#endif
	ret = b53_mdio_op(dev, page, reg, REG_MII_ADDR_READ);
	if (ret)
		return ret;

	for (i = 2; i >= 0; i--) {
		temp <<= 16;
		temp |= mdiobus_read(bus, B53_PSEUDO_PHY, REG_MII_DATA0 + i);
	}

	*val = temp;

	return 0;
}

static int b53_mdio_read64(struct b53_device *dev, u8 page, u8 reg, u64 *val)
{
	struct mii_bus *bus = dev->priv;
	u64 temp = 0;
	int i;
	int ret;
#if flow_log
printk("[arlen] b53_flow, %s\n", __func__);
#endif
	ret = b53_mdio_op(dev, page, reg, REG_MII_ADDR_READ);
	if (ret)
		return ret;

	for (i = 3; i >= 0; i--) {
		temp <<= 16;
		temp |= mdiobus_read(bus, B53_PSEUDO_PHY, REG_MII_DATA0 + i);
	}

	*val = temp;

	return 0;
}

static int b53_mdio_write8(struct b53_device *dev, u8 page, u8 reg, u8 value)
{
	struct mii_bus *bus = dev->priv;
	int ret;
#if flow_log
printk("[arlen] b53_flow, %s, page = %x, reg = %x, value=%x \n", __func__, page, reg, value);
#endif
	ret = mdiobus_write(bus, B53_PSEUDO_PHY, REG_MII_DATA0, value);
	if (ret)
		return ret;

	return b53_mdio_op(dev, page, reg, REG_MII_ADDR_WRITE);
}

static int b53_mdio_write16(struct b53_device *dev, u8 page, u8 reg,
			     u16 value)
{
	struct mii_bus *bus = dev->priv;
	int ret;
#if flow_log
printk("[arlen] b53_flow, %s, page = %x, reg = %x, value=%x \n", __func__, page, reg, value);
#endif
	ret = mdiobus_write(bus, B53_PSEUDO_PHY, REG_MII_DATA0, value);
	if (ret)
		return ret;

	return b53_mdio_op(dev, page, reg, REG_MII_ADDR_WRITE);
}

static int b53_mdio_write32(struct b53_device *dev, u8 page, u8 reg,
				    u32 value)
{
	struct mii_bus *bus = dev->priv;
	unsigned int i;
	u32 temp = value;
#if flow_log
printk("[arlen] b53_flow, %s\n", __func__);
#endif
	for (i = 0; i < 2; i++) {
		int ret = mdiobus_write(bus, B53_PSEUDO_PHY, REG_MII_DATA0 + i,
				    temp & 0xffff);
		if (ret)
			return ret;
		temp >>= 16;
	}

	return b53_mdio_op(dev, page, reg, REG_MII_ADDR_WRITE);

}

static int b53_mdio_write48(struct b53_device *dev, u8 page, u8 reg,
				    u64 value)
{
	struct mii_bus *bus = dev->priv;
	unsigned i;
	u64 temp = value;
#if flow_log
printk("[arlen] b53_flow, %s\n", __func__);
#endif
	for (i = 0; i < 3; i++) {
		int ret = mdiobus_write(bus, B53_PSEUDO_PHY, REG_MII_DATA0 + i,
				    temp & 0xffff);
		if (ret)
			return ret;
		temp >>= 16;
	}

	return b53_mdio_op(dev, page, reg, REG_MII_ADDR_WRITE);

}

static int b53_mdio_write64(struct b53_device *dev, u8 page, u8 reg,
			     u64 value)
{
	struct mii_bus *bus = dev->priv;
	unsigned i;
	u64 temp = value;
#if flow_log
printk("[arlen] b53_flow, %s\n", __func__);
#endif
	for (i = 0; i < 4; i++) {
		int ret = mdiobus_write(bus, B53_PSEUDO_PHY, REG_MII_DATA0 + i,
				    temp & 0xffff);
		if (ret)
			return ret;
		temp >>= 16;
	}

	return b53_mdio_op(dev, page, reg, REG_MII_ADDR_WRITE);
}

static struct b53_io_ops b53_mdio_ops = {
	.read8 = b53_mdio_read8,
	.read16 = b53_mdio_read16,
	.read32 = b53_mdio_read32,
	.read48 = b53_mdio_read48,
	.read64 = b53_mdio_read64,
	.write8 = b53_mdio_write8,
	.write16 = b53_mdio_write16,
	.write32 = b53_mdio_write32,
	.write48 = b53_mdio_write48,
	.write64 = b53_mdio_write64,
};

static int b53_phy_probe(struct phy_device *phydev)
{
	struct b53_device dev;
	int ret;
#if flow_log
printk("[arlen] b53_flow, %s\n", __func__);
#endif
	/* allow the generic phy driver to take over */
	if (phydev->addr != B53_PSEUDO_PHY && phydev->addr != 0)
		return -ENODEV;

	dev.current_page = 0xff;
	dev.priv = phydev->bus;
	dev.ops = &b53_mdio_ops;
	dev.pdata = NULL;
	mutex_init(&dev.reg_mutex);

	ret = b53_switch_detect(&dev);
	if (ret)
		return ret;

	if (is5325(&dev) || is5365(&dev))
		phydev->supported = SUPPORTED_100baseT_Full;
	else
		phydev->supported = SUPPORTED_1000baseT_Full;

	phydev->advertising = phydev->supported;

	return 0;
}

static int b53_phy_config_init(struct phy_device *phydev)
{
	struct b53_device *dev;
	int ret;
#if flow_log
printk("[arlen] b53_flow, %s\n", __func__);
#endif
	dev = b53_switch_alloc(&phydev->dev, &b53_mdio_ops, phydev->bus);
	if (!dev)
		return -ENOMEM;

	/* we don't use page 0xff, so force a page set */
	dev->current_page = 0xff;
	/* force the ethX as alias */
	dev->sw_dev.alias = phydev->attached_dev->name;

	ret = b53_switch_register(dev);
	if (ret) {
		dev_err(dev->dev, "failed to register switch: %i\n", ret);
		return ret;
	}

	phydev->priv = dev;

	return 0;
}

static void b53_phy_remove(struct phy_device *phydev)
{
	struct b53_device *priv = phydev->priv;
#if flow_log
printk("[arlen] b53_flow, %s\n", __func__);
#endif
	if (!priv)
		return;

	b53_switch_remove(priv);

	phydev->priv = NULL;
}

static int b53_phy_config_aneg(struct phy_device *phydev)
{
	return 0;
}

static int b53_phy_update_link(struct phy_device *phydev)
{
	struct b53_device *priv = phydev->priv;
	u16 lnk;

	b53_read16(priv, B53_STAT_PAGE, B53_LINK_STAT, &lnk);
	lnk = (lnk >> 3) & 1;
	phydev->link = lnk;

	printk("%s: phydev->link = %d\n", __func__, phydev->link);

	return 0;
}

static int b53_phy_read_status(struct phy_device *phydev)
{
	struct b53_device *priv = phydev->priv;
	u16 lnk;
	

	if (is5325(priv) || is5365(priv))
		phydev->speed = 100;
	else
#ifdef FIX_100Mbps
		phydev->speed = 100;
#else
		phydev->speed = 1000;
#endif
	phydev->duplex = DUPLEX_FULL;

#if 0
	phydev->link = 1;
	phydev->state = PHY_RUNNING;
	netif_carrier_on(phydev->attached_dev);
	phydev->adjust_link(phydev->attached_dev);
#else
	b53_read16(priv, B53_STAT_PAGE, B53_LINK_STAT, &lnk);
	lnk = (lnk >> 3) & 1;    //port3
	phydev->link = lnk;
#endif

	printk("%s: phydev->link = %d\n", __func__, phydev->link);

	return 0;
}

/* BCM5325, BCM539x */
static struct phy_driver b53_phy_driver_id1 = {
	.phy_id		= 0x0143bc00,
	.name		= "Broadcom B53 (1)",
	.phy_id_mask	= 0x1ffffc00,
	.features	= 0,
	.probe		= b53_phy_probe,
	.remove		= b53_phy_remove,
	.config_aneg	= b53_phy_config_aneg,
	.config_init	= b53_phy_config_init,
	.read_status	= b53_phy_read_status,
	.update_link	= b53_phy_update_link,
	.driver = {
		.owner = THIS_MODULE,
	},
};

/* BCM53125 */
static struct phy_driver b53_phy_driver_id2 = {
	.phy_id		= 0x03625c00,
	.name		= "Broadcom B53 (2)",
	.phy_id_mask	= 0x1ffffc00,
	.features	= 0,
	.probe		= b53_phy_probe,
	.remove		= b53_phy_remove,
	.config_aneg	= b53_phy_config_aneg,
	.config_init	= b53_phy_config_init,
	.read_status	= b53_phy_read_status,
	.update_link	= b53_phy_update_link,
	.driver = {
		.owner = THIS_MODULE,
	},
};

/* BCM5365 */
static struct phy_driver b53_phy_driver_id3 = {
	.phy_id		= 0x00406000,
	.name		= "Broadcom B53 (3)",
	.phy_id_mask	= 0x1ffffc00,
	.features	= 0,
	.probe		= b53_phy_probe,
	.remove		= b53_phy_remove,
	.config_aneg	= b53_phy_config_aneg,
	.config_init	= b53_phy_config_init,
	.read_status	= b53_phy_read_status,
	.update_link	= b53_phy_update_link,
	.driver = {
		.owner = THIS_MODULE,
	},
};

#if 1
int b53_phy_driver_register(void)
{
	int ret;
#if flow_log
printk("[arlen] b53_flow, %s\n", __func__);
#endif

	ret = phy_driver_register(&b53_phy_driver_id1);
printk(KERN_ERR "@WadeKu@ b53_phy_driver_id1 ret = %d\n", ret);
	if (ret)
		return ret;

	ret = phy_driver_register(&b53_phy_driver_id2);
printk(KERN_ERR "@WadeKu@ b53_phy_driver_id2 ret = %d\n", ret);
	if (ret)
		goto err1;

	ret = phy_driver_register(&b53_phy_driver_id3);
printk(KERN_ERR "@WadeKu@ b53_phy_driver_id3 ret = %d\n", ret);
	if (!ret)
		return 0;

	phy_driver_unregister(&b53_phy_driver_id2);
err1:
printk(KERN_ERR "@WadeKu@ b53_phy_driver_id3 err1\n");
	phy_driver_unregister(&b53_phy_driver_id1);
	return ret;
}

void b53_phy_driver_unregister(void)
{
#if flow_log
printk("[arlen] b53_flow, %s\n", __func__);
#endif
	phy_driver_unregister(&b53_phy_driver_id3);
	phy_driver_unregister(&b53_phy_driver_id2);
	phy_driver_unregister(&b53_phy_driver_id1);
}
#endif

/*
int __init b53_phy_driver_register(void)
{
	int ret;

	ret = phy_driver_register(&b53_phy_driver_id1);
	if (ret)
		return ret;

	ret = phy_driver_register(&b53_phy_driver_id2);
	if (ret)
		goto err1;

	ret = phy_driver_register(&b53_phy_driver_id3);
	if (!ret)
		return 0;

	phy_driver_unregister(&b53_phy_driver_id2);

	printk("^^^^^^^^^^^^%s: b53 phy driver register end\n", __func__);
err1:
	phy_driver_unregister(&b53_phy_driver_id1);
	return ret;
}

void __exit b53_phy_driver_unregister(void)
{
	phy_driver_unregister(&b53_phy_driver_id3);
	phy_driver_unregister(&b53_phy_driver_id2);
	phy_driver_unregister(&b53_phy_driver_id1);
}

module_init(b53_phy_driver_register);
module_exit(b53_phy_driver_unregister);
*/
MODULE_DESCRIPTION("B53 MDIO access driver");
MODULE_LICENSE("Dual BSD/GPL");
