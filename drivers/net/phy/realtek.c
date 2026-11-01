/*
 * drivers/net/phy/realtek.c
 *
 * Driver for Realtek PHYs
 *
 * Author: Johnson Leung <r58129@freescale.com>
 *
 * Copyright (c) 2004 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#include <linux/phy.h>

#define RTL821x_PHYCR		0x10
#define RTL821x_PHYCR_CLK125_DIS	0x0010
#define RTL821x_PHYSR		0x11
#define RTL821x_PHYSR_DUPLEX	0x2000
#define RTL821x_PHYSR_SPEED	0xc000
#define RTL821x_INER		0x12
#define RTL821x_INSR		0x13
#define RTL821x_INSR_FALSE_CARRIER	0x0100

#define RTL8211B_INER_INIT	0x6400
/* bits 14:13 are reserved on the RTL8211E, bit 10 is link status change */
#define RTL8211E_INER_LINK_STATUS	0x0400

MODULE_DESCRIPTION("Realtek PHY driver");
MODULE_AUTHOR("Johnson Leung");
MODULE_LICENSE("GPL");

static int rtl821x_ack_interrupt(struct phy_device *phydev)
{
	int err;

	err = phy_read(phydev, RTL821x_INSR);

	return (err < 0) ? err : 0;
}

static int rtl8211b_config_intr(struct phy_device *phydev)
{
	int err;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED)
		err = phy_write(phydev, RTL821x_INER,
				RTL8211B_INER_INIT);
	else
		err = phy_write(phydev, RTL821x_INER, 0);

	return err;
}

/*
 * RTL8211E as used on the IDL4K (sources from Inverto, firmware 1.17):
 *
 * - The PHY's CLK125 output is needed at 1000 Mbit/s only (with it disabled
 *   the link stays up but no traffic passes); it is switched off at the
 *   other speeds and while the link is down (genphy then reports 10).
 * - A false carrier event restarts auto-negotiation.
 *
 * INSR is read-clear, so this only works in polling mode: the driver does
 * not set PHY_HAS_INTERRUPT (in interrupt mode the ack would clear the
 * false carrier bit first, and reading INSR here could swallow a link
 * change interrupt).
 */
static int rtl8211e_read_status(struct phy_device *phydev)
{
	int err, val, new;

	err = genphy_read_status(phydev);
	if (err)
		return err;

	val = phy_read(phydev, RTL821x_INSR);
	if (val > 0 && (val & RTL821x_INSR_FALSE_CARRIER)) {
		if (printk_ratelimit())
			dev_info(&phydev->dev,
				 "false carrier, restarting autonegotiation\n");
		genphy_restart_aneg(phydev);
	}

	val = phy_read(phydev, RTL821x_PHYCR);
	if (val < 0)
		return val;

	if (phydev->speed == SPEED_1000)
		new = val & ~RTL821x_PHYCR_CLK125_DIS;
	else
		new = val | RTL821x_PHYCR_CLK125_DIS;

	if (new != val)
		err = phy_write(phydev, RTL821x_PHYCR, new);

	return err;
}

static int rtl8211e_config_intr(struct phy_device *phydev)
{
	int err;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED)
		err = phy_write(phydev, RTL821x_INER,
				RTL8211E_INER_LINK_STATUS);
	else
		err = phy_write(phydev, RTL821x_INER, 0);

	return err;
}

/* RTL8211B */
static struct phy_driver rtl8211b_driver = {
	.phy_id		= 0x001cc912,
	.name		= "RTL8211B Gigabit Ethernet",
	.phy_id_mask	= 0x001fffff,
	.features	= PHY_GBIT_FEATURES,
	.flags		= PHY_HAS_INTERRUPT,
	.config_aneg	= &genphy_config_aneg,
	.read_status	= &genphy_read_status,
	.ack_interrupt	= &rtl821x_ack_interrupt,
	.config_intr	= &rtl8211b_config_intr,
	.driver		= { .owner = THIS_MODULE,},
};

/* RTL8211E: polled, see rtl8211e_read_status() */
static struct phy_driver rtl8211e_driver = {
	.phy_id		= 0x001cc915,
	.name		= "RTL8211E Gigabit Ethernet",
	.phy_id_mask	= 0x001fffff,
	.features	= PHY_GBIT_FEATURES,
	.config_aneg	= &genphy_config_aneg,
	.read_status	= &rtl8211e_read_status,
	.ack_interrupt	= &rtl821x_ack_interrupt,
	.config_intr	= &rtl8211e_config_intr,
	.driver		= { .owner = THIS_MODULE,},
};

static int __init realtek_init(void)
{
	int ret;

	ret = phy_driver_register(&rtl8211b_driver);
	if (ret < 0)
		return ret;

	ret = phy_driver_register(&rtl8211e_driver);
	if (ret < 0)
		phy_driver_unregister(&rtl8211b_driver);

	return ret;
}

static void __exit realtek_exit(void)
{
	phy_driver_unregister(&rtl8211e_driver);
	phy_driver_unregister(&rtl8211b_driver);
}

module_init(realtek_init);
module_exit(realtek_exit);
