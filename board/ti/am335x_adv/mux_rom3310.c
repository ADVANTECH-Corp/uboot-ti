/*
 * mux.c
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <common.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/hardware.h>
#include <asm/arch/mux.h>
#include <asm/io.h>
#include <i2c.h>
#include "board.h"

static struct module_pin_mux uart0_pin_mux[] = {
	{OFFSET(uart0_rxd), (MODE(0) | PULLUP_EN | RXACTIVE)},	/* UART0_RXD */
	{OFFSET(uart0_txd), (MODE(0) | PULLUDEN)},		/* UART0_TXD */
	{-1},
};

static struct module_pin_mux uart1_pin_mux[] = {
	{OFFSET(uart1_rxd), (MODE(0) | PULLUP_EN | RXACTIVE)},	/* UART1_RXD */
	{OFFSET(uart1_txd), (MODE(0) | PULLUDEN)},		/* UART1_TXD */
	{-1},
};

static struct module_pin_mux uart2_pin_mux[] = {
	{OFFSET(mii1_crs), (MODE(6) | PULLUP_EN | RXACTIVE)},	/* UART2_RXD */
	{OFFSET(mii1_rxerr), (MODE(6) | PULLUDEN)},		/* UART2_TXD */
	{-1},
};

static struct module_pin_mux uart3_pin_mux[] = {
	{OFFSET(spi0_cs1), (MODE(1) | PULLUP_EN | RXACTIVE)},	/* UART3_RXD */
	{OFFSET(ecap0_in_pwm0_out), (MODE(1) | PULLUDEN)},	/* UART3_TXD */
	{-1},
};

static struct module_pin_mux uart4_pin_mux[] = {
	{OFFSET(gpmc_wait0), (MODE(6) | PULLUP_EN | RXACTIVE)},	/* UART4_RXD */
	{OFFSET(gpmc_wpn), (MODE(6) | PULLUDEN)},		/* UART4_TXD */
	{-1},
};

static struct module_pin_mux uart5_pin_mux[] = {
	{OFFSET(mii1_col), (MODE(3) | PULLUP_EN | RXACTIVE)},	/* UART5_RXD */
	{OFFSET(rmii1_refclk), (MODE(3) | PULLUDEN)},		/* UART5_TXD */
	{-1},
};

static struct module_pin_mux mmc0_pin_mux[] = {
	{OFFSET(mmc0_dat3), (MODE(0) | RXACTIVE | PULLUP_EN)},	/* MMC0_DAT3 */
	{OFFSET(mmc0_dat2), (MODE(0) | RXACTIVE | PULLUP_EN)},	/* MMC0_DAT2 */
	{OFFSET(mmc0_dat1), (MODE(0) | RXACTIVE | PULLUP_EN)},	/* MMC0_DAT1 */
	{OFFSET(mmc0_dat0), (MODE(0) | RXACTIVE | PULLUP_EN)},	/* MMC0_DAT0 */
	{OFFSET(mmc0_clk), (MODE(0) | RXACTIVE | PULLUP_EN)},	/* MMC0_CLK */
	{OFFSET(mmc0_cmd), (MODE(0) | RXACTIVE | PULLUP_EN)},	/* MMC0_CMD */
	{OFFSET(mcasp0_aclkr), (MODE(7) | RXACTIVE)},		/* MMC0_WP */
	{OFFSET(mcasp0_aclkx), (MODE(7) | RXACTIVE | PULLUP_EN)},	/* MMC0_CD */
	{-1},
};

static struct module_pin_mux mmc1_pin_mux[] = {
	{OFFSET(gpmc_ad3), (MODE(1) | RXACTIVE | PULLUP_EN)},	/* MMC1_DAT3 */
	{OFFSET(gpmc_ad2), (MODE(1) | RXACTIVE | PULLUP_EN)},	/* MMC1_DAT2 */
	{OFFSET(gpmc_ad1), (MODE(1) | RXACTIVE | PULLUP_EN)},	/* MMC1_DAT1 */
	{OFFSET(gpmc_ad0), (MODE(1) | RXACTIVE | PULLUP_EN)},	/* MMC1_DAT0 */
	{OFFSET(gpmc_csn1), (MODE(2) | RXACTIVE | PULLUP_EN)},	/* MMC1_CLK */
	{OFFSET(gpmc_csn2), (MODE(2) | RXACTIVE | PULLUP_EN)},	/* MMC1_CMD */
	{-1},
};

static struct module_pin_mux i2c0_pin_mux[] = {
	{OFFSET(i2c0_sda), (MODE(0) | RXACTIVE |
			PULLUDEN | SLEWCTRL)}, /* I2C_DATA */
	{OFFSET(i2c0_scl), (MODE(0) | RXACTIVE |
			PULLUDEN | SLEWCTRL)}, /* I2C_SCLK */
	{-1},
};

static struct module_pin_mux spi0_pin_mux[] = {
	{OFFSET(spi0_sclk), (MODE(0) | RXACTIVE | PULLUDEN)},	/* SPI0_SCLK */
	{OFFSET(spi0_d0), (MODE(0) | RXACTIVE |
			PULLUDEN | PULLUP_EN)},			/* SPI0_D0 */
	{OFFSET(spi0_d1), (MODE(0) | RXACTIVE | PULLUDEN)},	/* SPI0_D1 */
	{OFFSET(spi0_cs0), (MODE(0) | RXACTIVE |
			PULLUDEN | PULLUP_EN)},			/* SPI0_CS0 */
	{-1},
};

static struct module_pin_mux vtten_pin_mux[] = {
	{OFFSET(gpmc_csn3), (MODE(7) | PULLUDEN)},	/* GPIO2_0 */
	{-1},
};

static struct module_pin_mux rgmii2_pin_mux[] = {
	{OFFSET(gpmc_a0), MODE(2)},			/* RGMII1_TCTL */
	{OFFSET(gpmc_a1), MODE(2) | RXACTIVE},	/* RGMII1_RCTL */
	{OFFSET(gpmc_a2), MODE(2)},			/* RGMII1_TD3 */
	{OFFSET(gpmc_a3), MODE(2)},			/* RGMII1_TD2 */
	{OFFSET(gpmc_a4), MODE(2)},			/* RGMII1_TD1 */
	{OFFSET(gpmc_a5), MODE(2)},			/* RGMII1_TD0 */
	{OFFSET(gpmc_a6), MODE(2)},			/* RGMII1_TCLK */
	{OFFSET(gpmc_a7), MODE(2) | RXACTIVE},	/* RGMII1_RCLK */
	{OFFSET(gpmc_a8), MODE(2) | RXACTIVE},	/* RGMII1_RD3 */
	{OFFSET(gpmc_a9), MODE(2) | RXACTIVE},	/* RGMII1_RD2 */
	{OFFSET(gpmc_a10), MODE(2) | RXACTIVE},	/* RGMII1_RD1 */
	{OFFSET(gpmc_a11), MODE(2) | RXACTIVE},	/* RGMII1_RD0 */
	{OFFSET(mdio_data), MODE(0) | RXACTIVE | PULLUP_EN},/* MDIO_DATA */
	{OFFSET(mdio_clk), MODE(0) | PULLUP_EN},	/* MDIO_CLK */
	{-1},
};

void enable_uart0_pin_mux(void)
{
	configure_module_pin_mux(uart0_pin_mux);
}

void enable_uart1_pin_mux(void)
{
	configure_module_pin_mux(uart1_pin_mux);
}

void enable_uart2_pin_mux(void)
{
	configure_module_pin_mux(uart2_pin_mux);
}

void enable_uart3_pin_mux(void)
{
	configure_module_pin_mux(uart3_pin_mux);
}

void enable_uart4_pin_mux(void)
{
	configure_module_pin_mux(uart4_pin_mux);
}

void enable_uart5_pin_mux(void)
{
	configure_module_pin_mux(uart5_pin_mux);
}

void enable_i2c0_pin_mux(void)
{
	configure_module_pin_mux(i2c0_pin_mux);
}

void enable_board_pin_mux(void)
{
	/* Do board-specific muxes. */
	configure_module_pin_mux(vtten_pin_mux);
	configure_module_pin_mux(i2c0_pin_mux);
	configure_module_pin_mux(rgmii2_pin_mux);
	configure_module_pin_mux(mmc0_pin_mux);
	configure_module_pin_mux(mmc1_pin_mux);
	configure_module_pin_mux(spi0_pin_mux);
}

void config_phy_reg(const char *devname, unsigned char addr)
{
	/*PHY LED status*/
	miiphy_write(devname, addr, 0x1f, 0x0007);
	miiphy_write(devname, addr, 0x1e, 0x002c);
	miiphy_write(devname, addr, 0x1c, 0x9240);
	miiphy_write(devname, addr, 0x1a, 0x0091);
	miiphy_write(devname, addr, 0x1f, 0x0000);
	
	/*PHY LED speed realtek*/
	miiphy_write(devname, addr, 0x1f, 0x0005);
	miiphy_write(devname, addr, 0x05, 0x8b82);
	miiphy_write(devname, addr, 0x06, 0x052b);
	miiphy_write(devname, addr, 0x1f, 0x0000);
	
	/*PHY 125MHz disable and SSC enable, for EMI realtek FAE help*/
	miiphy_write(devname, addr, 0x1f, 0x0000);
	miiphy_write(devname, addr, 0x10, 0x017e);
	miiphy_write(devname, addr, 0x1f, 0x0000);
	miiphy_write(devname, addr, 0x1f, 0x0007);
	miiphy_write(devname, addr, 0x1e, 0x00a0);
	miiphy_write(devname, addr, 0x1a, 0x38d0);
	miiphy_write(devname, addr, 0x1f, 0x0000);
}
