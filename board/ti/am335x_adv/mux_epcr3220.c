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
	{OFFSET(uart1_ctsn), (MODE(0) | PULLUP_EN)},  		/* UART1_CTS */
	{OFFSET(uart1_rtsn), (MODE(0) | PULLUDEN)},             /* UART1_RTS */
	{-1},
};

static struct module_pin_mux uart3_pin_mux[] = {
	{OFFSET(spi0_cs1), (MODE(1) | PULLUP_EN | RXACTIVE)},	/* UART3_RXD */
	{OFFSET(ecap0_in_pwm0_out), (MODE(1) | PULLUDEN)},	/* UART3_TXD */
	{OFFSET(lcd_data10), (MODE(6) | PULLUP_EN)},		/* UART3_CTS */
	{OFFSET(lcd_data11), (MODE(6) | PULLUDEN)},		/* UART3_RTS */
	{-1},
};

static struct module_pin_mux uart4_pin_mux[] = {
	{OFFSET(gpmc_wait0), (MODE(6) | PULLUP_EN | RXACTIVE)},	/* UART4_RXD */
	{OFFSET(gpmc_wpn), (MODE(6) | PULLUDEN)},		/* UART4_TXD */
	{OFFSET(lcd_data12), (MODE(6) | PULLUP_EN)},		/* UART4_CTS */
	{OFFSET(lcd_data13), (MODE(6) | PULLUDEN)},		/* UART4_RTS */
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
        {OFFSET(gpmc_ad7), (MODE(1) | RXACTIVE | PULLUP_EN)},   /* MMC1_DAT7 */
        {OFFSET(gpmc_ad6), (MODE(1) | RXACTIVE | PULLUP_EN)},   /* MMC1_DAT6 */
        {OFFSET(gpmc_ad5), (MODE(1) | RXACTIVE | PULLUP_EN)},   /* MMC1_DAT5 */
        {OFFSET(gpmc_ad4), (MODE(1) | RXACTIVE | PULLUP_EN)},   /* MMC1_DAT4 */
	{OFFSET(gpmc_ad3), (MODE(1) | RXACTIVE | PULLUP_EN)},	/* MMC1_DAT3 */
	{OFFSET(gpmc_ad2), (MODE(1) | RXACTIVE | PULLUP_EN)},	/* MMC1_DAT2 */
	{OFFSET(gpmc_ad1), (MODE(1) | RXACTIVE | PULLUP_EN)},	/* MMC1_DAT1 */
	{OFFSET(gpmc_ad0), (MODE(1) | RXACTIVE | PULLUP_EN)},	/* MMC1_DAT0 */
	{OFFSET(gpmc_csn1), (MODE(2) | RXACTIVE | PULLUP_EN)},	/* MMC1_CLK */
	{OFFSET(gpmc_csn2), (MODE(2) | RXACTIVE | PULLUP_EN)},	/* MMC1_CMD */
	{-1},
};

static struct module_pin_mux mmc2_pin_mux[] = {
        {OFFSET(gpmc_ad15), (MODE(3) | RXACTIVE | PULLUP_EN)},   /* MMC2_DAT3 */
        {OFFSET(gpmc_ad14), (MODE(3) | RXACTIVE | PULLUP_EN)},   /* MMC2_DAT2 */
        {OFFSET(gpmc_ad13), (MODE(3) | RXACTIVE | PULLUP_EN)},   /* MMC2_DAT1 */
        {OFFSET(gpmc_ad12), (MODE(3) | RXACTIVE | PULLUP_EN)},   /* MMC2_DAT0 */
        {OFFSET(gpmc_clk), (MODE(3) | RXACTIVE | PULLUP_EN)},  /* MMC2_CLK */
        {OFFSET(gpmc_csn3), (MODE(3) | RXACTIVE | PULLUP_EN)},  /* MMC2_CMD */
        {-1},
};

static struct module_pin_mux i2c0_pin_mux[] = {
	{OFFSET(i2c0_sda), (MODE(0) | RXACTIVE |
			PULLUDEN | SLEWCTRL)}, /* I2C_DATA */
	{OFFSET(i2c0_scl), (MODE(0) | RXACTIVE |
			PULLUDEN | SLEWCTRL)}, /* I2C_SCLK */
	{-1},
};

static struct module_pin_mux i2c1_pin_mux[] = {
        {OFFSET(uart0_ctsn), (MODE(3) | RXACTIVE |
                        PULLUDEN | SLEWCTRL)}, /* I2C_DATA */
        {OFFSET(uart0_rtsn), (MODE(3) | RXACTIVE |
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

static struct module_pin_mux spi1_pin_mux[] = {
        {OFFSET(mii1_col), (MODE(2) | RXACTIVE | PULLUDEN)},   /* SPI1_SCLK */
        {OFFSET(mii1_crs), (MODE(2) | RXACTIVE |
                        PULLUDEN | PULLUP_EN)},                 /* SPI1_D0 */
        {OFFSET(mii1_rxerr), (MODE(2) | RXACTIVE | PULLUDEN)},     /* SPI1_D1 */
        {OFFSET(rmii1_refclk), (MODE(2) | RXACTIVE |
                        PULLUDEN | PULLUP_EN)},                 /* SPI1_CS0 */
        {-1},
};

static struct module_pin_mux vtten_pin_mux[] = {
	{OFFSET(gpmc_oen_ren), (MODE(7) | PULLUDEN)},	/* GPIO2_3 */
	{-1},
};

static struct module_pin_mux adv_wdt_pin_mux[] = {
	{OFFSET(gpmc_wen), (MODE(7) | PULLUDEN | PULLDOWN_EN)},             /* GPIO2_4 */
	{OFFSET(gpmc_be0n_cle), (MODE(7) | PULLUDEN | PULLDOWN_EN)},             /* GPIO2_5 */
	{-1},
};
static struct module_pin_mux adv_gpio_pin_mux[] = {
	{OFFSET(lcd_ac_bias_en), (MODE(7) | PULLUDEN | PULLUP_EN)},		/* GPIO2_25 */
	{OFFSET(mcasp0_ahclkx), (MODE(7) | PULLUDEN | PULLDOWN_EN)},	/* GPIO3_21 */
	{OFFSET(lcd_hsync), (MODE(7) | PULLUDEN | PULLDOWN_EN)},	/* GPIO2_23 */
	{-1},
};


static struct module_pin_mux rgmii1_pin_mux[] = {
	{OFFSET(mii1_txen), MODE(2)},			/* RGMII1_TCTL */
	{OFFSET(mii1_rxdv), MODE(2) | RXACTIVE},	/* RGMII1_RCTL */
	{OFFSET(mii1_txd3), MODE(2)},			/* RGMII1_TD3 */
	{OFFSET(mii1_txd2), MODE(2)},			/* RGMII1_TD2 */
	{OFFSET(mii1_txd1), MODE(2)},			/* RGMII1_TD1 */
	{OFFSET(mii1_txd0), MODE(2)},			/* RGMII1_TD0 */
	{OFFSET(mii1_txclk), MODE(2)},			/* RGMII1_TCLK */
	{OFFSET(mii1_rxclk), MODE(2) | RXACTIVE},	/* RGMII1_RCLK */
	{OFFSET(mii1_rxd3), MODE(2) | RXACTIVE},	/* RGMII1_RD3 */
	{OFFSET(mii1_rxd2), MODE(2) | RXACTIVE},	/* RGMII1_RD2 */
	{OFFSET(mii1_rxd1), MODE(2) | RXACTIVE},	/* RGMII1_RD1 */
	{OFFSET(mii1_rxd0), MODE(2) | RXACTIVE},	/* RGMII1_RD0 */
	{OFFSET(mdio_data), MODE(0) | RXACTIVE | PULLUP_EN},/* MDIO_DATA */
	{OFFSET(mdio_clk), MODE(0) | PULLUP_EN},	/* MDIO_CLK */
	{-1},
};

static struct module_pin_mux rgmii2_pin_mux[] = {
        {OFFSET(gpmc_a0), MODE(2)},                   /* RGMII2_TCTL */
        {OFFSET(gpmc_a1), MODE(2) | RXACTIVE},        /* RGMII2_RCTL */
        {OFFSET(gpmc_a2), MODE(2)},                   /* RGMII2_TD3 */
        {OFFSET(gpmc_a3), MODE(2)},                   /* RGMII2_TD2 */
        {OFFSET(gpmc_a4), MODE(2)},                   /* RGMII2_TD1 */
        {OFFSET(gpmc_a5), MODE(2)},                   /* RGMII2_TD0 */
        {OFFSET(gpmc_a6), MODE(2)},                  /* RGMII2_TCLK */
        {OFFSET(gpmc_a7), MODE(2) | RXACTIVE},       /* RGMII2_RCLK */
        {OFFSET(gpmc_a8), MODE(2) | RXACTIVE},        /* RGMII2_RD3 */
        {OFFSET(gpmc_a9), MODE(2) | RXACTIVE},        /* RGMII2_RD2 */
        {OFFSET(gpmc_a10), MODE(2) | RXACTIVE},        /* RGMII2_RD1 */
        {OFFSET(gpmc_a11), MODE(2) | RXACTIVE},        /* RGMII2_RD0 */
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
	//configure_module_pin_mux(uart2_pin_mux);
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
	//configure_module_pin_mux(uart5_pin_mux);
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
	configure_module_pin_mux(i2c1_pin_mux);
	configure_module_pin_mux(rgmii1_pin_mux);
	configure_module_pin_mux(rgmii2_pin_mux);
	configure_module_pin_mux(mmc0_pin_mux);
	configure_module_pin_mux(mmc1_pin_mux);
	configure_module_pin_mux(mmc2_pin_mux);
	configure_module_pin_mux(spi0_pin_mux);
	configure_module_pin_mux(spi1_pin_mux);
	configure_module_pin_mux(adv_gpio_pin_mux);
	//configure_module_pin_mux(adv_wdt_pin_mux);
}

void config_phy_reg(const char *devname, unsigned char addr)
{
	/*Enable RXC SSC*/
	//miiphy_write(devname, addr, 0x1f, 0x0c44);
	//miiphy_write(devname, addr, 0x13, 0x5f00);
	//miiphy_write(devname, addr, 0x1f, 0x0000);
	//miiphy_write(devname, addr, 0x00, 0x9200);

	/*CLK_OUT disable*/
	//miiphy_write(devname, addr, 0x1f, 0x0a43);
	//miiphy_write(devname, addr, 0x19, 0x0862);
	//miiphy_write(devname, addr, 0x1f, 0x0000);
	//miiphy_write(devname, addr, 0x00, 0x9200);

	/*Enable TX_delay*/
	//miiphy_write(devname, addr, 0x1f, 0x0d08);
	//miiphy_write(devname, addr, 0x11, 0x0109);
	//miiphy_write(devname, addr, 0x1f, 0x0000);

	/*Change PHY LED status*/
	miiphy_write(devname, addr, 0x1f, 0x0d04);
	miiphy_write(devname, addr, 0x10, 0x091b);
	miiphy_write(devname, addr, 0x11, 0x0000);
	miiphy_write(devname, addr, 0x12, 0x03ea);
	miiphy_write(devname, addr, 0x1f, 0x0000);
}

void adv_pcie_timing(void)
{
	gpio_request(PCIE_PWR_EN, "pcie_pwr_en");
	gpio_direction_output(PCIE_PWR_EN, 0);
	gpio_request(PCIE_RST, "pcie_rst");
	gpio_direction_output(PCIE_RST, 0);
	gpio_set_value(PCIE_PWR_EN, 1);
	udelay(500000);
	gpio_set_value(PCIE_RST, 1);
}

void adv_wdt_default_setting(void)
{
        gpio_request(ADV_WDT_EN, "adv_wdt_en");
        gpio_direction_output(ADV_WDT_EN, 0);
        gpio_set_value(ADV_WDT_EN, 0);
}

void adv_wdt_feed(void)
{
	gpio_request(ADV_WDT_GPIO, "adv_wdt_gpio");
	gpio_direction_output(ADV_WDT_GPIO, 0);
	gpio_set_value(ADV_WDT_GPIO, 0);
}

