// SPDX-License-Identifier: GPL-2.0+
/*
 * board.c
 *
 * Board functions for TI AM335X based boards
 *
 * Copyright (C) 2011, Texas Instruments, Incorporated - http://www.ti.com/
 */

#include <common.h>
#include <dm.h>
#include <env.h>
#include <errno.h>
#include <image.h>
#include <init.h>
#include <hang.h>
#include <malloc.h>
#include <net.h>
#include <spl.h>
#include <serial.h>
#include <asm/arch/cpu.h>
#include <asm/arch/hardware.h>
#include <asm/arch/omap.h>
#include <asm/arch/ddr_defs.h>
#include <asm/arch/clock.h>
#include <asm/arch/clk_synthesizer.h>
#include <asm/arch/gpio.h>
#include <asm/arch/mmc_host_def.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mem.h>
#include <asm/io.h>
#include <asm/emif.h>
#include <asm/gpio.h>
#include <asm/omap_common.h>
#include <asm/omap_sec_common.h>
#include <asm/omap_mmc.h>
#include <i2c.h>
#include <miiphy.h>
#include <cpsw.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <power/tps65217.h>
#include <power/tps65910.h>
#include <env_internal.h>
#include <watchdog.h>
#include "../common/board_detect.h"
#include <spi_flash.h>
#include "board.h"

DECLARE_GLOBAL_DATA_PTR;

static struct ctrl_dev *cdev = (struct ctrl_dev *)CTRL_DEVICE_BASE;

#define GPIO0_RISINGDETECT	(AM33XX_GPIO0_BASE + OMAP_GPIO_RISINGDETECT)
#define GPIO1_RISINGDETECT	(AM33XX_GPIO1_BASE + OMAP_GPIO_RISINGDETECT)

#define GPIO0_IRQSTATUS1	(AM33XX_GPIO0_BASE + OMAP_GPIO_IRQSTATUS1)
#define GPIO1_IRQSTATUS1	(AM33XX_GPIO1_BASE + OMAP_GPIO_IRQSTATUS1)

#define GPIO0_IRQSTATUSRAW	(AM33XX_GPIO0_BASE + 0x024)
#define GPIO1_IRQSTATUSRAW	(AM33XX_GPIO1_BASE + 0x024)

#ifndef CONFIG_SKIP_LOWLEVEL_INIT
static const struct ddr_data ddr3_evm_data = {
	.datardsratio0 = MT41J512M8RH125_RD_DQS,
	.datawdsratio0 = MT41J512M8RH125_WR_DQS,
	.datafwsratio0 = MT41J512M8RH125_PHY_FIFO_WE,
	.datawrsratio0 = MT41J512M8RH125_PHY_WR_DATA,
};

static const struct cmd_control ddr3_evm_cmd_ctrl_data = {
	.cmd0csratio = MT41J512M8RH125_RATIO,
	.cmd0iclkout = MT41J512M8RH125_INVERT_CLKOUT,

	.cmd1csratio = MT41J512M8RH125_RATIO,
	.cmd1iclkout = MT41J512M8RH125_INVERT_CLKOUT,

	.cmd2csratio = MT41J512M8RH125_RATIO,
	.cmd2iclkout = MT41J512M8RH125_INVERT_CLKOUT,
};

static struct emif_regs ddr3_evm_emif_reg_data = {
	.sdram_config = MT41J512M8RH125_EMIF_SDCFG,
	.ref_ctrl = MT41J512M8RH125_EMIF_SDREF,
	.sdram_tim1 = MT41J512M8RH125_EMIF_TIM1,
	.sdram_tim2 = MT41J512M8RH125_EMIF_TIM2,
	.sdram_tim3 = MT41J512M8RH125_EMIF_TIM3,
	.ocp_config =  EMIF_OCP_CONFIG_AM335X_EVM,
	.zq_config = MT41J512M8RH125_ZQ_CFG,
	.emif_ddr_phy_ctlr_1 = MT41J512M8RH125_EMIF_READ_LATENCY |
				PHY_EN_DYN_PWRDN,
};

const struct ctrl_ioregs ioregs_evm15 = {
	.cm0ioctl		= MT41J512M8RH125_IOCTRL_VALUE,
	.cm1ioctl		= MT41J512M8RH125_IOCTRL_VALUE,
	.cm2ioctl		= MT41J512M8RH125_IOCTRL_VALUE,
	.dt0ioctl		= MT41J512M8RH125_IOCTRL_VALUE,
	.dt1ioctl		= MT41J512M8RH125_IOCTRL_VALUE,
};

#define OSC	(V_OSCK/1000000)
const struct dpll_params dpll_ddr[] ={
	{266, OSC-1, 1, -1, -1, -1, -1}, 	/* 266MHz DDR dpll */
	{303, OSC-1, 1, -1, -1, -1, -1},	/* 303MHz DDR dpll */
	{400, OSC-1, 1, -1, -1, -1, -1}		/* 400MHz DDR dpll*/
};

#ifdef CONFIG_SPL_OS_BOOT
int spl_start_uboot(void)
{
#ifdef CONFIG_SPL_SERIAL_SUPPORT
	/* break into full u-boot on 'c' */
	if (serial_tstc() && serial_getc() == 'c')
		return 1;
#endif

#ifdef CONFIG_SPL_ENV_SUPPORT
	env_init();
	env_load();
	if (env_get_yesno("boot_os") != 1)
		return 1;
#endif

	return 0;
}
#endif

const struct dpll_params *get_dpll_ddr_params(void)
{
    return &dpll_ddr[2];
}

const struct dpll_params *get_dpll_mpu_params(void)
{
	int ind = get_sys_clk_index();
	int freq = am335x_get_efuse_mpu_max_freq(cdev);

	switch (freq) {
	case MPUPLL_M_1000:
		return &dpll_mpu_opp[ind][5];
	case MPUPLL_M_800:
		return &dpll_mpu_opp[ind][4];
	case MPUPLL_M_720:
		return &dpll_mpu_opp[ind][3];
	case MPUPLL_M_600:
		return &dpll_mpu_opp[ind][2];
	case MPUPLL_M_500:
		return &dpll_mpu_opp100;
	case MPUPLL_M_300:
		return &dpll_mpu_opp[ind][0];
	}

	return &dpll_mpu_opp[ind][0];
}

void scale_vcores_generic(int freq)
{
	int sil_rev, mpu_vdd;

	/*
	 * The GP EVM, IDK and EVM SK use a TPS65910 PMIC.  For all
	 * MPU frequencies we support we use a CORE voltage of
	 * 1.10V.  For MPU voltage we need to switch based on
	 * the frequency we are running at.
	 */
#ifndef CONFIG_DM_I2C
	if (i2c_probe(TPS65910_CTRL_I2C_ADDR))
		return;
#else
	if (power_tps65910_init(0))
		return;
#endif
	/*
	 * Depending on MPU clock and PG we will need a different
	 * VDD to drive at that speed.
	 */
	sil_rev = readl(&cdev->deviceid) >> 28;
	mpu_vdd = am335x_get_tps65910_mpu_vdd(sil_rev, freq);

	/* Tell the TPS65910 to use i2c */
	tps65910_set_i2c_control();

	/* First update MPU voltage. */
	if (tps65910_voltage_update(MPU, mpu_vdd))
		return;

	/* Second, update the CORE voltage. */
	if (tps65910_voltage_update(CORE, TPS65910_OP_REG_SEL_1_1_0))
		return;

#ifdef CONFIG_TARGET_AM335X_ADVANTECH
	/*update the VIO maximum load current. */
	if(adv_tps65910_config())
		return;
#endif
}

void gpi2c_init(void)
{
	/* When needed to be invoked prior to BSS initialization */
	static bool first_time = true;

	if (first_time) {
		enable_i2c0_pin_mux();
#ifndef CONFIG_DM_I2C
		i2c_init(CONFIG_SYS_OMAP24_I2C_SPEED,
			 CONFIG_SYS_OMAP24_I2C_SLAVE);
#endif
		first_time = false;
	}
}

void scale_vcores(void)
{
	int freq;

	gpi2c_init();
	freq = am335x_get_efuse_mpu_max_freq(cdev);

	scale_vcores_generic(freq);
}

void set_uart_mux_conf(void)
{
#if CONFIG_CONS_INDEX == 1
	enable_uart0_pin_mux();
#elif CONFIG_CONS_INDEX == 2
	enable_uart1_pin_mux();
#elif CONFIG_CONS_INDEX == 3
	enable_uart2_pin_mux();
#elif CONFIG_CONS_INDEX == 4
	enable_uart3_pin_mux();
#elif CONFIG_CONS_INDEX == 5
	enable_uart4_pin_mux();
#elif CONFIG_CONS_INDEX == 6
	enable_uart5_pin_mux();
#endif
}

void set_mux_conf_regs(void)
{
	enable_board_pin_mux();
}

void sdram_init(void)
{
#ifdef GPIO_DDR_VTT_EN
	gpio_request(GPIO_DDR_VTT_EN, "ddr_vtt_en");
	gpio_direction_output(GPIO_DDR_VTT_EN, 1);
#endif

#ifdef PCIE_PWR_EN
	adv_pcie_timing();
#endif

#ifdef ADV_WDT_EN
	adv_wdt_default_setting();
#endif

	config_ddr(400, &ioregs_evm15, &ddr3_evm_data,
		   &ddr3_evm_cmd_ctrl_data, &ddr3_evm_emif_reg_data, 0);
}
#endif

#if defined(CONFIG_OF_BOARD_SETUP) && defined(CONFIG_OF_CONTROL) && \
	defined(CONFIG_DM_ETH) && defined(CONFIG_DRIVER_TI_CPSW)
int ft_board_setup(void *fdt, struct bd_info *bd)
{
	ft_cpu_setup(fdt, bd);

	return 0;
}
#endif

/*
 * Basic board specific setup.  Pinmux has been handled already.
 */
int board_init(void)
{
#if defined(CONFIG_HW_WATCHDOG)
	hw_watchdog_init();
#endif

	gd->bd->bi_boot_params = CONFIG_SYS_SDRAM_BASE + 0x100;
#if defined(CONFIG_NOR) || defined(CONFIG_NAND)
	gpmc_init();
#endif

	return 0;
}

static int boardcfg_get_mac(uint8_t  ethnum)
{
    int rc = 0;
    static struct spi_flash *flash;
    struct boardcfg_t {
        uint8_t mac[CONFIG_ACTIVE_EPHY_NUM][6];
    } boardcfg;
    uint8_t use_default= 1;    // use default mac address.
    uint8_t *macaddr = NULL;
	uint32_t mac_hi, mac_lo;

    flash = spi_flash_probe(CONFIG_MAC_ADDR_SPI_BUS,CONFIG_MAC_ADDR_SPI_CS,
                            CONFIG_MAC_ADDR_SPI_HZ,CONFIG_MAC_ADDR_SPI_MODE);
    if (!flash){
        rc =-ENXIO;
		goto out;
	}

    if(spi_flash_read(flash,CONFIG_MAC_OFFSET,sizeof(boardcfg), &boardcfg)==0) {
		/* Read success! */
		/* We must matched the old mac_wrrite tools for mac[0][1].*/
		macaddr = &boardcfg.mac[0][0];
        if (is_valid_ethaddr(macaddr)){
				eth_env_set_enetaddr("ethaddr", macaddr);
				use_default= 0;
		}
		/* Ethernet 2 */
        if(ethnum > 1){
			macaddr = &boardcfg.mac[1][0];
                  if (is_valid_ethaddr(macaddr)){
					eth_env_set_enetaddr("eth1addr", macaddr);
					use_default= 0;
			}
		}
    } else {
        printf("SPI Read fail!!\n");
        rc = -1;
    }

	spi_flash_free(flash);

out:
	if(use_default){
		/* try reading mac address from efuse */
		puts("Skipped ethaddr assignment due to invalid,using default!\n");
		macaddr = &boardcfg.mac[0][0];

		mac_lo = readl(&cdev->macid0l);
		mac_hi = readl(&cdev->macid0h);
		macaddr[0] = mac_hi & 0xFF;
		macaddr[1] = (mac_hi & 0xFF00) >> 8;
		macaddr[2] = (mac_hi & 0xFF0000) >> 16;
		macaddr[3] = (mac_hi & 0xFF000000) >> 24;
		macaddr[4] = mac_lo & 0xFF;
		macaddr[5] = (mac_lo & 0xFF00) >> 8;
		if (is_valid_ethaddr(macaddr))
				eth_env_set_enetaddr("ethaddr", macaddr);

		if(ethnum > 1){
			mac_lo = readl(&cdev->macid1l);
			mac_hi = readl(&cdev->macid1h);
			macaddr[0] = mac_hi & 0xFF;
			macaddr[1] = (mac_hi & 0xFF00) >> 8;
			macaddr[2] = (mac_hi & 0xFF0000) >> 16;
			macaddr[3] = (mac_hi & 0xFF000000) >> 24;
			macaddr[4] = mac_lo & 0xFF;
			macaddr[5] = (mac_lo & 0xFF00) >> 8;
			if (is_valid_ethaddr(macaddr))
				eth_env_set_enetaddr("eth1addr", macaddr);
		}
	}

    return rc;
}

#ifdef CONFIG_BOARD_LATE_INIT
int board_late_init(void)
{
        env_set("board_name", CONFIG_SYS_BOARD_NAME);

#ifdef UART_POWER
	gpio_request(UART_POWER, "uart_power");
	gpio_direction_output(UART_POWER, 0);
	gpio_set_value(UART_POWER, 1);
#endif

#ifdef ADV_WDT_GPIO
	adv_wdt_feed();
#endif

#if (defined(CONFIG_DRIVER_TI_CPSW) && !defined(CONFIG_SPL_BUILD))
	boardcfg_get_mac(CONFIG_ACTIVE_EPHY_NUM);
#endif

	return 0;
}
#endif

/* CPSW platdata */
#if (defined(CONFIG_DRIVER_TI_CPSW) && !defined(CONFIG_SPL_BUILD))
struct cpsw_slave_data slave_data[] = {
#ifdef CONFIG_ENABLE_EPHY0
	{
		.slave_reg_ofs  = CPSW_SLAVE0_OFFSET,
		.sliver_reg_ofs = CPSW_SLIVER0_OFFSET,
		.phy_addr       = 0,
	},
#endif
#ifdef CONFIG_ENABLE_EPHY1
	{
		.slave_reg_ofs  = CPSW_SLAVE1_OFFSET,
		.sliver_reg_ofs = CPSW_SLIVER1_OFFSET,
		.phy_addr       = 1,
	},
#endif
};

struct cpsw_platform_data am335_eth_data = {
	.cpsw_base		= CPSW_BASE,
	.version		= CPSW_CTRL_VERSION_2,
	.bd_ram_ofs		= CPSW_BD_OFFSET,
	.ale_reg_ofs		= CPSW_ALE_OFFSET,
	.cpdma_reg_ofs		= CPSW_CPDMA_OFFSET,
	.mdio_div		= CPSW_MDIO_DIV,
	.host_port_reg_ofs	= CPSW_HOST_PORT_OFFSET,
	.channels		= 8,
	.slaves			= 2,
	.slave_data		= slave_data,
	.ale_entries		= 1024,
	.mac_control		= 0x20,
	.active_slave		= 0,
	.mdio_base		= 0x4a101000,
	.gmii_sel		= 0x44e10650,
	.phy_sel_compat		= "ti,am3352-cpsw-phy-sel",
	.syscon_addr		= 0x44e10630,
	.macid_sel_compat	= "cpsw,am33xx",
};

struct eth_pdata cpsw_pdata = {
	.iobase = 0x4a100000,
	.phy_interface = 0,
	.priv_pdata = &am335_eth_data,
};

U_BOOT_DEVICE(am335x_eth) = {
	.name = "eth_cpsw",
	.platdata = &cpsw_pdata,
};
#endif

#ifdef CONFIG_SPL_LOAD_FIT
int board_fit_config_name_match(const char *name)
{
	if(!strcmp(name, CONFIG_DEFAULT_DEVICE_TREE))
		return 0;
	else
		return -1;
}
#endif

#ifdef CONFIG_TI_SECURE_DEVICE
void board_fit_image_post_process(const void *fit, int node, void **p_image,
				  size_t *p_size)
{
	secure_boot_verify_image(p_image, p_size);
}
#endif

#if !CONFIG_IS_ENABLED(OF_CONTROL)
static const struct omap_hsmmc_plat am335x_mmc0_platdata = {
	.base_addr = (struct hsmmc *)OMAP_HSMMC1_BASE,
	.cfg.host_caps = MMC_MODE_HS_52MHz | MMC_MODE_HS | MMC_MODE_4BIT,
	.cfg.f_min = 400000,
	.cfg.f_max = 52000000,
	.cfg.voltages = MMC_VDD_32_33 | MMC_VDD_33_34 | MMC_VDD_165_195,
	.cfg.b_max = CONFIG_SYS_MMC_MAX_BLK_COUNT,
};

U_BOOT_DEVICE(am335x_mmc0) = {
	.name = "omap_hsmmc",
	.platdata = &am335x_mmc0_platdata,
};

static const struct omap_hsmmc_plat am335x_mmc1_platdata = {
	.base_addr = (struct hsmmc *)OMAP_HSMMC2_BASE,
	.cfg.host_caps = MMC_MODE_HS_52MHz | MMC_MODE_HS | MMC_MODE_8BIT,
	.cfg.f_min = 400000,
	.cfg.f_max = 52000000,
	.cfg.voltages = MMC_VDD_32_33 | MMC_VDD_33_34 | MMC_VDD_165_195,
	.cfg.b_max = CONFIG_SYS_MMC_MAX_BLK_COUNT,
};

U_BOOT_DEVICE(am335x_mmc1) = {
	.name = "omap_hsmmc",
	.platdata = &am335x_mmc1_platdata,
};
#endif
