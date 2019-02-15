/*
 * board.c
 *
 * Board functions for Advantech based boards
 *
 * Copyright (C) 2017, Advantech, Incorporated - http://www.advantech.com/
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <errno.h>
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
#include <i2c.h>
#include <miiphy.h>
#include <cpsw.h>
#include <power/tps65217.h>
#include <power/tps65910.h>
#include <watchdog.h>
#include <environment.h>
#include <libfdt.h>
#include <fdt_support.h>
#include <spi_flash.h>
#include "board.h"

DECLARE_GLOBAL_DATA_PTR;

#if  !defined(CONFIG_SPL_BUILD) && (defined( CONFIG_TARGET_UBC440A1_1G))
#define GPIO_TO_PIN(bank, gpio)		(32 * (bank) + (gpio))
#define UART_POWER		GPIO_TO_PIN(2, 25)
#endif

static struct ctrl_dev *cdev = (struct ctrl_dev *)CTRL_DEVICE_BASE;

static void  board_set_boot_device(void);

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
	.ocp_config = EMIF_OCP_CONFIG_BLACK,
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

#ifdef CONFIG_SPL_OS_BOOT
int spl_start_uboot(void)
{
	/* break into full u-boot on 'c' */
	if (serial_tstc() && serial_getc() == 'c')
		return 1;

#ifdef CONFIG_SPL_ENV_SUPPORT
	env_init();
	env_relocate_spec();
	if (getenv_yesno("boot_os") != 1)
		return 1;
#endif

	return 0;
}
#endif

#define OSC	(V_OSCK/1000000)
const struct dpll_params dpll_ddr[] ={
	{266, OSC-1, 1, -1, -1, -1, -1}, 	/* 266MHz DDR dpll */
	{303, OSC-1, 1, -1, -1, -1, -1},	/* 303MHz DDR dpll */
	{400, OSC-1, 1, -1, -1, -1, -1}		/* 400MHz DDR dpll*/
};

void am33xx_spl_board_init(void)
{
	int mpu_vdd;
	int sil_rev;	

	/* Enable I2c bus, and default bus is 0. */	
	i2c_set_bus_num(0);

	/* Get the frequency */
	dpll_mpu_opp100.m = am335x_get_efuse_mpu_max_freq(cdev);

	/*
	 * The GP EVM, IDK and EVM SK use a TPS65910 PMIC.  For all
	 * MPU frequencies we support we use a CORE voltage of
	 * 1.1375V.  For MPU voltage we need to switch based on
	 * the frequency we are running at.
	 */
	if (i2c_probe(TPS65910_CTRL_I2C_ADDR))
		return;

	/*
	 * Depending on MPU clock and PG we will need a different
	 * VDD to drive at that speed.
	 */
	sil_rev = readl(&cdev->deviceid) >> 28;
	mpu_vdd = am335x_get_tps65910_mpu_vdd(sil_rev,
					      dpll_mpu_opp100.m);

	/* Tell the TPS65910 to use i2c */
	tps65910_set_i2c_control();

	/* First update MPU voltage. */
	if (tps65910_voltage_update(MPU, mpu_vdd))
		return;

	/* Second, update the CORE voltage. */
	if (tps65910_voltage_update(CORE, TPS65910_OP_REG_SEL_1_1_3))
		return;

	/* Set CORE Frequencies to OPP100 */
	do_setup_dpll(&dpll_core_regs, &dpll_core_opp100);

	/* Set MPU Frequency to what we detected now that voltages are set */
	do_setup_dpll(&dpll_mpu_regs, &dpll_mpu_opp100);
}

const struct dpll_params *get_dpll_ddr_params(void)
{
	enable_i2c0_pin_mux();
	i2c_init(CONFIG_SYS_OMAP24_I2C_SPEED, CONFIG_SYS_OMAP24_I2C_SLAVE);

	/* We use the 400MHz frequency DDR.*/
	return &dpll_ddr[2];
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
	/*
	 * Advantech platform and later use GPIO_DDR_VTT_EN to enable DDR3.
	 * This is safe enough to do on older revs.
	 */
	gpio_request(GPIO_DDR_VTT_EN, "ddr_vtt_en");
	gpio_direction_output(GPIO_DDR_VTT_EN, 1);

	/* Configure DDR.. 
	* Note: we will no longre use the "board_is_xx" to decide
	* which board we used, because we has no rom/flash information
	* about board and the board has been certain.  
	* In reality, the first parameter will not be used, and it 
	* has configured the dpll in prcm_init(). 400 is a dummy data!
	*/
	config_ddr(400, &ioregs_evm15, &ddr3_evm_data,
		   &ddr3_evm_cmd_ctrl_data, &ddr3_evm_emif_reg_data, 0);
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

#ifdef CONFIG_BOARD_LATE_INIT
int board_late_init(void)
{
#ifndef CONFIG_SPL_BUILD
	board_set_boot_device();
#ifdef CONFIG_TARGET_UBC440A1_1G
	gpio_request(UART_POWER, "uart_power");
	gpio_direction_output(UART_POWER, 0);
	gpio_set_value(UART_POWER, 1);
#endif
#endif
	return 0;
}
#endif

#if (defined(CONFIG_DRIVER_TI_CPSW) && !defined(CONFIG_SPL_BUILD)) || \
	(defined(CONFIG_SPL_ETH_SUPPORT) && defined(CONFIG_SPL_BUILD))
static void cpsw_control(int enabled)
{
	/* VTP can be added here */

	return;
}

static struct cpsw_slave_data cpsw_slaves[] = {
#ifdef CONFIG_ENABLE_EPHY0
	{
		.slave_reg_ofs	= 0x208,
		.sliver_reg_ofs	= 0xd80,
		.phy_addr	= CONFIG_EPHY0_PHY_ADDR,
	},
#endif
#ifdef CONFIG_ENABLE_EPHY1
	{
		.slave_reg_ofs	= 0x308,
		.sliver_reg_ofs	= 0xdc0,
		.phy_addr	= CONFIG_EPHY1_PHY_ADDR,
	},
#endif
};

static struct cpsw_platform_data cpsw_data = {
	.mdio_base		= CPSW_MDIO_BASE,
	.cpsw_base		= CPSW_BASE,
	.mdio_div		= 0xff,
	.channels		= 8,
	.cpdma_reg_ofs		= 0x800,
	.slaves			= ARRAY_SIZE(cpsw_slaves),
	.slave_data		= cpsw_slaves,
	.active_slave	= CONFIG_DEFAULT_ACTIVE_EPHY,
	.ale_reg_ofs		= 0xd00,
	.ale_entries		= 1024,
	.host_port_reg_ofs	= 0x108,
	.hw_stats_reg_ofs	= 0x900,
	.bd_ram_ofs		= 0x2000,
	.mac_control		= (1 << 5),
	.control		= cpsw_control,
	.host_port_num		= 0,
	.version		= CPSW_CTRL_VERSION_2,
};

/************************************************************************
 *
 * This is to get MAC from SPIflash and set it to environment. If has no 
 * mac information on flash, it will read from cpu efuse. 
 *
 ************************************************************************
 */
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
				eth_setenv_enetaddr("ethaddr", macaddr);
				use_default= 0;
		} 
		/* Ethernet 2 */
        if(ethnum > 1){
			macaddr = &boardcfg.mac[1][0];
       		if (is_valid_ethaddr(macaddr)){
					eth_setenv_enetaddr("eth1addr", macaddr);
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
				eth_setenv_enetaddr("ethaddr", macaddr);

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
				eth_setenv_enetaddr("eth1addr", macaddr);
		}
	}

    return rc;
}

/*
 * This function will:
 * Read the eFuse for MAC addresses, and set ethaddr/eth1addr/usbnet_devaddr
 * in the environment
 * Perform fixups to the PHY present on certain boards.  We only need this
 * function in:
 * - SPL with either CPSW or USB ethernet support
 * - Full U-Boot, with either CPSW or USB ethernet
 * Build in only these cases to avoid warnings about unused variables
 * when we build an SPL that has neither option but full U-Boot will.
 */
int board_eth_init(bd_t *bis)
{
	int rv = 0;	
	int i;
	
#if (defined(CONFIG_DRIVER_TI_CPSW) && !defined(CONFIG_SPL_BUILD)) || \
	(defined(CONFIG_SPL_ETH_SUPPORT) && defined(CONFIG_SPL_BUILD))
	boardcfg_get_mac(CONFIG_ACTIVE_EPHY_NUM);

#ifdef CONFIG_DRIVER_TI_CPSW
	/* advantech am335x just support RGMII i/f .*/
	writel((RGMII_MODE_ENABLE), &cdev->miisel);
	for(i=0;i<CONFIG_ACTIVE_EPHY_NUM;i++)
		cpsw_slaves[i].phy_if = PHY_INTERFACE_MODE_RGMII;

	rv = cpsw_register(&cpsw_data);
	if (rv < 0)
		printf("Error %d registering CPSW switch\n", rv);

#ifndef CONFIG_TARGET_AM335X_ADVANTECH
	const char *devname;
	devname = miiphy_get_current_dev();
	for(i=0;i<CONFIG_ACTIVE_EPHY_NUM;i++)
	{
		/*PHY LED status*/
		miiphy_write(devname, i, 0x1f, 0x0007);
		miiphy_write(devname, i, 0x1e, 0x002c);
		miiphy_write(devname, i, 0x1c, 0x9240);
		miiphy_write(devname, i, 0x1a, 0x0091);
		miiphy_write(devname, i, 0x1f, 0x0000);

		/*PHY LED speed realtek*/
		miiphy_write(devname, i, 0x1f, 0x0005);
		miiphy_write(devname, i, 0x05, 0x8b82);
		miiphy_write(devname, i, 0x06, 0x052b);
		miiphy_write(devname, i, 0x1f, 0x0000);

		/*PHY 125MHz disable and SSC enable, for EMI realtek FAE help*/
		miiphy_write(devname, i, 0x1f, 0x0000);
		miiphy_write(devname, i, 0x10, 0x017e);
		miiphy_write(devname, i, 0x1f, 0x0000);
		miiphy_write(devname, i, 0x1f, 0x0007);
		miiphy_write(devname, i, 0x1e, 0x00a0);
		miiphy_write(devname, i, 0x1a, 0x38d0);
		miiphy_write(devname, i, 0x1f, 0x0000);
	}
#endif
#endif

#endif
	return rv;
}
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

#if defined(CONFIG_OF_LIBFDT) && defined(CONFIG_OF_BOARD_SETUP)
int ft_board_setup(void *fdt, bd_t *bd)
{
	return 0;
}
#endif

#ifndef CONFIG_SPL_BUILD
static void  board_set_boot_device(void)
{
    int dev = (*(int *)CONFIG_SPL_PARAM_ADDR);
    int bcb_flag;

#ifdef CONFIG_ADV_OTA_SUPPORT
    if (dev != 0)
    {
        bcb_flag= recovery_check_and_clean_command();
    }
    switch(dev) {
	case 0:
        /* booting from MMC0(SD)*/
		printf("booting from SD\n");
		break;
	case 1:
		/* booting from MMC1(Nand)& No image in SD*/
		printf("booting from MMC1\n");
		if(bcb_flag)
		{
			setenv("finduuid","part uuid mmc 1:3 uuid");
			setenv("bootpart","1:3");
			setenv("loadimage","load mmc 1:3 ${loadaddr} ${bootdir}/${bootfile}");
			setenv("loadfdt","load mmc 1:3 ${fdtaddr} ${bootdir}/${fdtfile}");
		}
		else
		{
			setenv("finduuid","part uuid mmc 1:2 uuid");
			setenv("bootpart","1:2");
			setenv("loadimage","load mmc 1:2 ${loadaddr} ${bootdir}/${bootfile}");
			setenv("loadfdt","load mmc 1:2 ${fdtaddr} ${bootdir}/${fdtfile}");
		}
		break;
	default:
		/* booting from MMC1(Nand) & no insert SD.*/
		printf("booting from MMC1\n");
		if(bcb_flag)
		{
			setenv("finduuid","part uuid mmc 1:3 uuid");
			setenv("bootpart","1:3");
			setenv("loadimage","load mmc 1:3 ${loadaddr} ${bootdir}/${bootfile}");
			setenv("loadfdt","load mmc 1:3 ${fdtaddr} ${bootdir}/${fdtfile}");
		}
		else
		{
			setenv("finduuid","part uuid mmc 1:2 uuid");
			setenv("bootpart","1:2");
			setenv("loadimage","load mmc 1:2 ${loadaddr} ${bootdir}/${bootfile}");
			setenv("loadfdt","load mmc 1:2 ${fdtaddr} ${bootdir}/${fdtfile}");
		}
		break;
	}

#else
    switch(dev) {
	case 0:
        /* booting from MMC0(SD)*/
		printf("booting from SD\n");
		setenv("mmcdev", "0");
		setenv("finduuid","part uuid mmc 0:2 uuid");
		break;
	case 1:
		/* booting from MMC1(Nand)& No image in SD*/
		printf("booting from MMC1\n");
		setenv("mmcdev", "1");
		setenv("finduuid","part uuid mmc 1:2 uuid");
		break;
	default:
		/* booting from MMC1(Nand) & no insert SD.*/
		printf("booting from MMC1\n");
		setenv("mmcdev", "1");
		setenv("finduuid","part uuid mmc 1:2 uuid");
		break;
	}
#endif
	
}
#endif
