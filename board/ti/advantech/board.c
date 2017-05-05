/*
 * Copyright (C) 2016 Advantech Co.,Ltd 
 *
 * Author: yanwei.cao <yanwei.cao@advantech.com.cn>
 *
 * Based on board/ti/advantech/board.c
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <palmas.h>
#include <sata.h>
#include <usb.h>
#include <spi.h>
#include <spi_flash.h>
#include <asm/omap_common.h>
#include <asm/omap_sec_common.h>
#include <asm/emif.h>
#include <asm/gpio.h>
#include <asm/arch/gpio.h>
#include <asm/arch/clock.h>
#include <asm/arch/dra7xx_iodelay.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mmc_host_def.h>
#include <asm/arch/sata.h>
#include <asm/arch/gpio.h>
#include <asm/arch/omap.h>
#include <environment.h>
#include <usb.h>
#include <linux/usb/gadget.h>
#include <dwc3-uboot.h>
#include <dwc3-omap-uboot.h>
#include <ti-usb-phy-uboot.h>
#ifdef CONFIG_DRIVER_TI_CPSW
#include <cpsw.h>
#endif

#include "../common/board_detect.h"
#include "mux_data.h"

DECLARE_GLOBAL_DATA_PTR;

#define SYSINFO_BOARD_NAME_MAX_LEN	45

const struct omap_sysinfo sysinfo = {
	"Board: advantech\n"
};

static void  board_set_boot_device(void);

void emif_get_dmm_regs(const struct dmm_lisa_map_regs **dmm_lisa_regs)
{	
	*dmm_lisa_regs = &am57xx_lisa_regs;
}

void emif_get_reg_dump(u32 emif_nr, const struct emif_regs **regs)
{
	if (is_dra72x()){
		/* AM571x or dra72x family just has a EMIF interfaace. */
		*regs = &am571x_ddr3_532mhz_emif_regs;
	} else {
		switch (emif_nr) {
		case 1:
			*regs = &am572x_emif1_ddr3_532mhz_emif_regs;
			break;
		case 2:
			*regs = &am572x_emif2_ddr3_532mhz_emif_regs;
			break;
		}
	}
}

void emif_get_ext_phy_ctrl_const_regs(u32 emif_nr, const u32 **regs, u32 *size)
{
	if (is_dra72x()){
		/* AM571x or dra72x family just has one EMIF interfaace. */
		*regs = am571x_ddr3_ext_phy_ctrl_const_regs;
		*size = ARRAY_SIZE(am571x_ddr3_ext_phy_ctrl_const_regs);
	} else {
		switch (emif_nr) {
		case 1:
			*regs = am572x_emif1_ddr3_ext_phy_ctrl_const_regs;
			*size = ARRAY_SIZE(am572x_emif1_ddr3_ext_phy_ctrl_const_regs);
			break;
		case 2:
			*regs = am572x_emif2_ddr3_ext_phy_ctrl_const_regs;
			*size = ARRAY_SIZE(am572x_emif2_ddr3_ext_phy_ctrl_const_regs);
			break;
		}
	}
}

/* Add the pmic initialize. In reality, probe is not needed.*/
static int  tps659037_i2c_init(int i2c_bus)
{
	int rc;

	if (i2c_bus >= 0) {
		rc = i2c_set_bus_num(i2c_bus);
		if (rc)
			return rc;
	}

	return -1;
}

/* 
* We enable some other essential clock for advboot
*  to use some hardware module normally.
*/
void enable_extra_advboot_clock(void)
{
	u32 const clk_domains_essential[] = {
		0
	};

	u32 const clk_modules_hw_auto_essential[] = {
		0
	};

	u32 const clk_modules_explicit_en_essential[] = {
		(*prcm)->cm_l4per_i2c2_clkctrl,
		0
	};

	do_enable_clocks(clk_domains_essential,
			 clk_modules_hw_auto_essential,
			 clk_modules_explicit_en_essential,
			 1);	
}

void do_board_detect(void)
{
	u8 val;

	snprintf(sysinfo.board_string, sizeof(sysinfo.board_string),
			"Board: %s\n", CONFIG_SYS_BOARD_NAME);
	/* Initial i2c pmic*/
	if (is_dra72x()){
		tps659037_i2c_init(CONFIG_AM57XX_PMIC_BUS_ADDRESS);
	} else {
		enable_extra_advboot_clock();	
		tps659037_i2c_init(CONFIG_AM57XX_PMIC_BUS_ADDRESS);
	}

	/* Set pmic long press time to 6s */
	palmas_i2c_read_u8(TPS65903X_CHIP_P1, 0xA9, &val);
	val &=~0xC;
	palmas_i2c_write_u8(TPS65903X_CHIP_P1, 0xA9, val);
}

/* Support Getting imporant env from SPI Flash */
#ifdef CONFIG_SUPPORT_SF
static void setup_board_sf_env(void)
{
#if CONFIG_MAC_IN_QSPI
	uchar enetaddr[6];
	struct spi_flash *flash;
	u32 offset;
	u32 valid=0;

	flash = spi_flash_probe(CONFIG_SF_DEFAULT_BUS, CONFIG_SF_DEFAULT_CS, CONFIG_SF_DEFAULT_SPEED, CONFIG_SF_DEFAULT_MODE);
	if(flash)
	{
		puts("Read MAC from qspi.\n");
		offset = CONFIG_MAC_OFFSET;
		spi_flash_read(flash, offset, 6, enetaddr);
		spi_flash_free(flash);
		valid = is_valid_ethaddr(enetaddr);
	}
	if (valid)
		eth_setenv_enetaddr("ethaddr", enetaddr);
	else
		puts("Skipped ethaddr assignment due to invalid,using default!\n");

#endif
}
#endif /* CONFIG_SUPPORT_SF */

void vcores_update(void)
{
	*omap_vcores = &am57xx_volts;
}

void hw_data_init(void)
{
	*prcm = &dra7xx_prcm;
	*dplls_data = &dra7xx_dplls;
	*omap_vcores = &am57xx_volts;
	*ctrl = &dra7xx_ctrl;
}

int board_init(void)
{
	gpmc_init();
	gd->bd->bi_boot_params = (CONFIG_SYS_SDRAM_BASE + 0x100);

	return 0;
}

int board_late_init(void)
{
#ifdef CONFIG_SUPPORT_SF
	setup_board_sf_env();
#endif
	/*
	 * DEV_CTRL.DEV_ON = 1 please - else palmas switches off in 8 seconds
	 * This is the POWERHOLD-in-Low behavior.
	 */
	palmas_i2c_write_u8(TPS65903X_CHIP_P1, 0xA0, 0x1);

#ifndef CONFIG_SPL_BUILD
	board_set_boot_device();
#endif

#ifdef CONFIG_C8051_WATCHDOG
	c8051_watchdog_init();
#endif

	return 0;
}

void set_muxconf_regs(void)
{
	do_set_mux32((*ctrl)->control_padconf_core_base,
		     early_padconf, ARRAY_SIZE(early_padconf));
}

#ifdef CONFIG_IODELAY_RECALIBRATION
void recalibrate_iodelay(void)
{
	const struct pad_conf_entry *pconf;
	const struct iodelay_cfg_entry *iod;
	int pconf_sz, iod_sz;	

	pconf = core_padconf_array_am57xx;
	pconf_sz = ARRAY_SIZE(core_padconf_array_am57xx);
	iod = iodelay_cfg_array_am57xx;
	iod_sz = ARRAY_SIZE(iodelay_cfg_array_am57xx);

	__recalibrate_iodelay(pconf, pconf_sz, iod, iod_sz);
}
#endif

#if !defined(CONFIG_SPL_BUILD) && defined(CONFIG_GENERIC_MMC)
int board_mmc_init(bd_t *bis)
{
	omap_mmc_init(0, 0, 0, -1, -1);
	omap_mmc_init(1, 0, 0, -1, -1);
	return 0;
}
#endif

#ifdef CONFIG_OMAP_HSMMC
int platform_fixup_disable_uhs_mode(void)
{
	return omap_revision() == DRA752_ES1_1;
}
#endif

#if defined(CONFIG_SPL_BUILD) && defined(CONFIG_SPL_OS_BOOT)
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

#ifdef CONFIG_USB_DWC3
static struct dwc3_device usb_otg_ss1 = {
	.maximum_speed = USB_SPEED_SUPER,
	.base = DRA7_USB_OTG_SS1_BASE,
	.tx_fifo_resize = false,
	.index = 0,
};

static struct dwc3_omap_device usb_otg_ss1_glue = {
	.base = (void *)DRA7_USB_OTG_SS1_GLUE_BASE,
	.utmi_mode = DWC3_OMAP_UTMI_MODE_SW,
	.index = 0,
};

static struct ti_usb_phy_device usb_phy1_device = {
	.pll_ctrl_base = (void *)DRA7_USB3_PHY1_PLL_CTRL,
	.usb2_phy_power = (void *)DRA7_USB2_PHY1_POWER,
	.usb3_phy_power = (void *)DRA7_USB3_PHY1_POWER,
	.index = 0,
};

static struct dwc3_device usb_otg_ss2 = {
	.maximum_speed = USB_SPEED_HIGH,
	.base = DRA7_USB_OTG_SS2_BASE,
	.tx_fifo_resize = false,
	.index = 1,
};

static struct dwc3_omap_device usb_otg_ss2_glue = {
	.base = (void *)DRA7_USB_OTG_SS2_GLUE_BASE,
	.utmi_mode = DWC3_OMAP_UTMI_MODE_SW,
	.index = 1,
};

static struct ti_usb_phy_device usb_phy2_device = {
	.usb2_phy_power = (void *)DRA7_USB2_PHY2_POWER,
	.index = 1,
};

int usb_gadget_handle_interrupts(int index)
{
	u32 status;

	status = dwc3_omap_uboot_interrupt_status(index);
	if (status)
		dwc3_uboot_handle_interrupt(index);

	return 0;
}
#endif /* CONFIG_USB_DWC3 */

#if defined(CONFIG_USB_DWC3) || defined(CONFIG_USB_XHCI_OMAP)
int board_usb_init(int index, enum usb_init_type init)
{
	enable_usb_clocks(index);
	switch (index) {
	case 0:
		if (init == USB_INIT_DEVICE) {
			printf("port %d can't be used as device\n", index);
			disable_usb_clocks(index);
			return -EINVAL;
		}
		break;
	case 1:
		if (init == USB_INIT_DEVICE) {
#ifdef CONFIG_USB_DWC3
			usb_otg_ss2.dr_mode = USB_DR_MODE_PERIPHERAL;
			usb_otg_ss2_glue.vbus_id_status = OMAP_DWC3_VBUS_VALID;
			ti_usb_phy_uboot_init(&usb_phy2_device);
			dwc3_omap_uboot_init(&usb_otg_ss2_glue);
			dwc3_uboot_init(&usb_otg_ss2);
#endif
		} else {
			printf("port %d can't be used as host\n", index);
			disable_usb_clocks(index);
			return -EINVAL;
		}

		break;
	default:
		printf("Invalid Controller Index\n");
	}

	return 0;
}

int board_usb_cleanup(int index, enum usb_init_type init)
{
#ifdef CONFIG_USB_DWC3
	switch (index) {
	case 0:
	case 1:
		if (init == USB_INIT_DEVICE) {
			ti_usb_phy_uboot_exit(index);
			dwc3_uboot_exit(index);
			dwc3_omap_uboot_exit(index);
		}
		break;
	default:
		printf("Invalid Controller Index\n");
	}
#endif
	disable_usb_clocks(index);
	return 0;
}
#endif /* defined(CONFIG_USB_DWC3) || defined(CONFIG_USB_XHCI_OMAP) */

#ifdef CONFIG_DRIVER_TI_CPSW

/* Delay value to add to calibrated value */
#define RGMII0_TXCTL_DLY_VAL		((0x3 << 5) + 0x8)
#define RGMII0_TXD0_DLY_VAL		((0x3 << 5) + 0x8)
#define RGMII0_TXD1_DLY_VAL		((0x3 << 5) + 0x2)
#define RGMII0_TXD2_DLY_VAL		((0x4 << 5) + 0x0)
#define RGMII0_TXD3_DLY_VAL		((0x4 << 5) + 0x0)
#define VIN2A_D13_DLY_VAL		((0x3 << 5) + 0x8)
#define VIN2A_D17_DLY_VAL		((0x3 << 5) + 0x8)
#define VIN2A_D16_DLY_VAL		((0x3 << 5) + 0x2)
#define VIN2A_D15_DLY_VAL		((0x4 << 5) + 0x0)
#define VIN2A_D14_DLY_VAL		((0x4 << 5) + 0x0)

static void cpsw_control(int enabled)
{
	/* VTP can be added here */
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

int board_eth_init(bd_t *bis)
{
	int ret;
	uint32_t ctrl_val;

	//config to rgmii
	ctrl_val = readl((*ctrl)->control_core_control_io1) & (~0x33);
	ctrl_val |= 0x22;
	writel(ctrl_val, (*ctrl)->control_core_control_io1);

	ret = cpsw_register(&cpsw_data);
	if (ret < 0)
		printf("Error %d registering CPSW switch\n", ret);

	return ret;
}
#endif

#ifdef CONFIG_BOARD_EARLY_INIT_F
/* VTT regulator enable */
static inline void vtt_regulator_enable(void)
{
	if (omap_hw_init_context() == OMAP_INIT_CONTEXT_UBOOT_AFTER_SPL)
		return;

	gpio_request(GPIO_DDR_VTT_EN, "ddr_vtt_en");
	gpio_direction_output(GPIO_DDR_VTT_EN, 1);
}

int board_early_init_f(void)
{
	vtt_regulator_enable();
	return 0;
}
#endif

#ifdef CONFIG_SPL_LOAD_FIT
int board_fit_config_name_match(const char *name)
{
	if(!strcmp(name, CONFIG_DEFAULT_DEVICE_TREE))
		return 0;
}
#endif

#if defined(CONFIG_OF_LIBFDT) && defined(CONFIG_OF_BOARD_SETUP)
int ft_board_setup(void *blob, bd_t *bd)
{
	ft_cpu_setup(blob, bd);

	return 0;
}
#endif

#ifndef CONFIG_SPL_BUILD
static void  board_set_boot_device(void)
{
    int dev = (*(int *)CONFIG_SPL_PARAM_ADDR);

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
	
}
#endif
