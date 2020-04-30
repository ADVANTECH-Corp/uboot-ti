/*
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com
 *
 * Author: Felipe Balbi <balbi@ti.com>
 *
 * Based on board/ti/dra7xx/evm.c
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
#include <mmc.h>

#include "../common/board_detect.h"
#include "mux_data.h"

#ifdef CONFIG_DRIVER_TI_CPSW
#include <cpsw.h>
#endif

DECLARE_GLOBAL_DATA_PTR;

#define GPIO_ETH_LCD		GPIO_TO_PIN(2, 22)

/* Touch screen controller to identify the LCD */
#define OSD_TS_FT_BUS_ADDRESS	0
#define OSD_TS_FT_CHIP_ADDRESS	0x38
#define OSD_TS_FT_REG_ID	0xA3
/*
 * Touchscreen IDs for various OSD panels
 * Ref: http://www.osddisplays.com/TI/OSD101T2587-53TS_A.1.pdf
 */
/* Used on newer osd101t2587 Panels */
#define OSD_TS_FT_ID_5x46	0x54
/* Used on older osd101t2045 Panels */
#define OSD_TS_FT_ID_5606	0x08

#define SYSINFO_BOARD_NAME_MAX_LEN	45

#define TPS65903X_PRIMARY_SECONDARY_PAD2	0xFB
#define TPS65903X_PAD2_POWERHOLD_MASK		0x20

const struct omap_sysinfo sysinfo = {
	"Board: UNKNOWN(BeagleBoard X15?) REV UNKNOWN\n"
};

void emif_get_dmm_regs(const struct dmm_lisa_map_regs **dmm_lisa_regs)
{
		*dmm_lisa_regs = &am57xx_lisa_regs;
}

void emif_get_reg_dump(u32 emif_nr, const struct emif_regs **regs)
{
	switch (emif_nr) {
	case 1:
			*regs = &am572x_emif1_ddr3_532mhz_emif_regs;
		break;
	case 2:
			*regs = &am572x_emif2_ddr3_532mhz_emif_regs;
		break;
	}
}

void emif_get_ext_phy_ctrl_const_regs(u32 emif_nr, const u32 **regs, u32 *size)
{
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

int get_voltrail_opp(int rail_offset)
{
	int opp;

	switch (rail_offset) {
	case VOLT_MPU:
		opp = DRA7_MPU_OPP;
		break;
	case VOLT_CORE:
		opp = DRA7_CORE_OPP;
		break;
	case VOLT_GPU:
		opp = DRA7_GPU_OPP;
		break;
	case VOLT_EVE:
		opp = DRA7_DSPEVE_OPP;
		break;
	case VOLT_IVA:
		opp = DRA7_IVA_OPP;
		break;
	default:
		opp = OPP_NOM;
	}

	return opp;
}


#ifdef CONFIG_SPL_BUILD
/* No env to setup for SPL */
static inline void setup_board_eeprom_env(void) { }

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

/* Override function to read eeprom information */
void do_board_detect(void)
{
	int r = -ENODEV;

	r = i2c_set_bus_num(OSD_TS_FT_BUS_ADDRESS);
	if (r) {
		printf("%s: Failed to set bus address to %d: %d\n",
		       __func__, OSD_TS_FT_BUS_ADDRESS, r);
	}

	enable_extra_advboot_clock();
}

#else	/* CONFIG_SPL_BUILD */

/* Override function to read eeprom information: actual i2c read done by SPL*/
void do_board_detect(void)
{
	char *bname = NULL;

	bname = CONFIG_SYS_BOARD_NAME;

	if (bname)
		snprintf(sysinfo.board_string, SYSINFO_BOARD_NAME_MAX_LEN,
			 "Board: %s \n", bname);
}

static void setup_board_eeprom_env(void)
{

	env_set("board_name", CONFIG_SYS_BOARD_NAME);
	env_set("board_serial", CONFIG_SYS_CONFIG_NAME);
	
}

#endif	/* CONFIG_SPL_BUILD */

void vcores_init(void)
{
	*omap_vcores = &am57xx_volts;
}

void hw_data_init(void)
{
	*prcm = &dra7xx_prcm;
	if (is_dra72x())
		*dplls_data = &dra72x_dplls;
	else
		*dplls_data = &dra7xx_dplls;
	*ctrl = &dra7xx_ctrl;
}

int board_init(void)
{
	gpmc_init();
	gd->bd->bi_boot_params = (CONFIG_SYS_SDRAM_BASE + 0x100);

	return 0;
}

#if CONFIG_IS_ENABLED(DM_USB) && CONFIG_IS_ENABLED(OF_CONTROL)
static int device_okay(const char *path)
{
	int node;

	node = fdt_path_offset(gd->fdt_blob, path);
	if (node < 0)
		return 0;

	return fdtdec_get_is_enabled(gd->fdt_blob, node);
}
#endif

static void adv_board_set_ethaddr(void)
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
		eth_env_set_enetaddr("ethaddr", enetaddr);
	else
		puts("Skipped ethaddr assignment due to invalid,using default!\n");

#endif
}

int board_late_init(void)
{
	setup_board_eeprom_env();
	u8 val;

	/*
	 * DEV_CTRL.DEV_ON = 1 please - else palmas switches off in 8 seconds
	 * This is the POWERHOLD-in-Low behavior.
	 */
	palmas_i2c_write_u8(TPS65903X_CHIP_P1, 0xA0, 0x1);

	/*
	 * Default FIT boot on HS devices. Non FIT images are not allowed
	 * on HS devices.
	 */
	if (get_device_type() == HS_DEVICE)
		env_set("boot_fit", "1");

	/*
	 * Set the GPIO7 Pad to POWERHOLD. This has higher priority
	 * over DEV_CTRL.DEV_ON bit. This can be reset in case of
	 * PMIC Power off. So to be on the safer side set it back
	 * to POWERHOLD mode irrespective of the current state.
	 */
	palmas_i2c_read_u8(TPS65903X_CHIP_P1, TPS65903X_PRIMARY_SECONDARY_PAD2,
			   &val);
	val = val | TPS65903X_PAD2_POWERHOLD_MASK;
	palmas_i2c_write_u8(TPS65903X_CHIP_P1, TPS65903X_PRIMARY_SECONDARY_PAD2,
			    val);

	omap_die_id_serial();
	omap_set_fastboot_vars();

#if !defined(CONFIG_SPL_BUILD)
	adv_board_set_ethaddr();
#endif

#if CONFIG_IS_ENABLED(DM_USB) && CONFIG_IS_ENABLED(OF_CONTROL)
	if (device_okay("/ocp/omap_dwc3_1@48880000"))
		enable_usb_clocks(0);
	if (device_okay("/ocp/omap_dwc3_2@488c0000"))
		enable_usb_clocks(1);
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
	const struct iodelay_cfg_entry *iod, *delta_iod;
	int pconf_sz, iod_sz, delta_iod_sz = 0;
	int ret;

	pconf = core_padconf_array_am57xx;
	pconf_sz = ARRAY_SIZE(core_padconf_array_am57xx);
	iod = iodelay_cfg_array_am57xx;
	iod_sz = ARRAY_SIZE(iodelay_cfg_array_am57xx);

	/* Setup I/O isolation */
	ret = __recalibrate_iodelay_start();
	if (ret)
		goto err;

	/* Do the muxing here */
	do_set_mux32((*ctrl)->control_padconf_core_base, pconf, pconf_sz);

	/* Setup IOdelay configuration */
	ret = do_set_iodelay((*ctrl)->iodelay_config_base, iod, iod_sz);
	if (delta_iod_sz)
		ret = do_set_iodelay((*ctrl)->iodelay_config_base, delta_iod,
				     delta_iod_sz);

err:
	/* Closeup.. remove isolation */
	__recalibrate_iodelay_end(ret);
}
#endif

#if defined(CONFIG_MMC)
int board_mmc_init(bd_t *bis)
{
	omap_mmc_init(0, 0, 0, -1, -1);
	omap_mmc_init(1, 0, 0, -1, -1);
	return 0;
}

static const struct mmc_platform_fixups am57x_es1_1_mmc1_fixups = {
	.hw_rev = "rev11",
	.unsupported_caps = MMC_CAP(MMC_HS_200) |
			    MMC_CAP(UHS_SDR104),
	.max_freq = 96000000,
};

static const struct mmc_platform_fixups am57x_es1_1_mmc23_fixups = {
	.hw_rev = "rev11",
	.unsupported_caps = MMC_CAP(MMC_HS_200) |
			    MMC_CAP(UHS_SDR104) |
			    MMC_CAP(UHS_SDR50),
	.max_freq = 48000000,
};

const struct mmc_platform_fixups *platform_fixups_mmc(uint32_t addr)
{
	switch (omap_revision()) {
	case DRA752_ES1_0:
	case DRA752_ES1_1:
		if (addr == OMAP_HSMMC1_BASE)
			return &am57x_es1_1_mmc1_fixups;
		else
			return &am57x_es1_1_mmc23_fixups;
	default:
		return NULL;
	}
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
	env_load();
	if (env_get_yesno("boot_os") != 1)
		return 1;
#endif

	return 0;
}
#endif

#ifdef CONFIG_USB_DWC3
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
	{
		.slave_reg_ofs	= 0x208,
		.sliver_reg_ofs	= 0xd80,
		.phy_addr	= 1,
	},
	{
		.slave_reg_ofs	= 0x308,
		.sliver_reg_ofs	= 0xdc0,
		.phy_addr	= 2,
	},
};

static struct cpsw_platform_data cpsw_data = {
	.mdio_base		= CPSW_MDIO_BASE,
	.cpsw_base		= CPSW_BASE,
	.mdio_div		= 0xff,
	.channels		= 8,
	.cpdma_reg_ofs		= 0x800,
	.slaves			= 1,
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

static u64 mac_to_u64(u8 mac[6])
{
	int i;
	u64 addr = 0;

	for (i = 0; i < 6; i++) {
		addr <<= 8;
		addr |= mac[i];
	}

	return addr;
}

static void u64_to_mac(u64 addr, u8 mac[6])
{
	mac[5] = addr;
	mac[4] = addr >> 8;
	mac[3] = addr >> 16;
	mac[2] = addr >> 24;
	mac[1] = addr >> 32;
	mac[0] = addr >> 40;
}

int board_eth_init(bd_t *bis)
{
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

#if defined(CONFIG_OF_LIBFDT) && defined(CONFIG_OF_BOARD_SETUP)
int ft_board_setup(void *blob, bd_t *bd)
{
	ft_cpu_setup(blob, bd);

	return 0;
}
#endif

#ifdef CONFIG_SPL_LOAD_FIT
int board_fit_config_name_match(const char *name)
{

	if (!strcmp(name, CONFIG_SYS_CONFIG_NAME))
		return 0;

	return -1;
}
#endif

#ifdef CONFIG_TI_SECURE_DEVICE
void board_fit_image_post_process(void **p_image, size_t *p_size)
{
	secure_boot_verify_image(p_image, p_size);
}

void board_tee_image_process(ulong tee_image, size_t tee_size)
{
	secure_tee_install((u32)tee_image);
}

U_BOOT_FIT_LOADABLE_HANDLER(IH_TYPE_TEE, board_tee_image_process);
#endif

