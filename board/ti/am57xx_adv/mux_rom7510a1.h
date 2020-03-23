/*
 * Copyright (C) 2016 Advantech Co.,Ltd 
 *
 * Author: yanwei.cao <yanwei.cao@advantech.com.cn>
 *
 * Based on board/ti/am57xx_adv/mux_data.h
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

 
#ifndef _MUX_ROM7510A2_H_
#define _MUX_ROM7510A2_H_
/* GPIO 7_10 */
#define GPIO_DDR_VTT_EN 202

const struct pad_conf_entry early_padconf[] = {
	{UART2_CTSN, (M2 | PIN_INPUT_SLEW)},	/* uart2_ctsn.uart3_rxd */
	{UART2_RTSN, (M1 | PIN_INPUT_SLEW)},	/* uart2_rtsn.uart3_txd */
	{I2C1_SDA, (PIN_INPUT_PULLUP | M0)},	/* I2C1_SDA */
	{I2C1_SCL, (PIN_INPUT_PULLUP | M0)},	/* I2C1_SCL */
};


const struct pad_conf_entry core_padconf_array_am57xx[] = {
	/* VOUT3 */
	/*LCD*/
	{VOUT1_CLK, (M0 | PIN_OUTPUT)},		/* vout1_clk.vout1_clk */
	{VOUT1_DE, (M0 | PIN_OUTPUT)},		/* vout1_de.vout1_de */
	{VOUT1_FLD, (M14 | PIN_OUTPUT_PULLUP)},		/* vout1_fld.gpio4_21 */
	{VOUT1_HSYNC, (M0 | PIN_OUTPUT)},	/* vout1_hsync.vout1_hsync */
	{VOUT1_VSYNC, (M0 | PIN_OUTPUT)},	/* vout1_vsync.vout1_vsync */
	{VOUT1_D0, (M0 | PIN_OUTPUT)},		/* vout1_d0.vout1_d0 */
	{VOUT1_D1, (M0 | PIN_OUTPUT)},		/* vout1_d1.vout1_d1 */
	{VOUT1_D2, (M0 | PIN_OUTPUT)},		/* vout1_d2.vout1_d2 */
	{VOUT1_D3, (M0 | PIN_OUTPUT)},		/* vout1_d3.vout1_d3 */
	{VOUT1_D4, (M0 | PIN_OUTPUT)},		/* vout1_d4.vout1_d4 */
	{VOUT1_D5, (M0 | PIN_OUTPUT)},		/* vout1_d5.vout1_d5 */
	{VOUT1_D6, (M0 | PIN_OUTPUT)},		/* vout1_d6.vout1_d6 */
	{VOUT1_D7, (M0 | PIN_OUTPUT)},		/* vout1_d7.vout1_d7 */
	{VOUT1_D8, (M0 | PIN_OUTPUT)},		/* vout1_d8.vout1_d8 */
	{VOUT1_D9, (M0 | PIN_OUTPUT)},		/* vout1_d9.vout1_d9 */
	{VOUT1_D10, (M0 | PIN_OUTPUT)},		/* vout1_d10.vout1_d10 */
	{VOUT1_D11, (M0 | PIN_OUTPUT)},		/* vout1_d11.vout1_d11 */
	{VOUT1_D12, (M0 | PIN_OUTPUT)},		/* vout1_d12.vout1_d12 */
	{VOUT1_D13, (M0 | PIN_OUTPUT)},		/* vout1_d13.vout1_d13 */
	{VOUT1_D14, (M0 | PIN_OUTPUT)},		/* vout1_d14.vout1_d14 */
	{VOUT1_D15, (M0 | PIN_OUTPUT)},		/* vout1_d15.vout1_d15 */
	{VOUT1_D16, (M0 | PIN_OUTPUT)},		/* vout1_d16.vout1_d16 */
	{VOUT1_D17, (M0 | PIN_OUTPUT)},		/* vout1_d17.vout1_d17 */
	{VOUT1_D18, (M0 | PIN_OUTPUT)},		/* vout1_d18.vout1_d18 */
	{VOUT1_D19, (M0 | PIN_OUTPUT)},		/* vout1_d19.vout1_d19 */
	{VOUT1_D20, (M0 | PIN_OUTPUT)},		/* vout1_d20.vout1_d20 */
	{VOUT1_D21, (M0 | PIN_OUTPUT)},		/* vout1_d21.vout1_d21 */
	{VOUT1_D22, (M0 | PIN_OUTPUT)},		/* vout1_d22.vout1_d22 */
	{VOUT1_D23, (M0 | PIN_OUTPUT)},		/* vout1_d23.vout1_d23 */
	{UART1_RXD, (M14 | PIN_OUTPUT_PULLUP)},	/* uart1_rxd.gpio7_22 */
	{VIN2A_HSYNC0, (M14 | PIN_OUTPUT_PULLUP)}, /* vin2a_hsync0.gpio3_31 */
	{VIN2A_VSYNC0, (M10 | PIN_INPUT_PULLDOWN)},	/* vin2a_vsync0.etrpwm1A */
	{GPMC_CS0, (M14 | PIN_OUTPUT_PULLDOWN)},	/* gpmc_cs0.gpio2_19 */
	/*LVDS*/
	/*HDMI*/
	{I2C2_SDA, (M1 | PIN_INPUT)},		/* i2c2_sda.hdmi1_ddc_scl */
	{I2C2_SCL, (M1 | PIN_INPUT)},		/* i2c2_scl.hdmi1_ddc_sda */
	{DCAN1_RX, (M6 | PIN_INPUT_PULLUP | SLEWCONTROL)},	/* dcan1_rx.hdmi1_cec */
	{DCAN1_TX, (M14 | PIN_INPUT_PULLUP)},	/* dcan1_tx.hdmi1_hpd */
	/*timer_in*/
	{GPMC_CLK, (M14 | PIN_INPUT_PULLUP)},					/* gpmc_clk.gpio2_22 */
	{VIN2A_D2, (M14 | PIN_INPUT_PULLDOWN)},		/* vin2a_d2.gpio4_3 */
	/*Watchdog*/
	{VIN1A_FLD0, (M14 | PIN_OUTPUT)}, 	/* vin1a_fld0.gpio3_1 */
	{VIN1A_VSYNC0, (M14 | PIN_OUTPUT_PULLDOWN)}, 	/* vin1a_vsync0.gpio3_3 */
	/*mcasp*/
	{XREF_CLK0, (M9 | PIN_INPUT_PULLDOWN)},	/* xref_clk0.xref_clk0 */
	{MCASP3_ACLKX, (M0 | PIN_INPUT_PULLDOWN)},	/* mcasp3_aclkx.mcasp3_aclkx */
	{MCASP3_FSX, (M0 | PIN_INPUT_PULLDOWN)},	/* mcasp3_fsx.mcasp3_fsx */
	{MCASP3_AXR0, (M0 | PIN_INPUT_PULLDOWN)},	/* mcasp3_axr0.mcasp3_axr0 */
	{MCASP3_AXR1, (M0 | PIN_INPUT_PULLDOWN)},	/* mcasp3_axr1.mcasp3_axr1 */
	/*SDIO*/
	{VIN1B_CLK1, (M14 | PIN_OUTPUT_PULLDOWN)},	/* vin1b_clk1.gpio2_31 */
	//{UART1_TXD, (M14 | PIN_OUTPUT_PULLUP)},	/* uart1_txd.gpio7_23 */
	/* QSPI1 */
	{GPMC_A13, (M1 | PIN_INPUT)},	/* gpmc_a13.qspi1_rtclk */
	{GPMC_A14, (M1 | PIN_INPUT)},	/* gpmc_a14.qspi1_d3 */
	{GPMC_A15, (M1 | PIN_INPUT)},	/* gpmc_a15.qspi1_d2 */
	{GPMC_A16, (M1 | PIN_INPUT)},	/* gpmc_a16.qspi1_d0 */
	{GPMC_A17, (M1 | PIN_INPUT)},	/* gpmc_a17.qspi1_d1 */
	{GPMC_A18, (M1 | PIN_OUTPUT)},	/* gpmc_a18.qspi1_sclk */
	{GPMC_CS2, (M1 | PIN_INPUT_PULLUP)},	/* gpmc_cs2.qspi1_cs0 */
	/* SPI */
	{MCASP1_AXR8, (M3 | PIN_INPUT_PULLDOWN)},	/* mcasp1_axr8.spi3_sclk */
	{MCASP1_AXR9, (M3 | PIN_INPUT_PULLDOWN)},	/* mcasp1_axr9.spi3_d1 */
	{MCASP1_AXR10, (M3 | PIN_INPUT_PULLDOWN)},	/* mcasp1_axr10.spi3_d0 */
	{MCASP1_AXR11, (M3 | PIN_INPUT_PULLUP)},	/* mcasp1_axr11.spi3_cs0 */
	{MCASP1_AXR12, (M3 | PIN_INPUT_PULLUP)},	/* mcasp1_axr12.spi3_cs1 */
	/* MMC2 */
	{GPMC_A19, (M1 | PIN_INPUT_PULLUP)},	/* gpmc_a19.mmc2_dat4 */
	{GPMC_A20, (M1 | PIN_INPUT_PULLUP)},	/* gpmc_a20.mmc2_dat5 */
	{GPMC_A21, (M1 | PIN_INPUT_PULLUP)},	/* gpmc_a21.mmc2_dat6 */
	{GPMC_A22, (M1 | PIN_INPUT_PULLUP)},	/* gpmc_a22.mmc2_dat7 */
	{GPMC_A23, (M1 | PIN_INPUT_PULLUP)},	/* gpmc_a23.mmc2_clk */
	{GPMC_A24, (M1 | PIN_INPUT_PULLUP)},	/* gpmc_a24.mmc2_dat0 */
	{GPMC_A25, (M1 | PIN_INPUT_PULLUP)},	/* gpmc_a25.mmc2_dat1 */
	{GPMC_A26, (M1 | PIN_INPUT_PULLUP)},	/* gpmc_a26.mmc2_dat2 */
	{GPMC_A27, (M1 | PIN_INPUT_PULLUP)},	/* gpmc_a27.mmc2_dat3 */
	{GPMC_CS1, (M1 | PIN_INPUT_PULLUP)},	/* gpmc_cs1.mmc2_cmd */
	/* VIN1A */
	//{VIN1A_D0, (M0 | PIN_INPUT)},
	/* VIN3A */
	/* PHY& RGMII */
	{VIN2A_D10, (M3 | PIN_OUTPUT_PULLDOWN)},	/* vin2a_d10.mdio_mclk */
	{VIN2A_D11, (M3 | PIN_INPUT_PULLDOWN)},	/* vin2a_d11.mdio_d */
	{VIN2A_D12, (M3 | PIN_INPUT_PULLDOWN | MANUAL_MODE)},	/* vin2a_d12.rgmii1_txc */
	{VIN2A_D13, (M3 | PIN_INPUT_PULLDOWN | MANUAL_MODE)},	/* vin2a_d13.rgmii1_txctl */
	{VIN2A_D14, (M3 | PIN_INPUT_PULLDOWN | MANUAL_MODE)},	/* vin2a_d14.rgmii1_txd3 */
	{VIN2A_D15, (M3 | PIN_INPUT_PULLDOWN | MANUAL_MODE)},	/* vin2a_d15.rgmii1_txd2 */
	{VIN2A_D16, (M3 | PIN_INPUT_PULLDOWN | MANUAL_MODE)},	/* vin2a_d16.rgmii1_txd1 */
	{VIN2A_D17, (M3 | PIN_INPUT_PULLDOWN | MANUAL_MODE)},	/* vin2a_d17.rgmii1_txd0 */
	{VIN2A_D18, (M3 | PIN_INPUT_PULLDOWN | MANUAL_MODE)},	/* vin2a_d18.rgmii1_rxc */
	{VIN2A_D19, (M3 | PIN_INPUT_PULLUP | MANUAL_MODE)},	/* vin2a_d19.rgmii1_rxctl */
	{VIN2A_D20, (M3 | PIN_INPUT_PULLUP | MANUAL_MODE)},	/* vin2a_d20.rgmii1_rxd3 */
	{VIN2A_D21, (M3 | PIN_INPUT_PULLUP | MANUAL_MODE)},	/* vin2a_d21.rgmii1_rxd2 */
	{VIN2A_D22, (M3 | PIN_INPUT_PULLUP | MANUAL_MODE)},	/* vin2a_d22.rgmii1_rxd1 */
	{VIN2A_D23, (M3 | PIN_INPUT_PULLUP | MANUAL_MODE)},	/* vin2a_d23.rgmii1_rxd0 */
	/* USB */
	{USB1_DRVVBUS, (M0 | PIN_INPUT_SLEW)},	/* usb1_drvvbus.usb1_drvvbus */
	{USB2_DRVVBUS, (M0 | PIN_INPUT_SLEW)},	/* usb2_drvvbus.usb2_drvvbus */
	{SPI2_CS0, (M14 | PIN_INPUT_PULLUP)},	/* spi2_cs0.gpio7_17 */
	{GPMC_A12, (M14 | PIN_INPUT)},	/* gpmc_a12.spi4_cs1 */
	/* I2C */
	{MCASP1_ACLKX, (M10 | PIN_INPUT_PULLUP)},	/* mcasp1_aclkx.i2c3_sda */
	{MCASP1_FSX, (M10 | PIN_INPUT_PULLUP)},		/* mcasp1_fsx.i2c3_scl */
	{MCASP1_ACLKR, (M10 | PIN_INPUT_PULLUP)},	/* mcasp1_aclkr.i2c4_sda */
	{MCASP1_FSR, (M10 | PIN_INPUT_PULLUP)},		/* mcasp1_fsr.i2c4_scl */
	{MCASP1_AXR0, (M10 | PIN_INPUT_PULLUP)},	/* mcasp1_axr0.i2c5_sda */
	{MCASP1_AXR1, (M10 | PIN_INPUT_PULLUP)},	/* mcasp1_axr1.i2c5_scl */
	{I2C1_SDA, (M0 | PIN_INPUT_PULLUP)},		/* i2c1_sda.i2c1_sda */
	{I2C1_SCL, (M0 | PIN_INPUT_PULLUP)},		/* i2c1_scl.i2c1_scl */
	/* UART */
	{RGMII0_TXD0, (M8 | PIN_OUTPUT_PULLDOWN)},		/* rgmii0_txd0.uart4_rtsn */
	{RGMII0_TXD1, (M8 | PIN_INPUT)},		/* rgmii0_txd1.uart4_ctsn */
	{GPMC_A0, (M8 | PIN_INPUT)},	/* gpmc_a0.uart5_rxd */
	{GPMC_A1, (M8 | PIN_OUTPUT)},	/* gpmc_a1.uart5_txd */
	{GPMC_A2, (M8 | PIN_INPUT)},	/* gpmc_a2.uart5_ctsn */
	{GPMC_A3, (M8 | PIN_OUTPUT_PULLDOWN)},	/* gpmc_a3.uart5_rtsn */
	{MCASP4_AXR0, (M4 | PIN_INPUT)},	/* mcasp4_axr0.uart4_rxd */
	{MCASP4_AXR1, (M4 | PIN_OUTPUT)},	/* mcasp4_axr1.uart4_txd */
	/* MMC1 */
	{MMC1_CLK, (M0 | PIN_INPUT_PULLUP)},	/* mmc1_clk.mmc1_clk */
	{MMC1_CMD, (M0 | PIN_INPUT_PULLUP)},	/* mmc1_cmd.mmc1_cmd */
	{MMC1_DAT0, (M0 | PIN_INPUT_PULLUP)},	/* mmc1_dat0.mmc1_dat0 */
	{MMC1_DAT1, (M0 | PIN_INPUT_PULLUP)},	/* mmc1_dat1.mmc1_dat1 */
	{MMC1_DAT2, (M0 | PIN_INPUT_PULLUP)},	/* mmc1_dat2.mmc1_dat2 */
	{MMC1_DAT3, (M0 | PIN_INPUT_PULLUP)},	/* mmc1_dat3.mmc1_dat3 */
	/*{MMC1_SDWP, (M0 | PIN_INPUT_PULLUP)},*/	/* mmc1_sdwp.mmc1_sdwp */
	{MMC1_SDCD, (M14 | PIN_INPUT_PULLUP | SLEWCONTROL)},	/* mmc1_sdcd.gpio6_27 */
	{MMC1_SDWP, (M14 | PIN_INPUT | SLEWCONTROL)},	/* mmc1_sdwp.gpio6_28 */
	/* PWM */
	{GPIO6_10, (M10 | PIN_INPUT_PULLDOWN)},	/* gpio6_10.ehrpwm2A */
	{GPIO6_11, (M10 | PIN_INPUT_PULLDOWN)},	/* gpio6_11.ehrpwm2B */
	/* CAN */
	{GPIO6_14, (M2  | PIN_INPUT_SLEW)},	/* gpio6_14.dcan2_tx */
	{GPIO6_15, (M2 | PIN_INPUT_SLEW)},	/* gpio6_15.dcan2_rx */
	/* SATA */
	{SPI1_CS1, (M14 | PIN_OUTPUT_PULLDOWN)},		/* spi1_cs1.gpio7_11 */
	/* Weakup*/
	{WAKEUP0, (M0 | PULL_UP)},	/* Wakeup0.Wakeup0 */
	{WAKEUP1, (M0)},			/* Wakeup1.Wakeup1 */
	{WAKEUP2, (M14 | PIN_INPUT)},			/* Wakeup2.gpio1_2 */
	{WAKEUP3, (M0 | PULL_UP)},	/* Wakeup3.Wakeup3 */
	{ON_OFF, (M0 | PIN_OUTPUT_PULLUP)},		/* on_off.on_off */
	{RTC_PORZ, (M0 | PIN_OUTPUT_PULLDOWN)},	/* rtc_porz.rtc_porz */
	{RTCK, (M0 | PIN_INPUT_PULLDOWN)},		/* rtck.rtck */
	/* PRU GPIO */
	/* GPIO */
	{VIN1A_D0, (M14 | PIN_INPUT_PULLDOWN)},
	{UART2_RXD, (M14 | PIN_INPUT_PULLDOWN)},	/* uart2_rxd.gpio7_26 */
	{UART2_TXD, (M14 | PIN_INPUT_PULLDOWN)},	/* uart2_txd..gpio7_27 */
	{VIN1A_D1, (M14 | PIN_INPUT_PULLDOWN)}, 	/* vin1a_d1.gpio3_5 */
	{VIN1A_D13, (M14 | PIN_INPUT_PULLDOWN)},	/* vin1a_d13.gpio3_17*/
	{VIN1A_D15, (M14 | PIN_INPUT_PULLDOWN)},	/* vin1a_d15.gpio3_19 */
	{VIN1A_D16, (M14 | PIN_INPUT_PULLDOWN)},	/* vin1a_d16.gpio3_20*/
	{VIN1A_D17, (M14 | PIN_INPUT_PULLDOWN)},	/* vin1a_d17.gpio3_21 */
	{VIN1A_D18, (M14 | PIN_INPUT_PULLDOWN)},	/* vin1a_d18.gpio3_22 */
	{VIN1A_D19, (M14 | PIN_INPUT_PULLDOWN)},	/* vin1a_d19.gpio3_23*/
	{VIN1A_D23, (M14 | PIN_INPUT_PULLDOWN)},	/* vin1a_d23.gpio3_27 */
	{VIN2A_D0, (M14 | PIN_INPUT_PULLDOWN)},	/* vin2a_d0.gpio4_1 */
	{VIN2A_D1, (M14 | PIN_INPUT_PULLUP)},	/* vin2a_d1.gpio4_2 */
	{VIN2A_D3, (M14 | PIN_INPUT_PULLDOWN)},	/* vin2a_d3.gpio4_4 */
	{VIN2A_D4, (M14 | PIN_INPUT_PULLDOWN)},	/* vin2a_d4.gpio4_5 */
	{UART1_CTSN, (M14 | PIN_INPUT_PULLDOWN)},	/* uart1_ctsn.gpio7_24 */
	{UART1_RTSN, (M14 | PIN_INPUT_PULLDOWN)},	/* uart1_rtsn.gpio7_25 */
	{GPMC_ADVN_ALE, (M14 |  PIN_OUTPUT_PULLUP)},	/* gpmc_advn_ale.gpio2_23 */
	{GPMC_OEN_REN, (M14 | PIN_INPUT_PULLDOWN)},		/* gpmc_oen_ren.gpio2_24 */
	{GPMC_WEN, (M14 | PIN_INPUT_PULLDOWN)},			/* gpmc_wen.gpio2_25 */
	{GPMC_BEN1, (M14 | PIN_INPUT_PULLDOWN)},		/* gpmc_ben1.gpio2_27 */
	{MCASP1_AXR4, (M14 | PIN_INPUT_PULLDOWN)},	/* mcasp1_axr4.gpio5_6 */
	{MCASP1_AXR6, (M14 | PIN_INPUT_PULLDOWN)},	/* mcasp1_axr6.gpio5_8 */
	{MCASP2_AXR4, (M14 | PIN_INPUT_PULLDOWN)},	/* mcasp2_axr4.gpio1_4 */
	{MCASP2_AXR5, (M14 | PIN_OUTPUT_PULLDOWN)},	/* mcasp2_axr5.gpio6_7 */
	/*PWR*/
	{MDIO_D, (M14 |  PIN_OUTPUT_PULLUP)},	/* mdio_d.gpio5_16 */
	{MDIO_MCLK, (M14 |  PIN_INPUT_PULLUP)},	/* mdio_mclk.gpio5_15 */
	{VIN1A_CLK0, (M14 | PIN_INPUT_PULLDOWN)},  /* vin1a_clk0.gpio2_30 */
	{SPI1_CS0, (M14 | PIN_OUTPUT_PULLUP)},	/* spi1_cs0.gpio7_10 */
#ifdef CONFIG_TEST_AS_GPIO
	{VIN1A_D0, (M14 | PIN_INPUT)},
	/*  PWM as gpio function */
	{GPIO6_10, (M0 | PIN_INPUT)},
	{GPIO6_11, (M0 | PIN_INPUT)},
	/* CLKOUT as gpio */
	/* CLKIN as gpio */
	/* CN7*/
#endif
};

#ifdef CONFIG_IODELAY_RECALIBRATION
const struct iodelay_cfg_entry iodelay_cfg_array_am57xx[] = {
	{0x0A70, 1551, 115},	/* CFG_VIN2A_D12_OUT */
	{0x0A7C, 816, 0},	/* CFG_VIN2A_D13_OUT */
	{0x0A88, 876, 0},	/* CFG_VIN2A_D14_OUT */
	{0x0A94, 312, 0},	/* CFG_VIN2A_D15_OUT */
	{0x0AA0, 58, 0},	/* CFG_VIN2A_D16_OUT */
	{0x0AAC, 0, 0},		/* CFG_VIN2A_D17_OUT */
	{0x0AB0, 702, 0},	/* CFG_VIN2A_D18_IN */
	{0x0ABC, 136, 976},	/* CFG_VIN2A_D19_IN */
	{0x0AD4, 210, 1357},	/* CFG_VIN2A_D20_IN */
	{0x0AE0, 189, 1462},	/* CFG_VIN2A_D21_IN */
	{0x0AEC, 232, 1278},	/* CFG_VIN2A_D22_IN */
	{0x0AF8, 0, 1397},	/* CFG_VIN2A_D23_IN */
};
#endif


static const struct dmm_lisa_map_regs am57xx_lisa_regs = {
	.dmm_lisa_map_0 = 0x00000000,					
	.dmm_lisa_map_1 = 0x00000000,					
	.dmm_lisa_map_2 = 0x80740300,					
	.dmm_lisa_map_3 = 0xFF020100,					
	.is_ma_present = 0x1	
};

static const struct emif_regs am572x_emif1_ddr3_532mhz_emif_regs = {
    .sdram_config_init = 0x61851B32,					
    .sdram_config = 0x61851B32,					
    .sdram_config2 = 0x00000000,					
    .ref_ctrl = 0x000040F1,					
    .ref_ctrl_final = 0x00001035,					
    .sdram_tim1 = 0xCCCF36B3,					
    .sdram_tim2 = 0x308F7FDA,					
    .sdram_tim3 = 0x407F88A8,					
    .read_idle_ctrl = 0x00050000,					
    .zq_config = 0x5007190B,					
    .temp_alert_config = 0x00000000,					
    .emif_rd_wr_lvl_rmp_ctl = 0x80000000,					
    .emif_rd_wr_lvl_ctl = 0x00000000,					
    .emif_ddr_phy_ctlr_1_init = 0x0024400B,					
    .emif_ddr_phy_ctlr_1 = 0x0E24400B,					
    .emif_rd_wr_exec_thresh = 0x00000305		
};

/* Ext phy ctrl regs 1-35 */
static const u32 am572x_emif1_ddr3_ext_phy_ctrl_const_regs[] = {
    0x04040100,	// EMIF1_EXT_PHY_CTRL_1				
    0x006B0090,	// EMIF1_EXT_PHY_CTRL_2				
    0x006B008B,	// EMIF1_EXT_PHY_CTRL_3				
    0x006B0091,	// EMIF1_EXT_PHY_CTRL_4				
    0x006B008E,	// EMIF1_EXT_PHY_CTRL_5				
    0x006B006B,	// EMIF1_EXT_PHY_CTRL_6				
    0x00320032,	// EMIF1_EXT_PHY_CTRL_7				
    0x00320032,	// EMIF1_EXT_PHY_CTRL_8				
    0x00320032,	// EMIF1_EXT_PHY_CTRL_9				
    0x00320032,	// EMIF1_EXT_PHY_CTRL_10				
    0x00320032,	// EMIF1_EXT_PHY_CTRL_11				
    0x00600066,	// EMIF1_EXT_PHY_CTRL_12				
    0x0060006B,	// EMIF1_EXT_PHY_CTRL_13				
    0x00600075,	// EMIF1_EXT_PHY_CTRL_14				
    0x00600078,	// EMIF1_EXT_PHY_CTRL_15				
    0x00600060,	// EMIF1_EXT_PHY_CTRL_16				
    0x00400046,	// EMIF1_EXT_PHY_CTRL_17				
    0x0040004B,	// EMIF1_EXT_PHY_CTRL_18				
    0x00400055,	// EMIF1_EXT_PHY_CTRL_19				
    0x00400058,	// EMIF1_EXT_PHY_CTRL_20				
    0x00400040,	// EMIF1_EXT_PHY_CTRL_21				
    0x00800080,	// EMIF1_EXT_PHY_CTRL_22				
    0x00800080,	// EMIF1_EXT_PHY_CTRL_23				
    0x40010080,	// EMIF1_EXT_PHY_CTRL_24				
    0x08102040,	// EMIF1_EXT_PHY_CTRL_25				
    0x005B0080,	// EMIF1_EXT_PHY_CTRL_26				
    0x005B007B,	// EMIF1_EXT_PHY_CTRL_27				
    0x005B0081,	// EMIF1_EXT_PHY_CTRL_28				
    0x005B007E,	// EMIF1_EXT_PHY_CTRL_29				
    0x005B005B,	// EMIF1_EXT_PHY_CTRL_30				
    0x00300036,	// EMIF1_EXT_PHY_CTRL_31				
    0x0030003B,	// EMIF1_EXT_PHY_CTRL_32				
    0x00300045,	// EMIF1_EXT_PHY_CTRL_33				
    0x00300048,	// EMIF1_EXT_PHY_CTRL_34				
    0x00300030,	// EMIF1_EXT_PHY_CTRL_35				
    0x00000077	// EMIF1_EXT_PHY_CTRL_36
};

static const struct emif_regs am572x_emif2_ddr3_532mhz_emif_regs = {
    .sdram_config_init = 0x61851B32,					
    .sdram_config = 0x61851B32,					
    .sdram_config2 = 0x00000000,					
    .ref_ctrl = 0x000040F1,					
    .ref_ctrl_final = 0x00001035,					
    .sdram_tim1 = 0xCCCF36B3,					
    .sdram_tim2 = 0x308F7FDA,					
    .sdram_tim3 = 0x407F88A8,					
    .read_idle_ctrl = 0x00050000,					
    .zq_config = 0x5007190B,					
    .temp_alert_config = 0x00000000,					
    .emif_rd_wr_lvl_rmp_ctl = 0x80000000,					
    .emif_rd_wr_lvl_ctl = 0x00000000,					
    .emif_ddr_phy_ctlr_1_init = 0x0024400B,					
    .emif_ddr_phy_ctlr_1 = 0x0E24400B,					
    .emif_rd_wr_exec_thresh = 0x00000305	
};

static const u32 am572x_emif2_ddr3_ext_phy_ctrl_const_regs[] = {
    0x04040100,	// EMIF2_EXT_PHY_CTRL_1				
    0x006B0083,	// EMIF2_EXT_PHY_CTRL_2				
    0x006B0082,	// EMIF2_EXT_PHY_CTRL_3				
    0x006B0091,	// EMIF2_EXT_PHY_CTRL_4				
    0x006B0090,	// EMIF2_EXT_PHY_CTRL_5				
    0x006B006B,	// EMIF2_EXT_PHY_CTRL_6				
    0x00320032,	// EMIF2_EXT_PHY_CTRL_7				
    0x00320032,	// EMIF2_EXT_PHY_CTRL_8				
    0x00320032,	// EMIF2_EXT_PHY_CTRL_9				
    0x00320032,	// EMIF2_EXT_PHY_CTRL_10				
    0x00320032,	// EMIF2_EXT_PHY_CTRL_11				
    0x00600068,	// EMIF2_EXT_PHY_CTRL_12				
    0x00600068,	// EMIF2_EXT_PHY_CTRL_13				
    0x00600069,	// EMIF2_EXT_PHY_CTRL_14				
    0x0060006B,	// EMIF2_EXT_PHY_CTRL_15				
    0x00600060,	// EMIF2_EXT_PHY_CTRL_16				
    0x00400048,	// EMIF2_EXT_PHY_CTRL_17				
    0x00400048,	// EMIF2_EXT_PHY_CTRL_18				
    0x00400049,	// EMIF2_EXT_PHY_CTRL_19				
    0x0040004B,	// EMIF2_EXT_PHY_CTRL_20				
    0x00400040,	// EMIF2_EXT_PHY_CTRL_21				
    0x00800080,	// EMIF2_EXT_PHY_CTRL_22				
    0x00800080,	// EMIF2_EXT_PHY_CTRL_23				
    0x40010080,	// EMIF2_EXT_PHY_CTRL_24				
    0x08102040,	// EMIF2_EXT_PHY_CTRL_25				
    0x005B0073,	// EMIF2_EXT_PHY_CTRL_26				
    0x005B0072,	// EMIF2_EXT_PHY_CTRL_27				
    0x005B0081,	// EMIF2_EXT_PHY_CTRL_28				
    0x005B0080,	// EMIF2_EXT_PHY_CTRL_29				
    0x005B005B,	// EMIF2_EXT_PHY_CTRL_30				
    0x00300038,	// EMIF2_EXT_PHY_CTRL_31				
    0x00300038,	// EMIF2_EXT_PHY_CTRL_32				
    0x00300039,	// EMIF2_EXT_PHY_CTRL_33				
    0x0030003B,	// EMIF2_EXT_PHY_CTRL_34				
    0x00300030,	// EMIF2_EXT_PHY_CTRL_35				
    0x00000077	// EMIF2_EXT_PHY_CTRL_36				
};


struct vcores_data am57xx_volts = {
	.mpu.value[OPP_NOM]	= VDD_MPU_DRA7_NOM,
	.mpu.efuse.reg[OPP_NOM]	= STD_FUSE_OPP_VMIN_MPU_NOM,
	.mpu.efuse.reg_bits     = DRA752_EFUSE_REGBITS,
	.mpu.addr		= TPS659038_REG_ADDR_SMPS12,
	.mpu.pmic		= &tps659038,
	.mpu.abb_tx_done_mask	= OMAP_ABB_MPU_TXDONE_MASK,

	.eve.value[OPP_NOM]	= VDD_EVE_DRA7_NOM,
	.eve.value[OPP_OD]	= VDD_EVE_DRA7_OD,
	.eve.value[OPP_HIGH]	= VDD_EVE_DRA7_HIGH,
	.eve.efuse.reg[OPP_NOM]	= STD_FUSE_OPP_VMIN_DSPEVE_NOM,
	.eve.efuse.reg[OPP_OD]	= STD_FUSE_OPP_VMIN_DSPEVE_OD,
	.eve.efuse.reg[OPP_HIGH]	= STD_FUSE_OPP_VMIN_DSPEVE_HIGH,
	.eve.efuse.reg_bits	= DRA752_EFUSE_REGBITS,
	.eve.addr		= TPS659038_REG_ADDR_SMPS45,
	.eve.pmic		= &tps659038,
	.eve.abb_tx_done_mask	= OMAP_ABB_EVE_TXDONE_MASK,

	.gpu.value[OPP_NOM]	= VDD_GPU_DRA7_NOM,
	.gpu.value[OPP_OD]	= VDD_GPU_DRA7_OD,
	.gpu.value[OPP_HIGH]	= VDD_GPU_DRA7_HIGH,
	.gpu.efuse.reg[OPP_NOM]	= STD_FUSE_OPP_VMIN_GPU_NOM,
	.gpu.efuse.reg[OPP_OD]	= STD_FUSE_OPP_VMIN_GPU_OD,
	.gpu.efuse.reg[OPP_HIGH]	= STD_FUSE_OPP_VMIN_GPU_HIGH,
	.gpu.efuse.reg_bits	= DRA752_EFUSE_REGBITS,
	.gpu.addr		= TPS659038_REG_ADDR_SMPS45,
	.gpu.pmic		= &tps659038,
	.gpu.abb_tx_done_mask	= OMAP_ABB_GPU_TXDONE_MASK,

	.core.value[OPP_NOM]	= VDD_CORE_DRA7_NOM,
	.core.efuse.reg[OPP_NOM]	= STD_FUSE_OPP_VMIN_CORE_NOM,
	.core.efuse.reg_bits	= DRA752_EFUSE_REGBITS,
	.core.addr		= TPS659038_REG_ADDR_SMPS6,
	.core.pmic		= &tps659038,

	.iva.value[OPP_NOM]	= VDD_IVA_DRA7_NOM,
	.iva.value[OPP_OD]	= VDD_IVA_DRA7_OD,
	.iva.value[OPP_HIGH]	= VDD_IVA_DRA7_HIGH,
	.iva.efuse.reg[OPP_NOM]	= STD_FUSE_OPP_VMIN_IVA_NOM,
	.iva.efuse.reg[OPP_OD]	= STD_FUSE_OPP_VMIN_IVA_OD,
	.iva.efuse.reg[OPP_HIGH]	= STD_FUSE_OPP_VMIN_IVA_HIGH,
	.iva.efuse.reg_bits	= DRA752_EFUSE_REGBITS,
	.iva.addr		= TPS659038_REG_ADDR_SMPS45,
	.iva.pmic		= &tps659038,
	.iva.abb_tx_done_mask	= OMAP_ABB_IVA_TXDONE_MASK,
};

#endif /* _MUX_ROM7510A2_H_ */

