/*
 * Copyright (C) 2016 Advantech Co.,Ltd 
 *
 * Author: yanwei.cao <yanwei.cao@advantech.com.cn>
 *
 * Based on board/ti/advantech/mux_data.h
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef _MUX_GF33B1_H_
#define _MUX_GF33B1_H_

/* GPIO 7_11 */
#define GPIO_DDR_VTT_EN 203

const struct pad_conf_entry early_padconf[] = {
	{UART2_CTSN, (M2 | PIN_INPUT_SLEW)},	/* uart2_ctsn.uart3_rxd */
	{UART2_RTSN, (M1 | PIN_INPUT_SLEW)},	/* uart2_rtsn.uart3_txd */
	{I2C2_SDA, (PIN_INPUT_PULLUP | M0)},	/* I2C2_SDA */
	{I2C2_SCL, (PIN_INPUT_PULLUP | M0)},	/* I2C2_SCL */
	{I2C1_SDA, (PIN_INPUT_PULLUP | M0)},	/* I2C1_SDA */
	{I2C1_SCL, (PIN_INPUT_PULLUP | M0)},	/* I2C1_SCL */
};

const struct pad_conf_entry core_padconf_array_am57xx[] = {
	/* VOUT3 */
	{GPMC_AD0, (M3 | PIN_OUTPUT)},	/* gpmc_ad0.vout3_d0 */
	{GPMC_AD1, (M3 | PIN_OUTPUT)},	/* gpmc_ad1.vout3_d1 */
	{GPMC_AD2, (M3 | PIN_OUTPUT)},	/* gpmc_ad2.vout3_d2 */
	{GPMC_AD3, (M3 | PIN_OUTPUT)},	/* gpmc_ad3.vout3_d3 */
	{GPMC_AD4, (M3 | PIN_OUTPUT)},	/* gpmc_ad4.vout3_d4 */
	{GPMC_AD5, (M3 | PIN_OUTPUT)},	/* gpmc_ad5.vout3_d5 */
	{GPMC_AD6, (M3 | PIN_OUTPUT)},	/* gpmc_ad6.vout3_d6 */
	{GPMC_AD7, (M3 | PIN_OUTPUT)},	/* gpmc_ad7.vout3_d7 */
	{GPMC_AD8, (M3 | PIN_OUTPUT)},	/* gpmc_ad8.vout3_d8 */
	{GPMC_AD9, (M3 | PIN_OUTPUT)},	/* gpmc_ad9.vout3_d9 */
	{GPMC_AD10, (M3 | PIN_OUTPUT)},	/* gpmc_ad10.vout3_d10 */
	{GPMC_AD11, (M3 | PIN_OUTPUT)},	/* gpmc_ad11.vout3_d11 */
	{GPMC_AD12, (M3 | PIN_OUTPUT)},	/* gpmc_ad12.vout3_d12 */
	{GPMC_AD13, (M3 | PIN_OUTPUT)},	/* gpmc_ad13.vout3_d13 */
	{GPMC_AD14, (M3 | PIN_OUTPUT)},	/* gpmc_ad14.vout3_d14 */
	{GPMC_AD15, (M3 | PIN_OUTPUT)},	/* gpmc_ad15.vout3_d15 */
	{GPMC_A0, (M3 | PIN_OUTPUT)},	/* gpmc_a0.vout3_d16 */
	{GPMC_A1, (M3 | PIN_OUTPUT)},	/* gpmc_a1.vout3_d17 */
	{GPMC_A2, (M3 | PIN_OUTPUT)},	/* gpmc_a2.vout3_d18 */
	{GPMC_A3, (M3 | PIN_OUTPUT)},	/* gpmc_a3.vout3_d19 */
	{GPMC_A4, (M3 | PIN_OUTPUT)},	/* gpmc_a4.vout3_d20 */
	{GPMC_A5, (M3 | PIN_OUTPUT)},	/* gpmc_a5.vout3_d21 */
	{GPMC_A6, (M3 | PIN_OUTPUT)},	/* gpmc_a6.vout3_d22 */
	{GPMC_A7, (M3 | PIN_OUTPUT)},	/* gpmc_a7.vout3_d23 */
	{GPMC_A8, (M3 | PIN_OUTPUT)},	/* gpmc_a8.vout3_hsync */
	{GPMC_A9, (M3 | PIN_OUTPUT)},	/* gpmc_a9.vout3_vsync */
	{GPMC_A10, (M3 | PIN_OUTPUT)},	/* gpmc_a10.vout3_de */
	{GPMC_A11, (M3 | PIN_OUTPUT)},	/* gpmc_a11.vout3_fld */
	{GPMC_CS3, (M3 | PIN_OUTPUT)},	/* gpmc_cs3.vout3_clk */
	/* QSPI1 */
	{GPMC_A13, (M1 | PIN_INPUT | MANUAL_MODE)},	/* gpmc_a13.qspi1_rtclk */
	{GPMC_A14, (M1 | PIN_INPUT | MANUAL_MODE)},	/* gpmc_a14.qspi1_d3 */
	{GPMC_A15, (M1 | PIN_INPUT | MANUAL_MODE)},	/* gpmc_a15.qspi1_d2 */
	{GPMC_A16, (M1 | PIN_INPUT | MANUAL_MODE)},	/* gpmc_a16.qspi1_d0 */
	{GPMC_A17, (M1 | PIN_INPUT | MANUAL_MODE)},	/* gpmc_a17.qspi1_d1 */
	{GPMC_A18, (M1 | PIN_OUTPUT | MANUAL_MODE)},	/* gpmc_a18.qspi1_sclk */
	{GPMC_CS2, (M1 | PIN_OUTPUT | MANUAL_MODE)},	/* gpmc_cs2.qspi1_cs0 */
	/* SPI */
	{SPI2_SCLK, (M0 | PIN_INPUT_PULLDOWN)},	/* spi2_sclk.spi2_sclk */
	{SPI2_D1, (M0 | PIN_INPUT_PULLDOWN)},	/* spi2_d1.spi2_d1 */
	{SPI2_D0, (M0 | PIN_INPUT_PULLDOWN)},	/* spi2_d0.spi2_d0 */
	{SPI2_CS0, (M0 | PIN_INPUT_PULLUP)},	/* spi2_cs0.spi2_cs0 */
	{MMC3_DAT0, (M1 | PIN_INPUT_PULLDOWN)},	/* mmc3_dat0.spi3_d1 */
	{MMC3_DAT1, (M1 | PIN_INPUT_PULLDOWN)},	/* mmc3_dat1.spi3_d0 */
	{MMC3_DAT2, (M1 | PIN_INPUT_PULLUP)},	/* mmc3_dat2.spi3_cs0 */
	{MMC3_CMD, (M1 | PIN_INPUT_PULLDOWN)},	/* mmc3_cmd.spi3_sclk */
	{VIN2A_HSYNC0, (M8 | PIN_INPUT_PULLDOWN)},	/* vin2a_hsync0.spi4_sclk */
	{VIN2A_VSYNC0, (M8 | PIN_INPUT_PULLDOWN)},	/* vin2a_vsync0.spi4_d1 */
	{VIN2A_D0, (M8 | PIN_INPUT_PULLDOWN)},	/* vin2a_d0.spi4_d0 */
	{VIN2A_D1, (M8 | PIN_INPUT_PULLUP)},	/* vin2a_d1.spi4_cs0 */
	{GPMC_A12, (M8 | PIN_INPUT_PULLUP)},	/* gpmc_a12.spi4_cs1 */
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
	{MCASP3_AXR1, (M7 | PIN_INPUT)},	/* mcasp3_axr1.vin1a_d0 */
	{MCASP3_AXR0, (M7 | PIN_INPUT)},	/* mcasp3_axr0.vin1a_d1 */
	{MCASP3_FSX, (M7 | PIN_INPUT)},		/* mcasp3_fsx.vin1a_d2 */
	{MCASP3_ACLKX, (M7 | PIN_INPUT)},	/* mcasp3_aclkx.vin1a_d3 */
	{MCASP2_AXR3, (M7 | PIN_INPUT)},	/* mcasp2_axr3.vin1a_d4 */
	{MCASP2_AXR2, (M7 | PIN_INPUT)},	/* mcasp2_axr2.vin1a_d5 */
	{MCASP2_FSX, (M7 | PIN_INPUT)},		/* mcasp2_fsx.vin1a_d6 */
	{MCASP2_ACLKX, (M7 | PIN_INPUT)},	/* mcasp2_aclkx.vin1a_d7 */
	{XREF_CLK1, (M7 | PIN_INPUT)},		/* xref_clk1.vin1a_clk0 */
	{MCASP1_AXR0, (M7 | PIN_INPUT)},	/* mcasp1_axr0.vin1a_vsync0 */
	{MCASP1_AXR1, (M7 | PIN_INPUT)},	/* mcasp1_axr1.vin1a_hsync0 */
	/* VIN2A */
	{VOUT1_D16, (M3 | PIN_INPUT)},	/* vout1_d16.vin2a_d0 */
	{VOUT1_D17, (M3 | PIN_INPUT)},	/* vout1_d17.vin2a_d1 */
	{VOUT1_D18, (M3 | PIN_INPUT)},	/* vout1_d18.vin2a_d2 */
	{VOUT1_D19, (M3 | PIN_INPUT)},	/* vout1_d19.vin2a_d3 */
	{VOUT1_D20, (M3 | PIN_INPUT)},	/* vout1_d20.vin2a_d4 */
	{VOUT1_D21, (M3 | PIN_INPUT)},	/* vout1_d21.vin2a_d5 */
	{VOUT1_D22, (M3 | PIN_INPUT)},	/* vout1_d22.vin2a_d6 */
	{VOUT1_D23, (M3 | PIN_INPUT)},	/* vout1_d23.vin2a_d7 */
	{VOUT1_FLD, (M3 | PIN_INPUT)},	/* vout1_fld.vin2a_clk0 */
	{VOUT1_HSYNC, (M3 | PIN_INPUT)},	/* vout1_hsync.vin2a_hsync0 */
	{VOUT1_VSYNC, (M3 | PIN_INPUT)},	/* vout1_vsync.vin2a_vsync0 */
	/* PHY& RGMII */
	{VIN2A_D10, (M3 | PIN_OUTPUT)},	/* vin2a_d10.mdio_mclk */
	{VIN2A_D11, (M3 | PIN_INPUT)},	/* vin2a_d11.mdio_d */
	{VIN2A_D12, (M3 | PIN_OUTPUT | MANUAL_MODE)},	/* vin2a_d12.rgmii1_txc */
	{VIN2A_D13, (M3 | PIN_OUTPUT | MANUAL_MODE)},	/* vin2a_d13.rgmii1_txctl */
	{VIN2A_D14, (M3 | PIN_OUTPUT | MANUAL_MODE)},	/* vin2a_d14.rgmii1_txd3 */
	{VIN2A_D15, (M3 | PIN_OUTPUT | MANUAL_MODE)},	/* vin2a_d15.rgmii1_txd2 */
	{VIN2A_D16, (M3 | PIN_OUTPUT | MANUAL_MODE)},	/* vin2a_d16.rgmii1_txd1 */
	{VIN2A_D17, (M3 | PIN_OUTPUT | MANUAL_MODE)},	/* vin2a_d17.rgmii1_txd0 */
	{VIN2A_D18, (M3 | PIN_INPUT | MANUAL_MODE)},	/* vin2a_d18.rgmii1_rxc */
	{VIN2A_D19, (M3 | PIN_INPUT | MANUAL_MODE)},	/* vin2a_d19.rgmii1_rxctl */
	{VIN2A_D20, (M3 | PIN_INPUT | MANUAL_MODE)},	/* vin2a_d20.rgmii1_rxd3 */
	{VIN2A_D21, (M3 | PIN_INPUT | MANUAL_MODE)},	/* vin2a_d21.rgmii1_rxd2 */
	{VIN2A_D22, (M3 | PIN_INPUT | MANUAL_MODE)},	/* vin2a_d22.rgmii1_rxd1 */
	{VIN2A_D23, (M3 | PIN_INPUT | MANUAL_MODE)},	/* vin2a_d23.rgmii1_rxd0 */
	/* USB */
	{USB1_DRVVBUS, (M0 | PIN_OUTPUT)},	/* usb1_drvvbus.usb1_drvvbus */
	{USB2_DRVVBUS, (M0 | PIN_OUTPUT)},	/* usb2_drvvbus.usb2_drvvbus */
	/* I2C */
	{I2C2_SDA, (M0 | PIN_INPUT)},	/* i2c2_sda.i2c2_sda */
	{I2C2_SCL, (M0 | PIN_INPUT)},	/* i2c2_scl.i2c2_scl */
	{MCASP1_ACLKX, (M10 | PIN_INPUT)},	/* mcasp1_aclkx.i2c3_sda */
	{MCASP1_FSX, (M10 | PIN_INPUT)},	/* mcasp1_fsx.i2c3_scl */
	{MCASP4_ACLKX, (M4 | PIN_INPUT)},	/* mcasp4_aclkx.i2c4_sda */
	{MCASP4_FSX, (M4 | PIN_INPUT)},	/* mcasp4_fsx.i2c4_scl */
	{MCASP5_ACLKX, (M4 | PIN_INPUT)},	/* mcasp5_aclkx.i2c5_sda */
	{MCASP5_FSX, (M4 | PIN_INPUT)},	/* mcasp5_fsx.i2c5_scl */
	/* UART */
	{UART1_RXD, (M0 | PIN_INPUT)},	/* uart1_rxd.uart1_rxd */
	{UART1_TXD, (M0 | PIN_OUTPUT)},	/* uart1_txd.uart1_txd */
	{UART2_RXD, (M4 | PIN_INPUT)},	/* uart2_rxd.uart2_rxd */
	{UART2_TXD, (M4 | PIN_OUTPUT)},	/* uart2_txd.uart2_txd */
	{MCASP4_AXR0, (M4 | PIN_INPUT)},	/* mcasp4_axr0.uart4_rxd */
	{MCASP4_AXR1, (M4 | PIN_OUTPUT)},	/* mcasp4_axr1.uart4_txd */
	{VOUT1_D8, (M2 | PIN_INPUT)},	/* vout1_d8.uart6_rxd */
	{VOUT1_D9, (M2 | PIN_OUTPUT)},	/* vout1_d9.uart6_txd */
	/* MMC1 */
	{MMC1_CLK, (M0 | PIN_INPUT_PULLUP)},	/* mmc1_clk.mmc1_clk */
	{MMC1_CMD, (M0 | PIN_INPUT_PULLUP)},	/* mmc1_cmd.mmc1_cmd */
	{MMC1_DAT0, (M0 | PIN_INPUT_PULLUP)},	/* mmc1_dat0.mmc1_dat0 */
	{MMC1_DAT1, (M0 | PIN_INPUT_PULLUP)},	/* mmc1_dat1.mmc1_dat1 */
	{MMC1_DAT2, (M0 | PIN_INPUT_PULLUP)},	/* mmc1_dat2.mmc1_dat2 */
	{MMC1_DAT3, (M0 | PIN_INPUT_PULLUP)},	/* mmc1_dat3.mmc1_dat3 */
	{MMC1_SDCD, (M0 | PIN_INPUT_PULLUP)},	/* mmc1_sdcd.mmc1_sdcd */
	{MMC1_SDWP, (M0 | PIN_INPUT_PULLUP)},	/* mmc1_sdwp.mmc1_sdwp */
	/* PWM */
	{GPIO6_10, (M10 | PIN_OUTPUT_PULLDOWN)},	/* gpio6_10.ehrpwm2A */
	{GPIO6_11, (M10 | PIN_OUTPUT_PULLDOWN)},	/* gpio6_11.ehrpwm2B */
	{MMC3_DAT4, (M10 | PIN_OUTPUT_PULLDOWN)},	/* mmc3_dat4.ehrpwm3A */
	{MMC3_DAT5, (M10 | PIN_OUTPUT_PULLDOWN)},	/* mmc3_dat5.ehrpwm3B */
	/* CAN */
	{SPI1_CS2, (M4 | PIN_INPUT_SLEW)},	/* spi1_cs2.dcan2_tx */
	{SPI1_CS3, (M4 | PIN_INPUT_SLEW)},	/* spi1_cs3.dcan2_rx */
	/* SATA */
	{DCAN1_RX, (M4 | PIN_OUTPUT)},	/* dcan1_rx.sata1_led */
	/* Weakup*/	
	{WAKEUP0, (M0 | PIN_INPUT)},	/* Wakeup0.Wakeup0 */
	{WAKEUP3, (M0 | PIN_INPUT)},	/* Wakeup3.Wakeup3 */
	{ON_OFF, (M0 | PIN_OUTPUT)},	/* on_off.on_off */
	{RTC_PORZ, (M0 | PIN_INPUT)},	/* rtc_porz.rtc_porz */
	{RTCK, (M0 | PIN_OUTPUT)},	/* rtck.rtck */	
	{RESETN, (M0 | PIN_OUTPUT_PULLUP)},	/* resetn.resetn */
	{RSTOUTN, (M0 | PIN_OUTPUT_PULLDOWN)},	/* rstoutn.rstoutn */
	/* PRU GPIO */
	{VIN2A_D4, (M12 | PIN_INPUT | MANUAL_MODE)},	/* vin2a_d4.pr1_pru1_gpi1 */
	{VIN2A_D5, (M12 | PIN_INPUT | MANUAL_MODE)},	/* vin2a_d5.pr1_pru1_gpi2 */
	{VIN2A_D6, (M12 | PIN_INPUT | MANUAL_MODE)},	/* vin2a_d6.pr1_pru1_gpi3 */
	/* GPIO 28*/
	{MCASP1_ACLKR, (M14 | PIN_INPUT_PULLUP)},	/* mcasp1_aclkr.gpio5_0 */
	{MCASP1_FSR, (M14 | PIN_INPUT_PULLUP)},	/* mcasp1_fsr.gpio5_1 */
	{MCASP1_AXR2, (M14 | PIN_INPUT_PULLDOWN)},	/* mcasp1_axr2.gpio5_4 */
	{MCASP1_AXR3, (M14 | PIN_INPUT_PULLUP)},	/* mcasp1_axr3.gpio5_5 */
	{MCASP1_AXR4, (M14 | PIN_INPUT_PULLDOWN)},	/* mcasp1_axr4.gpio5_6 */
	{MCASP1_AXR5, (M14 | PIN_INPUT_PULLDOWN)},	/* mcasp1_axr5.gpio5_7 */
	{MCASP1_AXR6, (M14 | PIN_INPUT_PULLDOWN)},	/* mcasp1_axr6.gpio5_8 */
	{MCASP1_AXR7, (M14 | PIN_INPUT)},	/* mcasp1_axr7.gpio5_9 */
	{MCASP1_AXR8, (M14 | PIN_INPUT_PULLUP)},	/* mcasp1_axr8.gpio5_10 */
	{MCASP1_AXR9, (M14 | PIN_INPUT_PULLUP)},	/* mcasp1_axr9.gpio5_11 */
	{MCASP1_AXR10, (M14 | PIN_INPUT)},	/* mcasp1_axr10.gpio5_12 */
	{MCASP1_AXR11, (M14 | PIN_INPUT)},	/* mcasp1_axr11.gpio4_17 */
	{MCASP1_AXR12, (M14 | PIN_INPUT)},	/* mcasp1_axr12.gpio4_18 */
	{MCASP1_AXR13, (M14 | PIN_INPUT_PULLUP)},   /* mcasp1_axr13.gpio6_4 */
	{MCASP1_AXR14, (M14 | PIN_INPUT_PULLUP)},   /* mcasp1_axr14.gpio6_5 */
	{MCASP1_AXR15, (M14 | PIN_INPUT_PULLUP)},   /* mcasp1_axr15.gpio6_6 */
	{MCASP2_AXR4, (M14 | PIN_INPUT_PULLDOWN)},	/* mcasp2_axr4.gpio1_4 */
	{MCASP2_AXR5, (M14 | PIN_INPUT_PULLDOWN)},	/* mcasp2_axr5.gpio6_7 */
	{MCASP2_AXR6, (M14 | PIN_INPUT_PULLDOWN)},	/* mcasp2_axr6.gpio2_29 */
	{MCASP2_AXR7, (M14 | PIN_INPUT_PULLDOWN)},	/* mcasp2_axr7.gpio1_5 */
	{VOUT1_D0, (M14 | PIN_INPUT_PULLDOWN)},	/* vout1_d0.gpio8_0 */
	{VOUT1_D1, (M14 | PIN_INPUT_PULLDOWN)},	/* vout1_d1.gpio8_1 */
	{VOUT1_D2, (M14 | PIN_INPUT_PULLDOWN)},	/* vout1_d2.gpio8_2 */
	{VOUT1_D3, (M14 | PIN_OUTPUT)},		/* vout1_d3.gpio8_3 */
	{VOUT1_D4, (M14 | PIN_OUTPUT_PULLDOWN)},	/* vout1_d4.gpio8_4 */
	{VOUT1_D5, (M14 | PIN_INPUT_PULLUP)},		/* vout1_d5.gpio8_5 */
	{VOUT1_D6, (M14 | PIN_OUTPUT)},		/* vout1_d6.gpio8_6 */
	{VOUT1_D7, (M14 | PIN_OUTPUT)},		/* vout1_d7.gpio8_7 */
	{VOUT1_D10, (M14 | PIN_OUTPUT)},	/* vout1_d10.gpio8_10 */
	{VOUT1_D11, (M14 | PIN_OUTPUT)},	/* vout1_d11.gpio8_11 */
	{VOUT1_D12, (M14 | PIN_OUTPUT)},	/* vout1_d12.gpio8_12 */
	{VOUT1_D13, (M14 | PIN_OUTPUT)},	/* vout1_d13.gpio8_13 */
	{VOUT1_D14, (M14 | PIN_OUTPUT)},	/* vout1_d14.gpio8_14 */
	{VOUT1_D15, (M14 | PIN_OUTPUT_PULLDOWN)},	/* vout1_d15.gpio8_15 */
	{VOUT1_DE, (M14 | PIN_OUTPUT)},		/* vout1_de.gpio4_20 */
	{VOUT1_CLK, (M14 | PIN_OUTPUT)},		/* vout1_clk.gpio4_19 */	
	{MDIO_MCLK, (M14 | PIN_INPUT_PULLDOWN)},	/* mdio_mclk.gpio5_15 */
	{MDIO_D, (M14 | PIN_INPUT_PULLDOWN)},		/* mdio_d.gpio5_16 */
	{RMII_MHZ_50_CLK, (M14 | PIN_INPUT_PULLDOWN)},	/* RMII_MHZ_50_CLK.gpio5_17 */
	{RGMII0_TXC, (M14 | PIN_INPUT_PULLDOWN)},	/* rgmii0_txc.gpio5_20 */
	{RGMII0_TXCTL, (M14 | PIN_INPUT_PULLDOWN)},	/* rgmii0_txctl.gpio5_21 */
	{RGMII0_TXD3, (M14 | PIN_INPUT_PULLDOWN)},	/* rgmii0_txd3.gpio5_22 */
	{RGMII0_TXD2, (M14 | PIN_INPUT_PULLDOWN)},	/* rgmii0_txd2.gpio5_23 */
	{RGMII0_TXD1, (M14 | PIN_INPUT_PULLDOWN)},	/* rgmii0_txd1.gpio5_24 */
	{RGMII0_TXD0, (M14 | PIN_INPUT_PULLDOWN)},	/* rgmii0_txd0.gpio5_25 */
	{RGMII0_RXC, (M14 | PIN_INPUT_PULLDOWN)},	/* rgmii0_rxc.gpio5_26 */
	{RGMII0_RXCTL, (M14 | PIN_INPUT_PULLDOWN)},	/* rgmii0_rxctl.gpio5_27 */
	{RGMII0_RXD3, (M14 | PIN_INPUT_PULLDOWN)},	/* rgmii0_rxd3.gpio5_28 */
	{RGMII0_RXD2, (M14 | PIN_INPUT_PULLDOWN)},	/* rgmii0_rxd2.gpio5_29 */
	{RGMII0_RXD1, (M14 | PIN_INPUT_PULLDOWN)},	/* rgmii0_rxd1.gpio5_30 */
	{RGMII0_RXD0, (M14 | PIN_INPUT_PULLDOWN)},	/* rgmii0_rxd0.gpio5_31 */
	{UART3_RXD, (M14 | PIN_INPUT_PULLDOWN)},	/* uart3_rxd.gpio5_18 */
	{UART3_TXD, (M14 | PIN_INPUT_PULLDOWN)},	/* uart3_txd.gpio5_19 */	
	{UART1_CTSN, (M14 | PIN_INPUT_PULLDOWN)},	/* uart1_ctsn.gpio7_24 */
	{UART1_RTSN, (M14 | PIN_INPUT_PULLDOWN)},	/* uart1_rtsn.gpio7_25 */
	{VIN2A_D2, (M14 | PIN_INPUT_PULLDOWN)},	/* vin2a_d2.gpio4_3 */
	{VIN2A_D3, (M14 | PIN_INPUT_PULLDOWN)},		/* vin2a_d3.gpio4_4 */
	{VIN2A_DE0, (M14 | PIN_INPUT_PULLDOWN)},	/* vin2a_de0.gpio3_29 */
	{VIN2A_FLD0, (M14 | PIN_INPUT_PULLDOWN)},	/* vin2a_fld0.gpio3_30 */
	{MMC3_CLK, (M14 | PIN_INPUT_PULLUP)},	/* mmc3_clk.gpio6_29 */
	{MMC3_DAT3, (M14 | PIN_INPUT_PULLUP)},	/* mmc3_dat3.gpio7_2 */
	{MMC3_DAT6, (M14 | PIN_INPUT_PULLDOWN)},	/* mmc3_dat6.gpio1_24 */
	{MMC3_DAT7, (M14 | PIN_INPUT_PULLDOWN)},	/* mmc3_dat7.gpio1_25 */	
	{XREF_CLK0, (M14 | PIN_INPUT_PULLDOWN)},	/* xref_clk0.gpio6_17 */
	{XREF_CLK2, (M14 | PIN_INPUT_PULLDOWN)},	/* xref_clk2.gpio6_19 */
	{XREF_CLK3, (M14 | PIN_INPUT_PULLDOWN)},	/* xref_clk3.gpio6_20 */
	{DCAN1_TX, (M14 | PIN_OUTPUT_PULLUP)},	/* dcan1_tx.gpio1_14 */
	{GPMC_CS0, (M14 | PIN_INPUT_PULLDOWN)},			/* gpmc_cs0.gpio2_19 */	
	{GPMC_CLK, (M14 | PIN_INPUT)},					/* gpmc_clk.gpio2_22 */	
	{GPMC_OEN_REN, (M14 | PIN_INPUT_PULLDOWN)},		/* gpmc_oen_ren.gpio2_24 */
	{GPMC_WEN, (M14 | PIN_INPUT_PULLDOWN)},			/* gpmc_wen.gpio2_25 */
	{GPMC_BEN0, (M14 | PIN_INPUT_PULLDOWN)},		/* gpmc_ben0.gpio2_26 */
	{GPMC_BEN1, (M14 | PIN_INPUT_PULLDOWN)},		/* gpmc_ben1.gpio2_27 */
	{GPMC_WAIT0, (M14 | PIN_INPUT_PULLDOWN)},		/* gpmc_wait0.gpio2_28 */	
	{GPIO6_14, (M14 | PIN_INPUT_PULLUP)},	/* gpio6_14.gpio6_14 */
	{GPIO6_15, (M14 | PIN_INPUT_PULLUP)},	/* gpio6_15.gpio6_15 */
	{GPIO6_16, (M14 | PIN_INPUT_PULLDOWN)},		/* gpio6_16.gpio6_16 */
	{SPI1_SCLK, (M14 | PIN_INPUT_PULLDOWN)},	/* spi1_sclk.gpio7_7 */
	{SPI1_D1, (M14 | PIN_INPUT_PULLDOWN)},	/* spi1_d1.gpio7_8 */
	{SPI1_D0, (M14 | PIN_INPUT_PULLDOWN)},	/* spi1_d0.gpio7_9 */
	{SPI1_CS0, (M14 | PIN_INPUT_PULLDOWN)},	/* spi1_cs0.gpio7_10 */
	{SPI1_CS1, (M14 | PIN_OUTPUT_PULLUP)},		/* spi1_cs1.gpio7_11 */
	{GPMC_ADVN_ALE, (M14 | PIN_INPUT_PULLDOWN)},	/* gpmc_advn_ale.gpio2_23 */
	{VIN2A_CLK0, (M14 | PIN_INPUT_PULLDOWN)},	/* vin2a_clk0.gpio3_28 */
#ifdef CONFIG_TEST_AS_GPIO
	/* As VIP GPIO Function! */
	{MCASP3_AXR1, (M13 | PIN_OUTPUT)},	/* mcasp3_axr1.vin1a_d0 */
	{MCASP3_AXR0, (M13 | PIN_OUTPUT)},	/* mcasp3_axr0.vin1a_d1 */
	{MCASP3_FSX, (M14 | PIN_INPUT)},		/* mcasp3_fsx.vin1a_d2 */
	{MCASP3_ACLKX, (M14 | PIN_INPUT)},	/* mcasp3_aclkx.vin1a_d3 */
	{MCASP2_AXR3, (M14 | PIN_INPUT)},	/* mcasp2_axr3.vin1a_d4 */
	{MCASP2_AXR2, (M14 | PIN_INPUT)},	/* mcasp2_axr2.vin1a_d5 */
	{MCASP2_FSX, (M13 | PIN_OUTPUT)},		/* mcasp2_fsx.vin1a_d6 */
	{MCASP2_ACLKX, (M13 | PIN_OUTPUT)},	/* mcasp2_aclkx.vin1a_d7 */
	{XREF_CLK1, (M14 | PIN_OUTPUT)},		/* xref_clk1.vin1a_clk0 */
	{MCASP1_AXR0, (M14 | PIN_INPUT)},	/* mcasp1_axr0.vin1a_vsync0 */
	{MCASP1_AXR1, (M14 | PIN_INPUT)},	
	{VOUT1_D16, (M14 | PIN_INPUT)},	/* vout1_d16.vin2a_d0 */
	{VOUT1_D17, (M14 | PIN_INPUT)},	/* vout1_d17.vin2a_d1 */
	{VOUT1_D18, (M14 | PIN_INPUT)},	/* vout1_d18.vin2a_d2 */
	{VOUT1_D19, (M14 | PIN_INPUT)},	/* vout1_d19.vin2a_d3 */
	{VOUT1_D20, (M14 | PIN_INPUT)},	/* vout1_d20.vin2a_d4 */
	{VOUT1_D21, (M14 | PIN_INPUT)},	/* vout1_d21.vin2a_d5 */
	{VOUT1_D22, (M14 | PIN_INPUT)},	/* vout1_d22.vin2a_d6 */
	{VOUT1_D23, (M14 | PIN_INPUT)},	/* vout1_d23.vin2a_d7 */
	{VOUT1_FLD, (M14 | PIN_INPUT_PULLDOWN)},	/* vout1_fld.vin2a_clk0 */
	{VOUT1_HSYNC, (M14 | PIN_INPUT)},	/* vout1_hsync.vin2a_hsync0 */
	{VOUT1_VSYNC, (M14 | PIN_INPUT)},
	/*  PWM as gpio function */
	{GPIO6_10, (M14 | PIN_INPUT)},	/* gpio6_10.ehrpwm2A */
	{GPIO6_11, (M14 | PIN_INPUT)},	/* gpio6_11.ehrpwm2B */
	{MMC3_DAT4, (M14 | PIN_INPUT)},	/* mmc3_dat4.ehrpwm3A */
	{MMC3_DAT5, (M14 | PIN_INPUT)},	/* mmc3_dat5.ehrpwm3B */
	/* PRU GPIO */
	{VIN2A_D4, (M14 | PIN_INPUT)},	
	{VIN2A_D5, (M14 | PIN_INPUT)},	/* vin2a_d5.pr1_pru1_gpi2 */
	{VIN2A_D6, (M14 | PIN_INPUT)},	/* vin2a_d6.pr1_pru1_gpi3 */
	{VOUT1_D12, (M14 | PIN_INPUT)},
	{VOUT1_D3, (M14 | PIN_INPUT)},	
	{VOUT1_D5, (M14 | PIN_INPUT)},		/* vout1_d5.gpio8_5 */
	{VOUT1_D6, (M14 | PIN_INPUT)},		/* vout1_d6.gpio8_6 */
	{VOUT1_D7, (M14 | PIN_INPUT)},		/* vout1_d7.gpio8_7 */
	{VOUT1_D10, (M14 | PIN_INPUT)},	/* vout1_d10.gpio8_10 */
	{VOUT1_D11, (M14 | PIN_INPUT)},	/* vout1_d11.gpio8_11 */
	{VOUT1_D12, (M14 | PIN_INPUT)},	/* vout1_d12.gpio8_12 */
	{VOUT1_D13, (M14 | PIN_INPUT)},	/* vout1_d13.gpio8_13 */
	{VOUT1_D14, (M14 | PIN_INPUT)},
	{VOUT1_DE, (M14 | PIN_INPUT)},		/* vout1_de.gpio4_20 */
	{VOUT1_CLK, (M14 | PIN_INPUT)},
	{DCAN1_TX, (M14 | PIN_INPUT)},
	{SPI1_CS1, (M14 | PIN_INPUT)},	
#endif
};

#ifdef CONFIG_IODELAY_RECALIBRATION
const struct iodelay_cfg_entry iodelay_cfg_array_am57xx[] = {
	{0x0144, 0, 0},	/* CFG_GPMC_A13_IN */
	{0x0150, 2062, 2277},	/* CFG_GPMC_A14_IN */
	{0x015C, 1960, 2289},	/* CFG_GPMC_A15_IN */
	{0x0168, 2058, 2386},	/* CFG_GPMC_A16_IN */
	{0x0170, 0, 0},	/* CFG_GPMC_A16_OUT */
	{0x0174, 2062, 2350},	/* CFG_GPMC_A17_IN */
	{0x0188, 0, 0},	/* CFG_GPMC_A18_OUT */
	{0x0374, 121, 0},	/* CFG_GPMC_CS2_OUT */
	{0x0A70, 280, 860},	/* CFG_VIN2A_D12_OUT */
	{0x0A7C, 0, 360},	/* CFG_VIN2A_D13_OUT */
	{0x0A88, 280, 240},	/* CFG_VIN2A_D14_OUT */
	{0x0A94, 0, 0},	/* CFG_VIN2A_D15_OUT */
	{0x0AA0, 120, 0},	/* CFG_VIN2A_D16_OUT */
	{0x0AAC, 60, 0},	/* CFG_VIN2A_D17_OUT */
	{0x0AB0, 596, 0},	/* CFG_VIN2A_D18_IN */
	{0x0ABC, 314, 980},	/* CFG_VIN2A_D19_IN */
	{0x0AD4, 241, 1536},	/* CFG_VIN2A_D20_IN */
	{0x0AE0, 103, 1689},	/* CFG_VIN2A_D21_IN */
	{0x0AEC, 161, 1563},	/* CFG_VIN2A_D22_IN */
	{0x0AF8, 0, 1613},	/* CFG_VIN2A_D23_IN */
	{0x0B1C, 0, 500},	/* CFG_VIN2A_D4_IN */
	{0x0B28, 0, 1000},	/* CFG_VIN2A_D5_IN */
	{0x0B34, 0, 500},	/* CFG_VIN2A_D6_IN */
};
#endif

static const struct dmm_lisa_map_regs am57xx_lisa_regs = {
	.dmm_lisa_map_0 = 0x00000000,					
	.dmm_lisa_map_1 = 0x00000000, 
#if 0
	.dmm_lisa_map_2 = 0x80700100,					
	.dmm_lisa_map_3 = 0xFF020100,	
#endif		
	.dmm_lisa_map_2 = 0x80700100,					
	.dmm_lisa_map_3 = 0xFF020100,		
	.is_ma_present = 0x1
};

static const struct emif_regs am572x_emif1_ddr3_532mhz_emif_regs = {
};

/* Ext phy ctrl regs 1-35 */
static const u32 am572x_emif1_ddr3_ext_phy_ctrl_const_regs[] = {
};

static const struct emif_regs am572x_emif2_ddr3_532mhz_emif_regs = {
};

static const u32 am572x_emif2_ddr3_ext_phy_ctrl_const_regs[] = {
};

/*
* DRA72x(Am571x) family soc DDR configuration.
*/
static const struct emif_regs am571x_ddr3_532mhz_emif_regs = {
#if 0
	.sdram_config_init = 0x618513B2,					
	.sdram_config = 0x618513B2,					
	.sdram_config2 = 0x00000000,					
	.ref_ctrl = 0x000040F1,					
	.ref_ctrl_final = 0x00001035,					
	.sdram_tim1 = 0xCEEF36B3,					
	.sdram_tim2 = 0x30BA7FDA,					
	.sdram_tim3 = 0x407F8BA8,					
	.read_idle_ctrl = 0x00050000,					
	.zq_config = 0x5007190B,					
	.temp_alert_config = 0x00000000,					
	.emif_rd_wr_lvl_rmp_ctl = 0x80000000,					
	.emif_rd_wr_lvl_ctl = 0x00000000,					
	.emif_ddr_phy_ctlr_1_init = 0x0824400A,					
	.emif_ddr_phy_ctlr_1 = 0x0E24400A,					
	.emif_rd_wr_exec_thresh = 0x00000305
#endif
	.sdram_config_init = 0x618513B2,					
    .sdram_config = 0x618513B2,					
	.sdram_config2 = 0x00000000,					
    .ref_ctrl = 0x000040F1,					
    .ref_ctrl_final = 0x00001035,					
    .sdram_tim1 = 0xCEEF36B3,					
    .sdram_tim2 = 0x30BF7FDA,					
    .sdram_tim3 = 0x407F8BA8,					
	.read_idle_ctrl = 0x00050000,					
    .zq_config = 0x5007190B,					
	.temp_alert_config = 0x00000000,					
	.emif_rd_wr_lvl_rmp_ctl = 0x80000000,					
	.emif_rd_wr_lvl_ctl = 0x00000000,					
    .emif_ddr_phy_ctlr_1_init = 0x0824400A,					
    .emif_ddr_phy_ctlr_1 = 0x0E24400A,					
	.emif_rd_wr_exec_thresh = 0x00000305	
};
static const u32 am571x_ddr3_ext_phy_ctrl_const_regs[] = {
#if 0
	0x04040100,	// EMIF1_EXT_PHY_CTRL_1				
	0x006B0085,	// EMIF1_EXT_PHY_CTRL_2				
	0x006B0084,	// EMIF1_EXT_PHY_CTRL_3				
	0x006B0094,	// EMIF1_EXT_PHY_CTRL_4				
	0x006B008E,	// EMIF1_EXT_PHY_CTRL_5				
	0x006B006B,	// EMIF1_EXT_PHY_CTRL_6				
	0x002F002F,	// EMIF1_EXT_PHY_CTRL_7				
	0x002F002F,	// EMIF1_EXT_PHY_CTRL_8				
	0x002F002F,	// EMIF1_EXT_PHY_CTRL_9				
	0x002F002F,	// EMIF1_EXT_PHY_CTRL_10				
	0x002F002F,	// EMIF1_EXT_PHY_CTRL_11				
	0x00600066,	// EMIF1_EXT_PHY_CTRL_12				
	0x00600066,	// EMIF1_EXT_PHY_CTRL_13				
	0x00600066,	// EMIF1_EXT_PHY_CTRL_14				
	0x0060006C,	// EMIF1_EXT_PHY_CTRL_15				
	0x00600060,	// EMIF1_EXT_PHY_CTRL_16				
	0x00400046,	// EMIF1_EXT_PHY_CTRL_17				
	0x00400046,	// EMIF1_EXT_PHY_CTRL_18				
	0x00400046,	// EMIF1_EXT_PHY_CTRL_19				
	0x0040004C,	// EMIF1_EXT_PHY_CTRL_20				
	0x00400040,	// EMIF1_EXT_PHY_CTRL_21				
	0x00800080,	// EMIF1_EXT_PHY_CTRL_22				
	0x00800080,	// EMIF1_EXT_PHY_CTRL_23				
	0x40010080,	// EMIF1_EXT_PHY_CTRL_24				
	0x08102040,	// EMIF1_EXT_PHY_CTRL_25				
	0x005B0075,	// EMIF1_EXT_PHY_CTRL_26				
	0x005B0074,	// EMIF1_EXT_PHY_CTRL_27				
	0x005B0084,	// EMIF1_EXT_PHY_CTRL_28				
	0x005B007E,	// EMIF1_EXT_PHY_CTRL_29				
	0x005B005B,	// EMIF1_EXT_PHY_CTRL_30				
	0x00300036,	// EMIF1_EXT_PHY_CTRL_31				
	0x00300036,	// EMIF1_EXT_PHY_CTRL_32				
	0x00300036,	// EMIF1_EXT_PHY_CTRL_33				
	0x0030003C,	// EMIF1_EXT_PHY_CTRL_34				
	0x00300030,	// EMIF1_EXT_PHY_CTRL_35				
	0x00000077	// EMIF1_EXT_PHY_CTRL_36
#endif
    0x04040100,	// EMIF1_EXT_PHY_CTRL_1				
    0x006B0085,	// EMIF1_EXT_PHY_CTRL_2				
    0x006B0084,	// EMIF1_EXT_PHY_CTRL_3				
    0x006B0094,	// EMIF1_EXT_PHY_CTRL_4				
    0x006B008E,	// EMIF1_EXT_PHY_CTRL_5				
    0x006B006B,	// EMIF1_EXT_PHY_CTRL_6				
    0x002F002F,	// EMIF1_EXT_PHY_CTRL_7				
    0x002F002F,	// EMIF1_EXT_PHY_CTRL_8				
    0x002F002F,	// EMIF1_EXT_PHY_CTRL_9				
    0x002F002F,	// EMIF1_EXT_PHY_CTRL_10				
    0x002F002F,	// EMIF1_EXT_PHY_CTRL_11				
    0x00600066,	// EMIF1_EXT_PHY_CTRL_12				
    0x00600066,	// EMIF1_EXT_PHY_CTRL_13				
    0x00600066,	// EMIF1_EXT_PHY_CTRL_14				
    0x0060006C,	// EMIF1_EXT_PHY_CTRL_15				
    0x00600060,	// EMIF1_EXT_PHY_CTRL_16				
    0x00400046,	// EMIF1_EXT_PHY_CTRL_17				
    0x00400046,	// EMIF1_EXT_PHY_CTRL_18				
    0x00400046,	// EMIF1_EXT_PHY_CTRL_19				
    0x0040004C,	// EMIF1_EXT_PHY_CTRL_20				
    0x00400040,	// EMIF1_EXT_PHY_CTRL_21				
    0x00800080,	// EMIF1_EXT_PHY_CTRL_22				
    0x00800080,	// EMIF1_EXT_PHY_CTRL_23				
    0x40010080,	// EMIF1_EXT_PHY_CTRL_24				
    0x08102040,	// EMIF1_EXT_PHY_CTRL_25				
    0x005B0075,	// EMIF1_EXT_PHY_CTRL_26				
    0x005B0074,	// EMIF1_EXT_PHY_CTRL_27				
    0x005B0084,	// EMIF1_EXT_PHY_CTRL_28				
    0x005B007E,	// EMIF1_EXT_PHY_CTRL_29				
    0x005B005B,	// EMIF1_EXT_PHY_CTRL_30				
    0x00300036,	// EMIF1_EXT_PHY_CTRL_31				
    0x00300036,	// EMIF1_EXT_PHY_CTRL_32				
    0x00300036,	// EMIF1_EXT_PHY_CTRL_33				
    0x0030003C,	// EMIF1_EXT_PHY_CTRL_34				
    0x00300030,	// EMIF1_EXT_PHY_CTRL_35				
    0x00000077	// EMIF1_EXT_PHY_CTRL_36
};

struct vcores_data am57xx_volts = {
	.mpu.value		= VDD_MPU_DRA7,
	.mpu.efuse.reg		= STD_FUSE_OPP_VMIN_MPU,
	.mpu.efuse.reg_bits     = DRA752_EFUSE_REGBITS,
	.mpu.addr		= TPS659038_REG_ADDR_SMPS12,
	.mpu.pmic		= &tps659038,
	.mpu.abb_tx_done_mask	= OMAP_ABB_MPU_TXDONE_MASK,

	.eve.value		= VDD_EVE_DRA7,
	.eve.efuse.reg		= STD_FUSE_OPP_VMIN_DSPEVE,
	.eve.efuse.reg_bits	= DRA752_EFUSE_REGBITS,
	.eve.addr		= TPS659038_REG_ADDR_SMPS45,
	.eve.pmic		= &tps659038,
	.eve.abb_tx_done_mask	= OMAP_ABB_EVE_TXDONE_MASK,

	.gpu.value		= VDD_GPU_DRA7,
	.gpu.efuse.reg		= STD_FUSE_OPP_VMIN_GPU,
	.gpu.efuse.reg_bits	= DRA752_EFUSE_REGBITS,
	.gpu.addr		= TPS659038_REG_ADDR_SMPS45,
	.gpu.pmic		= &tps659038,
	.gpu.abb_tx_done_mask	= OMAP_ABB_GPU_TXDONE_MASK,

	.core.value		= VDD_CORE_DRA7,
	.core.efuse.reg		= STD_FUSE_OPP_VMIN_CORE,
	.core.efuse.reg_bits	= DRA752_EFUSE_REGBITS,
	.core.addr		= TPS659038_REG_ADDR_SMPS6,
	.core.pmic		= &tps659038,

	.iva.value		= VDD_IVA_DRA7,
	.iva.efuse.reg		= STD_FUSE_OPP_VMIN_IVA,
	.iva.efuse.reg_bits	= DRA752_EFUSE_REGBITS,
	.iva.addr		= TPS659038_REG_ADDR_SMPS45,
	.iva.pmic		= &tps659038,
	.iva.abb_tx_done_mask	= OMAP_ABB_IVA_TXDONE_MASK,
};

#endif /* _MUX_GF33B1_H_ */
