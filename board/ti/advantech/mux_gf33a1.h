/*
 * Copyright (C) 2016 Advantech Co.,Ltd 
 *
 * Author: yanwei.cao <yanwei.cao@advantech.com.cn>
 *
 * Based on board/ti/advantech/mux_data.h
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef _MUX_GF33A1_H_
#define _MUX_GF33A1_H_

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
	{GPMC_A8, (M3 | PIN_OUTPUT)},	/* gpmc_a8.vout3_hsync0 */
	{GPMC_A9, (M3 | PIN_OUTPUT)},	/* gpmc_a9.vout3_vsync0 */
	{GPMC_A10, (M3 | PIN_OUTPUT)},	/* gpmc_a10.vout3_de0 */
	{GPMC_A11, (M3 | PIN_OUTPUT)},	/* gpmc_a11.vout3_fld0 */
	{GPMC_CS3, (M3 | PIN_OUTPUT)},	/* gpmc_cs3.vout3_clk */
	/* QSPI1 */
	{GPMC_A12, (M8 | PIN_INPUT_PULLUP)},	/* gpmc_a12.spi4_cs1 */
	{GPMC_A13, (M1 | PIN_INPUT)},	/* gpmc_a13.qspi1_rtclk */
	{GPMC_A14, (M1 | PIN_INPUT)},	/* gpmc_a14.qspi1_d3 */
	{GPMC_A15, (M1 | PIN_INPUT)},	/* gpmc_a15.qspi1_d2 */
	{GPMC_A16, (M1 | PIN_INPUT)},	/* gpmc_a16.qspi1_d0 */
	{GPMC_A17, (M1 | PIN_INPUT)},	/* gpmc_a17.qspi1_d1 */
	{GPMC_A18, (M1 | PIN_OUTPUT)},	/* gpmc_a18.qspi1_sclk */
	{GPMC_CS2, (M1 | PIN_INPUT_PULLUP)},	/* gpmc_cs2.qspi1_cs0 */
	/* SPI */
	{VIN2A_HSYNC0, (M8 | PIN_INPUT_PULLDOWN)},	/* vin2a_hsync0.spi4_sclk */
	{VIN2A_VSYNC0, (M8 | PIN_INPUT_PULLDOWN)},	/* vin2a_vsync0.spi4_d1 */
	{VIN2A_D0, (M8 | PIN_INPUT_PULLDOWN)},	/* vin2a_d0.spi4_d0 */
	{VIN2A_D1, (M8 | PIN_INPUT_PULLUP)},	/* vin2a_d1.spi4_cs0 */
	{MCASP1_AXR8, (M3 | PIN_INPUT_PULLDOWN)},	/* mcasp1_axr8.spi3_sclk */
	{MCASP1_AXR9, (M3 | PIN_INPUT_PULLDOWN)},	/* mcasp1_axr9.spi3_d1 */
	{MCASP1_AXR10, (M3 | PIN_INPUT_PULLDOWN)},	/* mcasp1_axr10.spi3_d0 */
	{MCASP1_AXR11, (M3 | PIN_INPUT_PULLUP)},	/* mcasp1_axr11.spi3_cs0 */
	{SPI2_SCLK, (M0 | PIN_INPUT_PULLDOWN)},	
	{SPI2_D1, (M0 | PIN_INPUT_PULLDOWN)},	
	{SPI2_D0, (M0 | PIN_INPUT_PULLDOWN)},	
	{SPI2_CS0, (M0 | PIN_INPUT_PULLUP)},
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
	{VIN1A_CLK0, (M0 | PIN_INPUT)},
	{VIN1A_HSYNC0, (M0 | PIN_INPUT)},
	{VIN1A_VSYNC0, (M0 | PIN_INPUT)},
	{VIN1A_D0, (M0 | PIN_INPUT)},
	{VIN1A_D1, (M0 | PIN_INPUT)},
	{VIN1A_D2, (M0 | PIN_INPUT)},
	{VIN1A_D3, (M0 | PIN_INPUT)},
	{VIN1A_D4, (M0 | PIN_INPUT)},
	{VIN1A_D5, (M0 | PIN_INPUT)},
	{VIN1A_D6, (M0 | PIN_INPUT)},
	{VIN1A_D7, (M0 | PIN_INPUT)},
	/* VIN3A */
	{VOUT1_D16, (M4 | PIN_INPUT)},		/* vout1_d16.vin3a_d0 */
	{VOUT1_D17, (M4 | PIN_INPUT)},		/* vout1_d17.vin3a_d1 */
	{VOUT1_D18, (M4 | PIN_INPUT)},		/* vout1_d18.vin3a_d2 */
	{VOUT1_D19, (M4 | PIN_INPUT)},		/* vout1_d19.vin3a_d3 */
	{VOUT1_D20, (M4 | PIN_INPUT)},		/* vout1_d20.vin3a_d4 */
	{VOUT1_D21, (M4 | PIN_INPUT)},		/* vout1_d21.vin3a_d5 */
	{VOUT1_D22, (M4 | PIN_INPUT)},		/* vout1_d22.vin3a_d6 */
	{VOUT1_D23, (M4 | PIN_INPUT)},		/* vout1_d23.vin3a_d7 */
	{VOUT1_FLD, (M4 | PIN_INPUT)},		/* vout1_fld.vin3a_clk0 */
	{VOUT1_HSYNC, (M4 | PIN_INPUT)},	/* vout1_hsync.vin3a_hsync0 */
	{VOUT1_VSYNC, (M4 | PIN_INPUT)},	/* vout1_vsync.vin3a_vsync0 */
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
	/* I2C */
	{MCASP1_ACLKX, (M10 | PIN_INPUT_PULLUP)},	/* mcasp1_aclkx.i2c3_sda */
	{MCASP1_FSX, (M10 | PIN_INPUT_PULLUP)},		/* mcasp1_fsx.i2c3_scl */
	{MCASP1_ACLKR, (M10 | PIN_INPUT_PULLUP)},	/* mcasp1_aclkr.i2c4_sda */
	{MCASP1_FSR, (M10 | PIN_INPUT_PULLUP)},		/* mcasp1_fsr.i2c4_scl */
	{MCASP1_AXR0, (M10 | PIN_INPUT_PULLUP | SLEWCONTROL)},	/* mcasp1_axr0.i2c5_sda */
	{MCASP1_AXR1, (M10 | PIN_INPUT_PULLUP | SLEWCONTROL)},	/* mcasp1_axr1.i2c5_scl */
	{I2C1_SDA, (M0 | PIN_INPUT_PULLUP)},		/* i2c1_sda.i2c1_sda */
	{I2C1_SCL, (M0 | PIN_INPUT_PULLUP)},		/* i2c1_scl.i2c1_scl */	
	/* UART */
	{MCASP3_AXR0, (M4 | PIN_INPUT_SLEW)},	/* mcasp3_axr0.uart5_rxd */
	{MCASP3_AXR1, (M4 | PIN_INPUT_SLEW)},	/* mcasp3_axr1.uart5_txd */
	{MCASP4_AXR0, (M4 | PIN_INPUT)},	/* mcasp4_axr0.uart4_rxd */
	{MCASP4_AXR1, (M4 | PIN_OUTPUT)},	/* mcasp4_axr1.uart4_txd */
	{UART2_RXD, (M4 | PIN_INPUT_SLEW)},	/* N/A.uart2_rxd */
	{UART2_TXD, (M4 | PIN_INPUT_SLEW)},	/* uart2_txd.uart2_txd */
	{UART1_RXD, (M0 | PIN_INPUT_SLEW)},	/* uart1_rxd.uart1_rxd */
	{UART1_TXD, (M0 | PIN_INPUT_SLEW)},	/* uart1_txd.uart1_txd */
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
	{WAKEUP0, (M0 | PULL_UP)},	/* Wakeup0.Wakeup0 */
	{WAKEUP1, (M0)},			/* Wakeup1.Wakeup1 */
	{WAKEUP2, (M0)},			/* Wakeup2.Wakeup2 */
	{WAKEUP3, (M0 | PULL_UP)},	/* Wakeup3.Wakeup3 */
	{ON_OFF, (M0 | PIN_OUTPUT_PULLUP)},		/* on_off.on_off */
	{RTC_PORZ, (M0 | PIN_OUTPUT_PULLDOWN)},	/* rtc_porz.rtc_porz */
	{RTCK, (M0 | PIN_INPUT_PULLDOWN)},		/* rtck.rtck */
	/* PRU GPIO */
	{MDIO_MCLK, (M13 | PIN_OUTPUT)},	/* mdio_mclk.pr2_pru1_gpo0 */
	{MDIO_D, (M13 | PIN_OUTPUT)},		/* mdio_d.pr2_pru1_gpo1 */
	{RMII_MHZ_50_CLK, (M13 | PIN_OUTPUT)},		/* RMII_MHZ_50_CLK.pr2_pru1_gpo2 */
	{UART3_RXD, (M13 | PIN_OUTPUT_PULLDOWN)},	/* uart3_rxd.pr2_pru1_gpo3 */
	{UART3_TXD, (M13 | PIN_OUTPUT_PULLDOWN)},	/* uart3_txd.pr2_pru1_gpo4 */
	{RGMII0_TXC, (M13 | PIN_OUTPUT)},		/* rgmii0_txc.pr2_pru1_gpo5 */
	{RGMII0_TXCTL, (M13 | PIN_OUTPUT)},		/* rgmii0_txctl.pr2_pru1_gpo6 */
	{RGMII0_TXD3, (M13 | PIN_OUTPUT)},		/* rgmii0_txd3.pr2_pru1_gpo7 */
	{RGMII0_TXD2, (M13 | PIN_OUTPUT)},		/* rgmii0_txd2.pr2_pru1_gpo8 */
	{RGMII0_TXD1, (M13 | PIN_OUTPUT)},		/* rgmii0_txd1.pr2_pru1_gpo9 */
	{RGMII0_TXD0, (M13 | PIN_OUTPUT)},		/* rgmii0_txd0.pr2_pru1_gpo10 */
	{RGMII0_RXC, (M13 | PIN_OUTPUT)},		/* rgmii0_rxc.pr2_pru1_gpo11 */
	{RGMII0_RXCTL, (M13 | PIN_OUTPUT)},		/* rgmii0_rxctl.pr2_pru1_gpo12 */
	{RGMII0_RXD3, (M13 | PIN_OUTPUT)},		/* rgmii0_rxd3.pr2_pru1_gpo13 */
	{RGMII0_RXD2, (M13 | PIN_OUTPUT)},		/* rgmii0_rxd2.pr2_pru1_gpo14 */
	{RGMII0_RXD1, (M13 | PIN_OUTPUT)},		/* rgmii0_rxd1.pr2_pru1_gpo15 */
	{RGMII0_RXD0, (M13 | PIN_OUTPUT)},		/* rgmii0_rxd0.pr2_pru1_gpo16 */
	{VIN2A_D4, (M12 | PIN_INPUT_PULLDOWN)},	/* vin2a_d4.pr1_pru1_gpi1 */
	{VIN2A_D5, (M12 | PIN_INPUT_PULLDOWN)},	/* vin2a_d5.pr1_pru1_gpi2 */
	{VIN2A_D6, (M12 | PIN_INPUT_PULLDOWN)},	/* vin2a_d6.pr1_pru1_gpi3 */
	{VOUT1_D0, (M13 | PIN_OUTPUT)},			/* vout1_d0.pr2_pru1_gpo18 */
	{VOUT1_D1, (M13 | PIN_OUTPUT)},			/* vout1_d1.pr2_pru1_gpo19 */
	{VOUT1_D2, (M13 | PIN_OUTPUT)},			/* vout1_d2.pr2_pru1_gpo20 */
	/* GPIO */
	{VIN1A_D11, (M14 | PIN_INPUT_PULLDOWN)},    /* vin1a_d11.gpio3_15*/
	{VIN1A_D12, (M14 | PIN_INPUT_PULLDOWN)},	/* vin1a_d12.gpio3_16*/
	{VIN1A_D13, (M14 | PIN_INPUT_PULLDOWN)},	/* vin1a_d13.gpio3_17*/
	{VIN1A_D14, (M14 | PIN_INPUT_PULLDOWN)},	/* vin1a_d14.gpio3_18*/
	{VIN1A_D15, (M14 | PIN_INPUT_PULLDOWN)},	/* vin1a_d15.gpio3_19 */
	{VIN1A_D16, (M14 | PIN_INPUT_PULLDOWN)},	/* vin1a_d16.gpio3_20*/
	{VIN1A_D17, (M14 | PIN_INPUT_PULLDOWN)},	/* vin1a_d17.gpio3_21 */
	{VIN1A_D18, (M14 | PIN_INPUT_PULLDOWN)},	/* vin1a_d18.gpio3_22 */
	{VIN1A_D19, (M14 | PIN_INPUT_PULLDOWN)},	/* vin1a_d19.gpio3_23*/
	{VIN1A_D21, (M14 | PIN_INPUT_PULLDOWN)},	/* vin1a_d21.gpio3_25 */
	{VIN1A_D23, (M14 | PIN_INPUT_PULLDOWN)},	/* vin1a_d23.gpio3_27 */
	{VIN2A_CLK0, (M14 | PIN_INPUT_PULLDOWN)},	/* vin2a_clk0.gpio3_28 */
	{VIN2A_D2, (M14 | PIN_INPUT_PULLDOWN)},		/* vin2a_d2.gpio4_3 */
	{UART1_CTSN, (M14 | PIN_INPUT_PULLDOWN)},	/* uart1_ctsn.gpio7_24 */
	{UART1_RTSN, (M14 | PIN_INPUT_PULLDOWN)},	/* uart1_rtsn.gpio7_25 */
	{GPMC_CS0, (M14 | PIN_INPUT_PULLDOWN)},			/* gpmc_cs0.gpio2_19 */	
	{GPMC_CLK, (M14 | PIN_INPUT)},					/* gpmc_clk.gpio2_22 */
	{GPMC_ADVN_ALE, (M14 | PIN_INPUT_PULLDOWN)},	/* gpmc_advn_ale.gpio2_23 */
	{GPMC_OEN_REN, (M14 | PIN_INPUT_PULLDOWN)},		/* gpmc_oen_ren.gpio2_24 */
	{GPMC_WEN, (M14 | PIN_INPUT_PULLDOWN)},			/* gpmc_wen.gpio2_25 */
	{GPMC_BEN0, (M14 | PIN_INPUT_PULLDOWN)},		/* gpmc_ben0.gpio2_26 */
	{GPMC_BEN1, (M14 | PIN_INPUT_PULLDOWN)},		/* gpmc_ben1.gpio2_27 */
	{GPMC_WAIT0, (M14 | PIN_INPUT_PULLDOWN)},		/* gpmc_wait0.gpio2_28 */
	{MMC3_CLK, (M14 | PIN_INPUT_PULLUP)},	/* mmc3_clk.gpio6_29 */
	{MMC3_CMD, (M14 | PIN_INPUT_PULLUP)},	/* mmc3_cmd.gpio6_30 */
	{MMC3_DAT0, (M14 | PIN_INPUT_PULLUP)},	/* mmc3_dat0.gpio6_31 */
	{MMC3_DAT1, (M14 | PIN_INPUT_PULLUP)},	/* mmc3_dat1.gpio7_0 */
	{MMC3_DAT2, (M14 | PIN_INPUT_PULLUP)},	/* mmc3_dat2.gpio7_1 */
	{MMC3_DAT3, (M14 | PIN_INPUT_PULLUP)},	/* mmc3_dat3.gpio7_2 */
	{MMC3_DAT6, (M14 | PIN_INPUT_PULLDOWN)},	/* mmc3_dat6.gpio1_24 */
	{MMC3_DAT7, (M14 | PIN_INPUT_PULLDOWN)},	/* mmc3_dat7.gpio1_25 */
	{SPI1_CS1, (M14 | PIN_OUTPUT_PULLUP)},		/* spi1_cs1.gpio7_11 */
	{MCASP1_AXR2, (M14 | PIN_INPUT_PULLDOWN)},	/* mcasp1_axr2.gpio5_4 */
	{MCASP1_AXR3, (M14 | PIN_INPUT_PULLDOWN)},	/* mcasp1_axr3.gpio5_5 */
	{MCASP1_AXR4, (M14 | PIN_INPUT_PULLDOWN)},	/* mcasp1_axr4.gpio5_6 */
	{MCASP1_AXR5, (M14 | PIN_INPUT_PULLDOWN)},	/* mcasp1_axr5.gpio5_7 */
	{MCASP1_AXR6, (M14 | PIN_INPUT_PULLDOWN)},	/* mcasp1_axr6.gpio5_8 */
	{MCASP1_AXR7, (M14 | PIN_INPUT_PULLDOWN)},	/* mcasp1_axr7.gpio5_9 */
	{GPIO6_16, (M14 | PIN_INPUT_PULLDOWN)},		/* gpio6_16.gpio6_16 */
	{XREF_CLK0, (M14 | PIN_INPUT_PULLDOWN)},	/* xref_clk0.gpio6_17 */
	{XREF_CLK1, (M14 | PIN_INPUT_PULLDOWN)},	/* xref_clk1.gpio6_18 */
	{XREF_CLK2, (M14 | PIN_INPUT_PULLDOWN)},	/* xref_clk2.gpio6_19 */
	{XREF_CLK3, (M14 | PIN_INPUT_PULLDOWN)},	/* xref_clk3.gpio6_20 */
	{MCASP1_AXR12, (M14 | PIN_INPUT_PULLDOWN)},	/* mcasp1_axr12.gpio4_18 */
	{MCASP1_AXR14, (M14 | PIN_INPUT_PULLDOWN)},	/* mcasp1_axr14.gpio6_5 */
	{MCASP2_AXR2, (M14 | PIN_INPUT_PULLDOWN)},	/* mcasp2_axr2.gpio6_8 */
	{MCASP2_AXR3, (M14 | PIN_INPUT_PULLDOWN)},	/* mcasp2_axr3.gpio6_9 */
	{MCASP2_AXR4, (M14 | PIN_INPUT_PULLDOWN)},	/* mcasp2_axr4.gpio1_4 */
	{MCASP2_AXR5, (M14 | PIN_INPUT_PULLDOWN)},	/* mcasp2_axr5.gpio6_7 */
	{MCASP2_AXR6, (M14 | PIN_INPUT_PULLDOWN)},	/* mcasp2_axr6.gpio2_29 */
	{MCASP2_AXR7, (M14 | PIN_INPUT_PULLDOWN)},	/* mcasp2_axr7.gpio1_5 */
	{MCASP3_ACLKX, (M14 | PIN_INPUT_PULLDOWN)},	/* mcasp3_aclkx.gpio5_13 */
	{MCASP3_FSX, (M14 | PIN_INPUT_PULLDOWN)},	/* mcasp3_fsx.gpio5_14 */
	{VOUT1_CLK, (M14 | PIN_OUTPUT)},		/* vout1_clk.gpio4_19 */
	{VOUT1_DE, (M14 | PIN_OUTPUT)},		/* vout1_de.gpio4_20 */
	{VOUT1_D3, (M14 | PIN_OUTPUT)},		/* vout1_d3.gpio8_3 */
	{VOUT1_D4, (M14 | PIN_OUTPUT)},		/* vout1_d4.gpio8_4 */
	{VOUT1_D5, (M14 | PIN_OUTPUT)},		/* vout1_d5.gpio8_5 */
	{VOUT1_D6, (M14 | PIN_OUTPUT)},		/* vout1_d6.gpio8_6 */
	{VOUT1_D7, (M14 | PIN_OUTPUT)},		/* vout1_d7.gpio8_7 */
	{VOUT1_D8, (M14 | PIN_OUTPUT)},		/* vout1_d8.gpio8_8 */
	{VOUT1_D9, (M14 | PIN_OUTPUT)},		/* vout1_d9.gpio8_9 */
	{VOUT1_D10, (M14 | PIN_OUTPUT)},	/* vout1_d10.gpio8_10 */
	{VOUT1_D11, (M14 | PIN_OUTPUT)},	/* vout1_d11.gpio8_11 */
	{VOUT1_D12, (M14 | PIN_OUTPUT)},	/* vout1_d12.gpio8_12 */
	{VOUT1_D13, (M14 | PIN_OUTPUT)},	/* vout1_d13.gpio8_13 */
	{VOUT1_D14, (M14 | PIN_OUTPUT)},	/* vout1_d14.gpio8_14 */
	{VOUT1_D15, (M14 | PIN_OUTPUT)},	/* vout1_d15.gpio8_15 */
#ifdef CONFIG_TEST_AS_GPIO
	/* As VIP GPIO Function! */
	{VOUT1_D16, (M14 | PIN_INPUT)}, /* gpio8_16*/
	{VOUT1_D17, (M14 | PIN_INPUT)},		
	{VOUT1_D18, (M14 | PIN_INPUT)},		
	{VOUT1_D19, (M14 | PIN_INPUT)},		
	{VOUT1_D20, (M14 | PIN_INPUT)},		
	{VOUT1_D21, (M14 | PIN_INPUT)},		
	{VOUT1_D22, (M14 | PIN_INPUT)},		
	{VOUT1_D23, (M14 | PIN_INPUT)},
	{VOUT1_FLD, (M14 | PIN_INPUT)},		
	{VOUT1_HSYNC, (M14 | PIN_INPUT)},	
	{VOUT1_VSYNC, (M14 | PIN_INPUT)},
	{VIN1A_CLK0, (M14 | PIN_INPUT)},
	{VIN1A_HSYNC0, (M14 | PIN_INPUT)},
	{VIN1A_VSYNC0, (M14 | PIN_INPUT)},
	{VIN1A_D0, (M14 | PIN_INPUT)},
	{VIN1A_D1, (M14 | PIN_INPUT)},
	{VIN1A_D2, (M14 | PIN_INPUT)},
	{VIN1A_D3, (M14 | PIN_INPUT)},
	{VIN1A_D4, (M14 | PIN_INPUT)},
	{VIN1A_D5, (M14 | PIN_INPUT)},
	{VIN1A_D6, (M14 | PIN_INPUT)},
	{VIN1A_D7, (M14 | PIN_INPUT)},
	/*  PWM as gpio function */
	{GPIO6_10, (M0 | PIN_INPUT)},
	{GPIO6_11, (M0 | PIN_INPUT)},
	{MMC3_DAT4, (M14 | PIN_INPUT)},
	{MMC3_DAT5, (M14 | PIN_INPUT)},
	/* CLKOUT as gpio */
	{GPMC_CLK, (M14 | PIN_INPUT)},
	{MCASP1_AXR12, (M14 | PIN_INPUT)},
	{MCASP1_AXR14, (M14 | PIN_INPUT)},	
	{GPMC_ADVN_ALE, (M14 | PIN_INPUT)},
	/* CLKIN as gpio */
	{VIN2A_D2, (M14 | PIN_INPUT)},
	{VIN2A_CLK0, (M14 | PIN_INPUT)},
	{MMC3_CMD, (M14 | PIN_INPUT)},
	{MMC3_DAT7, (M14 | PIN_INPUT)},	
	{VOUT1_D3, (M14 | PIN_INPUT)},
	{VOUT1_D12, (M14 | PIN_INPUT)},	
	{VOUT1_D5, (M14 | PIN_INPUT)},
	/* CN7*/
	{XREF_CLK0, (M14 | PIN_INPUT)},
	{MMC3_DAT0, (M14 | PIN_INPUT)},
	{VOUT1_D8, (M14 | PIN_INPUT)},		/* vout1_d8.vout1_d8 */
	{VOUT1_D9, (M14 | PIN_INPUT)},
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
	.sdram_config_init = 0x61873B32,					
	.sdram_config = 0x61873B32,					
	.sdram_config2 = 0x00000000,					
	.ref_ctrl = 0x000040F1,					
	.ref_ctrl_final = 0x00001035,					
	.sdram_tim1 = 0xCEEF2673,					
	.sdram_tim2 = 0x308F7FDA,					
	.sdram_tim3 = 0x407F88A8,					
	.read_idle_ctrl = 0x00050000,					
	.zq_config = 0x5007190B,					
	.temp_alert_config = 0x00000000,					
	.emif_rd_wr_lvl_rmp_ctl = 0x80000000,					
	.emif_rd_wr_lvl_ctl = 0x00000000,					
	.emif_ddr_phy_ctlr_1_init = 0x0024400F,					
	.emif_ddr_phy_ctlr_1 = 0x0E24400F,					
	.emif_rd_wr_exec_thresh = 0x00000305
};

/* Ext phy ctrl regs 1-35 */
static const u32 am572x_emif1_ddr3_ext_phy_ctrl_const_regs[] = {
	0x04040100,	// EMIF1_EXT_PHY_CTRL_1				
	0x006B008C,	// EMIF1_EXT_PHY_CTRL_2				
	0x006B008D,	// EMIF1_EXT_PHY_CTRL_3				
	0x006B0095,	// EMIF1_EXT_PHY_CTRL_4				
	0x006B008F,	// EMIF1_EXT_PHY_CTRL_5				
	0x006B006B,	// EMIF1_EXT_PHY_CTRL_6				
	0x00320032,	// EMIF1_EXT_PHY_CTRL_7				
	0x00320032,	// EMIF1_EXT_PHY_CTRL_8				
	0x00320032,	// EMIF1_EXT_PHY_CTRL_9				
	0x00320032,	// EMIF1_EXT_PHY_CTRL_10				
	0x00320032,	// EMIF1_EXT_PHY_CTRL_11				
	0x00600068,	// EMIF1_EXT_PHY_CTRL_12				
	0x00600067,	// EMIF1_EXT_PHY_CTRL_13				
	0x0060006C,	// EMIF1_EXT_PHY_CTRL_14				
	0x00600072,	// EMIF1_EXT_PHY_CTRL_15				
	0x00600060,	// EMIF1_EXT_PHY_CTRL_16				
	0x00400048,	// EMIF1_EXT_PHY_CTRL_17				
	0x00400047,	// EMIF1_EXT_PHY_CTRL_18				
	0x0040004C,	// EMIF1_EXT_PHY_CTRL_19				
	0x00400052,	// EMIF1_EXT_PHY_CTRL_20				
	0x00400040,	// EMIF1_EXT_PHY_CTRL_21				
	0x00800080,	// EMIF1_EXT_PHY_CTRL_22				
	0x00800080,	// EMIF1_EXT_PHY_CTRL_23				
	0x40010080,	// EMIF1_EXT_PHY_CTRL_24				
	0x08102040,	// EMIF1_EXT_PHY_CTRL_25				
	0x005B007C,	// EMIF1_EXT_PHY_CTRL_26				
	0x005B007D,	// EMIF1_EXT_PHY_CTRL_27				
	0x005B0085,	// EMIF1_EXT_PHY_CTRL_28				
	0x005B007F,	// EMIF1_EXT_PHY_CTRL_29				
	0x005B005B,	// EMIF1_EXT_PHY_CTRL_30				
	0x00300038,	// EMIF1_EXT_PHY_CTRL_31				
	0x00300037,	// EMIF1_EXT_PHY_CTRL_32				
	0x0030003C,	// EMIF1_EXT_PHY_CTRL_33				
	0x00300042,	// EMIF1_EXT_PHY_CTRL_34				
	0x00300030,	// EMIF1_EXT_PHY_CTRL_35				
	0x00000077	// EMIF1_EXT_PHY_CTRL_36	
};

static const struct emif_regs am572x_emif2_ddr3_532mhz_emif_regs = {
	.sdram_config_init = 0x61873B32,					
	.sdram_config = 0x61873B32,					
	.sdram_config2 = 0x00000000,					
	.ref_ctrl = 0x000040F1,					
	.ref_ctrl_final = 0x00001035,					
	.sdram_tim1 = 0xCEEF2673,					
	.sdram_tim2 = 0x308F7FDA,					
	.sdram_tim3 = 0x407F88A8,					
	.read_idle_ctrl = 0x00050000,					
	.zq_config = 0x5007190B,					
	.temp_alert_config = 0x00000000,					
	.emif_rd_wr_lvl_rmp_ctl = 0x80000000,					
	.emif_rd_wr_lvl_ctl = 0x00000000,					
	.emif_ddr_phy_ctlr_1_init = 0x0024400F,					
	.emif_ddr_phy_ctlr_1 = 0x0E24400F,					
	.emif_rd_wr_exec_thresh = 0x00000305
};

static const u32 am572x_emif2_ddr3_ext_phy_ctrl_const_regs[] = {
	0x04040100,	// EMIF2_EXT_PHY_CTRL_1				
	0x006B0087,	// EMIF2_EXT_PHY_CTRL_2				
	0x006B0083,	// EMIF2_EXT_PHY_CTRL_3				
	0x006B008C,	// EMIF2_EXT_PHY_CTRL_4				
	0x006B0089,	// EMIF2_EXT_PHY_CTRL_5				
	0x006B006B,	// EMIF2_EXT_PHY_CTRL_6				
	0x00320032,	// EMIF2_EXT_PHY_CTRL_7				
	0x00320032,	// EMIF2_EXT_PHY_CTRL_8				
	0x00320032,	// EMIF2_EXT_PHY_CTRL_9				
	0x00320032,	// EMIF2_EXT_PHY_CTRL_10				
	0x00320032,	// EMIF2_EXT_PHY_CTRL_11				
	0x00600060,	// EMIF2_EXT_PHY_CTRL_12				
	0x00600064,	// EMIF2_EXT_PHY_CTRL_13				
	0x00600069,	// EMIF2_EXT_PHY_CTRL_14				
	0x0060006C,	// EMIF2_EXT_PHY_CTRL_15				
	0x00600060,	// EMIF2_EXT_PHY_CTRL_16				
	0x00400040,	// EMIF2_EXT_PHY_CTRL_17				
	0x00400044,	// EMIF2_EXT_PHY_CTRL_18				
	0x00400049,	// EMIF2_EXT_PHY_CTRL_19				
	0x0040004C,	// EMIF2_EXT_PHY_CTRL_20				
	0x00400040,	// EMIF2_EXT_PHY_CTRL_21				
	0x00800080,	// EMIF2_EXT_PHY_CTRL_22				
	0x00800080,	// EMIF2_EXT_PHY_CTRL_23				
	0x40010080,	// EMIF2_EXT_PHY_CTRL_24				
	0x08102040,	// EMIF2_EXT_PHY_CTRL_25				
	0x005B0077,	// EMIF2_EXT_PHY_CTRL_26				
	0x005B0073,	// EMIF2_EXT_PHY_CTRL_27				
	0x005B007C,	// EMIF2_EXT_PHY_CTRL_28				
	0x005B0079,	// EMIF2_EXT_PHY_CTRL_29				
	0x005B005B,	// EMIF2_EXT_PHY_CTRL_30				
	0x00300030,	// EMIF2_EXT_PHY_CTRL_31				
	0x00300034,	// EMIF2_EXT_PHY_CTRL_32				
	0x00300039,	// EMIF2_EXT_PHY_CTRL_33				
	0x0030003C,	// EMIF2_EXT_PHY_CTRL_34				
	0x00300030,	// EMIF2_EXT_PHY_CTRL_35				
	0x00000077	// EMIF2_EXT_PHY_CTRL_36		
};

/*
* DRA72x(Am571x) family soc DDR configuration.
*/
static const struct emif_regs am571x_ddr3_532mhz_emif_regs = {
};
static const u32 am571x_ddr3_ext_phy_ctrl_const_regs[] = {
};

struct vcores_data am57xx_volts = {
	.mpu.value		= VDD_MPU_DRA7,
	.mpu.efuse.reg		= STD_FUSE_OPP_VMIN_MPU,
	.mpu.efuse.reg_bits     = DRA752_EFUSE_REGBITS,
	.mpu.addr		= TPS659038_REG_ADDR_SMPS12,
	.mpu.pmic		= &tps659038,
	.mpu.abb_tx_done_mask = OMAP_ABB_MPU_TXDONE_MASK,

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


#endif /* _MUX_GF33A1_H_ */
