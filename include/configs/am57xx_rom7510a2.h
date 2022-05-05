/*
 * (C) Copyright 2014
 * Texas Instruments Incorporated.
 * Felipe Balbi <balbi@ti.com>
 *
 * Configuration settings for the TI Beagle x15 board.
 * See ti_omap5_common.h for omap5 common settings.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_AM57XX_EVM_H
#define __CONFIG_AM57XX_EVM_H

/*
* boot from uart,then load uboot.img from sd
*/
/*
#define CONFIG_UART_BOOT_MLO
*/

#define CONFIG_AM57XX
#ifdef CONFIG_WATCHDOG
#define CONFIG_SPL_WATCHDOG_SUPPORT
#endif

#define CONFIG_BCB_SUPPORT
#define CONFIG_ADV_OTA_SUPPORT
#define CONFIG_CMD_READ

#define CONFIG_IODELAY_RECALIBRATION

#define CONFIG_BOARD_EARLY_INIT_F

/* SPI SPL boot support. */
#define CONFIG_SPL_SPI_SUPPORT
#define CONFIG_MMC_DEVICE_MAX 2
#define CONFIG_SPL_PARAM_ADDR	(0XC1000000)

/* SATA Boot related defines */
#define CONFIG_SPL_SATA_SUPPORT
#define CONFIG_SPL_SATA_BOOT_DEVICE		0
#define CONFIG_SYS_SATA_FAT_BOOT_PARTITION	1

#define CONFIG_NR_DRAM_BANKS		2

#define CONSOLEDEV			"ttyS2"
#define CONFIG_SYS_NS16550_COM1		UART1_BASE	/* Base EVM has UART0 */
#define CONFIG_SYS_NS16550_COM2		UART2_BASE	/* UART2 */
#define CONFIG_SYS_NS16550_COM3		UART3_BASE	/* UART3 */
#define CONFIG_BAUDRATE			115200

#define CONFIG_SYS_OMAP_ABE_SYSCK

/* Define the default GPT table for eMMC */
#define PARTS_DEFAULT \
	"uuid_disk=${uuid_gpt_disk};" \
	"name=rootfs,start=2MiB,size=-,uuid=${uuid_gpt_rootfs}"

#define ETHADDR \
	"ethaddr=00:01:02:03:04:05\0"

#include <configs/ti_omap5_common.h>

/* QSPI */
/* Change the Omap3 spi to ti Qspi. */
#ifdef CONFIG_OMAP3_SPI
#undef CONFIG_OMAP3_SPI
#endif
#define CONFIG_TI_QSPI
#define CONFIG_SUPPORT_SF
#define CONFIG_CMD_SF			/* Support SPI flash operation.*/
#define CONFIG_QSPI_QUAD_SUPPORT
#define CONFIG_SPI_FLASH_WINBOND
#define CONFIG_SF_DEFAULT_SPEED		(50000000)
#define CONFIG_SF_DEFAULT_MODE		SPI_MODE_0
#define CONFIG_SF_DEFAULT_BUS		0
#define CONFIG_SF_DEFAULT_CS		0

/* MAC */
#define CONFIG_MAC_IN_QSPI			(1)
#define CONFIG_MAC_OFFSET			(0x300000)

/* ENV SETTING */
#define CONFIG_ENV_IS_IN_SPI_FLASH
#define CONFIG_ENV_SIZE			(64 << 10)
#define CONFIG_ENV_OFFSET		(0x00310000)
#define CONFIG_ENV_SECT_SIZE		(64 << 10)
#define CONFIG_ENV_SPI_BUS		0
#define CONFIG_ENV_SPI_CS		0
#define CONFIG_ENV_SPI_MODE		SPI_MODE_0
#define CONFIG_ENV_SPI_MAX_HZ		CONFIG_SF_DEFAULT_SPEED

/* Enhance our eMMC support / experience. */
#define CONFIG_CMD_GPT
#define CONFIG_EFI_PARTITION

/* CPSW Ethernet */
#define CONFIG_BOOTP_DNS		/* Configurable parts of CMD_DHCP */
#define CONFIG_BOOTP_DNS2
#define CONFIG_BOOTP_SEND_HOSTNAME
#define CONFIG_BOOTP_GATEWAY
#define CONFIG_BOOTP_SUBNETMASK
#define CONFIG_NET_RETRY_COUNT		10
#define CONFIG_DRIVER_TI_CPSW		/* Driver for IP block */
#define CONFIG_MII			/* Required in net/eth.c */
#define CONFIG_PHY_GIGE			/* per-board part of CPSW */
#define CONFIG_PHYLIB
#define PHY_ANEG_TIMEOUT	8000	/* PHY needs longer aneg time at 1G */
/*
#define CONFIG_ENABLE_EPHY0
#define CONFIG_EPHY0_PHY_ADDR		0
*/
#define CONFIG_ENABLE_EPHY1
#define CONFIG_EPHY1_PHY_ADDR		4
#define CONFIG_PHY_REALTEK
#define PHY_RESET_GPIO	87

#define CONFIG_SUPPORT_EMMC_BOOT

/* USB xHCI HOST */
#define CONFIG_USB_HOST
#define CONFIG_USB_XHCI_DWC3
#define CONFIG_USB_XHCI
#define CONFIG_USB_XHCI_OMAP
#define CONFIG_USB_STORAGE
#define CONFIG_SYS_USB_XHCI_MAX_ROOT_PORTS 2

#define CONFIG_OMAP_USB_PHY
#define CONFIG_OMAP_USB3PHY1_HOST

/* SATA */
#define CONFIG_BOARD_LATE_INIT
#define CONFIG_CMD_SCSI
#define CONFIG_LIBATA
#define CONFIG_SCSI_AHCI
#define CONFIG_SCSI_AHCI_PLAT
#define CONFIG_SYS_SCSI_MAX_SCSI_ID	1
#define CONFIG_SYS_SCSI_MAX_LUN		1
#define CONFIG_SYS_SCSI_MAX_DEVICE	(CONFIG_SYS_SCSI_MAX_SCSI_ID * \
						CONFIG_SYS_SCSI_MAX_LUN)
/* I2C */
/* 
* we will give up the macro in future. Because it has low flexibility.
*/
#define CONFIG_SYS_SPD_BUS_NUM I2C_BUS
/* rtc i2c bus num */
#define CONFIG_SYS_RTC_BUS_NUM 0

#define CONFIG_PMIC_CHIP_ADDRESS 0x58
#define CONFIG_AM57XX_PMIC_BUS_ADDRESS 0

/* We don't support SPL boot OS .*/
#ifdef CONFIG_SPL_OS_BOOT
#undef CONFIG_SPL_OS_BOOT 
#endif

/* U-Boot memtest setup */
/* Begin and end addresses of the area used by the simple memory test */
#define CONFIG_SYS_MEMTEST_START	0xa0000000
#define CONFIG_SYS_MEMTEST_END	0xd0000000
#endif /* __CONFIG_AM57XX_EVM_H */
