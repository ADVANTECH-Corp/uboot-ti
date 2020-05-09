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

#ifndef __CONFIG_AM57XX_ROM7510A1_H
#define __CONFIG_AM57XX_ROM7510A1_H

#include <environment/ti/dfu.h>
#include <linux/sizes.h>

/*#define DEBUG*/

#define CONFIG_IODELAY_RECALIBRATION

#define CONFIG_NR_DRAM_BANKS		2

#define CONFIG_ADVANTECH_EMMC_BOOT
#define CONFIG_SPL_PARAM_ADDR (0XC1000000)

#define CONSOLEDEV			"ttyO2"
#define CONFIG_SYS_NS16550_COM1		UART1_BASE	/* Base EVM has UART0 */
#define CONFIG_SYS_NS16550_COM2		UART2_BASE	/* UART2 */
#define CONFIG_SYS_NS16550_COM3		UART3_BASE	/* UART3 */

#define CONFIG_SYS_OMAP_ABE_SYSCK

/* Define the default GPT table for eMMC */
#define PARTS_DEFAULT \
	/* Linux partitions */ \
	"uuid_disk=${uuid_gpt_disk};" \
	"name=bootloader,start=384K,size=1792K,uuid=${uuid_gpt_bootloader};" \
	"name=rootfs,start=2688K,size=-,uuid=${uuid_gpt_rootfs}\0" \
	/* Android partitions */ \
	"partitions_android=" \
	"uuid_disk=${uuid_gpt_disk};" \
	"name=xloader,start=128K,size=256K,uuid=${uuid_gpt_xloader};" \
	"name=bootloader,size=1792K,uuid=${uuid_gpt_bootloader};" \
	"name=environment,size=128K,uuid=${uuid_gpt_environment};" \
	"name=misc,size=128K,uuid=${uuid_gpt_misc};" \
	"name=reserved,size=256K,uuid=${uuid_gpt_reserved};" \
	"name=efs,size=16M,uuid=${uuid_gpt_efs};" \
	"name=crypto,size=16K,uuid=${uuid_gpt_crypto};" \
	"name=recovery,size=40M,uuid=${uuid_gpt_recovery};" \
	"name=boot,size=10M,uuid=${uuid_gpt_boot};" \
	"name=system,size=768M,uuid=${uuid_gpt_system};" \
	"name=vendor,size=256M,uuid=${uuid_gpt_vendor};" \
	"name=cache,size=256M,uuid=${uuid_gpt_cache};" \
	"name=ipu1,size=1M,uuid=${uuid_gpt_ipu1};" \
	"name=ipu2,size=1M,uuid=${uuid_gpt_ipu2};" \
	"name=userdata,size=-,uuid=${uuid_gpt_userdata}"

#define DFUARGS \
	"dfu_bufsiz=0x10000\0" \
	DFU_ALT_INFO_MMC \
	DFU_ALT_INFO_EMMC \
	DFU_ALT_INFO_RAM \

#include <configs/ti_omap5_common.h>

/* Enhance our eMMC support / experience. */
#define CONFIG_HSMMC2_8BIT

/* CPSW Ethernet */
#define CONFIG_BOOTP_DNS		/* Configurable parts of CMD_DHCP */
#define CONFIG_BOOTP_DNS2
#define CONFIG_BOOTP_SEND_HOSTNAME
#define CONFIG_BOOTP_GATEWAY
#define CONFIG_BOOTP_SUBNETMASK
#define CONFIG_NET_RETRY_COUNT		10
#define CONFIG_DRIVER_TI_CPSW		/* Driver for IP block */
#define CONFIG_MII			/* Required in net/eth.c */
#define PHY_ANEG_TIMEOUT	8000	/* PHY needs longer aneg time at 1G */

#define CONFIG_SUPPORT_EMMC_BOOT

/* USB xHCI HOST */
#define CONFIG_USB_XHCI_OMAP

#define CONFIG_OMAP_USB_PHY
#define CONFIG_OMAP_USB3PHY1_HOST

/* SATA */
#define CONFIG_SCSI_AHCI_PLAT
#define CONFIG_SYS_SCSI_MAX_SCSI_ID	1
#define CONFIG_SYS_SCSI_MAX_LUN		1
#define CONFIG_SYS_SCSI_MAX_DEVICE	(CONFIG_SYS_SCSI_MAX_SCSI_ID * \
						CONFIG_SYS_SCSI_MAX_LUN)

/* SPI */
#define CONFIG_QSPI_QUAD_SUPPORT
#define CONFIG_SF_DEFAULT_SPEED                (48000000)
#define CONFIG_SF_DEFAULT_MODE                 SPI_MODE_0
#define CONFIG_QSPI_QUAD_SUPPORT
#define CONFIG_SF_DEFAULT_BUS			0
#define CONFIG_SF_DEFAULT_CS			0

/* SPI ENV related defines */
#define CONFIG_ENV_SPI_MAX_HZ           CONFIG_SF_DEFAULT_SPEED
#define CONFIG_ENV_SIZE                 (64 << 10)
#define CONFIG_ENV_SECT_SIZE            (64 << 10) /* 64 KB sectors */
#define CONFIG_ENV_OFFSET               0x00310000
#define CONFIG_ENV_SPI_BUS             0
#define CONFIG_ENV_SPI_CS              0

/* MAC */
#define CONFIG_MAC_IN_QSPI                      (1)
#define CONFIG_MAC_OFFSET                       (0x300000)

#endif /* __CONFIG_AM57XX_ROM7510A1_H */
