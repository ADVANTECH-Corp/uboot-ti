/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Configuration header file for K3 AM62x SoC family
 *
 * Copyright (C) 2020-2022 Texas Instruments Incorporated - https://www.ti.com/
 *	Suman Anna <s-anna@ti.com>
 */

#ifndef __CONFIG_AM62x_RS10_H
#define __CONFIG_AM62x_RS10_H

#include <linux/sizes.h>
#include <config_distro_bootcmd.h>
#include <environment/ti/mmc.h>
#include <environment/ti/k3_dfu.h>
#include <environment/ti/k3_rproc.h>

/* DDR Configuration */
#define CONFIG_SYS_SDRAM_BASE1		0x880000000
#define CONFIG_SYS_BOOTM_LEN            SZ_64M

/* NAND support */

/* NAND Device Configuration : MT29F8G08ADAFAH4 chip */
#define CONFIG_SYS_NAND_PAGE_SIZE	4096
#define CONFIG_SYS_NAND_OOBSIZE		256
#define CONFIG_SYS_NAND_BLOCK_SIZE      SZ_256K
#define CONFIG_SYS_NAND_PAGE_COUNT	(CONFIG_SYS_NAND_BLOCK_SIZE / \
					 CONFIG_SYS_NAND_PAGE_SIZE)
#define CONFIG_SYS_NAND_5_ADDR_CYCLE

/* NAND Driver config */
#define CONFIG_SPL_NAND_INIT	1
#define CONFIG_SYS_NAND_ONFI_DETECTION
#define CONFIG_NAND_OMAP_ECCSCHEME      OMAP_ECC_BCH8_CODE_HW
#define CONFIG_SYS_NAND_BAD_BLOCK_POS   NAND_LARGE_BADBLOCK_POS

#define CONFIG_SYS_NAND_ECCPOS		{ 2, 3, 4, 5, 6, 7, 8, 9, \
					 10, 11, 12, 13, 14, 15, 16, 17, \
					 18, 19, 20, 21, 22, 23, 24, 25, \
					 26, 27, 28, 29, 30, 31, 32, 33, \
					 34, 35, 36, 37, 38, 39, 40, 41, \
					 42, 43, 44, 45, 46, 47, 48, 49, \
					 50, 51, 52, 53, 54, 55, 56, 57, }
#define CONFIG_SYS_NAND_ECCSIZE         512
#define CONFIG_SYS_NAND_ECCBYTES        14

#ifdef CONFIG_SYS_K3_SPL_ATF
#define CONFIG_SYS_NAND_U_BOOT_OFFS     0x200000	/* tispl.bin partition */
#else
#define CONFIG_SYS_NAND_U_BOOT_OFFS     0x600000	/* u-boot.img partition */
#endif

#define CONFIG_SYS_NAND_MAX_CHIPS       1

#define CONFIG_SYS_MAX_NAND_DEVICE      1

#define CONFIG_SYS_NAND_BASE            0x51000000

#if defined(CONFIG_ENV_IS_IN_NAND)
#define CONFIG_SYS_ENV_SECT_SIZE        CONFIG_SYS_NAND_BLOCK_SIZE
#endif

/*-- end NAND config --*/

#ifdef CONFIG_SYS_K3_SPL_ATF
#define CONFIG_SPL_FS_LOAD_PAYLOAD_NAME	"tispl.bin"
#endif

#if defined(CONFIG_TARGET_AM62x_A53_RS10)
#define CONFIG_SPL_MAX_SIZE		SZ_1M
#define CONFIG_SYS_INIT_SP_ADDR         (CONFIG_SPL_TEXT_BASE + SZ_4M)
#else
#define CONFIG_SPL_MAX_SIZE		CONFIG_SYS_K3_MAX_DOWNLODABLE_IMAGE_SIZE
/*
 * Maximum size in memory allocated to the SPL BSS. Keep it as tight as
 * possible (to allow the build to go through), as this directly affects
 * our memory footprint. The less we use for BSS the more we have available
 * for everything else.
 */
#define CONFIG_SPL_BSS_MAX_SIZE		0x3000
/*
 * Link BSS to be within SPL in a dedicated region located near the top of
 * the MCU SRAM, this way making it available also before relocation. Note
 * that we are not using the actual top of the MCU SRAM as there is a memory
 * location allocated for Device Manager data and some memory filled in by
 * the boot ROM that we want to read out without any interference from the
 * C context.
 */
#define CONFIG_SPL_BSS_START_ADDR	(0x43c3e000 -\
					 CONFIG_SPL_BSS_MAX_SIZE)
/* Set the stack right below the SPL BSS section */
#define CONFIG_SYS_INIT_SP_ADDR         0x43c3a7f0
/* Configure R5 SPL post-relocation malloc pool in DDR */
#define CONFIG_SYS_SPL_MALLOC_START    0x84000000
#define CONFIG_SYS_SPL_MALLOC_SIZE     SZ_16M
#endif

#define PARTS_DEFAULT \
	/* Linux partitions */ \
	"uuid_disk=${uuid_gpt_disk};" \
	"name=rootfs,start=0,size=-,uuid=${uuid_gpt_rootfs}\0" \
	/* Android partitions */ \
	"partitions_android=" \
	"uuid_disk=${uuid_gpt_disk};" \
	"name=bootloader,start=5M,size=8M,uuid=${uuid_gpt_bootloader};" \
	"name=tiboot3,start=4M,size=1M,uuid=${uuid_gpt_tiboot3};" \
	"name=uboot-env,start=13M,size=512K,uuid=${uuid_gpt_env};" \
	"name=misc,start=13824K,size=512K,uuid=${uuid_gpt_misc};" \
	"name=boot_a,size=40M,uuid=${uuid_gpt_boot_a};" \
	"name=boot_b,size=40M,uuid=${uuid_gpt_boot_b};" \
	"name=dtbo_a,size=8M,uuid=${uuid_gpt_dtbo_a};" \
	"name=dtbo_b,size=8M,uuid=${uuid_gpt_dtbo_b};" \
	"name=vbmeta_a,size=64K,uuid=${uuid_gpt_vbmeta_a};" \
	"name=vbmeta_b,size=64K,uuid=${uuid_gpt_vbmeta_b};" \
	"name=super,size=4608M,uuid=${uuid_gpt_super};" \
	"name=metadata,size=16M,uuid=${uuid_gpt_metadata};" \
	"name=persist,size=32M,uuid=${uuid_gpt_persist};" \
	"name=userdata,size=-,uuid=${uuid_gpt_userdata}\0"

/* U-Boot general configuration */
#define EXTRA_ENV_AM625_BOARD_SETTINGS					\
	"default_device_tree=" CONFIG_DEFAULT_DEVICE_TREE ".dtb\0"	\
	"findfdt="							\
		"setenv name_fdt ${default_device_tree};"		\
		"if test $board_name = am62x_rs10; then "		\
			"setenv name_fdt k3-am62x-rs10.dtb; fi;"		\
		"if test $board_name = am62x_lp_rs10; then "		\
			"setenv name_fdt k3-am62x-lp-rs10.dtb; fi;"	\
		"setenv fdtfile ${name_fdt}\0"				\
	"name_kern=Image\0"						\
	"console=ttyS2,115200n8\0"					\
	"args_all=setenv optargs ${optargs} "				\
		"earlycon=ns16550a,mmio32,0x02800000 ${mtdparts}\0"	\
	"run_kern=booti ${loadaddr} ${rd_spec} ${fdtaddr}\0"

/* U-Boot MMC-specific configuration */
#define EXTRA_ENV_AM625_BOARD_SETTINGS_MMC				\
	"boot=mmc\0"							\
	"mmcdev=1\0"							\
	"bootpart=1:2\0"						\
	"bootdir=/boot\0"						\
	"rd_spec=-\0"							\
	"init_mmc=run args_all args_mmc\0"				\
	"get_fdt_mmc=load mmc ${bootpart} ${fdtaddr} ${bootdir}/${name_fdt}\0" \
	"get_overlay_mmc="						\
		"fdt address ${fdtaddr};"				\
		"fdt resize 0x100000;"					\
		"for overlay in $name_overlays;"			\
		"do;"							\
		"load mmc ${bootpart} ${dtboaddr} ${bootdir}/${overlay} && "	\
		"fdt apply ${dtboaddr};"				\
		"done;\0"						\
	"get_kern_mmc=load mmc ${bootpart} ${loadaddr} "		\
		"${bootdir}/${name_kern}\0"				\
	"get_fit_mmc=load mmc ${bootpart} ${addr_fit} "			\
		"${bootdir}/${name_fit}\0"				\
	"partitions=" PARTS_DEFAULT

#define EXTRA_ENV_AM625_BOARD_SETTING_USBMSC				\
	"args_usb=run finduuid;setenv bootargs console=${console} "	\
		"${optargs} "						\
		"root=PARTUUID=${uuid} rw "				\
		"rootfstype=${mmcrootfstype}\0"				\
	"init_usb=run args_all args_usb\0"				\
	"get_fdt_usb=load usb ${bootpart} ${fdtaddr} ${bootdir}/${fdtfile}\0"	\
	"get_overlay_usb="						\
		"fdt address ${fdtaddr};"				\
		"fdt resize 0x100000;"					\
		"for overlay in $name_overlays;"			\
		"do;"							\
		"load usb ${bootpart} ${dtboaddr} ${bootdir}/${overlay} && "	\
		"fdt apply ${dtboaddr};"				\
		"done;\0"						\
	"get_kern_usb=load usb ${bootpart} ${loadaddr} "		\
		"${bootdir}/${name_kern}\0"				\
	"get_fit_usb=load usb ${bootpart} ${addr_fit} "			\
		"${bootdir}/${name_fit}\0"				\
	"usbboot=setenv boot usb;"					\
		"setenv bootpart 0:2;"					\
		"usb start;"						\
		"run findfdt;"						\
		"run init_usb;"						\
		"run get_kern_usb;"					\
		"run get_fdt_usb;"					\
		"run run_kern\0"

#ifdef CONFIG_TARGET_AM62x_A53_RS10
#define EXTRA_ENV_AM625_BOARD_SETTINGS_MTD				\
	"mtdids=" CONFIG_MTDIDS_DEFAULT "\0"				\
	"mtdparts=" CONFIG_MTDPARTS_DEFAULT "\0"
#else
#define EXTRA_ENV_AM625_BOARD_SETTINGS_MTD
#endif

#define EXTRA_ENV_AM625_BOARD_SETTINGS_OSPI_NAND			\
	"nbootpart=ospi.rootfs\0"					\
	"nbootvolume=ubi0:rootfs\0"					\
	"bootdir=/boot\0"						\
	"rd_spec=-\0"							\
	"ubi_init=ubi part ${nbootpart}; ubifsmount ${nbootvolume};\0"	\
	"args_ospi_nand=setenv bootargs console=${console} "		\
		"${optargs} ubi.mtd=${nbootpart} "			\
		"root=${nbootvolume} rootfstype=ubifs\0"		\
	"init_ospi_nand=run args_all args_ospi_nand ubi_init\0"		\
	"get_fdt_ospi_nand=ubifsload ${fdtaddr} ${bootdir}/${fdtfile};\0"	\
	"get_overlay_ospi_nand="					\
		"fdt address ${fdtaddr};"				\
		"fdt resize 0x100000;"					\
		"for overlay in $name_overlays;"			\
		"do;"							\
		"ubifsload ${dtboaddr} ${bootdir}/${overlay} && "	\
		"fdt apply ${dtboaddr};"				\
		"done;\0"						\
	"get_kern_ospi_nand=ubifsload ${loadaddr} ${bootdir}/${name_kern}\0"	\
	"get_fit_ospi_nand=ubifsload ${addr_fit} ${bootdir}/${name_fit}\0"

#define EXTRA_ENV_AM625_BOARD_SETTINGS_NAND				\
	"nbootpart=NAND.file-system\0"					\
	"nbootvolume=ubi0:rootfs\0"					\
	"bootdir=/boot\0"						\
	"rd_spec=-\0"							\
	"ubi_init=ubi part ${nbootpart}; ubifsmount ${nbootvolume};\0"	\
	"args_nand=setenv bootargs console=${console} "			\
		"${optargs} ubi.mtd=${nbootpart} "			\
		"root=${nbootvolume} rootfstype=ubifs\0"			\
	"init_nand=run args_all args_nand ubi_init\0"			\
	"get_fdt_nand=ubifsload ${fdtaddr} ${bootdir}/${fdtfile};\0"	\
	"get_overlay_nand="						\
		"fdt address ${fdtaddr};"				\
		"fdt resize 0x100000;"					\
		"for overlay in $name_overlays;"			\
		"do;"							\
		"ubifsload ${dtboaddr} ${bootdir}/${overlay} && "	\
		"fdt apply ${dtboaddr};"				\
		"done;\0"						\
	"get_kern_nand=ubifsload ${loadaddr} ${bootdir}/${name_kern}\0"	\
	"get_fit_nand=ubifsload ${addr_fit} ${bootdir}/${name_fit}\0"

#if defined(CONFIG_TARGET_AM62x_A53_RS10)
#if defined(DEFAULT_RPROCS)
#undef DEFAULT_RPROCS
#endif
#define DEFAULT_RPROCS ""						\
		"0 /lib/firmware/am62-mcu-m4f0_0-fw "
#endif

#define BOOTENV_DEV_LINUX(devtypeu, devtypel, instance) \
	"bootcmd_linux=" \
		"if test \"${android_boot}\" -eq 0; then;" \
			"run findfdt; run envboot; run init_${boot}; run boot_rprocs;" \
			"if test ${boot_fit} -eq 1; then;" \
				"run get_fit_${boot}; run get_fit_${boot}; run get_overlaystring; run run_fit;"\
			"else;" \
				"run get_kern_${boot}; run get_fdt_${boot}; run get_overlay_${boot}; run run_kern;" \
			"fi;" \
		"fi\0"

#define BOOTENV_DEV_NAME_LINUX(devtypeu, devtypel, instance)	\
		"linux "

#if defined(CONFIG_ANDROID_BOOT_IMAGE)

/* ANDROID BOOT */
#ifndef BOOT_PARTITION
#define BOOT_PARTITION "boot"
#endif

#ifndef CONTROL_PARTITION
#define CONTROL_PARTITION "misc"
#endif

#if defined(CONFIG_CMD_AVB)
#define AVB_VERIFY_CHECK \
	"if test \"${force_avb}\" -eq 1; then " \
		"if run avb_verify; then " \
			"echo AVB verification OK.;" \
			"setenv bootargs \"$bootargs $avb_bootargs\";" \
		"else " \
			"echo AVB verification failed.;" \
		"exit; fi;" \
	"else " \
		"setenv bootargs \"$bootargs androidboot.verifiedbootstate=orange\";" \
		"echo Running without AVB...; "\
	"fi;"

#define AVB_VERIFY_CMD "avb_verify=avb init ${mmcdev}; avb verify $slot_suffix;\0"
#else
#define AVB_VERIFY_CHECK ""
#define AVB_VERIFY_CMD ""
#endif

#if defined(CONFIG_CMD_AB_SELECT)
#define ANDROIDBOOT_GET_CURRENT_SLOT_CMD "get_current_slot=" \
	"if part number mmc ${mmcdev} " CONTROL_PARTITION " control_part_number; " \
	"then " \
		"echo " CONTROL_PARTITION \
			" partition number:${control_part_number};" \
		"ab_select current_slot mmc ${mmcdev}:${control_part_number};" \
	"else " \
		"echo " CONTROL_PARTITION " partition not found;" \
	"fi;\0"

#define AB_SELECT_SLOT \
	"run get_current_slot; " \
	"if test -e \"${current_slot}\"; " \
	"then " \
		"setenv slot_suffix _${current_slot}; " \
	"else " \
		"echo current_slot not found;" \
		"exit;" \
	"fi;"

#define AB_SELECT_ARGS \
	"setenv bootargs_ab androidboot.slot_suffix=${slot_suffix}; " \
	"echo A/B cmdline addition: ${bootargs_ab};" \
	"setenv bootargs ${bootargs} ${bootargs_ab};"

#define AB_BOOTARGS " androidboot.force_normal_boot=1"
#define RECOVERY_PARTITION "boot"
#else
#define AB_SELECT_SLOT ""
#define AB_SELECT_ARGS " "
#define ANDROIDBOOT_GET_CURRENT_SLOT_CMD ""
#define AB_BOOTARGS " "
#define RECOVERY_PARTITION "recovery"
#endif

/*
 * Prepares complete device tree blob for current board (for Android boot).
 *
 * Boot image or recovery image should be loaded into $loadaddr prior to running
 * these commands. The logic of these commnads is next:
 *
 *   1. Read correct DTB for current SoC/board from boot image in $loadaddr
 *      to $fdtaddr
 *   2. Merge all needed DTBO for current board from 'dtbo' partition into read
 *      DTB
 *   3. User should provide $fdtaddr as 3rd argument to 'bootm'
 */
#define PREPARE_FDT \
	"echo Preparing FDT...; " \
	"if test $board_name = am62x_rs10; then " \
		"echo \"  Reading DTB for am62x_rs10...\"; " \
		"setenv dtb_index 0;" \
	"elif test $board_name = am62x_lp_rs10; then " \
		"echo \"  Reading DTB for am62x_lp_rs10...\"; " \
		"setenv dtb_index 1;" \
	"else " \
		"echo Error: Android boot is not supported for $board_name; " \
		"exit; " \
	"fi; " \
	"abootimg get dtb --index=$dtb_index dtb_start dtb_size; " \
	"cp.b $dtb_start $fdt_addr_r $dtb_size; " \
	"fdt addr $fdt_addr_r $fdt_size; " \
	"part start mmc ${mmcdev} dtbo${slot_suffix} dtbo_start; " \
	"part size mmc ${mmcdev} dtbo${slot_suffix} dtbo_size; " \
	"mmc read ${dtboaddr} ${dtbo_start} ${dtbo_size}; " \
	"echo \"  Applying DTBOs...\"; " \
	"adtimg addr $dtboaddr; " \
	"dtbo_idx=''; " \
	"for index in $dtbo_index; do " \
		"adtimg get dt --index=$index dtbo_addr; " \
		"fdt resize; " \
		"fdt apply $dtbo_addr; " \
		"if test $dtbo_idx = ''; then " \
			"dtbo_idx=${index}; " \
		"else " \
			"dtbo_idx=${dtbo_idx},${index}; " \
		"fi; " \
	"done; " \
	"setenv bootargs \"$bootargs androidboot.dtbo_idx=$dtbo_idx \"; "


#define BOOT_CMD "bootm ${loadaddr} ${loadaddr} ${fdt_addr_r};"

#define BOOTENV_DEV_FASTBOOT(devtypeu, devtypel, instance) \
	"bootcmd_fastboot=" \
		"if test \"${android_boot}\" -eq 1; then;" \
			"setenv run_fastboot 0;" \
			"if gpt verify mmc ${mmcdev} ${partitions}; then; " \
			"else " \
				"echo Broken MMC partition scheme;" \
				"setenv run_fastboot 1;" \
			"fi; " \
			"if test \"${run_fastboot}\" -eq 0; then " \
				"if bcb load " __stringify(CONFIG_FASTBOOT_FLASH_MMC_DEV) " " \
				CONTROL_PARTITION "; then " \
					"if bcb test command = bootonce-bootloader; then " \
						"echo BCB: Bootloader boot...; " \
						"bcb clear command; bcb store; " \
						"setenv run_fastboot 1;" \
					"elif bcb test command = boot-fastboot; then " \
						"echo BCB: fastboot userspace boot...; " \
						"setenv force_recovery 1;" \
					"fi; " \
				"else " \
					"echo Warning: BCB is corrupted or does not exist; " \
				"fi;" \
			"fi;" \
			"if test \"${run_fastboot}\" -eq 1; then " \
				"echo Running Fastboot...;" \
				"fastboot " __stringify(CONFIG_FASTBOOT_USB_DEV) "; " \
			"fi;" \
		"fi\0"

#define BOOTENV_DEV_NAME_FASTBOOT(devtypeu, devtypel, instance)	\
		"fastboot "

#define BOOTENV_DEV_RECOVERY(devtypeu, devtypel, instance) \
	"bootcmd_recovery=" \
		"if test \"${android_boot}\" -eq 1; then;" \
			"setenv run_recovery 0;" \
			"if bcb load " __stringify(CONFIG_FASTBOOT_FLASH_MMC_DEV) " " \
			CONTROL_PARTITION "; then " \
				"if bcb test command = boot-recovery; then; " \
					"echo BCB: Recovery boot...; " \
					"setenv run_recovery 1;" \
				"fi;" \
			"else " \
				"echo Warning: BCB is corrupted or does not exist; " \
			"fi;" \
			"if test \"${skip_recovery}\" -eq 1; then " \
				"echo Recovery skipped by environment;" \
				"setenv run_recovery 0;" \
			"fi;" \
			"if test \"${force_recovery}\" -eq 1; then " \
				"echo Recovery forced by environment;" \
				"setenv run_recovery 1;" \
			"fi;" \
			"if test \"${run_recovery}\" -eq 1; then " \
				"echo Running Recovery...;" \
				"mmc dev ${mmcdev};" \
				"setenv bootargs \"${bootargs} androidboot.serialno=${serial#}\";" \
				AB_SELECT_SLOT \
				AB_SELECT_ARGS \
				AVB_VERIFY_CHECK \
				"part start mmc ${mmcdev} " RECOVERY_PARTITION "${slot_suffix} boot_start;" \
				"part size mmc ${mmcdev} " RECOVERY_PARTITION "${slot_suffix} boot_size;" \
				"if mmc read ${loadaddr} ${boot_start} ${boot_size}; then " \
					PREPARE_FDT \
					"echo Running Android Recovery...;" \
					BOOT_CMD \
				"fi;" \
				"echo Failed to boot Android...;" \
				"reset;" \
			"fi;" \
		"fi\0"

#define BOOTENV_DEV_NAME_RECOVERY(devtypeu, devtypel, instance)	\
		"recovery "

#define BOOTENV_DEV_SYSTEM(devtypeu, devtypel, instance) \
	"bootcmd_system=" \
		"if test \"${android_boot}\" -eq 1; then;" \
			"echo Loading Android " BOOT_PARTITION " partition...;" \
			"mmc dev ${mmcdev};" \
			"setenv bootargs ${bootargs} androidboot.serialno=${serial#};" \
			AB_SELECT_SLOT \
			AB_SELECT_ARGS \
			AVB_VERIFY_CHECK \
			"part start mmc ${mmcdev} " BOOT_PARTITION "${slot_suffix} boot_start;" \
			"part size mmc ${mmcdev} " BOOT_PARTITION "${slot_suffix} boot_size;" \
			"if mmc read ${loadaddr} ${boot_start} ${boot_size}; then " \
				PREPARE_FDT \
				"setenv bootargs \"${bootargs} " AB_BOOTARGS "\"  ; " \
				"echo Running Android...;" \
				BOOT_CMD \
			"fi;" \
			"echo Failed to boot Android...;" \
		"fi\0"

#define BOOTENV_DEV_NAME_SYSTEM(devtypeu, devtypel, instance)	\
		"system "

#define BOOTENV_DEV_PANIC(devtypeu, devtypel, instance) \
	"bootcmd_panic=" \
		"if test \"${android_boot}\" -eq 1; then;" \
			"fastboot " __stringify(CONFIG_FASTBOOT_USB_DEV) "; " \
			"reset;" \
		"fi\0"

#define BOOTENV_DEV_NAME_PANIC(devtypeu, devtypel, instance)	\
		"panic "

#define EXTRA_ANDROID_ENV_SETTINGS                                     \
	"set_android_boot=setenv android_boot 1;setenv partitions $partitions_android;setenv mmcdev 0;setenv force_avb 0;saveenv;\0" \
	ANDROIDBOOT_GET_CURRENT_SLOT_CMD                              \
	AVB_VERIFY_CMD                                                \
	BOOTENV

#define BOOT_TARGET_DEVICES(func) \
	func(LINUX, linux, na) \
	func(FASTBOOT, fastboot, na) \
	func(RECOVERY, recovery, na) \
	func(SYSTEM, system, na) \
	func(PANIC, panic, na) \

#else

#define EXTRA_ANDROID_ENV_SETTINGS  ""
#define BOOT_TARGET_DEVICES(func) \
	func(LINUX, linux, na) \

#endif

#ifdef CONFIG_TARGET_AM62x_A53_RS10
#define EXTRA_ENV_DFUARGS \
	DFU_ALT_INFO_MMC \
	DFU_ALT_INFO_EMMC \
	DFU_ALT_INFO_RAM \
	DFU_ALT_INFO_OSPI \
	DFU_ALT_INFO_OSPI_NAND \
	DFU_ALT_INFO_GPMC_NAND
#else
#define EXTRA_ENV_DFUARGS
#endif

/* Incorporate settings into the U-Boot environment */
#define CONFIG_EXTRA_ENV_SETTINGS					\
	EXTRA_ANDROID_ENV_SETTINGS					\
	DEFAULT_LINUX_BOOT_ENV						\
	DEFAULT_FIT_TI_ARGS						\
	DEFAULT_MMC_TI_ARGS						\
	EXTRA_ENV_AM625_BOARD_SETTINGS					\
	EXTRA_ENV_AM625_BOARD_SETTINGS_MMC				\
	EXTRA_ENV_AM625_BOARD_SETTINGS_NAND				\
	EXTRA_ENV_DFUARGS						\
	EXTRA_ENV_AM625_BOARD_SETTING_USBMSC				\
	EXTRA_ENV_AM625_BOARD_SETTINGS_OSPI_NAND			\
	EXTRA_ENV_RPROC_SETTINGS

/* Now for the remaining common defines */
#include <configs/ti_armv7_common.h>

/* MMC ENV related defines */
#ifdef CONFIG_ENV_IS_IN_MMC
#define CONFIG_SYS_MMC_ENV_DEV		0
#define CONFIG_SYS_MMC_ENV_PART		1
#endif

#ifdef CONFIG_SYS_MALLOC_LEN
#undef CONFIG_SYS_MALLOC_LEN
#endif
#define CONFIG_SYS_MALLOC_LEN           SZ_128M

#define ADV_WDT_EN 34
#define POWER_LED 42

#endif /* __CONFIG_AM62x_RS10_H */
