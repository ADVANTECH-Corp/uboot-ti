// SPDX-License-Identifier: GPL-2.0+
/*
 * DDRSS: DDR 1-bit inline ECC test command
 *
 * Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com/
 */

#include <asm/io.h>
#include <asm/cache.h>
#include <common.h>
#include <command.h>
#include <cpu_func.h>
#include <linux/bitops.h>

DECLARE_GLOBAL_DATA_PTR;

#define K3_DDRSS_MAX_ECC_REGIONS	3

#define DDRSS_BASE			0x0f300000 /* AM62/AM64 */
#define DDRSS_V2A_CTL_REG		0x0020
#define DDRSS_V2A_INT_RAW_REG		0x00a0
#define DDRSS_V2A_INT_STAT_REG		0x00a4
#define DDRSS_V2A_INT_ECC1BERR		BIT(3)
#define DDRSS_V2A_INT_ECC2BERR		BIT(4)
#define DDRSS_V2A_INT_ECCM1BERR		BIT(5)
#define DDRSS_ECC_CTRL_REG		0x0120
#define DDRSS_ECC_CTRL_REG_ECC_EN	BIT(0)
#define DDRSS_ECC_CTRL_REG_RMW_EN	BIT(1)
#define DDRSS_ECC_CTRL_REG_ECC_CK	BIT(2)
#define DDRSS_ECC_CTRL_REG_WR_ALLOC	BIT(4)
#define DDRSS_ECC_R0_STR_ADDR_REG	0x0130
#define DDRSS_ECC_Rx_STR_ADDR_REG(x)	(0x0130 + ((x) * 8))
#define DDRSS_ECC_Rx_END_ADDR_REG(x)	(0x0134 + ((x) * 8))
#define DDRSS_ECC_1B_ERR_CNT_REG	0x0150
#define DDRSS_ECC_1B_ERR_THRSH_REG	0x0154
#define DDRSS_ECC_1B_ERR_ADR_REG	0x0158
#define DDRSS_ECC_1B_ERR_MSK_LOG_REG	0x015c

static inline u32 ddrss_read(u32 reg)
{
	return readl((unsigned long)(DDRSS_BASE + reg));
}

static inline void ddrss_write(u32 value, u32 reg)
{
	writel(value, (unsigned long)(DDRSS_BASE + reg));
}

/* ddrss_check_ecc_status()
 * Report the ECC state after test. Check/clear the interrupt
 * status register, dump the ECC err counters and ECC error offset.
 */
static void ddrss_check_ecc_status(void)
{
	u32 ecc_1b_err_cnt, v2a_int_raw, ecc_1b_err_msk;
	phys_addr_t ecc_1b_err_adr;

	v2a_int_raw = ddrss_read(DDRSS_V2A_INT_RAW_REG);

	/* 1-bit correctable */
	if (v2a_int_raw & DDRSS_V2A_INT_ECC1BERR) {
		puts("\tECC test: DDR ECC 1-bit error\n");

		/* Dump the 1-bit counter and reset it, as we want a
		 * new interrupt to be generated when above the error
		 * threshold
		 */
		ecc_1b_err_cnt = ddrss_read(DDRSS_ECC_1B_ERR_CNT_REG);
		if (ecc_1b_err_cnt) {
			printf("\tECC test: 1-bit ECC err count: %u\n",
			       ecc_1b_err_cnt & 0xffff);
			ddrss_write(1, DDRSS_ECC_1B_ERR_CNT_REG);
		}

		/* ECC fault addresses are also recorded in a 2-word deep
		 * FIFO. Calculate and report the 8-byte range of the error
		 */
		ecc_1b_err_adr = ddrss_read(DDRSS_ECC_1B_ERR_ADR_REG);
		ecc_1b_err_msk = ddrss_read(DDRSS_ECC_1B_ERR_MSK_LOG_REG);
		if (ecc_1b_err_msk) {
			/* AM64/AM62:
			 * The address of the ecc error is 16-byte aligned.
			 * Each bit in 4 bit mask represents 8 bytes ECC quanta
			 * that has the 1-bit error
			 */
			ecc_1b_err_msk &= 0xf;
			ecc_1b_err_adr <<= 4;
			ecc_1b_err_adr += (fls(ecc_1b_err_msk) - 1) * 8;
			printf("\tECC test: 1-bit error in [0x%llx:0x%llx]\n",
			       ecc_1b_err_adr, ecc_1b_err_adr + 8);
			/* Pop the top of the addr/mask FIFOs */
			ddrss_write(1, DDRSS_ECC_1B_ERR_ADR_REG);
			ddrss_write(1, DDRSS_ECC_1B_ERR_MSK_LOG_REG);
		}
		ddrss_write(DDRSS_V2A_INT_ECC1BERR, DDRSS_V2A_INT_STAT_REG);
	}

	/* 2-bit uncorrectable */
	if (v2a_int_raw & DDRSS_V2A_INT_ECC2BERR) {
		puts("\tECC test: DDR ECC 2-bit error\n");
		ddrss_write(DDRSS_V2A_INT_ECC2BERR, DDRSS_V2A_INT_STAT_REG);
	}

	/* multiple 1-bit errors (uncorrectable) in multpile words */
	if (v2a_int_raw & DDRSS_V2A_INT_ECCM1BERR) {
		puts("\tECC test: DDR ECC multi 1-bit errors\n");
		ddrss_write(DDRSS_V2A_INT_ECCM1BERR, DDRSS_V2A_INT_STAT_REG);
	}
}

/* ddrss_memory_ecc_err()
 * Simulate an ECC error - change a 32b word at address in an ECC enabled
 * range. This removes the tested address from the ECC checks, changes a
 * word, and then restores the ECC range as configured by k3_ddrss in R5 SPL.
 */
static int ddrss_memory_ecc_err(u32 addr, u32 ecc_err, int range)
{
	u32 ecc_start_addr, ecc_end_addr, ecc_temp_addr;
	u32 val1, val2, val3;

	/* Flush and disable dcache */
	flush_dcache_all();
	dcache_disable();

	/* Setup a threshold for 1-bit errors to generate interrupt */
	ddrss_write(1, DDRSS_ECC_1B_ERR_THRSH_REG);

	puts("Testing DDR ECC:\n");
	/* Get the Rx range configuration */
	ecc_start_addr = ddrss_read(DDRSS_ECC_Rx_STR_ADDR_REG(range));
	ecc_end_addr = ddrss_read(DDRSS_ECC_Rx_END_ADDR_REG(range));

	/* Calculate the end of the Rx ECC region up to the tested address */
	ecc_temp_addr = (addr - gd->ram_base) >> 16;
	if (ecc_temp_addr)
		ecc_temp_addr--;
	puts("\tECC test: Disabling DDR ECC ...\n");
	/* Set the new range */
	ddrss_write(ecc_temp_addr, DDRSS_ECC_Rx_END_ADDR_REG(range));
	/* ECC is still on in a single block. The range is disabled if start > end */
	if (ecc_temp_addr == ecc_start_addr)
		ddrss_write(ecc_temp_addr + 1, DDRSS_ECC_Rx_STR_ADDR_REG(range));

	/* Flip some bits, one bit preferably, but let's allow more */
	addr &= ~3;
	val1 = readl((unsigned long)addr);
	val2 = val1 ^ ecc_err;
	writel(val2, (unsigned long)addr);
	val3 = readl((unsigned long)addr);

	/* Re-enable the ECC checks for the R0 region */
	ddrss_write(ecc_end_addr, DDRSS_ECC_Rx_END_ADDR_REG(range));
	ddrss_write(ecc_start_addr, DDRSS_ECC_Rx_STR_ADDR_REG(range));
	/* Make sure the ECC range is restored before doing anything else */
	mb();

	printf("\tECC test: addr 0x%x, read data 0x%x, written data 0x%x, "
	       "err pattern: 0x%x, read after write data 0x%x\n",
	       addr, val1, val2, ecc_err, val3);

	puts("\tECC test: Enabled DDR ECC ...\n");
	/* Read again from the address. This creates an ECC 1-bit error
	 * condition, and returns the corrected value
	 */
	val1 = readl((unsigned long)addr);
	printf("\tECC test: addr 0x%x, read data 0x%x\n", addr, val1);

	/* Set threshold for 1-bit errors to 0 to disable the interrupt */
	ddrss_write(0, DDRSS_ECC_1B_ERR_THRSH_REG);
	/* Report the ECC status */
	ddrss_check_ecc_status();

	dcache_enable();

	return 0;
}

/* ddrss_addr_in_range()
 * Check if the address is valid and within any configured ECC R[0-2] range.
 */
static int ddrss_addr_in_range(phys_addr_t addr)
{
	phys_addr_t ecc_start_addr, ecc_end_addr;
	int i;

	if (addr < gd->ram_base || addr >= gd->ram_top)
		return -1;

	addr -= gd->ram_base;
	/* Find if any of the enabled ECC Rx ranges contains the address.
	 * Only a single range (R0) is usually enabled by k3-ddrss in R5 SPL
	 * for the full memory.
	 */
	for (i = 0; i < K3_DDRSS_MAX_ECC_REGIONS; i++) {
		ecc_start_addr = ddrss_read(DDRSS_ECC_Rx_STR_ADDR_REG(i));
		ecc_end_addr = ddrss_read(DDRSS_ECC_Rx_END_ADDR_REG(i));
		/* Not configured/disabled */
		if (ecc_start_addr >= ecc_end_addr)
			continue;
		ecc_start_addr <<= 16;
		ecc_end_addr = (ecc_end_addr << 16) + 0xffff;

		if (addr >= ecc_start_addr && addr <= ecc_end_addr)
			return i;
	}

	return -1;
}

/* ddrss_is_ecc_enabled()
 * Report if ECC is enabled.
 */
static int ddrss_is_ecc_enabled(void)
{
	u32 ecc_ctrl = ddrss_read(DDRSS_ECC_CTRL_REG);

	/* Assume ECC is enabled only if all bits set by k3_ddrss are set */
	return (ecc_ctrl & (DDRSS_ECC_CTRL_REG_ECC_EN |
			    DDRSS_ECC_CTRL_REG_RMW_EN |
			    DDRSS_ECC_CTRL_REG_WR_ALLOC |
			    DDRSS_ECC_CTRL_REG_ECC_CK));
}

static int do_ddrss_test(struct cmd_tbl *cmdtp, int flag, int argc,
			 char *const argv[])
{
	u32 start_addr, ecc_err;
	int range;

	if (!(argc == 4 && (strncmp(argv[1], "ecc_err", 8) == 0)))
		return cmd_usage(cmdtp);

	if (!ddrss_is_ecc_enabled()) {
		puts("ECC not enabled. Please enable ECC any try again\n");
		return CMD_RET_FAILURE;
	}

	start_addr = simple_strtoul(argv[2], NULL, 16);
	ecc_err = simple_strtoul(argv[3], NULL, 16);

	range = ddrss_addr_in_range(start_addr);
	if (range < 0) {
		printf("Address 0x%x not in any configured ECC range\n",
		       start_addr);
		return CMD_RET_FAILURE;
	}

	ddrss_memory_ecc_err(start_addr, ecc_err, range);
	return 0;
}

U_BOOT_CMD(ddrss, 5, 1, do_ddrss_test,
	   "DDRSS test",
	   "ecc_err <addr in hex> <bit_err in hex> - generate bit errors\n"
	   "	in DDR data at <addr>, the command will read a 32-bit data\n"
	   "	from <addr>, and write (data ^ bit_err) back to <addr>\n"
);
