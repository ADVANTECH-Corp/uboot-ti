/*
 * Copyright (C) 2016 Advantech Co.,Ltd 
 *
 * Author: yanwei.cao <yanwei.cao@advantech.com.cn>
 *
 * Based on drivers/watchdog/c8051_wdt.c
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <cli.h>
#include <command.h>
#include <console.h>
#include <environment.h>
#include <search.h>
#include <errno.h>
#include <malloc.h>
#include <mapmem.h>
#include <watchdog.h>
#include <environment.h>
#include <i2c.h>
#include <asm/gpio.h>
#include <asm/arch/gpio.h>
#include <asm/byteorder.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/arch/sys_proto.h>

#define C8051_WDT_DEFAULT_TIME	60
#define C8051_WDT_DEFAULT_SHUTDOWN_TIME	3

#define C8051_IIC_BUS		0x02
#define C8051_IIC_ADDR		0x29
#define C8051_WDT_CMD_REG	0x16
#define C8051_WDT_EN_CMD	0x04
#define C8051_WDT_DIS_CMD	0x03
#define C8051_WDT_RESET_CMD	0x02
#define C8051_WDT_TIME_OUT_REG	0x15
#define C8051_WDT_SHUTDOWN_TIME_REG	0x17

#define GPIO_WDT_EN 239//gpio8_15
#define GPIO_WDT_FEED	 228//gpio8_4

static int enable=0;

void watchdog_reset(void)
{
#ifndef CONFIG_SPL_BUILD
	static int value=1;

	if(!enable)
		return;

	if(value)
		value = 0;
	else
		value = 1;
	gpio_direction_output(GPIO_WDT_FEED, value);
#endif
}

void c8051_watchdog_init(void)
{
 	int ret;
	u16	value;
	unsigned int bus;

	gpio_request(GPIO_WDT_EN, "wdt_en");
	gpio_direction_output(GPIO_WDT_EN, 1);
	gpio_request(GPIO_WDT_FEED, "wdt_feed");
	gpio_direction_output(GPIO_WDT_FEED, 1);

	bus = i2c_get_bus_num();
	ret = i2c_set_bus_num(C8051_IIC_BUS);
	if (ret != 0) {
		printf("i2c %d probe failed:%d\n",C8051_IIC_BUS,ret);
		return;
	}
	i2c_set_bus_speed(50000);

	value = C8051_WDT_DEFAULT_TIME;
	i2c_write(C8051_IIC_ADDR, C8051_WDT_TIME_OUT_REG, 1, &value, 2);
	value = C8051_WDT_DEFAULT_SHUTDOWN_TIME;
	i2c_write(C8051_IIC_ADDR, C8051_WDT_SHUTDOWN_TIME_REG, 1, &value, 1);

	i2c_set_bus_num(bus);

	enable = 1;
}

void reset_cpu(ulong ignored)
{
 	int ret;
	uchar	value;
	unsigned int bus;

	bus = i2c_get_bus_num();
	ret = i2c_set_bus_num(C8051_IIC_BUS);
	if (ret != 0) {
		printf("i2c %d probe failed:%d\n",C8051_IIC_BUS,ret);
		return;
	}
	i2c_set_bus_speed(50000);

	value = C8051_WDT_RESET_CMD;
	i2c_write(C8051_IIC_ADDR, C8051_WDT_CMD_REG, 1, &value, 1);

	i2c_set_bus_num(bus);
}

#if 0
 static int do_c8051_wdt(cmd_tbl_t *cmdtp, int flag, int argc,
				char * const argv[])
 {
 	int ret;
	uchar	value;
	unsigned int bus;

 	if (argc != 2)
		return CMD_RET_USAGE;

	if (strcmp (argv[1], "dis") == 0) {
		printf("disable c8051 watchdog...\n");
		value = C8051_WDT_DIS_CMD;
	} else if (strcmp (argv[1], "en") == 0) {
		printf("enable c8051 watchdog...\n");
		value = C8051_WDT_EN_CMD;
	} else {
		printf("Error using!!!\n");
		return CMD_RET_USAGE;
	}
	bus = i2c_get_bus_num();

 	ret = i2c_set_bus_num(C8051_IIC_BUS);
	if (ret != 0) {
		printf("i2c %d probe failed:%d\n",C8051_IIC_BUS,ret);
		return 1;
	}
	i2c_set_bus_speed(50000);
	i2c_write(C8051_IIC_ADDR, C8051_WDT_CMD_REG, 1, &value, 1);

	i2c_set_bus_num(bus);
	return 0;
}

U_BOOT_CMD(
	 c8051_wdt, 2, 1,  do_c8051_wdt,
	 "enable or disable c8051 watchdog",
	 "[dis,en]\n"
	 "dis		disable c8051 watchdog\n"
	 "en		enable c8051 watchdog"
 );
#endif
