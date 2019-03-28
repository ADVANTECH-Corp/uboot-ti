/*
 * Copyright (C) 2016 Advantech Co.,Ltd 
 *
 * Author: yanwei.cao <yanwei.cao@advantech.com.cn>
 *
 * Based on board/ti/am335x_adv/mux.c
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef _MUX_AM335X_ADVANTECH_H_
#define _MUX_AM335X_ADVANTECH_H_

#if defined(CONFIG_TARGET_RSB4220A1_512M) || defined(CONFIG_TARGET_RSB4220A1_1G)
#include "mux_rsb4220.c"
#elif defined(CONFIG_TARGET_RSB4221A1_512M) || defined(CONFIG_TARGET_RSB4221A1_1G)
#include "mux_rsb4221.c"
#elif defined(CONFIG_TARGET_ROM3310A1_512M)
#include "mux_rom3310.c"
#elif defined(CONFIG_TARGET_EPCRS210A1_1G)
#include "mux_epcrs210.c"
#endif

#endif /* _MUX_AM335X_ADVANTECH_H_ */
