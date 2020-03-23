/*
 * Copyright (C) 2016 Advantech Co.,Ltd 
 *
 * Author: yanwei.cao <yanwei.cao@advantech.com.cn>
 *
 * Based on board/ti/advantech/mux_data.h
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef _MUX_DATA_ADVANTECH_H_
#define _MUX_DATA_ADVANTECH_H_

#include <asm/arch/mux_dra7xx.h>

#if defined(CONFIG_TARGET_ROM7510A1_2G)
#include "mux_rom7510a1.h"
#elif defined(CONFIG_TARGET_ROM7510A2_2G)
#include "mux_rom7510a2.h"
#endif

#endif /* _MUX_DATA_ADVANTECH_H_ */


