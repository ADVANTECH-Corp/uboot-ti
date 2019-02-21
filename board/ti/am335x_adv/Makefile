#
# Makefile
#
# Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
#
# SPDX-License-Identifier:	GPL-2.0+
#

#ifeq ($(CONFIG_SKIP_LOWLEVEL_INIT),)
obj-y	:= mux.o
#endif

ifndef CONFIG_SPL_BUILD
ifdef CONFIG_ADV_OTA_SUPPORT
obj-$(CONFIG_BCB_SUPPORT) += command.o bcb.o
endif
endif

obj-y	+= board.o
