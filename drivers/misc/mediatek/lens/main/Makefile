#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#

#
# Makefile for misc devices that really don't fit anywhere else.
#

MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
ARCH_MTK_PROJECT := $(subst ",,$(CONFIG_ARCH_MTK_PROJECT))

MTK_CUSTOM_PATH  := $(srctree)/drivers/misc/mediatek


subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/main/inc
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/main/common/lc898122af
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/main/common/lc898212xdaf/inc
ifeq ($(CONFIG_MTK_LENS_BU63165AF_SUPPORT), y)
  subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/main/common/bu63165af/inc
else
  subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/main/common/bu63169af/inc
endif
ccflags-y += -I$(srctree)/drivers/misc/mediatek/timer/timesync/

ifeq ($(CONFIG_MTK_LENS_USE_MODULE), y)

MAIN_CFILES += main_lens.c

MAIN_CFILES  += common/fp5510e2af/FP5510E2AF.c
MAIN_CFILES  += common/bu6424af/BU6424AF.c
MAIN_CFILES  += common/bu6429af/BU6429AF.c
MAIN_CFILES  += common/bu6429af/bu64748af.c
MAIN_CFILES  += common/bu6429af/bu64748_function.c
MAIN_CFILES += common/dw9714af/DW9714AF.c
MAIN_CFILES += common/dw9763af/DW9763AF.c
MAIN_CFILES += common/cn3927e_hi556/CN3927E_HI556.c
MAIN_CFILES += common/cn3937/CN3937.c
MAIN_CFILES += common/cn3937af/CN3937AF.c
MAIN_CFILES += common/cn3927af/CN3927AF.c
MAIN_CFILES += common/cn3927e_gc5035/CN3927E_GC5035.c
MAIN_CFILES += common/gt9768af/GT9768AF.c
MAIN_CFILES  += common/dw9814af/DW9814AF.c
MAIN_CFILES  += common/dw9718af/DW9718AF.c
MAIN_CFILES  += common/dw9718saf/DW9718SAF.c
MAIN_CFILES  += common/dw9719taf/DW9719TAF.c
MAIN_CFILES += common/lc898212af/LC898212AF.c
MAIN_CFILES  += common/lc898212xdaf/LC898212XDAF.c
MAIN_CFILES  += common/lc898212xdaf/AfSTMV.c
MAIN_CFILES  += common/lc898212xdaf/AfInter.c
MAIN_CFILES  += common/lc898212xdaf/AfInit.c
MAIN_CFILES  += common/lc898214af/LC898214AF.c
MAIN_CFILES  += common/lc898217af/LC898217AF.c
MAIN_CFILES  += common/lc898217afa/LC898217AFA.c
MAIN_CFILES  += common/lc898217afb/LC898217AFB.c
MAIN_CFILES  += common/lc898217afc/LC898217AFC.c
MAIN_CFILES  += common/lc898122af/LC898122AF.c
MAIN_CFILES  += common/lc898122af/OisCmd.c
MAIN_CFILES  += common/lc898122af/OisIni.c
MAIN_CFILES  += common/ad5820af/AD5820AF.c
MAIN_CFILES  += common/wv511aaf/WV511AAF.c
MAIN_CFILES  += common/ak7371af/AK7371AF.c
ifeq ($(CONFIG_MTK_LENS_BU63165AF_SUPPORT), y)
MAIN_CFILES  += common/bu63165af/BU63165AF.c
MAIN_CFILES  += common/bu63165af/OIS_func.c
MAIN_CFILES  += common/bu63165af/OIS_main.c
MAIN_CFILES  += common/bu63165af/OIS_user.c
else
MAIN_CFILES  += common/bu63169af/BU63169AF.c
MAIN_CFILES  += common/bu63169af/OIS_func.c
MAIN_CFILES  += common/bu63169af/OIS_main.c
MAIN_CFILES  += common/bu63169af/OIS_user.c
endif

obj-m := mainaf.o
mainaf-objs := $(MAIN_CFILES:.c=.o)

else

obj-y  += main_lens.o
obj-y  += common/fp5510e2af/
obj-y  += common/bu6424af/
obj-y  += common/bu6429af/
obj-y  += common/bu64748af/
obj-y  += common/dw9714af/
obj-y  += common/dw9763af/
obj-y  += common/cn3927e_hi556/
obj-y  += common/cn3937/
obj-y  += common/cn3927e_gc5035/
obj-y  += common/gt9768af/
obj-y  += common/dw9814af/
obj-y  += common/dw9718af/
obj-y  += common/dw9718saf/
obj-y  += common/dw9719taf/



obj-y  += common/lc898212af/
obj-y  += common/lc898212xdaf/
obj-y  += common/lc898214af/
obj-y  += common/lc898217af/
obj-y  += common/lc898217afa/
obj-y  += common/lc898217afb/
obj-y  += common/lc898217afc/
obj-y  += common/lc898122af/
obj-y  += common/wv511aaf/
obj-y  += common/ak7371af/
obj-y  += common/cn3937af/
obj-y  += common/cn3927af/
ifeq ($(CONFIG_MTK_LENS_BU63165AF_SUPPORT), y)
  obj-y  += common/bu63165af/
else
  obj-y  += common/bu63169af/
endif

endif
