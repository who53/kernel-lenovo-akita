#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See http://www.gnu.org/licenses/gpl-2.0.html for more details.
#

MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
subdir-ccflags-y += -Werror
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include
subdir-ccflags-$(CONFIG_USB_MU3D_DRV) += -I$(srctree)/drivers/misc/mediatek/mu3d/drv
subdir-ccflags-$(CONFIG_EXTCON) += -I$(srctree)/drivers/extcon
subdir-ccflags-$(CONFIG_USB_MTK_EXTCON) += -I$(srctree)/drivers/extcon/mediatek
subdir-ccflags-$(CONFIG_USB_MTK_HDRC) += -I$(srctree)/drivers/misc/mediatek/usb20/
subdir-ccflags-$(CONFIG_MTK_PMIC_CHIP_MT6355) += -I$(srctree)/drivers/misc/mediatek/pmic/include/mt6355
subdir-ccflags-$(CONFIG_MTK_PMIC_CHIP_MT6356) += -I$(srctree)/drivers/misc/mediatek/pmic/include/mt6356
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/typec/tcpc/inc
subdir-ccflags-y += -I$(srctree)/drivers/power/supply/mediatek/misc/
subdir-ccflags-y += -I$(srctree)/drivers/power/supply/mediatek/battery/
ifeq  ($(WT_COMPILE_FACTORY_VERSION),yes)
	ccflags-y = -DFACTORY_VERSION
endif
obj-y += charger_class.o mtk_charger.o mtk_switch_charging.o mtk_pdc_intf.o\
	 mtk_chg_type_det.o mtk_pe40_intf.o adapter_class.o mtk_pd_adapter.o

obj-$(CONFIG_MTK_DUAL_CHARGER_SUPPORT) += mtk_dual_switch_charging.o
obj-$(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT) += mtk_pe_intf.o
obj-$(CONFIG_MTK_PUMP_EXPRESS_PLUS_20_SUPPORT) += mtk_pe20_intf.o
obj-$(CONFIG_CHARGER_RT9465) += rt9465.o
obj-$(CONFIG_CHARGER_BQ2560X) += bq2560x_charger.o
obj-$(CONFIG_POGO_CHARGER) += pogo_charger.o
obj-$(CONFIG_CHARGER_RT9471) += rt9471.o
obj-$(CONFIG_CHARGER_BQ25601) += bq25601.o
obj-$(CONFIG_MTK_BQ25890_SUPPORT) += bq25890.o
