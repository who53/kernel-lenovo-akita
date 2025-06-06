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
subdir-ccflags-y += -Werror

# dsp related code
obj-$(CONFIG_MTK_AUDIODSP_SUPPORT) += audio_dsp/
obj-$(CONFIG_SND_SOC_MTK_AUDIO_DSP) += audio_dsp/
obj-$(CONFIG_MTK_VOW_BARGE_IN_SUPPORT) += scp_vow/
# btcvsd
obj-$(CONFIG_SND_SOC_MTK_BTCVSD) += common/
obj-$(CONFIG_MTK_AUDIO_TUNNELING_SUPPORT) += common_int/
obj-$(CONFIG_SND_SOC_MEDIATEK) += common/
obj-$(CONFIG_SND_SOC_MT2701) += mt2701/
obj-$(CONFIG_SND_SOC_MT3967) += mt3967/
obj-$(CONFIG_SND_SOC_MT6779) += mt6779/
obj-$(CONFIG_SND_SOC_MT8173) += mt8173/
obj-$(CONFIG_SND_SOC_MT8163) += mt_soc_audio_8163/
obj-$(CONFIG_SND_SOC_AW87519) += aw87519/
obj-$(CONFIG_SND_SOC_DBMDX) += dbmdx/
obj-$(CONFIG_SND_SOC_DBMDX) += ac107/

# adsp
ifeq ($(CONFIG_MTK_AURISYS_PHONE_CALL_SUPPORT),y)
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/audio_ipi/common/framework/
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/adsp/$(MTK_PLATFORM)/
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/audio_ipi/common/tasks/call/
subdir-ccflags-y += -I$(srctree)/sound/soc/mediatek/audio_dsp
endif

ifeq ($(CONFIG_SND_SOC_MTK_SMART_PHONE),y)
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
endif

# New architecture
ifeq ($(CONFIG_MTK_SND_SOC_NEW_ARCH),y)

# MTK Platform Support
MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))

# MTK Codec Chip Support
MTK_CODEC := $(subst ",,$(CONFIG_MTK_SND_SOC_CODEC))

# flags for not ready feature, uncomment after ready
subdir-ccflags-y += -D_GIT318_READY
subdir-ccflags-y += -D_GIT318_PMIC_READY
subdir-ccflags-y += -DAUDIO_USING_WRAP_DRIVER
subdir-ccflags-y += -D_MT_IDLE_HEADER

# include path
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/accdet/mt6358
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include/mt-plat
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/base/power/include
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/base/power/$(MTK_PLATFORM)
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/dramc/$(MTK_PLATFORM)/

ifeq ($(CONFIG_MACH_MT6761),y)
AUDIO_PLATFORM := mt6765
else
AUDIO_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
endif

subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/scp/$(AUDIO_PLATFORM)/
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/audio_ipi/$(AUDIO_PLATFORM)/


# Common include path
subdir-ccflags-y += -Werror -I$(src)/common_int

# PMIC include path
ifneq  ($(MTK_CODEC),)
subdir-ccflags-y += -Werror -I$(src)/codec/$(MTK_CODEC)
endif

# vendor include path
subdir-ccflags-y += -Werror -I$(src)/vendor

subdir-ccflags-y += -Werror -Wno-incompatible-pointer-types

# Update pmic built flow to avoid codec init later than codec probe
ifneq  ($(MTK_CODEC),)
obj-y += codec/$(MTK_CODEC)/
endif
obj-y += vendor/
obj-y += common_int/

ifeq ($(MTK_PLATFORM),mt6758)
  obj-y += mt6758_int/
  subdir-ccflags-y += -Werror -I$(src)/mt6758_int
else
  obj-y += $(AUDIO_PLATFORM)/

# AP include path
  subdir-ccflags-y += -Werror -I$(src)/$(AUDIO_PLATFORM)
endif
endif
