# Copyright (c) 2015-2017 MICROTRUST Incorporated
# All Rights Reserved.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.

obj-$(CONFIG_MICROTRUST_TZ_DRIVER) += fp_func.o
ccflags-y += -fno-pic

MTK_PLATFORM := $(shell echo $(CONFIG_MTK_PLATFORM) | grep -o "[0-9]\+")
ccflags-y += -D MTK_PLATFORM=$(MTK_PLATFORM)
