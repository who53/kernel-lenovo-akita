/*
 * Copyright (C) 2018 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/pm_wakeup.h>
#include <linux/delay.h>
#include <linux/ratelimit.h>
#include <linux/timekeeping.h>
#include <linux/math64.h>
#include <linux/kthread.h>

#include <linux/iio/consumer.h>

#include "include/pmic.h"
#include "include/pmic_auxadc.h"
#include "include/mt635x-auxadc-internal.h"
#if defined(CONFIG_MTK_SELINUX_AEE_WARNING)
#include <mt-plat/aee.h>
#endif
#include <mt-plat/upmu_common.h>
#include <mt-plat/mtk_auxadc_intf.h>

#ifdef CONFIG_MTK_PMIC_WRAP_HAL
#include <mach/mtk_pmic_wrap.h>
#endif

#if (CONFIG_MTK_GAUGE_VERSION == 30)
#include <mt-plat/mtk_battery.h>
#include <mtk_battery_internal.h>
#endif

static int auxadc_bat_temp_cali(int bat_temp, int precision_factor);

/*********************************
 * PMIC AUXADC Exported API
 *********************************/
static DEFINE_MUTEX(auxadc_ch3_mutex);
static unsigned int g_pmic_pad_vbif28_vol;

unsigned int pmic_get_vbif28_volt(void)
{
	return g_pmic_pad_vbif28_vol;
}

bool is_isense_supported(void)
{
	/* PMIC MT6357 supports ISENSE */
	return true;
}

/* BAT_TEMP background control */
void wk_auxadc_bgd_ctrl(unsigned char en)
{
/*--MT6357 not support Battmp BGD--*/
}

void pmic_auxadc_suspend(void)
{
	wk_auxadc_bgd_ctrl(0);
	/* special call to restore bat_temp_prev when enter suspend */
	auxadc_bat_temp_cali(-1, -1);
}

void pmic_auxadc_resume(void)
{
	wk_auxadc_bgd_ctrl(1);
}

void lockadcch3(void)
{
	mutex_lock(&auxadc_ch3_mutex);
}

void unlockadcch3(void)
{
	mutex_unlock(&auxadc_ch3_mutex);
}

/*********************************
 * Legacy API for getting PMIC AUXADC value
 *********************************/
struct legacy_auxadc_t {
	const char *channel_name;
	struct iio_channel *chan;
};

#define LEGACY_AUXADC_GEN(_name)	\
{	\
	.channel_name = "AUXADC_"#_name,\
}

static struct legacy_auxadc_t legacy_auxadc[] = {
	LEGACY_AUXADC_GEN(BATADC),
	LEGACY_AUXADC_GEN(VCDT),
	LEGACY_AUXADC_GEN(BAT_TEMP),
	LEGACY_AUXADC_GEN(VBIF),
	LEGACY_AUXADC_GEN(CHIP_TEMP),
	LEGACY_AUXADC_GEN(DCXO_TEMP),
	LEGACY_AUXADC_GEN(ACCDET),
	LEGACY_AUXADC_GEN(TSX_TEMP),
	LEGACY_AUXADC_GEN(HPOFS_CAL),
	LEGACY_AUXADC_GEN(ISENSE),
	LEGACY_AUXADC_GEN(VCORE_TEMP),
	LEGACY_AUXADC_GEN(VPROC_TEMP),
};

static void legacy_auxadc_init(struct device *dev)
{
	int i = 0;

	for (i = AUXADC_LIST_START; i <= AUXADC_LIST_END; i++) {
		legacy_auxadc[i].chan =
			devm_iio_channel_get(dev,
					     legacy_auxadc[i].channel_name);
		if (IS_ERR(legacy_auxadc[i].chan))
			pr_debug("%s get fail with list %d\n",
				legacy_auxadc[i].channel_name, i);
	}
}

int pmic_get_auxadc_value(int list)
{
	int value = 0, ret = 0;
#if (CONFIG_MTK_GAUGE_VERSION == 30)
	int bat_cur = 0, is_charging = 0;
#endif

	if (list < AUXADC_LIST_START || list > AUXADC_LIST_END) {
		pr_debug("[%s] Invalid channel list(%d)\n", __func__, list);
		return -EINVAL;
	}
	if (IS_ERR(legacy_auxadc[list].chan)) {
		pr_debug("[%s] iio channel consumer error(%s)\n",
			__func__, legacy_auxadc[list].channel_name);
		return PTR_ERR(legacy_auxadc[list].chan);
	}
#if (CONFIG_MTK_GAUGE_VERSION == 30)
	if (list == AUXADC_LIST_BATTEMP) {
		is_charging = gauge_get_current(&bat_cur);
		if (is_charging == 0)
			bat_cur = 0 - bat_cur;
		pr_debug("[CH3_DBG] bat_cur = %d\n", bat_cur);
	}
#endif
	if (list == AUXADC_LIST_HPOFS_CAL) {
		ret = iio_read_channel_raw(
			legacy_auxadc[list].chan, &value);
	} else {
		ret = iio_read_channel_processed(
			legacy_auxadc[list].chan, &value);
	}
	if (ret < 0)
		return ret;
	return value;
}

/*********************************
 * PMIC AUXADC chip setting for IIO ADC driver
 *********************************/
static void auxadc_bat_temp_convert(unsigned char convert)
{
	if (convert == 1)
		mutex_lock(&auxadc_ch3_mutex);
	else if (convert == 0)
		mutex_unlock(&auxadc_ch3_mutex);
}

static void auxadc_dcxo_temp_convert(unsigned char convert)
{
	if (convert == 1)
		pmic_set_hk_reg_value(PMIC_AUXADC_DCXO_CH4_MUX_AP_SEL, 1);
	else if (convert == 0)
		pmic_set_hk_reg_value(PMIC_AUXADC_DCXO_CH4_MUX_AP_SEL, 0);
}

static void auxadc_vbif_convert(unsigned char convert)
{
	if (convert == 1)
		pmic_set_hk_reg_value(PMIC_BATON_TDET_EN, 0);
	else if (convert == 0)
		pmic_set_hk_reg_value(PMIC_BATON_TDET_EN, 1);
}

static int auxadc_bat_temp_cali(int bat_temp, int precision_factor)
{
	static int bat_temp_prev;

	if (bat_temp == -1 && precision_factor == -1) {
		bat_temp_prev = 0;
		return 0;
	}
	if (bat_temp_prev == 0)
		goto out;

out:
	bat_temp_prev = bat_temp;
	return bat_temp;
}

struct auxadc_regs_map {
	int channel;
	struct auxadc_regs regs;
};

static struct auxadc_regs_map pmic_auxadc_regs_map[] = {
	{
		.channel = AUXADC_BATADC,
		.regs = {
			PMIC_AUXADC_RQST_CH0,
			PMIC_AUXADC_ADC_RDY_CH0_BY_AP,
			PMIC_AUXADC_ADC_OUT_CH0_BY_AP
		},
	},
	{
		.channel = AUXADC_ISENSE,
		.regs = {
			PMIC_AUXADC_RQST_CH1,
			PMIC_AUXADC_ADC_RDY_CH1_BY_AP,
			PMIC_AUXADC_ADC_OUT_CH1_BY_AP
		},
	},
	{
		.channel = AUXADC_VCDT,
		.regs = {
			PMIC_AUXADC_RQST_CH2,
			PMIC_AUXADC_ADC_RDY_CH2,
			PMIC_AUXADC_ADC_OUT_CH2
		},
	},
	{
		.channel = AUXADC_BAT_TEMP,
		.regs = {
			PMIC_AUXADC_RQST_CH3,
			PMIC_AUXADC_ADC_RDY_CH3,
			PMIC_AUXADC_ADC_OUT_CH3
		},
	},
	{
		.channel = AUXADC_CHIP_TEMP,
		.regs = {
			PMIC_AUXADC_RQST_CH4,
			PMIC_AUXADC_ADC_RDY_CH4,
			PMIC_AUXADC_ADC_OUT_CH4
		},
	},
	{
		.channel = AUXADC_VCORE_TEMP,
		.regs = {
			PMIC_AUXADC_RQST_CH4_BY_THR1,
			PMIC_AUXADC_ADC_RDY_CH4_BY_THR1,
			PMIC_AUXADC_ADC_OUT_CH4_BY_THR1
		},
	},
	{
		.channel = AUXADC_VPROC_TEMP,
		.regs = {
			PMIC_AUXADC_RQST_CH4_BY_THR2,
			PMIC_AUXADC_ADC_RDY_CH4_BY_THR2,
			PMIC_AUXADC_ADC_OUT_CH4_BY_THR2
		},
	},
	{
		.channel = AUXADC_ACCDET,
		.regs = {
			PMIC_AUXADC_RQST_CH5,
			PMIC_AUXADC_ADC_RDY_CH5,
			PMIC_AUXADC_ADC_OUT_CH5
		},
	},
	{
		.channel = AUXADC_TSX_TEMP,
		.regs = {
			PMIC_AUXADC_RQST_CH7,
			PMIC_AUXADC_ADC_RDY_CH7,
			PMIC_AUXADC_ADC_OUT_CH7
		},
	},
	{
		.channel = AUXADC_HPOFS_CAL,
		.regs = {
			PMIC_AUXADC_RQST_CH9,
			PMIC_AUXADC_ADC_RDY_CH9,
			PMIC_AUXADC_ADC_OUT_CH9
		},
	},
	{
		.channel = AUXADC_DCXO_TEMP,
		.regs = {
			PMIC_AUXADC_RQST_CH4,
			PMIC_AUXADC_ADC_RDY_DCXO_BY_AP,
			PMIC_AUXADC_ADC_OUT_DCXO_BY_AP
		},
	},
	{
		.channel = AUXADC_VBIF,
		.regs = {
			PMIC_AUXADC_RQST_CH11,
			PMIC_AUXADC_ADC_RDY_CH11,
			PMIC_AUXADC_ADC_OUT_CH11
		},
	},
};

void pmic_auxadc_chip_timeout_handler(
	struct device *dev, bool is_timeout, unsigned char ch_num)
{
	static unsigned short timeout_times;

	if (is_timeout == false) {
		timeout_times = 0;
		return;
	}
	timeout_times++;
	dev_notice(dev, "(%d)Time out!STA0=0x%x,STA1=0x%x,STA2=0x%x\n",
		ch_num,
		upmu_get_reg_value(MT6357_AUXADC_STA0),
		upmu_get_reg_value(MT6357_AUXADC_STA1),
		upmu_get_reg_value(MT6357_AUXADC_STA2));
	dev_notice(dev, "RST: Reg[0x%x]=0x%x, Reg[0x%x]=0x%x\n",
		MT6357_STRUP_CON6,
		upmu_get_reg_value(MT6357_STRUP_CON6),
		MT6357_HK_TOP_RST_CON0,
		upmu_get_reg_value(MT6357_HK_TOP_RST_CON0));
	dev_notice(dev, "CLK: Reg[0x%x]=0x%x, Reg[0x%x]=0x%x\n",
		MT6357_HK_TOP_CLK_CON0,
		upmu_get_reg_value(MT6357_HK_TOP_CLK_CON0),
		MT6357_HK_TOP_CLK_CON1,
		upmu_get_reg_value(MT6357_HK_TOP_CLK_CON1));
}

int pmic_auxadc_chip_init(struct device *dev)
{
	int ret = 0;
	unsigned short i;
	struct iio_channel *chan_vbif;

	HKLOG("%s\n", __func__);

	for (i = 0; i < ARRAY_SIZE(pmic_auxadc_regs_map); i++) {
		auxadc_set_regs(
			pmic_auxadc_regs_map[i].channel,
			&(pmic_auxadc_regs_map[i].regs));
	}
	auxadc_set_convert_fn(AUXADC_BAT_TEMP, auxadc_bat_temp_convert);
	auxadc_set_convert_fn(AUXADC_DCXO_TEMP, auxadc_dcxo_temp_convert);
	auxadc_set_convert_fn(AUXADC_VBIF, auxadc_vbif_convert);
	auxadc_set_cali_fn(AUXADC_BAT_TEMP, auxadc_bat_temp_cali);

#if 1 /*TBD*/
	legacy_auxadc_init(dev);
#endif

	/* update VBIF28 by AUXADC */
	chan_vbif = iio_channel_get(dev, "AUXADC_VBIF");
	if (IS_ERR(chan_vbif)) {
		pr_debug("[%s] iio channel consumer error(AUXADC_VBIF)\n",
			__func__);
	} else {
		ret = iio_read_channel_processed(chan_vbif,
						 &g_pmic_pad_vbif28_vol);
		iio_channel_release(chan_vbif);
	}
	return ret;
}
