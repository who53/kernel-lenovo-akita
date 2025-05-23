/*
 *
 * (C) COPYRIGHT 2012-2019 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU licence.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 * SPDX-License-Identifier: GPL-2.0
 *
 */



/*
 * Run-time work-arounds helpers
 */

#include <mali_base_hwconfig_features.h>
#include <mali_base_hwconfig_issues.h>
#include <mali_midg_regmap.h>
#include "mali_kbase.h"
#include "mali_kbase_hw.h"

void kbase_hw_set_features_mask(struct kbase_device *kbdev)
{
	const enum base_hw_feature *features;
	u32 gpu_id;

	gpu_id = kbdev->gpu_props.props.raw_props.gpu_id;

	switch (gpu_id & GPU_ID2_PRODUCT_MODEL) {
	case GPU_ID2_PRODUCT_TMIX:
		features = base_hw_features_tMIx;
		break;
	case GPU_ID2_PRODUCT_THEX:
		features = base_hw_features_tHEx;
		break;
	case GPU_ID2_PRODUCT_TSIX:
		features = base_hw_features_tSIx;
		break;
	case GPU_ID2_PRODUCT_TDVX:
		features = base_hw_features_tDVx;
		break;
	case GPU_ID2_PRODUCT_TNOX:
		features = base_hw_features_tNOx;
		break;
	case GPU_ID2_PRODUCT_TGOX:
		features = base_hw_features_tGOx;
		break;
	case GPU_ID2_PRODUCT_TTRX:
		features = base_hw_features_tTRx;
		break;
	case GPU_ID2_PRODUCT_TNAX:
		features = base_hw_features_tNAx;
		break;
	case GPU_ID2_PRODUCT_LBEX:
	case GPU_ID2_PRODUCT_TBEX:
		features = base_hw_features_tBEx;
		break;
	case GPU_ID2_PRODUCT_TULX:
		features = base_hw_features_tULx;
		break;
	case GPU_ID2_PRODUCT_TDUX:
		features = base_hw_features_tDUx;
		break;
	case GPU_ID2_PRODUCT_TODX:
	case GPU_ID2_PRODUCT_LODX:
		features = base_hw_features_tODx;
		break;
	case GPU_ID2_PRODUCT_TIDX:
		features = base_hw_features_tIDx;
		break;
	case GPU_ID2_PRODUCT_TVAX:
		features = base_hw_features_tVAx;
		break;
	default:
		features = base_hw_features_generic;
		break;
	}

	for (; *features != BASE_HW_FEATURE_END; features++)
		set_bit(*features, &kbdev->hw_features_mask[0]);

#if defined(CONFIG_MALI_JOB_DUMP) || defined(CONFIG_MALI_VECTOR_DUMP)
	/* When dumping is enabled, need to disable flush reduction optimization
	 * for GPUs on which it is safe to have only cache clean operation at
	 * the end of job chain.
	 * This is required to make job dumping work. There is some discrepancy
	 * in the implementation of flush reduction optimization due to
	 * unclear or ambiguous ARCH spec.
	 */
	if (kbase_hw_has_feature(kbdev, BASE_HW_FEATURE_CLEAN_ONLY_SAFE))
		clear_bit(BASE_HW_FEATURE_FLUSH_REDUCTION,
			&kbdev->hw_features_mask[0]);
#endif
}

/**
 * kbase_hw_get_issues_for_new_id - Get the hardware issues for a new GPU ID
 * @kbdev: Device pointer
 *
 * Return: pointer to an array of hardware issues, terminated by
 * BASE_HW_ISSUE_END.
 *
 * In debugging versions of the driver, unknown versions of a known GPU will
 * be treated as the most recent known version not later than the actual
 * version. In such circumstances, the GPU ID in @kbdev will also be replaced
 * with the most recent known version.
 *
 * Note: The GPU configuration must have been read by kbase_gpuprops_get_props()
 * before calling this function.
 */
static const enum base_hw_issue *kbase_hw_get_issues_for_new_id(
					struct kbase_device *kbdev)
{
	const enum base_hw_issue *issues = NULL;

	struct base_hw_product {
		u32 product_model;
		struct {
			u32 version;
			const enum base_hw_issue *issues;
		} map[7];
	};

	static const struct base_hw_product base_hw_products[] = {
		{GPU_ID2_PRODUCT_TMIX,
		 {{GPU_ID2_VERSION_MAKE(0, 0, 1),
		   base_hw_issues_tMIx_r0p0_05dev0},
		  {GPU_ID2_VERSION_MAKE(0, 0, 2), base_hw_issues_tMIx_r0p0},
		  {GPU_ID2_VERSION_MAKE(0, 1, 0), base_hw_issues_tMIx_r0p1},
		  {U32_MAX /* sentinel value */, NULL} } },

		{GPU_ID2_PRODUCT_THEX,
		 {{GPU_ID2_VERSION_MAKE(0, 0, 0), base_hw_issues_tHEx_r0p0},
		  {GPU_ID2_VERSION_MAKE(0, 0, 1), base_hw_issues_tHEx_r0p0},
		  {GPU_ID2_VERSION_MAKE(0, 1, 0), base_hw_issues_tHEx_r0p1},
		  {GPU_ID2_VERSION_MAKE(0, 1, 1), base_hw_issues_tHEx_r0p1},
		  {GPU_ID2_VERSION_MAKE(0, 2, 0), base_hw_issues_tHEx_r0p2},
		  {GPU_ID2_VERSION_MAKE(0, 3, 0), base_hw_issues_tHEx_r0p3},
		  {U32_MAX, NULL} } },

		{GPU_ID2_PRODUCT_TSIX,
		 {{GPU_ID2_VERSION_MAKE(0, 0, 0), base_hw_issues_tSIx_r0p0},
		  {GPU_ID2_VERSION_MAKE(0, 0, 1), base_hw_issues_tSIx_r0p0},
		  {GPU_ID2_VERSION_MAKE(0, 1, 0), base_hw_issues_tSIx_r0p1},
		  {GPU_ID2_VERSION_MAKE(1, 0, 0), base_hw_issues_tSIx_r1p0},
		  {GPU_ID2_VERSION_MAKE(1, 1, 0), base_hw_issues_tSIx_r1p1},
		  {U32_MAX, NULL} } },

		{GPU_ID2_PRODUCT_TDVX,
		 {{GPU_ID2_VERSION_MAKE(0, 0, 0), base_hw_issues_tDVx_r0p0},
		  {U32_MAX, NULL} } },

		{GPU_ID2_PRODUCT_TNOX,
		 {{GPU_ID2_VERSION_MAKE(0, 0, 0), base_hw_issues_tNOx_r0p0},
		  {U32_MAX, NULL} } },

		{GPU_ID2_PRODUCT_TGOX,
		 {{GPU_ID2_VERSION_MAKE(0, 0, 0), base_hw_issues_tGOx_r0p0},
		  {GPU_ID2_VERSION_MAKE(1, 0, 0), base_hw_issues_tGOx_r1p0},
		  {U32_MAX, NULL} } },

		{GPU_ID2_PRODUCT_TTRX,
		 {{GPU_ID2_VERSION_MAKE(0, 0, 0), base_hw_issues_tTRx_r0p0},
		  {GPU_ID2_VERSION_MAKE(0, 0, 3), base_hw_issues_tTRx_r0p0},
		  {GPU_ID2_VERSION_MAKE(0, 1, 0), base_hw_issues_tTRx_r0p1},
		  {GPU_ID2_VERSION_MAKE(0, 1, 1), base_hw_issues_tTRx_r0p1},
		  {U32_MAX, NULL} } },

		{GPU_ID2_PRODUCT_TNAX,
		 {{GPU_ID2_VERSION_MAKE(0, 0, 0), base_hw_issues_tNAx_r0p0},
		  {GPU_ID2_VERSION_MAKE(0, 0, 3), base_hw_issues_tNAx_r0p0},
		  {GPU_ID2_VERSION_MAKE(0, 0, 4), base_hw_issues_tNAx_r0p0},
		  {GPU_ID2_VERSION_MAKE(0, 0, 5), base_hw_issues_tNAx_r0p0},
		  {GPU_ID2_VERSION_MAKE(0, 1, 0), base_hw_issues_tNAx_r0p1},
		  {GPU_ID2_VERSION_MAKE(0, 1, 1), base_hw_issues_tNAx_r0p1},
		  {U32_MAX, NULL} } },

		{GPU_ID2_PRODUCT_LBEX,
		 {{GPU_ID2_VERSION_MAKE(1, 0, 0), base_hw_issues_tBEx_r1p0},
		  {U32_MAX, NULL} } },

		{GPU_ID2_PRODUCT_TBEX,
		 {{GPU_ID2_VERSION_MAKE(0, 0, 0), base_hw_issues_tBEx_r0p0},
		  {GPU_ID2_VERSION_MAKE(0, 0, 3), base_hw_issues_tBEx_r0p0},
		  {GPU_ID2_VERSION_MAKE(1, 0, 0), base_hw_issues_tBEx_r1p0},
		  {U32_MAX, NULL} } },

		{GPU_ID2_PRODUCT_TULX,
		 {{GPU_ID2_VERSION_MAKE(0, 0, 0), base_hw_issues_tULx_r0p0},
		  {U32_MAX, NULL} } },

		{GPU_ID2_PRODUCT_TDUX,
		 {{GPU_ID2_VERSION_MAKE(0, 0, 0), base_hw_issues_tDUx_r0p0},
		  {U32_MAX, NULL} } },

		{GPU_ID2_PRODUCT_TODX,
		 {{GPU_ID2_VERSION_MAKE(0, 0, 0), base_hw_issues_tODx_r0p0},
		  {U32_MAX, NULL} } },

		{GPU_ID2_PRODUCT_LODX,
		 {{GPU_ID2_VERSION_MAKE(0, 0, 0), base_hw_issues_tODx_r0p0},
		  {U32_MAX, NULL} } },

		{GPU_ID2_PRODUCT_TIDX,
		 {{GPU_ID2_VERSION_MAKE(0, 0, 0), base_hw_issues_tIDx_r0p0},
		  {U32_MAX, NULL} } },

		{GPU_ID2_PRODUCT_TVAX,
		 {{GPU_ID2_VERSION_MAKE(0, 0, 0), base_hw_issues_tVAx_r0p0},
		  {U32_MAX, NULL} } },
	};

	u32 gpu_id = kbdev->gpu_props.props.raw_props.gpu_id;
	const u32 product_model = gpu_id & GPU_ID2_PRODUCT_MODEL;
	const struct base_hw_product *product = NULL;
	size_t p;

	/* Stop when we reach the end of the products array. */
	for (p = 0; p < ARRAY_SIZE(base_hw_products); ++p) {
		if (product_model == base_hw_products[p].product_model) {
			product = &base_hw_products[p];
			break;
		}
	}

	if (product != NULL) {
		/* Found a matching product. */
		const u32 version = gpu_id & GPU_ID2_VERSION;
		u32 fallback_version = 0;
		const enum base_hw_issue *fallback_issues = NULL;
		size_t v;

		/* Stop when we reach the end of the map. */
		for (v = 0; product->map[v].version != U32_MAX; ++v) {

			if (version == product->map[v].version) {
				/* Exact match so stop. */
				issues = product->map[v].issues;
				break;
			}

			/* Check whether this is a candidate for most recent
				known version not later than the actual
				version. */
			if ((version > product->map[v].version) &&
				(product->map[v].version >= fallback_version)) {
#if MALI_CUSTOMER_RELEASE
				/* Match on version's major and minor fields */
				if (((version ^ product->map[v].version) >>
					GPU_ID2_VERSION_MINOR_SHIFT) == 0)
#endif
				{
					fallback_version = product->map[v].version;
					fallback_issues = product->map[v].issues;
				}
			}
		}

		if ((issues == NULL) && (fallback_issues != NULL)) {
			/* Fall back to the issue set of the most recent known
				version not later than the actual version. */
			issues = fallback_issues;

#if MALI_CUSTOMER_RELEASE
			dev_warn(kbdev->dev,
				"GPU hardware issue table may need updating:\n"
#else
			dev_dbg(kbdev->dev,
#endif
				"r%dp%d status %d is unknown; treating as r%dp%d status %d",
				(gpu_id & GPU_ID2_VERSION_MAJOR) >>
					GPU_ID2_VERSION_MAJOR_SHIFT,
				(gpu_id & GPU_ID2_VERSION_MINOR) >>
					GPU_ID2_VERSION_MINOR_SHIFT,
				(gpu_id & GPU_ID2_VERSION_STATUS) >>
					GPU_ID2_VERSION_STATUS_SHIFT,
				(fallback_version & GPU_ID2_VERSION_MAJOR) >>
					GPU_ID2_VERSION_MAJOR_SHIFT,
				(fallback_version & GPU_ID2_VERSION_MINOR) >>
					GPU_ID2_VERSION_MINOR_SHIFT,
				(fallback_version & GPU_ID2_VERSION_STATUS) >>
					GPU_ID2_VERSION_STATUS_SHIFT);

			gpu_id &= ~GPU_ID2_VERSION;
			gpu_id |= fallback_version;
			kbdev->gpu_props.props.raw_props.gpu_id = gpu_id;

			kbase_gpuprops_update_core_props_gpu_id(
				&kbdev->gpu_props.props);
		}
	}
	return issues;
}

int kbase_hw_set_issues_mask(struct kbase_device *kbdev)
{
	const enum base_hw_issue *issues;
	u32 gpu_id;
	u32 impl_tech;

	gpu_id = kbdev->gpu_props.props.raw_props.gpu_id;
	impl_tech = kbdev->gpu_props.props.thread_props.impl_tech;

	if (impl_tech != IMPLEMENTATION_MODEL) {
		issues = kbase_hw_get_issues_for_new_id(kbdev);
		if (issues == NULL) {
			dev_err(kbdev->dev,
				"Unknown GPU ID %x", gpu_id);
			return -EINVAL;
		}

#if !MALI_CUSTOMER_RELEASE
		/* The GPU ID might have been replaced with the last
			known version of the same GPU. */
		gpu_id = kbdev->gpu_props.props.raw_props.gpu_id;
#endif
	} else {
		/* Software model */
		switch (gpu_id & GPU_ID2_PRODUCT_MODEL) {
		case GPU_ID2_PRODUCT_TMIX:
			issues = base_hw_issues_model_tMIx;
			break;
		case GPU_ID2_PRODUCT_THEX:
			issues = base_hw_issues_model_tHEx;
			break;
		case GPU_ID2_PRODUCT_TSIX:
			issues = base_hw_issues_model_tSIx;
			break;
		case GPU_ID2_PRODUCT_TDVX:
			issues = base_hw_issues_model_tDVx;
			break;
		case GPU_ID2_PRODUCT_TNOX:
			issues = base_hw_issues_model_tNOx;
			break;
		case GPU_ID2_PRODUCT_TGOX:
			issues = base_hw_issues_model_tGOx;
			break;
		case GPU_ID2_PRODUCT_TTRX:
			issues = base_hw_issues_model_tTRx;
			break;
		case GPU_ID2_PRODUCT_TNAX:
			issues = base_hw_issues_model_tNAx;
			break;
		case GPU_ID2_PRODUCT_LBEX:
		case GPU_ID2_PRODUCT_TBEX:
			issues = base_hw_issues_model_tBEx;
			break;
		case GPU_ID2_PRODUCT_TULX:
			issues = base_hw_issues_model_tULx;
			break;
		case GPU_ID2_PRODUCT_TDUX:
			issues = base_hw_issues_model_tDUx;
			break;
		case GPU_ID2_PRODUCT_TODX:
		case GPU_ID2_PRODUCT_LODX:
			issues = base_hw_issues_model_tODx;
			break;
		case GPU_ID2_PRODUCT_TIDX:
			issues = base_hw_issues_model_tIDx;
			break;
		case GPU_ID2_PRODUCT_TVAX:
			issues = base_hw_issues_model_tVAx;
			break;
		default:
			dev_err(kbdev->dev,
				"Unknown GPU ID %x", gpu_id);
			return -EINVAL;
		}
	}

	dev_dbg(kbdev->dev,
		"GPU identified as 0x%x arch %d.%d.%d r%dp%d status %d",
		(gpu_id & GPU_ID2_PRODUCT_MAJOR) >>
			GPU_ID2_PRODUCT_MAJOR_SHIFT,
		(gpu_id & GPU_ID2_ARCH_MAJOR) >>
			GPU_ID2_ARCH_MAJOR_SHIFT,
		(gpu_id & GPU_ID2_ARCH_MINOR) >>
			GPU_ID2_ARCH_MINOR_SHIFT,
		(gpu_id & GPU_ID2_ARCH_REV) >>
			GPU_ID2_ARCH_REV_SHIFT,
		(gpu_id & GPU_ID2_VERSION_MAJOR) >>
			GPU_ID2_VERSION_MAJOR_SHIFT,
		(gpu_id & GPU_ID2_VERSION_MINOR) >>
			GPU_ID2_VERSION_MINOR_SHIFT,
		(gpu_id & GPU_ID2_VERSION_STATUS) >>
			GPU_ID2_VERSION_STATUS_SHIFT);

	for (; *issues != BASE_HW_ISSUE_END; issues++)
		set_bit(*issues, &kbdev->hw_issues_mask[0]);

	return 0;
}
