/*
 * Copyright 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>

#include <zephyr/drivers/clock_management/clock_driver.h>
#include <zephyr/drivers/clock_management/clock_helpers.h>

#include "nxp_kinetis_internal.h"

#define DT_DRV_COMPAT nxp_kinetis_mcg_pll

/* PRDIV0 divisors above 25 are reserved */
#define PLL_PRDIV_MIN 1U
#define PLL_PRDIV_MAX 25U
/* VDIV0 multiplies the reference by 24-55 */
#define PLL_VDIV_MIN  24U
#define PLL_VDIV_MAX  55U
/* The divided PLL reference must be within 2-4 MHz */
#define PLL_REF_MIN   2000000
#define PLL_REF_MAX   4000000

struct kinetis_mcg_pll_config {
	STANDARD_CLK_SUBSYS_DATA_DEFINE
	/* MCG_C5, followed by MCG_C6 */
	volatile uint8_t *c5;
};

static volatile uint8_t *kinetis_pll_c6(const struct kinetis_mcg_pll_config *config)
{
	return config->c5 + 1;
}

static uint8_t kinetis_pll_prdiv(const struct kinetis_mcg_pll_config *config)
{
	return ((*config->c5 & MCG_C5_PRDIV0_MASK) >> MCG_C5_PRDIV0_SHIFT) + PLL_PRDIV_MIN;
}

static uint8_t kinetis_pll_vdiv(const struct kinetis_mcg_pll_config *config)
{
	return ((*kinetis_pll_c6(config) & MCG_C6_VDIV0_MASK) >> MCG_C6_VDIV0_SHIFT) + PLL_VDIV_MIN;
}

static clock_freq_t kinetis_pll_rate(clock_freq_t parent_rate, uint8_t prdiv, uint8_t vdiv)
{
	uint64_t rate = ((uint64_t)parent_rate * vdiv) / prdiv;

	return (clock_freq_t)MIN(rate, (uint64_t)INT32_MAX);
}

/* Whether the PLL will run once PLLCLKEN0 is set to @p enable */
static bool kinetis_pll_will_run(bool enable)
{
	return (enable || ((MCG->C6 & MCG_C6_PLLS_MASK) != 0U)) && !kinetis_mcg_lp_bypass();
}

static int kinetis_pll_validate(const struct kinetis_mcg_pll_config *config, uint8_t prdiv,
				uint8_t vdiv, bool enable, clock_freq_t parent_rate)
{
	bool changing = (prdiv != kinetis_pll_prdiv(config)) || (vdiv != kinetis_pll_vdiv(config));
	clock_freq_t ref;

	if (prdiv < PLL_PRDIV_MIN || prdiv > PLL_PRDIV_MAX || vdiv < PLL_VDIV_MIN ||
	    vdiv > PLL_VDIV_MAX) {
		return -EINVAL;
	}
	if (changing && (kinetis_mcg_clkst() == KINETIS_MCG_CLKST_PLL)) {
		/* Don't retune the PLL while it clocks MCGOUTCLK */
		return -EBUSY;
	}
	if (kinetis_pll_will_run(enable)) {
		ref = parent_rate / prdiv;
		if (ref < PLL_REF_MIN || ref > PLL_REF_MAX) {
			return -EINVAL;
		}
	}
	return 0;
}

static int kinetis_pll_apply(const struct clk *clk_hw, uint8_t prdiv, uint8_t vdiv, bool enable)
{
	const struct kinetis_mcg_pll_config *config = clk_hw->hw_data;
	volatile uint8_t *c6 = kinetis_pll_c6(config);
	clock_freq_t parent_rate = clock_management_clk_rate(GET_CLK_PARENT(clk_hw));
	int ret;

	if (parent_rate < 0) {
		return parent_rate;
	}
	ret = kinetis_pll_validate(config, prdiv, vdiv, enable, parent_rate);
	if (ret < 0) {
		return ret;
	}

	if ((prdiv != kinetis_pll_prdiv(config)) || (vdiv != kinetis_pll_vdiv(config))) {
		/* PRDIV and VDIV must not change while the enabled PLL is unlocked */
		if (kinetis_mcg_pll_enabled()) {
			ret = kinetis_mcg_wait_pll_lock();
			if (ret < 0) {
				return ret;
			}
		}
		*config->c5 =
			(*config->c5 & ~MCG_C5_PRDIV0_MASK) | MCG_C5_PRDIV0(prdiv - PLL_PRDIV_MIN);
		*c6 = (*c6 & ~MCG_C6_VDIV0_MASK) | MCG_C6_VDIV0(vdiv - PLL_VDIV_MIN);
	}

	if (enable) {
		*config->c5 |= MCG_C5_PLLCLKEN0_MASK;
	} else {
		*config->c5 &= ~MCG_C5_PLLCLKEN0_MASK;
	}

	if (!kinetis_mcg_pll_enabled()) {
		return 0;
	}
	if (kinetis_mcg_ext_ref_is_crystal()) {
		/* Enabling the PLL requests the oscillator */
		ret = kinetis_mcg_wait_osc_init();
		if (ret < 0) {
			return ret;
		}
	}
	return kinetis_mcg_wait_pll_lock();
}

static clock_freq_t kinetis_pll_recalc_rate(const struct clk *clk_hw, clock_freq_t parent_rate)
{
	const struct kinetis_mcg_pll_config *config = clk_hw->hw_data;

	if (!kinetis_mcg_pll_enabled()) {
		return 0;
	}
	return kinetis_pll_rate(parent_rate, kinetis_pll_prdiv(config), kinetis_pll_vdiv(config));
}

static int kinetis_pll_configure(const struct clk *clk_hw, const void *data)
{
	const struct nxp_kinetis_mcg_pll_cfg *cfg = data;

	return kinetis_pll_apply(clk_hw, cfg->prdiv, cfg->vdiv, cfg->enable != 0U);
}

#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
static clock_freq_t kinetis_pll_configure_recalc(const struct clk *clk_hw, const void *data,
						 clock_freq_t parent_rate)
{
	const struct nxp_kinetis_mcg_pll_cfg *cfg = data;
	bool enable = cfg->enable != 0U;
	int ret;

	ret = kinetis_pll_validate(clk_hw->hw_data, cfg->prdiv, cfg->vdiv, enable, parent_rate);
	if (ret < 0) {
		return ret;
	}
	if (!kinetis_pll_will_run(enable)) {
		return 0;
	}
	return kinetis_pll_rate(parent_rate, cfg->prdiv, cfg->vdiv);
}
#endif

#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
static clock_freq_t kinetis_pll_best_rate(const struct clk *clk_hw, clock_freq_t rate_req,
					  clock_freq_t parent_rate, bool commit)
{
	clock_freq_t best_rate = -ENOTSUP;
	clock_freq_t rate, ref;
	uint8_t best_prdiv = 0, best_vdiv = 0;
	int ret;

	if (rate_req <= 0) {
		if (commit) {
			MCG->C5 &= ~MCG_C5_PLLCLKEN0_MASK;
		}
		/* PLLS keeps the PLL running even without PLLCLKEN0 */
		return kinetis_pll_will_run(false) ? kinetis_pll_recalc_rate(clk_hw, parent_rate)
						   : 0;
	}

	for (uint8_t prdiv = PLL_PRDIV_MIN; prdiv <= PLL_PRDIV_MAX; prdiv++) {
		ref = parent_rate / prdiv;
		if (ref < PLL_REF_MIN || ref > PLL_REF_MAX) {
			continue;
		}
		for (uint8_t vdiv = PLL_VDIV_MIN; vdiv <= PLL_VDIV_MAX; vdiv++) {
			rate = kinetis_pll_rate(parent_rate, prdiv, vdiv);
			if (kinetis_pll_validate(clk_hw->hw_data, prdiv, vdiv, true, parent_rate) <
			    0) {
				continue;
			}
			if (best_rate < 0 || abs(rate - rate_req) < abs(best_rate - rate_req)) {
				best_rate = rate;
				best_prdiv = prdiv;
				best_vdiv = vdiv;
			}
		}
	}

	if (best_rate < 0) {
		/* No PRDIV setting produces a valid PLL reference */
		return best_rate;
	}

	if (commit) {
		ret = kinetis_pll_apply(clk_hw, best_prdiv, best_vdiv, true);
		if (ret < 0) {
			return ret;
		}
	}
	return best_rate;
}
#endif

const struct clock_management_standard_api nxp_kinetis_mcg_pll_api = {
	.shared.configure = kinetis_pll_configure,
	.recalc_rate = kinetis_pll_recalc_rate,
#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
	.configure_recalc = kinetis_pll_configure_recalc,
#endif
#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
	.best_rate = kinetis_pll_best_rate,
#endif
};

#define NXP_KINETIS_MCG_PLL_DEFINE(inst)                                                           \
	const struct kinetis_mcg_pll_config nxp_kinetis_mcg_pll_##inst = {                         \
		STANDARD_CLK_SUBSYS_DATA_INIT(CLOCK_DT_GET(DT_INST_PHANDLE(inst, input))).c5 =     \
			(volatile uint8_t *)DT_INST_REG_ADDR(inst),                                \
	};                                                                                         \
                                                                                                   \
	CLOCK_DT_INST_DEFINE(inst, &nxp_kinetis_mcg_pll_##inst, &nxp_kinetis_mcg_pll_api);

DT_INST_FOREACH_STATUS_OKAY(NXP_KINETIS_MCG_PLL_DEFINE)
