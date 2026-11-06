/*
 * Copyright 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/clock_management/clock_driver.h>
#include <zephyr/drivers/clock_management/clock_helpers.h>

#include "nxp_kinetis_internal.h"

#define DT_DRV_COMPAT nxp_kinetis_mcg_clks

/*
 * MCGOUTCLK source select. Configuration data holds the C1[CLKS] selection
 * and the C2[LP] bit, since the bypassed low power modes (BLPI and BLPE) are
 * defined as an IRC or external CLKS selection with LP set.
 */

/* C1[CLKS] selections: FLL/PLL output, internal reference, external reference */
#define KINETIS_MCG_CLKS_FLL_PLL 0U
#define KINETIS_MCG_CLKS_EXT     2U
#define KINETIS_MCG_CLKS_NUM     3U

#define KINETIS_MCG_CLKS_SEL_MASK 0xFFU

struct kinetis_mcg_clks_config {
	MUX_CLK_SUBSYS_DATA_DEFINE
	volatile uint8_t *c1;
};

static int kinetis_mcg_clks_validate(uint32_t data)
{
	uint32_t clks = data & KINETIS_MCG_CLKS_SEL_MASK;
	bool lp = (data >> NXP_KINETIS_MCG_CLKS_LP_SHIFT) != 0U;

	if (clks >= KINETIS_MCG_CLKS_NUM) {
		return -EINVAL;
	}
	/* LP only has an effect in bypass modes, so reject it with an engaged mode */
	if (lp && (clks == KINETIS_MCG_CLKS_FLL_PLL)) {
		return -EINVAL;
	}
	return (int)clks;
}

static int kinetis_mcg_clks_get_parent(const struct clk *clk_hw)
{
	/* CLKST reports the PLL separately, but it is reached via the PLLS mux */
	uint8_t clkst = kinetis_mcg_clkst();

	ARG_UNUSED(clk_hw);

	return (clkst == KINETIS_MCG_CLKST_PLL) ? KINETIS_MCG_CLKS_FLL_PLL : clkst;
}

static int kinetis_mcg_clks_configure(const struct clk *clk_hw, const void *data)
{
	const struct kinetis_mcg_clks_config *config = clk_hw->hw_data;
	uint32_t cfg = (uint32_t)(uintptr_t)data;
	bool lp = (cfg >> NXP_KINETIS_MCG_CLKS_LP_SHIFT) != 0U;
	bool pll_selected = (MCG->S & MCG_S_PLLST_MASK) != 0U;
	uint8_t clkst;
	int clks, ret;

	clks = kinetis_mcg_clks_validate(cfg);
	if (clks < 0) {
		return clks;
	}

	if (!lp && ((MCG->C2 & MCG_C2_LP_MASK) != 0U)) {
		/* Leave BLPI/BLPE first, which restarts the FLL or PLL */
		MCG->C2 &= ~MCG_C2_LP_MASK;
		if (pll_selected) {
			ret = kinetis_mcg_wait_pll_lock();
			if (ret < 0) {
				return ret;
			}
		} else {
			kinetis_mcg_fll_acquire_delay();
		}
	}

	if ((clks == KINETIS_MCG_CLKS_FLL_PLL) && pll_selected) {
		/* Don't switch to the PLL until it has locked */
		ret = kinetis_mcg_wait_pll_lock();
		if (ret < 0) {
			return ret;
		}
	}

	*config->c1 = (*config->c1 & ~MCG_C1_CLKS_MASK) | MCG_C1_CLKS(clks);

	if ((clks == KINETIS_MCG_CLKS_EXT) && kinetis_mcg_ext_ref_is_crystal()) {
		/* Selecting the external reference requests the oscillator */
		ret = kinetis_mcg_wait_osc_init();
		if (ret < 0) {
			return ret;
		}
	}

	if (clks == KINETIS_MCG_CLKS_FLL_PLL) {
		clkst = pll_selected ? KINETIS_MCG_CLKST_PLL : KINETIS_MCG_CLKST_FLL;
	} else {
		clkst = clks;
	}
	ret = kinetis_wait8(&MCG->S, MCG_S_CLKST_MASK, MCG_S_CLKST(clkst));
	if (ret < 0) {
		return ret;
	}

	if (lp) {
		/* Enter BLPI/BLPE once the bypass mode is active */
		MCG->C2 |= MCG_C2_LP_MASK;
	}
	return 0;
}

#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
static int kinetis_mcg_clks_configure_recalc(const struct clk *clk_hw, const void *data)
{
	ARG_UNUSED(clk_hw);

	return kinetis_mcg_clks_validate((uint32_t)(uintptr_t)data);
}

static int kinetis_mcg_clks_validate_parent(const struct clk *clk_hw, clock_freq_t parent_freq,
					    uint8_t new_idx)
{
	const struct kinetis_mcg_clks_config *config = clk_hw->hw_data;

	ARG_UNUSED(parent_freq);

	return clock_management_mux_validate_parent(config->parent_cnt, new_idx);
}
#endif

#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
static int kinetis_mcg_clks_set_parent(const struct clk *clk_hw, uint8_t new_idx)
{
	uint32_t cfg = new_idx;

	/* Keep the current low power setting, unless switching to an engaged mode */
	if ((new_idx != KINETIS_MCG_CLKS_FLL_PLL) && ((MCG->C2 & MCG_C2_LP_MASK) != 0U)) {
		cfg |= BIT(NXP_KINETIS_MCG_CLKS_LP_SHIFT);
	}
	return kinetis_mcg_clks_configure(clk_hw, (const void *)(uintptr_t)cfg);
}
#endif

const struct clock_management_mux_api nxp_kinetis_mcg_clks_api = {
	.shared.configure = kinetis_mcg_clks_configure,
	.get_parent = kinetis_mcg_clks_get_parent,
#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
	.mux_configure_recalc = kinetis_mcg_clks_configure_recalc,
	.mux_validate_parent = kinetis_mcg_clks_validate_parent,
#endif
#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
	.set_parent = kinetis_mcg_clks_set_parent,
#endif
};

#define GET_MUX_INPUT(node_id, prop, idx) CLOCK_DT_GET(DT_PHANDLE_BY_IDX(node_id, prop, idx)),

#define NXP_KINETIS_MCG_CLKS_DEFINE(inst)                                                          \
	BUILD_ASSERT(DT_INST_PROP_LEN(inst, input_sources) == KINETIS_MCG_CLKS_NUM,                \
		     "MCG CLKS mux must have FLL/PLL, internal and external reference inputs");    \
	static const struct clk *const nxp_kinetis_mcg_clks_##inst##_parents[] = {                 \
		DT_INST_FOREACH_PROP_ELEM(inst, input_sources, GET_MUX_INPUT)};                    \
	const struct kinetis_mcg_clks_config nxp_kinetis_mcg_clks_##inst = {                       \
		MUX_CLK_SUBSYS_DATA_INIT(nxp_kinetis_mcg_clks_##inst##_parents,                    \
					 DT_INST_PROP_LEN(inst, input_sources))                    \
		.c1 = (volatile uint8_t *)DT_INST_REG_ADDR(inst),                                  \
	};                                                                                         \
                                                                                                   \
	MUX_CLOCK_DT_INST_DEFINE(inst, &nxp_kinetis_mcg_clks_##inst, &nxp_kinetis_mcg_clks_api);

DT_INST_FOREACH_STATUS_OKAY(NXP_KINETIS_MCG_CLKS_DEFINE)
