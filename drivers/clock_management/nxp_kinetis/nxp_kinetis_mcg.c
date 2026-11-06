/*
 * Copyright (c) 2026 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdint.h>
#include <zephyr/drivers/clock_management/clock_driver.h>
#include <zephyr/kernel.h>

#include "nxp_kinetis_internal.h"

#define DT_DRV_COMPAT nxp_kinetis_mcg_clk

/* MCG_S[CLKST] encodings */
#define KINETIS_MCG_CLKST_FLL 0U
#define KINETIS_MCG_CLKST_IRC 1U
#define KINETIS_MCG_CLKST_EXT 2U
#define KINETIS_MCG_CLKST_PLL 3U

#define NXP_KINETIS_SIRC_FREQ 32768U
#define NXP_KINETIS_FIRC_FREQ MHZ(4)

/** Shared functions  */

static inline void nxp_kinetis_wait_fll_lock(void)
{
	/*
	 * Wait for FLL stability delay. Per K22 datasheet, this is 1 ms.
	 * kernel timing APIs may not be reliable in FEI mode though.
	 * The fastest possible system clock possible from the FLL is 100MHz,
	 * so delay for 100k CPU cycles to ensure the FLL will always lock
	 */
	for (volatile int i = 0; i < 100000; i++) {
		__asm__ volatile("nop");
	}
}

#ifdef CONFIG_CLOCK_MANAGEMENT_RUNTIME
static int nxp_kinetis_permit_parent(const struct clk *clk_hw, clock_freq_t new_freq,
				     uint8_t parent)
{
	return 0;
}
#endif

/* ===== MCGOUTCLK driver. Modelled as a mux ======= */

struct nxp_kinetis_mcgout_data {
	MUX_CLK_SUBSYS_DATA_DEFINE;
	MCG_Type *mcg;
};

static int nxp_kinetis_mcgout_get_parent(const struct clk *clk_hw)
{
	const struct nxp_kinetis_mcgout_data *data = clk_hw->hw_data;

	return (data->mcg->C1 & MCG_C1_CLKS_MASK) >> MCG_C1_CLKS_SHIFT;
}

#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
static int nxp_kinetis_mcgout_mux_configure_recalc(const struct clk *clk_hw, const void *data)
{
	return (int)data;
}
#endif

#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)

static int nxp_kinetis_mcgout_set_parent(const struct clk *clk_hw, uint8_t parent)
{
	const struct nxp_kinetis_mcgout_data *data = clk_hw->hw_data;
	uint8_t clk_st;
	int ret;

	switch (parent) {
	case 0:
		clk_st = data->mcg->C6 & MCG_C6_PLLS_MASK ? KINETIS_MCG_CLKST_PLL
							  : KINETIS_MCG_CLKST_FLL;
		break;
	case 1:
		clk_st = KINETIS_MCG_CLKST_IRC;
		break;
	case 2:
		clk_st = KINETIS_MCG_CLKST_EXT;
		break;
	default:
		return -ENOTSUP;
	}

	data->mcg->C1 = (data->mcg->C1 & ~MCG_C1_CLKS_MASK) | (parent << MCG_C1_CLKS_SHIFT);

	/* Wait for CLKSTAT to sync */
	if (!WAIT_FOR((data->mcg->S & MCG_S_CLKST_MASK) == MCG_S_CLKST(clk_st),
		      CONFIG_NXP_KINETIS_CLOCK_TIMEOUT, NULL)) {
		return -ETIMEDOUT;
	}

	if (clk_st == KINETIS_MCG_CLKST_FLL) {
		nxp_kinetis_wait_fll_lock();
	} else if (clk_st == KINETIS_MCG_CLKST_PLL) {
		ret = kinetis_mcg_wait_pll_lock();
		if (ret < 0) {
			return ret;
		}
	}
	return 0;
}
#endif

const struct clock_management_mux_api nxp_kinetis_mcgout_api = {
	.get_parent = nxp_kinetis_mcgout_get_parent,
#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
	.mux_configure_recalc = nxp_kinetis_mcgout_mux_configure_recalc,
	.mux_validate_parent = nxp_kinetis_permit_parent,
#endif
#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
	.set_parent = nxp_kinetis_mcgout_set_parent,
#endif
};

#define NXP_KINETIS_MCGOUT_INIT(inst)                                                              \
	static const struct clk *const nxp_kinetis_mcgout_parents_##inst[] = {                     \
		&CLOCK_NAME_GET(nxp_kinetis_plls_clk_##inst),                                      \
		&CLOCK_DT_NAME_GET(DT_INST_PHANDLE(inst, ircs)),                                   \
		&CLOCK_DT_NAME_GET(DT_INST_PHANDLE(inst, oscsel)),                                 \
	};                                                                                         \
	static const struct nxp_kinetis_mcgout_data nxp_kinetis_mcgout_data##inst = {              \
		MUX_CLK_SUBSYS_DATA_INIT(nxp_kinetis_mcgout_parents_##inst,                        \
					 ARRAY_SIZE(nxp_kinetis_mcgout_parents_##inst))            \
			.mcg = (MCG_Type *)DT_INST_REG_ADDR(inst),                                 \
	};                                                                                         \
	MUX_CLOCK_DT_DEFINE(DT_INST_CHILD(inst, mcgoutclk), &nxp_kinetis_mcgout_data##inst,        \
			    &nxp_kinetis_mcgout_api);

/* ===== PLLS driver. Modelled as a mux ======= */

struct nxp_kinetis_plls_data {
	MUX_CLK_SUBSYS_DATA_DEFINE;
	MCG_Type *mcg;
};

static int nxp_kinetis_plls_get_parent(const struct clk *clk_hw)
{
	const struct nxp_kinetis_plls_data *data = clk_hw->hw_data;

	return (data->mcg->C6 & MCG_C6_PLLS_MASK) >> MCG_C6_PLLS_SHIFT;
}

#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
static int nxp_kinetis_plls_mux_configure_recalc(const struct clk *clk_hw, const void *data)
{
	return (int)data;
}
#endif

#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)

static int nxp_kinetis_plls_set_parent(const struct clk *clk_hw, uint8_t parent)
{
	const struct nxp_kinetis_plls_data *data = clk_hw->hw_data;

	if (parent == 0) {
		data->mcg->C6 &= ~MCG_C6_PLLS_MASK;
	} else {
		data->mcg->C6 |= MCG_C6_PLLS_MASK;
	}

	/* Wait for PLLS to update */
	if (!WAIT_FOR((data->mcg->S & MCG_S_PLLST_MASK) == MCG_S_PLLST(parent),
		      CONFIG_NXP_KINETIS_CLOCK_TIMEOUT, NULL)) {
		return -ETIMEDOUT;
	}
	return 0;
}
#endif

const struct clock_management_mux_api nxp_kinetis_plls_api = {
	.get_parent = nxp_kinetis_plls_get_parent,
#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
	.mux_configure_recalc = nxp_kinetis_plls_mux_configure_recalc,
	.mux_validate_parent = nxp_kinetis_permit_parent,
#endif
#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
	.set_parent = nxp_kinetis_plls_set_parent,
#endif
};

#define NXP_KINETIS_PLLS_INIT(inst)                                                                \
	static const struct clk *const nxp_kinetis_plls_parents_##inst[] = {                       \
		CLOCK_DT_GET(DT_INST_CHILD(inst, mcgfllclk)),                                      \
		CLOCK_DT_GET(DT_INST_PHANDLE(inst, pll)),                                          \
	};                                                                                         \
	static const struct nxp_kinetis_plls_data nxp_kinetis_plls_data##inst = {                  \
		MUX_CLK_SUBSYS_DATA_INIT(nxp_kinetis_plls_parents_##inst,                          \
					 ARRAY_SIZE(nxp_kinetis_plls_parents_##inst))              \
			.mcg = (MCG_Type *)DT_INST_REG_ADDR(inst),                                 \
	};                                                                                         \
	MUX_CLOCK_DEFINE(nxp_kinetis_plls_clk_##inst, &nxp_kinetis_plls_data##inst,                \
			 &nxp_kinetis_plls_api,                                                    \
			 CLOCK_DT_NAME_GET(DT_INST_CHILD(inst, mcgoutclk)));

/* ===== FLL driver. Handles DMX32, DRST_DRS fields ======= */

struct nxp_kinetis_fll_data {
	STANDARD_CLK_SUBSYS_DATA_DEFINE;
	MCG_Type *mcg;
};

/* First key is DRST_DRS, second key is DMX32 */
const uint16_t nxp_kinetis_fll_factors[4][2] = {
	[0] = {640, 732},
	[1] = {1280, 1464},
	[2] = {1920, 2197},
	[3] = {2560, 2929},
};

static int nxp_kinetis_fll_configure(const struct clk *clk_hw, const void *data)
{
	const struct nxp_kinetis_fll_data *hw_data = clk_hw->hw_data;
	uint16_t mult_factor = (uint16_t)(uintptr_t)data;

	for (uint8_t i = 0; i < 4; i++) {
		for (uint8_t j = 0; j < 2; j++) {
			if (nxp_kinetis_fll_factors[i][j] == mult_factor) {
				hw_data->mcg->C4 = (hw_data->mcg->C4 &
						    ~(MCG_C4_DRST_DRS_MASK | MCG_C4_DMX32_MASK)) |
						   (i << MCG_C4_DRST_DRS_SHIFT) |
						   (j << MCG_C4_DMX32_SHIFT);
				nxp_kinetis_wait_fll_lock();
				return 0;
			}
		}
	}
	return -EINVAL;
}

static int nxp_kinetis_fll_recalc_rate(const struct clk *clk_hw, clock_freq_t parent_rate)
{
	const struct nxp_kinetis_fll_data *data = clk_hw->hw_data;
	uint8_t drst_drs = (data->mcg->C4 & MCG_C4_DRST_DRS_MASK) >> MCG_C4_DRST_DRS_SHIFT;
	uint8_t dmx32 = (data->mcg->C4 & MCG_C4_DMX32_MASK) >> MCG_C4_DMX32_SHIFT;

	return parent_rate * nxp_kinetis_fll_factors[drst_drs][dmx32];
}

#ifdef CONFIG_CLOCK_MANAGEMENT_RUNTIME
static int nxp_kinetis_fll_configure_recalc(const struct clk *clk_hw, const void *data,
					    clock_freq_t parent_rate)
{
	ARG_UNUSED(clk_hw);
	uint16_t mult_factor = (uint16_t)(uintptr_t)data;

	return parent_rate * mult_factor;
}
#endif

#ifdef CONFIG_CLOCK_MANAGEMENT_SET_RATE
static int nxp_kinetis_fll_best_rate(const struct clk *clk_hw, clock_freq_t rate_req,
				     clock_freq_t parent_rate, bool commit)
{
	const struct nxp_kinetis_fll_data *data = clk_hw->hw_data;
	uint8_t drst_drs, dmx32;
	uint32_t diff = UINT32_MAX;
	clock_freq_t best_rate, cand_rate;

	for (uint8_t i = 0; i < 4; i++) {
		for (uint8_t j = 0; j < 2; j++) {
			cand_rate = nxp_kinetis_fll_factors[i][j] * parent_rate;
			if (abs(cand_rate - rate_req) < diff) {
				diff = abs(cand_rate - rate_req);
				best_rate = cand_rate;
				drst_drs = i;
				dmx32 = j;
			}
		}
	}

	if (commit) {
		data->mcg->C4 = (data->mcg->C4 & ~(MCG_C4_DRST_DRS_MASK | MCG_C4_DMX32_MASK)) |
				(drst_drs << MCG_C4_DRST_DRS_SHIFT) | (dmx32 << MCG_C4_DMX32_SHIFT);
		nxp_kinetis_wait_fll_lock();
	}

	return best_rate;
}
#endif

const struct clock_management_standard_api nxp_kinetis_fll_api = {
	.shared.configure = nxp_kinetis_fll_configure,
	.recalc_rate = nxp_kinetis_fll_recalc_rate,
#ifdef CONFIG_CLOCK_MANAGEMENT_RUNTIME
	.configure_recalc = nxp_kinetis_fll_configure_recalc,
#endif
#ifdef CONFIG_CLOCK_MANAGEMENT_SET_RATE
	.best_rate = nxp_kinetis_fll_best_rate,
#endif
};

#define NXP_KINETIS_FLL_INIT(inst)                                                                 \
	static const struct nxp_kinetis_fll_data nxp_kinetis_fll_data##inst = {                    \
		STANDARD_CLK_SUBSYS_DATA_INIT(CLOCK_DT_GET(DT_INST_CHILD(inst, mcgffclk))).mcg =   \
			(MCG_Type *)DT_INST_REG_ADDR(inst),                                        \
	};                                                                                         \
	CLOCK_DT_DEFINE(DT_INST_CHILD(inst, mcgfllclk), &nxp_kinetis_fll_data##inst,               \
			&nxp_kinetis_fll_api, CLOCK_NAME_GET(nxp_kinetis_plls_clk_##inst));

/* ===== IREFS driver. Modelled as a mux ======= */
struct nxp_kinetis_irefs_data {
	MUX_CLK_SUBSYS_DATA_DEFINE;
	MCG_Type *mcg;
};

static int nxp_kinetis_irefs_get_parent(const struct clk *clk_hw)
{
	const struct nxp_kinetis_irefs_data *data = clk_hw->hw_data;

	return (data->mcg->C1 & MCG_C1_IREFS_MASK) >> MCG_C1_IREFS_SHIFT;
}

#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
static int nxp_kinetis_irefs_mux_configure_recalc(const struct clk *clk_hw, const void *data)
{
	return (int)data;
}
#endif

#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)

static int nxp_kinetis_irefs_set_parent(const struct clk *clk_hw, uint8_t parent)
{
	const struct nxp_kinetis_irefs_data *data = clk_hw->hw_data;
	int ret;

	if (parent == 0) {
		data->mcg->C1 &= ~MCG_C1_IREFS_MASK;
	} else {
		data->mcg->C1 |= MCG_C1_IREFS_MASK;
	}

	/* Wait for IREFS to update */
	if (!WAIT_FOR((data->mcg->S & MCG_S_IREFST_MASK) == MCG_S_IREFST(parent),
		      CONFIG_NXP_KINETIS_CLOCK_TIMEOUT, NULL)) {
		return -ETIMEDOUT;
	}

	/* If using the external reference as a crystal, wait for oscillator initialization */
	if ((parent == 0U) && kinetis_mcg_ext_ref_is_crystal()) {
		ret = kinetis_mcg_wait_osc_init();
		if (ret < 0) {
			return ret;
		}
	}
	return 0;
}
#endif

const struct clock_management_mux_api nxp_kinetis_irefs_api = {
	.get_parent = nxp_kinetis_irefs_get_parent,
#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
	.mux_configure_recalc = nxp_kinetis_irefs_mux_configure_recalc,
	.mux_validate_parent = nxp_kinetis_permit_parent,
#endif
#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
	.set_parent = nxp_kinetis_irefs_set_parent,
#endif
};

#define NXP_KINETIS_IREFS_INIT(inst)                                                               \
	static const struct clk *const nxp_kinetis_irefs_parents_##inst[] = {                      \
		CLOCK_DT_GET(DT_INST_PHANDLE(inst, sirc)),                                         \
		CLOCK_DT_GET(DT_INST_PHANDLE(inst, frdiv)),                                        \
	};                                                                                         \
	static const struct nxp_kinetis_irefs_data nxp_kinetis_irefs_data##inst = {                \
		MUX_CLK_SUBSYS_DATA_INIT(nxp_kinetis_irefs_parents_##inst,                         \
					 ARRAY_SIZE(nxp_kinetis_irefs_parents_##inst))             \
			.mcg = (MCG_Type *)DT_INST_REG_ADDR(inst),                                 \
	};                                                                                         \
	MUX_CLOCK_DT_DEFINE(DT_INST_CHILD(inst, mcgffclk), &nxp_kinetis_irefs_data##inst,          \
			    &nxp_kinetis_irefs_api, CLOCK_NAME_GET(nxp_kinetis_fll_clk_##inst));

/* ===== MCG driver. Modelled as a virtual configuration only clock node ======= */

enum nxp_kinetis_mcg_mode {
	MCG_MODE_FEI,
	MCG_MODE_FEE,
	MCG_MODE_FBI,
	MCG_MODE_FBE,
	MCG_MODE_PEE,
	MCG_MODE_PBE,
	MCG_MODE_BLPI,
	MCG_MODE_BLPE,
	MCG_MODE_END,
};

/* Allowed state transitions for MCG modes */
static const enum nxp_kinetis_mcg_mode nxp_kinetis_mcg_allowed_modes[MCG_MODE_END][MCG_MODE_END] = {
	[MCG_MODE_FEI] = {MCG_MODE_FEI, MCG_MODE_FEE, MCG_MODE_FBI, MCG_MODE_FBE, MCG_MODE_END},
	[MCG_MODE_FEE] = {MCG_MODE_FEI, MCG_MODE_FEE, MCG_MODE_FBI, MCG_MODE_FBE, MCG_MODE_END},
	[MCG_MODE_FBI] = {MCG_MODE_FEI, MCG_MODE_FEE, MCG_MODE_FBI, MCG_MODE_FBE, MCG_MODE_BLPI,
			  MCG_MODE_END},
	[MCG_MODE_FBE] = {MCG_MODE_FEI, MCG_MODE_FEE, MCG_MODE_FBI, MCG_MODE_FBE, MCG_MODE_BLPE,
			  MCG_MODE_PBE, MCG_MODE_END},
	[MCG_MODE_PEE] = {MCG_MODE_PEE, MCG_MODE_PBE, MCG_MODE_END},
	[MCG_MODE_PBE] = {MCG_MODE_PBE, MCG_MODE_PEE, MCG_MODE_BLPE, MCG_MODE_FBE, MCG_MODE_END},
	[MCG_MODE_BLPI] = {MCG_MODE_BLPI, MCG_MODE_FBI, MCG_MODE_END},
	[MCG_MODE_BLPE] = {MCG_MODE_BLPE, MCG_MODE_FBE, MCG_MODE_PBE, MCG_MODE_END},
};

struct nxp_kinetis_mcg_data {
	MCG_Type *mcg;
	const struct clk *irefs;
	const struct clk *plls;
	const struct clk *clks;
	enum nxp_kinetis_mcg_mode *mode;
};

static void nxp_kinetis_mode_settings(enum nxp_kinetis_mcg_mode mode, uint8_t *c1_clks,
				      uint8_t *c1_irefs, uint8_t *c2_lp, uint8_t *c6_plls,
				      uint8_t *s_clkst)
{
	switch (mode) {
	case MCG_MODE_FEI:
		*c1_clks = 0;
		*c1_irefs = 1;
		*c2_lp = 0;
		*c6_plls = 0;
		*s_clkst = KINETIS_MCG_CLKST_FLL;
		break;
	case MCG_MODE_FEE:
		*c1_clks = 0;
		*c1_irefs = 0;
		*c2_lp = 0;
		*c6_plls = 0;
		*s_clkst = KINETIS_MCG_CLKST_FLL;
		break;
	case MCG_MODE_FBI:
		*c1_clks = 1;
		*c1_irefs = 1;
		*c2_lp = 0;
		*c6_plls = 0;
		*s_clkst = KINETIS_MCG_CLKST_IRC;
		break;
	case MCG_MODE_FBE:
		*c1_clks = 2;
		*c1_irefs = 0;
		*c2_lp = 0;
		*c6_plls = 0;
		*s_clkst = KINETIS_MCG_CLKST_EXT;
		break;
	case MCG_MODE_PEE:
		*c1_clks = 0;
		*c1_irefs = 0;
		*c2_lp = 0;
		*c6_plls = 1;
		*s_clkst = KINETIS_MCG_CLKST_PLL;
		break;
	case MCG_MODE_PBE:
		*c1_clks = 2;
		*c1_irefs = 0;
		*c2_lp = 0;
		*c6_plls = 1;
		*s_clkst = KINETIS_MCG_CLKST_EXT;
		break;
	case MCG_MODE_BLPI:
		*c1_clks = 1;
		*c1_irefs = 1;
		*c2_lp = 1;
		*c6_plls = 0;
		*s_clkst = KINETIS_MCG_CLKST_IRC;
		break;
	case MCG_MODE_BLPE:
		*c1_clks = 2;
		*c1_irefs = 0;
		*c2_lp = 1;
		*c6_plls = 0;
		*s_clkst = KINETIS_MCG_CLKST_EXT;
		break;
	default:
		__ASSERT(false, "Invalid MCG mode");
	}
}

static int nxp_kinetis_mcg_program(MCG_Type *mcg, enum nxp_kinetis_mcg_mode mode)
{
	uint8_t c1_clks = 0, c1_irefs = 0, c2_lp = 0, c6_plls = 0, s_clkst = 0;
	int ret;

	nxp_kinetis_mode_settings(mode, &c1_clks, &c1_irefs, &c2_lp, &c6_plls, &s_clkst);
	bool c4_workaround = (mcg->S & MCG_S_IREFST_MASK) == MCG_S_IREFST_MASK;

	/*
	 * Errata: ERR007993
	 * Workaround: Invert MCG_C4[DMX32] or change MCG_C4[DRST_DRS] before
	 * reference clock source changes, then reset to previous value after
	 * reference clock changes.
	 */
	if (c4_workaround) {
		mcg->C4 ^= MCG_C4_DRST_DRS_MASK;
	}

	/* Program C1 */
	mcg->C1 = (mcg->C1 & ~(MCG_C1_CLKS_MASK | MCG_C1_IREFS_MASK)) | MCG_C1_CLKS(c1_clks) |
		  MCG_C1_IREFS(c1_irefs);
	/* Wait for IREFST to update */
	if (!WAIT_FOR((mcg->S & MCG_S_IREFST_MASK) == MCG_S_IREFST(c1_irefs),
		      CONFIG_NXP_KINETIS_CLOCK_TIMEOUT, NULL)) {
		return -ETIMEDOUT;
	}

	if ((c1_irefs == 0) && kinetis_mcg_ext_ref_is_crystal()) {
		ret = kinetis_mcg_wait_osc_init();
		if (ret < 0) {
			return ret;
		}
	}
	/* Program C2 */
	mcg->C2 = (mcg->C2 & ~MCG_C2_LP_MASK) | MCG_C2_LP(c2_lp);
	/* Program C6 */
	mcg->C6 = (mcg->C6 & ~MCG_C6_PLLS_MASK) | MCG_C6_PLLS(c6_plls);
	/* Wait for PLLS to update */
	if (!WAIT_FOR((mcg->S & MCG_S_PLLST_MASK) == MCG_S_PLLST(c6_plls),
		      CONFIG_NXP_KINETIS_CLOCK_TIMEOUT, NULL)) {
		return -ETIMEDOUT;
	}

	/* Errata: ERR007993 */
	if (c4_workaround) {
		mcg->C4 ^= MCG_C4_DRST_DRS_MASK;
	}

	/* Wait for CLKST to update */
	if (!WAIT_FOR((mcg->S & MCG_S_CLKST_MASK) == MCG_S_CLKST(s_clkst),
		      CONFIG_NXP_KINETIS_CLOCK_TIMEOUT, NULL)) {
		return -ETIMEDOUT;
	}

	if ((mcg->S & MCG_S_CLKST_MASK) == MCG_S_CLKST(KINETIS_MCG_CLKST_FLL)) {
		/* Wait for FLL to lock */
		nxp_kinetis_wait_fll_lock();
	} else if ((mcg->S & MCG_S_CLKST_MASK) == MCG_S_CLKST(KINETIS_MCG_CLKST_PLL)) {
		/* Wait for PLL to lock */
		ret = kinetis_mcg_wait_pll_lock();
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static int nxp_kinetis_mcg_configure(const struct clk *clk_hw, const void *data)
{
	const struct nxp_kinetis_mcg_data *clk_data = clk_hw->hw_data;
	const enum nxp_kinetis_mcg_mode mode = (uint32_t)data;
	const enum nxp_kinetis_mcg_mode *allowed_modes =
		nxp_kinetis_mcg_allowed_modes[*clk_data->mode];
	bool transition_allowed = false;
	int ret;

	/* Check if mode transition is allowed */
	for (int i = 0; allowed_modes[i] != MCG_MODE_END; i++) {
		if (allowed_modes[i] == mode) {
			transition_allowed = true;
			break;
		}
	}
	if (!transition_allowed) {
		return -EINVAL;
	}

	/* Perform the mode transition */
	ret = nxp_kinetis_mcg_program(clk_data->mcg, mode);
	if (ret < 0) {
		return ret;
	}
	*clk_data->mode = mode;
	return 0;
}

#ifdef CONFIG_CLOCK_MANAGEMENT_RUNTIME
static bool nxp_kinetis_mcg_query(const struct clk *clk_hw, const void *cfg,
				  const struct clk *tgt_clk, void **data)
{
	const struct nxp_kinetis_mcg_data *clk_data = clk_hw->hw_data;
	const enum nxp_kinetis_mcg_mode mode = (uint32_t)cfg;
	uint8_t c1_clks, c1_irefs, c2_lp, c6_plls, s_clkst;

	if (mode >= MCG_MODE_END) {
		/* Not a valid mode */
		return false;
	}

	/* Read mode settings, used as inputs for clock configure */
	nxp_kinetis_mode_settings(mode, &c1_clks, &c1_irefs, &c2_lp, &c6_plls, &s_clkst);
	if (tgt_clk == clk_data->irefs) {
		*data = (void *)(uintptr_t)c1_irefs;
		return true;
	} else if (tgt_clk == clk_data->plls) {
		*data = (void *)(uintptr_t)c6_plls;
		return true;
	} else if (tgt_clk == clk_data->clks) {
		*data = (void *)(uintptr_t)c1_clks;
		return true;
	}
	return false;
}
#endif

static const struct clock_management_virtual_api nxp_kinetis_mcg_virtual_api = {
	.shared.configure = nxp_kinetis_mcg_configure,
#ifdef CONFIG_CLOCK_MANAGEMENT_RUNTIME
	.query = nxp_kinetis_mcg_query,
#endif
};

#define NXP_KINETIS_MCG_INIT(inst)                                                                 \
	extern const struct clk CLOCK_NAME_GET(nxp_kinetis_plls_clk_##inst);                       \
	extern const struct clk CLOCK_NAME_GET(nxp_kinetis_irefs_clk_##inst);                      \
	NXP_KINETIS_MCGOUT_INIT(inst);                                                             \
	NXP_KINETIS_PLLS_INIT(inst);                                                               \
	NXP_KINETIS_FLL_INIT(inst);                                                                \
	NXP_KINETIS_IREFS_INIT(inst);                                                              \
	static enum nxp_kinetis_mcg_mode nxp_kinetis_mcg_##inst##_mode = MCG_MODE_FEI;             \
	const struct nxp_kinetis_mcg_data nxp_kinetis_mcg_##inst##_data = {                        \
		.mcg = (MCG_Type *)DT_INST_REG_ADDR(inst),                                         \
		.irefs = CLOCK_DT_GET(DT_INST_CHILD(inst, mcgffclk)),                              \
		.plls = &CLOCK_NAME_GET(nxp_kinetis_plls_clk_##inst),                              \
		.clks = CLOCK_DT_GET(DT_INST_CHILD(inst, mcgoutclk)),                              \
		.mode = &nxp_kinetis_mcg_##inst##_mode,                                            \
	};                                                                                         \
	VIRTUAL_CLOCK_DT_INST_DEFINE(inst, &nxp_kinetis_mcg_##inst##_data,                         \
				     &nxp_kinetis_mcg_virtual_api, nxp_kinetis_plls_clk_##inst)

DT_INST_FOREACH_STATUS_OKAY(NXP_KINETIS_MCG_INIT);
