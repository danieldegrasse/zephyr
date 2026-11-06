/*
 * Copyright 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/clock_management/clock_driver.h>

#include "nxp_kinetis_internal.h"

#define DT_DRV_COMPAT nxp_kinetis_osc

#define OSC_CR_CAP_MASK (OSC_CR_SC2P_MASK | OSC_CR_SC4P_MASK | OSC_CR_SC8P_MASK | OSC_CR_SC16P_MASK)

struct kinetis_osc_config {
	clock_freq_t rate;
	volatile uint8_t *cr;
	/* MCG_C2 RANGE/HGO/EREFS settings, and mask of the bits that matter */
	uint8_t c2_val;
	uint8_t c2_mask;
	/* OSC_CR SCxP load capacitor settings */
	uint8_t cap_load;
};

static bool kinetis_osc_is_configured(const struct kinetis_osc_config *config)
{
	return ((MCG->C2 & config->c2_mask) == config->c2_val) &&
	       ((*config->cr & OSC_CR_CAP_MASK) == config->cap_load);
}

/* The oscillator runs when OSCERCLK is enabled or the MCG requests it */
static bool kinetis_osc_in_use(const struct kinetis_osc_config *config)
{
	if ((*config->cr & OSC_CR_ERCLKEN_MASK) != 0U) {
		return true;
	}
	return (((MCG->C7 & MCG_C7_OSCSEL_MASK) >> MCG_C7_OSCSEL_SHIFT) ==
		KINETIS_MCG_OSCSEL_OSC0) &&
	       kinetis_mcg_ext_ref_in_use();
}

static clock_freq_t kinetis_osc_get_rate(const struct clk *clk_hw)
{
	const struct kinetis_osc_config *config = clk_hw->hw_data;

	return kinetis_osc_is_configured(config) ? config->rate : -ENOTCONN;
}

static int kinetis_osc_configure(const struct clk *clk_hw, const void *data)
{
	const struct kinetis_osc_config *config = clk_hw->hw_data;

	if (!(bool)data) {
		/*
		 * The oscillator has no enable bit- it stops when no consumer
		 * requests it, so it can't be gated here.
		 */
		return -ENOTSUP;
	}

	if (kinetis_osc_is_configured(config)) {
		return 0;
	}

	/* Range, gain and load must not change while the oscillator is running */
	if (kinetis_osc_in_use(config)) {
		return -EBUSY;
	}

	*config->cr = (*config->cr & ~OSC_CR_CAP_MASK) | config->cap_load;
	MCG->C2 = (MCG->C2 & ~(MCG_C2_RANGE_MASK | MCG_C2_HGO_MASK | MCG_C2_EREFS_MASK)) |
		  config->c2_val;

	return 0;
}

#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
static clock_freq_t kinetis_osc_configure_recalc(const struct clk *clk_hw, const void *data)
{
	const struct kinetis_osc_config *config = clk_hw->hw_data;

	return (bool)data ? config->rate : -ENOTSUP;
}
#endif

#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
static clock_freq_t kinetis_osc_best_rate(const struct clk *clk_hw, clock_freq_t rate_req,
					  bool commit)
{
	const struct kinetis_osc_config *config = clk_hw->hw_data;
	int ret;

	ARG_UNUSED(rate_req);

	if (commit) {
		ret = kinetis_osc_configure(clk_hw, (const void *)1);
		if (ret < 0) {
			return ret;
		}
	}
	return config->rate;
}
#endif

const struct clock_management_root_api nxp_kinetis_osc_api = {
	.get_rate = kinetis_osc_get_rate,
	.shared.configure = kinetis_osc_configure,
#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
	.root_configure_recalc = kinetis_osc_configure_recalc,
#endif
#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
	.root_best_rate = kinetis_osc_best_rate,
#endif
};

/* MCG_C2[RANGE] from the oscillator frequency: low (32-40 kHz), high (3-8 MHz), very high */
#define KINETIS_OSC_RANGE(freq) (((freq) <= 39063) ? 0 : (((freq) <= 8000000) ? 1 : 2))

#define KINETIS_OSC_MODE_IS(inst, val) DT_INST_ENUM_HAS_VALUE(inst, mode, val)

#define KINETIS_OSC_C2_VAL(inst)                                                                   \
	(MCG_C2_RANGE(KINETIS_OSC_RANGE(DT_INST_PROP(inst, frequency))) |                          \
	 (KINETIS_OSC_MODE_IS(inst, external) ? 0 : MCG_C2_EREFS_MASK) |                           \
	 (KINETIS_OSC_MODE_IS(inst, high_gain) ? MCG_C2_HGO_MASK : 0))

/* HGO has no effect when an external clock drives EXTAL */
#define KINETIS_OSC_C2_MASK(inst)                                                                  \
	(MCG_C2_RANGE_MASK | MCG_C2_EREFS_MASK |                                                   \
	 (KINETIS_OSC_MODE_IS(inst, external) ? 0 : MCG_C2_HGO_MASK))

#define KINETIS_OSC_CAP(pf)                                                                        \
	((((pf) & 2) ? OSC_CR_SC2P_MASK : 0) | (((pf) & 4) ? OSC_CR_SC4P_MASK : 0) |               \
	 (((pf) & 8) ? OSC_CR_SC8P_MASK : 0) | (((pf) & 16) ? OSC_CR_SC16P_MASK : 0))

#define NXP_KINETIS_OSC_DEFINE(inst)                                                               \
	BUILD_ASSERT((DT_INST_PROP(inst, load_capacitance_pf) % 2) == 0 &&                         \
			     DT_INST_PROP(inst, load_capacitance_pf) <= 30,                        \
		     "load-capacitance-pf must be an even value from 0 to 30");                    \
	const struct kinetis_osc_config nxp_kinetis_osc_##inst = {                                 \
		.rate = DT_INST_PROP(inst, frequency),                                             \
		.cr = (volatile uint8_t *)DT_INST_REG_ADDR(inst),                                  \
		.c2_val = KINETIS_OSC_C2_VAL(inst),                                                \
		.c2_mask = KINETIS_OSC_C2_MASK(inst),                                              \
		.cap_load = KINETIS_OSC_CAP(DT_INST_PROP(inst, load_capacitance_pf)),              \
	};                                                                                         \
                                                                                                   \
	ROOT_CLOCK_DT_INST_DEFINE(inst, &nxp_kinetis_osc_##inst, &nxp_kinetis_osc_api);

DT_INST_FOREACH_STATUS_OKAY(NXP_KINETIS_OSC_DEFINE)
