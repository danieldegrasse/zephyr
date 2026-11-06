/*
 * Copyright 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>

#include <zephyr/drivers/clock_management/clock_driver.h>

#include "nxp_kinetis_internal.h"

#define DT_DRV_COMPAT nxp_kinetis_mcg_fll

#define FLL_DRS_NUM 4U

/* FLL multiplication factor, indexed by C4[DRST_DRS] then C4[DMX32] */
static const uint16_t kinetis_fll_factors[FLL_DRS_NUM][2] = {
	{640, 732},
	{1280, 1464},
	{1920, 2197},
	{2560, 2929},
};

struct kinetis_mcg_fll_config {
	STANDARD_CLK_SUBSYS_DATA_DEFINE
	volatile uint8_t *c4;
};

static int kinetis_fll_factor_to_c4(uint32_t factor, uint8_t *drs, uint8_t *dmx32)
{
	for (uint8_t i = 0; i < FLL_DRS_NUM; i++) {
		for (uint8_t j = 0; j < 2; j++) {
			if (kinetis_fll_factors[i][j] == factor) {
				*drs = i;
				*dmx32 = j;
				return 0;
			}
		}
	}
	return -EINVAL;
}

static clock_freq_t kinetis_fll_rate(clock_freq_t parent_rate, uint32_t factor)
{
	uint64_t rate = (uint64_t)parent_rate * factor;

	/* An out of range FLL reference may produce an unrepresentable rate */
	return (clock_freq_t)MIN(rate, (uint64_t)INT32_MAX);
}

static clock_freq_t kinetis_fll_recalc_rate(const struct clk *clk_hw, clock_freq_t parent_rate)
{
	const struct kinetis_mcg_fll_config *config = clk_hw->hw_data;
	uint8_t c4 = *config->c4;
	uint8_t drs = (c4 & MCG_C4_DRST_DRS_MASK) >> MCG_C4_DRST_DRS_SHIFT;
	uint8_t dmx32 = (c4 & MCG_C4_DMX32_MASK) >> MCG_C4_DMX32_SHIFT;

	if (!kinetis_mcg_fll_enabled()) {
		return 0;
	}
	return kinetis_fll_rate(parent_rate, kinetis_fll_factors[drs][dmx32]);
}

static int kinetis_fll_write(const struct kinetis_mcg_fll_config *config, uint32_t factor)
{
	uint8_t drs, dmx32;
	int ret;

	ret = kinetis_fll_factor_to_c4(factor, &drs, &dmx32);
	if (ret < 0) {
		return ret;
	}
	/* DRS writes are ignored while C2[LP] is set */
	if ((MCG->C2 & MCG_C2_LP_MASK) != 0U) {
		return -EBUSY;
	}

	*config->c4 = (*config->c4 & ~(MCG_C4_DRST_DRS_MASK | MCG_C4_DMX32_MASK)) |
		      MCG_C4_DRST_DRS(drs) | MCG_C4_DMX32(dmx32);
	/* DRST reflects the new range once the DCO has switched */
	ret = kinetis_wait8(config->c4, MCG_C4_DRST_DRS_MASK, MCG_C4_DRST_DRS(drs));
	if (ret < 0) {
		return ret;
	}
	kinetis_mcg_fll_acquire_delay();
	return 0;
}

static int kinetis_fll_configure(const struct clk *clk_hw, const void *data)
{
	return kinetis_fll_write(clk_hw->hw_data, (uint32_t)(uintptr_t)data);
}

#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
static clock_freq_t kinetis_fll_configure_recalc(const struct clk *clk_hw, const void *data,
						 clock_freq_t parent_rate)
{
	uint32_t factor = (uint32_t)(uintptr_t)data;
	uint8_t drs, dmx32;
	int ret;

	ARG_UNUSED(clk_hw);

	ret = kinetis_fll_factor_to_c4(factor, &drs, &dmx32);
	if (ret < 0) {
		return ret;
	}
	if ((MCG->C2 & MCG_C2_LP_MASK) != 0U) {
		return -EBUSY;
	}
	return kinetis_mcg_fll_enabled() ? kinetis_fll_rate(parent_rate, factor) : 0;
}
#endif

#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
static clock_freq_t kinetis_fll_best_rate(const struct clk *clk_hw, clock_freq_t rate_req,
					  clock_freq_t parent_rate, bool commit)
{
	uint32_t best_factor = kinetis_fll_factors[0][0];
	clock_freq_t best_rate = kinetis_fll_rate(parent_rate, best_factor);
	clock_freq_t rate;
	int ret;

	if (!kinetis_mcg_fll_enabled()) {
		return 0;
	}

	for (uint8_t i = 0; i < FLL_DRS_NUM; i++) {
		for (uint8_t j = 0; j < 2; j++) {
			rate = kinetis_fll_rate(parent_rate, kinetis_fll_factors[i][j]);
			if (abs(rate - rate_req) < abs(best_rate - rate_req)) {
				best_rate = rate;
				best_factor = kinetis_fll_factors[i][j];
			}
		}
	}

	if (commit) {
		ret = kinetis_fll_write(clk_hw->hw_data, best_factor);
		if (ret < 0) {
			return ret;
		}
	}
	return best_rate;
}
#endif

const struct clock_management_standard_api nxp_kinetis_mcg_fll_api = {
	.shared.configure = kinetis_fll_configure,
	.recalc_rate = kinetis_fll_recalc_rate,
#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
	.configure_recalc = kinetis_fll_configure_recalc,
#endif
#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
	.best_rate = kinetis_fll_best_rate,
#endif
};

#define NXP_KINETIS_MCG_FLL_DEFINE(inst)                                                           \
	const struct kinetis_mcg_fll_config nxp_kinetis_mcg_fll_##inst = {                         \
		STANDARD_CLK_SUBSYS_DATA_INIT(CLOCK_DT_GET(DT_INST_PHANDLE(inst, input)))          \
		.c4 = (volatile uint8_t *)DT_INST_REG_ADDR(inst),                                  \
	};                                                                                         \
                                                                                                   \
	CLOCK_DT_INST_DEFINE(inst, &nxp_kinetis_mcg_fll_##inst, &nxp_kinetis_mcg_fll_api);

DT_INST_FOREACH_STATUS_OKAY(NXP_KINETIS_MCG_FLL_DEFINE)
