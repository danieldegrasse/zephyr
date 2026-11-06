/*
 * Copyright 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/clock_management/clock_driver.h>

#include "nxp_kinetis_internal.h"

#define DT_DRV_COMPAT nxp_kinetis_clock_gate

struct kinetis_clock_gate_config {
	STANDARD_CLK_SUBSYS_DATA_DEFINE
	volatile uint8_t *reg;
	uint8_t enable_offset;
	bool wait_osc_init;
};

static clock_freq_t kinetis_clock_gate_recalc_rate(const struct clk *clk_hw,
						   clock_freq_t parent_rate)
{
	const struct kinetis_clock_gate_config *config = clk_hw->hw_data;

	return (*config->reg & BIT(config->enable_offset)) ? parent_rate : 0;
}

static int kinetis_clock_gate_set(const struct kinetis_clock_gate_config *config, bool ungate)
{
	if (ungate) {
		*config->reg |= BIT(config->enable_offset);
		/* OSCINIT0 only asserts when the oscillator is in crystal mode */
		if (config->wait_osc_init && ((MCG->C2 & MCG_C2_EREFS_MASK) != 0U)) {
			return kinetis_mcg_wait_osc_init();
		}
	} else {
		*config->reg &= ~BIT(config->enable_offset);
	}
	return 0;
}

static int kinetis_clock_gate_configure(const struct clk *clk_hw, const void *data)
{
	return kinetis_clock_gate_set(clk_hw->hw_data, (bool)data);
}

#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
static clock_freq_t kinetis_clock_gate_configure_recalc(const struct clk *clk_hw, const void *data,
							clock_freq_t parent_rate)
{
	ARG_UNUSED(clk_hw);

	return (bool)data ? parent_rate : 0;
}
#endif

#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
static clock_freq_t kinetis_clock_gate_best_rate(const struct clk *clk_hw, clock_freq_t rate_req,
						 clock_freq_t parent_rate, bool commit)
{
	int ret;

	if (commit) {
		ret = kinetis_clock_gate_set(clk_hw->hw_data, rate_req != 0);
		if (ret < 0) {
			return ret;
		}
	}
	return (rate_req != 0) ? parent_rate : 0;
}
#endif

const struct clock_management_standard_api nxp_kinetis_clock_gate_api = {
	.recalc_rate = kinetis_clock_gate_recalc_rate,
	.shared.configure = kinetis_clock_gate_configure,
#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
	.configure_recalc = kinetis_clock_gate_configure_recalc,
#endif
#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
	.best_rate = kinetis_clock_gate_best_rate,
#endif
};

#define NXP_KINETIS_CLOCK_GATE_DEFINE(inst)                                                        \
	const struct kinetis_clock_gate_config nxp_kinetis_clock_gate_##inst = {                   \
		STANDARD_CLK_SUBSYS_DATA_INIT(CLOCK_DT_GET(DT_INST_PHANDLE(inst, input))).reg =    \
			(volatile uint8_t *)DT_INST_REG_ADDR(inst),                                \
		.enable_offset = (uint8_t)DT_INST_PROP(inst, offset),                              \
		.wait_osc_init = DT_INST_PROP(inst, wait_osc_init),                                \
	};                                                                                         \
                                                                                                   \
	CLOCK_DT_INST_DEFINE(inst, &nxp_kinetis_clock_gate_##inst, &nxp_kinetis_clock_gate_api);

DT_INST_FOREACH_STATUS_OKAY(NXP_KINETIS_CLOCK_GATE_DEFINE)
