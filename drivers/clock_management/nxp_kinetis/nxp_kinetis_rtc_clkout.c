/*
 * Copyright 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/clock_management/clock_driver.h>

#include "nxp_kinetis_internal.h"

#define DT_DRV_COMPAT nxp_kinetis_rtc_clkout

/* RTC_CR bit 14 is reserved, and must always be written as 0 */
#define RTC_CR_RESERVED_MASK BIT(14)

struct kinetis_rtc_clkout_config {
	STANDARD_CLK_SUBSYS_DATA_DEFINE
	mem_addr_t reg;
	/* Offset of the active-low gate bit (RTC_CR[CLKO]) */
	uint8_t gate_offset;
};

static clock_freq_t kinetis_rtc_clkout_recalc_rate(const struct clk *clk_hw,
						   clock_freq_t parent_rate)
{
	const struct kinetis_rtc_clkout_config *config = clk_hw->hw_data;
	bool access = kinetis_rtc_access_begin();
	bool gated = (sys_read32(config->reg) & BIT(config->gate_offset)) != 0U;

	kinetis_rtc_access_end(access);
	return gated ? 0 : parent_rate;
}

static void kinetis_rtc_clkout_set(const struct kinetis_rtc_clkout_config *config, bool ungate)
{
	bool access = kinetis_rtc_access_begin();
	uint32_t reg = sys_read32(config->reg) & ~RTC_CR_RESERVED_MASK;

	/* CLKO is active low: clearing it outputs the 32 kHz clock */
	if (ungate) {
		reg &= ~BIT(config->gate_offset);
	} else {
		reg |= BIT(config->gate_offset);
	}
	sys_write32(reg, config->reg);
	kinetis_rtc_access_end(access);
}

static int kinetis_rtc_clkout_configure(const struct clk *clk_hw, const void *data)
{
	kinetis_rtc_clkout_set(clk_hw->hw_data, (bool)data);
	return 0;
}

#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
static clock_freq_t kinetis_rtc_clkout_configure_recalc(const struct clk *clk_hw, const void *data,
							clock_freq_t parent_rate)
{
	ARG_UNUSED(clk_hw);

	return (bool)data ? parent_rate : 0;
}
#endif

#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
static clock_freq_t kinetis_rtc_clkout_best_rate(const struct clk *clk_hw, clock_freq_t rate_req,
						 clock_freq_t parent_rate, bool commit)
{
	if (commit) {
		kinetis_rtc_clkout_set(clk_hw->hw_data, rate_req != 0);
	}
	return (rate_req != 0) ? parent_rate : 0;
}
#endif

const struct clock_management_standard_api nxp_kinetis_rtc_clkout_api = {
	.recalc_rate = kinetis_rtc_clkout_recalc_rate,
	.shared.configure = kinetis_rtc_clkout_configure,
#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
	.configure_recalc = kinetis_rtc_clkout_configure_recalc,
#endif
#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
	.best_rate = kinetis_rtc_clkout_best_rate,
#endif
};

#define NXP_KINETIS_RTC_CLKOUT_DEFINE(inst)                                                        \
	const struct kinetis_rtc_clkout_config nxp_kinetis_rtc_clkout_##inst = {                   \
		STANDARD_CLK_SUBSYS_DATA_INIT(CLOCK_DT_GET(DT_INST_PHANDLE(inst, input))).reg =    \
			DT_INST_REG_ADDR(inst),                                                    \
		.gate_offset = (uint8_t)DT_INST_PROP(inst, offset),                                \
	};                                                                                         \
                                                                                                   \
	CLOCK_DT_INST_DEFINE(inst, &nxp_kinetis_rtc_clkout_##inst, &nxp_kinetis_rtc_clkout_api);

DT_INST_FOREACH_STATUS_OKAY(NXP_KINETIS_RTC_CLKOUT_DEFINE)
