/*
 * Copyright 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/clock_management/clock_driver.h>

#include "nxp_kinetis_internal.h"

#define DT_DRV_COMPAT nxp_kinetis_rtc_osc

#define RTC_CR_CAP_MASK (RTC_CR_SC2P_MASK | RTC_CR_SC4P_MASK | RTC_CR_SC8P_MASK | RTC_CR_SC16P_MASK)
/* RTC_CR bit 14 is reserved, and must always be written as 0 */
#define RTC_CR_RESERVED_MASK BIT(14)

struct kinetis_rtc_osc_config {
	clock_freq_t rate;
	mem_addr_t cr;
	/* RTC_CR SCxP load capacitor settings */
	uint32_t cap_load;
};

static clock_freq_t kinetis_rtc_osc_get_rate(const struct clk *clk_hw)
{
	const struct kinetis_rtc_osc_config *config = clk_hw->hw_data;
	bool access = kinetis_rtc_access_begin();
	bool enabled = (sys_read32(config->cr) & RTC_CR_OSCE_MASK) != 0U;

	kinetis_rtc_access_end(access);
	return enabled ? config->rate : 0;
}

static int kinetis_rtc_osc_configure(const struct clk *clk_hw, const void *data)
{
	const struct kinetis_rtc_osc_config *config = clk_hw->hw_data;
	bool enable = (bool)data;
	bool access = kinetis_rtc_access_begin();
	uint32_t cr = sys_read32(config->cr) & ~RTC_CR_RESERVED_MASK;
	bool was_enabled = (cr & RTC_CR_OSCE_MASK) != 0U;

	if (enable) {
		cr = (cr & ~RTC_CR_CAP_MASK) | config->cap_load | RTC_CR_OSCE_MASK;
	} else {
		cr &= ~RTC_CR_OSCE_MASK;
	}
	sys_write32(cr, config->cr);
	kinetis_rtc_access_end(access);

	if (enable && !was_enabled && (CONFIG_NXP_KINETIS_RTC_OSC_STARTUP_DELAY > 0)) {
		/* The RTC oscillator has no ready flag, so wait the startup time */
		k_busy_wait(CONFIG_NXP_KINETIS_RTC_OSC_STARTUP_DELAY);
	}
	return 0;
}

#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
static clock_freq_t kinetis_rtc_osc_configure_recalc(const struct clk *clk_hw, const void *data)
{
	const struct kinetis_rtc_osc_config *config = clk_hw->hw_data;

	return (bool)data ? config->rate : 0;
}
#endif

#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
static clock_freq_t kinetis_rtc_osc_best_rate(const struct clk *clk_hw, clock_freq_t rate_req,
					      bool commit)
{
	const struct kinetis_rtc_osc_config *config = clk_hw->hw_data;

	if (commit) {
		kinetis_rtc_osc_configure(clk_hw, (const void *)(uintptr_t)(rate_req != 0));
	}
	return (rate_req != 0) ? config->rate : 0;
}
#endif

const struct clock_management_root_api nxp_kinetis_rtc_osc_api = {
	.get_rate = kinetis_rtc_osc_get_rate,
	.shared.configure = kinetis_rtc_osc_configure,
#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
	.root_configure_recalc = kinetis_rtc_osc_configure_recalc,
#endif
#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
	.root_best_rate = kinetis_rtc_osc_best_rate,
#endif
};

#define KINETIS_RTC_OSC_CAP(pf)                                                                    \
	((((pf) & 2) ? RTC_CR_SC2P_MASK : 0) | (((pf) & 4) ? RTC_CR_SC4P_MASK : 0) |               \
	 (((pf) & 8) ? RTC_CR_SC8P_MASK : 0) | (((pf) & 16) ? RTC_CR_SC16P_MASK : 0))

#define NXP_KINETIS_RTC_OSC_DEFINE(inst)                                                           \
	BUILD_ASSERT((DT_INST_PROP(inst, load_capacitance_pf) % 2) == 0 &&                         \
			     DT_INST_PROP(inst, load_capacitance_pf) <= 30,                        \
		     "load-capacitance-pf must be an even value from 0 to 30");                    \
	const struct kinetis_rtc_osc_config nxp_kinetis_rtc_osc_##inst = {                         \
		.rate = DT_INST_PROP(inst, frequency),                                             \
		.cr = DT_INST_REG_ADDR(inst),                                                      \
		.cap_load = KINETIS_RTC_OSC_CAP(DT_INST_PROP(inst, load_capacitance_pf)),          \
	};                                                                                         \
                                                                                                   \
	ROOT_CLOCK_DT_INST_DEFINE(inst, &nxp_kinetis_rtc_osc_##inst, &nxp_kinetis_rtc_osc_api);

DT_INST_FOREACH_STATUS_OKAY(NXP_KINETIS_RTC_OSC_DEFINE)
