/*
 * Copyright 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/clock_management/clock_driver.h>

#include "nxp_kinetis_internal.h"

#define DT_DRV_COMPAT nxp_kinetis_mcg_fcrdiv

/* FCRDIV divides the fast IRC by 2^FCRDIV, from 1 to 128 */
#define FCRDIV_MAX_DIV 128U

/* MCG_SC status flags that are cleared by writing 1, so must be written as 0 */
#define MCG_SC_W1C_MASK (MCG_SC_LOCS0_MASK | MCG_SC_ATMF_MASK)

struct kinetis_mcg_fcrdiv_config {
	STANDARD_CLK_SUBSYS_DATA_DEFINE
	volatile uint8_t *sc;
};

static uint8_t kinetis_fcrdiv_field(const struct kinetis_mcg_fcrdiv_config *config)
{
	return (*config->sc & MCG_SC_FCRDIV_MASK) >> MCG_SC_FCRDIV_SHIFT;
}

/* The fast IRC runs when selected by IRCS and used for MCGIRCLK or MCGOUTCLK */
static bool kinetis_fast_irc_enabled(void)
{
	return ((MCG->C2 & MCG_C2_IRCS_MASK) != 0U) &&
	       (((MCG->C1 & MCG_C1_IRCLKEN_MASK) != 0U) ||
		(kinetis_mcg_clkst() == KINETIS_MCG_CLKST_IRC));
}

static int kinetis_fcrdiv_validate(uint32_t divisor)
{
	if (divisor == 0 || divisor > FCRDIV_MAX_DIV || !is_power_of_two(divisor)) {
		return -EINVAL;
	}
	return 0;
}

static int kinetis_fcrdiv_write(const struct kinetis_mcg_fcrdiv_config *config, uint32_t divisor)
{
	uint8_t field = find_lsb_set(divisor) - 1;

	if (field == kinetis_fcrdiv_field(config)) {
		return 0;
	}
	/* Changing the divider while the fast IRC is enabled is not supported */
	if (kinetis_fast_irc_enabled()) {
		return -EBUSY;
	}

	*config->sc =
		(*config->sc & ~(MCG_SC_FCRDIV_MASK | MCG_SC_W1C_MASK)) | MCG_SC_FCRDIV(field);
	return 0;
}

static clock_freq_t kinetis_fcrdiv_recalc_rate(const struct clk *clk_hw, clock_freq_t parent_rate)
{
	return parent_rate >> kinetis_fcrdiv_field(clk_hw->hw_data);
}

static int kinetis_fcrdiv_configure(const struct clk *clk_hw, const void *data)
{
	uint32_t divisor = (uint32_t)(uintptr_t)data;
	int ret = kinetis_fcrdiv_validate(divisor);

	if (ret < 0) {
		return ret;
	}
	return kinetis_fcrdiv_write(clk_hw->hw_data, divisor);
}

#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
static clock_freq_t kinetis_fcrdiv_configure_recalc(const struct clk *clk_hw, const void *data,
						    clock_freq_t parent_rate)
{
	const struct kinetis_mcg_fcrdiv_config *config = clk_hw->hw_data;
	uint32_t divisor = (uint32_t)(uintptr_t)data;
	int ret = kinetis_fcrdiv_validate(divisor);

	if (ret < 0) {
		return ret;
	}
	if ((find_lsb_set(divisor) - 1 != kinetis_fcrdiv_field(config)) &&
	    kinetis_fast_irc_enabled()) {
		return -EBUSY;
	}
	return parent_rate / divisor;
}
#endif

#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
static clock_freq_t kinetis_fcrdiv_best_rate(const struct clk *clk_hw, clock_freq_t rate_req,
					     clock_freq_t parent_rate, bool commit)
{
	uint32_t divisor;
	int ret;

	if (rate_req <= 0 || parent_rate <= 0) {
		return 0;
	}

	/* Smallest power of two divisor that does not exceed the requested rate */
	divisor = DIV_ROUND_UP((uint32_t)parent_rate, (uint32_t)rate_req);
	if (divisor > 1) {
		divisor = (divisor > FCRDIV_MAX_DIV) ? FCRDIV_MAX_DIV
						     : BIT(find_msb_set(divisor - 1));
	} else {
		divisor = 1;
	}

	if (commit) {
		ret = kinetis_fcrdiv_write(clk_hw->hw_data, divisor);
		if (ret < 0) {
			return ret;
		}
	}
	return parent_rate / divisor;
}
#endif

const struct clock_management_standard_api nxp_kinetis_mcg_fcrdiv_api = {
	.shared.configure = kinetis_fcrdiv_configure,
	.recalc_rate = kinetis_fcrdiv_recalc_rate,
#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
	.configure_recalc = kinetis_fcrdiv_configure_recalc,
#endif
#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
	.best_rate = kinetis_fcrdiv_best_rate,
#endif
};

#define NXP_KINETIS_MCG_FCRDIV_DEFINE(inst)                                                        \
	const struct kinetis_mcg_fcrdiv_config nxp_kinetis_mcg_fcrdiv_##inst = {                   \
		STANDARD_CLK_SUBSYS_DATA_INIT(CLOCK_DT_GET(DT_INST_PHANDLE(inst, input))).sc =     \
			(volatile uint8_t *)DT_INST_REG_ADDR(inst),                                \
	};                                                                                         \
                                                                                                   \
	CLOCK_DT_INST_DEFINE(inst, &nxp_kinetis_mcg_fcrdiv_##inst, &nxp_kinetis_mcg_fcrdiv_api);

DT_INST_FOREACH_STATUS_OKAY(NXP_KINETIS_MCG_FCRDIV_DEFINE)
