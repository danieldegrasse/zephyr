/*
 * Copyright 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>

#include <zephyr/drivers/clock_management/clock_driver.h>
#include <zephyr/sys/util.h>

#define DT_DRV_COMPAT nxp_kinetis_sim_frac_div

/*
 * SIM fractional divider, such as SIM_CLKDIV2[USBFRAC, USBDIV]:
 * output = input * (FRAC + 1) / (DIV + 1)
 */
#define SIM_FRAC_MASK     BIT(0)
#define SIM_DIV_SHIFT     1U
#define SIM_DIV_MASK      GENMASK(3, 1)
#define SIM_FRAC_MULT_MAX 2U
#define SIM_FRAC_DIV_MAX  8U

#define SIM_FRAC_DIV_DATA_DIV_MASK GENMASK(NXP_KINETIS_SIM_FRAC_DIV_MULT_SHIFT - 1, 0)

struct kinetis_sim_frac_div_config {
	STANDARD_CLK_SUBSYS_DATA_DEFINE
	mem_addr_t reg;
};

static int kinetis_frac_div_validate(uint32_t mult, uint32_t div)
{
	if (mult == 0 || mult > SIM_FRAC_MULT_MAX || div == 0 || div > SIM_FRAC_DIV_MAX) {
		return -EINVAL;
	}
	/* This is a divider- the output may not exceed the input */
	if (mult > div) {
		return -EINVAL;
	}
	return 0;
}

static void kinetis_frac_div_write(const struct kinetis_sim_frac_div_config *config, uint32_t mult,
				   uint32_t div)
{
	uint32_t reg = sys_read32(config->reg) & ~(SIM_FRAC_MASK | SIM_DIV_MASK);

	reg |= (mult - 1) | ((div - 1) << SIM_DIV_SHIFT);
	sys_write32(reg, config->reg);
}

static clock_freq_t kinetis_frac_div_rate(clock_freq_t parent_rate, uint32_t mult, uint32_t div)
{
	return (clock_freq_t)(((uint64_t)parent_rate * mult) / div);
}

static clock_freq_t kinetis_frac_div_recalc_rate(const struct clk *clk_hw, clock_freq_t parent_rate)
{
	const struct kinetis_sim_frac_div_config *config = clk_hw->hw_data;
	uint32_t reg = sys_read32(config->reg);
	uint32_t mult = (reg & SIM_FRAC_MASK) + 1;
	uint32_t div = ((reg & SIM_DIV_MASK) >> SIM_DIV_SHIFT) + 1;

	return kinetis_frac_div_rate(parent_rate, mult, div);
}

static int kinetis_frac_div_configure(const struct clk *clk_hw, const void *data)
{
	uint32_t cfg = (uint32_t)(uintptr_t)data;
	uint32_t mult = cfg >> NXP_KINETIS_SIM_FRAC_DIV_MULT_SHIFT;
	uint32_t div = cfg & SIM_FRAC_DIV_DATA_DIV_MASK;
	int ret = kinetis_frac_div_validate(mult, div);

	if (ret < 0) {
		return ret;
	}
	kinetis_frac_div_write(clk_hw->hw_data, mult, div);
	return 0;
}

#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
static clock_freq_t kinetis_frac_div_configure_recalc(const struct clk *clk_hw, const void *data,
						      clock_freq_t parent_rate)
{
	uint32_t cfg = (uint32_t)(uintptr_t)data;
	uint32_t mult = cfg >> NXP_KINETIS_SIM_FRAC_DIV_MULT_SHIFT;
	uint32_t div = cfg & SIM_FRAC_DIV_DATA_DIV_MASK;
	int ret = kinetis_frac_div_validate(mult, div);

	ARG_UNUSED(clk_hw);

	if (ret < 0) {
		return ret;
	}
	return kinetis_frac_div_rate(parent_rate, mult, div);
}
#endif

#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
static clock_freq_t kinetis_frac_div_best_rate(const struct clk *clk_hw, clock_freq_t rate_req,
					       clock_freq_t parent_rate, bool commit)
{
	uint32_t best_mult = 1, best_div = 1;
	clock_freq_t best_rate = parent_rate;
	clock_freq_t rate;

	for (uint32_t mult = 1; mult <= SIM_FRAC_MULT_MAX; mult++) {
		for (uint32_t div = mult; div <= SIM_FRAC_DIV_MAX; div++) {
			rate = kinetis_frac_div_rate(parent_rate, mult, div);
			if (abs(rate - rate_req) < abs(best_rate - rate_req)) {
				best_rate = rate;
				best_mult = mult;
				best_div = div;
			}
		}
	}

	if (commit) {
		kinetis_frac_div_write(clk_hw->hw_data, best_mult, best_div);
	}
	return best_rate;
}
#endif

const struct clock_management_standard_api nxp_kinetis_sim_frac_div_api = {
	.shared.configure = kinetis_frac_div_configure,
	.recalc_rate = kinetis_frac_div_recalc_rate,
#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
	.configure_recalc = kinetis_frac_div_configure_recalc,
#endif
#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
	.best_rate = kinetis_frac_div_best_rate,
#endif
};

#define NXP_KINETIS_SIM_FRAC_DIV_DEFINE(inst)                                                      \
	const struct kinetis_sim_frac_div_config nxp_kinetis_sim_frac_div_##inst = {               \
		STANDARD_CLK_SUBSYS_DATA_INIT(CLOCK_DT_GET(DT_INST_PHANDLE(inst, input))).reg =    \
			DT_INST_REG_ADDR(inst),                                                    \
	};                                                                                         \
                                                                                                   \
	CLOCK_DT_INST_DEFINE(inst, &nxp_kinetis_sim_frac_div_##inst, &nxp_kinetis_sim_frac_div_api);

DT_INST_FOREACH_STATUS_OKAY(NXP_KINETIS_SIM_FRAC_DIV_DEFINE)
