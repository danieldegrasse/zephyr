/*
 * Copyright (c) 2026 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/clock_management/clock_driver.h>
#include <zephyr/drivers/clock_management/clock_helpers.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>

#include "stm32_clocks.h"

#define DT_DRV_COMPAT st_stm32l4_pll

/* PLLxCFGR register: PLLN occupies bits [14:8] */
#define PLL_PLLN_SHIFT 8
#define PLL_PLLN_MASK  GENMASK(14, 8)

#define PLL_PLLN_MIN 8
#define PLL_PLLN_MAX 86

struct stm32l4_pll_config {
	STANDARD_CLK_SUBSYS_DATA_DEFINE
	mem_addr_t pllcfgr_reg;
	mem_addr_t cr_reg;
	uint8_t enable_bit;
};

static int stm32l4_pll_on_off(const struct clk *clk_hw, bool on)
{
	const struct stm32l4_pll_config *config = clk_hw->hw_data;
	uint32_t reg;
	bool done;

	reg = sys_read32(config->cr_reg);
	if (on) {
		reg |= BIT(config->enable_bit);
		sys_write32(reg, config->cr_reg);
		done = WAIT_FOR(sys_test_bit(config->cr_reg, config->enable_bit + 1),
				CONFIG_STM32L4_PLL_LOCK_TIMEOUT, NULL);
	} else {
		reg &= ~BIT(config->enable_bit);
		sys_write32(reg, config->cr_reg);
		done = WAIT_FOR(!sys_test_bit(config->cr_reg, config->enable_bit + 1),
				CONFIG_STM32L4_PLL_LOCK_TIMEOUT, NULL);
	}

	return done ? 0 : -ETIMEDOUT;
}

static void stm32l4_pll_write_plln(const struct stm32l4_pll_config *config,
				    uint32_t plln)
{
	uint32_t reg;

	reg = sys_read32(config->pllcfgr_reg);
	reg &= ~PLL_PLLN_MASK;
	reg |= (plln << PLL_PLLN_SHIFT) & PLL_PLLN_MASK;
	sys_write32(reg, config->pllcfgr_reg);
}

static int stm32l4_pll_configure(const struct clk *clk_hw, const void *data)
{
	const struct stm32l4_pll_config *config = clk_hw->hw_data;
	const struct stm32l4_pll_cfg *cfg = data;
	int ret;

	ret = clock_children_check_rate(clk_hw, 0);
	if (ret < 0) {
		return ret;
	}

	stm32l4_pll_on_off(clk_hw, false);
	stm32l4_pll_write_plln(config, cfg->plln);

	if (cfg->enable) {
		return stm32l4_pll_on_off(clk_hw, true);
	}

	return 0;
}

static clock_freq_t stm32l4_pll_recalc_rate(const struct clk *clk_hw,
					     clock_freq_t parent_rate)
{
	const struct stm32l4_pll_config *config = clk_hw->hw_data;
	uint32_t plln;

	if (!sys_test_bit(config->cr_reg, config->enable_bit)) {
		return 0;
	}

	plln = (sys_read32(config->pllcfgr_reg) & PLL_PLLN_MASK) >> PLL_PLLN_SHIFT;

	return parent_rate * plln;
}

#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)

static clock_freq_t stm32l4_pll_configure_recalc(const struct clk *clk_hw,
						  const void *data,
						  clock_freq_t parent_rate)
{
	const struct stm32l4_pll_cfg *cfg = data;
	int ret;

	ret = clock_children_check_rate(clk_hw, 0);
	if (ret < 0) {
		return ret;
	}

	if (!cfg->enable) {
		return 0;
	}

	return parent_rate * cfg->plln;
}

#endif /* CONFIG_CLOCK_MANAGEMENT_RUNTIME */

#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)

static clock_freq_t stm32l4_pll_best_rate(const struct clk *clk_hw,
					   clock_freq_t rate_req,
					   clock_freq_t parent_rate,
					   bool commit)
{
	const struct stm32l4_pll_config *config = clk_hw->hw_data;
	uint32_t plln;
	clock_freq_t vco_rate;
	int ret;

	if (rate_req == 0 || parent_rate == 0) {
		if (commit) {
			ret = clock_children_check_rate(clk_hw, 0);
			if (ret < 0) {
				return ret;
			}
			stm32l4_pll_on_off(clk_hw, false);
		}
		return 0;
	}

	plln = rate_req / parent_rate;
	plln = CLAMP(plln, PLL_PLLN_MIN, PLL_PLLN_MAX);

	vco_rate = parent_rate * plln;

	if (commit) {
		ret = clock_children_check_rate(clk_hw, 0);
		if (ret < 0) {
			return ret;
		}

		stm32l4_pll_on_off(clk_hw, false);
		stm32l4_pll_write_plln(config, plln);
		ret = stm32l4_pll_on_off(clk_hw, true);
		if (ret != 0) {
			return ret;
		}
	}

	return vco_rate;
}

#endif /* CONFIG_CLOCK_MANAGEMENT_SET_RATE */

const struct clock_management_standard_api stm32l4_pll_api = {
	.shared.configure = stm32l4_pll_configure,
	.shared.on_off = stm32l4_pll_on_off,
	.recalc_rate = stm32l4_pll_recalc_rate,
#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
	.configure_recalc = stm32l4_pll_configure_recalc,
#endif
#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
	.best_rate = stm32l4_pll_best_rate,
#endif
};

#define STM32L4_PLL_DEFINE(inst)                                                                   \
	static const struct stm32l4_pll_config stm32l4_pll_cfg_##inst = {                          \
		STANDARD_CLK_SUBSYS_DATA_INIT(CLOCK_DT_GET(DT_INST_PHANDLE(inst, input)))          \
			.pllcfgr_reg = DT_INST_REG_ADDR_BY_IDX(inst, 0),                          \
		.cr_reg = DT_INST_REG_ADDR_BY_IDX(inst, 1),                                       \
		.enable_bit = (uint8_t)DT_INST_PROP(inst, enable_bit),                             \
	};                                                                                         \
                                                                                                   \
	CLOCK_DT_INST_DEFINE(inst, &stm32l4_pll_cfg_##inst, &stm32l4_pll_api);

DT_INST_FOREACH_STATUS_OKAY(STM32L4_PLL_DEFINE)
