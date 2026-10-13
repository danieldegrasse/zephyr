/*
 * Copyright (c) 2026 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/clock_management/clock_driver.h>
#include <zephyr/drivers/clock_management/clock_helpers.h>
#include <zephyr/sys/util.h>

#define DT_DRV_COMPAT st_stm32_pll_postdiv

struct stm32_pll_postdiv_config {
	STANDARD_CLK_SUBSYS_DATA_DEFINE
	mem_addr_t div_reg;
	mem_addr_t en_reg;
	uint8_t offset;
	uint8_t width;
	uint8_t enable_offset;
	bool no_odd;
};

static int stm32_pll_postdiv_write_div(const struct stm32_pll_postdiv_config *config,
				       uint32_t div_val)
{
	uint32_t mask = GENMASK(config->offset + config->width - 1, config->offset);
	uint32_t reg = sys_read32(config->div_reg);

	if (div_val == 0) {
		return -EINVAL;
	} else if (div_val > BIT(config->width)) {
		return -EINVAL;
	} else if ((config->no_odd) && (div_val > 1) && (div_val & 1)) {
		return -EINVAL;
	}

	reg &= ~mask;
	reg |= ((div_val - 1) << config->offset) & mask;
	sys_write32(reg, config->div_reg);
	return 0;
}

static clock_freq_t stm32_pll_postdiv_recalc_rate(const struct clk *clk_hw,
						  clock_freq_t parent_rate)
{
	const struct stm32_pll_postdiv_config *config = clk_hw->hw_data;
	uint32_t mask, div_val;

	if (!sys_test_bit(config->en_reg, config->enable_offset)) {
		return 0;
	}

	mask = GENMASK(config->offset + config->width - 1, config->offset);
	div_val = ((sys_read32(config->div_reg) & mask) >> config->offset) + 1;

	return parent_rate / div_val;
}

static int stm32_pll_postdiv_on_off(const struct clk *clk_hw, bool on)
{
	const struct stm32_pll_postdiv_config *config = clk_hw->hw_data;

	if (on) {
		sys_set_bit(config->en_reg, config->enable_offset);
	} else {
		sys_clear_bit(config->en_reg, config->enable_offset);
	}

	return 0;
}

static int stm32_pll_postdiv_configure(const struct clk *clk_hw, const void *data)
{
	const struct stm32_pll_postdiv_config *config = clk_hw->hw_data;
	uint32_t div_val = (uint32_t)data & ST_STM32_PLL_POSTDIV_DIV_MASK;
	int ret;

	/* Directly disable the PLL parent before reconfiguring the post-divider */
	ret = clock_children_check_rate(config->parent, 0);
	if (ret < 0) {
		return ret;
	}
	ret = clock_onoff(config->parent, false);
	if (ret < 0) {
		return ret;
	}

	if ((div_val & BIT(ST_STM32_PLL_POSTDIV_EN_SHIFT)) == 0) {
		stm32_pll_postdiv_on_off(clk_hw, false);
	}

	ret = stm32_pll_postdiv_write_div(config, div_val);
	if (ret < 0) {
		return ret;
	}

	return clock_onoff(config->parent, true);
}

#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
static clock_freq_t stm32_pll_postdiv_configure_recalc(const struct clk *clk_hw, const void *data,
						       clock_freq_t parent_rate)
{
	uint32_t div_val = (uint32_t)data;

	if ((div_val & BIT(ST_STM32_PLL_POSTDIV_EN_SHIFT)) == 0) {
		return 0;
	}

	return parent_rate / (div_val & ST_STM32_PLL_POSTDIV_DIV_MASK);
}
#endif

#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
static clock_freq_t stm32_pll_postdiv_best_rate(const struct clk *clk_hw, clock_freq_t rate_req,
						clock_freq_t parent_rate, bool commit)
{
	const struct stm32_pll_postdiv_config *config = clk_hw->hw_data;
	uint32_t max_div = BIT(config->width);
	uint32_t div_val;
	int ret;

	if (commit) {
		/* Directly disable the PLL parent before reconfiguring the post-divider */
		ret = clock_children_check_rate(config->parent, 0);
		if (ret < 0) {
			return ret;
		}
		ret = clock_onoff(config->parent, false);
		if (ret < 0) {
			return ret;
		}
	}

	if (rate_req == 0 || parent_rate == 0) {
		if (commit) {
			stm32_pll_postdiv_on_off(clk_hw, false);
		}
		return 0;
	}

	div_val = DIV_ROUND_UP(parent_rate, rate_req);
	div_val = CLAMP(div_val, 1, max_div);

	if (config->no_odd && (div_val > 1) && (div_val & 1)) {
		div_val++;
		if (unlikely(div_val > max_div)) {
			div_val -= 2;
		}
	}

	if (commit) {
		ret = stm32_pll_postdiv_write_div(config, div_val);
		if (ret < 0) {
			return ret;
		}
		stm32_pll_postdiv_on_off(clk_hw, true);
	}

	/* Reenable PLL parent */
	ret = clock_onoff(config->parent, true);
	if (ret < 0) {
		return ret;
	}

	return parent_rate / div_val;
}
#endif

const struct clock_management_standard_api stm32_pll_postdiv_api = {
	.shared.configure = stm32_pll_postdiv_configure,
	.shared.on_off = stm32_pll_postdiv_on_off,
	.recalc_rate = stm32_pll_postdiv_recalc_rate,
#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
	.configure_recalc = stm32_pll_postdiv_configure_recalc,
#endif
#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
	.best_rate = stm32_pll_postdiv_best_rate,
#endif
};

#define STM32_PLL_POSTDIV_DEFINE(inst)                                                             \
	const struct stm32_pll_postdiv_config stm32_pll_postdiv_##inst = {                         \
		STANDARD_CLK_SUBSYS_DATA_INIT(CLOCK_DT_GET(DT_INST_PHANDLE(inst, input)))          \
			.div_reg = DT_INST_REG_ADDR_BY_IDX(inst, 0),                               \
		.en_reg = DT_INST_REG_ADDR_BY_IDX(inst, 1),                                        \
		.offset = (uint8_t)DT_INST_PROP(inst, offset),                                     \
		.width = (uint8_t)DT_INST_PROP(inst, width),                                       \
		.enable_offset = (uint8_t)DT_INST_PROP(inst, enable_offset),                       \
		.no_odd = DT_INST_PROP(inst, st_no_odd),                                           \
	};                                                                                         \
                                                                                                   \
	CLOCK_DT_INST_DEFINE(inst, &stm32_pll_postdiv_##inst, &stm32_pll_postdiv_api);

DT_INST_FOREACH_STATUS_OKAY(STM32_PLL_POSTDIV_DEFINE)
