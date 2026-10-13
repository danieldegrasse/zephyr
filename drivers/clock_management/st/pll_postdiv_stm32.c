/*
 * Copyright (c) 2026 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/clock_management/clock_driver.h>
#include <zephyr/drivers/clock_management/clock_helpers.h>
#include <zephyr/sys/util.h>

#define DT_DRV_COMPAT st_stm32_pll_postdiv

enum stm32_div_type {
	STM32_DIV_LINEAR,
	STM32_DIV_NO_ODD,
	STM32_DIV_L4P,
	STM32_DIV_L4,
	STM32_DIV_MAX,
};

BUILD_ASSERT(STM32_DIV_MAX <= UINT8_MAX, "stm32_div_type must fit in uint8_t");

struct stm32_pll_postdiv_config {
	STANDARD_CLK_SUBSYS_DATA_DEFINE
	mem_addr_t div_reg;
	mem_addr_t en_reg;
	uint8_t offset;
	uint8_t width;
	uint8_t enable_offset;
	uint8_t postdiv_type;
};

static uint32_t stm32_pll_postdiv_div_to_reg(const struct stm32_pll_postdiv_config *config,
					     int div_val)
{
	switch (config->postdiv_type) {
	case STM32_DIV_LINEAR:
		return div_val - 1;
	case STM32_DIV_NO_ODD:
		return div_val == 1 ? 0 : ((div_val & ~1) - 1);
	case STM32_DIV_L4:
		return find_msb_set(div_val) - 2;
	case STM32_DIV_L4P:
		return CLAMP(div_val, 2, BIT(config->width) - 1);
	default:
		return -EINVAL;
	}
}

static int stm32_pll_postdiv_reg_to_div(const struct stm32_pll_postdiv_config *config,
					uint32_t reg_val)
{
	uint32_t mask = GENMASK(config->offset + config->width - 1, config->offset);
	uint32_t div_val = (reg_val & mask) >> config->offset;

	switch (config->postdiv_type) {
	case STM32_DIV_LINEAR:
		return div_val + 1;
	case STM32_DIV_NO_ODD:
		return div_val == 0 ? 1 : (div_val + 1) & ~1;
	case STM32_DIV_L4:
		return BIT(div_val + 1);
	case STM32_DIV_L4P:
		if (div_val == 0) {
			/* Use PLLP bit */
			return (BIT(17) & reg_val) ? 17 : 7;
		} else {
			return div_val;
		}
	default:
		return -EINVAL;
	}
}

static clock_freq_t stm32_pll_postdiv_recalc_rate(const struct clk *clk_hw,
						  clock_freq_t parent_rate)
{
	const struct stm32_pll_postdiv_config *config = clk_hw->hw_data;
	uint32_t div_val;

	if (!sys_test_bit(config->en_reg, config->enable_offset)) {
		return 0;
	}

	div_val = stm32_pll_postdiv_reg_to_div(config, sys_read32(config->div_reg));

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
	uint32_t reg_val;
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

	reg_val = sys_read32(config->div_reg);
	reg_val &= ~GENMASK(config->offset + config->width - 1, config->offset);
	reg_val |= stm32_pll_postdiv_div_to_reg(config, div_val);
	sys_write32(reg_val, config->div_reg);

	return clock_onoff(config->parent, true);
}

#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
static clock_freq_t stm32_pll_postdiv_configure_recalc(const struct clk *clk_hw, const void *data,
						       clock_freq_t parent_rate)
{
	const struct stm32_pll_postdiv_config *config = clk_hw->hw_data;
	uint32_t div_val = (uint32_t)data;

	if ((div_val & BIT(ST_STM32_PLL_POSTDIV_EN_SHIFT)) == 0) {
		return 0;
	}

	return parent_rate / stm32_pll_postdiv_reg_to_div(config, div_val);
}
#endif

#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
static clock_freq_t stm32_pll_postdiv_best_rate(const struct clk *clk_hw, clock_freq_t rate_req,
						clock_freq_t parent_rate, bool commit)
{
	const struct stm32_pll_postdiv_config *config = clk_hw->hw_data;
	uint32_t max_div = BIT(config->width);
	uint32_t div_val, reg_val, reg;
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

	/* Convert back and forth to get realizable div value */
	reg_val = stm32_pll_postdiv_div_to_reg(config, div_val);
	div_val = stm32_pll_postdiv_reg_to_div(config, reg_val);

	if (commit) {
		reg = sys_read32(config->div_reg);
		reg &= ~GENMASK(config->offset + config->width - 1, config->offset);
		reg |= reg_val;
		sys_write32(reg, config->div_reg);
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

#define STM32_DIV_TYPE_EXPAND(token) _CONCAT(STM32_DIV_, token)
#define STM32_DIV_TYPE(inst) STM32_DIV_TYPE_EXPAND(DT_INST_STRING_UPPER_TOKEN(inst, div_type))

#define STM32_PLL_POSTDIV_DEFINE(inst)                                                             \
	const struct stm32_pll_postdiv_config stm32_pll_postdiv_##inst = {                         \
		STANDARD_CLK_SUBSYS_DATA_INIT(CLOCK_DT_GET(DT_INST_PHANDLE(inst, input)))          \
			.div_reg = DT_INST_REG_ADDR_BY_IDX(inst, 0),                               \
		.en_reg = DT_INST_REG_ADDR_BY_IDX(inst, 1),                                        \
		.offset = (uint8_t)DT_INST_PROP(inst, offset),                                     \
		.width = (uint8_t)DT_INST_PROP(inst, width),                                       \
		.enable_offset = (uint8_t)DT_INST_PROP(inst, enable_offset),                       \
		.postdiv_type = STM32_DIV_TYPE(inst),                                              \
	};                                                                                         \
                                                                                                   \
	CLOCK_DT_INST_DEFINE(inst, &stm32_pll_postdiv_##inst, &stm32_pll_postdiv_api);

DT_INST_FOREACH_STATUS_OKAY(STM32_PLL_POSTDIV_DEFINE)
