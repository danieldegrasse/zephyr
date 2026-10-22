/*
 * Copyright (c) 2026 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/clock_management/clock_driver.h>
#include <zephyr/drivers/clock_management/clock_helpers.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#define DT_DRV_COMPAT st_stm32_osc

struct stm32_osc_config {
	uint8_t enable_offset;
	uint8_t ready_offset;
	uint8_t bypass_offset;
	clock_freq_t rate;
	mem_addr_t reg;
};

static clock_freq_t stm32_osc_get_rate(const struct clk *clk_hw)
{
	const struct stm32_osc_config *config = clk_hw->hw_data;

	return sys_test_bit(config->reg, config->enable_offset) ? config->rate : 0;
}

static int stm32_osc_on_off(const struct clk *clk_hw, bool on)
{
	const struct stm32_osc_config *config = clk_hw->hw_data;
	bool timeout;

	if (on) {
		sys_set_bits(config->reg, config->enable_offset);
		timeout = WAIT_FOR(sys_test_bit(config->reg, config->ready_offset),
				   CONFIG_STM32_CLOCK_READY_TIMEOUT, NULL);
	} else {
		sys_clear_bit(config->reg, config->enable_offset);
		timeout = WAIT_FOR(sys_test_bit(config->reg, config->ready_offset) == 0,
				   CONFIG_STM32_CLOCK_READY_TIMEOUT, NULL);
	}
	return timeout ? -EIO : 0;
}

static int stm32_osc_configure(const struct clk *clk_hw, const void *data)
{
	const struct stm32_osc_config *config = clk_hw->hw_data;
	uint32_t cfg = (uint32_t)data;
	int ret;
	bool bypass = cfg & BIT(ST_STM32_OSC_BYPASS_SHIFT);

	if (bypass == ((bool)sys_test_bit(config->reg, config->bypass_offset)) &&
	    (cfg & BIT(0)) == ((bool)sys_test_bit(config->reg, config->enable_offset))) {
		/* No need to reconfigure */
		return 0;
	}
	/* Check if children can accept this clock gating */
	ret = clock_children_check_rate(clk_hw, 0);
	if (ret != 0) {
		return ret;
	}
	/* Gate clock */
	stm32_osc_on_off(clk_hw, false);
	/* Set bypass bit */
	if (cfg & BIT(ST_STM32_OSC_BYPASS_SHIFT)) {
		sys_set_bit(config->reg, config->bypass_offset);
	} else {
		sys_clear_bit(config->reg, config->bypass_offset);
	}

	/* Start clock if requested */
	return stm32_osc_on_off(clk_hw, cfg & BIT(0));
}

#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
static clock_freq_t stm32_osc_configure_recalc(const struct clk *clk_hw, const void *data)
{
	const struct stm32_osc_config *config = clk_hw->hw_data;
	bool ungate = (bool)data;

	return ungate ? config->rate : 0;
}
#endif

#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
static clock_freq_t stm32_osc_best_rate(const struct clk *clk_hw, clock_freq_t rate_req,
					bool commit)
{
	const struct stm32_osc_config *config = clk_hw->hw_data;
	int ret;

	if (commit) {
		ret = stm32_osc_on_off(clk_hw, rate_req != 0);
		if (ret != 0) {
			return ret;
		}
	}
	return (rate_req != 0) ? config->rate : 0;
}
#endif

const struct clock_management_root_api stm32_osc_api = {
	.get_rate = stm32_osc_get_rate,
	.shared.configure = stm32_osc_configure,
	.shared.on_off = stm32_osc_on_off,
#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
	.root_configure_recalc = stm32_osc_configure_recalc,
#endif
#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
	.root_best_rate = stm32_osc_best_rate,
#endif
};

#define STM32_OSC_DEFINE(inst)                                                                     \
	const struct stm32_osc_config stm32_osc_##inst = {                                         \
		.rate = DT_INST_PROP(inst, frequency),                                             \
		.reg = DT_INST_REG_ADDR(inst),                                                     \
		.enable_offset = (uint8_t)DT_INST_PROP(inst, offset),                              \
		.ready_offset = (uint8_t)DT_INST_PROP(inst, ready_offset),                         \
		.bypass_offset = (uint8_t)DT_INST_PROP(inst, bypass_offset),                       \
	};                                                                                         \
                                                                                                   \
	ROOT_CLOCK_DT_INST_DEFINE(inst, &stm32_osc_##inst, &stm32_osc_api);

DT_INST_FOREACH_STATUS_OKAY(STM32_OSC_DEFINE)
