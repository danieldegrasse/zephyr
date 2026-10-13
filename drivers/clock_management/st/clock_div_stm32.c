/*
 * Copyright (c) 2026 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/clock_management/clock_driver.h>
#include <zephyr/sys/util.h>

#define DT_DRV_COMPAT st_stm32_clock_div

enum stm32_div_type {
	STM32_DIV_POW2,
	STM32_DIV_LINEAR,
	STM32_DIV_RTCPRE,
	STM32_DIV_MAX,
};

BUILD_ASSERT(STM32_DIV_MAX <= UINT8_MAX, "stm32_div_type must fit in uint8_t");

struct stm32_clock_div_config {
	STANDARD_CLK_SUBSYS_DATA_DEFINE
	mem_addr_t reg;
	uint8_t offset;
	uint8_t width;
	uint8_t div_type;
};

static void stm32_clock_div_write_field(const struct stm32_clock_div_config *config, uint32_t val)
{
	uint32_t mask = GENMASK(config->offset + config->width - 1, config->offset);
	uint32_t reg = sys_read32(config->reg);

	reg &= ~mask;
	reg |= (val << config->offset) & mask;
	sys_write32(reg, config->reg);
}

static uint32_t stm32_div_field_to_divisor(enum stm32_div_type type, uint32_t field, uint8_t width)
{
	switch (type) {
	case STM32_DIV_POW2:
		/* STM32 divider encodings skip /32, so we need this if-else ladder */
		if (field < BIT(width - 1)) {
			return 1;
		} else if (field < 12) {
			return BIT(field - (BIT(width - 1) - 1));
		} else {
			return BIT(field - (BIT(width - 1) - 2));
		}
	case STM32_DIV_LINEAR:
		return field;
	case STM32_DIV_RTCPRE:
		return (field <= 1) ? 0 : field;
	case STM32_DIV_MAX:
		CODE_UNREACHABLE;
	}
	return 1;
}

static uint32_t stm32_div_divisor_to_field(enum stm32_div_type type, uint32_t divisor,
					   uint8_t width)
{
	uint32_t max_field = BIT(width) - 1;

	switch (type) {
	case STM32_DIV_POW2:
		if (divisor <= 1) {
			return 0;
		} else if (divisor <= 16) {
			return CLAMP(find_msb_set(divisor) + BIT(width - 1) - 2, 0, max_field);
		} else if (divisor <= 32) {
			return CLAMP(find_msb_set(divisor) + BIT(width - 1) - 3, 0, max_field);
		}
	case STM32_DIV_LINEAR:
		return CLAMP(divisor, 1, max_field);
	case STM32_DIV_RTCPRE:
		return CLAMP(divisor, 2, max_field);
	case STM32_DIV_MAX:
		CODE_UNREACHABLE;
	}
	return 0;
}

static clock_freq_t stm32_clock_div_recalc_rate(const struct clk *clk_hw, clock_freq_t parent_rate)
{
	const struct stm32_clock_div_config *config = clk_hw->hw_data;
	uint32_t mask = GENMASK(config->offset + config->width - 1, config->offset);
	uint32_t field = (sys_read32(config->reg) & mask) >> config->offset;
	uint32_t divisor = stm32_div_field_to_divisor(config->div_type, field, config->width);

	if (divisor == 0) {
		return 0;
	}

	return parent_rate / divisor;
}

static int stm32_clock_div_configure(const struct clk *clk_hw, const void *data)
{
	const struct stm32_clock_div_config *config = clk_hw->hw_data;
	uint32_t div_val = (uint32_t)data;
	uint32_t field;

	if (div_val == 0 || (config->div_type == STM32_DIV_POW2 && !is_power_of_two(div_val))) {
		return -EINVAL;
	}

	field = stm32_div_divisor_to_field(config->div_type, div_val, config->width);
	stm32_clock_div_write_field(config, field);

	return 0;
}

#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
static clock_freq_t stm32_clock_div_configure_recalc(const struct clk *clk_hw, const void *data,
						     clock_freq_t parent_rate)
{
	const struct stm32_clock_div_config *config = clk_hw->hw_data;
	uint32_t div_val = (uint32_t)data;

	if (div_val == 0 || (config->div_type == STM32_DIV_POW2 && !is_power_of_two(div_val))) {
		return -EINVAL;
	}

	return parent_rate / div_val;
}
#endif

#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
static clock_freq_t stm32_clock_div_best_rate(const struct clk *clk_hw, clock_freq_t rate_req,
					      clock_freq_t parent_rate, bool commit)
{
	const struct stm32_clock_div_config *config = clk_hw->hw_data;
	uint32_t div_target, field, divisor;

	if (rate_req == 0 || parent_rate == 0) {
		return 0;
	}

	div_target = DIV_ROUND_UP(parent_rate, rate_req);
	field = stm32_div_divisor_to_field(config->div_type, div_target, config->width);
	divisor = stm32_div_field_to_divisor(config->div_type, field, config->width);

	if (divisor == 0) {
		return 0;
	}

	if (commit) {
		stm32_clock_div_write_field(config, field);
	}

	return parent_rate / divisor;
}
#endif

const struct clock_management_standard_api stm32_clock_div_api = {
	.shared.configure = stm32_clock_div_configure,
	.recalc_rate = stm32_clock_div_recalc_rate,
#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
	.configure_recalc = stm32_clock_div_configure_recalc,
#endif
#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
	.best_rate = stm32_clock_div_best_rate,
#endif
};

#define STM32_DIV_TYPE_EXPAND(token) _CONCAT(STM32_DIV_, token)
#define STM32_DIV_TYPE(inst) STM32_DIV_TYPE_EXPAND(DT_INST_STRING_UPPER_TOKEN(inst, div_type))

#define STM32_CLOCK_DIV_DEFINE(inst)                                                               \
	const struct stm32_clock_div_config stm32_clock_div_##inst = {                             \
		STANDARD_CLK_SUBSYS_DATA_INIT(CLOCK_DT_GET(DT_INST_PHANDLE(inst, input))).reg =    \
			DT_INST_REG_ADDR(inst),                                                    \
		.offset = (uint8_t)DT_INST_PROP(inst, offset),                                     \
		.width = (uint8_t)DT_INST_PROP(inst, width),                                       \
		.div_type = STM32_DIV_TYPE(inst),                                                  \
	};                                                                                         \
                                                                                                   \
	CLOCK_DT_INST_DEFINE(inst, &stm32_clock_div_##inst, &stm32_clock_div_api);

DT_INST_FOREACH_STATUS_OKAY(STM32_CLOCK_DIV_DEFINE)
