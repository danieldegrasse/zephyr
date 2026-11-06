/*
 * Copyright 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/clock_management/clock_driver.h>
#include <zephyr/sys/util.h>

#define DT_DRV_COMPAT clock_div

enum clock_div_type {
	/* Divisor is the field value plus one */
	CLOCK_DIV_LINEAR,
	/* Divisor is two raised to the field value */
	CLOCK_DIV_POW2,
	CLOCK_DIV_MAX,
};

BUILD_ASSERT(CLOCK_DIV_MAX <= UINT8_MAX, "clock_div_type must fit in uint8_t");

struct clock_div_config {
	STANDARD_CLK_SUBSYS_DATA_DEFINE
	mem_addr_t reg;
	uint8_t offset;
	uint8_t width;
	/* Register access width, in bytes */
	uint8_t io_width;
	uint8_t div_type;
};

static uint32_t clock_div_reg_read(const struct clock_div_config *config)
{
	switch (config->io_width) {
	case 1:
		return sys_read8(config->reg);
	case 2:
		return sys_read16(config->reg);
	default:
		return sys_read32(config->reg);
	}
}

static void clock_div_reg_write(const struct clock_div_config *config, uint32_t val)
{
	switch (config->io_width) {
	case 1:
		sys_write8((uint8_t)val, config->reg);
		break;
	case 2:
		sys_write16((uint16_t)val, config->reg);
		break;
	default:
		sys_write32(val, config->reg);
		break;
	}
}

static uint32_t clock_div_read_field(const struct clock_div_config *config)
{
	uint32_t mask = GENMASK(config->offset + config->width - 1, config->offset);

	return (clock_div_reg_read(config) & mask) >> config->offset;
}

static void clock_div_write_field(const struct clock_div_config *config, uint32_t field)
{
	uint32_t mask = GENMASK(config->offset + config->width - 1, config->offset);
	uint32_t reg = clock_div_reg_read(config);

	reg &= ~mask;
	reg |= (field << config->offset) & mask;
	clock_div_reg_write(config, reg);
}

/* Largest field value usable with this divider */
static uint32_t clock_div_max_field(const struct clock_div_config *config)
{
	uint32_t max_field = BIT(config->width) - 1;

	if (config->div_type == CLOCK_DIV_POW2) {
		/* Keep the resulting divisor within 32 bits */
		max_field = MIN(max_field, 31U);
	}
	return max_field;
}

static uint32_t clock_div_field_to_divisor(const struct clock_div_config *config, uint32_t field)
{
	if (config->div_type == CLOCK_DIV_POW2) {
		return BIT(field);
	}
	return field + 1;
}

/* Returns field value for an exact divisor, or -EINVAL if it can't be encoded */
static int clock_div_divisor_to_field(const struct clock_div_config *config, uint32_t divisor)
{
	uint32_t field;

	if (divisor == 0) {
		return -EINVAL;
	}

	if (config->div_type == CLOCK_DIV_POW2) {
		if (!is_power_of_two(divisor)) {
			return -EINVAL;
		}
		field = find_lsb_set(divisor) - 1;
	} else {
		field = divisor - 1;
	}

	if (field > clock_div_max_field(config)) {
		return -EINVAL;
	}
	return (int)field;
}

static clock_freq_t clock_div_recalc_rate(const struct clk *clk_hw, clock_freq_t parent_rate)
{
	const struct clock_div_config *config = clk_hw->hw_data;

	return parent_rate / clock_div_field_to_divisor(config, clock_div_read_field(config));
}

static int clock_div_configure(const struct clk *clk_hw, const void *data)
{
	const struct clock_div_config *config = clk_hw->hw_data;
	int field = clock_div_divisor_to_field(config, (uint32_t)(uintptr_t)data);

	if (field < 0) {
		return field;
	}

	clock_div_write_field(config, (uint32_t)field);
	return 0;
}

#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
static clock_freq_t clock_div_configure_recalc(const struct clk *clk_hw, const void *data,
					       clock_freq_t parent_rate)
{
	const struct clock_div_config *config = clk_hw->hw_data;
	uint32_t divisor = (uint32_t)(uintptr_t)data;
	int field = clock_div_divisor_to_field(config, divisor);

	if (field < 0) {
		return field;
	}

	return parent_rate / divisor;
}
#endif

#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
static clock_freq_t clock_div_best_rate(const struct clk *clk_hw, clock_freq_t rate_req,
					clock_freq_t parent_rate, bool commit)
{
	const struct clock_div_config *config = clk_hw->hw_data;
	uint32_t max_div = clock_div_field_to_divisor(config, clock_div_max_field(config));
	uint32_t divisor;

	if (rate_req <= 0 || parent_rate <= 0) {
		return 0;
	}

	/* Pick the smallest divisor that does not exceed the requested rate */
	divisor = DIV_ROUND_UP((uint32_t)parent_rate, (uint32_t)rate_req);
	if (config->div_type == CLOCK_DIV_POW2 && divisor > 1) {
		/* Round up to the next power of two */
		divisor = (divisor > BIT(31)) ? max_div : BIT(find_msb_set(divisor - 1));
	}
	divisor = CLAMP(divisor, 1U, max_div);

	if (commit) {
		clock_div_write_field(config, (uint32_t)clock_div_divisor_to_field(config,
										   divisor));
	}

	return parent_rate / divisor;
}
#endif

const struct clock_management_standard_api clock_div_api = {
	.shared.configure = clock_div_configure,
	.recalc_rate = clock_div_recalc_rate,
#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
	.configure_recalc = clock_div_configure_recalc,
#endif
#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
	.best_rate = clock_div_best_rate,
#endif
};

#define CLOCK_DIV_TYPE(inst) _CONCAT(CLOCK_DIV_, DT_INST_STRING_UPPER_TOKEN(inst, div_type))

#define CLOCK_DIV_DEFINE(inst)                                                                     \
	BUILD_ASSERT(DT_INST_REG_SIZE(inst) > 0 && DT_INST_REG_SIZE(inst) < 32,                    \
		     "Divider bitfield width must be between 1 and 31 bits");                      \
	BUILD_ASSERT(DT_INST_PROP(inst, offset) + DT_INST_REG_SIZE(inst) <=                        \
			     (DT_INST_PROP(inst, reg_io_width) * 8),                               \
		     "Divider bitfield does not fit within register access width");                \
	const struct clock_div_config clock_div_##inst = {                                         \
		STANDARD_CLK_SUBSYS_DATA_INIT(CLOCK_DT_GET(DT_INST_PHANDLE(inst, input)))          \
		.reg = DT_INST_REG_ADDR(inst),                                                     \
		.offset = (uint8_t)DT_INST_PROP(inst, offset),                                     \
		.width = (uint8_t)DT_INST_REG_SIZE(inst),                                          \
		.io_width = (uint8_t)DT_INST_PROP(inst, reg_io_width),                             \
		.div_type = CLOCK_DIV_TYPE(inst),                                                  \
	};                                                                                         \
                                                                                                   \
	CLOCK_DT_INST_DEFINE(inst, &clock_div_##inst, &clock_div_api);

DT_INST_FOREACH_STATUS_OKAY(CLOCK_DIV_DEFINE)
