/*
 * Copyright 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/clock_management/clock_driver.h>

#include "nxp_kinetis_internal.h"

#define DT_DRV_COMPAT nxp_kinetis_mcg_frdiv

/* MCG_C7[OSCSEL] encoding for the RTC oscillator */
#define KINETIS_MCG_OSCSEL_RTC 1U

#define FRDIV_FIELD_MAX 7U

#define FRDIV_MIN_FREQ 31250U
#define FRDIV_MAX_FREQ 39062U

struct kinetis_mcg_frdiv_config {
	STANDARD_CLK_SUBSYS_DATA_DEFINE
	volatile uint8_t *c1;
};

/*
 * The FRDIV encoding depends on the external reference: with a low range
 * oscillator or the RTC oscillator it divides by 1-128, otherwise by 32-1536.
 */
static bool kinetis_frdiv_low_range(void)
{
	return ((MCG->C2 & MCG_C2_RANGE_MASK) == 0U) ||
	       (((MCG->C7 & MCG_C7_OSCSEL_MASK) >> MCG_C7_OSCSEL_SHIFT) == KINETIS_MCG_OSCSEL_RTC);
}

static uint32_t kinetis_frdiv_field_to_div(uint8_t field, bool low_range)
{
	if (low_range) {
		return BIT(field);
	}
	if (field < 6U) {
		return 32U << field;
	}
	return (field == 6U) ? 1280U : 1536U;
}

static int kinetis_frdiv_div_to_field(uint32_t divisor, bool low_range)
{
	for (uint8_t field = 0; field <= FRDIV_FIELD_MAX; field++) {
		if (kinetis_frdiv_field_to_div(field, low_range) == divisor) {
			return field;
		}
	}
	return -EINVAL;
}

static uint8_t kinetis_frdiv_field(const struct kinetis_mcg_frdiv_config *config)
{
	return (*config->c1 & MCG_C1_FRDIV_MASK) >> MCG_C1_FRDIV_SHIFT;
}

static void kinetis_frdiv_write(const struct kinetis_mcg_frdiv_config *config, uint8_t field)
{
	*config->c1 = (*config->c1 & ~MCG_C1_FRDIV_MASK) | MCG_C1_FRDIV(field);
}

static clock_freq_t kinetis_frdiv_recalc_rate(const struct clk *clk_hw, clock_freq_t parent_rate)
{
	const struct kinetis_mcg_frdiv_config *config = clk_hw->hw_data;

	return parent_rate /
	       kinetis_frdiv_field_to_div(kinetis_frdiv_field(config), kinetis_frdiv_low_range());
}

static int kinetis_frdiv_configure(const struct clk *clk_hw, const void *data)
{
	int field =
		kinetis_frdiv_div_to_field((uint32_t)(uintptr_t)data, kinetis_frdiv_low_range());

	if (field < 0) {
		return field;
	}
	kinetis_frdiv_write(clk_hw->hw_data, (uint8_t)field);
	return 0;
}

#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
static clock_freq_t kinetis_frdiv_configure_recalc(const struct clk *clk_hw, const void *data,
						   clock_freq_t parent_rate)
{
	uint32_t divisor = (uint32_t)(uintptr_t)data;
	int field = kinetis_frdiv_div_to_field(divisor, kinetis_frdiv_low_range());
	clock_freq_t cand_rate = parent_rate / divisor;

	ARG_UNUSED(clk_hw);

	if (cand_rate < FRDIV_MIN_FREQ || cand_rate > FRDIV_MAX_FREQ) {
		return -EINVAL;
	}

	if (field < 0) {
		return field;
	}
	return cand_rate;
}
#endif

#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
static clock_freq_t kinetis_frdiv_best_rate(const struct clk *clk_hw, clock_freq_t rate_req,
					    clock_freq_t parent_rate, bool commit)
{
	bool low_range = kinetis_frdiv_low_range();
	clock_freq_t cand_rate;
	uint8_t field;

	if (rate_req <= 0 || parent_rate <= 0) {
		return 0;
	}

	/* Divisors increase with the field value, so use the first that fits */
	for (field = 0; field < FRDIV_FIELD_MAX; field++) {
		cand_rate = parent_rate / kinetis_frdiv_field_to_div(field, low_range);
		if (cand_rate < FRDIV_MIN_FREQ || cand_rate > FRDIV_MAX_FREQ) {
			cand_rate = 0;
			continue;
		}
		if (cand_rate <= rate_req) {
			break;
		}
	}
	if (cand_rate == 0) {
		return -ENOENT;
	}

	if (commit) {
		kinetis_frdiv_write(clk_hw->hw_data, field);
	}
	return parent_rate / kinetis_frdiv_field_to_div(field, low_range);
}
#endif

const struct clock_management_standard_api nxp_kinetis_mcg_frdiv_api = {
	.shared.configure = kinetis_frdiv_configure,
	.recalc_rate = kinetis_frdiv_recalc_rate,
#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
	.configure_recalc = kinetis_frdiv_configure_recalc,
#endif
#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
	.best_rate = kinetis_frdiv_best_rate,
#endif
};

#define NXP_KINETIS_MCG_FRDIV_DEFINE(inst)                                                         \
	const struct kinetis_mcg_frdiv_config nxp_kinetis_mcg_frdiv_##inst = {                     \
		STANDARD_CLK_SUBSYS_DATA_INIT(CLOCK_DT_GET(DT_INST_PHANDLE(inst, input))).c1 =     \
			(volatile uint8_t *)DT_INST_REG_ADDR(inst),                                \
	};                                                                                         \
                                                                                                   \
	CLOCK_DT_INST_DEFINE(inst, &nxp_kinetis_mcg_frdiv_##inst, &nxp_kinetis_mcg_frdiv_api);

DT_INST_FOREACH_STATUS_OKAY(NXP_KINETIS_MCG_FRDIV_DEFINE)
