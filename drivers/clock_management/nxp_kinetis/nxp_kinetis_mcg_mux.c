/*
 * Copyright 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/clock_management/clock_driver.h>
#include <zephyr/drivers/clock_management/clock_helpers.h>

#include "nxp_kinetis_internal.h"

#define DT_DRV_COMPAT nxp_kinetis_mcg_mux

/*
 * Each MCG selection has its own synchronization requirements. The mux type
 * selects how a new selection is waited on, or validated.
 */
enum kinetis_mcg_mux_type {
	/* C2[IRCS]: slow or fast IRC, reflected by S[IRCST] */
	KINETIS_MCG_MUX_IRCS,
	/* C7[OSCSEL]: external reference, must not change while it is in use */
	KINETIS_MCG_MUX_OSCSEL,
	KINETIS_MCG_MUX_MAX,
};

BUILD_ASSERT(KINETIS_MCG_MUX_MAX <= UINT8_MAX, "kinetis_mcg_mux_type must fit in uint8_t");

struct kinetis_mcg_mux_config {
	MUX_CLK_SUBSYS_DATA_DEFINE
	volatile uint8_t *reg;
	uint8_t offset;
	uint8_t width;
	uint8_t type;
};

static uint8_t kinetis_mcg_mux_mask(const struct kinetis_mcg_mux_config *config)
{
	return GENMASK(config->offset + config->width - 1, config->offset);
}

static uint8_t kinetis_mcg_mux_get_sel(const struct kinetis_mcg_mux_config *config)
{
	return (*config->reg & kinetis_mcg_mux_mask(config)) >> config->offset;
}

/* Check if the mux can switch to a new selection */
static int kinetis_mcg_mux_check(const struct kinetis_mcg_mux_config *config, uint32_t sel)
{
	if (sel >= config->parent_cnt) {
		return -EINVAL;
	}
	if ((config->type == KINETIS_MCG_MUX_OSCSEL) && (sel != kinetis_mcg_mux_get_sel(config)) &&
	    kinetis_mcg_ext_ref_in_use()) {
		/* OSCSEL can't change while the external reference clocks the MCG */
		return -EBUSY;
	}
	return 0;
}

/* Wait for a new selection to take effect */
static int kinetis_mcg_mux_sync(const struct kinetis_mcg_mux_config *config, uint8_t sel)
{
	switch (config->type) {
	case KINETIS_MCG_MUX_IRCS:
		/* IRCST only updates while the internal reference clock is enabled */
		if (((MCG->C1 & MCG_C1_IRCLKEN_MASK) == 0U) &&
		    (kinetis_mcg_clkst() != KINETIS_MCG_CLKST_IRC)) {
			return 0;
		}
		return kinetis_wait8(&MCG->S, MCG_S_IRCST_MASK, MCG_S_IRCST(sel));
	default:
		return 0;
	}
}

static int kinetis_mcg_mux_get_parent(const struct clk *clk_hw)
{
	const struct kinetis_mcg_mux_config *config = clk_hw->hw_data;
	uint8_t sel = kinetis_mcg_mux_get_sel(config);

	return (sel < config->parent_cnt) ? sel : -ENOTCONN;
}

static int kinetis_mcg_mux_configure(const struct clk *clk_hw, const void *data)
{
	const struct kinetis_mcg_mux_config *config = clk_hw->hw_data;
	uint32_t sel = (uint32_t)(uintptr_t)data;
	uint8_t mask = kinetis_mcg_mux_mask(config);
	int ret;

	ret = kinetis_mcg_mux_check(config, sel);
	if (ret < 0) {
		return ret;
	}

	*config->reg = (*config->reg & ~mask) | ((sel << config->offset) & mask);
	return kinetis_mcg_mux_sync(config, sel);
}

#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
static int kinetis_mcg_mux_configure_recalc(const struct clk *clk_hw, const void *data)
{
	const struct kinetis_mcg_mux_config *config = clk_hw->hw_data;
	uint32_t sel = (uint32_t)(uintptr_t)data;
	int ret = kinetis_mcg_mux_check(config, sel);

	return (ret < 0) ? ret : (int)sel;
}

static int kinetis_mcg_mux_validate_parent(const struct clk *clk_hw, clock_freq_t parent_freq,
					   uint8_t new_idx)
{
	const struct kinetis_mcg_mux_config *config = clk_hw->hw_data;

	ARG_UNUSED(parent_freq);

	return kinetis_mcg_mux_check(config, new_idx);
}
#endif

#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
static int kinetis_mcg_mux_set_parent(const struct clk *clk_hw, uint8_t new_idx)
{
	return kinetis_mcg_mux_configure(clk_hw, (const void *)(uintptr_t)new_idx);
}
#endif

const struct clock_management_mux_api nxp_kinetis_mcg_mux_api = {
	.shared.configure = kinetis_mcg_mux_configure,
	.get_parent = kinetis_mcg_mux_get_parent,
#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
	.mux_configure_recalc = kinetis_mcg_mux_configure_recalc,
	.mux_validate_parent = kinetis_mcg_mux_validate_parent,
#endif
#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
	.set_parent = kinetis_mcg_mux_set_parent,
#endif
};

#define KINETIS_MCG_MUX_TYPE(inst)                                                                 \
	_CONCAT(KINETIS_MCG_MUX_, DT_INST_STRING_UPPER_TOKEN(inst, mux_type))

#define GET_MUX_INPUT(node_id, prop, idx) CLOCK_DT_GET(DT_PHANDLE_BY_IDX(node_id, prop, idx)),

#define NXP_KINETIS_MCG_MUX_DEFINE(inst)                                                           \
	static const struct clk *const nxp_kinetis_mcg_mux_##inst##_parents[] = {                  \
		DT_INST_FOREACH_PROP_ELEM(inst, input_sources, GET_MUX_INPUT)};                    \
	const struct kinetis_mcg_mux_config nxp_kinetis_mcg_mux_##inst = {                         \
		MUX_CLK_SUBSYS_DATA_INIT(nxp_kinetis_mcg_mux_##inst##_parents,                     \
					 DT_INST_PROP_LEN(inst, input_sources))                    \
			.reg = (volatile uint8_t *)DT_INST_REG_ADDR(inst),                         \
		.offset = (uint8_t)DT_INST_PROP(inst, offset),                                     \
		.width = (uint8_t)DT_INST_REG_SIZE(inst),                                          \
		.type = KINETIS_MCG_MUX_TYPE(inst),                                                \
	};                                                                                         \
                                                                                                   \
	MUX_CLOCK_DT_INST_DEFINE(inst, &nxp_kinetis_mcg_mux_##inst, &nxp_kinetis_mcg_mux_api);

DT_INST_FOREACH_STATUS_OKAY(NXP_KINETIS_MCG_MUX_DEFINE)
