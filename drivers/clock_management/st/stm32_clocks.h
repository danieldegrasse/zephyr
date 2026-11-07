/*
 * Copyright (c) 2026 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_CLOCK_MANAGEMENT_ST_STM32_CLOCKS_H_
#define ZEPHYR_DRIVERS_CLOCK_MANAGEMENT_ST_STM32_CLOCKS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ST_STM32_OSC_BYPASS_SHIFT     1
#define ST_STM32_PLL_POSTDIV_EN_SHIFT 31
#define ST_STM32_PLL_POSTDIV_DIV_MASK GENMASK(30, 0)

/** @cond INTERNAL_HIDDEN */

#define Z_CLOCK_MANAGEMENT_DATA_DEFINE_st_stm32_osc(node_id, prop, idx)
#define Z_CLOCK_MANAGEMENT_DATA_GET_st_stm32_osc(node_id, prop, idx)                               \
	((DT_PHA_BY_IDX(node_id, prop, idx, bypass) << ST_STM32_OSC_BYPASS_SHIFT) |                \
	 DT_PHA_BY_IDX(node_id, prop, idx, gate))

#define Z_CLOCK_MANAGEMENT_DATA_DEFINE_st_stm32_clock_source(node_id, prop, idx)
#define Z_CLOCK_MANAGEMENT_DATA_GET_st_stm32_clock_source(node_id, prop, idx)                      \
	DT_PHA_BY_IDX(node_id, prop, idx, gate)

#define Z_CLOCK_MANAGEMENT_DATA_DEFINE_st_stm32_pll_postdiv(node_id, prop, idx)
#define Z_CLOCK_MANAGEMENT_DATA_GET_st_stm32_pll_postdiv(node_id, prop, idx)                       \
	((DT_PHA_BY_IDX(node_id, prop, idx, divider) & ST_STM32_PLL_POSTDIV_DIV_MASK) |            \
	 (DT_PHA_BY_IDX(node_id, prop, idx, enable) << ST_STM32_PLL_POSTDIV_EN_SHIFT))

#define Z_CLOCK_MANAGEMENT_DATA_DEFINE_st_stm32_clock_div(node_id, prop, idx)
#define Z_CLOCK_MANAGEMENT_DATA_GET_st_stm32_clock_div(node_id, prop, idx)                         \
	DT_PHA_BY_IDX(node_id, prop, idx, divider)

struct stm32h7_pll_cfg {
	uint16_t divn;
	uint16_t fracn;
	uint8_t rge;
	uint8_t vcosel;
	uint8_t enable;
};

#define Z_CLOCK_MANAGEMENT_DATA_DEFINE_st_stm32h7_pll(node_id, prop, idx)                          \
	static const struct stm32h7_pll_cfg _CONCAT(stm32h7_pll_cfg_,                              \
						    _CONCAT(DT_DEP_ORD(node_id), idx)) = {         \
		.divn = DT_PHA_BY_IDX(node_id, prop, idx, divn),                                   \
		.fracn = DT_PHA_BY_IDX(node_id, prop, idx, fracn),                                 \
		.rge = DT_PHA_BY_IDX(node_id, prop, idx, rge),                                     \
		.vcosel = DT_PHA_BY_IDX(node_id, prop, idx, vcosel),                               \
		.enable = DT_PHA_BY_IDX(node_id, prop, idx, enable),                               \
	};
#define Z_CLOCK_MANAGEMENT_DATA_GET_st_stm32h7_pll(node_id, prop, idx)                             \
	&_CONCAT(stm32h7_pll_cfg_, _CONCAT(DT_DEP_ORD(node_id), idx))

struct stm32l4_pll_cfg {
	uint8_t plln;
	uint8_t enable;
};

#define Z_CLOCK_MANAGEMENT_DATA_DEFINE_st_stm32l4_pll(node_id, prop, idx)                          \
	static const struct stm32l4_pll_cfg _CONCAT(stm32l4_pll_cfg_,                              \
						    _CONCAT(DT_DEP_ORD(node_id), idx)) = {         \
		.plln = DT_PHA_BY_IDX(node_id, prop, idx, plln),                                   \
		.enable = DT_PHA_BY_IDX(node_id, prop, idx, enable),                               \
	};
#define Z_CLOCK_MANAGEMENT_DATA_GET_st_stm32l4_pll(node_id, prop, idx)                             \
	&_CONCAT(stm32l4_pll_cfg_, _CONCAT(DT_DEP_ORD(node_id), idx))

/** @endcond */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_CLOCK_MANAGEMENT_ST_STM32_CLOCKS_H_ */
