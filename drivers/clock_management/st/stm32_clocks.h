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

/** @cond INTERNAL_HIDDEN */

#define Z_CLOCK_MANAGEMENT_DATA_DEFINE_st_stm32_osc(node_id, prop, idx)
#define Z_CLOCK_MANAGEMENT_DATA_GET_st_stm32_osc(node_id, prop, idx)                               \
	((DT_PHA_BY_IDX(node_id, prop, idx, bypass) << ST_STM32_OSC_BYPASS_SHIFT) |                \
	 DT_PHA_BY_IDX(node_id, prop, idx, gate))

#define Z_CLOCK_MANAGEMENT_DATA_DEFINE_st_stm32_clock_source(node_id, prop, idx)
#define Z_CLOCK_MANAGEMENT_DATA_GET_st_stm32_clock_source(node_id, prop, idx)                      \
	DT_PHA_BY_IDX(node_id, prop, idx, gate)


/** @endcond */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_CLOCK_MANAGEMENT_ST_STM32_CLOCKS_H_ */
