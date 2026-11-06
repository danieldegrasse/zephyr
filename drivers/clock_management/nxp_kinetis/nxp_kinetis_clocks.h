/*
 * Copyright 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_CLOCK_MANAGEMENT_NXP_KINETIS_NXP_KINETIS_CLOCKS_H_
#define ZEPHYR_DRIVERS_CLOCK_MANAGEMENT_NXP_KINETIS_NXP_KINETIS_CLOCKS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @cond INTERNAL_HIDDEN */

#define Z_CLOCK_MANAGEMENT_DATA_DEFINE_nxp_kinetis_osc(node_id, prop, idx)
#define Z_CLOCK_MANAGEMENT_DATA_GET_nxp_kinetis_osc(node_id, prop, idx)                           \
	DT_PHA_BY_IDX(node_id, prop, idx, enable)

/** @endcond */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_CLOCK_MANAGEMENT_NXP_KINETIS_NXP_KINETIS_CLOCKS_H_ */
