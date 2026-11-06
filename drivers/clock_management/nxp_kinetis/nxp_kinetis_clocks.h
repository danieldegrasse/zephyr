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

/* Bit position of the LP flag within MCG CLKS mux configuration data */
#define NXP_KINETIS_MCG_CLKS_LP_SHIFT 8
/* Bit position of the multiplier within SIM fractional divider configuration data */
#define NXP_KINETIS_SIM_FRAC_DIV_MULT_SHIFT 16

#define Z_CLOCK_MANAGEMENT_DATA_DEFINE_nxp_kinetis_osc(node_id, prop, idx)
#define Z_CLOCK_MANAGEMENT_DATA_GET_nxp_kinetis_osc(node_id, prop, idx)                           \
	DT_PHA_BY_IDX(node_id, prop, idx, enable)

#define Z_CLOCK_MANAGEMENT_DATA_DEFINE_nxp_kinetis_clock_gate(node_id, prop, idx)
#define Z_CLOCK_MANAGEMENT_DATA_GET_nxp_kinetis_clock_gate(node_id, prop, idx)                    \
	DT_PHA_BY_IDX(node_id, prop, idx, gate)

#define Z_CLOCK_MANAGEMENT_DATA_DEFINE_nxp_kinetis_rtc_osc(node_id, prop, idx)
#define Z_CLOCK_MANAGEMENT_DATA_GET_nxp_kinetis_rtc_osc(node_id, prop, idx)                       \
	DT_PHA_BY_IDX(node_id, prop, idx, enable)

#define Z_CLOCK_MANAGEMENT_DATA_DEFINE_nxp_kinetis_rtc_clkout(node_id, prop, idx)
#define Z_CLOCK_MANAGEMENT_DATA_GET_nxp_kinetis_rtc_clkout(node_id, prop, idx)                    \
	DT_PHA_BY_IDX(node_id, prop, idx, gate)

#define Z_CLOCK_MANAGEMENT_DATA_DEFINE_nxp_kinetis_mcg_fcrdiv(node_id, prop, idx)
#define Z_CLOCK_MANAGEMENT_DATA_GET_nxp_kinetis_mcg_fcrdiv(node_id, prop, idx)                    \
	DT_PHA_BY_IDX(node_id, prop, idx, divider)

#define Z_CLOCK_MANAGEMENT_DATA_DEFINE_nxp_kinetis_mcg_mux(node_id, prop, idx)
#define Z_CLOCK_MANAGEMENT_DATA_GET_nxp_kinetis_mcg_mux(node_id, prop, idx)                       \
	DT_PHA_BY_IDX(node_id, prop, idx, mux)

#define Z_CLOCK_MANAGEMENT_DATA_DEFINE_nxp_kinetis_mcg_clks(node_id, prop, idx)
#define Z_CLOCK_MANAGEMENT_DATA_GET_nxp_kinetis_mcg_clks(node_id, prop, idx)                      \
	((DT_PHA_BY_IDX(node_id, prop, idx, lp) << NXP_KINETIS_MCG_CLKS_LP_SHIFT) |               \
	 DT_PHA_BY_IDX(node_id, prop, idx, clks))

#define Z_CLOCK_MANAGEMENT_DATA_DEFINE_nxp_kinetis_mcg_frdiv(node_id, prop, idx)
#define Z_CLOCK_MANAGEMENT_DATA_GET_nxp_kinetis_mcg_frdiv(node_id, prop, idx)                     \
	DT_PHA_BY_IDX(node_id, prop, idx, divider)

#define Z_CLOCK_MANAGEMENT_DATA_DEFINE_nxp_kinetis_mcg_fll(node_id, prop, idx)
#define Z_CLOCK_MANAGEMENT_DATA_GET_nxp_kinetis_mcg_fll(node_id, prop, idx)                       \
	DT_PHA_BY_IDX(node_id, prop, idx, multiplier)

#define Z_CLOCK_MANAGEMENT_DATA_DEFINE_nxp_kinetis_mcg_clk(node_id, prop, idx)
#define Z_CLOCK_MANAGEMENT_DATA_GET_nxp_kinetis_mcg_clk(node_id, prop, idx)                       \
	DT_PHA_BY_IDX(node_id, prop, idx, mode)

struct nxp_kinetis_mcg_pll_cfg {
	/* PLL reference divide factor (1-25) */
	uint8_t prdiv;
	/* VCO multiply factor (24-55) */
	uint8_t vdiv;
	/* Set C5[PLLCLKEN0], enabling the PLL independently of C6[PLLS] */
	uint8_t enable;
};

#define Z_CLOCK_MANAGEMENT_DATA_DEFINE_nxp_kinetis_mcg_pll(node_id, prop, idx)                    \
	static const struct nxp_kinetis_mcg_pll_cfg _CONCAT(                                       \
		nxp_kinetis_mcg_pll_cfg_, _CONCAT(DT_DEP_ORD(node_id), idx)) = {                   \
		.prdiv = DT_PHA_BY_IDX(node_id, prop, idx, prdiv),                                 \
		.vdiv = DT_PHA_BY_IDX(node_id, prop, idx, vdiv),                                   \
		.enable = DT_PHA_BY_IDX(node_id, prop, idx, enable),                               \
	};
#define Z_CLOCK_MANAGEMENT_DATA_GET_nxp_kinetis_mcg_pll(node_id, prop, idx)                       \
	&_CONCAT(nxp_kinetis_mcg_pll_cfg_, _CONCAT(DT_DEP_ORD(node_id), idx))

#define Z_CLOCK_MANAGEMENT_DATA_DEFINE_nxp_kinetis_sim_frac_div(node_id, prop, idx)
#define Z_CLOCK_MANAGEMENT_DATA_GET_nxp_kinetis_sim_frac_div(node_id, prop, idx)                  \
	((DT_PHA_BY_IDX(node_id, prop, idx, multiplier) << NXP_KINETIS_SIM_FRAC_DIV_MULT_SHIFT) |  \
	 DT_PHA_BY_IDX(node_id, prop, idx, divider))

/** @endcond */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_CLOCK_MANAGEMENT_NXP_KINETIS_NXP_KINETIS_CLOCKS_H_ */
