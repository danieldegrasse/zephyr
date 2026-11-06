/*
 * Copyright 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Helpers shared by the Kinetis clock drivers. Several MCG settings are
 * coupled (for example C6[PLLS] disables the FLL), so drivers inspect MCG
 * status directly through these helpers rather than through the clock tree.
 */

#ifndef ZEPHYR_DRIVERS_CLOCK_MANAGEMENT_NXP_KINETIS_NXP_KINETIS_INTERNAL_H_
#define ZEPHYR_DRIVERS_CLOCK_MANAGEMENT_NXP_KINETIS_NXP_KINETIS_INTERNAL_H_

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <fsl_common.h>

#ifdef __cplusplus
extern "C" {
#endif

/* MCG_S[CLKST] encodings */
#define KINETIS_MCG_CLKST_FLL 0U
#define KINETIS_MCG_CLKST_IRC 1U
#define KINETIS_MCG_CLKST_EXT 2U
#define KINETIS_MCG_CLKST_PLL 3U

/* MCG_C7[OSCSEL] encoding for the system oscillator (OSCCLK0) */
#define KINETIS_MCG_OSCSEL_OSC0 0U

static inline uint8_t kinetis_mcg_clkst(void)
{
	return (MCG->S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT;
}

/* True when C2[LP] has disabled the FLL and PLL (BLPI or BLPE mode) */
static inline bool kinetis_mcg_lp_bypass(void)
{
	uint8_t clkst = kinetis_mcg_clkst();

	return ((MCG->C2 & MCG_C2_LP_MASK) != 0U) &&
	       ((clkst == KINETIS_MCG_CLKST_IRC) || (clkst == KINETIS_MCG_CLKST_EXT));
}

/* The FLL is disabled whenever the PLL is selected, or in BLPx modes */
static inline bool kinetis_mcg_fll_enabled(void)
{
	return ((MCG->S & MCG_S_PLLST_MASK) == 0U) && !kinetis_mcg_lp_bypass();
}

/* The PLL runs when selected by PLLS or enabled by PLLCLKEN0, except in BLPx modes */
static inline bool kinetis_mcg_pll_enabled(void)
{
	return (((MCG->C5 & MCG_C5_PLLCLKEN0_MASK) != 0U) ||
		((MCG->S & MCG_S_PLLST_MASK) != 0U)) &&
	       !kinetis_mcg_lp_bypass();
}

/* True when the MCG is clocked from, or locking to, the external reference */
static inline bool kinetis_mcg_ext_ref_in_use(void)
{
	uint8_t clkst = kinetis_mcg_clkst();

	if ((clkst == KINETIS_MCG_CLKST_EXT) || (clkst == KINETIS_MCG_CLKST_PLL)) {
		return true;
	}
	if ((clkst == KINETIS_MCG_CLKST_FLL) && ((MCG->S & MCG_S_IREFST_MASK) == 0U)) {
		/* FEE mode */
		return true;
	}
	/* The PLL is always referenced from the external reference */
	return kinetis_mcg_pll_enabled();
}

/* True when the MCG external reference is the system oscillator in crystal mode */
static inline bool kinetis_mcg_ext_ref_is_crystal(void)
{
	return ((MCG->C2 & MCG_C2_EREFS_MASK) != 0U) &&
	       (((MCG->C7 & MCG_C7_OSCSEL_MASK) >> MCG_C7_OSCSEL_SHIFT) ==
		KINETIS_MCG_OSCSEL_OSC0);
}

/* Wait for a masked 8-bit register field to read back as a given value */
static inline int kinetis_wait8(volatile const uint8_t *reg, uint8_t mask, uint8_t val)
{
	return WAIT_FOR((*reg & mask) == val, CONFIG_NXP_KINETIS_CLOCK_TIMEOUT, NULL) ? 0
										       : -ETIMEDOUT;
}

static inline int kinetis_mcg_wait_osc_init(void)
{
	return kinetis_wait8(&MCG->S, MCG_S_OSCINIT0_MASK, MCG_S_OSCINIT0_MASK);
}

static inline int kinetis_mcg_wait_pll_lock(void)
{
	return kinetis_wait8(&MCG->S, MCG_S_LOCK0_MASK, MCG_S_LOCK0_MASK);
}

/*
 * RTC registers fault on access unless SIM_SCGC6[RTC] is set. Enable access
 * around RTC_CR accesses, and restore the previous gate state afterwards.
 */
static inline bool kinetis_rtc_access_begin(void)
{
	unsigned int key = irq_lock();
	bool was_enabled = (SIM->SCGC6 & SIM_SCGC6_RTC_MASK) != 0U;

	SIM->SCGC6 |= SIM_SCGC6_RTC_MASK;
	irq_unlock(key);
	return was_enabled;
}

static inline void kinetis_rtc_access_end(bool was_enabled)
{
	unsigned int key;

	if (!was_enabled) {
		key = irq_lock();
		SIM->SCGC6 &= ~SIM_SCGC6_RTC_MASK;
		irq_unlock(key);
	}
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_CLOCK_MANAGEMENT_NXP_KINETIS_NXP_KINETIS_INTERNAL_H_ */
