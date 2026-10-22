/*
 * Copyright (c) 2026 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/clock_management/clock_driver.h>
#include <zephyr/drivers/clock_management/clock_helpers.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>

#include "stm32_clocks.h"

#define DT_DRV_COMPAT st_stm32h7_pll

/* PLLCFGR is at offset 0x2C from RCC_CR */
#define RCC_PLLCFGR_OFFSET       0x2C
#define RCC_PLLCFGR_REG(config)  ((config)->cr_reg + RCC_PLLCFGR_OFFSET)
#define RCC_PLLFRACR_REG(config) ((config)->divr_reg + 4)

/* Per-PLL bit layout within PLLCFGR */
#define PLLCFGR_FRACEN_BIT(fracen_off) (fracen_off)
#define PLLCFGR_VCOSEL_BIT(fracen_off) (fracen_off + 1)
#define PLLCFGR_RGE_SHIFT(fracen_off)  ((fracen_off) + 2)
#define PLLCFGR_RGE_MASK(fracen_off)   (0x3U << PLLCFGR_RGE_SHIFT(fracen_off))

/* PLLxDIVR register: DIVN occupies bits [8:0] */
#define PLL_DIVR_DIVN_MASK GENMASK(8, 0)

/* PLLxFRACR register: FRACN occupies bits [15:3] */
#define PLL_FRACR_FRACN_SHIFT 3
#define PLL_FRACR_FRACN_MASK  GENMASK(15, 3)

struct stm32h7_pll_config {
	STANDARD_CLK_SUBSYS_DATA_DEFINE
	mem_addr_t divr_reg;
	mem_addr_t cr_reg;
	uint8_t enable_bit;
	uint8_t pll_idx;
	uint8_t fracen_bit;
};

static int stm32h7_pll_on_off(const struct clk *clk_hw, bool on)
{
	const struct stm32h7_pll_config *config = clk_hw->hw_data;
	uint32_t reg;
	bool locked;

	reg = sys_read32(config->cr_reg);
	if (on) {
		reg |= BIT(config->enable_bit);
		sys_write32(reg, config->cr_reg);
		/* PLL ready bit is directly after enable bit */
		locked = WAIT_FOR(sys_test_bit(config->cr_reg, config->enable_bit + 1),
				  CONFIG_STM32H7_PLL_LOCK_TIMEOUT, NULL);
	} else {
		reg &= ~BIT(config->enable_bit);
		sys_write32(reg, config->cr_reg);
		locked = WAIT_FOR(!sys_test_bit(config->cr_reg, config->enable_bit + 1),
				  CONFIG_STM32H7_PLL_LOCK_TIMEOUT, NULL);
	}

	return locked ? 0 : -ETIMEDOUT;
}

static void stm32h7_pll_write_cfg(const struct stm32h7_pll_config *config,
				  const struct stm32h7_pll_cfg *cfg)
{
	uint32_t reg;
	uint8_t fracen_bit = config->fracen_bit;

	/* Step 1: Set VCOSEL, RGE with FRACEN=0 in PLLCFGR */
	reg = sys_read32(RCC_PLLCFGR_REG(config));
	reg &= ~(BIT(PLLCFGR_FRACEN_BIT(fracen_bit)) | BIT(PLLCFGR_VCOSEL_BIT(fracen_bit)) |
		 PLLCFGR_RGE_MASK(fracen_bit));
	if (cfg->vcosel) {
		reg |= BIT(PLLCFGR_VCOSEL_BIT(fracen_bit));
	}
	reg |= (cfg->rge << PLLCFGR_RGE_SHIFT(fracen_bit)) & PLLCFGR_RGE_MASK(fracen_bit);
	sys_write32(reg, RCC_PLLCFGR_REG(config));

	/* Step 2: Write DIVN to PLLxDIVR (preserve P/Q/R fields) */
	reg = sys_read32(config->divr_reg);
	reg &= ~PLL_DIVR_DIVN_MASK;
	reg |= cfg->divn & PLL_DIVR_DIVN_MASK;
	sys_write32(reg, config->divr_reg);

	/* Step 3: Write FRACN to PLLxFRACR */
	reg = sys_read32(RCC_PLLFRACR_REG(config));
	reg &= ~PLL_FRACR_FRACN_MASK;
	reg |= (cfg->fracn << PLL_FRACR_FRACN_SHIFT) & PLL_FRACR_FRACN_MASK;
	sys_write32(reg, RCC_PLLFRACR_REG(config));

	/* Step 4: Latch FRACN by setting FRACEN (0→1 transition) */
	if (cfg->fracn != 0) {
		reg = sys_read32(RCC_PLLCFGR_REG(config));
		reg |= BIT(PLLCFGR_FRACEN_BIT(fracen_bit));
		sys_write32(reg, RCC_PLLCFGR_REG(config));
	}
}

static clock_freq_t stm32h7_pll_calc_vco(clock_freq_t parent_rate, uint32_t divn, uint32_t fracn)
{
	uint64_t vco;

	/*
	 * f_VCO = f_ref * (DIVN + (FRACN / 8192))
	 * f_VCO = f_ref * DIVN + (f_ref * FRACN) / 8192
	 */
	vco = parent_rate * ((uint64_t)divn);
	vco += (parent_rate * (uint64_t)fracn) / 8192ULL;

	return (clock_freq_t)vco;
}

static int stm32h7_pll_configure(const struct clk *clk_hw, const void *data)
{
	const struct stm32h7_pll_config *config = clk_hw->hw_data;
	const struct stm32h7_pll_cfg *cfg = data;
	int ret;

	/*
	 * TODO- it is safe to reconfigure the FRACN ONLY while the PLL is
	 * running. We need to support this usecase for runtime frequency scaling
	 */

	/* Validate children can tolerate the PLL going offline */
	ret = clock_children_check_rate(clk_hw, 0);
	if (ret < 0) {
		return ret;
	}

	/* Disable PLL before reconfiguration */
	stm32h7_pll_on_off(clk_hw, false);

	/* Program registers */
	stm32h7_pll_write_cfg(config, cfg);

	/* Re-enable PLL if requested */
	if (cfg->enable) {
		stm32h7_pll_on_off(clk_hw, true);
	}

	return 0;
}

static clock_freq_t stm32h7_pll_recalc_rate(const struct clk *clk_hw, clock_freq_t parent_rate)
{
	const struct stm32h7_pll_config *config = clk_hw->hw_data;
	uint32_t divn, fracn, reg;

	/* Check if PLL is enabled */
	if (!sys_test_bit(config->cr_reg, config->enable_bit)) {
		return 0;
	}

	divn = sys_read32(config->divr_reg) & PLL_DIVR_DIVN_MASK;
	reg = sys_read32(RCC_PLLFRACR_REG(config));
	fracn = (reg & PLL_FRACR_FRACN_MASK) >> PLL_FRACR_FRACN_SHIFT;

	return stm32h7_pll_calc_vco(parent_rate, divn, fracn);
}

#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)

static clock_freq_t stm32h7_pll_configure_recalc(const struct clk *clk_hw, const void *data,
						 clock_freq_t parent_rate)
{
	const struct stm32h7_pll_cfg *cfg = data;
	int ret;

	ret = clock_children_check_rate(clk_hw, 0);
	if (ret < 0) {
		return ret;
	}

	if (!cfg->enable) {
		return 0;
	}

	return stm32h7_pll_calc_vco(parent_rate, cfg->divn, cfg->fracn);
}

#endif /* CONFIG_CLOCK_MANAGEMENT_RUNTIME */

#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)

static clock_freq_t stm32h7_pll_best_rate(const struct clk *clk_hw, clock_freq_t rate_req,
					  clock_freq_t parent_rate, bool commit)
{
	const struct stm32h7_pll_config *config = clk_hw->hw_data;
	uint32_t divn, fracn;
	uint64_t target;
	clock_freq_t vco_rate;
	int ret;

	if (rate_req == 0 || parent_rate == 0) {
		if (commit) {
			ret = clock_children_check_rate(clk_hw, 0);
			if (ret < 0) {
				return ret;
			}
			stm32h7_pll_on_off(clk_hw, false);
		}
		return 0;
	}

	/*
	 * Compute best DIVN and FRACN from the requested VCO rate.
	 * f_VCO = parent * (DIVN + FRACN/8192)
	 *
	 * DIVN = rate_req / parent_rate  (integer part)
	 * FRACN = remainder * 8192 / parent_rate
	 */
	divn = (uint32_t)(rate_req / parent_rate);

	/* Clamp DIVN to valid range [3..511] (VCO mult 4..512) */
	divn = CLAMP(divn - 1, 3, 511);

	/* Compute fractional remainder */
	fracn = (rate_req - (divn * parent_rate)) * 8192ULL fracn = MIN(fracn, 8191);

	vco_rate = stm32h7_pll_calc_vco(parent_rate, divn, fracn);

	if (commit) {
		struct stm32h7_pll_cfg cfg = {
			.divn = divn,
			.fracn = fracn,
			.rge = 0,
			.vcosel = 0,
			.enable = 1,
		};

		/*
		 * Select RGE based on input reference frequency.
		 * 1-2 MHz: 0, 2-4 MHz: 1, 4-8 MHz: 2, 8-16 MHz: 3
		 */
		if (parent_rate <= MHZ(2)) {
			cfg.rge = 0;
		} else if (parent_rate <= MHZ(4)) {
			cfg.rge = 1;
		} else if (parent_rate <= MHZ(8)) {
			cfg.rge = 2;
		} else {
			cfg.rge = 3;
		}

		/* Select VCOSEL: 0=wide (192-836 MHz), 1=medium (150-420 MHz) */
		cfg.vcosel = (vco_rate <= MHZ(420)) ? 1 : 0;

		ret = clock_children_check_rate(clk_hw, 0);
		if (ret < 0) {
			return ret;
		}

		stm32h7_pll_on_off(clk_hw, false);
		stm32h7_pll_write_cfg(config, &cfg);
		stm32h7_pll_on_off(clk_hw, true);
	}

	return vco_rate;
}

#endif /* CONFIG_CLOCK_MANAGEMENT_SET_RATE */

const struct clock_management_standard_api stm32h7_pll_api = {
	.shared.configure = stm32h7_pll_configure,
	.shared.on_off = stm32h7_pll_on_off,
	.recalc_rate = stm32h7_pll_recalc_rate,
#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
	.configure_recalc = stm32h7_pll_configure_recalc,
#endif
#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
	.best_rate = stm32h7_pll_best_rate,
#endif
};

#define STM32H7_PLL_DEFINE(inst)                                                                   \
	static const struct stm32h7_pll_config stm32h7_pll_cfg_##inst = {                          \
		STANDARD_CLK_SUBSYS_DATA_INIT(CLOCK_DT_GET(DT_INST_PHANDLE(inst, input)))          \
			.divr_reg = DT_INST_REG_ADDR_BY_IDX(inst, 0),                              \
		.cr_reg = DT_INST_REG_ADDR_BY_IDX(inst, 1),                                        \
		.enable_bit = (uint8_t)DT_INST_PROP(inst, enable_bit),                             \
		.fracen_bit = (uint8_t)DT_INST_PROP(inst, fracen_bit),                             \
	};                                                                                         \
                                                                                                   \
	CLOCK_DT_INST_DEFINE(inst, &stm32h7_pll_cfg_##inst, &stm32h7_pll_api);

DT_INST_FOREACH_STATUS_OKAY(STM32H7_PLL_DEFINE)
