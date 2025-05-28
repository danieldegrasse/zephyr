/*
 * Copyright 2024 NXP
 * Copyright 2025 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * Internal APIs for clock management drivers
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_MANAGEMENT_CLOCK_DRIVER_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_MANAGEMENT_CLOCK_DRIVER_H_

#include <zephyr/sys/slist.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_management/clock.h>
#include <errno.h>
#include <zephyr/logging/log.h>
#include <zephyr/toolchain.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Clock Driver Interface
 * @defgroup clock_driver_interface Clock Driver Interface
 * @ingroup io_interfaces
 * @{
 */

/**
 * @brief Clock Driver API
 *
 * Clock driver API function prototypes. A pointer to a structure of this
 * type should be passed to "CLOCK_DT_DEFINE" when defining the @ref clk
 */
struct clock_management_driver_api {
	/** Gets clock rate in Hz */
	int (*get_rate)(const struct clk *clk_hw);
	/** Configure a clock with device specific data */
	int (*configure)(const struct clk *clk_hw, const void *data);
#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME) || defined(__DOXYGEN__)
	/** Recalculate a clock rate given device specific configuration data */
	int (*configure_recalc)(const struct clk *clk_hw, const void *data);
	/** Recalculate a clock rate given a parent's new clock rate */
	int (*recalc_rate)(const struct clk *clk_hw,
			   const struct clk *parent_clk, uint32_t parent_rate);
#endif
#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE) || defined(__DOXYGEN__)
	/** Gets nearest rate clock can support given rate request */
	int (*round_rate)(const struct clk *clk_hw, uint32_t rate_req);
	/** Sets clock rate using rate request */
	int (*set_rate)(const struct clk *clk_hw, uint32_t rate_req);
#endif
};

/**
 * @brief Configure a clock
 *
 * Configure a clock device using hardware specific data. Called by the clock
 * management subsystem, not intended to be used directly by clock drivers
 * @param clk_hw clock device to configure
 * @param data hardware specific clock configuration data
 * @return -ENOSYS if clock does not implement configure API
 * @return -EIO if clock could not be configured
 * @return -EBUSY if clock cannot be modified at this time
 * @return negative errno for other error configuring clock
 * @return 0 on successful clock configuration
 */
static inline int clock_configure(const struct clk *clk_hw, const void *data)
{
	int ret;

	if (!(clk_hw->api) || !(clk_hw->api->configure)) {
		return -ENOSYS;
	}
	ret = clk_hw->api->configure(clk_hw, data);
#ifdef CONFIG_CLOCK_MANAGEMENT_CLK_NAME
	TOOLCHAIN_DISABLE_WARNING(TOOLCHAIN_WARNING_SHADOW);
	LOG_MODULE_DECLARE(clock_management, CONFIG_CLOCK_MANAGEMENT_LOG_LEVEL);
	TOOLCHAIN_ENABLE_WARNING(TOOLCHAIN_WARNING_SHADOW);
	LOG_DBG("Clock %s reconfigured with result %d", clk_hw->clk_name, ret);
#endif
	return ret;
}

/**
 * @brief Get rate of a clock
 *
 * Gets the rate of a clock, in Hz. A rate of zero indicates the clock is
 * active or powered down.
 * @param clk_hw clock device to read rate from
 * @return -ENOSYS if clock does not implement get_rate API
 * @return -EIO if clock could not be read
 * @return negative errno for other error reading clock rate
 * @return frequency of clock output in HZ
 */
static inline int clock_get_rate(const struct clk *clk_hw)
{
	int ret;

	if (!(clk_hw->api) || !(clk_hw->api->get_rate)) {
		return -ENOSYS;
	}

	ret = clk_hw->api->get_rate(clk_hw);
#ifdef CONFIG_CLOCK_MANAGEMENT_CLK_NAME
	TOOLCHAIN_DISABLE_WARNING(TOOLCHAIN_WARNING_SHADOW);
	LOG_MODULE_DECLARE(clock_management, CONFIG_CLOCK_MANAGEMENT_LOG_LEVEL);
	TOOLCHAIN_ENABLE_WARNING(TOOLCHAIN_WARNING_SHADOW);
	LOG_DBG("Clock %s returns rate %d", clk_hw->clk_name, ret);
#endif
	return ret;
}

#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME) || defined(__DOXYGEN__)
/**
 * @brief Recalculate a clock frequency prior to configuration
 *
 * Calculate the new frequency a clock device would generate prior to
 * applying a hardware specific configuration blob. The clock should not
 * apply the setting when this function is called, simply calculate what
 * the new frequency would be. Called by the clock management subsystem, not
 * intended for use directly within drivers.
 * @param clk_hw clock device to query
 * @param data hardware specific clock configuration data
 * @return -ENOSYS if clock does not implement configure API
 * @return -EIO if clock cannot be configured with this data
 * @return -EBUSY if clock cannot be modified at this time
 * @return -ENOTSUP if API is not supported
 * @return negative errno for other error configuring clock
 * @return frequency in Hz that would be realized by applying this configuration
 */
static inline int clock_configure_recalc(const struct clk *clk_hw, const void *data)
{
	int ret;

	if (!(clk_hw->api) || !(clk_hw->api->configure_recalc)) {
		return -ENOSYS;
	}
	ret = clk_hw->api->configure_recalc(clk_hw, data);
#ifdef CONFIG_CLOCK_MANAGEMENT_CLK_NAME
	TOOLCHAIN_IGNORE_WSHADOW_BEGIN;
	LOG_MODULE_DECLARE(clock_management, CONFIG_CLOCK_MANAGEMENT_LOG_LEVEL);
	TOOLCHAIN_IGNORE_WSHADOW_END;
	LOG_DBG("Clock %s would produce frequency %d after configuration",
		clk_hw->clk_name, ret);
#endif
	return ret;
}

/**
 * @brief Recalculate a clock frequency given a new parent frequency
 *
 * Calculate the frequency that a clock would generate if its parent were
 * reconfigured to the frequency @p parent_rate. This call does not indicate
 * that the clock has been reconfigured, and is simply a query.Called by the
 * clock management subsystem, not intended for use directly within drivers.
 * @param clk_hw clock to recalculate rate for
 * @param parent_clk parent clock object whose rate would be changed
 * @param parent_rate new frequency parent would update to
 * @return -ENOSYS if API is not supported by this clock
 * @return -ENOTSUP if API is not supported
 * @return CLK_NOT_CONNECTED if clock is not attached to the parent clock
 * @return rate clock would produce on success
 */
static inline int clock_recalc_rate(const struct clk *clk_hw,
				    const struct clk *parent_clk,
				    uint32_t parent_rate)
{
	int ret;

	if (!(clk_hw->api) || !(clk_hw->api->recalc_rate)) {
		return -ENOSYS;
	}
	ret = clk_hw->api->recalc_rate(clk_hw, parent_clk, parent_rate);
#ifdef CONFIG_CLOCK_MANAGEMENT_CLK_NAME
	TOOLCHAIN_IGNORE_WSHADOW_BEGIN;
	LOG_MODULE_DECLARE(clock_management, CONFIG_CLOCK_MANAGEMENT_LOG_LEVEL);
	TOOLCHAIN_IGNORE_WSHADOW_END;
	LOG_DBG("Clock %s would produce frequency %d from parent %s, rate %d",
		clk_hw->clk_name, ret, parent_clk->clk_name, parent_rate);
#endif
	return ret;
}
#else
/* Stub functions to indicate recalc and recalc_rate aren't supported */

static inline int clock_configure_recalc(const struct clk *clk_hw, const void *data)
{
	return -ENOTSUP;
}

static inline int clock_recalc_rate(const struct clk *clk_hw,
				    const struct clk *parent_clk,
				    uint32_t parent_rate)
{
	return -ENOTSUP;
}
#endif

#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE) || defined(__DOXYGEN__)

/**
 * @brief Get nearest rate a clock can support given constraints
 *
 * Returns the actual rate that this clock would produce if `clock_set_rate`
 * was called with the requested frequency.
 * @param clk_hw clock device to query
 * @param rate_req requested rate
 * @return -ENOTSUP if API is not supported
 * @return -ENOENT if clock cannot satisfy request
 * @return -ENOSYS if clock does not implement round_rate API
 * @return -EINVAL if arguments are invalid
 * @return -EIO if clock could not be queried
 * @return negative errno for other error calculating rate
 * @return rate clock would produce (in Hz) on success
 */
static inline int clock_round_rate(const struct clk *clk_hw, uint32_t rate_req)
{
	int ret;

	if (!(clk_hw->api) || !(clk_hw->api->round_rate)) {
		return -ENOSYS;
	}

	ret = clk_hw->api->round_rate(clk_hw, rate_req);
#ifdef CONFIG_CLOCK_MANAGEMENT_CLK_NAME
	TOOLCHAIN_DISABLE_WARNING(TOOLCHAIN_WARNING_SHADOW);
	LOG_MODULE_DECLARE(clock_management, CONFIG_CLOCK_MANAGEMENT_LOG_LEVEL);
	TOOLCHAIN_ENABLE_WARNING(TOOLCHAIN_WARNING_SHADOW);
	LOG_DBG("Clock %s reports rate %d for rate %u",
		clk_hw->clk_name, ret, rate_req);
#endif
	return ret;
}

/**
 * @brief Set a clock rate
 *
 * Sets a clock to the closest frequency possible given the requested rate.
 * @param clk_hw clock device to set rate for
 * @param rate_req requested rate
 * @return -ENOTSUP if API is not supported
 * @return -ENOENT if clock cannot satisfy request
 * @return -ENOSYS if clock does not implement set_rate API
 * @return -EPERM if clock cannot be reconfigured
 * @return -EINVAL if arguments are invalid
 * @return -EIO if clock rate could not be set
 * @return negative errno for other error setting rate
 * @return rate clock is set to produce (in Hz) on success
 */
static inline int clock_set_rate(const struct clk *clk_hw, uint32_t rate_req)
{
	int ret;

	if (!(clk_hw->api) || !(clk_hw->api->set_rate)) {
		return -ENOSYS;
	}

	ret = clk_hw->api->set_rate(clk_hw, rate_req);
#ifdef CONFIG_CLOCK_MANAGEMENT_CLK_NAME
	TOOLCHAIN_DISABLE_WARNING(TOOLCHAIN_WARNING_SHADOW);
	LOG_MODULE_DECLARE(clock_management, CONFIG_CLOCK_MANAGEMENT_LOG_LEVEL);
	TOOLCHAIN_ENABLE_WARNING(TOOLCHAIN_WARNING_SHADOW);
	if (ret > 0) {
		LOG_DBG("Clock %s set to rate %d for request %u",
			clk_hw->clk_name, ret, rate_req);
	}
#endif
	return ret;
}

#else /* if !defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE) */

/* Stub functions to indicate set_rate and round_rate aren't supported */

static inline int clock_round_rate(const struct clk *clk_hw, uint32_t req_rate)
{
	return -ENOTSUP;
}

static inline int clock_set_rate(const struct clk *clk_hw, uint32_t req_rate)
{
	return -ENOTSUP;
}

#endif /* defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE) || defined(__DOXYGEN__) */


#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_MANAGEMENT_CLOCK_DRIVER_H_ */
