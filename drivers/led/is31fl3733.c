/*
 * Copyright 2022 Daniel DeGrasse <daniel@degrasse.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT issi_is31fl3733

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/led.h>
#include <zephyr/kernel.h>
#include <stdlib.h>
#include <math.h>

#define LOG_LEVEL CONFIG_LED_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(is31fl3733);

/* IS31FL3733 register definitions */
#define CMD_SEL_REG	0xFD /* Command/page selection reg */
#define CMD_SEL_LED	0x0  /* LED configuration page */
#define CMD_SEL_PWM	0x1  /* PWM configuration page */
#define CMD_SEL_BREATHE 0x2  /* Auto breathe mode page */
#define CMD_SEL_FUNC	0x3  /* Function configuration page */

#define CMD_LOCK_REG	0xFE /* Command selection lock reg */
#define CMD_LOCK_UNLOCK 0xC5 /* Command sel unlock value */

/* IS31FL3733 page specific register definitions */

/* Auto breathe mode page */
#define ABM_MODE_PWM 0x0 /* LED is in PWM mode */
#define ABM_MODE_1 0x1 /* LED is in auto breathe mode 1 */
#define ABM_MODE_2 0x2 /* LED is in auto breathe mode 2 */
#define ABM_MODE_3 0x3 /* LED is in auto breathe mode 3 */

/* Function configuration page */
#define CONF_REG	   0x0 /* configuration register */
#define CONF_REG_SSD_MASK  0x1 /* Software shutdown mask */
#define CONF_REG_SSD_SHIFT 0x0 /* Software shutdown shift */
#define CONF_REG_PFS_MASK 0x38 /* PWM frequency scaling mask */
#define CONF_REG_PFS_SHIFT 0x3 /* PWM frequency scaling shift */
#define CONF_REG_PFS_MAX  0x4 /* PWM frequency scaling shift */
#define CONF_REG_B_EN_MASK 0x2 /* Auto breathe enable mask */
#define CONF_REG_B_EN_SHIFT 0x1 /* Auto breathe enable shift */

#define GLOBAL_CURRENT_CTRL_REG 0x1 /* global current control register */

#define ABM1_CTRL1_REG 0x2 /* Auto breathe mode 1, control register 1 */
#define ABMX_CTRL1_T1_SHIFT 0x5 /* Timescale register 1 shift */
#define ABMX_CTRL1_T1_MASK 0xE0 /* Timescale register 1 mask */
#define ABMX_CTRL1_T1_MAX 0x7 /* Timescale register 1 max value */
#define ABMX_CTRL1_T2_SHIFT 0x1 /* Timescale register 2 shift */
#define ABMX_CTRL1_T2_MASK 0x1E /* Timescale register 2 mask */
#define ABMX_CTRL1_T2_MAX 0x8 /* Timescale register 2 max value */
#define ABM1_CTRL2_REG 0x3 /* Auto breathe mode 1, control register 2 */
#define ABMX_CTRL2_T3_SHIFT 0x5 /* Timescale register 3 shift */
#define ABMX_CTRL2_T3_MASK 0xE0 /* Timescale register 3 mask */
#define ABMX_CTRL2_T3_MAX 0x7 /* Timescale register 3 max value */
#define ABMX_CTRL2_T4_SHIFT 0x1 /* Timescale register 4 shift */
#define ABMX_CTRL2_T4_MASK 0x1E /* Timescale register 4 mask */
#define ABMX_CTRL2_T4_MAX 0xA /* Timescale register 4 max value */
#define ABM1_CTRL3_REG 0x4 /* Auto breathe mode 1, control register 3 */
#define ABM1_CTRL4_REG 0x5 /* Auto breathe mode 1, control register 4 */

#define TIME_UPDATE_REG 0xE /* Update auto breathe mode time registers */

#define RESET_REG 0x11 /* Reset all registers to POR state */

/* Matrix Layout definitions */
#define IS31FL3733_ROW_COUNT 12
#define IS31FL3733_COL_COUNT 16
#define IS31FL3733_MAX_LED (IS31FL3733_ROW_COUNT * IS31FL3733_COL_COUNT)

/* Constant used in led breathe mode calculations */
#define LOG2_3_2 0.584962501f

/* Multipliers for led breathe mode calculations
 * These multipliers are derived from the time registers T1-T4.
 * See Tables 15 and 16 in datasheet. All values are given in
 * milliseconds.
 */
static uint32_t is31fl3733_pfs_time_scale[] = {
	210,
	420,
	70,
	840,
	168,
};

struct is31fl3733_config {
	struct i2c_dt_spec bus;
	struct gpio_dt_spec sdb;
	uint8_t current_limit;
	uint8_t default_brightness;
	uint32_t abm_ramp_up;
	uint32_t abm_ramp_down;
};

struct is31fl3733_data {
	/* Active configuration page */
	uint32_t selected_page;
	/* Array of LED on/off states */
	uint8_t led_state[((IS31FL3733_MAX_LED) / 8) + 1];
	/* Array of LED PWM states */
	uint8_t pwm_state[(IS31FL3733_MAX_LED) + 1];
};

static int is31fl3733_led_set_brightness(const struct device *dev, uint32_t led, uint8_t value);

/* Selects target register page for IS31FL3733. After setting the
 * target page, all I2C writes will use the selected page until the selected
 * page is changed.
 */
static int is31fl3733_select_page(const struct device *dev, uint8_t page)
{
	const struct is31fl3733_config *config = dev->config;
	struct is31fl3733_data *data = dev->data;
	int ret = 0U;

	if (data->selected_page == page) {
		/* No change necessary */
		return 0;
	}

	/* Unlock page selection register */
	ret = i2c_reg_write_byte_dt(&config->bus, CMD_LOCK_REG, CMD_LOCK_UNLOCK);
	if (ret < 0) {
		LOG_ERR("Could not unlock page selection register");
		return ret;
	}

	/* Write to function select to select active page */
	ret = i2c_reg_write_byte_dt(&config->bus, CMD_SEL_REG, page);
	if (ret < 0) {
		LOG_ERR("Could not select active page");
		return ret;
	}
	data->selected_page = page;
	return ret;
}

/* Helper function to find best possible auto breathe mode setting
 * for requested ramp up, ramp down, on, and off times
 * returns cumulative error of best solution found.
 *
 * Sets t1_b, t2_b, t3_b, t4_b, and pfs_b to best values for
 * T1,T2,T3,T4, and PFS registers.
 */
static int is31fl3733_abm_solution(uint32_t ramp_up, uint32_t ramp_down,
	uint32_t delay_on, uint32_t delay_off, uint8_t *pfs_b, uint8_t *t1_b,
	uint8_t *t2_b, uint8_t *t3_b, uint8_t *t4_b)
{
	uint32_t error, cand, t1_err, t2_err, t3_err, t4_err, pfs_base;
	uint8_t t1, t2, t3, t4, pfs;
	/* Since the IS31FL3733 only supports discrete on and off times,
	 * we must search all possible settings of the PFS and time scale
	 * registers to find the one best approximating the ramp up, ramp
	 * down, time on, and time off values requested by the user.
	 *
	 * We will calculate the cumulative error of each PFS and timescale
	 * value, and compare it to the current best setting we've located.
	 * If the new solution wins, we'll keep it and continue searching
	 * Note: this process is rather compute intensive. If you'd like
	 * to reduce the impact, ensure the devicetree properties
	 * abm-ramp-up, abm-ramp-down, and the arguments delay_on and delay_off
	 * are exact values possible with the ABM control registers
	 * (see Tables 15 and 16 of the datasheet). The algorithm will always
	 * stop if it finds an exact solution for requested values.
	 */
	error = UINT32_MAX;
	for (pfs = 0; pfs <= CONF_REG_PFS_MAX; pfs++) {
		pfs_base = is31fl3733_pfs_time_scale[pfs];
		for (t1 = 0; t1 <= ABMX_CTRL1_T1_MAX; t1++) {
			/* T1 error */
			t1_err = abs(ramp_up - (pfs_base << t1));
			for (t2 = 0; t2 <= ABMX_CTRL1_T2_MAX; t2++) {
				/* T2 error */
				if (t2 == 0) {
					t2_err = delay_on;
				} else {
					t2_err = abs(delay_on - (pfs_base << (t2 - 1)));
				}
				for (t3 = 0; t3 <= ABMX_CTRL2_T3_MAX; t3++) {
					/* T3 error */
					t3_err = abs(ramp_down - (pfs_base << t3));
					for (t4 = 0; t4 <= ABMX_CTRL2_T4_MAX; t4++) {
						/* T4 error */
						if (t4 == 0) {
							t4_err = delay_off;
							if (pfs == 2) {
								/* Datasheet says this is
								 * an invalid setting
								 */
								continue;
							}
						} else {
							t4_err = abs(delay_off -
								(pfs_base << (t4 - 1)));
						}
						cand = (t1_err + t2_err + t3_err + t4_err);
						if (cand < error) {
							/* New best solution found */
							error = cand;
							*t1_b = t1;
							*t2_b = t2;
							*t3_b = t3;
							*t4_b = t4;
							*pfs_b = pfs;
							if (error == 0) {
								return 0;
							}
						}
					}
				}
			}
		}
	}
	return error;
}


/*
 * Helper function to find best possible auto breathe mode setting
 * for requested ramp up, ramp down, on, and off times
 * returns cumulative error of best solution found.
 *
 * Sets t1_b, t2_b, t3_b, t4_b, and pfs_b to best values for
 * T1,T2,T3,T4, and PFS registers.
 */
static int is31fl3733_abm_solution_2(uint32_t ramp_up, uint32_t ramp_down,
	uint32_t delay_on, uint32_t delay_off, uint8_t *pfs_b, uint8_t *t1_b,
	uint8_t *t2_b, uint8_t *t3_b, uint8_t *t4_b)
{
	float t1, t2, t3, t4;
	uint32_t pfs_base, err, cand;

	/*
	 * Since the IS31FL3733 only supports discrete delays,
	 * we must find the best fit for the PFS and T1-T4 values for
	 * a given ramp up/down time, and on/off time.
	 *
	 * This can be accomplished by minimizing a function calculating
	 * the difference between the desired ramp up/down time and on/off
	 * time, and the actual ramp up/down and on/off time for a given PFS
	 * and T1-T4.
	 * This function can be modeled with the following equation:
	 * e = (up - (f*2^t1))^2 + (on - (f*2^(t2 - 1)))^2 +
	 *	(down - (f*2^t3))^2 + (off - (f*2^(t4 - 1)))^2
	 * where e is error, and f is the base delay for a given PFS value
	 * We then calculate the gradient of this function, and find the
	 * gradient is zero at:
	 * <T1, T2, T3, T4> = <log2(up/f), log2(on/f) + 1, log2(down/f), log2(off/f)>
	 *
	 * Since the PFS base values are not easily differentiable as T1-T4
	 * values are, use the following algorithm:
	 * for each PFS base value:
	 *	calculate zero vector for gradient
	 *	round each T value to the nearest integer that minimizes error
	 *	calculate the error function with new Tx values
	 *	if error is better than current best error:
	 *		update best error
	 *		save new Tx and PFS values
	 *		if error is zero, exit
	 */


	err = UINT32_MAX;
	for (uint32_t pfs = 0; pfs <= CONF_REG_PFS_MAX; pfs++) {
		pfs_base = is31fl3733_pfs_time_scale[pfs];
		/* Calculate the optimal time values for this
		 * pfs value using the error function's gradient
		 */
		t1 = MAX(log2f(((float)ramp_up) / ((float)pfs_base)), 0);
		t2 = MAX(log2f(((float)delay_on) / ((float)pfs_base)) + 1, 0);
		t3 = MAX(log2f(((float)ramp_down) / ((float)pfs_base)), 0);
		t4 = MAX(log2f(((float)delay_off) / ((float)pfs_base)) + 1, 0);
		/*
		 * Round each T value in order to minimize the error of
		 * the function 2^T. We use this rounding because we know
		 * that in the error function, 2 will be raised to the power
		 * of T, and traditional integer rounding will not always
		 * minimize this.
		 * For a given range [x, x+1], 2^T where log2(x) < T < log2(x+1)
		 * will be closer to x when: T- floor(T) < log2(3/2)
		 */
		if (t1 - floorf(t1) > LOG2_3_2) {
			t1 = ceilf(t1);
		} else {
			t1 = floorf(t1);
		}
		if (t2 - floorf(t2) > LOG2_3_2) {
			t2 = ceilf(t2);
		} else {
			t2 = floorf(t2);
		}
		if (t3 - floorf(t3) > LOG2_3_2) {
			t3 = ceilf(t3);
		} else {
			t3 = floorf(t3);
		}
		if (t4 - floorf(t4) > LOG2_3_2) {
			t4 = ceilf(t4);
		} else {
			t4 = floorf(t4);
		}
		if ((t4 == 0) && (pfs == 2)) {
			/*
			 * Datasheet says this is an invalid setting.
			 * Test with T4=1
			 */
			t4 = 1;
		}
		/* Clamp T1-T4 to their maximum values */
		t1 = MIN(t1, ABMX_CTRL1_T1_MAX);
		t2 = MIN(t2, ABMX_CTRL1_T2_MAX);
		t3 = MIN(t3, ABMX_CTRL2_T3_MAX);
		t4 = MIN(t4, ABMX_CTRL2_T4_MAX);
		cand = abs(ramp_up - (pfs_base << ((uint8_t)t1)));
		cand += abs(delay_on - (pfs_base << (((uint8_t)t2) - 1)));
		cand += abs(ramp_down - (pfs_base << ((uint8_t)t3)));
		cand += abs(delay_off - (pfs_base << (((uint8_t)t4) - 1)));
		if (cand < err) {
			/* Save new best time values */
			*t1_b = (uint8_t)t1;
			*t2_b = (uint8_t)t2;
			*t3_b = (uint8_t)t3;
			*t4_b = (uint8_t)t4;
			*pfs_b = pfs;
			/* Update best error */
			err = cand;
			if (err == 0) {
				return 0;
			}
		}
	}
	return err;
}


/* Helper function to turn led on/off */
static int is31fl3733_led_on_off(const struct device *dev, uint32_t led, bool on)
{
	const struct is31fl3733_config *config = dev->config;
	struct is31fl3733_data *data = dev->data;
	int ret;
	uint8_t reg, mask;

	reg = led / 8;
	mask = BIT((led % 8));
	/* Note: first byte of led state reg is reserved for address
	 * data, used with burst writes
	 */
	if (on) {
		data->led_state[reg + 1] |= mask;
	} else {
		data->led_state[reg + 1] &= ~mask;
	}
	ret = is31fl3733_select_page(dev, CMD_SEL_LED);
	if (ret < 0) {
		return ret;
	}
	ret = i2c_reg_write_byte_dt(&config->bus, reg, data->led_state[reg + 1]);
	if (on) {
		/* Check PWM state. If not set, initialize LED to default
		 * brightness
		 */
		if (data->pwm_state[led + 1] == 0) {
			ret = is31fl3733_led_set_brightness(dev, led,
				config->default_brightness);
			if (ret < 0) {
				return ret;
			}
		}
	}
	return ret;
}

static int is31fl3733_led_on(const struct device *dev, uint32_t led)
{
	return is31fl3733_led_on_off(dev, led, true);
}

static int is31fl3733_led_off(const struct device *dev, uint32_t led)
{
	return is31fl3733_led_on_off(dev, led, false);
}

static int is31fl3733_led_blink(const struct device *dev, uint32_t led,
	uint32_t delay_on, uint32_t delay_off)
{
	const struct is31fl3733_config *config = dev->config;
	struct is31fl3733_data *data = dev->data;
	int ret;
	uint8_t t1 = 0, t2 = 0, t3 = 0, t4 = 0, pfs = 0;

	if ((data->led_state[(led / 8) + 1] & BIT((led % 8))) == 0) {
		/* LED is off. Enable LED */
		ret = is31fl3733_led_on(dev, led);
		if (ret < 0) {
			return ret;
		}
	}

	uint64_t delta = k_uptime_get();
	ret = is31fl3733_abm_solution(config->abm_ramp_up,
				config->abm_ramp_down,
				delay_on, delay_off,
				&pfs, &t1, &t2, &t3, &t4);
	LOG_DBG("ABM solution 1 took %llu ms\n", k_uptime_delta(&delta));
	LOG_DBG("Found solution for R_UP %d, R_DOWN %d, T_ON %d T_OFF %d",
		config->abm_ramp_up, config->abm_ramp_down, delay_on, delay_off);
	LOG_DBG("T1: %d T2: %d T3: %d T4: %d PFS: %d", t1, t2, t3, t4, pfs);
	LOG_DBG("Error: %d", ret);

	delta = k_uptime_get();
	ret = is31fl3733_abm_solution_2(config->abm_ramp_up,
				config->abm_ramp_down,
				delay_on, delay_off,
				&pfs, &t1, &t2, &t3, &t4);
	LOG_DBG("ABM solution 2 took %llu ms\n", k_uptime_delta(&delta));
	LOG_DBG("Found solution for R_UP %d, R_DOWN %d, T_ON %d T_OFF %d",
		config->abm_ramp_up, config->abm_ramp_down, delay_on, delay_off);
	LOG_DBG("T1: %d T2: %d T3: %d T4: %d PFS: %d", t1, t2, t3, t4, pfs);
	LOG_DBG("Error: %d", ret);


	/* Since Zephyr's LED API has no definition for switching modes,
	 * auto breathe mode 1 is used for the LED blink function. Other
	 * breathe modes are not currently utilized.
	 */
	ret = is31fl3733_select_page(dev, CMD_SEL_BREATHE);
	if (ret < 0) {
		return ret;
	}

	ret = i2c_reg_write_byte_dt(&config->bus, led, ABM_MODE_1);
	if (ret < 0) {
		return ret;
	}

	ret = is31fl3733_select_page(dev, CMD_SEL_FUNC);
	if (ret < 0) {
		return ret;
	}

	ret = i2c_reg_write_byte_dt(&config->bus, ABM1_CTRL1_REG,
		((t1 << ABMX_CTRL1_T1_SHIFT) & ABMX_CTRL1_T1_MASK) |
		((t2 << ABMX_CTRL1_T2_SHIFT) & ABMX_CTRL1_T2_MASK));
	if (ret < 0) {
		return ret;
	}

	ret = i2c_reg_write_byte_dt(&config->bus, ABM1_CTRL2_REG,
		((t3 << ABMX_CTRL2_T3_SHIFT) & ABMX_CTRL2_T3_MASK) |
		((t4 << ABMX_CTRL2_T4_SHIFT) & ABMX_CTRL2_T4_MASK));
	if (ret < 0) {
		return ret;
	}

	/* Per figure 16 in datasheet, we must first disable B_EN, then
	 * enable it
	 */
	ret =  i2c_reg_write_byte_dt(&config->bus, CONF_REG, CONF_REG_SSD_MASK);
	if (ret < 0) {
		return ret;
	}

	/* Set auto breathe mode enable, and PFS value.
	 * Note the PWM mode and breathe mode
	 * cannot be used together, so led brightness cannot be controlled
	 * dynamically when using led_blink API. It can still be set via
	 * the global current control register, however.
	 *
	 * Configuration register is write only. We therefore can't
	 * use the I2C update reg functions, and instead assume the SSD bit
	 * will be set here (since it is set during driver init)
	 */
	ret = i2c_reg_write_byte_dt(&config->bus, CONF_REG,
			((pfs << CONF_REG_PFS_SHIFT) & CONF_REG_PFS_MASK) |
			CONF_REG_B_EN_MASK |
			CONF_REG_SSD_MASK);
	if (ret < 0) {
		return ret;
	}

	/* Finally, per figure 16, we must write 0x0 to 0xEH in page 3
	 * to update time registers
	 */
	return i2c_reg_write_byte_dt(&config->bus, TIME_UPDATE_REG, 0x0);
}


static int is31fl3733_led_set_brightness(const struct device *dev, uint32_t led, uint8_t value)
{
	const struct is31fl3733_config *config = dev->config;
	struct is31fl3733_data *data = dev->data;
	int ret;

	/* Switch to PWM mode, and disable auto breathe mode */
	ret = is31fl3733_select_page(dev, CMD_SEL_BREATHE);
	if (ret < 0) {
		return ret;
	}
	ret = i2c_reg_write_byte_dt(&config->bus, led, ABM_MODE_PWM);
	if (ret < 0) {
		return ret;
	}

	/* Configure PWM mode */
	ret = is31fl3733_select_page(dev, CMD_SEL_PWM);
	if (ret < 0) {
		return ret;
	}
	ret = i2c_reg_write_byte_dt(&config->bus, led, value);
	if (ret < 0) {
		return ret;
	}
	data->pwm_state[led + 1] = value;
	if (value == 0) {
		if ((data->led_state[(led / 8) + 1] & BIT((led % 8)))) {
			/* Turn LED off as well */
			ret = is31fl3733_led_off(dev, led);
			if (ret < 0) {
				return ret;
			}
		}
	} else {
		if ((data->led_state[(led / 8) + 1] & BIT((led % 8))) == 0) {
			/* LED is off. Enable LED */
			ret = is31fl3733_led_on(dev, led);
			if (ret < 0) {
				return ret;
			}
		}
	}
	return ret;
}

static int is31fl3733_led_write_channels(const struct device *dev, uint32_t start_channel,
					 uint32_t num_channels, const uint8_t *buf)
{
	const struct is31fl3733_config *config = dev->config;
	struct is31fl3733_data *data = dev->data;
	int ret = 0U;
	uint32_t led;
	uint8_t *pwm_start, *pwm_cursor, *on_start, *on_cursor, mask;

	if ((start_channel + num_channels) > IS31FL3733_MAX_LED) {
		return -EINVAL;
	}
	pwm_cursor = pwm_start = data->pwm_state + start_channel;
	on_cursor = on_start = data->led_state + (start_channel / 8);
	/* Set PWM and LED target registers as first byte of each transfer */
	*pwm_cursor = start_channel;
	pwm_cursor++;
	*on_cursor = (start_channel / 8);
	on_cursor++;
	for (uint32_t i = 0; i < num_channels; i++) {
		led = (i + start_channel);
		*pwm_cursor = buf[i];
		mask = BIT((led % 8));
		if (buf[i]) {
			*on_cursor |= mask;
		} else {
			*on_cursor &= ~mask;
		}
		pwm_cursor++;
		/* If column boundary will be crossed, move on_cursor */
		if (mask == BIT(7)) {
			on_cursor++;
		}
	}

	/* Switch to PWM mode, and disable auto breathe mode */
	ret = is31fl3733_select_page(dev, CMD_SEL_BREATHE);
	if (ret < 0) {
		return ret;
	}
	for (uint32_t i = start_channel; i < num_channels; i++) {
		ret = i2c_reg_write_byte_dt(&config->bus, i, ABM_MODE_PWM);
		if (ret < 0) {
			return ret;
		}
	}

	/* Write LED on/off states */
	ret = is31fl3733_select_page(dev, CMD_SEL_LED);
	if (ret < 0) {
		return ret;
	}
	LOG_HEXDUMP_DBG(on_start, (on_cursor - on_start), "LED on states");
	ret = i2c_write_dt(&config->bus, on_start, (on_cursor - on_start));
	if (ret < 0) {
		return ret;
	}
	/* Write LED PWM states */
	ret = is31fl3733_select_page(dev, CMD_SEL_PWM);
	if (ret < 0) {
		return ret;
	}
	LOG_HEXDUMP_DBG(pwm_start, (pwm_cursor - pwm_start), "PWM states");
	ret = i2c_write_dt(&config->bus, pwm_start, (pwm_cursor - pwm_start));
	return ret;
}

static int is31fl3733_init(const struct device *dev)
{
	const struct is31fl3733_config *config = dev->config;
	struct is31fl3733_data *data = dev->data;
	int ret = 0U;
	uint8_t dummy;

	if (!device_is_ready(config->bus.bus)) {
		LOG_ERR("I2C device not ready");
		return -ENODEV;
	}
	if ((config->sdb.port != NULL) && !device_is_ready(config->sdb.port)) {
		LOG_ERR("GPIO SDB pin not ready");
		return -ENODEV;
	}

	ret = is31fl3733_select_page(dev, CMD_SEL_FUNC);
	if (ret < 0) {
		return ret;
	}
	/*
	 * read reset reg to reset all registers to POR state,
	 * in case we are booting from a warm reset.
	 */
	ret = i2c_reg_read_byte_dt(&config->bus, RESET_REG, &dummy);
	if (ret < 0) {
		return ret;
	}

	/* Clear LED state and PWM state buffers, to reflect led driver state */
	memset(data->led_state, 0, sizeof(data->led_state));
	memset(data->pwm_state, 0, sizeof(data->pwm_state));


	if (config->sdb.port != NULL) {
		/* Set SDB pin high to exit hardware shutdown */
		ret = gpio_pin_configure_dt(&config->sdb, GPIO_OUTPUT);
		if (ret < 0) {
			return ret;
		}
		ret = gpio_pin_set_dt(&config->sdb, 1);
		if (ret < 0) {
			return ret;
		}
	}
	ret = is31fl3733_select_page(dev, CMD_SEL_FUNC);
	if (ret < 0) {
		return ret;
	}
	/* Set global current control register based off devicetree value */
	ret = i2c_reg_write_byte_dt(&config->bus, GLOBAL_CURRENT_CTRL_REG, config->current_limit);
	if (ret < 0) {
		return ret;
	}
	/* As a final step, we exit software shutdown, disabling display
	 * blanking
	 */
	ret = is31fl3733_select_page(dev, CMD_SEL_FUNC);
	if (ret < 0) {
		return ret;
	}
	ret = i2c_reg_write_byte_dt(&config->bus, CONF_REG, CONF_REG_SSD_MASK);
	if (ret < 0) {
		return ret;
	}
	return ret;
}

static const struct led_driver_api is31fl3733_api = {
	.on = is31fl3733_led_on,
	.off = is31fl3733_led_off,
	.set_brightness = is31fl3733_led_set_brightness,
	.write_channels = is31fl3733_led_write_channels,
	.blink = is31fl3733_led_blink,
};

#define IS31FL3733_DEVICE(n)                                                                       \
                                                                                                   \
	static const struct is31fl3733_config is31fl3733_config_##n = {                            \
		.bus = I2C_DT_SPEC_INST_GET(n),                                                    \
		.sdb = GPIO_DT_SPEC_INST_GET_OR(n, sdb_gpios, {}),                                 \
		.current_limit = DT_INST_PROP(n, current_limit),                                   \
		.default_brightness = DT_INST_PROP(n, default_brightness),                         \
		.abm_ramp_up = DT_INST_PROP_OR(n, abm_ramp_up, 0),				   \
		.abm_ramp_down = DT_INST_PROP_OR(n, abm_ramp_down, 0),                             \
	};                                                                                         \
                                                                                                   \
	static struct is31fl3733_data is31fl3733_data_##n = {                                      \
		.selected_page = CMD_SEL_LED,                                                      \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &is31fl3733_init, NULL, &is31fl3733_data_##n,                     \
			      &is31fl3733_config_##n, POST_KERNEL, CONFIG_LED_INIT_PRIORITY,       \
			      &is31fl3733_api);

DT_INST_FOREACH_STATUS_OKAY(IS31FL3733_DEVICE)
