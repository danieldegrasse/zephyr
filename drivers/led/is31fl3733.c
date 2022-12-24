/*
 * Copyright 2022-2023 Daniel DeGrasse <daniel@degrasse.com>
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

#include <zephyr/drivers/led/is31fl3733.h>

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
#define CONF_REG_SYNC_SHIFT 0x6 /* Sync mode shift */
#define CONF_REG_SYNC_MASK 0xC /* Sync mode mask */

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
	uint8_t sync;
};

struct is31fl3733_data {
	/* Active configuration page */
	uint32_t selected_page;
	/* Array of LED on/off states */
	uint8_t led_state[((IS31FL3733_MAX_LED) / 8) + 1];
	/* Array of LED PWM states */
	uint8_t pwm_state[(IS31FL3733_MAX_LED) + 1];
	/* LED config reg state, IS31FL3733 conf reg is write only */
	uint8_t conf_reg;

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


/*
 * This helper returns the result of log2(num/denom), rounded to the nearest
 * positive integer to minimize the error of 2^log2(num/denom)
 */
int log2_helper(uint32_t num, uint32_t denom)
{
	uint32_t round_div, res = 0;

	/* First, divide the numerator and denominator. There are three
	 * cases we must handle here:
	 * - num * 2 < demon: special case, return -1
	 * - num < denom * 2: round to nearest integer.
	 * - num > denom * 2: truncate. This eliminates the possible error
	 *   of (demon/2) introduced by rounding up to the division, to a value
	 *   that would result in a higher base 2 log value
	 */
	if ((num << 1) < denom) {
		/* For some T calculations, T=0 results in an output time
		 * of 0. Any case where num is less than half of denom
		 * will use T=0, so return -1 here. For the T calculations
		 * that add 1, this will result in 0, and other calculations
		 * will simply force the final T value to be 0.
		 */
		return -1;
	} else if (num < (denom << 1)) {
		round_div = (num + (denom >> 1)) / denom;
	} else {
		round_div = num / denom;
	}
	/* Now, approximate log2(). We check the two highest MSBs to determine
	 * rounding direction
	 */
	res = find_msb_set(round_div);
	if (res == 0) {
		return res;
	}
	res--;
	/* Shift division result until only two highest MSBs are present */
	round_div = round_div >> res;
	/* If MSB and LSB of remaining value are set, log2() should round up */
	if (round_div == 3) {
		res += 2;
	} else if (round_div == 2) {
		res += 1;
	}
	return res;
}


/*
 * Helper function to find best possible auto breathe mode setting
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
		 * pfs value using the log2 helper function
		 */
		t1 = MAX(log2_helper(ramp_up, pfs_base), 0);
		t2 = MAX(log2_helper(delay_on, pfs_base) + 1, 0);
		t3 = MAX(log2_helper(ramp_down, pfs_base), 0);
		t4 = MAX(log2_helper(delay_off, pfs_base) + 1, 0);

		/* Clamp T1-T4 to their maximum values */
		t1 = MIN(t1, ABMX_CTRL1_T1_MAX);
		t2 = MIN(t2, ABMX_CTRL1_T2_MAX);
		t3 = MIN(t3, ABMX_CTRL2_T3_MAX);
		t4 = MIN(t4, ABMX_CTRL2_T4_MAX);

		if ((t4 == 0) && (pfs == 2)) {
			/*
			 * Datasheet says this is an invalid setting.
			 * Test with T4=1
			 */
			t4 = 1;
		}
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

static int is31fl3733_led_blink(const struct device *dev, uint32_t led,
	uint32_t delay_on, uint32_t delay_off)
{
	const struct is31fl3733_config *config = dev->config;
	struct is31fl3733_data *data = dev->data;
	int ret;
	uint8_t t1 = 0, t2 = 0, t3 = 0, t4 = 0, pfs = 0;
	uint16_t rise, fall, on, off;

	/* Extract slew rates and delays from the delay values */
	rise = delay_on >> IS31FL3733_SLEW_SHIFT;
	on = delay_on & IS31FL3733_DELAY_MASK;
	fall = delay_off >> IS31FL3733_SLEW_SHIFT;
	off = delay_off & IS31FL3733_DELAY_MASK;

	if ((data->led_state[(led / 8) + 1] & BIT((led % 8))) == 0) {
		/* LED is off. Enable LED */
		ret = is31fl3733_led_on(dev, led);
		if (ret < 0) {
			return ret;
		}
	}

	ret = is31fl3733_abm_solution(rise, fall, on, off,
				&pfs, &t1, &t2, &t3, &t4);
	LOG_DBG("Found solution for R_UP %d, R_DOWN %d, T_ON %d T_OFF %d",
		rise, fall, on, off);
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
	data->conf_reg &= ~CONF_REG_B_EN_MASK;
	ret =  i2c_reg_write_byte_dt(&config->bus, CONF_REG, data->conf_reg);
	if (ret < 0) {
		return ret;
	}

	/* Set auto breathe mode enable, and PFS value.
	 * Note the PWM mode and breathe mode
	 * cannot be used together, so led brightness cannot be controlled
	 * dynamically when using led_blink API. It can still be set via
	 * the global current control register, however.
	 */
	data->conf_reg &= ~CONF_REG_PFS_MASK;
	data->conf_reg |= (((pfs << CONF_REG_PFS_SHIFT) & CONF_REG_PFS_MASK) |
			CONF_REG_B_EN_MASK);
	ret = i2c_reg_write_byte_dt(&config->bus, CONF_REG, data->conf_reg);
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
	ret = i2c_reg_write_byte_dt(&config->bus, GLOBAL_CURRENT_CTRL_REG,
				config->current_limit);
	if (ret < 0) {
		return ret;
	}
	/* As a final step, we exit software shutdown, disabling display
	 * blanking. We also set the LED controller sync mode here.
	 */
	ret = is31fl3733_select_page(dev, CMD_SEL_FUNC);
	if (ret < 0) {
		return ret;
	}
	data->conf_reg = (config->sync << CONF_REG_SYNC_SHIFT) | CONF_REG_SSD_MASK;
	ret = i2c_reg_write_byte_dt(&config->bus, CONF_REG, data->conf_reg);
	if (ret < 0) {
		return ret;
	}
	return ret;
}

/* Custom IS31FL3733 specific APIs */

/**
 * @brief Blanks IS31FL3733 LED display.
 *
 * When blank_en is set, the LED display will be disabled. This can be used for
 * flicker-free display updates, or power saving.
 *
 * @param dev: LED device structure
 * @param blank_en: should blanking be enabled
 * @return 0 on success, or negative value on error.
 */
int is31fl3733_blank(const struct device *dev, bool blank_en)
{
	const struct is31fl3733_config *config = dev->config;
	struct is31fl3733_data *data = dev->data;
	int ret;

	ret = is31fl3733_select_page(dev, CMD_SEL_FUNC);
	if (ret < 0) {
		return ret;
	}

	if (blank_en) {
		data->conf_reg &= ~CONF_REG_SSD_MASK;
	} else {
		data->conf_reg |= CONF_REG_SSD_MASK;
	}
	return i2c_reg_write_byte_dt(&config->bus, CONF_REG, data->conf_reg);
}

/**
 * @brief Sets led current limit
 *
 * Sets the current limit for the LED driver. This is a separate value
 * from per-led brightness, and applies to all LEDs.
 * This value sets the output current limit according
 * to the following formula: (840/R_ISET) * (limit/256)
 * See table 14 of the datasheet for additional details.
 * @param dev: LED device structure
 * @param limit: current limit to apply
 * @return 0 on success, or negative value on error.
 */
int is31fl3733_current_limit(const struct device *dev, uint8_t limit)
{
	const struct is31fl3733_config *config = dev->config;
	int ret;

	ret = is31fl3733_select_page(dev, CMD_SEL_FUNC);
	if (ret < 0) {
		return ret;
	}
	/* Set global current control register based off devicetree value */
	return i2c_reg_write_byte_dt(&config->bus, GLOBAL_CURRENT_CTRL_REG, limit);
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
		.sync = DT_INST_ENUM_IDX(n, sync_mode),						   \
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
