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

/* Function configuration page */
#define CONF_REG	   0x0 /* configuration register */
#define CONF_REG_SSD_MASK  0x1 /* Software shutdown mask */
#define CONF_REG_SSD_SHIFT 0x0 /* Software shutdown shift */

#define GLOBAL_CURRENT_CTRL_REG 0x1 /* global current control register */

#define IS31FL3733_ROW_COUNT 12
#define IS31FL3733_COL_COUNT 16

struct is31fl3733_config {
	struct i2c_dt_spec bus;
	struct gpio_dt_spec sdb;
	uint8_t current_limit;
	uint8_t default_brightness;
	uint8_t row_count;
	uint8_t col_count;
};

struct is31fl3733_data {
	/* Active configuration page */
	uint32_t selected_page;
	/* Array of LED on/off states */
	uint8_t led_state[((IS31FL3733_ROW_COUNT * IS31FL3733_COL_COUNT) / 8) + 1];
	/* Array of LED PWM states */
	uint8_t pwm_state[(IS31FL3733_ROW_COUNT * IS31FL3733_COL_COUNT) + 1];
};

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

/* Helper function to get row and column index for given LED offset*/
static int is31fl3733_led_get_cords(const struct device *dev, uint32_t led, uint8_t *row,
				    uint8_t *col)
{
	const struct is31fl3733_config *config = dev->config;
	/* LED indicies are row major */
	*row = led / config->col_count;
	*col = led - ((*row) * config->col_count);
	if ((*row > config->row_count) || (*col > config->col_count)) {
		LOG_ERR("Led index out of range");
		return -EOVERFLOW;
	}
	return 0;
}

/* Helper function to turn led on/off */
static int is31fl3733_led_on_off(const struct device *dev, uint32_t led, bool on)
{
	const struct is31fl3733_config *config = dev->config;
	struct is31fl3733_data *data = dev->data;
	int ret;
	uint8_t row, col, reg, mask;

	ret = is31fl3733_select_page(dev, CMD_SEL_LED);
	if (ret < 0) {
		return ret;
	}
	ret = is31fl3733_led_get_cords(dev, led, &row, &col);
	if (ret < 0) {
		return ret;
	}
	/* Calculate LED on/off register, see table 6 of data sheet */
	reg = row * 2;
	if (col > 7) {
		reg++;
	}
	mask = BIT((col % 8));
	/* Note: first byte of led state reg is reserved for address
	 * data, used with burst writes
	 */
	if (on) {
		data->led_state[reg + 1] |= mask;
	} else {
		data->led_state[reg + 1] &= ~mask;
	}
	ret = i2c_reg_write_byte_dt(&config->bus, reg, data->led_state[reg + 1]);
	if (on) {
		LOG_DBG("Setting bit %d of LED on/off reg 0x%02X", (col % 8), reg);
	} else {
		LOG_DBG("Clearing bit %d of LED on/off reg 0x%02X", (col % 8), reg);
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

static int is31fl3733_led_set_brightness(const struct device *dev, uint32_t led, uint8_t value)
{
	const struct is31fl3733_config *config = dev->config;
	int ret;
	uint8_t row, col, reg;

	ret = is31fl3733_select_page(dev, CMD_SEL_PWM);
	if (ret < 0) {
		return ret;
	}
	ret = is31fl3733_led_get_cords(dev, led, &row, &col);
	if (ret < 0) {
		return ret;
	}
	if (value == 0) {
		/* Turn LED off as well */
		ret = is31fl3733_led_off(dev, led);
		if (ret) {
			return ret;
		}
	}
	/* Calculate LED PWM register, see figure 9 of data sheet */
	reg = (row << 4) + (col);
	LOG_DBG("Setting PWM reg 0x%02X to %d", reg, value);
	ret = i2c_reg_write_byte_dt(&config->bus, reg, value);
	return ret;
}

static int is31fl3733_led_write_channels(const struct device *dev, uint32_t start_channel,
					 uint32_t num_channels, const uint8_t *buf)
{
	const struct is31fl3733_config *config = dev->config;
	struct is31fl3733_data *data = dev->data;
	int ret = 0U;
	uint32_t idx = 0U;
	uint8_t *pwm_cursor, row, col, *pwm_start, *on_cursor, *on_start, mask;

	/* Calculate coordinates of start_channel */
	ret = is31fl3733_led_get_cords(dev, start_channel, &row, &col);
	if (ret < 0) {
		return ret;
	}
	/* Use row and column to calculate pwm_cursor start*/
	pwm_start = pwm_cursor = data->pwm_state + (row * IS31FL3733_COL_COUNT) + col;
	/* Save PWM start register to first byte of pwm_cursor */
	*pwm_cursor = (row << 4) + col;
	pwm_cursor++;
	/* Use row and column to calculate on_cursor start */
	on_start = on_cursor = data->led_state + (row * 2);
	if (col > 7) {
		on_start++;
	}
	/* Save LED on/off start register to first byte of on_cursor */
	*on_cursor = (row * 2);
	if (col > 7) {
		(*on_cursor)++;
	}
	on_cursor++;
	/* Set LED states from input buffer */
	while (idx < num_channels) {
		*pwm_cursor = buf[idx];
		mask = BIT((col % 8));
		if (buf[idx]) {
			/* Turn on LED */
			*on_cursor |= mask;
		} else {
			*on_cursor &= ~mask;
		}
		col++;
		idx++;
		pwm_cursor++;
		if (col == 8) {
			on_cursor++;
		}
		if (col == config->col_count) {
			/* Set PWM of remaining LEDs to 0 */
			memset(pwm_cursor, 0, (IS31FL3733_COL_COUNT - config->col_count) - 1);
			/* Move to next row */
			pwm_cursor += (IS31FL3733_COL_COUNT - config->col_count);
			/* Disable remaining LEDs */
			while (col < IS31FL3733_COL_COUNT) {
				mask = BIT((col % 8));
				if (col == 8) {
					on_cursor++;
				}
				*on_cursor &= ~mask;
				col++;
			}
			on_cursor++;
			col = 0;
			row++;
			/* Check bounds of new LED coordinates */
			if (row >= config->row_count && (idx != num_channels)) {
				return -EOVERFLOW;
			}
		}
	}
	ret = is31fl3733_select_page(dev, CMD_SEL_PWM);
	if (ret < 0) {
		return ret;
	}
	/* Write the new PWM states */
	ret = i2c_write_dt(&config->bus, pwm_start, (pwm_cursor - pwm_start));
	if (ret < 0) {
		return ret;
	}
	ret = is31fl3733_select_page(dev, CMD_SEL_LED);
	if (ret < 0) {
		return ret;
	}
	/* Write the new LED states */
	ret = i2c_write_dt(&config->bus, on_start, (on_cursor - on_start));
	return ret;
}

static int is31fl3733_init(const struct device *dev)
{
	const struct is31fl3733_config *config = dev->config;
	int ret = 0U;
	uint8_t led;

	if (!device_is_ready(config->bus.bus)) {
		LOG_ERR("I2C device not ready");
		return -ENODEV;
	}
	if ((config->sdb.port != NULL) && !device_is_ready(config->sdb.port)) {
		LOG_ERR("GPIO SDB pin not ready");
		return -ENODEV;
	}

	/*
	 * Before exiting hardware shutdown, we ensure chip will wake
	 * into software shutdown. This way, the LED array will be blanked
	 * when we configure PWM values and disable LEDs. This is desireable
	 * because if the LED array was enabled, user could see some flashing
	 * while LEDs are reconfigured after a reset
	 */
	ret = is31fl3733_select_page(dev, CMD_SEL_FUNC);
	if (ret < 0) {
		return ret;
	}
	/* Clear configuration register, so chip is in software shutdown */
	ret = i2c_reg_write_byte_dt(&config->bus, CONF_REG, 0x0);
	if (ret < 0) {
		return ret;
	}

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
	/* To simplify use of the LED driver, we set default PWM values
	 * for all LEDs in the array. This way, if the user enables an LED
	 * via led_on API, the led will actually illuminate.
	 */
	for (uint8_t i = 0; i < config->row_count; i++) {
		for (uint8_t j = 0; j < config->col_count; j++) {
			led = (i * config->col_count) + j;
			/* Disable LED */
			ret = is31fl3733_led_off(dev, led);
			if (ret) {
				return ret;
			}
			/* Set LED brightness */
			ret = is31fl3733_led_set_brightness(dev, led, config->default_brightness);
			if (ret < 0) {
				return ret;
			}
		}
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
};

#define IS31FL3733_DEVICE(n)                                                                       \
                                                                                                   \
	static const struct is31fl3733_config is31fl3733_config_##n = {                            \
		.bus = I2C_DT_SPEC_INST_GET(n),                                                    \
		.sdb = GPIO_DT_SPEC_INST_GET_OR(n, sdb_gpios, {}),                                 \
		.current_limit = DT_INST_PROP(n, current_limit),                                   \
		.default_brightness = DT_INST_PROP(n, default_brightness),                         \
		.col_count = DT_INST_PROP(n, col_count),                                           \
		.row_count = DT_INST_PROP(n, row_count),                                           \
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
