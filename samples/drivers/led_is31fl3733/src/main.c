/*
 * Copyright 2022 Daniel DeGrasse <daniel@degrasse.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/led/is31fl3733.h>
#include <string.h>

#define HW_ROW_COUNT 12
#define HW_COL_COUNT 16

/* LED matrix is addressed using a row major format */
#define LED_MATRIX_COORD(x, y) ((x) * HW_COL_COUNT) + (y)

static uint8_t led_state[HW_COL_COUNT * HW_ROW_COUNT];

static int led_breathe(const struct device *led)
{
	int ret;

	printk("Blank all LEDs\n");
	/* Blank all LEDs before setting output */
	ret = is31fl3733_blank(led, true);
	if (ret < 0) {
		printk("Error: led panel could not be blanked: (%d)\n", ret);
		return ret;
	}
	printk("Blink LEDs using auto breathe mode\n");
	for (uint8_t row = 0; row < CONFIG_LED_ROW_COUNT; row++) {
		for (uint8_t col = 0; col < CONFIG_LED_COLUMN_COUNT; col++) {
			/* All delays are given in ms. The first parameter
			 * to IS31FL3733_*_DELAY() functions is the rise/fall
			 * time for the LED brightness, and the second is
			 * the time the LED should remain on/off.
			 */
			ret = led_blink(led, LED_MATRIX_COORD(row, col),
				IS31FL3733_ON_DELAY(560, 1120),
				IS31FL3733_OFF_DELAY(280, 560));
			if (ret < 0) {
				printk("Error: could not set blink mode: (%d)\n", ret);
				return ret;
			}
		}
	}
	ret = is31fl3733_blank(led, false);
	if (ret < 0) {
		printk("Error: led panel blank could not be disabled (%d)\n", ret);
		return ret;
	}
	/* Delay for several seconds. During this time the
	 * hardware should automatically keep the LEDs blinking
	 */
	printk("Delay to allow LEDs to blink...\n");
	k_msleep(5000);
	return 0;
}

int led_channel_write(const struct device *led)
{
	int ret;
	uint32_t led_idx;

	/* Set all LEDs to full brightness */
	printk("Set all LEDs to full brightness\n");
	memset(led_state, 0, sizeof(led_state));
	for (uint8_t row = 0; row < CONFIG_LED_ROW_COUNT; row++) {
		for (uint8_t col = 0; col < CONFIG_LED_COLUMN_COUNT; col++) {
			led_idx = LED_MATRIX_COORD(row, col);
			led_state[led_idx] = 0xFF;
		}
	}
	ret = led_write_channels(led, 0, sizeof(led_state), led_state);
	if (ret) {
		printk("Error: could not write LED channels (%d)\n", ret);
		return ret;
	}
	k_msleep(1000);
	/* Dim quadrant of LED display */
	printk("Dim LED quadrant\n");
	for (uint8_t row = 0; row < CONFIG_LED_ROW_COUNT / 2; row++) {
		for (uint8_t col = 0; col < CONFIG_LED_COLUMN_COUNT / 2; col++) {
			led_idx = LED_MATRIX_COORD(row, col);
			led_state[led_idx] = 0x0F;
		}
	}
	ret = led_write_channels(led, 0,
		((CONFIG_LED_ROW_COUNT / 2) * HW_COL_COUNT), led_state);
	if (ret) {
		printk("Error: could not write LED channels (%d)\n", ret);
		return ret;
	}
	k_msleep(1000);
	return 0;
}

int led_brightness(const struct device *led)
{
	int ret;
	uint8_t row, col;

	/* Set LED brightness to low value sequentially */
	printk("Dim LEDs in sequence\n");
	for (row = 0; row < CONFIG_LED_ROW_COUNT; row++) {
		for (col = 0; col < CONFIG_LED_COLUMN_COUNT; col++) {
			ret = led_set_brightness(led, LED_MATRIX_COORD(row, col),
						0x10);
			if (ret < 0) {
				printk("Error: could not enable led "
					"at [%d, %d]: (%d)\n",
					row, col, ret);
				return ret;
			}
			k_msleep(100);
		}
	}
	return 0;
}

int led_on_off(const struct device *led)
{
	int ret;
	uint8_t row, col;

	printk("Toggle each led\n");
	/* Turn on each led for a short duration */
	for (row = 0; row < CONFIG_LED_ROW_COUNT; row++) {
		for (col = 0; col < CONFIG_LED_COLUMN_COUNT; col++) {
			ret = led_off(led, LED_MATRIX_COORD(row, col));
			if (ret < 0) {
				printk("Error: could not disable led "
					"at [%d, %d]: (%d)\n",
					row, col, ret);
				return ret;
			}
			k_msleep(100);
			ret = led_on(led, LED_MATRIX_COORD(row, col));
			if (ret < 0) {
				printk("Error: could not enable led "
					"at [%d, %d]: (%d)\n",
					row, col, ret);
				return ret;
			}
		}
	}
	k_msleep(500);
	return 0;
}


const struct device *led_dev = DEVICE_DT_GET_ONE(issi_is31fl3733);

void main(void)
{
	int ret;
	int current_limit = 0xFF;

	if (!device_is_ready(led_dev)) {
		printk("Error- LED device is not ready\n");
		return;
	}

	while (1) {
		ret = led_breathe(led_dev);
		if (ret < 0) {
			return;
		}
		ret = led_channel_write(led_dev);
		if (ret < 0) {
			return;
		}
		ret = led_brightness(led_dev);
		if (ret < 0) {
			return;
		}
		ret = led_on_off(led_dev);
		if (ret < 0) {
			return;
		}
		if (current_limit == 0xFF) {
			/* Select lower current limt */
			printk("Restarting sample with lower current limit\n");
			current_limit = 0x3F;
			ret = is31fl3733_current_limit(led_dev, current_limit);
			if (ret) {
				printk("Could not set LED current limit (%d)\n", ret);
				return;
			}
		} else {
			/* Select higher current limt */
			printk("Restarting sample with higher current limit\n");
			current_limit = 0xFF;
			ret = is31fl3733_current_limit(led_dev, current_limit);
			if (ret) {
				printk("Could not set LED current limit (%d)\n", ret);
				return;
			}
		}
	}
}
