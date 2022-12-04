/*
 * Copyright 2022 Daniel DeGrasse <daniel@degrasse.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

double low_temp;
double high_temp;

/**
 * @file Sample app using the TCN75A serial temperature sensor
 *
 * This app will utilize the alert functionality on the TCN75A to set
 * high and low thresholds. The user will be alerted when the temperature
 * readout exceeds either threshold.
 */

int read_temperature(const struct device *dev, double *temp)
{
	int ret;
	struct sensor_value val;

	if (temp == NULL) {
		return -EINVAL;
	}

	ret = sensor_sample_fetch_chan(dev, SENSOR_CHAN_AMBIENT_TEMP);
	if (ret < 0) {
		printf("Could not fetch temperature (%d)\n", ret);
		return ret;
	}

	ret = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &val);
	if (ret < 0) {
		printf("Could not get temperature (%d)\n", ret);
		return ret;
	}

	*temp = sensor_value_to_double(&val);
	return ret;
}

void temp_alert_handler(const struct device *dev, const struct sensor_trigger *trig)
{
	int ret;
	double temp;

	/* Read sensor value */
	ret = read_temperature(dev, &temp);
	if (ret < 0) {
		printf("Reading temperature failed (%d)\n", ret);
		return;
	}
	if (temp <= low_temp) {
		printf("Temperature below threshold: %.2f\n", temp);
	} else if (temp >= high_temp) {
		printf("Temperature above threshold: %.2f\n", temp);
	} else {
		printf("Error: temperature alert triggered without valid condition\n");
	}
}

void main(void)
{
	const struct device *const dev = DEVICE_DT_GET_ONE(microchip_tcn75a);
	double current_temp;
	struct sensor_value low_val;
	struct sensor_value high_val;
	const struct sensor_trigger trig = {
		.chan = SENSOR_CHAN_AMBIENT_TEMP,
		.type = SENSOR_TRIG_THRESHOLD,
	};
	int ret;

	if (!device_is_ready(dev)) {
		printk("sensor: device not ready.\n");
		return;
	}

	/* The TCN75A tends to read 0x0 from the temp register shortly after.
	 * power up. Briefly delay to avoid this.
	 */
	k_msleep(500);

	ret = read_temperature(dev, &current_temp);
	if (ret < 0) {
		printf("Reading temperature failed (%d)\n", ret);
		return;
	}
	printf("Current temperature is %.2f\n", current_temp);

	/* Set low and high thresholds to +0.5 and +1.5 degree C from ambient */
	low_temp = current_temp + 0.5;
	high_temp = current_temp + 1.5;
	ret = sensor_value_from_double(&low_val, low_temp);
	if (ret < 0) {
		printf("Could not covert low temp limit to sensor value (%d)\n", ret);
	}
	ret = sensor_value_from_double(&high_val, high_temp);
	if (ret < 0) {
		printf("Could not covert high temp limit to sensor value (%d)\n", ret);
	}
	ret = sensor_attr_set(dev, SENSOR_CHAN_AMBIENT_TEMP, SENSOR_ATTR_UPPER_THRESH, &high_val);
	if (ret < 0) {
		printf("Could not set temperature upper limit (%d)\n", ret);
	}
	printf("Set temperature upper limit to: %.2f C\n", high_temp);
	ret = sensor_attr_set(dev, SENSOR_CHAN_AMBIENT_TEMP, SENSOR_ATTR_LOWER_THRESH, &low_val);
	if (ret < 0) {
		printf("Could not set temperature lower limit (%d)\n", ret);
	}
	printf("Set temperature lower limit to: %.2f C\n", low_temp);

	ret = sensor_trigger_set(dev, &trig, temp_alert_handler);
	if (ret < 0) {
		printf("Could not add temperature trigger (%d)\n", ret);
	}
}
