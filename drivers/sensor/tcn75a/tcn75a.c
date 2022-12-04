/*
 * Copyright 2022 Daniel DeGrasse <daniel@degrasse.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_tcn75a

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tcn75a, CONFIG_SENSOR_LOG_LEVEL);

#include "tcn75a.h"

int tcn75a_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	const struct tcn75a_config *config = dev->config;
	struct tcn75a_data *data = dev->data;
	int ret;
	uint8_t temp_reg = TCN75A_TEMP_REG;
	uint8_t rx_buf[2];
#ifdef CONFIG_TCN75A_OP_MODE_ONESHOT
	uint8_t adc_conf[2] = {TCN75A_CONFIG_REG, 0x0};
#endif
	/* This sensor only supports ambient temperature */
	if ((chan != SENSOR_CHAN_ALL) && (chan != SENSOR_CHAN_AMBIENT_TEMP)) {
		return -ENOTSUP;
	}

#ifdef CONFIG_TCN75A_OP_MODE_ONESHOT
	/* Oneshot mode, requires one shot bit to be set in config register */
	adc_conf[1] = TCN75A_CONFIG_ONEDOWN;
	ret = i2c_write_dt(&config->i2c_spec, adc_conf, 2);
	if (ret) {
		return ret;
	}
#endif

	/* Fetch a sample from the 2 byte ambient temperature register */
	ret = i2c_write_read_dt(&config->i2c_spec, &temp_reg, 1, rx_buf, 2);
	if (ret) {
		return ret;
	}
	data->temp_sample = sys_get_be16(rx_buf);
	LOG_DBG("Raw sample: 0x%04x", data->temp_sample);

	return ret;
}

static int tcn75a_channel_get(const struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct tcn75a_data *data = dev->data;
	uint32_t temp_lsb;

	if (chan != SENSOR_CHAN_AMBIENT_TEMP) {
		return -ENOTSUP;
	}

	/* Convert fixed point to sensor value  */
	val->val1 = data->temp_sample >> TCN75A_TEMP_MSB_POS;
	temp_lsb = (data->temp_sample & TCN75A_TEMP_LSB_MASK);
	val->val2 = TCN75A_FIXED_PT_TO_SENSOR(temp_lsb);
	return 0;
}

static const struct sensor_driver_api tcn75a_api = {
	.sample_fetch = &tcn75a_sample_fetch,
	.channel_get = &tcn75a_channel_get,
#ifdef CONFIG_TCN75A_TRIGGER
	.attr_get = &tcn75a_attr_get,
	.attr_set = &tcn75a_attr_set,
	.trigger_set = &tcn75a_trigger_set,
#endif
};

static int tcn75a_init(const struct device *dev)
{
	const struct tcn75a_config *config = dev->config;
	int ret = 0;
#if !defined(CONFIG_TCN75A_RESOLUTION_9BIT) || defined(CONFIG_TCN75A_OP_MODE_ONESHOT)
	uint8_t adc_conf[2] = {TCN75A_CONFIG_REG, 0x0};
#endif

	if (!device_is_ready(config->i2c_spec.bus)) {
		LOG_ERR("I2C bus is not ready");
		return -ENODEV;
	}

#ifndef CONFIG_TCN75A_RESOLUTION_9BIT
	/* If user selected non-default resolution, configure it */
	if (IS_ENABLED(CONFIG_TCN75A_RESOLUTION_10BIT)) {
		adc_conf[1] |= TCN75A_CONFIG_10BIT_RES;
	} else if (IS_ENABLED(CONFIG_TCN75A_RESOLUTION_11BIT)) {
		adc_conf[1] |= TCN75A_CONFIG_11BIT_RES;
	} else if (IS_ENABLED(CONFIG_TCN75A_RESOLUTION_12BIT)) {
		adc_conf[1] |= TCN75A_CONFIG_12BIT_RES;
	}

#endif

#ifdef CONFIG_TCN75A_OP_MODE_ONESHOT
	adc_conf[1] |= TCN75A_CONFIG_SHUTDOWN;
#endif

#ifdef CONFIG_TCN75A_TRIGGER
	ret = tcn75a_trigger_init(dev);
	if (ret < 0) {
		return ret;
	}
#endif

#if !defined(CONFIG_TCN75A_RESOLUTION_9BIT) || defined(CONFIG_TCN75A_OP_MODE_ONESHOT)
	ret = i2c_write_dt(&config->i2c_spec, adc_conf, 2);
#endif

	return ret;
}

#ifdef CONFIG_TCN75A_TRIGGER
#define TCN75A_TRIGGER(n) .alert_gpios = GPIO_DT_SPEC_INST_GET(n, alert_gpios),
#else
#define TCN75A_TRIGGER(n)
#endif

#define TCN75A_INIT(n)                                                                             \
	static struct tcn75a_data tcn75a_data_##n;                                                 \
	static const struct tcn75a_config tcn75a_config_##n = {                                    \
		.i2c_spec = I2C_DT_SPEC_INST_GET(n), TCN75A_TRIGGER(n)};                           \
	DEVICE_DT_INST_DEFINE(n, &tcn75a_init, NULL, &tcn75a_data_##n, &tcn75a_config_##n,         \
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &tcn75a_api);

DT_INST_FOREACH_STATUS_OKAY(TCN75A_INIT)
