/*
 * Copyright (c) 2026 Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT alif_ensemble_phy_regulator

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <alif_se_service.h>

LOG_MODULE_REGISTER(regulator_alif_phy, CONFIG_REGULATOR_LOG_LEVEL);

BUILD_ASSERT(CONFIG_REGULATOR_ALIF_PHY_INIT_PRIORITY > CONFIG_ALIF_SE_SERVICE_INIT_PRIORITY,
	     "Alif PHY regulators must be initialized after the SE service");

struct regulator_alif_phy_config {
	struct regulator_common_config common;
	uint32_t mask;
};

struct regulator_alif_phy_data {
	struct regulator_common_data common;
};

static int regulator_alif_phy_set(const struct device *dev, bool enable)
{
	const struct regulator_alif_phy_config *config = dev->config;
	int ret;

	ret = alif_se_service_update_phy_pwr_gating(config->mask, enable);
	if (ret != 0) {
		LOG_ERR("%s: failed to %s PHY mask 0x%x (%d)", dev->name,
			enable ? "enable" : "disable", config->mask, ret);
	}

	/* SE response errors are positive; the regulator API expects -errno */
	return (ret > 0) ? -EIO : ret;
}

static int regulator_alif_phy_enable(const struct device *dev)
{
	return regulator_alif_phy_set(dev, true);
}

static int regulator_alif_phy_disable(const struct device *dev)
{
	return regulator_alif_phy_set(dev, false);
}

static DEVICE_API(regulator, regulator_alif_phy_api) = {
	.enable = regulator_alif_phy_enable,
	.disable = regulator_alif_phy_disable,
};

static int regulator_alif_phy_init(const struct device *dev)
{
	const struct regulator_alif_phy_config *config = dev->config;
	alif_run_profile_t runp;
	int ret;

	regulator_common_data_init(dev);

	ret = alif_se_service_get_last_set_run_cfg(&runp);
	if (ret != 0) {
		LOG_ERR("%s: failed to read run profile (%d)", dev->name, ret);
		return ret;
	}

	return regulator_common_init(dev, (runp.phy_pwr_gating & config->mask) != 0U);
}

#define REGULATOR_ALIF_PHY_DEFINE(inst)                                                            \
	static struct regulator_alif_phy_data regulator_alif_phy_data_##inst;                      \
                                                                                                   \
	static const struct regulator_alif_phy_config regulator_alif_phy_config_##inst = {         \
		.common = REGULATOR_DT_INST_COMMON_CONFIG_INIT(inst),                              \
		.mask = UTIL_CAT(DT_INST_STRING_UPPER_TOKEN(inst, phy), _MASK),                    \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, regulator_alif_phy_init, NULL,                                 \
			      &regulator_alif_phy_data_##inst, &regulator_alif_phy_config_##inst,  \
			      POST_KERNEL, CONFIG_REGULATOR_ALIF_PHY_INIT_PRIORITY,                \
			      &regulator_alif_phy_api);

DT_INST_FOREACH_STATUS_OKAY(REGULATOR_ALIF_PHY_DEFINE)
